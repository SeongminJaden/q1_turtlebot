"""
UWB Follow Me — 시리얼 직접 읽기, 외부 의존 없음.

리스너 USB에서 태그 위치를 직접 읽어
로봇이 사람을 추종합니다.

실행:
  ros2 run q1_turtlebot follow_me --ros-args -p serial_port:=/dev/ttyUSB1
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    import serial
except ImportError:
    raise ImportError("pyserial required: pip3 install pyserial")


class FollowMeNode(Node):
    IDLE = 0
    FOLLOWING = 1
    STOPPED = 2

    def __init__(self):
        super().__init__('follow_me')

        # 파라미터
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('robot_tag_id', '25A0')
        self.declare_parameter('target_tag_id', '1E52')
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('max_speed', 0.154)
        self.declare_parameter('max_angular', 1.988)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('qf_threshold', 30)
        self.declare_parameter('timeout_sec', 2.0)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.robot_tag = self.get_parameter('robot_tag_id').value
        self.target_tag = self.get_parameter('target_tag_id').value
        self.target_dist = self.get_parameter('target_distance').value
        self.tolerance = self.get_parameter('distance_tolerance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular = self.get_parameter('max_angular').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.qf_thresh = self.get_parameter('qf_threshold').value
        self.timeout = self.get_parameter('timeout_sec').value

        # 상태
        self.robot_pos = None
        self.target_pos = None
        self.robot_qf = 0
        self.target_qf = 0
        self.robot_yaw = 0.0
        self.last_target_time = 0.0
        self.state = self.IDLE
        self._lock = threading.Lock()

        # 앵커 (시각화용)
        self._anchors = {
            'AN0': {'x': 0.0, 'y': 0.0},
            'AN1': {'x': 6.1, 'y': 0.0},
            'AN2': {'x': 6.1, 'y': 7.04},
            'AN3': {'x': 0.0, 'y': 7.04},
        }

        # 구독
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # 발행
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/follow_me/status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/follow_me/markers', 10)

        self.create_timer(0.1, self._control_loop)
        self.create_timer(0.2, self._publish_viz)

        # 시리얼
        self.ser = None
        self._running = True
        self._connect_serial()

        self.get_logger().info(
            f'Follow-Me 시작  로봇={self.robot_tag}  타겟={self.target_tag}\n'
            f'  시리얼={self.port}  목표거리={self.target_dist}m\n'
            f'  QF≥{self.qf_thresh}  timeout={self.timeout}s')

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baud, timeout=1.0)
            self.get_logger().info(f'Serial connected: {self.port} @ {self.baud}')

            import time as t
            self.ser.write(b'\r')
            t.sleep(0.3)
            self.ser.write(b'lep\r')
            t.sleep(0.3)
            self.ser.reset_input_buffer()

            self._read_thread = threading.Thread(
                target=self._read_loop, daemon=True)
            self._read_thread.start()

        except serial.SerialException as e:
            self.get_logger().error(f'Serial failed: {e}')

    def _read_loop(self):
        while self._running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or not line.startswith('POS'):
                    continue
                self._parse_pos(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                break
            except Exception as e:
                self.get_logger().warn(f'Parse error: {e}')

    def _parse_pos(self, line: str):
        parts = line.split(',')
        if len(parts) < 7:
            return
        try:
            tag_id = parts[2].strip()
            x = float(parts[3])
            y = float(parts[4])
            qf = int(parts[6])
        except (ValueError, IndexError):
            return

        with self._lock:
            if tag_id == self.robot_tag:
                self.robot_pos = [x, y]
                self.robot_qf = qf
            elif tag_id == self.target_tag:
                self.target_pos = [x, y]
                self.target_qf = qf
                self.last_target_time = time.time()

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _control_loop(self):
        cmd = Twist()

        with self._lock:
            robot_pos = self.robot_pos
            target_pos = self.target_pos
            robot_qf = self.robot_qf
            target_qf = self.target_qf

        if robot_pos is None or target_pos is None:
            self.state = self.IDLE
            self.pub_cmd.publish(cmd)
            return

        if target_qf < self.qf_thresh or robot_qf < self.qf_thresh:
            self.state = self.STOPPED
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(
                data=f'STOPPED: QF low (R={robot_qf}, T={target_qf})'))
            return

        if time.time() - self.last_target_time > self.timeout:
            self.state = self.STOPPED
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(data='STOPPED: timeout'))
            return

        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        target_angle = math.atan2(dy, dx)
        # odom heading에 180도 오프셋 (UWB↔odom 좌표계 보정)
        angle_err = target_angle - (self.robot_yaw + math.pi)
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        self.state = self.FOLLOWING
        dist_err = distance - self.target_dist

        if abs(dist_err) < self.tolerance:
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(
                data=f'OK: dist={distance:.2f}m'))
            return

        if abs(angle_err) > math.radians(5):
            angular = max(-self.max_angular, min(self.max_angular,
                                                  self.kp_ang * angle_err))
            cmd.angular.z = angular
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(
                data=f'TURN: {math.degrees(angle_err):.0f}° ω={angular:.2f}'))
            return

        linear = max(-self.max_speed, min(self.max_speed,
                                           self.kp_lin * dist_err))
        angular = max(-self.max_angular, min(self.max_angular,
                                              self.kp_ang * angle_err))
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_cmd.publish(cmd)

        self.pub_status.publish(String(
            data=f'FOLLOW: d={distance:.2f}m v={linear:.2f} ω={angular:.2f}'))

    def _publish_viz(self):
        with self._lock:
            robot_pos = self.robot_pos
            target_pos = self.target_pos

        if robot_pos is None or target_pos is None:
            return

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0

        # 로봇 (파랑)
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'robot'
        m.id = mid; mid += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = robot_pos[0]
        m.pose.position.y = robot_pos[1]
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.9)
        m.lifetime.sec = 1
        ma.markers.append(m)

        # 타겟 (빨강)
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'target'
        m.id = mid; mid += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = target_pos[0]
        m.pose.position.y = target_pos[1]
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
        m.lifetime.sec = 1
        ma.markers.append(m)

        # 연결선
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'line'
        m.id = mid; mid += 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        colors = {self.IDLE: (0.5, 0.5, 0.5), self.FOLLOWING: (0.0, 1.0, 0.0),
                  self.STOPPED: (1.0, 0.0, 0.0)}
        c = colors.get(self.state, (0.5, 0.5, 0.5))
        m.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.8)
        m.points.append(Point(x=robot_pos[0], y=robot_pos[1], z=0.05))
        m.points.append(Point(x=target_pos[0], y=target_pos[1], z=0.05))
        m.lifetime.sec = 1
        ma.markers.append(m)

        # 거리 텍스트
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'info'
        m.id = mid; mid += 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = (robot_pos[0] + target_pos[0]) / 2
        m.pose.position.y = (robot_pos[1] + target_pos[1]) / 2
        m.pose.position.z = 0.4
        m.pose.orientation.w = 1.0
        m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = f'{dist:.2f}m'
        m.lifetime.sec = 1
        ma.markers.append(m)

        # 앵커
        for an_id, pos in self._anchors.items():
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'anchors'
            m.id = mid; mid += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = pos['x']
            m.pose.position.y = pos['y']
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8)
            m.lifetime.sec = 1
            ma.markers.append(m)

            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'anchor_labels'
            m.id = mid; mid += 1
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = pos['x']
            m.pose.position.y = pos['y']
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.z = 0.12
            m.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)
            m.text = an_id
            m.lifetime.sec = 1
            ma.markers.append(m)

        self.pub_markers.publish(ma)

    def destroy_node(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.pub_cmd.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
