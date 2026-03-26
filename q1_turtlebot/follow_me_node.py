"""
예제 2 — UWB Follow Me (실제 로봇).

리스너로부터 인적 태그와 로봇 태그 위치를 수신하여
IQR 필터링 후 로봇이 사람을 1.0m 거리로 추종합니다.

실행:
  ros2 launch q1_turtlebot follow_me.launch.py

필요 토픽:
  q1/tags     (q1_gateway_msgs/TagArray) — Q1 리스너 전체 태그
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from q1_gateway_msgs.msg import TagArray


class IQRFilter:
    """슬라이딩 윈도우 IQR 필터."""

    def __init__(self, window_size=20):
        self.window_size = window_size
        self.buf_x = deque(maxlen=window_size)
        self.buf_y = deque(maxlen=window_size)

    def update(self, x, y):
        """IQR 범위 내이면 필터된 (x, y) 반환, 이상치이면 None."""
        if len(self.buf_x) < 5:
            self.buf_x.append(x)
            self.buf_y.append(y)
            return x, y

        for buf, val in [(self.buf_x, x), (self.buf_y, y)]:
            arr = np.array(buf)
            q1 = np.percentile(arr, 25)
            q3 = np.percentile(arr, 75)
            iqr = q3 - q1
            lower = q1 - 1.5 * iqr
            upper = q3 + 1.5 * iqr
            if val < lower or val > upper:
                return None

        self.buf_x.append(x)
        self.buf_y.append(y)
        return x, y


class FollowMeNode(Node):
    IDLE = 0
    FOLLOWING = 1
    STOPPED = 2

    def __init__(self):
        super().__init__('follow_me')

        self.declare_parameter('robot_tag_id', '25A0')
        self.declare_parameter('target_tag_id', '1E52')
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('distance_tolerance', 1.0)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('qf_threshold', 30)
        self.declare_parameter('timeout_sec', 2.0)
        self.declare_parameter('iqr_window', 20)

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
        iqr_window = self.get_parameter('iqr_window').value

        self.robot_pos = None
        self.target_pos = None
        self.robot_qf = 0
        self.target_qf = 0
        self.robot_yaw = 0.0
        self.last_target_time = 0.0
        self.state = self.IDLE
        self.robot_dropped = 0
        self.target_dropped = 0

        # IQR 필터
        self.robot_filter = IQRFilter(iqr_window)
        self.target_filter = IQRFilter(iqr_window)

        # 구독
        self.create_subscription(TagArray, 'q1/tags', self._tags_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # 발행
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/follow_me/status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/follow_me/markers', 10)

        self.create_timer(0.1, self._control_loop)
        self.create_timer(0.5, self._publish_viz)

        self.get_logger().info(
            f'Follow-Me 시작  로봇={self.robot_tag}  타겟={self.target_tag}\n'
            f'  목표거리={self.target_dist}m  허용오차=±{self.tolerance}m\n'
            f'  QF≥{self.qf_thresh}  timeout={self.timeout}s\n'
            f'  IQR window={iqr_window}')

    def _tags_cb(self, msg: TagArray):
        for tag in msg.tags:
            if tag.id == self.robot_tag:
                result = self.robot_filter.update(tag.x, tag.y)
                if result is None:
                    self.robot_dropped += 1
                    continue
                self.robot_pos = list(result)
                self.robot_qf = tag.quality

            elif tag.id == self.target_tag:
                result = self.target_filter.update(tag.x, tag.y)
                if result is None:
                    self.target_dropped += 1
                    continue
                self.target_pos = list(result)
                self.target_qf = tag.quality
                self.last_target_time = time.time()

    def _odom_cb(self, msg: Odometry):
        """로봇 heading(yaw)을 odom에서 추출."""
        q = msg.pose.pose.orientation
        # quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _control_loop(self):
        cmd = Twist()

        # 위치 미수신
        if self.robot_pos is None or self.target_pos is None:
            self.state = self.IDLE
            self.pub_cmd.publish(cmd)
            return

        # QF 체크
        if self.target_qf < self.qf_thresh or self.robot_qf < self.qf_thresh:
            self.state = self.STOPPED
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(
                data=f'STOPPED: QF 저하 (robot={self.robot_qf}, target={self.target_qf})'))
            return

        # 타임아웃 체크
        if time.time() - self.last_target_time > self.timeout:
            self.state = self.STOPPED
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(data='STOPPED: 태그 타임아웃'))
            return

        # 거리/방향 계산
        dx = self.target_pos[0] - self.robot_pos[0]
        dy = self.target_pos[1] - self.robot_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        # 타겟까지의 절대 각도
        target_angle = math.atan2(dy, dx)

        # 로봇 heading 기준 상대 각도 (-pi ~ pi)
        angle_err = target_angle - self.robot_yaw
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        self.state = self.FOLLOWING

        # 거리 오차
        dist_err = distance - self.target_dist

        # 허용 오차 내면 정지
        if abs(dist_err) < 0.1:
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(
                data=f'FOLLOWING: dist={distance:.2f}m (목표 범위 내)'))
            return

        # 각도 오차가 크면 먼저 회전
        if abs(angle_err) > math.radians(30):
            angular = max(-self.max_angular, min(self.max_angular,
                                                  self.kp_ang * angle_err))
            cmd.angular.z = angular
            self.pub_cmd.publish(cmd)
            self.pub_status.publish(String(
                data=f'TURNING: dist={distance:.2f}m  angle_err={math.degrees(angle_err):.1f}°  '
                     f'ω={angular:.2f}'))
            return

        # P 제어
        linear = max(-self.max_speed, min(self.max_speed,
                                           self.kp_lin * dist_err))
        angular = max(-self.max_angular, min(self.max_angular,
                                              self.kp_ang * angle_err))

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_cmd.publish(cmd)

        self.pub_status.publish(String(
            data=f'FOLLOWING: dist={distance:.2f}m  err={dist_err:+.2f}m  '
                 f'yaw={math.degrees(self.robot_yaw):.1f}°  '
                 f'v={linear:.2f}  ω={angular:.2f}  '
                 f'dropped(R={self.robot_dropped},T={self.target_dropped})'))

    def _publish_viz(self):
        if self.robot_pos is None or self.target_pos is None:
            return

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 로봇 위치 (파랑, 점)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'robot'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.robot_pos[0]
        m.pose.position.y = self.robot_pos[1]
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.8)
        ma.markers.append(m)

        # 타겟 위치 (빨강, 점)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.target_pos[0]
        m.pose.position.y = self.target_pos[1]
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.8)
        ma.markers.append(m)

        # 목표 거리 원
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'target_ring'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.target_pos[0]
        m.pose.position.y = self.target_pos[1]
        m.pose.position.z = 0.01
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = self.target_dist * 2
        m.scale.z = 0.005
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)
        ma.markers.append(m)

        # 연결선
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'line'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        state_colors = {
            self.IDLE: ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5),
            self.FOLLOWING: ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
            self.STOPPED: ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),
        }
        m.color = state_colors.get(self.state, state_colors[self.IDLE])
        m.points.append(Point(x=self.robot_pos[0], y=self.robot_pos[1], z=0.1))
        m.points.append(Point(x=self.target_pos[0], y=self.target_pos[1], z=0.1))
        ma.markers.append(m)

        # 텍스트 정보
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = now
        m.ns = 'info'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = (self.robot_pos[0] + self.target_pos[0]) / 2
        m.pose.position.y = (self.robot_pos[1] + self.target_pos[1]) / 2
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.z = 0.12
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        dx = self.target_pos[0] - self.robot_pos[0]
        dy = self.target_pos[1] - self.robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        m.text = f'dist={dist:.2f}m\nIQR dropped R={self.robot_dropped} T={self.target_dropped}'
        ma.markers.append(m)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 정지
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
