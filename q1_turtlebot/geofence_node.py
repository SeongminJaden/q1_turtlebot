"""
예제 3 — Geofencing (실제 로봇).

YAML 설정 파일로 정의된 가상 구역에 로봇이 진입/이탈 시
정지, 감속, 경고 등의 동작을 수행합니다.

실행:
  ros2 launch q1_turtlebot geofence.launch.py

  # 조종 (geofence 필터 거침):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_input

데이터 흐름:
  teleop(/cmd_vel_input) → geofence 필터 → /cmd_vel → 로봇

필요 토픽:
  q1/tags          (TagArray) — Q1 리스너
  /cmd_vel_input   (Twist)   — 원본 조종 명령
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from q1_gateway_msgs.msg import TagArray


class Zone:
    def __init__(self, name, zone_type, x_min, y_min, x_max, y_max, speed_limit=1.0):
        self.name = name
        self.type = zone_type  # forbidden, slowdown, warn, allowed
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.speed_limit = speed_limit

    def contains(self, x, y):
        return (self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max)


class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence')

        self.declare_parameter('robot_tag_id', 'DEV_TAG_00')
        self.declare_parameter('qf_threshold', 30)

        # 구역 파라미터 (YAML에서 로드)
        self.declare_parameter('zones', [])

        self.robot_tag = self.get_parameter('robot_tag_id').value
        self.qf_thresh = self.get_parameter('qf_threshold').value

        # 구역 파싱
        self.zones = self._load_zones()

        self.robot_pos = None
        self.robot_qf = 0
        self.active_zones = []
        self.blocked = False

        # 구독
        self.create_subscription(TagArray, 'q1/tags', self._tags_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_input', self._cmd_input_cb, 10)

        # 발행
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_alert = self.create_publisher(String, '/geofence/alert', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/geofence/markers', 10)

        self.create_timer(0.5, self._publish_viz)

        for z in self.zones:
            self.get_logger().info(
                f'  구역 [{z.name}] type={z.type}  '
                f'({z.x_min},{z.y_min})-({z.x_max},{z.y_max})  '
                f'speed={z.speed_limit}')
        self.get_logger().info(
            f'Geofence 시작  {len(self.zones)}개 구역  tag={self.robot_tag}\n'
            f'  조종: ros2 run teleop_twist_keyboard teleop_twist_keyboard '
            f'--ros-args -r cmd_vel:=cmd_vel_input')

    def _load_zones(self):
        """config에서 구역 정의 로드."""
        zones = []
        # YAML에서 zone_count, zone_N_* 형태로 로드
        self.declare_parameter('zone_count', 0)
        n = self.get_parameter('zone_count').value

        for i in range(n):
            prefix = f'zone_{i}_'
            for p in ['name', 'type', 'x_min', 'y_min', 'x_max', 'y_max', 'speed_limit']:
                self.declare_parameter(prefix + p,
                                       '' if p in ('name', 'type') else 0.0)
            zones.append(Zone(
                name=self.get_parameter(prefix + 'name').value,
                zone_type=self.get_parameter(prefix + 'type').value,
                x_min=self.get_parameter(prefix + 'x_min').value,
                y_min=self.get_parameter(prefix + 'y_min').value,
                x_max=self.get_parameter(prefix + 'x_max').value,
                y_max=self.get_parameter(prefix + 'y_max').value,
                speed_limit=self.get_parameter(prefix + 'speed_limit').value,
            ))
        return zones

    def _tags_cb(self, msg: TagArray):
        for tag in msg.tags:
            if tag.id == self.robot_tag:
                self.robot_pos = [tag.x, tag.y]
                self.robot_qf = tag.quality
                self._check_zones()

    def _check_zones(self):
        if self.robot_pos is None:
            return

        self.active_zones = []
        self.blocked = False

        for z in self.zones:
            if z.contains(self.robot_pos[0], self.robot_pos[1]):
                self.active_zones.append(z)

                if z.type == 'forbidden':
                    self.blocked = True
                    self.pub_alert.publish(String(
                        data=f'⛔ 진입금지 구역 [{z.name}] 진입! 정지!'))
                elif z.type == 'warn':
                    self.pub_alert.publish(String(
                        data=f'⚠ 경고 구역 [{z.name}] 진입'))

    def _cmd_input_cb(self, msg: Twist):
        """조종 명령을 geofence 필터링 후 전달."""
        cmd = Twist()

        # QF 저하 → 정지
        if self.robot_qf < self.qf_thresh and self.robot_qf > 0:
            self.pub_cmd.publish(cmd)
            return

        # forbidden 구역 → 정지
        if self.blocked:
            self.pub_cmd.publish(cmd)
            return

        # slowdown 구역 → 속도 제한
        speed_limit = 1.0
        for z in self.active_zones:
            if z.type == 'slowdown':
                speed_limit = min(speed_limit, z.speed_limit)

        # 속도 제한 적용
        speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        if speed > speed_limit and speed > 0:
            scale = speed_limit / speed
            cmd.linear.x = msg.linear.x * scale
            cmd.linear.y = msg.linear.y * scale
        else:
            cmd.linear.x = msg.linear.x
            cmd.linear.y = msg.linear.y

        cmd.angular.z = msg.angular.z
        self.pub_cmd.publish(cmd)

    def _publish_viz(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        zone_colors = {
            'forbidden': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3),
            'slowdown': ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.2),
            'warn': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.2),
            'allowed': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.15),
        }

        for i, z in enumerate(self.zones):
            # 구역 사각형
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'zones'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = (z.x_min + z.x_max) / 2
            m.pose.position.y = (z.y_min + z.y_max) / 2
            m.pose.position.z = 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = z.x_max - z.x_min
            m.scale.y = z.y_max - z.y_min
            m.scale.z = 0.02

            active = z in self.active_zones
            color = zone_colors.get(z.type, zone_colors['allowed'])
            if active:
                color = ColorRGBA(r=color.r, g=color.g, b=color.b,
                                  a=min(1.0, color.a * 3))
            m.color = color
            ma.markers.append(m)

            # 라벨
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'zone_labels'
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = (z.x_min + z.x_max) / 2
            m.pose.position.y = (z.y_min + z.y_max) / 2
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.z = 0.15
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            status = ' [진입]' if active else ''
            m.text = f'{z.name} ({z.type}){status}'
            ma.markers.append(m)

        # 로봇 위치
        if self.robot_pos:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'robot'
            m.id = 0
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = self.robot_pos[0]
            m.pose.position.y = self.robot_pos[1]
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.25
            m.scale.z = 0.2
            c = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9) if self.blocked else \
                ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.9)
            m.color = c
            ma.markers.append(m)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
