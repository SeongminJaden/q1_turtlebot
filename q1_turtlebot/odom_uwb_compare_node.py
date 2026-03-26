"""
예제 1 — Odom vs UWB 위치 비교 시각화.

로봇의 추측 항법(Odom)과 UWB 위치를 동시에 RViz2에 표시합니다.
UWB가 실내에서 Odom 드리프트를 보정하는 효과를 시각적으로 확인할 수 있습니다.

실행:
  ros2 launch q1_turtlebot odom_uwb_compare.launch.py

필요 토픽:
  /odom        (nav_msgs/Odometry)  — TurtleBot3 오도메트리
  q1/tag/pose  (q1_gateway_msgs/Tag) — Q1 리스너 UWB 위치
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from q1_gateway_msgs.msg import Tag


class OdomUwbCompareNode(Node):
    def __init__(self):
        super().__init__('odom_uwb_compare')

        self.declare_parameter('robot_tag_id', 'DEV_TAG_00')
        self.declare_parameter('max_path_length', 500)

        self.robot_tag_id = self.get_parameter('robot_tag_id').value
        self.max_path = self.get_parameter('max_path_length').value

        # 상태
        self.odom_pos = [0.0, 0.0, 0.0]
        self.uwb_pos = [0.0, 0.0, 0.0]
        self.odom_path = Path()
        self.uwb_path = Path()
        self.odom_path.header.frame_id = 'odom'
        self.uwb_path.header.frame_id = 'odom'

        # 좌표계 오프셋 (첫 UWB 수신 시 계산)
        self.offset_computed = False
        self.offset_x = 0.0
        self.offset_y = 0.0

        # 구독
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Tag, 'q1/tag/pose', self._uwb_cb, 10)

        # 발행
        self.pub_odom_path = self.create_publisher(Path, '/compare/odom_path', 10)
        self.pub_uwb_path = self.create_publisher(Path, '/compare/uwb_path', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/compare/markers', 10)
        self.pub_drift = self.create_publisher(String, '/compare/drift', 10)

        # TF
        self.tf_br = TransformBroadcaster(self)
        self.static_tf_br = StaticTransformBroadcaster(self)

        self.create_timer(0.2, self._publish_viz)

        self.get_logger().info(
            f'Odom-UWB 비교 시작  tag={self.robot_tag_id}')

    def _odom_cb(self, msg: Odometry):
        self.odom_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z]

        # Odom 경로 추가
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.odom_path.poses.append(ps)
        if len(self.odom_path.poses) > self.max_path:
            self.odom_path.poses.pop(0)

    def _uwb_cb(self, msg: Tag):
        if msg.id != self.robot_tag_id:
            return

        # 첫 수신 시 좌표계 오프셋 계산
        if not self.offset_computed:
            self.offset_x = msg.x - self.odom_pos[0]
            self.offset_y = msg.y - self.odom_pos[1]
            self.offset_computed = True
            self.get_logger().info(
                f'좌표계 오프셋: ({self.offset_x:.2f}, {self.offset_y:.2f})')

            # uwb_map → odom 정적 변환 발행
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'uwb_map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = -self.offset_x
            t.transform.translation.y = -self.offset_y
            t.transform.rotation.w = 1.0
            self.static_tf_br.sendTransform(t)

        # UWB 위치 (odom 프레임으로 변환)
        self.uwb_pos = [
            msg.x - self.offset_x,
            msg.y - self.offset_y,
            msg.z]

        # UWB 경로 추가
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose.position.x = self.uwb_pos[0]
        ps.pose.position.y = self.uwb_pos[1]
        ps.pose.orientation.w = 1.0
        self.uwb_path.poses.append(ps)
        if len(self.uwb_path.poses) > self.max_path:
            self.uwb_path.poses.pop(0)

        # UWB TF 발행
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'uwb_position'
        t.transform.translation.x = self.uwb_pos[0]
        t.transform.translation.y = self.uwb_pos[1]
        t.transform.rotation.w = 1.0
        self.tf_br.sendTransform(t)

    def _publish_viz(self):
        now = self.get_clock().now().to_msg()

        # 경로 발행
        self.odom_path.header.stamp = now
        self.uwb_path.header.stamp = now
        self.pub_odom_path.publish(self.odom_path)
        self.pub_uwb_path.publish(self.uwb_path)

        # 드리프트 계산
        drift = math.sqrt(
            (self.odom_pos[0] - self.uwb_pos[0])**2 +
            (self.odom_pos[1] - self.uwb_pos[1])**2)

        self.pub_drift.publish(String(data=f'드리프트: {drift:.3f}m'))

        # 마커
        ma = MarkerArray()

        # Odom 위치 (빨강)
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'odom_pos'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.odom_pos[0]
        m.pose.position.y = self.odom_pos[1]
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
        ma.markers.append(m)

        # UWB 위치 (초록)
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'uwb_pos'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.uwb_pos[0]
        m.pose.position.y = self.uwb_pos[1]
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.9)
        ma.markers.append(m)

        # 드리프트 표시 텍스트
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'drift_text'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = (self.odom_pos[0] + self.uwb_pos[0]) / 2
        m.pose.position.y = (self.odom_pos[1] + self.uwb_pos[1]) / 2
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        m.text = f'Drift: {drift:.3f}m'
        ma.markers.append(m)

        # 드리프트 연결선
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = now
        m.ns = 'drift_line'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.02
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
        from geometry_msgs.msg import Point
        m.points.append(Point(x=self.odom_pos[0], y=self.odom_pos[1]))
        m.points.append(Point(x=self.uwb_pos[0], y=self.uwb_pos[1]))
        ma.markers.append(m)

        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = OdomUwbCompareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
