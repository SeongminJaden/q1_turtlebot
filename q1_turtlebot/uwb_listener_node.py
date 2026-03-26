"""
UWB Listener Node (standalone, q1_gateway_msgs 불필요).

Listener USB 시리얼에서 태그 위치를 읽어
visualization_msgs/MarkerArray로 RViz2에 표시합니다.

실행:
  ros2 run q1_turtlebot uwb_listener
  ros2 run q1_turtlebot uwb_listener --ros-args -p serial_port:=/dev/ttyUSB1

Serial format:
  POS,<idx>,<tagID>,<x>,<y>,<z>,<qf>[,<flags>]
"""

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    import serial
except ImportError:
    raise ImportError(
        "pyserial is required. Install with: pip3 install pyserial"
    )


class UwbListenerNode(Node):

    COLORS = [
        ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9),  # red
        ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.9),  # blue
        ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.9),  # green
        ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.9),  # yellow
        ColorRGBA(r=1.0, g=0.2, b=1.0, a=0.9),  # magenta
        ColorRGBA(r=0.2, g=1.0, b=1.0, a=0.9),  # cyan
    ]

    def __init__(self):
        super().__init__('uwb_listener')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'map')

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Anchors (mm → m)
        self._anchors = {
            'AN0': {'x': 0.0,  'y': 0.0,   'z': 0.0},
            'AN1': {'x': 6.1,  'y': 0.0,   'z': 0.0},
            'AN2': {'x': 6.1,  'y': 7.04,  'z': 0.0},
            'AN3': {'x': 0.0,  'y': 7.04,  'z': 0.0},
        }

        # Publishers
        self.pub_markers = self.create_publisher(MarkerArray, 'uwb/markers', 10)
        self.pub_status = self.create_publisher(String, 'uwb/status', 10)

        # Tag data
        self._tags = {}
        self._tag_colors = {}
        self._lock = threading.Lock()

        # Publish markers at 5Hz
        self.create_timer(0.2, self._publish_markers)

        # Serial
        self.ser = None
        self._running = True
        self._connect_serial()

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1.0,
            )
            self.get_logger().info(
                f'Listener connected: {self.port} @ {self.baud}')

            import time
            self.ser.write(b'\r')
            time.sleep(0.3)
            self.ser.write(b'lep\r')
            time.sleep(0.3)
            self.ser.reset_input_buffer()

            self._read_thread = threading.Thread(
                target=self._read_loop, daemon=True)
            self._read_thread.start()

        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')

    def _read_loop(self):
        while self._running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith('POS'):
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
            z = float(parts[5])
            qf = int(parts[6])
        except (ValueError, IndexError):
            return

        with self._lock:
            self._tags[tag_id] = {'x': x, 'y': y, 'z': z, 'qf': qf}
            if tag_id not in self._tag_colors:
                idx = len(self._tag_colors) % len(self.COLORS)
                self._tag_colors[tag_id] = self.COLORS[idx]

        self.get_logger().info(
            f'TAG {tag_id}: ({x:.2f}, {y:.2f}, {z:.2f}) qf={qf}')

    def _publish_markers(self):
        with self._lock:
            if not self._tags:
                return

            ma = MarkerArray()
            now = self.get_clock().now().to_msg()
            marker_id = 0

            for tag_id, data in self._tags.items():
                color = self._tag_colors.get(tag_id, self.COLORS[0])

                # Sphere marker (dot)
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp = now
                m.ns = tag_id
                m.id = marker_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = data['x']
                m.pose.position.y = data['y']
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0
                m.scale.x = 0.15
                m.scale.y = 0.15
                m.scale.z = 0.15
                m.color = color
                m.lifetime.sec = 1
                ma.markers.append(m)
                marker_id += 1

                # Text label
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp = now
                m.ns = tag_id + '_label'
                m.id = marker_id
                m.type = Marker.TEXT_VIEW_FACING
                m.action = Marker.ADD
                m.pose.position.x = data['x']
                m.pose.position.y = data['y']
                m.pose.position.z = 0.3
                m.pose.orientation.w = 1.0
                m.scale.z = 0.15
                m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                m.text = f'{tag_id}\n({data["x"]:.2f}, {data["y"]:.2f})\nQF={data["qf"]}'
                m.lifetime.sec = 1
                ma.markers.append(m)
                marker_id += 1

            # Anchor markers (cube, gray)
            for an_id, an_pos in self._anchors.items():
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp = now
                m.ns = 'anchors'
                m.id = marker_id
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = an_pos['x']
                m.pose.position.y = an_pos['y']
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0
                m.scale.x = 0.2
                m.scale.y = 0.2
                m.scale.z = 0.2
                m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.9)
                m.lifetime.sec = 1
                ma.markers.append(m)
                marker_id += 1

                # Anchor label
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp = now
                m.ns = 'anchor_labels'
                m.id = marker_id
                m.type = Marker.TEXT_VIEW_FACING
                m.action = Marker.ADD
                m.pose.position.x = an_pos['x']
                m.pose.position.y = an_pos['y']
                m.pose.position.z = 0.35
                m.pose.orientation.w = 1.0
                m.scale.z = 0.15
                m.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)
                m.text = an_id
                m.lifetime.sec = 1
                ma.markers.append(m)
                marker_id += 1

            self.pub_markers.publish(ma)

            # Status
            status_parts = [f'{tid}:({d["x"]:.2f},{d["y"]:.2f}) qf={d["qf"]}'
                            for tid, d in self._tags.items()]
            self.pub_status.publish(String(data=' | '.join(status_parts)))

    def destroy_node(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UwbListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
