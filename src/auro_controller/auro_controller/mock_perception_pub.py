# mock_perception_pub.py
# Fake perception publisher for offline testing. Publishes synthetic data on
# the two topics the controller reads: /perception/objects/aruco_id_0 and
# /perception/aruco/landmarks.
#
# The banana object is intentionally out of bounds so you can verify that
# the controller drops it and logs a BOUNDS FAIL warn.
#
# Usage:
#   Terminal 1: ros2 launch auro_controller controller.launch.py
#   Terminal 2: ros2 run auro_controller mock_perception_pub
#   Terminal 3: ros2 topic echo /detected_objects
#               (should show apple + water_bottle only)

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# Mock table corners (ArUco frame, metres).
# Marker 0 is the origin (0,0,0), not included in this list.
# Markers 1-3 are the other three corners.
MOCK_LANDMARKS = [
    {'id': 1, 'x':  0.0,  'y': -0.6,  'z': 0.0},
    {'id': 2, 'x':  1.0,  'y': -0.6,  'z': 0.0},
    {'id': 3, 'x':  1.0,  'y':  0.0,  'z': 0.0},
]

# Mock objects (ArUco frame, metres).
# Table XY extent from corners above: x=[0, 1], y=[-0.6, 0], plus 0.05m padding.
# apple and water_bottle are within bounds; banana is out of bounds (x=1.5).
MOCK_OBJECTS = [
    {'label': 'apple',        'x': 0.10, 'y': -0.20, 'z': 0.05},
    {'label': 'water_bottle', 'x': 0.30, 'y': -0.10, 'z': 0.11},
    {'label': 'banana',       'x': 1.50, 'y': -0.05, 'z': 0.04},  # out of bounds
]

PUBLISH_RATE_SEC = 2.0


class MockPerceptionPub(Node):

    def __init__(self):
        super().__init__('mock_perception_pub')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._obj_pub = self.create_publisher(
            String, '/perception/objects/aruco_id_0', qos)
        self._lm_pub = self.create_publisher(
            String, '/perception/aruco/landmarks', qos)

        self.create_timer(PUBLISH_RATE_SEC, self._publish)

        self.get_logger().info(
            f'MockPerceptionPub ready, publishing every {PUBLISH_RATE_SEC}s\n'
            f'  Objects:  {[o["label"] for o in MOCK_OBJECTS]}\n'
            f'  Expected pass: apple, water_bottle\n'
            f'  Expected drop: banana  (x=1.5 outside table x=[0, 1] + padding)'
        )

    def _publish(self):
        obj_msg = String()
        obj_msg.data = json.dumps(MOCK_OBJECTS)
        self._obj_pub.publish(obj_msg)

        lm_msg = String()
        lm_msg.data = json.dumps(MOCK_LANDMARKS)
        self._lm_pub.publish(lm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPerceptionPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
