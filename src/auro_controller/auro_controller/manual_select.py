# manual_select.py
# Stand-in for the LLM node during controller testing. Subscribes to
# /detected_objects, finds the requested object by name, and publishes
# it to /selected_object just as the real LLM node would.
#
# Usage:
#   ros2 run auro_controller manual_select <object_name>
#
# Example:
#   ros2 run auro_controller manual_select orange

import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ManualSelect(Node):

    def __init__(self, target: str):
        super().__init__('manual_select')
        self._target = target.strip().lower()
        self._done = False

        self._pub = self.create_publisher(String, '/selected_object', 10)
        self.create_subscription(String, '/detected_objects', self._on_objects, 10)

        self.get_logger().info(
            f'Waiting for /detected_objects to find "{self._target}"...')

    def _on_objects(self, msg: String):
        if self._done:
            return

        try:
            objects = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Bad JSON on /detected_objects: {exc}')
            return

        names = [o.get('name') for o in objects]
        match = next((o for o in objects
                      if o.get('name', '').lower() == self._target), None)

        if match is None:
            self.get_logger().warn(
                f'"{self._target}" not in current object list: {names}\n'
                f'Run again after the controller republishes, or check the name.')
            return

        self._done = True
        out = String()
        out.data = json.dumps({
            'name': match['name'],
            'x':    match['x'],
            'y':    match['y'],
            'z':    match['z'],
        })
        self._pub.publish(out)
        self.get_logger().info(
            f'Published /selected_object: {out.data}')


def main(args=None):
    if len(sys.argv) < 2:
        print('Usage: ros2 run auro_controller manual_select <object_name>')
        sys.exit(1)

    target = sys.argv[1]
    rclpy.init(args=args)
    node = ManualSelect(target)

    try:
        # Spin until we've published or the user ctrl-Cs
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
