# controller_node.py
# Filters detected objects to those within the table boundary and forwards
# them to the LLM. Manages pipeline state between pick cycles.
#
# Perception forwarding is NOT gated on internal state — we always accept
# the freshest perception (subject to a hash+time debounce). The internal
# state machine is used only to validate /selected_object messages. This
# avoids a deadlock where the controller would stay in OBJECTS_PUBLISHED
# after the arm finished a pick and refuse to refresh the scene.

from __future__ import annotations

import hashlib
import json
import threading
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from auro_controller.frame_transform import FrameTransformer


# maps detection labels to normalized object type strings
LABEL_TO_OBJECT_TYPE: dict[str, str] = {
    'foam_ball':    'foam_ball',
    'ball':         'foam_ball',
    'apple':        'apple',
    'orange':       'orange',
    'banana':       'small_box',
    'coffee_cup':   'coffee_cup',
    'cup':          'coffee_cup',
    'mug':          'coffee_cup',
    'water_bottle': 'water_bottle',
    'bottle':       'water_bottle',
    'small_box':    'small_box',
    'box':          'small_box',
}



class ControllerState(Enum):
    WAITING_FOR_OBJECTS = auto()
    OBJECTS_PUBLISHED   = auto()
    STOPPED             = auto()



class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('enable_arm', False)

        # Z bounds for the table (metres, ArUco frame).
        self.declare_parameter('table_z_min',          -0.05)
        self.declare_parameter('table_z_max',           0.60)

        # Extra XY margin around the ArUco-corner bounding box (metres).
        self.declare_parameter('table_bounds_padding',  0.05)

        # Minimum seconds between identical scene republications.
        self.declare_parameter('debounce_min_sec',      1.0)

        # Frame transform: ArUco marker 0 → arm base_link (static offset mode).
        # offset_yaw=-π/2 because arm +x ∥ aruco_0 +y, arm +y ∥ aruco_0 −x.
        # offset_y = distance from aruco_0 origin to arm base along aruco_0 +x.
        self.declare_parameter('use_tf',      False)
        self.declare_parameter('offset_x',    0.0)
        self.declare_parameter('offset_y',    0.35)
        self.declare_parameter('offset_z',    0.0)
        self.declare_parameter('offset_yaw', -1.5708)   # −π/2

        self._enable_arm   = self.get_parameter('enable_arm').value
        self._table_z_min  = self.get_parameter('table_z_min').value
        self._table_z_max  = self.get_parameter('table_z_max').value
        self._bounds_pad   = self.get_parameter('table_bounds_padding').value
        self._debounce_sec = self.get_parameter('debounce_min_sec').value

        self._transformer = FrameTransformer(
            node=self,
            use_tf=self.get_parameter('use_tf').value,
            offset_x=self.get_parameter('offset_x').value,
            offset_y=self.get_parameter('offset_y').value,
            offset_z=self.get_parameter('offset_z').value,
            offset_yaw=self.get_parameter('offset_yaw').value,
            perception_frame='aruco_id_0',
            ground_frame='base_link',
        )

        self._state         = ControllerState.WAITING_FOR_OBJECTS
        self._state_lock    = threading.Lock()
        self._stop_flag     = False
        self._current_objs: list[dict] = []
        self._last_hash     = ''
        self._last_pub_time = 0.0

        self._landmarks: list[dict] | None = None
        self._landmarks_lock = threading.Lock()

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Latched QoS for /detected_objects so late-joining subscribers
        # (LLM or arm node starting after the desktop) get the last message.
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._detected_pub = self.create_publisher(
            String, '/detected_objects', qos_latched)
        self._status_pub = self.create_publisher(
            String, '/arm_status', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/controller/object_markers', 10)

        self.create_subscription(
            String, '/perception/objects/aruco_id_0',
            self._on_perception_update,
            qos_reliable)

        self.create_subscription(
            String, '/perception/aruco/landmarks',
            self._on_landmarks,
            qos_reliable)

        self.create_subscription(
            String, '/selected_object',
            self._on_selected_object,
            10)

        self.create_subscription(
            Bool, '/arm_stop',
            self._on_arm_stop,
            10)

        self._publish_status('ready')

        self.get_logger().info(
            f'ControllerNode ready | arm={self._enable_arm} '
            f'z=[{self._table_z_min}, {self._table_z_max}] '
            f'bounds_pad={self._bounds_pad}'
        )

    def _on_landmarks(self, msg: String):
        try:
            landmarks = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Bad JSON on /perception/aruco/landmarks: {exc}')
            return
        with self._landmarks_lock:
            self._landmarks = landmarks

    def _on_perception_update(self, msg: String):
        # Hard gate: STOPPED means an emergency stop. Don't push fresh scenes
        # into a halted pipeline. All other states (including OBJECTS_PUBLISHED)
        # still accept perception so the scene can update in place when the
        # world changes between picks.
        with self._state_lock:
            if self._state == ControllerState.STOPPED:
                return

        try:
            raw_objects: list[dict] = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Bad JSON on /perception/objects/aruco_id_0: {exc}')
            return

        if not isinstance(raw_objects, list) or not raw_objects:
            return

        validated: list[dict] = []
        validated_positions: list[dict] = []
        for obj in raw_objects:
            result = self._process_one_object(obj)
            if result is not None:
                llm_dict, pos = result
                validated.append(llm_dict)
                validated_positions.append(pos)

        if not validated:
            self.get_logger().warn(
                'All detected objects failed table-bounds check. '
                'Nothing forwarded to LLM.',
                throttle_duration_sec=5.0)
            return

        self._publish_markers(validated_positions)

        scene_hash = hashlib.md5(
            json.dumps(validated, sort_keys=True).encode()
        ).hexdigest()

        now = time.monotonic()
        # Debounce: skip if the scene is identical AND we published recently.
        # If the hash differs (something actually changed) we always publish.
        # If the hash matches but the debounce window has elapsed, we still
        # republish — this keeps the latched topic fresh for late subscribers.
        if (scene_hash == self._last_hash and
                (now - self._last_pub_time) < self._debounce_sec):
            return

        self._last_hash     = scene_hash
        self._last_pub_time = now

        with self._state_lock:
            self._current_objs = validated
            # Mark that the LLM has a scene to choose from. We do not gate
            # /selected_object on this strictly — see _on_selected_object —
            # but it's still useful as a "have we ever published" indicator.
            self._state = ControllerState.OBJECTS_PUBLISHED

        out_msg = String()
        out_msg.data = json.dumps(validated)
        self._detected_pub.publish(out_msg)

        names = [o['name'] for o in validated]
        self.get_logger().info(
            f'Published {len(validated)} object(s) to /detected_objects: {names}')

    def _process_one_object(self, obj: dict) -> tuple | None:
        label = obj.get('label', 'unknown')
        x = obj.get('x')
        y = obj.get('y')
        z = obj.get('z')

        if x is None or y is None or z is None:
            self.get_logger().warn(f'Object "{label}" missing x/y/z, skipping.')
            return None

        x, y, z = float(x), float(y), float(z)

        if not self._is_on_table(label, x, y, z):
            return None

        tx, ty, tz = self._transformer.transform(x, y, z)
        self.get_logger().info(
            f'"{label}": ArUco({x:.3f}, {y:.3f}, {z:.3f}) '
            f'→ base_link({tx:.3f}, {ty:.3f}, {tz:.3f})')

        obj_type = LABEL_TO_OBJECT_TYPE.get(label, 'small_box')
        llm_dict = {
            'name': label,
            'x':    round(tx, 4),
            'y':    round(ty, 4),
            'z':    round(tz, 4),
            'type': obj_type,
        }
        return llm_dict, {'x': tx, 'y': ty, 'z': tz}

    def _is_on_table(self, label: str, x: float, y: float, z: float) -> bool:
        if not (self._table_z_min <= z <= self._table_z_max):
            self.get_logger().warn(
                f'BOUNDS FAIL: "{label}" at ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'z outside [{self._table_z_min}, {self._table_z_max}]')
            return False

        with self._landmarks_lock:
            landmarks = self._landmarks

        if landmarks is None:
            self.get_logger().warn(
                f'No ArUco landmarks yet, accepting "{label}" without XY check.',
                throttle_duration_sec=5.0)
            return True

        xs = [0.0] + [float(lm['x']) for lm in landmarks]
        ys = [0.0] + [float(lm['y']) for lm in landmarks]
        x_min = min(xs) - self._bounds_pad
        x_max = max(xs) + self._bounds_pad
        y_min = min(ys) - self._bounds_pad
        y_max = max(ys) + self._bounds_pad

        if x_min <= x <= x_max and y_min <= y <= y_max:
            return True

        self.get_logger().warn(
            f'BOUNDS FAIL: "{label}" at ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'outside table x=[{x_min:.3f}, {x_max:.3f}] y=[{y_min:.3f}, {y_max:.3f}]')
        return False

    def _on_selected_object(self, msg: String):
        # Validate against "have we ever published a scene?" rather than the
        # strict OBJECTS_PUBLISHED state. This is robust to selections that
        # arrive at any point after the first publication.
        with self._state_lock:
            have_scene = bool(self._current_objs)
            if not have_scene:
                self.get_logger().warn(
                    '/selected_object received but no scene has been '
                    'published yet, ignoring.')
                return

        try:
            selected = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid JSON in /selected_object: {exc}')
            return

        name = selected.get('name', 'unknown')
        self.get_logger().info(
            f'Selection received: "{name}" at '
            f'({selected.get("x")}, {selected.get("y")}, {selected.get("z")}) '
            f'({"arm_controller will execute" if self._enable_arm else "dry run, no arm"})'
        )

        # Clear the scene cache so the next perception update is guaranteed
        # to re-publish (its hash will differ from the empty string).
        with self._state_lock:
            self._current_objs = []
            self._last_hash = ''
            self._state = ControllerState.WAITING_FOR_OBJECTS

        if not self._enable_arm:
            # No arm_controller running, so publish arm_status ourselves
            # so the LLM doesn't stall.
            thread = threading.Thread(target=self._dry_run_arm_status, daemon=True)
            thread.start()

    def _dry_run_arm_status(self):
        # simulates the arm status cycle when no real arm is connected so the LLM can reset
        self._publish_status('moving')
        time.sleep(1.0)
        self._publish_status('ready')
        self.get_logger().info('[DRY RUN] Arm status cycle complete.')

    def _on_arm_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('EMERGENCY STOP received on /arm_stop!')
            self._stop_flag = True
            with self._state_lock:
                self._state = ControllerState.STOPPED

    def _publish_markers(self, positions: list[dict]):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, pos in enumerate(positions):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = 'base_link'
            m.ns              = 'controller_objects'
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.pose.position.x = pos['x']
            m.pose.position.y = pos['y']
            m.pose.position.z = pos['z']
            m.pose.orientation.w = 1.0
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 0.85
            m.lifetime.sec = 2
            ma.markers.append(m)
        self._marker_pub.publish(ma)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
