import json
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ArucoLocalizerNode(Node):
    def __init__(self):
        super().__init__('aruco_localizer_node')

        self.declare_parameter('calibration_file', '')
        self.declare_parameter('ref_stream_id',    1)
        self.declare_parameter('aruco_dict',       'DICT_4X4_50')
        self.declare_parameter('marker_size',      0.097)
        self.declare_parameter('marker_ids',       '0,1,2,3')
        self.declare_parameter('origin_id',        0)
        self.declare_parameter('update_rate',      1.0)
        self.declare_parameter('world_topic',      '/perception/objects/cam1')
        self.declare_parameter('aruco_topic',      '')
        self.declare_parameter('stale_timeout_sec', 3.0)

        self._ref_sid      = int(self.get_parameter('ref_stream_id').value)
        self._marker_size  = float(self.get_parameter('marker_size').value)
        self._marker_ids   = [int(i) for i in self.get_parameter('marker_ids').value.split(',')]
        self._origin_id    = int(self.get_parameter('origin_id').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout_sec').value)

        self._K:     np.ndarray | None = None
        self._D:     np.ndarray | None = None
        self._R_c2w: np.ndarray | None = None
        self._t_c2w: np.ndarray | None = None

        cal_file = self.get_parameter('calibration_file').value
        if cal_file:
            self._load_calibration(cal_file)
        else:
            self.get_logger().warn('No calibration_file — ArUco detection disabled')

        dict_name = self.get_parameter('aruco_dict').value
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        try:
            params = cv2.aruco.DetectorParameters()
        except AttributeError:
            params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, 'ArucoDetector'):
            _det = cv2.aruco.ArucoDetector(aruco_dict, params)
            self._detect_fn = lambda img: _det.detectMarkers(img)
        else:
            self._detect_fn = lambda img: cv2.aruco.detectMarkers(
                img, aruco_dict, parameters=params)

        h = self._marker_size / 2.0
        self._obj_pts = np.array([
            [-h,  h, 0],
            [ h,  h, 0],
            [ h, -h, 0],
            [-h, -h, 0],
        ], dtype=np.float64)

        self._bridge = CvBridge()
        self._latest_img: Image | None = None
        self._lock = threading.Lock()

        # id=origin_id pose in world frame
        self._R_origin: np.ndarray | None = None
        self._t_origin: np.ndarray | None = None
        # positions of landmark markers (ids != origin_id) in world frame
        self._landmark_world: dict[int, np.ndarray] = {}
        # marker id -> time.monotonic() of last detection
        self._last_seen: dict[int, float] = {}
        # marker id -> whether it was considered stale as of the last tick
        self._was_stale: dict[int, bool] = {mid: True for mid in self._marker_ids}

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            Image,
            f'/rtsp/stream_{self._ref_sid}/raw',
            self._on_image,
            qos_sub)

        world_topic = self.get_parameter('world_topic').value
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, world_topic, self._on_world, qos_rel)

        aruco_topic = self.get_parameter('aruco_topic').value or \
            f'/perception/objects/aruco_id_{self._origin_id}'
        self._aruco_topic   = aruco_topic
        self._pub_aruco     = self.create_publisher(String, aruco_topic,                    qos_rel)
        self._pub_landmarks = self.create_publisher(String, '/perception/aruco/landmarks',  qos_rel)

        update_rate = float(self.get_parameter('update_rate').value)
        self.create_timer(1.0 / update_rate, self._update_aruco)

        self.get_logger().info(
            f'ArucoLocalizerNode ready  ref_stream={self._ref_sid}  '
            f'marker_ids={self._marker_ids}  origin_id={self._origin_id}')

    def _load_calibration(self, path):
        with open(path) as f:
            data = yaml.safe_load(f)
        cam = data.get('cameras', {}).get(f'stream_{self._ref_sid}')
        if cam is None:
            self.get_logger().error(
                f'stream_{self._ref_sid} not found in calibration file')
            return
        self._K     = np.array(cam['intrinsics']['k_matrix'], dtype=np.float64)
        self._D     = np.array(cam['intrinsics'].get('distortion', [[0]*5]),
                               dtype=np.float64).flatten()
        self._R_c2w = np.array(cam['extrinsics']['R_c2w'], dtype=np.float64)
        self._t_c2w = np.array(cam['extrinsics']['t_c2w'], dtype=np.float64).flatten()
        self.get_logger().info(f'Calibration loaded for stream_{self._ref_sid}')

    def _on_image(self, msg: Image):
        with self._lock:
            self._latest_img = msg

    def _update_aruco(self):
        """Detect all configured ArUco markers and update their world-frame poses."""
        if self._K is None:
            return

        with self._lock:
            img_msg = self._latest_img

        if img_msg is None:
            return

        gray = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        corners_list, ids, _ = self._detect_fn(gray)

        now = time.monotonic()
        new_origin_R  = None
        new_origin_t  = None
        new_landmarks = {}
        seen_this_tick = set()

        if ids is not None:
            for corners, mid in zip(corners_list, ids.flatten()):
                mid = int(mid)
                if mid not in self._marker_ids:
                    continue

                ok, rvec, tvec = cv2.solvePnP(
                    self._obj_pts,
                    corners.reshape(4, 2).astype(np.float64),
                    self._K, self._D)
                if not ok:
                    continue

                R_m2c = cv2.Rodrigues(rvec)[0]

                # Transform marker pose from camera frame to world frame
                R_m2w = self._R_c2w @ R_m2c
                t_m2w = self._R_c2w @ tvec.flatten() + self._t_c2w

                seen_this_tick.add(mid)
                if mid == self._origin_id:
                    new_origin_R = R_m2w
                    new_origin_t = t_m2w
                else:
                    new_landmarks[mid] = t_m2w

        with self._lock:
            for mid in seen_this_tick:
                self._last_seen[mid] = now
            if new_origin_R is not None:
                self._R_origin = new_origin_R
                self._t_origin = new_origin_t
                self.get_logger().info(
                    f'ArUco id={self._origin_id} at {np.round(new_origin_t, 3).tolist()}',
                    throttle_duration_sec=5.0)
            if new_landmarks:
                self._landmark_world.update(new_landmarks)

            # Edge-triggered stale/visible transition logging, evaluated every
            # tick for every configured marker so a drop is caught as soon as
            # the timeout elapses, not only on ticks with a detection.
            for mid in self._marker_ids:
                last_t = self._last_seen.get(mid)
                is_stale = last_t is None or (now - last_t) > self._stale_timeout
                was_stale = self._was_stale.get(mid, True)
                if is_stale and not was_stale:
                    self.get_logger().info(
                        f'ArUco marker id={mid} is now STALE '
                        f'(no detection for >{self._stale_timeout:.1f}s)')
                elif not is_stale and was_stale:
                    self.get_logger().info(f'ArUco marker id={mid} is now VISIBLE')
                self._was_stale[mid] = is_stale

        self._publish_landmarks()

    def _on_world(self, msg: String):
        """Re-express world-frame objects in aruco id=0 frame and republish."""
        with self._lock:
            R_origin = self._R_origin
            t_origin = self._t_origin
            origin_last_seen = self._last_seen.get(self._origin_id)

        if R_origin is None:
            return

        age = (time.monotonic() - origin_last_seen) if origin_last_seen is not None else float('inf')
        if age > self._stale_timeout:
            self.get_logger().warn(
                f'ArUco origin marker id={self._origin_id} stale (last seen {age:.1f}s ago) — '
                f'pausing {self._aruco_topic} updates', throttle_duration_sec=2.0)
            return

        try:
            objects = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        out = []
        for obj in objects:
            p_world = np.array([obj['x'], obj['y'], obj['z']])
            p_a0    = R_origin.T @ (p_world - t_origin)
            out.append({
                'label': obj['label'],
                'x':     round(float(p_a0[0]), 3),
                'y':     round(float(p_a0[1]), 3),
                'z':     round(float(p_a0[2]), 3),
            })

        m      = String()
        m.data = json.dumps(out)
        self._pub_aruco.publish(m)

    def _publish_landmarks(self):
        """Publish positions of landmark markers (ids 1-3) in aruco id=0 frame."""
        with self._lock:
            R_origin  = self._R_origin
            t_origin  = self._t_origin
            landmarks = dict(self._landmark_world)
            last_seen = dict(self._last_seen)

        if R_origin is None:
            return

        now = time.monotonic()
        stale_ids = [mid for mid in landmarks
                     if (now - last_seen.get(mid, -float('inf'))) > self._stale_timeout]
        if stale_ids:
            self.get_logger().warn(
                f'ArUco landmark marker(s) {stale_ids} stale — excluded from '
                f'/perception/aruco/landmarks', throttle_duration_sec=2.0)

        out = []
        for mid, t_world in landmarks.items():
            if mid in stale_ids:
                continue
            p_a0 = R_origin.T @ (t_world - t_origin)
            out.append({
                'id': mid,
                'x':  round(float(p_a0[0]), 3),
                'y':  round(float(p_a0[1]), 3),
                'z':  round(float(p_a0[2]), 3),
            })

        m      = String()
        m.data = json.dumps(out)
        self._pub_landmarks.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
