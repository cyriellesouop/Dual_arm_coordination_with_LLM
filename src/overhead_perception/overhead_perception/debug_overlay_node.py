# debug_overlay_node.py
#
# Live multi-camera debug viewer: tiles all configured RTSP streams into one
# OpenCV window, overlaid with YOLO detections (reused from detector_node's
# published Detection2DArray), live per-camera ArUco marker detection, and
# the workspace boundary formed by the configured marker IDs. Meant as a
# visual complement to `ros2 run rqt_image_view rqt_image_view` for
# diagnosing dropped markers/objects after the table is repositioned.
#
# Parameters (see config/debug_overlay_config.yaml):
#   stream_ids       : comma-separated camera IDs to tile      (default "0,1,2,3")
#   ref_stream_id    : stream ID used by aruco_localizer_node  (default 1)
#   aruco_dict       : ArUco dictionary name                   (default "DICT_4X4_50")
#   marker_ids       : comma-separated marker IDs to track     (default "0,1,2,3")
#   aruco_detect_rate: Hz, independent of the display timer    (default 8.0)
#   tile_width       : pixels                                  (default 640)
#   tile_height      : pixels                                  (default 360)

import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D as _Det2D
from vision_msgs.msg import Detection2DArray

_ROS_HUMBLE = hasattr(_Det2D().bbox.center, 'position')

_MISSING_COLOR  = (0, 0, 255)
_REF_COLOR      = (0, 220, 220)
_BOUNDARY_COLOR = (255, 200, 0)


class DebugOverlayNode(Node):
    def __init__(self):
        super().__init__('debug_overlay_node')

        self.declare_parameter('stream_ids',        '0,1,2,3')
        self.declare_parameter('ref_stream_id',     1)
        self.declare_parameter('aruco_dict',        'DICT_4X4_50')
        self.declare_parameter('marker_ids',        '0,1,2,3')
        self.declare_parameter('aruco_detect_rate', 8.0)
        self.declare_parameter('tile_width',        640)
        self.declare_parameter('tile_height',       360)

        ids_str          = self.get_parameter('stream_ids').value
        self.stream_ids  = [int(s.strip()) for s in ids_str.split(',')]
        self.ref_sid     = int(self.get_parameter('ref_stream_id').value)
        self.marker_ids  = [int(i) for i in self.get_parameter('marker_ids').value.split(',')]
        self.tile_width  = self.get_parameter('tile_width').value
        self.tile_height = self.get_parameter('tile_height').value

        dict_name  = self.get_parameter('aruco_dict').value
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

        self._bridge = CvBridge()
        self._lock   = threading.Lock()
        self._frames: dict     = {sid: None for sid in self.stream_ids}
        self._detections: dict = {sid: [] for sid in self.stream_ids}
        self._aruco: dict      = {sid: (None, None) for sid in self.stream_ids}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        for sid in self.stream_ids:
            self.create_subscription(
                Image, f'/rtsp/stream_{sid}/raw',
                lambda msg, s=sid: self._on_image(msg, s), qos)
            self.create_subscription(
                Detection2DArray, f'/perception/stream_{sid}/detections_2d',
                lambda msg, s=sid: self._on_detections(msg, s), qos)

        n = len(self.stream_ids)
        self.cols = math.ceil(math.sqrt(n))
        self.rows = math.ceil(n / self.cols)

        self.win_name = 'Perception Debug Overlay'
        cv2.namedWindow(
            self.win_name,
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED,
        )
        cv2.resizeWindow(self.win_name, self.tile_width * self.cols, self.tile_height * self.rows)

        aruco_rate = float(self.get_parameter('aruco_detect_rate').value)
        self.create_timer(1.0 / aruco_rate, self._detect_aruco_all)
        self.display_timer = self.create_timer(1.0 / 30.0, self._display_callback)

        self.get_logger().info(
            f'DebugOverlayNode ready  streams={self.stream_ids}  ref_stream={self.ref_sid}  '
            f'marker_ids={self.marker_ids}')

    def _on_image(self, msg: Image, sid: int):
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lock:
            self._frames[sid] = cv_image

    def _on_detections(self, msg: Detection2DArray, sid: int):
        dets = []
        for det in msg.detections:
            if not det.results:
                continue
            r = det.results[0]
            if _ROS_HUMBLE:
                label  = r.hypothesis.class_id
                score  = r.hypothesis.score
                cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            else:
                label  = r.id
                score  = r.score
                cx, cy = det.bbox.center.x, det.bbox.center.y
            dets.append((label, score, cx, cy, det.bbox.size_x, det.bbox.size_y))
        with self._lock:
            self._detections[sid] = dets

    def _detect_aruco_all(self):
        with self._lock:
            frames = {sid: f for sid, f in self._frames.items() if f is not None}
        for sid, frame in frames.items():
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners_list, ids, _ = self._detect_fn(gray)
            with self._lock:
                self._aruco[sid] = (corners_list, ids)

    def _draw_tile(self, sid: int) -> np.ndarray:
        with self._lock:
            frame = self._frames.get(sid)
            dets  = list(self._detections.get(sid, []))
            corners_list, ids = self._aruco.get(sid, (None, None))

        if frame is None:
            tile = np.zeros((self.tile_height, self.tile_width, 3), dtype=np.uint8)
            cv2.putText(tile, f'stream_{sid}: waiting...',
                        (10, self.tile_height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
            return tile

        annotated = frame.copy()

        # YOLO detections (reused from detector_node's published output).
        for label, score, cx, cy, w, h in dets:
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f'{label} {score:.2f}', (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Live per-camera ArUco detection + workspace boundary.
        centers = {}
        if ids is not None and corners_list:
            cv2.aruco.drawDetectedMarkers(annotated, corners_list, ids)
            for corners, mid in zip(corners_list, ids.flatten()):
                centers[int(mid)] = corners.reshape(4, 2).mean(axis=0)

        missing = [mid for mid in self.marker_ids if mid not in centers]
        if not missing and len(self.marker_ids) >= 2:
            pts = np.array([centers[mid] for mid in self.marker_ids], dtype=np.int32)
            cv2.polylines(annotated, [pts], isClosed=True, color=_BOUNDARY_COLOR, thickness=2)
        else:
            for mid in self.marker_ids:
                if mid in centers:
                    pt = tuple(centers[mid].astype(int))
                    cv2.circle(annotated, pt, 6, _BOUNDARY_COLOR, -1)
        if missing:
            cv2.putText(annotated, f'MISSING MARKERS: {missing}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, _MISSING_COLOR, 2)

        # Mark whichever tile actually drives the production ArUco frame today.
        if sid == self.ref_sid:
            cv2.rectangle(annotated, (0, 0),
                          (annotated.shape[1] - 1, annotated.shape[0] - 1), _REF_COLOR, 6)
            cv2.putText(annotated, 'REFERENCE (drives ArUco frame/table bounds)',
                        (10, annotated.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, _REF_COLOR, 2)

        cv2.putText(annotated, f'stream_{sid}', (8, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 0), 2)

        return cv2.resize(annotated, (self.tile_width, self.tile_height))

    def _display_callback(self):
        tiles = [self._draw_tile(sid) for sid in self.stream_ids]

        blank = np.zeros((self.tile_height, self.tile_width, 3), dtype=np.uint8)
        while len(tiles) < self.rows * self.cols:
            tiles.append(blank)

        rows = [np.hstack(tiles[r * self.cols:(r + 1) * self.cols]) for r in range(self.rows)]
        cv2.imshow(self.win_name, np.vstack(rows))
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DebugOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
