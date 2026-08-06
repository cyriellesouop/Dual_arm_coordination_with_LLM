# camera_calibration_pkg/intrinsic_aruco_calibration_node.py
#
# Computes intrinsic camera matrix (K) and distortion coefficients using a
# ChArUco board (ArUco markers embedded in a checkerboard).
#
# Follows the official OpenCV 4.x calibration pattern:
#   detector.detectBoard()  ->  board.matchImagePoints()  ->  cv2.calibrateCamera()
#   https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html
#
# Run via launch file:
#   ros2 launch camera_calibration_pkg intrinsic_aruco_calibration.launch.py stream_id:=0
#
# Controls:
#   s  - toggle continuous auto-sampling on/off
#   c  - run final calibration and save (requires >= min_samples captured frames)
#   q  - quit without saving

import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

ARUCO_DICTS = {
    'DICT_4X4_50':    cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100':   cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250':   cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000':  cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50':    cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100':   cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250':   cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000':  cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50':    cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100':   cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250':   cv2.aruco.DICT_6X6_250,
    'DICT_6X6_1000':  cv2.aruco.DICT_6X6_1000,
    'DICT_7X7_50':    cv2.aruco.DICT_7X7_50,
    'DICT_7X7_100':   cv2.aruco.DICT_7X7_100,
    'DICT_7X7_250':   cv2.aruco.DICT_7X7_250,
    'DICT_7X7_1000':  cv2.aruco.DICT_7X7_1000,
}

# Minimum inner chessboard corners required to accept a detection at all.
MIN_CORNERS_PER_FRAME = 6

# True when OpenCV >= 4.7 (CharucoBoard constructor + CharucoDetector available).
_NEW_API = hasattr(cv2.aruco, 'CharucoDetector')


def _make_board(squares_x, squares_y, square_length, marker_length, aruco_dict):
    if _NEW_API:
        board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_length, marker_length, aruco_dict
        )
        board.setLegacyPattern(False)
    else:
        board = cv2.aruco.CharucoBoard_create(
            squares_x, squares_y, square_length, marker_length, aruco_dict
        )
    return board


def _make_detector(board, aruco_dict):
    """Return a detection callable: f(gray) -> (charuco_corners, charuco_ids, marker_corners, marker_ids)."""
    if _NEW_API:
        detector = cv2.aruco.CharucoDetector(
            board,
            cv2.aruco.CharucoParameters(),
            cv2.aruco.DetectorParameters(),
            cv2.aruco.RefineParameters(),
        )
        return detector.detectBoard
    else:
        det_params = cv2.aruco.DetectorParameters_create()

        def _detect_legacy(gray):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=det_params)
            if ids is None or len(ids) < 2:
                return None, None, corners, ids
            ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if ret < MIN_CORNERS_PER_FRAME:
                return None, None, corners, ids
            return ch_corners, ch_ids, corners, ids

        return _detect_legacy


class IntrinsicArucoCalibrationNode(Node):
    def __init__(self):
        super().__init__('intrinsic_aruco_calibration_node')

        self.declare_parameter('stream_id',               0)
        self.declare_parameter('squares_x',               12)
        self.declare_parameter('squares_y',               9)
        self.declare_parameter('square_length',           0.100)   # meters
        self.declare_parameter('marker_length',           0.075)   # meters
        self.declare_parameter('aruco_dict',              'DICT_4X4_100')
        self.declare_parameter('min_samples',             25)
        self.declare_parameter('min_corner_displacement', 20.0)    # pixels

        self.stream_id          = self.get_parameter('stream_id').value
        squares_x               = self.get_parameter('squares_x').value
        squares_y               = self.get_parameter('squares_y').value
        square_length           = self.get_parameter('square_length').value
        marker_length           = self.get_parameter('marker_length').value
        dict_name               = self.get_parameter('aruco_dict').value
        self.min_samples        = self.get_parameter('min_samples').value
        self.min_displacement   = self.get_parameter('min_corner_displacement').value

        if dict_name not in ARUCO_DICTS:
            self.get_logger().error(
                f"Unknown aruco_dict '{dict_name}'. Valid options: {list(ARUCO_DICTS)}"
            )
            raise ValueError(f"Unknown aruco_dict: {dict_name}")

        aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        self.board   = _make_board(squares_x, squares_y, square_length, marker_length, aruco_dict)
        self._detect = _make_detector(self.board, aruco_dict)

        self.get_logger().info(
            f'OpenCV {cv2.__version__} — using {"new (4.7+)" if _NEW_API else "legacy"} ArUco API'
        )

        # calibration data 
        self.all_charuco_corners: list = []
        self.all_charuco_ids:     list = []
        self.all_obj_points:      list = []
        self.all_img_points:      list = []

        self.last_accepted_centroid = None   # (cx, cy) centroid of last accepted frame

        # runtime state
        self.sampling    = False
        self.latest_frame: np.ndarray | None = None
        self.image_size:   tuple | None = None
        self.running     = True
        self.bridge      = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = f'/rtsp/stream_{self.stream_id}/raw'
        self.sub = self.create_subscription(Image, topic, self.frame_callback, qos)

        self.get_logger().info(
            f'ChArUco board: {squares_x}x{squares_y} squares, '
            f'square={square_length*1000:.1f}mm, marker={marker_length*1000:.1f}mm, dict={dict_name}\n'
            f'  min samples       : {self.min_samples}\n'
            f'  min displacement  : {self.min_displacement} px\n'
            f'  s     — toggle auto-sampling on/off\n'
            f'  space — manual single capture\n'
            f'  c     — calibrate & save\n'
            f'  q     — quit'
        )

        self.win_name = f'ArUco Intrinsic Calibration stream_{self.stream_id}'
        cv2.namedWindow(
            self.win_name,
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED,
        )
        cv2.resizeWindow(self.win_name, 640, 480)

        self.display_timer = self.create_timer(1.0 / 30.0, self.display_callback)

    # ROS callbacks 

    def frame_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None:
            self.get_logger().warning('Failed to decode frame', throttle_duration_sec=5.0)
            return
        self.latest_frame = frame

    def display_callback(self):
        if self.latest_frame is None or not self.running:
            return

        frame = self.latest_frame.copy()
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.image_size is None:
            self.image_size = (gray.shape[1], gray.shape[0])

        charuco_corners, charuco_ids, marker_corners, marker_ids = self._detect(gray)

        board_found = (
            charuco_corners is not None
            and charuco_ids is not None
            and len(charuco_corners) >= MIN_CORNERS_PER_FRAME
        )

        # Draw detections onto frame
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
        if board_found:
            cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)

        if self.sampling and board_found:
            self._try_sample(charuco_corners, charuco_ids)

        self._draw_overlay(frame, board_found, charuco_corners)

        small = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow(self.win_name, small)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.sampling = not self.sampling
            self.get_logger().info(f'Auto-sampling {"ON" if self.sampling else "OFF"}')

        elif key == ord(' ') and board_found:
            self._capture_frame(charuco_corners, charuco_ids)

        elif key == ord('c'):
            if len(self.all_obj_points) < self.min_samples:
                self.get_logger().warn(
                    f'Need at least {self.min_samples} samples, have {len(self.all_obj_points)}.'
                )
            else:
                self._run_final_calibration()

        elif key == ord('q'):
            self.get_logger().info('Quitting without saving.')
            cv2.destroyAllWindows()
            rclpy.shutdown()

    # smart sampling

    def _capture_frame(self, charuco_corners: np.ndarray, charuco_ids: np.ndarray):
        """Manual capture — no displacement gate, always accepts."""
        self._store_sample(charuco_corners, charuco_ids)

    def _try_sample(self, charuco_corners: np.ndarray, charuco_ids: np.ndarray):
        """Auto capture — reject if centroid hasn't moved min_displacement pixels."""
        pts      = charuco_corners.reshape(-1, 2)
        centroid = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))

        if self.last_accepted_centroid is not None:
            dx = centroid[0] - self.last_accepted_centroid[0]
            dy = centroid[1] - self.last_accepted_centroid[1]
            if np.sqrt(dx * dx + dy * dy) < self.min_displacement:
                return

        if len(charuco_corners) < 60:
            return 
        
        self._store_sample(charuco_corners, charuco_ids)

    def _store_sample(self, charuco_corners: np.ndarray, charuco_ids: np.ndarray):
        if _NEW_API:
            obj_pts, img_pts = self.board.matchImagePoints(charuco_corners, charuco_ids)
        else:
            board_corners_3d = self.board.chessboardCorners
            ids_flat = charuco_ids.flatten()
            obj_pts  = board_corners_3d[ids_flat].reshape(-1, 1, 3).astype(np.float32)
            img_pts  = charuco_corners

        if obj_pts is None or len(obj_pts) == 0:
            return

        pts      = charuco_corners.reshape(-1, 2)
        centroid = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))

        self.all_charuco_corners.append(charuco_corners)
        self.all_charuco_ids.append(charuco_ids)
        self.all_obj_points.append(obj_pts)
        self.all_img_points.append(img_pts)
        self.last_accepted_centroid = centroid

        self.get_logger().info(
            f'Sample accepted ({len(self.all_obj_points)})  corners: {len(charuco_corners)}'
        )

    def _make_initial_K(self) -> np.ndarray:
        w, h = self.image_size
        f = float(max(w, h))
        return np.array([
            [f,   0., w / 2.0],
            [0.,  f,  h / 2.0],
            [0.,  0., 1.0    ],
        ], dtype=np.float64)

    # overlay charuco

    def _draw_overlay(self, frame: np.ndarray, board_found: bool, charuco_corners):
        h, w = frame.shape[:2]

        s_color = (0, 220, 0) if self.sampling else (0, 0, 220)
        cv2.putText(frame,
            'SAMPLING: ON  [s] toggle' if self.sampling else 'SAMPLING: OFF  [s] toggle',
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, s_color, 2)

        n = len(self.all_obj_points)
        cv2.putText(frame,
            f'Samples: {n}',
            (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 165, 255), 2)

        corner_count = len(charuco_corners) if charuco_corners is not None else 0
        d_color = (0, 220, 0) if board_found else (0, 0, 220)
        cv2.putText(frame,
            f'Board: FOUND ({corner_count} corners)' if board_found else 'Board: NOT FOUND',
            (10, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.65, d_color, 2)

        cv2.putText(frame,
            '[s] auto-sample  [space] manual capture  [c] calibrate & save  [q] quit',
            (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 1)

    # final calibration

    def _run_final_calibration(self):
        self.get_logger().info(
            f'Running final calibration on {len(self.all_obj_points)} samples...'
        )
        K_init = self._make_initial_K()
        flags = (
            cv2.CALIB_USE_INTRINSIC_GUESS
            | cv2.CALIB_FIX_ASPECT_RATIO
            | cv2.CALIB_ZERO_TANGENT_DIST
            | cv2.CALIB_FIX_K3
        )
        try:
            rms, K, dist, _, _ = cv2.calibrateCamera(
                self.all_obj_points, self.all_img_points,
                self.image_size, K_init, None, flags=flags,
            )
        except cv2.error as e:
            self.get_logger().error(f'Calibration failed: {e}')
            return

        w, h    = self.image_size
        cx_err  = abs(K[0, 2] - w / 2)
        cy_err  = abs(K[1, 2] - h / 2)
        quality = (
            'Good'        if rms < 0.5 else
            'Acceptable'  if rms < 1.0 else
            'High — collect more diverse samples'
        )
        self.get_logger().info(
            f'\n── stream_{self.stream_id} intrinsics ──────────────────\n'
            f'  RMS: {rms:.4f} px  ({quality})\n'
            f'  cx={K[0,2]:.1f} (err {cx_err:.1f} px)  cy={K[1,2]:.1f} (err {cy_err:.1f} px)\n'
            f'  K:\n{K}\n'
            f'  dist: {dist}\n'
        )
        if cx_err > w * 0.1 or cy_err > h * 0.1:
            self.get_logger().warn(
                'Principal point is >10% off-center — fill all grid zones for better coverage.'
            )

        self._write_yaml(K, dist, rms)
        cv2.destroyAllWindows()
        self.running = False
        rclpy.shutdown()

    def _write_yaml(self, K: np.ndarray, dist: np.ndarray, rms: float):
        workspace_root = os.path.abspath(os.getcwd())
        calib_dir      = os.path.join(workspace_root, 'calibration')
        os.makedirs(calib_dir, exist_ok=True)

        file_path = os.path.join(
            calib_dir, f'intrinsics_aruco_stream{self.stream_id}_{self.image_size[1]}p.yaml'
        )
        data = {
            'stream_id':              int(self.stream_id),
            'image_size': {
                'width':  int(self.image_size[0]),
                'height': int(self.image_size[1]),
            },
            'rms_reprojection_error': float(rms),
            'k_matrix':               K.tolist(),
            'distortion':             dist.tolist(),
        }
        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f'Intrinsics saved to: {file_path}')


def main(args=None):
    rclpy.init(args=args)
    node = IntrinsicArucoCalibrationNode()
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
