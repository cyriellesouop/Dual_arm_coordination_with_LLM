# camera_calibration_pkg/stereo_aruco_calibration_node.py
#
# Computes stereo extrinsics (R, T, E, F) and rectification parameters
# (R1, R2, P1, P2, Q) between two camera streams using a ChArUco board.
# Reads per-camera intrinsics from YAML files produced by
# intrinsic_aruco_calibration_node.
#
# Because each camera may only see a partial view of the board, detections
# are intersected by marker ID so only commonly-visible corners are used.
#
# Frame pairs are only captured when both frames have timestamps within
# max_sync_delta_ms of each other — critical for avoiding sync artifacts
# when auto-sampling a moving board.
#
# Run via launch file:
#   ros2 launch camera_calibration_pkg stereo_aruco_calibration.launch.py
#
# Controls:
#   s     - toggle auto-sampling (captures when board moves enough)
#   space - manual capture of current pair
#   c     - run stereo calibration (requires >= min_samples pairs)
#   q     - quit without saving

import os
import time

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

MIN_CORNERS_PER_PAIR = 6

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
    """Return f(gray) -> (charuco_corners, charuco_ids, marker_corners, marker_ids)."""
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
            if ret < MIN_CORNERS_PER_PAIR:
                return None, None, corners, ids
            return ch_corners, ch_ids, corners, ids

        return _detect_legacy


class StereoArucoCalibrationNode(Node):
    def __init__(self):
        super().__init__('stereo_aruco_calibration_node')

        self.declare_parameter('stream_id_left',          0)
        self.declare_parameter('stream_id_right',         1)
        self.declare_parameter('intrinsic_yaml_left',     '')
        self.declare_parameter('intrinsic_yaml_right',    '')
        self.declare_parameter('squares_x',               12)
        self.declare_parameter('squares_y',               9)
        self.declare_parameter('square_length',           0.100)
        self.declare_parameter('marker_length',           0.075)
        self.declare_parameter('aruco_dict',              'DICT_4X4_100')
        self.declare_parameter('min_samples',             25)
        self.declare_parameter('min_corner_displacement', 20.0)
        self.declare_parameter('max_sync_delta_ms',       50.0)

        self.stream_id_left   = self.get_parameter('stream_id_left').value
        self.stream_id_right  = self.get_parameter('stream_id_right').value
        yaml_left             = self.get_parameter('intrinsic_yaml_left').value
        yaml_right            = self.get_parameter('intrinsic_yaml_right').value
        squares_x             = self.get_parameter('squares_x').value
        squares_y             = self.get_parameter('squares_y').value
        square_length         = self.get_parameter('square_length').value
        marker_length         = self.get_parameter('marker_length').value
        dict_name             = self.get_parameter('aruco_dict').value
        self.min_samples      = self.get_parameter('min_samples').value
        self.min_displacement = self.get_parameter('min_corner_displacement').value
        self.max_sync_ns      = int(self.get_parameter('max_sync_delta_ms').value * 1e6)

        if dict_name not in ARUCO_DICTS:
            raise ValueError(f"Unknown aruco_dict '{dict_name}'. Valid: {list(ARUCO_DICTS)}")

        self.K_left,  self.D_left  = self._load_intrinsics(yaml_left,  'left')
        self.K_right, self.D_right = self._load_intrinsics(yaml_right, 'right')

        aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        self.board   = _make_board(squares_x, squares_y, square_length, marker_length, aruco_dict)
        self._detect = _make_detector(self.board, aruco_dict)

        self.get_logger().info(
            f'OpenCV {cv2.__version__} — using {"new (4.7+)" if _NEW_API else "legacy"} ArUco API'
        )

        self.running     = True
        self.frame_left  = None
        self.frame_right = None
        self.recv_left_t  = None
        self.recv_right_t = None
        self.image_size  = None
        self.sampling    = False
        self.bridge      = CvBridge()

        self.obj_points:   list = []
        self.img_points_L: list = []
        self.img_points_R: list = []

        self.last_accepted_centroid_L = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic_left  = f'/rtsp/stream_{self.stream_id_left}/raw'
        topic_right = f'/rtsp/stream_{self.stream_id_right}/raw'
        self.sub_left  = self.create_subscription(Image, topic_left,  self._cb_left,  qos)
        self.sub_right = self.create_subscription(Image, topic_right, self._cb_right, qos)
        self.get_logger().info(f'Subscribed to {topic_left} and {topic_right}')
        self.get_logger().info(
            f'ChArUco board: {squares_x}x{squares_y}, '
            f'square={square_length*1000:.1f}mm, marker={marker_length*1000:.1f}mm, dict={dict_name}\n'
            f'  min pairs         : {self.min_samples}\n'
            f'  min displacement  : {self.min_displacement} px\n'
            f'  max sync delta    : {self.get_parameter("max_sync_delta_ms").value:.0f} ms\n'
            f'  s     — toggle auto-sampling\n'
            f'  space — manual capture\n'
            f'  c     — calibrate & save\n'
            f'  q     — quit'
        )

        self.win_name = (
            f'Stereo ArUco Calibration  '
            f'stream_{self.stream_id_left} | stream_{self.stream_id_right}'
        )
        cv2.namedWindow(
            self.win_name,
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED,
        )
        cv2.resizeWindow(self.win_name, 1280, 360)

        self.display_timer = self.create_timer(1.0 / 30.0, self.display_callback)

    # helper functions
    def _load_intrinsics(self, path: str, label: str):
        if not path:
            raise ValueError(
                f'intrinsic_yaml_{label} is empty — set the path in stereo_aruco_calibration_config.yaml'
            )
        if not os.path.exists(path):
            raise FileNotFoundError(f'Intrinsic YAML not found: {path}')
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        K = np.array(data['k_matrix'],   dtype=np.float64)
        D = np.array(data['distortion'], dtype=np.float64)
        self.get_logger().info(f'Loaded {label} intrinsics from {path}')
        return K, D

    def _frames_in_sync(self) -> bool:
        if self.recv_left_t is None or self.recv_right_t is None:
            return False
        delta_ms = abs(self.recv_left_t - self.recv_right_t) * 1000.0
        return delta_ms <= self.get_parameter('max_sync_delta_ms').value

    # frame callbacks
    def _cb_left(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is not None:
            self.frame_left  = frame
            self.recv_left_t = time.monotonic()

    def _cb_right(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is not None:
            self.frame_right  = frame
            self.recv_right_t = time.monotonic()

    # detection

    def _detect_and_draw(self, gray, frame):
        ch_corners, ch_ids, mk_corners, mk_ids = self._detect(gray)
        if mk_ids is not None and len(mk_ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, mk_corners, mk_ids)
        if ch_corners is not None and ch_ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(frame, ch_corners, ch_ids)
        return ch_corners, ch_ids

    def _match_corners(self, corners_L, ids_L, corners_R, ids_R):
        """
        Intersect detected IDs from both cameras and return matched
        (obj_pts, img_pts_L, img_pts_R). Returns None if fewer than
        MIN_CORNERS_PER_PAIR corners are shared.
        """
        if ids_L is None or ids_R is None:
            return None

        flat_L = ids_L.flatten()
        flat_R = ids_R.flatten()
        common_ids = np.intersect1d(flat_L, flat_R)

        if len(common_ids) < MIN_CORNERS_PER_PAIR:
            return None

        idx_L = np.where(np.isin(flat_L, common_ids))[0]
        idx_R = np.where(np.isin(flat_R, common_ids))[0]

        idx_L = idx_L[np.argsort(flat_L[idx_L])]
        idx_R = idx_R[np.argsort(flat_R[idx_R])]

        matched_corners_L = corners_L[idx_L]
        matched_corners_R = corners_R[idx_R]
        matched_ids       = flat_L[idx_L].reshape(-1, 1).astype(np.int32)

        if _NEW_API:
            obj_pts, _ = self.board.matchImagePoints(matched_corners_L, matched_ids)
        else:
            board_corners_3d = self.board.chessboardCorners
            obj_pts = board_corners_3d[matched_ids.flatten()].reshape(-1, 1, 3).astype(np.float32)

        if obj_pts is None or len(obj_pts) == 0:
            return None

        return obj_pts, matched_corners_L, matched_corners_R

    # display/capture
    def display_callback(self):
        if self.frame_left is None or self.frame_right is None or not self.running:
            return

        left  = self.frame_left.copy()
        right = self.frame_right.copy()

        gray_l = cv2.cvtColor(left,  cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        if self.image_size is None:
            self.image_size = (gray_l.shape[1], gray_l.shape[0])

        corners_L, ids_L = self._detect_and_draw(gray_l, left)
        corners_R, ids_R = self._detect_and_draw(gray_r, right)

        matched      = self._match_corners(corners_L, ids_L, corners_R, ids_R)
        both_found   = matched is not None
        in_sync      = self._frames_in_sync()
        ready        = both_found and in_sync
        shared_count = len(matched[0]) if matched else 0

        if self.sampling and ready and (shared_count > 50):
            self._try_sample(matched, corners_L)

        self._draw_overlay(left,  both_found, in_sync, shared_count, side='L')
        self._draw_overlay(right, both_found, in_sync, shared_count, side='R')

        small_l = cv2.resize(left,  (0, 0), fx=0.5, fy=0.5)
        small_r = cv2.resize(right, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow(self.win_name, np.hstack([small_l, small_r]))
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.sampling = not self.sampling
            self.get_logger().info(f'Auto-sampling {"ON" if self.sampling else "OFF"}')

        elif key == ord(' ') and ready:
            self._store_pair(matched)

        elif key == ord('c'):
            if len(self.obj_points) < self.min_samples:
                self.get_logger().warn(
                    f'Need at least {self.min_samples} pairs, have {len(self.obj_points)}.'
                )
            else:
                self._calibrate()

        elif key == ord('q'):
            self.get_logger().info('Quitting without saving.')
            cv2.destroyAllWindows()
            rclpy.shutdown()

    # sampling
    def _try_sample(self, matched, corners_L):
        pts      = corners_L.reshape(-1, 2)
        centroid = (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))

        if self.last_accepted_centroid_L is not None:
            dx = centroid[0] - self.last_accepted_centroid_L[0]
            dy = centroid[1] - self.last_accepted_centroid_L[1]
            if np.sqrt(dx * dx + dy * dy) < self.min_displacement:
                return

        self._store_pair(matched, centroid)

    def _store_pair(self, matched, centroid=None):
        obj_pts, img_pts_L, img_pts_R = matched
        self.obj_points.append(obj_pts)
        self.img_points_L.append(img_pts_L)
        self.img_points_R.append(img_pts_R)
        if centroid is not None:
            self.last_accepted_centroid_L = centroid
        self.get_logger().info(
            f'Pair captured ({len(self.obj_points)})  shared corners: {len(obj_pts)}'
        )

    # overlay 
    def _draw_overlay(self, frame, both_found, in_sync, shared_count, side):
        h, w = frame.shape[:2]
        n = len(self.obj_points)

        s_color = (0, 220, 0) if self.sampling else (0, 0, 220)
        cv2.putText(frame, 'AUTO: ON' if self.sampling else 'AUTO: OFF',
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, s_color, 2)

        cv2.putText(frame, f'Pairs: {n}',
            (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)

        b_color = (0, 220, 0) if both_found else (0, 0, 220)
        cv2.putText(frame,
            f'Live corners (L+R): {shared_count}' if both_found else 'Board: NOT FOUND',
            (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, b_color, 2)

        if not in_sync:
            delta_ms = (
                abs(self.recv_left_t - self.recv_right_t) * 1000.0
                if self.recv_left_t is not None and self.recv_right_t is not None
                else -1.0
            )
            cv2.putText(frame, f'OUT OF SYNC ({delta_ms:.0f}ms)',
                (10, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 220), 2)

        if side == 'L':
            cv2.putText(frame,
                '[s] auto  [space] capture  [c] calibrate  [q] quit',
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

    # stereo calibration

    def _calibrate(self):
        self.get_logger().info(
            f'Running stereo calibration with {len(self.obj_points)} pairs...'
        )

        # FIX_INTRINSIC: keep per-camera K/D from intrinsic calibration, only solve for R/T.
        # Fewer DOF → less overfitting, and intrinsics don't drift from their calibrated values.
        flags = cv2.CALIB_FIX_INTRINSIC

        ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
            self.obj_points,
            self.img_points_L,
            self.img_points_R,
            self.K_left.copy(),
            self.D_left.copy(),
            self.K_right.copy(),
            self.D_right.copy(),
            self.image_size,
            flags=flags,
        )

        self.get_logger().info(f'Stereo RMS reprojection error: {ret:.4f}')
        self.get_logger().info(f'R:\n{R}')
        self.get_logger().info(f'T:\n{T}')

        # alpha=0: crop to the valid overlapping region only.
        # With large rotation between cameras, alpha>0 produces black borders.
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1, D1, K2, D2, self.image_size, R, T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )

        self._write_yaml(ret, K1, D1, K2, D2, R, T, E, F, R1, R2, P1, P2, Q)
        self._show_rectified_preview(K1, D1, K2, D2, R1, R2, P1, P2)

        cv2.destroyWindow(self.win_name)
        self.display_timer = None
        self.running = False

    # rectified preview 
    def _show_rectified_preview(self, K1, D1, K2, D2, R1, R2, P1, P2):
        left  = self.frame_left.copy()
        right = self.frame_right.copy()

        map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, self.image_size, cv2.CV_16SC2)
        map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, self.image_size, cv2.CV_16SC2)

        rect_l = cv2.remap(left,  map1_l, map2_l, cv2.INTER_LINEAR)
        rect_r = cv2.remap(right, map1_r, map2_r, cv2.INTER_LINEAR)

        # Detect and match keypoints between the two rectified images.
        # Good calibration: matched points should lie on the same horizontal scanline.
        orb = cv2.ORB_create(500)
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        kp_l, des_l = orb.detectAndCompute(gray_l, None)
        kp_r, des_r = orb.detectAndCompute(gray_r, None)

        combined = np.hstack([
            cv2.resize(rect_l, (0, 0), fx=0.5, fy=0.5),
            cv2.resize(rect_r, (0, 0), fx=0.5, fy=0.5),
        ])
        h, w_half = combined.shape[0], combined.shape[1] // 2

        if des_l is not None and des_r is not None and len(des_l) > 0 and len(des_r) > 0:
            bf      = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des_l, des_r)
            matches = sorted(matches, key=lambda m: m.distance)[:80]

            # Draw only matches where y-coordinates are close (epipolar constraint).
            # Threshold in the half-res image (divide by 2).
            epipolar_thresh = 5.0
            good, bad = [], []
            for m in matches:
                y_l = kp_l[m.queryIdx].pt[1] * 0.5
                y_r = kp_r[m.trainIdx].pt[1] * 0.5
                (good if abs(y_l - y_r) <= epipolar_thresh else bad).append(m)

            for m in bad:
                pt_l = (int(kp_l[m.queryIdx].pt[0] * 0.5),       int(kp_l[m.queryIdx].pt[1] * 0.5))
                pt_r = (int(kp_r[m.trainIdx].pt[0] * 0.5) + w_half, int(kp_r[m.trainIdx].pt[1] * 0.5))
                cv2.line(combined, pt_l, pt_r, (0, 0, 200), 1)

            for m in good:
                pt_l = (int(kp_l[m.queryIdx].pt[0] * 0.5),       int(kp_l[m.queryIdx].pt[1] * 0.5))
                pt_r = (int(kp_r[m.trainIdx].pt[0] * 0.5) + w_half, int(kp_r[m.trainIdx].pt[1] * 0.5))
                cv2.circle(combined, pt_l, 3, (0, 220, 0), -1)
                cv2.circle(combined, pt_r, 3, (0, 220, 0), -1)
                cv2.line(combined, pt_l, pt_r, (0, 220, 0), 1)

            cv2.putText(combined,
                f'Good (same scanline): {len(good)}  Bad: {len(bad)}  — any key to exit',
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
        else:
            cv2.putText(combined,
                'No keypoints detected — any key to exit',
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 220), 2)

        win = 'Stereo Rectified Preview'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.resizeWindow(win, 1280, 480)
        cv2.imshow(win, combined)
        cv2.waitKey(0)
        cv2.destroyWindow(win)

    # store as yaml

    def _write_yaml(self, rms, K1, D1, K2, D2, R, T, E, F, R1, R2, P1, P2, Q):
        workspace_root = os.path.abspath(os.getcwd())
        calib_dir      = os.path.join(workspace_root, 'calibration')
        os.makedirs(calib_dir, exist_ok=True)

        file_path = os.path.join(
            calib_dir,
            f'stereo_aruco_stream{self.stream_id_left}'
            f'_stream{self.stream_id_right}_{self.image_size[1]}p.yaml',
        )

        data = {
            'stream_id_left':         int(self.stream_id_left),
            'stream_id_right':        int(self.stream_id_right),
            'image_size': {
                'width':  int(self.image_size[0]),
                'height': int(self.image_size[1]),
            },
            'rms_reprojection_error': float(rms),
            'k_matrix_left':          K1.tolist(),
            'distortion_left':        D1.tolist(),
            'k_matrix_right':         K2.tolist(),
            'distortion_right':       D2.tolist(),
            'R':  R.tolist(),
            'T':  T.tolist(),
            'E':  E.tolist(),
            'F':  F.tolist(),
            'R1': R1.tolist(),
            'R2': R2.tolist(),
            'P1': P1.tolist(),
            'P2': P2.tolist(),
            'Q':  Q.tolist(),
        }

        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'Extrinsics saved to: {file_path}')


def main(args=None):
    rclpy.init(args=args)
    node = StereoArucoCalibrationNode()
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
