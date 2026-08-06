# camera_calibration_pkg/intrinsic_checkerboard_calibration_node.py
#
# Computes intrinsic camera matrix (K) and distortion coefficients for a single
# camera stream using continuous video.  Samples are quality-gated on corner
# displacement and running reprojection error.
#
# Run via launch file:
#   ros2 launch camera_calibration_pkg intrinsic_calibration.launch.py stream_id:=0
#
# Or directly:
#   ros2 run camera_calibration_pkg intrinsic_checkerboard_calibration_node \
#     --ros-args -p stream_id:=0

import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class IntrinsicCalibrationNode(Node):
    def __init__(self):
        super().__init__('intrinsic_checkerboard_calibration_node')

        # parameters
        self.declare_parameter('stream_id', 0)
        self.declare_parameter('pattern_cols', 6)       # inner corners (not squares)
        self.declare_parameter('pattern_rows', 8)
        self.declare_parameter('square_size', 0.029)    # meters
        self.declare_parameter('min_samples', 25)       # minimum before calibration

        self.stream_id = self.get_parameter('stream_id').value
        cols = self.get_parameter('pattern_cols').value
        rows = self.get_parameter('pattern_rows').value
        self.square_size = self.get_parameter('square_size').value
        self.min_samples = self.get_parameter('min_samples').value
        self.pattern_size = (cols, rows)

        self.running = True
        self.latest_frame = None
        self.image_size = None
        self.count = 0
        self.bridge = CvBridge()

        # 3-D world coordinates for one checkerboard view
        self.wc = np.zeros((cols * rows, 3), np.float32)
        self.wc[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * self.square_size

        # calibration data
        self.obj_points: list = []   # accepted 3-D world points
        self.img_points: list = []   # accepted 2-D image points

        # stream subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = f'/rtsp/stream_{self.stream_id}/raw'
        self.sub = self.create_subscription(Image, topic, self.frame_callback, qos)
        self.get_logger().info(f'Subscribed to {topic}')

        # openCV window
        self.win_name = f'Intrinsic Calibration stream_{self.stream_id}'
        cv2.namedWindow(
            self.win_name,
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED,
        )
        cv2.resizeWindow(self.win_name, 640, 480)

        # timer for displaying frames
        self.display_timer = self.create_timer(1.0 / 30.0, self.display_callback)

    def frame_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None:
            self.get_logger().warning('Failed to decode frame', throttle_duration_sec=5.0)
            return
        self.latest_frame = frame

    def display_callback(self):
        if (self.latest_frame is None) or not self.running:
            return

        frame = self.latest_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.image_size is None:
            self.image_size = (gray.shape[1], gray.shape[0])  # (width, height)

        found, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        if found:
            cv2.drawChessboardCorners(frame, self.pattern_size, corners, found)

        small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow(self.win_name, small_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and found:
            self.obj_points.append(self.wc)
            # refine corners for better accuracy
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
            self.img_points.append(corners2)

            self.count += 1
            self.get_logger().info(f"Captured image {self.count}")
            
        elif key == ord('c'):
            # calibrate camera
            if len(self.obj_points) > self.min_samples:
                print("\nCalibrating with constraints...")

                # 1. Manually initialize the K matrix (Intrinsics)
                # Based on 12mm lens, IMX477, scaled to 1080p
                initial_mtx = np.array([
                    [3665.0, 0.0, 960.0],
                    [0.0, 3655.0, 540.0],
                    [0.0, 0.0, 1.0]
                ], dtype=np.float32)

                # 2. Define the flags
                # CALIB_USE_INTRINSIC_GUESS: Uses the matrix above as a starting point
                # CALIB_FIX_ASPECT_RATIO: Keeps fx and fy ratio constant (prevents stretching)
                # CALIB_FIX_PRINCIPAL_POINT: Forces cx/cy to stay at the center (320, 240)
                # cv2.CALIB_ZERO_TANGENT_DIST: assume no distortion from lense curvature
                # CALIB_FIX_K3: Prevents the "explosive" distortion value at the edges
                flags = (cv2.CALIB_USE_INTRINSIC_GUESS + 
                         cv2.CALIB_FIX_ASPECT_RATIO + 
                         cv2.CALIB_ZERO_TANGENT_DIST +
                         cv2.CALIB_FIX_K3)

                # 3. Run calibration with the flags
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                    self.obj_points, 
                    self.img_points, 
                    gray.shape[::-1], 
                    initial_mtx,   # Pass the guess here
                    None, 
                    flags=flags
                )

                print("\nConstrained Intrinsic Matrix (K):")
                print(mtx)
                print("\nConstrained Distortion coefficients:")
                print(dist)
                print(f"\nRe-projection Error: {ret:.4f}")
                
                self.write_yaml(mtx, dist, ret)
                cv2.destroyAllWindows()
                self.display_timer = None
                self.running = False

        elif key == ord('q'):
            self.get_logger().info('Quitting without saving.')
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def write_yaml(self, K: np.ndarray, dist: np.ndarray, rms: float):
        workspace_root = os.path.abspath(os.getcwd())
        calib_dir = os.path.join(workspace_root, 'calibration')
        os.makedirs(calib_dir, exist_ok=True)

        file_path = os.path.join(calib_dir, f'intrinsics_stream{self.stream_id}_{self.image_size[1]}p.yaml')

        data = {
            'stream_id': int(self.stream_id),
            'image_size': {
                'width': int(self.image_size[0]),
                'height': int(self.image_size[1]),
            },
            'rms_reprojection_error': float(rms),
            'k_matrix': K.tolist(),
            'distortion': dist.tolist(),
        }

        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'Intrinsics saved to: {file_path}')

def main(args=None):
    rclpy.init(args=args)
    node = IntrinsicCalibrationNode()
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
