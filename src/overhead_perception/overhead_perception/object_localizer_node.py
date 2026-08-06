import itertools
import json
import threading

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection2D as _Det2D
from vision_msgs.msg import Detection2DArray

_ROS_HUMBLE = hasattr(_Det2D().bbox.center, 'position')


def _build_projection(K, R_c2w, t_c2w):
    R_w2c = R_c2w.T
    t_w2c = (-R_w2c @ t_c2w).reshape(3, 1)
    return K @ np.hstack([R_w2c, t_w2c])


def _dlt(proj_matrices, pts_2d):
    """Direct Linear Transform for N >= 2 cameras. Returns world-frame 3D point or None."""
    A = []
    for P, (u, v) in zip(proj_matrices, pts_2d):
        A.append(u * P[2] - P[0])
        A.append(v * P[2] - P[1])
    _, _, Vt = np.linalg.svd(np.array(A))
    X = Vt[-1]
    if abs(X[3]) < 1e-9:
        return None
    return X[:3] / X[3]


def _reproj_err(P, X, pt):
    X_h = np.append(X, 1.0)
    proj = P @ X_h
    if abs(proj[2]) < 1e-9:
        return float('inf')
    proj = proj[:2] / proj[2]
    return float(np.linalg.norm(proj - np.array(pt)))


class ObjectLocalizerNode(Node):
    def __init__(self):
        super().__init__('object_localizer_node')

        self.declare_parameter('calibration_file', '')
        self.declare_parameter('stream_ids',       '0,1')
        self.declare_parameter('min_cameras',      2)
        self.declare_parameter('publish_rate',     5.0)
        self.declare_parameter('output_topic',     '/perception/objects/cam1')

        ids_str          = self.get_parameter('stream_ids').value
        self._stream_ids = [int(s) for s in ids_str.split(',')]
        self._min_cams   = int(self.get_parameter('min_cameras').value)

        self._P: dict = {}  # sid -> 3x4 projection matrix
        self._K: dict = {}  # sid -> 3x3 intrinsics matrix
        self._D: dict = {}  # sid -> distortion coefficients

        cal_file = self.get_parameter('calibration_file').value
        if cal_file:
            self._load_calibration(cal_file)
        else:
            self.get_logger().warn('No calibration_file — localisation disabled')

        self._dets: dict = {s: {} for s in self._stream_ids}
        self._lock = threading.Lock()

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        for sid in self._stream_ids:
            self.create_subscription(
                Detection2DArray,
                f'/perception/stream_{sid}/detections_2d',
                lambda msg, s=sid: self._on_det(msg, s),
                qos_sub)

        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        output_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(String, output_topic, qos_pub)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'ObjectLocalizerNode ready  streams={self._stream_ids}')

    def _load_calibration(self, path):
        with open(path) as f:
            data = yaml.safe_load(f)
        for key, cam in data.get('cameras', {}).items():
            sid   = int(key.split('_')[1])
            K     = np.array(cam['intrinsics']['k_matrix'], dtype=np.float64)
            D     = np.array(cam['intrinsics'].get('distortion', [[0] * 5]),
                             dtype=np.float64).flatten()
            R_c2w = np.array(cam['extrinsics']['R_c2w'],    dtype=np.float64)
            t_c2w = np.array(cam['extrinsics']['t_c2w'],    dtype=np.float64).flatten()
            self._K[sid] = K
            self._D[sid] = D
            self._P[sid] = _build_projection(K, R_c2w, t_c2w)
        self.get_logger().info(f'Calibration loaded: cameras {sorted(self._P)}')

    def _on_det(self, msg: Detection2DArray, sid: int):
        dets = {}
        for det in msg.detections:
            if not det.results:
                continue
            r = det.results[0]
            if _ROS_HUMBLE:
                label  = r.hypothesis.class_id
                cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            else:
                label  = r.id
                cx, cy = det.bbox.center.x, det.bbox.center.y
            if sid in self._K and sid in self._D:
                pt = np.array([[[cx, cy]]], dtype=np.float64)
                undist = cv2.undistortPoints(pt, self._K[sid], self._D[sid], P=self._K[sid])
                cx, cy = float(undist[0, 0, 0]), float(undist[0, 0, 1])
            dets.setdefault(label, []).append((cx, cy))
        with self._lock:
            self._dets[sid] = dets

    def _triangulate_label(self, visible_sids, Ps, det_lists):
        """
        Greedily assign and triangulate all instances of one label.

        Each iteration finds the best-scoring cross-camera combination among
        remaining (unused) detections, triangulates it with DLT using all
        cameras that still have unused detections, then marks those detections
        used. Continues until fewer than min_cameras have unused detections.

        This handles partial occlusion: if cam2 only sees one of two bottles,
        after that bottle is assigned cam2 drops out and the second bottle is
        triangulated with whichever cameras remain.
        """
        results  = []
        # remaining[i] = indices into det_lists[i] not yet assigned
        remaining = [list(range(len(dl))) for dl in det_lists]

        while True:
            active = [(i, s) for i, s in enumerate(visible_sids) if remaining[i]]
            if len(active) < self._min_cams:
                break

            a_idx  = [i for i, _ in active]
            a_sids = [s for _, s in active]
            a_Ps   = [Ps[i] for i in a_idx]
            a_rem  = [remaining[i] for i in a_idx]
            a_dets = [det_lists[i] for i in a_idx]

            best_err, best_X, best_combo = float('inf'), None, None

            for idx_combo in itertools.product(*a_rem):
                pts = tuple(a_dets[k][idx_combo[k]] for k in range(len(a_idx)))
                X   = _dlt(a_Ps, pts)
                if X is None or not np.isfinite(X).all():
                    continue
                err = sum(_reproj_err(a_Ps[k], X, pts[k])
                          for k in range(len(a_idx))) / len(a_idx)
                if err < best_err:
                    best_err, best_X, best_combo = err, X, idx_combo

            if best_X is None:
                break

            for k, orig_i in enumerate(a_idx):
                remaining[orig_i].remove(best_combo[k])

            results.append((best_X, best_err, a_sids))

        return results

    def _tick(self):
        if len(self._P) < 2:
            return

        with self._lock:
            snapshot = {s: dict(d) for s, d in self._dets.items()}

        class_views: dict = {}
        for sid, dets in snapshot.items():
            if sid not in self._P:
                continue
            for label, pts in dets.items():
                class_views.setdefault(label, {})[sid] = pts

        objects = []
        for label, cam_dets in class_views.items():
            visible_sids = [s for s in cam_dets if s in self._P]
            if len(visible_sids) < self._min_cams:
                continue

            Ps        = [self._P[s] for s in visible_sids]
            det_lists = [cam_dets[s] for s in visible_sids]

            for X, err, cams in self._triangulate_label(visible_sids, Ps, det_lists):
                objects.append({
                    'label':      label,
                    'x':          round(float(X[0]), 3),
                    'y':          round(float(X[1]), 3),
                    'z':          round(float(X[2]), 3),
                    'cameras':    cams,
                    'reproj_err': round(err, 2),
                })

        msg      = String()
        msg.data = json.dumps(objects)
        self._pub.publish(msg)
        if objects:
            self.get_logger().info(str(objects), throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
