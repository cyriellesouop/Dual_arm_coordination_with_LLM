# detector_node.py
#
# Subscribes to /overhead/cam_{id}/image_rect (sensor_msgs/Image, BEST_EFFORT)
# Runs YOLOv11m inference and publishes to /overhead/cam_{id}/detections_2d
# (vision_msgs/Detection2DArray, BEST_EFFORT)
#
# Parameters (see config/detector.yaml):
#   stream_ids          : comma-separated camera IDs to watch  (default "0")
#   model_path          : YOLO weights file                    (default "yolo11m.pt")
#   confidence_threshold: minimum detection score             (default 0.5)
#   max_fps             : inference rate cap per camera        (default 5.0)
#   device              : "auto", "cuda", or "cpu"            (default "auto")
#   class_filter        : list of COCO class names to keep    (default: common items)

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
_ROS_HUMBLE = hasattr(Detection2D().bbox.center, 'position')

import torch
from ultralytics import YOLO


_DEFAULT_CLASSES = [
    'apple', 'orange', 'banana', 'bottle', 'cup',
    'mouse', 'keyboard', 'laptop', 'cell phone', 'remote',
    'book', 'scissors',
]


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.declare_parameter('stream_ids',           '0')
        self.declare_parameter('model_path',           'yolo11m.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_fps',              5.0)
        self.declare_parameter('device',               'auto')
        self.declare_parameter('class_filter',         _DEFAULT_CLASSES)

        stream_ids_raw = self.get_parameter('stream_ids').value
        model_path     = self.get_parameter('model_path').value
        self.conf      = self.get_parameter('confidence_threshold').value
        self.max_fps   = self.get_parameter('max_fps').value
        device_param   = self.get_parameter('device').value
        class_filter   = list(self.get_parameter('class_filter').value)

        self.stream_ids = [int(s.strip()) for s in stream_ids_raw.split(',')]

        # Device selection
        if device_param == 'auto':
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device_param
        self.get_logger().info(f'Using device: {self.device}')

        # Load model once, shared across all cameras
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # Map class names → YOLO class indices for filtered inference
        name_to_idx = {v: k for k, v in self.model.names.items()}
        self.class_indices = [name_to_idx[n] for n in class_filter if n in name_to_idx]
        missing = [n for n in class_filter if n not in name_to_idx]
        if missing:
            self.get_logger().warn(f'Class names not in model: {missing}')
        self.get_logger().info(
            f'Filtering to {len(self.class_indices)} classes: '
            f'{[self.model.names[i] for i in self.class_indices]}'
        )

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._det_pubs     = {}
        self._last_infer_t = {}

        for sid in self.stream_ids:
            image_topic = f'/rtsp/stream_{sid}/compressed'
            det_topic   = f'/perception/stream_{sid}/detections_2d'

            self.create_subscription(
                CompressedImage,
                image_topic,
                lambda msg, s=sid: self._on_image(msg, s),
                sub_qos,
            )
            self._det_pubs[sid]     = self.create_publisher(Detection2DArray, det_topic, pub_qos)
            self._last_infer_t[sid] = 0.0

            self.get_logger().info(f'[stream_{sid}]  {image_topic} -> {det_topic}')

        self.get_logger().info(
            f'detector_node ready — streams: {self.stream_ids}  '
            f'model: {model_path}  conf: {self.conf}  max_fps: {self.max_fps}'
        )

    def _on_image(self, msg: CompressedImage, stream_id: int):
        now = time.perf_counter()
        min_interval = 1.0 / self.max_fps
        if now - self._last_infer_t[stream_id] < min_interval:
            return
        self._last_infer_t[stream_id] = now

        arr      = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        results = self.model.predict(
            cv_image,
            conf=self.conf,
            classes=self.class_indices if self.class_indices else None,
            device=self.device,
            verbose=False,
        )

        det_array = Detection2DArray()
        det_array.header = msg.header

        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                xyxy  = boxes.xyxy[i].cpu().tolist()
                score = float(boxes.conf[i].cpu())
                cls   = int(boxes.cls[i].cpu())

                x1, y1, x2, y2 = xyxy
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w  = x2 - x1
                h  = y2 - y1

                det = Detection2D()
                det.header = msg.header
                if _ROS_HUMBLE:
                    det.bbox.center.position.x = cx
                    det.bbox.center.position.y = cy
                else:
                    det.bbox.center.x = cx
                    det.bbox.center.y = cy
                det.bbox.size_x = w
                det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                if _ROS_HUMBLE:
                    hyp.hypothesis.class_id = self.model.names[cls]
                    hyp.hypothesis.score    = score
                else:
                    hyp.id    = self.model.names[cls]
                    hyp.score = score
                det.results.append(hyp)

                det_array.detections.append(det)

        self._det_pubs[stream_id].publish(det_array)

        elapsed_ms = (time.perf_counter() - now) * 1000
        self.get_logger().info(
            f'[stream_{stream_id}] {len(det_array.detections)} detections  '
            f'infer={elapsed_ms:.1f}ms',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
