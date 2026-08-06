import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CameraBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_bridge_node')

        self.declare_parameter('stream_ids',     '0')
        self.declare_parameter('input_prefix',   '/stream_')
        self.declare_parameter('output_prefix',  '/video/stream_')

        stream_ids_raw  = self.get_parameter('stream_ids').value
        input_prefix    = self.get_parameter('input_prefix').value
        output_prefix   = self.get_parameter('output_prefix').value
        self.stream_ids = [int(s.strip()) for s in stream_ids_raw.split(',')]
        self.bridge     = CvBridge()

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
        )

        self._img_pubs    = {}
        self._frame_count = {s: 0 for s in self.stream_ids}
        self._count_t0    = {s: time.perf_counter() for s in self.stream_ids}

        for sid in self.stream_ids:
            in_topic  = f'{input_prefix}{sid}/compressed_jpeg'
            out_topic = f'{output_prefix}{sid}/raw'
            self.create_subscription(
                CompressedImage,
                in_topic,
                lambda msg, s=sid: self._on_compressed(msg, s),
                sub_qos,
            )
            self._img_pubs[sid] = self.create_publisher(Image, out_topic, pub_qos)
            self.get_logger().info(f'[stream_{sid}]  {in_topic} -> {out_topic}')

        self.get_logger().info(f'camera_bridge_node ready — streams: {self.stream_ids}')

    def _on_compressed(self, msg: CompressedImage, stream_id: int):
        t0       = time.perf_counter()
        buf      = np.frombuffer(msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(buf, cv2.IMREAD_COLOR)

        if cv_image is None:
            self.get_logger().warn(
                f'[stream_{stream_id}] cv2.imdecode returned None',
                throttle_duration_sec=5.0,
            )
            return

        img_msg        = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        img_msg.header = msg.header
        self._img_pubs[stream_id].publish(img_msg)

        self._frame_count[stream_id] += 1
        elapsed = time.perf_counter() - self._count_t0[stream_id]
        if elapsed >= 5.0:
            self.get_logger().info(
                f'[stream_{stream_id}] {self._frame_count[stream_id] / elapsed:.1f} Hz  '
                f'decode={1000*(time.perf_counter()-t0):.1f}ms',
            )
            self._frame_count[stream_id] = 0
            self._count_t0[stream_id]    = time.perf_counter()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
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
