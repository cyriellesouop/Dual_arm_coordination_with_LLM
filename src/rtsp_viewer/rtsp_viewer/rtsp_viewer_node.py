import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class RtspViewerNode(Node):
    def __init__(self):
        super().__init__('rtsp_viewer')

        self.declare_parameter('stream_ids',   [0, 1, 2, 3])
        self.declare_parameter('input_prefix', '/rtsp/stream_')
        self.declare_parameter('tile_width',   640)
        self.declare_parameter('tile_height',  360)

        stream_ids          = self.get_parameter('stream_ids').value
        input_prefix        = self.get_parameter('input_prefix').value
        self.tile_width     = self.get_parameter('tile_width').value
        self.tile_height    = self.get_parameter('tile_height').value
        self.stream_ids     = list(stream_ids)

        self.frames: dict = {sid: None for sid in self.stream_ids}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._subs = []
        for sid in self.stream_ids:
            topic = f'{input_prefix}{sid}/compressed'
            sub = self.create_subscription(
                CompressedImage, topic,
                lambda msg, s=sid: self._callback(msg, s),
                qos,
            )
            self._subs.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')

        n = len(self.stream_ids)
        self.cols = math.ceil(math.sqrt(n))
        self.rows = math.ceil(n / self.cols)

        self.win_name = 'RTSP Streams'
        cv2.namedWindow(
            self.win_name,
            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED,
        )
        cv2.resizeWindow(self.win_name, self.tile_width * self.cols, self.tile_height * self.rows)

        self.display_timer = self.create_timer(1.0 / 30.0, self._display_callback)

    def _callback(self, msg: CompressedImage, stream_id: int):
        arr = np.frombuffer(msg.data, np.uint8)
        self.frames[stream_id] = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _display_callback(self):
        tiles = []
        for sid in self.stream_ids:
            frame = self.frames.get(sid)
            if frame is None:
                tile = np.zeros((self.tile_height, self.tile_width, 3), dtype=np.uint8)
                cv2.putText(tile, f'stream_{sid}: waiting...',
                            (10, self.tile_height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
            else:
                tile = cv2.resize(frame, (self.tile_width, self.tile_height))
            cv2.putText(tile, f'stream_{sid}',
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 0), 2)
            tiles.append(tile)

        blank = np.zeros((self.tile_height, self.tile_width, 3), dtype=np.uint8)
        while len(tiles) < self.rows * self.cols:
            tiles.append(blank)

        rows = [np.hstack(tiles[r * self.cols:(r + 1) * self.cols]) for r in range(self.rows)]
        cv2.imshow(self.win_name, np.vstack(rows))
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RtspViewerNode()
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
