import os
import threading
import time

import cv2
import rclpy

# Disable FFmpeg's internal buffer and enable low-delay decoding.
# fflags=nobuffer: don't buffer packets before decoding
# flags=low_delay: decoder skips reordering delay
# Without these, FFmpeg buffers several seconds and cap.read() serves stale frames.
os.environ.setdefault(
    'OPENCV_FFMPEG_CAPTURE_OPTIONS',
    'rtsp_transport;tcp|fflags;nobuffer|flags;low_delay',
)

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CameraBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_bridge_node')

        self.declare_parameter('stream_ids',      '0')
        self.declare_parameter('rtsp_urls',       '')
        self.declare_parameter('output_prefix',   '/rtsp/stream_')
        self.declare_parameter('rtspsrc_latency', 50)
        self.declare_parameter('jpeg_quality',    80)

        stream_ids_raw  = self.get_parameter('stream_ids').value
        rtsp_urls_raw   = self.get_parameter('rtsp_urls').value
        output_prefix   = self.get_parameter('output_prefix').value
        latency         = self.get_parameter('rtspsrc_latency').value
        self._jpeg_q    = int(self.get_parameter('jpeg_quality').value)

        self.stream_ids   = [int(s.strip()) for s in stream_ids_raw.split(',')]
        rtsp_url_list     = [u.strip() for u in rtsp_urls_raw.split(',') if u.strip()]

        if len(rtsp_url_list) != len(self.stream_ids):
            self.get_logger().error(
                f'rtsp_urls has {len(rtsp_url_list)} entries but stream_ids has {len(self.stream_ids)}'
            )
            raise RuntimeError('rtsp_urls / stream_ids length mismatch')

        self.rtsp_map = dict(zip(self.stream_ids, rtsp_url_list))
        self.bridge   = CvBridge()
        self._stop    = threading.Event()

        best_effort_1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._threads = []
        for sid, url in self.rtsp_map.items():
            pub_raw  = self.create_publisher(Image,           f'{output_prefix}{sid}/raw',        best_effort_1)
            pub_comp = self.create_publisher(CompressedImage, f'{output_prefix}{sid}/compressed', best_effort_1)
            t = threading.Thread(
                target=self._stream_thread,
                args=(sid, url, pub_raw, pub_comp, latency),
                daemon=True,
                name=f'rtsp_stream_{sid}',
            )
            self._threads.append(t)
            t.start()
            self.get_logger().info(
                f'[stream_{sid}]  {url}  ->  {output_prefix}{sid}/{{raw,compressed}}'
            )

        self.get_logger().info(f'camera_bridge_node ready — streams: {self.stream_ids}')

    def _stream_thread(self, stream_id: int, url: str, pub_raw, pub_comp, latency: int):
        frame_count = 0
        t0 = time.perf_counter()
        jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q]

        while not self._stop.is_set():
            self.get_logger().info(f'[stream_{stream_id}] Connecting to {url}')
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not cap.isOpened():
                self.get_logger().warning(
                    f'[stream_{stream_id}] Failed to open RTSP stream, retrying in 3s'
                )
                self._stop.wait(3.0)
                continue

            self.get_logger().info(f'[stream_{stream_id}] Connected')
            frame_count = 0
            t0 = time.perf_counter()

            # Grabber thread: reads from FFmpeg as fast as possible, keeping only the
            # latest frame. Runs independently so pub.publish() latency never stalls
            # the RTSP read loop.
            latest: list = [None]
            grab_ok: list = [True]
            lock = threading.Lock()

            def _grabber(cap=cap, latest=latest, grab_ok=grab_ok):
                while not self._stop.is_set():
                    ret, frame = cap.read()
                    with lock:
                        grab_ok[0] = ret
                        if ret:
                            latest[0] = frame
                    if not ret:
                        break

            grab_t = threading.Thread(
                target=_grabber, daemon=True, name=f'grab_{stream_id}'
            )
            grab_t.start()

            last_frame_id = id(None)

            while not self._stop.is_set():
                with lock:
                    ok    = grab_ok[0]
                    frame = latest[0]

                if not ok:
                    self.get_logger().warning(
                        f'[stream_{stream_id}] Read failed, reconnecting',
                        throttle_duration_sec=5.0,
                        clock=self.get_clock(),
                    )
                    break

                if frame is None or id(frame) == last_frame_id:
                    time.sleep(0.001)
                    continue

                last_frame_id = id(frame)
                now = self.get_clock().now().to_msg()

                # Compressed publish: JPEG encoding runs in C (releases GIL), message
                # is ~100 KB vs ~2.76 MB raw — eliminates cross-machine bandwidth
                # saturation and DDS deserialization cost in downstream subscribers.
                ret_enc, jpeg_buf = cv2.imencode('.jpg', frame, jpeg_params)
                if ret_enc:
                    comp_msg = CompressedImage()
                    comp_msg.header.stamp    = now
                    comp_msg.header.frame_id = f'camera_{stream_id}'
                    comp_msg.format          = 'jpeg'
                    comp_msg.data            = jpeg_buf.tobytes()
                    pub_comp.publish(comp_msg)

                # Raw publish: only serialise the full frame when something actually
                # needs it (calibration / aruco localizer). Skipping this when idle
                # prevents the large numpy→bytes copy from holding the GIL.
                if pub_raw.get_subscription_count() > 0:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    img_msg.header.stamp    = now
                    img_msg.header.frame_id = f'camera_{stream_id}'
                    pub_raw.publish(img_msg)

                frame_count += 1
                elapsed = time.perf_counter() - t0
                if elapsed >= 5.0:
                    self.get_logger().info(
                        f'[stream_{stream_id}] {frame_count / elapsed:.1f} Hz'
                    )
                    frame_count = 0
                    t0 = time.perf_counter()

            grab_t.join(timeout=2.0)
            cap.release()

        self.get_logger().info(f'[stream_{stream_id}] thread exiting')

    def destroy_node(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=5.0)
        super().destroy_node()


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
