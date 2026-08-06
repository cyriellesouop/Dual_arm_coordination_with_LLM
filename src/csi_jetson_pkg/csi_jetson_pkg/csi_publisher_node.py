# csi_jetson_pkg/csi_publisher_node.py
# 
# Captures from a Jetson Nano CSI camera via GStreamer (nvarguscamerasrc)
# and publishes sensor_msgs/CompressedImage.
#
# Launch (after specifying camera paramters in config/csi_params.yaml):
#   ros2 launch csi_jetson_pkg csi_jetson.launch.py
#
# Alternatively run:
#   ros2 run csi_jetson_pkg csi_publisher_node \
#     --ros-args \
#     -p camera_id:=0 \
#     -p stream_id:=0 \
#     -p width:=1280 \
#     -p height:=720 \
#     -p fps:=30 \
#     -p jpeg_quality:=80

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2

class CSIPublisherNode(Node):
    def __init__(self):
        super().__init__('csi_publisher')

        # parameters
        self.declare_parameter('camera_id', 0) # /dev/video*
        self.declare_parameter('stream_id', 0) # stream id
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 95)


        self.camera_id   = self.get_parameter('camera_id').value
        self.stream_id   = self.get_parameter('stream_id').value
        self.width       = self.get_parameter('width').value
        self.height      = self.get_parameter('height').value
        self.fps         = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value


        self.cap = self.open_capture()

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open csi camera (id={self.camera_id})'
            )
            raise RuntimeError('Camera failed to open')
        
        # only keep last frame (drop frames if subscriber is too slow)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        base_topic = f'/video/stream_{self.stream_id}'
        self.image_pub = self.create_publisher(
            CompressedImage,
            f'{base_topic}/compressed_jpeg',
            qos
        )

        self.frame_id = f'camera_{self.stream_id}'
        self.missed_frames = 0
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.get_logger().info(
            f'[stream_{self.stream_id}] CSI camera ready.\n'
            f'  compressed: {base_topic}compressed_jpeg\n'
        )

    # initialize capture

    def open_capture(self) -> cv2.VideoCapture:
        pipeline = self.gstreamer_csi_pipeline()
        self.get_logger().info(f'CSI GStreamer pipeline: {pipeline}')
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        return cap

    def gstreamer_csi_pipeline(self) -> str:
        '''
        NV12 stays in NVMM (Jetson multimedia API) through nvvidconv,
        then a single videoconvert step converts to BGR for cv_bridge.
        '''
        return (
            f'nvarguscamerasrc sensor-id={self.camera_id} ! '
            f'video/x-raw(memory:NVMM), '
            f'width=(int){self.width}, height=(int){self.height}, '
            f'format=(string)NV12, framerate=(fraction){self.fps}/1 ! '
            f'nvvidconv flip-method=0 ! '
            f'video/x-raw, format=(string)NV12 ! '
            f'videoconvert ! '
            f'video/x-raw, format=(string)BGR ! '
            f'appsink drop=1'
        )
    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.missed_frames += 1
            self.get_logger().warn(
                f'[stream_{self.stream_id}] Frame capture failed '
                f'(total missed: {self.missed_frames})',
                throttle_duration_sec=5.0, # dont spam log
            )
            return
        
        self.missed_frames = 0

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        success, buf = cv2.imencode('.jpg', frame, encode_params)
        if not success:
            self.get_logger().warn('JPEG encode failed')
            return

        msg = CompressedImage()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format          = 'jpeg'
        msg.data            = buf.tobytes()

        self.image_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info(
            f'[stream_{self.stream_id}] shutting down, releasing camera.'
        )
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CSIPublisherNode()
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