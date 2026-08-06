"""
frame_transform.py
Converts 3D points from the perception world frame to the arm base_link frame.
Supports a static offset mode (manually tuned offsets + yaw) and a tf2
live-lookup mode. Switch between them with use_tf in controller_params.yaml.
"""

import math

import rclpy.duration
import rclpy.logging


class FrameTransformer:
    # transforms 3D points from the perception frame to the arm planning frame

    def __init__(self, node, use_tf: bool, offset_x: float, offset_y: float,
                 offset_z: float, offset_yaw: float,
                 perception_frame: str = 'world',
                 ground_frame: str = 'base_link'):
        self._node = node
        self._use_tf = use_tf
        self._ox = offset_x
        self._oy = offset_y
        self._oz = offset_z
        self._yaw = offset_yaw
        self._perception_frame = perception_frame
        self._ground_frame = ground_frame
        self._log = node.get_logger()

        if use_tf:
            try:
                import tf2_ros
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(
                    self._tf_buffer, node)
                self._log.info(
                    f'FrameTransformer: tf2 mode  '
                    f'{perception_frame} to {ground_frame}')
            except Exception as exc:
                self._log.error(
                    f'FrameTransformer: tf2 init failed ({exc}). '
                    f'Falling back to static offset.')
                self._use_tf = False
        else:
            self._log.info(
                f'FrameTransformer: static-offset mode  '
                f'dx={offset_x} dy={offset_y} dz={offset_z} yaw={offset_yaw} rad')

    def transform(self, x: float, y: float, z: float,
                  source_frame: str | None = None) -> tuple:
        # converts a point to the arm planning frame, returns (tx, ty, tz)
        effective_source = source_frame if source_frame else self._perception_frame
        if self._use_tf:
            return self._tf_transform(x, y, z, effective_source)
        if effective_source != self._perception_frame:
            self._log.warn(
                f'FrameTransformer: source_frame "{effective_source}" differs from '
                f'configured perception_frame "{self._perception_frame}" but '
                f'use_tf=false, static offsets not calibrated for this frame. '
                f'Passing coordinates through unchanged.',
                throttle_duration_sec=5.0,
            )
            return (x, y, z)
        return self._static_transform(x, y, z)

    def _static_transform(self, x: float, y: float, z: float) -> tuple:
        # applies a yaw rotation then translation offset
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        tx = cos_y * x - sin_y * y + self._ox
        ty = sin_y * x + cos_y * y + self._oy
        tz = z + self._oz
        return (tx, ty, tz)

    def _tf_transform(self, x: float, y: float, z: float,
                      source_frame: str | None = None) -> tuple:
        try:
            import tf2_ros
            from geometry_msgs.msg import PointStamped
            import tf2_geometry_msgs  # noqa: F401, needed to register PointStamped transform

            pt = PointStamped()
            pt.header.frame_id = source_frame if source_frame else self._perception_frame
            pt.header.stamp = self._node.get_clock().now().to_msg()
            pt.point.x = x
            pt.point.y = y
            pt.point.z = z

            transformed = self._tf_buffer.transform(
                pt, self._ground_frame, timeout=rclpy.duration.Duration(seconds=0.1))
            return (transformed.point.x,
                    transformed.point.y,
                    transformed.point.z)

        except Exception as exc:
            self._log.warn(
                f'TF lookup failed ({exc}). Using identity (no transform).')
            return (x, y, z)
