import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('overhead_perception')

    localizer_config = os.path.join(pkg, 'config', 'object_localizer_config.yaml')
    aruco_config     = os.path.join(pkg, 'config', 'aruco_localizer_config.yaml')

    return LaunchDescription([
        # ── object_localizer_node ─────────────────────────────────────────────
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('stream_ids',       default_value='0,1'),
        DeclareLaunchArgument('min_cameras',      default_value='2'),
        DeclareLaunchArgument('publish_rate',     default_value='5.0'),

        Node(
            package='overhead_perception',
            executable='object_localizer_node',
            name='object_localizer_node',
            parameters=[
                {
                    'calibration_file': LaunchConfiguration('calibration_file'),
                    'stream_ids':       LaunchConfiguration('stream_ids'),
                    'min_cameras':      LaunchConfiguration('min_cameras'),
                    'publish_rate':     LaunchConfiguration('publish_rate'),
                },
                localizer_config,
            ],
            output='screen',
            emulate_tty=True,
        ),

        # ── aruco_localizer_node ──────────────────────────────────────────────
        DeclareLaunchArgument('ref_stream_id', default_value='1'),
        DeclareLaunchArgument('aruco_dict',    default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_size',   default_value='0.097'),
        DeclareLaunchArgument('marker_ids',    default_value='0,1,2,3'),
        DeclareLaunchArgument('origin_id',     default_value='0'),
        DeclareLaunchArgument('update_rate',   default_value='1.0'),
        DeclareLaunchArgument('stale_timeout_sec', default_value='3.0'),

        Node(
            package='overhead_perception',
            executable='aruco_localizer_node',
            name='aruco_localizer_node',
            parameters=[
                {
                    'ref_stream_id': LaunchConfiguration('ref_stream_id'),
                    'aruco_dict':    LaunchConfiguration('aruco_dict'),
                    'marker_size':   LaunchConfiguration('marker_size'),
                    'marker_ids':    LaunchConfiguration('marker_ids'),
                    'origin_id':     LaunchConfiguration('origin_id'),
                    'update_rate':   LaunchConfiguration('update_rate'),
                    'stale_timeout_sec': LaunchConfiguration('stale_timeout_sec'),
                },
                aruco_config,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
