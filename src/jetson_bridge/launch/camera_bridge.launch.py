import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('jetson_bridge'),
        'config', 'camera_bridge_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_ids',      default_value='0,1'),
        DeclareLaunchArgument('rtsp_urls',       default_value=''),
        DeclareLaunchArgument('output_prefix',   default_value='/rtsp/stream_'),
        DeclareLaunchArgument('rtspsrc_latency', default_value='50'),

        Node(
            package='jetson_bridge',
            executable='camera_bridge_node',
            name='camera_bridge_node',
            parameters=[
                {
                    'stream_ids':      LaunchConfiguration('stream_ids'),
                    'rtsp_urls':       LaunchConfiguration('rtsp_urls'),
                    'output_prefix':   LaunchConfiguration('output_prefix'),
                    'rtspsrc_latency': LaunchConfiguration('rtspsrc_latency'),
                },
                config,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
