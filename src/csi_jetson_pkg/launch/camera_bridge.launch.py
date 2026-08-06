import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('csi_jetson_pkg'),
        'config', 'camera_bridge_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_ids',    default_value='0,1'),
        DeclareLaunchArgument('input_prefix',  default_value='/stream_'),
        DeclareLaunchArgument('output_prefix', default_value='/video/stream_'),

        Node(
            package='csi_jetson_pkg',
            executable='camera_bridge_node',
            name='camera_bridge_node',
            parameters=[
                config,
                {
                    'stream_ids':    LaunchConfiguration('stream_ids'),
                    'input_prefix':  LaunchConfiguration('input_prefix'),
                    'output_prefix': LaunchConfiguration('output_prefix'),
                },
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
