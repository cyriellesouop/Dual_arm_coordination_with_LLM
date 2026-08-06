import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rtsp_viewer'),
        'config', 'rtsp_viewer_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('tile_width',  default_value='640',
                              description='Width of each stream tile in pixels'),
        DeclareLaunchArgument('tile_height', default_value='360',
                              description='Height of each stream tile in pixels'),

        Node(
            package='rtsp_viewer',
            executable='rtsp_viewer_node',
            name='rtsp_viewer',
            parameters=[
                {
                    'tile_width':  LaunchConfiguration('tile_width'),
                    'tile_height': LaunchConfiguration('tile_height'),
                },
                config,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
