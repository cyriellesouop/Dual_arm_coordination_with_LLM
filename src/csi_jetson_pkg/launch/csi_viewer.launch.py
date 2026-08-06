# src/csi_jetson_pkg/launch/csi_viewer.launch.py
#
# View CSI streams in a single tiled OpenCV window.
# Configure streams and tile size in config/csi_viewer_config.yaml.
#
# Override tile size at launch:
#   ros2 launch csi_jetson_pkg csi_viewer.launch.py tile_width:=1280 tile_height:=720

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('csi_jetson_pkg'),
        'config', 'csi_viewer_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('tile_width',  default_value='640',
                              description='Width of each stream tile in pixels'),
        DeclareLaunchArgument('tile_height', default_value='360',
                              description='Height of each stream tile in pixels'),

        Node(
            package='csi_jetson_pkg',
            executable='csi_subscriber_node',
            name='csi_subscriber',
            parameters=[
                config,
                {
                    'tile_width':  LaunchConfiguration('tile_width'),
                    'tile_height': LaunchConfiguration('tile_height'),
                },
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
