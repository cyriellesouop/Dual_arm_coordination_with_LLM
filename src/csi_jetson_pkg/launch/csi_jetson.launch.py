# launch/csi_jetson.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('csi_jetson_pkg'),
        'config', 'csi_params.yaml'
    )

    return LaunchDescription([
        # default values (if a usb_params.yaml does not exist)
        DeclareLaunchArgument('camera_id',    default_value='0'),
        DeclareLaunchArgument('stream_id',    default_value='0'),
        DeclareLaunchArgument('width',        default_value='1280'),
        DeclareLaunchArgument('height',       default_value='720'),
        DeclareLaunchArgument('fps',          default_value='30'),
        DeclareLaunchArgument('jpeg_quality', default_value='95'),

        Node(
            package='csi_jetson_pkg',
            executable='csi_publisher_node',
            name='csi_publisher',
            parameters=[
                {
                    'camera_id':    LaunchConfiguration('camera_id'),
                    'stream_id':    LaunchConfiguration('stream_id'),
                    'width':        LaunchConfiguration('width'),
                    'height':       LaunchConfiguration('height'),
                    'fps':          LaunchConfiguration('fps'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                },
                config
            ],
            output='screen',
            emulate_tty=True,
        )
    ])