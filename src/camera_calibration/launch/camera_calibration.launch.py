# src/camera_calibration/launch/camera_calibration.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_calibration'),
        'config', 'calibration_config.yaml'
    )

    return LaunchDescription([
        # default values (if a usb_params.yaml does not exist)
        DeclareLaunchArgument('stream_ids',   default_value='0'),
        DeclareLaunchArgument('pattern_cols', default_value='1'),
        DeclareLaunchArgument('pattern_rows', default_value='1'),
        DeclareLaunchArgument('square_size',  default_value='1.0'),
        DeclareLaunchArgument('min_samples',  default_value='10'),

        Node(
            package='camera_calibration',
            executable='camera_calibration_node',
            name='calibration',
            parameters=[
                {
                    'stream_ids':    LaunchConfiguration('stream_ids'),
                    'pattern_cols':  LaunchConfiguration('pattern_cols'),
                    'pattern_rows':  LaunchConfiguration('pattern_rows'),
                    'square_size':   LaunchConfiguration('square_size'),
                    'min_samples':   LaunchConfiguration('min_samples'),
                },
                config
            ],
            output='screen',
            emulate_tty=True,
        )
    ])