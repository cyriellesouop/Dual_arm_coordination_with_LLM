# src/camera_calibration/launch/intrinsic_calibration.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_calibration'),
        'config', 'intrinsic_calibration_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_id',                default_value='0'),
        DeclareLaunchArgument('pattern_cols',             default_value='6'),
        DeclareLaunchArgument('pattern_rows',             default_value='8'),
        DeclareLaunchArgument('square_size',              default_value='0.029'),
        DeclareLaunchArgument('min_samples',              default_value='10'),

        Node(
            package='camera_calibration',
            executable='intrinsic_checkerboard_calibration_node',
            name='intrinsic_checkerboard_calibration_node',
            parameters=[
                {
                    'stream_id':               LaunchConfiguration('stream_id'),
                    'pattern_cols':            LaunchConfiguration('pattern_cols'),
                    'pattern_rows':            LaunchConfiguration('pattern_rows'),
                    'square_size':             LaunchConfiguration('square_size'),
                    'min_samples':             LaunchConfiguration('min_samples'),
                },
                config,  # config file values override launch-arg defaults
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
