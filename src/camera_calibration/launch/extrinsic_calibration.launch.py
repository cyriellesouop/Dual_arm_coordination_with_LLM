# src/camera_calibration/launch/extrinsic_calibration.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_calibration'),
        'config', 'extrinsic_calibration_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_id_left',       default_value='0'),
        DeclareLaunchArgument('stream_id_right',      default_value='1'),
        DeclareLaunchArgument('intrinsic_yaml_left',  default_value=''),
        DeclareLaunchArgument('intrinsic_yaml_right', default_value=''),
        DeclareLaunchArgument('pattern_cols',         default_value='6'),
        DeclareLaunchArgument('pattern_rows',         default_value='8'),
        DeclareLaunchArgument('square_size',          default_value='0.029'),
        DeclareLaunchArgument('min_samples',          default_value='25'),

        Node(
            package='camera_calibration',
            executable='extrinsic_calibration_node',
            name='extrinsic_calibration_node',
            parameters=[
                {
                    'stream_id_left':       LaunchConfiguration('stream_id_left'),
                    'stream_id_right':      LaunchConfiguration('stream_id_right'),
                    'intrinsic_yaml_left':  LaunchConfiguration('intrinsic_yaml_left'),
                    'intrinsic_yaml_right': LaunchConfiguration('intrinsic_yaml_right'),
                    'pattern_cols':         LaunchConfiguration('pattern_cols'),
                    'pattern_rows':         LaunchConfiguration('pattern_rows'),
                    'square_size':          LaunchConfiguration('square_size'),
                    'min_samples':          LaunchConfiguration('min_samples'),
                },
                config,  # config file values override launch-arg defaults
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
