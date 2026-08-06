# src/camera_calibration/launch/intrinsic_aruco_calibration.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_calibration'),
        'config', 'intrinsic_aruco_calibration_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_id',      default_value='0'),
        DeclareLaunchArgument('squares_x',      default_value='11'),
        DeclareLaunchArgument('squares_y',      default_value='8'),
        DeclareLaunchArgument('square_length',  default_value='0.030'),
        DeclareLaunchArgument('marker_length',  default_value='0.022'),
        DeclareLaunchArgument('aruco_dict',     default_value='DICT_5X5_100'),
        DeclareLaunchArgument('min_samples',    default_value='25'),

        Node(
            package='camera_calibration',
            executable='intrinsic_aruco_calibration_node',
            name='intrinsic_aruco_calibration_node',
            parameters=[
                {
                    'stream_id':     LaunchConfiguration('stream_id'),
                    'squares_x':     LaunchConfiguration('squares_x'),
                    'squares_y':     LaunchConfiguration('squares_y'),
                    'square_length': LaunchConfiguration('square_length'),
                    'marker_length': LaunchConfiguration('marker_length'),
                    'aruco_dict':    LaunchConfiguration('aruco_dict'),
                    'min_samples':   LaunchConfiguration('min_samples'),
                },
                config,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
