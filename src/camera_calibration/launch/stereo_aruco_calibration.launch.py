# src/camera_calibration/launch/stereo_aruco_calibration.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_calibration'),
        'config', 'stereo_aruco_calibration_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_id_left',          default_value='0'),
        DeclareLaunchArgument('stream_id_right',         default_value='1'),
        DeclareLaunchArgument('intrinsic_yaml_left',     default_value=''),
        DeclareLaunchArgument('intrinsic_yaml_right',    default_value=''),
        DeclareLaunchArgument('squares_x',               default_value='12'),
        DeclareLaunchArgument('squares_y',               default_value='9'),
        DeclareLaunchArgument('square_length',           default_value='0.1'),
        DeclareLaunchArgument('marker_length',           default_value='0.075'),
        DeclareLaunchArgument('aruco_dict',              default_value='DICT_4X4_100'),
        DeclareLaunchArgument('min_samples',             default_value='25'),
        DeclareLaunchArgument('min_corner_displacement', default_value='20.0'),
        DeclareLaunchArgument('max_sync_delta_ms',       default_value='50.0'),

        Node(
            package='camera_calibration',
            executable='stereo_aruco_calibration_node',
            name='stereo_aruco_calibration_node',
            parameters=[
                {
                    'stream_id_left':          LaunchConfiguration('stream_id_left'),
                    'stream_id_right':         LaunchConfiguration('stream_id_right'),
                    'intrinsic_yaml_left':     LaunchConfiguration('intrinsic_yaml_left'),
                    'intrinsic_yaml_right':    LaunchConfiguration('intrinsic_yaml_right'),
                    'squares_x':               LaunchConfiguration('squares_x'),
                    'squares_y':               LaunchConfiguration('squares_y'),
                    'square_length':           LaunchConfiguration('square_length'),
                    'marker_length':           LaunchConfiguration('marker_length'),
                    'aruco_dict':              LaunchConfiguration('aruco_dict'),
                    'min_samples':             LaunchConfiguration('min_samples'),
                    'min_corner_displacement': LaunchConfiguration('min_corner_displacement'),
                    'max_sync_delta_ms':       LaunchConfiguration('max_sync_delta_ms'),
                },
                config,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
