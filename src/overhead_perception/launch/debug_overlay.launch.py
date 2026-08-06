import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('overhead_perception'),
        'config', 'debug_overlay_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_ids',        default_value='0,1,2,3'),
        DeclareLaunchArgument('ref_stream_id',     default_value='1'),
        DeclareLaunchArgument('aruco_dict',        default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_ids',        default_value='0,1,2,3'),
        DeclareLaunchArgument('aruco_detect_rate', default_value='8.0'),

        Node(
            package='overhead_perception',
            executable='debug_overlay_node',
            name='debug_overlay_node',
            parameters=[
                {
                    'stream_ids':        LaunchConfiguration('stream_ids'),
                    'ref_stream_id':     LaunchConfiguration('ref_stream_id'),
                    'aruco_dict':        LaunchConfiguration('aruco_dict'),
                    'marker_ids':        LaunchConfiguration('marker_ids'),
                    'aruco_detect_rate': LaunchConfiguration('aruco_detect_rate'),
                },
                config,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
