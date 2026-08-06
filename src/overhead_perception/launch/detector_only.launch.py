import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('overhead_perception'),
        'config', 'detector.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('stream_ids',           default_value='0,1'),
        DeclareLaunchArgument('model_path',           default_value='yolo11m.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('max_fps',              default_value='5.0'),
        DeclareLaunchArgument('device',               default_value='auto'),

        Node(
            package='overhead_perception',
            executable='detector_node',
            name='detector_node',
            parameters=[
                {
                    'stream_ids':           LaunchConfiguration('stream_ids'),
                    'model_path':           LaunchConfiguration('model_path'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                    'max_fps':              LaunchConfiguration('max_fps'),
                    'device':               LaunchConfiguration('device'),
                },
                config,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
