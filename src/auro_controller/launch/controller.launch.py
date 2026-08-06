# controller.launch.py
# Launch file for the AURO controller node.
#
# Usage:
#   ros2 launch auro_controller controller.launch.py
#   ros2 launch auro_controller controller.launch.py enable_arm:=true
#   ros2 launch auro_controller controller.launch.py params_file:=/my/path.yaml

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('auro_controller')
    default_params = os.path.join(pkg_share, 'config', 'controller_params.yaml')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the controller_params.yaml file',
    )

    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm', default_value='false',
        description='Set true to enable Kinova arm execution (MoveIt2 must be running)')

    controller_node = Node(
        package='auro_controller',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'enable_arm': LaunchConfiguration('enable_arm')},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        enable_arm_arg,
        controller_node,
    ])
