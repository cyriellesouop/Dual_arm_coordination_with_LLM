from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # standalone:=false (default) — arm_controller_ros: subscribes to /detected_objects
        #                               from controller_node; full pipeline integration.
        # standalone:=true            — arm_controller: original file; inject objects via
        #                               publish_detected_objects(); use for arm-only testing.
        DeclareLaunchArgument(
            'standalone', default_value='false',
            description='true = original arm_controller (standalone arm testing); '
                        'false = arm_controller_ros (integrated with controller_node)'),

        # Full pipeline mode (default)
        Node(
            package='kinova_pick_planner',
            executable='arm_controller_ros',
            name='arm_controller',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration('standalone')),
        ),

        # Standalone / fallback mode
        Node(
            package='kinova_pick_planner',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('standalone')),
        ),
    ])
