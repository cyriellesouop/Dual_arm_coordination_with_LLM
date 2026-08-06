import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('llm_node_pkg')
    params_file = os.path.join(pkg_share, 'config', 'llm_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ollama_model', default_value='llama3',
            description='Ollama model name (must be pulled first)'
        ),
        DeclareLaunchArgument(
            'whisper_model', default_value='base',
            description='Whisper model size: tiny, base, small, medium'
        ),
        DeclareLaunchArgument(
            'use_voice', default_value='true',
            description='Use microphone input (true) or keyboard input (false)'
        ),
        Node(
            package='llm_node_pkg',
            executable='llm_node',
            name='llm_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'ollama_model': LaunchConfiguration('ollama_model'),
                    'whisper_model': LaunchConfiguration('whisper_model'),
                    'use_voice': LaunchConfiguration('use_voice'),
                },
            ],
        ),
    ])
