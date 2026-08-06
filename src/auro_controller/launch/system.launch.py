# system.launch.py
# Single command to bring up the full AURO pipeline.
#
# Usage:
#   ros2 launch auro_controller system.launch.py
#
# Common overrides:
#   enable_llm:=false    skip the LLM node (use when voice laptop runs it separately)
#   use_voice:=true      enable microphone input (needs mic + Ollama)
#   enable_arm:=true     enable Kinova arm (needs MoveIt2 running separately)
#   stream_ids:=0,1,2,3  which Jetson camera streams to use
#   ollama_model:=llama3 Ollama model name (must be pulled first)
#
# Split-laptop voice setup:
#   Desktop (no mic): ros2 launch auro_controller system.launch.py enable_llm:=false
#   Voice laptop:     OLLAMA_HOST=http://<desktop-ip>:11434 \
#                     ros2 launch llm_node_pkg llm_node.launch.py use_voice:=true
#
# What this starts:
#   1. overhead_perception: camera bridge, YOLO detector, object localiser, ArUco TF
#   2. auro_controller: controller node, RViz, static TF anchors
#   3. llm_node_pkg: LLM conversation node (skipped if enable_llm:=false)
#
# Prerequisites (must be running before launch):
#   - Jetsons streaming: run scripts/launch_jetsons.sh from a lab-network machine
#   - Ollama: ollama serve  (and: ollama pull llama3)
#   - MoveIt2 + Kinova driver: only needed if enable_arm:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    csi_share          = get_package_share_directory('csi_jetson_pkg')
    perception_share   = get_package_share_directory('overhead_perception_pkg')
    controller_share   = get_package_share_directory('auro_controller')
    llm_share          = get_package_share_directory('llm_node_pkg')

    args = [
        DeclareLaunchArgument(
            'stream_ids', default_value='0,1',
            description='Comma-separated Jetson stream IDs to use'),

        DeclareLaunchArgument(
            'enable_arm', default_value='false',
            description='Enable Kinova arm execution (requires MoveIt2 running separately)'),

        DeclareLaunchArgument(
            'use_voice', default_value='false',
            description='Enable microphone input for LLM (requires mic, speakers, and Ollama)'),

        DeclareLaunchArgument(
            'ollama_model', default_value='llama3',
            description='Ollama model name, must be pulled first: ollama pull llama3'),

        DeclareLaunchArgument(
            'whisper_model', default_value='base',
            description='Whisper model size for speech recognition: tiny, base, small, medium'),

        DeclareLaunchArgument(
            'enable_llm', default_value='false',
            description='Start the LLM node here; set false when a separate voice laptop runs llm_node'),
    ]

    prereq_banner = LogInfo(msg="""
================================================================================
  AURO SYSTEM LAUNCH: prerequisites checklist
  (this launch will start but topics will be empty if these are not running)

  1. Jetsons streaming:
       bash ~/auro-final-project/scripts/launch_jetsons.sh
       (run from a machine on the lab network before launching this)

  2. Ollama running (required for LLM node):
       ollama serve
       ollama pull llama3   (first time only)

  3. MoveIt2 + Kinova driver (only if enable_arm:=true):
       launch your MoveIt2 bringup separately

  Verify cameras are visible first:
       ros2 topic list | grep overhead
================================================================================
""")

    # 1a. Camera bridge: receives Jetson JPEG streams and publishes sensor_msgs/Image
    camera_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(csi_share, 'launch', 'camera_bridge.launch.py')),
        launch_arguments={
            'stream_ids': LaunchConfiguration('stream_ids'),
        }.items(),
    )

    # 1b. YOLO detector: Image to Detection2D bounding boxes
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'detector_only.launch.py')),
        launch_arguments={
            'stream_ids': LaunchConfiguration('stream_ids'),
        }.items(),
    )

    # 1c. Object localiser + ArUco: Detection2D to 3-D positions
    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'object_localization.launch.py')),
        launch_arguments={
            'stream_ids': LaunchConfiguration('stream_ids'),
        }.items(),
    )

    # 2. Controller + RViz + static TF anchors
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')),
        launch_arguments={
            'enable_arm': LaunchConfiguration('enable_arm'),
        }.items(),
    )

    # 3. LLM node (skipped when enable_llm:=false; voice laptop runs llm_node separately)
    llm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(llm_share, 'launch', 'llm_node.launch.py')),
        launch_arguments={
            'use_voice':     LaunchConfiguration('use_voice'),
            'ollama_model':  LaunchConfiguration('ollama_model'),
            'whisper_model': LaunchConfiguration('whisper_model'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_llm')),
    )

    return LaunchDescription(args + [prereq_banner, camera_bridge, detector, localizer, controller, llm])