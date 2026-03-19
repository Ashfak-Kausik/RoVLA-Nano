"""
VLA System Launch File  (v2 — loads vla_params.yaml)
------------------------------------------------------
Usage:

  # Zero-hardware mock mode
  ros2 launch vla_system vla_full.launch.py

  # With real Intel RealSense camera
  ros2 launch vla_system vla_full.launch.py use_mock_vision:=false

  # LLM language parser (requires ANTHROPIC_API_KEY)
  ros2 launch vla_system vla_full.launch.py parser_mode:=llm

  # Full real hardware
  ros2 launch vla_system vla_full.launch.py \\
      use_mock_vision:=false use_mock_action:=false

  # Custom YOLO model
  ros2 launch vla_system vla_full.launch.py yolo_model:=/path/to/best.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('vla_system')

    args = [
        DeclareLaunchArgument(
            'use_mock_vision', default_value='true',
            description='Use mock YOLO (no camera/GPU required)'),
        DeclareLaunchArgument(
            'use_mock_action', default_value='true',
            description='Use mock MoveIt2 (no robot required)'),
        DeclareLaunchArgument(
            'parser_mode', default_value='rule',
            description="Language parser: 'rule' or 'llm'"),
        DeclareLaunchArgument(
            'yolo_model', default_value='yolo11n.pt',
            description='Path to YOLO model weights'),
        DeclareLaunchArgument(
            'conf_threshold', default_value='0.5',
            description='YOLO confidence threshold'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS2 log level'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'vla_params.yaml']),
            description='Path to parameter YAML'),
    ]

    params_file = LaunchConfiguration('params_file')
    log_args    = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    vision_node = Node(
        package='vision_node',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[params_file, {
            'use_mock':       LaunchConfiguration('use_mock_vision'),
            'model_path':     LaunchConfiguration('yolo_model'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
        }],
        arguments=log_args,
    )

    language_node = Node(
        package='language_node',
        executable='language_node',
        name='language_node',
        output='screen',
        parameters=[params_file, {
            'parser_mode': LaunchConfiguration('parser_mode'),
        }],
        arguments=log_args,
    )

    coordinator_node = Node(
        package='coordinator_node',
        executable='coordinator_node',
        name='task_coordinator',
        output='screen',
        parameters=[params_file],
        arguments=log_args,
    )

    action_node = Node(
        package='action_node',
        executable='action_node',
        name='action_node',
        output='screen',
        parameters=[params_file, {
            'use_mock': LaunchConfiguration('use_mock_action'),
        }],
        arguments=log_args,
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='╔══════════════════════════════════════╗'),
        LogInfo(msg='║        VLA System — Starting         ║'),
        LogInfo(msg='╚══════════════════════════════════════╝'),
        vision_node,
        language_node,
        coordinator_node,
        action_node,
        LogInfo(msg="VLA ready. Send: ros2 topic pub /raw_command std_msgs/String \"data: 'Pick the screw'\""),
    ])
