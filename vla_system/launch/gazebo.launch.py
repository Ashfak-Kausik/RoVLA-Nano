"""
Gazebo Simulation Launch
-------------------------
Spawns the VLA world with Franka Panda and starts all VLA nodes
configured for simulation.

Usage:
  ros2 launch vla_system gazebo.launch.py

Prerequisites:
  sudo apt install ros-humble-gazebo-ros-pkgs
  sudo apt install ros-humble-franka-description
  sudo apt install ros-humble-franka-gazebo  (for Gazebo integration)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_vla    = FindPackageShare('vla_system')
    pkg_gazebo = FindPackageShare('gazebo_ros')
    pkg_franka = FindPackageShare('franka_description')

    world_file = PathJoinSubstitution([pkg_vla, 'gazebo', 'vla_world.world'])

    # ── Launch Gazebo ──────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
    )

    # ── Spawn Franka Panda Robot ──────────────────────────────────────
    franka_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'panda',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.77',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
        output='screen',
    )

    # ── Robot State Publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', PathJoinSubstitution([pkg_franka, 'panda.urdf.xacro'])]),
        }],
    )

    # ── Camera bridge (Gazebo → ROS2) ──────────────────────────────────
    # Publishes /camera/color/image_raw and /camera/depth/image_rect_raw
    camera_bridge = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'realsense_camera',
            '-file', PathJoinSubstitution([pkg_vla, 'gazebo', 'realsense_d435.urdf']),
            '-x', '0.5', '-y', '0.0', '-z', '1.2',
            '-R', '0.0', '-P', '0.8', '-Y', '0.0',
        ],
        output='screen',
    )

    # ── VLA Pipeline (simulation mode) ─────────────────────────────────
    vla = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_vla, 'launch', 'vla_full.launch.py'])
        ),
        launch_arguments={
            'use_mock_vision': 'false',   # use real Gazebo camera
            'use_mock_action': 'false',   # use real MoveIt2 (sim)
            'parser_mode':     'rule',
        }.items(),
    )

    return LaunchDescription([
        LogInfo(msg='Launching VLA simulation in Gazebo...'),
        gazebo,
        franka_spawn,
        robot_state_publisher,
        camera_bridge,
        vla,
    ])
