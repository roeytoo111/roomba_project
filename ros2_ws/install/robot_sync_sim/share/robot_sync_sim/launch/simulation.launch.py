#!/usr/bin/env python3
"""Gazebo: world, dual robot_state_publisher, delayed spawn.

  Sets ROS_LOCALHOST_ONLY=1 (DDS discovery with gzserver). For cmd_vel from another shell:
    export ROS_LOCALHOST_ONLY=1 && source install/setup.bash

  Default: also starts leader + follower (robots move):
    ros2 launch robot_sync_sim simulation.launch.py

  Simulation only (no driving); run behavior.launch separately:
    ros2 launch robot_sync_sim simulation.launch.py start_behaviors:=false

  Do not run behavior.launch in parallel when start_behaviors=true (duplicate nodes).

  behaviors_use_sim_time (default false):
    leader/follower publish cmd_vel; timers work with wall clock unless true.
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _process_xacro(pkg_share: str, rel_path: str) -> str:
    path = os.path.join(pkg_share, rel_path)
    doc = xacro.process_file(path)
    return doc.toxml()


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sync_sim')
    world = os.path.join(pkg_share, 'worlds', 'empty.world')

    urdf_a_xml = _process_xacro(pkg_share, os.path.join('urdf', 'robot_a.urdf.xacro'))
    urdf_b_xml = _process_xacro(pkg_share, os.path.join('urdf', 'robot_b.urdf.xacro'))

    fd_a, path_a = tempfile.mkstemp(prefix='robot_a_', suffix='.urdf', text=True)
    fd_b, path_b = tempfile.mkstemp(prefix='robot_b_', suffix='.urdf', text=True)
    with os.fdopen(fd_a, 'w') as f:
        f.write(urdf_a_xml)
    with os.fdopen(fd_b, 'w') as f:
        f.write(urdf_b_xml)

    gazebo_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')

    sim_time = {'use_sim_time': True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    rsp_a = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot_a',
        output='screen',
        parameters=[
            sim_time,
            {'robot_description': urdf_a_xml},
        ],
    )

    rsp_b = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot_b',
        output='screen',
        parameters=[
            sim_time,
            {'robot_description': urdf_b_xml},
        ],
    )

    # base_link z: wheel bottom in base ~ -0.035 (h=0.07, R=0.036). Ground contact ~ spawn_z 0.035.
    spawn_a = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'robot_a',
            '-file', path_a,
            '-x', '0.0', '-y', '0.0', '-z', LaunchConfiguration('spawn_z'),
            '-R', '0', '-P', '0', '-Y', '0',
        ],
    )

    spawn_b = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'robot_b',
            '-file', path_b,
            '-x', '-1.0', '-y', '0.0', '-z', LaunchConfiguration('spawn_z'),
            '-R', '0', '-P', '0', '-Y', '0',
        ],
    )

    behavior_launch = os.path.join(pkg_share, 'launch', 'behavior.launch.py')
    behaviors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(behavior_launch),
        launch_arguments=[
            ('start_delay_sec', '12.0'),
            ('use_sim_time', LaunchConfiguration('behaviors_use_sim_time')),
        ],
        condition=IfCondition(LaunchConfiguration('start_behaviors')),
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_a],
    )
    delayed_spawn_b = TimerAction(
        period=2.6,
        actions=[spawn_b],
    )

    return LaunchDescription([
        # Improves DDS discovery between gzserver and Python nodes on multi-interface hosts.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start Gazebo client (set false for headless).'),
        DeclareLaunchArgument(
            'start_behaviors',
            default_value='true',
            description=(
                'If true (default), start leader and follower. '
                'If false, run behavior.launch.py separately.'
            ),
        ),
        DeclareLaunchArgument(
            'behaviors_use_sim_time',
            default_value='false',
            description='use_sim_time for leader/follower nodes.',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.038',
            description='Spawn height of base_link (meters). ~0.035 for ground contact.',
        ),
        gazebo,
        rsp_a,
        rsp_b,
        delayed_spawn,
        delayed_spawn_b,
        behaviors,
    ])
