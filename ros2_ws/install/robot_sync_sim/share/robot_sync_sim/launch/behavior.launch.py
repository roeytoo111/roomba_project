#!/usr/bin/env python3
"""
Behavior: leader (drive + trash spawn) and follower (pickup).

After simulation.launch.py:
  ros2 launch robot_sync_sim behavior.launch.py

Arguments:
  use_sim_time — default false (wall clock; use true if synced with Gazebo /clock).
  start_delay_sec — delay before starting nodes (default 1.0).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    cfg = context.launch_configurations
    use_sim = cfg.get('use_sim_time', 'false').lower() in ('true', '1', 'yes')
    delay = float(cfg.get('start_delay_sec', '1.0'))

    leader = Node(
        package='robot_sync_sim',
        executable='leader_node',
        name='leader_behavior',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim},
            {
                'linear_speed': 1.2,
                'drop_interval_m': 2.0,
                'drop_behind_m': 0.32,
                'progress_mode': 'full_path',
                'cmd_vel_forward_sign': 1.0,
                'odometry_warmup_samples': 15,
                'cmd_vel_topic': '/robot_a/cmd_vel',
                'odom_topic': '/robot_a/odom',
                'max_drops': 2,
            },
        ],
    )

    follower = Node(
        package='robot_sync_sim',
        executable='follower_node',
        name='follower_behavior',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim},
            {
                'b_spawn_x': -1.0,
                'b_spawn_y': 0.0,
                'goal_tolerance_m': 0.12,
                'vacuum_forward_offset_m': 0.14,
                'kp_linear': 2.8,
                'kp_angular': 3.0,
                'max_linear': 1.5,
                'max_angular': 2.0,
                'align_threshold_rad': 0.35,
                'cmd_vel_forward_sign': 1.0,
                'grasping_topic': '/robot_b/grasping',
                'grasp_timeout_s': 5.0,
                'post_grasp_hold_s': 0.5,
                'cmd_vel_topic': '/robot_b/cmd_vel',
                'odom_topic': '/robot_b/odom',
                'vacuum_service': '/robot_b/activate_vacuum',
                'max_pickups': 2,
                'post_mission_vacuum_duration_s': 5.0,
            },
        ],
    )

    return [TimerAction(period=delay, actions=[leader, follower])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'false: wall clock. true: use /clock from Gazebo (requires active simulation clock).'
            ),
        ),
        DeclareLaunchArgument(
            'start_delay_sec',
            default_value='1.0',
            description='Seconds to wait before starting leader and follower.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
