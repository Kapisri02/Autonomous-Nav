"""Launch the BeetleBot autonomous navigation node.

Usage on the robot, after the hardware bringup is already running:

    ros2 launch beetlebot_risk_nav beetlebot_nav.launch.py

Then set a destination with RViz's "2D Goal Pose" tool. Override anything from
the command line, for example:

    ros2 launch beetlebot_risk_nav beetlebot_nav.launch.py \\
        cmd_vel_topic:=/cmd_vel use_tf:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'beetlebot_risk_nav'


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'beetlebot_nav.yaml')

    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Parameter file for the navigation node.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_nav',
                              description='Where the safe velocity command is published.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_tf', default_value='true',
                              description='Use TF for the robot pose; false uses odometry only.'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # Needed to run against a simulator, which publishes /clock.
        # Defaults to false, so behaviour on the robot is unchanged.
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Follow /clock instead of the wall clock.'),
    ]

    node = Node(
        package=PACKAGE,
        executable='beetlebot_nav',
        name='beetlebot_nav',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'global_frame': LaunchConfiguration('global_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'use_tf': LaunchConfiguration('use_tf'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    return LaunchDescription(arguments + [node])
