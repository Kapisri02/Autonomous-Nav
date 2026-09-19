"""Simulated BeetleBot + the existing autonomous navigation package.

This is the end-to-end validation entry point: it starts the simulation and
then includes `beetlebot_risk_nav`'s own launch file, unmodified, with its own
parameter file. The code under test is the real one.

    # navigate in the odometry frame (no localisation needed)
    xvfb-run -a ros2 launch beetlebot_sim beetlebot_sim_nav.launch.py \
        world:=beetlebot_obstacles global_frame:=odom

    # map frame, with AMCL localising against a map
    xvfb-run -a ros2 launch beetlebot_sim beetlebot_sim_nav.launch.py \
        world:=beetlebot_corridor localisation:=amcl map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('beetlebot_sim')
    nav_share = get_package_share_directory('beetlebot_risk_nav')

    arguments = [
        DeclareLaunchArgument('world', default_value='beetlebot_obstacles'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('localisation', default_value='none'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # The navigation package's own arguments, passed straight through.
        DeclareLaunchArgument('global_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('use_tf', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            nav_share, 'config', 'beetlebot_nav.yaml')),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'beetlebot_sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'localisation': LaunchConfiguration('localisation'),
            'map': LaunchConfiguration('map'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': 'true',
        }.items())

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'beetlebot_nav.launch.py')),
        launch_arguments={
            'global_frame': LaunchConfiguration('global_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'use_tf': LaunchConfiguration('use_tf'),
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('params_file'),
        }.items())

    return LaunchDescription(arguments + [sim, navigation])
