"""Gazebo Harmonic simulation of the VEEROBOT BeetleBot.

Brings up the simulated robot and the ROS bridge only - no navigation. This is
the stand-in for the physical robot's bringup (`lyra_bringup robot.launch.py`):
it provides exactly the interfaces the real robot provides, so the existing
navigation package runs against it unmodified.

    ros2 launch beetlebot_sim beetlebot_sim.launch.py world:=beetlebot_obstacles

On a machine without a GPU or display, wrap the whole command:

    xvfb-run -a ros2 launch beetlebot_sim beetlebot_sim.launch.py

Localisation (`localisation:=`):
    none    only odom -> base_footprint, from the simulated wheel odometry.
            Navigation must then run with global_frame:=odom.
    static  adds a perfect, fixed map -> odom. Convenient for exercising the
            map-frame code path, but it is a simulation shortcut: the real
            robot's AMCL drifts and corrects. Do not judge localisation
            robustness with this.
    amcl    runs nav2's AMCL against a map, as the real robot does.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'beetlebot_sim'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    xacro_file = os.path.join(share, 'urdf', 'beetlebot.urdf.xacro')
    bridge_config = os.path.join(share, 'config', 'bridge.yaml')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localisation = LaunchConfiguration('localisation')
    map_yaml = LaunchConfiguration('map')

    world_file = PythonExpression(["'", world, "' + '.sdf'"])
    world_path = PathJoinSubstitution([share, 'worlds', world_file])

    arguments = [
        DeclareLaunchArgument('world', default_value='beetlebot_obstacles',
                              description='World name in beetlebot_sim/worlds (without .sdf).'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Run the Gazebo GUI as well as the server.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('localisation', default_value='none',
                              description="none | static | amcl"),
        DeclareLaunchArgument('map', default_value=''),
    ]

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '-v', '2', world_path],
        output='screen', condition=UnlessCondition(gui))
    gz_full = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world_path],
        output='screen', condition=IfCondition(gui))

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'beetlebot',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', '0.03',
                   '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time}])

    # Perfect map -> odom. A deliberate simulation shortcut; see the module
    # docstring. Only active with localisation:=static.
    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='sim_map_to_odom', output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", localisation, "' == 'static'"])))

    amcl_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", localisation, "' == 'amcl'"])),
        actions=[
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 output='screen',
                 parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'base_frame_id': 'base_footprint',
                              'odom_frame_id': 'odom',
                              'global_frame_id': 'map',
                              'scan_topic': '/scan',
                              'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                              'set_initial_pose': True,
                              'always_reset_initial_pose': False,
                              # Seed AMCL at the spawn pose, which is what the
                              # operator does with RViz's "2D Pose Estimate".
                              'initial_pose.x': ParameterValue(
                                  LaunchConfiguration('x'), value_type=float),
                              'initial_pose.y': ParameterValue(
                                  LaunchConfiguration('y'), value_type=float),
                              'initial_pose.z': 0.0,
                              'initial_pose.yaw': ParameterValue(
                                  LaunchConfiguration('yaw'), value_type=float),
                              'laser_max_range': 12.0,
                              'laser_min_range': 0.1,
                              'max_beams': 180,
                              'min_particles': 500,
                              'max_particles': 2000}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'node_names': ['map_server', 'amcl']}]),
        ])

    return LaunchDescription(arguments + [
        gz_server, gz_full, rsp, spawn, bridge, static_map_odom, amcl_group])
