"""Complete BeetleBot system: map, endpoint selection and autonomous navigation.

    ros2 launch beetlebot_risk_nav beetlebot_system.launch.py

Brings up both packages in one command:

    map_selection_test   publishes the existing map on /map and reports the
                         endpoint picked in RViz
    beetlebot_risk_nav   drives to that endpoint with obstacle avoidance and
                         safety supervision, publishing to /cmd_vel_nav

RViz and the robot bringup are started separately - RViz because it is
interactive, and the bringup because it owns the hardware.

Both nodes subscribe to /goal_pose independently: the selection node logs the
chosen coordinates, the navigation node drives to them. Nothing is relayed
between them, so neither can block the other.

Nav2 is not in the decision path. `nav2_map_server` is available purely as an
alternative map publisher (`use_map_server:=true`); goal-directed navigation and
obstacle avoidance remain entirely ours.

Localisation note: with `global_frame:=map` (the default) the navigation node
needs a map -> base_link transform from your localisation before it will move.
Without localisation, run the whole system in the odometry frame:

    ros2 launch beetlebot_risk_nav beetlebot_system.launch.py global_frame:=odom
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NAV_PACKAGE = 'beetlebot_risk_nav'
MAP_PACKAGE = 'map_selection_test'


def generate_launch_description():
    nav_share = get_package_share_directory(NAV_PACKAGE)
    map_share = get_package_share_directory(MAP_PACKAGE)
    default_map = os.path.join(map_share, 'maps', 'test.yaml')

    arguments = [
        DeclareLaunchArgument('map_yaml', default_value=default_map,
                              description='Map YAML; defaults to the packaged test.yaml.'),
        DeclareLaunchArgument('use_map_server', default_value='false',
                              description='Use nav2_map_server as the /map publisher '
                                          'instead of map_selection_node.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_nav',
                              description='Final velocity output; feeds cmd_vel_mux.'),
        DeclareLaunchArgument('global_frame', default_value='map',
                              description='Frame goals are expressed in. Use odom when '
                                          'no localisation is running.'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_tf', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Follow /clock instead of the wall clock.'),
    ]

    endpoint_selection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_share, 'launch', 'map_selection.launch.py')),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
            'use_map_server': LaunchConfiguration('use_map_server'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'beetlebot_nav.launch.py')),
        launch_arguments={
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'global_frame': LaunchConfiguration('global_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'use_tf': LaunchConfiguration('use_tf'),
            'log_level': LaunchConfiguration('log_level'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription(arguments + [endpoint_selection, navigation])
