"""Map display and endpoint selection.

Publishes the existing map and listens for the pose selected in RViz:

    ros2 launch map_selection_test map_selection.launch.py

The map is read from this package's installed share directory, so there is no
dependence on any particular workspace path.

`map_selection_node` publishes /map itself, which is enough for RViz. Set
`use_map_server:=true` to bring up `nav2_map_server` as the map publisher
instead - it is lifecycle-managed, so this launch also configures and activates
it. Do not run both as map publishers: two transient-local publishers on /map
leave it ambiguous which one a subscriber latches.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'map_selection_test'


def generate_launch_description():
    default_map = os.path.join(get_package_share_directory(PACKAGE), 'maps', 'test.yaml')

    arguments = [
        DeclareLaunchArgument('map_yaml', default_value=default_map,
                              description='Map YAML; defaults to the packaged test.yaml.'),
        DeclareLaunchArgument('use_map_server', default_value='false',
                              description='Publish /map with nav2_map_server instead '
                                          'of map_selection_node.'),
    ]

    map_yaml = LaunchConfiguration('map_yaml')
    use_map_server = LaunchConfiguration('use_map_server')

    selection_node = Node(
        package=PACKAGE,
        executable='map_selection_node',
        name='map_selection_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'map_yaml': map_yaml}],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(use_map_server),
        parameters=[{'yaml_filename': map_yaml, 'topic_name': 'map', 'frame_id': 'map'}],
    )

    # map_server is a lifecycle node: it publishes nothing until it is
    # configured and activated. These are the two commands from the manual
    # four-terminal workflow, issued automatically.
    activate_map_server = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 2 && ros2 lifecycle set /map_server configure && '
             'ros2 lifecycle set /map_server activate'],
        output='screen',
        condition=IfCondition(use_map_server),
    )

    return LaunchDescription(arguments + [selection_node, map_server, activate_map_server])
