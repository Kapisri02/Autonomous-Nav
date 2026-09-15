"""Guards on the merged two-package workspace.

These are cheap structural checks that catch the mistakes a workspace merge
actually makes: a package that stops being discoverable, a resource marker that
was not carried over, an entry point that no longer matches the module, or a
path that still points into the old workspace.
"""

import os
import re

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO_ROOT, 'src')
PACKAGES = ('beetlebot_risk_nav', 'map_selection_test')


def read(*parts):
    with open(os.path.join(*parts)) as handle:
        return handle.read()


@pytest.mark.parametrize('package', PACKAGES)
def test_package_is_a_complete_ament_python_package(package):
    """Everything colcon needs to discover and build the package."""
    root = os.path.join(SRC, package)
    assert os.path.isdir(root), '{} is missing from the workspace'.format(package)
    for required in ('package.xml', 'setup.py', 'setup.cfg'):
        assert os.path.isfile(os.path.join(root, required)), \
            '{}/{} is missing'.format(package, required)
    # The ament index marker: without it the package builds but is invisible to
    # ros2 run / ros2 launch.
    assert os.path.isfile(os.path.join(root, 'resource', package)), \
        '{}/resource/{} marker is missing'.format(package, package)
    assert os.path.isfile(os.path.join(root, package, '__init__.py'))
    assert '<build_type>ament_python</build_type>' in read(root, 'package.xml')


@pytest.mark.parametrize('package,executable,module', [
    ('beetlebot_risk_nav', 'beetlebot_nav',
     'beetlebot_risk_nav.nodes.beetlebot_nav_node:main'),
    ('map_selection_test', 'map_selection_node',
     'map_selection_test.map_selection_node:main'),
])
def test_entry_point_matches_a_real_module(package, executable, module):
    setup_py = read(SRC, package, 'setup.py')
    assert executable in setup_py
    path = module.split(':')[0].replace('.', os.sep) + '.py'
    assert os.path.isfile(os.path.join(SRC, package, path)), \
        'entry point {} points at a module that does not exist'.format(module)


def test_no_hardcoded_old_workspace_paths():
    """Nothing may reference ~/ros2_ws or a specific home directory."""
    offenders = []
    pattern = re.compile(r'ros2_ws|/home/[a-zA-Z0-9_.-]+/')
    for base, dirs, files in os.walk(SRC):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'build', 'install')]
        for name in files:
            # Code and configuration only. Documentation may legitimately name
            # the old workspace - maps/README.md tells you to copy test.pgm out
            # of it - and that is migration instructions, not a runtime path.
            if not name.endswith(('.py', '.xml', '.yaml', '.cfg')):
                continue
            path = os.path.join(base, name)
            for number, line in enumerate(read(path).splitlines(), 1):
                if pattern.search(line) and 'kapisri02@' not in line:
                    offenders.append('{}:{}: {}'.format(
                        os.path.relpath(path, REPO_ROOT), number, line.strip()))
    assert not offenders, 'old-workspace paths remain:\n' + '\n'.join(offenders)


def test_map_yaml_is_packaged_and_consistent():
    maps = os.path.join(SRC, 'map_selection_test', 'maps')
    yaml_path = os.path.join(maps, 'test.yaml')
    assert os.path.isfile(yaml_path), 'test.yaml was not carried over'
    content = read(yaml_path)
    assert 'image: test.pgm' in content
    # test.pgm is a binary asset copied in separately; the YAML must still name
    # a file that sits beside it, not an absolute path.
    assert not re.search(r'image:\s*/', content), 'map image must be a relative name'


def test_maps_are_installed_into_the_package_share():
    """Installing maps/ is what removes the need for an absolute map path."""
    setup_py = read(SRC, 'map_selection_test', 'setup.py')
    assert "'/maps'" in setup_py or '/maps' in setup_py
    assert 'glob' in setup_py


def test_endpoint_selection_node_is_preserved_verbatim():
    """The working node's behaviour must not have been altered by the merge."""
    source = read(SRC, 'map_selection_test', 'map_selection_test',
                  'map_selection_node.py')
    assert "super().__init__('map_selection_node')" in source
    assert "self.declare_parameter('map_yaml', '')" in source
    assert "'/map'" in source and "'/goal_pose'" in source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source
    # goal_callback logs the selection and publishes nothing: /goal_pose is
    # goal-only, and the start comes from localisation.
    assert 'def goal_callback' in source
    assert 'Selected point' in source


def test_navigation_goal_topic_is_configurable():
    """The nav node must be retargetable without touching the algorithm."""
    node = read(SRC, 'beetlebot_risk_nav', 'beetlebot_risk_nav', 'nodes',
                'beetlebot_nav_node.py')
    assert "declare_parameter('goal_topic', '/goal_pose')" in node
    assert "declare_parameter('cmd_vel_topic', '/cmd_vel_nav')" in node


def test_unified_launch_starts_both_packages():
    launch = read(SRC, 'beetlebot_risk_nav', 'launch', 'beetlebot_system.launch.py')
    assert 'map_selection_test' in launch
    assert 'beetlebot_nav.launch.py' in launch
    assert 'map_selection.launch.py' in launch


def test_nav2_is_not_in_the_decision_path():
    """nav2_map_server may publish the map; it must not plan or control.

    Checked by import, not by mentioning the word: params.py cites the robot's
    own nav2_params.yaml as the source of the verified robot_radius, which is
    provenance for a number, not a dependency on Nav2.
    """
    forbidden = re.compile(r'^\s*(?:import|from)\s+nav2', re.MULTILINE)
    nav_core = os.path.join(SRC, 'beetlebot_risk_nav', 'beetlebot_risk_nav')
    for base, dirs, files in os.walk(nav_core):
        dirs[:] = [d for d in dirs if d != '__pycache__']
        for name in files:
            if name.endswith('.py'):
                source = read(os.path.join(base, name))
                assert not forbidden.search(source), \
                    '{} imports from nav2'.format(name)
                for symbol in ('BasicNavigator', 'NavigateToPose', 'ComputePathToPose'):
                    assert symbol not in source, \
                        '{} uses the Nav2 action interface {}'.format(name, symbol)
