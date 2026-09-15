"""Complete-system integration: the whole pipeline in one run.

Every stage is exercised together through the ROS node - message conversion,
filtering, detection, tracking, risk, speed envelope, motion selection,
footprint checking, safety supervision, goal completion and logging - and the
resulting log is then read back by the analysis tool.

Robot poses are **scripted**, not integrated from the commands. This is a test
of the decision pipeline, not a simulation of the robot: there is no dynamics
model here, and nothing in this file says anything about physical behaviour.
"""

import csv
import importlib.util
import os

import pytest

import ros_stubs

ros_stubs.install()

import math  # noqa: E402

from beetlebot_risk_nav.nodes import beetlebot_nav_node as node_module  # noqa: E402
from synthetic import corridor, make_scan  # noqa: E402

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'scripts')


def scan_msg(stamp, **scan_kwargs):
    data = make_scan(stamp=stamp, **scan_kwargs)
    msg = ros_stubs.LaserScan()
    msg.header.frame_id = 'laser'
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp % 1.0) * 1e9)
    msg.angle_min = data.angle_min
    msg.angle_increment = data.angle_increment
    msg.range_min = data.range_min
    msg.range_max = data.range_max
    msg.ranges = list(data.ranges)
    return msg


def odom_msg(x, stamp, v=0.1):
    msg = ros_stubs.Odometry()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp % 1.0) * 1e9)
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = v
    return msg


def goal_msg(x, y):
    # Odometry frame: the stubs provide no tf2_ros, so there is no map ->
    # base_link transform and a map-frame goal would (correctly) be refused.
    msg = ros_stubs.PoseStamped()
    msg.header.frame_id = 'odom'
    msg.pose.position.x = x
    msg.pose.position.y = y
    return msg


@pytest.fixture
def running_node(tmp_path):
    node = node_module.BeetleBotNavNode()
    node.set_parameter_value('global_frame', 'odom')
    node.config.logging.directory = str(tmp_path)
    node.logger.config.directory = str(tmp_path)
    return node


def test_full_start_to_goal_run_reaches_success_and_logs_it(running_node, tmp_path):
    """Start -> navigate past a static obstacle and a mover -> arrive -> SUCCESS."""
    node = running_node
    node.subscriptions_by_topic['/goal_pose'](goal_msg(2.0, 0.0))

    # Poses are scripted, so obstacles are placed clear of the scripted path:
    # the robot cannot steer away from something put directly in front of it
    # when its pose does not respond to its commands. What is under test here is
    # that the whole pipeline runs and stays within its limits, not avoidance
    # behaviour, which is covered by the local planner and navigator tests.
    x = 0.0
    commands = []
    for k in range(40):
        stamp = 1.0 + k * 0.1
        node._clock.seconds = stamp
        # A static box to one side, and an obstacle moving at 0.25 m/s that
        # stays clear of the scripted path. (An earlier version of this test put
        # the mover across the path: the planner correctly commanded a stop, the
        # scripted pose drove on regardless, and the "collision" was an artefact
        # of the harness rather than a fault in the system.)
        mover_y = -2.0 + 0.25 * (k * 0.1)
        node.subscriptions_by_topic['/scan'](scan_msg(
            stamp, walls=corridor(1.8),
            circles=[(1.3 - x, 0.80, 0.22), (1.8 - x, mover_y, 0.25)]))
        node.subscriptions_by_topic['/odom'](odom_msg(x, stamp))
        node.timers[0].callback()
        commands.append(node.publishers_by_topic['/cmd_vel_nav'].messages[-1])
        x += 0.055

    # --- the run finished successfully ---
    assert node.navigator.state == 'GOAL_REACHED'
    reached = [m.data for m in node.publishers_by_topic['~/goal_reached'].messages]
    assert reached.count(True) == 1

    # --- it actually drove, and never exceeded the hard safety limits ---
    assert any(m.linear.x > 0.0 for m in commands), 'the robot never moved'
    limits = node.config.safety
    for msg in commands:
        assert abs(msg.linear.x) <= limits.max_linear_speed + 1e-9
        assert abs(msg.angular.z) <= limits.max_angular_speed + 1e-9
        assert math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)

    # --- and it stayed stopped afterwards ---
    tail = node.publishers_by_topic['/cmd_vel_nav'].messages[-3:]
    assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in tail)

    # --- the run was logged, and the log is readable by the analysis tool ---
    logs = [f for f in os.listdir(str(tmp_path)) if f.endswith('.csv')]
    assert len(logs) == 1
    path = os.path.join(str(tmp_path), logs[0])
    with open(path, newline='') as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > 10
    assert rows[-1]['state'] == 'GOAL_REACHED'

    spec = importlib.util.spec_from_file_location(
        'analyze_log', os.path.join(SCRIPTS, 'analyze_log.py'))
    analyze = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyze)
    summary = analyze.summarise(path)
    assert summary['succeeded'] is True
    assert summary['min_clearance'] > 0.0, 'the log shows the footprint was violated'


def test_pipeline_stages_all_contribute_during_a_run(running_node):
    """Every stage must show evidence of doing its job in a single run."""
    node = running_node
    node.subscriptions_by_topic['/goal_pose'](goal_msg(3.0, 0.0))

    saw_turn = False

    x = 0.0
    for k in range(35):
        stamp = 1.0 + k * 0.1
        node._clock.seconds = stamp
        mover_y = -1.2 + 0.7 * (k * 0.1)
        node.subscriptions_by_topic['/scan'](scan_msg(
            stamp, walls=corridor(1.6), circles=[(1.6 - x, mover_y, 0.25)]))
        node.subscriptions_by_topic['/odom'](odom_msg(x, stamp))
        node.timers[0].callback()
        x += 0.03

        if abs(node.publishers_by_topic['/cmd_vel_nav'].messages[-1].angular.z) > 0.05:
            saw_turn = True

    # Inspect one cycle directly for evidence that each stage produced output.
    scan = make_scan(stamp=99.0, walls=corridor(1.6), circles=[(1.2, 0.2, 0.25)])
    plan = node.navigator.planner.plan(scan, (3.0, 0.0))
    assert plan.debug.n_points > 0, 'filtering produced no points'
    assert plan.debug.n_obstacles > 0, 'detection produced no obstacles'
    assert plan.debug.n_tracks > 0, 'tracking produced no tracks'
    assert plan.speed_limit > 0.0, 'the speed envelope produced no limit'
    assert plan.assessment.level in ('CLEAR', 'CAUTION', 'DANGER', 'CRITICAL')
    assert plan.assessment.reason, 'risk assessment gave no reason'
    assert plan.debug.chosen, 'no motion option was selected'
    assert saw_turn, 'the robot never steered at all during the run'


def test_sensor_failure_mid_run_stops_the_robot(running_node):
    """A LiDAR dropout partway through a run must halt it, not coast."""
    node = running_node
    node.subscriptions_by_topic['/goal_pose'](goal_msg(3.0, 0.0))

    x = 0.0
    for k in range(6):
        stamp = 1.0 + k * 0.1
        node._clock.seconds = stamp
        node.subscriptions_by_topic['/scan'](scan_msg(stamp, walls=corridor(1.6)))
        node.subscriptions_by_topic['/odom'](odom_msg(x, stamp))
        node.timers[0].callback()
        x += 0.02
    assert node.publishers_by_topic['/cmd_vel_nav'].messages[-1].linear.x > 0.0

    # The LiDAR goes silent; odometry keeps arriving.
    for k in range(8):
        stamp = 2.0 + k * 0.1
        node._clock.seconds = stamp
        node.subscriptions_by_topic['/odom'](odom_msg(x, stamp))
        node.timers[0].callback()
    last = node.publishers_by_topic['/cmd_vel_nav'].messages[-1]
    assert (last.linear.x, last.angular.z) == (0.0, 0.0)


def test_baseline_remains_available_alongside_the_new_system():
    """The fallback must still be there and still be importable."""
    from beetlebot_risk_nav.baseline.baseline_controller import BaselineAvoidanceController
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    original = os.path.join(root, 'baseline', 'lyra_control', 'obstacle_avoidance.py')
    assert os.path.exists(original)
    assert BaselineAvoidanceController().safe_distance == pytest.approx(0.5)
