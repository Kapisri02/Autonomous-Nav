"""Tests for the ROS node's own logic, using stubbed ROS modules.

What this covers: message conversion, parameter declaration, topic wiring, the
control cycle, goal handling, SUCCESS reporting and the shutdown stop.

What this does NOT cover: anything that depends on real middleware - QoS
negotiation, TF, message transport, timing under load. Those remain unverified
until the package runs on the robot.
"""

import math

import pytest

import ros_stubs

ros_stubs.install()

from beetlebot_risk_nav.nodes import beetlebot_nav_node as node_module  # noqa: E402
from synthetic import corridor, make_scan  # noqa: E402


def make_node(**parameters):
    node = node_module.BeetleBotNavNode()
    for key, value in parameters.items():
        node.set_parameter_value(key, value)
    return node


def scan_msg(stamp=1.0, **scan_kwargs):
    data = make_scan(stamp=stamp, **scan_kwargs)
    msg = ros_stubs.LaserScan()
    msg.header.frame_id = 'laser'
    msg.header.sec = int(stamp)
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp % 1.0) * 1e9)
    msg.angle_min = data.angle_min
    msg.angle_increment = data.angle_increment
    msg.range_min = data.range_min
    msg.range_max = data.range_max
    msg.ranges = list(data.ranges)
    return msg


def odom_msg(x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0, stamp=1.0):
    msg = ros_stubs.Odometry()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp % 1.0) * 1e9)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = v
    msg.twist.twist.angular.z = w
    return msg


def goal_msg(x, y, frame='map'):
    msg = ros_stubs.PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = x
    msg.pose.position.y = y
    return msg


def feed(node, stamp, **scan_kwargs):
    node._clock.seconds = stamp
    node.subscriptions_by_topic['/scan'](scan_msg(stamp=stamp, **scan_kwargs))
    node.subscriptions_by_topic['/odom'](odom_msg(stamp=stamp))
    node.timers[0].callback()


# ----------------------------------------------------------------------
def test_node_subscribes_and_publishes_on_the_expected_topics():
    node = make_node()
    assert set(node.subscriptions_by_topic) == {'/scan', '/odom', '/goal_pose'}
    assert '/cmd_vel_nav' in node.publishers_by_topic, 'must default to the baseline topic'
    assert '~/status' in node.publishers_by_topic
    assert '~/goal_reached' in node.publishers_by_topic


def test_control_loop_runs_at_the_configured_rate():
    node = make_node()
    assert node.timers and node.timers[0].period == pytest.approx(0.1)


def test_every_configuration_value_is_exposed_as_a_parameter():
    node = make_node()
    for key in node.config.to_flat():
        assert key in node._parameters, '{} is not settable from ROS'.format(key)


def test_laserscan_is_converted_faithfully():
    node = make_node()
    node.subscriptions_by_topic['/scan'](scan_msg(stamp=2.0, circles=[(1.0, 0.0, 0.2)]))
    assert node._scan is not None
    assert node._scan.count == 360
    assert node._scan.stamp == pytest.approx(2.0, abs=1e-6)
    assert node._scan.range_max > 0.0


def test_odometry_yaw_and_velocity_are_converted():
    node = make_node()
    node.subscriptions_by_topic['/odom'](odom_msg(x=1.0, y=2.0, yaw=0.5, v=0.15, w=-0.2))
    assert node._odom_pose[0] == pytest.approx(1.0)
    assert node._odom_pose[2] == pytest.approx(0.5)
    assert node._velocity == pytest.approx((0.15, -0.2))


def test_no_pose_yet_publishes_a_stop_rather_than_nothing():
    node = make_node()
    node.timers[0].callback()
    published = node.publishers_by_topic['/cmd_vel_nav'].messages
    assert published and published[-1].linear.x == 0.0


def test_without_a_goal_the_robot_is_commanded_to_hold_still():
    node = make_node()
    feed(node, 1.0, walls=corridor(2.0))
    last = node.publishers_by_topic['/cmd_vel_nav'].messages[-1]
    assert (last.linear.x, last.angular.z) == (0.0, 0.0)


def test_goal_from_rviz_starts_navigation_and_commands_motion():
    node = make_node()
    node.subscriptions_by_topic['/goal_pose'](goal_msg(3.0, 0.0))
    assert node.navigator.goal == (3.0, 0.0)
    for k in range(5):
        feed(node, 1.0 + k * 0.1, walls=corridor(2.0))
    last = node.publishers_by_topic['/cmd_vel_nav'].messages[-1]
    assert last.linear.x > 0.0


def test_reaching_the_goal_publishes_success_once_and_then_stops():
    node = make_node()
    node.subscriptions_by_topic['/goal_pose'](goal_msg(0.1, 0.0))
    for k in range(5):
        feed(node, 1.0 + k * 0.1, walls=corridor(2.0))

    reached = [m.data for m in node.publishers_by_topic['~/goal_reached'].messages]
    assert reached.count(True) == 1, 'SUCCESS must be announced exactly once'
    commands = node.publishers_by_topic['/cmd_vel_nav'].messages
    assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in commands[-3:])
    assert any('SUCCESS' in text for text in node._logger.records.get('info', []))


def test_status_messages_are_published():
    node = make_node()
    node.subscriptions_by_topic['/goal_pose'](goal_msg(3.0, 0.0))
    feed(node, 1.0, walls=corridor(2.0))
    assert node.publishers_by_topic['~/status'].messages


def test_stale_lidar_results_in_a_stop_command():
    node = make_node()
    node.subscriptions_by_topic['/goal_pose'](goal_msg(3.0, 0.0))
    for k in range(3):
        feed(node, 1.0 + k * 0.1, walls=corridor(2.0))
    # Scans stop arriving; only the clock advances.
    node._clock.seconds = 5.0
    node.subscriptions_by_topic['/odom'](odom_msg(stamp=5.0))
    node.timers[0].callback()
    last = node.publishers_by_topic['/cmd_vel_nav'].messages[-1]
    assert (last.linear.x, last.angular.z) == (0.0, 0.0)


def test_halt_publishes_repeated_stops():
    """Ctrl+C must never leave the wheels turning."""
    node = make_node()
    before = len(node.publishers_by_topic['/cmd_vel_nav'].messages)
    node.halt(repeats=10)
    published = node.publishers_by_topic['/cmd_vel_nav'].messages[before:]
    assert len(published) == 10
    assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in published)


def test_goal_in_an_unknown_frame_is_refused_without_tf():
    """Without TF we cannot honour a goal in another frame, and must say so."""
    node = make_node()
    node.subscriptions_by_topic['/goal_pose'](goal_msg(3.0, 0.0, frame='odd_frame'))
    assert node.navigator.goal is None
    assert node._logger.records.get('error')


def test_invalid_configuration_refuses_to_start():
    """Starting with a broken configuration is worse than not starting."""
    import beetlebot_risk_nav.core.params as params

    original = params.NavConfig.validate
    params.NavConfig.validate = lambda self: ['deliberate test failure']
    try:
        with pytest.raises(RuntimeError):
            node_module.BeetleBotNavNode()
    finally:
        params.NavConfig.validate = original
