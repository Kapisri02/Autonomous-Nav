"""Integration tests for the complete start-to-goal navigation flow."""

from beetlebot_risk_nav.core.navigator import (STATE_FAILED, STATE_GOAL_REACHED,
                                               STATE_IDLE, STATE_NAVIGATING,
                                               STATE_RECOVERY, Navigator)
from beetlebot_risk_nav.core.params import NavConfig
from synthetic import corridor, make_scan, room


def drive(navigator, cycles, pose_fn, scan_fn, dt=0.1, start=0.0):
    """Feed the navigator a sequence of scans and poses; return the last result."""
    result = None
    for k in range(cycles):
        now = start + k * dt
        pose = pose_fn(k)
        result = navigator.update(scan_fn(k, now), pose, (0.0, 0.0), now, odom_stamp=now)
    return result


def test_no_goal_means_idle_and_no_motion():
    navigator = Navigator()
    result = navigator.update(make_scan(), (0, 0, 0), (0, 0), 0.0, odom_stamp=0.0)
    assert result.state == STATE_IDLE
    assert result.command.as_tuple() == (0.0, 0.0)


def test_missing_scan_means_idle_and_no_motion():
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    result = navigator.update(None, (0, 0, 0), (0, 0), 0.0, odom_stamp=0.0)
    assert result.command.as_tuple() == (0.0, 0.0)


def test_goal_within_tolerance_reports_success_immediately():
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    result = navigator.update(make_scan(walls=corridor(2.0)), (2.95, 0.0, 0.0),
                              (0.0, 0.0), 0.1, odom_stamp=0.1)
    assert result.state == STATE_GOAL_REACHED
    assert result.succeeded is True
    assert result.command.as_tuple() == (0.0, 0.0)


def test_goal_reached_is_terminal_and_stays_stopped():
    """The completion requirement: no navigation commands after success."""
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    navigator.update(make_scan(walls=corridor(2.0)), (2.95, 0, 0), (0, 0), 0.1,
                     odom_stamp=0.1)
    for k in range(10):
        now = 0.2 + k * 0.1
        result = navigator.update(make_scan(stamp=now, walls=corridor(2.0)),
                                  (0.0, 0.0, 0.0),      # even if the pose jumps away
                                  (0.0, 0.0), now, odom_stamp=now)
        assert result.state == STATE_GOAL_REACHED
        assert result.command.as_tuple() == (0.0, 0.0)
        assert result.succeeded


def test_navigating_toward_a_clear_goal_commands_forward_motion():
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    result = drive(navigator, 5,
                   lambda k: (0.0, 0.0, 0.0),
                   lambda k, now: make_scan(stamp=now, walls=corridor(2.0)))
    assert result.state == STATE_NAVIGATING
    assert result.command.v > 0.0


def test_a_new_goal_resets_a_finished_navigation():
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    navigator.update(make_scan(walls=corridor(2.0)), (3.0, 0, 0), (0, 0), 0.1,
                     odom_stamp=0.1)
    assert navigator.state == STATE_GOAL_REACHED
    navigator.set_goal((6.0, 0.0), 1.0)
    assert navigator.state == STATE_NAVIGATING


def test_cancel_returns_to_idle_and_stops():
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    navigator.cancel()
    result = navigator.update(make_scan(walls=corridor(2.0)), (0, 0, 0), (0, 0), 0.5,
                              odom_stamp=0.5)
    assert result.state == STATE_IDLE
    assert result.command.as_tuple() == (0.0, 0.0)


def test_no_progress_eventually_fails_rather_than_running_forever():
    cfg = NavConfig()
    cfg.goal.progress_timeout = 1.0
    navigator = Navigator(cfg)
    navigator.set_goal((3.0, 0.0), 0.0)
    result = drive(navigator, 40,
                   lambda k: (0.0, 0.0, 0.0),          # never actually moves
                   lambda k, now: make_scan(stamp=now, walls=corridor(2.0)))
    assert result.state == STATE_FAILED
    assert result.command.as_tuple() == (0.0, 0.0)
    assert 'no progress' in result.status


def test_goal_timeout_fails():
    cfg = NavConfig()
    cfg.goal.goal_timeout = 1.0
    cfg.goal.progress_timeout = 1000.0
    navigator = Navigator(cfg)
    navigator.set_goal((30.0, 0.0), 0.0)
    result = drive(navigator, 30,
                   lambda k: (k * 0.02, 0.0, 0.0),
                   lambda k, now: make_scan(stamp=now, walls=corridor(2.0)))
    assert result.state == STATE_FAILED
    assert 'timeout' in result.status


def test_boxed_in_robot_enters_recovery_and_reverses():
    """Trapped against a wall: the robot must try the baseline's escape."""
    cfg = NavConfig()
    cfg.safety.stuck_time = 0.3
    navigator = Navigator(cfg)
    navigator.set_goal((3.0, 0.0), 0.0)
    entered_recovery = False
    reversed_at_some_point = False
    for k in range(40):
        now = k * 0.1
        scan = make_scan(stamp=now, circles=[(0.5, 0.0, 0.2)], walls=corridor(0.8))
        result = navigator.update(scan, (0.0, 0.0, 0.0), (0.0, 0.0), now, odom_stamp=now)
        if result.state == STATE_RECOVERY:
            entered_recovery = True
        if result.command.v < -1e-6:
            reversed_at_some_point = True
    assert entered_recovery, 'a boxed-in robot never attempted recovery'
    assert reversed_at_some_point, 'recovery never reversed'


def test_recovery_does_not_reverse_into_a_wall_behind():
    """With no room behind, recovery must rotate instead of backing into it."""
    cfg = NavConfig()
    cfg.safety.stuck_time = 0.3
    navigator = Navigator(cfg)
    navigator.set_goal((3.0, 0.0), 0.0)
    for k in range(25):
        now = k * 0.1
        # Blocked in front and hard up against a wall behind.
        scan = make_scan(stamp=now, circles=[(0.5, 0.0, 0.2)],
                         walls=[(-0.4, -2.0, -0.4, 2.0)])
        result = navigator.update(scan, (0.0, 0.0, 0.0), (0.0, 0.0), now, odom_stamp=now)
        assert result.command.v >= -1e-6, 'reversed with a wall directly behind'


def test_safety_supervisor_can_override_the_navigator():
    """A sensor fault must stop the robot regardless of what the planner wants."""
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    now = 0.0
    for k in range(4):
        now = k * 0.1
        navigator.update(make_scan(stamp=now, walls=corridor(2.0)), (0, 0, 0), (0, 0),
                         now, odom_stamp=now)
    stale = now + navigator.config.safety.odom_timeout + 0.5
    result = navigator.update(make_scan(stamp=stale, walls=corridor(2.0)),
                              (0, 0, 0), (0, 0), stale, odom_stamp=now)
    assert result.command.as_tuple() == (0.0, 0.0)
    assert result.safety is not None and not result.safety.healthy


def test_every_command_is_within_the_hard_safety_limits():
    cfg = NavConfig()
    navigator = Navigator(cfg)
    navigator.set_goal((5.0, 1.0), 0.0)
    for k in range(40):
        now = k * 0.1
        scan = make_scan(stamp=now, walls=room(-4, -3, 4, 3),
                         circles=[(1.5, 0.2, 0.3)])
        result = navigator.update(scan, (k * 0.01, 0.0, 0.0), (0.1, 0.0), now,
                                  odom_stamp=now)
        assert abs(result.command.v) <= cfg.safety.max_linear_speed + 1e-9
        assert abs(result.command.w) <= cfg.safety.max_angular_speed + 1e-9


def test_status_always_explains_the_current_decision():
    navigator = Navigator()
    navigator.set_goal((3.0, 0.0), 0.0)
    result = drive(navigator, 3, lambda k: (0, 0, 0),
                   lambda k, now: make_scan(stamp=now, walls=corridor(2.0)))
    assert 'to goal' in result.status
    assert 'risk' in result.status
