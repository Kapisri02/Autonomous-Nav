"""Integration tests for local obstacle avoidance.

These exercise the whole Phase 1 chain in one call - filter, cluster, track,
risk, speed envelope, motion selection, footprint check - against synthetic
scans whose correct behaviour is known. They are software tests: they say
nothing about how the robot behaves physically.
"""

import math

import pytest

from beetlebot_risk_nav.core.local_planner import LocalPlanner
from beetlebot_risk_nav.core.params import NavConfig
from beetlebot_risk_nav.core.types import ScanData
from synthetic import corridor, make_scan, room

GOAL_AHEAD = (3.0, 0.0)


def run(planner, scans, goal=GOAL_AHEAD):
    """Feed scans in sequence, closing the loop only on commanded velocity."""
    result = None
    for scan in scans:
        result = planner.plan(scan, goal, robot_velocity=planner.last_command)
    return result


def steady(planner, goal=GOAL_AHEAD, cycles=4, **scan_kwargs):
    return run(planner, [make_scan(stamp=k * 0.1, **scan_kwargs) for k in range(cycles)], goal)


# ----------------------------------------------------------------------
def test_open_space_with_no_returns_is_still_navigable():
    """All beams report no-return: that IS information - nothing is in range."""
    planner = LocalPlanner()
    clear = ScanData(stamp=0.0, angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                     ranges=[float('inf')] * 360, range_min=0.1, range_max=12.0)
    result = planner.plan(clear, GOAL_AHEAD)
    assert result.command.v > 0.0
    assert result.debug.state == 'PLAN'


def test_clear_corridor_drives_straight_at_full_speed():
    result = steady(LocalPlanner(), walls=corridor(1.5))
    assert result.command.v == pytest.approx(NavConfig().velocity.nominal_speed, rel=0.05)
    assert result.command.w == pytest.approx(0.0)
    assert result.assessment.level == 'CLEAR'


def test_empty_scan_produces_no_motion():
    """Regression: an empty scan once read as "no obstacles" and drove at full speed."""
    empty = ScanData(stamp=0.0, angle_min=-math.pi, angle_increment=0.01, ranges=[])
    result = LocalPlanner().plan(empty, GOAL_AHEAD)
    assert result.command.v == 0.0 and result.command.w == 0.0
    assert 'insufficient' in result.command.reason.lower()


def test_all_invalid_scan_produces_no_forward_motion():
    """Every beam NaN: the robot knows nothing and must not drive blindly."""
    blind = ScanData(stamp=0.0, angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                     ranges=[float('nan')] * 360, range_min=0.1, range_max=12.0)
    result = LocalPlanner().plan(blind, GOAL_AHEAD)
    assert result.scan.count == 0
    assert result.command.v == 0.0 and result.command.w == 0.0
    assert result.debug.state == 'NO_DATA'


def test_wall_ahead_makes_the_robot_turn_rather_than_approach():
    result = steady(LocalPlanner(), walls=[(1.2, -1.5, 1.2, 1.5)])
    assert abs(result.command.w) > 0.1, 'expected a turn away from the wall'
    assert result.command.v < NavConfig().velocity.nominal_speed


def test_obstacle_ahead_is_steered_around_while_slowing():
    result = steady(LocalPlanner(), circles=[(0.9, 0.0, 0.25)])
    assert result.assessment.level in ('CAUTION', 'DANGER')
    assert abs(result.command.w) > 0.1
    assert 0.0 < result.command.v < NavConfig().velocity.nominal_speed


def test_critically_close_obstacle_stops_the_robot():
    result = steady(LocalPlanner(), circles=[(0.55, 0.0, 0.2)])
    assert result.assessment.level == 'CRITICAL'
    assert result.command.v == 0.0
    assert result.command.w == 0.0
    assert 'critical' in result.command.reason.lower()


def test_gap_between_obstacles_is_used_rather_than_avoided():
    result = steady(LocalPlanner(), circles=[(1.5, 0.9, 0.3), (1.5, -0.9, 0.3)])
    assert result.command.v > 0.0
    assert abs(result.command.w) < 0.2, 'should drive through the gap, not swerve'


def test_goal_to_the_side_turns_toward_it():
    left = steady(LocalPlanner(), goal=(2.0, 2.0), walls=corridor(3.0))
    right = steady(LocalPlanner(), goal=(2.0, -2.0), walls=corridor(3.0))
    assert left.command.w > 0.0
    assert right.command.w < 0.0


def test_speed_falls_as_an_obstacle_gets_closer():
    speeds = []
    for distance in (3.0, 2.0, 1.4, 1.0):
        planner = LocalPlanner()
        result = steady(planner, circles=[(distance, 0.0, 0.25)])
        speeds.append(result.command.v)
    assert speeds == sorted(speeds, reverse=True), speeds


def test_every_issued_command_respects_the_speed_limit():
    planner = LocalPlanner()
    for k in range(20):
        scan = make_scan(stamp=k * 0.1, walls=room(-4, -3, 4, 3),
                         circles=[(1.5, 0.3, 0.3)])
        result = planner.plan(scan, GOAL_AHEAD, robot_velocity=planner.last_command)
        assert result.command.v <= result.speed_limit + 1e-9
        assert abs(result.command.w) <= NavConfig().robot.max_angular_speed + 1e-9


def test_chosen_motion_is_always_collision_free_under_the_footprint():
    """Whatever is commanded, its rollout must clear the measured scan points."""
    planner = LocalPlanner()
    for k in range(20):
        scan = make_scan(stamp=k * 0.1, walls=room(-3, -2.5, 3, 2.5),
                         circles=[(1.2, 0.2, 0.3), (2.0, -0.8, 0.25)])
        result = planner.plan(scan, GOAL_AHEAD, robot_velocity=planner.last_command)
        if result.best is None:
            continue
        footprint = planner.footprint_min if result.degraded_margin else planner.footprint
        points = planner._static_points(result.scan, planner._reach_radius())
        assert footprint.path_clear_of_points(result.best.poses, points), \
            'commanded {} was not collision free'.format(result.best.name)


def test_reported_option_matches_the_command():
    """The log must never say "forward" while the robot is commanded to stop."""
    planner = LocalPlanner()
    for distance in (3.0, 1.5, 0.9, 0.55):
        result = steady(planner, circles=[(distance, 0.0, 0.25)])
        if result.best is None:
            assert result.debug.chosen == 'stop'
            assert result.command.v == 0.0 and result.command.w == 0.0
        else:
            assert result.debug.chosen == result.best.name
            assert (result.command.v, result.command.w) == (result.best.v, result.best.w)


def test_degraded_margin_is_used_before_giving_up_in_tight_spaces():
    """A narrow corridor must not deadlock the robot over a comfort margin."""
    cfg = NavConfig()
    half_width = cfg.robot.half_width + cfg.robot.footprint_inflation + 0.02
    planner = LocalPlanner(cfg)
    result = steady(planner, walls=corridor(half_width), cycles=3)
    assert result.best is not None or result.assessment.level == 'CRITICAL'


def test_moving_obstacle_is_predicted_not_just_observed():
    """A crosser that will be in the way must influence the decision early."""
    planner = LocalPlanner()
    result = None
    for k in range(25):
        t = k * 0.1
        crosser = (2.0, -1.2 + 0.7 * t, 0.25)
        scan = make_scan(stamp=t, walls=corridor(3.0), circles=[crosser])
        result = planner.plan(scan, GOAL_AHEAD, robot_velocity=planner.last_command)
    assert result.debug.n_moving >= 1, 'crossing obstacle was never seen as moving'


def test_planner_reset_clears_state():
    planner = LocalPlanner()
    steady(planner, circles=[(1.0, 0.0, 0.25)])
    planner.reset()
    assert planner.last_command == (0.0, 0.0)
    assert planner.tracker.tracks == {}


def test_cycle_time_is_suitable_for_ten_hertz_control():
    """Budget check on the development machine; the robot is slower, so the
    margin matters. Measured, not assumed."""
    planner = LocalPlanner()
    worst = 0.0
    for k in range(25):
        scan = make_scan(stamp=k * 0.1, walls=room(-4, -3, 4, 3),
                         circles=[(1.5, 0.3, 0.3), (2.5, -1.0, 0.25)])
        result = planner.plan(scan, GOAL_AHEAD, robot_velocity=planner.last_command)
        worst = max(worst, result.debug.compute_time)
    assert worst < 0.030, 'worst cycle {:.1f} ms'.format(worst * 1000)
