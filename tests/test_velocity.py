"""Unit tests for the risk-adaptive speed envelope."""

import pytest

from beetlebot_risk_nav.core.params import RobotConfig, VelocityConfig
from beetlebot_risk_nav.core.velocity import AdaptiveVelocity


def fresh():
    return AdaptiveVelocity(VelocityConfig(), RobotConfig())


def test_clear_world_allows_the_nominal_speed():
    velocity = fresh()
    limit = 0.0
    for _ in range(20):                       # allow the upward filter to settle
        limit = velocity.speed_limit(1.0, float('inf'))
    assert limit == pytest.approx(VelocityConfig().nominal_speed)


def test_critical_risk_forces_zero():
    assert fresh().speed_limit(0.0, 5.0) == 0.0


def test_speed_scale_is_applied():
    velocity = fresh()
    for _ in range(20):
        limit = velocity.speed_limit(0.6, float('inf'))
    assert limit == pytest.approx(0.6 * VelocityConfig().nominal_speed, rel=0.05)


def test_braking_cap_guarantees_the_robot_can_stop_in_the_space_it_sees():
    """The central safety property: v <= sqrt(2 a (clearance - margin))."""
    cfg = VelocityConfig()
    robot = RobotConfig()
    velocity = AdaptiveVelocity(cfg, robot)
    for clearance in (0.15, 0.3, 0.6, 1.0, 2.0):
        cap = velocity.braking_cap(clearance)
        stopping_distance = cap * cap / (2.0 * robot.max_linear_decel)
        assert stopping_distance <= max(0.0, clearance - cfg.brake_margin) + 1e-9


def test_no_usable_space_means_no_forward_motion():
    velocity = fresh()
    assert velocity.braking_cap(VelocityConfig().brake_margin) == 0.0
    assert velocity.speed_limit(1.0, 0.0) == 0.0


def test_slowing_down_is_immediate_but_speeding_up_is_filtered():
    """Asymmetric filtering: never sluggish about braking."""
    velocity = fresh()
    for _ in range(20):
        velocity.speed_limit(1.0, float('inf'))
    high = velocity.last_limit
    dropped = velocity.speed_limit(0.3, float('inf'))
    assert dropped < high * 0.5                     # took effect at once

    velocity.reset()
    velocity.speed_limit(0.0, 0.0)                  # forced to a standstill
    first = velocity.speed_limit(1.0, float('inf'))
    assert first < VelocityConfig().nominal_speed   # ramps back up gradually


def test_acceleration_cap_limits_change_per_cycle():
    velocity = fresh()
    velocity.reset()
    limit = velocity.speed_limit(1.0, float('inf'), current_speed=0.0, dt=0.1)
    assert limit <= RobotConfig().max_linear_accel * 0.1 + 1e-9


def test_limit_never_exceeds_the_robot_maximum():
    cfg = VelocityConfig()
    cfg.nominal_speed = 10.0
    velocity = AdaptiveVelocity(cfg, RobotConfig())
    for _ in range(50):
        limit = velocity.speed_limit(1.0, float('inf'))
    assert limit <= RobotConfig().max_linear_speed + 1e-9
