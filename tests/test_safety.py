"""Unit tests for the independent safety layer."""


import pytest

from beetlebot_risk_nav.core.params import SafetyConfig
from beetlebot_risk_nav.core.safety import (SAFETY_ESTOP, SAFETY_INVALID, SAFETY_OK,
                                            SAFETY_SENSOR_FAULT, SafetySupervisor)
from beetlebot_risk_nav.core.types import Command


def supervisor():
    return SafetySupervisor(SafetyConfig())


def healthy(sup, now, command, **kwargs):
    kwargs.setdefault('scan_stamp', now)
    kwargs.setdefault('odom_stamp', now)
    return sup.check(now, command, **kwargs)


def test_safe_command_passes_through():
    sup = supervisor()
    sup._last_command = (0.2, 0.0)          # already at speed
    status = healthy(sup, 1.0, Command(0.2, 0.0))
    assert status.state == SAFETY_OK
    assert status.command.as_tuple() == (0.2, 0.0)


def test_missing_lidar_stops_the_robot():
    status = supervisor().check(1.0, Command(0.2, 0.0), scan_stamp=None, odom_stamp=1.0)
    assert status.state == SAFETY_SENSOR_FAULT
    assert status.is_stopped
    assert 'no LiDAR' in status.reason


def test_stale_lidar_stops_the_robot():
    cfg = SafetyConfig()
    sup = SafetySupervisor(cfg)
    stale = 10.0 + cfg.scan_timeout + 0.01
    status = sup.check(stale, Command(0.2, 0.0), scan_stamp=10.0, odom_stamp=stale)
    assert status.state == SAFETY_SENSOR_FAULT
    assert status.is_stopped


def test_stale_odometry_stops_the_robot_when_required():
    cfg = SafetyConfig()
    sup = SafetySupervisor(cfg)
    now = 10.0 + cfg.odom_timeout + 0.01
    status = sup.check(now, Command(0.2, 0.0), scan_stamp=now, odom_stamp=10.0)
    assert status.state == SAFETY_SENSOR_FAULT


def test_odometry_can_be_made_optional():
    cfg = SafetyConfig()
    cfg.require_odom = False
    sup = SafetySupervisor(cfg)
    status = sup.check(1.0, Command(0.05, 0.0), scan_stamp=1.0, odom_stamp=None)
    assert status.state != SAFETY_SENSOR_FAULT


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_non_finite_commands_are_rejected(bad):
    for command in (Command(bad, 0.0), Command(0.0, bad)):
        status = healthy(supervisor(), 1.0, command)
        assert status.state == SAFETY_INVALID
        assert status.is_stopped


def test_emergency_stop_on_critical_proximity():
    cfg = SafetyConfig()
    status = healthy(SafetySupervisor(cfg), 1.0, Command(0.2, 0.0),
                     clearance=cfg.estop_distance - 0.01)
    assert status.state == SAFETY_ESTOP
    assert status.is_stopped and status.estop_active


def test_emergency_stop_on_imminent_collision():
    cfg = SafetyConfig()
    status = healthy(SafetySupervisor(cfg), 1.0, Command(0.2, 0.0),
                     ttc=cfg.estop_ttc - 0.01)
    assert status.state == SAFETY_ESTOP


def test_emergency_stop_latches_and_needs_hysteresis_to_release():
    """Without hysteresis the robot chatters in and out of E-stop at the limit."""
    cfg = SafetyConfig()
    sup = SafetySupervisor(cfg)
    healthy(sup, 1.0, Command(0.2, 0.0), clearance=0.05)
    assert sup.estop_active

    # Space appears immediately, but the minimum latch time has not elapsed.
    assert healthy(sup, 1.0 + cfg.estop_min_duration / 2,
                   Command(0.2, 0.0), clearance=1.0).state == SAFETY_ESTOP

    # Enough time, but not enough clearance to release.
    between = (cfg.estop_distance + cfg.estop_clear_distance) / 2
    assert healthy(sup, 1.0 + cfg.estop_min_duration + 0.1,
                   Command(0.2, 0.0), clearance=between).state == SAFETY_ESTOP

    # Both conditions satisfied: released.
    assert healthy(sup, 1.0 + cfg.estop_min_duration + 0.2, Command(0.05, 0.0),
                   clearance=cfg.estop_clear_distance + 0.1).state != SAFETY_ESTOP
    assert not sup.estop_active


def test_speed_limits_are_absolute():
    cfg = SafetyConfig()
    sup = SafetySupervisor(cfg)
    for _ in range(50):                      # let the rate limiter settle
        status = healthy(sup, 1.0, Command(99.0, 99.0))
    assert status.command.v == pytest.approx(cfg.max_linear_speed)
    assert status.command.w == pytest.approx(cfg.max_angular_speed)


def test_rate_limits_prevent_command_jumps():
    cfg = SafetyConfig()
    sup = SafetySupervisor(cfg)
    status = healthy(sup, 1.0, Command(cfg.max_linear_speed, 0.0))
    assert status.command.v <= cfg.max_linear_jump + 1e-9
    assert status.limited


def test_supervisor_never_invents_motion():
    """A stop request must always come out as a stop."""
    sup = supervisor()
    for clearance in (0.05, 0.5, 5.0):
        status = healthy(sup, 1.0, Command(0.0, 0.0), clearance=clearance)
        assert status.is_stopped


def test_reset_clears_the_latch():
    sup = supervisor()
    healthy(sup, 1.0, Command(0.2, 0.0), clearance=0.05)
    assert sup.estop_active
    sup.reset()
    assert not sup.estop_active


# ----------------------------------------------------------------------
# Escaping an emergency stop
# ----------------------------------------------------------------------
def estopped(cfg=None):
    cfg = cfg or SafetyConfig()
    sup = SafetySupervisor(cfg)
    sup.check(1.0, Command(0.2, 0.0), scan_stamp=1.0, odom_stamp=1.0, clearance=0.05)
    assert sup.estop_active
    return sup, cfg


def test_emergency_stop_permits_a_retreat_when_the_rear_is_clear():
    """An E-stop that cannot be escaped is not a safe state.

    Something set down in front of the robot would otherwise freeze it for good.
    """
    sup, cfg = estopped()
    status = sup.check(1.1, Command(-0.12, 0.0), scan_stamp=1.1, odom_stamp=1.1,
                       clearance=0.05, rear_clearance=1.0)
    assert status.state == SAFETY_ESTOP
    assert status.command.v < 0.0
    assert abs(status.command.v) <= cfg.recovery_reverse_speed + 1e-9
    assert status.command.w == 0.0


def test_emergency_stop_blocks_a_retreat_into_an_obstacle_behind():
    sup, cfg = estopped()
    status = sup.check(1.1, Command(-0.12, 0.0), scan_stamp=1.1, odom_stamp=1.1,
                       clearance=0.05, rear_clearance=cfg.recovery_rear_clearance - 0.05)
    assert status.is_stopped


def test_emergency_stop_never_permits_forward_motion():
    sup, _ = estopped()
    status = sup.check(1.1, Command(0.2, 0.0), scan_stamp=1.1, odom_stamp=1.1,
                       clearance=0.05, rear_clearance=5.0)
    assert status.is_stopped


def test_emergency_stop_never_permits_rotation():
    """A rectangular chassis sweeps its corners when it turns; under E-stop
    there is by definition not enough room to be sure that is safe."""
    sup, _ = estopped()
    status = sup.check(1.1, Command(0.0, 0.8), scan_stamp=1.1, odom_stamp=1.1,
                       clearance=0.05, rear_clearance=5.0)
    assert status.is_stopped


def test_retreat_is_not_offered_when_recovery_is_disabled():
    cfg = SafetyConfig()
    cfg.recovery_enabled = False
    sup, _ = estopped(cfg)
    status = sup.check(1.1, Command(-0.12, 0.0), scan_stamp=1.1, odom_stamp=1.1,
                       clearance=0.05, rear_clearance=5.0)
    assert status.is_stopped
