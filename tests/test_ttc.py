"""Unit tests for time-to-collision."""

import math

import pytest

from beetlebot_risk_nav.core.ttc import braking_distance, disc_ttc, minimum_ttc, track_ttc
from beetlebot_risk_nav.core.types import Track


def test_head_on_closure():
    assert disc_ttc(2.0, 0.0, -1.0, 0.0, 0.5) == pytest.approx(1.5)


def test_already_overlapping_is_zero():
    assert disc_ttc(0.2, 0.0, -1.0, 0.0, 0.5) == 0.0


def test_separating_never_collides():
    assert math.isinf(disc_ttc(2.0, 0.0, 1.0, 0.0, 0.5))


def test_no_relative_motion_never_collides():
    assert math.isinf(disc_ttc(2.0, 0.0, 0.0, 0.0, 0.5))


def test_passing_wide_never_collides():
    assert math.isinf(disc_ttc(2.0, 3.0, -1.0, 0.0, 0.5))


def test_grazing_contact_is_detected():
    ttc = disc_ttc(2.0, 0.4, -1.0, 0.0, 0.5)
    assert 1.0 < ttc < 2.0


def test_ttc_scales_inversely_with_closing_speed():
    slow = disc_ttc(3.0, 0.0, -0.5, 0.0, 0.5)
    fast = disc_ttc(3.0, 0.0, -1.0, 0.0, 0.5)
    assert slow == pytest.approx(2.0 * fast)


def test_track_ttc_uses_relative_velocity():
    track = Track(1, x=0, y=0, rel_x=2.0, rel_y=0.0, rel_vx=-1.0, rel_vy=0.0,
                  radius=0.2, velocity_valid=True, is_moving=True)
    assert track_ttc(track, 0.3) == pytest.approx(1.5)


def test_minimum_ttc_picks_the_most_urgent_track():
    near = Track(1, x=0, y=0, rel_x=1.5, rel_y=0.0, rel_vx=-1.0, radius=0.1,
                 velocity_valid=True, is_moving=True)
    far = Track(2, x=0, y=0, rel_x=5.0, rel_y=0.0, rel_vx=-1.0, radius=0.1,
                velocity_valid=True, is_moving=True)
    ttc, track_id = minimum_ttc([far, near], 0.3)
    assert track_id == 1
    assert ttc == pytest.approx(track_ttc(near, 0.3))


def test_braking_distance_is_the_textbook_value():
    assert braking_distance(0.22, 0.6) == pytest.approx(0.22 ** 2 / 1.2)
    assert braking_distance(0.0, 0.6) == 0.0
