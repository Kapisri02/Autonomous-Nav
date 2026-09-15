"""Unit tests for the 2D geometry primitives."""

import math

import pytest

from beetlebot_risk_nav.core import geometry as g


def test_normalize_angle_wraps_to_half_open_interval():
    assert g.normalize_angle(0.0) == pytest.approx(0.0)
    assert g.normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert g.normalize_angle(-3 * math.pi) == pytest.approx(math.pi)
    assert g.normalize_angle(2 * math.pi + 0.5) == pytest.approx(0.5)
    for raw in (-20.0, -7.3, -0.1, 0.0, 1.0, 12.5, 99.9):
        assert -math.pi < g.normalize_angle(raw) <= math.pi + 1e-12


def test_angle_diff_takes_the_short_way_round():
    assert g.angle_diff(math.radians(170), math.radians(-170)) == pytest.approx(
        math.radians(-20), abs=1e-9)


def test_clamp_handles_inverted_bounds():
    assert g.clamp(5.0, 0.0, 1.0) == 1.0
    assert g.clamp(-5.0, 0.0, 1.0) == 0.0
    assert g.clamp(0.5, 1.0, 0.0) == 0.5


def test_transform_and_inverse_are_inverses():
    pose = (1.5, -2.0, 0.7)
    point = (3.0, 4.0)
    back = g.inverse_transform_point(g.transform_point(point, pose), pose)
    assert back[0] == pytest.approx(point[0])
    assert back[1] == pytest.approx(point[1])


def test_arc_pose_straight_line():
    x, y, th = g.arc_pose((0.0, 0.0, 0.0), 0.5, 0.0, 2.0)
    assert (x, y, th) == pytest.approx((1.0, 0.0, 0.0))


def test_arc_pose_pure_rotation_does_not_translate():
    x, y, th = g.arc_pose((1.0, 2.0, 0.0), 0.0, 1.0, 0.5)
    assert (x, y) == pytest.approx((1.0, 2.0))
    assert th == pytest.approx(0.5)


def test_arc_pose_quarter_circle():
    """v/w = 1 m radius; a quarter turn must land at (1, 1) facing +y."""
    x, y, th = g.arc_pose((0.0, 0.0, 0.0), 1.0, 1.0, math.pi / 2)
    assert (x, y) == pytest.approx((1.0, 1.0), abs=1e-9)
    assert th == pytest.approx(math.pi / 2)


def test_arc_pose_matches_fine_euler_integration():
    pose_exact = g.arc_pose((0.0, 0.0, 0.0), 0.2, 0.6, 1.5)
    pose = (0.0, 0.0, 0.0)
    for _ in range(15000):
        pose = g.arc_pose(pose, 0.2, 0.6, 1.5 / 15000)
    assert pose_exact[0] == pytest.approx(pose[0], abs=1e-6)
    assert pose_exact[1] == pytest.approx(pose[1], abs=1e-6)


def test_point_segment_distance():
    assert g.point_segment_distance((0, 1), (-1, 0), (1, 0)) == pytest.approx(1.0)
    assert g.point_segment_distance((2, 0), (-1, 0), (1, 0)) == pytest.approx(1.0)
    assert g.point_segment_distance((0, 0), (0, 0), (0, 0)) == pytest.approx(0.0)


def test_polygon_point_distance_sign():
    square = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    assert g.polygon_point_distance(square, (0, 0)) == pytest.approx(-1.0)
    assert g.polygon_point_distance(square, (2, 0)) == pytest.approx(1.0)


def test_yaw_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 2.9):
        x, y, z, w = g.quaternion_from_yaw(yaw)
        assert g.yaw_from_quaternion(x, y, z, w) == pytest.approx(yaw)


def test_path_length():
    assert g.path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7.0)
    assert g.path_length([]) == 0.0
