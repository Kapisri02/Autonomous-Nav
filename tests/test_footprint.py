"""Unit tests for footprint-aware collision checking.

The fast inlined rectangle distance is checked against an independent polygon
routine, because it is the single piece of maths that stands between the planner
and a collision.
"""

import math
import random

import pytest

from beetlebot_risk_nav.core.footprint import FootprintChecker
from beetlebot_risk_nav.core.geometry import (inverse_transform_point,
                                              polygon_point_distance)
from beetlebot_risk_nav.core.params import RobotConfig


def _footprint_centre(pose, robot):
    """World position of the footprint centre for a given robot pose."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    ox, oy = robot.footprint_offset_x, robot.footprint_offset_y
    return (x + c * ox - s * oy, y + s * ox + c * oy)


def test_footprint_is_centred_on_the_rotation_centre():
    """The footprint stays at the origin of the planning frame.

    The sensor offset is applied to the scan points instead (LidarConfig.
    mount_offset_x), so the footprint, the obstacle set and the rollout all
    share the chassis frame. Displacing the footprint here would instead model
    the robot as rotating about its LiDAR, which it does not.
    """
    from beetlebot_risk_nav.core.params import LidarConfig
    robot = RobotConfig()
    assert robot.footprint_offset_x == pytest.approx(0.0)
    assert LidarConfig().mount_offset_x == pytest.approx(0.085)


def test_rectangular_distance_matches_polygon_reference():
    checker = FootprintChecker(RobotConfig())
    rng = random.Random(1234)
    for _ in range(5000):
        pose = (rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
        point = (rng.uniform(-3, 3), rng.uniform(-3, 3))
        fast = checker.distance_to_point(pose, point)
        reference = polygon_point_distance(checker.polygon,
                                           inverse_transform_point(point, pose))
        assert fast == pytest.approx(reference, abs=1e-12)


def test_circular_distance_matches_analytic_circle():
    """The disc is centred on the chassis centre, not on the scan origin.

    The LiDAR sits forward of the chassis centre, so the footprint - circular or
    rectangular - is displaced by robot.footprint_offset_x in the scan frame.
    The analytic expectation has to account for that displacement, otherwise it
    silently assumes a sensor at the centre of the robot.
    """
    robot = RobotConfig()
    robot.use_circular_footprint = True
    checker = FootprintChecker(robot)
    radius = robot.robot_radius + robot.footprint_inflation
    rng = random.Random(99)
    for _ in range(5000):
        pose = (rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
        point = (rng.uniform(-3, 3), rng.uniform(-3, 3))
        centre = _footprint_centre(pose, robot)
        expected = math.hypot(point[0] - centre[0], point[1] - centre[1]) - radius
        assert checker.distance_to_point(pose, point) == pytest.approx(expected, abs=1e-12)


def test_distance_accounts_for_the_disc_radius():
    checker = FootprintChecker(RobotConfig())
    bare = checker.distance_to_point((0, 0, 0), (1.0, 0.0))
    with_radius = checker.distance_to_disc((0, 0, 0), (1.0, 0.0, 0.25))
    assert with_radius == pytest.approx(bare - 0.25)


def test_penetration_is_negative_and_detected():
    checker = FootprintChecker(RobotConfig())
    inside = (0.05, 0.0)
    assert checker.distance_to_point((0, 0, 0), inside) < 0.0
    assert checker.collides((0, 0, 0), [(inside[0], inside[1], 0.0)])


def test_rotation_matters_for_a_non_circular_robot():
    """A point that clears the robot's side can be hit by its corner when turned.

    This is the whole reason the footprint is not modelled as a disc.
    """
    robot = RobotConfig()
    checker = FootprintChecker(robot)
    # A point on the chassis axis at a radius between the inscribed and
    # circumscribed circles: beyond the flat face when the robot is square-on,
    # but inside the region the corner sweeps through once it turns. A disc
    # model of the robot cannot represent this distinction at all.
    half_l = robot.half_length + robot.footprint_inflation
    half_w = robot.half_width + robot.footprint_inflation
    radius = 0.5 * (min(half_l, half_w) + math.hypot(half_l, half_w))
    point = (robot.footprint_offset_x + radius, 0.0)
    assert checker.distance_to_point((0, 0, 0), point) > 0.0
    assert checker.distance_to_point((0, 0, math.radians(45)), point) < 0.0


def test_clearance_with_index_identifies_the_closest_obstacle():
    checker = FootprintChecker(RobotConfig())
    discs = [(3.0, 0.0, 0.1), (1.0, 0.0, 0.1), (2.0, 0.0, 0.1)]
    distance, index = checker.clearance_with_index((0, 0, 0), discs)
    assert index == 1
    assert distance == pytest.approx(checker.distance_to_disc((0, 0, 0), discs[1]))


def test_clearance_of_empty_obstacle_set_is_infinite():
    checker = FootprintChecker(RobotConfig())
    assert math.isinf(checker.clearance((0, 0, 0), []))
    assert math.isinf(checker.clearance_to_points((0, 0, 0), []))


def test_inflation_reduces_clearance_by_exactly_the_margin():
    robot = RobotConfig()
    wide = FootprintChecker(robot, inflation=0.10)
    narrow = FootprintChecker(robot, inflation=0.02)
    point = (1.0, 0.0)
    assert narrow.distance_to_point((0, 0, 0), point) - \
        wide.distance_to_point((0, 0, 0), point) == pytest.approx(0.08)


def test_path_clear_of_points_rejects_a_rollout_that_hits_something():
    checker = FootprintChecker(RobotConfig())
    poses = [(x * 0.1, 0.0, 0.0) for x in range(12)]
    assert checker.path_clear_of_points(poses, [(5.0, 5.0)])
    assert not checker.path_clear_of_points(poses, [(1.0, 0.0)])
