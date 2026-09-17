"""Unit tests for obstacle detection (clustering)."""

import math

import pytest

from beetlebot_risk_nav.core.clustering import ObstacleDetector, obstacle_summary
from beetlebot_risk_nav.core.params import ClusterConfig
from beetlebot_risk_nav.core.scan_filter import LidarFilter
from synthetic import make_scan, room


def detect(**scan_kwargs):
    filtered = LidarFilter().filter(make_scan(**scan_kwargs))
    return ObstacleDetector().detect(filtered), filtered


def test_no_points_yields_no_obstacles():
    obstacles, _ = detect()
    assert obstacles == []
    assert obstacle_summary(obstacles) == (0, float('inf'))


def test_two_separated_objects_appear_at_two_distinct_bearings():
    """One object may become several small discs; the two must stay separate.

    Clusters are capped at max_cluster_radius so a single object spanning half a
    metre is legitimately covered by more than one disc. What matters is that no
    disc bridges the empty space between the two objects.
    """
    obstacles, _ = detect(circles=[(1.5, 0.0, 0.2), (0.0, 1.5, 0.2)])
    assert len(obstacles) >= 2
    bearings = [math.degrees(o.bearing) for o in obstacles]
    ahead = [b for b in bearings if abs(b) < 30]
    left = [b for b in bearings if abs(b - 90) < 30]
    assert ahead and left
    assert len(ahead) + len(left) == len(bearings)


def test_cluster_discs_stay_small_enough_to_hug_surfaces():
    """A disc around a whole wall would swallow the free space beside it."""
    cfg = ClusterConfig()
    obstacles, _ = detect(walls=[(2.0, -3.0, 2.0, 3.0)])
    assert obstacles
    assert max(o.radius for o in obstacles) <= cfg.max_cluster_radius + 1e-6


def test_wall_is_flagged_as_structure_and_small_object_is_not():
    wall, _ = detect(walls=[(2.0, -3.0, 2.0, 3.0)])
    assert wall and all(o.is_structure for o in wall)
    obj, _ = detect(circles=[(1.5, 0.0, 0.15)])
    assert obj and not any(o.is_structure for o in obj)


def test_min_distance_is_the_nearest_surface_point():
    from beetlebot_risk_nav.core.params import LidarConfig
    obstacles, _ = detect(circles=[(2.0, 0.0, 0.3)])
    assert obstacles
    # Measured from the chassis centre, which sits behind the sensor.
    expected = 1.7 + LidarConfig().mount_offset_x
    assert min(o.min_distance for o in obstacles) == pytest.approx(expected, abs=0.02)


def test_max_obstacles_keeps_the_nearest():
    cfg = ClusterConfig()
    cfg.max_obstacles = 3
    filtered = LidarFilter().filter(make_scan(walls=room(-4, -3, 4, 3)))
    obstacles = ObstacleDetector(cfg).detect(filtered)
    assert len(obstacles) == 3
    all_obstacles = ObstacleDetector().detect(filtered)
    nearest = sorted(o.min_distance for o in all_obstacles)[:3]
    assert sorted(o.min_distance for o in obstacles) == pytest.approx(nearest)


def test_every_point_belongs_to_some_cluster_for_a_simple_object():
    obstacles, filtered = detect(circles=[(1.5, 0.0, 0.25)])
    assert sum(len(o.points) for o in obstacles) == filtered.count


def test_range_truncated_clusters_are_treated_as_structure():
    """A cluster reaching the usable range is cut off, not small.

    Its visible extent says nothing about its true size, and its centroid
    migrates rapidly as the range gate sweeps along the surface. Left
    unflagged, a wall at the edge of sensor range was classified as a moving
    object at 0.75 m/s.
    """
    from beetlebot_risk_nav.core.params import LidarConfig
    cfg = LidarConfig()
    lidar = LidarFilter(cfg)
    detector = ObstacleDetector()
    # A short wall segment placed just inside the usable range.
    scan = make_scan(walls=[(cfg.max_usable_range - 0.08, -0.30,
                             cfg.max_usable_range - 0.08, 0.30)])
    filtered = lidar.filter(scan)
    plain = detector.detect(filtered)
    flagged = detector.detect(filtered, truncation_range=cfg.max_usable_range,
                              sensor_origin=(cfg.mount_offset_x, cfg.mount_offset_y))
    assert plain and flagged
    assert not any(o.is_structure for o in plain), 'extent alone should not flag it'
    assert all(o.is_structure for o in flagged), 'truncated cluster must be structure'


def test_truncation_test_is_applied_in_the_sensor_frame():
    """Behind the robot the range gate bites sooner, because the sensor is forward.

    Comparing chassis-frame ranges against a constant offset leaves rear
    clusters unflagged, which is precisely where the gate acts first.
    """
    from beetlebot_risk_nav.core.params import LidarConfig
    cfg = LidarConfig()
    detector = ObstacleDetector()
    # A wall astern, at a chassis-frame range inside the gate but a
    # sensor-frame range at it.
    behind = -(cfg.max_usable_range - cfg.mount_offset_x - 0.02)
    scan = make_scan(walls=[(behind, -0.30, behind, 0.30)])
    filtered = LidarFilter(cfg).filter(scan)
    flagged = detector.detect(filtered, truncation_range=cfg.max_usable_range,
                              sensor_origin=(cfg.mount_offset_x, cfg.mount_offset_y))
    assert flagged
    assert all(o.is_structure for o in flagged)
