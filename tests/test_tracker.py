"""Unit tests for obstacle tracking and the static/moving decision.

The moving-obstacle classification is the tracker's most safety-relevant output:
believing a wall is moving produces phantom threats that throttle the robot,
while missing a real mover removes the anticipation the system exists to provide.
"""

import math

import pytest

from beetlebot_risk_nav.core.clustering import ObstacleDetector
from beetlebot_risk_nav.core.scan_filter import LidarFilter
from beetlebot_risk_nav.core.tracker import ObstacleTracker
from beetlebot_risk_nav.core.types import Obstacle
from synthetic import make_scan, room


def feed(tracker, obstacles_over_time, pose=(0.0, 0.0, 0.0), velocity=(0.0, 0.0), dt=0.1):
    tracks = []
    for step, obstacles in enumerate(obstacles_over_time):
        tracks = tracker.update(obstacles, step * dt, pose, velocity)
    return tracks


def moving_obstacle(distance, radius=0.2, stamp=0.0):
    return Obstacle(center=(distance, 0.0), radius=radius, stamp=stamp,
                    min_distance=distance - radius)


def test_new_detection_creates_a_track():
    tracker = ObstacleTracker()
    tracks = tracker.update([moving_obstacle(2.0)], 0.0)
    assert len(tracks) == 1
    assert tracks[0].hits == 1
    assert not tracks[0].velocity_valid


def test_track_survives_brief_dropouts_then_expires():
    tracker = ObstacleTracker()
    tracker.update([moving_obstacle(2.0)], 0.0)
    for step in range(1, tracker.config.max_misses + 1):
        assert len(tracker.update([], step * 0.1)) == 1
    assert tracker.update([], 10 * 0.1) == []


def test_constant_velocity_is_recovered():
    tracker = ObstacleTracker()
    frames = [[moving_obstacle(3.0 - 0.5 * (k * 0.1), stamp=k * 0.1)] for k in range(25)]
    tracks = feed(tracker, frames)
    assert tracks[0].vx == pytest.approx(-0.5, abs=0.05)
    assert tracks[0].is_moving


def test_stationary_obstacle_is_not_classified_as_moving():
    tracker = ObstacleTracker()
    tracks = feed(tracker, [[moving_obstacle(2.0, stamp=k * 0.1)] for k in range(25)])
    assert not tracks[0].is_moving
    assert tracks[0].speed < tracker.config.moving_speed_threshold


def test_robot_motion_is_not_mistaken_for_obstacle_motion():
    """A parked obstacle must read as stationary while the robot drives at it."""
    tracker = ObstacleTracker()
    speed = 0.2
    for k in range(25):
        t = k * 0.1
        travelled = speed * t
        relative = 3.0 - travelled           # obstacle is fixed in the world
        tracks = tracker.update([moving_obstacle(relative, stamp=t)], t,
                                robot_pose=(travelled, 0.0, 0.0), robot_velocity=(speed, 0.0))
    assert tracks[0].speed < tracker.config.moving_speed_threshold
    assert not tracks[0].is_moving
    # ...but the gap is still closing, which is what risk assessment needs.
    assert tracks[0].closing_speed == pytest.approx(speed, abs=0.05)


def test_walls_never_produce_phantom_movers():
    """Regression: sliding cluster boundaries once faked velocities near 1 m/s.

    Chunks covering one long surface trade points as the robot moves, so their
    centroids drift along the wall. That drift is indistinguishable from motion
    at the level of a single cluster, so structure is excluded by extent instead.
    """
    tracker = ObstacleTracker()
    detector = ObstacleDetector()
    lidar = LidarFilter()
    speed = 0.15
    worst = 0.0
    for k in range(40):
        t = k * 0.1
        x = speed * t
        scan = make_scan(stamp=t, walls=room(-4.0 - x, -3.0, 4.0 - x, 3.0))
        tracks = tracker.update(detector.detect(lidar.filter(scan)), t,
                                robot_pose=(x, 0.0, 0.0), robot_velocity=(speed, 0.0))
        for track in tracks:
            if track.is_moving:
                worst = max(worst, track.speed)
    assert worst == 0.0, 'static structure was classified as moving at {:.2f} m/s'.format(worst)


def test_genuine_mover_is_still_detected_among_structure():
    tracker = ObstacleTracker()
    detector = ObstacleDetector()
    lidar = LidarFilter()
    best_estimate = 0.0
    detected = False
    for k in range(40):
        t = k * 0.1
        crosser = (2.0, -1.5 + 0.6 * t, 0.25)     # walks across at 0.6 m/s
        scan = make_scan(stamp=t, walls=room(-4, -3, 4, 3), circles=[crosser])
        tracks = tracker.update(detector.detect(lidar.filter(scan)), t)
        for track in tracks:
            if track.is_moving and math.hypot(track.x - crosser[0], track.y - crosser[1]) < 0.6:
                detected = True
                best_estimate = max(best_estimate, track.speed)
    assert detected, 'a 0.6 m/s crossing obstacle was never classified as moving'
    # The windowed estimator lags while it fills its history, so the converged
    # estimate is what matters, not the value at the instant of first detection.
    assert best_estimate == pytest.approx(0.6, abs=0.15)


def test_time_going_backwards_resets_velocity_estimates():
    tracker = ObstacleTracker()
    feed(tracker, [[moving_obstacle(3.0 - 0.5 * (k * 0.1), stamp=k * 0.1)] for k in range(20)])
    tracks = tracker.update([moving_obstacle(2.0)], 0.0)      # clock jumped back
    assert not tracks[0].velocity_valid
    assert tracks[0].speed == pytest.approx(0.0)


def test_reset_clears_all_state():
    tracker = ObstacleTracker()
    tracker.update([moving_obstacle(2.0)], 0.0)
    tracker.reset()
    assert tracker.tracks == {}
