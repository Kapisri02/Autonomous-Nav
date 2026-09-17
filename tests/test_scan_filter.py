"""Unit tests for LiDAR filtering, including its safety-biased behaviour."""

import math

import pytest

from beetlebot_risk_nav.core.params import LidarConfig
from beetlebot_risk_nav.core.scan_filter import LidarFilter, sector_minimum
from beetlebot_risk_nav.core.types import ScanData
from synthetic import make_scan


def scan_from(ranges, range_min=0.1, range_max=12.0):
    n = max(1, len(ranges))
    return ScanData(stamp=0.0, angle_min=-math.pi, angle_increment=2 * math.pi / n,
                    ranges=ranges, range_min=range_min, range_max=range_max)


def test_empty_scan_is_handled():
    result = LidarFilter().filter(scan_from([]))
    assert result.count == 0 and result.raw_count == 0
    assert result.min_range() == float('inf')


def test_invalid_readings_are_rejected_and_counted():
    ranges = [2.0] * 36
    ranges[0] = float('nan')
    ranges[1] = float('inf')
    ranges[2] = 0.01          # below range_min
    ranges[3] = 99.0          # beyond range_max
    result = LidarFilter().filter(scan_from(ranges))
    assert result.rejected['nan'] == 1
    assert result.rejected['inf'] == 1
    assert result.rejected['below_min'] == 1
    assert result.rejected['above_max'] == 1
    assert result.count == 32


def test_all_invalid_scan_yields_no_points():
    result = LidarFilter().filter(scan_from([float('nan')] * 36))
    assert result.count == 0
    assert result.valid_fraction == 0.0


def test_median_filter_never_pushes_an_obstacle_further_away():
    """A close return must survive smoothing even when its neighbours are far.

    Filtering may only ever pull a reading closer; if smoothing could move a
    reading outward it would be able to hide an obstacle, which is the one
    failure mode a safety-critical filter must not have.
    """
    ranges = [3.0] * 36
    ranges[18] = 0.40
    result = LidarFilter().filter(scan_from(ranges))
    # Ranges are reported in the chassis frame, so a return 0.40 m ahead of the
    # sensor lies 0.40 + mount_offset_x from the centre of rotation.
    assert result.min_range() == pytest.approx(0.40 + LidarConfig().mount_offset_x)


def test_close_lone_returns_are_never_discarded_as_speckle():
    cfg = LidarConfig()
    ranges = [3.0] * 36
    ranges[18] = cfg.speckle_min_range - 0.05
    result = LidarFilter().filter(scan_from(ranges))
    assert result.min_range() == pytest.approx(
        cfg.speckle_min_range - 0.05 + cfg.mount_offset_x)


def test_coarse_scan_is_not_entirely_discarded():
    """A low-resolution or decimated scan must still produce points.

    With a fixed support radius, a 36-beam scan puts neighbouring returns 0.35 m
    apart at 2 m, every point looks isolated, and the filter throws the whole
    scan away - the robot goes blind without any error.
    """
    result = LidarFilter().filter(scan_from([2.0] * 36))
    assert result.count == 36
    assert result.rejected['speckle'] == 0


def test_isolated_far_return_is_rejected_as_speckle():
    """At range, a genuine object spans several beams; one lone beam is noise."""
    ranges = [3.0] * 360
    ranges[180] = 1.2
    result = LidarFilter().filter(scan_from(ranges))
    assert result.rejected['speckle'] >= 1
    # Nearest surviving return is the background behind the robot, which the
    # forward sensor offset brings closer to the chassis centre.
    assert result.min_range() == pytest.approx(3.0 - LidarConfig().mount_offset_x)


def test_points_are_consistent_with_ranges_and_angles():
    result = LidarFilter().filter(make_scan(circles=[(1.5, 0.0, 0.3)]))
    for (x, y), rng, ang in zip(result.points, result.ranges, result.angles):
        assert math.hypot(x, y) == pytest.approx(rng, abs=1e-9)
        assert math.atan2(y, x) == pytest.approx(ang, abs=1e-9)


def test_beyond_usable_range_is_dropped():
    cfg = LidarConfig()
    result = LidarFilter(cfg).filter(scan_from([cfg.max_usable_range + 1.0] * 36))
    assert result.count == 0
    assert result.rejected['above_max'] == 36


def test_field_of_view_restriction():
    cfg = LidarConfig()
    cfg.field_of_view = math.pi          # +/- 90 degrees
    result = LidarFilter(cfg).filter(scan_from([2.0] * 360))
    assert result.rejected['out_of_fov'] > 0
    assert all(abs(a) <= math.pi / 2 + 1e-9 for a in result.angles)


def test_decimation_reduces_point_count():
    cfg = LidarConfig()
    cfg.decimation = 3
    dense = LidarFilter().filter(scan_from([2.0] * 360)).count
    sparse = LidarFilter(cfg).filter(scan_from([2.0] * 360)).count
    assert sparse < dense


def test_points_are_translated_into_the_chassis_frame():
    """A return ahead of the sensor is further from the chassis centre.

    The LiDAR sits 0.085 m forward of the centre of rotation, so a reading of
    1.000 m straight ahead is 1.085 m from the chassis centre and a reading
    directly behind is 0.915 m.
    """
    cfg = LidarConfig()
    result = LidarFilter(cfg).filter(scan_from([1.0] * 360))
    ahead = min(r for a, r in zip(result.angles, result.ranges) if abs(a) < 0.02)
    behind = min(r for a, r in zip(result.angles, result.ranges)
                 if abs(abs(a) - math.pi) < 0.02)
    assert ahead == pytest.approx(1.0 + cfg.mount_offset_x, abs=1e-3)
    assert behind == pytest.approx(1.0 - cfg.mount_offset_x, abs=1e-3)


def test_sector_minimum_is_directional():
    scan = make_scan(circles=[(1.0, 0.0, 0.2)], background=6.0)
    filtered = LidarFilter().filter(scan)
    ahead = sector_minimum(filtered, 0.0, math.radians(20))
    behind = sector_minimum(filtered, math.pi, math.radians(20))
    assert ahead == pytest.approx(0.8 + LidarConfig().mount_offset_x, abs=0.05)
    assert behind > 3.0
