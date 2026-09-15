"""Unit tests for the four-level risk assessment."""

import math

from beetlebot_risk_nav.core.footprint import FootprintChecker
from beetlebot_risk_nav.core.params import NavConfig, RiskConfig
from beetlebot_risk_nav.core.risk import RiskAssessor
from beetlebot_risk_nav.core.types import Track

FOOTPRINT = FootprintChecker(NavConfig().robot)


def track_at(distance, bearing=0.0, closing=0.0, radius=0.2, moving=False):
    return Track(1, x=0.0, y=0.0,
                 rel_x=distance * math.cos(bearing), rel_y=distance * math.sin(bearing),
                 rel_vx=-closing * math.cos(bearing), rel_vy=-closing * math.sin(bearing),
                 radius=radius, hits=5, confidence=1.0,
                 velocity_valid=moving, is_moving=moving)


def level_of(track, forward_clearance=float('inf')):
    return RiskAssessor().assess([track], FOOTPRINT, forward_clearance).level


def test_empty_world_is_clear():
    assessment = RiskAssessor().assess([], FOOTPRINT)
    assert assessment.level == 'CLEAR'
    assert assessment.speed_scale == 1.0


def test_levels_escalate_as_an_obstacle_gets_closer():
    assert level_of(track_at(4.0)) == 'CLEAR'
    assert level_of(track_at(1.4)) == 'CAUTION'
    assert level_of(track_at(0.85)) == 'DANGER'
    assert level_of(track_at(0.55)) == 'CRITICAL'


def test_speed_scale_decreases_monotonically_with_level():
    assessor = RiskAssessor()
    scales = [assessor.speed_scale(level)
              for level in ('CLEAR', 'CAUTION', 'DANGER', 'CRITICAL')]
    assert scales == sorted(scales, reverse=True)
    assert scales[0] == 1.0 and scales[-1] == 0.0


def test_obstacle_behind_does_not_throttle_a_forward_moving_robot():
    """Regression: a wall one metre behind used to hold the robot at a crawl."""
    ahead = level_of(track_at(1.0, bearing=0.0))
    behind = level_of(track_at(1.0, bearing=math.pi))
    assert ahead in ('CAUTION', 'DANGER')
    assert behind == 'CLEAR'


def test_approaching_obstacle_escalates_by_ttc_while_still_far():
    """A distance-only test would call this safe until it was too late."""
    far_static = level_of(track_at(3.0))
    far_closing = level_of(track_at(3.0, closing=1.5, moving=True))
    assert far_static == 'CLEAR'
    assert far_closing in ('CAUTION', 'DANGER', 'CRITICAL')


def test_receding_obstacle_is_not_a_threat():
    assert level_of(track_at(3.0, closing=-1.0, moving=True)) == 'CLEAR'


def test_scan_clearance_alone_can_raise_the_level():
    """Even with no tracks, a close forward return must slow the robot down."""
    assessment = RiskAssessor().assess([], FOOTPRINT, forward_clearance=0.20)
    assert assessment.level == 'CRITICAL'
    assert 'forward clearance' in assessment.reason


def test_worst_track_determines_the_level():
    assessor = RiskAssessor()
    tracks = [track_at(5.0), track_at(0.55), track_at(3.0)]
    assessment = assessor.assess(tracks, FOOTPRINT)
    assert assessment.level == 'CRITICAL'


def test_assessment_always_explains_itself():
    assessment = RiskAssessor().assess([track_at(0.6)], FOOTPRINT)
    assert assessment.reason
    assert assessment.at_least('DANGER')


def test_thresholds_are_honoured_exactly():
    cfg = RiskConfig()
    assessor = RiskAssessor(cfg)
    just_inside = assessor.assess([], FOOTPRINT,
                                  forward_clearance=cfg.caution_distance - 0.01)
    just_outside = assessor.assess([], FOOTPRINT,
                                   forward_clearance=cfg.caution_distance + 0.01)
    assert just_inside.level == 'CAUTION'
    assert just_outside.level == 'CLEAR'
