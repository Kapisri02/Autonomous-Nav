"""Simple, interpretable safety assessment.

Risk is a **level**, not a score to be optimised:

    CLEAR     -> continue normally          (speed scale 1.00)
    CAUTION   -> slow down                  (speed scale 0.60)
    DANGER    -> move cautiously            (speed scale 0.30)
    CRITICAL  -> stop / pick a safer motion (speed scale 0.00)

The level is the worst verdict from two independent tests, so either one alone
can escalate:

* **clearance** - how much space is left between the footprint and the nearest
  obstacle, weighted by bearing so that an obstacle the robot is driving *away*
  from does not throttle it;
* **time to collision** - catches a fast obstacle that is still far away, which
  a pure distance test would call safe until it is too late.

Deliberately no weighted sum and no tuned coefficients: with thresholds, the
reason for every decision is legible in the log, and a surprising behaviour on
the robot can be traced to exactly one threshold.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

from .params import RiskConfig
from .ttc import track_ttc
from .types import RISK_LEVELS, RiskAssessment, Track


class RiskAssessor:
    def __init__(self, config: Optional[RiskConfig] = None) -> None:
        self.config = config or RiskConfig()

    # ------------------------------------------------------------------
    def assess(self, tracks: Sequence[Track], footprint,
               forward_clearance: float = float('inf')) -> RiskAssessment:
        """Worst-case risk over all tracks, plus the measured forward clearance.

        Clearances are the **exact footprint-to-obstacle distances** from
        ``footprint``, not centre-distance minus a bounding radius. The
        approximation double-counted the cluster radius against the robot's
        circumscribed radius and read about 13 cm pessimistic on this chassis,
        which was enough to declare CRITICAL - and stop the robot - with a
        comfortable 40 cm of real space ahead.

        ``forward_clearance`` is measured from the filtered scan rather than from
        clustering, so a clustering fault cannot make the world look safe.
        """
        cfg = self.config
        result = RiskAssessment()
        worst = 0
        max_closing = 0.0
        result.clearance = forward_clearance

        # The scan-derived forward clearance can raise the level on its own.
        level_from_scan = self._distance_level(forward_clearance)
        if level_from_scan > worst:
            worst = level_from_scan
            result.reason = 'forward clearance {:.2f} m'.format(forward_clearance)

        for track in tracks:
            clearance = max(0.0, footprint.distance_to_disc(
                (0.0, 0.0, 0.0), (track.rel_x, track.rel_y, track.radius)))
            weighted = self._weighted_clearance(clearance, track.rel_bearing)
            ttc = track_ttc(track, footprint.circumscribed)

            if track.closing_speed > max_closing:
                max_closing = track.closing_speed

            level = max(self._distance_level(weighted), self._ttc_level(ttc))
            if clearance < result.clearance:
                result.clearance = clearance
            if ttc < result.ttc:
                result.ttc = ttc
                result.closing_speed = track.closing_speed

            if level > worst:
                worst = level
                result.critical_track_id = track.track_id
                result.critical_bearing = track.rel_bearing
                result.reason = ('track {} at {:.2f} m, bearing {:+.0f} deg, '
                                 'ttc {:.2f} s{}').format(
                    track.track_id, clearance, math.degrees(track.rel_bearing),
                    ttc, ', moving' if track.is_moving else '')

        result.max_closing_speed = max_closing
        result.approaching = max_closing >= cfg.closing_speed_threshold
        result.level = RISK_LEVELS[worst]
        result.speed_scale = self.speed_scale(result.level)
        if not result.reason:
            result.reason = 'clear'
        return result

    # ------------------------------------------------------------------
    def _weighted_clearance(self, clearance: float, bearing: float) -> float:
        """Inflate the clearance of obstacles that are not ahead of the robot.

        A wall one metre behind a forward-moving robot is not a threat. The
        weight is 1.0 dead ahead falling to ``rear_weight`` behind; dividing the
        clearance by it makes rear obstacles look proportionally farther away,
        which is exactly how much less they matter.
        """
        cfg = self.config
        forward = 0.5 * (1.0 + math.cos(bearing))
        weight = cfg.rear_weight + (1.0 - cfg.rear_weight) * forward
        return clearance / max(weight, 1e-3)

    def _distance_level(self, clearance: float) -> int:
        cfg = self.config
        if clearance <= cfg.critical_distance:
            return 3
        if clearance <= cfg.danger_distance:
            return 2
        if clearance <= cfg.caution_distance:
            return 1
        return 0

    def _ttc_level(self, ttc: float) -> int:
        cfg = self.config
        if math.isinf(ttc):
            return 0
        if ttc <= cfg.critical_ttc:
            return 3
        if ttc <= cfg.danger_ttc:
            return 2
        if ttc <= cfg.caution_ttc:
            return 1
        return 0

    # ------------------------------------------------------------------
    def speed_scale(self, level: str) -> float:
        cfg = self.config
        return {
            'CLEAR': cfg.scale_clear,
            'CAUTION': cfg.scale_caution,
            'DANGER': cfg.scale_danger,
            'CRITICAL': cfg.scale_critical,
        }.get(level, 0.0)
