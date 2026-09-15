"""Faithful re-implementation of the original BeetleBot obstacle-avoidance logic.

The original ROS node is preserved byte-for-byte at
``baseline/lyra_control/obstacle_avoidance.py`` in the repository root and is
never modified. This module reproduces its *decision logic* without ROS so that
the baseline can be run in the simulator and compared against the new planner on
identical scenarios.

Differences from the original, and why:

* Sectors are selected by **angle** rather than by array index. The original
  assumes index 0 is straight ahead and slices ``ranges[-n/12:] + ranges[:n/12]``;
  doing it by angle is behaviourally identical for that layout but also correct
  for a scan that starts at -pi, which is what the simulator produces.
* ``time.time()`` is replaced by the timestamp supplied by the caller, so runs
  are deterministic and reproducible.

Thresholds, speeds, the reverse-then-turn state machine and the
"turn toward whichever side is clearer" rule are unchanged.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from ..core.geometry import normalize_angle


class BaselineAvoidanceController:
    """The preserved baseline: reactive obstacle avoidance with no goal."""

    def __init__(self, safe_distance: float = 0.5, front_distance: float = 0.45,
                 move_speed: float = 0.2, reverse_speed: float = 0.12,
                 turn_speed: float = 0.4, reverse_time: float = 0.8) -> None:
        self.safe_distance = safe_distance
        self.front_distance = front_distance
        self.move_speed = move_speed
        self.reverse_speed = reverse_speed
        self.turn_speed = turn_speed
        self.reverse_time = reverse_time
        self.reversing = False
        self.reverse_start_time: Optional[float] = None
        self.name = 'baseline'

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.reversing = False
        self.reverse_start_time = None

    # ------------------------------------------------------------------
    @staticmethod
    def _sector_min(scan, low: float, high: float) -> float:
        """Minimum valid range with a bearing in ``[low, high]`` radians."""
        best = float('inf')
        for i, raw in enumerate(scan.ranges):
            r = float(raw)
            if r != r or math.isinf(r) or r <= 0.0:
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = normalize_angle(scan.angle_min + i * scan.angle_increment)
            if low <= angle <= high and r < best:
                best = r
        return best

    def sectors(self, scan) -> Tuple[float, float, float]:
        """``(front, left, right)`` minima, matching the original's windows."""
        front = min(self._sector_min(scan, -math.pi / 12.0, math.pi / 12.0),
                    self._sector_min(scan, 2.0 * math.pi - math.pi / 12.0, 2.0 * math.pi))
        left = self._sector_min(scan, math.pi / 12.0, math.pi / 3.0)
        right = self._sector_min(scan, -math.pi / 3.0, -math.pi / 12.0)
        return (front, left, right)

    # ------------------------------------------------------------------
    def control(self, obs) -> Tuple[float, float]:
        scan = obs.scan
        now = obs.stamp
        front, left, right = self.sectors(scan)

        if math.isinf(front) and math.isinf(left) and math.isinf(right):
            return (0.0, 0.0)               # "No valid laser readings. Stopping."

        if self.reversing:
            elapsed = now - (self.reverse_start_time or now)
            if elapsed < self.reverse_time:
                return (-self.reverse_speed, 0.0)
            self.reversing = False
            if left > right:
                return (0.0, self.turn_speed)
            return (0.0, -self.turn_speed)

        if front < self.front_distance:
            self.reversing = True
            self.reverse_start_time = now
            return (-self.reverse_speed, 0.0)

        if front < self.safe_distance:
            if left > right:
                return (0.0, self.turn_speed)
            return (0.0, -self.turn_speed)

        return (self.move_speed, 0.0)


class BaselineGoalController(BaselineAvoidanceController):
    """Baseline avoidance plus minimal goal seeking.

    The original controller has no notion of a goal: with a clear front it always
    drives straight. Comparing it against a goal-directed planner on a
    "did you reach the goal" metric would therefore be meaningless. This variant
    keeps the baseline's avoidance logic *exactly* and only changes what it does
    when the front is clear - it steers toward the goal with a proportional
    controller instead of driving blindly forward. That isolates the quality of
    the avoidance behaviour, which is what the comparison is about.
    """

    def __init__(self, *args, heading_gain: float = 1.2, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.heading_gain = heading_gain
        self.name = 'baseline+goal'

    def control(self, obs) -> Tuple[float, float]:
        scan = obs.scan
        front, left, right = self.sectors(scan)

        if math.isinf(front) and math.isinf(left) and math.isinf(right):
            return (0.0, 0.0)

        if self.reversing:
            elapsed = obs.stamp - (self.reverse_start_time or obs.stamp)
            if elapsed < self.reverse_time:
                return (-self.reverse_speed, 0.0)
            self.reversing = False
            return (0.0, self.turn_speed if left > right else -self.turn_speed)

        if front < self.front_distance:
            self.reversing = True
            self.reverse_start_time = obs.stamp
            return (-self.reverse_speed, 0.0)

        if front < self.safe_distance:
            return (0.0, self.turn_speed if left > right else -self.turn_speed)

        # Front clear: head for the goal.
        gx, gy = obs.goal
        x, y, th = obs.pose
        heading_error = normalize_angle(math.atan2(gy - y, gx - x) - th)
        w = max(-self.turn_speed, min(self.turn_speed, self.heading_gain * heading_error))
        v = self.move_speed if abs(heading_error) < math.pi / 4.0 else 0.0
        return (v, w)
