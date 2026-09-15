"""Plain data containers shared across the navigation pipeline (ROS-free)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Pose = Tuple[float, float, float]


@dataclass
class ScanData:
    """A LiDAR scan, decoupled from ``sensor_msgs/LaserScan``."""

    stamp: float
    angle_min: float
    angle_increment: float
    ranges: Sequence[float]
    range_min: float = 0.0
    range_max: float = float('inf')
    frame_id: str = 'laser'

    def angle_at(self, index: int) -> float:
        return self.angle_min + index * self.angle_increment

    @property
    def count(self) -> int:
        return len(self.ranges)


@dataclass
class FilteredScan:
    """Validated scan points expressed in the robot base frame."""

    stamp: float
    points: List[Point] = field(default_factory=list)
    angles: List[float] = field(default_factory=list)
    ranges: List[float] = field(default_factory=list)
    raw_count: int = 0
    rejected: Dict[str, int] = field(default_factory=dict)

    @property
    def count(self) -> int:
        return len(self.points)

    @property
    def valid_fraction(self) -> float:
        return (self.count / self.raw_count) if self.raw_count else 0.0

    def min_range(self) -> float:
        return min(self.ranges) if self.ranges else float('inf')

    @property
    def informative_fraction(self) -> float:
        """Fraction of beams that told us something about the world.

        A finite reading is information, and so is an explicit no-return (the
        sensor looked and found nothing within range). A NaN is not: the sensor
        failed to measure, and that must never be read as clear space.
        """
        if not self.raw_count:
            return 0.0
        return max(0.0, self.raw_count - self.rejected.get('nan', 0)) / self.raw_count


@dataclass
class Obstacle:
    """A clustered obstacle observed in a single scan (robot base frame)."""

    center: Point
    radius: float
    points: List[Point] = field(default_factory=list)
    stamp: float = 0.0
    min_distance: float = 0.0
    #: Along-scan extent of the connected group this cluster came from.
    parent_extent: float = 0.0
    #: True when the parent group is long enough to be structure, not an object.
    is_structure: bool = False

    @property
    def distance(self) -> float:
        return math.hypot(self.center[0], self.center[1])

    @property
    def bearing(self) -> float:
        return math.atan2(self.center[1], self.center[0])


@dataclass
class Track:
    """A tracked obstacle with an estimated velocity.

    Positions and velocities are stored in the odom (world) frame so that robot
    motion is not mistaken for obstacle motion. The ``rel_*`` fields give the
    robot-frame view used by TTC and risk.
    """

    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.1
    stamp: float = 0.0
    hits: int = 1
    misses: int = 0
    age: int = 1
    confidence: float = 0.0
    velocity_valid: bool = False
    is_moving: bool = False
    is_structure: bool = False
    rel_x: float = 0.0
    rel_y: float = 0.0
    rel_vx: float = 0.0
    rel_vy: float = 0.0
    #: ``(stamp, x, y)`` world-frame history for windowed velocity estimation.
    history: List[Tuple[float, float, float]] = field(default_factory=list)

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def rel_distance(self) -> float:
        return math.hypot(self.rel_x, self.rel_y)

    @property
    def rel_bearing(self) -> float:
        return math.atan2(self.rel_y, self.rel_x)

    @property
    def closing_speed(self) -> float:
        """Rate at which the gap to the robot shrinks (positive = approaching)."""
        d = self.rel_distance
        if d < 1e-6:
            return 0.0
        return -((self.rel_x * self.rel_vx) + (self.rel_y * self.rel_vy)) / d

    def predict(self, t: float) -> Tuple[float, float]:
        """Robot-frame position ``t`` seconds from now (constant velocity).

        Only believed for genuinely moving obstacles; structure and jittery
        clusters are held at their measured position.
        """
        if not self.is_moving:
            return (self.rel_x, self.rel_y)
        return (self.rel_x + self.rel_vx * t, self.rel_y + self.rel_vy * t)


#: Risk levels, ordered from safest to most dangerous.
RISK_LEVELS = ('CLEAR', 'CAUTION', 'DANGER', 'CRITICAL')


@dataclass
class RiskAssessment:
    """Output of the risk assessment for one control cycle."""

    level: str = 'CLEAR'
    speed_scale: float = 1.0
    clearance: float = float('inf')
    ttc: float = float('inf')
    closing_speed: float = 0.0
    critical_track_id: Optional[int] = None
    critical_bearing: float = 0.0
    reason: str = ''

    @property
    def index(self) -> int:
        return RISK_LEVELS.index(self.level)

    def at_least(self, level: str) -> bool:
        return self.index >= RISK_LEVELS.index(level)


@dataclass
class MotionOption:
    """One of the small set of motion options the local planner chooses between."""

    name: str
    v: float
    w: float
    poses: List[Pose] = field(default_factory=list)
    cost: float = 0.0
    feasible: bool = True
    clearance: float = float('inf')
    headway: float = float('inf')
    collision_time: float = float('inf')
    rejected_reason: str = ''
    cost_terms: Dict[str, float] = field(default_factory=dict)


@dataclass
class Command:
    """A velocity command with the reasoning that produced it."""

    v: float = 0.0
    w: float = 0.0
    source: str = 'planner'
    reason: str = ''

    def as_tuple(self) -> Tuple[float, float]:
        return (self.v, self.w)


@dataclass
class PlannerDebug:
    """Diagnostics for one control cycle (logged, published, asserted on)."""

    stamp: float = 0.0
    n_points: int = 0
    n_obstacles: int = 0
    n_tracks: int = 0
    n_moving: int = 0
    risk_level: str = 'CLEAR'
    ttc: float = float('inf')
    clearance: float = float('inf')
    speed_limit: float = 0.0
    n_options: int = 0
    n_feasible: int = 0
    chosen: str = ''
    chosen_cost: float = 0.0
    state: str = ''
    notes: str = ''
    compute_time: float = 0.0
