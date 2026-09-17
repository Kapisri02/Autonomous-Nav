"""Configuration for BeetleBot autonomous navigation.

The canonical serialised form is a *flat dotted* dictionary
(``{'robot.max_linear_speed': 0.22, ...}``) because that is exactly how ROS 2
exposes nested YAML parameters, so the same object is built from a YAML file,
from ROS parameters, or from a test.

Robot dimensions below are the VERIFIED values for the BeetleBot, not guesses:

* ``robot_radius = 0.22 m`` - read from the robot's own ``nav2_params.yaml``.
* ``0.375 m x 0.360 m`` chassis - official VEEROBOT documentation.

The chassis is nearly square, so the rectangular footprint is used by default:
it is the accurate description, and the collision checker evaluates an exact
rectangle about as cheaply as a disc. Set ``robot.use_circular_footprint`` to
fall back to the 0.22 m disc that Nav2 itself uses.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from typing import Any, Dict, List


@dataclass
class RobotConfig:
    """Physical description and velocity limits of the BeetleBot."""

    # VERIFIED: official VEEROBOT chassis dimensions, metres.
    footprint_length: float = 0.375
    footprint_width: float = 0.360
    # VERIFIED: robot_radius from the robot's nav2_params.yaml. Used when
    # use_circular_footprint is true, and as a cross-check otherwise.
    robot_radius: float = 0.22
    use_circular_footprint: bool = False
    # Footprint centre within the planning frame. This stays at the origin:
    # scan points are translated into the chassis frame by the filter (see
    # LidarConfig.mount_offset_x), so the footprint, the rotation centre and
    # the obstacle set all share one frame. Displacing the footprint here
    # instead would make the rollout rotate the robot about the SENSOR rather
    # than about its chassis centre, which measurably degraded the ability to
    # turn past an obstacle.
    footprint_offset_x: float = 0.0
    footprint_offset_y: float = 0.0
    # Preferred safety margin around the footprint, metres.
    footprint_inflation: float = 0.06
    # Margin the planner may fall back to when no motion clears the preferred
    # margin. Prevents deadlock in tight spaces; never violated.
    footprint_min_inflation: float = 0.02

    max_linear_speed: float = 0.22       # m/s forward cap
    max_reverse_speed: float = 0.12      # m/s (matches the proven baseline)
    max_angular_speed: float = 1.0       # rad/s
    max_linear_accel: float = 0.35       # m/s^2
    max_linear_decel: float = 0.60       # m/s^2 (braking, positive)
    max_angular_accel: float = 2.0       # rad/s^2

    @property
    def half_length(self) -> float:
        return self.footprint_length * 0.5

    @property
    def half_width(self) -> float:
        return self.footprint_width * 0.5

    @property
    def circumscribed_radius(self) -> float:
        """Radius of the smallest circle containing the inflated footprint."""
        if self.use_circular_footprint:
            return self.robot_radius + self.footprint_inflation
        return math.hypot(self.half_length, self.half_width) + self.footprint_inflation

    @property
    def inscribed_radius(self) -> float:
        if self.use_circular_footprint:
            return self.robot_radius + self.footprint_inflation
        return min(self.half_length, self.half_width) + self.footprint_inflation


@dataclass
class LidarConfig:
    """LiDAR pre-filtering. Defaults suit the RPLiDAR C1 fitted to the BeetleBot."""

    min_range: float = 0.10          # m, readings closer than this are dropped
    max_range: float = 12.0          # m, RPLiDAR C1 maximum
    max_usable_range: float = 4.0    # m, beyond this points are ignored for planning
    # Speckle rejection: a point survives only if it has >= min_neighbours
    # neighbours within neighbour_radius metres among nearby beams.
    neighbour_radius: float = 0.18
    min_neighbours: int = 1
    neighbour_window: int = 2
    # The support radius also scales with the beam spacing at that range
    # (range * angle_increment * window * this factor). A fixed radius is
    # resolution-dependent: on a coarse or decimated scan the neighbouring beams
    # land further apart than the radius, every point looks isolated, and the
    # filter silently discards the entire scan - blinding the robot.
    neighbour_spacing_factor: float = 1.5
    # Median filter window (beams, odd, 1 disables). Applied in the safety-biased
    # direction only: it may pull a reading closer but never pushes one away, so
    # smoothing can never hide an obstacle.
    median_window: int = 3
    # Readings closer than this are never discarded as speckle: at close range a
    # lone return is more likely a thin obstacle than noise.
    speckle_min_range: float = 0.50
    decimation: int = 1              # keep every Nth surviving beam (CPU control)
    # Fraction of beams that must carry information (a finite reading, or an
    # explicit no-return, which means "nothing out there") before the scan is
    # trusted. Absence of data is not evidence of clear space: an empty or
    # mostly-NaN scan must stop the robot, not licence it to drive at speed.
    min_valid_fraction: float = 0.50
    # Position of the LiDAR relative to the chassis centre, metres. The
    # RPLiDAR C1 is mounted laterally centred, 0.085 m forward and 0.206 m
    # above the floor. Points are translated by this offset so that planning,
    # collision checking and the rollout all work in the chassis frame, whose
    # origin is the robot's centre of rotation.
    mount_offset_x: float = 0.085
    mount_offset_y: float = 0.0
    field_of_view: float = 2.0 * math.pi


@dataclass
class ClusterConfig:
    """Grouping of scan points into obstacles (needed to identify moving objects)."""

    base_threshold: float = 0.10     # m, constant part of the split threshold
    range_factor: float = 0.06       # m per metre of range (beam divergence)
    min_points: int = 2
    # Extended structure must be covered by SMALL discs: a disc around a whole
    # wall bulges by its own radius into the free space beside it.
    max_cluster_radius: float = 0.12
    max_obstacles: int = 150
    # A connected run of points longer than this is structure (a wall, a bench)
    # rather than an object, and its apparent velocity is never believed.
    structure_extent: float = 0.80
    # A cluster reaching the usable-range limit is CUT OFF, not small: its true
    # extent continues beyond what the sensor reports. Its apparent length
    # therefore says nothing about whether it is an object, and its centroid
    # migrates rapidly as the range gate sweeps along the surface. Such
    # clusters are treated as structure regardless of measured extent.
    truncation_margin: float = 0.15  # m below the usable range that counts as cut off


@dataclass
class TrackerConfig:
    """Basic obstacle tracking - just enough to tell moving obstacles from static."""

    association_gate: float = 0.45   # m, max centroid jump for association
    size_gate: float = 0.60          # m, max radius change for association
    alpha: float = 0.55              # position blending gain
    beta: float = 0.30               # velocity smoothing gain
    # Velocity is measured as displacement across a window, not scan-to-scan.
    # Cluster centroids jitter as visibility changes; that jitter cancels over a
    # window while real motion accumulates.
    velocity_window: float = 0.6     # s of history used for velocity
    min_velocity_span: float = 0.3   # s of history required before trusting it
    max_misses: int = 3              # scans a track survives without a detection
    min_hits_for_velocity: int = 3
    max_track_speed: float = 3.0     # m/s clamp for obviously bogus estimates
    moving_speed_threshold: float = 0.20  # m/s to be considered a moving obstacle
    # Gaps longer than this mean the velocity history spans a discontinuity
    # (a clock jump, a dropped sensor) and the estimates are reset.
    max_dt: float = 1.0               # s


@dataclass
class RiskConfig:
    """Simple, interpretable four-level risk assessment.

    Risk is a LEVEL, not a score to be optimised: CLEAR / CAUTION / DANGER /
    CRITICAL, each mapping to a speed scale. The inputs are the three things that
    actually matter - how close the obstacle is, how fast the gap is closing, and
    the time to collision.
    """

    caution_distance: float = 1.20   # m of clearance at/below which we slow down
    danger_distance: float = 0.60    # m of clearance at/below which we get cautious
    critical_distance: float = 0.25  # m of clearance at/below which we stop
    caution_ttc: float = 4.0         # s
    danger_ttc: float = 2.0          # s
    critical_ttc: float = 0.8        # s
    # Proximity only counts when it is ahead of the robot: 1.0 dead ahead,
    # falling to rear_weight directly behind. Without this a wall one metre
    # behind a forward-moving robot suppresses its speed for no reason.
    rear_weight: float = 0.10
    # Closing speed above which a threat counts as APPROACHING. Halting is a
    # sufficient response to a hazard the robot is driving into, but not to one
    # that is driving into the robot: the gap keeps shrinking either way, so
    # separation has to be created actively.
    closing_speed_threshold: float = 0.10   # m/s
    # Speed scale applied at each level.
    scale_clear: float = 1.00
    scale_caution: float = 0.60
    scale_danger: float = 0.30
    scale_critical: float = 0.00


@dataclass
class VelocityConfig:
    """Risk-adaptive speed envelope."""

    nominal_speed: float = 0.20      # m/s cruise speed when the world is clear
    min_progress_speed: float = 0.05  # m/s floor while still allowed to move
    brake_margin: float = 0.10       # m kept in hand when braking
    # Half-angle of the sector used for the braking constraint. The braking cap
    # limits FORWARD speed, so it must come from forward clearance.
    brake_sector: float = 1.31       # rad (~75 deg)
    smoothing: float = 0.55          # low-pass on speed increases (1 = none)


@dataclass
class PlannerConfig:
    """Local avoidance: a small fixed set of motion options, scored and checked."""

    sim_time: float = 3.0            # s of forward rollout per option
    sim_step: float = 0.25           # s between rollout poses
    # Angular rates of the steering options, rad/s.
    slight_turn: float = 0.35
    strong_turn: float = 0.80
    # Speed of the steering options as a fraction of the current speed limit.
    slight_turn_speed: float = 0.80
    strong_turn_speed: float = 0.45
    slow_forward_speed: float = 0.45
    # Free arc length ahead, capped here. Without this the planner only reacts
    # once an obstacle enters the rollout, which makes it creep up to walls
    # before turning; with it, a blocked heading is expensive from far away.
    headway_distance: float = 1.60   # m
    headway_step: float = 0.20       # m of arc between headway checks
    # Grid cell for downsampling scan points before collision checking. A
    # 1-degree scan puts beams 1.7 cm apart at 1 m but 7 cm apart at 4 m, so the
    # near field is hugely redundant; one point per cell keeps every obstacle at
    # a fraction of the cost. 0 disables downsampling.
    point_grid: float = 0.05         # m
    clearance_saturation: float = 1.00  # m, clearance beyond this scores the same
    # Scoring weights (lower total cost wins).
    weight_goal: float = 2.20        # progress toward the goal
    weight_heading: float = 1.10     # final heading error toward the goal
    weight_clearance: float = 1.30   # proximity along the rollout
    weight_headway: float = 1.80     # blocked heading
    weight_speed: float = 0.70       # prefer using the speed we are allowed
    weight_smoothness: float = 0.50  # penalise changes from the last command


@dataclass
class SafetyConfig:
    """Independent safety layer: limits, watchdogs, emergency stop, recovery."""

    scan_timeout: float = 0.5        # s without a scan before data is stale
    odom_timeout: float = 0.6        # s without odometry before it is stale
    require_odom: bool = True
    # Emergency stop triggers.
    estop_distance: float = 0.18     # m, obstacle inside this halts immediately
    estop_ttc: float = 0.45          # s
    estop_clear_distance: float = 0.30  # m clearance needed to leave E-stop
    estop_min_duration: float = 0.5  # s minimum latch time (anti-chatter)
    # Hard command limits, enforced on every outgoing command regardless of
    # what the planner asked for.
    max_linear_speed: float = 0.25   # m/s
    max_angular_speed: float = 1.2   # rad/s
    max_linear_jump: float = 0.10    # m/s per control cycle
    max_angular_jump: float = 0.35   # rad/s per control cycle
    # Recovery (mirrors the proven baseline: reverse, then turn).
    recovery_enabled: bool = True
    recovery_reverse_speed: float = 0.12
    recovery_reverse_time: float = 0.8
    recovery_rotate_speed: float = 0.40
    recovery_rotate_time: float = 1.5
    recovery_max_attempts: int = 3
    recovery_rear_clearance: float = 0.30  # m needed behind to reverse
    stuck_speed: float = 0.03        # m/s below which the robot counts as stuck
    stuck_time: float = 5.0          # s of being stuck before recovery


@dataclass
class GoalConfig:
    """Goal seeking and completion."""

    xy_tolerance: float = 0.20       # m - the goal-reached test
    align_final_yaw: bool = False    # the brief requires position only
    yaw_tolerance: float = 0.35      # rad, used only if align_final_yaw
    goal_timeout: float = 300.0      # s before an attempt is abandoned
    progress_timeout: float = 45.0   # s without progress before failing


@dataclass
class LoggingConfig:
    enabled: bool = True
    directory: str = 'beetlebot_logs'
    flush_every: int = 20


@dataclass
class NavConfig:
    """Root configuration object."""

    robot: RobotConfig = field(default_factory=RobotConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    cluster: ClusterConfig = field(default_factory=ClusterConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    risk: RiskConfig = field(default_factory=RiskConfig)
    velocity: VelocityConfig = field(default_factory=VelocityConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    goal: GoalConfig = field(default_factory=GoalConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)

    # ------------------------------------------------------------------
    def to_nested(self) -> Dict[str, Any]:
        return asdict(self)

    def to_flat(self) -> Dict[str, Any]:
        flat: Dict[str, Any] = {}
        for section in fields(self):
            sub = getattr(self, section.name)
            for f in fields(sub):
                value = getattr(sub, f.name)
                flat['{}.{}'.format(section.name, f.name)] = value
        return flat

    def apply_flat(self, values: Dict[str, Any]) -> List[str]:
        """Apply dotted ``section.key`` overrides; returns the unknown keys."""
        unknown: List[str] = []
        section_names = {f.name for f in fields(self)}
        for key, value in values.items():
            section_name, _, attr = key.partition('.')
            if not attr or section_name not in section_names:
                unknown.append(key)
                continue
            section = getattr(self, section_name)
            if not hasattr(section, attr):
                unknown.append(key)
                continue
            setattr(section, attr, _coerce(getattr(section, attr), value))
        return unknown

    def apply_nested(self, values: Dict[str, Any]) -> List[str]:
        return self.apply_flat(flatten(values))

    @classmethod
    def from_flat(cls, values: Dict[str, Any]) -> 'NavConfig':
        cfg = cls()
        cfg.apply_flat(values)
        return cfg

    # ------------------------------------------------------------------
    def validate(self) -> List[str]:
        """Human-readable configuration problems (empty list = valid)."""
        problems: List[str] = []
        r, li, v, s, p, g = (self.robot, self.lidar, self.velocity, self.safety,
                             self.planner, self.goal)
        if r.footprint_length <= 0 or r.footprint_width <= 0:
            problems.append('robot footprint dimensions must be positive')
        if r.robot_radius <= 0:
            problems.append('robot.robot_radius must be positive')
        if r.footprint_min_inflation > r.footprint_inflation:
            problems.append('robot.footprint_min_inflation must not exceed '
                            'robot.footprint_inflation')
        if r.footprint_min_inflation < 0:
            problems.append('robot.footprint_min_inflation must be >= 0')
        if r.max_linear_speed <= 0 or r.max_angular_speed <= 0:
            problems.append('robot speed limits must be positive')
        if r.max_linear_decel <= 0:
            problems.append('robot.max_linear_decel must be positive')
        if li.max_range <= li.min_range:
            problems.append('lidar range window is inverted')
        if li.max_usable_range > li.max_range:
            problems.append('lidar.max_usable_range exceeds lidar.max_range')
        if li.median_window % 2 == 0:
            problems.append('lidar.median_window must be odd')
        if v.nominal_speed > r.max_linear_speed + 1e-9:
            problems.append('velocity.nominal_speed exceeds robot.max_linear_speed')
        if s.max_linear_speed < r.max_linear_speed - 1e-9:
            problems.append('safety.max_linear_speed is below robot.max_linear_speed; '
                            'the supervisor would permanently clip the planner')
        if s.estop_distance >= s.estop_clear_distance:
            problems.append('safety.estop_clear_distance must exceed safety.estop_distance')
        if p.sim_step <= 0 or p.sim_time <= 0:
            problems.append('planner sim_time/sim_step must be positive')
        if g.xy_tolerance <= 0:
            problems.append('goal.xy_tolerance must be positive')
        risk = self.risk
        if not (risk.critical_distance < risk.danger_distance < risk.caution_distance):
            problems.append('risk distances must increase: critical < danger < caution')
        if not (risk.critical_ttc < risk.danger_ttc < risk.caution_ttc):
            problems.append('risk TTC thresholds must increase: critical < danger < caution')
        return problems


def _coerce(current: Any, value: Any) -> Any:
    if isinstance(current, bool):
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)
    if isinstance(current, int) and not isinstance(current, bool):
        return int(value)
    if isinstance(current, float):
        return float(value)
    if isinstance(current, str):
        return str(value)
    return value


def flatten(values: Dict[str, Any], prefix: str = '') -> Dict[str, Any]:
    flat: Dict[str, Any] = {}
    for key, value in values.items():
        full = '{}{}'.format(prefix, key)
        if isinstance(value, dict):
            flat.update(flatten(value, full + '.'))
        else:
            flat[full] = value
    return flat


def default_config() -> NavConfig:
    return NavConfig()


def describe(cfg: NavConfig) -> str:
    """Multi-line dump, logged by the node at startup."""
    lines: List[str] = []
    for section in fields(cfg):
        sub = getattr(cfg, section.name)
        if not is_dataclass(sub):
            continue
        lines.append('[{}]'.format(section.name))
        for f in fields(sub):
            lines.append('  {} = {}'.format(f.name, getattr(sub, f.name)))
    return '\n'.join(lines)
