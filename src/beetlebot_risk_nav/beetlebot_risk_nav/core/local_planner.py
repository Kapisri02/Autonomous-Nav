"""Local obstacle avoidance: choose one of a small set of motion options.

The robot picks between nine concrete manoeuvres - forward, slow forward, slight
and strong turns either way, turn in place either way, and stop. Each is rolled
out for a few seconds, rejected outright if the **footprint** would touch
anything, and otherwise scored on progress toward the goal, clearance, whether
the heading is blocked, and smoothness. The cheapest surviving option is issued.

Two representations are used, each where it is right:

* **Static structure** is checked against the filtered scan *points* directly.
  Points are raw measurements, so no clustering error can creep into the
  collision test. They are grid-downsampled first, which removes the heavy
  redundancy of near-field beams without losing any obstacle.
* **Moving obstacles** are checked as discs advanced along their measured
  velocity, so the robot is compared against where a person will actually be,
  not where they were.

A moving obstacle's *current* points remain in the static set, so the space it
is vacating stays blocked for the length of the rollout. That is deliberately
conservative: the cost is a slightly wider berth, the benefit is that a
mis-estimated velocity can never open a hole in the collision check.

If no option clears the preferred safety margin, the planner retries against the
smaller ``footprint_min_inflation`` margin before giving up. Refusing to move
because a wall is five centimetres inside a comfort margin would strand the
robot in exactly the cluttered spaces it most needs to handle.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .clustering import ObstacleDetector
from .footprint import FootprintChecker
from .geometry import angle_diff, arc_pose, clamp
from .params import NavConfig
from .risk import RiskAssessor
from .scan_filter import LidarFilter
from .tracker import ObstacleTracker
from .types import (Command, FilteredScan, MotionOption, Obstacle, PlannerDebug,
                    Pose, Point, RiskAssessment, ScanData, Track)
from .velocity import AdaptiveVelocity

Disc = Tuple[float, float, float]


@dataclass
class PlanResult:
    """Everything one planning cycle produced - the command plus its provenance."""

    command: Command = field(default_factory=Command)
    debug: PlannerDebug = field(default_factory=PlannerDebug)
    scan: Optional[FilteredScan] = None
    obstacles: List[Obstacle] = field(default_factory=list)
    tracks: List[Track] = field(default_factory=list)
    assessment: RiskAssessment = field(default_factory=RiskAssessment)
    options: List[MotionOption] = field(default_factory=list)
    best: Optional[MotionOption] = None
    speed_limit: float = 0.0
    forward_clearance: float = float('inf')
    degraded_margin: bool = False

    @property
    def feasible(self) -> bool:
        return self.best is not None


class LocalPlanner:
    def __init__(self, config: Optional[NavConfig] = None) -> None:
        self.config = config or NavConfig()
        cfg = self.config
        self.filter = LidarFilter(cfg.lidar)
        self.detector = ObstacleDetector(cfg.cluster)
        self.tracker = ObstacleTracker(cfg.tracker)
        self.risk = RiskAssessor(cfg.risk)
        self.velocity = AdaptiveVelocity(cfg.velocity, cfg.robot)
        self.footprint = FootprintChecker(cfg.robot)
        self.footprint_min = FootprintChecker(cfg.robot,
                                              inflation=cfg.robot.footprint_min_inflation)
        self._last_command: Tuple[float, float] = (0.0, 0.0)
        self._last_stamp: Optional[float] = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.tracker.reset()
        self.velocity.reset()
        self._last_command = (0.0, 0.0)
        self._last_stamp = None

    @property
    def last_command(self) -> Tuple[float, float]:
        return self._last_command

    # ------------------------------------------------------------------
    def motion_options(self, speed_limit: float, allow_reverse: bool = False
                       ) -> List[MotionOption]:
        """The fixed menu of manoeuvres, scaled to the speed currently allowed."""
        cfg = self.config.planner
        w_max = self.config.robot.max_angular_speed
        slight = min(cfg.slight_turn, w_max)
        strong = min(cfg.strong_turn, w_max)
        v = max(0.0, speed_limit)
        # 'stop' is deliberately NOT in this menu. Stopping is a fallback taken
        # when nothing here is safe, or when risk is CRITICAL - not an option
        # that competes on cost. Scoring it alongside the others made it win
        # whenever the speed limit was low, and because a stopped robot has a
        # low acceleration ceiling the next cycle, that fed back into an even
        # lower limit: the robot talked itself into standing still.
        menu: List[Tuple[str, float, float]] = [
            ('forward', v, 0.0),
            ('slow_forward', v * cfg.slow_forward_speed, 0.0),
            ('slight_left', v * cfg.slight_turn_speed, slight),
            ('slight_right', v * cfg.slight_turn_speed, -slight),
            ('strong_left', v * cfg.strong_turn_speed, strong),
            ('strong_right', v * cfg.strong_turn_speed, -strong),
            ('turn_left', 0.0, strong),
            ('turn_right', 0.0, -strong),
        ]
        if allow_reverse:
            reverse = -min(self.config.robot.max_reverse_speed, 0.12)
            menu.append(('reverse', reverse, 0.0))
        # When the speed limit collapses the straight-line options become
        # indistinguishable from stopping; dropping them keeps the menu honest,
        # so the planner never reports "forward" while commanding nothing.
        return [MotionOption(name=name, v=vv, w=ww) for name, vv, ww in menu
                if abs(vv) > 1e-6 or abs(ww) > 1e-6]

    def rollout_times(self) -> List[float]:
        cfg = self.config.planner
        times: List[float] = []
        t = cfg.sim_step
        while t <= cfg.sim_time + 1e-9:
            times.append(round(t, 6))
            t += cfg.sim_step
        return times

    def rollout(self, v: float, w: float, times: Sequence[float]) -> List[Pose]:
        poses: List[Pose] = []
        pose: Pose = (0.0, 0.0, 0.0)
        previous = 0.0
        for t in times:
            pose = arc_pose(pose, v, w, t - previous)
            previous = t
            poses.append(pose)
        return poses

    # ------------------------------------------------------------------
    def plan(self, scan: ScanData, goal_local: Point,
             robot_pose: Pose = (0.0, 0.0, 0.0),
             robot_velocity: Tuple[float, float] = (0.0, 0.0),
             allow_reverse: bool = False) -> PlanResult:
        started = time.monotonic()
        cfg = self.config
        result = PlanResult()

        dt = None if self._last_stamp is None else max(0.0, scan.stamp - self._last_stamp)
        self._last_stamp = scan.stamp

        # --- perception -----------------------------------------------------
        filtered = self.filter.filter(scan)
        result.scan = filtered

        # Absence of data is not evidence of clear space. An empty or mostly-NaN
        # scan gives no grounds to believe the way ahead is free, so the robot
        # holds still. The safety supervisor independently handles scans that
        # stop arriving at all; this covers scans that arrive but say nothing.
        if filtered.informative_fraction < cfg.lidar.min_valid_fraction:
            result.command = Command(v=0.0, w=0.0, source='local_planner',
                                     reason='insufficient LiDAR data ({:.0f}% informative)'
                                            .format(100.0 * filtered.informative_fraction))
            self._last_command = (0.0, 0.0)
            result.debug.stamp = scan.stamp
            result.debug.n_points = filtered.count
            result.debug.state = 'NO_DATA'
            result.debug.chosen = 'stop'
            result.debug.notes = result.command.reason
            result.debug.compute_time = time.monotonic() - started
            return result
        obstacles = self.detector.detect(filtered)
        result.obstacles = obstacles
        tracks = self.tracker.update(obstacles, scan.stamp, robot_pose, robot_velocity)
        result.tracks = tracks

        # --- obstacle set -----------------------------------------------------
        times = self.rollout_times()
        static_points = self._static_points(filtered, self._reach_radius())
        movers = [t for t in tracks if t.is_moving]

        # --- risk and speed envelope ------------------------------------------
        # Forward clearance is the exact footprint-to-point distance within the
        # braking sector. It is measured from scan points rather than clusters so
        # that a clustering fault cannot remove the braking limit, and it is
        # restricted to the forward sector because the braking cap governs
        # forward speed - an obstacle behind must not throttle the robot.
        forward_clearance = self._forward_clearance(static_points)
        result.forward_clearance = forward_clearance
        assessment = self.risk.assess(tracks, self.footprint, forward_clearance)
        result.assessment = assessment

        speed_limit = self.velocity.speed_limit(assessment.speed_scale, forward_clearance,
                                                current_speed=robot_velocity[0], dt=dt)
        result.speed_limit = speed_limit

        # --- score the menu ---------------------------------------------------
        options = self.motion_options(speed_limit, allow_reverse)
        for option in options:
            option.poses = self.rollout(option.v, option.w, times)
        self._evaluate(options, goal_local, static_points, movers, times, speed_limit,
                       self.footprint)
        best = self._best(options)

        if best is None and cfg.robot.footprint_min_inflation < cfg.robot.footprint_inflation:
            for option in options:
                option.feasible = True
                option.rejected_reason = ''
                option.cost = 0.0
            self._evaluate(options, goal_local, static_points, movers, times, speed_limit,
                           self.footprint_min)
            best = self._best(options)
            result.degraded_margin = best is not None

        result.options = options

        # Selection follows the brief's priority: SAFETY > AVOIDANCE > GOAL.
        # Critical risk stops the robot outright; escaping from there is the
        # recovery behaviour's job, not the local planner's.
        if assessment.level == 'CRITICAL':
            result.best = None
            command = Command(v=0.0, w=0.0, source='local_planner',
                              reason='critical risk: ' + assessment.reason)
        elif best is not None:
            result.best = best
            command = Command(v=best.v, w=best.w, source='local_planner',
                              reason=(best.name + ' (degraded margin)'
                                      if result.degraded_margin else best.name))
        else:
            result.best = None
            command = Command(v=0.0, w=0.0, source='local_planner',
                              reason='no safe motion available')
        result.command = command
        self._last_command = (command.v, command.w)

        # --- diagnostics -------------------------------------------------------
        debug = result.debug
        debug.stamp = scan.stamp
        debug.n_points = filtered.count
        debug.n_obstacles = len(obstacles)
        debug.n_tracks = len(tracks)
        debug.n_moving = len(movers)
        debug.risk_level = assessment.level
        debug.ttc = assessment.ttc
        debug.clearance = forward_clearance
        debug.speed_limit = speed_limit
        debug.n_options = len(options)
        debug.n_feasible = sum(1 for o in options if o.feasible)
        debug.chosen = result.best.name if result.best else 'stop'
        debug.chosen_cost = result.best.cost if result.best else math.inf
        debug.state = 'PLAN'
        debug.notes = assessment.reason
        debug.compute_time = time.monotonic() - started
        return result

    # ------------------------------------------------------------------
    def _reach_radius(self) -> float:
        """Radius beyond which nothing can affect this cycle's decision."""
        cfg = self.config
        travel = cfg.robot.max_linear_speed * cfg.planner.sim_time
        return cfg.robot.circumscribed_radius + travel + cfg.planner.headway_distance

    def _forward_clearance(self, points: Sequence[Point]) -> float:
        sector = self.config.velocity.brake_sector
        ahead = [p for p in points if abs(math.atan2(p[1], p[0])) <= sector]
        if not ahead:
            return float('inf')
        return max(0.0, self.footprint.clearance_to_points((0.0, 0.0, 0.0), ahead))

    def _static_points(self, filtered: FilteredScan, reach: float) -> List[Point]:
        """Points within reach, grid-downsampled to remove near-field redundancy.

        A 1-degree scan puts beams 1.7 cm apart at one metre but 7 cm apart at
        four. Keeping every near-field point multiplies the cost of the collision
        loop without adding information, so one point per grid cell is kept - the
        one closest to the robot, which is the one that matters.
        """
        cell = self.config.planner.point_grid
        reach_sq = reach * reach
        if cell <= 0.0:
            return [p for p in filtered.points if (p[0] * p[0] + p[1] * p[1]) <= reach_sq]
        buckets: Dict[Tuple[int, int], Tuple[float, Point]] = {}
        inv = 1.0 / cell
        for p in filtered.points:
            d_sq = p[0] * p[0] + p[1] * p[1]
            if d_sq > reach_sq:
                continue
            key = (int(math.floor(p[0] * inv)), int(math.floor(p[1] * inv)))
            current = buckets.get(key)
            if current is None or d_sq < current[0]:
                buckets[key] = (d_sq, p)
        return [entry[1] for entry in buckets.values()]

    # ------------------------------------------------------------------
    def _evaluate(self, options: Sequence[MotionOption], goal_local: Point,
                  static_points: Sequence[Point], movers: Sequence[Track],
                  times: Sequence[float], speed_limit: float,
                  footprint: FootprintChecker) -> None:
        goal_distance = max(1e-3, math.hypot(goal_local[0], goal_local[1]))
        mover_discs = [[(px, py, m.radius) for m in movers
                        for px, py in (m.predict(t),)] for t in times]
        for option in options:
            self._score(option, goal_local, goal_distance, static_points, mover_discs,
                        times, speed_limit, footprint)

    def _score(self, option: MotionOption, goal_local: Point, goal_distance: float,
               static_points: Sequence[Point], mover_discs: Sequence[Sequence[Disc]],
               times: Sequence[float], speed_limit: float,
               footprint: FootprintChecker) -> None:
        cfg = self.config.planner
        poses = option.poses
        min_clearance = math.inf

        for i, pose in enumerate(poses):
            clearance = footprint.clearance_to_points(pose, static_points, stop_below=0.0)
            if mover_discs and i < len(mover_discs) and mover_discs[i]:
                dynamic = footprint.clearance(pose, mover_discs[i], stop_below=0.0)
                if dynamic < clearance:
                    clearance = dynamic
            if clearance < min_clearance:
                min_clearance = clearance
            if clearance <= 0.0:
                option.feasible = False
                option.clearance = clearance
                option.collision_time = times[i] if i < len(times) else math.inf
                option.rejected_reason = 'collision at t={:.2f}s'.format(option.collision_time)
                option.cost = math.inf
                return

        option.clearance = min_clearance
        option.feasible = True
        option.rejected_reason = ''

        # --- cost terms ------------------------------------------------------
        end = poses[-1]
        goal_error = math.hypot(goal_local[0] - end[0], goal_local[1] - end[1])
        goal_term = clamp(goal_error / goal_distance, 0.0, 2.0)

        # Heading error at the end of the rollout. Without this term a pure
        # rotation scores identically whichever way it turns, because rotating in
        # place does not change the distance to the goal - the robot would have
        # no basis for choosing a direction to turn.
        heading_error = abs(angle_diff(math.atan2(goal_local[1] - end[1],
                                                  goal_local[0] - end[0]), end[2]))
        heading_term = clamp(heading_error / math.pi, 0.0, 1.0)

        sat = max(1e-3, cfg.clearance_saturation)
        clearance_term = 1.0 - clamp(min_clearance / sat, 0.0, 1.0)

        free = self._headway(end, option, static_points, footprint)
        option.headway = free
        headway_term = 1.0 - clamp(free / max(1e-3, cfg.headway_distance), 0.0, 1.0)

        if speed_limit > 1e-6:
            speed_term = clamp((speed_limit - max(0.0, option.v)) / speed_limit, 0.0, 1.0)
        else:
            speed_term = 0.0

        robot = self.config.robot
        dv = abs(option.v - self._last_command[0]) / max(1e-6, robot.max_linear_speed)
        dw = abs(option.w - self._last_command[1]) / max(1e-6, robot.max_angular_speed)
        smoothness_term = clamp(0.5 * (dv + dw), 0.0, 1.0)

        option.cost_terms = {
            'goal': cfg.weight_goal * goal_term,
            'heading': cfg.weight_heading * heading_term,
            'clearance': cfg.weight_clearance * clearance_term,
            'headway': cfg.weight_headway * headway_term,
            'speed': cfg.weight_speed * speed_term,
            'smoothness': cfg.weight_smoothness * smoothness_term,
        }
        option.cost = sum(option.cost_terms.values())

    def _headway(self, start: Pose, option: MotionOption, points: Sequence[Point],
                 footprint: FootprintChecker) -> float:
        """Free arc length ahead of the rollout, capped at ``headway_distance``.

        Without this the planner only reacts once an obstacle enters the rollout,
        which makes it creep up to a wall before turning. With it, a blocked
        heading is expensive from far away and the robot deviates early.
        """
        cfg = self.config.planner
        speed = abs(option.v)
        if speed < 1e-6 or not points:
            return cfg.headway_distance
        step = max(0.05, cfg.headway_step)
        dt = step / speed
        pose = start
        travelled = 0.0
        while travelled < cfg.headway_distance:
            pose = arc_pose(pose, option.v, option.w, dt)
            travelled += step
            if footprint.clearance_to_points(pose, points, stop_below=0.0) <= 0.0:
                return max(0.0, travelled - step)
        return cfg.headway_distance

    @staticmethod
    def _best(options: Sequence[MotionOption]) -> Optional[MotionOption]:
        best: Optional[MotionOption] = None
        for option in options:
            if not option.feasible:
                continue
            if best is None or option.cost < best.cost:
                best = option
        return best

    # ------------------------------------------------------------------
    def describe(self, result: PlanResult) -> str:
        d = result.debug
        return ('pts={} obs={} trk={}({} moving) {} clr={:.2f} ttc={:.2f} vmax={:.2f} '
                '-> {} v={:.3f} w={:+.3f} [{}/{} ok{}]').format(
                    d.n_points, d.n_obstacles, d.n_tracks, d.n_moving, d.risk_level,
                    d.clearance, d.ttc, d.speed_limit, d.chosen, result.command.v,
                    result.command.w, d.n_feasible, d.n_options,
                    ', degraded' if result.degraded_margin else '')
