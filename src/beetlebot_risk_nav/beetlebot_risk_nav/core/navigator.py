"""Start-to-goal navigation flow.

This is the whole pipeline in one object: given a goal and a stream of scans and
poses, it produces the velocity command to send to the base, and it decides when
the job is done.

    goal + pose -> local planner -> safety supervisor -> command
                                 -> goal tolerance     -> SUCCESS

States:

``IDLE``          no goal; the robot is commanded to hold still.
``NAVIGATING``    driving toward the goal with local avoidance.
``RECOVERY``      boxed in or stuck; performing the reverse-then-turn manoeuvre
                  inherited from the proven baseline controller.
``GOAL_REACHED``  within tolerance. **Terminal**: the robot is stopped and no
                  further navigation commands are issued, which is the explicit
                  completion requirement.
``FAILED``        gave up (timed out, made no progress, or recovery exhausted).
                  Also terminal, and also commands a stop.

The navigator owns the safety supervisor rather than leaving it to the ROS node,
so that the safety layer is exercised by the same tests as everything else and
cannot be bypassed by a different caller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

from .geometry import inverse_transform_point
from .local_planner import LocalPlanner, PlanResult
from .params import NavConfig
from .safety import SAFETY_OK, SafetyStatus, SafetySupervisor
from .scan_filter import sector_minimum
from .types import Command, Pose, ScanData

STATE_IDLE = 'IDLE'
STATE_NAVIGATING = 'NAVIGATING'
STATE_RECOVERY = 'RECOVERY'
STATE_GOAL_REACHED = 'GOAL_REACHED'
STATE_FAILED = 'FAILED'

TERMINAL_STATES = (STATE_GOAL_REACHED, STATE_FAILED)


@dataclass
class NavResult:
    """What the navigator decided this cycle."""

    command: Command = field(default_factory=Command)
    state: str = STATE_IDLE
    status: str = ''
    distance_to_goal: float = float('inf')
    succeeded: bool = False
    plan: Optional[PlanResult] = None
    safety: Optional[SafetyStatus] = None

    @property
    def finished(self) -> bool:
        return self.state in TERMINAL_STATES


class _Recovery:
    """Reverse, then turn toward the clearer side - the baseline's manoeuvre.

    Kept because it is the behaviour already proven on this robot, and because a
    differential-drive base genuinely has few options once it is boxed in.
    """

    REVERSING = 'reversing'
    ROTATING = 'rotating'
    DONE = 'done'

    def __init__(self, config) -> None:
        self.config = config
        self.phase = self.DONE
        self.started = 0.0
        self.direction = 1.0
        self.attempts = 0

    def reset(self) -> None:
        self.phase = self.DONE
        self.attempts = 0

    @property
    def active(self) -> bool:
        return self.phase != self.DONE

    def start(self, now: float, rear_clearance: float, turn_left: bool) -> None:
        cfg = self.config
        self.attempts += 1
        self.started = now
        self.direction = 1.0 if turn_left else -1.0
        # Only reverse if there is room behind; otherwise turn on the spot.
        self.phase = (self.REVERSING if rear_clearance >= cfg.recovery_rear_clearance
                      else self.ROTATING)

    def step(self, now: float) -> Command:
        cfg = self.config
        elapsed = now - self.started
        if self.phase == self.REVERSING:
            if elapsed < cfg.recovery_reverse_time:
                return Command(-cfg.recovery_reverse_speed, 0.0, source='recovery',
                               reason='reversing')
            self.phase = self.ROTATING
            self.started = now
            elapsed = 0.0
        if self.phase == self.ROTATING:
            if elapsed < cfg.recovery_rotate_time:
                return Command(0.0, self.direction * cfg.recovery_rotate_speed,
                               source='recovery', reason='rotating to find a way out')
            self.phase = self.DONE
        return Command(0.0, 0.0, source='recovery', reason='recovery complete')


class Navigator:
    def __init__(self, config: Optional[NavConfig] = None) -> None:
        self.config = config or NavConfig()
        self.planner = LocalPlanner(self.config)
        self.supervisor = SafetySupervisor(self.config.safety)
        self._recovery = _Recovery(self.config.safety)

        self.state = STATE_IDLE
        self.goal: Optional[Tuple[float, float]] = None
        self._goal_started: Optional[float] = None
        self._best_distance = float('inf')
        self._last_progress_time: Optional[float] = None
        self._blocked_since: Optional[float] = None
        self._moving_since: Optional[float] = None
        self._final_status = ''

    # ------------------------------------------------------------------
    def set_goal(self, goal: Tuple[float, float], now: float) -> None:
        """Accept a new goal (from RViz 2D Goal Pose) and start navigating."""
        self.goal = (float(goal[0]), float(goal[1]))
        self.state = STATE_NAVIGATING
        self._goal_started = now
        self._best_distance = float('inf')
        self._last_progress_time = now
        self._final_status = ''
        self._blocked_since = None
        self._moving_since = now
        self._recovery.reset()
        self.planner.reset()
        self.supervisor.reset()

    def cancel(self) -> None:
        self.goal = None
        self.state = STATE_IDLE
        self._recovery.reset()

    def reset(self) -> None:
        self.cancel()
        self.planner.reset()
        self.supervisor.reset()

    # ------------------------------------------------------------------
    def update(self, scan: Optional[ScanData], robot_pose: Pose,
               robot_velocity: Tuple[float, float], now: float,
               odom_stamp: Optional[float] = None) -> NavResult:
        """Run one control cycle and return the command to publish."""
        result = NavResult(state=self.state)

        # Terminal states latch: once the goal is reached the robot stops and
        # stays stopped. Issuing further navigation commands after success is
        # exactly the failure the completion requirement rules out.
        if self.state in TERMINAL_STATES:
            result.succeeded = self.state == STATE_GOAL_REACHED
            # Keep reporting *why* it ended. A bare "navigation failed" tells an
            # operator nothing, and the reason is only computed on the cycle the
            # transition happens.
            result.status = self._final_status or (
                'goal reached - navigation complete' if result.succeeded
                else 'navigation failed')
            result.command = Command(0.0, 0.0, source='navigator', reason=result.status)
            if self.goal is not None:
                result.distance_to_goal = self._distance(robot_pose)
            return result

        if self.goal is None or scan is None:
            result.state = self.state = STATE_IDLE
            result.status = 'waiting for a goal' if self.goal is None else 'waiting for a scan'
            result.command = Command(0.0, 0.0, source='navigator', reason=result.status)
            return result

        distance = self._distance(robot_pose)
        result.distance_to_goal = distance

        # --- goal completion --------------------------------------------------
        if distance <= self.config.goal.xy_tolerance:
            self.state = STATE_GOAL_REACHED
            result.state = self.state
            result.succeeded = True
            result.status = 'goal reached ({:.2f} m <= {:.2f} m tolerance)'.format(
                distance, self.config.goal.xy_tolerance)
            self._final_status = result.status
            result.command = Command(0.0, 0.0, source='navigator', reason=result.status)
            return result

        # --- progress and timeouts ---------------------------------------------
        self._track_progress(distance, now)
        failure = self._timeout_failure(now, distance)
        if failure:
            self.state = STATE_FAILED
            result.state = self.state
            result.status = failure
            self._final_status = failure
            result.command = Command(0.0, 0.0, source='navigator', reason=failure)
            return result

        # --- plan ---------------------------------------------------------------
        goal_local = inverse_transform_point(self.goal, robot_pose)
        plan = self.planner.plan(scan, goal_local, robot_pose=robot_pose,
                                 robot_velocity=robot_velocity)
        result.plan = plan

        # --- recovery ------------------------------------------------------------
        command = self._recovery_or_plan(plan, now, robot_velocity)
        result.state = self.state

        # --- safety supervisor (always last, always able to override) -----------
        safety = self.supervisor.check(
            now, command, scan_stamp=scan.stamp, odom_stamp=odom_stamp,
            clearance=plan.forward_clearance, ttc=plan.assessment.ttc,
            rear_clearance=self._rear_clearance(plan))
        result.safety = safety
        result.command = safety.command
        result.status = self._status_text(plan, safety, distance)
        return result

    # ------------------------------------------------------------------
    def _recovery_or_plan(self, plan: PlanResult, now: float,
                          robot_velocity: Tuple[float, float]) -> Command:
        cfg = self.config.safety

        if self._recovery.active:
            command = self._recovery.step(now)
            if self._recovery.active:
                self.state = STATE_RECOVERY
                return command
            self.state = STATE_NAVIGATING
            self._blocked_since = None
            self._moving_since = now
            self.planner.reset()

        blocked = plan.best is None
        stuck = self._update_stuck(plan, robot_velocity, now)

        if (blocked or stuck) and cfg.recovery_enabled:
            if self._blocked_since is None:
                self._blocked_since = now
            waited = now - self._blocked_since
            # A brief block is normal - a person stepping past is enough. Only
            # commit to a recovery manoeuvre once it persists.
            if stuck or waited >= cfg.stuck_time:
                if self._recovery.attempts >= cfg.recovery_max_attempts:
                    return Command(0.0, 0.0, source='navigator',
                                   reason='recovery attempts exhausted')
                rear = self._rear_clearance(plan)
                self._recovery.start(now, rear, turn_left=self._clearer_side_is_left(plan))
                self.state = STATE_RECOVERY
                self._blocked_since = None
                self._moving_since = now
                return self._recovery.step(now)
        elif not blocked:
            self._blocked_since = None

        self.state = STATE_NAVIGATING
        return plan.command

    def _update_stuck(self, plan: PlanResult, robot_velocity: Tuple[float, float],
                      now: float) -> bool:
        """True when the robot is being told to move but is not moving."""
        cfg = self.config.safety
        commanded = abs(plan.command.v) > 1e-3 or abs(plan.command.w) > 1e-3
        measured = max(abs(robot_velocity[0]), abs(robot_velocity[1]))
        if not commanded or measured > cfg.stuck_speed:
            self._moving_since = now
            return False
        if self._moving_since is None:
            self._moving_since = now
            return False
        return (now - self._moving_since) >= cfg.stuck_time

    def _rear_clearance(self, plan: PlanResult) -> float:
        if plan.scan is None:
            return 0.0
        behind = sector_minimum(plan.scan, math.pi, math.radians(50))
        if math.isinf(behind):
            return float('inf')
        return max(0.0, behind - self.config.robot.circumscribed_radius)

    def _clearer_side_is_left(self, plan: PlanResult) -> bool:
        """Pick the turn direction the way the baseline does: whichever is roomier."""
        if plan.scan is None:
            return True
        left = sector_minimum(plan.scan, math.radians(60), math.radians(45))
        right = sector_minimum(plan.scan, math.radians(-60), math.radians(45))
        return left >= right

    # ------------------------------------------------------------------
    def _distance(self, robot_pose: Pose) -> float:
        if self.goal is None:
            return float('inf')
        return math.hypot(self.goal[0] - robot_pose[0], self.goal[1] - robot_pose[1])

    def _track_progress(self, distance: float, now: float) -> None:
        if distance < self._best_distance - 0.05:
            self._best_distance = distance
            self._last_progress_time = now

    def _timeout_failure(self, now: float, distance: float) -> str:
        cfg = self.config.goal
        if self._goal_started is not None and now - self._goal_started > cfg.goal_timeout:
            return 'goal timeout after {:.0f} s, still {:.2f} m away'.format(
                now - self._goal_started, distance)
        if (self._last_progress_time is not None
                and now - self._last_progress_time > cfg.progress_timeout):
            return 'no progress for {:.0f} s, still {:.2f} m away'.format(
                now - self._last_progress_time, distance)
        return ''

    def _status_text(self, plan: PlanResult, safety: SafetyStatus,
                     distance: float) -> str:
        parts = ['{} {:.2f} m to goal'.format(self.state, distance)]
        parts.append('risk {}'.format(plan.assessment.level))
        parts.append('clr {:.2f} m'.format(plan.forward_clearance))
        parts.append(plan.command.reason or plan.debug.chosen)
        if safety.state != SAFETY_OK:
            parts.append('safety: {} ({})'.format(safety.state, safety.reason))
        return ' | '.join(parts)
