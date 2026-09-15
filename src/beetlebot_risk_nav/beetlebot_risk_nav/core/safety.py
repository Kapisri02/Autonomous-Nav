"""Independent safety layer.

Everything the navigator asks for passes through here, and this layer can always
override it. Its priority order is fixed and matches the project brief:

    SAFETY  >  OBSTACLE AVOIDANCE  >  GOAL PROGRESS

Checks are applied in order of severity, and the first one that fires wins:

1. **Sensor watchdog** - LiDAR or odometry that has stopped arriving. Stale data
   is worse than no data, because it looks valid.
2. **Command validity** - NaN, infinity, or anything not finite. A single NaN
   reaching the base controller is an unbounded-motion hazard.
3. **Emergency stop** - an obstacle critically close or a collision imminent.
   Latched, with a minimum duration and a larger clearance required to release
   it, so the robot cannot chatter in and out of E-stop at the threshold.
   Forward motion is absolutely blocked while it is latched, but a *retreat* is
   allowed when there is verified room behind. An E-stop that cannot be escaped
   is not a safe state: a box set down in front of the robot would otherwise
   freeze it permanently, with no way back short of carrying it.
4. **Command limits** - absolute speed caps plus per-cycle rate limits, applied
   to whatever survives the checks above.

The supervisor deliberately holds no opinion about *where* the robot should go.
It only decides whether what was asked for is safe to execute.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .geometry import clamp
from .params import SafetyConfig
from .types import Command

#: Supervisor states, ordered by severity.
SAFETY_OK = 'OK'
SAFETY_LIMITED = 'LIMITED'
SAFETY_ESTOP = 'EMERGENCY_STOP'
SAFETY_SENSOR_FAULT = 'SENSOR_FAULT'
SAFETY_INVALID = 'INVALID_COMMAND'


@dataclass
class SafetyStatus:
    """Verdict for one cycle."""

    state: str = SAFETY_OK
    reason: str = ''
    command: Command = field(default_factory=Command)
    estop_active: bool = False
    limited: bool = False
    faults: List[str] = field(default_factory=list)

    @property
    def is_stopped(self) -> bool:
        return abs(self.command.v) < 1e-9 and abs(self.command.w) < 1e-9

    @property
    def healthy(self) -> bool:
        return self.state in (SAFETY_OK, SAFETY_LIMITED)


class SafetySupervisor:
    def __init__(self, config: Optional[SafetyConfig] = None) -> None:
        self.config = config or SafetyConfig()
        self._estop_active = False
        self._estop_since: Optional[float] = None
        self._last_command: Tuple[float, float] = (0.0, 0.0)

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._estop_active = False
        self._estop_since = None
        self._last_command = (0.0, 0.0)

    @property
    def estop_active(self) -> bool:
        return self._estop_active

    # ------------------------------------------------------------------
    def check(self, now: float, command: Command,
              scan_stamp: Optional[float] = None,
              odom_stamp: Optional[float] = None,
              clearance: float = float('inf'),
              ttc: float = float('inf'),
              rear_clearance: float = float('inf')) -> SafetyStatus:
        """Validate and, if necessary, override ``command``."""
        status = SafetyStatus()

        # --- 1. sensor watchdog -------------------------------------------
        faults = self._sensor_faults(now, scan_stamp, odom_stamp)
        if faults:
            status.faults = faults
            status.state = SAFETY_SENSOR_FAULT
            status.reason = '; '.join(faults)
            status.command = Command(0.0, 0.0, source='safety', reason=status.reason)
            self._last_command = (0.0, 0.0)
            return status

        # --- 2. command validity -------------------------------------------
        if not (_finite(command.v) and _finite(command.w)):
            status.state = SAFETY_INVALID
            status.reason = 'non-finite command v={} w={}'.format(command.v, command.w)
            status.command = Command(0.0, 0.0, source='safety', reason=status.reason)
            self._last_command = (0.0, 0.0)
            return status

        # --- 3. emergency stop ---------------------------------------------
        estop_reason = self._update_estop(now, clearance, ttc)
        if self._estop_active:
            status.state = SAFETY_ESTOP
            status.estop_active = True
            escape = self._escape_command(command, rear_clearance)
            if escape is not None:
                status.reason = estop_reason + ' (retreat permitted)'
                status.command = Command(escape, 0.0, source='safety',
                                         reason=status.reason)
                self._last_command = (escape, 0.0)
            else:
                status.reason = estop_reason
                status.command = Command(0.0, 0.0, source='safety', reason=estop_reason)
                self._last_command = (0.0, 0.0)
            return status

        # --- 4. command limits ----------------------------------------------
        limited_v, limited_w, limited = self._apply_limits(command.v, command.w)
        status.command = Command(v=limited_v, w=limited_w, source=command.source,
                                 reason=command.reason)
        status.limited = limited
        status.state = SAFETY_LIMITED if limited else SAFETY_OK
        if limited:
            status.reason = 'command clamped to safe limits'
        self._last_command = (limited_v, limited_w)
        return status

    # ------------------------------------------------------------------
    def _sensor_faults(self, now: float, scan_stamp: Optional[float],
                       odom_stamp: Optional[float]) -> List[str]:
        cfg = self.config
        faults: List[str] = []
        if scan_stamp is None:
            faults.append('no LiDAR data received')
        elif now - scan_stamp > cfg.scan_timeout:
            faults.append('LiDAR stale by {:.2f} s'.format(now - scan_stamp))
        if cfg.require_odom:
            if odom_stamp is None:
                faults.append('no odometry received')
            elif now - odom_stamp > cfg.odom_timeout:
                faults.append('odometry stale by {:.2f} s'.format(now - odom_stamp))
        return faults

    def _update_estop(self, now: float, clearance: float, ttc: float) -> str:
        """Latch, hold and release the emergency stop."""
        cfg = self.config
        trigger = ''
        if clearance <= cfg.estop_distance:
            trigger = 'obstacle at {:.2f} m (limit {:.2f} m)'.format(
                clearance, cfg.estop_distance)
        elif ttc <= cfg.estop_ttc:
            trigger = 'collision in {:.2f} s (limit {:.2f} s)'.format(ttc, cfg.estop_ttc)

        if trigger:
            if not self._estop_active:
                self._estop_active = True
                self._estop_since = now
            return 'emergency stop: ' + trigger

        if not self._estop_active:
            return ''

        # Release requires BOTH a minimum latch time and more clearance than it
        # took to trigger; without the hysteresis the robot would oscillate in
        # and out of E-stop while sitting at the threshold.
        held = now - (self._estop_since if self._estop_since is not None else now)
        if held < cfg.estop_min_duration:
            return 'emergency stop: held for {:.2f} s of {:.2f} s'.format(
                held, cfg.estop_min_duration)
        if clearance < cfg.estop_clear_distance:
            return 'emergency stop: clearance {:.2f} m below release {:.2f} m'.format(
                clearance, cfg.estop_clear_distance)

        self._estop_active = False
        self._estop_since = None
        return ''

    def _escape_command(self, command: Command, rear_clearance: float) -> Optional[float]:
        """The only motion allowed out of an emergency stop: a slow, clear retreat.

        Forward motion stays blocked unconditionally, and rotation is withheld
        too - a rectangular chassis sweeps its corners when it turns, and under
        E-stop there is by definition not enough room to be sure that is safe.
        Backing straight out of trouble only needs the space behind, which is
        checked here rather than assumed.
        """
        cfg = self.config
        if not cfg.recovery_enabled or command.v >= -1e-6:
            return None
        if rear_clearance < cfg.recovery_rear_clearance:
            return None
        return -min(abs(command.v), abs(cfg.recovery_reverse_speed))

    def _apply_limits(self, v: float, w: float) -> Tuple[float, float, bool]:
        cfg = self.config
        original = (v, w)
        v = clamp(v, -abs(cfg.max_linear_speed), abs(cfg.max_linear_speed))
        w = clamp(w, -abs(cfg.max_angular_speed), abs(cfg.max_angular_speed))
        last_v, last_w = self._last_command
        v = clamp(v, last_v - cfg.max_linear_jump, last_v + cfg.max_linear_jump)
        w = clamp(w, last_w - cfg.max_angular_jump, last_w + cfg.max_angular_jump)
        limited = (abs(v - original[0]) > 1e-9) or (abs(w - original[1]) > 1e-9)
        return (v, w, limited)


def _finite(value: float) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False
