"""Per-cycle CSV logging of navigation runs.

One row per control cycle, written to a plain CSV so that a run can be examined
with ``scripts/analyze_log.py``, a spreadsheet, or anything else. This is the
evidence trail for physical testing: when the robot does something unexpected on
the floor, the log says what it saw, what it decided and why.

The logger never raises into the control loop. A navigation stack that stops
driving because a disk filled up would be a worse failure than losing the log,
so every I/O error disables logging and is reported once.
"""

from __future__ import annotations

import csv
import os
import time
from typing import Any, Dict, Optional, Sequence, Tuple

from .params import LoggingConfig

FIELDS: Sequence[str] = (
    'time', 'state', 'x', 'y', 'yaw', 'cmd_v', 'cmd_w',
    'risk_level', 'clearance', 'ttc', 'speed_limit', 'chosen',
    'n_points', 'n_obstacles', 'n_tracks', 'n_moving',
    'safety_state', 'estop', 'distance_to_goal', 'compute_ms', 'notes',
)


class NavigationLogger:
    def __init__(self, config: Optional[LoggingConfig] = None) -> None:
        self.config = config or LoggingConfig()
        self.path: Optional[str] = None
        self._handle = None
        self._writer = None
        self._rows_since_flush = 0
        self._disabled_reason = ''

    # ------------------------------------------------------------------
    @property
    def active(self) -> bool:
        return self._writer is not None

    def start_episode(self, goal: Tuple[float, float],
                      timestamp: Optional[float] = None) -> Optional[str]:
        """Open a new log file for one navigation attempt."""
        self.close()
        if not self.config.enabled:
            return None
        try:
            os.makedirs(self.config.directory, exist_ok=True)
            name = 'run_{}_goal_{:.2f}_{:.2f}.csv'.format(
                time.strftime('%Y%m%d_%H%M%S', time.localtime(timestamp)),
                goal[0], goal[1])
            self.path = os.path.join(self.config.directory, name)
            self._handle = open(self.path, 'w', newline='')
            self._writer = csv.DictWriter(self._handle, fieldnames=list(FIELDS))
            self._writer.writeheader()
            self._rows_since_flush = 0
            return self.path
        except OSError as exc:
            self._disable('could not open log file: {}'.format(exc))
            return None

    # ------------------------------------------------------------------
    def log_cycle(self, now: float, result, pose: Tuple[float, float, float]) -> None:
        """Append one row. Safe to call when logging is disabled."""
        if self._writer is None:
            return
        plan = result.plan
        debug = plan.debug if plan is not None else None
        safety = result.safety
        row: Dict[str, Any] = {
            'time': round(now, 3),
            'state': result.state,
            'x': round(pose[0], 4),
            'y': round(pose[1], 4),
            'yaw': round(pose[2], 4),
            'cmd_v': round(result.command.v, 4),
            'cmd_w': round(result.command.w, 4),
            'risk_level': plan.assessment.level if plan else '',
            'clearance': _round(plan.forward_clearance if plan else None),
            'ttc': _round(plan.assessment.ttc if plan else None),
            'speed_limit': _round(plan.speed_limit if plan else None),
            'chosen': debug.chosen if debug else '',
            'n_points': debug.n_points if debug else 0,
            'n_obstacles': debug.n_obstacles if debug else 0,
            'n_tracks': debug.n_tracks if debug else 0,
            'n_moving': debug.n_moving if debug else 0,
            'safety_state': safety.state if safety else '',
            'estop': int(bool(safety.estop_active)) if safety else 0,
            'distance_to_goal': _round(result.distance_to_goal),
            'compute_ms': round(debug.compute_time * 1000.0, 2) if debug else 0.0,
            'notes': (result.status or '')[:200],
        }
        try:
            self._writer.writerow(row)
            self._rows_since_flush += 1
            if self._rows_since_flush >= max(1, self.config.flush_every):
                self._handle.flush()
                self._rows_since_flush = 0
        except (OSError, ValueError) as exc:
            self._disable('could not write log row: {}'.format(exc))

    # ------------------------------------------------------------------
    def close(self) -> None:
        if self._handle is not None:
            try:
                self._handle.flush()
                self._handle.close()
            except OSError:
                pass
        self._handle = None
        self._writer = None

    def _disable(self, reason: str) -> None:
        self.close()
        self._disabled_reason = reason

    @property
    def disabled_reason(self) -> str:
        return self._disabled_reason


def _round(value: Optional[float], digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    try:
        if value != value or value in (float('inf'), float('-inf')):
            return None            # CSV readers handle an empty cell better than "inf"
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None
