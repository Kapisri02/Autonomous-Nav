"""Risk-adaptive speed envelope.

Produces the maximum forward speed allowed this cycle as the smallest of three
independent caps:

1. **Risk cap** - ``nominal * speed_scale`` from the risk level (1.0 / 0.6 / 0.3 / 0.0).
2. **Braking cap** - ``sqrt(2 * a * (clearance - margin))``: guarantees the robot
   can always stop inside the free space it can actually see. This is the cap
   that keeps the robot safe when the risk assessment is optimistic.
3. **Acceleration cap** - current speed plus one control period of acceleration,
   so the command is dynamically achievable.

The result is low-pass filtered *upward only*: speeding up is gradual, slowing
down takes effect on the very next cycle.
"""

from __future__ import annotations

import math
from typing import Optional

from .geometry import clamp
from .params import RobotConfig, VelocityConfig


class AdaptiveVelocity:
    def __init__(self, config: Optional[VelocityConfig] = None,
                 robot: Optional[RobotConfig] = None) -> None:
        self.config = config or VelocityConfig()
        self.robot = robot or RobotConfig()
        self._previous_limit = self.config.nominal_speed

    def reset(self) -> None:
        self._previous_limit = self.config.nominal_speed

    @property
    def last_limit(self) -> float:
        return self._previous_limit

    # ------------------------------------------------------------------
    def speed_limit(self, speed_scale: float, clearance: float,
                    current_speed: float = 0.0, dt: Optional[float] = None) -> float:
        cfg = self.config
        scale = clamp(speed_scale, 0.0, 1.0)
        if scale <= 0.0:
            self._previous_limit = 0.0
            return 0.0

        brake_cap = self.braking_cap(clearance)
        limit = min(cfg.nominal_speed * scale, brake_cap, self.robot.max_linear_speed)

        # Keep making progress unless we are genuinely close to something.
        if limit < cfg.min_progress_speed and brake_cap > cfg.min_progress_speed:
            limit = cfg.min_progress_speed

        if dt is not None and dt > 0.0:
            limit = min(limit, abs(current_speed) + self.robot.max_linear_accel * dt)

        if limit > self._previous_limit:
            limit = self._previous_limit + clamp(cfg.smoothing, 0.0, 1.0) * (
                limit - self._previous_limit)
        self._previous_limit = max(0.0, limit)
        return self._previous_limit

    # ------------------------------------------------------------------
    def braking_cap(self, clearance: float) -> float:
        """Largest speed from which the robot can still stop inside ``clearance``."""
        cfg = self.config
        if math.isinf(clearance):
            return self.robot.max_linear_speed
        usable = clearance - cfg.brake_margin
        if usable <= 0.0:
            return 0.0
        return math.sqrt(2.0 * self.robot.max_linear_decel * usable)
