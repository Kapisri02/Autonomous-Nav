"""Deterministic synthetic LiDAR data for algorithm tests.

This is a **test fixture**, not a robot simulator: it builds the range array of a
``LaserScan`` from simple geometry so that filtering, clustering, tracking, risk
assessment and motion selection can be exercised against inputs whose correct
answer is known analytically. There is no robot model, no physics and no closed
loop here, and results obtained with it are *software test results only* - they
say nothing about physical robot behaviour.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from beetlebot_risk_nav.core.types import ScanData

Circle = Tuple[float, float, float]          # x, y, radius
Wall = Tuple[float, float, float, float]     # x1, y1, x2, y2


def ray_circle(angle: float, circle: Circle) -> Optional[float]:
    """Distance from the origin along ``angle`` to a circle, or None."""
    cx, cy, radius = circle
    dx, dy = math.cos(angle), math.sin(angle)
    b = -2.0 * (cx * dx + cy * dy)
    c = cx * cx + cy * cy - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for t in ((-b - sq) * 0.5, (-b + sq) * 0.5):
        if t > 0.0:
            return t
    return None


def ray_wall(angle: float, wall: Wall) -> Optional[float]:
    """Distance from the origin along ``angle`` to a line segment, or None."""
    ax, ay, bx, by = wall
    dx, dy = math.cos(angle), math.sin(angle)
    ex, ey = bx - ax, by - ay
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    fx, fy = ax, ay
    t = (fx * ey - fy * ex) / denom
    u = (fx * dy - fy * dx) / denom
    if 0.0 <= u <= 1.0 and t > 0.0:
        return t
    return None


def make_scan(stamp: float = 0.0, circles: Sequence[Circle] = (),
              walls: Sequence[Wall] = (), n_beams: int = 360,
              range_min: float = 0.10, range_max: float = 12.0,
              background: Optional[float] = None) -> ScanData:
    """Build a scan of ``n_beams`` covering 360 degrees, starting at -pi.

    Beams that hit nothing report ``inf``, which is what a real LiDAR driver
    publishes for a no-return, unless ``background`` gives a surrounding radius.
    """
    increment = 2.0 * math.pi / n_beams
    ranges: List[float] = []
    for i in range(n_beams):
        angle = -math.pi + i * increment
        best = background if background is not None else float('inf')
        for circle in circles:
            hit = ray_circle(angle, circle)
            if hit is not None and hit < best:
                best = hit
        for wall in walls:
            hit = ray_wall(angle, wall)
            if hit is not None and hit < best:
                best = hit
        if best < range_min:
            best = range_min
        ranges.append(best)
    return ScanData(stamp=stamp, angle_min=-math.pi, angle_increment=increment,
                    ranges=ranges, range_min=range_min, range_max=range_max,
                    frame_id='laser')


def room(x_min: float, y_min: float, x_max: float, y_max: float) -> List[Wall]:
    """Four walls of an axis-aligned rectangular room, in robot-relative coords."""
    return [
        (x_min, y_min, x_max, y_min),
        (x_max, y_min, x_max, y_max),
        (x_max, y_max, x_min, y_max),
        (x_min, y_max, x_min, y_min),
    ]


def corridor(half_width: float, length: float = 8.0) -> List[Wall]:
    """Two parallel walls running along +x, centred on the robot."""
    return [
        (-length * 0.5, -half_width, length * 0.5, -half_width),
        (-length * 0.5, half_width, length * 0.5, half_width),
    ]
