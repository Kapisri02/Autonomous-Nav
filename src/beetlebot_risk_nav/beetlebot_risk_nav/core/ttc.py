"""Time-to-collision between the robot's disc and an obstacle's disc.

With both bodies modelled as discs under constant relative velocity, contact
happens when ``|p + v t| = R``. Expanding gives a quadratic in ``t`` whose
smallest non-negative root is the time to collision; no root means the obstacle
passes by. This is exact for the disc model and costs a handful of
multiplications, which is what makes it affordable inside the candidate loop.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from .types import Track

INF = float('inf')


def disc_ttc(px: float, py: float, vx: float, vy: float, combined_radius: float) -> float:
    """TTC for a disc at ``(px, py)`` closing with relative velocity ``(vx, vy)``.

    Returns ``0.0`` when the discs already overlap and ``inf`` when they never
    touch on the current relative course.
    """
    r2 = combined_radius * combined_radius
    c = px * px + py * py - r2
    if c <= 0.0:
        return 0.0
    a = vx * vx + vy * vy
    if a <= 1e-12:
        return INF                      # no relative motion: never collides
    b = 2.0 * (px * vx + py * vy)
    if b >= 0.0:
        return INF                      # separating (or tangential): never closes
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return INF                      # passes alongside
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    candidates = [t for t in (t1, t2) if t >= 0.0]
    return min(candidates) if candidates else INF


def track_ttc(track: Track, robot_radius: float) -> float:
    """TTC between the robot disc and a track, using the track's relative velocity."""
    return disc_ttc(track.rel_x, track.rel_y, track.rel_vx, track.rel_vy,
                    robot_radius + track.radius)


def minimum_ttc(tracks: Sequence[Track], robot_radius: float
                ) -> Tuple[float, Optional[int]]:
    """Smallest TTC over all tracks and the id of the track responsible."""
    best = INF
    best_id: Optional[int] = None
    for track in tracks:
        t = track_ttc(track, robot_radius)
        if t < best:
            best = t
            best_id = track.track_id
    return (best, best_id)


def braking_distance(speed: float, decel: float) -> float:
    """Distance needed to stop from ``speed`` at ``decel`` (m/s^2)."""
    if decel <= 0.0:
        return INF
    return (speed * speed) / (2.0 * decel)


def stopping_time(speed: float, decel: float) -> float:
    if decel <= 0.0:
        return INF
    return abs(speed) / decel
