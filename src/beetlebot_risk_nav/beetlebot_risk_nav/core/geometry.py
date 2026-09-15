"""Pure-Python 2D geometry helpers.

Deliberately stdlib-only (``math``) so that every algorithmic module in this
package can be imported and tested on a machine without ROS or NumPy, and runs
unchanged on the robot.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]

TWO_PI = 2.0 * math.pi


def normalize_angle(angle: float) -> float:
    """Wrap ``angle`` to (-pi, pi]."""
    a = math.fmod(angle + math.pi, TWO_PI)
    if a <= 0.0:
        a += TWO_PI
    return a - math.pi


def angle_diff(a: float, b: float) -> float:
    """Shortest signed difference ``a - b``, wrapped to (-pi, pi]."""
    return normalize_angle(a - b)


def clamp(value: float, low: float, high: float) -> float:
    if low > high:
        low, high = high, low
    return low if value < low else (high if value > high else value)


def hypot(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


def polar_to_cartesian(angle: float, rng: float) -> Point:
    return (rng * math.cos(angle), rng * math.sin(angle))


def rotate(point: Point, theta: float) -> Point:
    c, s = math.cos(theta), math.sin(theta)
    x, y = point
    return (c * x - s * y, s * x + c * y)


def transform_point(point: Point, pose: Tuple[float, float, float]) -> Point:
    """Transform ``point`` from a frame described by ``pose`` into the parent frame."""
    px, py, pth = pose
    rx, ry = rotate(point, pth)
    return (px + rx, py + ry)


def inverse_transform_point(point: Point, pose: Tuple[float, float, float]) -> Point:
    """Express a parent-frame ``point`` in the local frame described by ``pose``."""
    px, py, pth = pose
    dx, dy = point[0] - px, point[1] - py
    return rotate((dx, dy), -pth)


def centroid(points: Sequence[Point]) -> Point:
    n = len(points)
    if n == 0:
        raise ValueError('centroid() of empty point set')
    sx = sy = 0.0
    for x, y in points:
        sx += x
        sy += y
    return (sx / n, sy / n)


def bounding_radius(points: Sequence[Point], center: Point) -> float:
    """Largest distance from ``center`` to any point (0.0 for an empty set)."""
    best = 0.0
    cx, cy = center
    for x, y in points:
        d = math.hypot(x - cx, y - cy)
        if d > best:
            best = d
    return best


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Shortest distance from point ``p`` to the segment ``ab``."""
    px, py = p
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = clamp(((px - ax) * vx + (py - ay) * vy) / denom, 0.0, 1.0)
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def polygon_contains(polygon: Sequence[Point], p: Point) -> bool:
    """Even-odd ray-casting test. Points exactly on the border may report either way."""
    x, y = p
    inside = False
    n = len(polygon)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            x_cross = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x_cross > x:
                inside = not inside
        j = i
    return inside


def polygon_point_distance(polygon: Sequence[Point], p: Point) -> float:
    """Distance from ``p`` to the polygon boundary; negative when ``p`` is inside."""
    n = len(polygon)
    if n == 0:
        return float('inf')
    if n == 1:
        return math.hypot(p[0] - polygon[0][0], p[1] - polygon[0][1])
    best = float('inf')
    for i in range(n):
        a = polygon[i]
        b = polygon[(i + 1) % n]
        d = point_segment_distance(p, a, b)
        if d < best:
            best = d
    return -best if polygon_contains(polygon, p) else best


def arc_pose(pose: Tuple[float, float, float], v: float, w: float,
             dt: float) -> Tuple[float, float, float]:
    """Exact unicycle integration of a constant ``(v, w)`` command over ``dt``."""
    x, y, th = pose
    if abs(w) < 1e-6:
        return (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, normalize_angle(th + w * dt))
    r = v / w
    th_new = th + w * dt
    x_new = x + r * (math.sin(th_new) - math.sin(th))
    y_new = y - r * (math.cos(th_new) - math.cos(th))
    return (x_new, y_new, normalize_angle(th_new))


def path_length(points: Iterable[Point]) -> float:
    total = 0.0
    prev = None
    for p in points:
        if prev is not None:
            total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return total


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (Z) extracted from a quaternion, matching tf2's convention."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return ``(x, y, z, w)`` for a rotation of ``yaw`` about Z."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def resample_polyline(points: Sequence[Point], spacing: float) -> List[Point]:
    """Resample a polyline at roughly uniform ``spacing`` (keeps first and last)."""
    if spacing <= 0.0 or len(points) < 2:
        return list(points)
    out: List[Point] = [points[0]]
    carry = 0.0
    for i in range(1, len(points)):
        ax, ay = points[i - 1]
        bx, by = points[i]
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 1e-9:
            continue
        pos = spacing - carry
        while pos < seg:
            t = pos / seg
            out.append((ax + t * (bx - ax), ay + t * (by - ay)))
            pos += spacing
        carry = (carry + seg) % spacing
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out
