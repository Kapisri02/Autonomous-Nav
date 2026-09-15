"""Obstacle detection: adaptive-threshold clustering of filtered scan points.

Adjacent LiDAR points belong to the same object when the gap between them is
small relative to the beam spacing at that range. The split threshold therefore
grows with range (``base_threshold + range_factor * range``), which keeps a wall
at 4 m from shattering into dozens of fragments while still separating two
people standing 0.4 m apart at 1 m.

Each cluster is reduced to a *bounding disc* (centre + radius). The disc is the
representation every downstream stage uses, because disc/disc tests are cheap
enough to run over hundreds of candidate rollout poses in pure Python. The
member points are retained so that the final footprint check can be exact.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from .geometry import centroid, bounding_radius, path_length
from .params import ClusterConfig
from .types import FilteredScan, Obstacle, Point


class ObstacleDetector:
    def __init__(self, config: Optional[ClusterConfig] = None) -> None:
        self.config = config or ClusterConfig()

    # ------------------------------------------------------------------
    def detect(self, scan: FilteredScan) -> List[Obstacle]:
        cfg = self.config
        n = scan.count
        if n == 0:
            return []

        groups = self._segment(scan)
        obstacles: List[Obstacle] = []
        for group in groups:
            if len(group) < cfg.min_points:
                continue
            extent = path_length(group)
            structure = extent >= cfg.structure_extent
            for obstacle in self._split_oversized(group, scan.stamp):
                obstacle.parent_extent = extent
                obstacle.is_structure = structure
                obstacles.append(obstacle)

        # CPU guard: when a scan is unusually rich, keep the closest clusters —
        # those are the ones that can actually reach the robot within the
        # planning horizon.
        if len(obstacles) > cfg.max_obstacles:
            obstacles.sort(key=lambda o: o.min_distance)
            obstacles = obstacles[:cfg.max_obstacles]
        return obstacles

    # ------------------------------------------------------------------
    def _segment(self, scan: FilteredScan) -> List[List[Point]]:
        cfg = self.config
        groups: List[List[Point]] = []
        current: List[Point] = [scan.points[0]]
        for i in range(1, scan.count):
            prev = scan.points[i - 1]
            cur = scan.points[i]
            gap = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            reference = min(scan.ranges[i], scan.ranges[i - 1])
            threshold = cfg.base_threshold + cfg.range_factor * reference
            if gap <= threshold:
                current.append(cur)
            else:
                groups.append(current)
                current = [cur]
        groups.append(current)

        # A 360-degree scan wraps around: merge the first and last group when the
        # scan is circular and the endpoints are close enough to be one object.
        if len(groups) > 1 and scan.count > 2:
            first, last = groups[0], groups[-1]
            gap = math.hypot(first[0][0] - last[-1][0], first[0][1] - last[-1][1])
            reference = min(scan.ranges[0], scan.ranges[-1])
            if gap <= cfg.base_threshold + cfg.range_factor * reference:
                groups[0] = last + first
                groups.pop()
        return groups

    def _split_oversized(self, points: Sequence[Point], stamp: float) -> List[Obstacle]:
        """Split a cluster whose extent exceeds ``max_cluster_radius``.

        A long wall is a single connected cluster, but a single huge disc around
        it would swallow free space the robot could legally drive through, so it
        is cut into fixed-size chunks whose discs hug the surface.
        """
        cfg = self.config
        center = centroid(points)
        radius = bounding_radius(points, center)
        if radius <= cfg.max_cluster_radius or len(points) < 2 * cfg.min_points:
            return [self._make_obstacle(points, stamp)]

        chunks: List[List[Point]] = []
        current: List[Point] = [points[0]]
        for p in points[1:]:
            trial = current + [p]
            c = centroid(trial)
            if bounding_radius(trial, c) > cfg.max_cluster_radius:
                chunks.append(current)
                current = [p]
            else:
                current = trial
        chunks.append(current)

        out: List[Obstacle] = []
        for chunk in chunks:
            if len(chunk) >= cfg.min_points:
                out.append(self._make_obstacle(chunk, stamp))
            elif out:
                # Fold a leftover tail into the previous chunk rather than losing it.
                merged = out[-1].points + list(chunk)
                out[-1] = self._make_obstacle(merged, stamp)
            else:
                out.append(self._make_obstacle(chunk, stamp))
        return out

    @staticmethod
    def _make_obstacle(points: Sequence[Point], stamp: float) -> Obstacle:
        pts = list(points)
        center = centroid(pts)
        radius = bounding_radius(pts, center)
        min_distance = min(math.hypot(x, y) for x, y in pts)
        return Obstacle(center=center, radius=radius, points=pts, stamp=stamp,
                        min_distance=min_distance)


def nearest_obstacle(obstacles: Sequence[Obstacle]) -> Optional[Obstacle]:
    best: Optional[Obstacle] = None
    for o in obstacles:
        if best is None or o.min_distance < best.min_distance:
            best = o
    return best


def obstacle_summary(obstacles: Sequence[Obstacle]) -> Tuple[int, float]:
    """``(count, nearest surface distance)`` — convenient for logging."""
    if not obstacles:
        return (0, float('inf'))
    return (len(obstacles), min(o.min_distance for o in obstacles))
