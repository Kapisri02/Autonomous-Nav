"""LiDAR filtering: reject invalid beams, suppress speckle, convert to points.

The filter is deliberately conservative: a beam is dropped only when it is
provably unusable (NaN, Inf, outside the sensor's own reported limits, or an
isolated speckle). Dropping a real obstacle is far more dangerous than keeping a
noisy one, so every rejection rule is narrow and counted for diagnostics.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

from .geometry import normalize_angle, polar_to_cartesian
from .params import LidarConfig
from .types import FilteredScan, ScanData

_REJECT_KEYS = ('nan', 'inf', 'below_min', 'above_max', 'out_of_fov', 'speckle', 'decimated')


class LidarFilter:
    """Stateless (per-scan) LiDAR pre-processor."""

    def __init__(self, config: Optional[LidarConfig] = None) -> None:
        self.config = config or LidarConfig()

    # ------------------------------------------------------------------
    def filter(self, scan: ScanData) -> FilteredScan:
        cfg = self.config
        out = FilteredScan(stamp=scan.stamp, raw_count=scan.count)
        out.rejected = {key: 0 for key in _REJECT_KEYS}
        if scan.count == 0:
            return out

        lower = max(cfg.min_range, scan.range_min if scan.range_min > 0.0 else cfg.min_range)
        upper = min(cfg.max_usable_range, cfg.max_range,
                    scan.range_max if scan.range_max > 0.0 else cfg.max_range)
        half_fov = min(abs(cfg.field_of_view) * 0.5, math.pi)

        # Pass 1: validity, range window and field of view.
        kept_idx: List[int] = []
        kept_rng: List[float] = []
        kept_ang: List[float] = []
        for i, raw in enumerate(scan.ranges):
            try:
                r = float(raw)
            except (TypeError, ValueError):
                out.rejected['nan'] += 1
                continue
            if r != r:                      # NaN
                out.rejected['nan'] += 1
                continue
            if math.isinf(r):
                out.rejected['inf'] += 1
                continue
            if r < lower:
                out.rejected['below_min'] += 1
                continue
            if r > upper:
                out.rejected['above_max'] += 1
                continue
            angle = normalize_angle(scan.angle_at(i))
            if abs(angle) > half_fov:
                out.rejected['out_of_fov'] += 1
                continue
            kept_idx.append(i)
            kept_rng.append(r)
            kept_ang.append(angle)

        if not kept_idx:
            return out

        # Pass 2: median smoothing over adjacent *surviving* beams, applied in
        # the safety-biased direction only (see LidarConfig.median_window).
        smoothed = self._median_filter(kept_rng, cfg.median_window)
        smoothed = [min(raw, med) for raw, med in zip(kept_rng, smoothed)]

        # Pass 3: speckle rejection using Cartesian neighbourhood support.
        points = [polar_to_cartesian(a, r) for a, r in zip(kept_ang, smoothed)]
        keep_flags = self._speckle_mask(points, smoothed, abs(scan.angle_increment))

        step = max(1, int(cfg.decimation))
        survivors = 0
        for j, keep in enumerate(keep_flags):
            if not keep:
                out.rejected['speckle'] += 1
                continue
            if survivors % step != 0:
                out.rejected['decimated'] += 1
                survivors += 1
                continue
            survivors += 1
            out.points.append(points[j])
            out.angles.append(kept_ang[j])
            out.ranges.append(smoothed[j])
        return out

    # ------------------------------------------------------------------
    @staticmethod
    def _median_filter(values: Sequence[float], window: int) -> List[float]:
        if window <= 1 or len(values) < window:
            return list(values)
        half = window // 2
        n = len(values)
        out: List[float] = []
        for i in range(n):
            lo = max(0, i - half)
            hi = min(n, i + half + 1)
            chunk = sorted(values[lo:hi])
            out.append(chunk[len(chunk) // 2])
        return out

    def _speckle_mask(self, points: Sequence, ranges: Sequence[float],
                      angle_increment: float = 0.0) -> List[bool]:
        cfg = self.config
        n = len(points)
        if cfg.min_neighbours <= 0 or n == 0:
            return [True] * n
        window = max(1, int(cfg.neighbour_window))
        flags: List[bool] = []
        for i in range(n):
            if ranges[i] <= cfg.speckle_min_range:
                flags.append(True)      # never discard a close return
                continue
            xi, yi = points[i]
            # Support radius adapts to the beam spacing at this range, so the
            # filter behaves the same on a 360-beam scan and a decimated one.
            radius = max(cfg.neighbour_radius,
                         cfg.neighbour_spacing_factor * ranges[i] * angle_increment * window)
            radius_sq = radius * radius
            support = 0
            lo = max(0, i - window)
            hi = min(n, i + window + 1)
            for j in range(lo, hi):
                if j == i:
                    continue
                xj, yj = points[j]
                dx, dy = xj - xi, yj - yi
                if dx * dx + dy * dy <= radius_sq:
                    support += 1
                    if support >= cfg.min_neighbours:
                        break
            flags.append(support >= cfg.min_neighbours)
        return flags


def sector_minimum(scan: FilteredScan, center: float, half_width: float) -> float:
    """Smallest range within ``+/- half_width`` of bearing ``center``.

    Used by the safety supervisor and the recovery behaviour, and mirrors the
    front/left/right sector logic of the preserved baseline controller.
    """
    best = float('inf')
    for angle, rng in zip(scan.angles, scan.ranges):
        if abs(normalize_angle(angle - center)) <= half_width and rng < best:
            best = rng
    return best
