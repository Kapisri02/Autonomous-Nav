"""Obstacle tracking and relative-velocity estimation.

A global-nearest-neighbour associator feeds one filter per track. Position is
smoothed with an alpha-beta blend; **velocity is measured as displacement across
a time window** rather than from consecutive scans.

That choice is deliberate and was driven by measurement. Scan-to-scan velocity
on clustered LiDAR data is dominated by segmentation noise: as the robot moves,
the boundaries between clusters covering one extended surface slide along it, so
a centroid appears to move even though the wall is bolted to the floor. In
simulation this produced phantom velocities approaching 1 m/s on static walls,
which in turn produced phantom risk and collapsed the speed limit. Windowed
displacement cancels that jitter (it reverses direction from scan to scan) while
genuine motion accumulates, and the ``is_moving`` flag derived from it is what
decides whether an obstacle's own velocity is believed at all.

Tracks are maintained in the world (odom) frame when a robot pose is supplied,
so that robot motion is not mistaken for obstacle motion. The ``rel_*`` fields
on each track give the robot-frame view used by TTC and risk estimation.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from .geometry import clamp, inverse_transform_point, rotate, transform_point
from .params import TrackerConfig
from .types import Obstacle, Track

Pose = Tuple[float, float, float]


class ObstacleTracker:
    def __init__(self, config: Optional[TrackerConfig] = None) -> None:
        self.config = config or TrackerConfig()
        self.tracks: Dict[int, Track] = {}
        self._next_id = 1
        self._last_stamp: Optional[float] = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self.tracks.clear()
        self._last_stamp = None

    # ------------------------------------------------------------------
    def update(self, obstacles: Sequence[Obstacle], stamp: float,
               robot_pose: Pose = (0.0, 0.0, 0.0),
               robot_velocity: Tuple[float, float] = (0.0, 0.0)) -> List[Track]:
        """Advance the tracker by one scan and return the live tracks."""
        cfg = self.config
        dt = 0.0 if self._last_stamp is None else stamp - self._last_stamp
        if dt < 0.0 or dt > cfg.max_dt:
            # Time went backwards (bag loop / clock jump) or a long gap: velocity
            # estimates are meaningless across the discontinuity.
            self._invalidate_velocities()
            dt = 0.0
        self._last_stamp = stamp

        # Detections in the world frame.
        detections: List[Tuple[float, float, float, Obstacle]] = []
        for obs in obstacles:
            wx, wy = transform_point(obs.center, robot_pose)
            detections.append((wx, wy, obs.radius, obs))

        # Predict every track forward to this timestamp.
        for track in self.tracks.values():
            if track.velocity_valid and dt > 0.0:
                track.x += track.vx * dt
                track.y += track.vy * dt

        matches, unmatched_det, unmatched_trk = self._associate(detections)

        for track_id, det_index in matches.items():
            wx, wy, radius, obs = detections[det_index]
            self._update_track(self.tracks[track_id], wx, wy, radius, dt, stamp,
                               obs.is_structure)

        for det_index in unmatched_det:
            wx, wy, radius, obs = detections[det_index]
            self._spawn_track(wx, wy, radius, stamp, obs.is_structure)

        for track_id in unmatched_trk:
            track = self.tracks[track_id]
            track.misses += 1
            track.age += 1
            track.confidence = self._confidence(track)

        self._prune()
        self._refresh_relative(robot_pose, robot_velocity)
        return self.live_tracks()

    # ------------------------------------------------------------------
    def live_tracks(self) -> List[Track]:
        return sorted(self.tracks.values(), key=lambda t: t.rel_distance)

    def confirmed_tracks(self) -> List[Track]:
        return [t for t in self.live_tracks() if t.velocity_valid]

    def moving_tracks(self) -> List[Track]:
        return [t for t in self.live_tracks() if t.is_moving]

    # ------------------------------------------------------------------
    def _associate(self, detections: Sequence[Tuple[float, float, float, Obstacle]]):
        """Greedy global nearest neighbour with distance and size gating."""
        cfg = self.config
        pairs: List[Tuple[float, int, int]] = []
        for track_id, track in self.tracks.items():
            for j, (wx, wy, radius, _) in enumerate(detections):
                d = math.hypot(wx - track.x, wy - track.y)
                if d > cfg.association_gate:
                    continue
                if abs(radius - track.radius) > cfg.size_gate:
                    continue
                pairs.append((d, track_id, j))
        pairs.sort(key=lambda item: item[0])

        matches: Dict[int, int] = {}
        used_dets = set()
        used_trks = set()
        for _, track_id, det_index in pairs:
            if track_id in used_trks or det_index in used_dets:
                continue
            matches[track_id] = det_index
            used_trks.add(track_id)
            used_dets.add(det_index)

        unmatched_det = [j for j in range(len(detections)) if j not in used_dets]
        unmatched_trk = [tid for tid in self.tracks if tid not in used_trks]
        return matches, unmatched_det, unmatched_trk

    def _update_track(self, track: Track, wx: float, wy: float, radius: float,
                      dt: float, stamp: float, is_structure: bool = False) -> None:
        cfg = self.config
        residual_x = wx - track.x
        residual_y = wy - track.y

        track.x += cfg.alpha * residual_x
        track.y += cfg.alpha * residual_y

        track.history.append((stamp, track.x, track.y))
        cutoff = stamp - cfg.velocity_window
        while len(track.history) > 2 and track.history[0][0] < cutoff:
            track.history.pop(0)
        self._estimate_velocity(track, stamp)

        track.radius = 0.7 * track.radius + 0.3 * radius
        track.is_structure = is_structure
        track.hits += 1
        track.age += 1
        track.misses = 0
        track.stamp = stamp
        track.velocity_valid = (track.hits >= cfg.min_hits_for_velocity
                                and (stamp - track.history[0][0]) >= cfg.min_velocity_span)
        track.confidence = self._confidence(track)

    def _estimate_velocity(self, track: Track, stamp: float) -> None:
        """Velocity from displacement across the history window.

        The alpha-beta ``beta`` gain is applied as a low-pass on the *result*, so
        the estimate is smooth without reintroducing scan-to-scan noise.
        """
        cfg = self.config
        history = track.history
        if len(history) < 2:
            return
        t0, x0, y0 = history[0]
        span = stamp - t0
        if span < cfg.min_velocity_span:
            track.velocity_valid = False
            return
        measured_vx = (track.x - x0) / span
        measured_vy = (track.y - y0) / span
        speed = math.hypot(measured_vx, measured_vy)
        if speed > cfg.max_track_speed:
            scale = cfg.max_track_speed / speed
            measured_vx *= scale
            measured_vy *= scale
        blend = clamp(cfg.beta, 0.0, 1.0)
        track.vx += blend * (measured_vx - track.vx)
        track.vy += blend * (measured_vy - track.vy)

    def _spawn_track(self, wx: float, wy: float, radius: float, stamp: float,
                     is_structure: bool = False) -> Track:
        track = Track(track_id=self._next_id, x=wx, y=wy, radius=radius, stamp=stamp,
                      is_structure=is_structure)
        track.history.append((stamp, wx, wy))
        track.confidence = self._confidence(track)
        self.tracks[track.track_id] = track
        self._next_id += 1
        return track

    def _confidence(self, track: Track) -> float:
        """Confidence grows with consecutive hits and decays with misses."""
        cfg = self.config
        base = min(1.0, track.hits / float(max(1, cfg.min_hits_for_velocity)))
        penalty = 1.0 - (track.misses / float(cfg.max_misses + 1))
        return clamp(base * penalty, 0.0, 1.0)

    def _invalidate_velocities(self) -> None:
        for track in self.tracks.values():
            track.vx = 0.0
            track.vy = 0.0
            track.velocity_valid = False
            track.is_moving = False
            track.hits = 1
            track.history.clear()

    def _prune(self) -> None:
        dead = [tid for tid, t in self.tracks.items() if t.misses > self.config.max_misses]
        for tid in dead:
            del self.tracks[tid]

    def _refresh_relative(self, robot_pose: Pose, robot_velocity: Tuple[float, float]) -> None:
        """Express every track in the robot frame.

        The relative velocity is the *translational* difference between the
        obstacle and the robot, rotated into the robot frame. The robot's own
        rotation rate is deliberately excluded: it does not change the distance
        to an obstacle, and including it would make a spinning robot see phantom
        closing speeds.
        """
        v, _w = robot_velocity
        th = robot_pose[2]
        robot_vx = v * math.cos(th)
        robot_vy = v * math.sin(th)
        threshold = self.config.moving_speed_threshold
        for track in self.tracks.values():
            rel = inverse_transform_point((track.x, track.y), robot_pose)
            track.rel_x, track.rel_y = rel
            track.is_moving = (track.velocity_valid and not track.is_structure
                               and track.speed >= threshold)
            if track.is_moving:
                dvx, dvy = track.vx - robot_vx, track.vy - robot_vy
            else:
                dvx, dvy = -robot_vx, -robot_vy
            track.rel_vx, track.rel_vy = rotate((dvx, dvy), -th)


def tracks_to_discs(tracks: Sequence[Track]) -> List[Tuple[float, float, float]]:
    """Robot-frame ``(x, y, radius)`` discs — the form the collision checker uses."""
    return [(t.rel_x, t.rel_y, t.radius) for t in tracks]
