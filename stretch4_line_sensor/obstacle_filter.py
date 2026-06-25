"""Spatial and temporal filtering for line sensor obstacle point clouds."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from stretch4_line_sensor.transform_utils import transform_point

try:
    import open3d as o3d

    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


@dataclass
class _ObstacleTrack:
    tracking_centroid: np.ndarray
    points: np.ndarray
    consecutive_hits: int = 1
    last_seen: float = field(default_factory=time.time)


class ObstacleFilter:
    """
    Filter obstacle candidates with spatial clustering and temporal persistence.

    Pipeline:
    1. DBSCAN clustering to remove isolated noise points
    2. Drop clusters smaller than min_cluster_width_m
    3. Match clusters to tracks across frames (optionally in odom)
    4. Publish only tracks seen for min_consecutive_frames in a row

    When base_to_tracking is provided, cluster centroids are transformed out
    of base_link before matching so static obstacles stay fixed while the
    robot moves. Published points remain in the input (base_link) frame.
    """

    def __init__(
        self,
        cluster_eps: float = 0.03,
        cluster_min_points: int = 10,
        min_cluster_width_m: float = 0.01,
        track_match_thresh_m: float = 0.10,
        track_max_age_s: float = 1.0,
        min_consecutive_frames: int = 3,
    ):
        self.cluster_eps = cluster_eps
        self.cluster_min_points = cluster_min_points
        self.min_cluster_width_m = min_cluster_width_m
        self.track_match_thresh_m = track_match_thresh_m
        self.track_max_age_s = track_max_age_s
        self.min_consecutive_frames = max(1, int(min_consecutive_frames))
        self._tracks: dict[int, _ObstacleTrack] = {}
        self._next_track_id = 0

    def filter(
        self,
        points: np.ndarray,
        base_to_tracking: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return obstacle points that pass spatial and temporal filters."""
        if len(points) == 0:
            self._prune_stale_tracks(time.time())
            return np.zeros((0, 3))

        if not HAS_OPEN3D:
            return points

        clusters = self._spatial_cluster(points)
        return self._apply_temporal_filter(clusters, base_to_tracking)

    def _spatial_cluster(self, points: np.ndarray) -> list[np.ndarray]:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self.cluster_eps,
                min_points=self.cluster_min_points,
                print_progress=False,
            )
        )
        if len(labels) == 0 or labels.max() < 0:
            return []

        clusters: list[np.ndarray] = []
        for label in range(labels.max() + 1):
            cluster_pts = points[labels == label]
            if len(cluster_pts) == 0:
                continue

            extent = cluster_pts.max(axis=0) - cluster_pts.min(axis=0)
            if float(np.max(extent)) < self.min_cluster_width_m:
                continue
            clusters.append(cluster_pts)

        return clusters

    def _cluster_tracking_centroid(
        self,
        cluster: np.ndarray,
        base_to_tracking: np.ndarray | None,
    ) -> np.ndarray:
        centroid_base = np.mean(cluster, axis=0)
        if base_to_tracking is None:
            return centroid_base
        return transform_point(base_to_tracking, centroid_base)

    def _apply_temporal_filter(
        self,
        clusters: list[np.ndarray],
        base_to_tracking: np.ndarray | None = None,
    ) -> np.ndarray:
        now = time.time()
        matched_track_ids: set[int] = set()
        tracking_centroids = [
            self._cluster_tracking_centroid(cluster, base_to_tracking)
            for cluster in clusters
            if len(cluster) > 0
        ]

        candidates: list[tuple[float, int, int]] = []
        active_ids = list(self._tracks.keys())
        for cluster_idx, centroid in enumerate(tracking_centroids):
            for track_id in active_ids:
                dist = float(
                    np.linalg.norm(
                        centroid - self._tracks[track_id].tracking_centroid,
                    )
                )
                if dist < self.track_match_thresh_m:
                    candidates.append((dist, cluster_idx, track_id))

        candidates.sort(key=lambda item: item[0])
        cluster_to_track: dict[int, int] = {}
        used_tracks: set[int] = set()

        for _, cluster_idx, track_id in candidates:
            if cluster_idx in cluster_to_track or track_id in used_tracks:
                continue
            cluster_to_track[cluster_idx] = track_id
            used_tracks.add(track_id)

        confirmed_clusters: list[np.ndarray] = []

        for cluster_idx, cluster in enumerate(clusters):
            tracking_centroid = tracking_centroids[cluster_idx]
            if cluster_idx in cluster_to_track:
                track_id = cluster_to_track[cluster_idx]
                track = self._tracks[track_id]
                track.tracking_centroid = tracking_centroid
                track.points = cluster
                track.consecutive_hits += 1
                track.last_seen = now
                matched_track_ids.add(track_id)
            else:
                track_id = self._next_track_id
                self._next_track_id += 1
                self._tracks[track_id] = _ObstacleTrack(
                    tracking_centroid=tracking_centroid,
                    points=cluster,
                    consecutive_hits=1,
                    last_seen=now,
                )
                matched_track_ids.add(track_id)

        for track_id, track in list(self._tracks.items()):
            if track_id not in matched_track_ids:
                track.consecutive_hits = 0

        self._prune_stale_tracks(now)

        for track_id in matched_track_ids:
            track = self._tracks.get(track_id)
            if track is None:
                continue
            if track.consecutive_hits >= self.min_consecutive_frames:
                confirmed_clusters.append(track.points)

        if not confirmed_clusters:
            return np.zeros((0, 3))
        return np.vstack(confirmed_clusters)

    def _prune_stale_tracks(self, now: float) -> None:
        stale_ids = [
            track_id
            for track_id, track in self._tracks.items()
            if now - track.last_seen > self.track_max_age_s
        ]
        for track_id in stale_ids:
            del self._tracks[track_id]
