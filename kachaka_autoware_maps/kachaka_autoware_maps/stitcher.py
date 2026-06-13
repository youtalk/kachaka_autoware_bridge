# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure accumulation logic for the loop-stitched pointcloud map (no ROS imports).

scripts/stitch_map drives a CloudStitcher: each OS-1 cloud arrives with its
map<-sensor transform (from Kachaka SLAM TF + the URDF chain). The stitcher
gates keyframes by sensor motion, range-filters in the sensor frame (near
points are robot/shelf self-hits), transforms to map, and voxel-deduplicates
so memory stays bounded over a multi-lap capture.
"""

from __future__ import annotations

import math

import numpy as np


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_rotation(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


class CloudStitcher:
    def __init__(
        self,
        voxel_size: float = 0.05,
        min_range: float = 0.8,
        max_range: float = 20.0,
        keyframe_translation: float = 0.15,
        keyframe_yaw: float = 0.2,
    ) -> None:
        if voxel_size <= 0.0:
            raise ValueError(f"voxel_size must be > 0, got {voxel_size}")
        if min_range >= max_range:
            raise ValueError(f"min_range {min_range} must be < max_range {max_range}")
        self._voxel_size = float(voxel_size)
        self._min_range = float(min_range)
        self._max_range = float(max_range)
        self._keyframe_translation = float(keyframe_translation)
        self._keyframe_yaw = float(keyframe_yaw)
        self._voxels: dict[tuple[int, int, int], tuple[float, float, float, float]] = {}
        self._last_key_pose: tuple[float, float, float] | None = None
        self.n_frames_added = 0

    def should_add(self, x: float, y: float, yaw: float) -> bool:
        """Keyframe gate on SENSOR pose: accept the first frame, then only
        frames whose pose moved past the translation or yaw threshold."""
        if self._last_key_pose is None:
            return True
        lx, ly, lyaw = self._last_key_pose
        return (
            math.hypot(x - lx, y - ly) >= self._keyframe_translation
            or abs(wrap_pi(yaw - lyaw)) >= self._keyframe_yaw
        )

    def add_cloud(self, xyzi: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> int:
        """Accumulate one cloud. ``xyzi`` is (N, 4) in the SENSOR frame;
        ``rotation``/``translation`` map sensor coordinates into map. Applies
        the keyframe gate itself; returns the number of NEW voxels added (0
        when gated out). A cloud that passes the keyframe gate advances the
        keyframe pose and frame count even if it is empty or fully range-
        filtered (it contributes no voxels but still marks a keyframe)."""
        x, y = float(translation[0]), float(translation[1])
        yaw = yaw_from_rotation(rotation)
        if not self.should_add(x, y, yaw):
            return 0
        self._last_key_pose = (x, y, yaw)
        self.n_frames_added += 1
        pts = np.asarray(xyzi, dtype=np.float32).reshape(-1, 4)
        if pts.shape[0] == 0:
            return 0
        rng = np.linalg.norm(pts[:, :3], axis=1)
        pts = pts[(rng >= self._min_range) & (rng <= self._max_range)]
        if pts.shape[0] == 0:
            return 0
        xyz_map = pts[:, :3] @ np.asarray(rotation, dtype=np.float32).T + np.asarray(
            translation, dtype=np.float32
        )
        keys = np.floor(xyz_map.astype(np.float64) / self._voxel_size).astype(np.int64)
        added = 0
        for key, p, intensity in zip(map(tuple, keys), xyz_map, pts[:, 3]):
            if key not in self._voxels:
                self._voxels[key] = (float(p[0]), float(p[1]), float(p[2]), float(intensity))
                added += 1
        return added

    @property
    def n_points(self) -> int:
        return len(self._voxels)

    def points(self) -> list[tuple[float, float, float, float]]:
        return list(self._voxels.values())
