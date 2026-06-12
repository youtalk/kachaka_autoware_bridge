# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import numpy as np
import pytest

from kachaka_autoware_maps.stitcher import CloudStitcher, wrap_pi, yaw_from_rotation


def _identity():
    return np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)


def _yaw_rotation(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)


def test_wrap_pi() -> None:
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1, abs=1e-9)
    assert wrap_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1, abs=1e-9)


def test_yaw_from_rotation() -> None:
    assert yaw_from_rotation(_yaw_rotation(0.7)) == pytest.approx(0.7, abs=1e-6)


def test_first_cloud_is_always_a_keyframe() -> None:
    st = CloudStitcher()
    assert st.should_add(0.0, 0.0, 0.0) is True


def test_keyframe_gate_blocks_stationary_frames() -> None:
    st = CloudStitcher(keyframe_translation=0.15, keyframe_yaw=0.2)
    r, t = _identity()
    cloud = np.array([[2.0, 0.0, 0.0, 1.0]], dtype=np.float32)
    assert st.add_cloud(cloud, r, t) == 1
    # Same pose again: gated out, nothing accumulates.
    assert st.add_cloud(cloud, r, t) == 0
    assert st.n_frames_added == 1
    # Move past the translation gate: accepted.
    t2 = np.array([0.2, 0.0, 0.0], dtype=np.float32)
    assert st.should_add(0.2, 0.0, 0.0) is True
    st.add_cloud(cloud, r, t2)
    assert st.n_frames_added == 2


def test_keyframe_gate_accepts_pure_rotation() -> None:
    st = CloudStitcher(keyframe_translation=0.15, keyframe_yaw=0.2)
    r, t = _identity()
    st.add_cloud(np.array([[2.0, 0.0, 0.0, 1.0]], dtype=np.float32), r, t)
    assert st.should_add(0.0, 0.0, 0.3) is True
    assert st.should_add(0.0, 0.0, 0.1) is False


def test_range_filter_drops_near_and_far() -> None:
    st = CloudStitcher(min_range=0.8, max_range=20.0)
    r, t = _identity()
    cloud = np.array(
        [
            [0.3, 0.0, 0.0, 1.0],   # too near (robot/shelf self-hit)
            [5.0, 0.0, 0.0, 1.0],   # kept
            [30.0, 0.0, 0.0, 1.0],  # too far
        ],
        dtype=np.float32,
    )
    assert st.add_cloud(cloud, r, t) == 1
    assert st.n_points == 1


def test_voxel_dedupe_keeps_one_point_per_voxel() -> None:
    st = CloudStitcher(voxel_size=0.1, min_range=0.0, max_range=100.0)
    r, t = _identity()
    cloud = np.array(
        [
            [5.00, 0.00, 0.0, 1.0],
            [5.01, 0.02, 0.0, 2.0],  # same 0.1 m voxel as above
            [5.20, 0.00, 0.0, 3.0],  # different voxel
        ],
        dtype=np.float32,
    )
    assert st.add_cloud(cloud, r, t) == 2
    assert st.n_points == 2


def test_transform_is_applied_before_voxelization() -> None:
    st = CloudStitcher(voxel_size=0.1, min_range=0.0, max_range=100.0)
    # Sensor at (10, 0) rotated +90 deg: sensor-frame (2, 0) -> map (10, 2).
    r = _yaw_rotation(math.pi / 2.0)
    t = np.array([10.0, 0.0, 0.0], dtype=np.float32)
    st.add_cloud(np.array([[2.0, 0.0, 0.0, 7.0]], dtype=np.float32), r, t)
    pts = st.points()
    assert len(pts) == 1
    x, y, z, intensity = pts[0]
    assert x == pytest.approx(10.0, abs=1e-5)
    assert y == pytest.approx(2.0, abs=1e-5)
    assert intensity == pytest.approx(7.0)


def test_empty_cloud_is_a_noop_but_counts_as_keyframe() -> None:
    st = CloudStitcher()
    r, t = _identity()
    assert st.add_cloud(np.zeros((0, 4), dtype=np.float32), r, t) == 0
    assert st.n_frames_added == 1
    assert st.n_points == 0


def test_invalid_params_raise() -> None:
    with pytest.raises(ValueError):
        CloudStitcher(voxel_size=0.0)
    with pytest.raises(ValueError):
        CloudStitcher(min_range=5.0, max_range=1.0)
