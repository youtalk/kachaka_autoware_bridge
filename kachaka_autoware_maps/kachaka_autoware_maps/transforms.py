# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure SE(3) transform helpers for the maps package (no ROS imports).

Shared by scripts/stitch_map and scripts/snapshot_map, which both turn a
geometry_msgs/TransformStamped (Kachaka SLAM TF + the URDF chain) into a
rotation matrix + translation to project sensor clouds into the map frame.
"""

from __future__ import annotations

import numpy as np


def quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Right-handed quaternion (x, y, z, w) -> 3x3 rotation matrix (float32)."""
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float32,
    )


def tf_to_matrix(transform_stamped):
    """Return (R 3x3, t 3) from a geometry_msgs/TransformStamped."""
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    r = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
    return r, np.array([t.x, t.y, t.z], dtype=np.float32)
