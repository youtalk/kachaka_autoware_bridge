# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Minimal rigid-transform helpers on (translation, quaternion) pairs.

Quaternions are (x, y, z, w); translations (x, y, z). Used by the
map_to_odom_adapter (synthesize map->odom from the EKF's map->base_link,
AMCL-style) and the localization_monitor (compose Kachaka's raw map->odom with
odom->base_link). Pure Python so it unit-tests without ROS.
"""

from __future__ import annotations

import math

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


def quat_multiply(q1: Quat, q2: Quat) -> Quat:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    # v' = v + w*t + qv x t with t = 2 * (qv x v) (cheaper than two products).
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_to_yaw(q: Quat) -> float:
    """Yaw (rad) about +Z. Assumes a unit quaternion (ROS message quaternions are)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def invert(translation: Vec3, q: Quat) -> tuple[Vec3, Quat]:
    qi = quat_conjugate(q)
    ti = quat_rotate(qi, translation)
    return ((-ti[0], -ti[1], -ti[2]), qi)


def compose(t1: Vec3, q1: Quat, t2: Vec3, q2: Quat) -> tuple[Vec3, Quat]:
    """T1 * T2 (apply T2 first, then T1)."""
    r = quat_rotate(q1, t2)
    return ((t1[0] + r[0], t1[1] + r[1], t1[2] + r[2]), quat_multiply(q1, q2))


def map_to_odom(
    map_base_t: Vec3, map_base_q: Quat, odom_base_t: Vec3, odom_base_q: Quat
) -> tuple[Vec3, Quat]:
    """map->odom = (map->base) * (odom->base)^-1 -- the AMCL correction
    transform: composing it with the live odom->base reproduces map->base."""
    inv_t, inv_q = invert(odom_base_t, odom_base_q)
    return compose(map_base_t, map_base_q, inv_t, inv_q)
