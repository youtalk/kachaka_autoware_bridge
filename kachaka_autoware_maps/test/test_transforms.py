# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import numpy as np
import pytest

from kachaka_autoware_maps.transforms import quat_to_rotation_matrix, tf_to_matrix


def test_identity_quaternion_gives_identity_matrix() -> None:
    r = quat_to_rotation_matrix(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(r, np.eye(3), atol=1e-6)


def test_yaw_90_about_z() -> None:
    # +90 deg about z maps sensor +x -> map +y, sensor +y -> map -x.
    s = math.sin(math.pi / 4.0)
    r = quat_to_rotation_matrix(0.0, 0.0, s, s)
    expected = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    np.testing.assert_allclose(r, expected, atol=1e-6)


class _FakeTransformStamped:
    """Duck-typed geometry_msgs/TransformStamped for a ROS-free unit test."""

    class _T:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class _Q:
        def __init__(self, x, y, z, w):
            self.x, self.y, self.z, self.w = x, y, z, w

    class _Tf:
        def __init__(self, t, q):
            self.translation = t
            self.rotation = q

    def __init__(self, tx, ty, tz, qx, qy, qz, qw):
        self.transform = self._Tf(self._T(tx, ty, tz), self._Q(qx, qy, qz, qw))


def test_tf_to_matrix_returns_rotation_and_translation() -> None:
    s = math.sin(math.pi / 4.0)
    r, t = tf_to_matrix(_FakeTransformStamped(1.0, 2.0, 3.0, 0.0, 0.0, s, s))
    expected_r = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    np.testing.assert_allclose(r, expected_r, atol=1e-6)
    np.testing.assert_allclose(t, np.array([1.0, 2.0, 3.0]), atol=1e-6)
    assert r.dtype == np.float32
    assert t.dtype == np.float32
