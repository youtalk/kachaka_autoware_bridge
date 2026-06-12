# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import pytest

from kachaka_autoware_bridge.se3 import (
    compose,
    invert,
    map_to_odom,
    quat_conjugate,
    quat_multiply,
    quat_rotate,
    quat_to_yaw,
)

IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _assert_close(a, b, tol=1e-9) -> None:
    for x, y in zip(a, b):
        assert x == pytest.approx(y, abs=tol)


def test_quat_multiply_identity() -> None:
    q = _yaw_quat(0.7)
    _assert_close(quat_multiply(q, IDENTITY_Q), q)
    _assert_close(quat_multiply(IDENTITY_Q, q), q)


def test_quat_rotate_90deg_yaw() -> None:
    q = _yaw_quat(math.pi / 2.0)
    _assert_close(quat_rotate(q, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


def test_quat_conjugate_undoes_rotation() -> None:
    q = _yaw_quat(0.4)
    v = (1.0, 2.0, 3.0)
    _assert_close(quat_rotate(quat_conjugate(q), quat_rotate(q, v)), v)


def test_quat_to_yaw_round_trip() -> None:
    assert quat_to_yaw(_yaw_quat(1.1)) == pytest.approx(1.1, abs=1e-9)
    assert quat_to_yaw(_yaw_quat(-2.5)) == pytest.approx(-2.5, abs=1e-9)


def test_invert_round_trip() -> None:
    t, q = (1.0, 2.0, 0.5), _yaw_quat(0.9)
    ti, qi = invert(t, q)
    t_id, q_id = compose(t, q, ti, qi)
    _assert_close(t_id, (0.0, 0.0, 0.0))
    _assert_close(q_id[:3], (0.0, 0.0, 0.0))
    assert abs(q_id[3]) == pytest.approx(1.0)


def test_compose_translation_then_rotation() -> None:
    # T1 = +90 deg yaw at origin, T2 = translate (1, 0, 0):
    # composed maps origin to (0, 1, 0).
    t, q = compose((0.0, 0.0, 0.0), _yaw_quat(math.pi / 2.0), (1.0, 0.0, 0.0), IDENTITY_Q)
    _assert_close(t, (0.0, 1.0, 0.0))


def test_map_to_odom_reconstructs_map_base() -> None:
    # Given map->base and odom->base, map->odom must satisfy
    # (map->odom) * (odom->base) == map->base.
    map_base_t, map_base_q = (3.0, 1.0, 0.0), _yaw_quat(1.2)
    odom_base_t, odom_base_q = (0.5, -0.2, 0.0), _yaw_quat(0.3)
    mo_t, mo_q = map_to_odom(map_base_t, map_base_q, odom_base_t, odom_base_q)
    rec_t, rec_q = compose(mo_t, mo_q, odom_base_t, odom_base_q)
    _assert_close(rec_t, map_base_t, tol=1e-9)
    assert quat_to_yaw(rec_q) == pytest.approx(quat_to_yaw(map_base_q), abs=1e-9)


def test_map_to_odom_identity_odom() -> None:
    # If odom->base is identity, map->odom IS map->base.
    t, q = map_to_odom((2.0, 0.0, 0.0), _yaw_quat(0.5), (0.0, 0.0, 0.0), IDENTITY_Q)
    _assert_close(t, (2.0, 0.0, 0.0))
    assert quat_to_yaw(q) == pytest.approx(0.5, abs=1e-9)


def test_quat_multiply_noncommutative() -> None:
    # q_x * q_z != q_z * q_x; exercises the cross-axis sign terms that the
    # pure-Z tests leave at zero.
    s = math.sin(math.pi / 8)
    c = math.cos(math.pi / 8)
    q_x = (s, 0.0, 0.0, c)  # 45 deg about X
    q_z = (0.0, 0.0, s, c)  # 45 deg about Z
    r1 = quat_multiply(q_x, q_z)
    r2 = quat_multiply(q_z, q_x)
    assert r1 != pytest.approx(r2, abs=1e-9)


def test_quat_rotate_full_3d() -> None:
    # 90 deg about X maps (0, 1, 0) -> (0, 0, 1); exercises the tz term.
    q_x90 = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    _assert_close(quat_rotate(q_x90, (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0))
