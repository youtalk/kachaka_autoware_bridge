# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import pytest

from kachaka_autoware_bridge.centerline import Centerline, GoalPose


def _square():
    # Unit square (side 2, centred at origin), CCW, 4 vertices, perimeter 8.
    return Centerline([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)])


def test_total_length_is_perimeter():
    assert _square().total_length == pytest.approx(8.0)


def test_pose_at_walks_the_perimeter_with_forward_tangent():
    cl = _square()
    p0 = cl.pose_at(0.0)
    assert (p0.x, p0.y) == pytest.approx((-1.0, -1.0))
    assert p0.yaw == pytest.approx(0.0)
    assert (cl.pose_at(1.0).x, cl.pose_at(1.0).y) == pytest.approx((0.0, -1.0))
    assert (cl.pose_at(3.0).x, cl.pose_at(3.0).y) == pytest.approx((1.0, 0.0))
    assert cl.pose_at(3.0).yaw == pytest.approx(math.pi / 2)


def test_pose_at_wraps_modulo_total_length():
    cl = _square()
    assert (cl.pose_at(8.0).x, cl.pose_at(8.0).y) == pytest.approx((-1.0, -1.0))
    assert (cl.pose_at(9.0).x, cl.pose_at(9.0).y) == pytest.approx((0.0, -1.0))


def test_project_returns_arclength_and_lateral_error():
    cl = _square()
    s, err = cl.project(0.0, -1.25)
    assert s == pytest.approx(1.0)
    assert err == pytest.approx(0.25)


def test_project_on_centerline_has_zero_error():
    s, err = _square().project(0.0, -1.0)
    assert s == pytest.approx(1.0)
    assert err == pytest.approx(0.0, abs=1e-9)


def test_advance_wraps():
    cl = _square()
    assert cl.advance(7.0, 2.0) == pytest.approx(1.0)
    assert cl.advance(1.0, -2.0) == pytest.approx(7.0)


def test_too_few_vertices_raise():
    with pytest.raises(ValueError):
        Centerline([(0.0, 0.0), (1.0, 0.0)])


def test_returns_goalpose_type():
    assert isinstance(_square().pose_at(0.0), GoalPose)
