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


# Circle parity: a finely-sampled circle Centerline reproduces the old circle
# helpers, so the arc-length cutover does not regress circle behaviour.
from kachaka_autoware_bridge.loop_route import (  # noqa: E402
    CLOCKWISE,
    carrot_goal,
    centerline_carrot,
)


def _circle_vertices(cx, cy, r, n):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def test_circle_centerline_matches_old_carrot_goal():
    cx, cy, r = 0.98, 0.13, 0.81
    cl = Centerline(_circle_vertices(cx, cy, r, 720))
    lead_len = (2 * math.pi * r) / 8.0
    rx, ry = cx + r, cy
    new = centerline_carrot(cl, rx, ry, lead_len, CLOCKWISE)
    old = carrot_goal(cx, cy, r, rx, ry, lead_angle_rad=2 * math.pi / 8.0, travel_direction=CLOCKWISE)
    assert (new.x, new.y) == pytest.approx((old.x, old.y), abs=2e-2)
    assert abs(math.atan2(math.sin(new.yaw - old.yaw), math.cos(new.yaw - old.yaw))) < 5e-2
