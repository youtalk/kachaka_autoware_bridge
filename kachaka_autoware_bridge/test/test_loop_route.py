# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import pytest

from kachaka_autoware_bridge.loop_route import GoalPose, compute_lap_goal


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def test_goal_is_just_behind_robot_counter_clockwise() -> None:
    # Robot east of centre (theta=0) on a unit loop. CCW travel: "behind" is a
    # smaller angle, so the goal sits at theta = -pi/8 and faces the CCW tangent.
    g = compute_lap_goal(0.0, 0.0, 1.0, robot_x=1.0, robot_y=0.0, behind_angle_rad=math.pi / 8)
    assert isinstance(g, GoalPose)
    assert g.x == pytest.approx(math.cos(-math.pi / 8))
    assert g.y == pytest.approx(math.sin(-math.pi / 8))
    assert g.yaw == pytest.approx(_norm(-math.pi / 8 + math.pi / 2))


def test_goal_clockwise_direction() -> None:
    g = compute_lap_goal(
        0.0, 0.0, 1.0, robot_x=1.0, robot_y=0.0, behind_angle_rad=math.pi / 8, clockwise=True
    )
    assert g.x == pytest.approx(math.cos(math.pi / 8))
    assert g.y == pytest.approx(math.sin(math.pi / 8))
    assert g.yaw == pytest.approx(_norm(math.pi / 8 - math.pi / 2))


def test_goal_respects_offset_center_and_radius() -> None:
    g = compute_lap_goal(
        2.0, -1.0, 0.9, robot_x=2.0 + 0.9, robot_y=-1.0, behind_angle_rad=math.pi / 6
    )
    assert g.x == pytest.approx(2.0 + 0.9 * math.cos(-math.pi / 6))
    assert g.y == pytest.approx(-1.0 + 0.9 * math.sin(-math.pi / 6))


def test_goal_uses_robot_angle_not_robot_radius() -> None:
    # Robot inside the ring (radius 0.3) but at theta=0 -> goal still on the ring.
    g = compute_lap_goal(0.0, 0.0, 1.0, robot_x=0.3, robot_y=0.0, behind_angle_rad=math.pi / 8)
    assert math.hypot(g.x, g.y) == pytest.approx(1.0)


def test_invalid_radius_raises() -> None:
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, 0.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, -1.0, 1.0, 0.0)


def test_invalid_behind_angle_raises() -> None:
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, 1.0, 1.0, 0.0, behind_angle_rad=0.0)
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, 1.0, 1.0, 0.0, behind_angle_rad=7.0)  # > 2*pi
