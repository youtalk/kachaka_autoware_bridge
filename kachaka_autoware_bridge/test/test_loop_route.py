# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import pytest

from kachaka_autoware_bridge.loop_route import (
    CLOCKWISE,
    COUNTERCLOCKWISE,
    GoalPose,
    _normalize_angle,
    compute_lap_goal,
    nearest_ring_pose,
    ring_pose_at,
    travel_direction_from_tangent,
)


def _angdiff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_default_direction_is_clockwise() -> None:
    # The generated loops load clockwise in lanelet2, so the default must be CW:
    # a robot east of centre (theta=0) gets a goal one step BEHIND in CW travel,
    # i.e. at +theta, facing the CW (decreasing-theta) tangent.
    g = compute_lap_goal(0.0, 0.0, 1.0, robot_x=1.0, robot_y=0.0, behind_angle_rad=math.pi / 8)
    assert isinstance(g, GoalPose)
    assert g.x == pytest.approx(math.cos(math.pi / 8))
    assert g.y == pytest.approx(math.sin(math.pi / 8))
    assert g.yaw == pytest.approx(_normalize_angle(math.pi / 8 - math.pi / 2))


def test_counter_clockwise_direction() -> None:
    g = compute_lap_goal(
        0.0, 0.0, 1.0, robot_x=1.0, robot_y=0.0,
        behind_angle_rad=math.pi / 8, travel_direction=COUNTERCLOCKWISE,
    )
    assert g.x == pytest.approx(math.cos(-math.pi / 8))
    assert g.y == pytest.approx(math.sin(-math.pi / 8))
    assert g.yaw == pytest.approx(_normalize_angle(-math.pi / 8 + math.pi / 2))


def test_cw_and_ccw_goal_yaws_are_anti_parallel() -> None:
    # This is the crux of the routing bug: the two directions' tangents differ by
    # pi, so using the wrong one yields a 180-deg-off goal that route_handler's
    # 90-deg start-yaw gate rejects.
    cw = compute_lap_goal(0.5, -0.2, 0.8, 1.0, 1.0, travel_direction=CLOCKWISE)
    ccw = compute_lap_goal(0.5, -0.2, 0.8, 1.0, 1.0, travel_direction=COUNTERCLOCKWISE)
    assert cw.x != pytest.approx(ccw.x) or cw.y != pytest.approx(ccw.y)  # different goal points
    # ...but at any shared angular position the tangents are exactly anti-parallel.
    assert _angdiff(ring_pose_at(0.0, 0.0, 1.0, 1.234, CLOCKWISE).yaw,
                    ring_pose_at(0.0, 0.0, 1.0, 1.234, COUNTERCLOCKWISE).yaw) == pytest.approx(math.pi)


def test_goal_respects_offset_center_and_radius() -> None:
    g = compute_lap_goal(2.0, -1.0, 0.9, robot_x=2.0 + 0.9, robot_y=-1.0, behind_angle_rad=math.pi / 6)
    # robot at theta=0; CW -> goal at +pi/6
    assert g.x == pytest.approx(2.0 + 0.9 * math.cos(math.pi / 6))
    assert g.y == pytest.approx(-1.0 + 0.9 * math.sin(math.pi / 6))


def test_goal_uses_robot_angle_not_robot_radius() -> None:
    # Robot inside the ring (radius 0.3) but at theta=0 -> goal still on the ring.
    g = compute_lap_goal(0.0, 0.0, 1.0, robot_x=0.3, robot_y=0.0, behind_angle_rad=math.pi / 8)
    assert math.hypot(g.x, g.y) == pytest.approx(1.0)


def test_nearest_ring_pose_projects_onto_centerline_and_faces_travel() -> None:
    # Robot off the ring (1.39 from centre, like the real off-ring failure) at
    # theta ~ 225 deg. Target must sit on the ring and face the CW tangent.
    cx, cy, r = 0.98, 0.13, 0.81
    rx, ry = 0.005, -0.864
    t = nearest_ring_pose(cx, cy, r, rx, ry)
    assert math.hypot(t.x - cx, t.y - cy) == pytest.approx(r)
    theta = math.atan2(ry - cy, rx - cx)
    assert t.yaw == pytest.approx(_normalize_angle(theta - math.pi / 2))  # CW tangent


def test_nearest_ring_pose_ccw_is_opposite_heading() -> None:
    cx, cy, r = 0.0, 0.0, 1.0
    cw = nearest_ring_pose(cx, cy, r, 2.0, 0.0, CLOCKWISE)
    ccw = nearest_ring_pose(cx, cy, r, 2.0, 0.0, COUNTERCLOCKWISE)
    assert (cw.x, cw.y) == pytest.approx((ccw.x, ccw.y))
    assert _angdiff(cw.yaw, ccw.yaw) == pytest.approx(math.pi)


def test_ring_pose_at_basic() -> None:
    p = ring_pose_at(0.0, 0.0, 2.0, math.pi / 2, CLOCKWISE)
    assert p.x == pytest.approx(0.0)
    assert p.y == pytest.approx(2.0)
    assert p.yaw == pytest.approx(_normalize_angle(math.pi / 2 - math.pi / 2))  # = 0 (pointing +x)


def test_invalid_travel_direction_raises() -> None:
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, 1.0, 1.0, 0.0, travel_direction="sideways")
    with pytest.raises(ValueError):
        nearest_ring_pose(0.0, 0.0, 1.0, 1.0, 0.0, travel_direction="")


def test_invalid_radius_raises() -> None:
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, 0.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, -1.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        nearest_ring_pose(0.0, 0.0, 0.0, 1.0, 0.0)


def test_invalid_behind_angle_raises() -> None:
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, 1.0, 1.0, 0.0, behind_angle_rad=0.0)
    with pytest.raises(ValueError):
        compute_lap_goal(0.0, 0.0, 1.0, 1.0, 0.0, behind_angle_rad=7.0)  # > 2*pi


def test_travel_direction_from_tangent() -> None:
    # At several angular positions, a tangent of theta+90 is CCW, theta-90 is CW.
    for theta in (0.0, 1.0, -2.5, math.pi, 3.0):
        assert travel_direction_from_tangent(theta, _normalize_angle(theta + math.pi / 2)) == COUNTERCLOCKWISE
        assert travel_direction_from_tangent(theta, _normalize_angle(theta - math.pi / 2)) == CLOCKWISE
    # Slightly noisy tangents still classify correctly (closest of the two).
    assert travel_direction_from_tangent(0.0, math.radians(80)) == COUNTERCLOCKWISE
    assert travel_direction_from_tangent(0.0, math.radians(-100)) == CLOCKWISE


def test_direction_helper_round_trips_with_ring_pose() -> None:
    # ring_pose_at's yaw fed back into the classifier returns the same direction.
    for d in (CLOCKWISE, COUNTERCLOCKWISE):
        theta = 1.234
        p = ring_pose_at(0.5, -0.3, 0.8, theta, d)
        assert travel_direction_from_tangent(theta, p.yaw) == d


def test_normalize_angle_boundaries() -> None:
    assert _normalize_angle(0.0) == pytest.approx(0.0)
    assert _normalize_angle(math.pi) == pytest.approx(math.pi) or _normalize_angle(
        math.pi
    ) == pytest.approx(-math.pi)
    assert _normalize_angle(2.0 * math.pi) == pytest.approx(0.0)
    assert _normalize_angle(3.0 * math.pi) == pytest.approx(math.pi) or _normalize_angle(
        3.0 * math.pi
    ) == pytest.approx(-math.pi)
    assert _normalize_angle(-math.pi / 2) == pytest.approx(-math.pi / 2)
