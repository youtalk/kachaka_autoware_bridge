# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import pytest

from kachaka_autoware_bridge.loop_route import (
    CLOCKWISE,
    COUNTERCLOCKWISE,
    GoalPose,
    _normalize_angle,
    angular_position,
    carrot_goal,
    compute_lap_goal,
    nearest_ring_pose,
    remaining_arc,
    ring_pose_at,
    should_refresh_carrot,
    signed_progress,
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


# ---------------------------------------------------------------------------
# Task A1: angular_position and carrot_goal
# ---------------------------------------------------------------------------


def test_angular_position_offset_center() -> None:
    # Point due north of an offset centre -> +pi/2.
    assert angular_position(2.0, -1.0, 2.0, 0.0) == pytest.approx(math.pi / 2)


def test_carrot_goal_is_ahead_in_travel_direction() -> None:
    # CW (default): robot east of centre (theta=0); the carrot sits AHEAD, i.e.
    # at -lead (CW decreases theta), on the ring, facing the CW tangent.
    g = carrot_goal(0.0, 0.0, 1.0, robot_x=1.0, robot_y=0.0, lead_angle_rad=math.pi / 2)
    assert g.x == pytest.approx(math.cos(-math.pi / 2))
    assert g.y == pytest.approx(math.sin(-math.pi / 2))
    assert g.yaw == pytest.approx(_normalize_angle(-math.pi / 2 - math.pi / 2))


def test_carrot_goal_ccw_is_ahead_the_other_way() -> None:
    g = carrot_goal(
        0.0, 0.0, 1.0, robot_x=1.0, robot_y=0.0,
        lead_angle_rad=math.pi / 2, travel_direction=COUNTERCLOCKWISE,
    )
    assert g.x == pytest.approx(math.cos(math.pi / 2))
    assert g.y == pytest.approx(math.sin(math.pi / 2))


def test_carrot_goal_is_anti_position_of_lap_goal_sign() -> None:
    # carrot is ahead, compute_lap_goal is behind: at the same robot/lead the two
    # angular positions straddle the robot.
    lead = math.pi / 4
    carrot = carrot_goal(0.0, 0.0, 1.0, 1.0, 0.0, lead_angle_rad=lead)  # CW -> -lead
    behind = compute_lap_goal(0.0, 0.0, 1.0, 1.0, 0.0, behind_angle_rad=lead)  # CW -> +lead
    assert math.atan2(carrot.y, carrot.x) == pytest.approx(-lead)
    assert math.atan2(behind.y, behind.x) == pytest.approx(lead)


def test_carrot_goal_invalid_lead_raises() -> None:
    with pytest.raises(ValueError):
        carrot_goal(0.0, 0.0, 1.0, 1.0, 0.0, lead_angle_rad=0.0)
    with pytest.raises(ValueError):
        carrot_goal(0.0, 0.0, 1.0, 1.0, 0.0, lead_angle_rad=7.0)  # > 2*pi
    with pytest.raises(ValueError):
        carrot_goal(0.0, 0.0, 0.0, 1.0, 0.0, lead_angle_rad=1.0)  # radius <= 0


# ---------------------------------------------------------------------------
# Task A2: signed_progress
# ---------------------------------------------------------------------------


def test_signed_progress_forward_is_positive_each_direction() -> None:
    # CW travel decreases theta: a step from 0.0 to -0.1 is +0.1 forward.
    assert signed_progress(0.0, -0.1, CLOCKWISE) == pytest.approx(0.1)
    # CCW travel increases theta: 0.0 -> +0.1 is +0.1 forward.
    assert signed_progress(0.0, 0.1, COUNTERCLOCKWISE) == pytest.approx(0.1)


def test_signed_progress_backward_is_negative() -> None:
    assert signed_progress(0.0, 0.1, CLOCKWISE) == pytest.approx(-0.1)


def test_signed_progress_wraps_across_pi() -> None:
    # CW crossing the -pi/+pi seam: -3.0 -> 3.0 is a small forward step, not -6.
    p = signed_progress(-3.0, 3.0, CLOCKWISE)
    assert p == pytest.approx(2.0 * math.pi - 6.0)
    assert 0.0 < p < 0.5


def test_signed_progress_accumulates_to_one_lap() -> None:
    # Eight CW steps of -pi/4 sum to one full lap (2*pi) of forward progress.
    thetas = [(-k) * math.pi / 4 for k in range(9)]
    total = sum(
        signed_progress(thetas[i], thetas[i + 1], CLOCKWISE) for i in range(8)
    )
    assert total == pytest.approx(2.0 * math.pi)


# ---------------------------------------------------------------------------
# Task A3: remaining_arc and should_refresh_carrot
# ---------------------------------------------------------------------------


def test_remaining_arc_shrinks_as_robot_advances_cw() -> None:
    # CW, goal 1.5 rad ahead of theta=0 sits at theta=-1.5.
    assert remaining_arc(0.0, -1.5, CLOCKWISE) == pytest.approx(1.5)
    # Robot advances to theta=-1.4 -> only 0.1 left.
    assert remaining_arc(-1.4, -1.5, CLOCKWISE) == pytest.approx(0.1)


def test_remaining_arc_is_in_zero_two_pi() -> None:
    r = remaining_arc(0.2, 0.1, COUNTERCLOCKWISE)  # goal just behind -> almost a full lap
    assert 0.0 <= r < 2.0 * math.pi
    assert r == pytest.approx(2.0 * math.pi - 0.1)


def test_should_refresh_when_within_threshold() -> None:
    assert should_refresh_carrot(-1.45, -1.5, CLOCKWISE, refresh_angle_rad=0.1) is True
    assert should_refresh_carrot(-0.5, -1.5, CLOCKWISE, refresh_angle_rad=0.1) is False


def test_signed_and_arc_reject_unknown_direction() -> None:
    with pytest.raises(ValueError):
        signed_progress(0.0, 0.1, "sideways")
    with pytest.raises(ValueError):
        remaining_arc(0.0, 0.1, "")


def test_should_refresh_ccw_direction() -> None:
    # CCW: goal 0.05 rad ahead (theta increases) -> within a 0.1 threshold.
    assert should_refresh_carrot(0.0, 0.05, COUNTERCLOCKWISE, refresh_angle_rad=0.1) is True
    assert should_refresh_carrot(0.0, 1.5, COUNTERCLOCKWISE, refresh_angle_rad=0.1) is False


# ---------------------------------------------------------------------------
# Task A4: direction-aware Centerline helpers
# ---------------------------------------------------------------------------

from kachaka_autoware_bridge.centerline import Centerline  # noqa: E402
from kachaka_autoware_bridge.loop_route import (  # noqa: E402
    centerline_carrot,
    centerline_goal_behind,
    centerline_progress,
    centerline_remaining,
)


def _ccw_square():
    return Centerline([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)])


def test_centerline_carrot_is_ahead_in_travel_direction():
    cl = _ccw_square()
    g = centerline_carrot(cl, 0.0, -1.0, lead_len=1.0, travel_direction=COUNTERCLOCKWISE)
    assert (g.x, g.y) == pytest.approx((1.0, -1.0))
    g_cw = centerline_carrot(cl, 0.0, -1.0, lead_len=1.0, travel_direction=CLOCKWISE)
    assert (g_cw.x, g_cw.y) == pytest.approx((-1.0, -1.0))


def test_centerline_carrot_faces_travel_tangent_cw_is_reversed():
    cl = _ccw_square()
    ccw = centerline_carrot(cl, 0.0, -1.0, lead_len=0.5, travel_direction=COUNTERCLOCKWISE)
    cw = centerline_carrot(cl, 0.0, -1.0, lead_len=0.5, travel_direction=CLOCKWISE)
    assert _angdiff(ccw.yaw, cw.yaw) == pytest.approx(math.pi)


def test_centerline_goal_behind_is_a_near_full_lap():
    cl = _ccw_square()
    g = centerline_goal_behind(cl, 0.0, -1.0, behind_len=0.5, travel_direction=COUNTERCLOCKWISE)
    assert (g.x, g.y) == pytest.approx((-0.5, -1.0))


def test_centerline_progress_forward_positive_each_direction():
    cl = _ccw_square()
    s_a, _ = cl.project(0.0, -1.0)
    s_b, _ = cl.project(0.5, -1.0)
    assert centerline_progress(s_a, s_b, cl.total_length, COUNTERCLOCKWISE) == pytest.approx(0.5)
    assert centerline_progress(s_a, s_b, cl.total_length, CLOCKWISE) == pytest.approx(-0.5)


def test_centerline_progress_wraps_across_seam():
    cl = _ccw_square()
    assert centerline_progress(7.5, 0.5, cl.total_length, COUNTERCLOCKWISE) == pytest.approx(1.0)


def test_centerline_remaining_is_in_zero_total():
    cl = _ccw_square()
    r = centerline_remaining(1.0, 0.5, cl.total_length, COUNTERCLOCKWISE)
    assert 0.0 <= r < cl.total_length
    assert r == pytest.approx(cl.total_length - 0.5)


def test_centerline_helpers_reject_unknown_direction():
    cl = _ccw_square()
    with pytest.raises(ValueError):
        centerline_progress(0.0, 1.0, cl.total_length, "sideways")
    with pytest.raises(ValueError):
        centerline_carrot(cl, 0.0, -1.0, lead_len=1.0, travel_direction="")
