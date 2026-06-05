# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math

import pytest

from kachaka_autoware_bridge.trajectory_gen import (
    generate_straight_trajectory,
    yaw_to_quaternion,
)


def test_point_count_is_intervals_plus_one() -> None:
    points = generate_straight_trajectory(0.0, 0.0, 0.0, length=1.0, spacing=0.1, velocity_mps=0.2)
    # 1.0 m at 0.1 m spacing = 10 intervals = 11 points (both ends inclusive).
    assert len(points) == 11


def test_more_than_five_points_so_controller_moves() -> None:
    # autoware_simple_pure_pursuit treats <=5 points as "at goal" and commands
    # zero velocity, so a verification trajectory must have more than 5 points.
    points = generate_straight_trajectory(0.0, 0.0, 0.0, length=1.0, spacing=0.1, velocity_mps=0.2)
    assert len(points) > 5


def test_points_advance_along_zero_heading() -> None:
    points = generate_straight_trajectory(0.0, 0.0, 0.0, length=1.0, spacing=0.1, velocity_mps=0.2)
    assert points[0].x == pytest.approx(0.0)
    assert points[0].y == pytest.approx(0.0)
    assert points[1].x == pytest.approx(0.1)
    assert points[1].y == pytest.approx(0.0)
    assert points[-1].x == pytest.approx(1.0)
    assert points[-1].y == pytest.approx(0.0)


def test_heading_rotates_advance_direction() -> None:
    points = generate_straight_trajectory(
        2.0, -1.0, math.pi / 2.0, length=0.5, spacing=0.1, velocity_mps=0.2
    )
    # Facing +y: x stays at the start, y increases.
    assert points[1].x == pytest.approx(2.0, abs=1e-9)
    assert points[1].y == pytest.approx(-1.0 + 0.1, abs=1e-9)
    for p in points:
        assert p.yaw == pytest.approx(math.pi / 2.0)


def test_intermediate_velocity_is_constant_last_is_zero() -> None:
    points = generate_straight_trajectory(0.0, 0.0, 0.0, length=1.0, spacing=0.1, velocity_mps=0.2)
    assert all(p.velocity_mps == pytest.approx(0.2) for p in points[:-1])
    # Terminal point at zero so the goal-stop produces a clean halt.
    assert points[-1].velocity_mps == pytest.approx(0.0)


def test_non_integer_ratio_reaches_exact_length() -> None:
    # length is not a multiple of spacing: grid points land at 0, 0.3, 0.6, 0.9,
    # then an exact endpoint at length (1.0) is appended so the path reaches it
    # instead of stopping short at 0.9.
    points = generate_straight_trajectory(0.0, 0.0, 0.0, length=1.0, spacing=0.3, velocity_mps=0.2)
    assert [p.x for p in points] == pytest.approx([0.0, 0.3, 0.6, 0.9, 1.0])
    # The appended endpoint carries the terminal zero velocity; the point before
    # it keeps the cruising velocity, and the final segment is intentionally < spacing.
    assert points[-1].velocity_mps == pytest.approx(0.0)
    assert points[-2].velocity_mps == pytest.approx(0.2)


def test_nonpositive_spacing_raises() -> None:
    with pytest.raises(ValueError):
        generate_straight_trajectory(0.0, 0.0, 0.0, length=1.0, spacing=0.0, velocity_mps=0.2)


def test_nonpositive_length_raises() -> None:
    with pytest.raises(ValueError):
        generate_straight_trajectory(0.0, 0.0, 0.0, length=0.0, spacing=0.1, velocity_mps=0.2)


def test_spacing_larger_than_length_raises() -> None:
    with pytest.raises(ValueError):
        generate_straight_trajectory(0.0, 0.0, 0.0, length=0.3, spacing=0.5, velocity_mps=0.2)


def test_negative_velocity_raises() -> None:
    with pytest.raises(ValueError):
        generate_straight_trajectory(0.0, 0.0, 0.0, length=1.0, spacing=0.1, velocity_mps=-0.2)


def test_yaw_to_quaternion_identity_and_half_turn() -> None:
    z0, w0 = yaw_to_quaternion(0.0)
    assert z0 == pytest.approx(0.0)
    assert w0 == pytest.approx(1.0)
    zp, wp = yaw_to_quaternion(math.pi)
    assert zp == pytest.approx(1.0)
    assert wp == pytest.approx(0.0, abs=1e-9)
