# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure-geometry generation of a short straight-ahead verification trajectory.

No ROS imports: this module is unit-tested with plain pytest. The rclpy node in
scripts/publish_canned_trajectory turns these primitives into an
autoware_planning_msgs/Trajectory and republishes it. This is an M2 bring-up
verification aid; M3 replaces it with the real planner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TrajectoryPoint2D:
    """A planar trajectory point: map-frame position, heading, and target speed."""

    x: float
    y: float
    yaw: float
    velocity_mps: float


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """Return (z, w) of the unit quaternion for a planar rotation `yaw` (rad).

    x and y are identically zero for a flat-floor heading, so only the z/w
    components are needed to fill a geometry_msgs/Quaternion.
    """
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def generate_straight_trajectory(
    start_x: float,
    start_y: float,
    start_yaw: float,
    length: float,
    spacing: float,
    velocity_mps: float,
) -> list[TrajectoryPoint2D]:
    """Generate a straight path of points from a start pose along `start_yaw`.

    Points are spaced `spacing` m apart along `start_yaw`, and the final point is
    placed exactly at `length` from the start: when `length` is not an integer
    multiple of `spacing`, an extra endpoint is appended so the path always
    reaches `length` (its last segment is then shorter than `spacing`). All
    points share `start_yaw`. Every point carries `velocity_mps` except the final
    point, which is 0.0 so the controller's goal-stop yields a clean halt.

    Raises ValueError on non-positive `spacing`/`length`, on `spacing > length`,
    and on negative `velocity_mps`, so a misconfigured tool fails loudly instead
    of publishing a degenerate trajectory (autoware_simple_pure_pursuit treats
    <=5 points as "at goal" and never moves).
    """
    if spacing <= 0.0:
        raise ValueError(f"spacing must be > 0, got {spacing}")
    if length <= 0.0:
        raise ValueError(f"length must be > 0, got {length}")
    if spacing > length:
        raise ValueError(
            f"spacing ({spacing}) must be <= length ({length}); "
            "otherwise only the start point is generated and the controller "
            "treats it as already at the goal"
        )
    if velocity_mps < 0.0:
        raise ValueError(f"velocity_mps must be >= 0, got {velocity_mps}")

    cos_yaw = math.cos(start_yaw)
    sin_yaw = math.sin(start_yaw)

    # Grid points every `spacing` m. +1e-9 absorbs binary-float error (e.g.
    # 1.0/0.1 == 9.999999999999998) so a whole number of intervals is not
    # silently dropped.
    num_intervals = int(math.floor(length / spacing + 1e-9))
    distances = [i * spacing for i in range(num_intervals + 1)]
    # When `length` is not a multiple of `spacing` the last grid point stops
    # short of `length`; append the exact endpoint so the path always reaches it.
    if length - distances[-1] > 1e-9:
        distances.append(length)

    last_index = len(distances) - 1
    points: list[TrajectoryPoint2D] = []
    for i, distance in enumerate(distances):
        points.append(
            TrajectoryPoint2D(
                x=start_x + distance * cos_yaw,
                y=start_y + distance * sin_yaw,
                yaw=start_yaw,
                velocity_mps=0.0 if i == last_index else velocity_mps,
            )
        )
    return points
