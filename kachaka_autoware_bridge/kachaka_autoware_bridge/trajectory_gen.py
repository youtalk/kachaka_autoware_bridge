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

    Points are spaced `spacing` m apart (both ends inclusive) and all share
    `start_yaw`. Every point carries `velocity_mps` except the final point,
    which is 0.0 so the controller's goal-stop yields a clean halt.

    Raises ValueError on non-positive `spacing`/`length` so a misconfigured tool
    fails loudly instead of publishing an empty or single-point trajectory
    (autoware_simple_pure_pursuit treats <=5 points as "at goal" and never moves).
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

    # +1e-9 absorbs binary-float error (e.g. 1.0/0.1 == 9.999999999999998) so a
    # whole number of intervals is not silently dropped.
    num_intervals = int(math.floor(length / spacing + 1e-9))
    num_points = num_intervals + 1
    cos_yaw = math.cos(start_yaw)
    sin_yaw = math.sin(start_yaw)

    points: list[TrajectoryPoint2D] = []
    for i in range(num_points):
        distance = i * spacing
        is_last = i == num_points - 1
        points.append(
            TrajectoryPoint2D(
                x=start_x + distance * cos_yaw,
                y=start_y + distance * sin_yaw,
                yaw=start_yaw,
                velocity_mps=0.0 if is_last else velocity_mps,
            )
        )
    return points
