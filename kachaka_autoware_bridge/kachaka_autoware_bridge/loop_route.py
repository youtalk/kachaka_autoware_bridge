# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure-geometry computation of a full-lap goal pose on the circular loop.

No ROS imports: unit-tested with plain pytest. scripts/set_loop_route reads the
loop centre/radius (loop_params.yaml from kachaka_autoware_maps) and the robot's
current pose, then uses this to place an AD-API route goal just behind the robot
so a route in the travel direction covers a near-complete lap.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GoalPose:
    """Planar goal: map-frame position and heading (rad)."""

    x: float
    y: float
    yaw: float


def _normalize_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def compute_lap_goal(
    center_x: float,
    center_y: float,
    radius: float,
    robot_x: float,
    robot_y: float,
    behind_angle_rad: float = math.pi / 8,
    clockwise: bool = False,
) -> GoalPose:
    """Goal pose on the loop just behind the robot, for a near-complete lap.

    The robot's angular position on the loop is theta = atan2(robot_y - center_y,
    robot_x - center_x). The goal is placed `behind_angle_rad` *behind* that
    angle in the travel direction (counter-clockwise by default), projected onto
    the centerline circle of `radius`, facing the travel tangent. Routing in the
    travel direction from the robot's lanelet to the goal lanelet then spans
    (2*pi - behind_angle_rad) of the loop ~= one lap.

    Raises ValueError if radius <= 0 or behind_angle_rad is not in (0, 2*pi).
    """
    if radius <= 0.0:
        raise ValueError(f"radius must be > 0, got {radius}")
    if not (0.0 < behind_angle_rad < 2.0 * math.pi):
        raise ValueError(f"behind_angle_rad must be in (0, 2*pi), got {behind_angle_rad}")

    theta_robot = math.atan2(robot_y - center_y, robot_x - center_x)
    direction = -1.0 if clockwise else 1.0
    theta_goal = theta_robot - direction * behind_angle_rad
    goal_x = center_x + radius * math.cos(theta_goal)
    goal_y = center_y + radius * math.sin(theta_goal)
    goal_yaw = _normalize_angle(theta_goal + direction * math.pi / 2.0)
    return GoalPose(x=goal_x, y=goal_y, yaw=goal_yaw)
