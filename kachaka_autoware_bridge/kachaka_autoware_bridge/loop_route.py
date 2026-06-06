# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure-geometry helpers for routing the robot around the circular loop.

No ROS imports: unit-tested with plain pytest. scripts/set_loop_route reads the
loop centre/radius/travel-direction (loop_params.yaml from kachaka_autoware_maps)
and the robot's current pose, then uses these to (1) place the robot onto the
loop facing the travel direction and (2) put an AD-API route goal just behind the
robot so a route in the travel direction covers a near-complete lap.

Travel direction matters: ``autoware_route_handler`` rejects a start lanelet
whose centerline direction differs from the ego heading by more than 90 deg
(route_handler.cpp, ``yaw_threshold = M_PI/2``). The generated loops load as
**clockwise** in lanelet2 (the loader normalises winding; see
kachaka_autoware_maps.loop_map_gen.LOADED_TRAVEL_DIRECTION), so goal/target poses
must face the clockwise tangent or routing fails with "Failed to find a proper
route!". The earlier counter-clockwise assumption produced an anti-parallel
(180 deg) goal yaw, which is exactly what triggered that failure.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

CLOCKWISE = "clockwise"
COUNTERCLOCKWISE = "counterclockwise"
TRAVEL_DIRECTIONS = (CLOCKWISE, COUNTERCLOCKWISE)


@dataclass(frozen=True)
class GoalPose:
    """Planar pose: map-frame position and heading (rad)."""

    x: float
    y: float
    yaw: float


def _normalize_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi].

    The exact boundary behaviour is float-dependent: atan2(sin(-pi), cos(-pi))
    returns -pi because sin(-pi) is a small negative float rather than exact 0.
    """
    return math.atan2(math.sin(a), math.cos(a))


def _direction_sign(travel_direction: str) -> float:
    """+1 if theta increases along travel (counter-clockwise), -1 if it decreases
    (clockwise). Raises ValueError on an unknown direction."""
    if travel_direction == COUNTERCLOCKWISE:
        return 1.0
    if travel_direction == CLOCKWISE:
        return -1.0
    raise ValueError(
        f"travel_direction must be one of {TRAVEL_DIRECTIONS}, got {travel_direction!r}"
    )


def _tangent_yaw(theta: float, sign: float) -> float:
    """Heading of the travel tangent at angular position ``theta`` on the loop."""
    return _normalize_angle(theta + sign * math.pi / 2.0)


def travel_direction_from_tangent(theta: float, tangent_yaw: float) -> str:
    """Classify a loop lanelet's travel direction from its centerline tangent.

    ``theta`` = atan2(point_y - center_y, point_x - center_x) for a
    point on the loop; ``tangent_yaw`` = heading of the lanelet centerline there.
    Counter-clockwise travel has tangent ~ theta + pi/2, clockwise
    ~ theta - pi/2. Used by set_loop_route to read the *actual* travel
    direction from the loaded /map/vector_map rather than trusting a static guess
    (the lanelet2 winding normalisation differs between a raw load and Autoware's
    map loader, so the direction must come from the live map).
    """
    d_ccw = abs(_normalize_angle(tangent_yaw - _tangent_yaw(theta, 1.0)))
    d_cw = abs(_normalize_angle(tangent_yaw - _tangent_yaw(theta, -1.0)))
    return COUNTERCLOCKWISE if d_ccw <= d_cw else CLOCKWISE


def ring_pose_at(
    center_x: float,
    center_y: float,
    radius: float,
    theta: float,
    travel_direction: str = CLOCKWISE,
) -> GoalPose:
    """Pose on the centerline circle at angular position ``theta``, facing travel.

    Raises ValueError if radius <= 0 or travel_direction is unknown.
    """
    if radius <= 0.0:
        raise ValueError(f"radius must be > 0, got {radius}")
    sign = _direction_sign(travel_direction)
    return GoalPose(
        x=center_x + radius * math.cos(theta),
        y=center_y + radius * math.sin(theta),
        yaw=_tangent_yaw(theta, sign),
    )


def nearest_ring_pose(
    center_x: float,
    center_y: float,
    radius: float,
    robot_x: float,
    robot_y: float,
    travel_direction: str = CLOCKWISE,
) -> GoalPose:
    """Closest centerline point to the robot, facing the travel tangent.

    This is the target pose for driving the robot *onto* the loop before routing:
    same angular position as the robot, projected onto the centerline circle,
    heading aligned with the loop's travel direction so route_handler's start-yaw
    gate accepts the robot's start lanelet.

    Raises ValueError if radius <= 0 or travel_direction is unknown. The
    degenerate robot-exactly-at-centre case uses atan2(0, 0) = 0 (treated as due
    east); callers on the loop never hit it.
    """
    theta = math.atan2(robot_y - center_y, robot_x - center_x)
    return ring_pose_at(center_x, center_y, radius, theta, travel_direction)


def compute_lap_goal(
    center_x: float,
    center_y: float,
    radius: float,
    robot_x: float,
    robot_y: float,
    behind_angle_rad: float = math.pi / 8,
    travel_direction: str = CLOCKWISE,
) -> GoalPose:
    """Goal pose on the loop just behind the robot, for a near-complete lap.

    The robot's angular position is theta = atan2(robot_y - center_y, robot_x -
    center_x). The goal is placed ``behind_angle_rad`` *behind* that angle in the
    loop's travel direction, projected onto the centerline circle, facing the
    travel tangent. Routing in the travel direction from the robot's lanelet to
    the goal lanelet then spans (2*pi - behind_angle_rad) ~= one lap.

    ``travel_direction`` must match the loaded map (our generator's loops load
    clockwise). Raises ValueError if radius <= 0, behind_angle_rad is not in
    (0, 2*pi), or travel_direction is unknown.
    """
    if radius <= 0.0:
        raise ValueError(f"radius must be > 0, got {radius}")
    if not (0.0 < behind_angle_rad < 2.0 * math.pi):
        raise ValueError(f"behind_angle_rad must be in (0, 2*pi), got {behind_angle_rad}")
    sign = _direction_sign(travel_direction)
    theta_robot = math.atan2(robot_y - center_y, robot_x - center_x)
    theta_goal = theta_robot - sign * behind_angle_rad
    return ring_pose_at(center_x, center_y, radius, theta_goal, travel_direction)


def angular_position(center_x: float, center_y: float, x: float, y: float) -> float:
    """Angle (rad) of point (x, y) about the loop centre, atan2 in [-pi, pi]."""
    return math.atan2(y - center_y, x - center_x)


def carrot_goal(
    center_x: float,
    center_y: float,
    radius: float,
    robot_x: float,
    robot_y: float,
    lead_angle_rad: float,
    travel_direction: str = CLOCKWISE,
) -> GoalPose:
    """A goal placed ``lead_angle_rad`` *ahead* of the robot in the travel
    direction, on the centerline, facing the travel tangent.

    This is the receding-horizon "carrot": continuously re-placing it ahead of
    the robot keeps a route set without the robot ever arriving, so the loop is
    driven continuously (the opposite placement of ``compute_lap_goal``, which
    puts the goal behind for a single near-full lap). Raises ValueError if
    radius <= 0, lead_angle_rad is not in (0, 2*pi), or travel_direction is
    unknown.
    """
    if radius <= 0.0:
        raise ValueError(f"radius must be > 0, got {radius}")
    if not (0.0 < lead_angle_rad < 2.0 * math.pi):
        raise ValueError(f"lead_angle_rad must be in (0, 2*pi), got {lead_angle_rad}")
    sign = _direction_sign(travel_direction)
    theta_robot = angular_position(center_x, center_y, robot_x, robot_y)
    theta_goal = theta_robot + sign * lead_angle_rad
    return ring_pose_at(center_x, center_y, radius, theta_goal, travel_direction)


def signed_progress(prev_theta: float, curr_theta: float, travel_direction: str) -> float:
    """Forward arc (rad) travelled from ``prev_theta`` to ``curr_theta`` along the
    travel direction, using the shortest angular step (so it is wrap-safe across
    the +/-pi seam). Positive = forward, negative = the robot slipped backward.
    Accumulate this across consecutive odometry samples to count laps without a
    discrete goal. Raises ValueError on an unknown travel_direction.
    """
    sign = _direction_sign(travel_direction)
    delta = _normalize_angle(curr_theta - prev_theta)
    return sign * delta


def remaining_arc(robot_theta: float, goal_theta: float, travel_direction: str) -> float:
    """Forward arc (rad) in [0, 2*pi) the robot must still travel to reach
    ``goal_theta`` along the travel direction. Raises ValueError on an unknown
    travel_direction.
    """
    sign = _direction_sign(travel_direction)
    return (sign * (goal_theta - robot_theta)) % (2.0 * math.pi)


def should_refresh_carrot(
    robot_theta: float,
    goal_theta: float,
    travel_direction: str,
    refresh_angle_rad: float,
) -> bool:
    """True when the carrot goal is close enough ahead that it should be replaced
    with a fresh one further along (so the robot never actually arrives).
    Raises ValueError on an unknown travel_direction."""
    return remaining_arc(robot_theta, goal_theta, travel_direction) <= refresh_angle_rad
