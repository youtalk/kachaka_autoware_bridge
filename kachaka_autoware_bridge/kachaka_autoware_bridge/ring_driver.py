# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Shared onto-ring preflight: resolve the live-map travel direction and drive
the robot onto the loop centerline via Kachaka MOVE_TO_POSE.

Both scripts/set_loop_route and scripts/run_endurance need to (1) read the loop's
travel direction from the live /map/vector_map (lanelet2 winding normalisation
differs between a raw load and Autoware's loader, so it must come from the loaded
map) and (2) place the robot onto the nearest ring point facing travel before any
route can be set. This class wraps those steps around an existing rclpy Node so
the logic lives in one place. The pure geometry it relies on is in loop_route.py
and is unit-tested there.
"""

from __future__ import annotations

import math
import os
import tempfile

import rclpy
from autoware_map_msgs.msg import LaneletMapBin
from rclpy.duration import Duration
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_srvs.srv import SetBool

from kachaka_autoware_bridge.loop_route import (
    nearest_ring_pose,
    travel_direction_from_tangent,
)

DEFAULT_MAP_TOPIC = "/map/vector_map"
DEFAULT_COMMAND_ACTION = "/kachaka/kachaka_command/execute"
DEFAULT_MANUAL_CONTROL_SERVICE = "/kachaka/manual_control/set_enabled"
DEFAULT_SERVICE_TIMEOUT_SEC = 10.0
ALIGN_TOLERANCE_RAD = math.radians(60.0)  # route_handler gate is 90 deg; keep margin


def _angle_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class RingDriver:
    """Onto-ring preflight bound to an rclpy Node and a parsed LoopFile."""

    def __init__(self, node: Node, loop, position_timeout_sec: float = 120.0) -> None:
        self._node = node
        self._loop = loop
        self._position_timeout_sec = position_timeout_sec
        self._latest_map: LaneletMapBin | None = None
        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(LaneletMapBin, DEFAULT_MAP_TOPIC, self._on_map, latched)
        self._manual_cli = node.create_client(SetBool, DEFAULT_MANUAL_CONTROL_SERVICE)
        self._move_client = ActionClient(node, ExecKachakaCommand, DEFAULT_COMMAND_ACTION)

    @property
    def _log(self):
        return self._node.get_logger()

    def _on_map(self, msg: LaneletMapBin) -> None:
        self._latest_map = msg

    def _wait_for_map(self, timeout_sec: float) -> bool:
        deadline = self._node.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self._latest_map is None:
            if self._node.get_clock().now() > deadline:
                return False
            rclpy.spin_once(self._node, timeout_sec=0.1)
        return self._latest_map is not None

    def resolve_travel_direction(self, robot_x: float, robot_y: float) -> str:
        """Read the loop travel direction from the live /map/vector_map; fall back
        to loop_params.travel_direction if the map or lanelet2 python is missing
        or the ring point lies in no lanelet."""
        fallback = self._loop.travel_direction
        if not self._wait_for_map(5.0):
            self._log.warning(f"no {DEFAULT_MAP_TOPIC}; using '{fallback}'")
            return fallback
        try:
            from lanelet2 import core, geometry, io
        except ImportError:
            self._log.warning(f"lanelet2 python missing; using '{fallback}'")
            return fallback
        tmp = None
        try:
            with tempfile.NamedTemporaryFile("wb", suffix=".bin", delete=False) as handle:
                handle.write(bytes(self._latest_map.data))
                tmp = handle.name
            lmap = io.load(tmp, io.Origin(0.0, 0.0))
            theta = math.atan2(robot_y - self._loop.center_y, robot_x - self._loop.center_x)
            ring = core.BasicPoint2d(
                self._loop.center_x + self._loop.radius * math.cos(theta),
                self._loop.center_y + self._loop.radius * math.sin(theta),
            )
            here = [ll for ll in lmap.laneletLayer if geometry.inside(ll, ring)]
            if not here:
                self._log.warning(f"ring point in no lanelet; using '{fallback}'")
                return fallback
            c = here[0].centerline
            tangent = math.atan2(c[-1].y - c[0].y, c[-1].x - c[0].x)
            direction = travel_direction_from_tangent(theta, tangent)
            self._log.info(f"live map travel direction = {direction} (lanelet {here[0].id})")
            return direction
        except Exception as exc:  # noqa: BLE001 - any failure -> fall back
            self._log.warning(f"could not read live map direction ({exc}); using '{fallback}'")
            return fallback
        finally:
            if tmp is not None and os.path.exists(tmp):
                os.unlink(tmp)

    def set_manual_control(self, enabled: bool) -> None:
        """Best-effort toggle of Kachaka manual_control; logs and continues."""
        if not self._manual_cli.wait_for_service(timeout_sec=DEFAULT_SERVICE_TIMEOUT_SEC):
            self._log.warning(
                f"{DEFAULT_MANUAL_CONTROL_SERVICE} unavailable; leaving manual_control as-is"
            )
            return
        fut = self._manual_cli.call_async(SetBool.Request(data=enabled))
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=DEFAULT_SERVICE_TIMEOUT_SEC)
        resp = fut.result()
        if resp is None or not resp.success:
            self._log.warning(f"manual_control set_enabled({enabled}) did not confirm")

    def drive_to_pose(self, x: float, y: float, yaw: float) -> bool:
        """Drive the robot to (x, y, yaw) via Kachaka MOVE_TO_POSE; blocks to done."""
        if not self._move_client.wait_for_server(timeout_sec=DEFAULT_SERVICE_TIMEOUT_SEC):
            self._log.error(f"{DEFAULT_COMMAND_ACTION} unavailable; is the bridge up?")
            return False
        goal = ExecKachakaCommand.Goal()
        goal.kachaka_command.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        goal.kachaka_command.move_to_pose_command_x = x
        goal.kachaka_command.move_to_pose_command_y = y
        goal.kachaka_command.move_to_pose_command_yaw = yaw
        self._log.info(f"Driving onto loop: target=({x:.3f}, {y:.3f}, yaw={yaw:.3f})")
        send_fut = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_fut, timeout_sec=DEFAULT_SERVICE_TIMEOUT_SEC)
        handle = send_fut.result()
        if handle is None or not handle.accepted:
            self._log.error("MOVE_TO_POSE goal was rejected")
            return False
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_fut, timeout_sec=self._position_timeout_sec)
        wrapped = result_fut.result()
        if wrapped is None:
            self._log.error(f"MOVE_TO_POSE did not finish within {self._position_timeout_sec}s")
            return False
        result = wrapped.result
        if not result.success:
            self._log.error(f"MOVE_TO_POSE failed: error_code={result.error_code} {result.message}")
            return False
        return True

    def ensure_on_ring(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        travel_direction: str,
    ) -> bool:
        """Drive onto the nearest ring point facing travel, if not already there.
        Returns True when the robot is on the ring (or already was)."""
        target = nearest_ring_pose(
            self._loop.center_x, self._loop.center_y, self._loop.radius,
            robot_x, robot_y, travel_direction,
        )
        dist_err = abs(
            math.hypot(robot_x - self._loop.center_x, robot_y - self._loop.center_y)
            - self._loop.radius
        )
        yaw_err = _angle_diff(robot_yaw, target.yaw)
        if dist_err <= self._loop.lane_width * 0.4 and yaw_err <= ALIGN_TOLERANCE_RAD:
            self._log.info(
                f"Already on the loop (radial err {dist_err:.3f} m, yaw err "
                f"{math.degrees(yaw_err):.0f} deg); skipping drive-on."
            )
            return True
        self.set_manual_control(False)  # Kachaka autonomous nav owns the move
        return self.drive_to_pose(target.x, target.y, target.yaw)
