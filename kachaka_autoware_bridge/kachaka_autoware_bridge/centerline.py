# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Closed-polyline centerline parameterized by arc length (no ROS imports).

This is the shape-agnostic geometry the endurance harness runs on. A circle is
just a finely-sampled polyline; a rounded rectangle is straights + corner arcs.
Keeping the runtime on a polyline means a new trajectory shape never touches the
orchestrator -- it only feeds a different vertex list (from loop_params.yaml).
Unit-tested with plain pytest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GoalPose:
    """Planar pose: map-frame position and heading (rad)."""

    x: float
    y: float
    yaw: float


def _normalize_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Centerline:
    """An ordered, implicitly-closed polyline with arc-length lookups.

    ``vertices`` are map-frame (x, y) points in traversal order; closure is
    implicit (the last segment joins vertices[-1] -> vertices[0]), so do NOT
    repeat the first vertex at the end. ``pose_at`` faces the stored (forward)
    tangent; the travel direction (which way along the polyline the robot drives)
    is applied by the caller (loop_route) from the live-map winding.
    """

    def __init__(self, vertices: list[tuple[float, float]]) -> None:
        if len(vertices) < 3:
            raise ValueError(f"need >= 3 vertices, got {len(vertices)}")
        self._v = [(float(x), float(y)) for x, y in vertices]
        n = len(self._v)
        self._cum = [0.0]  # cum[i] = arc length at vertex i; cum[n] = total
        for i in range(n):
            x0, y0 = self._v[i]
            x1, y1 = self._v[(i + 1) % n]
            self._cum.append(self._cum[-1] + math.hypot(x1 - x0, y1 - y0))
        self._total = self._cum[-1]
        if self._total <= 0.0:
            raise ValueError("degenerate centerline (zero length)")

    @property
    def total_length(self) -> float:
        return self._total

    def _segment_index(self, s: float) -> int:
        # Largest i with cum[i] <= s (s already in [0, total)).
        lo, hi = 0, len(self._v)
        while lo < hi:
            mid = (lo + hi) // 2
            if self._cum[mid + 1] <= s:
                lo = mid + 1
            else:
                hi = mid
        return min(lo, len(self._v) - 1)

    def pose_at(self, s: float) -> GoalPose:
        s = s % self._total
        i = self._segment_index(s)
        x0, y0 = self._v[i]
        x1, y1 = self._v[(i + 1) % len(self._v)]
        seg_len = self._cum[i + 1] - self._cum[i]
        t = 0.0 if seg_len <= 0.0 else (s - self._cum[i]) / seg_len
        return GoalPose(
            x=x0 + t * (x1 - x0),
            y=y0 + t * (y1 - y0),
            yaw=math.atan2(y1 - y0, x1 - x0),
        )

    def advance(self, s: float, ds: float) -> float:
        return (s + ds) % self._total

    def project(self, x: float, y: float) -> tuple[float, float]:
        """Nearest centerline point to (x, y): returns (arc_length, lateral_error).

        ``lateral_error`` is the absolute perpendicular distance to the polyline
        (0 = exactly on it), used to detect off-centerline drift during RUNNING.
        """
        best_s = 0.0
        best_d2 = math.inf
        n = len(self._v)
        for i in range(n):
            x0, y0 = self._v[i]
            x1, y1 = self._v[(i + 1) % n]
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 <= 0.0 else ((x - x0) * dx + (y - y0) * dy) / seg2
            t = max(0.0, min(1.0, t))
            px, py = x0 + t * dx, y0 + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = self._cum[i] + t * math.hypot(dx, dy)
        return best_s % self._total, math.sqrt(best_d2)
