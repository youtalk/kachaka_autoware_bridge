# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure-logic generation of a circular one-way loop lanelet2 map.

No ROS imports: this module is unit-tested with plain pytest. The rclpy tool in
scripts/generate_loop_map snapshots /kachaka/mapping/map (nav_msgs/OccupancyGrid)
and turns these primitives into lanelet2_map.osm + sidecars. The loop is placed
inside the largest free rectangle of the occupancy grid so the robot has room to
drive a full lap; sizing/placement is computed here from the grid geometry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FreeRectangle:
    """Axis-aligned rectangle of free cells, inclusive cell indices (row, col)."""

    row0: int
    col0: int
    row1: int
    col1: int

    @property
    def rows(self) -> int:
        return self.row1 - self.row0 + 1

    @property
    def cols(self) -> int:
        return self.col1 - self.col0 + 1

    @property
    def area(self) -> int:
        return self.rows * self.cols


def _largest_in_histogram(
    heights: list[int], current_row: int, best: "FreeRectangle | None"
) -> "FreeRectangle | None":
    """Largest rectangle in a column histogram, mapped back to grid cells.

    `heights[c]` is the number of consecutive free cells ending at `current_row`
    in column c. A maximal bar of height h spanning columns [l, r] covers grid
    rows [current_row - h + 1, current_row] and columns [l, r]. Returns the
    larger of `best` and any rectangle found in this histogram.
    """
    stack: list[int] = []  # column indices with strictly increasing bar heights
    n = len(heights)
    for i in range(n + 1):
        h = heights[i] if i < n else 0
        while stack and heights[stack[-1]] >= h:
            bar_h = heights[stack.pop()]
            left = stack[-1] + 1 if stack else 0
            right = i - 1
            if bar_h > 0:
                rect = FreeRectangle(current_row - bar_h + 1, left, current_row, right)
                if best is None or rect.area > best.area:
                    best = rect
        stack.append(i)
    return best


def largest_free_rectangle(
    data: list[int],
    width: int,
    height: int,
    occupied_threshold: int = 50,
    treat_unknown_as_occupied: bool = True,
) -> FreeRectangle:
    """Return the largest axis-aligned rectangle of free cells.

    `data` is a nav_msgs/OccupancyGrid `data` array (length width*height,
    row-major, row 0 first). A cell with value v is "free" when 0 <= v <
    occupied_threshold; unknown cells (v < 0, typically -1) count as occupied
    when `treat_unknown_as_occupied` (the default) so the loop is only placed in
    explicitly-mapped free space.

    Raises ValueError on non-positive dimensions, a data/dimension length
    mismatch, or a grid with no free cell.
    """
    if width <= 0 or height <= 0:
        raise ValueError(f"width and height must be > 0, got {width}x{height}")
    if len(data) != width * height:
        raise ValueError(f"data length {len(data)} != width*height {width * height}")

    def is_free(v: int) -> bool:
        if v < 0:
            return not treat_unknown_as_occupied
        return v < occupied_threshold

    heights = [0] * width
    best: FreeRectangle | None = None
    for row in range(height):
        base = row * width
        for col in range(width):
            heights[col] = heights[col] + 1 if is_free(data[base + col]) else 0
        best = _largest_in_histogram(heights, row, best)

    if best is None:
        raise ValueError("occupancy grid has no free cell")
    return best
