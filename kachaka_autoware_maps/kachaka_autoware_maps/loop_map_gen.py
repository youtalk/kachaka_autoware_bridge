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

import yaml

# Fallback travel direction recorded in loop_params.yaml. The AUTHORITATIVE
# direction is read at runtime from the live /map/vector_map by set_loop_route,
# because lanelet2's winding normalisation is NOT load-invariant: a raw
# lanelet2.io.load (degenerate-then-patched coords) winds this OSM CLOCKWISE, but
# Autoware's map loader (LocalProjector, real coords present at parse) winds the
# very same OSM COUNTER-CLOCKWISE. route_handler uses Autoware's loader, so this
# fallback matches it. If a consumer faces the wrong way, route_handler's 90-deg
# start-yaw gate rejects the start lanelet ("Failed to find a proper route!").
# Treat this only as a hint for when the live map cannot be read.
LOADED_TRAVEL_DIRECTION = "counterclockwise"


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


@dataclass(frozen=True)
class LoopParams:
    """Circular loop centre (map frame, metres) and centerline radius (m)."""

    center_x: float
    center_y: float
    radius: float


def rect_to_loop_params(
    rect: FreeRectangle,
    resolution: float,
    origin_x: float,
    origin_y: float,
    lane_width: float,
    margin: float,
    max_radius: float,
) -> LoopParams:
    """Map a free-cell rectangle to a circular-loop centre + radius (map frame).

    Cell-corner (row, col) maps to map-frame metres as
        x = origin_x + col * resolution,  y = origin_y + row * resolution
    where (origin_x, origin_y) is the grid's info.origin.position; this assumes
    zero grid rotation (true for Kachaka's axis-aligned map). The loop centre is
    the rectangle centre. The centerline radius is the largest circle that fits
    inside the rectangle while leaving half the lane plus `margin` of clearance:
        usable = min(width_m, height_m) / 2 - lane_width / 2 - margin
        radius = min(usable, max_radius)
    Raises ValueError on non-positive resolution/lane_width or a rectangle too
    small to fit a positive radius.
    """
    if resolution <= 0.0:
        raise ValueError(f"resolution must be > 0, got {resolution}")
    if lane_width <= 0.0:
        raise ValueError(f"lane_width must be > 0, got {lane_width}")

    center_col = rect.col0 + rect.cols / 2.0
    center_row = rect.row0 + rect.rows / 2.0
    center_x = origin_x + center_col * resolution
    center_y = origin_y + center_row * resolution

    width_m = rect.cols * resolution
    height_m = rect.rows * resolution
    usable = min(width_m, height_m) / 2.0 - lane_width / 2.0 - margin
    radius = min(usable, max_radius)
    if radius <= 0.0:
        raise ValueError(
            f"free rectangle {width_m:.2f}x{height_m:.2f} m is too small for "
            f"lane_width={lane_width} + margin={margin}"
        )
    return LoopParams(center_x=center_x, center_y=center_y, radius=radius)


MAP_PROJECTOR_INFO_YAML = "projector_type: Local\nvertical_datum: WGS84\n"

_OSM_HEADER = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<osm version="0.6" generator="kachaka_autoware_maps">\n'
)
_OSM_FOOTER = "</osm>\n"


def generate_circle_loop_osm(
    center_x: float,
    center_y: float,
    radius: float,
    lane_width: float,
    speed_limit: float,
    num_segments: int,
) -> str:
    """Return a lanelet2 OSM document for one circular counter-clockwise loop.

    The centerline is a circle of `radius` m about (center_x, center_y) in the
    map frame (Local projector: node local_x/local_y are map-frame metres). The
    circle is split into `num_segments` arcs; each arc is one lanelet whose left
    bound is the inner circle (radius - lane_width/2) and right bound the outer
    circle (radius + lane_width/2) — correct for counter-clockwise travel.
    Consecutive arcs SHARE their cross-section node ids and the last arc reuses
    arc 0's nodes, so the lanelet2 routing graph forms a closed cycle.

    `speed_limit` is in **m/s** (the generator's natural input unit). It is
    converted to **km/h** before being written to the OSM ``speed_limit`` tag,
    per the lanelet2 / Autoware convention (Autoware sample maps use km/h).

    Raises ValueError if num_segments < 3, if radius <= lane_width/2 (inner
    radius would be non-positive), or if lane_width/speed_limit are non-positive.
    """
    if num_segments < 3:
        raise ValueError(f"num_segments must be >= 3, got {num_segments}")
    if lane_width <= 0.0:
        raise ValueError(f"lane_width must be > 0, got {lane_width}")
    if speed_limit <= 0.0:
        raise ValueError(f"speed_limit must be > 0, got {speed_limit}")
    inner = radius - lane_width / 2.0
    outer = radius + lane_width / 2.0
    if inner <= 0.0:
        raise ValueError(f"radius ({radius}) must be > lane_width/2 ({lane_width / 2.0})")

    # lanelet2 speed_limit tag is in km/h; convert from the m/s input.
    speed_limit_kmh = speed_limit * 3.6

    # Node ids: inner (left) 1..N at cross-sections 0..N-1; outer (right) N+1..2N.
    left_ids: list[int] = []
    right_ids: list[int] = []
    node_xml: list[str] = []
    for i in range(num_segments):
        theta = 2.0 * math.pi * i / num_segments
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        left_id = i + 1
        right_id = num_segments + i + 1
        left_ids.append(left_id)
        right_ids.append(right_id)
        for nid, rad in ((left_id, inner), (right_id, outer)):
            x = center_x + rad * cos_t
            y = center_y + rad * sin_t
            node_xml.append(
                f'  <node id="{nid}" lat="0.0" lon="0.0">'
                f'<tag k="local_x" v="{x:.6f}"/>'
                f'<tag k="local_y" v="{y:.6f}"/>'
                f'<tag k="ele" v="0.0"/></node>'
            )

    way_xml: list[str] = []
    relation_xml: list[str] = []
    for i in range(num_segments):
        j = (i + 1) % num_segments
        left_way = 2 * num_segments + i + 1
        right_way = 3 * num_segments + i + 1
        lanelet = 4 * num_segments + i + 1
        way_xml.append(
            f'  <way id="{left_way}"><nd ref="{left_ids[i]}"/><nd ref="{left_ids[j]}"/>'
            f'<tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>'
        )
        way_xml.append(
            f'  <way id="{right_way}"><nd ref="{right_ids[i]}"/><nd ref="{right_ids[j]}"/>'
            f'<tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>'
        )
        relation_xml.append(
            f'  <relation id="{lanelet}">\n'
            f'    <member type="way" ref="{left_way}" role="left"/>\n'
            f'    <member type="way" ref="{right_way}" role="right"/>\n'
            f'    <tag k="type" v="lanelet"/>\n'
            f'    <tag k="subtype" v="road"/>\n'
            f'    <tag k="speed_limit" v="{speed_limit_kmh:g}"/>\n'
            f'    <tag k="location" v="urban"/>\n'
            f'    <tag k="one_way" v="yes"/>\n'
            f"  </relation>"
        )

    return (
        _OSM_HEADER
        + "\n".join(node_xml)
        + "\n"
        + "\n".join(way_xml)
        + "\n"
        + "\n".join(relation_xml)
        + "\n"
        + _OSM_FOOTER
    )


@dataclass(frozen=True)
class LoopFile:
    """Full contents of loop_params.yaml (centre/radius + sizing + direction)."""

    center_x: float
    center_y: float
    radius: float
    lane_width: float
    speed_limit: float
    num_segments: int
    travel_direction: str


def loop_params_yaml(
    params: "LoopParams",
    lane_width: float,
    speed_limit: float,
    num_segments: int,
    travel_direction: str = LOADED_TRAVEL_DIRECTION,
) -> str:
    """Human/machine-readable sidecar describing the generated loop (consumed by
    M4's full-lap route helper and useful for debugging).

    ``travel_direction`` records how the loop is traversed once loaded (default
    LOADED_TRAVEL_DIRECTION); set_loop_route reads it to align the robot and goal.
    """
    return (
        f"center_x: {params.center_x}\n"
        f"center_y: {params.center_y}\n"
        f"radius: {params.radius}\n"
        f"lane_width: {lane_width}\n"
        f"speed_limit: {speed_limit}\n"
        f"num_segments: {num_segments}\n"
        f"travel_direction: {travel_direction}\n"
    )


def parse_loop_params(text: str) -> "LoopFile":
    """Parse loop_params.yaml text into a LoopFile (inverse of loop_params_yaml).

    Shared loader so every consumer reads the same schema and it can't drift. A
    file written before travel_direction existed defaults to
    LOADED_TRAVEL_DIRECTION. Raises KeyError on a missing required field and
    ValueError on a non-numeric value.
    """
    data = yaml.safe_load(text) or {}
    return LoopFile(
        center_x=float(data["center_x"]),
        center_y=float(data["center_y"]),
        radius=float(data["radius"]),
        lane_width=float(data["lane_width"]),
        speed_limit=float(data["speed_limit"]),
        num_segments=int(data["num_segments"]),
        travel_direction=str(data.get("travel_direction", LOADED_TRAVEL_DIRECTION)),
    )


def occupancy_to_loop_osm(
    data: list[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    *,
    lane_width: float,
    margin: float,
    max_radius: float,
    speed_limit: float,
    num_segments: int,
    occupied_threshold: int = 50,
    treat_unknown_as_occupied: bool = True,
) -> "tuple[str, LoopParams]":
    """Find the largest free rectangle, fit a loop, and return (osm, params)."""
    rect = largest_free_rectangle(
        data, width, height, occupied_threshold, treat_unknown_as_occupied
    )
    params = rect_to_loop_params(
        rect, resolution, origin_x, origin_y, lane_width, margin, max_radius
    )
    osm = generate_circle_loop_osm(
        params.center_x, params.center_y, params.radius,
        lane_width, speed_limit, num_segments,
    )
    return osm, params
