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
    wall_clearance: float,
    margin: float,
    max_radius: float,
) -> LoopParams:
    """Map a free-cell rectangle to a circular-loop centre + radius (map frame).

    Cell-corner (row, col) maps to map-frame metres as
        x = origin_x + col * resolution,  y = origin_y + row * resolution
    where (origin_x, origin_y) is the grid's info.origin.position; this assumes
    zero grid rotation (true for Kachaka's axis-aligned map). The loop centre is
    the rectangle centre. The centerline radius is the largest circle that fits
    inside the rectangle while leaving ``wall_clearance`` (the PHYSICAL half-
    corridor the robot needs from the centerline to a wall) plus ``margin``:
        usable = min(width_m, height_m) / 2 - wall_clearance - margin
        radius = min(usable, max_radius)
    ``wall_clearance`` is deliberately INDEPENDENT of the drawn lanelet width
    (generate_circle_loop_osm's ``lane_width``): the lanelet is drawn wide so
    path_generator's goal-connection stays inside the lane on a tight loop, but
    the physical room the robot needs is set only by its footprint -- so a wide
    drawn lane must not shrink the radius. Raises ValueError on non-positive
    resolution/wall_clearance or a rectangle too small to fit a positive radius.
    """
    if resolution <= 0.0:
        raise ValueError(f"resolution must be > 0, got {resolution}")
    if wall_clearance <= 0.0:
        raise ValueError(f"wall_clearance must be > 0, got {wall_clearance}")

    center_col = rect.col0 + rect.cols / 2.0
    center_row = rect.row0 + rect.rows / 2.0
    center_x = origin_x + center_col * resolution
    center_y = origin_y + center_row * resolution

    width_m = rect.cols * resolution
    height_m = rect.rows * resolution
    usable = min(width_m, height_m) / 2.0 - wall_clearance - margin
    radius = min(usable, max_radius)
    if radius <= 0.0:
        raise ValueError(
            f"free rectangle {width_m:.2f}x{height_m:.2f} m is too small for "
            f"wall_clearance={wall_clearance} + margin={margin}"
        )
    return LoopParams(center_x=center_x, center_y=center_y, radius=radius)


MAP_PROJECTOR_INFO_YAML = "projector_type: Local\nvertical_datum: WGS84\n"

_OSM_HEADER = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    '<osm version="0.6" generator="kachaka_autoware_maps">\n'
)
_OSM_FOOTER = "</osm>\n"


def _node_xml(nid: int, x: float, y: float) -> str:
    return (
        f'  <node id="{nid}" lat="0.0" lon="0.0">'
        f'<tag k="local_x" v="{x:.6f}"/>'
        f'<tag k="local_y" v="{y:.6f}"/>'
        f'<tag k="ele" v="0.0"/></node>'
    )


def _vertex_normals(vertices: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Unit normal at each vertex (mean of adjacent segment normals). For a
    regular polygon this is the radial direction, so the circle bounds match the
    old generator exactly."""
    n = len(vertices)
    seg_norm = []
    for i in range(n):
        x0, y0 = vertices[i]
        x1, y1 = vertices[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy) or 1.0
        seg_norm.append((-dy / length, dx / length))
    normals = []
    for i in range(n):
        nx = seg_norm[i - 1][0] + seg_norm[i][0]
        ny = seg_norm[i - 1][1] + seg_norm[i][1]
        length = math.hypot(nx, ny) or 1.0
        normals.append((nx / length, ny / length))
    return normals


def generate_loop_osm(
    vertices: list[tuple[float, float]],
    lane_width: float,
    speed_limit: float,
) -> str:
    """lanelet2 OSM for a one-way loop along ``vertices`` (implicitly closed).

    One lanelet per segment; left bound = centerline + normal*lane_width/2, right
    bound = centerline - normal*lane_width/2; consecutive segments share their
    cross-section node ids and the last reuses segment 0's, so the routing graph
    is a closed cycle. ``speed_limit`` is m/s, written to the OSM tag in km/h.
    Raises ValueError on < 3 vertices or non-positive lane_width/speed_limit.
    """
    n = len(vertices)
    if n < 3:
        raise ValueError(f"need >= 3 vertices, got {n}")
    if lane_width <= 0.0:
        raise ValueError(f"lane_width must be > 0, got {lane_width}")
    if speed_limit <= 0.0:
        raise ValueError(f"speed_limit must be > 0, got {speed_limit}")
    half = lane_width / 2.0
    normals = _vertex_normals(vertices)
    speed_limit_kmh = speed_limit * 3.6

    left_ids, right_ids, node_xml = [], [], []
    for i in range(n):
        cx, cy = vertices[i]
        nx, ny = normals[i]
        left_id, right_id = i + 1, n + i + 1
        left_ids.append(left_id)
        right_ids.append(right_id)
        node_xml.append(_node_xml(left_id, cx + half * nx, cy + half * ny))
        node_xml.append(_node_xml(right_id, cx - half * nx, cy - half * ny))

    way_xml, relation_xml = [], []
    for i in range(n):
        j = (i + 1) % n
        left_way, right_way, lanelet = 2 * n + i + 1, 3 * n + i + 1, 4 * n + i + 1
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
        _OSM_HEADER + "\n".join(node_xml) + "\n" + "\n".join(way_xml) + "\n"
        + "\n".join(relation_xml) + "\n" + _OSM_FOOTER
    )


def generate_circle_loop_osm(
    center_x: float,
    center_y: float,
    radius: float,
    lane_width: float,
    speed_limit: float,
    num_segments: int,
) -> str:
    """Circular one-way loop OSM (kept for the circle shape + existing callers).
    Delegates to generate_loop_osm over a regular-polygon centerline.

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
    if radius - lane_width / 2.0 <= 0.0:
        raise ValueError(f"radius ({radius}) must be > lane_width/2 ({lane_width / 2.0})")
    return generate_loop_osm(
        circle_centerline_vertices(center_x, center_y, radius, num_segments),
        lane_width, speed_limit,
    )


@dataclass(frozen=True)
class RoundedRectFile:
    """Rounded-rectangle centerline geometry stored in loop_params.yaml."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    corner_radius: float
    segments_per_corner: int
    stop_line_segments: tuple[int, ...]


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
    shape: str = "circle"
    rect: "RoundedRectFile | None" = None


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


def rounded_rect_params_yaml(
    rect: "RoundedRectFile", lane_width: float, speed_limit: float,
    travel_direction: str = LOADED_TRAVEL_DIRECTION,
) -> str:
    """Sidecar for a rounded-rectangle loop. Circle fields are written as the
    rectangle centre + the inscribed circle radius for schema stability; the
    authoritative geometry is in the rect_* fields and shape=rounded_rectangle."""
    cx = (rect.x_min + rect.x_max) / 2.0
    cy = (rect.y_min + rect.y_max) / 2.0
    r = min(rect.x_max - rect.x_min, rect.y_max - rect.y_min) / 2.0
    seg = ",".join(str(s) for s in rect.stop_line_segments)
    return (
        f"shape: rounded_rectangle\n"
        f"center_x: {cx}\ncenter_y: {cy}\nradius: {r}\n"
        f"lane_width: {lane_width}\nspeed_limit: {speed_limit}\n"
        f"num_segments: 0\n"
        f"travel_direction: {travel_direction}\n"
        f"rect_x_min: {rect.x_min}\nrect_x_max: {rect.x_max}\n"
        f"rect_y_min: {rect.y_min}\nrect_y_max: {rect.y_max}\n"
        f"corner_radius: {rect.corner_radius}\n"
        f"segments_per_corner: {rect.segments_per_corner}\n"
        f"stop_line_segments: \"{seg}\"\n"
    )


def parse_loop_params(text: str) -> "LoopFile":
    """Parse loop_params.yaml text into a LoopFile (inverse of the *_yaml writers).

    Old circle files (no ``shape`` field) parse as shape='circle'. A
    rounded_rectangle file additionally carries rect_* geometry. Raises KeyError
    on a missing required field and ValueError on a non-numeric value.
    """
    data = yaml.safe_load(text) or {}
    shape = str(data.get("shape", "circle"))
    rect = None
    if shape == "rounded_rectangle":
        seg_raw = str(data.get("stop_line_segments", "")).strip()
        segs = tuple(int(s) for s in seg_raw.split(",") if s != "")
        rect = RoundedRectFile(
            x_min=float(data["rect_x_min"]), x_max=float(data["rect_x_max"]),
            y_min=float(data["rect_y_min"]), y_max=float(data["rect_y_max"]),
            corner_radius=float(data["corner_radius"]),
            segments_per_corner=int(data["segments_per_corner"]),
            stop_line_segments=segs,
        )
    return LoopFile(
        center_x=float(data["center_x"]),
        center_y=float(data["center_y"]),
        radius=float(data["radius"]),
        lane_width=float(data["lane_width"]),
        speed_limit=float(data["speed_limit"]),
        num_segments=int(data["num_segments"]),
        travel_direction=str(data.get("travel_direction", LOADED_TRAVEL_DIRECTION)),
        shape=shape,
        rect=rect,
    )


def circle_centerline_vertices(
    center_x: float, center_y: float, radius: float, num_segments: int
) -> list[tuple[float, float]]:
    """Vertices of a regular ``num_segments``-gon on the circle (CCW order)."""
    if num_segments < 3:
        raise ValueError(f"num_segments must be >= 3, got {num_segments}")
    if radius <= 0.0:
        raise ValueError(f"radius must be > 0, got {radius}")
    return [
        (center_x + radius * math.cos(2.0 * math.pi * i / num_segments),
         center_y + radius * math.sin(2.0 * math.pi * i / num_segments))
        for i in range(num_segments)
    ]


def rounded_rect_centerline_vertices(
    x_min: float, x_max: float, y_min: float, y_max: float,
    corner_radius: float, segments_per_corner: int = 6,
) -> list[tuple[float, float]]:
    """Vertices of a rounded-rectangle centerline (CCW), straights joined by four
    quarter-circle corners of ``corner_radius``. Raises ValueError if the radius
    does not fit the rectangle (> half the shorter side) or is non-positive.

    Each corner arc is sampled with ``segments_per_corner + 1`` points (including
    both endpoints). The straight run between consecutive corners is represented by
    one additional midpoint (at the midpoint of the run) if the run has nonzero
    length, ensuring axis-aligned straight runs have vertices on them. Duplicate
    points are removed after assembly so shared endpoints are not repeated.
    """
    if corner_radius <= 0.0:
        raise ValueError(f"corner_radius must be > 0, got {corner_radius}")
    w, h = x_max - x_min, y_max - y_min
    if w <= 0.0 or h <= 0.0:
        raise ValueError(f"degenerate rectangle {w} x {h}")
    if corner_radius > min(w, h) / 2.0 + 1e-9:
        raise ValueError(
            f"corner_radius {corner_radius} exceeds half the shorter side "
            f"{min(w, h) / 2.0}"
        )
    r = corner_radius
    # Each entry: (corner_center_x, corner_center_y, arc_start_angle_radians).
    # Arc sweeps CCW (increasing angle) by π/2. Traversal: bottom-right corner
    # (270°→360°), top-right (0°→90°), top-left (90°→180°), bottom-left (180°→270°).
    corners = [
        (x_max - r, y_min + r, 1.5 * math.pi),   # bottom-right: 270°→360°
        (x_max - r, y_max - r, 0.0),              # top-right:    0°→90°
        (x_min + r, y_max - r, 0.5 * math.pi),   # top-left:     90°→180°
        (x_min + r, y_min + r, math.pi),          # bottom-left:  180°→270°
    ]
    # Pre-compute the arc endpoint that each corner ENDS on (= start of next straight).
    corner_ends: list[tuple[float, float]] = []
    for cx, cy, a0 in corners:
        a_end = a0 + math.pi / 2.0
        corner_ends.append((cx + r * math.cos(a_end), cy + r * math.sin(a_end)))

    verts: list[tuple[float, float]] = []
    n = len(corners)
    for i in range(n):
        cx, cy, a0 = corners[i]
        # Emit corner arc (segments_per_corner+1 points including both endpoints).
        for k in range(segments_per_corner + 1):
            a = a0 + (math.pi / 2.0) * k / segments_per_corner
            verts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
        # Emit the midpoint of the straight run to the next corner's start.
        run_start = corner_ends[i]
        # Next corner starts at: (cx_next + r*cos(a0_next), cy_next + r*sin(a0_next))
        cx_next, cy_next, a0_next = corners[(i + 1) % n]
        run_end = (cx_next + r * math.cos(a0_next), cy_next + r * math.sin(a0_next))
        mid = ((run_start[0] + run_end[0]) / 2.0, (run_start[1] + run_end[1]) / 2.0)
        if math.hypot(run_end[0] - run_start[0], run_end[1] - run_start[1]) > 1e-9:
            verts.append(mid)

    deduped: list[tuple[float, float]] = []
    for v in verts:
        if not deduped or math.hypot(v[0] - deduped[-1][0], v[1] - deduped[-1][1]) > 1e-9:
            deduped.append(v)
    if math.hypot(deduped[0][0] - deduped[-1][0], deduped[0][1] - deduped[-1][1]) <= 1e-9:
        deduped.pop()
    return deduped


def occupancy_to_loop_osm(
    data: list[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    *,
    lane_width: float,
    wall_clearance: float,
    margin: float,
    max_radius: float,
    speed_limit: float,
    num_segments: int,
    occupied_threshold: int = 50,
    treat_unknown_as_occupied: bool = True,
) -> "tuple[str, LoopParams]":
    """Find the largest free rectangle, fit a loop, and return (osm, params).

    ``wall_clearance`` sizes the radius (the physical room the robot needs);
    ``lane_width`` is the wider DRAWN lanelet width used only for the OSM bounds.
    Keeping them separate lets the lanelet be drawn wide (for path_generator)
    without shrinking the loop radius.
    """
    rect = largest_free_rectangle(
        data, width, height, occupied_threshold, treat_unknown_as_occupied
    )
    params = rect_to_loop_params(
        rect, resolution, origin_x, origin_y, wall_clearance, margin, max_radius
    )
    osm = generate_circle_loop_osm(
        params.center_x, params.center_y, params.radius,
        lane_width, speed_limit, num_segments,
    )
    return osm, params


@dataclass(frozen=True)
class StopLineSpec:
    """A stop line to emit at the START vertex of lanelet ``segment_index``."""

    segment_index: int


def generate_loop_osm_with_stop_lines(
    vertices: list[tuple[float, float]],
    lane_width: float,
    speed_limit: float,
    stop_lines: "list[StopLineSpec] | None" = None,
) -> str:
    """generate_loop_osm + Autoware stop-line regulatory geometry.

    Each StopLineSpec emits: a ``type=stop_line`` way across the lane at the
    segment's start vertex; a small ``type=traffic_sign, subtype=stop_sign``
    refers way beside the lane; a ``type=regulatory_element, subtype=traffic_sign``
    relation with ref_line=stop_line + refers=sign; and a ``regulatory_element``
    member added to that segment's lanelet. This is exactly what
    autoware_behavior_velocity_stop_line_module consumes. Raises ValueError on an
    out-of-range segment_index.
    """
    base = generate_loop_osm(vertices, lane_width, speed_limit)
    if not stop_lines:  # None or empty -> no stop-line geometry to add
        return base
    n = len(vertices)
    normals = _vertex_normals(vertices)
    half = lane_width / 2.0
    nid = 100 * n + 1
    extra_nodes, extra_ways, extra_rels = [], [], []
    lanelet_regelem_members: dict[int, list[int]] = {}

    for spec in stop_lines:
        if not (0 <= spec.segment_index < n):
            raise ValueError(f"segment_index {spec.segment_index} out of range [0,{n})")
        cx, cy = vertices[spec.segment_index]
        nx, ny = normals[spec.segment_index]
        a_id, b_id = nid, nid + 1
        extra_nodes.append(_node_xml(a_id, cx + half * nx, cy + half * ny))
        extra_nodes.append(_node_xml(b_id, cx - half * nx, cy - half * ny))
        stop_way = nid + 2
        extra_ways.append(
            f'  <way id="{stop_way}"><nd ref="{a_id}"/><nd ref="{b_id}"/>'
            f'<tag k="type" v="stop_line"/></way>'
        )
        s_id, t_id = nid + 3, nid + 4
        ox, oy = cx + (half + 0.1) * nx, cy + (half + 0.1) * ny
        extra_nodes.append(_node_xml(s_id, ox - 0.05 * ny, oy + 0.05 * nx))
        extra_nodes.append(_node_xml(t_id, ox + 0.05 * ny, oy - 0.05 * nx))
        sign_way = nid + 5
        extra_ways.append(
            f'  <way id="{sign_way}"><nd ref="{s_id}"/><nd ref="{t_id}"/>'
            f'<tag k="type" v="traffic_sign"/><tag k="subtype" v="stop_sign"/></way>'
        )
        regelem = nid + 6
        extra_rels.append(
            f'  <relation id="{regelem}">\n'
            f'    <member type="way" ref="{stop_way}" role="ref_line"/>\n'
            f'    <member type="way" ref="{sign_way}" role="refers"/>\n'
            f'    <tag k="type" v="regulatory_element"/>\n'
            f'    <tag k="subtype" v="traffic_sign"/>\n'
            f"  </relation>"
        )
        lanelet_id = 4 * n + spec.segment_index + 1
        lanelet_regelem_members.setdefault(lanelet_id, []).append(regelem)
        nid += 10

    out = base.replace(_OSM_FOOTER, "")
    out += "\n".join(extra_nodes) + "\n" + "\n".join(extra_ways) + "\n" + "\n".join(extra_rels) + "\n"
    for lanelet_id, regelems in lanelet_regelem_members.items():
        members = "".join(
            f'    <member type="relation" ref="{re}" role="regulatory_element"/>\n'
            for re in regelems
        )
        anchor = f'  <relation id="{lanelet_id}">\n'
        out = out.replace(anchor, anchor + members, 1)
    return out + _OSM_FOOTER
