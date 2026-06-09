# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import math
import xml.etree.ElementTree as ET

import pytest

from kachaka_autoware_maps.loop_map_gen import (
    FreeRectangle,
    largest_free_rectangle,
)


def test_all_free_grid_returns_whole_grid() -> None:
    # 3 wide x 2 tall, every cell free (0).
    rect = largest_free_rectangle([0] * 6, width=3, height=2)
    assert rect == FreeRectangle(row0=0, col0=0, row1=1, col1=2)
    assert rect.area == 6


def test_occupied_column_splits_grid_and_larger_side_wins() -> None:
    # 4 wide x 3 tall, col 2 fully occupied (100). Left block (cols 0-1, 3 tall)
    # has area 6; right block (col 3, 3 tall) area 3. Larger side wins.
    data = [
        0, 0, 100, 0,
        0, 0, 100, 0,
        0, 0, 100, 0,
    ]
    rect = largest_free_rectangle(data, width=4, height=3)
    assert rect == FreeRectangle(row0=0, col0=0, row1=2, col1=1)


def test_unknown_cells_excluded_by_default() -> None:
    # -1 is unknown; treated as occupied by default, so only cols 1-2 are free.
    rect = largest_free_rectangle([-1, 0, 0], width=3, height=1)
    assert rect == FreeRectangle(row0=0, col0=1, row1=0, col1=2)


def test_unknown_cells_included_when_flag_set() -> None:
    rect = largest_free_rectangle(
        [-1, 0, 0], width=3, height=1, treat_unknown_as_occupied=False
    )
    assert rect == FreeRectangle(row0=0, col0=0, row1=0, col1=2)


def test_occupied_threshold_controls_freeness() -> None:
    # value 40 counts as free with threshold 50, occupied with threshold 30.
    assert largest_free_rectangle([40], 1, 1, occupied_threshold=50).area == 1
    with pytest.raises(ValueError):
        largest_free_rectangle([40], 1, 1, occupied_threshold=30)


def test_bad_dimensions_raise() -> None:
    with pytest.raises(ValueError):
        largest_free_rectangle([0, 0], width=3, height=1)  # length mismatch
    with pytest.raises(ValueError):
        largest_free_rectangle([], width=0, height=0)


def test_no_free_cell_raises() -> None:
    with pytest.raises(ValueError):
        largest_free_rectangle([100, 100, 100, 100], width=2, height=2)


from kachaka_autoware_maps.loop_map_gen import (  # noqa: E402
    LoopParams,
    rect_to_loop_params,
)


def test_rect_center_maps_to_map_frame() -> None:
    # 20x20-cell free rect at the grid origin, 0.05 m cells. The rectangle spans
    # map corners (0,0)..(1,1) m, so its centre is (0.5, 0.5) m plus the grid
    # origin offset.
    rect = FreeRectangle(row0=0, col0=0, row1=19, col1=19)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=-2.0, origin_y=-3.0,
        wall_clearance=0.3, margin=0.05, max_radius=0.9,
    )
    assert params.center_x == pytest.approx(-2.0 + 0.5)
    assert params.center_y == pytest.approx(-3.0 + 0.5)


def test_radius_fits_inside_small_rect() -> None:
    # 1.0 m x 1.0 m free area: usable = 0.5 - 0.3 - 0.05 = 0.15 m < cap.
    rect = FreeRectangle(row0=0, col0=0, row1=19, col1=19)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        wall_clearance=0.3, margin=0.05, max_radius=0.9,
    )
    assert params.radius == pytest.approx(0.15)


def test_radius_capped_in_large_rect() -> None:
    # 3.0 m x 3.0 m free area: usable = 1.5 - 0.3 - 0.05 = 1.15 m, capped to 0.9.
    rect = FreeRectangle(row0=0, col0=0, row1=59, col1=59)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        wall_clearance=0.3, margin=0.05, max_radius=0.9,
    )
    assert params.radius == pytest.approx(0.9)


def test_radius_uses_shorter_side() -> None:
    # Wide-but-short rect: 3.0 m wide x 1.0 m tall -> shorter side (1.0) governs.
    rect = FreeRectangle(row0=0, col0=0, row1=19, col1=59)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        wall_clearance=0.3, margin=0.05, max_radius=0.9,
    )
    assert params.radius == pytest.approx(0.15)


def test_rect_too_small_raises() -> None:
    rect = FreeRectangle(row0=0, col0=0, row1=5, col1=5)  # 0.3 m x 0.3 m
    with pytest.raises(ValueError):
        rect_to_loop_params(
            rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
            wall_clearance=0.3, margin=0.05, max_radius=0.9,
        )


from kachaka_autoware_maps.loop_map_gen import (  # noqa: E402
    MAP_PROJECTOR_INFO_YAML,
    generate_circle_loop_osm,
    loop_params_yaml,
)


def _parse(osm: str) -> ET.Element:
    return ET.fromstring(osm)


def test_osm_is_valid_xml_with_osm_root() -> None:
    root = _parse(generate_circle_loop_osm(0.0, 0.0, 0.9, 0.6, 0.3, num_segments=8))
    assert root.tag == "osm"


def test_counts_scale_with_segments() -> None:
    root = _parse(generate_circle_loop_osm(0.0, 0.0, 0.9, 0.6, 0.3, num_segments=8))
    # 2 nodes (inner+outer) per cross-section, 2 ways + 1 lanelet per arc.
    assert len(root.findall("node")) == 16
    assert len(root.findall("way")) == 16
    assert len(root.findall("relation")) == 8


def test_every_lanelet_is_one_way_road() -> None:
    root = _parse(generate_circle_loop_osm(0.0, 0.0, 0.9, 0.6, 0.3, num_segments=6))
    relations = root.findall("relation")
    assert len(relations) == 6
    for rel in relations:
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        assert tags["type"] == "lanelet"
        assert tags["subtype"] == "road"
        assert tags["one_way"] == "yes"
        # Each lanelet references exactly one left and one right way.
        roles = sorted(m.get("role") for m in rel.findall("member"))
        assert roles == ["left", "right"]


def test_inner_and_outer_radii_match_lane_width() -> None:
    cx, cy, r, w = 1.0, 2.0, 0.9, 0.6
    root = _parse(generate_circle_loop_osm(cx, cy, r, w, 0.3, num_segments=4))
    radii = set()
    for node in root.findall("node"):
        tags = {t.get("k"): float(t.get("v")) for t in node.findall("tag") if t.get("k") in ("local_x", "local_y")}
        radii.add(round(math.hypot(tags["local_x"] - cx, tags["local_y"] - cy), 3))
    assert radii == {round(r - w / 2, 3), round(r + w / 2, 3)}


def test_loop_is_closed_last_arc_reuses_first_nodes() -> None:
    # The last lanelet's bound ways must end on cross-section 0's node ids, so
    # the routing graph forms a cycle. Collect each way's node refs and assert
    # exactly one way starts where another ends, all the way around (every node
    # id appears as a start exactly once and an end exactly once).
    root = _parse(generate_circle_loop_osm(0.0, 0.0, 0.9, 0.6, 0.3, num_segments=5))
    starts: list[str] = []
    ends: list[str] = []
    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        starts.append(refs[0])
        ends.append(refs[-1])
    assert sorted(starts) == sorted(ends)  # closed loop: no dangling endpoint


def test_segment_count_floor_is_three() -> None:
    with pytest.raises(ValueError):
        generate_circle_loop_osm(0.0, 0.0, 0.9, 0.6, 0.3, num_segments=2)


def test_radius_must_exceed_half_lane_width() -> None:
    with pytest.raises(ValueError):
        generate_circle_loop_osm(0.0, 0.0, 0.2, 0.6, 0.3, num_segments=8)


def test_sidecars_have_expected_content() -> None:
    assert "projector_type: Local" in MAP_PROJECTOR_INFO_YAML
    assert "vertical_datum: WGS84" in MAP_PROJECTOR_INFO_YAML
    yaml = loop_params_yaml(LoopParams(1.0, 2.0, 0.9), lane_width=0.6, speed_limit=0.3, num_segments=16)
    assert "center_x: 1.0" in yaml
    assert "radius: 0.9" in yaml
    assert "num_segments: 16" in yaml


def test_speed_limit_tag_is_kmh_from_mps() -> None:
    root = _parse(generate_circle_loop_osm(0.0, 0.0, 0.9, 0.6, 0.3, num_segments=4))
    rel = root.findall("relation")[0]
    tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
    # 0.3 m/s -> 1.08 km/h (lanelet2 speed_limit is km/h)
    assert float(tags["speed_limit"]) == pytest.approx(0.3 * 3.6)


from kachaka_autoware_maps.loop_map_gen import occupancy_to_loop_osm  # noqa: E402


def test_occupancy_to_loop_osm_end_to_end() -> None:
    # 80x80 free cells at 0.05 m = 4 m x 4 m free area centred via origin.
    data = [0] * (80 * 80)
    osm, params = occupancy_to_loop_osm(
        data, width=80, height=80, resolution=0.05,
        origin_x=-2.0, origin_y=-2.0,
        lane_width=0.6, wall_clearance=0.3, margin=0.05, max_radius=0.9,
        speed_limit=0.3, num_segments=16,
    )
    # Free area centre is (0, 0) in the map frame (origin -2 + 2 m).
    assert params.center_x == pytest.approx(0.0)
    assert params.center_y == pytest.approx(0.0)
    assert params.radius == pytest.approx(0.9)  # capped
    root = _parse(osm)
    assert len(root.findall("relation")) == 16


from kachaka_autoware_maps.loop_map_gen import (  # noqa: E402
    LOADED_TRAVEL_DIRECTION,
    LoopFile,
    parse_loop_params,
)


def test_loop_params_yaml_includes_travel_direction() -> None:
    text = loop_params_yaml(
        LoopParams(1.0, 2.0, 0.9), lane_width=0.6, speed_limit=0.3, num_segments=16
    )
    assert "travel_direction: counterclockwise" in text


def test_loop_params_round_trip() -> None:
    text = loop_params_yaml(
        LoopParams(1.5, -0.5, 0.81), lane_width=0.6, speed_limit=0.3, num_segments=12
    )
    assert parse_loop_params(text) == LoopFile(
        1.5, -0.5, 0.81, 0.6, 0.3, 12, LOADED_TRAVEL_DIRECTION
    )


def test_parse_loop_params_defaults_direction_for_old_files() -> None:
    text = (
        "center_x: 0.0\ncenter_y: 0.0\nradius: 0.9\n"
        "lane_width: 0.6\nspeed_limit: 0.3\nnum_segments: 16\n"
    )
    assert parse_loop_params(text).travel_direction == LOADED_TRAVEL_DIRECTION


def test_loaded_loop_is_a_routable_cycle() -> None:
    """The generated loop loads as a single routable one-way cycle.

    This asserts CONNECTIVITY (the real invariant the generator must hold), not a
    direction: lanelet2's winding normalisation is not load-invariant (a raw load
    winds this clockwise, Autoware's loader counter-clockwise), so set_loop_route
    reads the actual direction from the live /map/vector_map. Skipped where the
    lanelet2 python bindings are not installed.
    """
    import os
    import tempfile

    pytest.importorskip("lanelet2")
    from lanelet2 import io, projection, routing, traffic_rules

    osm = generate_circle_loop_osm(0.98, 0.13, 0.81, 0.6, 0.3, 16)
    handle = tempfile.NamedTemporaryFile("w", suffix=".osm", delete=False)
    handle.write(osm)
    handle.close()
    try:
        m = io.load(handle.name, projection.UtmProjector(io.Origin(0.0, 0.0)))
    finally:
        os.unlink(handle.name)
    for p in m.pointLayer:
        a = p.attributes
        if "local_x" in a:
            p.x = float(a["local_x"])
            p.y = float(a["local_y"])
    rules = traffic_rules.create(
        traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle
    )
    graph = routing.RoutingGraph(m, rules)
    lanelets = sorted(m.laneletLayer, key=lambda L: L.id)
    assert len(lanelets) == 16
    # Every lanelet has exactly one successor, and following the chain forms one
    # cycle visiting all 16 lanelets exactly once.
    for ll in lanelets:
        assert len(graph.following(ll)) == 1
    visited = []
    cur = lanelets[0]
    for _ in range(16):
        visited.append(cur.id)
        cur = graph.following(cur)[0]
    assert cur.id == lanelets[0].id  # closed the cycle
    assert sorted(visited) == [ll.id for ll in lanelets]  # all, once each


def test_radius_uses_wall_clearance_not_drawn_lane_width() -> None:
    # 3.0 m x 3.0 m free area, generous cap so the clearance term governs.
    rect = FreeRectangle(row0=0, col0=0, row1=59, col1=59)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        wall_clearance=0.3, margin=0.05, max_radius=2.0,
    )
    # usable = 1.5 - 0.3 - 0.05 = 1.15; sized by the PHYSICAL clearance only.
    assert params.radius == pytest.approx(1.15)


def test_wide_drawn_lane_does_not_shrink_the_radius() -> None:
    # The working map: a wide planner-facing lane (1.3 m) must NOT eat into the
    # radius, which is sized only by wall_clearance. Same rect + same clearance
    # -> same radius whether the drawn lane is 0.6 or 1.3, and the 1.3 m lane is
    # actually drawn (inner = radius - 1.3/2).
    data = [0] * (60 * 60)  # 3.0 m x 3.0 m
    _, narrow = occupancy_to_loop_osm(
        data, width=60, height=60, resolution=0.05, origin_x=0.0, origin_y=0.0,
        lane_width=0.6, wall_clearance=0.3, margin=0.05, max_radius=2.0,
        speed_limit=0.3, num_segments=16,
    )
    osm_wide, wide = occupancy_to_loop_osm(
        data, width=60, height=60, resolution=0.05, origin_x=0.0, origin_y=0.0,
        lane_width=1.3, wall_clearance=0.3, margin=0.05, max_radius=2.0,
        speed_limit=0.3, num_segments=16,
    )
    assert wide.radius == pytest.approx(narrow.radius)
    root = ET.fromstring(osm_wide)
    radii = set()
    for node in root.findall("node"):
        tags = {
            t.get("k"): float(t.get("v"))
            for t in node.findall("tag")
            if t.get("k") in ("local_x", "local_y")
        }
        radii.add(round(math.hypot(tags["local_x"] - wide.center_x, tags["local_y"] - wide.center_y), 3))
    assert radii == {round(wide.radius - 0.65, 3), round(wide.radius + 0.65, 3)}


from kachaka_autoware_maps.loop_map_gen import (  # noqa: E402
    circle_centerline_vertices,
    rounded_rect_centerline_vertices,
)


def test_circle_vertices_count_and_radius():
    v = circle_centerline_vertices(1.0, 2.0, 0.8, num_segments=16)
    assert len(v) == 16
    for x, y in v:
        assert math.hypot(x - 1.0, y - 2.0) == pytest.approx(0.8)


def test_rounded_rect_vertices_lie_within_the_rectangle():
    v = rounded_rect_centerline_vertices(0.0, 3.0, 0.0, 2.0, corner_radius=0.5,
                                         segments_per_corner=6)
    assert len(v) >= 4 * 6
    for x, y in v:
        assert -1e-9 <= x <= 3.0 + 1e-9
        assert -1e-9 <= y <= 2.0 + 1e-9


def test_rounded_rect_has_axis_aligned_straight_runs():
    v = rounded_rect_centerline_vertices(0.0, 4.0, 0.0, 4.0, corner_radius=1.0,
                                         segments_per_corner=4)
    ys_on_bottom = [y for x, y in v if 1.0 < x < 3.0 and y < 0.5]
    assert ys_on_bottom and all(yy == pytest.approx(0.0) for yy in ys_on_bottom)


def test_rounded_rect_corner_radius_must_fit():
    with pytest.raises(ValueError):
        rounded_rect_centerline_vertices(0.0, 5.0, 0.0, 2.0, corner_radius=2.0)


from kachaka_autoware_maps.loop_map_gen import generate_loop_osm  # noqa: E402


def test_generate_loop_osm_counts_match_segments():
    verts = circle_centerline_vertices(0.0, 0.0, 0.9, 8)
    root = _parse(generate_loop_osm(verts, lane_width=0.6, speed_limit=0.3))
    assert len(root.findall("node")) == 16
    assert len(root.findall("way")) == 16
    assert len(root.findall("relation")) == 8


def test_generate_loop_osm_bounds_are_lane_width_apart():
    verts = circle_centerline_vertices(1.0, 2.0, 0.9, 4)
    root = _parse(generate_loop_osm(verts, lane_width=0.6, speed_limit=0.3))
    radii = set()
    for node in root.findall("node"):
        t = {k.get("k"): float(k.get("v")) for k in node.findall("tag")
             if k.get("k") in ("local_x", "local_y")}
        radii.add(round(math.hypot(t["local_x"] - 1.0, t["local_y"] - 2.0), 3))
    assert radii == {round(0.9 - 0.3, 3), round(0.9 + 0.3, 3)}


def test_rounded_rect_osm_loads_as_routable_cycle():
    pytest.importorskip("lanelet2")
    import os
    import tempfile
    from lanelet2 import io, projection, routing, traffic_rules

    verts = rounded_rect_centerline_vertices(0.0, 3.0, 0.0, 2.0, corner_radius=0.5,
                                             segments_per_corner=4)
    osm = generate_loop_osm(verts, lane_width=0.6, speed_limit=0.3)
    h = tempfile.NamedTemporaryFile("w", suffix=".osm", delete=False)
    h.write(osm)
    h.close()
    try:
        m = io.load(h.name, projection.UtmProjector(io.Origin(0.0, 0.0)))
    finally:
        os.unlink(h.name)
    for p in m.pointLayer:
        a = p.attributes
        if "local_x" in a:
            p.x, p.y = float(a["local_x"]), float(a["local_y"])
    rules = traffic_rules.create(traffic_rules.Locations.Germany, traffic_rules.Participants.Vehicle)
    graph = routing.RoutingGraph(m, rules)
    lanelets = list(m.laneletLayer)
    assert len(lanelets) == len(verts)
    for ll in lanelets:
        assert len(graph.following(ll)) == 1


from kachaka_autoware_maps.loop_map_gen import (  # noqa: E402
    StopLineSpec,
    generate_loop_osm_with_stop_lines,
)


def _rect_osm_with_stops():
    verts = rounded_rect_centerline_vertices(0.0, 4.0, 0.0, 4.0, corner_radius=1.0,
                                             segments_per_corner=4)
    n = len(verts)
    specs = [StopLineSpec(segment_index=n // 4), StopLineSpec(segment_index=3 * n // 4)]
    return generate_loop_osm_with_stop_lines(verts, lane_width=0.8, speed_limit=0.3,
                                             stop_lines=specs), specs


def test_stop_line_ways_and_regelems_emitted():
    osm, specs = _rect_osm_with_stops()
    root = _parse(osm)
    stop_line_ways = [w for w in root.findall("way")
                      if any(t.get("k") == "type" and t.get("v") == "stop_line"
                             for t in w.findall("tag"))]
    assert len(stop_line_ways) == len(specs)
    regelems = [r for r in root.findall("relation")
                if any(t.get("k") == "subtype" and t.get("v") == "traffic_sign"
                       for t in r.findall("tag"))]
    assert len(regelems) == len(specs)
    for rel in regelems:
        roles = sorted(m.get("role") for m in rel.findall("member"))
        assert roles == ["ref_line", "refers"]
    sign_ways = [w for w in root.findall("way")
                 if any(t.get("k") == "subtype" and t.get("v") == "stop_sign"
                        for t in w.findall("tag"))]
    assert len(sign_ways) == len(specs)


def test_each_stop_line_is_referenced_by_a_lanelet():
    osm, specs = _rect_osm_with_stops()
    root = _parse(osm)
    regelem_ids = {r.get("id") for r in root.findall("relation")
                   if any(t.get("k") == "subtype" and t.get("v") == "traffic_sign"
                          for t in r.findall("tag"))}
    referenced = set()
    for rel in root.findall("relation"):
        for m in rel.findall("member"):
            if m.get("role") == "regulatory_element":
                referenced.add(m.get("ref"))
    assert regelem_ids <= referenced


def test_stop_line_crosses_the_lane_width():
    osm, _ = _rect_osm_with_stops()
    root = _parse(osm)
    nodes = {nd.get("id"): nd for nd in root.findall("node")}
    for way in root.findall("way"):
        if any(t.get("k") == "type" and t.get("v") == "stop_line" for t in way.findall("tag")):
            refs = [nd.get("ref") for nd in way.findall("nd")]
            pts = []
            for rid in refs:
                t = {k.get("k"): float(k.get("v")) for k in nodes[rid].findall("tag")
                     if k.get("k") in ("local_x", "local_y")}
                pts.append((t["local_x"], t["local_y"]))
            span = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
            assert span == pytest.approx(0.8, abs=0.05)


def test_stop_line_regelem_attached_to_lanelet_via_lanelet2():
    pytest.importorskip("lanelet2")
    import os
    import tempfile
    from lanelet2 import io, projection

    osm, specs = _rect_osm_with_stops()
    h = tempfile.NamedTemporaryFile("w", suffix=".osm", delete=False)
    h.write(osm)
    h.close()
    try:
        m = io.load(h.name, projection.UtmProjector(io.Origin(0.0, 0.0)))
    finally:
        os.unlink(h.name)
    with_regelem = [ll for ll in m.laneletLayer if len(ll.regulatoryElements) > 0]
    assert len(with_regelem) >= len(specs)


def test_stop_line_out_of_range_segment_raises():
    verts = rounded_rect_centerline_vertices(0.0, 4.0, 0.0, 4.0, corner_radius=1.0,
                                             segments_per_corner=4)
    with pytest.raises(ValueError):
        generate_loop_osm_with_stop_lines(verts, lane_width=0.8, speed_limit=0.3,
                                          stop_lines=[StopLineSpec(segment_index=len(verts))])


from kachaka_autoware_maps.loop_map_gen import (  # noqa: E402
    RoundedRectFile,
    rounded_rect_params_yaml,
)


def test_old_circle_file_parses_with_shape_circle():
    text = (
        "center_x: 0.0\ncenter_y: 0.0\nradius: 0.9\n"
        "lane_width: 1.3\nspeed_limit: 0.3\nnum_segments: 16\n"
    )
    f = parse_loop_params(text)
    assert f.shape == "circle"
    assert f.radius == pytest.approx(0.9)


def test_rounded_rect_sidecar_round_trip():
    text = rounded_rect_params_yaml(
        RoundedRectFile(x_min=0.0, x_max=3.0, y_min=0.0, y_max=2.0,
                        corner_radius=0.5, segments_per_corner=6,
                        stop_line_segments=(3, 9)),
        lane_width=0.8, speed_limit=0.3, travel_direction="counterclockwise",
    )
    f = parse_loop_params(text)
    assert f.shape == "rounded_rectangle"
    assert f.rect == RoundedRectFile(0.0, 3.0, 0.0, 2.0, 0.5, 6, (3, 9))
    assert f.lane_width == pytest.approx(0.8)
    assert f.travel_direction == "counterclockwise"


from kachaka_autoware_maps.loop_map_gen import (  # noqa: E402
    occupancy_to_rounded_rect_osm,
    rect_to_rounded_rect_params,
)


def test_rect_to_rounded_rect_insets_by_clearance_and_caps_corner():
    rect = FreeRectangle(row0=0, col0=0, row1=59, col1=59)  # 3 m x 3 m
    rr = rect_to_rounded_rect_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        wall_clearance=0.3, margin=0.05, lane_width=0.8, corner_radius_request=0.45,
    )
    assert rr.x_min == pytest.approx(0.35)
    assert rr.x_max == pytest.approx(2.65)
    assert rr.corner_radius == pytest.approx(0.45)


def test_rect_to_rounded_rect_enforces_corner_radius_gt_half_lane():
    rect = FreeRectangle(row0=0, col0=0, row1=59, col1=59)
    with pytest.raises(ValueError):
        rect_to_rounded_rect_params(
            rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
            wall_clearance=0.3, margin=0.05, lane_width=1.3, corner_radius_request=0.5,
        )


def test_occupancy_to_rounded_rect_osm_end_to_end():
    data = [0] * (80 * 80)  # 4 m x 4 m free
    osm, loop_file = occupancy_to_rounded_rect_osm(
        data, width=80, height=80, resolution=0.05, origin_x=-2.0, origin_y=-2.0,
        lane_width=0.8, wall_clearance=0.3, margin=0.05, corner_radius=0.5,
        speed_limit=0.3, segments_per_corner=6, stop_lines_per_corner=1,
    )
    assert loop_file.shape == "rounded_rectangle"
    root = _parse(osm)
    stop_line_ways = [w for w in root.findall("way")
                      if any(t.get("k") == "type" and t.get("v") == "stop_line"
                             for t in w.findall("tag"))]
    assert len(stop_line_ways) == 4


def test_large_corner_radius_still_yields_four_stop_lines():
    # corner_radius_request far larger than the rect -> clamped, but Fix 1 keeps a
    # straight so all four corner stop lines are still emitted (not silently zero).
    data = [0] * (80 * 80)  # 4 m x 4 m free
    osm, loop_file = occupancy_to_rounded_rect_osm(
        data, width=80, height=80, resolution=0.05, origin_x=-2.0, origin_y=-2.0,
        lane_width=0.8, wall_clearance=0.3, margin=0.05, corner_radius=5.0,
        speed_limit=0.3, segments_per_corner=6, stop_lines_per_corner=1,
    )
    root = _parse(osm)
    stop_line_ways = [w for w in root.findall("way")
                      if any(t.get("k") == "type" and t.get("v") == "stop_line"
                             for t in w.findall("tag"))]
    assert len(stop_line_ways) == 4
    assert len(loop_file.rect.stop_line_segments) == 4


def test_stop_lines_per_corner_above_one_raises():
    data = [0] * (80 * 80)
    with pytest.raises(ValueError):
        occupancy_to_rounded_rect_osm(
            data, width=80, height=80, resolution=0.05, origin_x=-2.0, origin_y=-2.0,
            lane_width=0.8, wall_clearance=0.3, margin=0.05, corner_radius=0.5,
            speed_limit=0.3, segments_per_corner=6, stop_lines_per_corner=2,
        )
