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
        lane_width=0.6, margin=0.05, max_radius=0.9,
    )
    assert params.center_x == pytest.approx(-2.0 + 0.5)
    assert params.center_y == pytest.approx(-3.0 + 0.5)


def test_radius_fits_inside_small_rect() -> None:
    # 1.0 m x 1.0 m free area: usable = 0.5 - 0.3 - 0.05 = 0.15 m < cap.
    rect = FreeRectangle(row0=0, col0=0, row1=19, col1=19)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        lane_width=0.6, margin=0.05, max_radius=0.9,
    )
    assert params.radius == pytest.approx(0.15)


def test_radius_capped_in_large_rect() -> None:
    # 3.0 m x 3.0 m free area: usable = 1.5 - 0.3 - 0.05 = 1.15 m, capped to 0.9.
    rect = FreeRectangle(row0=0, col0=0, row1=59, col1=59)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        lane_width=0.6, margin=0.05, max_radius=0.9,
    )
    assert params.radius == pytest.approx(0.9)


def test_radius_uses_shorter_side() -> None:
    # Wide-but-short rect: 3.0 m wide x 1.0 m tall -> shorter side (1.0) governs.
    rect = FreeRectangle(row0=0, col0=0, row1=19, col1=59)
    params = rect_to_loop_params(
        rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
        lane_width=0.6, margin=0.05, max_radius=0.9,
    )
    assert params.radius == pytest.approx(0.15)


def test_rect_too_small_raises() -> None:
    rect = FreeRectangle(row0=0, col0=0, row1=5, col1=5)  # 0.3 m x 0.3 m
    with pytest.raises(ValueError):
        rect_to_loop_params(
            rect, resolution=0.05, origin_x=0.0, origin_y=0.0,
            lane_width=0.6, margin=0.05, max_radius=0.9,
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


from kachaka_autoware_maps.loop_map_gen import occupancy_to_loop_osm  # noqa: E402


def test_occupancy_to_loop_osm_end_to_end() -> None:
    # 80x80 free cells at 0.05 m = 4 m x 4 m free area centred via origin.
    data = [0] * (80 * 80)
    osm, params = occupancy_to_loop_osm(
        data, width=80, height=80, resolution=0.05,
        origin_x=-2.0, origin_y=-2.0,
        lane_width=0.6, margin=0.05, max_radius=0.9,
        speed_limit=0.3, num_segments=16,
    )
    # Free area centre is (0, 0) in the map frame (origin -2 + 2 m).
    assert params.center_x == pytest.approx(0.0)
    assert params.center_y == pytest.approx(0.0)
    assert params.radius == pytest.approx(0.9)  # capped
    root = _parse(osm)
    assert len(root.findall("relation")) == 16
