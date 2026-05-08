# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

from pathlib import Path

from kachaka_autoware_maps.sidecars import write_sidecars_if_absent


def test_creates_all_three_files(tmp_path: Path) -> None:
    write_sidecars_if_absent(tmp_path)

    metadata = tmp_path / "pointcloud_map" / "metadata.yaml"
    lanelet = tmp_path / "lanelet2_map.osm"
    projector = tmp_path / "map_projector_info.yaml"

    assert metadata.exists()
    assert lanelet.exists()
    assert projector.exists()

    metadata_text = metadata.read_text()
    assert "x_resolution: 50.0" in metadata_text
    assert "pointcloud_map.pcd: [0, 0]" in metadata_text

    projector_text = projector.read_text()
    assert "projector_type: Local" in projector_text
    assert "vertical_datum: WGS84" in projector_text

    lanelet_text = lanelet.read_text()
    assert '<osm version="0.6"' in lanelet_text
    assert 'k="local_x"' in lanelet_text
    assert 'k="type" v="lanelet"' in lanelet_text


def test_does_not_overwrite_existing_files(tmp_path: Path) -> None:
    custom_lanelet = "<!-- user-edited -->\n"
    (tmp_path / "lanelet2_map.osm").write_text(custom_lanelet)
    (tmp_path / "pointcloud_map").mkdir()
    custom_metadata = "x_resolution: 99.0\n"
    (tmp_path / "pointcloud_map" / "metadata.yaml").write_text(custom_metadata)

    write_sidecars_if_absent(tmp_path)

    assert (tmp_path / "lanelet2_map.osm").read_text() == custom_lanelet
    assert (tmp_path / "pointcloud_map" / "metadata.yaml").read_text() == custom_metadata
    assert (tmp_path / "map_projector_info.yaml").exists()
