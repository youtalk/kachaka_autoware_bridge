# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import struct
from pathlib import Path

from kachaka_autoware_maps.pcd_io import write_pcd_binary


def _read_pcd(path: Path) -> tuple[list[str], list[tuple[float, float, float, float]]]:
    raw = path.read_bytes()
    header_end = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    header_lines = raw[:header_end].decode("ascii").splitlines()
    body = raw[header_end:]
    points = []
    for offset in range(0, len(body), 16):
        x, y, z, i = struct.unpack("<ffff", body[offset:offset + 16])
        points.append((x, y, z, i))
    return header_lines, points


def test_writes_header_and_points(tmp_path: Path) -> None:
    out = tmp_path / "pointcloud_map.pcd"
    points = [(0.0, 0.0, 0.0, 10.0), (1.5, -2.5, 0.25, 200.0), (-3.0, 4.0, -1.0, 0.0)]

    write_pcd_binary(out, points)

    header, read_back = _read_pcd(out)
    assert "VERSION 0.7" in header
    assert "FIELDS x y z intensity" in header
    assert "SIZE 4 4 4 4" in header
    assert "TYPE F F F F" in header
    assert "COUNT 1 1 1 1" in header
    assert "WIDTH 3" in header
    assert "HEIGHT 1" in header
    assert "POINTS 3" in header
    assert "DATA binary" in header
    assert read_back == points


def test_empty_input_writes_zero_point_pcd(tmp_path: Path) -> None:
    out = tmp_path / "empty.pcd"
    write_pcd_binary(out, [])
    header, read_back = _read_pcd(out)
    assert "WIDTH 0" in header
    assert "POINTS 0" in header
    assert read_back == []
