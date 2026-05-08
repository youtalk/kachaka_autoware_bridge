# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

from pathlib import Path

_METADATA_YAML = """\
x_resolution: 50.0
y_resolution: 50.0
pointcloud_map.pcd: [0, 0]
"""

_PROJECTOR_INFO_YAML = """\
projector_type: Local
vertical_datum: WGS84
"""

_MIN_LANELET2_OSM = """\
<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="kachaka_autoware_maps">
  <node id="1" lat="0.0" lon="0.0"><tag k="local_x" v="0.0"/><tag k="local_y" v="-0.3"/><tag k="ele" v="0.0"/></node>
  <node id="2" lat="0.0" lon="0.0"><tag k="local_x" v="2.0"/><tag k="local_y" v="-0.3"/><tag k="ele" v="0.0"/></node>
  <node id="3" lat="0.0" lon="0.0"><tag k="local_x" v="0.0"/><tag k="local_y" v="0.3"/><tag k="ele" v="0.0"/></node>
  <node id="4" lat="0.0" lon="0.0"><tag k="local_x" v="2.0"/><tag k="local_y" v="0.3"/><tag k="ele" v="0.0"/></node>
  <way id="10"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>
  <way id="11"><nd ref="3"/><nd ref="4"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>
  <relation id="100">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="speed_limit" v="0.3"/>
    <tag k="location" v="urban"/>
    <tag k="one_way" v="yes"/>
  </relation>
</osm>
"""


def _write_if_absent(path: Path, content: str) -> None:
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def write_sidecars_if_absent(directory: "Path | str") -> None:
    base = Path(directory)
    _write_if_absent(base / "pointcloud_map" / "metadata.yaml", _METADATA_YAML)
    _write_if_absent(base / "lanelet2_map.osm", _MIN_LANELET2_OSM)
    _write_if_absent(base / "map_projector_info.yaml", _PROJECTOR_INFO_YAML)
