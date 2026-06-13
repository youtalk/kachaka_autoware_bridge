# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

"""Pure TF edge filtering for the kachaka bridge TF gate (no ROS imports).

The vendored bridge launch remaps the DynamicTfComponent's /tf output to
/kachaka/tf_raw; scripts/tf_gate forwards it back to /tf minus the edges named
in drop_edges ("parent:child" comma list). In ndt localization mode the
dropped edge is map:odom (the map_to_odom_adapter owns it instead); in
kachaka/shadow modes the drop set is empty and the gate is a pass-through, so
all modes share one launch path.
"""

from __future__ import annotations

from typing import Iterable

Edge = tuple[str, str]


def parse_edges(spec: str) -> frozenset[Edge]:
    edges: set[Edge] = set()
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        parts = item.split(":")
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise ValueError(f"bad edge spec {item!r}; expected 'parent:child'")
        edges.add((parts[0].strip().lstrip("/"), parts[1].strip().lstrip("/")))
    return frozenset(edges)


def partition_transforms(transforms: Iterable, drop: frozenset[Edge]):
    """Split TransformStamped-likes into (kept, dropped) by (parent, child)
    edge, tolerating leading slashes in frame ids."""
    kept, dropped = [], []
    for t in transforms:
        edge = (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"))
        (dropped if edge in drop else kept).append(t)
    return kept, dropped
