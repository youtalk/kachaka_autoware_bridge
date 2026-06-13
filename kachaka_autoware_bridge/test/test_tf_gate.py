# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import pytest
from types import SimpleNamespace

from kachaka_autoware_bridge.tf_gate import parse_edges, partition_transforms


def _tf(parent: str, child: str):
    return SimpleNamespace(header=SimpleNamespace(frame_id=parent), child_frame_id=child)


def test_parse_edges_empty_and_whitespace() -> None:
    assert parse_edges("") == frozenset()
    assert parse_edges("  ") == frozenset()


def test_parse_edges_single_and_multiple() -> None:
    assert parse_edges("map:odom") == frozenset({("map", "odom")})
    assert parse_edges("map:odom, a:b") == frozenset({("map", "odom"), ("a", "b")})


def test_parse_edges_rejects_malformed() -> None:
    with pytest.raises(ValueError):
        parse_edges("map-odom")
    with pytest.raises(ValueError):
        parse_edges("map:odom:extra")


def test_partition_passthrough_when_no_drops() -> None:
    tfs = [_tf("map", "odom"), _tf("odom", "base_footprint")]
    kept, dropped = partition_transforms(tfs, frozenset())
    assert kept == tfs
    assert dropped == []


def test_partition_drops_only_the_named_edge() -> None:
    tfs = [_tf("map", "odom"), _tf("odom", "base_footprint")]
    kept, dropped = partition_transforms(tfs, frozenset({("map", "odom")}))
    assert [t.child_frame_id for t in kept] == ["base_footprint"]
    assert [t.child_frame_id for t in dropped] == ["odom"]


def test_partition_ignores_leading_slashes() -> None:
    tfs = [_tf("/map", "/odom")]
    kept, dropped = partition_transforms(tfs, frozenset({("map", "odom")}))
    assert kept == []
    assert len(dropped) == 1


def test_parse_edges_normalizes_leading_slashes() -> None:
    # A slashed spec must match the same edge as the unslashed form, since
    # partition_transforms strips leading slashes from runtime frame ids.
    assert parse_edges("/map:/odom") == frozenset({("map", "odom")})
    assert parse_edges("/map:/odom") == parse_edges("map:odom")
