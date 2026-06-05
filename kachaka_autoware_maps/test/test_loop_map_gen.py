# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

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
