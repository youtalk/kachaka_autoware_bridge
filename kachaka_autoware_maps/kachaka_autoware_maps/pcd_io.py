# Copyright 2026 Yutaka Kondo
# Licensed under the Apache License, Version 2.0 (the "License").

import struct
from pathlib import Path
from typing import Iterable, Sequence

PointXYZI = Sequence[float]


def write_pcd_binary(path: Path | str, points: Iterable[PointXYZI]) -> None:
    pts = [tuple(p) for p in points]
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("wb") as f:
        f.write(header.encode("ascii"))
        for x, y, z, intensity in pts:
            f.write(struct.pack("<ffff", float(x), float(y), float(z), float(intensity)))
