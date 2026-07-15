"""Verify the Ouster OS-1 mount in kachaka_with_shelf.urdf.xacro.

Expands the xacro and checks (a) the sensor hangs off the bottom shelf board,
(b) the resolved base_link -> os_lidar height matches the bottom-board mount,
and (c) with_shelf:=false drops the shelf + OS-1 (bare Kachaka body). Skipped
when the `xacro` CLI is unavailable (ROS not sourced), e.g. a bare `pytest`
run; it executes under `colcon test`.
"""
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(__file__)
XACRO_PATH = os.path.normpath(
    os.path.join(HERE, "..", "urdf", "kachaka_with_shelf.urdf.xacro")
)

# base_link -> os_lidar at rest (docking lift = 0):
#   shelf mount 0.115 + board thickness 0.015 + lidar_to_sensor_z 0.03618
EXPECTED_OS_LIDAR_Z = 0.115 + 0.015 + 0.03618  # = 0.16618


def _expand(with_shelf=None):
    xacro = shutil.which("xacro")
    if xacro is None:
        pytest.skip("xacro CLI not available (source ROS 2 to run this test)")
    cmd = [xacro, XACRO_PATH]
    if with_shelf is not None:
        cmd.append(f"with_shelf:={with_shelf}")
    result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return ET.fromstring(result.stdout)


def _joint_child_map(root):
    """child_link -> (parent_link, [x, y, z]) for every joint."""
    joints = {}
    for joint in root.findall("joint"):
        child = joint.find("child").get("link")
        parent = joint.find("parent").get("link")
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        if origin is not None and origin.get("xyz"):
            xyz = [float(v) for v in origin.get("xyz").split()]
        joints[child] = (parent, xyz)
    return joints


def test_ouster_mounted_on_bottom_board():
    parent, _ = _joint_child_map(_expand())["os_sensor"]
    assert parent == "shelf_bottom_board", (
        f"OS-1 mount parent is {parent!r}, expected 'shelf_bottom_board'"
    )


def test_os_lidar_height_matches_bottom_mount():
    joints = _joint_child_map(_expand())
    frame, z, chain = "os_lidar", 0.0, []
    while frame != "base_link":
        assert frame in joints, f"frame {frame!r} not connected to base_link"
        parent, xyz = joints[frame]
        # NOTE: sums local z only. Valid because every origin on the
        # base_link->os_lidar chain is pure-Z ("0 0 z") and the only rotation
        # (the os_sensor->os_lidar 180 deg ICD yaw) is about Z, which preserves
        # each z term.
        z += xyz[2]
        chain.append(frame)
        frame = parent
    assert abs(z - EXPECTED_OS_LIDAR_Z) < 1e-3, (
        f"base_link->os_lidar z={z:.5f}, expected {EXPECTED_OS_LIDAR_Z:.5f}; "
        f"chain={chain}"
    )


def test_with_shelf_false_is_bare_body():
    links = {link.get("name") for link in _expand(with_shelf="false").findall("link")}
    assert "base_link" in links, "bare body must still have base_link"
    assert not (links & {"os_sensor", "os_lidar", "os_imu"}), (
        f"with_shelf:=false must drop the OS-1; found {links & {'os_sensor', 'os_lidar', 'os_imu'}}"
    )
    assert not any(name.startswith("shelf_") for name in links), (
        f"with_shelf:=false must drop the shelf; found {[n for n in links if n.startswith('shelf_')]}"
    )
