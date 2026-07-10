"""Verify the Ouster OS-1 mount in kachaka_with_shelf.urdf.xacro.

Expands the xacro and checks (a) the sensor hangs off the bottom shelf board
and (b) the resolved base_link -> os_lidar height matches the bottom-board
mount. Skipped when the `xacro` CLI is unavailable (ROS not sourced), e.g. a
bare `pytest` run; it executes under `colcon test`.
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


def _joint_child_map():
    xacro = shutil.which("xacro")
    if xacro is None:
        pytest.skip("xacro CLI not available (source ROS 2 to run this test)")
    result = subprocess.run(
        [xacro, XACRO_PATH], capture_output=True, text=True, check=True
    )
    root = ET.fromstring(result.stdout)
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
    parent, _ = _joint_child_map()["os_sensor"]
    assert parent == "shelf_bottom_board", (
        f"OS-1 mount parent is {parent!r}, expected 'shelf_bottom_board'"
    )


def test_os_lidar_height_matches_bottom_mount():
    joints = _joint_child_map()
    frame, z, chain = "os_lidar", 0.0, []
    while frame != "base_link":
        assert frame in joints, f"frame {frame!r} not connected to base_link"
        parent, xyz = joints[frame]
        # NOTE: sums local z only — valid because every joint on the
        # base_link->os_lidar chain is rpy="0 0 0" (rotation-free).
        z += xyz[2]
        chain.append(frame)
        frame = parent
    assert abs(z - EXPECTED_OS_LIDAR_Z) < 1e-3, (
        f"base_link->os_lidar z={z:.5f}, expected {EXPECTED_OS_LIDAR_Z:.5f}; "
        f"chain={chain}"
    )
