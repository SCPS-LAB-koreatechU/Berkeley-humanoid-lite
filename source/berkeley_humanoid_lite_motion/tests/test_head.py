"""The ZED 2i head: a pan joint on the torso and the camera frames the ZED
wrapper expects, looking forward at zero.

The frames are copied from Stereolabs' zed_macro.urdf.xacro, so a sign slip
here would put the left image on the right camera without any error at
runtime -- only a mirrored point cloud.
"""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conftest import REPO

URDF_DIR = REPO / "ros2_ws/src/berkeley_humanoid_lite_description/urdf"
MODELS = [
    "berkeley_humanoid_lite.urdf",
    "berkeley_humanoid_lite_dexhand.urdf",
    "berkeley_humanoid_lite_v1arm.urdf",
    "berkeley_humanoid_lite_tuning.urdf",
]
FRAMES = [
    "head_mount", "head_pan", "zed_camera_link", "zed_camera_center",
    "zed_left_camera_frame", "zed_left_camera_frame_optical",
    "zed_right_camera_frame", "zed_right_camera_frame_optical", "zed_imu_link",
]


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pose(root, link):
    """World pose of `link` with every joint at zero."""
    parents = {j.find("child").get("link"): j for j in root.findall("joint")}
    T = np.eye(4)
    while link in parents:
        joint = parents[link]
        origin = joint.find("origin")
        local = np.eye(4)
        local[:3, :3] = rpy(*map(float, origin.get("rpy", "0 0 0").split()))
        local[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
        T = local @ T
        link = joint.find("parent").get("link")
    return T


@pytest.mark.parametrize("model", MODELS)
def test_every_model_carries_the_head(model):
    root = ET.parse(URDF_DIR / model).getroot()
    links = {l.get("name") for l in root.findall("link")}
    assert set(FRAMES) <= links
    yaw = root.find("joint[@name='head_yaw']")
    assert yaw.get("type") == "revolute"
    assert yaw.find("parent").get("link") == "head_mount"
    assert np.allclose(list(map(float, yaw.find("axis").get("xyz").split())), [0, 0, 1])


def test_the_camera_looks_forward_with_left_on_the_left():
    root = ET.parse(URDF_DIR / "berkeley_humanoid_lite_v1arm.urdf").getroot()
    left = pose(root, "zed_left_camera_frame")
    right = pose(root, "zed_right_camera_frame")
    optical = pose(root, "zed_left_camera_frame_optical")

    assert np.isclose(left[1, 3] - right[1, 3], 0.12)          # baseline, left is +y
    assert np.allclose(optical[:3, 2], [1, 0, 0], atol=1e-9)   # optical z = forward
    assert np.allclose(optical[:3, 0], [0, -1, 0], atol=1e-9)  # optical x = image right
    # above the torso's top rail (z 0.830 m), facing the same way as the base
    assert pose(root, "zed_camera_link")[2, 3] > 0.830
    assert np.allclose(pose(root, "head_pan")[:3, :3], np.eye(3), atol=1e-9)
