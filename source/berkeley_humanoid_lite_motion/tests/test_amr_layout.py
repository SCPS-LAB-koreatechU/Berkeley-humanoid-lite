"""The AMR lower body (build_amr_stl.py).

The point of the layout is that the AMR plugs into the same place the legs do,
and that the hands end up at worktable height; both are pinned here, together
with the clearances the STL is meant to show.
"""

import importlib.util
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conftest import REPO

SCRIPTS = REPO / "ros2_ws/src/berkeley_humanoid_lite_description/scripts"
V1ARM = REPO / "ros2_ws/src/berkeley_humanoid_lite_description/urdf/berkeley_humanoid_lite_v1arm.urdf"
UPSTREAM = (REPO / "source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/"
            "berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf")


@pytest.fixture(scope="module")
def B():
    sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location("build_amr_stl", SCRIPTS / "build_amr_stl.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def params(B):
    return {k: v for k, (v, _) in B.PARAMS.items()}


@pytest.fixture(scope="module")
def layout(B, params):
    return B.build(params, str(V1ARM))


def signed_volume(tris):
    return np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0


def test_hands_sit_just_above_the_worktable(layout, params):
    amr, adapter, robot, d = layout
    assert d["hand_lowest_z"] == pytest.approx(params["table_z"] + params["hand_clear"])
    hand_low = min(t[:, :, 2].min() for n, t in robot if not n.startswith(("base/", "head", "zed")))
    assert hand_low == pytest.approx(d["hand_lowest_z"])
    # the hands are the lowest thing on the robot in the working pose, and stay above the deck
    assert d["hand_lowest_z"] < d["torso_bottom_z"]
    assert d["hand_above_deck"] > 100.0


def test_adapter_bolts_to_the_same_flanges_as_the_legs(B, params, layout):
    """The wedge faces lie on the hip-roll output flanges taken from the
    upstream legs' URDF, so legs and AMR swap at one interface."""
    if not UPSTREAM.is_file():
        pytest.skip("assets submodule not checked out")
    sys.path.insert(0, str(SCRIPTS))
    import merge_urdf_stl as m
    T, _ = m.link_transforms(ET.parse(UPSTREAM).getroot(), {})
    _, adapter, _, d = layout
    off = np.array([params["mount_x"], 0.0, d["base_origin_z"]])
    for side, sy in (("left", 1), ("right", -1)):
        J = T[f"leg_{side}_hip_roll"][:3, 3] * 1000.0
        axis = T[f"leg_{side}_hip_roll"][:3, 2]
        assert np.allclose(J, B.HIP_J * [1, sy, 1], atol=1e-3)
        assert np.allclose(axis, B.HIP_AX, atol=1e-4)
        wedge = dict(adapter)[f"adapter_hip_wedge_{side}"].reshape(-1, 3) - off
        along = (wedge - J) @ axis
        face = np.isclose(along, -B.HIP_LIP, atol=1e-3)   # constants are rounded to 1 um
        assert face.sum() >= 4                       # the flange pad is one face of the wedge
        assert along.max() == pytest.approx(-B.HIP_LIP, abs=1e-3)   # nothing pokes into the actuator


def test_generated_solids_are_closed_and_outward(layout):
    amr, adapter, _, _ = layout
    for name, tris in amr + adapter:
        assert signed_volume(tris) > 0, name


def test_vehicle_matches_the_public_numbers(layout, params):
    amr, _, _, _ = layout
    wheels = {n: t.reshape(-1, 3) for n, t in amr if n.startswith("amr_wheel")}
    centres = {n: (v.min(0) + v.max(0)) / 2 for n, v in wheels.items()}
    assert centres["amr_wheel_front_left"][0] - centres["amr_wheel_rear_left"][0] == pytest.approx(params["wheelbase"])
    for n, v in wheels.items():
        assert v[:, 2].min() == pytest.approx(0.0, abs=1e-6), n          # on the ground
        assert v[:, 2].max() == pytest.approx(2 * params["wheel_r"], abs=1e-6), n


def test_adapter_stack_closes_from_deck_to_torso(layout, params):
    _, adapter, _, d = layout
    parts = dict(adapter)
    post = parts["adapter_post_front_left"].reshape(-1, 3)
    deck_plate = parts["adapter_deck_plate"].reshape(-1, 3)
    plate = parts["adapter_plate"].reshape(-1, 3)
    assert deck_plate[:, 2].min() == pytest.approx(d["deck_top_z"])
    assert post[:, 2].min() == pytest.approx(deck_plate[:, 2].max())
    assert post[:, 2].max() == pytest.approx(plate[:, 2].min())
    assert plate[:, 2].max() == pytest.approx(d["adapter_plate_top_z"])
    assert 5.0 < d["torso_bottom_z"] - d["adapter_plate_top_z"] < 30.0      # clear of the hip housings


def test_nothing_on_the_robot_runs_into_the_amr(B, layout):
    pytest.importorskip("scipy")
    amr, adapter, robot, _ = layout
    assert B.interference(amr + adapter, robot) == {}
