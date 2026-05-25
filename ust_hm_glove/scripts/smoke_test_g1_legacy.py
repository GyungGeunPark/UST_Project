"""Lightweight smoke test for the Windows/SteamVR port.

Runs without Isaac Sim and without any VR hardware.  Exercises:

1. ``coord_transforms`` — identity round-trip and known-axis checks.
2. ``fingertip_extractor`` — index selection from a synthetic 31-bone array.
3. ``udcap_finger_mapper`` — VMC + skeletal branches produce finite 12D output.
4. ``g1_retargeter`` — 38D tensor shape, idle-pose fallback, finger pipeline.
5. ``pico_udcap_device`` — module imports cleanly.

Exits non-zero on failure.  Invoke with ``python -m ust_ws.ust_hm_glove.scripts.smoke_test``.
"""

from __future__ import annotations

import math
import sys
import traceback

import numpy as np


def _ok(msg: str) -> None:
    print(f"[smoke] PASS  {msg}")


def _fail(msg: str, exc: BaseException) -> None:
    print(f"[smoke] FAIL  {msg}")
    traceback.print_exception(type(exc), exc, exc.__traceback__)


def test_coord_transforms() -> None:
    from ust_ws.ust_hm_glove.teleop import coord_transforms as ct

    # Identity quaternion → basis change only
    p_il, q_il = ct.svr_to_isaaclab((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0))
    assert np.allclose(p_il, np.array([-3.0, -1.0, 2.0])), f"pos {p_il}"

    # quat round-trip through matrix
    q = ct.normalise_quat(np.array([0.7071068, 0.0, 0.7071068, 0.0]))
    q_rt = ct.matrix_to_quat_wxyz(ct.quat_wxyz_to_matrix(q))
    # quaternion sign is ambiguous; compare absolute dot
    assert abs(abs(float(np.dot(q, q_rt))) - 1.0) < 1e-6

    # forearm_to_wrist: +X offset in tracker frame, no rotation
    wrist = ct.forearm_to_wrist(
        {"pos": np.array([0.0, 0.0, 0.0]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        forearm_to_wrist_offset=(0.12, 0.0, 0.0),
    )
    assert np.allclose(wrist["pos"], np.array([0.12, 0.0, 0.0])), wrist["pos"]


def test_fingertip_extractor() -> None:
    from ust_ws.ust_hm_glove.teleop.fingertip_extractor import (
        STEAMVR_TIP_BONE_INDICES,
        extract_fingertips_wrist_frame,
    )

    bones = np.zeros((31, 7), dtype=np.float32)
    bones[:, 3] = 1.0  # identity quat
    for rank, idx in enumerate(STEAMVR_TIP_BONE_INDICES):
        bones[idx, :3] = (0.01 * (rank + 1), 0.02 * (rank + 1), 0.03 * (rank + 1))

    tips = extract_fingertips_wrist_frame(bones)
    assert tips.shape == (5, 3)
    assert np.allclose(tips[0], (0.01, 0.02, 0.03))
    assert np.allclose(tips[4], (0.05, 0.10, 0.15))


def test_udcap_finger_mapper() -> None:
    from ust_ws.ust_hm_glove.teleop.udcap_finger_mapper import (
        UDCAPFingerMapper,
        INSPIRE_JOINT_DIM,
        pack_24d,
    )

    mapper = UDCAPFingerMapper()
    bones = {
        "LeftIndexProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        "LeftMiddleProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        "LeftThumbProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
    }
    out = mapper.map_hand(bones, is_right=False)
    assert out.shape == (INSPIRE_JOINT_DIM,)
    assert np.isfinite(out).all()

    skeletal = np.zeros((31, 7), dtype=np.float32)
    skeletal[:, 3] = 1.0
    out_s = mapper.map_from_skeletal(skeletal, is_right=True, finger_curls=[0.1, 0.2, 0.3, 0.4, 0.5])
    assert out_s.shape == (INSPIRE_JOINT_DIM,)

    packed = pack_24d(out, out_s)
    assert packed.shape == (24,)


def test_g1_retargeter() -> None:
    from ust_ws.ust_hm_glove.teleop.g1_retargeter import G1SteamVRRetargeter

    r = G1SteamVRRetargeter(debug=False)
    empty_snap = {"timestamp": 0.0, "hmd": None, "trackers": {}, "hands": {"left": None, "right": None}, "controllers": {"left": None, "right": None}, "frame_count": 1}
    action = r.retarget(empty_snap)
    assert action.shape == (38,)
    # idle pose values match the DEFAULT_LEFT_POS / DEFAULT_RIGHT_POS constants
    assert abs(float(action[0]) - (-0.1487)) < 1e-4
    assert abs(float(action[7]) - (0.1487)) < 1e-4

    # With a forearm tracker injected, source becomes "forearm"
    snap = {
        **empty_snap,
        "trackers": {
            "left_forearm": {"pos": np.array([0.0, 1.2, 0.4]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "right_forearm": {"pos": np.array([0.0, 1.2, -0.4]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        },
    }
    r.retarget(snap)
    info = r.get_source_info()
    assert info["left_eef"] == "forearm"
    assert info["right_eef"] == "forearm"


def test_waist_origin_shift() -> None:
    """Target positions should shift when the waist tracker moves
    (verifies use_waist_origin actually uses the waist tracker)."""
    from ust_ws.ust_hm_glove.teleop.g1_retargeter import G1SteamVRRetargeter

    base_snap = {"timestamp": 0.0, "hmd": None, "hands": {"left": None, "right": None}, "controllers": {"left": None, "right": None}, "frame_count": 1}

    # Scenario A: user at SteamVR origin (waist at (0, 0, 1.0))
    r_a = G1SteamVRRetargeter(use_waist_origin=True, debug=False)
    snap_a = {
        **base_snap,
        "trackers": {
            "waist": {"pos": np.array([0.0, 1.0, 0.0]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "left_forearm": {"pos": np.array([0.5, 1.2, 0.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "right_forearm": {"pos": np.array([-0.5, 1.2, 0.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        },
    }
    act_a = r_a.retarget(snap_a)

    # Scenario B: same hand-vs-body pose, but the user has moved to play area (2, 3)
    # (waist X/Y += (2, 3); forearm X/Y += (2, 3))
    r_b = G1SteamVRRetargeter(use_waist_origin=True, debug=False)
    snap_b = {
        **base_snap,
        "trackers": {
            "waist": {"pos": np.array([2.0, 1.0, 3.0]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "left_forearm": {"pos": np.array([2.5, 1.2, 3.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "right_forearm": {"pos": np.array([1.5, 1.2, 3.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        },
    }
    act_b = r_b.retarget(snap_b)

    # The pelvis-frame targets should be (almost) identical — user's body
    # configuration didn't change, only where they stood.
    diff = (act_a[0:14] - act_b[0:14]).abs().max().item()
    assert diff < 1e-4, f"pelvis-frame target shifted with user position (diff={diff:.4f})"

    # Without the flag, raw world coords would leak: verify the opposite.
    r_raw = G1SteamVRRetargeter(use_waist_origin=False, debug=False)
    act_a_raw = r_raw.retarget(snap_a)
    act_b_raw = r_raw.retarget(snap_b)
    diff_raw = (act_a_raw[0:14] - act_b_raw[0:14]).abs().max().item()
    assert diff_raw > 0.5, f"use_waist_origin=False should leak world coords, but diff={diff_raw:.4f}"


def test_device_import() -> None:
    from ust_ws.ust_hm_glove.teleop.pico_udcap_device import (
        PicoUDCAPDevice,
        PicoUDCAPDeviceCfg,
    )

    cfg = PicoUDCAPDeviceCfg()
    assert cfg.tracker_binding_json.endswith("tracker_binding.json")
    assert cfg.actions_json.endswith("actions.json")
    assert PicoUDCAPDevice.ACTION_DIM == 38


def main() -> int:
    failed = 0
    tests = [
        ("coord_transforms", test_coord_transforms),
        ("fingertip_extractor", test_fingertip_extractor),
        ("udcap_finger_mapper", test_udcap_finger_mapper),
        ("g1_retargeter", test_g1_retargeter),
        ("waist_origin_shift", test_waist_origin_shift),
        ("device_import", test_device_import),
    ]
    for name, fn in tests:
        try:
            fn()
            _ok(name)
        except BaseException as exc:  # noqa: BLE001
            _fail(name, exc)
            failed += 1
    print(f"[smoke] {len(tests) - failed}/{len(tests)} passed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
