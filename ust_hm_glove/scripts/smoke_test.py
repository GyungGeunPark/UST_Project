"""Lightweight smoke test for the Fourier GR1T2 port.

Runs without Isaac Sim and without any VR hardware.  Exercises:

1. ``fourier_hand_mapper``  — VMC + skeletal produce finite 11-vectors;
   ``pack_22d`` respects the canonical joint order.
2. ``waist_estimator``      — returns zero at the neutral frame and a
   gain-scaled delta when the hips rotate.
3. ``gr1t2_retargeter``     — 36D tensor shape, idle-pose fallback,
   forearm-priority source tracking, right-wrist Z180 correction.
4. ``waist_origin_shift``   — EEF targets are invariant to user
   translation when ``use_waist_origin`` is on.
5. ``gr1t2_udcap_device``   — module imports cleanly without Isaac Sim;
   ``ACTION_DIM == 36`` and all config paths are resolved relative to
   the ust_hm_glove package.
6. ``gym_env_registration``  — the env classes load (import only; actual
   environment construction needs Isaac Sim).

Exits non-zero on failure.  Invoke with
``python -m ust_ws.ust_hm_glove.scripts.smoke_test``.
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


# ── 1. fourier_hand_mapper ─────────────────────────────────────────────
def test_fourier_hand_mapper() -> None:
    from ust_ws.ust_hm_glove.teleop.fourier_hand_mapper import (
        FOURIER_JOINT_DIM,
        FOURIER_TOTAL_HAND_JOINTS,
        FourierHandMapper,
        IDX_INDEX_PROX,
        IDX_THUMB_YAW,
        pack_22d,
    )

    mapper = FourierHandMapper()

    # T-pose: skeletal bones all identity, curls = zero → outputs ~ 0
    bones = np.zeros((31, 7), dtype=np.float32)
    bones[:, 3] = 1.0  # w = 1 (identity)
    out = mapper.map_hand_skeletal(bones, is_right=False, finger_curls=[0.0] * 5)
    assert out.shape == (FOURIER_JOINT_DIM,)
    assert np.isfinite(out).all()
    # All finger drivers should be zero or very small at rest.
    assert float(out[IDX_INDEX_PROX]) < 0.05, out[IDX_INDEX_PROX]

    # Fist: curls = 1 → proximals ~ max bend.
    fist = mapper.map_hand_skeletal(bones, is_right=True, finger_curls=[1.0] * 5)
    assert fist.shape == (FOURIER_JOINT_DIM,)
    assert float(fist[IDX_INDEX_PROX]) > 0.4, fist[IDX_INDEX_PROX]

    # VMC branch
    vmc_bones = {
        "RightIndexProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        "RightMiddleProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        "RightThumbProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        "RightThumbMetacarpal": (0.0, 0.0, 0.3, math.cos(math.pi / 8)),
    }
    vmc_out = mapper.map_hand_vmc(vmc_bones, is_right=True)
    assert vmc_out.shape == (FOURIER_JOINT_DIM,)
    assert np.isfinite(vmc_out).all()

    # pack_22d preserves shape + canonical order; 9.9 fix applies a
    # per-slot sign so proximal joints (slots 0..3, 5..8) and thumb_yaw
    # (slots 4, 9) are NEGATED to match the GR1T2 USD's negative
    # finger-flexion direction.
    packed = pack_22d(fist, vmc_out)
    assert packed.shape == (FOURIER_TOTAL_HAND_JOINTS,)
    # Slot 0 is L_index_proximal (from fist) -- negated.
    assert abs(float(packed[0]) - (-float(fist[0]))) < 1e-6
    # Slot 5 is R_index_proximal (from vmc_out) -- negated.
    assert abs(float(packed[5]) - (-float(vmc_out[0]))) < 1e-6
    # Slots 20/21 are thumb distal L/R (mimic of thumb pitch, kept positive).
    assert np.isfinite(packed[20]) and np.isfinite(packed[21])


# ── 2. waist_estimator ─────────────────────────────────────────────────
def test_waist_estimator() -> None:
    from ust_ws.ust_hm_glove.teleop.waist_estimator import WaistEstimator

    # zero_cal_frames=1 selects the legacy single-frame zero capture.
    est = WaistEstimator(source="hips_tracker", low_pass_alpha=1.0, zero_cal_frames=1)

    # First call sets the neutral pose → returns zero.
    snap = {
        "trackers": {
            "waist": {
                "pos": np.array([0.0, 0.0, 1.0]),
                "quat": np.array([1.0, 0.0, 0.0, 0.0]),
            }
        }
    }
    out0 = est.estimate(snap)
    assert abs(out0.yaw) < 1e-6
    assert abs(out0.pitch) < 1e-6
    assert abs(out0.roll) < 1e-6

    # 30° yaw rotation → ~0.5 rad (0.524) after gain=1.0.
    half = math.radians(15.0)  # half-angle for the quat
    snap2 = {
        "trackers": {
            "waist": {
                "pos": np.array([0.0, 0.0, 1.0]),
                "quat": np.array([math.cos(half), 0.0, 0.0, math.sin(half)]),
            }
        }
    }
    out1 = est.estimate(snap2)
    assert abs(out1.yaw - math.radians(30.0)) < 1e-3, out1.yaw
    assert abs(out1.pitch) < 1e-3
    assert abs(out1.roll) < 1e-3

    # Reset clears neutral pose; next estimate resumes from zero.
    est.reset()
    out2 = est.estimate(snap)
    assert abs(out2.yaw) < 1e-6


# ── 3. gr1t2_retargeter ────────────────────────────────────────────────
def test_gr1t2_retargeter() -> None:
    from ust_ws.ust_hm_glove.teleop.gr1t2_retargeter import (
        ACTION_DIM,
        DEFAULT_LEFT_POS,
        DEFAULT_RIGHT_POS,
        GR1T2FourierSteamVRRetargeter,
    )

    r = GR1T2FourierSteamVRRetargeter(debug=False)
    empty_snap = {
        "timestamp": 0.0,
        "hmd": None,
        "trackers": {},
        "hands": {"left": None, "right": None},
        "controllers": {"left": None, "right": None},
        "frame_count": 1,
    }
    action = r.retarget(empty_snap)
    assert action.shape == (ACTION_DIM,)
    assert abs(float(action[0]) - DEFAULT_LEFT_POS[0]) < 1e-4
    assert abs(float(action[7]) - DEFAULT_RIGHT_POS[0]) < 1e-4
    # Hand joints should be zero at idle.
    assert float(action[14:36].abs().max().item()) < 1e-6

    # Forearm tracker → source becomes "forearm"
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


def test_right_wrist_z180() -> None:
    """The right-wrist Z180 correction should flip the quaternion's w,z sign."""
    from ust_ws.ust_hm_glove.teleop.gr1t2_retargeter import (
        GR1T2FourierSteamVRRetargeter,
    )

    # With correction on (default)
    r_on = GR1T2FourierSteamVRRetargeter(right_wrist_z180=True, debug=False)
    # Without correction
    r_off = GR1T2FourierSteamVRRetargeter(right_wrist_z180=False, debug=False)

    snap = {
        "timestamp": 0.0,
        "hmd": None,
        "trackers": {
            "right_forearm": {
                "pos": np.array([0.0, 1.2, -0.4]),
                "quat": np.array([1.0, 0.0, 0.0, 0.0]),  # identity
            },
        },
        "hands": {"left": None, "right": None},
        "controllers": {"left": None, "right": None},
        "frame_count": 1,
    }
    a_on = r_on.retarget(snap)
    a_off = r_off.retarget(snap)
    # Right wrist quat differs between the two modes.
    diff = (a_on[10:14] - a_off[10:14]).abs().max().item()
    assert diff > 0.1, f"z180 correction had no effect: diff={diff}"
    # Both should be unit-norm.
    for tag, act in (("on", a_on), ("off", a_off)):
        q = act[10:14].numpy()
        assert abs(float(np.linalg.norm(q)) - 1.0) < 1e-5, f"non-unit quat ({tag}): {q}"


# ── 4. waist_origin_shift ──────────────────────────────────────────────
def test_waist_origin_shift() -> None:
    """Pelvis-frame targets should be invariant to user translation."""
    from ust_ws.ust_hm_glove.teleop.gr1t2_retargeter import (
        GR1T2FourierSteamVRRetargeter,
    )

    base_snap = {
        "timestamp": 0.0,
        "hmd": None,
        "hands": {"left": None, "right": None},
        "controllers": {"left": None, "right": None},
        "frame_count": 1,
    }
    r_a = GR1T2FourierSteamVRRetargeter(use_waist_origin=True, debug=False)
    snap_a = {
        **base_snap,
        "trackers": {
            "waist": {"pos": np.array([0.0, 1.0, 0.0]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "left_forearm": {"pos": np.array([0.5, 1.2, 0.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "right_forearm": {"pos": np.array([-0.5, 1.2, 0.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        },
    }
    act_a = r_a.retarget(snap_a)

    r_b = GR1T2FourierSteamVRRetargeter(use_waist_origin=True, debug=False)
    snap_b = {
        **base_snap,
        "trackers": {
            "waist": {"pos": np.array([2.0, 1.0, 3.0]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "left_forearm": {"pos": np.array([2.5, 1.2, 3.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "right_forearm": {"pos": np.array([1.5, 1.2, 3.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        },
    }
    act_b = r_b.retarget(snap_b)
    diff = (act_a[0:14] - act_b[0:14]).abs().max().item()
    assert diff < 1e-4, f"pelvis-frame target shifted (diff={diff:.4f})"

    # With use_waist_origin=False the world coords leak.
    r_raw = GR1T2FourierSteamVRRetargeter(use_waist_origin=False, debug=False)
    act_a_raw = r_raw.retarget(snap_a)
    act_b_raw = r_raw.retarget(snap_b)
    diff_raw = (act_a_raw[0:14] - act_b_raw[0:14]).abs().max().item()
    assert diff_raw > 0.5, f"use_waist_origin=False should leak world coords, got {diff_raw}"


# ── 5. device import ──────────────────────────────────────────────────
def test_device_import() -> None:
    from ust_ws.ust_hm_glove.teleop.gr1t2_udcap_device import (
        GR1T2FourierUDCAPDevice,
        GR1T2FourierUDCAPDeviceCfg,
    )

    cfg = GR1T2FourierUDCAPDeviceCfg()
    assert cfg.tracker_binding_json.endswith("tracker_binding.json")
    assert cfg.actions_json.endswith("actions.json")
    assert GR1T2FourierUDCAPDevice.ACTION_DIM == 36


# ── 6. gym env_cfg import (without Isaac Sim) ─────────────────────────
def test_env_cfg_import() -> None:
    """Importing the env_cfg module requires Isaac Lab; skip gracefully if missing."""
    try:
        from ust_ws.ust_hm_glove import kitchen_sorting_gr1t2_env_cfg as cfg  # type: ignore
    except Exception as exc:
        print(f"[smoke] SKIP  env_cfg_import (Isaac Lab not available: {exc!r})")
        return
    assert cfg.FOURIER_HAND_JOINT_NAMES[0] == "L_index_proximal_joint"
    assert len(cfg.FOURIER_HAND_JOINT_NAMES) == 22
    assert cfg.GR1T2_PALM_FRAME_NAMES[0].endswith("left_hand_pitch_link")


def main() -> int:
    failed = 0
    tests = [
        ("fourier_hand_mapper", test_fourier_hand_mapper),
        ("waist_estimator", test_waist_estimator),
        ("gr1t2_retargeter", test_gr1t2_retargeter),
        ("right_wrist_z180", test_right_wrist_z180),
        ("waist_origin_shift", test_waist_origin_shift),
        ("device_import", test_device_import),
        ("env_cfg_import", test_env_cfg_import),
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
