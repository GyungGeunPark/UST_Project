"""Standalone smoke test for ust_hm_grip.

Runs a minimal sequence of import + retargeter sanity checks WITHOUT
Isaac Sim, so it can be used in CI / pre-commit hooks.  Mirrors
``ust_hm_glove/scripts/smoke_test.py`` but adjusted for the
gripper-only retargeter and 16-D action layout.

Pass criteria (7 tests):
1. ``GR1T2GripperRetargeterCfg`` can be instantiated with defaults.
2. ``GR1T2GripperSteamVRRetargeter`` returns a 16-element float32 tensor.
3. Idle action (no trackers / no controllers) matches the configured idle pose.
4. With controller pose set + trigger above the close threshold, gripper
   command flips to GRIPPER_CLOSE (-1).
5. Hysteresis: once closed, dropping trigger to mid-band (between open
   and close threshold) keeps the gripper closed.
6. ``GR1T2GripperDevice`` can be imported in standalone mode (no Isaac
   Sim) and ``GR1T2GripperDeviceCfg`` accepts the full param set.
7. ``forearm_to_wrist`` + ``svr_to_isaaclab`` chain produces finite,
   non-zero output for a realistic forearm pose.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


_PASS_CHAR = "PASS"
_FAIL_CHAR = "FAIL"


def _print(test_no: int, name: str, ok: bool, detail: str = "") -> None:
    mark = _PASS_CHAR if ok else _FAIL_CHAR
    print(f"  [{mark}] {test_no}. {name}{(' — ' + detail) if detail else ''}")


def main() -> int:
    print("ust_hm_grip smoke test")
    print("=" * 60)
    failures = 0

    # 1. Cfg instantiation
    try:
        from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_retargeter import (
            GR1T2GripperRetargeterCfg,
        )
        _ = GR1T2GripperRetargeterCfg()
        _print(1, "GR1T2GripperRetargeterCfg defaults", True)
    except Exception as exc:  # noqa: BLE001
        failures += 1
        _print(1, "GR1T2GripperRetargeterCfg defaults", False, repr(exc))
        return failures

    # 2. retarget() shape + dtype
    try:
        import torch
        from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_retargeter import (
            ACTION_DIM,
            GR1T2GripperSteamVRRetargeter,
        )
        rt = GR1T2GripperSteamVRRetargeter()
        snap = {"frame_count": 1, "trackers": {}, "controllers": {"left": None, "right": None}}
        out = rt.retarget(snap, action_inputs=None)
        ok = (
            isinstance(out, torch.Tensor)
            and out.dtype == torch.float32
            and out.shape == (ACTION_DIM,)
            and ACTION_DIM == 16
        )
        _print(2, "retarget output 16D float32", ok,
               detail=f"shape={tuple(out.shape)} dtype={out.dtype}")
        if not ok:
            failures += 1
    except Exception as exc:  # noqa: BLE001
        failures += 1
        _print(2, "retarget output 16D float32", False, repr(exc))
        return failures

    # 3. Idle action matches configured idle pose
    try:
        import numpy as np
        rt2 = GR1T2GripperSteamVRRetargeter()
        snap = {"frame_count": 1, "trackers": {}, "controllers": {}}
        out = rt2.retarget(snap, action_inputs=None)
        l_pos = out[0:3].numpy()
        r_pos = out[7:10].numpy()
        l_grip = float(out[14])
        r_grip = float(out[15])
        ok = (
            np.allclose(l_pos, np.array(rt2.cfg.idle_left_pos), atol=1e-4)
            and np.allclose(r_pos, np.array(rt2.cfg.idle_right_pos), atol=1e-4)
            and l_grip > 0
            and r_grip > 0
        )
        _print(3, "Idle action = idle pose + gripper open", ok,
               detail=f"L_pos={l_pos.tolist()} L_grip={l_grip} R_grip={r_grip}")
        if not ok:
            failures += 1
    except Exception as exc:  # noqa: BLE001
        failures += 1
        _print(3, "Idle action = idle pose + gripper open", False, repr(exc))

    # 4. Trigger above close threshold → gripper closes
    try:
        rt3 = GR1T2GripperSteamVRRetargeter()
        # No controller poses, just action_inputs trigger=0.8 left, 0.0 right.
        snap = {"frame_count": 2, "trackers": {}, "controllers": {}}
        ai = {
            "left": {"trigger": 0.8, "grip": 0.0, "menu": False},
            "right": {"trigger": 0.0, "grip": 0.0, "menu": False},
        }
        out = rt3.retarget(snap, action_inputs=ai)
        l_grip = float(out[14])
        r_grip = float(out[15])
        ok = l_grip < 0 and r_grip > 0
        _print(4, "Trigger > 0.6 closes (left only)", ok,
               detail=f"L_grip={l_grip} R_grip={r_grip}")
        if not ok:
            failures += 1
    except Exception as exc:  # noqa: BLE001
        failures += 1
        _print(4, "Trigger > 0.6 closes (left only)", False, repr(exc))

    # 5. Hysteresis: 0.8 → 0.5 stays closed; 0.8 → 0.3 opens
    try:
        rt4 = GR1T2GripperSteamVRRetargeter()
        snap = {"frame_count": 1, "trackers": {}, "controllers": {}}
        # Squeeze
        ai_close = {
            "left": {"trigger": 0.8, "grip": 0.0, "menu": False},
            "right": {"trigger": 0.0, "grip": 0.0, "menu": False},
        }
        rt4.retarget(snap, action_inputs=ai_close)
        # Hold at 0.5 (between thresholds).  Should stay closed.
        ai_mid = {
            "left": {"trigger": 0.5, "grip": 0.0, "menu": False},
            "right": {"trigger": 0.0, "grip": 0.0, "menu": False},
        }
        out_mid = rt4.retarget(snap, action_inputs=ai_mid)
        # Release to 0.3 (below open threshold).  Should reopen.
        ai_open = {
            "left": {"trigger": 0.3, "grip": 0.0, "menu": False},
            "right": {"trigger": 0.0, "grip": 0.0, "menu": False},
        }
        out_open = rt4.retarget(snap, action_inputs=ai_open)
        l_mid = float(out_mid[14])
        l_open = float(out_open[14])
        ok = l_mid < 0 and l_open > 0
        _print(5, "Hysteresis: 0.8→0.5 holds, 0.5→0.3 opens", ok,
               detail=f"mid_L={l_mid} open_L={l_open}")
        if not ok:
            failures += 1
    except Exception as exc:  # noqa: BLE001
        failures += 1
        _print(5, "Hysteresis", False, repr(exc))

    # 6. Device cfg can be instantiated standalone
    try:
        from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_device import (
            GR1T2GripperDeviceCfg,
        )
        cfg = GR1T2GripperDeviceCfg(
            tracker_binding_json="dummy.json",
            actions_json="dummy.json",
            vrmanifest_json="dummy.json",
            forearm_wrist_offset=(0.28, 0.0, 0.0),
        )
        ok = cfg.app_key == "ust.teleop.gr1t2_gripper" and cfg.prefer_controller_for_eef
        _print(6, "GR1T2GripperDeviceCfg standalone instantiation", ok)
        if not ok:
            failures += 1
    except Exception as exc:  # noqa: BLE001
        failures += 1
        _print(6, "GR1T2GripperDeviceCfg standalone instantiation", False, repr(exc))

    # 7. coord_transforms chain (forearm_to_wrist + svr_to_isaaclab)
    try:
        import numpy as np
        from ust_ws.ust_hm_grip.teleop import coord_transforms as ct
        forearm = {
            "pos": np.array([0.5, 1.4, 0.3]),         # SteamVR world (Y up)
            "quat": np.array([1.0, 0.0, 0.0, 0.0]),    # identity
        }
        wrist = ct.forearm_to_wrist(forearm, (0.28, 0.0, 0.0))
        pos_il, quat_il = ct.svr_to_isaaclab(wrist["pos"], wrist["quat"])
        ok = (
            np.all(np.isfinite(pos_il))
            and np.all(np.isfinite(quat_il))
            and not np.allclose(pos_il, 0.0)
        )
        _print(7, "forearm_to_wrist + svr_to_isaaclab chain", ok,
               detail=f"pos_il={pos_il.tolist()}")
        if not ok:
            failures += 1
    except Exception as exc:  # noqa: BLE001
        failures += 1
        _print(7, "coord_transforms chain", False, repr(exc))

    print("=" * 60)
    if failures == 0:
        print(f"OK -- 7/7 passed")
        return 0
    else:
        print(f"FAIL -- {failures} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
