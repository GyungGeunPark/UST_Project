"""pytest regression tests for ``GR1T2GripperSteamVRRetargeter``.

These tests run WITHOUT Isaac Sim — only ``numpy`` and ``torch`` are
required.  Run with::

    PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_grip/tests/
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import torch

from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_retargeter import (
    ACTION_DIM,
    DEFAULT_LEFT_POS,
    DEFAULT_LEFT_QUAT,
    DEFAULT_RIGHT_POS,
    DEFAULT_RIGHT_QUAT,
    GRIPPER_CLOSE,
    GRIPPER_OPEN,
    GR1T2GripperRetargeterCfg,
    GR1T2GripperSteamVRRetargeter,
)


def _empty_snap() -> dict:
    return {"frame_count": 1, "trackers": {}, "controllers": {}}


def _ai(left_trigger=0.0, left_grip=0.0, right_trigger=0.0, right_grip=0.0):
    return {
        "left":  {"trigger": left_trigger,  "grip": left_grip,  "menu": False},
        "right": {"trigger": right_trigger, "grip": right_grip, "menu": False},
    }


# ── Action shape / type ────────────────────────────────────────────────
def test_action_dim_is_16():
    assert ACTION_DIM == 16


def test_retarget_returns_float32_tensor_of_correct_shape():
    rt = GR1T2GripperSteamVRRetargeter()
    out = rt.retarget(_empty_snap(), action_inputs=None)
    assert isinstance(out, torch.Tensor)
    assert out.dtype == torch.float32
    assert tuple(out.shape) == (16,)


# ── Idle behaviour ─────────────────────────────────────────────────────
def test_idle_action_matches_default_pose_and_open_grippers():
    rt = GR1T2GripperSteamVRRetargeter()
    out = rt.retarget(_empty_snap(), action_inputs=None)
    np.testing.assert_allclose(out[0:3].numpy(), np.array(DEFAULT_LEFT_POS), atol=1e-5)
    np.testing.assert_allclose(out[3:7].numpy(), np.array(DEFAULT_LEFT_QUAT), atol=1e-5)
    np.testing.assert_allclose(out[7:10].numpy(), np.array(DEFAULT_RIGHT_POS), atol=1e-5)
    np.testing.assert_allclose(out[10:14].numpy(), np.array(DEFAULT_RIGHT_QUAT), atol=1e-5)
    assert float(out[14]) == GRIPPER_OPEN
    assert float(out[15]) == GRIPPER_OPEN


def test_disable_arm_tracking_keeps_idle_pose():
    cfg = GR1T2GripperRetargeterCfg(disable_arm_tracking=True)
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _empty_snap()
    # Even with bogus forearm pose far away, idle is enforced.
    snap["trackers"] = {
        "left_forearm": {"pos": (5.0, 5.0, 5.0), "quat": (1.0, 0, 0, 0)},
    }
    out = rt.retarget(snap)
    np.testing.assert_allclose(out[0:3].numpy(), np.array(DEFAULT_LEFT_POS), atol=1e-5)


# ── Gripper threshold + hysteresis ─────────────────────────────────────
# 9.28: default gripper_signal_source is now "grip"; trigger-driven tests
# explicitly opt into "trigger" source to preserve their original semantic.
def test_left_trigger_above_close_threshold_closes_gripper():
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="trigger")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.8))
    assert float(out[14]) == GRIPPER_CLOSE
    assert float(out[15]) == GRIPPER_OPEN


def test_right_trigger_above_close_threshold_closes_only_right():
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="trigger")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(_empty_snap(), action_inputs=_ai(right_trigger=0.7))
    assert float(out[14]) == GRIPPER_OPEN
    assert float(out[15]) == GRIPPER_CLOSE


def test_grip_alone_can_close_when_use_grip_as_close_true():
    # New default already chooses grip; this test now also matches the
    # explicit "grip" source to make intent unambiguous.
    cfg = GR1T2GripperRetargeterCfg(use_grip_as_close=True, gripper_signal_source="grip")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_grip=0.9))
    assert float(out[14]) == GRIPPER_CLOSE


def test_grip_ignored_when_signal_source_trigger():
    """When the source is 'trigger', grip alone must NOT close.  Replaces
    the old ``test_grip_ignored_when_use_grip_as_close_false`` which
    relied on the now-deprecated ``use_grip_as_close`` switch."""
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="trigger")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_grip=0.9))
    assert float(out[14]) == GRIPPER_OPEN


def test_hysteresis_holds_through_midband():
    """Once closed, a trigger value between open and close thresholds
    keeps the gripper closed.  A trigger below the open threshold
    re-opens it."""
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="trigger")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _empty_snap()

    # Step 1: close
    out = rt.retarget(snap, action_inputs=_ai(left_trigger=0.8))
    assert float(out[14]) == GRIPPER_CLOSE

    # Step 2: hold mid-band
    out = rt.retarget(snap, action_inputs=_ai(left_trigger=0.5))
    assert float(out[14]) == GRIPPER_CLOSE  # still closed

    # Step 3: release
    out = rt.retarget(snap, action_inputs=_ai(left_trigger=0.3))
    assert float(out[14]) == GRIPPER_OPEN


def test_threshold_validation_via_cfg():
    """Constructing the cfg with open >= close still works (no built-in
    validation), but the run_teleop CLI rejects this.  We test the
    retargeter just doesn't crash."""
    cfg = GR1T2GripperRetargeterCfg(
        gripper_close_threshold=0.5,
        gripper_open_threshold=0.5,
        gripper_signal_source="trigger",
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.7))


# ── 9.28 new: gripper_signal_source field ──────────────────────────────
def test_default_signal_source_is_grip():
    """Default cfg should now reflect the user's request: grip drives close."""
    cfg = GR1T2GripperRetargeterCfg()
    assert cfg.gripper_signal_source == "grip"


def test_signal_source_grip_closes_on_grip_only():
    rt = GR1T2GripperSteamVRRetargeter()  # default = grip
    # grip closes
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_grip=0.9))
    assert float(out[14]) == GRIPPER_CLOSE
    # reset and try trigger only — should NOT close
    rt.reset()
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.9))
    assert float(out[14]) == GRIPPER_OPEN


def test_signal_source_trigger_closes_on_trigger_only():
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="trigger")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.9))
    assert float(out[14]) == GRIPPER_CLOSE
    rt.reset()
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_grip=0.9))
    assert float(out[14]) == GRIPPER_OPEN


def test_signal_source_both_acts_as_logical_or():
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="both")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    # grip alone → close
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_grip=0.9))
    assert float(out[14]) == GRIPPER_CLOSE
    rt.reset()
    # trigger alone → close
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.9))
    assert float(out[14]) == GRIPPER_CLOSE
    rt.reset()
    # neither above threshold → open
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.3, left_grip=0.3))
    assert float(out[14]) == GRIPPER_OPEN


def test_signal_source_unknown_falls_back_to_grip():
    """An unknown source string should not crash; it falls back to 'grip'."""
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="garbage")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(_empty_snap(), action_inputs=_ai(left_grip=0.9))
    assert float(out[14]) == GRIPPER_CLOSE


# ── EEF resolution priority ────────────────────────────────────────────
def test_controller_pose_takes_priority_when_prefer_controller_true():
    rt = GR1T2GripperSteamVRRetargeter()
    snap = _empty_snap()
    snap["controllers"] = {
        "left": {
            "pose": {
                "pos": np.array([1.0, 1.0, 0.5]),
                "quat": np.array([1.0, 0.0, 0.0, 0.0]),
            },
            "buttons": {},
        },
        "right": None,
    }
    out = rt.retarget(snap)
    info = rt.get_source_info()
    assert info["left_eef"] == "controller"
    # Pose should not be the default idle anymore.
    assert not np.allclose(out[0:3].numpy(), np.array(DEFAULT_LEFT_POS), atol=1e-3)


def test_forearm_pose_used_when_no_controller():
    rt = GR1T2GripperSteamVRRetargeter()
    snap = _empty_snap()
    snap["trackers"] = {
        "left_forearm": {
            "pos": np.array([0.5, 1.4, 0.3]),
            "quat": np.array([1.0, 0.0, 0.0, 0.0]),
        }
    }
    out = rt.retarget(snap)
    info = rt.get_source_info()
    assert info["left_eef"] == "forearm"
    assert not np.allclose(out[0:3].numpy(), np.array(DEFAULT_LEFT_POS), atol=1e-3)


def test_default_pose_when_neither_source_available():
    rt = GR1T2GripperSteamVRRetargeter()
    out = rt.retarget(_empty_snap())
    info = rt.get_source_info()
    assert info["left_eef"] == "default"
    assert info["right_eef"] == "default"


# ── Right wrist Z180 correction ────────────────────────────────────────
def test_right_wrist_z180_changes_quat():
    cfg = GR1T2GripperRetargeterCfg(right_wrist_z180=True)
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _empty_snap()
    snap["controllers"] = {
        "right": {
            "pose": {
                "pos": np.array([1.0, 1.4, 0.3]),
                "quat": np.array([1.0, 0.0, 0.0, 0.0]),  # identity SVR
            },
            "buttons": {},
        },
        "left": None,
    }
    out_with = rt.retarget(snap)

    cfg2 = GR1T2GripperRetargeterCfg(right_wrist_z180=False)
    rt2 = GR1T2GripperSteamVRRetargeter(cfg2)
    out_without = rt2.retarget(snap)

    # Quaternion should differ between the two configs.
    assert not np.allclose(out_with[10:14].numpy(), out_without[10:14].numpy(), atol=1e-4)


# ── Reset behaviour ────────────────────────────────────────────────────
def test_reset_returns_to_open_state():
    cfg = GR1T2GripperRetargeterCfg(gripper_signal_source="trigger")
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.9))
    assert rt._gripper_state_left == GRIPPER_CLOSE  # noqa: SLF001
    rt.reset()
    assert rt._gripper_state_left == GRIPPER_OPEN  # noqa: SLF001
    assert rt._gripper_state_right == GRIPPER_OPEN  # noqa: SLF001
    assert rt._frames == 0  # noqa: SLF001


# ── Forearm offset effect ──────────────────────────────────────────────
def test_forearm_wrist_offset_affects_pos():
    """Different forearm offsets should produce different wrist positions."""
    snap = _empty_snap()
    snap["trackers"] = {
        "left_forearm": {
            "pos": np.array([0.5, 1.4, 0.3]),
            "quat": np.array([1.0, 0.0, 0.0, 0.0]),
        }
    }
    rt_short = GR1T2GripperSteamVRRetargeter(
        GR1T2GripperRetargeterCfg(forearm_wrist_offset=(0.12, 0.0, 0.0))
    )
    rt_long = GR1T2GripperSteamVRRetargeter(
        GR1T2GripperRetargeterCfg(forearm_wrist_offset=(0.28, 0.0, 0.0))
    )
    out_short = rt_short.retarget(snap)
    out_long = rt_long.retarget(snap)
    diff = np.abs(out_short[0:3].numpy() - out_long[0:3].numpy()).max()
    assert diff > 0.05


# ── Source info diagnostic ─────────────────────────────────────────────
def test_source_info_reports_max_trigger():
    rt = GR1T2GripperSteamVRRetargeter()
    rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.8))
    rt.retarget(_empty_snap(), action_inputs=_ai(left_trigger=0.5))
    info = rt.get_source_info()
    assert info["max_trigger_left"] >= 0.8
    assert info["max_trigger_right"] == 0.0


# ── pose_in_il_frame (13th session) ────────────────────────────────────
def test_pose_in_il_frame_skips_double_transform():
    """When the sampler already converts poses to IL frame (XR backend),
    setting ``pose_in_il_frame=True`` must skip the legacy
    ``svr_to_isaaclab`` step.  Otherwise R_SVR2IL is applied twice and
    the controller pose lands somewhere unrelated.

    Test pattern: feed a controller pose that's already in IL frame
    (e.g. xrobotoolkit pre-converted) and compare against feeding the
    same numerical values through the legacy SVR path.  The IL-frame
    branch should produce a position close to the *raw* controller pos
    (up to forearm-to-wrist offset + idle-pose origin subtraction),
    while the SVR-frame branch transforms it.
    """
    # Controller pose at (0.4, -0.3, 1.2) in IL coords with identity quat.
    # In IL: +X=forward, +Y=left, +Z=up.  So this is 0.4 m forward,
    # 0.3 m to the user's right, 1.2 m above ground — a plausible
    # hand-out-front-and-slightly-right pose.
    ctrl_il_pos = np.array([0.4, -0.3, 1.2], dtype=np.float64)
    ctrl_il_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    snap = _empty_snap()
    snap["controllers"] = {
        "left":  {"pose": {"pos": ctrl_il_pos.copy(), "quat": ctrl_il_quat.copy()},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        "right": {"pose": {"pos": ctrl_il_pos.copy(), "quat": ctrl_il_quat.copy()},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
    }

    # use_waist_origin=False so we can compare raw output directly.
    cfg_il = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=False,
        pose_in_il_frame=True,
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
    )
    rt_il = GR1T2GripperSteamVRRetargeter(cfg_il)
    out_il = rt_il.retarget(snap)

    cfg_svr = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=False,
        pose_in_il_frame=False,  # legacy path
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
    )
    rt_svr = GR1T2GripperSteamVRRetargeter(cfg_svr)
    out_svr = rt_svr.retarget(snap)

    # IL-frame branch: pass-through (no transform) → wrist pos = controller pos
    np.testing.assert_allclose(out_il[0:3].numpy(), ctrl_il_pos, atol=1e-5)
    # SVR-frame branch: applies svr_to_isaaclab once → result differs
    diff = float(np.abs(out_svr[0:3].numpy() - ctrl_il_pos).max())
    assert diff > 0.1, (
        "Expected the legacy SVR path to transform the IL-frame input "
        f"(double-transform), got SVR_out={out_svr[0:3].numpy()} ≈ IL input "
        f"{ctrl_il_pos} — flag may have no effect."
    )


# ── Startup wrist calibration (13th-bis, controller_pose_zero) ────────
def test_controller_pose_zero_anchors_idle_at_startup():
    """When controller_pose_zero is set to the current controller poses
    AND the controllers haven't moved, the retargeter returns the IDLE
    wrist target (so the robot stays in T-pose at startup regardless of
    where the user's hands happen to be)."""
    ctrl_il_pos_l = np.array([0.4, 0.1, 0.5], dtype=np.float64)
    ctrl_il_pos_r = np.array([0.5, -0.1, 0.6], dtype=np.float64)
    snap = _empty_snap()
    snap["controllers"] = {
        "left":  {"pose": {"pos": ctrl_il_pos_l.copy(),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        "right": {"pose": {"pos": ctrl_il_pos_r.copy(),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
    }
    cfg = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=False,
        pose_in_il_frame=True,
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
        controller_pose_zero={
            "left":  {"pos": ctrl_il_pos_l.copy(),
                      "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "right": {"pos": ctrl_il_pos_r.copy(),
                      "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        },
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(snap)
    # Left wrist = idle pose
    np.testing.assert_allclose(out[0:3].numpy(), np.array(DEFAULT_LEFT_POS),
                                atol=1e-5)
    # Right wrist = idle pose
    np.testing.assert_allclose(out[7:10].numpy(), np.array(DEFAULT_RIGHT_POS),
                                atol=1e-5)


def test_controller_pose_zero_delta_tracks_user_movement():
    """User moves controller +0.1m in X relative to startup → robot wrist
    moves +0.1m in X from idle.  Position scale = 1.0 → 1:1 tracking."""
    ctrl_il_zero = np.array([0.4, 0.1, 0.5], dtype=np.float64)
    ctrl_il_now = ctrl_il_zero + np.array([0.1, 0.0, 0.0])  # moved +X
    snap = _empty_snap()
    snap["controllers"] = {
        "left":  {"pose": {"pos": ctrl_il_now.copy(),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        "right": {"pose": {"pos": ctrl_il_zero.copy(),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
    }
    cfg = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=False,
        pose_in_il_frame=True,
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
        position_scale=1.0,
        controller_pose_zero={
            "left":  {"pos": ctrl_il_zero.copy(),
                      "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "right": {"pos": ctrl_il_zero.copy(),
                      "quat": np.array([1.0, 0.0, 0.0, 0.0])},
        },
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(snap)
    # Left wrist = idle + delta (+0.1 in X)
    expected_l = np.array(DEFAULT_LEFT_POS) + np.array([0.1, 0.0, 0.0])
    np.testing.assert_allclose(out[0:3].numpy(), expected_l, atol=1e-5)
    # Right wrist = idle (no movement)
    np.testing.assert_allclose(out[7:10].numpy(), np.array(DEFAULT_RIGHT_POS),
                                atol=1e-5)


def test_controller_pose_zero_wrist_quat_starts_at_idle():
    """At calibration moment (raw quat == zero quat), wrist target
    quaternion must equal the idle quaternion — robot wrist sits at
    idle orientation regardless of user's controller orientation."""
    # User holds controller with some arbitrary tilt
    user_quat = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64)  # 120° about (1,1,1)
    snap = _empty_snap()
    snap["controllers"] = {
        "left":  {"pose": {"pos": np.array([0.4, 0.1, 0.5]),
                            "quat": user_quat.copy()},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        "right": {"pose": {"pos": np.array([0.5, -0.1, 0.6]),
                            "quat": user_quat.copy()},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
    }
    cfg = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=False,
        pose_in_il_frame=True,
        right_wrist_z180=False,  # disable Z180 so we can compare with raw idle
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
        controller_pose_zero={
            "left":  {"pos": np.array([0.4, 0.1, 0.5]),
                      "quat": user_quat.copy()},
            "right": {"pos": np.array([0.5, -0.1, 0.6]),
                      "quat": user_quat.copy()},
        },
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(snap)
    # Left wrist quat == idle_left_quat (normalised by quat_multiply
    # internally, so tolerance loosened to 1e-3 to accommodate the
    # difference between literal (0.707, 0, 0.707, 0) and the proper
    # √0.5 = 0.7071068).
    np.testing.assert_allclose(out[3:7].numpy(), np.array(DEFAULT_LEFT_QUAT),
                                atol=1e-3)
    np.testing.assert_allclose(out[10:14].numpy(), np.array(DEFAULT_RIGHT_QUAT),
                                atol=1e-3)


def test_controller_pose_zero_wrist_quat_tracks_delta():
    """User rotates controller 90° about IL +Z (yaw rotation) after
    calibration → robot wrist target rotates by that same 90° applied
    to idle orientation."""
    user_zero = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # identity zero
    # User now rotates 90° about IL +Z (yaw left)
    half = np.sqrt(0.5)
    user_now = np.array([half, 0.0, 0.0, half], dtype=np.float64)
    snap = _empty_snap()
    snap["controllers"] = {
        "left":  {"pose": {"pos": np.array([0.4, 0.1, 0.5]),
                            "quat": user_now.copy()},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        "right": {"pose": {"pos": np.array([0.5, -0.1, 0.6]),
                            "quat": user_zero.copy()},  # right unchanged
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
    }
    cfg = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=False,
        pose_in_il_frame=True,
        right_wrist_z180=False,
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
        controller_pose_zero={
            "left":  {"pos": np.array([0.4, 0.1, 0.5]),
                      "quat": user_zero.copy()},
            "right": {"pos": np.array([0.5, -0.1, 0.6]),
                      "quat": user_zero.copy()},
        },
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(snap)
    # delta_q = 90° about +Z
    # left_wrist = delta_q * idle_left_quat
    from ust_ws.ust_hm_grip.teleop import coord_transforms as _ct
    expected_l = _ct.quat_multiply(user_now, np.array(DEFAULT_LEFT_QUAT))
    np.testing.assert_allclose(out[3:7].numpy(), expected_l, atol=1e-3)
    # Right wrist = identity delta * idle_right_quat = idle_right_quat
    np.testing.assert_allclose(out[10:14].numpy(), np.array(DEFAULT_RIGHT_QUAT),
                                atol=1e-3)


def test_controller_pose_zero_partial_fallback():
    """If only one side has a calibration zero, the other side falls
    back to the legacy waist-origin-subtract path."""
    snap = _empty_snap()
    snap["controllers"] = {
        "left":  {"pose": {"pos": np.array([0.4, 0.1, 0.5]),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        "right": {"pose": {"pos": np.array([0.5, -0.1, 0.6]),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
    }
    cfg = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=False,
        pose_in_il_frame=True,
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
        controller_pose_zero={
            "left":  {"pos": np.array([0.4, 0.1, 0.5]),
                      "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            # right intentionally missing
        },
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(snap)
    # Left = idle (calibrated)
    np.testing.assert_allclose(out[0:3].numpy(), np.array(DEFAULT_LEFT_POS),
                                atol=1e-5)
    # Right = raw IL pose (legacy path, no waist origin since use_waist_origin=False)
    np.testing.assert_allclose(out[7:10].numpy(), np.array([0.5, -0.1, 0.6]),
                                atol=1e-5)


def test_pose_in_il_frame_waist_origin_passthrough():
    """When pose_in_il_frame=True and a waist tracker is supplied in IL
    frame, the resolved EEF should subtract the IL-frame waist XY from
    the IL-frame controller pose without any svr_to_isaaclab call."""
    snap = _empty_snap()
    snap["trackers"] = {
        "waist": {"pos": np.array([0.05, 0.10, 1.0]),
                  "quat": np.array([1.0, 0.0, 0.0, 0.0])},
    }
    snap["controllers"] = {
        "left":  {"pose": {"pos": np.array([0.4, 0.0, 1.2]),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        "right": {"pose": {"pos": np.array([0.4, 0.0, 1.2]),
                            "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                  "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
    }
    cfg = GR1T2GripperRetargeterCfg(
        prefer_controller_for_eef=True,
        use_waist_origin=True,
        subtract_waist_z=False,
        pose_in_il_frame=True,
        controller_to_wrist_offset=(0.0, 0.0, 0.0),
        controller_pos_offset=(0.0, 0.0, 0.0),
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    out = rt.retarget(snap)
    # Expected: ctrl_pos - waist_xy = (0.4-0.05, 0.0-0.10, 1.2)
    np.testing.assert_allclose(
        out[0:3].numpy(), np.array([0.35, -0.10, 1.2]), atol=1e-5,
    )
