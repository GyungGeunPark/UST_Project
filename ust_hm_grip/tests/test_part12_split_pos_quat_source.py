"""Part 12 regression — split position/orientation sources.

User wants:
  * Wrist tracker (SMPL idx 20/21) → robot wrist EEF POSITION
  * Controller → robot wrist EEF ORIENTATION ONLY (no position
    contribution).  Previously the controller drove both pos and quat,
    so moving the controller moved the robot arm/wrist; the user wants
    the body skeleton wrist to drive position so the arm reach matches
    the user's actual hand location.

New cfg fields on ``GR1T2GripperRetargeterCfg``:
  * ``wrist_pos_source``: "controller" (legacy) | "wrist_tracker" (new default)
  * ``wrist_pose_zero``: {"left": {"pos": (3,)}, "right": {"pos": (3,)}}
    captured by run_teleop on first valid sample / A-button recal.
"""
from __future__ import annotations

import numpy as np
import pytest

from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_retargeter import (
    DEFAULT_LEFT_POS, DEFAULT_RIGHT_POS,
    GR1T2GripperRetargeterCfg, GR1T2GripperSteamVRRetargeter,
)


def _q_identity():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _make_snapshot(*, ctrl_left_pos, ctrl_left_quat=None,
                   ctrl_right_pos=(0.0, -0.3, 1.0),
                   wt_left_pos=None, wt_right_pos=None):
    if ctrl_left_quat is None:
        ctrl_left_quat = _q_identity()
    trackers = {"waist": {"pos": np.zeros(3), "quat": _q_identity()}}
    if wt_left_pos is not None:
        trackers["left_wrist"] = {
            "pos": np.asarray(wt_left_pos, dtype=np.float64),
            "quat": _q_identity(),
        }
    if wt_right_pos is not None:
        trackers["right_wrist"] = {
            "pos": np.asarray(wt_right_pos, dtype=np.float64),
            "quat": _q_identity(),
        }
    return {
        "hmd": {"pos": np.zeros(3), "quat": _q_identity()},
        "trackers": trackers,
        "controllers": {
            "left":  {"pose": {"pos": np.asarray(ctrl_left_pos, dtype=np.float64),
                                "quat": np.asarray(ctrl_left_quat, dtype=np.float64)},
                      "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
            "right": {"pose": {"pos": np.asarray(ctrl_right_pos, dtype=np.float64),
                                "quat": _q_identity()},
                      "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        },
    }


# ── default cfg values ─────────────────────────────────────────────────


def test_default_wrist_pos_source_is_controller():
    """Direct cfg default — keep legacy for backcompat in tests."""
    cfg = GR1T2GripperRetargeterCfg()
    assert cfg.wrist_pos_source == "controller"


def test_default_wrist_pose_zero_none():
    cfg = GR1T2GripperRetargeterCfg()
    assert cfg.wrist_pose_zero is None


def test_device_cfg_default_uses_wrist_tracker():
    """``GR1T2GripperDeviceCfg`` default is the new ``wrist_tracker``."""
    from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_device import (
        GR1T2GripperDeviceCfg,
    )
    dcfg = GR1T2GripperDeviceCfg()
    assert dcfg.wrist_pos_source == "wrist_tracker"


# ── controller-source mode (legacy) — unchanged behaviour ──────────────


def test_controller_pos_mode_unchanged_with_wrist_tracker_unused():
    """``wrist_pos_source='controller'`` ignores wrist tracker entirely."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="controller",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    # Controller moved +0.1 X.  Body wrist tracker far away at (5, 5, 5)
    # MUST be ignored because pos_source="controller".
    snap = _make_snapshot(
        ctrl_left_pos=(0.1, 0.3, 1.0),
        wt_left_pos=(5.0, 5.0, 5.0),
        wt_right_pos=(5.0, 5.0, 5.0),
    )
    action = rt.retarget(snap)
    expected_x = DEFAULT_LEFT_POS[0] + 0.1   # 1.0 X scale
    assert abs(float(action[0]) - expected_x) < 1e-5


# ── wrist_tracker-source mode (new default) ────────────────────────────


def test_wrist_tracker_pos_mode_uses_body_skeleton_delta():
    """Position delta comes from body wrist tracker, NOT controller."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0])},
        "right": {"pos": np.array([0.0, -0.3, 1.0])},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    # Controller did NOT move (delta 0).  Body wrist tracker moved +0.2 X.
    # In wrist_tracker mode, robot wrist target X should reflect tracker
    # delta, not controller delta.
    snap = _make_snapshot(
        ctrl_left_pos=(0.0, 0.3, 1.0),     # no change
        wt_left_pos=(0.2, 0.3, 1.0),       # +0.2 X
        wt_right_pos=(0.0, -0.3, 1.0),
    )
    action = rt.retarget(snap)
    expected_x = DEFAULT_LEFT_POS[0] + 0.2 * 1.0    # X axis_scale=1
    assert abs(float(action[0]) - expected_x) < 1e-5


def test_wrist_tracker_pos_isolated_from_controller_movement():
    """Moving ONLY the controller (not the body wrist tracker) must
    NOT translate the robot wrist target.  Controller drives quat only."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0])},
        "right": {"pos": np.array([0.0, -0.3, 1.0])},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    # Controller moved +0.5 m (huge).  Body wrist tracker did NOT move.
    snap = _make_snapshot(
        ctrl_left_pos=(0.5, 0.3, 1.0),
        wt_left_pos=(0.0, 0.3, 1.0),
        wt_right_pos=(0.0, -0.3, 1.0),
    )
    action = rt.retarget(snap)
    expected_x = DEFAULT_LEFT_POS[0]  # delta from tracker = 0
    assert abs(float(action[0]) - expected_x) < 1e-5


def test_wrist_tracker_z_axis_scale_applies():
    """Per-axis scale (default Z=1.5) still applies to wrist-tracker delta."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_pos_scale_per_axis=(1.0, 1.0, 1.5),
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0])},
        "right": {"pos": np.array([0.0, -0.3, 1.0])},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.0, 0.3, 1.0),
        wt_left_pos=(0.0, 0.3, 1.2),       # +0.2 Z body
        wt_right_pos=(0.0, -0.3, 1.0),
    )
    action = rt.retarget(snap)
    expected_z = DEFAULT_LEFT_POS[2] + 0.2 * 1.5
    assert abs(float(action[2]) - expected_z) < 1e-5


# ── orientation still from controller in wrist_tracker mode ────────────


def test_quat_still_from_controller_in_wrist_tracker_mode():
    """User rotates controller 90° about Z → wrist quat reflects it,
    even though pos source is body wrist tracker."""
    # 90° yaw rotation quat (wxyz)
    q90 = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)],
                    dtype=np.float64)
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0])},
        "right": {"pos": np.array([0.0, -0.3, 1.0])},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.0, 0.3, 1.0),
        ctrl_left_quat=q90,                   # rotated controller
        wt_left_pos=(0.0, 0.3, 1.0),
        wt_right_pos=(0.0, -0.3, 1.0),
    )
    action = rt.retarget(snap)
    left_quat = action[3:7].numpy()
    # Should NOT be identity-rotated idle quat.  Controller delta is
    # 90° yaw, so delta_q * idle_q must reflect that rotation.
    idle_q = np.array(DEFAULT_LEFT_POS) * 0  # placeholder for shape
    assert not np.allclose(left_quat, np.array([0.0, 0.0, 1.0, 0.0]), atol=1e-3), (
        "wrist quat should change with controller rotation, even in "
        "wrist_tracker pos source mode"
    )


# ── fallback to controller pos when wrist tracker unavailable ──────────


def test_no_controller_fallback_when_wrist_tracker_absent():
    """Part 13 update: when wrist_pos_source='wrist_tracker' but body
    tracker is missing, robot wrist target stays at IDLE (delta=0).
    Controller pos MUST NOT silently re-take ownership — that's exactly
    the coupling part 13 removed."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    # NO wrist_pose_zero captured (body data never arrived).
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.15, 0.3, 1.0),     # huge controller move
        wt_left_pos=None,                    # body tracker missing
        wt_right_pos=None,
    )
    action = rt.retarget(snap)
    # Robot stays at idle — controller doesn't reintroduce coupling.
    assert abs(float(action[0]) - DEFAULT_LEFT_POS[0]) < 1e-5
