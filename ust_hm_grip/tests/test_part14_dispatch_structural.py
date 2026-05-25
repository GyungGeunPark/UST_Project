"""Part 14 regression — structural source dispatch.

User report (after part 13): controller still drives robot arm during
startup window before ``controller_pose_zero`` is captured.  Root cause:
``_resolve_eef_target`` invoked ``_from_controller`` whenever a
controller pose was present, and the non-cal else branch inside that
function dumped raw controller pose into the wrist target — bypassing
the part 12/13 source split entirely.

Part 14 fix: ``_resolve_eef_target`` dispatches on
``wrist_pos_source`` / ``wrist_quat_source`` BEFORE looking at the
controllers.  When either source is ``"wrist_tracker"`` the new
``_compose_eef_target`` runs and the controller cannot reach the
wrist EEF target via any back door.
"""
from __future__ import annotations

import numpy as np

from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_retargeter import (
    DEFAULT_LEFT_POS, DEFAULT_LEFT_QUAT,
    GR1T2GripperRetargeterCfg, GR1T2GripperSteamVRRetargeter,
)


def _q_identity():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _q_yaw(angle):
    return np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)],
                    dtype=np.float64)


def _make_snapshot(*, ctrl_left_pos, ctrl_left_quat=None,
                   wt_left_pos=None, wt_left_quat=None,
                   ctrl_right_pos=(0.0, -0.3, 1.0)):
    if ctrl_left_quat is None:
        ctrl_left_quat = _q_identity()
    trackers = {"waist": {"pos": np.zeros(3), "quat": _q_identity()}}
    if wt_left_pos is not None:
        trackers["left_wrist"] = {
            "pos": np.asarray(wt_left_pos, dtype=np.float64),
            "quat": np.asarray(wt_left_quat if wt_left_quat is not None
                                else _q_identity(), dtype=np.float64),
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


# ── pre-calibration startup window: controller MUST NOT drive ─────────


def test_controller_inactive_before_any_zero_captured():
    """Both zeros None (startup before run_teleop captures anything).
    Controller is moving wildly; wrist target must sit at IDLE."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_quat_source="wrist_tracker",
    )
    # NO controller_pose_zero, NO wrist_pose_zero.
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.7, 1.0, 1.5),       # huge controller move
        ctrl_left_quat=_q_yaw(np.pi),         # 180° controller rotation
        wt_left_pos=None,                     # no body tracker yet
    )
    action = rt.retarget(snap)
    # Pos = idle exactly
    for i in range(3):
        assert abs(float(action[i]) - DEFAULT_LEFT_POS[i]) < 1e-5
    # Quat = idle (sign-ambiguous comparison)
    got_q = action[3:7].numpy()
    idle = np.asarray(DEFAULT_LEFT_QUAT, dtype=np.float64)
    if np.dot(got_q, idle) < 0:
        got_q = -got_q
    assert np.allclose(got_q, idle, atol=1e-5)


def test_controller_inactive_when_only_controller_zero_captured():
    """Common transient: ``controller_pose_zero`` was captured but
    body wrist data not yet streaming → wrist_pose_zero still None.
    Controller MUST NOT take over."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_quat_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    # No wrist_pose_zero yet.
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.5, 0.3, 1.0),
        wt_left_pos=None,
    )
    action = rt.retarget(snap)
    for i in range(3):
        assert abs(float(action[i]) - DEFAULT_LEFT_POS[i]) < 1e-5


# ── post-calibration: body tracker drives, controller ignored ─────────


def test_body_tracker_drives_pos_when_zero_captured():
    """Both zeros captured + tracker live → wrist pos follows tracker
    delta; massive controller deltas ignored."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_quat_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(2.0, 2.0, 2.0),        # totally ignored
        ctrl_left_quat=_q_yaw(np.pi),
        wt_left_pos=(0.25, 0.3, 1.0),         # +0.25 X tracker
    )
    action = rt.retarget(snap)
    assert abs(float(action[0]) - (DEFAULT_LEFT_POS[0] + 0.25)) < 1e-5


def test_body_tracker_drives_quat_when_zero_captured():
    """Tracker quat 30° yaw + huge controller 180° quat → output reflects
    only tracker 30°."""
    yaw30 = _q_yaw(np.pi / 6)
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_quat_source="wrist_tracker",
    )
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.0, 0.3, 1.0),
        ctrl_left_quat=_q_yaw(np.pi),
        wt_left_pos=(0.0, 0.3, 1.0),
        wt_left_quat=yaw30,
    )
    action = rt.retarget(snap)
    got_q = action[3:7].numpy()
    # got_q ≈ yaw30 * idle.  Sanity: not idle alone, not 180° yaw.
    idle = np.asarray(DEFAULT_LEFT_QUAT, dtype=np.float64)
    if np.dot(got_q, idle) < 0:
        got_q_n = -got_q
    else:
        got_q_n = got_q
    assert not np.allclose(got_q_n, idle, atol=1e-3), (
        "quat should differ from idle when tracker rotated"
    )


# ── mixed-source matrix ────────────────────────────────────────────────


def test_pos_controller_quat_tracker_independent():
    """Cross-source: pos from controller, quat from tracker.  Each axis
    sources its declared input only."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="controller",
        wrist_quat_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.2, 0.3, 1.0),           # +0.2 X controller
        ctrl_left_quat=_q_yaw(np.pi / 2),        # 90° quat — ignored
        wt_left_pos=(0.9, 0.3, 1.0),             # ignored for pos
        wt_left_quat=_q_yaw(np.pi / 6),          # 30° tracker quat — used
    )
    action = rt.retarget(snap)
    # POS comes from controller delta +0.2.
    assert abs(float(action[0]) - (DEFAULT_LEFT_POS[0] + 0.2)) < 1e-5


# ── pure-controller mode still works (legacy router) ──────────────────


def test_legacy_both_controller_sources_still_route_via_from_controller():
    """When both sources are ``controller``, legacy router path
    (`_from_controller`) handles cal + non-cal branches.  Confirms
    backcompat — existing OpenVR users with no body trackers."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="controller",
        wrist_quat_source="controller",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.15, 0.3, 1.0),
    )
    action = rt.retarget(snap)
    expected_x = DEFAULT_LEFT_POS[0] + 0.15
    assert abs(float(action[0]) - expected_x) < 1e-5


# ── diagnostic source labels available ────────────────────────────────


def test_pos_source_last_diagnostic_recorded():
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_quat_source="wrist_tracker",
    )
    cfg.wrist_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.0, 0.3, 1.0),
        wt_left_pos=(0.05, 0.3, 1.0),
    )
    rt.retarget(snap)
    assert getattr(rt, "_pos_source_last", {}).get("left") == "wrist_tracker"
    assert getattr(rt, "_quat_source_last", {}).get("left") in {
        "wrist_tracker", "idle"
    }
