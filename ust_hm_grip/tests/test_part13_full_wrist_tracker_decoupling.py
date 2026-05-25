"""Part 13 regression — controller fully decoupled from wrist EEF target.

User report (after part 12): controller still drove robot arm position
because the silent fallback ``wrist_pos_source="wrist_tracker"`` had
when the body skeleton wasn't yet streaming.  Furthermore, controller
orientation also still drove the wrist quat.

Part 13 fixes:
  * Add ``wrist_quat_source`` cfg (default ``"wrist_tracker"``).
  * NO controller-pos fallback when ``wrist_pos_source="wrist_tracker"``
    and the tracker is unavailable — wrist stays at IDLE delta=0
    instead of silently re-coupling controller pose.
  * NO controller-quat fallback when ``wrist_quat_source="wrist_tracker"``
    and the tracker is unavailable — wrist quat snaps to idle_q.
  * ``wrist_pose_zero[side]`` now carries BOTH ``pos`` and ``quat`` so
    SMPL wrist quat can be deltaed against startup orientation.
"""
from __future__ import annotations

import numpy as np
import pytest

from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_retargeter import (
    DEFAULT_LEFT_POS, DEFAULT_LEFT_QUAT, DEFAULT_RIGHT_POS,
    GR1T2GripperRetargeterCfg, GR1T2GripperSteamVRRetargeter,
)


def _q_identity():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _q_yaw(angle):
    return np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)],
                    dtype=np.float64)


def _make_snapshot(*, ctrl_left_pos, ctrl_left_quat=None,
                   wt_left_pos=None, wt_left_quat=None,
                   ctrl_right_pos=(0.0, -0.3, 1.0),
                   wt_right_pos=None, wt_right_quat=None):
    if ctrl_left_quat is None:
        ctrl_left_quat = _q_identity()
    trackers = {"waist": {"pos": np.zeros(3), "quat": _q_identity()}}
    if wt_left_pos is not None:
        trackers["left_wrist"] = {
            "pos": np.asarray(wt_left_pos, dtype=np.float64),
            "quat": np.asarray(wt_left_quat if wt_left_quat is not None
                                else _q_identity(), dtype=np.float64),
        }
    if wt_right_pos is not None:
        trackers["right_wrist"] = {
            "pos": np.asarray(wt_right_pos, dtype=np.float64),
            "quat": np.asarray(wt_right_quat if wt_right_quat is not None
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


# ── cfg defaults ───────────────────────────────────────────────────────


def test_default_wrist_quat_source_cfg_is_controller():
    """Direct cfg default for backcompat; device cfg flips to tracker."""
    cfg = GR1T2GripperRetargeterCfg()
    assert cfg.wrist_quat_source == "controller"


def test_device_cfg_default_split_matches_pico_app():
    """Part 15 — match PICO Motion Tracker app behavior: SMPL body
    drives wrist position; controller drives wrist rotation (since SMPL
    wrist joints don't carry fine controller-grip rotation, per NVIDIA
    SONIC/GR00T ZMQ v3 and research/48).
    """
    from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_device import (
        GR1T2GripperDeviceCfg,
    )
    dcfg = GR1T2GripperDeviceCfg()
    assert dcfg.wrist_pos_source == "wrist_tracker"
    assert dcfg.wrist_quat_source == "controller"


# ── tracker available: pos+quat both from tracker ─────────────────────


def test_pos_and_quat_both_from_wrist_tracker():
    """Both pos+quat source = wrist_tracker, tracker live → controller
    contribution = zero on both pos and quat."""
    yaw45 = _q_yaw(np.pi / 4)
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
    # Controller moved + rotated MASSIVELY; tracker only +0.1 X and yaw45.
    snap = _make_snapshot(
        ctrl_left_pos=(0.9, 0.3, 1.0),
        ctrl_left_quat=_q_yaw(np.pi),   # 180° controller
        wt_left_pos=(0.1, 0.3, 1.0),
        wt_left_quat=yaw45,
        wt_right_pos=(0.0, -0.3, 1.0),
    )
    action = rt.retarget(snap)
    # X target reflects ONLY tracker delta +0.1 (1.0 X scale).
    assert abs(float(action[0]) - (DEFAULT_LEFT_POS[0] + 0.1)) < 1e-5
    # Quat target reflects ONLY tracker yaw45 * idle_q.
    # Controller's 180° yaw is irrelevant.
    expected_q = np.array([
        yaw45[0] * DEFAULT_LEFT_QUAT[0] - yaw45[1] * DEFAULT_LEFT_QUAT[1]
        - yaw45[2] * DEFAULT_LEFT_QUAT[2] - yaw45[3] * DEFAULT_LEFT_QUAT[3],
        yaw45[0] * DEFAULT_LEFT_QUAT[1] + yaw45[1] * DEFAULT_LEFT_QUAT[0]
        + yaw45[2] * DEFAULT_LEFT_QUAT[3] - yaw45[3] * DEFAULT_LEFT_QUAT[2],
        yaw45[0] * DEFAULT_LEFT_QUAT[2] - yaw45[1] * DEFAULT_LEFT_QUAT[3]
        + yaw45[2] * DEFAULT_LEFT_QUAT[0] + yaw45[3] * DEFAULT_LEFT_QUAT[1],
        yaw45[0] * DEFAULT_LEFT_QUAT[3] + yaw45[1] * DEFAULT_LEFT_QUAT[2]
        - yaw45[2] * DEFAULT_LEFT_QUAT[1] + yaw45[3] * DEFAULT_LEFT_QUAT[0],
    ], dtype=np.float64)
    expected_q /= np.linalg.norm(expected_q)
    got_q = action[3:7].numpy()
    # Allow sign ambiguity (q and -q same rotation).
    if np.dot(got_q, expected_q) < 0:
        got_q = -got_q
    assert np.allclose(got_q, expected_q, atol=1e-4), (
        f"expected quat ≈ {expected_q}, got {got_q}"
    )


# ── tracker missing: NO controller fallback, snap to idle ─────────────


def test_pos_falls_back_to_idle_not_controller_when_tracker_missing():
    """Body tracker absent, wrist_pos_source='wrist_tracker' → wrist
    target POS = idle (delta=0).  Controller pose MUST NOT influence."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_quat_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    # Body wrist tracker NOT in snapshot, no wrist_pose_zero captured.
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.5, 0.3, 1.0),     # +0.5 X controller move
        wt_left_pos=None,
        wt_right_pos=None,
    )
    action = rt.retarget(snap)
    # Wrist target stays at idle pos regardless of controller move.
    assert abs(float(action[0]) - DEFAULT_LEFT_POS[0]) < 1e-5
    assert abs(float(action[1]) - DEFAULT_LEFT_POS[1]) < 1e-5
    assert abs(float(action[2]) - DEFAULT_LEFT_POS[2]) < 1e-5


def test_quat_falls_back_to_idle_not_controller_when_tracker_missing():
    """Tracker missing → wrist quat = idle_q, controller quat ignored."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_source="wrist_tracker",
        wrist_quat_source="wrist_tracker",
    )
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]), "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]), "quat": _q_identity()},
    }
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    snap = _make_snapshot(
        ctrl_left_pos=(0.0, 0.3, 1.0),
        ctrl_left_quat=_q_yaw(np.pi),     # 180° controller rotation
        wt_left_pos=None,
    )
    action = rt.retarget(snap)
    got_q = action[3:7].numpy()
    idle = np.asarray(DEFAULT_LEFT_QUAT, dtype=np.float64)
    if np.dot(got_q, idle) < 0:
        got_q = -got_q
    assert np.allclose(got_q, idle, atol=1e-4)


# ── controller cannot reintroduce pos coupling even if controller_pose_zero set ──


def test_huge_controller_translation_zero_robot_effect():
    """1 m controller movement → 0 robot wrist effect."""
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
        ctrl_left_pos=(1.0, 1.3, 2.0),       # ridiculous controller
        wt_left_pos=(0.0, 0.3, 1.0),         # tracker idle
        wt_right_pos=(0.0, -0.3, 1.0),
    )
    action = rt.retarget(snap)
    # Robot stays at idle regardless of controller insanity.
    for i, expected in enumerate(DEFAULT_LEFT_POS):
        assert abs(float(action[i]) - expected) < 1e-5


# ── wrist_pose_zero must accept quat key now ──────────────────────────


def test_wrist_pose_zero_carries_pos_and_quat():
    """Schema check — zero dict must accept both pos and quat keys."""
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
    yaw30 = _q_yaw(np.pi / 6)
    snap = _make_snapshot(
        ctrl_left_pos=(0.0, 0.3, 1.0),
        wt_left_pos=(0.0, 0.3, 1.0),
        wt_left_quat=yaw30,
        wt_right_pos=(0.0, -0.3, 1.0),
    )
    action = rt.retarget(snap)
    # Quat must reflect 30° yaw delta from zero.
    got_q = action[3:7].numpy()
    # Just check that quat is NOT identity-equal to idle_q.
    idle = np.asarray(DEFAULT_LEFT_QUAT, dtype=np.float64)
    if np.dot(got_q, idle) < 0:
        got_q_n = -got_q
    else:
        got_q_n = got_q
    assert not np.allclose(got_q_n, idle, atol=1e-3), (
        "wrist quat should reflect tracker yaw rotation"
    )
