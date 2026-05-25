"""Part 10 regression — scales + head actuator restoration.

User report (2026-05-17 00:39): part 9 fix worked for waist yaw only.
Remaining: waist pitch weak, wrist Z weak, head rotation absent.

Root causes:
  * Head: ``GR1T2_HIGH_PD_CFG`` (isaaclab_assets/robots/fourier.py:129)
    uses ``replace(actuators={...})`` which OVERWRITES the entire
    actuators dict from base ``GR1T2_CFG``, silently dropping the
    ``"head"`` actuator group.  No PD on head joints → joint targets
    written by Phase D never produce motion.
  * Waist pitch / wrist Z: pelvis tracker captures pelvis-only rotation
    (small for spinal forward bend), and controller delta Z maps 1:1
    by default.  Need amplification factors.

Fix: add head actuator in ``env_cfg``, add per-axis scales for waist
Euler delta and wrist position delta.
"""
from __future__ import annotations

import numpy as np
import pytest

from ust_ws.ust_hm_grip.teleop import coord_transforms as ct
from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_retargeter import (
    DEFAULT_LEFT_POS,
    GR1T2GripperRetargeterCfg,
    GR1T2GripperSteamVRRetargeter,
)


def _q_identity():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _make_snapshot(left_pos):
    """Pose snapshot with both controllers + waist tracker."""
    return {
        "hmd": {"pos": np.zeros(3), "quat": _q_identity()},
        "trackers": {"waist": {"pos": np.zeros(3), "quat": _q_identity()}},
        "controllers": {
            "left":  {"pose": {"pos": np.asarray(left_pos, dtype=np.float64),
                                "quat": _q_identity()},
                      "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
            "right": {"pose": {"pos": np.array([0.0, -0.3, 1.0]),
                                "quat": _q_identity()},
                      "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        },
    }


# ── wrist Z scale ──────────────────────────────────────────────────────


def test_wrist_z_scale_default_amplifies_1p5x():
    """Default ``wrist_pos_scale_per_axis=(1, 1, 1.5)`` makes Z delta
    1.5× the raw controller Z delta."""
    cfg = GR1T2GripperRetargeterCfg(pose_in_il_frame=True)
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    # Zero at (0, 0, 1.0); now at (0, 0, 1.2) → delta_z=+0.2 → scaled +0.30
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]),
                   "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]),
                   "quat": _q_identity()},
    }
    snap = _make_snapshot(left_pos=(0.0, 0.3, 1.2))
    action = rt.retarget(snap)
    left_z = float(action[2])
    expected = DEFAULT_LEFT_POS[2] + 0.2 * 1.5
    assert abs(left_z - expected) < 1e-5, (
        f"Z scale 1.5 expected wrist target Z={expected:.4f}, got {left_z:.4f}"
    )


def test_wrist_xy_unscaled():
    """Default X/Y stay 1.0 — only Z amplified."""
    cfg = GR1T2GripperRetargeterCfg(pose_in_il_frame=True)
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]),
                   "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]),
                   "quat": _q_identity()},
    }
    # +0.1 m forward (IL +X) → robot wrist X +0.1, no amplification
    snap = _make_snapshot(left_pos=(0.1, 0.3, 1.0))
    action = rt.retarget(snap)
    left_x = float(action[0])
    expected = DEFAULT_LEFT_POS[0] + 0.1 * 1.0
    assert abs(left_x - expected) < 1e-5


def test_wrist_z_scale_override():
    """Override via cfg works."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_scale_per_axis=(1.0, 1.0, 2.5),
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]),
                   "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]),
                   "quat": _q_identity()},
    }
    snap = _make_snapshot(left_pos=(0.0, 0.3, 1.2))
    action = rt.retarget(snap)
    expected = DEFAULT_LEFT_POS[2] + 0.2 * 2.5
    assert abs(float(action[2]) - expected) < 1e-5


def test_wrist_unity_scale_disables_amplification():
    """``(1, 1, 1)`` reproduces pre-part10 behaviour."""
    cfg = GR1T2GripperRetargeterCfg(
        pose_in_il_frame=True,
        wrist_pos_scale_per_axis=(1.0, 1.0, 1.0),
    )
    rt = GR1T2GripperSteamVRRetargeter(cfg)
    cfg.controller_pose_zero = {
        "left":  {"pos": np.array([0.0, 0.3, 1.0]),
                   "quat": _q_identity()},
        "right": {"pos": np.array([0.0, -0.3, 1.0]),
                   "quat": _q_identity()},
    }
    snap = _make_snapshot(left_pos=(0.0, 0.3, 1.2))
    action = rt.retarget(snap)
    expected = DEFAULT_LEFT_POS[2] + 0.2
    assert abs(float(action[2]) - expected) < 1e-5


# ── waist scale math (mirrors run_teleop _WAIST_SCALE × clamp) ─────────


_WAIST_LIMITS = {"yaw": (-1.2, 1.2), "pitch": (-1.0, 1.0), "roll": (-0.7, 0.7)}
_WAIST_SCALE = {"yaw": 1.0, "pitch": 2.0, "roll": 1.5}


def _apply_waist_phase_d(yaw_in, pitch_in, roll_in):
    yaw = yaw_in * _WAIST_SCALE["yaw"]
    pitch = pitch_in * _WAIST_SCALE["pitch"]
    roll = roll_in * _WAIST_SCALE["roll"]
    yaw = max(_WAIST_LIMITS["yaw"][0], min(_WAIST_LIMITS["yaw"][1], yaw))
    pitch = max(_WAIST_LIMITS["pitch"][0], min(_WAIST_LIMITS["pitch"][1], pitch))
    roll = max(_WAIST_LIMITS["roll"][0], min(_WAIST_LIMITS["roll"][1], roll))
    return yaw, pitch, roll


def test_waist_yaw_unscaled_pass_through():
    y, p, r = _apply_waist_phase_d(0.5, 0.0, 0.0)
    assert abs(y - 0.5) < 1e-9 and p == 0 and r == 0


def test_waist_pitch_doubled():
    """0.3 rad pelvis pitch → 0.6 rad robot waist pitch."""
    y, p, r = _apply_waist_phase_d(0.0, 0.3, 0.0)
    assert abs(p - 0.6) < 1e-9


def test_waist_pitch_clamps_at_new_limit():
    """Scale 2.0 × 0.6 = 1.2 → clamps to 1.0 (new limit)."""
    y, p, r = _apply_waist_phase_d(0.0, 0.6, 0.0)
    assert abs(p - 1.0) < 1e-9


def test_waist_roll_scaled_1p5x():
    y, p, r = _apply_waist_phase_d(0.0, 0.0, 0.2)
    assert abs(r - 0.3) < 1e-9


def test_waist_pitch_negative_clamps_to_lower_bound():
    """Forward-bend in either sign clamps."""
    y, p, r = _apply_waist_phase_d(0.0, -0.6, 0.0)
    assert abs(p - (-1.0)) < 1e-9


# ── head actuator presence (declarative; no Isaac Sim required) ────────


def test_head_actuator_added_to_env_cfg_articulation():
    """The env_cfg's _gripper_robot_articulation() must include a
    ``head`` actuator entry (added in part 10).  Without this entry the
    GR1T2 head joints have NO PD and Phase D's joint targets do nothing.

    This test imports the env_cfg module purely to inspect the source —
    it does not construct the cfg (which would require Isaac Sim).
    """
    import pathlib
    env_cfg_path = pathlib.Path(__file__).resolve().parents[1] / (
        "kitchen_sorting_gr1t2_gripper_env_cfg.py"
    )
    src = env_cfg_path.read_text(encoding="utf-8")
    assert 'actuators["head"]' in src, (
        "expected actuators[\"head\"] entry in _gripper_robot_articulation "
        "(part 10 head-actuator restoration)"
    )
    assert 'head_.*' in src, (
        "expected head joint regex 'head_.*' in head actuator joint_names_expr"
    )
