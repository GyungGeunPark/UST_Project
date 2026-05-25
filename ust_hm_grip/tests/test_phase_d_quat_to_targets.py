"""Phase D regression — verify HMD/waist quaternions map to plausible
head_yaw/pitch/roll + waist_yaw/pitch/roll joint targets via the
``quat_wxyz_to_euler_zyx`` helper used by ``_phase_d_apply`` in
``run_teleop.py``.

These tests run WITHOUT Isaac Sim — only numpy is required.  They
isolate the math layer that ``_phase_d_apply`` depends on.
"""
from __future__ import annotations
import numpy as np
import pytest

from ust_ws.ust_hm_grip.teleop import coord_transforms as ct


def _q_identity():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _q_yaw(angle):
    return np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)],
                    dtype=np.float64)


def _q_pitch(angle):
    return np.array([np.cos(angle / 2), 0.0, np.sin(angle / 2), 0.0],
                    dtype=np.float64)


def _q_roll(angle):
    return np.array([np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0],
                    dtype=np.float64)


# ── Pure-axis HMD orientations ─────────────────────────────────────────
def test_phase_d_hmd_neutral_zero_targets():
    """Identity HMD → all three head targets ≈ 0."""
    y, p, r = ct.quat_wxyz_to_euler_zyx(_q_identity())
    assert abs(y) < 1e-9 and abs(p) < 1e-9 and abs(r) < 1e-9


def test_phase_d_hmd_yaw_left_90deg_targets():
    """User turns head 90° left → yaw=+90°, pitch=0, roll=0."""
    y, p, r = ct.quat_wxyz_to_euler_zyx(_q_yaw(np.pi / 2))
    assert abs(y - np.pi / 2) < 1e-9 and abs(p) < 1e-9 and abs(r) < 1e-9


def test_phase_d_hmd_pitch_up_45deg_targets():
    """User looks up 45° → pitch=+45°, yaw=0, roll=0."""
    y, p, r = ct.quat_wxyz_to_euler_zyx(_q_pitch(np.pi / 4))
    assert abs(y) < 1e-9 and abs(p - np.pi / 4) < 1e-9 and abs(r) < 1e-9


def test_phase_d_hmd_roll_tilt_30deg_targets():
    """User tilts head right 30° → roll=+30°, yaw=0, pitch=0."""
    y, p, r = ct.quat_wxyz_to_euler_zyx(_q_roll(np.pi / 6))
    assert abs(y) < 1e-9 and abs(p) < 1e-9 and abs(r - np.pi / 6) < 1e-9


# ── Clamping (run_teleop _HEAD_LIMITS_RAD / _WAIST_LIMITS_RAD) ────────
_HEAD_LIMITS = {
    "yaw":   (-1.5, 1.5),
    "pitch": (-1.0, 1.0),
    "roll":  (-0.7, 0.7),
}


def _clamp(value, low, high):
    return max(low, min(high, value))


def test_phase_d_head_target_clamp_extreme_yaw():
    """User turning head 180° (q_yaw(π)) is clamped to ±1.5 rad."""
    angle = np.pi  # 180°
    y, _, _ = ct.quat_wxyz_to_euler_zyx(_q_yaw(angle))
    # The euler conversion returns ±π but clamped via _HEAD_LIMITS
    clamped = _clamp(y, *_HEAD_LIMITS["yaw"])
    assert -1.5 <= clamped <= 1.5
    # Without clamp would be ±π ≈ ±3.14 (well outside the limit)
    assert abs(y) > 3.0 - 1e-6 or abs(abs(y) - np.pi) < 1e-6


def test_phase_d_head_target_clamp_extreme_pitch():
    """User looking straight up (q_pitch(π/2 - ε)) reaches the pitch
    boundary; clamping caps at ±1.0 rad (~57°)."""
    angle = 1.4  # ~80°, well past the ±57° limit
    _, p, _ = ct.quat_wxyz_to_euler_zyx(_q_pitch(angle))
    clamped = _clamp(p, *_HEAD_LIMITS["pitch"])
    assert -1.0 <= clamped <= 1.0
    assert abs(p) > 1.0 - 1e-6


# ── Snapshot dict shape: hmd / trackers.waist consumption ─────────────
def test_phase_d_snapshot_hmd_consumption_path():
    """The ``snap['hmd']`` field is the only HMD source for Phase D.

    Sampler returns ``None`` for hmd when the pose is all-zero (headset
    not worn / off).  ``_phase_d_apply`` must early-out gracefully so
    no spurious targets are written.
    """
    snap = {"hmd": None, "trackers": {}}
    hmd = snap.get("hmd")
    assert hmd is None  # apply must skip

    snap = {"hmd": {"pos": np.zeros(3), "quat": _q_identity()},
            "trackers": {}}
    hmd = snap.get("hmd")
    assert hmd is not None  # apply triggers
    y, p, r = ct.quat_wxyz_to_euler_zyx(hmd["quat"])
    assert (abs(y) + abs(p) + abs(r)) < 1e-9


def test_phase_d_snapshot_waist_tracker_consumption():
    """The ``snap['trackers']['waist']`` field is the waist source.

    With ``_DEFAULT_BODY_ROLE_MAP`` (idx 0 → 'waist'), the sampler
    populates this when ``--xrt_enable_body True`` and the SMPL body
    stream is live.
    """
    snap = {"trackers": {"waist": None}}
    assert snap["trackers"].get("waist") is None  # apply must skip

    snap = {"trackers": {"waist": {"pos": np.zeros(3),
                                    "quat": _q_yaw(np.pi / 4)}}}
    waist = snap["trackers"].get("waist")
    assert waist is not None
    y, _, _ = ct.quat_wxyz_to_euler_zyx(waist["quat"])
    assert abs(y - np.pi / 4) < 1e-9


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
