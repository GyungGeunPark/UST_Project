"""Waist 3-DoF estimator for GR1T2 WaistEnabled posture control.

In Isaac Lab's ``PickPlaceGR1T2WaistEnabledEnvCfg`` the waist joints
``waist_yaw_joint``, ``waist_pitch_joint``, ``waist_roll_joint`` are
appended to ``pink_controlled_joint_names`` so the Pink IK solver uses
them as null-space redundancy when the palm-frame wrist targets exceed
the arm-only reachable volume.  The 36D action tensor does **not** carry
waist channels; instead the waist joints track the
``NullSpacePostureTask`` posture, which defaults to the joint's rest
position.

When a user wants *explicit* waist control (e.g. "twist the torso to the
right") this estimator derives a ``(yaw, pitch, roll)`` tuple from VR
inputs:

1. ``source="hips_tracker"`` (recommended) — uses the ``waist`` tracker
   provided by Virtual Desktop's Full Body emulation.  The first frame
   after ``reset()`` becomes the neutral orientation; subsequent frames
   report the **relative** rotation so the user's starting stance is
   zeroed.
2. ``source="hmd_yaw"`` — fallback when no hips tracker is available.
   Only yaw is recovered (head-rotation-as-torso-rotation), with
   configurable gain.

Downstream use: the env_cfg reads the estimate and overrides the
``NullSpacePostureTask`` posture targets for the three waist joints so
Pink IK is nudged toward the user's actual torso orientation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np


__all__ = ["WaistEstimator", "WaistEstimate"]


def _quat_wxyz_conjugate(q: Sequence[float]) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_multiply(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def _quat_to_yaw_pitch_roll(q: Sequence[float]) -> Tuple[float, float, float]:
    """Convert a ``(wxyz)`` quaternion to Tait-Bryan ``yaw, pitch, roll``.

    Convention — rotation about Z (yaw), then Y (pitch), then X (roll)
    in the world frame (aircraft / ROS REP-103 sequence).
    """
    w, x, y, z = q
    # Yaw (Z)
    t0 = 2.0 * (w * z + x * y)
    t1 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)
    # Pitch (Y)
    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    # Roll (X)
    t3 = 2.0 * (w * x + y * z)
    t4 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t3, t4)
    return yaw, pitch, roll


@dataclass
class WaistEstimate:
    """Container for the three waist targets (radians)."""
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.yaw, self.pitch, self.roll)

    def as_array(self) -> np.ndarray:
        return np.array([self.yaw, self.pitch, self.roll], dtype=np.float32)


class WaistEstimator:
    """Derive ``(yaw, pitch, roll)`` for the GR1T2 waist joints.

    Parameters
    ----------
    source:
        ``"hips_tracker"`` → use ``snapshot["trackers"]["waist"]["quat"]``.
        ``"hmd_yaw"``      → use ``snapshot["hmd"]["quat"]`` for yaw only.
        ``"off"``          → always return zeros.
    gain:
        Per-axis output scale ``(yaw_gain, pitch_gain, roll_gain)`` applied
        after the relative rotation is computed.  Yaw at 1.0 gives 1:1
        tracking of torso rotation; pitch/roll at 0.5/0.3 reduce the
        smaller-range URDF joints so the user can comfortably reach their
        limits.
    clamp:
        ``(yaw_limit, pitch_limit, roll_limit)`` in radians.  The default
        matches the GR1T2 URDF range (yaw ≈ ±1.0, pitch ≈ [-0.3, 0.5],
        roll ≈ ±0.4).
    low_pass_alpha:
        Exponential smoothing factor in ``[0, 1]``.  Higher = less filtering.
        The estimator is stateless except for this single-pole filter.
    """

    def __init__(
        self,
        source: str = "hips_tracker",
        gain: Sequence[float] = (1.0, 0.5, 0.3),
        clamp: Sequence[Sequence[float]] = ((-1.0, 1.0), (-0.3, 0.5), (-0.4, 0.4)),
        low_pass_alpha: float = 0.3,
        zero_cal_frames: int = 30,
        deadband_rad: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        if source not in ("hips_tracker", "hmd_yaw", "off"):
            raise ValueError(f"unknown waist estimator source: {source!r}")
        self.source = source
        gain_arr = np.asarray(gain, dtype=np.float64).reshape(-1)
        if gain_arr.size != 3:
            raise ValueError(f"gain must be length 3, got {gain_arr.size}")
        self.gain = gain_arr
        self.clamp = [(float(lo), float(hi)) for lo, hi in clamp]
        if len(self.clamp) != 3:
            raise ValueError("clamp must have 3 entries (yaw, pitch, roll)")
        self.alpha = float(max(0.0, min(1.0, low_pass_alpha)))

        # 9.14 — averaged zero-calibration window.  The 9.13 implementation
        # captured the zero quat from a SINGLE frame, which is unreliable
        # when Virtual Desktop's AI-inferred hips tracker has noise / mid-
        # initialisation artefacts on the first sample.  Now we accumulate
        # over ``zero_cal_frames`` samples and use the averaged quaternion
        # as the rest pose.  At 20 Hz teleop, ~30 frames = 1.5 s of standing
        # still — long enough for the AI to converge but short enough not to
        # delay teleop start.
        self.zero_cal_frames = max(1, int(zero_cal_frames))
        # 9.15 — per-axis deadband applied to (yaw, pitch, roll) AFTER
        # rest subtraction.  Absorbs Virtual Desktop AI body-tracker
        # noise: observed waist_pitch raw range up to 111° even with
        # user standing perfectly still.  A pitch deadband of e.g. 0.3
        # rad (17°) zeros out anything below that, preventing the
        # robot from auto-bending due to hips quat noise alone.
        # Outputs above the deadband are mapped through a piecewise
        # linear curve (no jump at the deadband boundary).
        deadband_arr = np.asarray(deadband_rad, dtype=np.float64).reshape(-1)
        if deadband_arr.size != 3:
            raise ValueError(f"deadband_rad must be length 3, got {deadband_arr.size}")
        self.deadband = deadband_arr
        self._zero_accum: Optional[np.ndarray] = None
        self._zero_count: int = 0
        self._zero_quat: Optional[np.ndarray] = None
        self._last: WaistEstimate = WaistEstimate()

    # ── public API ─────────────────────────────────────────────────────
    def reset(self) -> None:
        """Forget the neutral pose so the next ``estimate()`` re-zeroes."""
        self._zero_quat = None
        self._zero_accum = None
        self._zero_count = 0
        self._last = WaistEstimate()

    def estimate(self, snapshot: Mapping[str, Any]) -> WaistEstimate:
        """Compute the waist target from the latest SteamVR snapshot.

        When the requested source is missing, returns the last estimate
        (so the target doesn't snap to zero mid-motion).  The caller can
        always call :meth:`reset` before a demo or on episode boundary.
        """
        if self.source == "off":
            self._last = WaistEstimate()
            return self._last

        quat = self._extract_quat(snapshot)
        if quat is None:
            return self._last

        # 9.14 — averaged zero-calibration: accumulate first N quats then
        # finalize the rest pose.  Returns zero-vector during the window so
        # the robot doesn't twitch from initial noise.
        if self._zero_quat is None:
            quat = quat.astype(np.float64).reshape(-1)[:4]
            if self._zero_accum is None:
                self._zero_accum = quat.copy()
                self._zero_count = 1
            else:
                # Hemisphere consistency before accumulating.
                if float(np.dot(self._zero_accum, quat)) < 0.0:
                    quat = -quat
                self._zero_accum = self._zero_accum + quat
                self._zero_count += 1
            if self._zero_count >= self.zero_cal_frames:
                avg = self._zero_accum / float(self._zero_count)
                norm = float(np.linalg.norm(avg))
                if norm > 1e-9:
                    self._zero_quat = avg / norm
                else:
                    self._zero_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            return self._last

        q_rel = _quat_multiply(quat, _quat_wxyz_conjugate(self._zero_quat))
        # Make sure quaternion hemisphere stays consistent (prefers the
        # short arc from current to zero).
        if q_rel[0] < 0.0:
            q_rel = -q_rel
        yaw, pitch, roll = _quat_to_yaw_pitch_roll(q_rel)

        # 9.15 — apply per-axis deadband BEFORE gain.  Subtract the
        # deadband from the absolute value, clamp negative to 0,
        # restore sign.  This produces a smooth ramp starting at the
        # deadband boundary instead of a discontinuous jump.
        def _apply_db(v: float, db: float) -> float:
            if db <= 0.0:
                return v
            sign = 1.0 if v >= 0.0 else -1.0
            mag = max(0.0, abs(v) - db)
            return sign * mag

        yaw = _apply_db(float(yaw), float(self.deadband[0]))
        pitch = _apply_db(float(pitch), float(self.deadband[1]))
        roll = _apply_db(float(roll), float(self.deadband[2]))

        yaw = yaw * float(self.gain[0])
        pitch = pitch * float(self.gain[1])
        roll = roll * float(self.gain[2])

        # Clamp to URDF-safe range.
        yaw = max(self.clamp[0][0], min(self.clamp[0][1], yaw))
        pitch = max(self.clamp[1][0], min(self.clamp[1][1], pitch))
        roll = max(self.clamp[2][0], min(self.clamp[2][1], roll))

        # Low-pass filter.
        a = self.alpha
        yaw = a * yaw + (1.0 - a) * self._last.yaw
        pitch = a * pitch + (1.0 - a) * self._last.pitch
        roll = a * roll + (1.0 - a) * self._last.roll

        self._last = WaistEstimate(yaw=yaw, pitch=pitch, roll=roll)
        return self._last

    # ── helpers ────────────────────────────────────────────────────────
    def _extract_quat(self, snapshot: Mapping[str, Any]) -> Optional[np.ndarray]:
        if self.source == "hips_tracker":
            trackers = snapshot.get("trackers") or {}
            waist = trackers.get("waist")
            if not waist:
                return None
            quat = waist.get("quat")
            if quat is None:
                return None
            return np.asarray(quat, dtype=np.float64).reshape(-1)[:4]

        if self.source == "hmd_yaw":
            hmd = snapshot.get("hmd")
            if not hmd:
                return None
            quat = hmd.get("quat")
            if quat is None:
                return None
            q = np.asarray(quat, dtype=np.float64).reshape(-1)[:4]
            # Strip pitch/roll by projecting onto Z-rotation only.
            yaw, _, _ = _quat_to_yaw_pitch_roll(q)
            half = 0.5 * yaw
            return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)

        return None
