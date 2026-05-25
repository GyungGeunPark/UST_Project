"""HeadEstimator — HMD orientation → GR1T2 head joint targets.

The robot's head links form a yaw-roll-pitch chain
(``head_yaw_link`` → ``head_roll_link`` → ``head_pitch_link``).  When the
user moves their head, we want the robot to mirror that motion so the
camera mounted on the robot head sees what the user expects.

This mirrors :class:`WaistEstimator` in structure (zero-calibration over
N frames, per-axis gain, clamp, low-pass, deadband) but reads the
quaternion from ``snapshot["hmd"]["quat"]`` instead of the hips tracker.
The output is a 3-tuple ``(yaw, pitch, roll)`` ready to be fed to
the robot's head joint position targets.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np


__all__ = ["HeadEstimator", "HeadEstimate"]


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
    """``(wxyz)`` → Tait-Bryan ``yaw, pitch, roll`` (Z, Y, X)."""
    w, x, y, z = q
    t0 = 2.0 * (w * z + x * y)
    t1 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)
    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    t3 = 2.0 * (w * x + y * z)
    t4 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t3, t4)
    return yaw, pitch, roll


@dataclass
class HeadEstimate:
    """Head joint targets (radians).  Order: yaw, pitch, roll."""
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.yaw, self.pitch, self.roll)

    def as_array(self) -> np.ndarray:
        return np.array([self.yaw, self.pitch, self.roll], dtype=np.float32)


class HeadEstimator:
    """HMD orientation → GR1T2 ``(head_yaw, head_pitch, head_roll)`` joint
    targets in radians.

    Parameters
    ----------
    gain:
        Per-axis output scale.  Default ``(1.0, 0.7, 0.5)`` — yaw 1:1
        (looking left/right tracks naturally), pitch attenuated 0.7
        (head_pitch URDF range is smaller), roll 0.5 (heads rarely roll).
    clamp:
        ``(yaw, pitch, roll)`` joint limits — set to typical GR1T2
        head URDF ranges.  Override if your USD differs.
    low_pass_alpha:
        Single-pole filter ``[0, 1]``.  Higher = more responsive,
        lower = more damping.  0.4 default for moderate smoothing
        (head moves are fast and short, so we don't damp too hard).
    zero_cal_frames:
        Average the first N frames as the rest pose.  Defaults to 15
        (~0.75 s) — shorter than waist's 30 because the user typically
        looks straight at the screen at startup, so a single sample is
        nearly correct already.
    deadband_rad:
        Per-axis deadband below which the output is forced to 0.
        Default 0 — head tracking should be responsive even at small
        motion (unlike waist, where we damp AI body-tracker noise).
    """

    def __init__(
        self,
        gain: Sequence[float] = (1.0, 0.7, 0.5),
        clamp: Sequence[Sequence[float]] = (
            (-1.57, 1.57),  # yaw  ~±90°
            (-0.5, 0.5),    # pitch ±29°
            (-0.5, 0.5),    # roll ±29°
        ),
        low_pass_alpha: float = 0.4,
        zero_cal_frames: int = 15,
        deadband_rad: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        gain_arr = np.asarray(gain, dtype=np.float64).reshape(-1)
        if gain_arr.size != 3:
            raise ValueError(f"gain must be length 3, got {gain_arr.size}")
        self.gain = gain_arr
        self.clamp = [(float(lo), float(hi)) for lo, hi in clamp]
        if len(self.clamp) != 3:
            raise ValueError("clamp must have 3 entries (yaw, pitch, roll)")
        self.alpha = float(max(0.0, min(1.0, low_pass_alpha)))
        self.zero_cal_frames = max(1, int(zero_cal_frames))
        deadband_arr = np.asarray(deadband_rad, dtype=np.float64).reshape(-1)
        if deadband_arr.size != 3:
            raise ValueError(f"deadband_rad must be length 3, got {deadband_arr.size}")
        self.deadband = deadband_arr

        self._zero_accum: Optional[np.ndarray] = None
        self._zero_count: int = 0
        self._zero_quat: Optional[np.ndarray] = None
        self._last: HeadEstimate = HeadEstimate()

    def reset(self) -> None:
        self._zero_quat = None
        self._zero_accum = None
        self._zero_count = 0
        self._last = HeadEstimate()

    def estimate(self, snapshot: Mapping[str, Any]) -> HeadEstimate:
        hmd = snapshot.get("hmd")
        if not hmd:
            return self._last
        quat = hmd.get("quat")
        if quat is None:
            return self._last
        quat = np.asarray(quat, dtype=np.float64).reshape(-1)[:4]

        # Averaged zero-calibration window.
        if self._zero_quat is None:
            if self._zero_accum is None:
                self._zero_accum = quat.copy()
                self._zero_count = 1
            else:
                # Hemisphere consistency.
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
        if q_rel[0] < 0.0:
            q_rel = -q_rel
        yaw, pitch, roll = _quat_to_yaw_pitch_roll(q_rel)

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

        yaw = max(self.clamp[0][0], min(self.clamp[0][1], yaw))
        pitch = max(self.clamp[1][0], min(self.clamp[1][1], pitch))
        roll = max(self.clamp[2][0], min(self.clamp[2][1], roll))

        a = self.alpha
        yaw = a * yaw + (1.0 - a) * self._last.yaw
        pitch = a * pitch + (1.0 - a) * self._last.pitch
        roll = a * roll + (1.0 - a) * self._last.roll

        self._last = HeadEstimate(yaw=yaw, pitch=pitch, roll=roll)
        return self._last
