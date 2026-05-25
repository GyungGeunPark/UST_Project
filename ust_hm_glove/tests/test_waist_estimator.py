"""Unit tests for ``ust_hm_glove.teleop.waist_estimator``."""

from __future__ import annotations

import math

import numpy as np
import pytest

from ust_ws.ust_hm_glove.teleop.waist_estimator import (
    WaistEstimate,
    WaistEstimator,
)


def _yaw_quat(deg: float) -> np.ndarray:
    half = math.radians(deg) / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def _pitch_quat(deg: float) -> np.ndarray:
    half = math.radians(deg) / 2.0
    return np.array([math.cos(half), 0.0, math.sin(half), 0.0], dtype=np.float64)


def _hips_snap(quat: np.ndarray, pos=(0.0, 0.0, 1.0)) -> dict:
    return {
        "trackers": {
            "waist": {"pos": np.asarray(pos, dtype=np.float64), "quat": quat},
        }
    }


class TestSources:
    def test_unknown_source_rejected(self):
        with pytest.raises(ValueError):
            WaistEstimator(source="unknown")

    def test_off_source_always_zero(self):
        est = WaistEstimator(source="off")
        out = est.estimate(_hips_snap(_yaw_quat(45.0)))
        assert out.yaw == 0.0 and out.pitch == 0.0 and out.roll == 0.0

    def test_hips_tracker_neutral_first_frame(self):
        est = WaistEstimator(source="hips_tracker", low_pass_alpha=1.0)
        first = est.estimate(_hips_snap(_yaw_quat(0.0)))
        assert first.yaw == 0.0

    def test_hips_tracker_yaw_delta(self):
        est = WaistEstimator(source="hips_tracker", low_pass_alpha=1.0, zero_cal_frames=1)
        est.estimate(_hips_snap(_yaw_quat(0.0)))
        out = est.estimate(_hips_snap(_yaw_quat(45.0)))
        assert abs(out.yaw - math.radians(45.0)) < 1e-3
        assert abs(out.pitch) < 1e-3
        assert abs(out.roll) < 1e-3

    def test_hmd_yaw_source(self):
        est = WaistEstimator(source="hmd_yaw", low_pass_alpha=1.0, zero_cal_frames=1)
        snap0 = {"hmd": {"pos": np.zeros(3), "quat": _yaw_quat(0.0)}}
        snap1 = {"hmd": {"pos": np.zeros(3), "quat": _yaw_quat(30.0)}}
        est.estimate(snap0)
        out = est.estimate(snap1)
        assert abs(out.yaw - math.radians(30.0)) < 2e-3


class TestGainAndClamp:
    def test_gain_scales_yaw(self):
        est = WaistEstimator(gain=(0.5, 1.0, 1.0), low_pass_alpha=1.0, zero_cal_frames=1)
        est.estimate(_hips_snap(_yaw_quat(0.0)))
        out = est.estimate(_hips_snap(_yaw_quat(60.0)))
        assert abs(out.yaw - 0.5 * math.radians(60.0)) < 5e-3

    def test_clamp_yaw(self):
        # Default yaw clamp is ±1.0 rad (~57°).
        est = WaistEstimator(gain=(2.0, 1.0, 1.0), low_pass_alpha=1.0, zero_cal_frames=1)
        est.estimate(_hips_snap(_yaw_quat(0.0)))
        out = est.estimate(_hips_snap(_yaw_quat(179.0)))  # gain=2 → ≥3 rad before clamp
        assert abs(out.yaw) <= 1.0 + 1e-6

    def test_custom_clamp(self):
        est = WaistEstimator(
            clamp=((-0.2, 0.2), (-0.1, 0.1), (-0.1, 0.1)),
            low_pass_alpha=1.0,
            zero_cal_frames=1,
        )
        est.estimate(_hips_snap(_yaw_quat(0.0)))
        out = est.estimate(_hips_snap(_yaw_quat(60.0)))
        assert out.yaw == 0.2  # clamped


class TestLowPassAndReset:
    def test_low_pass_smooths(self):
        est = WaistEstimator(low_pass_alpha=0.1, zero_cal_frames=1)
        est.estimate(_hips_snap(_yaw_quat(0.0)))
        out = est.estimate(_hips_snap(_yaw_quat(60.0)))
        assert out.yaw < math.radians(60.0)

    def test_reset_reinitialises(self):
        est = WaistEstimator(low_pass_alpha=1.0, zero_cal_frames=1)
        est.estimate(_hips_snap(_yaw_quat(0.0)))
        out1 = est.estimate(_hips_snap(_yaw_quat(45.0)))
        assert out1.yaw > 0.5
        est.reset()
        out2 = est.estimate(_hips_snap(_yaw_quat(45.0)))
        # After reset, the first estimate re-zeros.
        assert out2.yaw == 0.0


class TestDeadband:
    """9.15: per-axis deadband for absorbing AI body-tracker noise."""

    def test_default_deadband_is_zero_yaw_zero_roll_pitch_only(self):
        est = WaistEstimator()
        assert tuple(est.deadband.tolist()) == (0.0, 0.0, 0.0)

    def test_pitch_deadband_absorbs_small_motion(self):
        est = WaistEstimator(
            zero_cal_frames=1, low_pass_alpha=1.0,
            deadband_rad=(0.0, 0.3, 0.0),
        )
        est.estimate(_hips_snap(_pitch_quat(0.0)))
        # 10° pitch is below 17° deadband → should output 0.
        out = est.estimate(_hips_snap(_pitch_quat(10.0)))
        assert abs(out.pitch) < 1e-3, out.pitch

    def test_pitch_above_deadband_ramps(self):
        est = WaistEstimator(
            zero_cal_frames=1, low_pass_alpha=1.0,
            gain=(1.0, 1.0, 1.0),
            deadband_rad=(0.0, math.radians(5.0), 0.0),  # 5° deadband
        )
        est.estimate(_hips_snap(_pitch_quat(0.0)))
        # 30° pitch → after subtracting 5° deadband → 25° remaining.
        out = est.estimate(_hips_snap(_pitch_quat(30.0)))
        assert abs(out.pitch - math.radians(25.0)) < 0.01

    def test_yaw_unaffected_when_only_pitch_deadband(self):
        est = WaistEstimator(
            zero_cal_frames=1, low_pass_alpha=1.0,
            deadband_rad=(0.0, 0.3, 0.0),
        )
        est.estimate(_hips_snap(_yaw_quat(0.0)))
        # 30° yaw should pass through (no yaw deadband).
        out = est.estimate(_hips_snap(_yaw_quat(30.0)))
        assert abs(out.yaw - math.radians(30.0)) < 0.01


class TestAveragedZeroCal:
    """9.14: averaged zero-calibration over the first N frames."""

    def test_default_frames_is_30(self):
        est = WaistEstimator()
        assert est.zero_cal_frames == 30

    def test_first_n_frames_return_zero(self):
        est = WaistEstimator(low_pass_alpha=1.0, zero_cal_frames=5)
        # Send 5 frames at 30° yaw — all should return zero (calibration).
        for _ in range(5):
            out = est.estimate(_hips_snap(_yaw_quat(30.0)))
            assert out.yaw == 0.0
        # The 6th frame should reflect the relative motion (which, given
        # all 5 cal frames were 30°, is zero — user is "still" at 30°).
        out = est.estimate(_hips_snap(_yaw_quat(30.0)))
        assert abs(out.yaw) < 1e-3

    def test_averaging_smooths_initial_noise(self):
        """The first sample being noisy doesn't drag the rest pose."""
        est = WaistEstimator(low_pass_alpha=1.0, zero_cal_frames=10)
        # Frame 1: noisy 20° yaw outlier; frames 2-10: clean 0° yaw.
        est.estimate(_hips_snap(_yaw_quat(20.0)))
        for _ in range(9):
            est.estimate(_hips_snap(_yaw_quat(0.0)))
        # Now ask for the relative motion at 0° — averaged rest is closer
        # to 0° than to 20°, so the relative reading at 0° should be small.
        out = est.estimate(_hips_snap(_yaw_quat(0.0)))
        # 9 frames of 0 + 1 of 20 → averaged rest ≈ 2°.  Relative at 0° is
        # ≈ -2° = -0.035 rad — well within ±0.1 rad.
        assert abs(out.yaw) < 0.1


class TestDataclass:
    def test_as_tuple_and_array(self):
        est = WaistEstimate(yaw=0.5, pitch=-0.1, roll=0.2)
        assert est.as_tuple() == (0.5, -0.1, 0.2)
        arr = est.as_array()
        assert arr.shape == (3,)
        assert arr.dtype == np.float32
