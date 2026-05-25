"""Unit tests for ``ust_hm_glove.teleop.head_estimator``."""

from __future__ import annotations

import math

import numpy as np

from ust_ws.ust_hm_glove.teleop.head_estimator import (
    HeadEstimate,
    HeadEstimator,
)


def _yaw_quat(deg: float) -> np.ndarray:
    half = math.radians(deg) / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def _pitch_quat(deg: float) -> np.ndarray:
    half = math.radians(deg) / 2.0
    return np.array([math.cos(half), 0.0, math.sin(half), 0.0], dtype=np.float64)


def _hmd_snap(quat: np.ndarray) -> dict:
    return {"hmd": {"pos": np.zeros(3), "quat": quat}}


class TestDataclass:
    def test_as_tuple_and_array(self):
        est = HeadEstimate(yaw=0.5, pitch=-0.1, roll=0.2)
        assert est.as_tuple() == (0.5, -0.1, 0.2)
        arr = est.as_array()
        assert arr.shape == (3,)
        assert arr.dtype == np.float32


class TestZeroCal:
    def test_default_zero_cal_frames_is_15(self):
        est = HeadEstimator()
        assert est.zero_cal_frames == 15

    def test_first_n_frames_return_zero(self):
        est = HeadEstimator(zero_cal_frames=3, low_pass_alpha=1.0)
        for _ in range(3):
            out = est.estimate(_hmd_snap(_yaw_quat(45.0)))
            assert out.yaw == 0.0
        # 4th sample at the same orientation -> zero (rest captured).
        out = est.estimate(_hmd_snap(_yaw_quat(45.0)))
        assert abs(out.yaw) < 1e-3

    def test_relative_motion_after_cal(self):
        est = HeadEstimator(
            zero_cal_frames=1, low_pass_alpha=1.0,
            gain=(1.0, 1.0, 1.0),
        )
        est.estimate(_hmd_snap(_yaw_quat(0.0)))
        out = est.estimate(_hmd_snap(_yaw_quat(30.0)))
        assert abs(out.yaw - math.radians(30.0)) < 1e-3


class TestNoHMD:
    def test_missing_hmd_returns_last(self):
        est = HeadEstimator(zero_cal_frames=1, low_pass_alpha=1.0)
        # No "hmd" key — should not crash, returns previous (zero) estimate.
        out = est.estimate({})
        assert out.yaw == 0.0 and out.pitch == 0.0 and out.roll == 0.0

    def test_missing_quat_returns_last(self):
        est = HeadEstimator(zero_cal_frames=1, low_pass_alpha=1.0)
        out = est.estimate({"hmd": {"pos": np.zeros(3)}})
        assert out.yaw == 0.0


class TestClampAndDeadband:
    def test_clamp_yaw(self):
        est = HeadEstimator(
            zero_cal_frames=1, low_pass_alpha=1.0,
            gain=(2.0, 1.0, 1.0),
            clamp=((-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)),
        )
        est.estimate(_hmd_snap(_yaw_quat(0.0)))
        out = est.estimate(_hmd_snap(_yaw_quat(60.0)))
        assert abs(out.yaw) <= 0.5 + 1e-6

    def test_pitch_deadband(self):
        est = HeadEstimator(
            zero_cal_frames=1, low_pass_alpha=1.0,
            gain=(1.0, 1.0, 1.0),
            deadband_rad=(0.0, 0.3, 0.0),
        )
        est.estimate(_hmd_snap(_pitch_quat(0.0)))
        out = est.estimate(_hmd_snap(_pitch_quat(10.0)))  # below 17° deadband
        assert abs(out.pitch) < 1e-3


class TestReset:
    def test_reset_reinitialises_zero(self):
        est = HeadEstimator(zero_cal_frames=1, low_pass_alpha=1.0)
        est.estimate(_hmd_snap(_yaw_quat(0.0)))
        out1 = est.estimate(_hmd_snap(_yaw_quat(45.0)))
        assert out1.yaw > 0.5
        est.reset()
        out2 = est.estimate(_hmd_snap(_yaw_quat(45.0)))
        # First post-reset call captures rest -> returns zero.
        assert out2.yaw == 0.0
