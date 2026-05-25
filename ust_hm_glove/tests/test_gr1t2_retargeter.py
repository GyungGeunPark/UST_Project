"""Unit tests for ``ust_hm_glove.teleop.gr1t2_retargeter``."""

from __future__ import annotations

import math

import numpy as np
import pytest

from ust_ws.ust_hm_glove.teleop.gr1t2_retargeter import (
    ACTION_DIM,
    DEFAULT_LEFT_POS,
    DEFAULT_LEFT_QUAT,
    DEFAULT_RIGHT_POS,
    DEFAULT_RIGHT_QUAT,
    GR1T2FourierRetargeterCfg,
    GR1T2FourierSteamVRRetargeter,
    HAND_DIM_PER_SIDE,
    HAND_DIM_TOTAL,
)


def _empty_snap() -> dict:
    return {
        "timestamp": 0.0,
        "hmd": None,
        "trackers": {},
        "hands": {"left": None, "right": None},
        "controllers": {"left": None, "right": None},
        "frame_count": 1,
    }


def _forearm_pose(x: float, y: float, z: float) -> dict:
    return {
        "pos": np.array([x, y, z], dtype=np.float64),
        "quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    }


class TestShape:
    def test_action_dim_constants(self):
        assert ACTION_DIM == 36
        assert HAND_DIM_PER_SIDE == 11
        assert HAND_DIM_TOTAL == 22

    def test_default_output_shape(self):
        import torch

        r = GR1T2FourierSteamVRRetargeter(debug=False)
        out = r.retarget(_empty_snap())
        assert out.shape == (ACTION_DIM,)
        assert out.dtype == torch.float32


class TestIdleFallback:
    def test_idle_positions_match_defaults(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        out = r.retarget(_empty_snap())
        assert abs(float(out[0]) - DEFAULT_LEFT_POS[0]) < 1e-4
        assert abs(float(out[1]) - DEFAULT_LEFT_POS[1]) < 1e-4
        assert abs(float(out[2]) - DEFAULT_LEFT_POS[2]) < 1e-4
        assert abs(float(out[7]) - DEFAULT_RIGHT_POS[0]) < 1e-4
        assert abs(float(out[9]) - DEFAULT_RIGHT_POS[2]) < 1e-4

    def test_custom_idle_respected(self):
        cfg = GR1T2FourierRetargeterCfg(
            idle_left_pos=(1.0, 2.0, 3.0),
            idle_right_pos=(-1.0, -2.0, -3.0),
        )
        r = GR1T2FourierSteamVRRetargeter(cfg)
        out = r.retarget(_empty_snap())
        assert abs(float(out[0]) - 1.0) < 1e-5
        assert abs(float(out[7]) - (-1.0)) < 1e-5

    def test_idle_hand_zero(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        out = r.retarget(_empty_snap())
        assert float(out[14:36].abs().max().item()) < 1e-6


class TestSourcePriority:
    def test_forearm_preferred_over_idle(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        snap = _empty_snap()
        snap["trackers"]["left_forearm"] = _forearm_pose(0.0, 1.2, 0.4)
        snap["trackers"]["right_forearm"] = _forearm_pose(0.0, 1.2, -0.4)
        r.retarget(snap)
        info = r.get_source_info()
        assert info["left_eef"] == "forearm"
        assert info["right_eef"] == "forearm"

    def test_controller_fallback(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        snap = _empty_snap()
        snap["controllers"]["left"] = {
            "pose": {"pos": np.array([0.1, 1.0, 0.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "buttons": {},
        }
        snap["controllers"]["right"] = {
            "pose": {"pos": np.array([-0.1, 1.0, 0.3]), "quat": np.array([1.0, 0.0, 0.0, 0.0])},
            "buttons": {},
        }
        r.retarget(snap)
        info = r.get_source_info()
        assert info["left_eef"] == "controller"
        assert info["right_eef"] == "controller"

    def test_default_when_no_sources(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        r.retarget(_empty_snap())
        info = r.get_source_info()
        assert info["left_eef"] == "default"
        assert info["right_eef"] == "default"


class TestOrientation:
    def test_right_wrist_z180_changes_quat(self):
        r_on = GR1T2FourierSteamVRRetargeter(right_wrist_z180=True, debug=False)
        r_off = GR1T2FourierSteamVRRetargeter(right_wrist_z180=False, debug=False)
        snap = _empty_snap()
        snap["trackers"]["right_forearm"] = _forearm_pose(0.0, 1.2, -0.4)
        a_on = r_on.retarget(snap)
        a_off = r_off.retarget(snap)
        diff = (a_on[10:14] - a_off[10:14]).abs().max().item()
        assert diff > 0.1

    def test_quaternion_unit_norm(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        snap = _empty_snap()
        snap["trackers"]["left_forearm"] = _forearm_pose(0.0, 1.2, 0.4)
        snap["trackers"]["right_forearm"] = _forearm_pose(0.0, 1.2, -0.4)
        a = r.retarget(snap)
        l_q = a[3:7].numpy()
        r_q = a[10:14].numpy()
        assert abs(float(np.linalg.norm(l_q)) - 1.0) < 1e-4
        assert abs(float(np.linalg.norm(r_q)) - 1.0) < 1e-4

    def test_freeze_orientation_uses_idle_quat(self):
        r = GR1T2FourierSteamVRRetargeter(freeze_orientation=True, debug=False)
        snap = _empty_snap()
        # Use a non-identity forearm quat so we can detect whether it
        # leaked into the action.
        q = np.array([math.cos(math.pi / 6), 0.0, math.sin(math.pi / 6), 0.0], dtype=np.float64)
        snap["trackers"]["left_forearm"] = {"pos": np.zeros(3), "quat": q}
        a = r.retarget(snap)
        # Left quat should be the idle quat, not the forearm.
        expected = np.asarray(DEFAULT_LEFT_QUAT, dtype=np.float32)
        assert np.allclose(a[3:7].numpy(), expected, atol=1e-5)


class TestWaistOrigin:
    def test_origin_subtraction_cancels_translation(self):
        r_a = GR1T2FourierSteamVRRetargeter(use_waist_origin=True, debug=False)
        r_b = GR1T2FourierSteamVRRetargeter(use_waist_origin=True, debug=False)

        snap_a = _empty_snap()
        snap_a["trackers"]["waist"] = {"pos": np.array([0.0, 1.0, 0.0]), "quat": np.array([1.0, 0, 0, 0])}
        snap_a["trackers"]["left_forearm"] = _forearm_pose(0.5, 1.2, 0.3)
        snap_a["trackers"]["right_forearm"] = _forearm_pose(-0.5, 1.2, 0.3)

        snap_b = _empty_snap()
        snap_b["trackers"]["waist"] = {"pos": np.array([2.0, 1.0, 3.0]), "quat": np.array([1.0, 0, 0, 0])}
        snap_b["trackers"]["left_forearm"] = _forearm_pose(2.5, 1.2, 3.3)
        snap_b["trackers"]["right_forearm"] = _forearm_pose(1.5, 1.2, 3.3)

        a = r_a.retarget(snap_a)
        b = r_b.retarget(snap_b)
        diff = (a[0:14] - b[0:14]).abs().max().item()
        assert diff < 1e-4

    def test_disabled_leaks_translation(self):
        r = GR1T2FourierSteamVRRetargeter(use_waist_origin=False, debug=False)
        snap_a = _empty_snap()
        snap_a["trackers"]["left_forearm"] = _forearm_pose(0.5, 1.2, 0.3)
        snap_b = _empty_snap()
        snap_b["trackers"]["left_forearm"] = _forearm_pose(2.5, 1.2, 3.3)
        a = r.retarget(snap_a)
        b = r.retarget(snap_b)
        assert (a[0:3] - b[0:3]).abs().max().item() > 0.5


class TestFingerCurlActionSource:
    """Verify the retargeter consumes action-API finger curls as priority 3."""

    def _curl_input(self, left_curls, right_curls):
        return {
            "left":  {"trigger": 0.0, "grip": 0.0, "finger_curls": left_curls},
            "right": {"trigger": 0.0, "grip": 0.0, "finger_curls": right_curls},
        }

    def test_finger_action_source_reported(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        action_inputs = self._curl_input([0.8, 0.9, 0.5, 0.3, 0.2], [0.2] * 5)
        r.retarget(_empty_snap(), action_inputs=action_inputs)
        info = r.get_source_info()
        assert info["left_finger"] == "finger_action"
        assert info["right_finger"] == "finger_action"

    def test_hand_joints_track_finger_curls(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        # Full fist left, open right
        action_inputs = self._curl_input([1.0] * 5, [0.0] * 5)
        a = r.retarget(_empty_snap(), action_inputs=action_inputs)
        # 22 hand joints start at index 14
        # left_11 occupies slots with L_* joints interleaved (pack_22d)
        # max value across left-hand slots should be large; right-hand slots zero
        left_max = float(a[14:36].abs().max().item())
        assert left_max > 0.5  # fist → high proximal values

    def test_all_zero_curls_fall_through(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        action_inputs = self._curl_input([0.0] * 5, [0.0] * 5)
        r.retarget(_empty_snap(), action_inputs=action_inputs)
        info = r.get_source_info()
        # all zero → shouldn't claim finger_action
        assert info["left_finger"] != "finger_action"
        assert info["right_finger"] != "finger_action"

    def test_action_trigger_used_by_button_fallback(self):
        """When finger_curls are zero but action trigger is nonzero,
        button fallback fires with the action-API value even if legacy
        ctrls.*.buttons.trigger is zero."""
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        snap = _empty_snap()
        # Empty/legacy controller buttons (mimicking vr_sampler's
        # ``getControllerState`` returning 0 for knuckles).
        snap["controllers"] = {
            "left":  {"pose": None, "buttons": {"trigger": 0.0, "grip": 0.0}},
            "right": {"pose": None, "buttons": {"trigger": 0.0, "grip": 0.0}},
        }
        action_inputs = {
            "left":  {"trigger": 0.9, "grip": 0.0, "finger_curls": [0.0] * 5},
            "right": {"trigger": 0.0, "grip": 0.9, "finger_curls": [0.0] * 5},
        }
        a = r.retarget(snap, action_inputs=action_inputs)
        info = r.get_source_info()
        assert info["left_finger"] == "button"
        assert info["right_finger"] == "button"
        assert float(a[14:36].abs().max().item()) > 0.1


class TestButtonGripFallback:
    """Verify the retargeter falls through to controller-button grip when
    no skeletal / VMC data is available."""

    def test_trigger_drives_fingers_when_hands_none(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        snap = _empty_snap()
        # Populate controllers with a strong trigger on both sides
        snap["controllers"] = {
            "left": {
                "pose": {"pos": np.array([0.0, 1.0, 0.0]), "quat": np.array([1.0, 0, 0, 0])},
                "buttons": {"trigger": 0.9, "grip": 0.0, "menu": False},
            },
            "right": {
                "pose": {"pos": np.array([0.0, 1.0, 0.0]), "quat": np.array([1.0, 0, 0, 0])},
                "buttons": {"trigger": 0.9, "grip": 0.0, "menu": False},
            },
        }
        a = r.retarget(snap)
        info = r.get_source_info()
        assert info["left_finger"] == "button"
        assert info["right_finger"] == "button"
        # Some hand joints should be nonzero (pinch gesture)
        assert float(a[14:36].abs().max().item()) > 0.1

    def test_idle_when_buttons_at_rest(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        snap = _empty_snap()
        snap["controllers"] = {
            "left": {"pose": None, "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
            "right": {"pose": None, "buttons": {"trigger": 0.0, "grip": 0.0, "menu": False}},
        }
        r.retarget(snap)
        info = r.get_source_info()
        # No input → no fallback fired → idle
        assert info["left_finger"] == "idle"
        assert info["right_finger"] == "idle"

    def test_fallback_disabled(self):
        r = GR1T2FourierSteamVRRetargeter(enable_button_grip_fallback=False, debug=False)
        snap = _empty_snap()
        snap["controllers"] = {
            "left": {"pose": None, "buttons": {"trigger": 0.9, "grip": 0.9, "menu": False}},
            "right": {"pose": None, "buttons": {"trigger": 0.9, "grip": 0.9, "menu": False}},
        }
        r.retarget(snap)
        info = r.get_source_info()
        # Fallback off → even full trigger stays idle
        assert info["left_finger"] == "idle"
        assert info["right_finger"] == "idle"


class TestReset:
    def test_reset_source_info(self):
        r = GR1T2FourierSteamVRRetargeter(debug=False)
        snap = _empty_snap()
        snap["trackers"]["left_forearm"] = _forearm_pose(0.0, 1.2, 0.4)
        r.retarget(snap)
        r.reset()
        info = r.get_source_info()
        assert info["left_eef"] == "default"


class TestDisableArmTracking:
    """9.26 — `disable_arm_tracking=True` forces idle pose for both arms
    regardless of forearm tracker / controller state."""

    @staticmethod
    def _full_snap_with_tracker_and_controller():
        """Snapshot with BOTH a forearm tracker AND a controller pose set
        (worst case: would normally route through forearm path)."""
        return {
            "timestamp": 0.0,
            "hmd": None,
            "trackers": {
                "left_forearm": _forearm_pose(0.5, 1.0, 0.5),
                "right_forearm": _forearm_pose(-0.5, 1.0, 0.5),
            },
            "hands": {"left": None, "right": None},
            "controllers": {
                "left": {"pose": {"pos": np.array([0.7, 0.2, 0.8]),
                                  "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                          "buttons": {}},
                "right": {"pose": {"pos": np.array([0.7, -0.2, 0.8]),
                                   "quat": np.array([1.0, 0.0, 0.0, 0.0])},
                           "buttons": {}},
            },
            "frame_count": 1,
        }

    def test_disabled_falls_back_to_idle_with_tracker_present(self):
        """Even when forearm tracker is present, idle pose is returned."""
        cfg = GR1T2FourierRetargeterCfg(disable_arm_tracking=True, debug=False)
        r = GR1T2FourierSteamVRRetargeter(cfg)
        snap = self._full_snap_with_tracker_and_controller()
        out = r.retarget(snap)
        # Left arm @ DEFAULT_LEFT_POS
        assert abs(float(out[0]) - DEFAULT_LEFT_POS[0]) < 1e-4
        assert abs(float(out[1]) - DEFAULT_LEFT_POS[1]) < 1e-4
        assert abs(float(out[2]) - DEFAULT_LEFT_POS[2]) < 1e-4
        # Right arm @ DEFAULT_RIGHT_POS
        assert abs(float(out[7]) - DEFAULT_RIGHT_POS[0]) < 1e-4
        assert abs(float(out[8]) - DEFAULT_RIGHT_POS[1]) < 1e-4
        assert abs(float(out[9]) - DEFAULT_RIGHT_POS[2]) < 1e-4
        info = r.get_source_info()
        assert info["left_eef"] == "default(disabled)"
        assert info["right_eef"] == "default(disabled)"

    def test_disabled_with_controller_priority_still_idle(self):
        """`prefer_controller=True` is overridden by `disable_arm_tracking`."""
        cfg = GR1T2FourierRetargeterCfg(
            disable_arm_tracking=True,
            prefer_controller_for_eef=True,
            debug=False,
        )
        r = GR1T2FourierSteamVRRetargeter(cfg)
        snap = self._full_snap_with_tracker_and_controller()
        out = r.retarget(snap)
        # Should still land at idle pose even with prefer_controller=True
        assert abs(float(out[0]) - DEFAULT_LEFT_POS[0]) < 1e-4
        assert abs(float(out[7]) - DEFAULT_RIGHT_POS[0]) < 1e-4

    def test_default_false_uses_normal_priority(self):
        """When `disable_arm_tracking=False` (default), forearm path runs."""
        cfg = GR1T2FourierRetargeterCfg(disable_arm_tracking=False, debug=False)
        r = GR1T2FourierSteamVRRetargeter(cfg)
        snap = self._full_snap_with_tracker_and_controller()
        out = r.retarget(snap)
        info = r.get_source_info()
        # Should pick "forearm" (priority chain default)
        assert info["left_eef"] == "forearm"
        assert info["right_eef"] == "forearm"


class TestFingerLowPass:
    """9.23 — single-pole low-pass on the 22D finger output (memory.md §10.31)."""

    @staticmethod
    def _curl_action_inputs(curl_value: float) -> dict:
        # 5 per-side curls (thumb, index, middle, ring, pinky) — drives the
        # finger_action source path so we exercise the EMA on a non-trivial
        # output regardless of the trackers/skeletal state.
        five = [curl_value] * 5
        return {
            "left": {"trigger": 0.0, "grip": 0.0, "finger_curls": list(five)},
            "right": {"trigger": 0.0, "grip": 0.0, "finger_curls": list(five)},
        }

    def test_alpha_one_disables_filter(self):
        cfg = GR1T2FourierRetargeterCfg(finger_low_pass_alpha=1.0, debug=False)
        r = GR1T2FourierSteamVRRetargeter(cfg)
        # First call seeds prev; second call with new value should pass
        # through verbatim when alpha == 1.
        r.retarget(_empty_snap(), action_inputs=self._curl_action_inputs(0.5))
        out = r.retarget(_empty_snap(), action_inputs=self._curl_action_inputs(0.0))
        # Resting curls -> exactly 0 in the 22D slice (no smoothing carry-over).
        assert float(out[14:36].abs().max().item()) < 1e-6

    def test_alpha_smooths_step_response(self):
        cfg = GR1T2FourierRetargeterCfg(finger_low_pass_alpha=0.4, debug=False)
        r = GR1T2FourierSteamVRRetargeter(cfg)
        # Frame 1 — fully extended (curls at 1.0).  prev is None so this
        # passes through verbatim and seeds prev.
        out_first = r.retarget(_empty_snap(), action_inputs=self._curl_action_inputs(1.0))
        first_max = float(out_first[14:36].abs().max().item())
        # Frame 2 — instantaneously released (curls at 0).  Without the
        # filter the output would also be all-zero; with alpha=0.4 the
        # output must retain (1 - 0.4) = 60% of the previous magnitude.
        out_second = r.retarget(_empty_snap(), action_inputs=self._curl_action_inputs(0.0))
        second_max = float(out_second[14:36].abs().max().item())
        assert first_max > 0.5, first_max
        # 0.6 * first_max ± numerical tolerance.
        expected = 0.6 * first_max
        assert abs(second_max - expected) < 1e-3, (first_max, second_max, expected)

    def test_first_frame_passes_through(self):
        """A fresh retargeter must NOT add startup lag on the very first call."""
        cfg = GR1T2FourierRetargeterCfg(finger_low_pass_alpha=0.2, debug=False)
        r = GR1T2FourierSteamVRRetargeter(cfg)
        out = r.retarget(_empty_snap(), action_inputs=self._curl_action_inputs(0.7))
        # With prev unset, the EMA must be skipped — the output should
        # match the unfiltered curl-mapping path.
        non_zero = float(out[14:36].abs().max().item())
        # Non-zero (curls > 0); for alpha != 1 with prev=None this proves
        # the first frame avoids being attenuated to zero by the filter.
        assert non_zero > 0.05, non_zero

    def test_reset_clears_prev(self):
        cfg = GR1T2FourierRetargeterCfg(finger_low_pass_alpha=0.4, debug=False)
        r = GR1T2FourierSteamVRRetargeter(cfg)
        r.retarget(_empty_snap(), action_inputs=self._curl_action_inputs(1.0))
        # Mid-trajectory reset — prev should drop, so the next first
        # frame at curl=0 must produce all-zero output (no carry-over
        # from the previous "fully closed" state).
        r.reset()
        out = r.retarget(_empty_snap(), action_inputs=self._curl_action_inputs(0.0))
        assert float(out[14:36].abs().max().item()) < 1e-6
