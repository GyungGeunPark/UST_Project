"""Unit tests for ``ust_hm_glove.teleop.fourier_hand_mapper``."""

from __future__ import annotations

import math

import numpy as np
import pytest

from ust_ws.ust_hm_glove.teleop.fourier_hand_mapper import (
    FOURIER_JOINT_DIM,
    FOURIER_TOTAL_HAND_JOINTS,
    FourierHandMapper,
    IDX_INDEX_INT,
    IDX_INDEX_PROX,
    IDX_MIDDLE_INT,
    IDX_MIDDLE_PROX,
    IDX_PINKY_INT,
    IDX_PINKY_PROX,
    IDX_RING_INT,
    IDX_RING_PROX,
    IDX_THUMB_DIST,
    IDX_THUMB_PITCH,
    IDX_THUMB_YAW,
    pack_22d,
)


def _identity_skeleton() -> np.ndarray:
    """31-bone identity pose — qw=1, zero positions."""
    bones = np.zeros((31, 7), dtype=np.float32)
    bones[:, 3] = 1.0
    return bones


class TestSkeletalBranch:
    def test_tpose_produces_small_values(self):
        mapper = FourierHandMapper()
        out = mapper.map_hand_skeletal(_identity_skeleton(), is_right=False, finger_curls=[0.0] * 5)
        assert out.shape == (FOURIER_JOINT_DIM,)
        assert np.isfinite(out).all()
        assert float(out[IDX_INDEX_PROX]) < 0.05
        assert float(out[IDX_THUMB_PITCH]) < 0.05

    def test_fist_curl_maxes_proximal(self):
        mapper = FourierHandMapper()
        out = mapper.map_hand_skeletal(_identity_skeleton(), is_right=False, finger_curls=[1.0] * 5)
        # Bend ~ 0.5 * 0 + 0.5 * 1.0 * 1.57 = 0.785 on blend.
        for idx in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_RING_PROX, IDX_PINKY_PROX):
            assert 0.5 < float(out[idx]) <= 1.57 + 1e-6
        assert float(out[IDX_THUMB_PITCH]) > 0.4

    def test_mimic_intermediate_fills(self):
        mapper = FourierHandMapper(intermediate_mimic=0.9)
        out = mapper.map_hand_skeletal(_identity_skeleton(), is_right=False, finger_curls=[1.0] * 5)
        # Intermediates default to proximal * mimic ratio.
        for prox, inter in (
            (IDX_INDEX_PROX, IDX_INDEX_INT),
            (IDX_MIDDLE_PROX, IDX_MIDDLE_INT),
            (IDX_RING_PROX, IDX_RING_INT),
            (IDX_PINKY_PROX, IDX_PINKY_INT),
        ):
            assert abs(float(out[inter]) - 0.9 * float(out[prox])) < 1e-5

    def test_handles_nan_gracefully(self):
        mapper = FourierHandMapper()
        bones = _identity_skeleton()
        bones[6] = np.nan  # poison index proximal
        out = mapper.map_hand_skeletal(bones, is_right=True, finger_curls=[0.2] * 5)
        assert np.isfinite(out).all()


class TestVMCBranch:
    def test_vmc_basic_fingers(self):
        # Disable rest-pose subtraction for this legacy single-frame test.
        mapper = FourierHandMapper(vmc_subtract_rest=False)
        bones = {
            "LeftIndexProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
            "LeftMiddleProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
            "LeftThumbProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        }
        out = mapper.map_hand_vmc(bones, is_right=False)
        assert out.shape == (FOURIER_JOINT_DIM,)
        assert np.isfinite(out).all()
        assert float(out[IDX_INDEX_PROX]) > 0.1

    def test_empty_vmc(self):
        mapper = FourierHandMapper()
        out = mapper.map_hand_vmc({}, is_right=True)
        assert out.shape == (FOURIER_JOINT_DIM,)
        # No bones → mimic still fills with zero (still all zero).
        assert float(np.abs(out).max()) < 1e-6


class TestVMCRestPoseCalibration:
    """9.14: per-bone REST POSE calibration solves UDCAP's static offset."""

    def test_default_subtract_rest_is_true(self):
        m = FourierHandMapper()
        assert m.vmc_subtract_rest is True
        # 9.18 default: 10 frames (~0.5 s) -- shorter than 9.14's 30 to
        # reduce the chance the user fidgets during cal and absorbs
        # later motion.
        assert m.vmc_rest_frames == 10

    def test_first_n_frames_return_zero_curl(self):
        """During the calibration window, all VMC outputs are ~zero."""
        mapper = FourierHandMapper(vmc_rest_frames=5)
        # Bone at 60° rotation as the user's "rest" — should NOT show as curl.
        bones = {
            "LeftIndexProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        }
        for _ in range(5):
            out = mapper.map_hand_vmc(bones, is_right=False)
            assert float(out[IDX_INDEX_PROX]) < 1e-3

    def test_post_calibration_rest_yields_zero(self):
        """After cal, the same bone at the rest pose still returns zero."""
        mapper = FourierHandMapper(vmc_rest_frames=3)
        bones_rest = {
            "LeftIndexProximal": (0.0, 0.0, 0.0, math.cos(math.pi / 6)),
        }
        # Run 3 cal frames at the rest pose.
        for _ in range(3):
            mapper.map_hand_vmc(bones_rest, is_right=False)
        # 4th frame at the same pose → relative motion is zero.
        out = mapper.map_hand_vmc(bones_rest, is_right=False)
        assert float(out[IDX_INDEX_PROX]) < 1e-3

    def test_post_calibration_motion_relative_to_rest(self):
        """Motion AWAY from rest yields nonzero curl."""
        mapper = FourierHandMapper(
            vmc_rest_frames=3, proximal_scale=1.0, use_tanh=False
        )
        # Calibrate at "open hand" rest (small ~5° offset).
        rest_bend = math.radians(5.0)
        rest_quat = (0.0, 0.0, 0.0, math.cos(rest_bend / 2.0))
        bones_rest = {"LeftIndexProximal": rest_quat}
        for _ in range(3):
            mapper.map_hand_vmc(bones_rest, is_right=False)
        # Now user curls fully (60° rotation around X axis).
        full_bend = math.radians(60.0)
        bones_curl = {
            "LeftIndexProximal": (
                math.sin(full_bend / 2.0), 0.0, 0.0,
                math.cos(full_bend / 2.0),
            ),
        }
        out = mapper.map_hand_vmc(bones_curl, is_right=False)
        # 60° → 1.05 rad of motion → ~33% of _PROXIMAL_MAX (1.57)  → 0.52
        # but signed -> we compute |relative bend| > 0.4.
        assert float(out[IDX_INDEX_PROX]) > 0.3

    def test_thumb_yaw_zero_when_at_rest(self):
        """9.15 fix: at rest (yaw=0), thumb_yaw output must be 0, not the
        URDF range midpoint.  Previously the formula
            out = yaw*(hi-lo)/2 + (lo+hi)/2
        mapped yaw=0 to (-0.5+1.0)/2 = +0.25, which after the pack_22d
        sign negation produced the constant L_thb_yaw=-0.250 we observed
        across 460 frames in the 9.14 verification run.

        Uses Unity convention (ThumbIntermediate present) so
        ThumbProximal is interpreted as the CMC (yaw) joint.
        """
        mapper = FourierHandMapper(vmc_subtract_rest=False)
        bones = {
            # Unity convention: presence of ThumbIntermediate switches
            # the thumb_cmc lookup to ThumbProximal.
            "LeftThumbProximal": (0.0, 0.0, 0.0, 1.0),       # CMC (yaw) at rest
            "LeftThumbIntermediate": (0.0, 0.0, 0.0, 1.0),   # MCP (pitch) at rest
        }
        out = mapper.map_hand_vmc(bones, is_right=False)
        assert abs(float(out[IDX_THUMB_YAW])) < 1e-6, out[IDX_THUMB_YAW]

    def test_thumb_yaw_full_positive(self):
        """Yaw=+90° (full Z rotation) maps to range_max (+1.0)."""
        mapper = FourierHandMapper(vmc_subtract_rest=False)
        bones = {
            "LeftThumbProximal": (
                0.0, 0.0, math.sin(math.radians(45.0)), math.cos(math.radians(45.0)),
            ),
            "LeftThumbIntermediate": (0.0, 0.0, 0.0, 1.0),
        }
        out = mapper.map_hand_vmc(bones, is_right=False)
        # _quat_to_yaw normalises by π/2, so 90° → 1.0 → mapped to range_max=1.0.
        assert float(out[IDX_THUMB_YAW]) > 0.9

    def test_thumb_yaw_full_negative(self):
        """Yaw=-90° maps to range_min (-0.5)."""
        mapper = FourierHandMapper(vmc_subtract_rest=False)
        bones = {
            "LeftThumbProximal": (
                0.0, 0.0, math.sin(math.radians(-45.0)), math.cos(math.radians(45.0)),
            ),
            "LeftThumbIntermediate": (0.0, 0.0, 0.0, 1.0),
        }
        out = mapper.map_hand_vmc(bones, is_right=False)
        # Should be near -0.5 (range_min).
        assert float(out[IDX_THUMB_YAW]) < -0.4
        assert float(out[IDX_THUMB_YAW]) >= -0.5 - 1e-6

    def test_reset_vmc_rest_clears_calibration(self):
        mapper = FourierHandMapper(vmc_rest_frames=2, vmc_subtract_rest=True)
        bones = {"LeftIndexProximal": (0.0, 0.0, 0.0, 1.0)}
        # Calibrate.
        for _ in range(2):
            mapper.map_hand_vmc(bones, is_right=False)
        assert mapper._vmc_rest_calibrated
        mapper.reset_vmc_rest()
        assert not mapper._vmc_rest_calibrated
        assert mapper._vmc_rest_quats == {}


class TestPack22D:
    def test_length(self):
        left = np.zeros(FOURIER_JOINT_DIM, dtype=np.float32)
        right = np.zeros(FOURIER_JOINT_DIM, dtype=np.float32)
        out = pack_22d(left, right)
        assert out.shape == (FOURIER_TOTAL_HAND_JOINTS,)

    def test_interleaving_order(self):
        # 9.9 fix: pack_22d now applies a per-slot sign vector
        # (PACK_22D_SIGNS) so the unsigned curl magnitudes from the mapper
        # land in the GR1T2 joints' actual flexion direction.  Proximal /
        # intermediate / thumb_yaw joints are negated (their USD limits
        # are [-x, 0]); thumb_pitch / thumb_distal stay positive.
        left = np.arange(FOURIER_JOINT_DIM, dtype=np.float32)         # 0..10
        right = np.arange(100, 100 + FOURIER_JOINT_DIM, dtype=np.float32)  # 100..110
        packed = pack_22d(left, right)
        # Slot 0 = L_index_proximal ← -left[0]  (proximal is negative range)
        assert packed[0] == -left[IDX_INDEX_PROX]
        # Slot 5 = R_index_proximal ← -right[0]
        assert packed[5] == -right[IDX_INDEX_PROX]
        # Slot 14 = L_thumb_proximal_pitch ← +left[9]  (thumb pitch positive)
        assert packed[14] == +left[IDX_THUMB_PITCH]
        # Slot 19 = R_thumb_proximal_pitch ← +right[9]
        assert packed[19] == +right[IDX_THUMB_PITCH]
        # Slot 20 = L_thumb_distal ← +left[10]  (thumb distal follows pitch)
        assert packed[20] == +left[IDX_THUMB_DIST]
        # Slot 21 = R_thumb_distal ← +right[10]
        assert packed[21] == +right[IDX_THUMB_DIST]
        # Spot-check intermediate (negated)
        assert packed[10] == -left[IDX_INDEX_INT]   # L_index_intermediate
        # Spot-check thumb_yaw (negated)
        assert packed[4] == -left[IDX_THUMB_YAW]    # L_thumb_proximal_yaw

    def test_pack_22d_signs_constant_well_formed(self):
        """PACK_22D_SIGNS must have one sign per output slot (22)."""
        from ust_ws.ust_hm_glove.teleop.fourier_hand_mapper import (
            PACK_22D_SIGNS,
            FOURIER_TOTAL_HAND_JOINTS,
        )
        assert len(PACK_22D_SIGNS) == FOURIER_TOTAL_HAND_JOINTS
        # Every entry is +1 or -1.
        for s in PACK_22D_SIGNS:
            assert s in (-1.0, 1.0)
        # Thumb pitch / thumb distal are positive; everything else negative.
        positive_slots = {14, 19, 20, 21}
        for i, s in enumerate(PACK_22D_SIGNS):
            expected = +1.0 if i in positive_slots else -1.0
            assert s == expected, f"slot {i}: expected sign {expected}, got {s}"

    def test_rejects_wrong_length(self):
        with pytest.raises(ValueError):
            pack_22d(np.zeros(5, dtype=np.float32), np.zeros(FOURIER_JOINT_DIM, dtype=np.float32))
        with pytest.raises(ValueError):
            pack_22d(np.zeros(FOURIER_JOINT_DIM, dtype=np.float32), np.zeros(99, dtype=np.float32))


class TestFingerCurlMapper:
    def test_rest_returns_zero(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_finger_curls([0.0, 0.0, 0.0, 0.0, 0.0])
        assert out.shape == (FOURIER_JOINT_DIM,)
        assert np.isfinite(out).all()
        # proximals open
        for idx in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_RING_PROX, IDX_PINKY_PROX):
            assert float(out[idx]) < 1e-6
        assert float(out[IDX_THUMB_PITCH]) < 1e-6

    def test_full_fist(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_finger_curls([1.0, 1.0, 1.0, 1.0, 1.0])
        for idx in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_RING_PROX, IDX_PINKY_PROX):
            assert float(out[idx]) > 0.8
        assert float(out[IDX_THUMB_PITCH]) > 0.8

    def test_per_finger_independence(self):
        """Only index curled → only index proximal moves."""
        mapper = FourierHandMapper()
        out = mapper.map_from_finger_curls([0.0, 1.0, 0.0, 0.0, 0.0])
        assert float(out[IDX_INDEX_PROX]) > 0.5
        assert float(out[IDX_MIDDLE_PROX]) < 0.05
        assert float(out[IDX_RING_PROX]) < 0.05
        assert float(out[IDX_PINKY_PROX]) < 0.05
        assert float(out[IDX_THUMB_PITCH]) < 0.05

    def test_thumb_yaw_scales_with_thumb_curl(self):
        mapper = FourierHandMapper()
        out_open = mapper.map_from_finger_curls([0.0, 0.0, 0.0, 0.0, 0.0])
        out_fold = mapper.map_from_finger_curls([1.0, 0.0, 0.0, 0.0, 0.0])
        # thumb yaw should differ between open-thumb vs folded-thumb
        assert float(out_fold[IDX_THUMB_YAW]) > float(out_open[IDX_THUMB_YAW])

    def test_clamps_out_of_range(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_finger_curls([2.0, -1.0, 0.5, None, "0.2"])  # type: ignore[list-item]
        # No NaN, no negatives above clamp bounds
        assert np.isfinite(out).all()
        for idx in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_RING_PROX, IDX_PINKY_PROX):
            assert float(out[idx]) >= 0.0

    def test_short_input_padded_with_zeros(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_finger_curls([1.0, 1.0])  # only thumb+index provided
        assert float(out[IDX_INDEX_PROX]) > 0.5
        assert float(out[IDX_THUMB_PITCH]) > 0.5
        assert float(out[IDX_MIDDLE_PROX]) < 1e-6
        assert float(out[IDX_RING_PROX]) < 1e-6
        assert float(out[IDX_PINKY_PROX]) < 1e-6


class TestButtonGripFallback:
    def test_rest_state_is_zero(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_controller_buttons(trigger=0.0, grip=0.0)
        assert out.shape == (FOURIER_JOINT_DIM,)
        assert np.isfinite(out).all()
        # No input → fully open hand
        for idx in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_RING_PROX, IDX_PINKY_PROX):
            assert float(out[idx]) < 1e-6

    def test_trigger_curls_precision_fingers(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_controller_buttons(trigger=1.0, grip=0.0)
        # Precision fingers (index, middle) should curl from trigger
        assert float(out[IDX_INDEX_PROX]) > 0.6
        assert float(out[IDX_MIDDLE_PROX]) > 0.6
        # Power fingers (ring, pinky) should not curl much without grip
        assert float(out[IDX_RING_PROX]) < 0.1
        assert float(out[IDX_PINKY_PROX]) < 0.1
        # Thumb should engage (pinch gesture)
        assert float(out[IDX_THUMB_PITCH]) > 0.3

    def test_grip_curls_power_fingers(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_controller_buttons(trigger=0.0, grip=1.0)
        assert float(out[IDX_RING_PROX]) > 0.8
        assert float(out[IDX_PINKY_PROX]) > 0.8
        # Precision fingers also gain some from grip boost
        assert float(out[IDX_INDEX_PROX]) > 0.1

    def test_thumb_touch_engages_thumb(self):
        mapper = FourierHandMapper()
        out = mapper.map_from_controller_buttons(trigger=0.0, grip=0.0, thumb_touch=True)
        assert float(out[IDX_THUMB_YAW]) > 0.4


class TestClamping:
    def test_clamp_proximal_to_max(self):
        mapper = FourierHandMapper(proximal_scale=10.0)  # exaggerate to hit the clamp
        out = mapper.map_hand_skeletal(_identity_skeleton(), is_right=False, finger_curls=[1.0] * 5)
        for idx in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_RING_PROX, IDX_PINKY_PROX):
            assert float(out[idx]) <= 1.57 + 1e-6

    def test_clamp_thumb_yaw(self):
        mapper = FourierHandMapper()
        bones = {
            # Strong Z-rotation quaternion
            "LeftThumbMetacarpal": (0.0, 0.0, math.sin(math.pi / 2), math.cos(math.pi / 2)),
        }
        out = mapper.map_hand_vmc(bones, is_right=False)
        assert -0.5 - 1e-6 <= float(out[IDX_THUMB_YAW]) <= 1.0 + 1e-6


class TestTanhAmplification:
    """9.12 fix: ``use_tanh=True`` shapes the curl→angle curve."""

    def test_linear_path_full_curl_can_clip(self):
        """With use_tanh=False scale*raw>1 hard-clips to limit."""
        linear = FourierHandMapper(proximal_scale=4.0, use_tanh=False)
        out_full = linear.map_from_finger_curls([0.0, 1.0, 0.0, 0.0, 0.0])
        # Without tanh, scale*1.0 = 4.0 → 4.0*1.57 → clamps to 1.57.
        assert abs(float(out_full[IDX_INDEX_PROX]) - 1.57) < 1e-3

    def test_tanh_path_full_curl_just_below_limit(self):
        """With use_tanh=True the asymptote stays just below joint limit."""
        tanh = FourierHandMapper(proximal_scale=4.0, use_tanh=True)
        out_full = tanh.map_from_finger_curls([0.0, 1.0, 0.0, 0.0, 0.0])
        # tanh(4.0)=0.9993 → 1.57*0.9993 ≈ 1.569.  Same neighbourhood as
        # the linear path here, but crucially the *derivative* at full
        # curl is near zero (smooth saturation) instead of an abrupt clip.
        val = float(out_full[IDX_INDEX_PROX])
        assert val > 1.55
        assert val <= 1.57 + 1e-6

    def test_tanh_path_mid_curl_above_linear_at_low_scale(self):
        """At scale=1, small curl with tanh ~ matches linear; large curl
        with tanh stays below linear (compression toward limit)."""
        linear = FourierHandMapper(proximal_scale=1.0, use_tanh=False)
        tanh = FourierHandMapper(proximal_scale=1.0, use_tanh=True)
        # Small curl = 0.1: linear gives 0.1*1.57=0.157, tanh gives
        # 1.57*tanh(0.1)=1.57*0.0997=0.156.  Within 1%.
        small_lin = float(linear.map_from_finger_curls([0.0, 0.1, 0, 0, 0])[IDX_INDEX_PROX])
        small_tnh = float(tanh.map_from_finger_curls([0.0, 0.1, 0, 0, 0])[IDX_INDEX_PROX])
        assert abs(small_lin - small_tnh) < small_lin * 0.05
        # Large curl = 1.0: linear gives 1.57; tanh gives 1.57*tanh(1)=1.196.
        big_lin = float(linear.map_from_finger_curls([0.0, 1.0, 0, 0, 0])[IDX_INDEX_PROX])
        big_tnh = float(tanh.map_from_finger_curls([0.0, 1.0, 0, 0, 0])[IDX_INDEX_PROX])
        assert big_tnh < big_lin

    def test_tanh_avoids_saturation_under_asymmetric_signal(self):
        """Simulates the UDCAP L/R RSSI asymmetry scenario.  L glove reports
        raw=0.5, R glove reports raw=0.25 (50% weaker).  With scale=4 (the
        9.11 default) linear path saturates BOTH hands at 1.57 -- the user
        loses all dynamic range.  Tanh keeps them distinct."""
        linear = FourierHandMapper(proximal_scale=4.0, use_tanh=False)
        tanh = FourierHandMapper(proximal_scale=4.0, use_tanh=True)
        l_lin = float(linear.map_from_finger_curls([0, 0.5, 0, 0, 0])[IDX_INDEX_PROX])
        r_lin = float(linear.map_from_finger_curls([0, 0.25, 0, 0, 0])[IDX_INDEX_PROX])
        l_tnh = float(tanh.map_from_finger_curls([0, 0.5, 0, 0, 0])[IDX_INDEX_PROX])
        r_tnh = float(tanh.map_from_finger_curls([0, 0.25, 0, 0, 0])[IDX_INDEX_PROX])
        # Linear: L is clamped at 1.57; R clamped at 1.57.  No distinction.
        assert abs(l_lin - r_lin) < 1e-3
        # Tanh: L at 1.57*tanh(2.0)=1.515, R at 1.57*tanh(1.0)=1.196 → diff > 0.2.
        assert l_tnh > r_tnh + 0.2

    def test_default_use_tanh_is_true(self):
        """The 9.12 default ON/True so existing callers benefit immediately."""
        m = FourierHandMapper()
        assert m.use_tanh is True

    def test_tanh_preserves_zero_at_zero_curl(self):
        m = FourierHandMapper(proximal_scale=4.0, use_tanh=True)
        out = m.map_from_finger_curls([0.0] * 5)
        assert float(out[IDX_INDEX_PROX]) < 1e-6
        assert float(out[IDX_THUMB_PITCH]) < 1e-6
