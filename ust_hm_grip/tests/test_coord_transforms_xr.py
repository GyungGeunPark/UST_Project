"""Unit tests for the XR -> Isaac Lab coordinate transforms (research/47 §7)."""

from __future__ import annotations

import numpy as np
import pytest

from ust_ws.ust_hm_grip.teleop import coord_transforms as ct


def _q_identity_xyzw():
    return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def _q_identity_wxyz():
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def test_xyzw_to_wxyz_roundtrip():
    q_xyzw = np.array([0.1, 0.2, 0.3, 0.927], dtype=np.float64)
    q_wxyz = ct.xyzw_to_wxyz(q_xyzw)
    q_back = ct.wxyz_to_xyzw(q_wxyz)
    np.testing.assert_allclose(q_back, q_xyzw, atol=1e-9)


def test_xr_to_isaaclab_identity_pose():
    """Origin + identity quaternion in XR maps to origin + identity in IL."""
    pos, quat = ct.xr_to_isaaclab((0.0, 0.0, 0.0), _q_identity_xyzw())
    np.testing.assert_allclose(pos, (0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(quat, _q_identity_wxyz(), atol=1e-9)


def test_xr_to_isaaclab_x_axis():
    """XR +X (right) maps to IL -Y (left becomes negative)."""
    pos, _ = ct.xr_to_isaaclab((1.0, 0.0, 0.0), _q_identity_xyzw())
    np.testing.assert_allclose(pos, (0.0, -1.0, 0.0), atol=1e-9)


def test_xr_to_isaaclab_y_axis():
    """XR +Y (up) maps to IL +Z (up)."""
    pos, _ = ct.xr_to_isaaclab((0.0, 1.0, 0.0), _q_identity_xyzw())
    np.testing.assert_allclose(pos, (0.0, 0.0, 1.0), atol=1e-9)


def test_xr_to_isaaclab_z_axis():
    """XR +Z (back, per OpenXR LOCAL) maps to IL -X (behind robot)."""
    pos, _ = ct.xr_to_isaaclab((0.0, 0.0, 1.0), _q_identity_xyzw())
    np.testing.assert_allclose(pos, (-1.0, 0.0, 0.0), atol=1e-9)


def test_xr_to_isaaclab_combined():
    """A point at (1, 2, 3) in XR -> (-3, -1, 2) in IL."""
    pos, _ = ct.xr_to_isaaclab((1.0, 2.0, 3.0), _q_identity_xyzw())
    np.testing.assert_allclose(pos, (-3.0, -1.0, 2.0), atol=1e-9)


def test_xr_to_isaaclab_quaternion_x_rotation():
    """A 90° rotation about XR +X (right) -> 90° about IL -Y (-left)."""
    half = np.sqrt(0.5)
    q_xyzw = np.array([half, 0.0, 0.0, half], dtype=np.float64)
    _, q_il = ct.xr_to_isaaclab((0.0, 0.0, 0.0), q_xyzw)
    expected = np.array([half, 0.0, -half, 0.0], dtype=np.float64)
    np.testing.assert_allclose(q_il, expected, atol=1e-9)


def test_xr_to_isaaclab_array_shape():
    """Vectorised version returns (N, 7) array."""
    poses = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    out = ct.xr_to_isaaclab_array(poses)
    assert out.shape == (2, 7)
    np.testing.assert_allclose(out[0, :3], (0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(out[0, 3:7], (1.0, 0.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(out[1, :3], (-3.0, -1.0, 2.0), atol=1e-9)


def test_xr_to_isaaclab_round_trip_via_inverse():
    """Position round-trip recovers the original."""
    pos_xr = np.array([0.5, 1.0, -0.3])
    q_xr_xyzw = np.array([0.1, 0.2, 0.3, np.sqrt(1 - 0.14)], dtype=np.float64)
    q_xr_xyzw /= np.linalg.norm(q_xr_xyzw)

    pos_il, _q_il_wxyz = ct.xr_to_isaaclab(pos_xr, q_xr_xyzw)
    pos_xr_back = ct.R_XR2IL.T @ pos_il
    np.testing.assert_allclose(pos_xr_back, pos_xr, atol=1e-9)


@pytest.mark.parametrize("bad_quat", [
    [0.0, 0.0, 0.0],
    [0.0],
])
def test_xr_to_isaaclab_invalid_quat_raises(bad_quat):
    with pytest.raises(ValueError):
        ct.xr_to_isaaclab((0.0, 0.0, 0.0), bad_quat)


# ── Phase D calibration: quat_conjugate ────────────────────────────────
def test_quat_conjugate_identity():
    """conjugate(identity) = identity."""
    q = ct.quat_conjugate(_q_identity_wxyz())
    np.testing.assert_allclose(q, _q_identity_wxyz(), atol=1e-9)


def test_quat_conjugate_flips_vector_part():
    """conjugate((w, x, y, z)) = (w, -x, -y, -z) — raw, no normalisation."""
    q_in = np.array([0.5, 0.1, 0.2, 0.3], dtype=np.float64)
    q_out = ct.quat_conjugate(q_in)
    np.testing.assert_allclose(q_out, np.array([0.5, -0.1, -0.2, -0.3]),
                                atol=1e-9)


def test_quat_conjugate_is_inverse_for_unit_quat():
    """For unit quat q, q * conjugate(q) = identity."""
    # 60° about (1, 1, 0)/√2
    angle = np.pi / 3
    axis = np.array([1.0, 1.0, 0.0]) / np.sqrt(2)
    q = np.array(
        [np.cos(angle / 2),
         axis[0] * np.sin(angle / 2),
         axis[1] * np.sin(angle / 2),
         axis[2] * np.sin(angle / 2)],
        dtype=np.float64,
    )
    inv = ct.quat_conjugate(q)
    product = ct.quat_multiply(q, inv)
    np.testing.assert_allclose(product, _q_identity_wxyz(), atol=1e-9)


def test_quat_delta_from_calibration_zero():
    """If raw_quat == zero_quat, delta = identity (no rotation)."""
    zero = np.array([np.cos(np.pi / 6), 0.0, np.sin(np.pi / 6), 0.0],
                    dtype=np.float64)  # 60° about +Y
    raw = zero.copy()
    delta = ct.quat_multiply(raw, ct.quat_conjugate(zero))
    np.testing.assert_allclose(delta, _q_identity_wxyz(), atol=1e-9)
    # And the resulting Euler angles are all 0
    y, p, r = ct.quat_wxyz_to_euler_zyx(delta)
    assert abs(y) < 1e-9 and abs(p) < 1e-9 and abs(r) < 1e-9


def test_quat_delta_isolates_actual_rotation():
    """User starts with body tilted (zero = 30° pitch).  Then user
    rotates head 45° more in pitch.  Delta should be 45°, not 75°."""
    base_pitch = np.pi / 6      # 30°
    extra_pitch = np.pi / 4     # 45°
    q_zero = np.array(
        [np.cos(base_pitch / 2), 0.0, np.sin(base_pitch / 2), 0.0],
        dtype=np.float64,
    )
    q_raw = np.array(
        [np.cos((base_pitch + extra_pitch) / 2), 0.0,
         np.sin((base_pitch + extra_pitch) / 2), 0.0],
        dtype=np.float64,
    )
    delta = ct.quat_multiply(q_raw, ct.quat_conjugate(q_zero))
    y, p, r = ct.quat_wxyz_to_euler_zyx(delta)
    assert abs(y) < 1e-6
    assert abs(p - extra_pitch) < 1e-6
    assert abs(r) < 1e-6


# ── Phase D (13th-bis): quat_wxyz_to_euler_zyx ─────────────────────────
def test_euler_zyx_identity():
    """Identity quat → (0, 0, 0)."""
    y, p, r = ct.quat_wxyz_to_euler_zyx(_q_identity_wxyz())
    assert abs(y) < 1e-9 and abs(p) < 1e-9 and abs(r) < 1e-9


def test_euler_zyx_yaw_90():
    """90° about +Z → yaw=π/2, pitch=0, roll=0."""
    half = np.sqrt(0.5)
    # 90° about +Z (wxyz): w=cos(45°)=√0.5, x=0, y=0, z=sin(45°)=√0.5
    q = np.array([half, 0.0, 0.0, half], dtype=np.float64)
    y, p, r = ct.quat_wxyz_to_euler_zyx(q)
    assert abs(y - np.pi / 2) < 1e-9
    assert abs(p) < 1e-9
    assert abs(r) < 1e-9


def test_euler_zyx_pitch_45():
    """45° about +Y → yaw=0, pitch=π/4, roll=0."""
    angle = np.pi / 4
    q = np.array([np.cos(angle / 2), 0.0, np.sin(angle / 2), 0.0], dtype=np.float64)
    y, p, r = ct.quat_wxyz_to_euler_zyx(q)
    assert abs(y) < 1e-9
    assert abs(p - angle) < 1e-9
    assert abs(r) < 1e-9


def test_euler_zyx_roll_30():
    """30° about +X → yaw=0, pitch=0, roll=π/6."""
    angle = np.pi / 6
    q = np.array([np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0], dtype=np.float64)
    y, p, r = ct.quat_wxyz_to_euler_zyx(q)
    assert abs(y) < 1e-9
    assert abs(p) < 1e-9
    assert abs(r - angle) < 1e-9


def test_euler_zyx_gimbal_lock_safe():
    """asin argument is clamped to [-1, 1] so near-singular inputs don't crash."""
    # Near pitch=90° (sin(p) ≈ 1)
    q = np.array([np.cos(np.pi / 4 - 1e-7), 0.0, np.sin(np.pi / 4 - 1e-7), 0.0],
                 dtype=np.float64)
    y, p, r = ct.quat_wxyz_to_euler_zyx(q)
    # Should not raise, pitch should be close to π/2 but ≤ π/2
    assert abs(p) <= np.pi / 2 + 1e-9


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
