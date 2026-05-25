"""7 Layer-1 regression tests — drive ``FourierHandMapper`` with synthetic
canned poses and assert on the 11D / 22D output.

These tests catch root causes C5 / C6 / C7 from research/34. §3.1 in
under 5 seconds without Isaac Sim:

    * test_open_hand_outputs_zero       → C5 (rest pose pollution)
    * test_full_fist_outputs_curl       → C7 (mimic threshold too strict)
    * test_thumb_yaw_zero_at_rest       → C6 (thumb yaw midpoint bug)
    * test_point_index_isolation        → per-finger isolation
    * test_pinch_pose                   → thumb yaw + pitch + index together
    * test_left_right_symmetry          → mapper L/R asymmetry
    * test_pack_22d_signs_applied       → pack_22d sign flip correctness

Run with (from repo root)::

    PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_glove/validation/tests/test_finger_replay.py -v
"""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np
import pytest

from ust_ws.ust_hm_glove.validation._bootstrap import load_fourier_hand_mapper
from ust_ws.ust_hm_glove.validation.tools import synth_vmc


# Per-side joint slot indices — must match fourier_hand_mapper.
IDX_INDEX_PROX = 0
IDX_MIDDLE_PROX = 1
IDX_PINKY_PROX = 2
IDX_RING_PROX = 3
IDX_THUMB_YAW = 4
IDX_INDEX_INT = 5
IDX_MIDDLE_INT = 6
IDX_PINKY_INT = 7
IDX_RING_INT = 8
IDX_THUMB_PITCH = 9
IDX_THUMB_DIST = 10


@pytest.fixture(scope="module")
def fhm():
    """The module bootstrap loader returns the production fourier_hand_mapper."""
    return load_fourier_hand_mapper()


@pytest.fixture
def mapper(fhm):
    """Fresh mapper without rest-pose calibration (synthetic input is exact)."""
    return fhm.FourierHandMapper(
        proximal_scale=2.5,
        thumb_scale=2.5,
        use_tanh=True,
        vmc_subtract_rest=False,        # synthetic poses are exact, no cal
    )


def _drive_both_sides(mapper, pose_name: str) -> Tuple[np.ndarray, np.ndarray]:
    """Run the mapper on both sides and return (left_11, right_11)."""
    pose = synth_vmc.build_pose(pose_name)
    left = mapper.map_hand_vmc(pose, is_right=False)
    right = mapper.map_hand_vmc(pose, is_right=True)
    return np.asarray(left, dtype=np.float64), np.asarray(right, dtype=np.float64)


# ── 1. open hand → all outputs ~ zero (C5 detector) ─────────────────


def test_open_hand_outputs_zero(mapper):
    left, right = _drive_both_sides(mapper, "open_hand")
    # All four finger proximals must be < 0.05 rad (~3°).
    for j in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_PINKY_PROX, IDX_RING_PROX):
        assert abs(left[j]) < 0.05, f"L finger {j} not zero at rest: {left[j]:.4f}"
        assert abs(right[j]) < 0.05, f"R finger {j} not zero at rest: {right[j]:.4f}"
    # Thumb yaw / pitch / distal also ~zero
    assert abs(left[IDX_THUMB_YAW]) < 0.05
    assert abs(right[IDX_THUMB_YAW]) < 0.05
    assert abs(left[IDX_THUMB_PITCH]) < 0.05
    assert abs(right[IDX_THUMB_PITCH]) < 0.05


# ── 2. full fist → 4 finger proximals strong + intermediates mimic-filled (C7) ──


def test_full_fist_outputs_curl(mapper):
    left, right = _drive_both_sides(mapper, "full_fist")
    # All 4 finger proximals should reach > 0.5 rad.
    for j in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_PINKY_PROX, IDX_RING_PROX):
        assert left[j] > 0.5, f"L finger {j} not curled: {left[j]:.3f}"
        assert right[j] > 0.5, f"R finger {j} not curled: {right[j]:.3f}"
    # Intermediates should be either mimic-filled (≈ prox * 0.85) OR
    # directly filled by mapper (also large).  Either way, must be > 0.3.
    for prox_idx, int_idx in (
        (IDX_INDEX_PROX, IDX_INDEX_INT),
        (IDX_MIDDLE_PROX, IDX_MIDDLE_INT),
        (IDX_PINKY_PROX, IDX_PINKY_INT),
        (IDX_RING_PROX, IDX_RING_INT),
    ):
        assert left[int_idx] > 0.3, (
            f"L intermediate {int_idx} not filled: prox={left[prox_idx]:.3f} "
            f"int={left[int_idx]:.3f}"
        )
        assert right[int_idx] > 0.3, (
            f"R intermediate {int_idx} not filled: prox={right[prox_idx]:.3f} "
            f"int={right[int_idx]:.3f}"
        )
    # Thumb pitch should engage on full fist
    assert left[IDX_THUMB_PITCH] > 0.2, f"L thumb pitch idle on fist: {left[IDX_THUMB_PITCH]:.3f}"
    assert right[IDX_THUMB_PITCH] > 0.2, f"R thumb pitch idle on fist: {right[IDX_THUMB_PITCH]:.3f}"


# ── 3. thumb yaw zero at rest (C6 detector) ──────────────────────────


def test_thumb_yaw_zero_at_rest(mapper):
    """Regression for the 9.15 thumb-yaw midpoint bug.

    With pure rest input, thumb_yaw output must be ~0 (not the URDF range
    midpoint that would land at +0.25).
    """
    left, right = _drive_both_sides(mapper, "open_hand")
    assert abs(left[IDX_THUMB_YAW]) < 0.01, (
        f"thumb_yaw stuck at non-zero rest: {left[IDX_THUMB_YAW]:.4f}"
    )
    assert abs(right[IDX_THUMB_YAW]) < 0.01


# ── 4. point index → isolation: index ~0 while others curled ─────────


def test_point_index_isolation(mapper):
    left, right = _drive_both_sides(mapper, "point_index")
    # Index proximal stays low.
    assert left[IDX_INDEX_PROX] < 0.1, f"L index leaked: {left[IDX_INDEX_PROX]:.3f}"
    assert right[IDX_INDEX_PROX] < 0.1, f"R index leaked: {right[IDX_INDEX_PROX]:.3f}"
    # Middle / Ring / Little all > 0.5
    for j in (IDX_MIDDLE_PROX, IDX_PINKY_PROX, IDX_RING_PROX):
        assert left[j] > 0.5, f"L finger {j} should be curled: {left[j]:.3f}"
        assert right[j] > 0.5, f"R finger {j} should be curled: {right[j]:.3f}"


# ── 5. pinch → thumb yaw + thumb pitch + index all engaged ──────────


def test_pinch_pose(mapper):
    left, right = _drive_both_sides(mapper, "pinch_thumb_index")
    # Thumb yaw > 0
    assert left[IDX_THUMB_YAW] > 0.05, f"L thumb yaw not engaged: {left[IDX_THUMB_YAW]:.3f}"
    assert right[IDX_THUMB_YAW] > 0.05, f"R thumb yaw not engaged: {right[IDX_THUMB_YAW]:.3f}"
    # Thumb pitch > 0.2
    assert left[IDX_THUMB_PITCH] > 0.2
    assert right[IDX_THUMB_PITCH] > 0.2
    # Index proximal in mid-range
    assert left[IDX_INDEX_PROX] > 0.3
    assert right[IDX_INDEX_PROX] > 0.3


# ── 6. L/R symmetry: same input → same output across both hands ─────


def test_left_right_symmetry(mapper):
    """Synthetic poses are bilaterally symmetric, so L/R outputs must match."""
    for pose_name in ("full_fist", "point_index", "pinch_thumb_index"):
        left, right = _drive_both_sides(mapper, pose_name)
        # Pose data is symmetric — mapper outputs must be ≤ small numerical tol.
        diff = np.abs(left - right)
        assert diff.max() < 0.01, (
            f"L/R asymmetric on {pose_name}: max diff={diff.max():.4f}, "
            f"left={left}, right={right}"
        )


# ── 7. pack_22d sign convention applied correctly ────────────────────


def test_pack_22d_signs_applied(mapper, fhm):
    """The 22D packed output must apply PACK_22D_SIGNS — most slots negate
    the unsigned magnitudes from the per-side mapper.
    """
    left, right = _drive_both_sides(mapper, "full_fist")
    packed = fhm.pack_22d(left, right)
    assert packed.shape == (22,)
    # Slot 0 = L_index_proximal (sign = -1), comes from left[IDX_INDEX_PROX] > 0
    assert packed[0] < 0, f"L_index_prox should be negative after sign flip, got {packed[0]:.3f}"
    # Slot 5 = R_index_proximal (sign = -1)
    assert packed[5] < 0, f"R_index_prox should be negative, got {packed[5]:.3f}"
    # Slot 14 = L_thumb_proximal_pitch (sign = +1)
    assert packed[14] > 0, f"L_thb_pitch should be positive, got {packed[14]:.3f}"
    # Slot 19 = R_thumb_proximal_pitch (sign = +1)
    assert packed[19] > 0, f"R_thb_pitch should be positive, got {packed[19]:.3f}"


# ── 8. Determinism ──────────────────────────────────────────────────


def test_determinism(fhm):
    """Same input twice → same mapper output."""
    pose = synth_vmc.build_pose("full_fist")
    m1 = fhm.FourierHandMapper(proximal_scale=2.5, vmc_subtract_rest=False)
    m2 = fhm.FourierHandMapper(proximal_scale=2.5, vmc_subtract_rest=False)
    out1 = m1.map_hand_vmc(pose, is_right=False)
    out2 = m2.map_hand_vmc(pose, is_right=False)
    np.testing.assert_allclose(out1, out2, atol=1e-9)


# ── 9. JSONL roundtrip via replay_vmc → mapper feeder ───────────────


def test_replay_to_mapper_jsonl(tmp_path):
    """End-to-end Layer-1: synth pose → JSONL fixture → replay_vmc feeder
    → mapper output JSONL.  Verifies the full Layer-1 plumbing.
    """
    fixture = tmp_path / "full_fist.vmc.jsonl"
    n_pkts = synth_vmc.write_jsonl_fixture(
        "full_fist", fixture, n_frames=5, frame_interval_us=50_000,
    )
    assert n_pkts == 150

    mapper_out = tmp_path / "full_fist.mapper.jsonl"
    from ust_ws.ust_hm_glove.validation.tools import replay_vmc
    records = list(replay_vmc.iter_records(fixture))
    n_frames = replay_vmc.feed_to_mapper_jsonl(
        records, output=mapper_out,
        proximal_scale=2.5, vmc_subtract_rest=False,
        frame_window_us=30_000,
    )
    assert n_frames >= 1, f"expected ≥1 frame, got {n_frames}"
    assert mapper_out.exists()
    # Sanity-check first frame: index proximal should be > 0.5 rad (full fist)
    import json
    first = json.loads(mapper_out.read_text(encoding="utf-8").splitlines()[0])
    assert len(first["packed_22"]) == 22
    assert abs(first["packed_22"][0]) > 0.5, (
        f"first frame's L_idx_prox magnitude too low: {first['packed_22'][0]:.3f}"
    )
