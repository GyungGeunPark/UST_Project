"""Sanity checks for ``tools.synth_vmc`` — the synthetic pose generator
that the rest of Layer-1 depends on.

Verifies:
  * All 6 canonical poses produce 30 finger bones
  * Identity / non-identity quat counts match expectations
  * write_jsonl_fixture roundtrips cleanly
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ust_ws.ust_hm_glove.validation.tools import synth_vmc


def test_pose_names_and_bone_count():
    assert set(synth_vmc.POSE_NAMES) == {
        "open_hand", "full_fist", "point_index",
        "pinch_thumb_index", "ok_sign", "thumb_oppose",
    }
    # 2 sides × (4 fingers × 3 parts + 3 thumb parts) = 30
    assert len(synth_vmc.BONE_NAMES_ALL) == 30


def test_open_hand_is_identity():
    pose = synth_vmc.build_pose("open_hand")
    assert len(pose) == 30
    for name, q in pose.items():
        assert q == (0.0, 0.0, 0.0, 1.0), f"{name} not identity: {q}"


def test_full_fist_has_nonidentity_finger_proximals():
    pose = synth_vmc.build_pose("full_fist")
    # All proximal bones should be non-identity (qw < 0.95).
    for side in ("Left", "Right"):
        for finger in ("Index", "Middle", "Ring", "Little"):
            q = pose[f"{side}{finger}Proximal"]
            assert q[3] < 0.95, f"{side}{finger}Proximal qw={q[3]:.3f} too close to identity"


def test_thumb_oppose_only_yaw():
    pose = synth_vmc.build_pose("thumb_oppose")
    # Post 9.19 C8 patch: synth encodes thumb opposition on X-axis to match
    # actual UDCAP VMC broadcast (see research/34. §3.1 C8).
    for side in ("Left", "Right"):
        qx, qy, qz, qw = pose[f"{side}ThumbProximal"]
        assert abs(qx) > 0.05, f"{side}ThumbProximal qx should be non-zero"
        assert abs(qy) < 1e-6, f"{side}ThumbProximal qy should be 0"
        assert abs(qz) < 1e-6, f"{side}ThumbProximal qz should be 0"
    # Fingers stay identity.
    for side in ("Left", "Right"):
        for finger in ("Index", "Middle", "Ring", "Little"):
            assert pose[f"{side}{finger}Proximal"] == (0.0, 0.0, 0.0, 1.0)


def test_point_index_has_index_identity_others_flexed():
    pose = synth_vmc.build_pose("point_index")
    for side in ("Left", "Right"):
        # Index identity
        assert pose[f"{side}IndexProximal"] == (0.0, 0.0, 0.0, 1.0)
        # Other 3 fingers flexed (qw < 0.95)
        for finger in ("Middle", "Ring", "Little"):
            q = pose[f"{side}{finger}Proximal"]
            assert q[3] < 0.95, f"{side}{finger}Proximal not flexed: qw={q[3]:.3f}"


def test_axis_angle_quat_basic():
    q = synth_vmc.axis_angle_quat("x", np.pi / 2)
    # 90° around X: qx = sin(45°) ≈ 0.7071, qw = cos(45°) ≈ 0.7071
    assert abs(q[0] - np.sin(np.pi / 4)) < 1e-6
    assert abs(q[3] - np.cos(np.pi / 4)) < 1e-6


def test_pose_as_packets_count():
    pose = synth_vmc.build_pose("full_fist")
    packets = synth_vmc.pose_as_packets(pose)
    # One packet per bone
    assert len(packets) == 30
    for rec in packets:
        assert rec["address"] == "/VMC/Ext/Bone/Pos"
        # [name, px, py, pz, qx, qy, qz, qw] = 8 args
        assert len(rec["args"]) == 8


def test_write_jsonl_fixture(tmp_path: Path):
    out = tmp_path / "test_full_fist.vmc.jsonl"
    n = synth_vmc.write_jsonl_fixture(
        "full_fist", out, n_frames=3, frame_interval_us=50_000,
    )
    # 3 frames × 30 bones = 90 records
    assert n == 90
    assert out.exists()
    lines = out.read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 90
    rec0 = json.loads(lines[0])
    assert rec0["address"] == "/VMC/Ext/Bone/Pos"
    assert isinstance(rec0["t_us"], int)
