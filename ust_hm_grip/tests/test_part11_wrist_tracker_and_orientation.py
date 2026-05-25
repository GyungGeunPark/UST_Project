"""Part 11 regression — wrist trackers exposed + controller orientation.

User report (after part 10): waist+head OK; wrist-tracker data not
exposed to the retargeter; controller rotation doesn't visibly rotate
the robot wrist.

Fixes:
  * Add SMPL L_Wrist (idx 20) + R_Wrist (idx 21) to
    ``_DEFAULT_BODY_ROLE_MAP`` so they appear in snapshot's ``trackers``.
  * Bump Pink IK ``FrameTask.orientation_cost`` from 1.0 to 6.0 so the
    wrist 3-DoF chain actually tracks the controller orientation target
    (was effectively ignored at 1/8 the position weight).
"""
from __future__ import annotations

import numpy as np
import pytest


# ── role-map mapping ───────────────────────────────────────────────────


def test_role_map_includes_left_wrist():
    from ust_ws.ust_hm_grip.teleop.xrobo_sampler import _DEFAULT_BODY_ROLE_MAP
    assert 20 in _DEFAULT_BODY_ROLE_MAP
    assert _DEFAULT_BODY_ROLE_MAP[20] == "left_wrist"


def test_role_map_includes_right_wrist():
    from ust_ws.ust_hm_grip.teleop.xrobo_sampler import _DEFAULT_BODY_ROLE_MAP
    assert 21 in _DEFAULT_BODY_ROLE_MAP
    assert _DEFAULT_BODY_ROLE_MAP[21] == "right_wrist"


def test_role_map_size_grew_to_five_entries():
    """Pelvis + 2 forearms + 2 wrists = 5 active body slots."""
    from ust_ws.ust_hm_grip.teleop.xrobo_sampler import _DEFAULT_BODY_ROLE_MAP
    assert len(_DEFAULT_BODY_ROLE_MAP) == 5
    assert set(_DEFAULT_BODY_ROLE_MAP.values()) == {
        "waist", "left_forearm", "right_forearm",
        "left_wrist", "right_wrist",
    }


def test_role_map_ankles_still_excluded():
    """Regression — ankles must stay out (drift / wobble per part 2)."""
    from ust_ws.ust_hm_grip.teleop.xrobo_sampler import _DEFAULT_BODY_ROLE_MAP
    assert 7 not in _DEFAULT_BODY_ROLE_MAP    # LEFT_ANKLE
    assert 8 not in _DEFAULT_BODY_ROLE_MAP    # RIGHT_ANKLE


# ── sampler emits wrist trackers when body data available ─────────────


def _fake_xrt_module(body_avail=True):
    """Build a fake ``xrobotoolkit_sdk`` with all 24 SMPL joints live."""
    import types
    fake = types.SimpleNamespace()

    def make_pose(idx):
        # Distinct non-zero pose per index so the sampler can't
        # accidentally pick the same data for multiple joints.
        return [float(idx) * 0.01, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    poses = np.array([make_pose(i) for i in range(24)], dtype=np.float64)

    fake.init = lambda: None
    fake.close = lambda: None
    fake.get_headset_pose = lambda: np.array(
        [0.0, 1.4, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
    )
    fake.get_left_controller_pose = lambda: np.array(
        [0.0, 1.0, 0.3, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
    )
    fake.get_right_controller_pose = lambda: np.array(
        [0.0, 1.0, -0.3, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
    )
    fake.get_left_trigger = lambda: 0.0
    fake.get_left_grip = lambda: 0.0
    fake.get_right_trigger = lambda: 0.0
    fake.get_right_grip = lambda: 0.0
    fake.is_body_data_available = lambda: body_avail
    fake.get_body_joints_pose = lambda: poses
    fake.num_motion_data_available = lambda: 0
    fake.get_motion_tracker_serial_numbers = lambda: []
    return fake


def test_sampler_emits_wrist_trackers(monkeypatch):
    import sys
    import time
    import importlib
    fake = _fake_xrt_module()
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake)
    from ust_ws.ust_hm_grip.teleop import xrobo_sampler
    importlib.reload(xrobo_sampler)
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", fake)
    sampler = xrobo_sampler.XRoboSampler(
        rate_hz=120.0, enable_body=True, debug=False,
    )
    sampler.start()
    try:
        # Wait briefly for the poll thread to tick.
        for _ in range(20):
            time.sleep(0.02)
            snap = sampler.snapshot()
            if snap["trackers"]:
                break
        snap = sampler.snapshot()
        assert "left_wrist" in snap["trackers"]
        assert "right_wrist" in snap["trackers"]
        assert "waist" in snap["trackers"]
    finally:
        sampler.stop()


# ── env_cfg orientation_cost bump ──────────────────────────────────────


def test_env_cfg_orientation_cost_bumped_to_6():
    """Static-source check: Pink IK FrameTask orientation_cost must be
    >= 6.0 so controller rotation visibly drives the wrist chain."""
    import pathlib
    import re
    src = (
        pathlib.Path(__file__).resolve().parents[1]
        / "kitchen_sorting_gr1t2_gripper_env_cfg.py"
    ).read_text(encoding="utf-8")
    # Strip comments so docstrings / explanatory comments mentioning the
    # OLD value of 1.0 don't trip the regression guard.
    code_only_lines = []
    for line in src.splitlines():
        stripped = line.split("#", 1)[0]
        code_only_lines.append(stripped)
    code_only = "\n".join(code_only_lines)
    # Match orientation_cost= followed by a numeric literal.
    matches = re.findall(r"orientation_cost=([\d.]+)", code_only)
    assert matches, "no orientation_cost= kwarg found in env_cfg source"
    values = [float(v) for v in matches]
    assert min(values) >= 6.0, (
        f"orientation_cost values must all be ≥ 6.0 after part 11; "
        f"found {values}"
    )


def test_env_cfg_both_sides_bumped():
    """Both left + right FrameTask must be bumped, not just one."""
    import pathlib
    src = (
        pathlib.Path(__file__).resolve().parents[1]
        / "kitchen_sorting_gr1t2_gripper_env_cfg.py"
    ).read_text(encoding="utf-8")
    # Count occurrences of orientation_cost=6.0 (one per FrameTask).
    assert src.count("orientation_cost=6.0") >= 2, (
        "expected orientation_cost=6.0 in BOTH FrameTasks (left + right)"
    )
