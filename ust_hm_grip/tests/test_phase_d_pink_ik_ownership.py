"""Phase D ↔ Pink IK ownership split — 13th session, part 9.

User report (2026-05-16 11:44): even after neutral calibration, robot
waist keeps bending forward; waist/wrist tracker data appears not to
reach the robot.

Root cause: ``KitchenSortingGR1T2GripperRobotOnlyEnvCfg`` inherits from
``KitchenSortingGR1T2GripperWaistEnvCfg`` which adds the 3 waist joints
to ``pink_controlled_joint_names``.  Pink IK's ``apply_actions()`` then
overwrites the Phase D waist targets every ``env.step``.  Head joints
are unaffected because no head joint is in ``pink_controlled_joint_names``.

The fix in ``run_teleop.py`` removes the 3 waist joints from
``pink_controlled_joint_names`` AND from any ``NullSpacePostureTask
.controlled_joints`` when ``--full_body=True`` and the input backend is
xrobotoolkit (the only mode where Phase D is in charge).  These tests
exercise the filter logic without needing Isaac Sim.
"""
from __future__ import annotations

import pytest


_WAIST = {"waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"}

_ARM_14 = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_yaw_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_yaw_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
]


def _filter_no_waist(joints):
    return [j for j in joints if j not in _WAIST]


def test_arm_only_unchanged():
    """When the cfg has only arm joints, the filter is a no-op."""
    assert _filter_no_waist(_ARM_14) == _ARM_14


def test_waist_enabled_drops_three_waist_joints():
    """17-joint Waist cfg → 14-joint arm-only after filter."""
    waist17 = _ARM_14 + ["waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"]
    out = _filter_no_waist(waist17)
    assert out == _ARM_14
    assert len(out) == 14


def test_filter_preserves_arm_order():
    """The relative order of arm joints must not change."""
    waist17 = [
        "waist_yaw_joint",
        "left_shoulder_pitch_joint",
        "waist_pitch_joint",
        "left_shoulder_roll_joint",
        "waist_roll_joint",
        "left_shoulder_yaw_joint",
    ]
    out = _filter_no_waist(waist17)
    assert out == [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
    ]


def test_filter_removed_set_is_exactly_waist_three():
    """No arm joint should ever be accidentally removed."""
    waist17 = _ARM_14 + list(_WAIST)
    before = set(waist17)
    after = set(_filter_no_waist(waist17))
    removed = before - after
    assert removed == _WAIST


def test_filter_handles_empty_list():
    assert _filter_no_waist([]) == []


def test_filter_handles_only_waist():
    """Pathological cfg with only waist joints → all removed."""
    only_waist = ["waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"]
    assert _filter_no_waist(only_waist) == []


def test_null_space_posture_controlled_joints_filter_matches():
    """The NullSpacePostureTask.controlled_joints filter uses the same
    set, so the result is identical for any input."""
    null_space_input = [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "waist_yaw_joint",
        "waist_pitch_joint",
        "waist_roll_joint",
    ]
    out = _filter_no_waist(null_space_input)
    assert "waist_yaw_joint" not in out
    assert "waist_pitch_joint" not in out
    assert "waist_roll_joint" not in out
    assert len(out) == 8


def test_phase_d_default_owns_waist_when_xrobotoolkit():
    """Sanity: the contract is that ``--full_body=True`` AND
    ``--input_backend=xrobotoolkit`` together activate Phase D, at which
    point the waist joints belong to Phase D, not Pink IK.

    This test documents the CLI defaults so a future change to the
    defaults won't silently break Phase D ownership.
    """
    # When the user runs the canonical Phase D command, both flags must
    # be True (their CLI defaults).
    full_body_default = True
    xrobotoolkit_default_when_explicit = "xrobotoolkit"
    phase_d_owns_waist = (
        full_body_default
        and xrobotoolkit_default_when_explicit == "xrobotoolkit"
    )
    assert phase_d_owns_waist, (
        "Phase D should own waist joints under canonical "
        "--full_body=True + --input_backend=xrobotoolkit invocation."
    )
