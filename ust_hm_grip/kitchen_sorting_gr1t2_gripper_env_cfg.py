"""Kitchen Sorting environment configs for GR1T2 + 2-finger gripper (option B).

This module is the post-UDCAP migration counterpart of
``ust_hm_glove/kitchen_sorting_gr1t2_env_cfg.py``.  The robot model
is unchanged at the joint chain level (Fourier GR1T2) but each hand is
replaced by a 2-finger parallel gripper attached to the wrist via a
fixed joint (see ``isaac_file/build_gripper_usd.py`` for the USD build
script).  This collapses the action layout to 16-D::

    [0:3]    L wrist position  (base_link frame)
    [3:7]    L wrist quaternion (wxyz)
    [7:10]   R wrist position
    [10:14]  R wrist quaternion (wxyz)
    [14]     L gripper command  (-1 = close, +1 = open)
    [15]     R gripper command  (-1 = close, +1 = open)

Pink IK now controls only the 14 arm joints (no hand DoFs).  Gripper
control flows through ``BinaryJointPositionAction`` and is independent
of Pink IK.

The gripper USD must exist at ``isaac_file/GR1T2_with_gripper.usd`` —
run ``isaac_file/build_gripper_usd.py`` once (inside an Isaac Sim
Python environment) to materialise it from the stock GR1T2 asset.

Env classes (mirrors ust_hm_glove (was ust_fourier_260421 pre-9.36)):

* :class:`KitchenSortingGR1T2GripperEnvCfg`              — base
* :class:`KitchenSortingGR1T2GripperWaistEnvCfg`         — Pink IK + 3 waist joints
* :class:`KitchenSortingGR1T2GripperVisionEnvCfg`        — + cameras
* :class:`KitchenSortingGR1T2GripperMonitorEnvCfg`       — PC-only view
* :class:`KitchenSortingGR1T2GripperVREnvCfg`            — long episodes
* :class:`KitchenSortingGR1T2GripperDataCollectEnvCfg`   — dataset recording
* :class:`KitchenSortingGR1T2GripperRobotOnlyEnvCfg`     — empty scene
"""

from __future__ import annotations

# CRITICAL: apply the osqp 0.6 ↔ qpsolvers 4.x compat shim BEFORE any
# pink / qpsolvers module-level import.  Without this, Pink IK has no QP
# solver at runtime and the robot stays frozen.
from ust_ws.ust_hm_grip.teleop import _osqp_compat as _osqp_compat  # noqa: F401
_osqp_compat.apply()

# Also patch isaaclab's PinkInverseKinematicsAction so it handles
# ``hand_joint_dim == 0`` correctly.  See the module docstring for the
# slicing bug that otherwise produces a (1, 31) action tensor on a
# 17-controlled-joint robot.  No-op until isaaclab.envs is importable
# (apply() returns early in that case); we re-call after AppLauncher
# below so the patch is guaranteed to land before the first env.step.
from ust_ws.ust_hm_grip.teleop import _pink_hand_dim_zero_patch  # noqa: F401
_pink_hand_dim_zero_patch.apply()

import os
import tempfile
from typing import Tuple

import torch

import carb
from pink.tasks import FrameTask

import isaaclab.controllers.utils as ControllerUtils
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.controllers.pink_ik import (
    NullSpacePostureTask,
    PinkIKControllerCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg
from isaaclab.devices.openxr import XrCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.actions_cfg import BinaryJointPositionActionCfg
from isaaclab.envs.mdp.actions.pink_actions_cfg import PinkInverseKinematicsActionCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

import isaaclab.envs.mdp as base_mdp

# Re-use the G1 scene + MDP components — they are robot-agnostic.
from ust_ws.ust_260220.kitchen_sorting_env_cfg import (
    EventCfg,
    KitchenSortingSceneCfg,
    KitchenSortingUSDSceneCfg,
    KitchenSortingVisionSceneCfg,
    RewardsCfg,
    TerminationsCfg,
)
from ust_ws.ust_260220.mdp import observations as sorting_obs
from ust_ws.ust_260220 import mdp as sorting_mdp

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab_assets.robots.fourier import GR1T2_HIGH_PD_CFG  # isort: skip


__all__ = [
    "GRIPPER_JOINT_NAMES_LEFT",
    "GRIPPER_JOINT_NAMES_RIGHT",
    "GR1T2_PINK_CONTROLLED_ARM_JOINTS",
    "GR1T2_WAIST_JOINT_NAMES",
    "GR1T2_GRIPPER_FRAME_NAMES",
    "GR1T2_EEF_LINK_NAMES",
    "GripperActionsCfg",
    "KitchenSortingGR1T2GripperEnvCfg",
    "KitchenSortingGR1T2GripperWaistEnvCfg",
    "KitchenSortingGR1T2GripperVisionEnvCfg",
    "KitchenSortingGR1T2GripperMonitorEnvCfg",
    "KitchenSortingGR1T2GripperVREnvCfg",
    "KitchenSortingGR1T2GripperDataCollectEnvCfg",
    "KitchenSortingGR1T2GripperRobotOnlyEnvCfg",
]


# ── Joint name conventions ─────────────────────────────────────────────
# These names are emitted by ``isaac_file/build_gripper_usd.py``.  The
# regex pattern is symmetric so a single ``BinaryJointPositionAction`` per
# side can match both finger joints.
GRIPPER_JOINT_NAMES_LEFT = [
    "left_gripper_finger_left_joint",
    "left_gripper_finger_right_joint",
]
GRIPPER_JOINT_NAMES_RIGHT = [
    "right_gripper_finger_left_joint",
    "right_gripper_finger_right_joint",
]

GR1T2_PINK_CONTROLLED_ARM_JOINTS = [
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

GR1T2_WAIST_JOINT_NAMES = [
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
]

# Gripper-base link names emitted by ``build_robotiq_usd.py`` (option A,
# 43. robotiq_2f85_optionA_migration_design_guide.md).  The Robotiq 2F-85
# subtree is attached under ``{side}_robotiq_arg2f_85`` and the base_link
# is at the gripper mount face.  A separate ``{side}_gripper_tcp_link`` is
# fixed-jointed +0.150 m along the gripper's local Z so Pink IK can target
# the fingertip midpoint directly.
GR1T2_GRIPPER_FRAME_NAMES = (
    "left_gripper_tcp_link",
    "right_gripper_tcp_link",
)

# Observation EEF link names — TCP link so EEF telemetry tracks where the
# gripper actually closes (fingertip midpoint).
GR1T2_EEF_LINK_NAMES = {
    "left":  "left_gripper_tcp_link",
    "right": "right_gripper_tcp_link",
}

# Robotiq 2F-85 joint names — ``build_robotiq_usd.py`` flattens the Robotiq
# stock USD into the humanoid stage and prefixes every gripper joint with
# ``{side}_`` so Isaac Lab's name-based ``find_joints`` can target each side
# independently and the URDF converter doesn't trip on duplicate leaf names.
#
# The lead joint drives the 4-bar linkage; the 5 followers (per gripper) are
# coupled to the lead by ``PhysxMimicJointAPI``.  Stock Robotiq USD relies
# on the mimic constraint alone (no drives on followers), but Isaac Sim 5.1
# has a known issue where the mimic constraint registration sometimes
# silently no-ops for merged articulations — the followers then float free
# and gravity warps the gripper into a chain-like shape.
#
# Fallback (per design-guide #43 §6.6): actuate all 6 joints with explicit
# per-joint targets, applying the correct gearing sign on each follower
# (gearing ``-1`` followers need negative target on close).  ``build_robotiq_usd.py``
# also widens the outer-knuckle joint limits from stock ``[0°, 47°]`` to
# symmetric ``[-47°, 47°]`` so the negative target is reachable.
ROBOTIQ_LEAD_JOINT_LEFT  = "left_finger_joint"
ROBOTIQ_LEAD_JOINT_RIGHT = "right_finger_joint"
ROBOTIQ_CLOSE_RAD  = 0.785  # canonical close angle (community standard)

# All 6 revolute joints per gripper, in the order the Robotiq stock USD
# defines them.  Each tuple is ``(joint_name, gearing)`` where ``gearing``
# is the PhysxMimicJointAPI gearing (lead = +1 by definition).  Close
# commands multiply ``ROBOTIQ_CLOSE_RAD`` by the gearing.
# 12th session — measured signs corrected.  test_robotiq_close.py with the
# fixed USD (maxJointVelocity=10000, armature=0.01, lead K=200) showed the
# 4-bar linkage drives followers to these actual final positions when the
# lead reaches +40°:
#   left_right_outer_knuckle_joint        ≈ +40°  → gear = +1 (was -1)
#   left_right_inner_finger_joint         ≈ +40°  → gear = +1 (was -1)
#   left_right_inner_finger_knuckle_joint ≈ -40°  → gear = -1 (was +1)
#   left_left_inner_finger_knuckle_joint  ≈ -40°  → gear = -1 (was +1)
#   left_left_inner_finger_joint          ≈ -40°  → gear = -1 (was +1)
# Stock USD's PhysxMimicJointAPI gearing values (-1 for outer_knuckle,
# +1 for inner_finger_*) were authored for a different axis convention
# and PhysX 5.1 doesn't strongly enforce them anyway — the mechanical
# 4-bar linkage drives the actual motion.  These gear values are only
# consumed by test_robotiq_close.py and diagnose_robotiq_attach.py; the
# lead-only BinaryJointPositionAction below doesn't use them.
_ROBOTIQ_LEFT_JOINTS_WITH_GEAR = [
    ("left_finger_joint",                     +1.0),  # lead (drive)
    ("left_right_outer_knuckle_joint",        +1.0),  # rotZ mimic, measured
    ("left_right_inner_finger_joint",         +1.0),  # rotX mimic, measured
    ("left_right_inner_finger_knuckle_joint", -1.0),  # rotX mimic, measured
    ("left_left_inner_finger_knuckle_joint",  -1.0),  # rotX mimic, measured
    ("left_left_inner_finger_joint",          -1.0),  # rotX mimic, measured
]
_ROBOTIQ_RIGHT_JOINTS_WITH_GEAR = [
    ("right_finger_joint",                     +1.0),
    ("right_right_outer_knuckle_joint",        +1.0),
    ("right_right_inner_finger_joint",         +1.0),
    ("right_right_inner_finger_knuckle_joint", -1.0),
    ("right_left_inner_finger_knuckle_joint",  -1.0),
    ("right_left_inner_finger_joint",          -1.0),
]
ROBOTIQ_ALL_JOINTS_LEFT  = [j for j, _ in _ROBOTIQ_LEFT_JOINTS_WITH_GEAR]
ROBOTIQ_ALL_JOINTS_RIGHT = [j for j, _ in _ROBOTIQ_RIGHT_JOINTS_WITH_GEAR]


def _robotiq_open_targets(joints_with_gear) -> dict[str, float]:
    return {name: 0.0 for name, _ in joints_with_gear}


def _robotiq_close_targets(joints_with_gear) -> dict[str, float]:
    return {name: gear * ROBOTIQ_CLOSE_RAD for name, gear in joints_with_gear}


# ── Gripper-equipped GR1T2 articulation ────────────────────────────────
DEFAULT_GRIPPER_USD_PATH = (
    "./ust_ws/ust_hm_grip/isaac_file/GR1T2_with_robotiq.usd"
)


def _resolve_usd_path() -> str:
    """Return absolute path to the gripper-equipped GR1T2 USD.

    Falls back to the stock GR1T2 USD when the gripper USD doesn't exist
    yet (e.g. running a smoke test before invoking ``build_gripper_usd.py``).
    The fallback prints a warning so the user knows that they're running
    the un-modified humanoid-hand asset.
    """
    abs_path = os.path.abspath(DEFAULT_GRIPPER_USD_PATH)
    if os.path.exists(abs_path):
        return abs_path
    fallback = GR1T2_HIGH_PD_CFG.spawn.usd_path
    print(
        f"[ust_hm_grip] WARNING — gripper USD not found at {abs_path!r}.\n"
        f"  Falling back to stock GR1T2 USD ({fallback!r}).\n"
        f"  Run isaac_file/build_gripper_usd.py inside Isaac Sim Python\n"
        f"  to materialise the gripper-equipped USD before live teleop."
    )
    return fallback


def _gripper_robot_articulation() -> ArticulationCfg:
    """Return the GR1T2-with-gripper articulation cfg.

    Notes:
    * Spawn rotation = identity (research/36 §3.1 carry-over from 9.27).
    * Hand actuator entries are removed because the USD no longer has L_*/R_*
      joints once the Fourier-hand chain is replaced by gripper joints.
    * Gripper actuator stiffness/damping copied from Franka panda_finger
      (effort_limit_sim=200, stiffness=2e3, damping=1e2) which is well-
      matched to a 0.04 m stroke parallel gripper.
    """
    actuators = dict(GR1T2_HIGH_PD_CFG.actuators)
    actuators.pop("left-hand", None)
    actuators.pop("right-hand", None)
    # 13th session, part 10 — head actuator restoration.  ``GR1T2_HIGH_PD_CFG``
    # (isaaclab_assets/robots/fourier.py:129) uses ``replace(actuators={...})``
    # which OVERWRITES the entire actuators dict from the base ``GR1T2_CFG``,
    # so the original ``"head"`` entry (head_.* joints, default K/D) is
    # silently dropped.  Without an actuator the implicit solver has no PD
    # to act on the joint, so ``set_joint_position_target`` writes the
    # Phase D head target but the joint never moves.  Symptom: HMD rotation
    # is captured + delta euler computed + target written + log shows
    # "drives robot head_yaw/pitch/roll" — yet robot head sits still.
    # Add an explicit ``head`` actuator matching the trunk gains so HMD
    # rotation drives the three head joints (yaw / pitch / roll).
    actuators["head"] = ImplicitActuatorCfg(
        joint_names_expr=["head_.*"],
        effort_limit_sim=50.0,
        velocity_limit_sim=20.0,
        stiffness=300.0,
        damping=20.0,
        armature=0.005,
    )
    # Robotiq 2F-85 actuators — drive all 6 revolute joints per gripper
    # (design-guide #43 §6.6 fallback for Isaac Sim 5.1 mimic-joint known
    # issue).  Driving every joint individually decouples behaviour from
    # the unreliable PhysxMimicJointAPI registration: the followers are
    # held to their target angles independently of whether the mimic
    # constraint registered.  The mimic schemas are kept in the USD as a
    # secondary stabiliser.
    #
    # PD values follow Issue #3347 / Disc #2665 safe zone (low stiffness
    # + moderate damping; high stiffness triggers a wrist-rotation
    # artifact in the merged articulation).
    #
    # 2026-05-13 (9th session) -- lead-only drive architecture.
    # The Robotiq 2F-85 is a closed 4-bar kinematic linkage: when the
    # lead joint rotates, the 5 follower joints are mechanically and
    # kinematically constrained by the link geometry + PhysxMimicJointAPI.
    # Earlier sessions drove all 6 joints with explicit PD (design-guide
    # #43 sec 6.6 fallback) because Isaac Sim 5.1 had a known issue
    # where the mimic constraint sometimes failed to register on merged
    # articulations.
    #
    # E2E close test (`test_robotiq_close`, 9th session) showed that
    # 6-joint driving combined with the closed loop + mimic creates a
    # solver instability: follower joints diverged to >100deg and even
    # >10000deg within 2 seconds of CLOSE command, regardless of PD
    # gains (tried K=10/D=80, K=50/D=5, K=200/D=20, K=200/D=80 --
    # all produced runaway).  The independent per-joint drives fight
    # the linkage constraints because their targets (gear*lead) cannot
    # exactly satisfy the closed-loop geometry at every step of the
    # transient, so accumulated error compounds.
    #
    # The fix: drive ONLY the lead joint (`{side}_finger_joint`) with
    # K=200 / D=20 (slow-pole rate 10 rad/s, tau~0.1s -> reaches close
    # in ~0.3s).  Followers get a passive ImplicitActuator with K=0 +
    # mild damping (D=2.0): no position target enforcement, just
    # velocity damping so they don't oscillate on their own.  The 5
    # follower joints' motion is then driven *exclusively* by the
    # 4-bar linkage's kinematic constraint + PhysxMimicJointAPI.
    #
    # This matches the original Robotiq stock USD design intent (the
    # stock asset only drove the lead).  We keep the BinaryJointPosition
    # action targeting only the lead joint so the rest of the chain
    # is unconstrained by position targets.
    # 12th session — second tuning iteration.  First attempt (K=800,
    # D=40, effort=500, vel=20): lead overshot to +135° in 30 steps then
    # bounced because K*err_max=628 > effort_limit → output clamps →
    # bang-bang.  Solution: keep K*err_max strictly < effort_limit so
    # the implicit solver stays in linear-PD regime.
    #
    # 13th session — speed up close (user request).  Bumped K from 200
    # to 400 (still in linear regime: K*err_max = 400*0.785 = 314 N·m
    # < effort_limit=500, no clamp).  Damping unchanged at 40, so the
    # time constant τ = D/K halves from 0.2s to 0.1s.  Close target
    # (+0.785 rad) is now reached in ~5τ = 0.5s instead of 1.0s, while
    # remaining well-damped (followers' 4-bar linkage absorbs the
    # transient via D=20 on followers).
    # Followers stay K=0 with D=20 (mild damping that absorbs the
    # 4-bar linkage's transient oscillations).
    # USD-side maxForce=500 in build_robotiq_usd.py provides headroom.
    actuators["robotiq-left-lead"] = ImplicitActuatorCfg(
        joint_names_expr=[ROBOTIQ_LEAD_JOINT_LEFT],
        effort_limit_sim=500.0,
        velocity_limit_sim=1000.0,
        stiffness=400.0,    # 13th session: 200 → 400 (twice as fast)
        damping=40.0,
    )
    actuators["robotiq-left-followers"] = ImplicitActuatorCfg(
        joint_names_expr=[j for j in ROBOTIQ_ALL_JOINTS_LEFT
                          if j != ROBOTIQ_LEAD_JOINT_LEFT],
        effort_limit_sim=100.0,
        velocity_limit_sim=1000.0,
        stiffness=0.0,
        damping=20.0,
    )
    actuators["robotiq-right-lead"] = ImplicitActuatorCfg(
        joint_names_expr=[ROBOTIQ_LEAD_JOINT_RIGHT],
        effort_limit_sim=500.0,
        velocity_limit_sim=1000.0,
        stiffness=400.0,    # 13th session: 200 → 400 (twice as fast)
        damping=40.0,
    )
    actuators["robotiq-right-followers"] = ImplicitActuatorCfg(
        joint_names_expr=[j for j in ROBOTIQ_ALL_JOINTS_RIGHT
                          if j != ROBOTIQ_LEAD_JOINT_RIGHT],
        effort_limit_sim=100.0,
        velocity_limit_sim=1000.0,
        stiffness=0.0,
        damping=20.0,
    )

    return GR1T2_HIGH_PD_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        spawn=GR1T2_HIGH_PD_CFG.spawn.replace(usd_path=_resolve_usd_path()),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.93),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={
                ".*_shoulder_pitch_joint": 0.0,
                ".*_shoulder_roll_joint": 0.0,
                ".*_shoulder_yaw_joint": 0.0,
                ".*_elbow_pitch_joint": 0.0,
                ".*_wrist_yaw_joint": 0.0,
                ".*_wrist_roll_joint": 0.0,
                ".*_wrist_pitch_joint": 0.0,
                "waist_yaw_joint": 0.0,
                "waist_pitch_joint": 0.0,
                "waist_roll_joint": 0.0,
                ".*_hip_.*": 0.0,
                ".*_knee_.*": 0.0,
                ".*_ankle_.*": 0.0,
                "head_.*": 0.0,
                # Robotiq 2F-85 — every revolute joint starts at 0
                # (fully open).  Set explicitly so the simulator does
                # not pick a random in-limit value at reset.
                **{j: 0.0 for j in ROBOTIQ_ALL_JOINTS_LEFT},
                **{j: 0.0 for j in ROBOTIQ_ALL_JOINTS_RIGHT},
            },
            joint_vel={".*": 0.0},
        ),
        actuators=actuators,
    )


@configclass
class KitchenSortingSceneCfgGR1T2Gripper(KitchenSortingSceneCfg):
    """G1 kitchen scene with the robot swapped for GR1T2-with-gripper."""

    robot: ArticulationCfg = _gripper_robot_articulation()


@configclass
class KitchenSortingVisionSceneCfgGR1T2Gripper(KitchenSortingVisionSceneCfg):
    """Vision scene with GR1T2-with-gripper robot."""

    robot: ArticulationCfg = _gripper_robot_articulation()


# ── Observation configuration (gripper-base link names) ────────────────
@configclass
class GripperObservationsCfg:
    """Observation specifications with gripper-base link EEF naming.

    Drops the 22-DoF hand_joint_state observation (no hand joints any
    more).  Adds a binary gripper state observation (closed vs open)
    derived from the gripper finger joint positions.
    """

    @configclass
    class PolicyCfg(ObsGroup):
        actions = ObsTerm(func=sorting_mdp.last_action)

        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_pos = ObsTerm(
            func=base_mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_rot = ObsTerm(
            func=base_mdp.root_quat_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        robot_links_state = ObsTerm(func=sorting_obs.get_all_robot_link_state)

        left_eef_pos = ObsTerm(
            func=sorting_obs.get_eef_pos,
            params={"link_name": GR1T2_EEF_LINK_NAMES["left"]},
        )
        left_eef_quat = ObsTerm(
            func=sorting_obs.get_eef_quat,
            params={"link_name": GR1T2_EEF_LINK_NAMES["left"]},
        )
        right_eef_pos = ObsTerm(
            func=sorting_obs.get_eef_pos,
            params={"link_name": GR1T2_EEF_LINK_NAMES["right"]},
        )
        right_eef_quat = ObsTerm(
            func=sorting_obs.get_eef_quat,
            params={"link_name": GR1T2_EEF_LINK_NAMES["right"]},
        )

        # Gripper finger joint state — replaces the 22D hand_joint_state.
        gripper_joint_state = ObsTerm(
            func=sorting_obs.get_robot_joint_state,
            params={"joint_names": [ROBOTIQ_LEAD_JOINT_LEFT, ROBOTIQ_LEAD_JOINT_RIGHT]},
        )

        # Object / bin observations — robot-agnostic.
        object_positions = ObsTerm(func=sorting_obs.object_positions)
        object_rotations = ObsTerm(func=sorting_obs.object_rotations)
        bin_positions = ObsTerm(func=sorting_obs.bin_positions)

        eef_to_objects = ObsTerm(
            func=sorting_obs.eef_to_nearest_object,
            params={
                "left_eef_link_name": GR1T2_EEF_LINK_NAMES["left"],
                "right_eef_link_name": GR1T2_EEF_LINK_NAMES["right"],
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


# ── Action configuration ───────────────────────────────────────────────
@configclass
class GripperActionsCfg:
    """Pink IK arms (14 joints) + 2 BinaryJointPositionAction grippers (2 binary)."""

    # NOTE on frame naming (post-9.45 USD inspection)
    # ------------------------------------------------
    # GR1T2's actual link convention is ``{side}_hand_pitch_link`` (the
    # build_gripper_usd.py header comment confirms this — its 9.45 fix).
    # The corresponding *joint* is ``{side}_wrist_pitch_joint`` (URDF
    # convention: joint named after the DoF, link named after the segment).
    # Earlier drafts of this env_cfg referenced ``*_wrist_pitch_link``
    # which does not exist in either USD or the converted URDF, so Pink IK
    # threw FrameNotFound at startup.
    #
    # ``target_eef_link_names`` takes the **Isaac Lab body name** (no
    # prefix), while ``FrameTask`` / ``NullSpacePostureTask.controlled_frames``
    # take the **Pinocchio frame name** which carries the ``GR1T2_fourier_hand_6dof_``
    # prefix because the wrist links live under that USD prim.  Gripper
    # links added by build_gripper_usd.py are siblings of that prim, so
    # they do NOT carry the prefix (see ``GR1T2_GRIPPER_FRAME_NAMES`` above).
    #
    # Why not target ``*_gripper_base_link`` directly?  The retargeter
    # already applies ``controller_to_wrist_offset=(0,0,-0.05)`` to convert
    # PICO controller pose → wrist target, and the gripper base sits +5 cm
    # above the wrist (build_gripper_usd line 257), so wrist-targeting + the
    # offset chain naturally places the gripper TCP at the controller
    # position.  Targeting gripper_base_link would double-apply the offset.
    pink_ik_cfg = PinkInverseKinematicsActionCfg(
        pink_controlled_joint_names=list(GR1T2_PINK_CONTROLLED_ARM_JOINTS),
        # Empty list — the gripper joints are not controlled by Pink IK.
        # ``num_hand_joints`` therefore must be 0 below.
        hand_joint_names=[],
        # Option A (Robotiq 2F-85) Pink IK targets — TCP frames added by
        # build_robotiq_usd.py at +0.150 m local Z from each gripper base.
        # These have no GR1T2 prefix because they sit as siblings of the
        # gripper subtree (root direct children, same convention as the
        # 9.45 option-B gripper attach).
        target_eef_link_names={
            "left_wrist":  "left_gripper_tcp_link",
            "right_wrist": "right_gripper_tcp_link",
        },
        asset_name="robot",
        controller=PinkIKControllerCfg(
            articulation_name="robot",
            base_link_name="base_link",
            num_hand_joints=0,
            show_ik_warnings=True,
            fail_on_joint_limit_violation=False,
            variable_input_tasks=[
                # 13th session, part 11 — orientation_cost bumped 1.0 → 6.0.
                # User reported controller rotation does not visibly rotate
                # the robot wrist.  The retargeter math IS correct (cal
                # branch composes delta_q on top of idle_q so wrist target
                # rotates 1:1 with controller).  Root cause was IK weight
                # imbalance: position_cost=8.0 vs orientation_cost=1.0 →
                # IK prioritised position by 8× and treated orientation
                # as nearly-free, so the wrist 3-DoF chain (yaw/roll/pitch)
                # ended up at whatever pose minimised position error,
                # largely ignoring the rotation target.  6.0 keeps
                # position dominant but lets orientation actually drive
                # the wrist joints.
                FrameTask(
                    "left_gripper_tcp_link",
                    position_cost=8.0,
                    orientation_cost=6.0,
                    lm_damping=12,
                    gain=0.5,
                ),
                FrameTask(
                    "right_gripper_tcp_link",
                    position_cost=8.0,
                    orientation_cost=6.0,
                    lm_damping=12,
                    gain=0.5,
                ),
                NullSpacePostureTask(
                    cost=0.5,
                    lm_damping=1,
                    controlled_frames=[
                        "left_gripper_tcp_link",
                        "right_gripper_tcp_link",
                    ],
                    controlled_joints=[
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
                    ],
                ),
            ],
            fixed_input_tasks=[],
            xr_enabled=bool(carb.settings.get_settings().get("/app/xr/enabled")),
        ),
        enable_gravity_compensation=False,
    )

    # Robotiq 2F-85 binary grippers — independent L/R control thanks to
    # the per-side prefix rename in ``build_robotiq_usd.py``.
    #
    # 2026-05-13 (9th session) -- lead-only binary action.  Drive ONLY
    # the lead joint (`{side}_finger_joint`); the 5 followers per side
    # are coordinated by the 4-bar linkage + PhysxMimicJointAPI (see
    # the actuator block above for the matching K=0 passive drive on
    # followers).  Driving all 6 joints with independent position
    # targets created a numerical instability in PhysX's closed-loop
    # solver where the followers' explicit targets fought the mimic
    # constraint and the linkage geometry, causing runaway angles.
    left_gripper_action = BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=[ROBOTIQ_LEAD_JOINT_LEFT],
        open_command_expr ={ROBOTIQ_LEAD_JOINT_LEFT: 0.0},
        close_command_expr={ROBOTIQ_LEAD_JOINT_LEFT: ROBOTIQ_CLOSE_RAD},
    )
    right_gripper_action = BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=[ROBOTIQ_LEAD_JOINT_RIGHT],
        open_command_expr ={ROBOTIQ_LEAD_JOINT_RIGHT: 0.0},
        close_command_expr={ROBOTIQ_LEAD_JOINT_RIGHT: ROBOTIQ_CLOSE_RAD},
    )


def _gripper_idle_action() -> torch.Tensor:
    """16-D idle action — true T-pose wrists + grippers fully open.

    13th-bis session, part 8 — values measured from the actual robot's
    default joint state via ``scripts/inspect_tcp_world_pose.py``.
    Previous values (X = ±0.20, Z = +1.05, quat = 90° about Y) were
    legacy from a pre-Robotiq-attach setup; they put the targets ~1 m
    above the pelvis (above the robot's head) and L/R asymmetric on
    the X axis, causing Pink IK to twist the left arm backwards even
    in idle.  Anchoring to the measured T-pose means Pink IK has zero
    error at default joint state → robot stays still in idle.

    Measured T-pose:
      L_TCP in base_link: pos=(+0.003, +0.229, -0.235), quat=(0,0,1,0)
      R_TCP in base_link: pos=(+0.003, -0.229, -0.235), quat=(0,0,1,0)
      (quat (0,0,1,0) = 180° about Y = palm-down convention from
       build_robotiq_usd.py's Ry(180°) gripper attach).
    """
    return torch.tensor(
        [
            # Left wrist: pos(3) + quat(4)   (base_link frame, measured)
            0.003, 0.229, -0.235,   0.0, 0.0, 1.0, 0.0,
            # Right wrist: pos(3) + quat(4)
            0.003, -0.229, -0.235,  0.0, 0.0, 1.0, 0.0,
            # Gripper commands: +1 = open
            +1.0, +1.0,
        ],
        dtype=torch.float32,
    )


# ── Base environment ───────────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2GripperEnvCfg(ManagerBasedRLEnvCfg):
    """GR1T2 + 2-finger gripper kitchen sorting environment (16-D action)."""

    scene: KitchenSortingSceneCfgGR1T2Gripper = KitchenSortingSceneCfgGR1T2Gripper(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )

    observations: GripperObservationsCfg = GripperObservationsCfg()
    actions: GripperActionsCfg = GripperActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    events = EventCfg()

    commands = None
    curriculum = None

    xr: XrCfg = XrCfg(
        anchor_pos=(0.0, 0.0, 0.0),
        anchor_rot=(1.0, 0.0, 0.0, 0.0),
    )

    temp_urdf_dir = tempfile.gettempdir()

    idle_action = _gripper_idle_action()

    def __post_init__(self):
        # Defensive: ``GripperActionsCfg.pink_ik_cfg`` is a class attribute
        # so its mutable fields (the joint-name lists) are SHARED across
        # instances.  Reset them to fresh copies on every cfg construction
        # so accidental mutations from prior runs don't accumulate.  This
        # is the partner of the matching fix in
        # ``KitchenSortingGR1T2GripperWaistEnvCfg.__post_init__``.
        self.actions.pink_ik_cfg.pink_controlled_joint_names = list(
            GR1T2_PINK_CONTROLLED_ARM_JOINTS
        )

        # Inherit 9.24 fix: decimation=1 (env step at 120 Hz) so gripper
        # toggling latency stays < 10 ms even at peak finger speed.
        self.decimation = 1
        self.episode_length_s = 60.0

        self.sim.dt = 1 / 120
        self.sim.render_interval = 1
        self.sim.disable_contact_processing = False
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.enable_scene_query_support = True
        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,    # Higher friction so gripper holds objects
            dynamic_friction=0.8,
            restitution=0.0,
        )

        # Convert USD → URDF for Pink IK.  First run re-converts (cache miss);
        # subsequent runs hit the cache in temp_urdf_dir/urdf/ unless the USD
        # mtime is newer.  ``force_conversion=True`` made every launch take an
        # extra 30-60s on this rig and -- combined with PowerShell's stdout
        # buffering -- it looked like the script was hung between the
        # ``[run_teleop] env_cfg = …`` line and the next print, prompting
        # users to Ctrl+C.
        usd_path = self.scene.robot.spawn.usd_path
        urdf_cache_path = os.path.join(self.temp_urdf_dir, "urdf",
                                        os.path.basename(usd_path).split(".")[0] + ".urdf")
        meshes_cache_dir = os.path.join(self.temp_urdf_dir, "meshes")
        cache_hit = (os.path.exists(urdf_cache_path)
                     and os.path.exists(meshes_cache_dir)
                     and os.path.getmtime(urdf_cache_path) >= os.path.getmtime(usd_path))
        if cache_hit:
            print(f"[ust_hm_grip] URDF cache hit -> {urdf_cache_path}", flush=True)
        else:
            print(
                f"[ust_hm_grip] Converting USD -> URDF for Pink IK ...\n"
                f"  source : {usd_path}\n"
                f"  cache  : {urdf_cache_path}\n"
                f"  This typically takes 30-90 seconds on first run; subsequent\n"
                f"  launches reuse the cached URDF.  Delete {urdf_cache_path}\n"
                f"  if you change the USD and need a forced re-convert.",
                flush=True,
            )
        temp_urdf_output_path, temp_urdf_meshes_output_path = ControllerUtils.convert_usd_to_urdf(
            usd_path,
            self.temp_urdf_dir,
            force_conversion=not cache_hit,
        )
        print(f"[ust_hm_grip] URDF ready: {temp_urdf_output_path}", flush=True)
        self.actions.pink_ik_cfg.controller.urdf_path = temp_urdf_output_path
        self.actions.pink_ik_cfg.controller.mesh_path = temp_urdf_meshes_output_path

        # PICO controller device cfg — read by run_teleop.py
        self.pico_device_cfg = {
            "tracker_binding_json":
                "./ust_ws/ust_hm_grip/config/tracker_binding.json",
            "actions_json":
                "./ust_ws/ust_hm_grip/config/openvr_actions/actions.json",
            "vrmanifest_json":
                "./ust_ws/ust_hm_grip/config/openvr_actions/manifest.vrmanifest",
            "app_key": "ust.teleop.gr1t2_gripper",
            "forearm_wrist_offset": (0.28, 0.0, 0.0),  # elbow-mounted tracker
            "position_scale": 1.0,
            "rotation_scale": 1.0,
            "body_pos_offset": (0.0, 0.0, 0.0),
            "controller_pos_offset": (0.0, 0.0, 0.0),
            "use_waist_origin": True,
            # 13th-bis session: enable Z subtraction by default.  Without
            # it the user's controller height (relative to the XR LOCAL
            # origin, typically near the headset) is fed straight into
            # the robot's wrist target Z (relative to base_link / pelvis).
            # That mismatch put the wrist target ~0.4-0.5 m BELOW the
            # robot pelvis even when the user was holding hands at chest
            # height — Pink IK then drove the robot's arms into awkward
            # "near hips" poses instead of natural mid-air positions,
            # which the user perceived as "tracker not working".
            # With subtract_waist_z=True the waist tracker's Z (≈ user
            # pelvis height) is subtracted, so controller-Z minus waist-Z
            # ≈ "how far above the user's pelvis the hand is", which
            # maps directly to robot wrist-above-pelvis height.
            "subtract_waist_z": True,
            "freeze_orientation": False,
            # 13th-bis session, part 8 — Z180 hack disabled.  With the
            # measured palm-down idle quat (0, 0, 1, 0), both sides
            # naturally target palm-down — no extra Z flip needed.
            "right_wrist_z180": False,
            "prefer_controller_for_eef": True,
            "controller_to_wrist_offset": (0.0, 0.0, -0.05),
            "gripper_close_threshold": 0.6,
            "gripper_open_threshold": 0.4,
            "use_grip_as_close": True,
            "disable_arm_tracking": False,
            # 13th session, part 15 — match PICO Motion Tracker app
            # cal-view: SMPL body wrist drives arm+wrist POSITION;
            # controller drives wrist fine ROTATION (the SMPL wrist
            # joints idx 20/21 are AI-smoothed and don't capture fine
            # controller-grip rotation; NVIDIA SONIC/GR00T uses the
            # same overlay).  Controller POS contribution = zero.
            "wrist_pos_source": "wrist_tracker",
            "wrist_quat_source": "controller",
            "debug": True,
        }


# ── WaistEnabled variant ───────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2GripperWaistEnvCfg(KitchenSortingGR1T2GripperEnvCfg):
    """Pink IK also controls the 3 waist joints (null-space redundancy)."""

    def __post_init__(self):
        super().__post_init__()
        # ``GripperActionsCfg.pink_ik_cfg`` is a class-level attribute, so its
        # ``pink_controlled_joint_names`` list is the SAME object across every
        # cfg instance built from this class.  The previous ``append`` loop
        # mutated that shared list, so multiple cfg constructions accumulated:
        # the 31 we saw in the IK shape-mismatch traceback is exactly what
        # you get when the list is appended twice (14 → 17 → 17+14 = 31 when
        # the base list is reused).  Always REPLACE the list with a fresh
        # copy so the result is idempotent regardless of how many times the
        # cfg gets constructed in a single Python session.
        self.actions.pink_ik_cfg.pink_controlled_joint_names = (
            list(GR1T2_PINK_CONTROLLED_ARM_JOINTS) + list(GR1T2_WAIST_JOINT_NAMES)
        )
        print(
            f"[ust_hm_grip][waist] pink_controlled_joint_names "
            f"({len(self.actions.pink_ik_cfg.pink_controlled_joint_names)} joints): "
            f"{self.actions.pink_ik_cfg.pink_controlled_joint_names}",
            flush=True,
        )


# ── Vision (adds cameras) ─────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2GripperVisionEnvCfg(KitchenSortingGR1T2GripperEnvCfg):
    """Same as base but with the vision scene (wrist + shoulder cameras)."""

    scene: KitchenSortingVisionSceneCfgGR1T2Gripper = (
        KitchenSortingVisionSceneCfgGR1T2Gripper(
            num_envs=1,
            env_spacing=3.0,
            replicate_physics=True,
        )
    )


# ── PC-only monitor mode (no XR) ──────────────────────────────────────
@configclass
class KitchenSortingGR1T2GripperMonitorEnvCfg(KitchenSortingGR1T2GripperEnvCfg):
    """PC-only view; run_teleop.py sets xr=False via --render_mode monitor."""

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 30.0


# ── VR long-episode variant ────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2GripperVREnvCfg(KitchenSortingGR1T2GripperEnvCfg):
    """1-hour episodes for live VR teleop."""

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 3600.0


# ── Data-collection variant ────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2GripperDataCollectEnvCfg(
    KitchenSortingGR1T2GripperVisionEnvCfg
):
    """Medium episodes + cameras for HDF5 recording."""

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 90.0


# ── RobotOnly variant — minimal scene for debugging ───────────────────
@configclass
class RobotOnlySceneCfgGR1T2Gripper(InteractiveSceneCfg):
    """Minimal scene — only the GR1T2-with-gripper robot, ground, light."""

    robot: ArticulationCfg = _gripper_robot_articulation()

    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=GroundPlaneCfg(),
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(
            color=(0.75, 0.75, 0.75),
            intensity=3000.0,
        ),
    )


@configclass
class GripperRobotOnlyObservationsCfg:
    """Minimal observations — drops object/bin terms for empty-scene env."""

    @configclass
    class PolicyCfg(ObsGroup):
        actions = ObsTerm(func=sorting_mdp.last_action)

        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_pos = ObsTerm(
            func=base_mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_rot = ObsTerm(
            func=base_mdp.root_quat_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        robot_links_state = ObsTerm(func=sorting_obs.get_all_robot_link_state)

        left_eef_pos = ObsTerm(
            func=sorting_obs.get_eef_pos,
            params={"link_name": GR1T2_EEF_LINK_NAMES["left"]},
        )
        right_eef_pos = ObsTerm(
            func=sorting_obs.get_eef_pos,
            params={"link_name": GR1T2_EEF_LINK_NAMES["right"]},
        )

        gripper_joint_state = ObsTerm(
            func=sorting_obs.get_robot_joint_state,
            params={"joint_names": [ROBOTIQ_LEAD_JOINT_LEFT, ROBOTIQ_LEAD_JOINT_RIGHT]},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class GripperRobotOnlyEventCfg:
    reset_all = EventTerm(func=base_mdp.reset_scene_to_default, mode="reset")


@configclass
class GripperRobotOnlyTerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)


@configclass
class GripperRobotOnlyRewardsCfg:
    pass


@configclass
class KitchenSortingGR1T2GripperRobotOnlyEnvCfg(
    KitchenSortingGR1T2GripperWaistEnvCfg
):
    """Empty-scene GR1T2 + gripper env: robot + ground + light only."""

    scene: RobotOnlySceneCfgGR1T2Gripper = RobotOnlySceneCfgGR1T2Gripper(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )

    observations: GripperRobotOnlyObservationsCfg = GripperRobotOnlyObservationsCfg()
    rewards: GripperRobotOnlyRewardsCfg = GripperRobotOnlyRewardsCfg()
    terminations: GripperRobotOnlyTerminationsCfg = GripperRobotOnlyTerminationsCfg()
    events = GripperRobotOnlyEventCfg()
