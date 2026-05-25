"""Kitchen Sorting environment configs for Fourier GR1T2 + 6-DoF hand.

Parallel to ``ust_ws.ust_260220.kitchen_sorting_env_cfg`` but targets
``GR1T2_HIGH_PD_CFG`` with the palm-frame Pink IK target and the 36D
action layout mandated by Isaac Lab 2.3's built-in
``PickPlaceGR1T2EnvCfg``.

Scene, observations, rewards, events and terminations are reused from
``ust_260220`` verbatim.  Only the robot asset, the ``ActionsCfg`` (Pink
IK frame task + hand_joint_names) and the 36D idle action differ.

Env classes:

* :class:`KitchenSortingGR1T2EnvCfg`              — base (waist in null-space posture only)
* :class:`KitchenSortingGR1T2WaistEnvCfg`         — Pink IK also controls 3 waist joints
* :class:`KitchenSortingGR1T2VisionEnvCfg`        — + shoulder/wrist cameras
* :class:`KitchenSortingGR1T2MonitorEnvCfg`       — PC-only view (no XR)
* :class:`KitchenSortingGR1T2VREnvCfg`            — long-episode VR variant (1 h)
* :class:`KitchenSortingGR1T2DataCollectEnvCfg`   — dataset recording variant
"""

from __future__ import annotations

# CRITICAL: apply the osqp 0.6 ↔ qpsolvers 4.x compat shim BEFORE any
# pink / qpsolvers module-level import.  Without this, Pink IK has no QP
# solver at runtime and the robot stays frozen (see _osqp_compat module
# docstring for root cause).
from ust_ws.ust_hm_glove.teleop import _osqp_compat as _osqp_compat  # noqa: F401
_osqp_compat.apply()

import tempfile
from typing import Tuple

import torch

import carb
# ``FrameTask`` lives in the upstream ``pink.tasks`` module.
#
# NOTE: Isaac Lab's built-in ``pickplace_gr1t2_env_cfg.py`` references
# ``pink.tasks.DampingTask`` as an extra entry in ``variable_input_tasks``,
# but Isaac Lab 0.48.0's ``PinkIKController.__init__`` (see
# ``source/isaaclab/isaaclab/controllers/pink_ik/pink_ik.py:97-102``)
# unconditionally calls ``task.set_target_from_configuration(...)`` on
# every non-``NullSpacePostureTask`` — and ``DampingTask`` does not
# implement that method.  Including it in our env_cfg therefore crashes
# ``ManagerBasedRLEnv(cfg=env_cfg)`` with
# ``AttributeError: 'DampingTask' object has no attribute
# 'set_target_from_configuration'``.
#
# The working ``ust_260220`` G1 env_cfg already drops ``DampingTask`` for
# the same reason.  Posture regularisation is still provided by
# ``NullSpacePostureTask``, which has the extra benefit of keeping the
# elbow/shoulder/waist near a target posture instead of merely damping
# velocity.  If a future Isaac Lab version special-cases ``DampingTask``
# in ``pink_ik.py`` we can re-introduce it here.
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
from isaaclab.devices.device_base import DevicesCfg
from isaaclab.devices.openxr import ManusViveCfg, OpenXRDeviceCfg, XrCfg
from isaaclab.devices.openxr.retargeters.humanoid.fourier.gr1t2_retargeter import (
    GR1T2RetargeterCfg,
)
from isaaclab.envs import ManagerBasedRLEnvCfg
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
    USDEventCfg,
    USDRewardsCfg,
    USDTerminationsCfg,
)
from ust_ws.ust_260220.mdp import observations as sorting_obs
from ust_ws.ust_260220 import mdp as sorting_mdp

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab_assets.robots.fourier import GR1T2_HIGH_PD_CFG  # isort: skip

from ust_ws.ust_hm_glove.teleop.gr1t2_udcap_device import (
    GR1T2FourierUDCAPDeviceCfg,
)


__all__ = [
    "FOURIER_HAND_JOINT_NAMES",
    "GR1T2_PINK_CONTROLLED_ARM_JOINTS",
    "GR1T2_WAIST_JOINT_NAMES",
    "GR1T2_PALM_FRAME_NAMES",
    "FourierActionsCfg",
    "KitchenSortingGR1T2EnvCfg",
    "KitchenSortingGR1T2WaistEnvCfg",
    "KitchenSortingGR1T2VisionEnvCfg",
    "KitchenSortingGR1T2MonitorEnvCfg",
    "KitchenSortingGR1T2VREnvCfg",
    "KitchenSortingGR1T2DataCollectEnvCfg",
    "KitchenSortingGR1T2RobotOnlyEnvCfg",
]


# ── Canonical Fourier hand joint names (22 total) ───────────────────────
# Order mirrors source/isaaclab_tasks/.../pick_place/pickplace_gr1t2_env_cfg.py
# so that our action tensor [14:36] aligns with what the Pink IK expects.
FOURIER_HAND_JOINT_NAMES = [
    # Proximal drivers — interleaved L/R
    "L_index_proximal_joint",
    "L_middle_proximal_joint",
    "L_pinky_proximal_joint",
    "L_ring_proximal_joint",
    "L_thumb_proximal_yaw_joint",
    "R_index_proximal_joint",
    "R_middle_proximal_joint",
    "R_pinky_proximal_joint",
    "R_ring_proximal_joint",
    "R_thumb_proximal_yaw_joint",
    # Intermediate mimic — interleaved L/R (slot 14/19 is thumb proximal pitch)
    "L_index_intermediate_joint",
    "L_middle_intermediate_joint",
    "L_pinky_intermediate_joint",
    "L_ring_intermediate_joint",
    "L_thumb_proximal_pitch_joint",
    "R_index_intermediate_joint",
    "R_middle_intermediate_joint",
    "R_pinky_intermediate_joint",
    "R_ring_intermediate_joint",
    "R_thumb_proximal_pitch_joint",
    # Thumb distal (mimic)
    "L_thumb_distal_joint",
    "R_thumb_distal_joint",
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

GR1T2_PALM_FRAME_NAMES = (
    "GR1T2_fourier_hand_6dof_left_hand_pitch_link",
    "GR1T2_fourier_hand_6dof_right_hand_pitch_link",
)

# GR1T2 end-effector body names (for ObservationsCfg / eef-to-object deltas).
# Matches Isaac Lab's built-in ``pickplace_gr1t2_env_cfg.py`` choice — the
# roll link is the last rigid body in the hand chain so its world pose is
# a reasonable proxy for "palm centre".  G1 uses ``*_wrist_yaw_link`` here;
# GR1T2 has no such body and raises ValueError when queried.
GR1T2_EEF_LINK_NAMES = {
    "left":  "left_hand_roll_link",
    "right": "right_hand_roll_link",
}


def _fourier_robot_articulation() -> ArticulationCfg:
    """Return the GR1T2 articulation cfg bound to ``/World/envs/env_.*/Robot``.

    Spawn orientation (2026-04-22 fix)
    ---------------------------------
    The G1 cfg used ``rot=(0.7071, 0, 0, 0.7071)`` so that the robot's
    body +X pointed at world +Y, aligning it with the kitchen-sorting
    scene's table (at ``pos=(0, +0.55, …)``).  Carried over verbatim to
    GR1T2, this caused two distinct problems for the VR teleop operator:

    1. **Visual appearance** — in the default monitor / VD desktop-theater
       camera view the robot looks "twisted 90° to the right" since its
       face points at world +Y while the camera looks at world -Z.
    2. **Pink IK frame mismatch** — our retargeter emits wrist targets
       in Isaac-Lab world axes (after ``svr_to_isaaclab``), but Pink IK
       interprets them in the articulation's ``base_link`` frame.  With
       yaw=+90° the base_link X-axis (robot "forward") is world +Y, so
       a user reaching forward at the headset sends a target along
       world +X which the solver maps to base_link -Y = robot **right
       shoulder** instead of **forward**.  The arms track but with a
       90° azimuth offset.

    Setting ``rot=(1, 0, 0, 0)`` (identity) puts base_link axes coincident
    with the world axes, so the retargeter's world-frame coordinates are
    the correct base_link-local frame for Pink IK.  Side effect: the
    G1-era kitchen table at ``y=+0.55`` now sits on the robot's **left**
    side.  The user-facing impact is limited because the SteamVR world
    origin is typically re-seated with VD's ``Reset View`` gesture
    anyway — they can stand wherever matches the scene.  A fuller scene
    rotation (table/bins moved to +X) is left as follow-up; it can be
    done in a subclass without touching ``ust_260220``.
    """
    # NOTE on hand actuator override (2026-04-26 9.6/9.7 fix)
    # --------------------------------------------------------
    # ``GR1T2_HIGH_PD_CFG`` ships ``stiffness=None, damping=None`` for the
    # ``left-hand`` and ``right-hand`` ImplicitActuator entries (see
    # source/isaaclab_assets/.../robots/fourier.py L151-160).  ``None``
    # means "fall back to the value baked into the USD".  For the GR1T2
    # 6-DoF hand the USD's per-finger stiffness/effort_limit is too low
    # to enforce the joint position targets that Pink IK writes -- the
    # action tensor's finger slice ``[14:36]`` swings by 30+ degrees
    # frame-to-frame (verified by the [GR1T2Retarget #N] periodic log
    # showing L_idx target ranging [0.05, 0.70] rad) yet the physical
    # joint stays frozen.  The arm joints in the same cfg use
    # ``stiffness=4400`` AND ``effort_limit=torch.inf`` and respond
    # instantly.  9.6 (stiffness=500) was insufficient -- 9.7 cranks
    # both stiffness AND effort_limit + velocity_limit explicitly so
    # nothing baked into the USD can clamp the actuator output.
    #
    # We pass the override via ``.replace(actuators=...)`` in a single
    # call rather than mutating ``cfg.actuators`` afterwards, since
    # configclass dataclass mutation behavior across nested
    # ``.replace()`` calls has bitten us silently before.
    # 9.27 fix (research/33 §2.2 / §2.3): the legacy ``effort_limit`` and
    # ``velocity_limit`` parameters are SILENTLY IGNORED on
    # ``ImplicitActuator`` (see source/isaaclab/isaaclab/actuators/
    # actuator_pd.py:79-100 — they get nullified with an omni.log.warn the
    # user rarely sees).  9.7 set ``velocity_limit=10.0`` thinking it would
    # enforce a 10 rad/s cap; in reality whatever was baked into the GR1T2
    # USD ``physics:maxJointVelocity`` is what PhysX uses, and that value
    # may be much lower than the natural 12-20 rad/s of human finger flex.
    # Passing the ``_sim`` suffix variants instead writes the value into
    # the ArticulationView at startup so PhysX honours it.  We bump
    # velocity to 50 rad/s (well above natural max finger speed) and
    # effort to 200 N·m (so the PD output isn't torque-clipped during the
    # initial fast transient when target jumps by ~1.57 rad).  Stiffness/
    # damping unchanged from 9.7 — those values are honoured.  See
    # research/33 §2.2 for the latency budget derivation and §3.2 for the
    # before/after measurement protocol.
    actuators = dict(GR1T2_HIGH_PD_CFG.actuators)
    actuators["left-hand"] = ImplicitActuatorCfg(
        joint_names_expr=["L_.*"],
        stiffness=10000.0,
        damping=100.0,
        effort_limit_sim=200.0,
        velocity_limit_sim=50.0,
        armature=0.001,
    )
    actuators["right-hand"] = ImplicitActuatorCfg(
        joint_names_expr=["R_.*"],
        stiffness=10000.0,
        damping=100.0,
        effort_limit_sim=200.0,
        velocity_limit_sim=50.0,
        armature=0.001,
    )
    return GR1T2_HIGH_PD_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
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
                "L_.*": 0.0,
                "R_.*": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
        actuators=actuators,
    )


@configclass
class KitchenSortingSceneCfgGR1T2(KitchenSortingSceneCfg):
    """G1 kitchen scene with the robot swapped for GR1T2."""

    robot: ArticulationCfg = _fourier_robot_articulation()


@configclass
class KitchenSortingVisionSceneCfgGR1T2(KitchenSortingVisionSceneCfg):
    """Vision scene with GR1T2 robot."""

    robot: ArticulationCfg = _fourier_robot_articulation()


@configclass
class KitchenSortingUSDSceneCfgGR1T2(KitchenSortingUSDSceneCfg):
    """Custom-USD scene with GR1T2 robot."""

    robot: ArticulationCfg = _fourier_robot_articulation()


# ── Observation configuration (GR1T2 link names) ───────────────────────
# G1's ``ObservationsCfg`` hardcodes ``left_wrist_yaw_link`` /
# ``right_wrist_yaw_link`` in five ObsTerms (EEF pos/quat × 2 + the default
# params of ``eef_to_nearest_object``).  GR1T2 exposes no such bodies,
# which raises ``ValueError: 'left_wrist_yaw_link' is not in list`` during
# ``ObservationManager._prepare_terms``.  We replicate the G1 observation
# layout below but substitute the GR1T2 palm-roll link everywhere.
#
# Identical shape & ordering to G1's ObservationsCfg so recordings remain
# compatible at the structural level (dim layouts unchanged).
@configclass
class FourierObservationsCfg:
    """Observation specifications with GR1T2 end-effector link names."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for the policy."""

        # Last actions
        actions = ObsTerm(func=sorting_mdp.last_action)

        # Robot state
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

        # All robot link states
        robot_links_state = ObsTerm(func=sorting_obs.get_all_robot_link_state)

        # EEF state — GR1T2 palm-roll links
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

        # Hand joint state — GR1T2 uses the same L_* / R_* prefix convention
        # as G1 Inspire, so this regex still matches.
        hand_joint_state = ObsTerm(
            func=sorting_obs.get_robot_joint_state,
            params={"joint_names": ["R_.*", "L_.*"]},
        )

        # Objects / bins (robot-agnostic)
        object_positions = ObsTerm(func=sorting_obs.object_positions)
        object_rotations = ObsTerm(func=sorting_obs.object_rotations)
        bin_positions = ObsTerm(func=sorting_obs.bin_positions)

        # EEF-to-object deltas with GR1T2 link overrides
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
class FourierActionsCfg:
    """Dual-arm palm Pink IK + 22 DoF Fourier hand control (36D total)."""

    pink_ik_cfg = PinkInverseKinematicsActionCfg(
        pink_controlled_joint_names=list(GR1T2_PINK_CONTROLLED_ARM_JOINTS),
        hand_joint_names=list(FOURIER_HAND_JOINT_NAMES),
        target_eef_link_names={
            "left_wrist": "left_hand_pitch_link",
            "right_wrist": "right_hand_pitch_link",
        },
        asset_name="robot",
        controller=PinkIKControllerCfg(
            articulation_name="robot",
            base_link_name="base_link",
            num_hand_joints=22,
            # True by default so the first week of bring-up logs solver
            # failures.  Flip to False once convergence is trusted.
            show_ik_warnings=True,
            fail_on_joint_limit_violation=False,
            variable_input_tasks=[
                FrameTask(
                    GR1T2_PALM_FRAME_NAMES[0],
                    position_cost=8.0,
                    orientation_cost=1.0,
                    lm_damping=12,
                    gain=0.5,
                ),
                FrameTask(
                    GR1T2_PALM_FRAME_NAMES[1],
                    position_cost=8.0,
                    orientation_cost=1.0,
                    lm_damping=12,
                    gain=0.5,
                ),
                # NOTE: ``DampingTask(cost=0.5)`` intentionally omitted — see
                # the module-level NOTE on the pink.tasks import.
                NullSpacePostureTask(
                    cost=0.5,
                    lm_damping=1,
                    controlled_frames=list(GR1T2_PALM_FRAME_NAMES),
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


def _gr1t2_idle_action() -> torch.Tensor:
    """36D idle action: T-pose palm pose for each wrist + 22 zeros."""
    return torch.tensor(
        [
            # Left wrist: pos(3) + quat(4)   (base_link frame)
            -0.20, 0.00, 1.05,  0.707, 0.0, 0.707, 0.0,
            # Right wrist: pos(3) + quat(4)
            0.20, 0.00, 1.05,  0.707, 0.0, 0.707, 0.0,
            # 22 hand joints (rest)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ],
        dtype=torch.float32,
    )


# ── Base environment ───────────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2EnvCfg(ManagerBasedRLEnvCfg):
    """GR1T2 + Fourier 6-DoF hand kitchen sorting environment.

    Action layout (36):
        [0:3]   left  wrist position  (base_link frame)
        [3:7]   left  wrist quat wxyz
        [7:10]  right wrist position
        [10:14] right wrist quat wxyz
        [14:36] 22 hand joints, order per :data:`FOURIER_HAND_JOINT_NAMES`
    """

    scene: KitchenSortingSceneCfgGR1T2 = KitchenSortingSceneCfgGR1T2(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )

    observations: FourierObservationsCfg = FourierObservationsCfg()
    actions: FourierActionsCfg = FourierActionsCfg()
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

    idle_action = _gr1t2_idle_action()

    def __post_init__(self):
        # 9.24 fix: decimation 6 → 1 so env.step runs at 120 Hz instead
        # of 20 Hz.  UDCAP broadcasts at ~140 Hz, SteamVR samples at 120
        # Hz — the previous 7:1 sub-sampling produced visible 0↔limit
        # aliasing in the [GR1T2Retarget #N] logs (e.g. l_idx swinging
        # -1.26 → -0.00 → -1.26 across 40 frames in the 9.22 user
        # video).  See research/30 for the hardware-specific analysis
        # (Pink IK QP P99 ≈ 5 ms warm-start vs 8.33 ms budget — 40%
        # margin on Ryzen 9 7950X3D + 3D V-Cache) and memory.md §10.32
        # for the verification.  Pair with --finger_lp_alpha 0.4 (or
        # 0.6 for more responsive, 0.2 for stronger smoothing).
        self.decimation = 1
        self.episode_length_s = 60.0

        self.sim.dt = 1 / 120
        self.sim.render_interval = 1
        self.sim.disable_contact_processing = False
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.enable_scene_query_support = True
        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=0.7,
            dynamic_friction=0.5,
            restitution=0.1,
        )

        # Convert USD → URDF for Pink IK (re-runs on every startup; cached).
        temp_urdf_output_path, temp_urdf_meshes_output_path = ControllerUtils.convert_usd_to_urdf(
            self.scene.robot.spawn.usd_path,
            self.temp_urdf_dir,
            force_conversion=True,
        )
        self.actions.pink_ik_cfg.controller.urdf_path = temp_urdf_output_path
        self.actions.pink_ik_cfg.controller.mesh_path = temp_urdf_meshes_output_path

        # Teleop devices
        self.teleop_devices = DevicesCfg(
            devices={
                "handtracking": OpenXRDeviceCfg(
                    retargeters=[
                        GR1T2RetargeterCfg(
                            enable_visualization=False,
                            num_open_xr_hand_joints=2 * 26,
                            sim_device=self.sim.device,
                            hand_joint_names=self.actions.pink_ik_cfg.hand_joint_names,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
                "manusvive": ManusViveCfg(
                    retargeters=[
                        GR1T2RetargeterCfg(
                            enable_visualization=False,
                            num_open_xr_hand_joints=2 * 26,
                            sim_device=self.sim.device,
                            hand_joint_names=self.actions.pink_ik_cfg.hand_joint_names,
                        ),
                    ],
                    sim_device=self.sim.device,
                    xr_cfg=self.xr,
                ),
            },
        )

        # PICO UDCAP (Windows/SteamVR) — read by ust_hm_glove run_teleop.py
        self.pico_device_cfg = {
            "tracker_binding_json": "./ust_ws/ust_hm_glove/config/tracker_binding.json",
            "actions_json": "./ust_ws/ust_hm_glove/config/openvr_actions/actions.json",
            "left_hand_retarget_yaml": (
                "./ust_ws/ust_hm_glove/config/dex_retargeting/fourier_left_dexpilot.yml"
            ),
            "right_hand_retarget_yaml": (
                "./ust_ws/ust_hm_glove/config/dex_retargeting/fourier_right_dexpilot.yml"
            ),
            "forearm_wrist_offset": (0.12, 0.0, 0.0),
            "position_scale": 1.0,
            "rotation_scale": 1.0,
            "gripper_threshold": 0.5,
            "body_pos_offset": (0.0, 0.0, 0.0),
            "controller_pos_offset": (0.0, 0.0, 0.0),
            "use_waist_origin": True,
            "subtract_waist_z": False,
            "freeze_orientation": False,
            "right_wrist_z180": True,
            # Base env: waist DoF is implicit (null-space only); WaistEnabled
            # variant enables explicit estimation.
            "enable_waist_dof": False,
            "waist_source": "hips_tracker",
            "debug": True,
        }


# ── WaistEnabled variant ───────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2WaistEnvCfg(KitchenSortingGR1T2EnvCfg):
    """Pink IK also controls the 3 waist joints (null-space redundancy).

    Mirrors Isaac Lab 2.3 ``PickPlaceGR1T2WaistEnabledEnvCfg`` — the
    action tensor dim is unchanged (36).  The WaistEnabled behaviour
    comes from appending the three ``waist_*_joint`` names to
    ``pink_controlled_joint_names`` so the solver uses them when wrist
    targets are out of the arm-only reachable set.

    Additionally enables the device-side :class:`WaistEstimator` so
    training data carries the user's torso orientation signal.
    """

    def __post_init__(self):
        super().__post_init__()
        for joint_name in GR1T2_WAIST_JOINT_NAMES:
            self.actions.pink_ik_cfg.pink_controlled_joint_names.append(joint_name)
        self.pico_device_cfg["enable_waist_dof"] = True
        self.pico_device_cfg["waist_source"] = "hips_tracker"


# ── Vision (adds cameras) ─────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2VisionEnvCfg(KitchenSortingGR1T2EnvCfg):
    """Same as base but with the vision scene (wrist + shoulder cameras)."""

    scene: KitchenSortingVisionSceneCfgGR1T2 = KitchenSortingVisionSceneCfgGR1T2(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )


# ── PC-only monitor mode (no XR) ──────────────────────────────────────
@configclass
class KitchenSortingGR1T2MonitorEnvCfg(KitchenSortingGR1T2EnvCfg):
    """PC-only view; run_teleop.py sets xr=False via --render_mode monitor."""

    def __post_init__(self):
        super().__post_init__()
        # Shorter episodes for development iteration.
        self.episode_length_s = 30.0


# ── VR long-episode variant (CloudXR / VD Desktop Theater) ────────────
@configclass
class KitchenSortingGR1T2VREnvCfg(KitchenSortingGR1T2EnvCfg):
    """1-hour episodes with render_interval=1 for smooth VR playback."""

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 3600.0
        self.sim.render_interval = 1


# ── Data-collection variant ────────────────────────────────────────────
@configclass
class KitchenSortingGR1T2DataCollectEnvCfg(KitchenSortingGR1T2VisionEnvCfg):
    """Medium episodes + cameras for HDF5 recording."""

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 90.0


# ── RobotOnly variant — minimal scene for finger teleop diagnosis ──────
# 9.12 fix: empty scene (just robot + ground + light) so the operator can
# see finger curl visually without the kitchen table / bins / objects
# obscuring the robot's hands.  Useful for tuning glove sensitivity and
# confirming the joint mapping with a clear viewport.
@configclass
class RobotOnlySceneCfgGR1T2(InteractiveSceneCfg):
    """Minimal scene — only the GR1T2 robot, ground plane, dome light."""

    robot: ArticulationCfg = _fourier_robot_articulation()

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
class FourierRobotOnlyObservationsCfg:
    """Minimal observations for RobotOnly env — no obj/bin terms.

    Same robot-state observations as :class:`FourierObservationsCfg` but
    drops ``object_positions`` / ``object_rotations`` / ``bin_positions``
    / ``eef_to_objects`` (which would crash because no objects exist).
    Action / joint / EEF / hand obs preserved so the recorder pipeline
    still produces meaningful demos for downstream BC if desired.
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

        hand_joint_state = ObsTerm(
            func=sorting_obs.get_robot_joint_state,
            params={"joint_names": ["R_.*", "L_.*"]},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class FourierRobotOnlyEventCfg:
    """Minimal event — only scene reset, no object reset."""

    reset_all = EventTerm(func=base_mdp.reset_scene_to_default, mode="reset")


@configclass
class FourierRobotOnlyTerminationsCfg:
    """Minimal terminations — time_out only (no object-sorting condition)."""

    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)


@configclass
class FourierRobotOnlyRewardsCfg:
    """Empty rewards — teleop diagnostic env, no learning signal needed."""

    pass


@configclass
class KitchenSortingGR1T2RobotOnlyEnvCfg(KitchenSortingGR1T2WaistEnvCfg):
    """Empty-scene GR1T2 env: robot + ground + light only.

    Use this to visually verify finger / arm teleop motion without
    the kitchen sorting scene's table and bins obscuring the robot's
    hands.

    Inherits from WaistEnabled so Pink IK still has the 3 waist
    joints in its null-space.  But the device-side ``WaistEstimator``
    is **disabled** by default (``enable_waist_dof=False``) so a
    mis-mounted hips tracker (Virtual Desktop's AI-inferred waist
    pose can carry a ~30° pitch bias) doesn't auto-bend the robot
    forward and obscure finger motion.  Re-enable with
    ``--enable_waist_dof`` if needed.

    Gym ID: ``Isaac-KitchenSorting-GR1T2-Fourier-RobotOnly-v0``
    """

    scene: RobotOnlySceneCfgGR1T2 = RobotOnlySceneCfgGR1T2(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )

    observations: FourierRobotOnlyObservationsCfg = FourierRobotOnlyObservationsCfg()
    rewards: FourierRobotOnlyRewardsCfg = FourierRobotOnlyRewardsCfg()
    terminations: FourierRobotOnlyTerminationsCfg = FourierRobotOnlyTerminationsCfg()
    events = FourierRobotOnlyEventCfg()

    def __post_init__(self):
        super().__post_init__()
        # 9.25 fix: revert to 9.13 default (False).  9.14 turned this back
        # ON trusting the 30-frame averaging zero-cal to absorb VD AI body-
        # tracker bias, but 9.25 user video showed the robot's torso still
        # bending forward when only PICO HMD + UDCAP gloves + 2 controllers
        # are present (no physical tracker on the user).  Virtual Desktop
        # AND/OR UDCAP synthesise 5 fake trackers that report a static
        # forward-leaning hips quaternion.  The 30-frame zero-cal locks
        # that bad pose as the "rest" → robot stays bent forward
        # indefinitely.  Use ``--enable_waist_dof true`` only when a real
        # hips tracker is mounted on the user.  See memory.md §10.33.
        self.pico_device_cfg["enable_waist_dof"] = False
