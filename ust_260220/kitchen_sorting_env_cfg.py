"""
G1 Kitchen Sorting Environment Configuration.

Unitree G1 robot with INSPIRE 5-finger hand sorts kitchen objects into
categorized bins on a table. Fixed base, dual-arm Pink IK control.

Based on: pickplace_unitree_g1_inspire_hand_env_cfg.py
Design guide: ust_ws/research/7. g1_kitchen_sorting_usd_scene_design_guide.md
GPU target: NVIDIA RTX PRO 6000 (96GB VRAM, Blackwell)
"""

import tempfile
import torch

import carb
from pink.tasks import FrameTask

import isaaclab.controllers.utils as ControllerUtils
import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers.pink_ik import NullSpacePostureTask, PinkIKControllerCfg
from isaaclab.devices.device_base import DevicesCfg
from isaaclab.devices.openxr import ManusViveCfg, OpenXRDeviceCfg, XrCfg
from isaaclab.devices.openxr.retargeters.humanoid.unitree.inspire.g1_upper_body_retargeter import (
    UnitreeG1RetargeterCfg,
)
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.pink_actions_cfg import PinkInverseKinematicsActionCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass

from .mdp import observations as sorting_obs
from .mdp import terminations as sorting_term
from .mdp import events as sorting_events
from . import mdp

from isaaclab_assets.robots.unitree import G1_INSPIRE_FTP_CFG  # isort: skip

# ============================================================================
# Table constants (critical for object placement)
# ============================================================================
from .constants import TABLE_HEIGHT, TABLE_SURFACE_Z  # noqa: E402


# ============================================================================
# Scene definition
# ============================================================================

@configclass
class KitchenSortingSceneCfg(InteractiveSceneCfg):
    """G1 Kitchen Sorting scene with table, bins, and objects.

    Layout (top-down view):
        BinKitchen(-0.28,0.82)  BinFood(0,0.82)  BinMisc(0.28,0.82)  Y=0.82
        ┌───────────────────────────────────────────────────┐
        │   Objects scattered on table (Y=0.40~0.70)        │  Y=0.55
        └───────────────────────────────────────────────────┘
                        Robot G1 (0.5, 0, 0)                     Y=0.0
    """

    # -- Robot (G1 with INSPIRE 5-finger hand, fixed base) --
    robot: ArticulationCfg = G1_INSPIRE_FTP_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 1),
            rot=(0.7071, 0, 0, 0.7071),  # 90° Z-rotation → face +Y direction
            joint_pos={
                # arms
                "right_shoulder_pitch_joint": 0.0,
                "right_shoulder_roll_joint": 0.0,
                "right_shoulder_yaw_joint": 0.0,
                "right_elbow_joint": 0.0,
                "right_wrist_yaw_joint": 0.0,
                "right_wrist_roll_joint": 0.0,
                "right_wrist_pitch_joint": 0.0,
                "left_shoulder_pitch_joint": 0.0,
                "left_shoulder_roll_joint": 0.0,
                "left_shoulder_yaw_joint": 0.0,
                "left_elbow_joint": 0.0,
                "left_wrist_yaw_joint": 0.0,
                "left_wrist_roll_joint": 0.0,
                "left_wrist_pitch_joint": 0.0,
                # waist, legs
                "waist_.*": 0.0,
                ".*_hip_.*": 0.0,
                ".*_knee_.*": 0.0,
                ".*_ankle_.*": 0.0,
                # hands
                ".*_thumb_.*": 0.0,
                ".*_index_.*": 0.0,
                ".*_middle_.*": 0.0,
                ".*_ring_.*": 0.0,
                ".*_pinky_.*": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
    )

    # -- Table (cuboid primitive kitchen table, 0.9x0.6x0.75m) --
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.55, TABLE_HEIGHT / 2),  # Cuboid center = half height
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.9, 0.6, TABLE_HEIGHT),  # 90cm width × 60cm depth × 75cm height
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.6, 0.4, 0.2),  # wood color
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        ),
    )

    # -- Bins (3 sorting destination bins, kinematic) --
    # BinKitchen: left (blue) - for mug, plate, bowl
    bin_kitchen = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/BinKitchen",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.28, 0.82, TABLE_SURFACE_Z + 0.05), rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.25, 0.18, 0.10),  # sorting bin: 25cm x 18cm x 10cm
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.4, 0.8)),  # blue
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "bin_kitchen")],
        ),
    )

    # BinFood: center (blue) - for can, bottle, apple
    bin_food = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/BinFood",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.82, TABLE_SURFACE_Z + 0.05), rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.25, 0.18, 0.10),  # sorting bin: 25cm x 18cm x 10cm
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.4, 0.8)),  # blue
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "bin_food")],
        ),
    )

    # BinMisc: right (black) - for sponge, teddy
    bin_misc = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/BinMisc",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.28, 0.82, TABLE_SURFACE_Z + 0.05), rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.25, 0.18, 0.10),  # sorting bin: 25cm x 18cm x 10cm
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15)),  # black
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "bin_misc")],
        ),
    )

    # -- Category A: Kitchen objects (→ BinKitchen) --
    mug = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Mug",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.15, 0.55, TABLE_SURFACE_Z + 0.04), rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CylinderCfg(
            radius=0.04,
            height=0.08,  # mug shape: 8cm tall, 4cm radius
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.7, 0.5, 0.3)),  # ceramic brown
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                max_angular_velocity=1000.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.25),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "mug"), ("category", "kitchen")],
        ),
    )

    plate = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Plate",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.10, 0.50, TABLE_SURFACE_Z + 0.02), rot=(1, 0, 0, 0),  # surface + half plate height
        ),
        spawn=sim_utils.CylinderCfg(
            radius=0.10,
            height=0.02,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.95, 0.95)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.35),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "plate"), ("category", "kitchen")],
        ),
    )

    bowl = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Bowl",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.05, 0.60, TABLE_SURFACE_Z + 0.025), rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.CylinderCfg(
            radius=0.07,
            height=0.05,  # bowl shape: 5cm tall, 7cm radius (wide & shallow)
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.85, 0.2)),  # yellow
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.20),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "bowl"), ("category", "kitchen")],
        ),
    )

    # -- Category B: Food objects (→ BinFood) --
    can = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Can",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.18, 0.52, TABLE_SURFACE_Z + 0.06), rot=(1, 0, 0, 0),  # surface + half can height + margin
        ),
        spawn=sim_utils.CylinderCfg(
            radius=0.033,
            height=0.10,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.1, 0.1)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.30),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "can"), ("category", "food")],
        ),
    )

    bottle = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Bottle",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.20, 0.48, TABLE_SURFACE_Z + 0.09), rot=(1, 0, 0, 0),  # surface + half bottle height
        ),
        spawn=sim_utils.CylinderCfg(
            radius=0.03,
            height=0.18,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.6, 0.2)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.40),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "bottle"), ("category", "food")],
        ),
    )

    apple = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Apple",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.08, 0.45, TABLE_SURFACE_Z + 0.04), rot=(1, 0, 0, 0),  # surface + apple radius
        ),
        spawn=sim_utils.SphereCfg(
            radius=0.04,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.18),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "apple"), ("category", "food")],
        ),
    )

    # -- Category C: Misc objects (→ BinMisc) --
    sponge = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Sponge",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(-0.22, 0.58, TABLE_SURFACE_Z + 0.015), rot=(1, 0, 0, 0),  # surface + half sponge height
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.10, 0.06, 0.03),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.9, 0.1)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.03),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "sponge"), ("category", "misc")],
        ),
    )

    teddy = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TeddyBear",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.25, 0.62, TABLE_SURFACE_Z + 0.05), rot=(1, 0, 0, 0),
        ),
        spawn=sim_utils.SphereCfg(
            radius=0.05,  # teddy bear: ~10cm diameter sphere
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.4, 0.2)),  # brown
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.15),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            semantic_tags=[("class", "teddy_bear"), ("category", "misc")],
        ),
    )

    # -- Ground plane --
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=GroundPlaneCfg(),
    )

    # -- Lights --
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(
            color=(0.95, 0.90, 0.80),  # warm kitchen tone
            intensity=3000.0,
        ),
    )

    spot_light = AssetBaseCfg(
        prim_path="/World/SpotLight",
        spawn=sim_utils.DistantLightCfg(
            color=(1.0, 1.0, 1.0),
            intensity=1500.0,
        ),
    )


# ============================================================================
# USD-based scene variant (ust_human1.usd 커스텀 씬 사용)
# ============================================================================

# USD 씬 파일 경로 (Isaac Sim GUI에서 모델링한 전체 씬)
import os as _os
_UST_HUMAN1_USD_PATH = _os.path.join(
    _os.path.dirname(_os.path.abspath(__file__)),
    "..", "isaac_file", "ust_human1.usd",
)


@configclass
class KitchenSortingUSDSceneCfg(InteractiveSceneCfg):
    """커스텀 USD 씬 기반 G1 주방 분류 환경.

    ust_ws/isaac_file/ust_human1.usd 에 테이블, 빈, 물체, 조명이
    포함되어 있음 (로봇은 USD에서 제거됨).
    로봇만 코드에서 스폰하여 물리 제어를 가능하게 함.
    물체/빈은 USD의 시각적 에셋만 사용 (물리 시뮬레이션 없음).
    """

    # -- 커스텀 USD 배경 씬 (테이블, 빈, 조명, 바닥 포함) --
    background = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Background",
        spawn=UsdFileCfg(
            usd_path=_UST_HUMAN1_USD_PATH,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    # -- Robot (코드에서 스폰, USD에서 g1 로봇은 제거됨) --
    # 위치/회전은 USD 씬의 원래 g1 배치와 동일하게 설정
    robot: ArticulationCfg = G1_INSPIRE_FTP_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(2.1459, -0.7785, 0.9796),
            rot=(0.7071, 0, 0, 0.7071),
            joint_pos={
                "right_shoulder_pitch_joint": 0.0,
                "right_shoulder_roll_joint": 0.0,
                "right_shoulder_yaw_joint": 0.0,
                "right_elbow_joint": 0.0,
                "right_wrist_yaw_joint": 0.0,
                "right_wrist_roll_joint": 0.0,
                "right_wrist_pitch_joint": 0.0,
                "left_shoulder_pitch_joint": 0.0,
                "left_shoulder_roll_joint": 0.0,
                "left_shoulder_yaw_joint": 0.0,
                "left_elbow_joint": 0.0,
                "left_wrist_yaw_joint": 0.0,
                "left_wrist_roll_joint": 0.0,
                "left_wrist_pitch_joint": 0.0,
                "waist_.*": 0.0,
                ".*_hip_.*": 0.0,
                ".*_knee_.*": 0.0,
                ".*_ankle_.*": 0.0,
                ".*_thumb_.*": 0.0,
                ".*_index_.*": 0.0,
                ".*_middle_.*": 0.0,
                ".*_ring_.*": 0.0,
                ".*_pinky_.*": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
    )

    # 참고: 물체, 빈, 테이블, 조명, 바닥은 모두 USD에 포함되어 있으므로
    # 코드에서 별도로 스폰하지 않음. 텔레오퍼레이션 데모 수집 시
    # USD 씬의 시각적 에셋만 사용하고, 물리 기반 분류 판정은 하지 않음.


# ============================================================================
# Vision scene variant (Isaac Sim 시뮬레이션 카메라 - 물리 카메라 없음)
# ============================================================================

@configclass
class KitchenSortingVisionSceneCfg(KitchenSortingSceneCfg):
    """시뮬레이션 내부 가상 카메라가 포함된 씬.

    물리적 카메라는 사용하지 않으며, Isaac Sim의 RTX 렌더러가
    시뮬레이션 내부에서 가상 카메라 이미지를 생성합니다.
    --enable_cameras CLI 플래그 필요.

    카메라 데이터 접근:
        env.scene["robot_head_cam"].data.output["rgb"]    # (N, H, W, 4) RGBA
        env.scene["overhead_cam"].data.output["rgb"]      # (N, H, W, 4) RGBA
        env.scene["robot_head_cam"].data.output["distance_to_camera"]  # (N, H, W, 1)
    """

    # 로봇 헤드 카메라 (시뮬레이션 가상 카메라 - pelvis에 부착, 테이블 하향)
    robot_head_cam = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/pelvis/HeadCamera",
        update_period=0.0167,  # ~60 FPS (RTX PRO 6000 can handle this)
        height=720,
        width=1280,  # 720p (RTX PRO 6000 supports 1080p but 720p is efficient)
        data_types=["rgb", "semantic_segmentation", "distance_to_camera"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.15,
            focus_distance=0.6,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 3.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.0, 0.12, 0.68),  # above pelvis, looking at table
            rot=(-0.259, 0.966, 0.0, 0.0),  # ~30 degree downward tilt
            convention="ros",
        ),
        semantic_filter="class:*",
    )

    # 오버헤드 카메라 (시뮬레이션 가상 카메라 - 테이블 전체 조감)
    overhead_cam = CameraCfg(
        prim_path="{ENV_REGEX_NS}/OverheadCamera",
        update_period=0.033,  # 30 FPS (less critical than head cam)
        height=720,
        width=1280,
        data_types=["rgb", "semantic_segmentation"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=12.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 5.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.0, 0.55, 2.0),  # 2m above table center
            rot=(0.0, 1.0, 0.0, 0.0),  # looking straight down
            convention="world",
        ),
        semantic_filter="class:*",
    )


# ============================================================================
# Action configuration (Pink IK + INSPIRE hand)
# ============================================================================

@configclass
class ActionsCfg:
    """Action specifications: dual-arm Pink IK + 24-DOF hand control."""

    pink_ik_cfg = PinkInverseKinematicsActionCfg(
        pink_controlled_joint_names=[
            ".*_shoulder_pitch_joint",
            ".*_shoulder_roll_joint",
            ".*_shoulder_yaw_joint",
            ".*_elbow_joint",
            ".*_wrist_yaw_joint",
            ".*_wrist_roll_joint",
            ".*_wrist_pitch_joint",
        ],
        hand_joint_names=[
            # Left proximal (5)
            "L_index_proximal_joint",
            "L_middle_proximal_joint",
            "L_pinky_proximal_joint",
            "L_ring_proximal_joint",
            "L_thumb_proximal_yaw_joint",
            # Right proximal (5)
            "R_index_proximal_joint",
            "R_middle_proximal_joint",
            "R_pinky_proximal_joint",
            "R_ring_proximal_joint",
            "R_thumb_proximal_yaw_joint",
            # Left intermediate (5)
            "L_index_intermediate_joint",
            "L_middle_intermediate_joint",
            "L_pinky_intermediate_joint",
            "L_ring_intermediate_joint",
            "L_thumb_proximal_pitch_joint",
            # Right intermediate (5)
            "R_index_intermediate_joint",
            "R_middle_intermediate_joint",
            "R_pinky_intermediate_joint",
            "R_ring_intermediate_joint",
            "R_thumb_proximal_pitch_joint",
            # Thumb distal (4)
            "L_thumb_intermediate_joint",
            "R_thumb_intermediate_joint",
            "L_thumb_distal_joint",
            "R_thumb_distal_joint",
        ],
        target_eef_link_names={
            "left_wrist": "left_wrist_yaw_link",
            "right_wrist": "right_wrist_yaw_link",
        },
        asset_name="robot",
        controller=PinkIKControllerCfg(
            articulation_name="robot",
            base_link_name="pelvis",
            num_hand_joints=24,
            show_ik_warnings=False,
            fail_on_joint_limit_violation=False,
            variable_input_tasks=[
                FrameTask(
                    "g1_29dof_rev_1_0_left_wrist_yaw_link",
                    position_cost=8.0,
                    orientation_cost=2.0,
                    lm_damping=10,
                    gain=0.5,
                ),
                FrameTask(
                    "g1_29dof_rev_1_0_right_wrist_yaw_link",
                    position_cost=8.0,
                    orientation_cost=2.0,
                    lm_damping=10,
                    gain=0.5,
                ),
                NullSpacePostureTask(
                    cost=0.5,
                    lm_damping=1,
                    controlled_frames=[
                        "g1_29dof_rev_1_0_left_wrist_yaw_link",
                        "g1_29dof_rev_1_0_right_wrist_yaw_link",
                    ],
                    controlled_joints=[
                        "left_shoulder_pitch_joint",
                        "left_shoulder_roll_joint",
                        "left_shoulder_yaw_joint",
                        "right_shoulder_pitch_joint",
                        "right_shoulder_roll_joint",
                        "right_shoulder_yaw_joint",
                        "waist_yaw_joint",
                        "waist_pitch_joint",
                        "waist_roll_joint",
                    ],
                    gain=0.3,
                ),
            ],
            fixed_input_tasks=[],
            xr_enabled=bool(carb.settings.get_settings().get("/app/xr/enabled")),
        ),
        enable_gravity_compensation=False,
    )


# ============================================================================
# Observation configuration
# ============================================================================

@configclass
class ObservationsCfg:
    """Observation specifications for kitchen sorting."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for the policy."""

        # Last actions
        actions = ObsTerm(func=mdp.last_action)

        # Robot state
        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_root_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("robot")})

        # All robot link states
        robot_links_state = ObsTerm(func=sorting_obs.get_all_robot_link_state)

        # EEF state
        left_eef_pos = ObsTerm(func=sorting_obs.get_eef_pos, params={"link_name": "left_wrist_yaw_link"})
        left_eef_quat = ObsTerm(func=sorting_obs.get_eef_quat, params={"link_name": "left_wrist_yaw_link"})
        right_eef_pos = ObsTerm(func=sorting_obs.get_eef_pos, params={"link_name": "right_wrist_yaw_link"})
        right_eef_quat = ObsTerm(func=sorting_obs.get_eef_quat, params={"link_name": "right_wrist_yaw_link"})

        # Hand joint state
        hand_joint_state = ObsTerm(
            func=sorting_obs.get_robot_joint_state,
            params={"joint_names": ["R_.*", "L_.*"]},
        )

        # Object positions and rotations
        object_positions = ObsTerm(func=sorting_obs.object_positions)
        object_rotations = ObsTerm(func=sorting_obs.object_rotations)

        # Bin positions
        bin_positions = ObsTerm(func=sorting_obs.bin_positions)

        # EEF-to-object distances
        eef_to_objects = ObsTerm(func=sorting_obs.eef_to_nearest_object)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


# ============================================================================
# Reward configuration
# ============================================================================

@configclass
class RewardsCfg:
    """Reward terms for kitchen sorting."""

    sorting_progress = RewTerm(
        func=sorting_term.count_sorted_objects,
        weight=1.0,
        params={"threshold": 0.12},
    )


# ============================================================================
# Termination configuration
# ============================================================================

@configclass
class TerminationsCfg:
    """Termination terms for kitchen sorting."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    all_sorted = DoneTerm(
        func=sorting_term.all_objects_sorted,
        params={"threshold": 0.12},
    )


# ============================================================================
# Event configuration (domain randomization)
# ============================================================================

@configclass
class EventCfg:
    """Event terms for resetting/randomizing the scene."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_objects = EventTerm(
        func=sorting_events.reset_sorting_objects,
        mode="reset",
        params={
            "pose_range": {
                "x": [-0.25, 0.25],   # full table X range
                "y": [-0.10, 0.10],   # Y scatter around default
                "yaw": [-3.14, 3.14],  # full 360° rotation
            },
        },
    )


# ============================================================================
# USD 씬 전용 MDP Config (물체/빈 없는 로봇 전용)
# ============================================================================

@configclass
class USDObservationsCfg:
    """USD 씬 전용 관측 설정. 물체/빈 관련 관측 제거, 로봇 상태만 포함."""

    @configclass
    class PolicyCfg(ObsGroup):
        """로봇 상태 전용 관측."""

        actions = ObsTerm(func=mdp.last_action)

        robot_joint_pos = ObsTerm(
            func=base_mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        robot_root_pos = ObsTerm(func=base_mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg("robot")})
        robot_root_rot = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("robot")})

        robot_links_state = ObsTerm(func=sorting_obs.get_all_robot_link_state)

        left_eef_pos = ObsTerm(func=sorting_obs.get_eef_pos, params={"link_name": "left_wrist_yaw_link"})
        left_eef_quat = ObsTerm(func=sorting_obs.get_eef_quat, params={"link_name": "left_wrist_yaw_link"})
        right_eef_pos = ObsTerm(func=sorting_obs.get_eef_pos, params={"link_name": "right_wrist_yaw_link"})
        right_eef_quat = ObsTerm(func=sorting_obs.get_eef_quat, params={"link_name": "right_wrist_yaw_link"})

        hand_joint_state = ObsTerm(
            func=sorting_obs.get_robot_joint_state,
            params={"joint_names": ["R_.*", "L_.*"]},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class USDTerminationsCfg:
    """USD 씬 전용 종료 조건. 시간 초과만 사용 (물체 분류 판정 없음)."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class USDRewardsCfg:
    """USD 씬 전용 보상. 텔레오퍼레이션 모드에서는 보상 불필요."""

    pass


@configclass
class USDEventCfg:
    """USD 씬 전용 이벤트. 물체 리셋 없이 씬 기본값 리셋만 수행."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


# ============================================================================
# USD 씬 + VR 텔레오퍼레이션 환경 (PICO + UDCAP 용)
# ============================================================================

@configclass
class KitchenSortingG1USDEnvCfg(ManagerBasedRLEnvCfg):
    """커스텀 USD 씬 기반 G1 주방 분류 환경.

    ust_human1.usd를 배경으로 사용하며, PICO 텔레오퍼레이션과 함께 동작.
    물체/빈은 USD에 시각적으로 배치되어 있으며 코드에서 별도 스폰하지 않음.
    """

    scene: KitchenSortingUSDSceneCfg = KitchenSortingUSDSceneCfg(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )

    observations: USDObservationsCfg = USDObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: USDTerminationsCfg = USDTerminationsCfg()
    rewards: USDRewardsCfg = USDRewardsCfg()
    events = USDEventCfg()

    commands = None
    curriculum = None

    # XR 앵커: 로봇 1인칭 시점
    # 로봇 pelvis pos=(2.1459, -0.7785, 0.9796), rot=90° Z (Y+ 방향 바라봄)
    # anchor_pos: VR 원점(바닥)에 매핑되는 시뮬레이션 위치
    # anchor_rot: VR 정면이 시뮬레이션의 어느 방향을 바라보는지 결정
    #   (0.7071, 0, 0, -0.7071) = Z축 -90° → VR 정면이 Y+ 방향(주방 카운터)을 향함
    xr: XrCfg = XrCfg(
        anchor_pos=(2.1459, -0.7785, 0.0),
        anchor_rot=(0.7071, 0.0, 0.0, -0.7071),
    )

    temp_urdf_dir = tempfile.gettempdir()

    idle_action = torch.tensor([
        -0.1487, 0.2038, 1.0952, 0.707, 0.0, 0.0, 0.707,
        0.1487, 0.2038, 1.0952, 0.707, 0.0, 0.0, 0.707,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ])

    def __post_init__(self):
        """Post initialization."""
        self.decimation = 6
        self.episode_length_s = 3600.0  # 1시간 (VR 텔레옵용 긴 에피소드)

        self.sim.dt = 1 / 120
        self.sim.render_interval = 2  # ~60Hz render for VR
        self.sim.disable_contact_processing = False
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.enable_scene_query_support = True

        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=0.7,
            dynamic_friction=0.5,
            restitution=0.1,
        )

        temp_urdf_output_path, temp_urdf_meshes_output_path = ControllerUtils.convert_usd_to_urdf(
            self.scene.robot.spawn.usd_path, self.temp_urdf_dir, force_conversion=True
        )
        self.actions.pink_ik_cfg.controller.urdf_path = temp_urdf_output_path
        self.actions.pink_ik_cfg.controller.mesh_path = temp_urdf_meshes_output_path

        self.teleop_devices = DevicesCfg(
            devices={
                "handtracking": OpenXRDeviceCfg(
                    retargeters=[
                        UnitreeG1RetargeterCfg(
                            enable_visualization=True,
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
                        UnitreeG1RetargeterCfg(
                            enable_visualization=True,
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

        self.pico_device_cfg = {
            "bridge_host": "localhost",
            "bridge_port": 8889,
            "use_udcap_fingers": True,
            "position_scale": 1.0,
            "rotation_scale": 1.0,
            "gripper_threshold": 0.5,
            "body_pos_offset": (0.0, 0.0, 0.0),
            "controller_pos_offset": (0.0, 0.0, 0.0),
            "debug": True,
        }


# ============================================================================
# Main environment configuration (VR Teleoperation - RTX PRO 6000)
# ============================================================================

@configclass
class KitchenSortingG1EnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for G1 Kitchen Sorting environment.

    Default: Single-env VR teleoperation mode.
    GPU: RTX PRO 6000 (96GB VRAM) - render every frame, high physics fidelity.
    """

    # Scene (base - no cameras for teleop mode)
    scene: KitchenSortingSceneCfg = KitchenSortingSceneCfg(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )

    # MDP
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    events = EventCfg()

    # Unused
    commands = None
    curriculum = None

    # XR anchor
    xr: XrCfg = XrCfg(
        anchor_pos=(0.0, 0.0, 0.0),
        anchor_rot=(1.0, 0.0, 0.0, 0.0),
    )

    # Temporary directory for URDF files
    temp_urdf_dir = tempfile.gettempdir()

    # Idle action (38D: 14 EEF + 24 hand joints) - hold default pose
    idle_action = torch.tensor([
        # Left EEF: pos(3) + quat(4)
        -0.1487, 0.2038, 1.0952, 0.707, 0.0, 0.0, 0.707,
        # Right EEF: pos(3) + quat(4)
        0.1487, 0.2038, 1.0952, 0.707, 0.0, 0.0, 0.707,
        # 24 hand joints (all zero)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ])

    def __post_init__(self):
        """Post initialization."""
        # Timing
        self.decimation = 6
        self.episode_length_s = 60.0

        # Physics - RTX PRO 6000 can handle high fidelity
        self.sim.dt = 1 / 120  # 120 Hz physics
        self.sim.render_interval = 1  # render every frame (RTX PRO 6000)
        self.sim.disable_contact_processing = False
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024

        # Scene query support (CRITICAL for VR rendering and CloudXR)
        self.sim.enable_scene_query_support = True

        # Global physics material (friction for all contacts)
        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=0.7,
            dynamic_friction=0.5,
            restitution=0.1,
        )

        # Convert USD to URDF for Pink IK controller
        temp_urdf_output_path, temp_urdf_meshes_output_path = ControllerUtils.convert_usd_to_urdf(
            self.scene.robot.spawn.usd_path, self.temp_urdf_dir, force_conversion=True
        )
        self.actions.pink_ik_cfg.controller.urdf_path = temp_urdf_output_path
        self.actions.pink_ik_cfg.controller.mesh_path = temp_urdf_meshes_output_path

        # Teleop devices
        self.teleop_devices = DevicesCfg(
            devices={
                "handtracking": OpenXRDeviceCfg(
                    retargeters=[
                        UnitreeG1RetargeterCfg(
                            enable_visualization=True,
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
                        UnitreeG1RetargeterCfg(
                            enable_visualization=True,
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

        # PICO + UDCAP 디바이스 설정 (XRoboToolkit 경유, CloudXR과 독립)
        self.pico_device_cfg = {
            "bridge_host": "localhost",    # Docker network_mode:host → localhost
            "bridge_port": 8889,           # 통합 브릿지 포트
            "use_udcap_fingers": True,     # UDCAP VR Gloves 사용
            "position_scale": 1.0,         # 사용자 키 ÷ G1 키 = ~1.0 (1.7m / 1.32m → ~1.3 권장)
            "rotation_scale": 1.0,
            "gripper_threshold": 0.5,
            # PICO Body 좌표 → G1 pelvis 프레임 평행이동
            # PICO Body 원점 = 캘리브레이션 시 사용자 pelvis 위치 (월드)
            # G1 pelvis idle z = 1.05m. body 데이터의 wrist는 사용자 wrist 절대 좌표이므로
            # 보통 (0, 0, 0) 오프셋으로 시작 후 사용자에 맞춰 미세조정.
            "body_pos_offset": (0.0, 0.0, 0.0),
            "controller_pos_offset": (0.0, 0.0, 0.0),
            "debug": True,
        }


# ============================================================================
# 비전 환경 (시뮬레이션 가상 카메라 포함, 비전 기반 학습용)
# ============================================================================

@configclass
class KitchenSortingG1VisionEnvCfg(KitchenSortingG1EnvCfg):
    """시뮬레이션 내부 가상 카메라가 포함된 환경.

    물리적 카메라 없음. Isaac Sim RTX 렌더러가 시뮬레이션 내에서
    가상 카메라 이미지(RGB, 깊이, 시맨틱)를 생성합니다.
    --enable_cameras CLI 플래그 필요.
    RTX PRO 6000: 720p 시뮬레이션 카메라 60 FPS.
    """

    scene: KitchenSortingVisionSceneCfg = KitchenSortingVisionSceneCfg(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )


# ============================================================================
# Training environment (multi-env, no cameras, for RL training)
# ============================================================================

@configclass
class KitchenSortingG1TrainEnvCfg(KitchenSortingG1EnvCfg):
    """Multi-environment config for policy training.

    RTX PRO 6000 (96GB VRAM): supports up to 16 envs with full physics.
    No cameras, render_interval=2 for speed.
    """

    scene: KitchenSortingSceneCfg = KitchenSortingSceneCfg(
        num_envs=16,
        env_spacing=3.0,
        replicate_physics=True,
    )

    def __post_init__(self):
        super().__post_init__()
        self.sim.render_interval = 2  # reduce render overhead for training
        self.sim.enable_scene_query_support = False  # not needed for headless training
        self.episode_length_s = 120.0  # longer episodes for training


# ============================================================================
# VR Teleoperation environment (long episodes for VR sessions)
# ============================================================================

@configclass
class KitchenSortingG1VREnvCfg(KitchenSortingG1EnvCfg):
    """VR teleoperation environment with CloudXR.

    Optimized for live VR sessions via Quest 3S / Vision Pro.
    Long episode (1 hour) to avoid interruptions during demo collection.
    render_interval=2 for stable VR frame rate with CloudXR.
    """

    def __post_init__(self):
        super().__post_init__()
        # VR-optimized timing
        self.sim.render_interval = 2  # ~60Hz render (120Hz physics / 2)
        self.episode_length_s = 3600.0  # 1 hour for uninterrupted VR sessions
        # Scene query is already True from base class


# ============================================================================
# Monitor 텔레오퍼레이션 환경 (XR 없음, PC 모니터에서만 렌더링)
# ============================================================================

@configclass
class KitchenSortingG1MonitorEnvCfg(KitchenSortingG1EnvCfg):
    """모니터 뷰 텔레오퍼레이션 환경 (방안3).

    XR/CloudXR/SteamVR 없이 PC 모니터에서만 Isaac Lab 화면을 렌더링합니다.
    PICO XRoboToolkit으로 전신 트래킹, UDCAP으로 손가락 트래킹만 사용.
    VR 헤드셋에 렌더링이 불필요하므로 즉시 사용 가능합니다.

    장점:
      - CloudXR/SteamVR 설정 불필요
      - XRoboToolkit + UDCAP만으로 전신+손가락 제어 가능
      - 디버깅/개발에 최적

    실행:
      python run_teleop.py --teleop_device pico --render_mode monitor
    """

    def __post_init__(self):
        super().__post_init__()
        # 모니터 렌더링: 매 프레임 렌더 (VR 프레임 드롭 걱정 없음)
        self.sim.render_interval = 1
        # 1시간 에피소드 (텔레옵 세션)
        self.episode_length_s = 3600.0
        # Scene query 불필요 (XR 렌더링 없음) → 약간의 성능 향상
        self.sim.enable_scene_query_support = False


# ============================================================================
# 데이터 수집 환경 (시뮬레이션 가상 카메라 + VR 텔레옵 데모 녹화)
# ============================================================================

@configclass
class KitchenSortingG1DataCollectEnvCfg(KitchenSortingG1EnvCfg):
    """VR 텔레옵 데모 녹화 + 시뮬레이션 카메라 이미지 캡처 환경.

    물리적 카메라 없음. Isaac Sim RTX 렌더러가 시뮬레이션 내에서
    가상 카메라 이미지를 생성하여 HDF5에 함께 저장합니다.
    비전 기반 정책 학습 (HG-DAgger Phase 2+) 시 이미지 데이터 필요.
    --enable_cameras CLI 플래그 필요.

    시뮬레이션 카메라 이미지 접근:
        env.scene["robot_head_cam"].data.output["rgb"]    # (N, H, W, 4)
        env.scene["overhead_cam"].data.output["rgb"]      # (N, H, W, 4)
    """

    scene: KitchenSortingVisionSceneCfg = KitchenSortingVisionSceneCfg(
        num_envs=1,
        env_spacing=3.0,
        replicate_physics=True,
    )

    def __post_init__(self):
        super().__post_init__()
        self.sim.render_interval = 2  # 안정적 VR 렌더 속도
        self.episode_length_s = 3600.0  # VR 세션과 동일 (1시간)
