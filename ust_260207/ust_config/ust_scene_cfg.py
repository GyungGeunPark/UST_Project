# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST Mobile Manipulator Scene Configuration."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, RayCasterCfg, patterns
from isaaclab.utils import configclass

from .ust_mobile_manipulator_cfg import UST_MOBILE_MANIPULATOR_CFG


@configclass
class USTSceneCfg(InteractiveSceneCfg):
    """UST 모바일 매니퓰레이터 씬 설정.

    이 설정은 TurtleBot3 Waffle Pi + OpenMANIPULATOR-X 로봇과
    함께 RealSense D455 카메라, Livox MID-360 LiDAR를 포함합니다.
    """

    # 환경 병렬화 설정
    num_envs: int = 1
    env_spacing: float = 5.0

    # 지면 평면
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
            ),
            size=(100.0, 100.0),
        ),
    )

    # 돔 라이트 (환경 조명)
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(1.0, 1.0, 1.0),
        ),
    )

    # 디스턴트 라이트 (방향성 조명)
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(
            intensity=500.0,
            color=(1.0, 0.98, 0.95),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            rot=(0.866, 0.0, 0.5, 0.0),  # 45도 각도
        ),
    )

    # 로봇 (TurtleBot3 + OpenMANIPULATOR-X)
    robot = UST_MOBILE_MANIPULATOR_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    # 조작 대상 객체 (큐브) - 지면 위에 배치
    target_object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetObject",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                max_depenetration_velocity=1.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 0.8, 0.0),  # 녹색
                metallic=0.2,
                roughness=0.6,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.3, 0.0, 0.02),  # 로봇 전방 30cm, 지면 위
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )


@configclass
class USTSceneWithSensorsCfg(USTSceneCfg):
    """센서를 포함한 UST 씬 설정.

    RealSense D455 RGB-D 카메라와 Livox MID-360 LiDAR를 포함합니다.
    """

    # RGB-D 카메라 (Intel RealSense D455)
    # NOTE: camera_link는 USD에 이미 Xform으로 존재하므로,
    # 카메라 prim은 그 아래 새 경로에 spawn해야 합니다.
    camera_d455 = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/realsense2_camera/camera_link/CameraSensor",
        update_period=0.033,  # 30 FPS
        height=720,
        width=1280,
        data_types=["rgb", "distance_to_camera"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.93,  # D455 focal length (mm)
            focus_distance=400.0,
            horizontal_aperture=3.6,
            clipping_range=(0.4, 6.0),  # D455 depth range
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(0.5, -0.5, 0.5, -0.5),  # 카메라 방향 조정
            convention="ros",
        ),
    )

    # LiDAR (Livox MID-360) - RayCaster로 시뮬레이션
    # livox_mid360/livox 가 실제 RigidBody (물리 시뮬레이션으로 로봇과 함께 이동)
    lidar_mid360 = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/livox_mid360/livox",
        update_period=0.1,  # 10 Hz
        offset=RayCasterCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        attach_yaw_only=True,
        pattern_cfg=patterns.LidarPatternCfg(
            channels=32,
            vertical_fov_range=(-15.0, 15.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=1.0,
        ),
        max_distance=40.0,
        mesh_prim_paths=["/World/ground"],
    )


@configclass
class USTSimpleSceneCfg(InteractiveSceneCfg):
    """간단한 UST 씬 설정 (센서 없이).

    빠른 테스트와 학습을 위한 경량 씬입니다.
    """

    num_envs: int = 1
    env_spacing: float = 5.0

    # 지면
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # 조명
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0),
    )

    # 로봇
    robot = UST_MOBILE_MANIPULATOR_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    # 대상 객체
    target_object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetObject",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.8, 0.0)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.3, 0.0, 0.02)),
    )
