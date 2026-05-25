# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST Mobile Manipulator Teleoperation Environment Configuration."""

from __future__ import annotations

from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
import isaaclab.envs.mdp as mdp
from isaaclab.utils import configclass

from .ust_scene_cfg import USTSceneCfg, USTSceneWithSensorsCfg, USTSimpleSceneCfg
from .ust_actions_cfg import USTActionsCfg, USTTeleopActionsCfg, USTSimpleActionsCfg
from .ust_observations_cfg import (
    USTObservationsCfg,
    USTTeleopObservationsCfg,
    USTSimpleObservationsCfg,
)


@configclass
class USTEventsCfg:
    """UST 환경 이벤트 설정.

    리셋 시 발생하는 이벤트들을 정의합니다.
    """

    # 로봇 초기화
    reset_robot = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": mdp.SceneEntityCfg("robot"),
            "pose_range": {
                "x": (-0.1, 0.1),
                "y": (-0.1, 0.1),
                "yaw": (-0.1, 0.1),
            },
            "velocity_range": {},
        },
    )

    # 객체 초기화 - 지면 위에 랜덤 배치
    reset_object = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": mdp.SceneEntityCfg("target_object"),
            "pose_range": {
                "x": (0.25, 0.35),
                "y": (-0.1, 0.1),
                "z": (0.02, 0.02),
            },
            "velocity_range": {},
        },
    )


@configclass
class USTTerminationsCfg:
    """UST 환경 종료 조건 설정."""

    # 시간 제한
    time_out = DoneTerm(
        func=mdp.time_out,
        time_out=True,
    )


@configclass
class USTMobileManipulatorTeleopEnvCfg(ManagerBasedEnvCfg):
    """UST 모바일 매니퓰레이터 텔레오퍼레이션 환경 설정.

    VR 텔레오퍼레이션을 위한 전체 환경 구성입니다.

    주요 특징:
    - TurtleBot3 Waffle Pi + OpenMANIPULATOR-X 로봇
    - RealSense D455 RGB-D 카메라
    - Livox MID-360 LiDAR
    - VR 핸드 트래킹 기반 제어
    """

    # 씬 설정
    scene: USTSceneCfg = USTSceneCfg(num_envs=1, env_spacing=5.0)

    # 액션/관측 설정
    actions: USTActionsCfg = USTActionsCfg()
    observations: USTObservationsCfg = USTObservationsCfg()

    # 이벤트 설정
    events: USTEventsCfg = USTEventsCfg()

    # 종료 조건
    terminations: USTTerminationsCfg = USTTerminationsCfg()

    def __post_init__(self):
        """환경 초기화 후 시뮬레이션 파라미터 설정."""
        # 시뮬레이션 타이밍
        self.decimation = 4  # 50Hz 제어 (200Hz 물리 / 4)
        self.sim.dt = 0.005  # 200Hz 물리 시뮬레이션
        self.sim.render_interval = self.decimation

        # GPU 물리 최적화
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024

        # Scene Query 활성화 (VR 렌더링용)
        self.sim.enable_scene_query_support = True

        # 에피소드 길이
        self.episode_length_s = 60.0  # 60초


@configclass
class USTMobileManipulatorVREnvCfg(ManagerBasedEnvCfg):
    """VR 텔레오퍼레이션 전용 환경 설정.

    CloudXR을 통한 Quest 3S 연결을 위한 최적화된 설정입니다.
    """

    # 씬 설정 (센서 포함)
    scene: USTSceneWithSensorsCfg = USTSceneWithSensorsCfg(num_envs=1, env_spacing=5.0)

    # 텔레옵 전용 액션/관측
    actions: USTTeleopActionsCfg = USTTeleopActionsCfg()
    observations: USTTeleopObservationsCfg = USTTeleopObservationsCfg()

    # 이벤트 설정
    events: USTEventsCfg = USTEventsCfg()

    def __post_init__(self):
        """VR 환경 초기화."""
        # 시뮬레이션 타이밍 (VR 최적화)
        self.decimation = 4
        self.sim.dt = 0.005  # 200Hz
        self.sim.render_interval = 2  # 100Hz 렌더링 (VR용)

        # GPU 물리
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024

        # VR 렌더링 지원
        self.sim.enable_scene_query_support = True

        # 에피소드 길이 (무제한에 가깝게)
        self.episode_length_s = 3600.0  # 1시간


@configclass
class USTMobileManipulatorTrainEnvCfg(ManagerBasedEnvCfg):
    """정책 학습용 환경 설정.

    대규모 병렬 학습을 위한 최적화된 설정입니다.
    """

    # 간단한 씬 (센서 없이, 빠른 시뮬레이션)
    scene: USTSimpleSceneCfg = USTSimpleSceneCfg(num_envs=4096, env_spacing=3.0)

    # 간단한 액션/관측
    actions: USTSimpleActionsCfg = USTSimpleActionsCfg()
    observations: USTSimpleObservationsCfg = USTSimpleObservationsCfg()

    # 이벤트 설정
    events: USTEventsCfg = USTEventsCfg()

    # 종료 조건
    terminations: USTTerminationsCfg = USTTerminationsCfg()

    def __post_init__(self):
        """학습 환경 초기화."""
        # 빠른 시뮬레이션 타이밍
        self.decimation = 2  # 100Hz 제어
        self.sim.dt = 0.005  # 200Hz 물리
        self.sim.render_interval = self.decimation

        # GPU 물리 (대규모 병렬)
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 2 * 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 32 * 1024

        # 렌더링 비활성화 (학습 시)
        self.sim.enable_scene_query_support = False

        # 에피소드 길이
        self.episode_length_s = 10.0  # 10초


@configclass
class USTMobileManipulatorDataCollectEnvCfg(ManagerBasedEnvCfg):
    """데이터 수집용 환경 설정.

    모방 학습을 위한 데모 데이터 수집에 최적화됩니다.
    """

    # 센서 포함 씬
    scene: USTSceneWithSensorsCfg = USTSceneWithSensorsCfg(num_envs=1, env_spacing=5.0)

    # 전체 액션/관측
    actions: USTActionsCfg = USTActionsCfg()
    observations: USTObservationsCfg = USTObservationsCfg()

    # 이벤트 설정
    events: USTEventsCfg = USTEventsCfg()

    def __post_init__(self):
        """데이터 수집 환경 초기화."""
        # 표준 타이밍
        self.decimation = 4  # 50Hz 제어
        self.sim.dt = 0.005  # 200Hz 물리
        self.sim.render_interval = self.decimation

        # GPU 물리
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024

        # 렌더링 활성화 (VR 피드백용)
        self.sim.enable_scene_query_support = True

        # 에피소드 길이
        self.episode_length_s = 120.0  # 2분
