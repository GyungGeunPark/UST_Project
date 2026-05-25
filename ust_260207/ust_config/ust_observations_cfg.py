# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST Mobile Manipulator Observation Configuration (Dual Arm)."""

from __future__ import annotations

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
import isaaclab.envs.mdp as mdp
from isaaclab.utils import configclass


@configclass
class USTObservationsCfg:
    """UST 환경 관측 설정 (듀얼 암).

    세 가지 관측 그룹을 정의합니다:
    1. policy: 정책 학습용 저차원 상태 관측
    2. vision: 카메라 이미지 관측 (RGB-D)
    3. critic: 크리틱 네트워크용 전체 상태 (옵션)
    """

    @configclass
    class PolicyCfg(ObsGroup):
        """정책 학습용 관측 (듀얼 암).

        저차원 상태 정보를 제공합니다:
        - 오른쪽/왼쪽 암 관절 위치/속도
        - 오른쪽/왼쪽 그리퍼 상태
        - 오른쪽/왼쪽 End-effector 위치/방향
        - 대상 객체 위치
        """

        # 오른쪽 암 관절 위치 (4D)
        right_arm_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_joint1", "right_joint2", "right_joint3", "right_joint4"])},
        )

        # 오른쪽 암 관절 속도 (4D)
        right_arm_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_joint1", "right_joint2", "right_joint3", "right_joint4"])},
        )

        # 오른쪽 그리퍼 위치 (1D)
        right_gripper_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_left_joint"])},
        )

        # 오른쪽 End-Effector 포즈 (7D: 위치 3D + 쿼터니언 4D)
        right_ee_pose = ObsTerm(
            func=mdp.body_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["right_link5"])},
        )

        # 왼쪽 암 관절 위치 (4D)
        left_arm_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_joint1", "left_joint2", "left_joint3", "left_joint4"])},
        )

        # 왼쪽 암 관절 속도 (4D)
        left_arm_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_joint1", "left_joint2", "left_joint3", "left_joint4"])},
        )

        # 왼쪽 그리퍼 위치 (1D)
        left_gripper_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_gripper_left_joint"])},
        )

        # 왼쪽 End-Effector 포즈 (7D: 위치 3D + 쿼터니언 4D)
        left_ee_pose = ObsTerm(
            func=mdp.body_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["left_link5"])},
        )

        # 대상 객체 위치 (3D, 월드 좌표)
        object_pos = ObsTerm(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("target_object")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class VisionCfg(ObsGroup):
        """비전 관측 (카메라 이미지).

        RealSense D455에서 RGB 및 깊이 이미지를 제공합니다.
        """

        # RGB 이미지 (1280 x 720 x 3)
        rgb_image = ObsTerm(
            func=mdp.image,
            params={
                "sensor_cfg": SceneEntityCfg("camera_d455"),
                "data_type": "rgb",
            },
        )

        # 깊이 이미지 (1280 x 720 x 1)
        depth_image = ObsTerm(
            func=mdp.image,
            params={
                "sensor_cfg": SceneEntityCfg("camera_d455"),
                "data_type": "distance_to_camera",
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False  # 이미지는 연결하지 않음

    @configclass
    class CriticCfg(ObsGroup):
        """크리틱 네트워크용 전체 상태 관측.

        정책 관측에 추가로 로봇 전체 상태 정보를 포함합니다.
        """

        # 베이스 위치 (3D)
        base_pos = ObsTerm(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        # 베이스 방향 (4D, 쿼터니언)
        base_quat = ObsTerm(
            func=mdp.root_quat_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        # 베이스 선속도 (3D)
        base_lin_vel = ObsTerm(
            func=mdp.root_lin_vel_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        # 베이스 각속도 (3D)
        base_ang_vel = ObsTerm(
            func=mdp.root_ang_vel_w,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        # 모든 관절 위치 (16D: 4 wheels + 4 right arm + 2 right gripper + 4 left arm + 2 left gripper)
        all_joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        # 모든 관절 속도 (16D)
        all_joint_vel = ObsTerm(
            func=mdp.joint_vel,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # 관측 그룹 인스턴스
    policy: PolicyCfg = PolicyCfg()
    vision: VisionCfg = VisionCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class USTTeleopObservationsCfg:
    """VR 텔레오퍼레이션용 관측 설정 (듀얼 암).

    필요한 최소한의 상태만 관측합니다.
    """

    @configclass
    class PolicyCfg(ObsGroup):
        """텔레옵용 관측 (듀얼 암)."""

        # 오른쪽 암 관절 위치
        right_arm_joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_joint1", "right_joint2", "right_joint3", "right_joint4"])},
        )

        # 오른쪽 그리퍼 위치
        right_gripper_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_left_joint"])},
        )

        # 오른쪽 EE 포즈
        right_ee_pose = ObsTerm(
            func=mdp.body_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["right_link5"])},
        )

        # 왼쪽 암 관절 위치
        left_arm_joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_joint1", "left_joint2", "left_joint3", "left_joint4"])},
        )

        # 왼쪽 그리퍼 위치
        left_gripper_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_gripper_left_joint"])},
        )

        # 왼쪽 EE 포즈
        left_ee_pose = ObsTerm(
            func=mdp.body_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["left_link5"])},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class USTSimpleObservationsCfg:
    """간단한 관측 설정 (센서 없이, 듀얼 암).

    카메라/LiDAR 없이 로봇 상태만 관측합니다.
    """

    @configclass
    class PolicyCfg(ObsGroup):
        """간단한 정책 관측 (듀얼 암)."""

        # 오른쪽 암 관절 위치
        right_arm_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_joint1", "right_joint2", "right_joint3", "right_joint4"])},
        )

        # 오른쪽 그리퍼 위치
        right_gripper_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["right_gripper_left_joint"])},
        )

        # 오른쪽 EE 포즈
        right_ee_pose = ObsTerm(
            func=mdp.body_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["right_link5"])},
        )

        # 왼쪽 암 관절 위치
        left_arm_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_joint1", "left_joint2", "left_joint3", "left_joint4"])},
        )

        # 왼쪽 그리퍼 위치
        left_gripper_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["left_gripper_left_joint"])},
        )

        # 왼쪽 EE 포즈
        left_ee_pose = ObsTerm(
            func=mdp.body_pose_w,
            params={"asset_cfg": SceneEntityCfg("robot", body_names=["left_link5"])},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
