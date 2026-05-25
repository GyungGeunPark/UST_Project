# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST Mobile Manipulator Action Configuration (Dual Arm)."""

from __future__ import annotations

from isaaclab.managers import ActionTermCfg as ActionTerm
from isaaclab.envs.mdp.actions import actions_cfg as mdp
from isaaclab.utils import configclass

# ===== IK 솔버 선택 =====
# - "dls": Damped Least Squares (기본, 실시간 텔레오퍼레이션에 적합)
# - "lula": Lula IK (all_dev.md 스펙, RMP 기반 모션 플래닝에 적합)
# Lula 사용 시 config/lula_ik_cfg.py의 LulaIKWrapper를 별도 모듈로 활용
IK_METHOD = "dls"


@configclass
class USTActionsCfg:
    """UST 환경 액션 설정 (듀얼 암).

    다섯 가지 액션 그룹:
    1. base_action: 4바퀴 속도 제어 (모바일 베이스)
    2. right_arm_action: 오른쪽 매니퓰레이터 암 Differential IK
    3. right_gripper_action: 오른쪽 바이너리 그리퍼 제어
    4. left_arm_action: 왼쪽 매니퓰레이터 암 Differential IK
    5. left_gripper_action: 왼쪽 바이너리 그리퍼 제어
    """

    # 모바일 베이스 - 4바퀴 속도 제어
    # 입력: 4D (각 바퀴 속도)
    # scale=1.0: 양의 회전 = 전진 방향 (테스트 확인)
    base_action: ActionTerm = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["wheel_.*_joint"],
        scale=1.0,
    )

    # 오른쪽 매니퓰레이터 암 - Differential Inverse Kinematics
    # 입력: SE3 delta pose (position 3D + rotation 3D = 6D)
    right_arm_action: ActionTerm = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["right_joint1", "right_joint2", "right_joint3", "right_joint4"],
        body_name="right_link5",
        controller=mdp.DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=True,
            ik_method="dls",
            ik_params={"lambda_val": 0.05},
        ),
        scale=1.0,
        body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(
            pos=(0.126, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    # 오른쪽 그리퍼 - 바이너리 위치 제어 (양쪽 핑거)
    # gripper_right_joint의 localRot0=(0,1,0,0) (180° X회전)이 축 방향을 반전하므로
    # 양쪽 핑거 모두 같은 부호 = 같은 물리적 동작 (+값=open, -값=close)
    right_gripper_action: ActionTerm = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["right_gripper_left_joint", "right_gripper_right_joint"],
        open_command_expr={"right_gripper_left_joint": 0.019, "right_gripper_right_joint": 0.019},
        close_command_expr={"right_gripper_left_joint": -0.01, "right_gripper_right_joint": -0.01},
    )

    # 왼쪽 매니퓰레이터 암 - Differential Inverse Kinematics
    left_arm_action: ActionTerm = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["left_joint1", "left_joint2", "left_joint3", "left_joint4"],
        body_name="left_link5",
        controller=mdp.DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=True,
            ik_method="dls",
            ik_params={"lambda_val": 0.05},
        ),
        scale=1.0,
        body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(
            pos=(0.126, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    # 왼쪽 그리퍼 - 바이너리 위치 제어 (양쪽 핑거)
    # gripper_right_joint의 localRot0=(0,1,0,0) (180° X회전)이 축 방향을 반전하므로
    # 양쪽 핑거 모두 같은 부호 = 같은 물리적 동작 (+값=open, -값=close)
    left_gripper_action: ActionTerm = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["left_gripper_left_joint", "left_gripper_right_joint"],
        open_command_expr={"left_gripper_left_joint": 0.019, "left_gripper_right_joint": 0.019},
        close_command_expr={"left_gripper_left_joint": -0.01, "left_gripper_right_joint": -0.01},
    )


@configclass
class USTTeleopActionsCfg:
    """VR 텔레오퍼레이션용 액션 설정 (듀얼 암).

    VR 컨트롤러에서 직접 받은 SE3 pose를 사용합니다.
    - 오른쪽 컨트롤러 → 오른쪽 암
    - 왼쪽 컨트롤러 → 왼쪽 암
    """

    # 모바일 베이스 - 4바퀴 속도 제어
    # scale=1.0: 양의 회전 = 전진 방향 (테스트 확인)
    base_action: ActionTerm = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["wheel_.*_joint"],
        scale=1.0,
    )

    # 오른쪽 암 - 상대 IK (VR 오른쪽 컨트롤러)
    right_arm_action: ActionTerm = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["right_joint1", "right_joint2", "right_joint3", "right_joint4"],
        body_name="right_link5",
        controller=mdp.DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=True,
            ik_method="dls",
            ik_params={"lambda_val": 0.05},
        ),
        scale=1.0,
        body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(
            pos=(0.126, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    # 오른쪽 그리퍼 - 연속 제어 (오른쪽 트리거)
    # NOTE: VR에서 양쪽 핑거를 제어하려면 action_dim이 2로 변경되므로
    # VR 컨트롤러 코드도 함께 수정 필요. 현재는 left_joint만 제어.
    right_gripper_action: ActionTerm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["right_gripper_left_joint"],
        scale=1.0,
    )

    # 왼쪽 암 - 상대 IK (VR 왼쪽 컨트롤러)
    left_arm_action: ActionTerm = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["left_joint1", "left_joint2", "left_joint3", "left_joint4"],
        body_name="left_link5",
        controller=mdp.DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=True,
            ik_method="dls",
            ik_params={"lambda_val": 0.05},
        ),
        scale=1.0,
        body_offset=mdp.DifferentialInverseKinematicsActionCfg.OffsetCfg(
            pos=(0.126, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    # 왼쪽 그리퍼 - 연속 제어 (왼쪽 트리거)
    # NOTE: VR에서 양쪽 핑거를 제어하려면 action_dim이 2로 변경되므로
    # VR 컨트롤러 코드도 함께 수정 필요. 현재는 left_joint만 제어.
    left_gripper_action: ActionTerm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["left_gripper_left_joint"],
        scale=1.0,
    )


@configclass
class USTSimpleActionsCfg:
    """간단한 액션 설정 (관절 위치 직접 제어, 듀얼 암).

    IK 없이 관절 위치를 직접 제어합니다.
    빠른 테스트와 디버깅에 유용합니다.
    """

    # 모바일 베이스 - 4바퀴 속도 제어
    # scale=1.0: 양의 회전 = 전진 방향 (테스트 확인)
    base_action: ActionTerm = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["wheel_.*_joint"],
        scale=1.0,
    )

    # 오른쪽 암 관절 위치 직접 제어
    right_arm_action: ActionTerm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["right_joint1", "right_joint2", "right_joint3", "right_joint4"],
        scale=1.0,
    )

    # 오른쪽 그리퍼 위치 제어
    right_gripper_action: ActionTerm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["right_gripper_left_joint"],
        scale=1.0,
    )

    # 왼쪽 암 관절 위치 직접 제어
    left_arm_action: ActionTerm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["left_joint1", "left_joint2", "left_joint3", "left_joint4"],
        scale=1.0,
    )

    # 왼쪽 그리퍼 위치 제어
    left_gripper_action: ActionTerm = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["left_gripper_left_joint"],
        scale=1.0,
    )
