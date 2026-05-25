# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST Mobile Manipulator (TurtleBot3 Waffle Pi + Dual OpenMANIPULATOR-X) Configuration."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg


# USD 파일 경로 (Robot 서브트리만 참조하는 래퍼 USD)
# 원본 ust_project1.usd는 defaultPrim이 없어 Isaac Lab에서 로드 실패하므로,
# /World/Robot 서브트리만 참조하는 래퍼 파일을 사용합니다.
UST_USD_PATH = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_robot.usd"

# ===== 암 액추에이터 파라미터 프리셋 =====
# all_dev.md 원본 스펙 파라미터
ALL_DEV_ARM_PARAMS = {
    "stiffness": 80.0,
    "damping": 4.0,
    "velocity_limit": 2.0,
}

# TurtleBot3 + OpenMANIPULATOR-X 튜닝 파라미터 (현재 기본값)
TURTLEBOT3_ARM_PARAMS = {
    "stiffness": 100.0,
    "damping": 10.0,
    "velocity_limit": 4.8,
}

# 사용할 파라미터 세트 선택 (변경 시 ALL_DEV_ARM_PARAMS로 전환 가능)
ACTIVE_ARM_PARAMS = TURTLEBOT3_ARM_PARAMS

UST_MOBILE_MANIPULATOR_CFG = ArticulationCfg(
    # USD 파일 로딩 설정
    spawn=sim_utils.UsdFileCfg(
        usd_path=UST_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=64,
            solver_velocity_iteration_count=4,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        activate_contact_sensors=False,
    ),

    # 초기 상태 설정
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.033),  # 바퀴 반지름(0.033m) 높이 → 바퀴가 지면에 닿음
        rot=(1.0, 0.0, 0.0, 0.0),  # Identity rotation (wxyz)
        joint_pos={
            # 바퀴 관절 (4개)
            "wheel_left_front_joint": 0.0,
            "wheel_right_front_joint": 0.0,
            "wheel_left_rear_joint": 0.0,
            "wheel_right_rear_joint": 0.0,
            # 오른쪽 OpenMANIPULATOR-X 암 관절
            "right_joint1": 0.0,       # Base rotation (Z-axis)
            "right_joint2": -0.5,      # Shoulder (Y-axis)
            "right_joint3": 0.3,       # Elbow (Y-axis)
            "right_joint4": 0.2,       # Wrist (Y-axis)
            # 오른쪽 그리퍼 관절 (localRot0 180° X회전으로 축 반전 → 같은 부호 = 같은 동작)
            "right_gripper_left_joint": 0.015,    # 열린 상태
            "right_gripper_right_joint": 0.015,   # 열린 상태 (같은 부호)
            # 왼쪽 OpenMANIPULATOR-X 암 관절
            "left_joint1": 0.0,
            "left_joint2": -0.5,
            "left_joint3": 0.3,
            "left_joint4": 0.2,
            # 왼쪽 그리퍼 관절 (localRot0 180° X회전으로 축 반전 → 같은 부호 = 같은 동작)
            "left_gripper_left_joint": 0.015,
            "left_gripper_right_joint": 0.015,    # 열린 상태 (같은 부호)
        },
        joint_vel={
            ".*": 0.0,  # 모든 관절 속도 0
        },
    ),

    # 액추에이터 그룹 정의
    actuators={
        # 바퀴 관절 (4개) - 속도 제어
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["wheel_.*_joint"],
            effort_limit=20.0,      # 로봇 전체 중량(~2.5kg) 대비 충분한 토크
            velocity_limit=30.0,    # rad/s
            stiffness=0.0,          # 속도 제어: stiffness=0
            damping=50.0,           # 속도 추종을 위한 높은 댐핑
        ),
        # 오른쪽 매니퓰레이터 암 관절 - 위치 제어
        "arm_right": ImplicitActuatorCfg(
            joint_names_expr=["right_joint[1-4]"],
            effort_limit=10.0,
            velocity_limit=ACTIVE_ARM_PARAMS["velocity_limit"],
            stiffness=ACTIVE_ARM_PARAMS["stiffness"],
            damping=ACTIVE_ARM_PARAMS["damping"],
        ),
        # 오른쪽 그리퍼 관절 - 위치 제어
        "gripper_right": ImplicitActuatorCfg(
            joint_names_expr=["right_gripper_.*_joint"],
            effort_limit=5.0,
            velocity_limit=0.5,
            stiffness=200.0,
            damping=10.0,
        ),
        # 왼쪽 매니퓰레이터 암 관절 - 위치 제어
        "arm_left": ImplicitActuatorCfg(
            joint_names_expr=["left_joint[1-4]"],
            effort_limit=10.0,
            velocity_limit=ACTIVE_ARM_PARAMS["velocity_limit"],
            stiffness=ACTIVE_ARM_PARAMS["stiffness"],
            damping=ACTIVE_ARM_PARAMS["damping"],
        ),
        # 왼쪽 그리퍼 관절 - 위치 제어
        "gripper_left": ImplicitActuatorCfg(
            joint_names_expr=["left_gripper_.*_joint"],
            effort_limit=5.0,
            velocity_limit=0.5,
            stiffness=200.0,
            damping=10.0,
        ),
    },

    # 관절 제한 여유 계수
    soft_joint_pos_limit_factor=0.95,
)


# 관절 인덱스 매핑 (참조용)
# 순서: 바퀴 4개 → 오른쪽 암 4개 → 오른쪽 그리퍼 2개 → 왼쪽 암 4개 → 왼쪽 그리퍼 2개 = 총 16개
JOINT_NAMES = {
    "wheel_left_front_joint": 0,
    "wheel_right_front_joint": 1,
    "wheel_left_rear_joint": 2,
    "wheel_right_rear_joint": 3,
    "right_joint1": 4,
    "right_joint2": 5,
    "right_joint3": 6,
    "right_joint4": 7,
    "right_gripper_left_joint": 8,
    "right_gripper_right_joint": 9,
    "left_joint1": 10,
    "left_joint2": 11,
    "left_joint3": 12,
    "left_joint4": 13,
    "left_gripper_left_joint": 14,
    "left_gripper_right_joint": 15,
}

# 관절 그룹
WHEEL_JOINTS = [
    "wheel_left_front_joint", "wheel_right_front_joint",
    "wheel_left_rear_joint", "wheel_right_rear_joint",
]
RIGHT_ARM_JOINTS = ["right_joint1", "right_joint2", "right_joint3", "right_joint4"]
RIGHT_GRIPPER_JOINTS = ["right_gripper_left_joint", "right_gripper_right_joint"]
LEFT_ARM_JOINTS = ["left_joint1", "left_joint2", "left_joint3", "left_joint4"]
LEFT_GRIPPER_JOINTS = ["left_gripper_left_joint", "left_gripper_right_joint"]

# 로봇 기구학 파라미터
ROBOT_PARAMS = {
    # TurtleBot3 Waffle Pi
    "wheel_radius": 0.033,          # m
    "wheel_base": 0.287,            # m (좌우 바퀴 간격)
    "max_linear_velocity": 0.26,    # m/s
    "max_angular_velocity": 1.82,   # rad/s

    # OpenMANIPULATOR-X (양쪽 동일)
    "arm_reach": 0.38,              # m (최대 도달 거리)
    "right_ee_offset": (0.126, 0.0, 0.0),  # m (right_link5에서 EE까지 오프셋)
    "left_ee_offset": (0.126, 0.0, 0.0),   # m (left_link5에서 EE까지 오프셋)

    # 관절 제한 (rad) - 양쪽 동일
    "joint_limits": {
        "right_joint1": (-3.14, 3.14),
        "right_joint2": (-1.5, 1.5),
        "right_joint3": (-1.5, 1.4),
        "right_joint4": (-1.7, 1.97),
        "right_gripper_left_joint": (-0.01, 0.019),
        "left_joint1": (-3.14, 3.14),
        "left_joint2": (-1.5, 1.5),
        "left_joint3": (-1.5, 1.4),
        "left_joint4": (-1.7, 1.97),
        "left_gripper_left_joint": (-0.01, 0.019),
    },
}
