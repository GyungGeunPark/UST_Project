# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""Lula IK 솔버 설정 및 래퍼.

all_dev.md 스펙에 따른 Lula IK 통합.
ust_project1/scripts/ik_controller.py 기반.

사용법:
    from ust_config.lula_ik_cfg import LulaIKWrapper, LulaIKConfig

    ik = LulaIKWrapper(robot_articulation)
    action, success = ik.compute_ik(target_pos, target_rot)
"""

from __future__ import annotations

import os
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

# 설정 파일 경로
CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROBOT_DESC = os.path.join(CONFIG_DIR, "open_x1_des.yaml")
DEFAULT_URDF = os.path.join(CONFIG_DIR, "open_manipulator_x.urdf")


@dataclass
class LulaIKConfig:
    """Lula IK 설정."""
    robot_description_path: str = DEFAULT_ROBOT_DESC
    urdf_path: str = DEFAULT_URDF
    end_effector_frame: str = "end_effector_link"
    max_iterations: int = 200
    # 작업 공간 제한 (미터)
    workspace_min_radius: float = 0.08
    workspace_max_radius: float = 0.38
    workspace_min_height: float = -0.05
    workspace_max_height: float = 0.40


class LulaIKWrapper:
    """OpenMANIPULATOR-X Lula IK 래퍼 클래스.

    all_dev.md 스펙의 OpenManipulatorIK 클래스 구현.
    Isaac Sim의 LulaKinematicsSolver를 래핑하여
    ManagerBasedEnv에서 사용 가능한 인터페이스를 제공합니다.

    참고: 이 클래스는 Isaac Sim 런타임에서만 사용 가능합니다.
    (isaacsim.robot_motion.motion_generation 모듈 필요)
    """

    def __init__(self, robot_articulation, cfg: LulaIKConfig = None):
        """초기화.

        Args:
            robot_articulation: Isaac Sim Articulation 또는 Isaac Lab ArticulationData
            cfg: Lula IK 설정
        """
        self.cfg = cfg or LulaIKConfig()

        # Isaac Sim 5.1.0 API 경로 (all_dev.md 스펙)
        try:
            from isaacsim.robot_motion.motion_generation import (
                LulaKinematicsSolver,
                ArticulationKinematicsSolver,
            )
        except ImportError:
            # 구버전 fallback
            from omni.isaac.motion_generation import (
                LulaKinematicsSolver,
                ArticulationKinematicsSolver,
            )

        # Lula 솔버 초기화
        self.kinematics_solver = LulaKinematicsSolver(
            robot_description_path=self.cfg.robot_description_path,
            urdf_path=self.cfg.urdf_path,
        )

        # 프레임 확인
        self.available_frames = self.kinematics_solver.get_all_frame_names()
        print(f"[LulaIK] Available frames: {self.available_frames}")

        # EE 프레임 검증
        ee_frame = self.cfg.end_effector_frame
        if ee_frame not in self.available_frames:
            ee_frame = self.available_frames[-1]
            print(f"[LulaIK] Fallback EE frame: {ee_frame}")

        # ArticulationKinematicsSolver 래핑
        self.articulation_kinematics = ArticulationKinematicsSolver(
            robot_articulation, self.kinematics_solver, ee_frame
        )

    def compute_ik(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        robot_articulation=None,
    ) -> Tuple[object, bool]:
        """IK 계산 및 적용.

        Args:
            target_position: [x, y, z] 미터 단위
            target_orientation: [w, x, y, z] 쿼터니언 (None이면 위치만)
            robot_articulation: 모바일 베이스 포즈 업데이트용

        Returns:
            (action, success) 튜플
        """
        # 모바일 베이스 포즈 업데이트
        if robot_articulation is not None:
            base_pos, base_rot = robot_articulation.get_world_pose()
            self.kinematics_solver.set_robot_base_pose(base_pos, base_rot)

        # IK 계산 (4축: orientation=None이면 위치만)
        action, success = self.articulation_kinematics.compute_inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation,
        )

        if success and action is not None and robot_articulation is not None:
            robot_articulation.apply_action(action)

        return action, success

    def compute_fk(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """순기구학 계산.

        Args:
            joint_positions: 관절 위치 배열

        Returns:
            (position, rotation_matrix) 튜플
        """
        position, rotation_matrix = self.kinematics_solver.compute_forward_kinematics(
            frame_name=self.cfg.end_effector_frame,
            joint_positions=joint_positions,
        )
        return position, rotation_matrix

    def is_in_workspace(self, target_position: np.ndarray) -> bool:
        """타겟 위치가 작업 공간 내에 있는지 확인.

        Args:
            target_position: [x, y, z] 미터 단위

        Returns:
            작업 공간 내 여부
        """
        xy_dist = np.sqrt(target_position[0] ** 2 + target_position[1] ** 2)
        z = target_position[2]

        return (
            self.cfg.workspace_min_radius <= xy_dist <= self.cfg.workspace_max_radius
            and self.cfg.workspace_min_height <= z <= self.cfg.workspace_max_height
        )
