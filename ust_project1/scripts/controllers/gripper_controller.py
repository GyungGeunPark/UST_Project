"""
controllers/gripper_controller.py
OpenMANIPULATOR-X 그리퍼 컨트롤러

프리스매틱 조인트 기반 그리퍼를 제어합니다.

그리퍼 조인트:
    - gripper_left_joint: -0.01 (닫힘) ~ 0.02 (열림)
    - gripper_right_joint: mimic joint (자동 동기화)

Usage:
    from controllers import GripperController

    gripper = GripperController(robot_articulation)
    gripper.open()
    gripper.close()
    gripper.set_from_trigger(0.5)  # 50% 닫힘

Author: UST Robotics Project
Date: 2024
"""

import numpy as np
from typing import Optional, Union, List
from enum import Enum


class GripperState(Enum):
    """그리퍼 상태 열거형"""
    OPEN = "open"
    CLOSED = "closed"
    MOVING = "moving"
    UNKNOWN = "unknown"


class GripperController:
    """
    OpenMANIPULATOR-X 그리퍼 컨트롤러

    프리스매틱 조인트를 사용한 그리퍼 제어를 제공합니다.

    Attributes:
        robot: Isaac Sim Articulation 객체
        gripper_joint_name: 그리퍼 조인트 이름

    Constants:
        JOINT_MIN: 완전 닫힘 위치 (-0.01)
        JOINT_MAX: 완전 열림 위치 (0.02)

    Example:
        >>> gripper = GripperController(robot_articulation)
        >>> gripper.open()   # 그리퍼 열기
        >>> gripper.close()  # 그리퍼 닫기
        >>> gripper.set_from_trigger(0.5)  # 트리거 값으로 제어
    """

    # 그리퍼 조인트 한계
    JOINT_MIN = -0.01  # 완전 닫힘
    JOINT_MAX = 0.02   # 완전 열림
    JOINT_RANGE = JOINT_MAX - JOINT_MIN

    # 상태 판정 임계값
    OPEN_THRESHOLD = 0.015    # 이 값 이상이면 OPEN
    CLOSED_THRESHOLD = -0.005  # 이 값 이하면 CLOSED

    # 기본 그리퍼 조인트 이름
    DEFAULT_JOINT_NAME = "gripper_left_joint"

    def __init__(
        self,
        robot_articulation=None,
        gripper_joint_name: str = None,
        joint_min: float = None,
        joint_max: float = None
    ):
        """
        GripperController 초기화

        Args:
            robot_articulation: Isaac Sim Articulation 객체
            gripper_joint_name: 그리퍼 조인트 이름 (기본: "gripper_left_joint")
            joint_min: 그리퍼 최소 위치 (닫힘)
            joint_max: 그리퍼 최대 위치 (열림)
        """
        self._robot = robot_articulation
        self._joint_name = gripper_joint_name or self.DEFAULT_JOINT_NAME

        # 조인트 한계 (커스터마이징 가능)
        if joint_min is not None:
            self.JOINT_MIN = joint_min
        if joint_max is not None:
            self.JOINT_MAX = joint_max
            self.JOINT_RANGE = self.JOINT_MAX - self.JOINT_MIN

        # 내부 상태
        self._target_position = self.JOINT_MAX  # 기본: 열림
        self._current_state = GripperState.UNKNOWN
        self._joint_index: Optional[int] = None

        # 조인트 인덱스 찾기
        if self._robot is not None:
            self._find_joint_index()

    def set_robot(self, robot_articulation):
        """
        로봇 Articulation 설정

        Args:
            robot_articulation: Isaac Sim Articulation 객체
        """
        self._robot = robot_articulation
        self._find_joint_index()

    def _find_joint_index(self) -> bool:
        """그리퍼 조인트 인덱스 찾기"""
        if self._robot is None:
            return False

        try:
            joint_names = self._robot.dof_names
            if joint_names is not None and self._joint_name in joint_names:
                self._joint_index = list(joint_names).index(self._joint_name)
                return True
        except Exception:
            pass

        self._joint_index = None
        return False

    def open(self) -> bool:
        """
        그리퍼 열기

        Returns:
            성공 여부
        """
        return self.set_position(self.JOINT_MAX)

    def close(self) -> bool:
        """
        그리퍼 닫기

        Returns:
            성공 여부
        """
        return self.set_position(self.JOINT_MIN)

    def set_position(self, position: float) -> bool:
        """
        그리퍼 위치 직접 설정

        Args:
            position: 조인트 위치 (JOINT_MIN ~ JOINT_MAX)

        Returns:
            성공 여부
        """
        # 클램핑
        position = np.clip(position, self.JOINT_MIN, self.JOINT_MAX)
        self._target_position = position

        if self._robot is None:
            print("[GripperController] Robot not set")
            return False

        if self._joint_index is None:
            if not self._find_joint_index():
                print(f"[GripperController] Joint '{self._joint_name}' not found")
                return False

        try:
            # 현재 조인트 위치 가져오기
            current_positions = self._robot.get_joint_positions()
            if current_positions is None:
                return False

            # 그리퍼 조인트만 수정
            new_positions = current_positions.copy()
            new_positions[self._joint_index] = position

            # ArticulationAction 적용
            from omni.isaac.core.utils.types import ArticulationAction
            action = ArticulationAction(joint_positions=new_positions)
            self._robot.apply_action(action)

            self._update_state()
            return True

        except Exception as e:
            print(f"[GripperController] Set position failed: {e}")
            return False

    def set_from_trigger(self, trigger_value: float) -> bool:
        """
        VR 컨트롤러 트리거 값으로 그리퍼 제어

        트리거 값을 그리퍼 위치로 매핑합니다:
            0.0 (미누름) → JOINT_MAX (열림)
            1.0 (완전 누름) → JOINT_MIN (닫힘)

        Args:
            trigger_value: 트리거 값 (0.0 ~ 1.0)

        Returns:
            성공 여부
        """
        # 트리거 값 클램핑
        trigger_value = np.clip(float(trigger_value), 0.0, 1.0)

        # 트리거 → 그리퍼 위치 매핑
        position = self.JOINT_MAX - (trigger_value * self.JOINT_RANGE)

        return self.set_position(position)

    def set_normalized(self, normalized_value: float) -> bool:
        """
        정규화된 값 (0~1)으로 그리퍼 제어

        Args:
            normalized_value: 0.0 (닫힘) ~ 1.0 (열림)

        Returns:
            성공 여부
        """
        normalized_value = np.clip(float(normalized_value), 0.0, 1.0)
        position = self.JOINT_MIN + (normalized_value * self.JOINT_RANGE)
        return self.set_position(position)

    def _update_state(self):
        """현재 상태 업데이트"""
        if self._target_position >= self.OPEN_THRESHOLD:
            self._current_state = GripperState.OPEN
        elif self._target_position <= self.CLOSED_THRESHOLD:
            self._current_state = GripperState.CLOSED
        else:
            self._current_state = GripperState.MOVING

    def get_state(self) -> GripperState:
        """
        현재 그리퍼 상태 반환

        Returns:
            GripperState 열거형 값
        """
        return self._current_state

    def get_position(self) -> Optional[float]:
        """
        현재 그리퍼 조인트 위치 반환

        Returns:
            조인트 위치 (실패 시 None)
        """
        if self._robot is None or self._joint_index is None:
            return None

        try:
            positions = self._robot.get_joint_positions()
            if positions is not None:
                return float(positions[self._joint_index])
        except Exception:
            pass

        return None

    def get_normalized_position(self) -> Optional[float]:
        """
        정규화된 그리퍼 위치 반환 (0~1)

        Returns:
            0.0 (닫힘) ~ 1.0 (열림), 실패 시 None
        """
        pos = self.get_position()
        if pos is None:
            return None

        return (pos - self.JOINT_MIN) / self.JOINT_RANGE

    def is_open(self) -> bool:
        """그리퍼가 열려 있는지 확인"""
        pos = self.get_position()
        return pos is not None and pos >= self.OPEN_THRESHOLD

    def is_closed(self) -> bool:
        """그리퍼가 닫혀 있는지 확인"""
        pos = self.get_position()
        return pos is not None and pos <= self.CLOSED_THRESHOLD

    def toggle(self) -> bool:
        """
        그리퍼 상태 토글 (열림 ↔ 닫힘)

        Returns:
            성공 여부
        """
        if self.is_open():
            return self.close()
        else:
            return self.open()

    def get_info(self) -> dict:
        """그리퍼 정보 반환"""
        return {
            "joint_name": self._joint_name,
            "joint_index": self._joint_index,
            "joint_min": self.JOINT_MIN,
            "joint_max": self.JOINT_MAX,
            "current_position": self.get_position(),
            "normalized_position": self.get_normalized_position(),
            "target_position": self._target_position,
            "state": self._current_state.value,
        }

    def __repr__(self) -> str:
        pos = self.get_position()
        pos_str = f"{pos:.4f}" if pos is not None else "N/A"
        return f"GripperController(joint='{self._joint_name}', position={pos_str}, state={self._current_state.value})"


# =============================================================================
# Standalone Gripper Control (without Articulation)
# =============================================================================

class StandaloneGripperController:
    """
    독립형 그리퍼 컨트롤러

    Articulation 없이 조인트 위치를 직접 계산하여 반환합니다.
    OmniGraph Script Node에서 사용하기 적합합니다.
    """

    def __init__(
        self,
        joint_min: float = -0.01,
        joint_max: float = 0.02
    ):
        self.joint_min = joint_min
        self.joint_max = joint_max
        self.range = joint_max - joint_min

    def trigger_to_position(self, trigger_value: float) -> float:
        """트리거 값 → 그리퍼 위치"""
        trigger_value = np.clip(float(trigger_value), 0.0, 1.0)
        return self.joint_max - (trigger_value * self.range)

    def normalized_to_position(self, normalized: float) -> float:
        """정규화 값 (0=닫힘, 1=열림) → 그리퍼 위치"""
        normalized = np.clip(float(normalized), 0.0, 1.0)
        return self.joint_min + (normalized * self.range)

    def position_to_normalized(self, position: float) -> float:
        """그리퍼 위치 → 정규화 값"""
        return (position - self.joint_min) / self.range

    @property
    def open_position(self) -> float:
        return self.joint_max

    @property
    def closed_position(self) -> float:
        return self.joint_min
