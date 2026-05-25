"""
controllers/coordinate_transform.py
Quest2ROS와 Isaac Sim 간 좌표계 변환 유틸리티

좌표계 차이:
    Quest2ROS (Unity 좌표계):
        - Y-up, Left-handed
        - Position: (x, y, z) where y is up

    Isaac Sim (USD 좌표계):
        - Z-up, Right-handed
        - Position: (x, y, z) where z is up

변환 공식:
    Isaac_X = Quest_X
    Isaac_Y = Quest_Z
    Isaac_Z = Quest_Y

Usage:
    from controllers import CoordinateTransform

    # 위치 변환
    isaac_pos = CoordinateTransform.position_quest_to_isaac([1, 2, 3])
    # Result: [1, 3, 2]

    # 작업 공간 검증
    validator = WorkspaceValidator()
    if validator.is_reachable(target_position):
        # Execute IK
        pass

Author: UST Robotics Project
Date: 2024
"""

import numpy as np
from typing import List, Tuple, Optional, Union


class CoordinateTransform:
    """
    좌표계 변환 클래스

    Quest2ROS와 Isaac Sim 간의 좌표 변환을 수행합니다.

    Class Methods:
        position_quest_to_isaac: Quest 위치 → Isaac 위치
        quaternion_quest_to_isaac: Quest 쿼터니언 → Isaac 쿼터니언
        twist_quest_to_isaac: Quest Twist → Isaac Twist
        position_isaac_to_quest: Isaac 위치 → Quest 위치
    """

    # 변환 행렬: Quest → Isaac
    # Isaac(X, Y, Z) = Quest(X, Z, Y)
    QUEST_TO_ISAAC_MATRIX = np.array([
        [1, 0, 0],  # Isaac X = Quest X
        [0, 0, 1],  # Isaac Y = Quest Z
        [0, 1, 0]   # Isaac Z = Quest Y
    ], dtype=np.float64)

    # 역변환 행렬: Isaac → Quest
    ISAAC_TO_QUEST_MATRIX = np.linalg.inv(QUEST_TO_ISAAC_MATRIX)

    @classmethod
    def position_quest_to_isaac(
        cls,
        quest_position: Union[List[float], np.ndarray],
        scale: float = 1.0,
        offset: Union[List[float], np.ndarray] = None
    ) -> np.ndarray:
        """
        Quest 위치를 Isaac 좌표계로 변환

        Args:
            quest_position: Quest2ROS 좌표 [x, y, z]
            scale: 스케일 팩터 (기본: 1.0)
            offset: Isaac 좌표계에서의 오프셋 [x, y, z]

        Returns:
            Isaac Sim 좌표 [x, y, z]

        Example:
            >>> CoordinateTransform.position_quest_to_isaac([1, 2, 3])
            array([1., 3., 2.])
        """
        pos = np.array(quest_position[:3], dtype=np.float64)
        isaac_pos = cls.QUEST_TO_ISAAC_MATRIX @ pos * scale

        if offset is not None:
            isaac_pos += np.array(offset, dtype=np.float64)

        return isaac_pos

    @classmethod
    def position_isaac_to_quest(
        cls,
        isaac_position: Union[List[float], np.ndarray],
        scale: float = 1.0
    ) -> np.ndarray:
        """
        Isaac 위치를 Quest 좌표계로 변환

        Args:
            isaac_position: Isaac Sim 좌표 [x, y, z]
            scale: 스케일 팩터

        Returns:
            Quest 좌표 [x, y, z]
        """
        pos = np.array(isaac_position[:3], dtype=np.float64) / scale
        return cls.ISAAC_TO_QUEST_MATRIX @ pos

    @classmethod
    def quaternion_quest_to_isaac(
        cls,
        quest_quat: Union[List[float], np.ndarray]
    ) -> np.ndarray:
        """
        Quest 쿼터니언을 Isaac 좌표계로 변환

        Args:
            quest_quat: Quest2ROS 쿼터니언 [x, y, z, w]

        Returns:
            Isaac Sim 쿼터니언 [x, y, z, w]

        Note:
            축 교환에 따라 쿼터니언 요소도 교환됩니다.
            Quest [x, y, z, w] → Isaac [x, z, y, w]
        """
        q = np.array(quest_quat[:4], dtype=np.float64)
        # 축 교환: Y ↔ Z
        return np.array([q[0], q[2], q[1], q[3]])

    @classmethod
    def twist_quest_to_isaac(
        cls,
        linear: Union[List[float], np.ndarray],
        angular: Union[List[float], np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Quest Twist (선속도, 각속도)를 Isaac으로 변환

        Args:
            linear: 선속도 [x, y, z]
            angular: 각속도 [x, y, z]

        Returns:
            (isaac_linear, isaac_angular) 튜플
        """
        lin = np.array(linear[:3], dtype=np.float64)
        ang = np.array(angular[:3], dtype=np.float64)

        isaac_linear = cls.QUEST_TO_ISAAC_MATRIX @ lin
        isaac_angular = cls.QUEST_TO_ISAAC_MATRIX @ ang

        return isaac_linear, isaac_angular

    @classmethod
    def euler_quest_to_isaac(
        cls,
        quest_euler: Union[List[float], np.ndarray]
    ) -> np.ndarray:
        """
        Quest 오일러 각도를 Isaac으로 변환

        Args:
            quest_euler: Quest 오일러 각도 [rx, ry, rz] (라디안)

        Returns:
            Isaac 오일러 각도 [rx, ry, rz]
        """
        euler = np.array(quest_euler[:3], dtype=np.float64)
        # Y ↔ Z 축 교환
        return np.array([euler[0], euler[2], euler[1]])


class WorkspaceValidator:
    """
    작업 공간 유효성 검사 클래스

    OpenMANIPULATOR-X의 도달 가능 영역을 검증합니다.

    기본 작업 공간 (로봇 베이스 기준):
        - 최소 도달 거리: 0.08 m
        - 최대 도달 거리: 0.38 m
        - 최소 높이: -0.05 m
        - 최대 높이: 0.40 m

    Example:
        >>> validator = WorkspaceValidator()
        >>> validator.is_reachable([0.2, 0.0, 0.15])
        True
        >>> validator.is_reachable([0.5, 0.0, 0.0])  # 너무 멂
        False
    """

    # 기본 작업 공간 한계
    DEFAULT_MIN_RADIUS = 0.08   # 최소 수평 도달 거리 (m)
    DEFAULT_MAX_RADIUS = 0.38   # 최대 수평 도달 거리 (m)
    DEFAULT_MIN_HEIGHT = -0.05  # 최소 높이 (m)
    DEFAULT_MAX_HEIGHT = 0.40   # 최대 높이 (m)

    def __init__(
        self,
        min_radius: float = None,
        max_radius: float = None,
        min_height: float = None,
        max_height: float = None,
        base_position: Union[List[float], np.ndarray] = None
    ):
        """
        WorkspaceValidator 초기화

        Args:
            min_radius: 최소 수평 도달 거리 (m)
            max_radius: 최대 수평 도달 거리 (m)
            min_height: 최소 높이 (m)
            max_height: 최대 높이 (m)
            base_position: 로봇 베이스 위치 [x, y, z]
        """
        self.min_radius = min_radius if min_radius is not None else self.DEFAULT_MIN_RADIUS
        self.max_radius = max_radius if max_radius is not None else self.DEFAULT_MAX_RADIUS
        self.min_height = min_height if min_height is not None else self.DEFAULT_MIN_HEIGHT
        self.max_height = max_height if max_height is not None else self.DEFAULT_MAX_HEIGHT
        self.base_position = np.array(base_position or [0, 0, 0], dtype=np.float64)

    def set_base_position(self, position: Union[List[float], np.ndarray]):
        """로봇 베이스 위치 설정"""
        self.base_position = np.array(position[:3], dtype=np.float64)

    def is_reachable(self, target_position: Union[List[float], np.ndarray]) -> bool:
        """
        타겟 위치가 작업 공간 내에 있는지 확인

        Args:
            target_position: 타겟 위치 [x, y, z]

        Returns:
            도달 가능 여부 (True/False)
        """
        target = np.array(target_position[:3], dtype=np.float64)
        relative = target - self.base_position

        # XY 평면 거리 (수평 도달 거리)
        horizontal_dist = np.sqrt(relative[0]**2 + relative[1]**2)

        # 높이 (Z 좌표)
        height = relative[2]

        # 검사
        if horizontal_dist < self.min_radius:
            return False
        if horizontal_dist > self.max_radius:
            return False
        if height < self.min_height:
            return False
        if height > self.max_height:
            return False

        return True

    def clamp_to_workspace(
        self,
        target_position: Union[List[float], np.ndarray]
    ) -> np.ndarray:
        """
        타겟 위치를 작업 공간 내로 클램핑

        작업 공간 외부의 타겟을 가장 가까운 유효 위치로 조정합니다.

        Args:
            target_position: 타겟 위치 [x, y, z]

        Returns:
            클램핑된 위치 [x, y, z]
        """
        target = np.array(target_position[:3], dtype=np.float64)
        relative = target - self.base_position
        result = relative.copy()

        # XY 평면 클램핑
        horizontal_dist = np.sqrt(relative[0]**2 + relative[1]**2)

        if horizontal_dist > 0:
            if horizontal_dist < self.min_radius:
                # 최소 거리로 확장
                scale = self.min_radius / horizontal_dist
                result[0] *= scale
                result[1] *= scale
            elif horizontal_dist > self.max_radius:
                # 최대 거리로 축소
                scale = self.max_radius / horizontal_dist
                result[0] *= scale
                result[1] *= scale

        # Z 높이 클램핑
        result[2] = np.clip(result[2], self.min_height, self.max_height)

        return result + self.base_position

    def get_workspace_info(self) -> dict:
        """작업 공간 정보 반환"""
        return {
            "min_radius": self.min_radius,
            "max_radius": self.max_radius,
            "min_height": self.min_height,
            "max_height": self.max_height,
            "base_position": self.base_position.tolist(),
        }

    def visualize_workspace(self) -> str:
        """작업 공간 시각화 (ASCII)"""
        return f"""
        Workspace Limits (Top View)
        ===========================

              Max Radius: {self.max_radius:.2f}m
                    ___
                 /       \\
               /           \\
              |      o      |  <- Base ({self.base_position[0]:.2f}, {self.base_position[1]:.2f})
               \\           /
                 \\ ___ /
              Min Radius: {self.min_radius:.2f}m

        Height Range: {self.min_height:.2f}m ~ {self.max_height:.2f}m
        """


class VRControllerMapper:
    """
    VR 컨트롤러 입력 매퍼

    Quest2ROS 입력을 로봇 제어 명령으로 매핑합니다.
    """

    # 기본 속도 스케일
    DEFAULT_LINEAR_SCALE = 0.5   # m/s per unit
    DEFAULT_ANGULAR_SCALE = 1.0  # rad/s per unit

    # 데드존 (thumbstick)
    DEFAULT_DEADZONE = 0.1

    def __init__(
        self,
        linear_scale: float = None,
        angular_scale: float = None,
        deadzone: float = None
    ):
        """
        VRControllerMapper 초기화

        Args:
            linear_scale: 선속도 스케일
            angular_scale: 각속도 스케일
            deadzone: thumbstick 데드존
        """
        self.linear_scale = linear_scale or self.DEFAULT_LINEAR_SCALE
        self.angular_scale = angular_scale or self.DEFAULT_ANGULAR_SCALE
        self.deadzone = deadzone or self.DEFAULT_DEADZONE

    def thumbstick_to_twist(
        self,
        thumbstick_h: float,
        thumbstick_v: float,
        speed_multiplier: float = 1.0
    ) -> Tuple[float, float]:
        """
        Thumbstick 입력을 Twist (linear.x, angular.z)로 변환

        Args:
            thumbstick_h: 수평 입력 (-1.0 ~ 1.0, 좌우)
            thumbstick_v: 수직 입력 (-1.0 ~ 1.0, 전후)
            speed_multiplier: 속도 배율 (0.0 ~ 1.0)

        Returns:
            (linear_x, angular_z) 튜플
        """
        # 데드존 적용
        if abs(thumbstick_h) < self.deadzone:
            thumbstick_h = 0.0
        if abs(thumbstick_v) < self.deadzone:
            thumbstick_v = 0.0

        # 속도 계산
        linear_x = thumbstick_v * self.linear_scale * speed_multiplier
        angular_z = -thumbstick_h * self.angular_scale * speed_multiplier  # 좌:+, 우:-

        return linear_x, angular_z

    def trigger_to_gripper(
        self,
        trigger_value: float,
        min_pos: float = -0.01,
        max_pos: float = 0.02
    ) -> float:
        """
        트리거 입력을 그리퍼 위치로 변환

        Args:
            trigger_value: 트리거 값 (0.0 ~ 1.0)
            min_pos: 그리퍼 최소 위치 (닫힘)
            max_pos: 그리퍼 최대 위치 (열림)

        Returns:
            그리퍼 위치
        """
        trigger_value = np.clip(trigger_value, 0.0, 1.0)
        # 0.0 → max_pos (열림), 1.0 → min_pos (닫힘)
        return max_pos - (trigger_value * (max_pos - min_pos))
