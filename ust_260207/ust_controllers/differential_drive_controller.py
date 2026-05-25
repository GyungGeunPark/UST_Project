# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""Differential Drive Controller for TurtleBot3 Waffle Pi."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple
import numpy as np
import torch


@dataclass
class DifferentialDriveParams:
    """차동 구동 로봇 파라미터.

    TurtleBot3 Waffle Pi 기본값으로 설정됩니다.

    Attributes:
        wheel_radius: 바퀴 반지름 (m)
        wheel_base: 좌우 바퀴 중심 간격 (m)
        max_linear_vel: 최대 선속도 (m/s)
        max_angular_vel: 최대 각속도 (rad/s)
        max_wheel_vel: 최대 바퀴 각속도 (rad/s)
    """

    wheel_radius: float = 0.033  # TurtleBot3 바퀴 반지름
    wheel_base: float = 0.287  # TurtleBot3 좌우 바퀴 간격
    max_linear_vel: float = 0.26  # m/s
    max_angular_vel: float = 1.82  # rad/s
    max_wheel_vel: float = 30.0  # rad/s (모터 제한)


class DifferentialDriveController:
    """차동 구동 컨트롤러.

    VR 썸스틱 입력을 좌/우 바퀴 속도로 변환합니다.

    속도 모델:
        v = (v_left + v_right) / 2 × r
        ω = (v_right - v_left) / L × r

    여기서:
        v: 선속도 (m/s)
        ω: 각속도 (rad/s)
        v_left, v_right: 좌/우 바퀴 각속도 (rad/s)
        r: 바퀴 반지름 (m)
        L: 바퀴 간격 (m)
    """

    def __init__(self, params: DifferentialDriveParams = None):
        """컨트롤러 초기화.

        Args:
            params: 차동 구동 파라미터 (None이면 기본값 사용)
        """
        self.params = params or DifferentialDriveParams()

    def thumbstick_to_wheel_velocities(
        self,
        thumbstick_x: float,
        thumbstick_y: float,
    ) -> Tuple[float, float]:
        """썸스틱 입력을 바퀴 속도로 변환.

        Args:
            thumbstick_x: 좌우 입력 (-1 ~ 1), 양수 = 우회전
            thumbstick_y: 전후 입력 (-1 ~ 1), 양수 = 전진

        Returns:
            (left_wheel_vel, right_wheel_vel): 바퀴 각속도 (rad/s)
        """
        # 입력 클램핑
        thumbstick_x = np.clip(thumbstick_x, -1.0, 1.0)
        thumbstick_y = np.clip(thumbstick_y, -1.0, 1.0)

        # 데드존 적용 (작은 입력 무시)
        deadzone = 0.1
        if abs(thumbstick_x) < deadzone:
            thumbstick_x = 0.0
        if abs(thumbstick_y) < deadzone:
            thumbstick_y = 0.0

        # 선속도/각속도 계산
        linear_vel = thumbstick_y * self.params.max_linear_vel
        angular_vel = -thumbstick_x * self.params.max_angular_vel

        # 역기구학: (v, ω) → 바퀴 속도
        # v_left = (v - L/2 × ω) / r
        # v_right = (v + L/2 × ω) / r
        half_base = self.params.wheel_base / 2.0
        r = self.params.wheel_radius

        v_left = (linear_vel - half_base * angular_vel) / r
        v_right = (linear_vel + half_base * angular_vel) / r

        # 속도 제한
        v_left, v_right = self.clamp_velocities(v_left, v_right)

        return v_left, v_right

    def cmd_vel_to_wheel_velocities(
        self,
        linear_vel: float,
        angular_vel: float,
    ) -> Tuple[float, float]:
        """Twist 명령 (cmd_vel)을 바퀴 속도로 변환.

        ROS2 Twist 메시지와 호환되는 인터페이스입니다.

        Args:
            linear_vel: 선속도 (m/s)
            angular_vel: 각속도 (rad/s)

        Returns:
            (left_wheel_vel, right_wheel_vel): 바퀴 각속도 (rad/s)
        """
        half_base = self.params.wheel_base / 2.0
        r = self.params.wheel_radius

        v_left = (linear_vel - half_base * angular_vel) / r
        v_right = (linear_vel + half_base * angular_vel) / r

        v_left, v_right = self.clamp_velocities(v_left, v_right)

        return v_left, v_right

    def wheel_velocities_to_cmd_vel(
        self,
        left_vel: float,
        right_vel: float,
    ) -> Tuple[float, float]:
        """바퀴 속도를 Twist 명령으로 변환 (순기구학).

        Args:
            left_vel: 좌측 바퀴 각속도 (rad/s)
            right_vel: 우측 바퀴 각속도 (rad/s)

        Returns:
            (linear_vel, angular_vel): 선속도 (m/s), 각속도 (rad/s)
        """
        r = self.params.wheel_radius
        L = self.params.wheel_base

        # 순기구학
        # v = (v_left + v_right) / 2 × r
        # ω = (v_right - v_left) / L × r
        linear_vel = (left_vel + right_vel) / 2.0 * r
        angular_vel = (right_vel - left_vel) / L * r

        return linear_vel, angular_vel

    def clamp_velocities(
        self,
        left_vel: float,
        right_vel: float,
    ) -> Tuple[float, float]:
        """바퀴 속도 제한.

        비율을 유지하면서 최대 속도 이내로 클램핑합니다.

        Args:
            left_vel: 좌측 바퀴 속도
            right_vel: 우측 바퀴 속도

        Returns:
            클램핑된 (left_vel, right_vel)
        """
        max_vel = self.params.max_wheel_vel
        max_input = max(abs(left_vel), abs(right_vel))

        if max_input > max_vel:
            # 비율 유지하면서 스케일 다운
            scale = max_vel / max_input
            left_vel *= scale
            right_vel *= scale

        return left_vel, right_vel

    def thumbstick_to_wheel_velocities_tensor(
        self,
        thumbstick: torch.Tensor,
    ) -> torch.Tensor:
        """배치 썸스틱 입력을 바퀴 속도로 변환 (PyTorch 버전).

        Args:
            thumbstick: 썸스틱 입력 텐서 (batch_size, 2) [x, y]

        Returns:
            바퀴 속도 텐서 (batch_size, 2) [left, right]
        """
        # 입력 클램핑
        thumbstick = torch.clamp(thumbstick, -1.0, 1.0)

        # 데드존 적용
        deadzone = 0.1
        mask_x = torch.abs(thumbstick[:, 0]) < deadzone
        mask_y = torch.abs(thumbstick[:, 1]) < deadzone
        thumbstick = thumbstick.clone()
        thumbstick[mask_x, 0] = 0.0
        thumbstick[mask_y, 1] = 0.0

        # 선속도/각속도 계산
        linear_vel = thumbstick[:, 1] * self.params.max_linear_vel
        angular_vel = -thumbstick[:, 0] * self.params.max_angular_vel

        # 역기구학
        half_base = self.params.wheel_base / 2.0
        r = self.params.wheel_radius

        v_left = (linear_vel - half_base * angular_vel) / r
        v_right = (linear_vel + half_base * angular_vel) / r

        # 속도 제한
        wheel_vels = torch.stack([v_left, v_right], dim=1)
        max_vel = self.params.max_wheel_vel
        max_input = torch.max(torch.abs(wheel_vels), dim=1, keepdim=True)[0]
        scale = torch.where(
            max_input > max_vel,
            max_vel / max_input,
            torch.ones_like(max_input),
        )
        wheel_vels = wheel_vels * scale

        return wheel_vels


class DifferentialDriveOdometry:
    """차동 구동 오도메트리.

    바퀴 인코더 데이터로부터 로봇 위치를 추정합니다.
    """

    def __init__(self, params: DifferentialDriveParams = None):
        """오도메트리 초기화.

        Args:
            params: 차동 구동 파라미터
        """
        self.params = params or DifferentialDriveParams()

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 이전 휠 위치
        self._prev_left_pos = 0.0
        self._prev_right_pos = 0.0
        self._initialized = False

    def update(self, left_pos: float, right_pos: float) -> Tuple[float, float, float]:
        """휠 위치로부터 오도메트리 업데이트.

        Args:
            left_pos: 좌측 휠 위치 (rad)
            right_pos: 우측 휠 위치 (rad)

        Returns:
            (x, y, theta): 로봇 위치 (m, m, rad)
        """
        if not self._initialized:
            self._prev_left_pos = left_pos
            self._prev_right_pos = right_pos
            self._initialized = True
            return self.x, self.y, self.theta

        # 휠 이동량 계산
        delta_left = left_pos - self._prev_left_pos
        delta_right = right_pos - self._prev_right_pos

        # 선형/각도 이동량
        r = self.params.wheel_radius
        L = self.params.wheel_base

        delta_s = (delta_left + delta_right) / 2.0 * r
        delta_theta = (delta_right - delta_left) / L * r

        # 위치 업데이트
        self.x += delta_s * np.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * np.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # 각도 정규화 (-π, π)
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # 이전 위치 저장
        self._prev_left_pos = left_pos
        self._prev_right_pos = right_pos

        return self.x, self.y, self.theta

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """오도메트리 리셋.

        Args:
            x: 초기 x 위치 (m)
            y: 초기 y 위치 (m)
            theta: 초기 방향 (rad)
        """
        self.x = x
        self.y = y
        self.theta = theta
        self._initialized = False

    def get_pose(self) -> Tuple[float, float, float]:
        """현재 로봇 포즈 반환.

        Returns:
            (x, y, theta): 위치 (m, m, rad)
        """
        return self.x, self.y, self.theta
