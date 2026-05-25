# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST VR Teleoperation Device Configuration (Dual Arm)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import torch


@dataclass
class USTTeleopDeviceCfg:
    """UST VR 텔레오퍼레이션 장치 설정 (듀얼 암).

    Meta Quest 3S 컨트롤러를 통한 듀얼 암 로봇 제어 설정입니다.

    VR 컨트롤러 매핑 (vr_handtracking.txt 기준):
      - 오른쪽 컨트롤러 → 오른쪽 매니퓰레이터 트래킹
      - 왼쪽 컨트롤러 → 왼쪽 매니퓰레이터 트래킹
      - 오른쪽 트리거 → 오른쪽 그리퍼 on/off
      - 왼쪽 트리거 → 왼쪽 그리퍼 on/off
      - 오른쪽 조이스틱 전/후 → 오른쪽 바퀴 2개 (RF, RR)
      - 왼쪽 조이스틱 전/후 → 왼쪽 바퀴 2개 (LF, LR)
    """

    # XR 앵커 설정
    xr_anchor_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    xr_anchor_rot: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    # 손목 트래킹 설정
    use_wrist_position: bool = True
    use_wrist_rotation: bool = True
    zero_out_xy_rotation: bool = True

    # 스케일링 설정
    delta_pos_scale: float = 10.0
    delta_rot_scale: float = 10.0
    alpha_smoothing: float = 0.5

    # 그리퍼 임계값 (핀치 거리, 미터)
    gripper_close_threshold: float = 0.03
    gripper_open_threshold: float = 0.05

    # 시각화
    enable_visualization: bool = True

    # 바퀴 속도 스케일 (조이스틱 → rad/s)
    wheel_speed_scale: float = 5.0


# ===== 텔레오퍼레이션 프리셋 =====

# all_dev.md 스펙 프리셋
ALL_DEV_TELEOP_PRESET = USTTeleopDeviceCfg(
    use_wrist_position=False,
    use_wrist_rotation=True,
    zero_out_xy_rotation=True,
    delta_pos_scale=1.0,
    delta_rot_scale=1.0,
    alpha_smoothing=0.3,
    gripper_close_threshold=0.04,
    gripper_open_threshold=0.06,
)

# 현재 (RelRetargeter 최적화) 프리셋
CURRENT_TELEOP_PRESET = USTTeleopDeviceCfg(
    use_wrist_position=True,
    use_wrist_rotation=True,
    zero_out_xy_rotation=True,
    delta_pos_scale=10.0,
    delta_rot_scale=10.0,
    alpha_smoothing=0.5,
    gripper_close_threshold=0.03,
    gripper_open_threshold=0.05,
)


def create_ust_teleop_device(
    cfg: USTTeleopDeviceCfg,
    sim_device: str = "cuda:0",
    retargeter_type: str = "rel",
):
    """UST VR 텔레오퍼레이션 장치 생성 (듀얼 암).

    Meta Quest 3S 컨트롤러 매핑:
      - 오른쪽 컨트롤러 → 오른쪽 매니퓰레이터 SE3 트래킹
      - 왼쪽 컨트롤러 → 왼쪽 매니퓰레이터 SE3 트래킹
      - 오른쪽 앞쪽 트리거 → 오른쪽 그리퍼 on/off
      - 왼쪽 앞쪽 트리거 → 왼쪽 그리퍼 on/off
      - 오른쪽 조이스틱 전/후 → 오른쪽 바퀴 2개 (RF, RR)
      - 왼쪽 조이스틱 전/후 → 왼쪽 바퀴 2개 (LF, LR)

    Args:
        cfg: 장치 설정
        sim_device: 시뮬레이션 장치 (CUDA 디바이스)
        retargeter_type: "rel" (Se3RelRetargeter, 기본) 또는
                        "abs" (Se3AbsRetargeter, all_dev.md 스펙)

    Returns:
        OpenXRDevice 인스턴스 (CloudXR 연결 시)
        또는 KeyboardDevice 인스턴스 (폴백)
    """
    try:
        from isaaclab.devices import OpenXRDevice, OpenXRDeviceCfg
        from isaaclab.devices.openxr import XrCfg
        from isaaclab.devices.openxr.retargeters import (
            Se3RelRetargeter,
            Se3RelRetargeterCfg,
            GripperRetargeter,
            GripperRetargeterCfg,
        )
        from isaaclab.devices import DeviceBase

        xr_cfg = XrCfg(
            anchor_pos=list(cfg.xr_anchor_pos),
            anchor_rot=list(cfg.xr_anchor_rot),
        )

        if retargeter_type == "abs":
            from isaaclab.devices.openxr.retargeters import (
                Se3AbsRetargeter,
                Se3AbsRetargeterCfg,
            )
            # 오른쪽 암 리타게터
            right_arm_retargeter = Se3AbsRetargeter(
                Se3AbsRetargeterCfg(
                    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
                    zero_out_xy_rotation=cfg.zero_out_xy_rotation,
                    use_wrist_position=cfg.use_wrist_position,
                    use_wrist_rotation=cfg.use_wrist_rotation,
                    enable_visualization=cfg.enable_visualization,
                )
            )
            # 왼쪽 암 리타게터
            left_arm_retargeter = Se3AbsRetargeter(
                Se3AbsRetargeterCfg(
                    bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
                    zero_out_xy_rotation=cfg.zero_out_xy_rotation,
                    use_wrist_position=cfg.use_wrist_position,
                    use_wrist_rotation=cfg.use_wrist_rotation,
                    enable_visualization=cfg.enable_visualization,
                )
            )
        else:
            # 오른쪽 암 리타게터 (Se3RelRetargeter)
            right_arm_retargeter = Se3RelRetargeter(
                Se3RelRetargeterCfg(
                    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
                    zero_out_xy_rotation=cfg.zero_out_xy_rotation,
                    use_wrist_position=cfg.use_wrist_position,
                    use_wrist_rotation=cfg.use_wrist_rotation,
                    delta_pos_scale_factor=cfg.delta_pos_scale,
                    delta_rot_scale_factor=cfg.delta_rot_scale,
                    alpha_pos=cfg.alpha_smoothing,
                    alpha_rot=cfg.alpha_smoothing,
                    enable_visualization=cfg.enable_visualization,
                    sim_device=sim_device,
                )
            )
            # 왼쪽 암 리타게터 (Se3RelRetargeter)
            left_arm_retargeter = Se3RelRetargeter(
                Se3RelRetargeterCfg(
                    bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
                    zero_out_xy_rotation=cfg.zero_out_xy_rotation,
                    use_wrist_position=cfg.use_wrist_position,
                    use_wrist_rotation=cfg.use_wrist_rotation,
                    delta_pos_scale_factor=cfg.delta_pos_scale,
                    delta_rot_scale_factor=cfg.delta_rot_scale,
                    alpha_pos=cfg.alpha_smoothing,
                    alpha_rot=cfg.alpha_smoothing,
                    enable_visualization=cfg.enable_visualization,
                    sim_device=sim_device,
                )
            )

        # 오른손 그리퍼 리타게터
        right_gripper_retargeter = GripperRetargeter(
            GripperRetargeterCfg(
                bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
                sim_device=sim_device,
            )
        )

        # 왼손 그리퍼 리타게터
        left_gripper_retargeter = GripperRetargeter(
            GripperRetargeterCfg(
                bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
                sim_device=sim_device,
            )
        )

        # OpenXR 장치 생성 (4개 리타게터: 오른쪽 암, 오른쪽 그리퍼, 왼쪽 암, 왼쪽 그리퍼)
        device = OpenXRDevice(
            OpenXRDeviceCfg(xr_cfg=xr_cfg),
            retargeters=[
                right_arm_retargeter, right_gripper_retargeter,
                left_arm_retargeter, left_gripper_retargeter,
            ],
        )

        print(f"[INFO] OpenXR device created (dual arm, retargeter: {retargeter_type}).")
        print(f"  Right controller → Right arm tracking + Right gripper (trigger)")
        print(f"  Left controller → Left arm tracking + Left gripper (trigger)")
        print(f"  Right joystick → Right wheels (RF, RR)")
        print(f"  Left joystick → Left wheels (LF, LR)")
        return device

    except ImportError as e:
        print(f"[WARNING] OpenXR not available: {e}")
        print("[INFO] Falling back to keyboard device.")
        return create_fallback_keyboard_device()


def create_fallback_keyboard_device():
    """폴백 키보드 장치 생성.

    VR이 사용 불가능할 때 키보드로 제어합니다.
    모드 전환으로 오른쪽 암, 왼쪽 암, 모바일 베이스를 모두 제어할 수 있습니다.

    조작 모드 (숫자키로 전환):
      1: 오른쪽 암 (기본)
      2: 왼쪽 암
      3: 모바일 베이스

    공통 조작키 (Se3Keyboard 기본):
      W/S: 전/후진 (X축 이동 or 직진/후진)
      A/D: 좌/우 이동 (Y축 이동 or 좌/우 회전)
      Q/E: 상/하 이동 (Z축)
      Z/X: Roll 회전
      T/G: Pitch 회전
      C/V: Yaw 회전
      K: 그리퍼 토글 (열림/닫힘)
      L: 리셋 (delta 초기화)
    """
    try:
        from isaaclab.devices import Se3Keyboard
        from isaaclab.devices.keyboard.se3_keyboard import Se3KeyboardCfg

        kbd_cfg = Se3KeyboardCfg(
            pos_sensitivity=0.4,
            rot_sensitivity=0.4,
        )
        device = Se3Keyboard(kbd_cfg)

        print("[INFO] Keyboard device created (dual arm + mobile base).")
        print("  모드 전환: 1=오른쪽 암 | 2=왼쪽 암 | 3=모바일 베이스")
        print("  이동: W/S(전후) A/D(좌우) Q/E(상하)")
        print("  회전: Z/X(Roll) T/G(Pitch) C/V(Yaw)")
        print("  K: 그리퍼 토글 | L: 리셋")
        return device

    except ImportError:
        print("[ERROR] Neither OpenXR nor Keyboard devices available.")
        return None


class USTTeleopController:
    """UST 텔레오퍼레이션 컨트롤러 (듀얼 암 + 모바일 베이스).

    VR 장치 또는 키보드에서 받은 명령을 로봇 액션으로 변환합니다.

    키보드 모드에서는 숫자키(1/2/3)로 제어 대상을 전환합니다:
      1: 오른쪽 암 (기본) - WASD/QE로 EE 이동, K로 그리퍼
      2: 왼쪽 암 - WASD/QE로 EE 이동, K로 그리퍼
      3: 모바일 베이스 - W/S로 직진/후진, A/D로 좌/우 회전

    액션 텐서 구조 (18D):
    - [0:4]: wheel velocities (LF, RF, LR, RR)
    - [4:10]: right arm delta pose (position 3D + rotation 3D)
    - [10]: right gripper command
    - [11:17]: left arm delta pose (position 3D + rotation 3D)
    - [17]: left gripper command
    """

    # 액션 차원
    ACTION_DIM = 18

    # 제어 모드
    MODE_RIGHT_ARM = 1
    MODE_LEFT_ARM = 2
    MODE_BASE = 3

    def __init__(
        self,
        device,
        cfg: USTTeleopDeviceCfg,
        device_str: str = "cuda:0",
    ):
        self.device = device
        self.cfg = cfg
        self.torch_device = torch.device(device_str)

        # 상태 변수
        self.gripper_closed_right = False
        self.gripper_closed_left = False
        self.reset_triggered = False

        # 그리퍼 명령 상태 유지 (모드 전환 시 비활성 그리퍼 상태 보존)
        # BinaryJointPositionAction: >0 → open, <=0 → close
        self._last_right_gripper_cmd = 1.0  # 기본: 열림
        self._last_left_gripper_cmd = 1.0   # 기본: 열림

        # 각 암별 그리퍼 토글 상태 (Se3Keyboard의 _close_gripper는 공유 상태이므로 별도 관리)
        self._right_gripper_closed = False
        self._left_gripper_closed = False

        # 키보드 모드 전환 상태
        self._control_mode = self.MODE_RIGHT_ARM
        self._is_keyboard_device = False

        # 키보드 장치일 경우 모드 전환 콜백 등록
        self._setup_keyboard_mode_switching()

        # 스무딩 버퍼
        self._prev_right_delta_pos = torch.zeros(3, device=self.torch_device)
        self._prev_right_delta_rot = torch.zeros(3, device=self.torch_device)
        self._prev_left_delta_pos = torch.zeros(3, device=self.torch_device)
        self._prev_left_delta_rot = torch.zeros(3, device=self.torch_device)

    def _setup_keyboard_mode_switching(self):
        """키보드 모드 전환 콜백 등록."""
        if hasattr(self.device, 'add_callback'):
            self._is_keyboard_device = True

            def _switch_to_right_arm():
                if self._control_mode != self.MODE_RIGHT_ARM:
                    # 현재 모드의 그리퍼 상태 저장
                    self._save_current_gripper_state()
                    self._control_mode = self.MODE_RIGHT_ARM
                    # 오른쪽 암의 그리퍼 상태로 Se3Keyboard 동기화
                    self._sync_gripper_state(self._right_gripper_closed)
                    print("[MODE] >>> 오른쪽 암 제어 (WASD/QE=EE이동, K=그리퍼)")

            def _switch_to_left_arm():
                if self._control_mode != self.MODE_LEFT_ARM:
                    self._save_current_gripper_state()
                    self._control_mode = self.MODE_LEFT_ARM
                    self._sync_gripper_state(self._left_gripper_closed)
                    print("[MODE] >>> 왼쪽 암 제어 (WASD/QE=EE이동, K=그리퍼)")

            def _switch_to_base():
                if self._control_mode != self.MODE_BASE:
                    self._save_current_gripper_state()
                    self._control_mode = self.MODE_BASE
                    print("[MODE] >>> 모바일 베이스 제어 (W/S=직진/후진, A/D=좌/우 회전)")

            self.device.add_callback("NUMPAD_1", _switch_to_right_arm)
            self.device.add_callback("NUMPAD_2", _switch_to_left_arm)
            self.device.add_callback("NUMPAD_3", _switch_to_base)
            self.device.add_callback("KEY_1", _switch_to_right_arm)
            self.device.add_callback("KEY_2", _switch_to_left_arm)
            self.device.add_callback("KEY_3", _switch_to_base)

    def _save_current_gripper_state(self):
        """현재 모드의 그리퍼 상태를 Se3Keyboard에서 읽어 저장."""
        if hasattr(self.device, '_close_gripper'):
            if self._control_mode == self.MODE_RIGHT_ARM:
                self._right_gripper_closed = self.device._close_gripper
            elif self._control_mode == self.MODE_LEFT_ARM:
                self._left_gripper_closed = self.device._close_gripper

    def _sync_gripper_state(self, closed: bool):
        """Se3Keyboard의 _close_gripper를 지정된 상태로 동기화."""
        if hasattr(self.device, '_close_gripper'):
            self.device._close_gripper = closed

    @property
    def control_mode(self) -> int:
        """현재 제어 모드 반환."""
        return self._control_mode

    @property
    def control_mode_name(self) -> str:
        """현재 제어 모드 이름 반환."""
        if self._control_mode == self.MODE_RIGHT_ARM:
            return "오른쪽 암"
        elif self._control_mode == self.MODE_LEFT_ARM:
            return "왼쪽 암"
        else:
            return "모바일 베이스"

    def advance(self) -> Optional[dict]:
        """장치 업데이트 및 명령 획득.

        Returns:
            명령 딕셔너리:
            - right_arm_delta_pose: 오른쪽 암 SE3 delta pose (6D)
            - right_gripper_command: 오른쪽 그리퍼 명령
            - left_arm_delta_pose: 왼쪽 암 SE3 delta pose (6D)
            - left_gripper_command: 왼쪽 그리퍼 명령
            - right_joystick_y / left_joystick_y: 조이스틱 입력
            - base_linear / base_angular: 베이스 속도 명령
            - reset_triggered: 리셋 제스처 감지됨
        """
        if self.device is None:
            return None

        try:
            raw_commands = self.device.advance()
        except Exception as e:
            print(f"[WARNING] Device advance failed: {e}")
            return None

        if raw_commands is None:
            return None

        commands = self._parse_commands(raw_commands)
        return commands

    def _parse_commands(self, raw) -> dict:
        """Raw 명령을 UST 포맷으로 변환 (듀얼 암 + 모바일 베이스).

        Args:
            raw: 장치에서 반환된 명령.
                - Se3Keyboard: Tensor [dx, dy, dz, rx, ry, rz, gripper] (7D)
                - OpenXRDevice: dict (듀얼 암 리타게터 결과)

        Returns:
            명령 딕셔너리
        """
        # BinaryJointPositionAction: >0 → open, <=0 → close
        # 비활성 그리퍼는 마지막 상태를 유지 (기본값 1.0 = 열림)
        commands = {
            "right_arm_delta_pose": [0.0] * 6,
            "right_gripper_command": self._last_right_gripper_cmd,
            "left_arm_delta_pose": [0.0] * 6,
            "left_gripper_command": self._last_left_gripper_cmd,
            "right_joystick_y": 0.0,
            "left_joystick_y": 0.0,
            "base_linear": 0.0,
            "base_angular": 0.0,
            "thumbstick": (0.0, 0.0),
            "reset_triggered": False,
        }

        # Tensor 반환 (Se3Keyboard - 모드에 따라 제어 대상 전환)
        if isinstance(raw, torch.Tensor):
            raw_np = raw.cpu().numpy()
            delta_pose = list(raw_np[:6])

            # K키 그리퍼 명령: 선택된 매니퓰레이터의 그리퍼만 작동 (양쪽 핑거 동시)
            # Se3Keyboard: raw_np[6] < 0 = 그리퍼 닫힘
            # BinaryJointPositionAction: 음수 → close, 양수 → open
            gripper_cmd = 1.0  # 기본: 열림
            if len(raw_np) > 6:
                gripper_cmd = -1.0 if raw_np[6] < 0 else 1.0

            # 선택된 매니퓰레이터의 그리퍼만 업데이트, 다른 쪽은 마지막 상태 유지
            if self._control_mode == self.MODE_RIGHT_ARM:
                commands["right_gripper_command"] = gripper_cmd
                self._last_right_gripper_cmd = gripper_cmd
                commands["left_gripper_command"] = self._last_left_gripper_cmd
            elif self._control_mode == self.MODE_LEFT_ARM:
                commands["left_gripper_command"] = gripper_cmd
                self._last_left_gripper_cmd = gripper_cmd
                commands["right_gripper_command"] = self._last_right_gripper_cmd
            else:
                # 베이스 모드: 양쪽 그리퍼 모두 마지막 상태 유지
                commands["right_gripper_command"] = self._last_right_gripper_cmd
                commands["left_gripper_command"] = self._last_left_gripper_cmd

            if self._control_mode == self.MODE_RIGHT_ARM:
                commands["right_arm_delta_pose"] = delta_pose
            elif self._control_mode == self.MODE_LEFT_ARM:
                commands["left_arm_delta_pose"] = delta_pose
            elif self._control_mode == self.MODE_BASE:
                # 베이스 모드: W/S(전진/후진)=delta_pose[0], A/D(회전)=delta_pose[1]
                commands["base_linear"] = raw_np[0]    # W/S → 전/후진
                commands["base_angular"] = -raw_np[1]  # A/D → 좌/우 회전 (부호 반전)

            return commands

        # Dict 반환 (OpenXRDevice - 듀얼 암)
        if isinstance(raw, dict):
            # 오른쪽 암 SE3 delta pose
            if "delta_pose" in raw:
                delta_pose = raw["delta_pose"]
                if isinstance(delta_pose, torch.Tensor):
                    delta_pose = delta_pose.cpu().numpy()
                commands["right_arm_delta_pose"] = list(delta_pose[:6])

            # 오른쪽 그리퍼
            if "gripper" in raw:
                gripper_val = raw["gripper"]
                if isinstance(gripper_val, torch.Tensor):
                    gripper_val = gripper_val.item()
                commands["right_gripper_command"] = float(gripper_val)

            # 왼쪽 암 SE3 delta pose
            if "left_delta_pose" in raw:
                left_dp = raw["left_delta_pose"]
                if isinstance(left_dp, torch.Tensor):
                    left_dp = left_dp.cpu().numpy()
                commands["left_arm_delta_pose"] = list(left_dp[:6])

            # 왼쪽 그리퍼
            if "left_gripper" in raw:
                left_grip = raw["left_gripper"]
                if isinstance(left_grip, torch.Tensor):
                    left_grip = left_grip.item()
                commands["left_gripper_command"] = float(left_grip)

            # VR 조이스틱 → 바퀴 제어
            if "right_joystick" in raw:
                joy_r = raw["right_joystick"]
                if isinstance(joy_r, torch.Tensor):
                    joy_r = joy_r.cpu().numpy()
                commands["right_joystick_y"] = float(joy_r[1])

            if "left_joystick" in raw:
                joy_l = raw["left_joystick"]
                if isinstance(joy_l, torch.Tensor):
                    joy_l = joy_l.cpu().numpy()
                commands["left_joystick_y"] = float(joy_l[1])

            # 레거시 thumbstick
            if "thumbstick" in raw:
                thumbstick = raw["thumbstick"]
                if isinstance(thumbstick, torch.Tensor):
                    thumbstick = thumbstick.cpu().numpy()
                commands["thumbstick"] = tuple(thumbstick[:2])

            # 리셋 제스처
            if "reset" in raw:
                commands["reset_triggered"] = bool(raw["reset"])
                self.reset_triggered = commands["reset_triggered"]

        return commands

    def get_action_tensor(
        self,
        commands: dict,
        base_velocities: tuple[float, float] | None = None,
    ) -> torch.Tensor:
        """명령을 액션 텐서로 변환 (듀얼 암 + 모바일 베이스).

        Args:
            commands: advance()에서 반환된 명령 딕셔너리
            base_velocities: (left_wheel_vel, right_wheel_vel)
                키보드 모드에서 DifferentialDriveController 출력.

        Returns:
            액션 텐서 (18D):
            - [0:4]: wheel velocities (LF, RF, LR, RR)
            - [4:10]: right arm delta pose (position 3D + rotation 3D)
            - [10]: right gripper command
            - [11:17]: left arm delta pose (position 3D + rotation 3D)
            - [17]: left gripper command
        """
        action = torch.zeros(self.ACTION_DIM, device=self.torch_device)

        # 바퀴 속도 결정
        left_joy = commands.get("left_joystick_y", 0.0)
        right_joy = commands.get("right_joystick_y", 0.0)

        if abs(left_joy) > 0.01 or abs(right_joy) > 0.01:
            scale = self.cfg.wheel_speed_scale
            action[0] = left_joy * scale     # wheel_left_front
            action[1] = right_joy * scale    # wheel_right_front
            action[2] = left_joy * scale     # wheel_left_rear
            action[3] = right_joy * scale    # wheel_right_rear
        elif base_velocities is not None:
            left_vel, right_vel = base_velocities
            action[0] = left_vel
            action[1] = right_vel
            action[2] = left_vel
            action[3] = right_vel

        # 오른쪽 암 delta pose
        right_arm_pose = commands.get("right_arm_delta_pose", [0.0] * 6)
        action[4:10] = torch.tensor(right_arm_pose, device=self.torch_device)

        # 오른쪽 그리퍼
        right_grip = commands.get("right_gripper_command", 0.0)
        action[10] = right_grip

        # 왼쪽 암 delta pose
        left_arm_pose = commands.get("left_arm_delta_pose", [0.0] * 6)
        action[11:17] = torch.tensor(left_arm_pose, device=self.torch_device)

        # 왼쪽 그리퍼
        left_grip = commands.get("left_gripper_command", 0.0)
        action[17] = left_grip

        # 디버그: 그리퍼 명령이 바뀔 때만 출력
        if not hasattr(self, '_prev_grip_debug'):
            self._prev_grip_debug = (0.0, 0.0)
        if (right_grip, left_grip) != self._prev_grip_debug:
            print(f"[GRIP DEBUG] mode={self._control_mode} right={right_grip:.1f} left={left_grip:.1f} action[10]={action[10]:.1f} action[17]={action[17]:.1f}")
            self._prev_grip_debug = (right_grip, left_grip)

        return action

    def reset(self):
        """컨트롤러 상태 리셋."""
        self.gripper_closed_right = False
        self.gripper_closed_left = False
        self.reset_triggered = False
        self._control_mode = self.MODE_RIGHT_ARM
        self._last_right_gripper_cmd = 1.0  # 열림으로 리셋
        self._last_left_gripper_cmd = 1.0   # 열림으로 리셋
        self._right_gripper_closed = False
        self._left_gripper_closed = False
        self._prev_right_delta_pos.zero_()
        self._prev_right_delta_rot.zero_()
        self._prev_left_delta_pos.zero_()
        self._prev_left_delta_rot.zero_()

        if hasattr(self.device, 'reset'):
            self.device.reset()
