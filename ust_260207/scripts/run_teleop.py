#!/usr/bin/env python3
# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST Mobile Manipulator VR Teleoperation Script.

이 스크립트는 VR 핸드 트래킹을 통해 UST 모바일 매니퓰레이터를 제어합니다.

사용법:
    # VR 핸드 트래킹 (기본)
    ./isaaclab.sh -p scripts/run_teleop.py --teleop_device handtracking

    # 키보드 제어 (폴백)
    ./isaaclab.sh -p scripts/run_teleop.py --teleop_device keyboard

    # SpaceMouse 제어
    ./isaaclab.sh -p scripts/run_teleop.py --teleop_device spacemouse

컨트롤:
    - 오른손: 암 End-Effector 제어
    - 오른손 핀치: 그리퍼 열림/닫힘
    - 왼손 썸스틱: 모바일 베이스 이동
    - 손바닥 아래로: 환경 리셋
"""

from __future__ import annotations

import argparse
import sys
import os

# ★ CloudXR 6.0.1 IPC 버전 호환성: AppLauncher/Kit 시작 전에 반드시 설정
# Kit(OpenXR 로더)이 CloudXR Runtime과 IPC 핸드셰이크할 때 버전 체크를 무시하도록 함
# 이 환경변수가 없으면 XR_ERROR_RUNTIME_UNAVAILABLE 발생
if "IPC_IGNORE_VERSION" not in os.environ:
    os.environ["IPC_IGNORE_VERSION"] = "1"

# 프로젝트 경로 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Isaac Lab AppLauncher 설정 (다른 임포트 전에 수행해야 함)
from isaaclab.app import AppLauncher

# 명령줄 인자 파싱
parser = argparse.ArgumentParser(description="UST Mobile Manipulator VR Teleoperation")
parser.add_argument(
    "--teleop_device",
    type=str,
    default="handtracking",
    choices=["handtracking", "keyboard", "spacemouse"],
    help="텔레오퍼레이션 장치 선택",
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=1,
    help="병렬 환경 수",
)
## --headless, --enable_cameras 는 AppLauncher.add_app_launcher_args()에서 자동 추가됨

# AppLauncher 인자 추가 및 파싱
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Isaac Sim 실행
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Isaac Lab 및 프로젝트 모듈 임포트 (AppLauncher 이후)
import torch
from isaaclab.envs import ManagerBasedEnv

# 프로젝트 설정 임포트
from ust_config.ust_teleop_env_cfg import USTMobileManipulatorTeleopEnvCfg
from ust_config.ust_teleop_device_cfg import (
    USTTeleopDeviceCfg,
    USTTeleopController,
    create_ust_teleop_device,
    create_fallback_keyboard_device,
)
from ust_controllers.differential_drive_controller import DifferentialDriveController


def create_teleop_device(device_type: str, env_device: str):
    """텔레오퍼레이션 장치 생성.

    Args:
        device_type: 장치 타입 ("handtracking", "keyboard", "spacemouse")
        env_device: 환경 디바이스 ("cuda:0" 등)

    Returns:
        텔레오퍼레이션 장치 인스턴스
    """
    if device_type == "handtracking":
        cfg = USTTeleopDeviceCfg(enable_visualization=True)
        device = create_ust_teleop_device(cfg, sim_device=env_device)
        return device, cfg

    elif device_type == "keyboard":
        device = create_fallback_keyboard_device()
        cfg = USTTeleopDeviceCfg()
        return device, cfg

    elif device_type == "spacemouse":
        try:
            from isaaclab.devices import Se3SpaceMouse
            device = Se3SpaceMouse(pos_sensitivity=0.1, rot_sensitivity=0.1)
            cfg = USTTeleopDeviceCfg()
            print("[INFO] SpaceMouse device created.")
            return device, cfg
        except ImportError:
            print("[WARNING] SpaceMouse not available. Falling back to keyboard.")
            device = create_fallback_keyboard_device()
            cfg = USTTeleopDeviceCfg()
            return device, cfg

    else:
        raise ValueError(f"Unknown device type: {device_type}")


def print_controls():
    """조작 방법 출력."""
    print("\n" + "=" * 60)
    print("       UST Mobile Manipulator Teleoperation")
    print("=" * 60)
    print("\n[VR Controls]")
    print("  Right Controller:  Right arm end-effector tracking")
    print("  Right Trigger:     Right gripper open/close")
    print("  Left Controller:   Left arm end-effector tracking")
    print("  Left Trigger:      Left gripper open/close")
    print("  Right Joystick:    Right wheels (RF, RR)")
    print("  Left Joystick:     Left wheels (LF, LR)")
    print("  Palm Down:         Reset environment")
    print("\n[Keyboard Controls]")
    print("  ═══ 모드 전환 ═══════════════════════")
    print("  1:             오른쪽 암 (기본)")
    print("  2:             왼쪽 암")
    print("  3:             모바일 베이스")
    print("  ═══ 암 모드 (1, 2) ══════════════════")
    print("  W/S:           전/후진 (EE X축)")
    print("  A/D:           좌/우 이동 (EE Y축)")
    print("  Q/E:           상/하 이동 (EE Z축)")
    print("  Z/X:           Roll 회전")
    print("  T/G:           Pitch 회전")
    print("  C/V:           Yaw 회전")
    print("  K:             그리퍼 토글 (열림/닫힘)")
    print("  L:             리셋 (delta 초기화)")
    print("  ═══ 베이스 모드 (3) ═════════════════")
    print("  W/S:           직진/후진")
    print("  A/D:           좌/우 회전")
    print("\n" + "=" * 60 + "\n")


def main():
    """메인 텔레오퍼레이션 루프."""
    # 조작 방법 출력
    print_controls()

    # 환경 설정 및 생성
    print("[INFO] Creating environment...")
    env_cfg = USTMobileManipulatorTeleopEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs

    env = ManagerBasedEnv(cfg=env_cfg)
    print(f"[INFO] Environment created. Device: {env.device}")
    print(f"[INFO] Action space dim: {env.action_manager.total_action_dim}")

    # 텔레오퍼레이션 장치 생성
    print(f"[INFO] Creating {args_cli.teleop_device} device...")
    teleop_device, teleop_cfg = create_teleop_device(args_cli.teleop_device, str(env.device))

    if teleop_device is None:
        print("[ERROR] Failed to create teleop device. Exiting.")
        env.close()
        return

    # 텔레오퍼레이션 컨트롤러
    teleop_controller = USTTeleopController(
        device=teleop_device,
        cfg=teleop_cfg,
        device_str=str(env.device),
    )

    # 차동 구동 컨트롤러
    base_controller = DifferentialDriveController()

    # 환경 리셋
    print("[INFO] Resetting environment...")
    obs, info = env.reset()

    # 상태 변수
    step_count = 0
    episode_count = 0

    print("[INFO] Starting teleoperation loop. Press Ctrl+C to exit.")
    print("-" * 60)

    try:
        while simulation_app.is_running():
            with torch.inference_mode():
                # 텔레오퍼레이션 명령 획득
                commands = teleop_controller.advance()

                if commands is None:
                    # 명령 없으면 제로 액션
                    action = torch.zeros(
                        env.num_envs,
                        env.action_manager.total_action_dim,
                        device=env.device,
                    )
                else:
                    # 바퀴 속도 결정
                    base_vels = None
                    if args_cli.teleop_device == "keyboard":
                        base_linear = commands.get("base_linear", 0.0)
                        base_angular = commands.get("base_angular", 0.0)
                        if abs(base_linear) > 0.001 or abs(base_angular) > 0.001:
                            scale = 5.0
                            left_vel, right_vel = base_controller.cmd_vel_to_wheel_velocities(
                                base_linear * scale, base_angular * scale
                            )
                            base_vels = (left_vel, right_vel)
                    # VR 모드: 조이스틱 직접 매핑 (get_action_tensor 내부)

                    # 액션 텐서 생성 (4바퀴 + 암 IK + 그리퍼)
                    action = teleop_controller.get_action_tensor(
                        commands,
                        base_velocities=base_vels,
                    )

                    # 배치 차원 추가
                    action = action.unsqueeze(0).expand(env.num_envs, -1)

                # 환경 스텝
                obs, info = env.step(action)
                step_count += 1

                # 리셋 처리
                if teleop_controller.reset_triggered:
                    print(f"[INFO] Episode {episode_count} completed. Steps: {step_count}")
                    obs, info = env.reset()
                    teleop_controller.reset()
                    episode_count += 1
                    step_count = 0

                # 주기적 상태 출력 (100 스텝마다)
                if step_count % 100 == 0 and step_count > 0:
                    mode_str = f"[Mode: {teleop_controller.control_mode_name}]" if args_cli.teleop_device == "keyboard" else ""
                    # EE 위치 출력 (디버그용)
                    policy_obs = obs.get("policy", {})
                    if isinstance(policy_obs, dict):
                        if "right_ee_pose" in policy_obs:
                            r_ee = policy_obs["right_ee_pose"][0].cpu().numpy()
                            print(f"[Step {step_count}] {mode_str} Right EE: ({r_ee[0]:.3f}, {r_ee[1]:.3f}, {r_ee[2]:.3f})")
                        if "left_ee_pose" in policy_obs:
                            l_ee = policy_obs["left_ee_pose"][0].cpu().numpy()
                            print(f"[Step {step_count}] {mode_str} Left EE:  ({l_ee[0]:.3f}, {l_ee[1]:.3f}, {l_ee[2]:.3f})")

    except KeyboardInterrupt:
        print("\n[INFO] Teleoperation interrupted by user.")

    finally:
        # 정리
        print(f"\n[INFO] Session ended. Episodes: {episode_count}, Total steps: {step_count}")
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
