#!/usr/bin/env python3
# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST Mobile Manipulator Demo Recording Script.

이 스크립트는 텔레오퍼레이션을 통해 모방 학습용 데모 데이터를 수집합니다.

** 중요: 키보드 텔레오퍼레이션은 GUI 윈도우가 필요합니다. **
** --headless 모드에서는 키보드 입력이 작동하지 않습니다! **

사용법:
    # 키보드 텔레오퍼레이션 (GUI 필요 - --headless 사용 불가!)
    ./isaaclab.sh -p scripts/record_demos.py --num_demos 10 --teleop_device keyboard --enable_cameras

    # VR 핸드트래킹 (headless 가능)
    ./isaaclab.sh -p scripts/record_demos.py --num_demos 20 --headless --enable_cameras

    # 이미지 포함
    ./isaaclab.sh -p scripts/record_demos.py --num_demos 20 --include_images --enable_cameras

키보드 컨트롤:
    모드 전환:
    - 1: 오른쪽 암 (기본)
    - 2: 왼쪽 암
    - 3: 모바일 베이스
    암 모드 (1, 2):
    - W/S: 전/후진 (End-Effector X축)
    - A/D: 좌/우 이동 (End-Effector Y축)
    - Q/E: 상/하 이동 (End-Effector Z축)
    - Z/X: Roll 회전, T/G: Pitch 회전, C/V: Yaw 회전
    - K: 그리퍼 토글 (열림/닫힘)
    - L: 리셋 (delta 초기화)
    베이스 모드 (3):
    - W/S: 직진/후진
    - A/D: 좌/우 회전
    공통:
    - SPACE: 에피소드 종료 (성공으로 저장)
    - BACKSPACE: 에피소드 폐기 (실패, 재시작)

VR 컨트롤:
    - 오른쪽 컨트롤러: 오른쪽 암 End-Effector 트래킹
    - 오른쪽 트리거: 오른쪽 그리퍼 열림/닫힘
    - 왼쪽 컨트롤러: 왼쪽 암 End-Effector 트래킹
    - 왼쪽 트리거: 왼쪽 그리퍼 열림/닫힘
    - 오른쪽 조이스틱: 오른쪽 바퀴 2개 (RF, RR)
    - 왼쪽 조이스틱: 왼쪽 바퀴 2개 (LF, LR)
    - 손바닥 아래로: 에피소드 종료 (성공)
    - 주먹 제스처: 에피소드 폐기 (실패)
"""

from __future__ import annotations

import argparse
import sys
import os
from datetime import datetime
from pathlib import Path

# 프로젝트 경로 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Isaac Lab AppLauncher 설정
from isaaclab.app import AppLauncher

# 명령줄 인자 파싱
parser = argparse.ArgumentParser(description="UST Demo Recording")
parser.add_argument(
    "--num_demos",
    type=int,
    default=20,
    help="수집할 데모 수",
)
parser.add_argument(
    "--dataset_path",
    type=str,
    default="./datasets",
    help="데이터셋 저장 경로",
)
parser.add_argument(
    "--task",
    type=str,
    default="manipulation",
    help="태스크 이름 (데이터셋 파일명에 포함)",
)
parser.add_argument(
    "--include_images",
    action="store_true",
    help="RGB-D 이미지 저장",
)
parser.add_argument(
    "--teleop_device",
    type=str,
    default="handtracking",
    choices=["handtracking", "keyboard", "spacemouse"],
    help="텔레오퍼레이션 장치",
)
parser.add_argument(
    "--max_episode_length",
    type=int,
    default=500,
    help="최대 에피소드 길이 (스텝)",
)
parser.add_argument(
    "--min_episode_length",
    type=int,
    default=10,
    help="최소 에피소드 길이 (너무 짧은 에피소드 폐기)",
)

# AppLauncher 인자 추가
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# ============================================================
# 핵심 체크: keyboard/spacemouse 장치는 GUI 윈도우 필요!
# Se3Keyboard는 omni.appwindow를 통해 키보드 이벤트를 받으므로
# --headless 모드에서는 키 입력을 받을 수 없습니다.
# ============================================================
if args_cli.teleop_device in ("keyboard", "spacemouse") and args_cli.headless:
    print("\n" + "!" * 60)
    print("  [ERROR] --headless 모드에서는 키보드/스페이스마우스를")
    print("  사용할 수 없습니다!")
    print()
    print("  Se3Keyboard는 Isaac Sim GUI 윈도우를 통해 키보드")
    print("  이벤트를 받습니다. --headless 모드에서는 GUI가 없어서")
    print("  키 입력이 전혀 작동하지 않습니다.")
    print()
    print("  해결 방법:")
    print("    1) --headless를 제거하세요 (GUI 필요):")
    print("       ./isaaclab.sh -p scripts/record_demos.py \\")
    print("           --teleop_device keyboard --enable_cameras")
    print()
    print("    2) VR 핸드트래킹은 headless 모드에서도 사용 가능:")
    print("       ./isaaclab.sh -p scripts/record_demos.py \\")
    print("           --teleop_device handtracking --headless --enable_cameras")
    print("!" * 60 + "\n")
    sys.exit(1)

# Isaac Sim 실행
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 모듈 임포트
import torch
import numpy as np
from isaaclab.envs import ManagerBasedEnv

from ust_config.ust_teleop_env_cfg import USTMobileManipulatorDataCollectEnvCfg
from ust_config.ust_teleop_device_cfg import (
    USTTeleopDeviceCfg,
    USTTeleopController,
    create_ust_teleop_device,
    create_fallback_keyboard_device,
)
from ust_controllers.differential_drive_controller import DifferentialDriveController
from ust_utils.hdf5_recorder import HDF5DatasetRecorder


def create_teleop_device(device_type: str, env_device: str):
    """텔레오퍼레이션 장치 생성.

    Returns:
        (device, cfg) 튜플
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
            device = Se3SpaceMouse(pos_sensitivity=0.1)
            cfg = USTTeleopDeviceCfg()
            return device, cfg
        except ImportError:
            device = create_fallback_keyboard_device()
            cfg = USTTeleopDeviceCfg()
            return device, cfg
    else:
        raise ValueError(f"Unknown device type: {device_type}")


def setup_keyboard_episode_controls(device) -> dict:
    """키보드 에피소드 제어 키 등록.

    Se3Keyboard는 Isaac Sim GUI 윈도우를 통해 키 이벤트를 받으므로,
    에피소드 성공/실패 제어도 같은 방식으로 등록합니다.

    키 매핑 (Se3Keyboard 기본 키와 충돌 방지):
        Se3Keyboard에서 사용하는 키: W/S/A/D/Q/E (이동), Z/X/T/G/C/V (회전), K (그리퍼), L (리셋)
        → 에피소드 제어에는 사용하지 않는 키를 할당

        SPACE:  에피소드 종료 (성공으로 저장)
        BACKSPACE: 에피소드 폐기 (실패, 재시작)

    Returns:
        상태를 추적하는 dict (mutable reference)
    """
    import carb.input
    import omni.appwindow

    state = {
        "episode_success": False,
        "episode_discard": False,
    }

    appwindow = omni.appwindow.get_default_app_window()
    input_iface = carb.input.acquire_input_interface()
    keyboard = appwindow.get_keyboard()

    def _on_episode_key(event, *args):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.SPACE:
                state["episode_success"] = True
            elif event.input == carb.input.KeyboardInput.BACKSPACE:
                state["episode_discard"] = True
        return True

    input_iface.subscribe_to_keyboard_events(keyboard, _on_episode_key)
    return state


def print_recording_info(args):
    """녹화 정보 출력."""
    print("\n" + "=" * 60)
    print("       UST Demo Recording Session")
    print("=" * 60)
    print(f"\n[Configuration]")
    print(f"  Target demos: {args.num_demos}")
    print(f"  Task: {args.task}")
    print(f"  Include images: {args.include_images}")
    print(f"  Max episode length: {args.max_episode_length}")
    print(f"  Min episode length: {args.min_episode_length}")
    print(f"  Device: {args.teleop_device}")

    if args.teleop_device == "keyboard":
        print(f"\n[키보드 컨트롤] (GUI 윈도우에 포커스 필요!)")
        print("  ═══ 모드 전환 ═══════════════════════")
        print("  1:             오른쪽 암 (기본)")
        print("  2:             왼쪽 암")
        print("  3:             모바일 베이스")
        print("  ═══ 암 모드 (1, 2) ══════════════════")
        print("  W/S:           전/후진 (End-Effector X축)")
        print("  A/D:           좌/우 이동 (End-Effector Y축)")
        print("  Q/E:           상/하 이동 (End-Effector Z축)")
        print("  Z/X:           Roll 회전")
        print("  T/G:           Pitch 회전")
        print("  C/V:           Yaw 회전")
        print("  K:             그리퍼 토글 (열림/닫힘)")
        print("  L:             리셋 (delta 초기화)")
        print("  ═══ 베이스 모드 (3) ═════════════════")
        print("  W/S:           직진/후진")
        print("  A/D:           좌/우 회전")
        print("  ─────────────────────────────────────")
        print("  SPACE:         에피소드 종료 (성공으로 저장)")
        print("  BACKSPACE:     에피소드 폐기 (실패, 재시작)")
    else:
        print(f"\n[VR 컨트롤]")
        print("  오른쪽 컨트롤러:  오른쪽 암 End-Effector 트래킹")
        print("  오른쪽 트리거:    오른쪽 그리퍼 열림/닫힘")
        print("  왼쪽 컨트롤러:    왼쪽 암 End-Effector 트래킹")
        print("  왼쪽 트리거:      왼쪽 그리퍼 열림/닫힘")
        print("  오른쪽 조이스틱:  오른쪽 바퀴 (RF, RR)")
        print("  왼쪽 조이스틱:    왼쪽 바퀴 (LF, LR)")
        print("  ─────────────────────────────────────")
        print("  손바닥 아래로:    에피소드 종료 (성공)")
        print("  주먹 제스처:      에피소드 폐기 (실패)")

    print("\n  max length 도달 시: 자동 타임아웃 (폐기됨)")
    print("=" * 60 + "\n")


def main():
    """메인 녹화 루프."""
    print_recording_info(args_cli)

    # 환경 생성
    print("[INFO] Creating environment...")
    env_cfg = USTMobileManipulatorDataCollectEnvCfg()
    env_cfg.scene.num_envs = 1

    env = ManagerBasedEnv(cfg=env_cfg)
    print(f"[INFO] Environment created. Device: {env.device}")

    # 텔레오퍼레이션 장치
    print(f"[INFO] Creating {args_cli.teleop_device} device...")
    teleop_device, teleop_cfg = create_teleop_device(
        args_cli.teleop_device, str(env.device)
    )

    if teleop_device is None:
        print("[ERROR] Failed to create teleop device.")
        env.close()
        return

    # 키보드 에피소드 제어 키 등록
    # Se3Keyboard의 기본 키(WASD/QE/K 등)와 별도로 에피소드 종료/폐기 키를 등록
    kbd_episode_state = None
    if args_cli.teleop_device == "keyboard":
        kbd_episode_state = setup_keyboard_episode_controls(teleop_device)
        print("[INFO] 에피소드 제어 키 등록 완료: SPACE=성공저장, BACKSPACE=폐기")

    # 컨트롤러
    teleop_controller = USTTeleopController(
        device=teleop_device,
        cfg=teleop_cfg,
        device_str=str(env.device),
    )
    base_controller = DifferentialDriveController()

    # 데이터 레코더 설정
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dataset_file = Path(args_cli.dataset_path) / f"ust_{args_cli.task}_{timestamp}.hdf5"

    observation_keys = [
        "right_arm_joint_pos",  # 4D
        "right_gripper_pos",    # 1D
        "right_ee_pose",        # 7D (pos 3D + quat 4D)
        "left_arm_joint_pos",   # 4D
        "left_gripper_pos",     # 1D
        "left_ee_pose",         # 7D (pos 3D + quat 4D)
        "object_pos",           # 3D
    ]
    action_dim = 18  # 4 (wheels) + 6 (right arm IK) + 1 (right gripper) + 6 (left arm IK) + 1 (left gripper)

    recorder = HDF5DatasetRecorder(
        file_path=str(dataset_file),
        observation_keys=observation_keys,
        action_dim=action_dim,
        include_images=args_cli.include_images,
        image_shape=(720, 1280, 3),
    )

    # 상태 변수
    completed_demos = 0
    discarded_demos = 0
    current_step = 0

    # 환경 리셋 및 녹화 시작
    obs, info = env.reset()
    recorder.start_episode()

    print(f"\n[Recording] Demo 1/{args_cli.num_demos}")
    print("-" * 40)

    try:
        while simulation_app.is_running() and completed_demos < args_cli.num_demos:
            with torch.inference_mode():
                # 명령 획득
                commands = teleop_controller.advance()

                if commands is None:
                    action = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
                else:
                    # 바퀴 속도 결정
                    # VR 모드: 조이스틱 직접 매핑 (get_action_tensor 내부에서 처리)
                    # 키보드 모드: 모드 3(베이스)일 때 DifferentialDriveController 사용
                    base_vels = None
                    if args_cli.teleop_device == "keyboard":
                        base_linear = commands.get("base_linear", 0.0)
                        base_angular = commands.get("base_angular", 0.0)
                        if abs(base_linear) > 0.001 or abs(base_angular) > 0.001:
                            # 베이스 모드 활성 시 base_linear/angular → wheel velocities
                            scale = 5.0  # 키보드 입력을 적절한 속도로 스케일
                            left_vel, right_vel = base_controller.cmd_vel_to_wheel_velocities(
                                base_linear * scale, base_angular * scale
                            )
                            base_vels = (left_vel, right_vel)

                    # 액션 텐서 (베이스 4바퀴 + 암 IK + 그리퍼)
                    action = teleop_controller.get_action_tensor(
                        commands,
                        base_velocities=base_vels,
                    ).unsqueeze(0)

                # 관측 추출
                if "policy" in obs:
                    obs_np = obs["policy"].cpu().numpy()[0]
                else:
                    # 개별 관측 결합 (듀얼 암)
                    obs_np = np.concatenate([
                        obs.get("right_arm_joint_pos", torch.zeros(1, 4))[0].cpu().numpy(),
                        obs.get("right_gripper_pos", torch.zeros(1, 1))[0].cpu().numpy(),
                        obs.get("right_ee_pose", torch.zeros(1, 7))[0].cpu().numpy(),
                        obs.get("left_arm_joint_pos", torch.zeros(1, 4))[0].cpu().numpy(),
                        obs.get("left_gripper_pos", torch.zeros(1, 1))[0].cpu().numpy(),
                        obs.get("left_ee_pose", torch.zeros(1, 7))[0].cpu().numpy(),
                        obs.get("object_pos", torch.zeros(1, 3))[0].cpu().numpy(),
                    ])

                action_np = action.cpu().numpy()[0]

                # 이미지 (선택적)
                rgb_image = None
                depth_image = None
                if args_cli.include_images and "vision" in obs:
                    rgb_image = obs["vision"].get("rgb_image", [None])[0]
                    depth_image = obs["vision"].get("depth_image", [None])[0]
                    if rgb_image is not None:
                        rgb_image = rgb_image.cpu().numpy()
                    if depth_image is not None:
                        depth_image = depth_image.cpu().numpy()

                # 데이터 기록
                recorder.add_step(
                    obs=obs_np,
                    action=action_np,
                    done=False,
                    rgb_image=rgb_image,
                    depth_image=depth_image,
                )

                # 환경 스텝
                obs, info = env.step(action)
                current_step += 1

                # ── 에피소드 종료 판단 ──
                episode_ended = False
                success = False
                discard = False

                # VR 모드: 리셋 제스처 → 성공
                if teleop_controller.reset_triggered:
                    episode_ended = True
                    success = True

                # 키보드 모드: L키 → 성공, Enter → 폐기
                if kbd_episode_state is not None:
                    if kbd_episode_state["episode_success"]:
                        episode_ended = True
                        success = True
                        kbd_episode_state["episode_success"] = False
                    elif kbd_episode_state["episode_discard"]:
                        episode_ended = True
                        discard = True
                        kbd_episode_state["episode_discard"] = False

                # 최대 길이 도달 → 타임아웃 (폐기)
                if current_step >= args_cli.max_episode_length:
                    episode_ended = True
                    discard = True
                    print("[INFO] Max episode length reached (timeout).")

                # ── 에피소드 종료 처리 ──
                if episode_ended:
                    if discard:
                        # 폐기 처리
                        recorder.discard_current_episode()
                        discarded_demos += 1
                        print(f"[DISCARD] Episode discarded. Steps: {current_step}")
                    elif current_step < args_cli.min_episode_length:
                        # 최소 길이 미달
                        print(f"[WARNING] Episode too short ({current_step} steps). Discarding.")
                        recorder.discard_current_episode()
                        discarded_demos += 1
                    else:
                        # 성공 저장
                        recorder.end_episode(success=success)
                        completed_demos += 1
                        print(f"[SUCCESS] Demo {completed_demos}/{args_cli.num_demos} completed. Steps: {current_step}")

                    # 다음 에피소드 준비
                    if completed_demos < args_cli.num_demos:
                        obs, info = env.reset()
                        teleop_controller.reset()
                        recorder.start_episode()
                        current_step = 0
                        print(f"\n[Recording] Demo {completed_demos + 1}/{args_cli.num_demos}")
                        print("-" * 40)

                # 진행 상황 출력 (50 스텝마다)
                if current_step % 50 == 0 and current_step > 0:
                    mode_str = f"[{teleop_controller.control_mode_name}]" if args_cli.teleop_device == "keyboard" else ""
                    print(f"  Step {current_step}... {mode_str}")

    except KeyboardInterrupt:
        print("\n[INFO] Recording interrupted by user.")
        # 현재 에피소드 저장 시도
        if recorder.get_current_episode_length() >= args_cli.min_episode_length:
            recorder.end_episode(success=False)
            print("[INFO] Current episode saved (marked as incomplete).")
        else:
            recorder.discard_current_episode()
            print("[INFO] Current episode discarded (too short).")

    finally:
        # 데이터셋 저장
        if completed_demos > 0:
            recorder.save()
            print(f"\n[SUCCESS] Dataset saved: {dataset_file}")
            print(f"  Completed demos: {completed_demos}")
            print(f"  Discarded demos: {discarded_demos}")
        else:
            print("\n[WARNING] No demos completed. Dataset not saved.")

        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
