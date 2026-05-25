#!/usr/bin/env python3
"""
G1 주방 물체 분류 - VR 텔레오퍼레이션 데모 녹화.

VR 핸드 트래킹으로 조작한 데모를 HDF5 형식으로 저장합니다.
물리적 카메라는 사용하지 않으며, 시뮬레이션 내부 가상 카메라
이미지가 필요한 경우 --with_sim_camera 옵션을 사용합니다.

사전 준비:
    1. CloudXR Runtime 6.0.1 Docker 실행 중
    2. source ust_ws/ust_260220/setup_cloudxr_env.sh
    3. VR 헤드셋 연결

사용법:
    # 기본: VR 텔레옵 데모 녹화 (카메라 없음, 1시간 에피소드)
    python ust_ws/ust_260220/scripts/record_demos.py \\
        --teleop_device handtracking \\
        --num_demos 50

    # 시뮬레이션 카메라 이미지 포함 녹화 (비전 정책 학습용)
    python ust_ws/ust_260220/scripts/record_demos.py \\
        --teleop_device handtracking \\
        --num_demos 50 \\
        --with_sim_camera \\
        --enable_cameras

    # 빠른 테스트 (10개 데모)
    python ust_ws/ust_260220/scripts/record_demos.py --num_demos 10
"""

import argparse
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))  # import ust_ws.ust_260220

# CloudXR 6.0.1 호환성
os.environ.setdefault("IPC_IGNORE_VERSION", "1")

# Import pinocchio before AppLauncher to force the use of the version installed
# by IsaacLab and not the one installed by Isaac Sim.
# pinocchio is required by the Pink IK controllers.
import pinocchio  # noqa: F401

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="G1 주방 분류 - 데모 녹화")
parser.add_argument("--teleop_device", type=str, default="handtracking",
                    choices=["handtracking", "manusvive", "pico", "pico_no_udcap"],
                    help="텔레오퍼레이션 디바이스: handtracking(Quest), pico(PICO+UDCAP), pico_no_udcap")
parser.add_argument("--num_demos", type=int, default=10, help="수집할 데모 수")
parser.add_argument("--output_dir", type=str, default="ust_ws/ust_260220/data/demos/")
parser.add_argument("--train_split", type=float, default=0.8, help="학습/검증 분할 비율")
parser.add_argument("--with_sim_camera", action="store_true",
                    help="시뮬레이션 내부 가상 카메라 이미지 녹화 (--enable_cameras 필요)")
parser.add_argument("--use_usd_scene", action=argparse.BooleanOptionalAction, default=True,
                    help="커스텀 USD 씬(ust_human1.usd) 사용 (기본값: True). 비활성화: --no-use_usd_scene")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--disable_fabric", action="store_true")
parser.add_argument(
    "--render_mode", type=str, default="monitor",
    choices=["monitor", "pico_connect", "cloudxr"],
    help="렌더링 모드: monitor(PC화면), pico_connect(SteamVR), cloudxr(CloudXR 6.0.1)",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 렌더링 모드에 따른 XR 설정
if args_cli.render_mode == "monitor":
    args_cli.xr = False
    print("[Config] 렌더링 모드: MONITOR (PC 화면, XR 없음)")
elif args_cli.render_mode in ("pico_connect", "cloudxr"):
    args_cli.xr = True
    print(f"[Config] 렌더링 모드: {args_cli.render_mode.upper()}")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- AppLauncher 이후 ---
import torch
import numpy as np
import gymnasium as gym

import ust_ws.ust_260220  # noqa: F401

from isaaclab.devices.teleop_device_factory import create_teleop_device
from isaaclab.envs import ManagerBasedRLEnv
from ust_ws.ust_260220.utils.hdf5_recorder import HDF5DatasetRecorder


def flatten_obs(obs: dict, env_idx: int = 0) -> np.ndarray:
    """관측 딕셔너리를 단일 numpy 배열로 평탄화.

    Args:
        obs: Observation dictionary (possibly nested).
        env_idx: Environment index to extract (default: 0).
    """
    obs_flat = []
    for key in sorted(obs.keys()):
        val = obs[key]
        if isinstance(val, dict):
            nested = flatten_obs(val, env_idx)
            if nested.size > 0:
                obs_flat.append(nested)
        elif isinstance(val, torch.Tensor):
            obs_flat.append(val[env_idx].cpu().numpy().flatten())
        elif isinstance(val, np.ndarray):
            obs_flat.append(val[env_idx].flatten() if val.ndim > 1 else val.flatten())
    if obs_flat:
        return np.concatenate(obs_flat).astype(np.float32)
    return np.array([], dtype=np.float32)


def get_sim_camera_images(env) -> tuple:
    """시뮬레이션 내부 가상 카메라에서 이미지 추출.

    물리적 카메라가 아닌 Isaac Sim RTX 렌더러가 생성한 가상 이미지입니다.

    Returns:
        (rgb_image, depth_image): numpy 배열 또는 (None, None)
    """
    rgb_image = None
    depth_image = None

    try:
        # robot_head_cam에서 RGB 이미지 추출
        head_cam = env.scene["robot_head_cam"]
        if head_cam is not None and hasattr(head_cam, "data"):
            rgb_data = head_cam.data.output.get("rgb")
            if rgb_data is not None:
                # (N, H, W, 4) RGBA -> (H, W, 3) RGB (첫 번째 환경만)
                rgb_image = rgb_data[0, :, :, :3].cpu().numpy().astype(np.uint8)

            depth_data = head_cam.data.output.get("distance_to_camera")
            if depth_data is not None:
                # (N, H, W, 1) -> (H, W)
                depth_image = depth_data[0, :, :, 0].cpu().numpy().astype(np.float32)
    except (KeyError, AttributeError, IndexError):
        # 카메라 센서가 없는 환경 (VR-v0 등)에서는 무시
        pass

    return rgb_image, depth_image


def main():
    """메인 데모 녹화 루프."""
    # 환경 선택: USD 씬 / 카메라 포함 / 모니터 / VR
    if args_cli.use_usd_scene:
        task_id = "Isaac-KitchenSorting-G1-InspireFTP-USD-v0"
        print("[INFO] 커스텀 USD 씬(ust_human1.usd) 사용")
    elif args_cli.with_sim_camera:
        task_id = "Isaac-KitchenSorting-G1-InspireFTP-DataCollect-v0"
    elif args_cli.render_mode == "monitor":
        task_id = "Isaac-KitchenSorting-G1-InspireFTP-Monitor-v0"
    else:
        task_id = "Isaac-KitchenSorting-G1-InspireFTP-VR-v0"

    env_cfg_cls = gym.spec(task_id).kwargs["env_cfg_entry_point"]
    env_cfg = env_cfg_cls()
    env_cfg.scene.num_envs = args_cli.num_envs

    env = ManagerBasedRLEnv(cfg=env_cfg)

    # 레코더 생성
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    hdf5_path = os.path.join(args_cli.output_dir, f"kitchen_sorting_demos_{timestamp}.hdf5")
    recorder = HDF5DatasetRecorder(
        file_path=hdf5_path,
        action_dim=38,  # 14 EEF + 24 핸드 관절
        include_images=args_cli.with_sim_camera,
        image_shape=(720, 1280, 3),
    )

    print("\n" + "=" * 60)
    print("  G1 주방 물체 분류 - 데모 녹화")
    print(f"  목표 데모 수: {args_cli.num_demos}")
    print(f"  VR 디바이스: {args_cli.teleop_device}")
    print(f"  환경: {task_id}")
    print(f"  에피소드 길이: {env_cfg.episode_length_s:.0f}초")
    print(f"  시뮬레이션 카메라: {'사용' if args_cli.with_sim_camera else '미사용'}")
    if args_cli.with_sim_camera:
        print(f"    (물리 카메라 아님 - Isaac Sim 내부 가상 카메라)")
        print(f"    robot_head_cam: 720p RGB + 깊이")
    print(f"  출력: {hdf5_path}")
    print(f"  학습/검증 분할: {args_cli.train_split:.0%}/{1-args_cli.train_split:.0%}")
    print(f"  GPU: RTX PRO 6000")
    print("  ")
    print("  조작 방법:")
    print("    VR 핸드 트래킹으로 로봇 양팔 제어")
    print("    성공 또는 시간 초과 시 에피소드 자동 저장")
    print("=" * 60 + "\n")

    # 텔레옵 디바이스 초기화
    teleop_device = None
    if args_cli.teleop_device in ("pico", "pico_no_udcap"):
        from ust_ws.ust_260220.teleop import PICOFullBodyTeleopDevice
        cfg = env_cfg.pico_device_cfg
        teleop_device = PICOFullBodyTeleopDevice(
            bridge_host=cfg["bridge_host"],
            bridge_port=cfg["bridge_port"],
            use_udcap_fingers=(args_cli.teleop_device == "pico"),
            position_scale=cfg["position_scale"],
            rotation_scale=cfg["rotation_scale"],
            gripper_threshold=cfg["gripper_threshold"],
            body_pos_offset=cfg.get("body_pos_offset", (0.0, 0.0, 0.0)),
            controller_pos_offset=cfg.get("controller_pos_offset", (0.0, 0.0, 0.0)),
            debug=cfg.get("debug", True),
        )
        teleop_device.start()
        print(f"[INFO] PICO 디바이스 초기화 (UDCAP: {args_cli.teleop_device == 'pico'})")
    elif args_cli.teleop_device in env_cfg.teleop_devices.devices:
        teleop_device = create_teleop_device(args_cli.teleop_device, env_cfg.teleop_devices.devices)
        teleop_device.reset()
        print(f"[INFO] 텔레옵 디바이스 '{args_cli.teleop_device}' 초기화 완료.")
    else:
        print(f"[WARN] 디바이스 '{args_cli.teleop_device}'를 찾을 수 없습니다. idle 액션으로 실행합니다.")

    # PICO 디바이스 사용 시 XR 앵커 수동 설정 (OpenXRDevice는 자동, PICO는 수동 필요)
    if args_cli.teleop_device in ("pico", "pico_no_udcap") and hasattr(env_cfg, "xr"):
        import carb
        from isaacsim.core.prims import SingleXFormPrim
        xr = env_cfg.xr
        xr_anchor = SingleXFormPrim("/XRAnchor", position=xr.anchor_pos, orientation=xr.anchor_rot)
        settings = carb.settings.get_settings()
        # 전역 + AR + VR 프로필 모두 custom anchor 모드로 설정
        for profile in ["", "/profile/ar", "/profile/vr"]:
            prefix = f"/persistent/xr{profile}"
            settings.set_string(f"{prefix}/anchorMode", "custom anchor")
            settings.set_float(f"{prefix}/render/nearPlane", xr.near_plane)
        # customAnchor prim path 설정 (xrstage 경로)
        for profile in ["/profile/ar", "/profile/vr"]:
            settings.set_string(f"/xrstage{profile}/customAnchor", xr_anchor.prim_path)
        # 전역 customAnchor 도 설정
        settings.set_string("/xrstage/customAnchor", xr_anchor.prim_path)
        print(f"[INFO] XR 앵커 설정: pos={xr.anchor_pos}, rot={xr.anchor_rot}")

    obs, info = env.reset()
    idle_action = env_cfg.idle_action.unsqueeze(0).repeat(env.num_envs, 1).to(env.device)

    # 첫 번째 에피소드 시작
    recorder.start_episode()

    step = 0
    while simulation_app.is_running() and recorder.num_episodes < args_cli.num_demos:
        with torch.inference_mode():
            if teleop_device is not None:
                teleop_action = teleop_device.advance()
                if teleop_action is not None:
                    action = teleop_action.unsqueeze(0).to(env.device)
                else:
                    action = idle_action
            else:
                action = idle_action

            # 관측 평탄화
            obs_dict = obs if isinstance(obs, dict) else {"obs": obs}
            obs_flat = flatten_obs(obs_dict)
            action_flat = action[0].cpu().numpy()

            # 시뮬레이션 카메라 이미지 (옵션)
            rgb_image = None
            depth_image = None
            if args_cli.with_sim_camera:
                rgb_image, depth_image = get_sim_camera_images(env)

            # 스텝 기록
            recorder.add_step(
                obs=obs_flat,
                action=action_flat,
                done=False,
                reward=0.0,
                rgb_image=rgb_image,
                depth_image=depth_image,
                timestamp=time.time(),
            )

            # 환경 스텝
            obs, rewards, terminated, truncated, info = env.step(action)
            step += 1

            # 주기적 상태 출력
            if step % 200 == 0:
                ep_len = recorder.get_current_episode_length()
                print(f"[Step {step}] 에피소드 {recorder.num_episodes} 녹화 중, "
                      f"스텝: {ep_len}")

            if terminated.any() or truncated.any():
                if args_cli.use_usd_scene:
                    # USD 씬 모드: 물체/빈이 없으므로 분류 판정 스킵
                    success = True
                    sorted_count = 0
                else:
                    from ust_ws.ust_260220.mdp.terminations import all_objects_sorted, count_sorted_objects
                    success = all_objects_sorted(env).any().item()
                    sorted_count = count_sorted_objects(env).mean().item()

                # 에피소드 종료
                recorder.end_episode(success=success)
                print(f"[진행] {recorder.num_episodes}/{args_cli.num_demos} 데모 "
                      f"(분류: {sorted_count:.0f}/8, 성공: {success})")

                # 리셋 및 다음 에피소드 시작
                obs, info = env.reset()
                if teleop_device is not None:
                    teleop_device.reset()

                if recorder.num_episodes < args_cli.num_demos:
                    recorder.start_episode()

    # 전체 에피소드 저장
    recorder.save(train_split=args_cli.train_split)
    env.close()


if __name__ == "__main__":
    main()
