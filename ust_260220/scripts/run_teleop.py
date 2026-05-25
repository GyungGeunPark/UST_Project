#!/usr/bin/env python3
"""
G1 Kitchen Sorting 텔레오퍼레이션 스크립트.

3가지 렌더링 모드 지원:
  monitor      : PC 모니터에서만 렌더링 (XR 없음, 즉시 사용 가능)
  pico_connect : PICO Connect PCVR 스트리밍 (SteamVR OpenXR 사용)
  cloudxr      : CloudXR 6.0.1 WebRTC 렌더링 (기존 방식)

사용법:
    # 방안3: 모니터 뷰 (XRoboToolkit 트래킹 + PC 모니터 렌더링)
    python run_teleop.py --teleop_device pico --render_mode monitor

    # 방안1: PICO Connect (XRoboToolkit 트래킹 + SteamVR VR 렌더링)
    python run_teleop.py --teleop_device pico --render_mode pico_connect

    # CloudXR (기존 방식)
    python run_teleop.py --teleop_device handtracking --render_mode cloudxr
    python run_teleop.py --teleop_device pico --render_mode cloudxr

    # UDCAP 글러브 없이 (XRT 핸드 트래킹만 사용)
    python run_teleop.py --teleop_device pico_no_udcap --render_mode monitor

    # USD 씬은 기본값(--use_usd_scene). 기본 키친 씬으로 돌리려면 --no-use_usd_scene
    python run_teleop.py --teleop_device pico --render_mode monitor --no-use_usd_scene
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

# CloudXR 6.0.1 compatibility - MUST be set before AppLauncher
os.environ.setdefault("IPC_IGNORE_VERSION", "1")

# Import pinocchio before AppLauncher to force the use of the version installed
# by IsaacLab and not the one installed by Isaac Sim.
import pinocchio  # noqa: F401

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="G1 Kitchen Sorting - Teleoperation")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument(
    "--teleop_device", type=str, default="pico",
    choices=["handtracking", "manusvive", "pico", "pico_no_udcap"],
    help="텔레오퍼레이션 디바이스",
)
parser.add_argument(
    "--render_mode", type=str, default="monitor",
    choices=["monitor", "pico_connect", "cloudxr"],
    help="렌더링 모드: monitor(PC화면), pico_connect(SteamVR PCVR), cloudxr(CloudXR 6.0.1)",
)
parser.add_argument("--use_usd_scene", action=argparse.BooleanOptionalAction, default=True,
                    help="ust_human1.usd 커스텀 씬 사용 (기본값: True). 비활성화: --no-use_usd_scene")
parser.add_argument("--disable_fabric", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# ── 렌더링 모드에 따른 XR 설정 ──
if args_cli.render_mode == "monitor":
    # 방안3: XR 비활성화, PC 모니터에서만 렌더링
    args_cli.xr = False
    print("[Config] 렌더링 모드: MONITOR (PC 화면, XR 없음)")

elif args_cli.render_mode == "pico_connect":
    # 방안1: SteamVR OpenXR 사용 (PICO Connect 스트리밍)
    args_cli.xr = True
    # SteamVR OpenXR 런타임 사용 확인
    xr_json = os.environ.get("XR_RUNTIME_JSON", "")
    if "steam" not in xr_json.lower() and xr_json:
        print(f"[WARN] XR_RUNTIME_JSON이 SteamVR가 아닙니다: {xr_json}")
        print("       PICO Connect 모드에서는 SteamVR OpenXR 런타임이 필요합니다.")
        print("       export XR_RUNTIME_JSON=~/.steam/steam/steamapps/common/SteamVR/steamxr_linux64.json")
    print("[Config] 렌더링 모드: PICO CONNECT (SteamVR + PCVR 스트리밍)")

elif args_cli.render_mode == "cloudxr":
    # CloudXR: 기존 방식
    args_cli.xr = True
    print("[Config] 렌더링 모드: CLOUDXR (CloudXR 6.0.1 WebRTC)")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- After app launch ---
import torch
import gymnasium as gym

import ust_ws.ust_260220  # noqa: F401

from isaaclab.devices.teleop_device_factory import create_teleop_device
from isaaclab.envs import ManagerBasedRLEnv


def main():
    """Main teleoperation loop."""
    # ── 환경 설정 선택 (USD 씬 우선) ──
    if args_cli.use_usd_scene:
        env_id = "Isaac-KitchenSorting-G1-InspireFTP-USD-v0"
        print("[INFO] 커스텀 USD 씬(ust_human1.usd) 사용")
    elif args_cli.render_mode == "monitor":
        env_id = "Isaac-KitchenSorting-G1-InspireFTP-Monitor-v0"
        print("[INFO] 기본 키친 씬(Monitor 모드) 사용")
    else:
        env_id = "Isaac-KitchenSorting-G1-InspireFTP-VR-v0"
        print("[INFO] 기본 키친 씬(VR 모드) 사용")

    env_cfg_cls = gym.spec(env_id).kwargs["env_cfg_entry_point"]
    env_cfg = env_cfg_cls()
    env_cfg.scene.num_envs = args_cli.num_envs

    env = ManagerBasedRLEnv(cfg=env_cfg)

    render_mode_kr = {
        "monitor": "모니터 뷰 (XR 없음)",
        "pico_connect": "PICO Connect (SteamVR PCVR)",
        "cloudxr": "CloudXR 6.0.1",
    }

    print("\n" + "=" * 60)
    print("  G1 Kitchen Sorting - Teleoperation")
    print(f"  디바이스: {args_cli.teleop_device}")
    print(f"  렌더링: {render_mode_kr.get(args_cli.render_mode, args_cli.render_mode)}")
    print(f"  에피소드: {env_cfg.episode_length_s:.0f}s ({env_cfg.episode_length_s/60:.0f} min)")
    print(f"  물리: {1/env_cfg.sim.dt:.0f} Hz, 렌더: every {env_cfg.sim.render_interval} steps")
    print(f"  XR: {'활성' if args_cli.render_mode != 'monitor' else '비활성'}")
    print(f"  씬: {'USD (ust_human1.usd)' if args_cli.use_usd_scene else '기본 키친 씬'}")
    print("  ")
    if not args_cli.use_usd_scene:
        print("  물체 → 빈:")
        print("    mug, plate, bowl      → BinKitchen (left)")
        print("    can, bottle, apple    → BinFood (center)")
        print("    sponge, teddy         → BinMisc (right)")
    print("=" * 60 + "\n")

    # ── 텔레오퍼레이션 디바이스 초기화 ──
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
        udcap_str = "UDCAP 활성" if args_cli.teleop_device == "pico" else "UDCAP 비활성"
        print(f"[INFO] PICO 디바이스 초기화 ({udcap_str})")

    elif args_cli.teleop_device in env_cfg.teleop_devices.devices:
        teleop_device = create_teleop_device(args_cli.teleop_device, env_cfg.teleop_devices.devices)
        teleop_device.reset()
        print(f"[INFO] Teleop device '{args_cli.teleop_device}' initialized.")
    else:
        print(f"[WARN] Device '{args_cli.teleop_device}' not found. Running with idle actions.")

    # ── XR 앵커 설정 (PICO 디바이스 사용 시: monitor 모드 포함 항상 적용) ──
    # PICO는 OpenXRDevice가 아니므로 자동 앵커 동기화가 없어 수동 설정 필요.
    # monitor 모드에서도 XRoboToolkit 좌표를 로봇 pelvis에 정렬하기 위해 동일하게 적용한다.
    if args_cli.teleop_device in ("pico", "pico_no_udcap") and hasattr(env_cfg, "xr"):
        import carb
        from isaacsim.core.prims import SingleXFormPrim
        xr = env_cfg.xr
        xr_anchor = SingleXFormPrim("/XRAnchor", position=xr.anchor_pos, orientation=xr.anchor_rot)
        settings = carb.settings.get_settings()
        for profile in ["", "/profile/ar", "/profile/vr"]:
            prefix = f"/persistent/xr{profile}"
            settings.set_string(f"{prefix}/anchorMode", "custom anchor")
            settings.set_float(f"{prefix}/render/nearPlane", xr.near_plane)
        for profile in ["/profile/ar", "/profile/vr"]:
            settings.set_string(f"/xrstage{profile}/customAnchor", xr_anchor.prim_path)
        settings.set_string("/xrstage/customAnchor", xr_anchor.prim_path)
        print(f"[INFO] XR 앵커 설정: pos={xr.anchor_pos}, rot={xr.anchor_rot}")

    obs, info = env.reset()
    idle_action = env_cfg.idle_action.unsqueeze(0).repeat(env.num_envs, 1).to(env.device)

    step = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            if teleop_device is not None:
                teleop_action = teleop_device.advance()
                if teleop_action is not None:
                    action = teleop_action.unsqueeze(0).to(env.device)
                else:
                    action = idle_action
            else:
                action = idle_action

            obs, rewards, terminated, truncated, info = env.step(action)
            step += 1

            if step % 600 == 0:
                left_eef = action[0, :3].cpu().numpy()
                right_eef = action[0, 7:10].cpu().numpy()
                conn_str = ""
                if hasattr(teleop_device, "is_connected"):
                    conn_str = f" | Bridge: {'연결됨' if teleop_device.is_connected else '대기중'}"
                if args_cli.use_usd_scene:
                    # USD 씬에는 분류 빈/물체가 없어 sorted count 미산출
                    print(f"[Step {step}] L-EEF: ({left_eef[0]:.3f},{left_eef[1]:.3f},{left_eef[2]:.3f}) "
                          f"R-EEF: ({right_eef[0]:.3f},{right_eef[1]:.3f},{right_eef[2]:.3f})"
                          f"{conn_str}")
                else:
                    from ust_ws.ust_260220.mdp.terminations import count_sorted_objects
                    sorted_count = count_sorted_objects(env)
                    print(f"[Step {step}] Sorted: {sorted_count.mean().item():.0f}/8 | "
                          f"L-EEF: ({left_eef[0]:.3f},{left_eef[1]:.3f},{left_eef[2]:.3f}) "
                          f"R-EEF: ({right_eef[0]:.3f},{right_eef[1]:.3f},{right_eef[2]:.3f})"
                          f"{conn_str}")

            if terminated.any() or truncated.any():
                if args_cli.use_usd_scene:
                    print(f"[Step {step}] Episode done (USD scene). Resetting...")
                else:
                    from ust_ws.ust_260220.mdp.terminations import count_sorted_objects, all_objects_sorted
                    sorted_count = count_sorted_objects(env)
                    success = all_objects_sorted(env).any().item()
                    print(f"[Step {step}] Episode done. Sorted: {sorted_count.mean().item():.0f}/8, "
                          f"Success: {success}. Resetting...")
                obs, info = env.reset()
                if teleop_device is not None:
                    teleop_device.reset()

    env.close()


if __name__ == "__main__":
    main()
