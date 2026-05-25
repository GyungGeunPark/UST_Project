"""Windows/SteamVR teleoperation entrypoint.

Port of ``ust_260220/scripts/run_teleop.py``:

  * Replaces the Ubuntu TCP bridge + XRoboToolkit Console subprocess with an
    in-process :class:`~ust_ws.ust_hm_glove.teleop.pico_udcap_device.PicoUDCAPDevice`.
  * ``--render_mode`` choices now reflect the Windows path: ``monitor``,
    ``steamvr_desktop`` (Virtual Desktop "Desktop Theater"), ``steamvr_native``
    (experimental Isaac Sim 4.5 path), ``cloudxr`` (future).
  * ``--teleop_device pico_udcap`` is the primary option; ``handtracking`` /
    ``manusvive`` remain available when Isaac Lab's builtin devices are
    registered in ``env_cfg.teleop_devices``.

Environments reused from :mod:`ust_ws.ust_260220` — no new Gym IDs needed.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]   # repo root (contains ust_ws/)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Keep CloudXR-era env var harmless on Windows — does nothing if unused.
os.environ.setdefault("IPC_IGNORE_VERSION", "1")

# Default the Windows OpenXR runtime to SteamVR when the user has not set it.
_DEFAULT_STEAMVR_OPENXR = r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\steamxr_win64.json"


# Keep Pinocchio loaded before Isaac Sim to win the dynamic-link race.
try:
    import pinocchio  # noqa: F401
except Exception:  # pragma: no cover — optional
    pass

# Pre-load h5py BEFORE Isaac Sim boots.  Windows coalesces same-name DLLs
# per process, so whichever hdf5.dll loads first wins.  Isaac Sim ships an
# older HDF5 (1.10/1.12 ABI, 3.2 MB) under isaacsim/...sensors/..., while
# h5py 3.16+ expects the newer HDF5 1.14 ABI bundled in
# site-packages/h5py/hdf5.dll (3.9 MB).  If Isaac Sim loads first, h5py's
# compiled _errors.pyd tries to resolve versioned symbols (H5Efoo_2 etc.)
# against the older DLL and fails with "ImportError: DLL load failed while
# importing _errors: 지정된 프로시저를 찾을 수 없습니다."  Preloading h5py
# forces its own hdf5.dll to win; later isaaclab.utils.datasets imports
# reuse it seamlessly.
try:
    import h5py  # noqa: F401
except Exception:  # pragma: no cover — diagnostic deferred to isaaclab loader
    pass

from isaaclab.app import AppLauncher  # type: ignore


def _parse_args():
    parser = argparse.ArgumentParser(description="G1 Kitchen Sorting — Windows/SteamVR teleop")
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument(
        "--teleop_device",
        type=str,
        default="pico_udcap",
        choices=["handtracking", "manusvive", "pico_udcap"],
    )
    parser.add_argument(
        "--render_mode",
        type=str,
        default="monitor",
        choices=["monitor", "steamvr_desktop", "steamvr_native", "cloudxr"],
        help=(
            "monitor: PC window only | "
            "steamvr_desktop: Virtual Desktop Desktop Theater (1단계 기본) | "
            "steamvr_native: Isaac Sim 4.5 omni.kit.xr.system.steamvr (실험) | "
            "cloudxr: 3단계 예약"
        ),
    )
    parser.add_argument(
        "--use_usd_scene",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the ust_human1.usd preset (as in Ubuntu build).  --no-use_usd_scene for default kitchen.",
    )
    parser.add_argument(
        "--path_b_port",
        type=int,
        default=0,
        help="Enable VMC OSC fallback on this UDP port (0 = disabled).",
    )
    parser.add_argument(
        "--steamvr_openxr_json",
        type=str,
        default=_DEFAULT_STEAMVR_OPENXR,
        help="Path used to set XR_RUNTIME_JSON when --render_mode != monitor.",
    )
    parser.add_argument("--disable_fabric", action="store_true")
    # ── Diagnostic flags ────────────────────────────────────────────────
    parser.add_argument(
        "--debug_ik",
        action="store_true",
        help="Force show_ik_warnings=True on the Pink IK controller so convergence failures are printed.",
    )
    parser.add_argument(
        "--freeze_orientation",
        action="store_true",
        help="Send idle-pose quaternion instead of forearm-tracker rotation "
             "(isolates whether Pink IK fails on orientation vs position).",
    )
    parser.add_argument(
        "--diag",
        type=str,
        default="off",
        choices=["off", "idle", "oscillate"],
        help=(
            "Override action source for pipeline diagnosis. "
            "'idle': send the env's idle_action every step (robot should stay put). "
            "'oscillate': send a time-varying sine-wave wrist target (robot should sway)."
        ),
    )
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def _configure_xr(args: argparse.Namespace) -> None:
    if args.render_mode == "monitor":
        args.xr = False
        return
    args.xr = True
    if args.render_mode in ("steamvr_desktop", "steamvr_native"):
        current = os.environ.get("XR_RUNTIME_JSON", "")
        if "steam" not in current.lower():
            if args.steamvr_openxr_json and os.path.exists(args.steamvr_openxr_json):
                os.environ["XR_RUNTIME_JSON"] = args.steamvr_openxr_json
                print(f"[run_teleop] Set XR_RUNTIME_JSON -> {args.steamvr_openxr_json}")
            else:
                print(
                    "[run_teleop][WARN] XR_RUNTIME_JSON not pointing at SteamVR and the expected "
                    f"manifest '{args.steamvr_openxr_json}' is missing.  Configure SteamVR → "
                    "Settings → Developer → Set SteamVR as OpenXR runtime."
                )


def _pick_env_id(render_mode: str, use_usd: bool) -> str:
    if use_usd:
        return "Isaac-KitchenSorting-G1-InspireFTP-USD-v0"
    if render_mode == "monitor":
        return "Isaac-KitchenSorting-G1-InspireFTP-Monitor-v0"
    return "Isaac-KitchenSorting-G1-InspireFTP-VR-v0"


def main() -> int:
    args = _parse_args()
    _configure_xr(args)

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    # Deferred imports — Isaac Sim must be running first
    import gymnasium as gym
    import torch

    import ust_ws.ust_260220  # noqa: F401 — gym registration side-effects

    from isaaclab.devices.teleop_device_factory import create_teleop_device
    from isaaclab.envs import ManagerBasedRLEnv

    from ust_ws.ust_hm_glove.teleop.pico_udcap_device import (
        PicoUDCAPDevice,
        PicoUDCAPDeviceCfg,
    )

    env_id = _pick_env_id(args.render_mode, args.use_usd_scene)
    env_cfg_cls = gym.spec(env_id).kwargs["env_cfg_entry_point"]
    env_cfg = env_cfg_cls()
    env_cfg.scene.num_envs = args.num_envs

    # Runtime-force Pink IK warnings when --debug_ik is set.  The env cfg
    # ships with show_ik_warnings=False to keep logs quiet; in diagnosis we
    # want to see EXACTLY why the solver returned no-op joint positions.
    if args.debug_ik:
        try:
            env_cfg.actions.pink_ik_cfg.controller.show_ik_warnings = True
            print("[run_teleop] show_ik_warnings=True (Pink IK convergence failures will be logged).")
        except AttributeError as exc:
            print(f"[run_teleop][WARN] Could not enable show_ik_warnings: {exc}")

    env = ManagerBasedRLEnv(cfg=env_cfg)

    print("\n" + "=" * 60)
    print("  G1 Kitchen Sorting — Windows/SteamVR Teleop")
    print(f"  device      : {args.teleop_device}")
    print(f"  render_mode : {args.render_mode}")
    print(f"  scene       : {'USD (ust_human1.usd)' if args.use_usd_scene else 'default kitchen'}")
    print(f"  episode     : {env_cfg.episode_length_s:.0f}s")
    print(f"  physics     : {1/env_cfg.sim.dt:.0f} Hz, render every {env_cfg.sim.render_interval}")
    print("=" * 60 + "\n")

    # ── Teleop device ────────────────────────────────────────────────────
    teleop_device = None
    if args.teleop_device == "pico_udcap":
        legacy_cfg = getattr(env_cfg, "pico_device_cfg", {}) or {}
        cfg_kwargs = {
            "position_scale": float(legacy_cfg.get("position_scale", 1.0)),
            "rotation_scale": float(legacy_cfg.get("rotation_scale", 1.0)),
            "gripper_threshold": float(legacy_cfg.get("gripper_threshold", 0.5)),
            "body_pos_offset": tuple(legacy_cfg.get("body_pos_offset", (0.0, 0.0, 0.0))),
            "controller_pos_offset": tuple(legacy_cfg.get("controller_pos_offset", (0.0, 0.0, 0.0))),
            "debug": bool(legacy_cfg.get("debug", True)),
            "path_b_port": int(args.path_b_port),
            "freeze_orientation": bool(args.freeze_orientation),
        }
        device_cfg = PicoUDCAPDeviceCfg(**cfg_kwargs)
        teleop_device = PicoUDCAPDevice(device_cfg)
        teleop_device.start()
    elif args.teleop_device in getattr(env_cfg, "teleop_devices", object()).__dict__.get("devices", {}):
        teleop_device = create_teleop_device(args.teleop_device, env_cfg.teleop_devices.devices)
        teleop_device.reset()
    else:
        print(f"[run_teleop][WARN] Device '{args.teleop_device}' not registered — idling.")

    # ── Optional XR anchor (same behaviour as Ubuntu run_teleop.py) ──────
    if args.teleop_device == "pico_udcap" and hasattr(env_cfg, "xr"):
        import carb  # noqa: WPS433 — only valid after AppLauncher
        from isaacsim.core.prims import SingleXFormPrim

        xr = env_cfg.xr
        anchor = SingleXFormPrim("/XRAnchor", position=xr.anchor_pos, orientation=xr.anchor_rot)
        settings = carb.settings.get_settings()
        for profile in ("", "/profile/ar", "/profile/vr"):
            prefix = f"/persistent/xr{profile}"
            settings.set_string(f"{prefix}/anchorMode", "custom anchor")
            settings.set_float(f"{prefix}/render/nearPlane", xr.near_plane)
        for profile in ("/profile/ar", "/profile/vr"):
            settings.set_string(f"/xrstage{profile}/customAnchor", anchor.prim_path)
        settings.set_string("/xrstage/customAnchor", anchor.prim_path)
        print(f"[run_teleop] XR anchor @ pos={xr.anchor_pos} rot={xr.anchor_rot}")

    obs, _info = env.reset()
    idle_action = env_cfg.idle_action.unsqueeze(0).repeat(env.num_envs, 1).to(env.device)

    # ── Diagnostic action generators ─────────────────────────────────────
    # When --diag != 'off' the teleop device is ignored; these generators
    # let us prove the env.step / Pink IK pipeline moves the robot on
    # known-good synthetic targets.
    import math as _math
    import time as _time

    def _oscillate_action(t_seconds: float) -> torch.Tensor:
        """Sway both wrists ±10 cm around idle on a 0.5 Hz sine."""
        amp = 0.10
        s = _math.sin(2.0 * _math.pi * 0.5 * t_seconds)
        act = idle_action.clone()
        act[0, 0] += amp * s          # L X
        act[0, 7] -= amp * s          # R X (opposite phase)
        act[0, 2] += 0.5 * amp * _math.cos(2.0 * _math.pi * 0.5 * t_seconds)   # L Z
        act[0, 9] += 0.5 * amp * _math.cos(2.0 * _math.pi * 0.5 * t_seconds)   # R Z
        return act

    diag_t0 = _time.perf_counter()
    if args.diag != "off":
        print(f"[run_teleop] DIAGNOSTIC MODE: --diag={args.diag} — teleop device ignored.")

    step = 0
    try:
        while simulation_app.is_running():
            with torch.inference_mode():
                if args.diag == "idle":
                    action = idle_action
                elif args.diag == "oscillate":
                    action = _oscillate_action(_time.perf_counter() - diag_t0)
                else:
                    teleop_action = teleop_device.advance() if teleop_device is not None else None
                    action = (
                        teleop_action.unsqueeze(0).to(env.device)
                        if teleop_action is not None
                        else idle_action
                    )
                obs, _rewards, terminated, truncated, _info = env.step(action)
                step += 1

                if step % 600 == 0:
                    l_eef = action[0, :3].cpu().numpy()
                    r_eef = action[0, 7:10].cpu().numpy()
                    conn = ""
                    if teleop_device is not None and hasattr(teleop_device, "is_connected"):
                        conn = f" | sampler: {'ok' if teleop_device.is_connected else 'waiting'}"
                    print(
                        f"[Step {step}] L_EEF=({l_eef[0]:.3f},{l_eef[1]:.3f},{l_eef[2]:.3f}) "
                        f"R_EEF=({r_eef[0]:.3f},{r_eef[1]:.3f},{r_eef[2]:.3f}){conn}"
                    )

                if terminated.any() or truncated.any():
                    obs, _info = env.reset()
                    if teleop_device is not None:
                        teleop_device.reset()
    finally:
        if teleop_device is not None and hasattr(teleop_device, "stop"):
            teleop_device.stop()
        env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
