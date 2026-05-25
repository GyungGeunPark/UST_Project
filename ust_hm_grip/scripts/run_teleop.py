"""GR1T2 + 2-finger gripper Windows/SteamVR teleop entrypoint.

Successor to ``ust_hm_glove/scripts/run_teleop.py`` for the post-
UDCAP migration (research/36 option B).  Differences:

* Teleop device is :class:`GR1T2GripperDevice` (no UDCAP, no VMC).
* Action layout is 16-D (7 + 7 + 2) instead of 36-D.
* All UDCAP / finger-curl tuning flags are dropped — the gripper is
  binary so there's nothing to scale.
* New flag ``--gripper_close_threshold`` / ``--gripper_open_threshold``
  for trigger hysteresis.

The osqp shim, Pink IK config, env-cfg loading, and process-priority
helpers all carry over verbatim from ust_hm_glove (was ust_fourier_260421 pre-9.36)'s run_teleop.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]   # repo root (contains ust_ws/)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.environ.setdefault("IPC_IGNORE_VERSION", "1")

# PowerShell's host buffers stdout aggressively when running ``python -X utf8``
# without ``-u``; the URDF conversion + Pink IK setup phase emit no output for
# 30-90s on a cold cache, which looked like a hung process and prompted users
# to Ctrl+C.  Force line-buffered stdout/stderr so progress prints surface
# in real time regardless of how python was invoked.
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except (AttributeError, ValueError):
    pass

_DEFAULT_STEAMVR_OPENXR = (
    r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\steamxr_win64.json"
)


# DLL-load-order guards
try:
    import pinocchio  # noqa: F401
except Exception:  # pragma: no cover
    pass

try:
    import h5py  # noqa: F401
except Exception:  # pragma: no cover
    pass


# Apply osqp 0.6 ↔ qpsolvers 4.x compat shim before pink import
try:
    from ust_ws.ust_hm_grip.teleop import _osqp_compat  # noqa: F401
    _osqp_compat.apply()
except Exception as _exc:
    print(f"[run_teleop][WARN] osqp compat shim not applied: {_exc}")


_ENV_BASE = "Isaac-KitchenSorting-GR1T2-Gripper-v0"
_ENV_WAIST = "Isaac-KitchenSorting-GR1T2-Gripper-WaistEnabled-v0"
_ENV_MONITOR = "Isaac-KitchenSorting-GR1T2-Gripper-Monitor-v0"
_ENV_VR = "Isaac-KitchenSorting-GR1T2-Gripper-VR-v0"
_ENV_VISION = "Isaac-KitchenSorting-GR1T2-Gripper-Vision-v0"
_ENV_DATA = "Isaac-KitchenSorting-GR1T2-Gripper-DataCollect-v0"
_ENV_ROBOT_ONLY = "Isaac-KitchenSorting-GR1T2-Gripper-RobotOnly-v0"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="GR1T2 + 2-finger gripper Kitchen Sorting — Windows/SteamVR teleop"
    )
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument(
        "--teleop_device",
        type=str,
        default="pico_gripper",
        choices=["pico_gripper"],
        help="Only the PICO gripper device is supported in ust_hm_grip.",
    )
    parser.add_argument(
        "--render_mode",
        type=str,
        default="monitor",
        choices=["monitor", "steamvr_desktop", "steamvr_native", "cloudxr"],
    )
    parser.add_argument(
        "--env_variant",
        type=str,
        default="auto",
        choices=[
            "auto", "base", "waist_enabled", "monitor", "vr", "vision",
            "data_collect", "robot_only",
        ],
    )
    parser.add_argument(
        "--forearm_offset",
        type=float,
        default=None,
        help=(
            "Forearm-to-wrist offset along the tracker's local +X axis "
            "(metres).  Default None = use env_cfg.pico_device_cfg "
            "(0.28 m, calibrated for elbow-mounted PICO motion trackers)."
        ),
    )
    parser.add_argument(
        "--prefer_controller",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help="Default True — wrist EEF target is taken from the PICO "
             "controller pose.  Forearm tracker is the fallback.",
    )
    parser.add_argument(
        "--ignore_arms",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=False,
        help="When True, lock arms in idle T-pose regardless of controller "
             "/ forearm tracker state (useful for gripper-only debug).",
    )
    parser.add_argument(
        "--gripper_close_threshold",
        type=float,
        default=0.6,
        help="Trigger / grip value above which the gripper closes (rising "
             "edge).  Default 0.6.",
    )
    parser.add_argument(
        "--gripper_open_threshold",
        type=float,
        default=0.4,
        help="Trigger / grip value below which the gripper opens (falling "
             "edge).  Default 0.4.  Must be < close_threshold.",
    )
    parser.add_argument(
        "--use_grip_as_close",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help="DEPRECATED (kept for backward compat) — use "
             "--gripper_signal_source instead.  When True (default) and "
             "--gripper_signal_source is unset, behaviour is unchanged.",
    )
    parser.add_argument(
        "--gripper_signal_source",
        type=str,
        default="grip",
        choices=["grip", "trigger", "both"],
        help="9.28 (user request 2026-05-XX): which controller input drives "
             "the parallel-gripper close/open.  'grip' (default) = grip "
             "Pull only; 'trigger' = trigger Pull only (legacy 9.27); "
             "'both' = logical OR of grip and trigger.",
    )
    parser.add_argument(
        "--input_backend",
        type=str,
        default="openvr",
        choices=["openvr", "xrobotoolkit"],
        help=(
            "Source of controller analog/pose data.\n"
            "  openvr        — legacy SteamVR Action API path.  Requires a\n"
            "                  valid Personal Binding for the app key\n"
            "                  'ust.teleop.gr1t2_gripper'.  Blocked in our\n"
            "                  PICO Connect 10.6.6 + SteamVR 2.15.6 setup\n"
            "                  (memory.md §10).\n"
            "  xrobotoolkit  — PICO official XRoboToolkit gRPC path.\n"
            "                  Requires XRoboToolkit-PC-Service running and\n"
            "                  XRoboToolkit-Unity-Client APK active on the\n"
            "                  headset with Direction=Send.  Bypasses SteamVR.\n"
            "                  See research/47 §1-§4."
        ),
    )
    parser.add_argument(
        "--xrt_enable_body",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help="With --input_backend=xrobotoolkit, populate snapshot trackers "
             "from the PICO 24-joint body mocap stream.  Default True (13th "
             "session): when the user has paired PICO Motion Trackers and "
             "Body=ON in the Unity Client APK, the 24-joint estimate "
             "supplies the waist + left_forearm + right_forearm trackers "
             "the retargeter uses.  Set --xrt_enable_body False to revert "
             "to controller-only EEF targeting (no torso anchoring).",
    )
    parser.add_argument(
        "--xrt_enable_hand",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=False,
        help="With --input_backend=xrobotoolkit, populate snapshot hands "
             "from OpenXR 26-joint hand tracking.  Unused by current 16-D "
             "retargeter; reserved for dexterous-hand upgrades.",
    )
    parser.add_argument(
        "--full_body",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help="13th-bis session — Phase D full-body teleop side channel.  "
             "When True (default with --input_backend=xrobotoolkit), the "
             "main loop writes head_yaw/pitch/roll + waist_yaw/pitch/roll "
             "joint position targets each frame from the snapshot's HMD "
             "pose + waist tracker quaternion (ZYX Euler).  Bypasses the "
             "action manager — the 16-D Pink IK action layout stays "
             "unchanged.  Set --full_body False to revert to wrist-only "
             "teleop (idle head + idle torso).",
    )
    parser.add_argument(
        "--vr_runtime",
        type=str,
        default="auto",
        choices=["auto", "pico_connect", "virtual_desktop", "steamvr_native"],
        help=(
            "9.37 -- which streaming app routes the PICO HMD/controllers/"
            "trackers into SteamVR.  Selects the tracker_binding template "
            "and prints the recommended SteamVR Add-On layout.\n"
            "  pico_connect    : PICO Connect (prism driver) -- 'PICO "
            "Connect -> SteamVR -> PC -> Isaac Lab' pipeline.  Uses the "
            "PICO Motion Tracker template "
            "`config/tracker_binding_pico_connect.json` when present.\n"
            "  virtual_desktop : Virtual Desktop Streamer + 'Forward "
            "tracking to SteamVR' (legacy default; uses VD AI body "
            "tracking segments).\n"
            "  steamvr_native  : Headset has its own SteamVR driver "
            "(rare for PICO 4 Ultra).\n"
            "  auto            : leave SteamVR add-on selection up to "
            "the user; uses the default tracker_binding.json."
        ),
    )
    parser.add_argument(
        "--steamvr_openxr_json",
        type=str,
        default=_DEFAULT_STEAMVR_OPENXR,
        help="Path used to set XR_RUNTIME_JSON when --render_mode != monitor.",
    )
    parser.add_argument("--disable_fabric", action="store_true")
    parser.add_argument(
        "--render_interval",
        type=int,
        default=None,
        help="Override sim.render_interval (default 1; bump to 2 for VR).",
    )
    parser.add_argument(
        "--process_priority",
        type=str,
        default="high",
        choices=["normal", "high", "realtime"],
        help="Windows process priority class.  Default 'high' to reduce "
             "P99 jitter against background apps.",
    )
    parser.add_argument(
        "--diag",
        type=str,
        default="off",
        choices=["off", "idle", "oscillate"],
        help="When != 'off', bypass the teleop device and drive the env "
             "with synthetic actions (diagnostic mode).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=0,
        help="If > 0, run only this many env.step calls then exit (smoke "
             "test).  Default 0 = run until episode timeout / Ctrl-C.",
    )
    parser.add_argument(
        "--max_seconds",
        type=float,
        default=0.0,
        help="If > 0, run for at most this many wall-clock seconds then exit "
             "cleanly.  Convenient alternative to --steps for interactive "
             "demos (e.g. --max_seconds 300 = 5 min).  When both --steps "
             "and --max_seconds are set, whichever comes first wins.  "
             "Default 0 = run until --steps or Ctrl-C.",
    )
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--enable_cameras", action="store_true")
    parser.add_argument(
        "--livestream",
        type=int,
        default=-1,
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()
    if args.gripper_open_threshold >= args.gripper_close_threshold:
        parser.error(
            "--gripper_open_threshold must be strictly less than "
            "--gripper_close_threshold (otherwise hysteresis collapses)."
        )
    return args


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


def _pick_env_id(render_mode: str, variant: str) -> str:
    table = {
        "base": _ENV_BASE,
        "waist_enabled": _ENV_WAIST,
        "monitor": _ENV_MONITOR,
        "vr": _ENV_VR,
        "vision": _ENV_VISION,
        "data_collect": _ENV_DATA,
        "robot_only": _ENV_ROBOT_ONLY,
    }
    if variant in table:
        return table[variant]
    if render_mode == "monitor":
        return _ENV_MONITOR
    if render_mode in ("steamvr_desktop", "steamvr_native", "cloudxr"):
        return _ENV_VR
    return _ENV_BASE


def _set_process_priority(pref: str) -> None:
    if sys.platform != "win32":
        return
    pref = (pref or "normal").lower()
    if pref == "normal":
        return
    try:
        import psutil  # type: ignore[import-not-found]
    except ImportError:
        print(
            "[run_teleop][--process_priority] psutil not installed; "
            "leaving priority at NORMAL."
        )
        return
    try:
        proc = psutil.Process(os.getpid())
        target = (
            psutil.REALTIME_PRIORITY_CLASS
            if pref == "realtime"
            else psutil.HIGH_PRIORITY_CLASS
        )
        proc.nice(target)
        print(f"[run_teleop] Windows process priority -> {pref.upper()}")
    except (psutil.AccessDenied, OSError) as exc:
        print(
            f"[run_teleop][--process_priority] {pref!r} not granted "
            f"({type(exc).__name__}: {exc})."
        )


def main() -> int:
    args = _parse_args()
    _configure_xr(args)
    _set_process_priority(args.process_priority)

    # Pre-startup notice so users don't panic at Isaac Sim's faulthandler
    # noise (typically `Windows fatal exception: code 0xc0000139` from
    # `isaacsim.sensors.rtx` failing to find the `_generic_model_output`
    # DLL).  These traces are non-fatal — Isaac Sim continues to function.
    print(
        "[run_teleop] ⏳ Starting Isaac Sim (~10s).  "
        "You may see 'Windows fatal exception: 0xc0000139' / "
        "'isaacsim.sensors.rtx' / 'omni.ext._impl' warnings during startup — "
        "they are NON-FATAL Isaac Sim sensor module messages and do NOT "
        "indicate a crash.  Watch for the '✅ READY' banner after env_cfg loads.",
        flush=True,
    )

    from isaaclab.app import AppLauncher  # type: ignore
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    # Deferred imports — Isaac Sim must be running first.
    import gymnasium as gym  # noqa: F401
    import numpy as np
    import torch

    from isaaclab.envs import ManagerBasedRLEnv

    # isaaclab.envs is now importable; apply the hand_joint_dim==0 slicing
    # patch (no-op if already applied at env_cfg import time).
    from ust_ws.ust_hm_grip.teleop import _pink_hand_dim_zero_patch
    _pink_hand_dim_zero_patch.apply()

    # ── Env cfg resolution ────────────────────────────────────────────
    env_variant = args.env_variant
    if env_variant == "auto":
        env_variant = (
            "monitor" if args.render_mode == "monitor"
            else "vr" if args.render_mode in ("steamvr_desktop", "steamvr_native", "cloudxr")
            else "base"
        )

    print(f"[run_teleop] Loading env_cfg for variant='{env_variant}'...", flush=True)
    try:
        from ust_ws.ust_hm_grip.kitchen_sorting_gr1t2_gripper_env_cfg import (
            KitchenSortingGR1T2GripperDataCollectEnvCfg,
            KitchenSortingGR1T2GripperEnvCfg,
            KitchenSortingGR1T2GripperMonitorEnvCfg,
            KitchenSortingGR1T2GripperRobotOnlyEnvCfg,
            KitchenSortingGR1T2GripperVREnvCfg,
            KitchenSortingGR1T2GripperVisionEnvCfg,
            KitchenSortingGR1T2GripperWaistEnvCfg,
        )
    except BaseException:
        import traceback as _tb
        print("\n" + "=" * 72)
        print("[run_teleop] FATAL: kitchen_sorting_gr1t2_gripper_env_cfg failed to import.")
        print("=" * 72)
        _tb.print_exc()
        return 2

    variant_map = {
        "base":          (KitchenSortingGR1T2GripperEnvCfg,            _ENV_BASE),
        "waist_enabled": (KitchenSortingGR1T2GripperWaistEnvCfg,       _ENV_WAIST),
        "monitor":       (KitchenSortingGR1T2GripperMonitorEnvCfg,     _ENV_MONITOR),
        "vr":            (KitchenSortingGR1T2GripperVREnvCfg,          _ENV_VR),
        "vision":        (KitchenSortingGR1T2GripperVisionEnvCfg,      _ENV_VISION),
        "data_collect":  (KitchenSortingGR1T2GripperDataCollectEnvCfg, _ENV_DATA),
        "robot_only":    (KitchenSortingGR1T2GripperRobotOnlyEnvCfg,   _ENV_ROBOT_ONLY),
    }
    env_cfg_cls, env_id = variant_map[env_variant]
    print(f"[run_teleop] env_cfg = {env_cfg_cls.__name__}  →  {env_id}", flush=True)

    env_cfg = env_cfg_cls()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.sim.device = args.device

    if args.render_interval is not None:
        env_cfg.sim.render_interval = int(args.render_interval)
        print(
            f"[run_teleop] render_interval override → {args.render_interval}"
        )

    # Apply CLI overrides to the device cfg.
    dcfg = env_cfg.pico_device_cfg
    dcfg["prefer_controller_for_eef"] = bool(args.prefer_controller)
    dcfg["disable_arm_tracking"] = bool(args.ignore_arms)
    dcfg["gripper_close_threshold"] = float(args.gripper_close_threshold)
    dcfg["gripper_open_threshold"] = float(args.gripper_open_threshold)
    dcfg["use_grip_as_close"] = bool(args.use_grip_as_close)
    dcfg["gripper_signal_source"] = str(args.gripper_signal_source)
    if args.forearm_offset is not None:
        dcfg["forearm_wrist_offset"] = (float(args.forearm_offset), 0.0, 0.0)

    # ─── XRoboToolkit backend pre-flight check (research/47, 12th session) ───
    if args.input_backend == "xrobotoolkit":
        print(
            "[run_teleop] --input_backend xrobotoolkit -- using PICO "
            "XRoboToolkit gRPC pipeline (bypassing SteamVR Action API)."
        )

        # 12th session: PICO 4 Ultra OS allows only ONE streaming APK at a time.
        # `--render_mode steamvr_native` requires PICO Connect (or another
        # SteamVR companion) on the headset to render HMD stereo — but PICO
        # Connect's SteamVR session kills the XRoboToolkit Unity Client APK.
        # Therefore this combination is fundamentally broken on the user's
        # PICO 4 Ultra; warn loudly so the user can pick a different mode.
        if args.render_mode in ("steamvr_native", "steamvr_desktop"):
            print(
                "[run_teleop][⚠️ HMD-CONFLICT] --render_mode "
                f"{args.render_mode} + --input_backend xrobotoolkit is "
                "INCOMPATIBLE on PICO 4 Ultra.\n"
                "  Reason: the PICO HMD allows only one streaming APK at a "
                "time.  PICO Connect's SteamVR session terminates the\n"
                "  XRoboToolkit Unity Client APK the moment it starts, so "
                "the xrt sampler will see frozen / zero poses.\n"
                "  Fix: switch to '--render_mode monitor' (PC monitor "
                "rendering, recommended for current Phase A) or pick a\n"
                "       different input backend.  See "
                "XROBOTOOLKIT_EXECUTION_GUIDE §10 (Phase B options) for HMD "
                "stereo alternatives."
            )

        try:
            import psutil  # type: ignore[import-not-found]
            svc_running = any(
                "roboticsserviceprocess" in (proc.info["name"] or "").lower()
                for proc in psutil.process_iter(["name"])
            )
            if not svc_running:
                print(
                    "[run_teleop][WARN] RoboticsServiceProcess.exe is NOT "
                    "running.  Start it before continuing:\n"
                    "    & 'C:\\develop\\IsaacLab\\ust_ws\\XRoboToolkit-PC-Service.win\\runService.bat'\n"
                    "  Otherwise xrt.init() will fail and teleop will not start."
                )
            pico_connect = any(
                (proc.info["name"] or "").lower() == "pico connect.exe"
                for proc in psutil.process_iter(["name"])
            )
            if pico_connect:
                # 12th session: not just "may compete", PICO Connect actively
                # kills the XRoboToolkit Unity Client APK on the headset.
                print(
                    "[run_teleop][⚠️ APK-CONFLICT] 'Pico Connect.exe' is "
                    "running on the PC.\n"
                    "  PICO 4 Ultra OS allows only ONE streaming APK at a "
                    "time: PICO Connect's SteamVR session has terminated\n"
                    "  the XRoboToolkit Unity Client APK on the headset, so "
                    "the controller pose / grip / trigger stream is dead.\n"
                    "  Fix:\n"
                    "    Get-Process 'Pico Connect' | Stop-Process -Force\n"
                    "  Then re-pair the Unity Client APK on the headset "
                    "(Apps -> XRoboToolkit -> Connect -> Direction=Send)."
                )
        except ImportError:
            print("[run_teleop][WARN] psutil not installed; can't verify "
                  "RoboticsServiceProcess / Pico Connect state.")

    # 9.37 -- PICO Connect -> SteamVR -> Isaac Lab pipeline.  When the
    # user opts into the new pipeline, swap the tracker_binding template
    # to the PICO-specific file (PMT_* serials with auto-mapping support
    # via enumerate_trackers.py).  The legacy tracker_binding.json keeps
    # the Virtual Desktop body-segment names ("hips" / "*_arm_lower" /
    # "*_lower_leg") that are NOT emitted by PICO Connect's prism driver,
    # so without this swap the SteamVRSampler would silently treat every
    # forearm tracker as missing -- the wrist EEF would then fall back to
    # the controller pose only, with no graceful degradation when the
    # controller drops out of view.
    if args.vr_runtime == "pico_connect":
        from pathlib import Path as _Path
        pico_template = _Path(
            "./ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json"
        )
        if pico_template.exists():
            dcfg["tracker_binding_json"] = str(pico_template)
            print(
                f"[run_teleop] --vr_runtime pico_connect -> using "
                f"tracker_binding template {pico_template}"
            )
        else:
            print(
                "[run_teleop][WARN] --vr_runtime pico_connect requested "
                "but config/tracker_binding_pico_connect.json is missing. "
                "Falling back to the default tracker_binding.json (which "
                "is calibrated for Virtual Desktop body segments).  Run "
                "`python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers "
                "--out ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json` "
                "while PICO Connect is streaming to generate the template."
            )
        # Print SteamVR Add-On guidance once at startup so the user can
        # double-check Manage Add-Ons matches the expected pipeline.
        print(
            "[run_teleop] PICO Connect pipeline -- recommended SteamVR "
            "Add-Ons:\n"
            "    prism                            ON   (PICO Connect)\n"
            "    Virtual Desktop Streamer (Quest) OFF  (avoid driver conflict)\n"
            "    udcap                            OFF  (gloves not used in grip)\n"
            "  Run `python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pico_connect` "
            "to verify the layered pipeline before live teleop."
        )
    elif args.vr_runtime == "virtual_desktop":
        print(
            "[run_teleop] --vr_runtime virtual_desktop -- recommended "
            "SteamVR Add-Ons:\n"
            "    Virtual Desktop Streamer (Quest) ON\n"
            "    prism                            OFF\n"
            "  Default tracker_binding.json (VD body segments) is correct."
        )
    elif args.vr_runtime == "steamvr_native":
        print(
            "[run_teleop] --vr_runtime steamvr_native -- assuming the "
            "headset registers its own SteamVR driver and exposes both "
            "controllers + (optionally) trackers natively.  Default "
            "tracker_binding.json applies; override per-rig if needed."
        )

    # ── 13th session, part 9 — Phase D ↔ Pink IK ownership split ─────
    # When ``--full_body=True`` the Phase D side-channel directly writes
    # ``waist_yaw/pitch/roll_joint`` position targets via the articulation
    # API every frame.  Without intervention, Pink IK *also* controls
    # those joints (the ``robot_only`` env variant inherits from
    # ``KitchenSortingGR1T2GripperWaistEnvCfg`` which adds the 3 waist
    # joints to ``pink_controlled_joint_names``), so Pink IK's
    # ``apply_actions()`` call later in ``env.step`` overwrites the
    # Phase D targets with its own IK redundancy solution.
    #
    # Symptom: even with neutral upright calibration, the robot's waist
    # bends forward (Pink IK's IK solver picks waist bending as the
    # "cheapest" redundancy DOF when the wrist target is off the
    # nominal idle position) and the user's waist-tracker motion never
    # visibly drives the robot.  Head works fine because no head joint
    # is in ``pink_controlled_joint_names``.
    #
    # Fix: drop the waist joints from Pink IK's controlled set (and
    # from the ``NullSpacePostureTask.controlled_joints`` to keep the
    # null-space mask consistent) when Phase D will own them anyway.
    if args.full_body and args.input_backend == "xrobotoolkit":
        _WAIST_JOINTS = {"waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"}
        try:
            _pink_cfg = env_cfg.actions.pink_ik_cfg
            _before_pj = list(_pink_cfg.pink_controlled_joint_names)
            _pink_cfg.pink_controlled_joint_names = [
                j for j in _before_pj if j not in _WAIST_JOINTS
            ]
            _removed_pj = [j for j in _before_pj if j in _WAIST_JOINTS]
            # Mirror the change on every NullSpacePostureTask so the
            # null-space joint mask doesn't reference joints Pink IK is
            # no longer solving for.
            from isaaclab.controllers.pink_ik.null_space_posture_task import (
                NullSpacePostureTask as _NSPT,
            )
            _removed_ns: list[str] = []
            for _task in _pink_cfg.controller.variable_input_tasks:
                if isinstance(_task, _NSPT):
                    _before_ns = list(_task.controlled_joints)
                    _task.controlled_joints = [
                        j for j in _before_ns if j not in _WAIST_JOINTS
                    ]
                    _removed_ns.extend(j for j in _before_ns if j in _WAIST_JOINTS)
            print(
                "[run_teleop][phase_d] Phase D owns the waist joints — "
                "removed from Pink IK control to stop Pink IK from "
                "overwriting the direct articulation targets.\n"
                f"  pink_controlled_joint_names: {len(_before_pj)} → "
                f"{len(_pink_cfg.pink_controlled_joint_names)} "
                f"(removed {_removed_pj})\n"
                f"  NullSpacePostureTask.controlled_joints: removed "
                f"{_removed_ns}"
            )
        except Exception as _exc:  # noqa: BLE001
            print(
                f"[run_teleop][phase_d] WARNING: could not patch Pink IK "
                f"controlled joints ({type(_exc).__name__}: {_exc}).  "
                f"Phase D waist targets may be overwritten by Pink IK."
            )

    # ── Construct env + device ────────────────────────────────────────
    env = ManagerBasedRLEnv(cfg=env_cfg)
    obs, _ = env.reset()

    from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_device import (
        GR1T2GripperDevice,
        GR1T2GripperDeviceCfg,
    )

    device_cfg = GR1T2GripperDeviceCfg(
        tracker_binding_json=dcfg["tracker_binding_json"],
        actions_json=dcfg["actions_json"],
        vrmanifest_json=dcfg["vrmanifest_json"],
        app_key=dcfg["app_key"],
        forearm_wrist_offset=tuple(dcfg["forearm_wrist_offset"]),
        position_scale=dcfg.get("position_scale", 1.0),
        rotation_scale=dcfg.get("rotation_scale", 1.0),
        body_pos_offset=tuple(dcfg.get("body_pos_offset", (0.0, 0.0, 0.0))),
        controller_pos_offset=tuple(dcfg.get("controller_pos_offset", (0.0, 0.0, 0.0))),
        use_waist_origin=dcfg.get("use_waist_origin", True),
        subtract_waist_z=dcfg.get("subtract_waist_z", False),
        freeze_orientation=dcfg.get("freeze_orientation", False),
        right_wrist_z180=dcfg.get("right_wrist_z180", True),
        prefer_controller_for_eef=dcfg["prefer_controller_for_eef"],
        controller_to_wrist_offset=tuple(dcfg.get("controller_to_wrist_offset", (0.0, 0.0, -0.05))),
        gripper_close_threshold=dcfg["gripper_close_threshold"],
        gripper_open_threshold=dcfg["gripper_open_threshold"],
        use_grip_as_close=dcfg["use_grip_as_close"],
        gripper_signal_source=dcfg["gripper_signal_source"],
        disable_arm_tracking=dcfg["disable_arm_tracking"],
        # 13th session, part 12-13 — split pos/quat source.
        wrist_pos_source=dcfg.get("wrist_pos_source", "wrist_tracker"),
        wrist_quat_source=dcfg.get("wrist_quat_source", "wrist_tracker"),
        debug=True,
        # ─── research/47: XRoboToolkit backend ────────────────────────
        input_backend=args.input_backend,
        xrt_enable_body=args.xrt_enable_body,
        xrt_enable_hand=args.xrt_enable_hand,
    )
    device = GR1T2GripperDevice(device_cfg)
    device.start()

    # ── Main loop ─────────────────────────────────────────────────────
    import time as _time

    n_steps = int(args.steps) if args.steps > 0 else None
    max_seconds = float(args.max_seconds) if args.max_seconds > 0 else None
    loop_start_time = _time.perf_counter()
    step = 0
    idle_action = env_cfg.idle_action.unsqueeze(0).to(env.device)
    exit_reason = "unknown"  # populated when the loop exits

    # 13th-bis part 8: cache the latest action vector emitted by the
    # device.advance() so the TRACK diagnostic can show what the robot
    # is actually receiving WITHOUT a double-advance (which would
    # corrupt the retargeter's gripper hysteresis state).
    _last_action_cached = None

    # 13th-bis session, part 8 — diagnostic: at startup, print the
    # actual gripper TCP world + base_link pose so the user can verify
    # the env_cfg DEFAULT_*_POS values are consistent with the robot's
    # real T-pose geometry.  Helps catch L/R asymmetric idle poses that
    # cause one wrist to look twisted while the other is fine.
    try:
        _tcp_diag_robot = env.scene["robot"]
        _bn = list(_tcp_diag_robot.data.body_names)
        _pw = _tcp_diag_robot.data.body_pos_w[0].detach().cpu().numpy()
        _qw = _tcp_diag_robot.data.body_quat_w[0].detach().cpu().numpy()
        if "base_link" in _bn:
            from ust_ws.ust_hm_grip.teleop import coord_transforms as _ct_diag
            _bl_idx = _bn.index("base_link")
            _bl_pos = _pw[_bl_idx]
            _bl_quat = _qw[_bl_idx]
            _bl_quat_inv = _ct_diag.quat_conjugate(_bl_quat)
            print(
                "[run_teleop][tcp_diag] base_link world pos="
                f"({_bl_pos[0]:+.3f},{_bl_pos[1]:+.3f},{_bl_pos[2]:+.3f}) "
                f"quat=({_bl_quat[0]:+.3f},{_bl_quat[1]:+.3f},{_bl_quat[2]:+.3f},{_bl_quat[3]:+.3f})"
            )
            for _tcp_name in ("left_gripper_tcp_link", "right_gripper_tcp_link"):
                if _tcp_name not in _bn:
                    print(f"[run_teleop][tcp_diag] {_tcp_name}: NOT FOUND in body_names")
                    continue
                _i = _bn.index(_tcp_name)
                _tp = _pw[_i]
                _tq = _qw[_i]
                _delta_w = _tp - _bl_pos
                _tp_bl = _ct_diag.quat_rotate_vec(_bl_quat_inv, _delta_w)
                _tq_bl = _ct_diag.quat_multiply(_bl_quat_inv, _tq)
                print(
                    f"[run_teleop][tcp_diag] {_tcp_name}: "
                    f"world=({_tp[0]:+.3f},{_tp[1]:+.3f},{_tp[2]:+.3f})  "
                    f"in base_link: pos=({_tp_bl[0]:+.4f},{_tp_bl[1]:+.4f},{_tp_bl[2]:+.4f}) "
                    f"quat=({_tq_bl[0]:+.4f},{_tq_bl[1]:+.4f},{_tq_bl[2]:+.4f},{_tq_bl[3]:+.4f})"
                )
    except Exception as _exc:  # noqa: BLE001
        print(f"[run_teleop][tcp_diag] failed: {type(_exc).__name__}: {_exc}")

    # 13th-bis session — Phase D full-body side channel.  Resolve head +
    # waist joint indices once after env.reset has populated joint_names.
    # On each frame, extract HMD / waist tracker quaternions from the
    # snapshot, convert to ZYX Euler (yaw, pitch, roll), and write them
    # as joint position targets via the articulation API.  This bypasses
    # the action manager so the 16-D Pink IK action layout stays valid
    # for the retargeter + recorder.  Failures are logged ONCE per cause
    # then silenced so they don't spam the log every frame.
    _fb = {
        "enabled": bool(args.full_body),
        "resolved": False,
        "head_ids": None,     # list[int] | None
        "waist_ids": None,
        "warned": set(),      # set of one-shot keys
        "first_target": False,
        # 13th-bis session — startup calibration.  At startup the user's
        # actual head / pelvis orientation is whatever it happens to be
        # (seated, slightly slouched, looking down etc.).  Without
        # calibration the raw quaternion goes straight to the robot, so
        # the robot reproduces the user's slouch as a permanent posture
        # (e.g. waist_pitch ≈ +0.5 rad → robot bent forward → wrists
        # naturally too low → user has to raise controllers very high).
        # Fix: capture the FIRST live HMD / waist quaternion as the
        # "zero" reference, then every subsequent target uses
        # ``delta = raw * inverse(zero)`` so the robot stays in T-pose
        # at startup regardless of user posture, and only deltas are
        # transmitted.
        "zero_hmd_quat": None,
        "zero_waist_quat": None,
        # 13th-bis session — controller A-button recalibration.
        # Rising-edge detector + cooldown so a single press triggers
        # exactly one re-zero (head + waist + wrist).  Default
        # _RECAL_COOLDOWN_SEC=0.5 prevents accidental double-trigger
        # from button-bounce while still allowing the user to recal
        # twice in quick succession if they want.
        "prev_a_button": False,
        "last_recal_time": -1.0,
    }
    _RECAL_COOLDOWN_SEC = 0.5

    def _phase_d_resolve_joints() -> bool:
        if _fb["resolved"]:
            return _fb["head_ids"] is not None or _fb["waist_ids"] is not None
        _fb["resolved"] = True
        try:
            robot = env.scene["robot"]
            joint_names = list(robot.data.joint_names)
        except Exception as exc:  # noqa: BLE001
            print(f"[run_teleop][phase_d] could not list robot joints: "
                  f"{type(exc).__name__}: {exc}; disabling.")
            return False
        head = []
        for jn in ("head_yaw_joint", "head_pitch_joint", "head_roll_joint"):
            if jn in joint_names:
                head.append(joint_names.index(jn))
            else:
                if "head_missing" not in _fb["warned"]:
                    print(f"[run_teleop][phase_d] head joint {jn!r} not in "
                          f"articulation — head-follow disabled (this env "
                          f"variant doesn't expose head DoF).")
                    _fb["warned"].add("head_missing")
                head = None
                break
        waist = []
        for jn in ("waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"):
            if jn in joint_names:
                waist.append(joint_names.index(jn))
            else:
                if "waist_missing" not in _fb["warned"]:
                    print(f"[run_teleop][phase_d] waist joint {jn!r} not in "
                          f"articulation — waist-follow disabled.")
                    _fb["warned"].add("waist_missing")
                waist = None
                break
        _fb["head_ids"] = head
        _fb["waist_ids"] = waist
        print(
            f"[run_teleop][phase_d] joint resolution: "
            f"head_ids={head} waist_ids={waist}"
        )
        return head is not None or waist is not None

    # Clamp head/waist joint targets to safe ranges.  GR1T2 head joints
    # have ~±60° pitch/roll and ~±90° yaw nominal; waist has tighter
    # limits.  Use conservative ranges so user inputs don't slam into
    # joint stops.
    _HEAD_LIMITS_RAD = {
        "yaw":   (-1.5, 1.5),    # ~±86°
        "pitch": (-1.0, 1.0),    # ~±57°
        "roll":  (-0.7, 0.7),    # ~±40°
    }
    # 13th session, part 10 — waist pitch limit bumped from ±0.6 to ±1.0
    # because user reports forward-bend tracking is weak.  Pelvis tracker
    # quat captures user's PELVIS tilt, which is small relative to the
    # spine bend the user perceives (most forward-bend motion is in the
    # lumbar / thoracic spine, not the pelvis).  Wider clamp lets the
    # scaling factor below amplify the small pelvis delta into a more
    # visible robot waist bend.
    _WAIST_LIMITS_RAD = {
        "yaw":   (-1.2, 1.2),    # ~±69°
        "pitch": (-1.0, 1.0),    # ~±57° (was ±34°)
        "roll":  (-0.7, 0.7),    # ~±40° (was ±29°)
    }
    # 13th session, part 10 — amplification factors for the weakly-driven
    # axes.  Pelvis tracker pitch captures only the pelvis rotation, but
    # users perceive forward-bend including spinal contribution.  Multiply
    # the delta by these scales so a small pelvis tilt produces a larger
    # robot waist target.  Clamped afterwards by ``_WAIST_LIMITS_RAD``.
    # Yaw stays 1.0 because part 9 confirmed yaw tracks perfectly already.
    _WAIST_SCALE = {"yaw": 1.0, "pitch": 2.0, "roll": 1.5}
    # 13th session, part 10 — head scale.  HMD pitch/roll come from
    # headset orientation directly; full 1.0 is correct.  Kept for
    # symmetry / future tuning hook.
    _HEAD_SCALE = {"yaw": 1.0, "pitch": 1.0, "roll": 1.0}

    def _phase_d_check_recalibration(snapshot):
        """Rising-edge A button (right controller) → reset all zeros so
        the next ``_phase_d_apply`` re-captures the calibration.

        Returns True if a re-calibration just fired (so the apply path
        can log it conveniently).  No-op if A is not pressed or is
        within the cooldown window.
        """
        ctrls = (snapshot or {}).get("controllers") or {}
        r = ctrls.get("right") or {}
        btn = r.get("buttons") or {}
        a_now = bool(btn.get("a", False))
        a_prev = bool(_fb.get("prev_a_button", False))
        _fb["prev_a_button"] = a_now
        if a_now and not a_prev:
            now = _time.perf_counter()
            if (now - _fb.get("last_recal_time", -1.0)) > _RECAL_COOLDOWN_SEC:
                _fb["last_recal_time"] = now
                _fb["zero_hmd_quat"] = None
                _fb["zero_waist_quat"] = None
                _fb["first_target"] = False  # so we re-print first head target log
                try:
                    device._retargeter.cfg.controller_pose_zero = None  # type: ignore[attr-defined]
                except Exception:  # noqa: BLE001
                    pass
                # 13th session, part 12 — also reset body wrist tracker
                # zeros so the next frame captures both quat (controller)
                # AND pos (body skeleton wrist) at the user's CURRENT
                # pose.  Otherwise position drift accumulates across
                # recalibration cycles.
                try:
                    device._retargeter.cfg.wrist_pose_zero = None  # type: ignore[attr-defined]
                except Exception:  # noqa: BLE001
                    pass

                # 13th-bis session — INSTANT snap to default joint
                # pose.  Without this the robot smoothly transitions
                # from its current (often weird) pose to the new idle
                # over the PD time constant (~100-200 ms), during
                # which the user sees the robot drift through
                # awkward intermediate poses.  Hard-writing
                # default_joint_pos forces every joint to snap to its
                # rest position in a single sim step — the next
                # _phase_d_apply then captures the user's current
                # pose as the new zero and the robot's pose smoothly
                # tracks user motion from idle.
                try:
                    robot = env.scene["robot"]
                    default_pos = robot.data.default_joint_pos.clone()
                    default_vel = torch.zeros_like(default_pos)
                    # Apply to ALL envs (typically just 1 in teleop).
                    robot.write_joint_state_to_sim(default_pos, default_vel)
                except Exception as exc:  # noqa: BLE001
                    if "recal_snap" not in _fb["warned"]:
                        print(
                            f"[run_teleop][phase_d] instant-snap to default "
                            f"pose failed: {type(exc).__name__}: {exc}  — "
                            f"falling back to PD transition (slow)."
                        )
                        _fb["warned"].add("recal_snap")

                print(
                    "[run_teleop][phase_d] 🔄 A button pressed — "
                    "re-calibration triggered.  Robot snapped to "
                    "default idle pose; next frame will re-capture "
                    "HMD / waist / wrist zeros from your CURRENT pose."
                )
                return True
        return False

    def _phase_d_apply(snapshot):
        if not _fb["enabled"]:
            return
        if not _fb["resolved"]:
            if not _phase_d_resolve_joints():
                return
        if not (_fb["head_ids"] or _fb["waist_ids"]):
            return
        try:
            from ust_ws.ust_hm_grip.teleop import coord_transforms as _ct
        except Exception:  # noqa: BLE001
            return

        # 13th-bis session — check A-button recalibration trigger
        # BEFORE any zero capture so the new pose becomes the new zero.
        _phase_d_check_recalibration(snapshot)

        robot = env.scene["robot"]

        # ── Startup calibration capture (one-shot per channel) ──
        # Record the FIRST live (non-zero) HMD / waist quaternion as
        # the user's startup posture.  All subsequent targets are
        # computed as deltas from this zero so the robot stays in idle
        # T-pose at startup regardless of how the user happens to be
        # seated / slouched.
        hmd = (snapshot or {}).get("hmd")
        if hmd is not None and _fb["zero_hmd_quat"] is None:
            zero_q = np.asarray(hmd["quat"], dtype=np.float64).copy()
            if not np.allclose(zero_q, 0.0, atol=1e-6):
                _fb["zero_hmd_quat"] = zero_q
                print(
                    f"[run_teleop][phase_d] HMD calibrated to zero quat = "
                    f"({zero_q[0]:+.3f},{zero_q[1]:+.3f},{zero_q[2]:+.3f},{zero_q[3]:+.3f}) "
                    f"— head deltas are now relative to this orientation."
                )
        trackers_snap = (snapshot or {}).get("trackers") or {}
        waist_snap = trackers_snap.get("waist")
        if waist_snap is not None and _fb["zero_waist_quat"] is None:
            zero_q = np.asarray(waist_snap["quat"], dtype=np.float64).copy()
            if not np.allclose(zero_q, 0.0, atol=1e-6):
                _fb["zero_waist_quat"] = zero_q
                print(
                    f"[run_teleop][phase_d] waist calibrated to zero quat = "
                    f"({zero_q[0]:+.3f},{zero_q[1]:+.3f},{zero_q[2]:+.3f},{zero_q[3]:+.3f}) "
                    f"— torso deltas are now relative to this orientation."
                )

        # ── Wrist calibration: record startup controller poses ──
        # When the retargeter has not yet been calibrated AND both
        # controllers are live (non-zero), capture their current poses
        # as the wrist "zero".  After this, the retargeter switches to
        # delta-from-zero + idle wrist mode so robot stays in T-pose
        # at startup and wrist tracking is 1:1 from there on.
        try:
            _rt_cfg = device._retargeter.cfg  # type: ignore[attr-defined]
            ctrls_snap = (snapshot or {}).get("controllers") or {}
            l = ctrls_snap.get("left")
            r = ctrls_snap.get("right")
            if (_rt_cfg.controller_pose_zero is None
                    and l is not None and r is not None):
                l_pose = l.get("pose")
                r_pose = r.get("pose")
                if (l_pose is not None and r_pose is not None
                        and not np.allclose(l_pose.get("pos", [0, 0, 0]), 0.0, atol=1e-6)
                        and not np.allclose(r_pose.get("pos", [0, 0, 0]), 0.0, atol=1e-6)):
                    _rt_cfg.controller_pose_zero = {
                        "left":  {"pos": np.asarray(l_pose["pos"], dtype=np.float64).copy(),
                                   "quat": np.asarray(l_pose["quat"], dtype=np.float64).copy()},
                        "right": {"pos": np.asarray(r_pose["pos"], dtype=np.float64).copy(),
                                   "quat": np.asarray(r_pose["quat"], dtype=np.float64).copy()},
                    }
                    lp = _rt_cfg.controller_pose_zero["left"]["pos"]
                    rp = _rt_cfg.controller_pose_zero["right"]["pos"]
                    print(
                        f"[run_teleop][phase_d] wrist calibrated — "
                        f"L_zero=({lp[0]:+.3f},{lp[1]:+.3f},{lp[2]:+.3f}) "
                        f"R_zero=({rp[0]:+.3f},{rp[1]:+.3f},{rp[2]:+.3f}); "
                        f"robot now stays in idle T-pose at startup, "
                        f"wrist tracks user controller deltas 1:1."
                    )
        except Exception as exc:  # noqa: BLE001 — diagnostic only
            if "wrist_cal" not in _fb["warned"]:
                print(f"[run_teleop][phase_d] wrist calibration capture failed: "
                      f"{type(exc).__name__}: {exc}")
                _fb["warned"].add("wrist_cal")

        # ── 13th session, part 12 — Body wrist tracker calibration ──
        # When ``cfg.wrist_pos_source == "wrist_tracker"`` the retargeter
        # uses the body skeleton wrist (SMPL idx 20/21) for position
        # delta.  Capture each side's first live wrist tracker pos as
        # its zero so the robot starts in idle T-pose regardless of
        # where the user's hand happens to be at startup.
        try:
            _rt_cfg2 = device._retargeter.cfg  # type: ignore[attr-defined]
            need_wrist_zero = (
                getattr(_rt_cfg2, "wrist_pos_source", "controller") == "wrist_tracker"
                or getattr(_rt_cfg2, "wrist_quat_source", "controller") == "wrist_tracker"
            )
            if need_wrist_zero:
                trks = trackers_snap or {}
                lwt = trks.get("left_wrist")
                rwt = trks.get("right_wrist")
                if _rt_cfg2.wrist_pose_zero is None:
                    _rt_cfg2.wrist_pose_zero = {}
                zero_dict = _rt_cfg2.wrist_pose_zero
                for side, wt_pose in (("left", lwt), ("right", rwt)):
                    if (
                        wt_pose is not None
                        and zero_dict.get(side) is None
                        and not np.allclose(
                            wt_pose.get("pos", [0, 0, 0]), 0.0, atol=1e-6
                        )
                    ):
                        # 13th session, part 13 — capture BOTH pos and
                        # quat zeros so the SMPL wrist orientation can
                        # be deltaed against the user's startup
                        # wrist orientation (decoupled from controller).
                        zero_dict[side] = {
                            "pos": np.asarray(
                                wt_pose["pos"], dtype=np.float64
                            ).copy(),
                            "quat": np.asarray(
                                wt_pose.get("quat", [1.0, 0.0, 0.0, 0.0]),
                                dtype=np.float64,
                            ).copy(),
                        }
                        zp = zero_dict[side]["pos"]
                        zq = zero_dict[side]["quat"]
                        print(
                            f"[run_teleop][phase_d] body-wrist {side} "
                            f"calibrated — pos_zero=({zp[0]:+.3f},{zp[1]:+.3f},{zp[2]:+.3f}) "
                            f"quat_zero=({zq[0]:+.3f},{zq[1]:+.3f},{zq[2]:+.3f},{zq[3]:+.3f}); "
                            f"wrist EEF pos+quat now driven by SMPL "
                            f"{side}_wrist delta.  Controller pose unused."
                        )
        except Exception as exc:  # noqa: BLE001
            if "body_wrist_cal" not in _fb["warned"]:
                print(
                    f"[run_teleop][phase_d] body-wrist calibration failed: "
                    f"{type(exc).__name__}: {exc}"
                )
                _fb["warned"].add("body_wrist_cal")

        # ── head: HMD pose delta from zero ──
        if (_fb["head_ids"] is not None and hmd is not None
                and _fb["zero_hmd_quat"] is not None):
            try:
                raw_q = np.asarray(hmd["quat"], dtype=np.float64)
                # delta = raw * inverse(zero); inverse of unit quat = conjugate
                zero_inv = _ct.quat_conjugate(_fb["zero_hmd_quat"])
                delta_q = _ct.quat_multiply(raw_q, zero_inv)
                yaw, pitch, roll = _ct.quat_wxyz_to_euler_zyx(delta_q)
                # Scale (part 10) then clamp.
                yaw   *= _HEAD_SCALE["yaw"]
                pitch *= _HEAD_SCALE["pitch"]
                roll  *= _HEAD_SCALE["roll"]
                ly, lp, lr = _HEAD_LIMITS_RAD["yaw"], _HEAD_LIMITS_RAD["pitch"], _HEAD_LIMITS_RAD["roll"]
                yaw = max(ly[0], min(ly[1], yaw))
                pitch = max(lp[0], min(lp[1], pitch))
                roll = max(lr[0], min(lr[1], roll))
                target = torch.zeros(env.num_envs, 3, device=env.device)
                target[:, 0] = float(yaw)
                target[:, 1] = float(pitch)
                target[:, 2] = float(roll)
                robot.set_joint_position_target(target, joint_ids=_fb["head_ids"])
                if not _fb["first_target"]:
                    print(
                        f"[run_teleop][phase_d] first head target applied "
                        f"(delta from zero): yaw={yaw:+.3f} pitch={pitch:+.3f} "
                        f"roll={roll:+.3f}"
                    )
                    _fb["first_target"] = True
            except Exception as exc:  # noqa: BLE001
                if "head_apply" not in _fb["warned"]:
                    print(f"[run_teleop][phase_d] head apply failed: "
                          f"{type(exc).__name__}: {exc}")
                    _fb["warned"].add("head_apply")

        # ── waist: pelvis tracker pose delta from zero ──
        if (_fb["waist_ids"] is not None and waist_snap is not None
                and _fb["zero_waist_quat"] is not None):
            try:
                raw_q = np.asarray(waist_snap["quat"], dtype=np.float64)
                zero_inv = _ct.quat_conjugate(_fb["zero_waist_quat"])
                delta_q = _ct.quat_multiply(raw_q, zero_inv)
                yaw, pitch, roll = _ct.quat_wxyz_to_euler_zyx(delta_q)
                # Scale (part 10) then clamp — pelvis pitch/roll captures
                # only the pelvis-frame rotation, but the user perceives
                # the *spinal* bend.  Amplifying pelvis delta makes the
                # robot waist track the user's intent more closely.
                yaw   *= _WAIST_SCALE["yaw"]
                pitch *= _WAIST_SCALE["pitch"]
                roll  *= _WAIST_SCALE["roll"]
                ly, lp, lr = _WAIST_LIMITS_RAD["yaw"], _WAIST_LIMITS_RAD["pitch"], _WAIST_LIMITS_RAD["roll"]
                yaw = max(ly[0], min(ly[1], yaw))
                pitch = max(lp[0], min(lp[1], pitch))
                roll = max(lr[0], min(lr[1], roll))
                target = torch.zeros(env.num_envs, 3, device=env.device)
                target[:, 0] = float(yaw)
                target[:, 1] = float(pitch)
                target[:, 2] = float(roll)
                robot.set_joint_position_target(target, joint_ids=_fb["waist_ids"])
            except Exception as exc:  # noqa: BLE001
                if "waist_apply" not in _fb["warned"]:
                    print(f"[run_teleop][phase_d] waist apply failed: "
                          f"{type(exc).__name__}: {exc}")
                    _fb["warned"].add("waist_apply")

    # OS-level yield interval.  On --process_priority=high + decimation=1 +
    # render_interval=1 + sim.dt=1/120 the main loop runs ~120 Hz and pegs
    # one core, which starves Windows' input-thread delivery enough that the
    # Isaac Sim window becomes un-clickable / un-draggable -- the omni.kit
    # message pump inside env.step()'s _app.update() runs, but mouse events
    # never reach it because the input thread can't compete for CPU.  Yield
    # to the OS scheduler once per frame and additionally bump an extra
    # ``simulation_app.update()`` every ``UI_PUMP_INTERVAL`` frames so menus,
    # window dragging, and clicks stay responsive even at HIGH priority.
    UI_PUMP_INTERVAL = 16

    # XR sampler health check + live tracker dump (only meaningful with
    # ``--input_backend xrobotoolkit``).  Wall-clock based (not step-based)
    # because env.step + Pink IK throttle the actual loop rate well below
    # the sampler's 120 Hz — step-based intervals would fire much less often
    # than expected.  Wall-clock fires every XR_HEALTH_CHECK_SECONDS.
    XR_HEALTH_CHECK_SECONDS = 3.0   # 13th-bis session — every 3 s
    XR_STALE_SNAPSHOT_THRESHOLD = 1.0  # seconds since last sampler tick

    # Last wall-clock time we printed the XR-HEALTH / TRACK diagnostic.
    _last_xr_check_time = 0.0
    _last_xr_health_warn_step = -1

    # Compact loop budget summary so the user can tell what will end the run.
    if n_steps and max_seconds:
        budget = f"steps_limit={n_steps}, max_seconds={max_seconds:.0f}"
    elif n_steps:
        budget = f"steps_limit={n_steps}"
    elif max_seconds:
        budget = f"max_seconds={max_seconds:.0f}"
    else:
        budget = "unbounded (Ctrl-C to stop)"

    print(
        "=" * 72 + "\n"
        f"[run_teleop] ✅ READY — Isaac Sim + env_cfg loaded, teleop pipeline alive.\n"
        f"           render_mode={args.render_mode}, xr={args.xr}, input_backend={args.input_backend}\n"
        f"           budget: {budget}\n"
        f"           (Any 'Windows fatal exception: 0xc0000139' / 'isaacsim.sensors.rtx'\n"
        f"            errors above are non-fatal Isaac Sim sensor module warnings —\n"
        f"            the main teleop loop is unaffected.)\n"
        + "=" * 72,
        flush=True,
    )

    try:
        while simulation_app.is_running():
            # Get teleop action
            if args.diag == "idle":
                action = idle_action
            elif args.diag == "oscillate":
                # Sin oscillation on left wrist X for visual diagnosis
                import math as _m
                t = step / 120.0
                base = idle_action.clone()
                base[0, 0] = -0.20 + 0.10 * _m.sin(t * 2.0)
                action = base
            else:
                a = device.advance()
                if a is None:
                    action = idle_action
                else:
                    action = a.unsqueeze(0).to(env.device)
                    _last_action_cached = a.detach().cpu().numpy()

            # 13th-bis session — Phase D full-body side channel.  Write
            # head_yaw/pitch/roll + waist_yaw/pitch/roll joint position
            # targets from the snapshot's HMD + waist tracker quaternions.
            # MUST run BEFORE env.step so PhysX picks up the new targets
            # this frame.  Action layout (Pink IK + grippers) untouched.
            if args.input_backend == "xrobotoolkit":
                try:
                    _phase_d_apply(device.snapshot() if device is not None else None)
                except Exception as exc:  # noqa: BLE001 — diagnostic only
                    if "phase_d_outer" not in _fb["warned"]:
                        print(f"[run_teleop][phase_d] outer apply failed: "
                              f"{type(exc).__name__}: {exc}")
                        _fb["warned"].add("phase_d_outer")

            obs, _rew, term, trunc, _info = env.step(action)

            if torch.any(term) or torch.any(trunc):
                obs, _ = env.reset()
                device.reset()

            step += 1

            # OS scheduler yield -- lets the OS deliver pending mouse/keyboard
            # events to the Isaac Sim window without measurably delaying us
            # (sleep(0) only yields if a runnable thread is waiting).
            _time.sleep(0)

            # Extra UI message pump every UI_PUMP_INTERVAL frames so menu
            # interactions stay snappy at HIGH process priority.  Cost is one
            # _app.update() call every ~133 ms at 120 Hz, negligible vs the
            # per-frame physics + IK cost.
            if step % UI_PUMP_INTERVAL == 0:
                simulation_app.update()

            if n_steps and step >= n_steps:
                exit_reason = f"reached --steps={n_steps}"
                print(f"[run_teleop] {exit_reason}; exiting.", flush=True)
                break

            if max_seconds is not None:
                elapsed = _time.perf_counter() - loop_start_time
                if elapsed >= max_seconds:
                    exit_reason = f"reached --max_seconds={max_seconds:.0f}s (actual={elapsed:.1f}s, steps={step})"
                    print(f"[run_teleop] {exit_reason}; exiting.", flush=True)
                    break

            # Periodic health check for the XRoboToolkit backend.  Warn (don't
            # exit) if the sampler hasn't ticked in a while — typically caused
            # by the Unity Client APK on the headset losing its connection.
            now_wall = _time.perf_counter()
            if (
                args.input_backend == "xrobotoolkit"
                and step > 0
                and (now_wall - _last_xr_check_time) >= XR_HEALTH_CHECK_SECONDS
            ):
                _last_xr_check_time = now_wall
                try:
                    snap = device.snapshot() or {}
                    sample_ts = snap.get("timestamp") or 0.0
                    age = _time.perf_counter() - sample_ts if sample_ts > 0 else float("inf")
                    if age > XR_STALE_SNAPSHOT_THRESHOLD:
                        _last_xr_health_warn_step = step
                        print(
                            f"[run_teleop][XR-HEALTH] sampler snapshot is "
                            f"{age:.1f}s old at step {step} -- the Unity Client "
                            f"APK may have stopped streaming.  Check the headset "
                            f"and toggle Direction=Send if needed.",
                            flush=True,
                        )

                    # 13th-bis session — every 5 s, dump which tracker channels
                    # have live (non-zero) data and what wrist target the
                    # retargeter chose.  Lets the user see exactly which body
                    # parts are contributing vs. silently ignored.
                    trackers = snap.get("trackers") or {}
                    ctrls = snap.get("controllers") or {}
                    hmd = snap.get("hmd")
                    def _fmt_p(p):
                        if p is None:
                            return "(none)"
                        pp = p.get("pos") if isinstance(p, dict) else p
                        if pp is None:
                            return "(none)"
                        return f"({pp[0]:+.2f},{pp[1]:+.2f},{pp[2]:+.2f})"
                    hmd_str = _fmt_p(hmd)
                    waist_str = _fmt_p(trackers.get("waist"))
                    lfa_str = _fmt_p(trackers.get("left_forearm"))
                    rfa_str = _fmt_p(trackers.get("right_forearm"))
                    # 13th session, part 11 — wrist trackers (SMPL idx 20/21).
                    lw_str = _fmt_p(trackers.get("left_wrist"))
                    rw_str = _fmt_p(trackers.get("right_wrist"))
                    lc = ctrls.get("left", {}) or {}
                    rc = ctrls.get("right", {}) or {}
                    lc_str = _fmt_p(lc.get("pose"))
                    rc_str = _fmt_p(rc.get("pose"))
                    # Controller quat → euler delta from zero (rotation
                    # diagnostic — proves controller orientation is
                    # flowing into the retargeter as a delta).
                    lc_eul_str = rc_eul_str = "(no zero)"
                    try:
                        from ust_ws.ust_hm_grip.teleop import (
                            coord_transforms as _ct_rot,
                        )
                        _rt_cfg = device._retargeter.cfg  # type: ignore[attr-defined]
                        cal = _rt_cfg.controller_pose_zero
                        if cal is not None:
                            for side, ctrl_dict, slot in (
                                ("left",  lc, "lc_eul_str"),
                                ("right", rc, "rc_eul_str"),
                            ):
                                z = (cal.get(side) or {})
                                z_q = z.get("quat")
                                raw_q = (ctrl_dict.get("pose") or {}).get("quat")
                                if z_q is None or raw_q is None:
                                    continue
                                d_q = _ct_rot.quat_multiply(
                                    np.asarray(raw_q, dtype=np.float64),
                                    _ct_rot.quat_conjugate(np.asarray(z_q, dtype=np.float64)),
                                )
                                y, p, r = _ct_rot.quat_wxyz_to_euler_zyx(d_q)
                                fmt = f"yaw={y:+.2f} pitch={p:+.2f} roll={r:+.2f}"
                                if slot == "lc_eul_str":
                                    lc_eul_str = fmt
                                else:
                                    rc_eul_str = fmt
                    except Exception:  # noqa: BLE001
                        pass
                    # 13th-bis part 8: also include the Phase D
                    # delta euler (head/waist) and the wrist action
                    # vector (action[0:7], action[7:14]) so user can
                    # see EXACTLY what targets the robot is receiving.
                    head_delta_str = "(no zero)"
                    waist_delta_str = "(no zero)"
                    try:
                        if _fb["zero_hmd_quat"] is not None and hmd is not None:
                            raw_q = np.asarray(hmd["quat"], dtype=np.float64)
                            from ust_ws.ust_hm_grip.teleop import coord_transforms as _ct_track
                            delta_q = _ct_track.quat_multiply(
                                raw_q, _ct_track.quat_conjugate(_fb["zero_hmd_quat"])
                            )
                            y, p, r = _ct_track.quat_wxyz_to_euler_zyx(delta_q)
                            head_delta_str = f"yaw={y:+.2f} pitch={p:+.2f} roll={r:+.2f}"
                        if _fb["zero_waist_quat"] is not None and trackers.get("waist") is not None:
                            raw_q = np.asarray(trackers["waist"]["quat"], dtype=np.float64)
                            from ust_ws.ust_hm_grip.teleop import coord_transforms as _ct_track
                            delta_q = _ct_track.quat_multiply(
                                raw_q, _ct_track.quat_conjugate(_fb["zero_waist_quat"])
                            )
                            y, p, r = _ct_track.quat_wxyz_to_euler_zyx(delta_q)
                            waist_delta_str = f"yaw={y:+.2f} pitch={p:+.2f} roll={r:+.2f}"
                    except Exception:  # noqa: BLE001
                        pass

                    # Read the cached latest action (set by the main
                    # loop right after device.advance()).  This avoids
                    # a double-advance that would corrupt the
                    # retargeter's gripper hysteresis state.
                    _a = _last_action_cached
                    if _a is not None and len(_a) >= 16:
                        lpos_str = f"({_a[0]:+.2f},{_a[1]:+.2f},{_a[2]:+.2f})"
                        rpos_str = f"({_a[7]:+.2f},{_a[8]:+.2f},{_a[9]:+.2f})"
                        lq_str = f"({_a[3]:+.2f},{_a[4]:+.2f},{_a[5]:+.2f},{_a[6]:+.2f})"
                        rq_str = f"({_a[10]:+.2f},{_a[11]:+.2f},{_a[12]:+.2f},{_a[13]:+.2f})"
                        grip_str = f"L_grip={_a[14]:+.0f} R_grip={_a[15]:+.0f}"
                    else:
                        lpos_str = rpos_str = lq_str = rq_str = "(no action yet)"
                        grip_str = ""

                    # 13th session, part 9 — actual robot waist + head
                    # joint state.  If Phase D is in charge, the ACTUAL
                    # values should track ``waist_delta_str`` /
                    # ``head_delta_str`` (the Phase D targets).  When
                    # they diverge (Pink IK is overwriting waist), the
                    # waist actuals will sit near whatever IK chose
                    # rather than user input.  This is the smoking-gun
                    # diagnostic for "tracker not reflected on robot".
                    actual_waist_str = "(no waist joints)"
                    actual_head_str = "(no head joints)"
                    # 13th session, part 11 — robot wrist actual joint
                    # angles (yaw/roll/pitch) for the wrist 3-DoF chain.
                    actual_left_wrist_str = "(no wrist joints)"
                    actual_right_wrist_str = "(no wrist joints)"
                    try:
                        _robot_diag = env.scene["robot"]
                        _jp = _robot_diag.data.joint_pos[0].detach().cpu().numpy()
                        _jn = list(_robot_diag.data.joint_names)
                        if _fb.get("waist_ids"):
                            _w = [float(_jp[i]) for i in _fb["waist_ids"]]
                            actual_waist_str = (
                                f"yaw={_w[0]:+.2f} pitch={_w[1]:+.2f} roll={_w[2]:+.2f}"
                            )
                        if _fb.get("head_ids"):
                            _h = [float(_jp[i]) for i in _fb["head_ids"]]
                            actual_head_str = (
                                f"yaw={_h[0]:+.2f} pitch={_h[1]:+.2f} roll={_h[2]:+.2f}"
                            )
                        for side, slot in (("left", "actual_left_wrist_str"),
                                            ("right", "actual_right_wrist_str")):
                            try:
                                yidx = _jn.index(f"{side}_wrist_yaw_joint")
                                ridx = _jn.index(f"{side}_wrist_roll_joint")
                                pidx = _jn.index(f"{side}_wrist_pitch_joint")
                                vals = (
                                    f"yaw={float(_jp[yidx]):+.2f} "
                                    f"roll={float(_jp[ridx]):+.2f} "
                                    f"pitch={float(_jp[pidx]):+.2f}"
                                )
                                if slot == "actual_left_wrist_str":
                                    actual_left_wrist_str = vals
                                else:
                                    actual_right_wrist_str = vals
                            except ValueError:
                                pass
                    except Exception:  # noqa: BLE001
                        pass

                    print(
                        f"[run_teleop][TRACK step={step}] LIVE channels:\n"
                        f"  HMD pose:        {hmd_str}     "
                        f"{'→ head target euler ' + head_delta_str + ' (drives robot head_yaw/pitch/roll)' if hmd is not None else '(missing — wear headset)'}\n"
                        f"  waist tracker:   {waist_str}   "
                        f"{'→ waist target euler ' + waist_delta_str + ' (drives robot waist_yaw/pitch/roll)' if trackers.get('waist') is not None else '(missing — APK Body toggle off?)'}\n"
                        f"  left_forearm:    {lfa_str}     (fallback only — ignored while controller is present)\n"
                        f"  right_forearm:   {rfa_str}     (fallback only — ignored while controller is present)\n"
                        f"  left_wrist:      {lw_str}      (body-skeleton, surfaced only — controller is PRIMARY)\n"
                        f"  right_wrist:     {rw_str}      (body-skeleton, surfaced only — controller is PRIMARY)\n"
                        f"  left controller: {lc_str}      (PRIMARY L wrist driver)  rotation_delta={lc_eul_str}\n"
                        f"  right controller:{rc_str}      (PRIMARY R wrist driver)  rotation_delta={rc_eul_str}\n"
                        f"  → action       L_wrist pos={lpos_str} quat={lq_str}\n"
                        f"                 R_wrist pos={rpos_str} quat={rq_str}  {grip_str}\n"
                        f"  → robot actual head  {actual_head_str}\n"
                        f"                 waist {actual_waist_str}  "
                        f"(should match Phase D target euler above — divergence means Pink IK is overwriting)\n"
                        f"                 L_wrist {actual_left_wrist_str}\n"
                        f"                 R_wrist {actual_right_wrist_str}  "
                        f"(should track rotation_delta above — Pink IK solves wrist joints from quat target)",
                        flush=True,
                    )
                except Exception as exc:  # noqa: BLE001 — diagnostic only
                    print(
                        f"[run_teleop][XR-HEALTH] snapshot probe failed at "
                        f"step {step}: {type(exc).__name__}: {exc}",
                        flush=True,
                    )
        else:
            # ``while … else`` runs when the condition turns False without a ``break``.
            # We hit this when simulation_app.is_running() returns False — typically
            # because Isaac Sim's XR session collapsed or omni.kit was asked to
            # quit (e.g. SteamVR closing the HMD focus, or the user closing the
            # Kit window via Ctrl-Q / "X").
            exit_reason = f"simulation_app.is_running() became False at step={step}"
    except KeyboardInterrupt:
        exit_reason = f"KeyboardInterrupt at step={step}"
        print(f"\n[run_teleop] {exit_reason}", flush=True)
    except BaseException as exc:  # noqa: BLE001 — surface the real culprit
        import traceback as _tb
        exit_reason = f"{type(exc).__name__} at step={step}: {exc}"
        print(f"\n[run_teleop] FATAL during step loop: {exit_reason}", flush=True)
        _tb.print_exc()
    finally:
        # Final post-mortem so the user can tell whether the loop exited
        # because Isaac Sim quit on us vs. because we asked it to.  The
        # banner explicitly distinguishes "normal exit" (--steps / --max_seconds
        # reached, KeyboardInterrupt) from "anomalous exit" (window closed,
        # simulation_app died, traceback) so users don't mistake a normal
        # smoke run for a crash.
        elapsed_total = _time.perf_counter() - loop_start_time
        is_normal_exit = (
            exit_reason.startswith("reached --steps=")
            or exit_reason.startswith("reached --max_seconds=")
            or exit_reason.startswith("KeyboardInterrupt")
        )
        banner_top = "=" * 72
        if is_normal_exit:
            print(
                "\n" + banner_top + "\n"
                f"[run_teleop] ✅ NORMAL EXIT — reason: {exit_reason}\n"
                f"           steps_completed={step}, elapsed={elapsed_total:.1f}s, "
                f"avg_rate={step/max(elapsed_total, 1e-6):.1f} Hz\n"
                f"           simulation_app.is_running()={simulation_app.is_running()}\n"
                + banner_top,
                flush=True,
            )
        else:
            print(
                "\n" + banner_top + "\n"
                f"[run_teleop] ⚠️ ANOMALOUS EXIT — reason: {exit_reason}\n"
                f"           steps_completed={step}, elapsed={elapsed_total:.1f}s\n"
                f"           simulation_app.is_running()={simulation_app.is_running()}\n"
                + banner_top,
                flush=True,
            )
        if exit_reason.startswith("simulation_app.is_running()"):
            print(
                "[run_teleop] HINT: Isaac Sim shut down on its own.  Common causes\n"
                "  for --render_mode=steamvr_native specifically:\n"
                "    * PICO headset is asleep / not on the head, so SteamVR put the\n"
                "      OpenXR session into standby and Kit followed.  Wear the\n"
                "      headset (or wake it from sleep) before relaunching.\n"
                "    * Another OpenXR client (Oculus Home, VRChat...) grabbed the\n"
                "      session.  Quit it and retry.\n"
                "    * Try --render_mode steamvr_desktop (renders to the PC window;\n"
                "      no HMD focus needed) or --render_mode monitor (no XR at all)\n"
                "      to confirm the rest of the pipeline works.",
                flush=True,
            )
        try:
            device.stop()
        except Exception as exc:  # noqa: BLE001
            print(f"[run_teleop] device.stop() failed: {type(exc).__name__}: {exc}",
                  flush=True)
        try:
            env.close()
        except Exception as exc:  # noqa: BLE001
            print(f"[run_teleop] env.close() failed: {type(exc).__name__}: {exc}",
                  flush=True)
        try:
            simulation_app.close()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
