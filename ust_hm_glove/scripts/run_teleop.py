"""Fourier GR1T2 Windows/SteamVR teleoperation entrypoint.

Mirrors ``ust_hm_glove/scripts/run_teleop.py`` with a GR1T2-specific
env-id picker and a 36D idle action.  The teleop device hierarchy is:

    pico_udcap   → :class:`GR1T2FourierUDCAPDevice`  (Windows/SteamVR UDCAP stack, 36D)
    handtracking → OpenXR hand tracking via ``GR1T2RetargeterCfg``
    manusvive    → ManusViveCfg + ``GR1T2RetargeterCfg``

``--diag {off,idle,oscillate}`` bypasses the teleop device and drives
the env with known-good synthetic actions to isolate whether a
stationary robot is caused by the VR pipeline vs Pink IK / env_cfg.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]   # repo root (contains ust_ws/)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.environ.setdefault("IPC_IGNORE_VERSION", "1")

_DEFAULT_STEAMVR_OPENXR = r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\steamxr_win64.json"


# DLL-load-order guards: keep these BEFORE Isaac Sim boot.
try:
    import pinocchio  # noqa: F401
except Exception:  # pragma: no cover
    pass

try:
    import h5py  # noqa: F401
except Exception:  # pragma: no cover
    pass


from isaaclab.app import AppLauncher  # type: ignore

# Apply the osqp 0.6 ↔ qpsolvers 4.x SolverStatus shim before anything
# in the import chain touches qpsolvers.  This keeps the
# ``isaacsim-core==0.6.7.post3`` pin intact while restoring
# ``available_solvers: ['osqp']`` so Pink IK actually moves the robot.
# (Imported lazily inside a try/except because this script must still
# run for ``--help`` etc. even if osqp isn't installed.)
try:
    from ust_ws.ust_hm_glove.teleop import _osqp_compat  # noqa: F401
    _osqp_compat.apply()
except Exception as _exc:
    print(f"[run_teleop][WARN] osqp compat shim not applied: {_exc}")


_ENV_MONITOR = "Isaac-KitchenSorting-GR1T2-Fourier-Monitor-v0"
_ENV_BASE = "Isaac-KitchenSorting-GR1T2-Fourier-v0"
_ENV_VR = "Isaac-KitchenSorting-GR1T2-Fourier-VR-v0"
_ENV_WAIST = "Isaac-KitchenSorting-GR1T2-Fourier-WaistEnabled-v0"
_ENV_DATA = "Isaac-KitchenSorting-GR1T2-Fourier-DataCollect-v0"
_ENV_VISION = "Isaac-KitchenSorting-GR1T2-Fourier-Vision-v0"
_ENV_ROBOT_ONLY = "Isaac-KitchenSorting-GR1T2-Fourier-RobotOnly-v0"


def _parse_args():
    parser = argparse.ArgumentParser(description="Fourier GR1T2 Kitchen Sorting — Windows/SteamVR teleop")
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
            "steamvr_desktop: Virtual Desktop Desktop Theater | "
            "steamvr_native: omni.kit.xr.system.steamvr (experimental) | "
            "cloudxr: future"
        ),
    )
    parser.add_argument(
        "--env_variant",
        type=str,
        default="auto",
        choices=["auto", "base", "waist_enabled", "monitor", "vr", "vision", "data_collect", "robot_only"],
        help=(
            "Which GR1T2 env variant to launch.  'auto' picks monitor/VR based on --render_mode. "
            "'robot_only' uses an empty scene (robot+ground+light) for finger teleop diagnosis "
            "without table/box obstructions."
        ),
    )
    parser.add_argument(
        "--path_b_port",
        type=int,
        default=0,
        help=(
            "VMC OSC port for UDCAP bone broadcast (UDP).  "
            "9.37 default CHANGED 39539 -> 0 to migrate to Skeleton 2.0 "
            "(SteamVR Skeletal Input) as the primary finger source.  Pass "
            "39539 to re-enable VMC OSC reception when UDCAP's Skeletal "
            "Input 2.0 path is unavailable (the retargeter still prefers "
            "skeletal whenever both sources are present)."
        ),
    )
    parser.add_argument(
        "--skeleton2",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help=(
            "9.37 -- enable SteamVR Skeletal Input 2.0 (31-bone hand) as "
            "the primary finger source.  Default True; set to false only "
            "for headless / sampler-disabled diagnostics.  When the UDCAP "
            "driver does not implement CreateSkeletonComponent the runtime "
            "probe will report bActive=False; in that case re-arm VMC with "
            "--path_b_port 39539 or use the per-finger curl Action API "
            "fallback (always on)."
        ),
    )
    parser.add_argument(
        "--vr_runtime",
        type=str,
        default="auto",
        choices=["auto", "pico_connect", "virtual_desktop", "steamvr_native"],
        help=(
            "9.37 -- which streaming app routes the PICO HMD/controllers/"
            "trackers into SteamVR.\n"
            "  pico_connect    : PICO Connect (prism driver) -- 'PICO Connect "
            "-> SteamVR -> PC -> Isaac Lab' pipeline.  Use the PICO Motion "
            "Tracker template `config/tracker_binding_pico_connect.json`.\n"
            "  virtual_desktop : Virtual Desktop Streamer + 'Forward tracking "
            "to SteamVR' (legacy default; uses VD AI body tracking segments).\n"
            "  steamvr_native  : Headset has its own SteamVR driver (rare for "
            "PICO 4 Ultra).\n"
            "  auto            : leave SteamVR add-on selection up to the user."
        ),
    )
    parser.add_argument(
        "--enable_waist_dof",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=None,
        help=(
            "Force device-side WaistEstimator on/off.  Default None = "
            "use the env_cfg's pico_device_cfg setting (RobotOnly "
            "defaults OFF for finger debug; WaistEnabled defaults ON).  "
            "Disable when Virtual Desktop's hips tracker has a pitch "
            "bias that auto-bends the robot forward."
        ),
    )
    parser.add_argument(
        "--forearm_offset",
        type=float,
        default=None,
        help=(
            "Forearm-to-wrist offset along the tracker's local +X axis "
            "(metres).  Default None = use env_cfg.pico_device_cfg "
            "(0.12 m, calibrated for Vive Enhanced Forearm trackers).  "
            "Bump to 0.25 for Virtual Desktop's AI body-tracking 'arm_lower' "
            "estimates which sit closer to the elbow than the forearm."
        ),
    )
    parser.add_argument(
        "--prefer_controller",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=False,
        help=(
            "When True, the wrist EEF target uses the Touch / knuckles "
            "controller pose instead of the *_forearm tracker.  9.16: "
            "default False -- the typical rig has physical Vive trackers "
            "mounted on the user's wrists/forearms (bound to "
            "left_arm_lower / right_arm_lower) which directly track wrist "
            "motion.  Pass true ONLY if you do not wear wrist trackers "
            "AND have configured UDCAP Space Plan to match your "
            "controller hardware (see UDCAP CONFIGURATION CHECK warning)."
        ),
    )
    parser.add_argument(
        "--ignore_trackers",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=False,
        help=(
            "9.26 -- umbrella safety flag for the no-tracker rig (only "
            "PICO HMD + UDCAP gloves + 2 controllers).  When True, this "
            "single flag locks the arms in idle T-pose so only the "
            "fingers (via UDCAP VMC) animate:\n"
            "    --prefer_controller    → False (controller path NOT used)\n"
            "    --enable_waist_dof     → False (waist stays at idle)\n"
            "    --head_follow_hmd      → False (head stays at idle)\n"
            "    --disable_arm_tracking → True  (arm EEF locked at idle)\n"
            "9.25 routed this through `prefer_controller=True` which let "
            "the controllers drive the wrist — the OPPOSITE of the user's "
            "intent ('팔은 트래커가 있을 때만 움직여야 한다, 컨트롤러가 "
            "아닌 트래커에만').  9.26 forces `disable_arm_tracking=True` "
            "instead so the arm stays still until a real tracker is added "
            "and the user explicitly removes the flag.  See memory.md §10.34."
        ),
    )
    parser.add_argument(
        "--disable_arm_tracking",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=False,
        help=(
            "9.26 -- when True, the retargeter forces the arm EEF targets "
            "to the idle T-pose regardless of forearm tracker / controller "
            "state.  Independent of `--ignore_trackers`; can be used to "
            "lock arms while keeping waist/head estimators active for "
            "users with only a hips tracker but no wrist trackers."
        ),
    )
    parser.add_argument(
        "--vmc_subtract_rest",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help=(
            "9.14 -- when True (default), the FourierHandMapper averages "
            "the first 30 VMC bone frames as a per-bone REST POSE and "
            "computes only the relative rotation thereafter.  Solves "
            "UDCAP's static ~16 deg pinky/ring/thumb offset that made the "
            "hand look 'half-curled' even at user open hand.  Set false "
            "to restore raw absolute mapping (legacy 9.13 behaviour)."
        ),
    )
    parser.add_argument(
        "--vmc_rest_frames",
        type=int,
        default=10,
        help=(
            "Number of VMC bone frames to average for the per-bone rest "
            "pose (--vmc_subtract_rest).  9.18 default 10 (~0.5 s) -- "
            "shorter than 9.14's 30 because a long window lets user "
            "fidgeting during cal absorb later motion.  Bump if you can "
            "guarantee the user holds still for longer."
        ),
    )
    parser.add_argument(
        "--waist_pitch_deadband_deg",
        type=float,
        default=17.0,
        help=(
            "9.15 -- ignore hips pitch motion below this many degrees "
            "(default 17 deg ~= 0.3 rad).  Absorbs Virtual Desktop AI "
            "body-tracker noise (observed up to 111 deg apparent pitch "
            "even with user standing still).  Set 0 to disable."
        ),
    )
    parser.add_argument(
        "--follow_hmd",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=False,
        help=(
            "9.16/9.18 -- when True, the Isaac Sim viewport camera "
            "follows the HMD pose each step (first-person from world).  "
            "DEFAULT FALSE in 9.18 because the user requested removing "
            "the camera-follow and using head-joint tracking instead "
            "(see --head_follow_hmd).  Keep False unless you specifically "
            "want the viewport camera to follow user head."
        ),
    )
    parser.add_argument(
        "--head_follow_hmd",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help=(
            "9.18 -- when True (default), the robot's head_yaw / "
            "head_pitch / head_roll joints are driven from the user's "
            "HMD orientation each step.  The robot's head mirrors the "
            "user's head turn / nod / tilt.  Set False to leave the "
            "robot head at rest."
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
        "--finger_proximal_scale",
        type=float,
        default=2.5,
        help="Multiplier for proximal/intermediate finger curl magnitudes "
             "(FourierHandMapper.proximal_scale).  9.12 default 2.5 -- "
             "video review of 4.0/6.0 runs showed L_idx_proximal saturating "
             "at the joint limit (-1.570) within frame 20 and staying "
             "flat-lined.  2.5 paired with --finger_use_tanh keeps the "
             "user's mid-fist responsive without losing upper-half range.  "
             "Tune lower (1.5) if robot fingers oscillate; higher (4.0) "
             "if user's glove reads particularly low quat magnitudes.",
    )
    parser.add_argument(
        "--finger_thumb_scale",
        type=float,
        default=2.5,
        help="Multiplier for thumb flexion (FourierHandMapper.thumb_scale). "
             "9.12 default 2.5 -- same reasoning as --finger_proximal_scale.",
    )
    parser.add_argument(
        "--finger_use_tanh",
        type=lambda s: s.lower() in ("1", "true", "yes", "on"),
        default=True,
        help="Apply tanh-shaped non-linear amplification (default True) so "
             "finger output asymptotes smoothly at the joint limit instead "
             "of hard-clipping once raw*scale > 1.  Set to 'false' to "
             "restore the legacy linear path (matches behaviour pre-9.12).",
    )
    parser.add_argument(
        "--finger_lp_alpha",
        type=float,
        default=0.4,
        help="9.23 -- single-pole low-pass alpha applied to the 22D "
             "finger output every advance().  UDCAP runs at ~140 Hz / "
             "SteamVR at 120 Hz but env.step is only 20 Hz (decimation 6 "
             "@ 120 Hz physics), so the per-frame 'latest only' read "
             "produces visible 0<->limit aliasing on natural finger "
             "motion.  Default 0.4 = mild smoothing without perceptible "
             "lag (~30 ms time constant); 1.0 disables; 0.2 = very "
             "smooth (recommended if jitter persists).",
    )
    parser.add_argument(
        "--sampler_init_timeout",
        type=float,
        default=30.0,
        help="9.38 -- watchdog timeout (seconds) for openvr.init() inside "
             "SteamVRSampler.start().  When SteamVR is not running OR the "
             "PICO Connect / Steam Link / VD streaming layer has not yet "
             "exposed the HMD provider, openvr.init() blocks indefinitely "
             "with no diagnostic output (silent hang right after the "
             "'generated runtime manifest' log line).  This flag converts "
             "that hang into a TimeoutError with the 4 most likely root "
             "causes printed to the console.  Default 30s; bump to 60s on "
             "slow machines or if PICO Connect takes a long time to "
             "complete its first handshake; 0 disables the watchdog (NOT "
             "recommended).  Run "
             "`python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect` "
             "for a 6-layer pipeline probe before raising the timeout.",
    )
    parser.add_argument(
        "--render_interval",
        type=int,
        default=None,
        help="9.27 (research/33 Cause #4) -- override env_cfg.sim.render_interval. "
             "9.24 default = 1 (render at 120 Hz, every physics step) which "
             "lets GPU render thread starve the encoder/streaming thread. "
             "Set to 2 to render at 60 Hz with 120 Hz physics intact "
             "(saves ~5-10 ms wall-time per env.step on RTX PRO 6000); "
             "set to 4 for 30 Hz render in headless data-collection runs. "
             "Pico 4 Ultra display caps at 90 Hz anyway so 60 Hz render "
             "is the recommended VR setting.  Leave unset to keep the "
             "env_cfg default (currently 1).",
    )
    parser.add_argument(
        "--process_priority",
        type=str,
        default="high",
        choices=["normal", "high", "realtime"],
        help="9.27 (research/33 Cause #5) -- Windows process priority class. "
             "Default 'high' reduces P99 env.step jitter (~30 ms -> ~12 ms) "
             "by giving the Isaac Sim python process priority over Discord/"
             "browser/AV processes on the same core.  'realtime' is "
             "stronger but can freeze the desktop UI, so it is rarely "
             "needed.  No-op on non-Windows platforms.",
    )
    # Diagnostic flags
    parser.add_argument(
        "--debug_ik",
        action="store_true",
        help="Force show_ik_warnings=True on the Pink IK controller.",
    )
    parser.add_argument(
        "--freeze_orientation",
        action="store_true",
        help="Send the idle-pose quaternion instead of forearm tracker rotation "
             "(isolates orientation vs position failure).",
    )
    parser.add_argument(
        "--diag",
        type=str,
        default="off",
        choices=["off", "idle", "oscillate", "finger_sine"],
        help=(
            "Override action source for pipeline diagnosis. "
            "'idle' sends env.idle_action every step. "
            "'oscillate' sends a sine-wave wrist target. "
            "'finger_sine' sends a 0.5 Hz sine-wave on every L_*/R_* hand "
            "joint (slot 14:36 of the action) to test whether the hand "
            "joints physically respond to commanded positions, bypassing "
            "the retargeter / VMC entirely."
        ),
    )
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    # 9.26: ``--ignore_trackers`` umbrella overrides four sub-flags so the
    # user doesn't have to remember the quartet.  9.25 mistakenly forced
    # `--prefer_controller=true` here, letting controllers drive the wrist;
    # the user's intent was the opposite ("팔은 트래커가 있을 때만 움직여야
    # 한다") so 9.26 forces `--disable_arm_tracking=true` and explicitly
    # keeps `--prefer_controller=false` to lock arms at idle.
    if getattr(args, "ignore_trackers", False):
        if args.prefer_controller:
            print(
                "[run_teleop][--ignore_trackers] forcing "
                "--prefer_controller=false (controllers must NOT drive the wrist)."
            )
            args.prefer_controller = False
        if args.enable_waist_dof is not False:
            print(
                "[run_teleop][--ignore_trackers] forcing "
                "--enable_waist_dof=false (waist stays at idle)."
            )
            args.enable_waist_dof = False
        if args.head_follow_hmd:
            print(
                "[run_teleop][--ignore_trackers] forcing "
                "--head_follow_hmd=false (head stays at idle)."
            )
            args.head_follow_hmd = False
        if not args.disable_arm_tracking:
            print(
                "[run_teleop][--ignore_trackers] forcing "
                "--disable_arm_tracking=true (arm EEF locked at idle T-pose)."
            )
            args.disable_arm_tracking = True
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
            else:
                print(
                    "[run_teleop][WARN] XR_RUNTIME_JSON not pointing at SteamVR and the expected "
                    f"manifest '{args.steamvr_openxr_json}' is missing."
                )


def _pick_env_id(render_mode: str, variant: str) -> str:
    if variant == "base":
        return _ENV_BASE
    if variant == "waist_enabled":
        return _ENV_WAIST
    if variant == "monitor":
        return _ENV_MONITOR
    if variant == "vr":
        return _ENV_VR
    if variant == "vision":
        return _ENV_VISION
    if variant == "data_collect":
        return _ENV_DATA
    # auto
    if render_mode == "monitor":
        return _ENV_MONITOR
    if render_mode in ("steamvr_desktop", "steamvr_native", "cloudxr"):
        return _ENV_VR
    return _ENV_BASE


def _set_process_priority(pref: str) -> None:
    """9.27 (research/33 Cause #5): bump Windows process priority.

    Defaults to ``high`` so the Isaac Sim python process wins CPU
    arbitration against background apps (Discord, browser, AV).  This
    reduces P99 env.step jitter (~30 ms tail -> ~12 ms in our internal
    measurements) which is the dominant remaining contributor to
    perceived "fingers-don't-track" lag once VR streaming and the
    velocity_limit_sim fix have been removed.  No-op on non-Windows.
    """
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
            "leaving priority at NORMAL.  ``./isaaclab.sh -p -m pip "
            "install psutil`` once to enable this."
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
            f"({type(exc).__name__}: {exc}).  Try running the terminal "
            f"as Administrator if you need REALTIME."
        )


def main() -> int:
    args = _parse_args()
    _configure_xr(args)
    _set_process_priority(args.process_priority)

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    # Deferred imports — Isaac Sim must be running first
    import gymnasium as gym
    import torch

    from isaaclab.devices.teleop_device_factory import create_teleop_device
    from isaaclab.envs import ManagerBasedRLEnv

    # ── Env cfg resolution (direct, not through gym.register side-effects) ───
    # Earlier we tried to rely on ``import ust_ws.ust_hm_glove`` side
    # effects to populate the Gym registry.  That path swallowed any
    # ImportError silently when Isaac Sim's stdout capture masked our
    # traceback, producing an opaque ``NameNotFound`` downstream.  Now we
    # import the env_cfg module *directly* in the main flow so every
    # exception surfaces with a real traceback exactly where the user
    # expects it — and we still call into the package's ``_register()`` so
    # other scripts (``record_demos.py`` etc.) that DO go through the Gym
    # registry keep working.
    env_variant = args.env_variant
    if env_variant == "auto":
        env_variant = (
            "monitor" if args.render_mode == "monitor"
            else "vr" if args.render_mode in ("steamvr_desktop", "steamvr_native", "cloudxr")
            else "base"
        )

    print(f"[run_teleop] Loading env_cfg directly for variant='{env_variant}'...", flush=True)
    try:
        from ust_ws.ust_hm_glove.kitchen_sorting_gr1t2_env_cfg import (
            KitchenSortingGR1T2DataCollectEnvCfg,
            KitchenSortingGR1T2EnvCfg,
            KitchenSortingGR1T2MonitorEnvCfg,
            KitchenSortingGR1T2RobotOnlyEnvCfg,
            KitchenSortingGR1T2VREnvCfg,
            KitchenSortingGR1T2VisionEnvCfg,
            KitchenSortingGR1T2WaistEnvCfg,
        )
    except BaseException:
        import traceback as _tb
        print(
            "\n" + "=" * 72
            + "\n[run_teleop] FATAL: ust_hm_glove.kitchen_sorting_gr1t2_env_cfg "
            "failed to import.\n" + "=" * 72,
            flush=True,
        )
        _tb.print_exc()
        print(
            "=" * 72 + "\n"
            "[run_teleop] Hint: run the standalone diagnostic to narrow it down:\n"
            "    python -m ust_ws.ust_hm_glove.scripts.diagnose_env_cfg --headless\n"
            + "=" * 72,
            flush=True,
        )
        raise

    variant_map = {
        "base":          (KitchenSortingGR1T2EnvCfg,             _ENV_BASE),
        "waist_enabled": (KitchenSortingGR1T2WaistEnvCfg,        _ENV_WAIST),
        "monitor":       (KitchenSortingGR1T2MonitorEnvCfg,      _ENV_MONITOR),
        "vr":            (KitchenSortingGR1T2VREnvCfg,           _ENV_VR),
        "vision":        (KitchenSortingGR1T2VisionEnvCfg,       _ENV_VISION),
        "data_collect":  (KitchenSortingGR1T2DataCollectEnvCfg,  _ENV_DATA),
        "robot_only":    (KitchenSortingGR1T2RobotOnlyEnvCfg,    _ENV_ROBOT_ONLY),
    }
    env_cfg_cls, env_id = variant_map[env_variant]
    print(f"[run_teleop] env_cfg class: {env_cfg_cls.__name__}  →  Gym ID: {env_id}", flush=True)

    # Trigger the package's Gym registration as a side effect so that other
    # entry points (record_demos, run_hg_dagger, evaluate.py) that look
    # environments up by ID still find them.  Failures here are non-fatal
    # because we already hold the env_cfg class directly.
    try:
        import ust_ws.ust_hm_glove as _gr1t2_pkg  # noqa: F401
        # Re-run registration explicitly now that the import side-effects
        # are settled — this lets us log a clean summary even if the
        # package was previously imported during a circular-import chain
        # where gym registry manipulation may not have stuck.
        if hasattr(_gr1t2_pkg, "register_envs_now"):
            _gr1t2_pkg.register_envs_now()
        gr1t2_ids = sorted(k for k in gym.registry.keys() if "GR1T2" in k)
        print(
            f"[run_teleop] Gym registry (GR1T2/*): {len(gr1t2_ids)} ids registered",
            flush=True,
        )
    except BaseException as exc:
        print(f"[run_teleop][WARN] Gym registry side-effect import failed: {exc}", flush=True)

    from ust_ws.ust_hm_glove.teleop.gr1t2_udcap_device import (
        GR1T2FourierUDCAPDevice,
        GR1T2FourierUDCAPDeviceCfg,
    )

    env_cfg = env_cfg_cls()
    env_cfg.scene.num_envs = args.num_envs

    # 9.27 (research/33 Cause #4): allow overriding render_interval from
    # CLI without editing env_cfg files.  Default of None means "keep
    # whatever env_cfg has" (currently 1 from 9.24).  Setting to 2 halves
    # GPU render load while keeping physics at 120 Hz, freeing wall-time
    # for the encoder/streaming thread.
    if args.render_interval is not None:
        if args.render_interval < 1:
            print(f"[run_teleop][WARN] --render_interval {args.render_interval} < 1 ignored.")
        else:
            old = env_cfg.sim.render_interval
            env_cfg.sim.render_interval = int(args.render_interval)
            print(
                f"[run_teleop] render_interval override: {old} -> "
                f"{env_cfg.sim.render_interval} "
                f"(render rate = {(1.0 / env_cfg.sim.dt) / env_cfg.sim.render_interval:.0f} Hz, "
                f"physics rate = {1.0 / env_cfg.sim.dt:.0f} Hz)"
            )

    if args.debug_ik:
        try:
            env_cfg.actions.pink_ik_cfg.controller.show_ik_warnings = True
            print("[run_teleop] show_ik_warnings=True — Pink IK failures will be logged.")
        except AttributeError as exc:
            print(f"[run_teleop][WARN] Could not enable show_ik_warnings: {exc}")

    env = ManagerBasedRLEnv(cfg=env_cfg)

    print("\n" + "=" * 60)
    print("  Fourier GR1T2 Kitchen Sorting — Windows/SteamVR Teleop")
    print(f"  env_id      : {env_id}")
    print(f"  device      : {args.teleop_device}")
    print(f"  render_mode : {args.render_mode}")
    print(f"  episode     : {env_cfg.episode_length_s:.0f}s")
    print(f"  physics     : {1/env_cfg.sim.dt:.0f} Hz, render every {env_cfg.sim.render_interval}")
    print("=" * 60 + "\n")

    # ── Teleop device ────────────────────────────────────────────────────
    teleop_device = None
    if args.teleop_device == "pico_udcap":
        legacy_cfg = getattr(env_cfg, "pico_device_cfg", {}) or {}
        cfg_kwargs = dict(legacy_cfg)
        cfg_kwargs["path_b_port"] = int(args.path_b_port)
        # 9.37: Skeleton 2.0 is the primary finger source.  ``enable_skeletal``
        # gates whether the SteamVR Skeletal Input 2.0 query path is active in
        # the sampler.  Default True; --skeleton2 false disables the 31-bone
        # query and lets the chain fall through to the per-finger curl Action
        # API (then VMC if --path_b_port > 0, then trigger/grip).
        cfg_kwargs["enable_skeletal"] = bool(args.skeleton2)
        # 9.37: PICO Connect tracker binding template auto-selection.  The
        # default repo path matches Virtual Desktop's full-body segments
        # (hips / *_arm_lower / *_lower_leg).  When the user is on the
        # 'PICO Connect -> SteamVR' pipeline, switch to the dedicated
        # template if it exists alongside the legacy file.
        if args.vr_runtime == "pico_connect":
            from pathlib import Path as _Path
            pico_template = _Path(
                "./ust_ws/ust_hm_glove/config/tracker_binding_pico_connect.json"
            )
            if pico_template.exists():
                cfg_kwargs["tracker_binding_json"] = str(pico_template)
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
                    "`python -m ust_ws.ust_hm_glove.scripts.enumerate_trackers "
                    "--out ust_ws/ust_hm_glove/config/tracker_binding_pico_connect.json` "
                    "while PICO Connect is streaming to generate the template."
                )
        cfg_kwargs["freeze_orientation"] = bool(args.freeze_orientation)
        cfg_kwargs["hand_proximal_scale"] = float(args.finger_proximal_scale)
        cfg_kwargs["hand_thumb_scale"] = float(args.finger_thumb_scale)
        cfg_kwargs["hand_use_tanh_amplification"] = bool(args.finger_use_tanh)
        cfg_kwargs["finger_low_pass_alpha"] = float(args.finger_lp_alpha)
        cfg_kwargs["disable_arm_tracking"] = bool(args.disable_arm_tracking)
        if args.enable_waist_dof is not None:
            cfg_kwargs["enable_waist_dof"] = bool(args.enable_waist_dof)
        if args.forearm_offset is not None:
            cfg_kwargs["forearm_wrist_offset"] = (float(args.forearm_offset), 0.0, 0.0)
        cfg_kwargs["prefer_controller_for_eef"] = bool(args.prefer_controller)
        cfg_kwargs["hand_vmc_subtract_rest"] = bool(args.vmc_subtract_rest)
        cfg_kwargs["hand_vmc_rest_frames"] = int(args.vmc_rest_frames)
        deadband_rad = math.radians(float(args.waist_pitch_deadband_deg))
        cfg_kwargs["waist_deadband_rad"] = (0.0, deadband_rad, 0.0)
        cfg_kwargs["enable_head_follow_hmd"] = bool(args.head_follow_hmd)
        # 9.38: openvr.init() watchdog timeout — surface SteamVR not running
        # / HMD not streaming as a TimeoutError instead of a silent infinite
        # hang.  Default 30s.  See memory.md §10.45 / gotcha #30.
        cfg_kwargs["sampler_init_timeout_sec"] = float(args.sampler_init_timeout)
        # Filter unknown keys so future env_cfg extensions don't break us.
        known = set(GR1T2FourierUDCAPDeviceCfg.__dataclass_fields__.keys())
        cfg_kwargs = {k: v for k, v in cfg_kwargs.items() if k in known}
        device_cfg = GR1T2FourierUDCAPDeviceCfg(**cfg_kwargs)
        teleop_device = GR1T2FourierUDCAPDevice(device_cfg)
        teleop_device.start()
    elif args.teleop_device in getattr(env_cfg, "teleop_devices", object()).__dict__.get("devices", {}):
        teleop_device = create_teleop_device(args.teleop_device, env_cfg.teleop_devices.devices)
        teleop_device.reset()
    else:
        print(f"[run_teleop][WARN] Device '{args.teleop_device}' not registered — idling.")

    # ── Optional XR anchor ──────────────────────────────────────────────
    if args.teleop_device == "pico_udcap" and hasattr(env_cfg, "xr") and args.render_mode != "monitor":
        import carb  # noqa: WPS433
        from isaacsim.core.prims import SingleXFormPrim

        xr = env_cfg.xr
        anchor = SingleXFormPrim("/XRAnchor", position=xr.anchor_pos, orientation=xr.anchor_rot)
        settings = carb.settings.get_settings()
        for profile in ("", "/profile/ar", "/profile/vr"):
            prefix = f"/persistent/xr{profile}"
            settings.set_string(f"{prefix}/anchorMode", "custom anchor")
            settings.set_float(f"{prefix}/render/nearPlane", getattr(xr, "near_plane", 0.1))
        for profile in ("/profile/ar", "/profile/vr"):
            settings.set_string(f"/xrstage{profile}/customAnchor", anchor.prim_path)
        settings.set_string("/xrstage/customAnchor", anchor.prim_path)
        print(f"[run_teleop] XR anchor @ pos={xr.anchor_pos} rot={xr.anchor_rot}")

    obs, _info = env.reset()
    idle_action = env_cfg.idle_action.unsqueeze(0).repeat(env.num_envs, 1).to(env.device)

    # ── Runtime verification: hand actuator + joint properties ──────────
    # 9.7 confirmed stiffness=10000 reaches the articulation but the
    # robot's fingers still don't move.  9.8 dumps everything else that
    # might silently clamp finger motion: joint position limits,
    # friction, current pose, plus the resolved joint_id (so we can
    # later index ``robot.data.joint_pos`` directly to compare against
    # Pink IK targets).  If joint limits are (0, 0.001) the joint is
    # effectively locked regardless of stiffness; if friction is huge,
    # PD output gets cancelled out.
    hand_joint_ids: list[int] = []
    hand_joint_names: list[str] = []
    try:
        robot = env.scene["robot"]
        joint_names = list(robot.data.joint_names)
        joint_stiffness = robot.data.joint_stiffness[0].cpu().numpy()
        joint_damping = robot.data.joint_damping[0].cpu().numpy()
        # joint_pos_limits is shape (n_envs, n_joints, 2) -> [lo, hi]
        joint_limits = robot.data.joint_pos_limits[0].cpu().numpy()
        joint_pos = robot.data.joint_pos[0].cpu().numpy()
        # friction lives on the underlying physx view, not on data
        try:
            joint_friction = robot.root_physx_view.get_dof_friction_coefficients()[0].cpu().numpy()
        except Exception:
            joint_friction = None
        sample_arms = ("left_elbow_pitch_joint",)
        # ALL 22 hand joints in canonical FOURIER_HAND_JOINT_NAMES order so
        # we can verify per-joint sign + range.  Critical because GR1T2's
        # finger flexion is in the negative direction for proximal/
        # intermediate/yaw joints (range like [-1.57, 0]) but positive for
        # thumb pitch.  Sending +0.5 to a [-1.57, 0] joint gets clamped
        # to 0 -> joint never moves.
        sample_hands = (
            # L proximal drivers
            "L_index_proximal_joint",
            "L_middle_proximal_joint",
            "L_pinky_proximal_joint",
            "L_ring_proximal_joint",
            "L_thumb_proximal_yaw_joint",
            # R proximal drivers
            "R_index_proximal_joint",
            "R_middle_proximal_joint",
            "R_pinky_proximal_joint",
            "R_ring_proximal_joint",
            "R_thumb_proximal_yaw_joint",
            # L intermediate / thumb pitch
            "L_index_intermediate_joint",
            "L_middle_intermediate_joint",
            "L_pinky_intermediate_joint",
            "L_ring_intermediate_joint",
            "L_thumb_proximal_pitch_joint",
            # R intermediate / thumb pitch
            "R_index_intermediate_joint",
            "R_middle_intermediate_joint",
            "R_pinky_intermediate_joint",
            "R_ring_intermediate_joint",
            "R_thumb_proximal_pitch_joint",
            # thumb distals
            "L_thumb_distal_joint",
            "R_thumb_distal_joint",
        )
        print("[run_teleop][joint-property-verify]")
        for jn in sample_arms + sample_hands:
            if jn in joint_names:
                idx = joint_names.index(jn)
                if jn.startswith(("L_", "R_")):
                    hand_joint_ids.append(idx)
                    hand_joint_names.append(jn)
                k = float(joint_stiffness[idx])
                d = float(joint_damping[idx])
                lo, hi = float(joint_limits[idx, 0]), float(joint_limits[idx, 1])
                pos = float(joint_pos[idx])
                fric = float(joint_friction[idx]) if joint_friction is not None else float("nan")
                # Highlight the most likely smoking guns.
                flags = []
                if hi - lo < 0.05:
                    flags.append("!!RANGE_TOO_SMALL")
                if fric > 0.5:
                    flags.append("!!HIGH_FRICTION")
                if k < 100:
                    flags.append("!!STIFFNESS_TOO_LOW")
                flag_str = " " + " ".join(flags) if flags else ""
                print(
                    f"  {jn:32s} stiff={k:8.1f} damp={d:7.2f} "
                    f"limits=[{lo:+.3f},{hi:+.3f}] (range={hi-lo:.3f}) "
                    f"fric={fric:.4f} pos={pos:+.4f}{flag_str}"
                )
            else:
                print(f"  {jn:32s} NOT FOUND in articulation joint_names")
        # Also collect ALL hand joint IDs (in articulation order) so the
        # FingerCmp diagnostic can read joint_pos directly.
        for jn in joint_names:
            if jn.startswith(("L_", "R_")) and jn not in hand_joint_names:
                hand_joint_ids.append(joint_names.index(jn))
                hand_joint_names.append(jn)
    except Exception as exc:  # noqa: BLE001
        print(f"[run_teleop][joint-property-verify] failed ({type(exc).__name__}: {exc})")

    import math as _math
    import time as _time

    def _oscillate_action(t_seconds: float) -> "torch.Tensor":
        """Sway both wrists ±10 cm around idle on a 0.5 Hz sine."""
        amp = 0.10
        s = _math.sin(2.0 * _math.pi * 0.5 * t_seconds)
        act = idle_action.clone()
        act[0, 0] += amp * s          # L X
        act[0, 7] -= amp * s          # R X (opposite phase)
        act[0, 2] += 0.5 * amp * _math.cos(2.0 * _math.pi * 0.5 * t_seconds)
        act[0, 9] += 0.5 * amp * _math.cos(2.0 * _math.pi * 0.5 * t_seconds)
        return act

    def _finger_sine_action(t_seconds: float) -> "torch.Tensor":
        """Drive every hand joint with a 0.5 Hz sine in [0, 0.8] rad.

        Bypasses the retargeter and VMC entirely.  If the robot's
        fingers visibly oscillate, the hand-joint actuator and joint
        kinematics are healthy (the issue is upstream — retargeter /
        VMC mapping).  If the fingers stay frozen even with this hard
        direct command, the joint physics itself is the problem
        (joint locked in USD, drive type wrong, etc).
        """
        s = (1.0 + _math.sin(2.0 * _math.pi * 0.5 * t_seconds)) * 0.5  # [0, 1]
        amp = 0.8 * s  # [0, 0.8] rad
        act = idle_action.clone()
        # action[14:36] are the 22 hand joint targets.  Drive them all
        # to ``amp`` so every finger curls in unison.
        act[0, 14:36] = amp
        return act

    # 9.16 — HMD-follow viewport camera support.  When --follow_hmd is on,
    # we update the active viewport's camera each step to match the user's
    # HMD pose (in the robot's base_link frame).  Combine with
    # --render_mode steamvr_desktop or Virtual Desktop Desktop Theater to
    # see the result inside the headset.  Imports are kept inside the
    # branch so a missing isaacsim viewport module on import-only paths
    # doesn't break the teleop launcher.
    follow_hmd_enabled = bool(args.follow_hmd)
    set_camera_view = None
    follow_hmd_warned = {
        "missing_api": False, "no_hmd": False, "no_head_link": False,
        "head_link_resolved": False,
    }
    # 9.17 — cache the robot's head link index so we don't re-search
    # body_names every advance.  Resolved lazily on first call after
    # env.reset() has populated body_names.
    follow_hmd_head_idx: list[int] = [-1]  # use list for mutable closure capture
    if follow_hmd_enabled:
        try:
            from isaacsim.core.utils.viewports import set_camera_view  # type: ignore
            print("[run_teleop] --follow_hmd: HMD-follow viewport camera ENABLED.")
        except Exception as exc:  # noqa: BLE001
            print(
                f"[run_teleop][WARN] --follow_hmd requested but "
                f"isaacsim.core.utils.viewports.set_camera_view import failed: {exc}.  "
                f"Continuing without HMD follow."
            )
            follow_hmd_enabled = False

    def _quat_rotate_vec_wxyz(q, v):
        """Rotate 3-vector v by quaternion q (w, x, y, z).  Uses the
        Rodrigues form which is stable for unit quaternions."""
        import numpy as _np
        qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        qvec = _np.array([qx, qy, qz], dtype=_np.float64)
        v = _np.asarray(v, dtype=_np.float64)
        t = 2.0 * _np.cross(qvec, v)
        return v + qw * t + _np.cross(qvec, t)

    def _resolve_head_link_idx() -> int:
        """Find the robot's eye-level head link index in robot.data.body_names.

        Tries the GR1T2 chain head_pitch_link -> head_roll_link -> head_yaw_link
        (most distal first; head_pitch_link is at eye height).  Returns -1 if
        not found, in which case the camera falls back to following the HMD
        pose in user-space (legacy 9.16 behaviour).
        """
        try:
            robot = env.scene["robot"]
            body_names = list(robot.data.body_names)
        except Exception:
            return -1
        for candidate in (
            "head_pitch_link",  # GR1T2 most-distal head link (eye height)
            "head_roll_link",
            "head_yaw_link",
            "head",
        ):
            if candidate in body_names:
                if not follow_hmd_warned["head_link_resolved"]:
                    print(
                        f"[run_teleop][follow_hmd] anchoring camera at robot "
                        f"link {candidate!r} (idx={body_names.index(candidate)})."
                    )
                    follow_hmd_warned["head_link_resolved"] = True
                return body_names.index(candidate)
        if not follow_hmd_warned["no_head_link"]:
            print(
                f"[run_teleop][follow_hmd] WARN: no head_*_link found in "
                f"robot body_names ({len(body_names)} bodies).  Falling back "
                f"to HMD-only camera (will land at user's physical head height)."
            )
            follow_hmd_warned["no_head_link"] = True
        return -1

    # 9.18 — head joint follow.  When the device's HeadEstimator is
    # enabled we read its (yaw, pitch, roll) estimate every step and
    # write the values to the robot's head_yaw/pitch/roll joint position
    # targets via the articulation API.  Bypasses the action manager
    # because the head joints are not part of the 36D Pink IK action.
    head_follow_enabled = bool(args.head_follow_hmd)
    head_joint_ids: list[int] = []
    head_joint_resolved = [False]
    head_warned = {"no_estimator": False, "no_joints": False, "first_target": False}

    def _resolve_head_joint_ids() -> bool:
        if head_joint_resolved[0]:
            return bool(head_joint_ids)
        head_joint_resolved[0] = True
        try:
            robot = env.scene["robot"]
            joint_names = list(robot.data.joint_names)
        except Exception as exc:  # noqa: BLE001
            print(
                f"[run_teleop][head_follow] could not access robot joint "
                f"names ({type(exc).__name__}: {exc}); disabling."
            )
            return False
        for jn in ("head_yaw_joint", "head_pitch_joint", "head_roll_joint"):
            if jn not in joint_names:
                if not head_warned["no_joints"]:
                    print(
                        f"[run_teleop][head_follow] joint {jn!r} not found "
                        f"in articulation; disabling head follow."
                    )
                    head_warned["no_joints"] = True
                return False
            head_joint_ids.append(joint_names.index(jn))
        print(
            f"[run_teleop][head_follow] head joints resolved: yaw={head_joint_ids[0]} "
            f"pitch={head_joint_ids[1]} roll={head_joint_ids[2]}"
        )
        return True

    def _update_head_follow():
        if not head_follow_enabled or teleop_device is None:
            return
        if not hasattr(teleop_device, "head_estimate"):
            if not head_warned["no_estimator"]:
                print(
                    "[run_teleop][head_follow] device has no head_estimate(); "
                    "make sure --head_follow_hmd matches the device cfg."
                )
                head_warned["no_estimator"] = True
            return
        if not head_joint_resolved[0]:
            if not _resolve_head_joint_ids():
                return
        if not head_joint_ids:
            return
        est = teleop_device.head_estimate()
        if est is None:
            return
        try:
            robot = env.scene["robot"]
            target = torch.zeros(env.num_envs, 3, device=env.device)
            target[:, 0] = float(est.yaw)
            target[:, 1] = float(est.pitch)
            target[:, 2] = float(est.roll)
            robot.set_joint_position_target(target, joint_ids=head_joint_ids)
            if not head_warned["first_target"]:
                print(
                    f"[run_teleop][head_follow] first head target applied: "
                    f"yaw={est.yaw:+.3f} pitch={est.pitch:+.3f} roll={est.roll:+.3f}"
                )
                head_warned["first_target"] = True
        except Exception as exc:  # noqa: BLE001
            if not head_warned["no_estimator"]:
                print(
                    f"[run_teleop][head_follow] set_joint_position_target raised "
                    f"{type(exc).__name__}: {exc}; disabling head follow."
                )
                head_warned["no_estimator"] = True

    def _update_follow_hmd_camera():
        if not follow_hmd_enabled or set_camera_view is None:
            return
        if teleop_device is None:
            return
        snap = teleop_device._get_raw_data() if hasattr(teleop_device, "_get_raw_data") else None
        if not snap:
            return
        hmd = snap.get("hmd")
        if not hmd:
            if not follow_hmd_warned["no_hmd"]:
                print("[run_teleop][follow_hmd] snapshot has no HMD pose yet; skipping.")
                follow_hmd_warned["no_hmd"] = True
            return
        try:
            from ust_ws.ust_hm_glove.teleop import coord_transforms as _ct
            import numpy as _np

            # Resolve head link index lazily (after env.reset has populated
            # body_names + body_state buffers).
            if follow_hmd_head_idx[0] < 0:
                follow_hmd_head_idx[0] = _resolve_head_link_idx()

            # Compute the user's HMD orientation in IL world; we use only
            # the orientation (head turns).  Position comes from the robot
            # so the camera sits at the robot's eye level, not the user's.
            _, quat_il = _ct.svr_to_isaaclab(hmd["pos"], hmd["quat"])
            forward = _quat_rotate_vec_wxyz(quat_il, _np.array([1.0, 0.0, 0.0]))
            up = _quat_rotate_vec_wxyz(quat_il, _np.array([0.0, 0.0, 1.0]))
            del up  # currently unused — reserved for set_camera_view-with-up variant

            head_idx = follow_hmd_head_idx[0]
            if head_idx >= 0:
                # Robot-anchored camera (preferred).  9.17 default behaviour.
                try:
                    robot = env.scene["robot"]
                    head_pos_w = robot.data.body_pos_w[0, head_idx]
                    eye = head_pos_w.detach().cpu().numpy().astype(_np.float64)
                except Exception as exc:  # noqa: BLE001
                    if not follow_hmd_warned["missing_api"]:
                        print(
                            f"[run_teleop][follow_hmd] body_pos_w lookup raised "
                            f"{type(exc).__name__}: {exc}; disabling further updates."
                        )
                        follow_hmd_warned["missing_api"] = True
                    return
                # Small forward+up offset so the camera doesn't clip into the
                # robot's head mesh.  Roughly 5 cm forward (in robot's IL +X)
                # and 5 cm above the link origin to land near eye height.
                eye = eye + _np.array([0.05, 0.0, 0.05])
            else:
                # Legacy fallback: place camera at HMD position in user space
                # (subtract waist XY for base_link-relative).
                pos_il, _ = _ct.svr_to_isaaclab(hmd["pos"], hmd["quat"])
                waist = (snap.get("trackers") or {}).get("waist")
                if waist:
                    wpos_il, _wq = _ct.svr_to_isaaclab(waist["pos"], waist["quat"])
                    pos_il = pos_il - _np.array([wpos_il[0], wpos_il[1], 0.0])
                eye = pos_il

            target = eye + forward
            try:
                set_camera_view(
                    eye=tuple(float(x) for x in eye),
                    target=tuple(float(x) for x in target),
                    camera_prim_path="/OmniverseKit_Persp",
                )
            except Exception as exc:  # noqa: BLE001
                if not follow_hmd_warned["missing_api"]:
                    print(
                        f"[run_teleop][follow_hmd] set_camera_view raised "
                        f"{type(exc).__name__}: {exc}.  Disabling follow_hmd."
                    )
                    follow_hmd_warned["missing_api"] = True
        except Exception as exc:  # noqa: BLE001
            if not follow_hmd_warned["missing_api"]:
                print(
                    f"[run_teleop][follow_hmd] update raised "
                    f"{type(exc).__name__}: {exc}; disabling further updates."
                )
                follow_hmd_warned["missing_api"] = True

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
                elif args.diag == "finger_sine":
                    action = _finger_sine_action(_time.perf_counter() - diag_t0)
                else:
                    teleop_action = teleop_device.advance() if teleop_device is not None else None
                    action = (
                        teleop_action.unsqueeze(0).to(env.device)
                        if teleop_action is not None
                        else idle_action
                    )
                # 9.18: head-joint follow runs BEFORE env.step so the
                # head joint targets are written into the articulation
                # this physics tick.  set_joint_position_target stages
                # the value; physx applies it on the next sim step.
                if head_follow_enabled:
                    _update_head_follow()
                obs, _rewards, terminated, truncated, _info = env.step(action)
                step += 1
                # 9.16: HMD-follow viewport camera (when --follow_hmd).
                if follow_hmd_enabled:
                    _update_follow_hmd_camera()

                # Periodic finger target-vs-actual report.  Reads
                # ``robot.data.joint_pos`` directly (bypasses the
                # observation manager's hand_joint_state which is dict-
                # keyed when ``concatenate_terms=False``), and also
                # reads ``robot.data.joint_pos_target`` to confirm the
                # commanded target actually reached the articulation.
                # This is the decisive test:
                #   * action_tgt vs joint_pos_target -> Pink IK routing
                #     (if these disagree, the action term lost the value)
                #   * joint_pos_target vs joint_pos -> physics tracking
                #     (if these disagree, joint can't move despite cmd)
                if step % 20 == 0 and hand_joint_ids:
                    try:
                        robot = env.scene["robot"]
                        joint_pos = robot.data.joint_pos[0]
                        # joint_pos_target may exist; fall back to action
                        try:
                            joint_pos_target = robot.data.joint_pos_target[0]
                            jpt_available = True
                        except AttributeError:
                            joint_pos_target = None
                            jpt_available = False
                        # action[0, 14:36] holds the 22 hand targets in the
                        # canonical FOURIER_HAND_JOINT_NAMES order, NOT in
                        # the articulation joint order.  Map by name.
                        from ust_ws.ust_hm_glove.kitchen_sorting_gr1t2_env_cfg import (
                            FOURIER_HAND_JOINT_NAMES as _HJN,
                        )
                        # Pick a representative subset to print
                        sample_names = (
                            "L_index_proximal_joint",
                            "L_thumb_proximal_yaw_joint",
                            "R_index_proximal_joint",
                            "R_thumb_proximal_yaw_joint",
                        )
                        parts = []
                        for jn in sample_names:
                            if jn not in hand_joint_names:
                                continue
                            art_idx = hand_joint_ids[hand_joint_names.index(jn)]
                            try:
                                act_idx = 14 + _HJN.index(jn)
                                tgt = float(action[0, act_idx])
                            except (ValueError, IndexError):
                                tgt = float("nan")
                            jpt = float(joint_pos_target[art_idx]) if jpt_available else float("nan")
                            act = float(joint_pos[art_idx])
                            short = jn.replace("_proximal_joint", "p").replace("_proximal_yaw_joint", "y")
                            parts.append(f"{short}: act_tgt={tgt:+.3f} jpt={jpt:+.3f} pos={act:+.3f}")
                        print(f"[FingerCmp #{step}] " + " | ".join(parts))
                    except Exception as _exc:  # noqa: BLE001
                        print(f"[FingerCmp #{step}] failed: {type(_exc).__name__}: {_exc}")

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
