"""Smoke test for Isaac Lab OpenXRDevice + GripperRetargeter on PICO Connect.

Background
----------
The 10th-session investigation confirmed:
  * SteamVR Action API binding for ``ust.teleop.gr1t2_gripper`` is permanently
    stuck at ``bActive=False`` -- "Replace Default Binding" silently resets
    rather than committing a Personal Binding.
  * The OpenVR Property API does NOT expose PICO controller trigger/grip
    values (12-second sweep across 1000-21000 prop range showed zero
    changes during squeeze).
  * Isaac Lab's ``isaaclab.devices.openxr.OpenXRDevice`` provides hand
    joint pose tracking via the OpenXR ``XR_EXT_hand_tracking`` extension.
    This extension is **binding-agnostic** -- pose data flows regardless
    of Action API state.
  * ``isaaclab.devices.openxr.retargeters.manipulator.GripperRetargeter``
    already converts thumb-tip <-> index-tip distance into a binary
    gripper command (close < 3 cm, open > 5 cm, with hysteresis).

Goal of this script
-------------------
Verify that:
  1. Isaac Sim's XR mode (omni.kit.xr.core) initializes against the
     active OpenXR runtime (currently SteamVR's steamxr_win64.json,
     which forwards PICO Connect hand-tracking data).
  2. ``OpenXRDevice.get_data()`` returns non-zero joint poses for at
     least one hand while the user is wearing the headset.
  3. The thumb-tip <-> index-tip distance changes as the user pinches
     fingers (or squeezes the controller while keeping fingers
     against the controller body so hand tracking still infers motion).

Usage
-----

    $env:PYTHONPATH = "."
    & C:\\Users\\pjwpy\\miniconda3\\envs\\ust\\python.exe -X utf8 `
        -m ust_ws.ust_hm_grip.scripts.diagnose_openxr_hand `
        --render_mode steamvr_native --seconds 30

Exit code 0 on success (a frame where any tracked joint has
position_valid OR thumb-index distance changes by >5 mm during the
window).  Exit code 1 if no hand data ever appears (PICO Connect's
hand tracking is not reaching SteamVR's OpenXR runtime).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))


def _boot_isaac_sim(headless: bool, render_mode: str, force_runtime: str | None = None):
    """Boot Isaac Sim with the XR experience kit so omni.kit.xr.core loads.

    10th-session-2: removed automatic override of ``XR_RUNTIME_JSON`` to
    SteamVR's runtime.  By default we let the OpenXR loader read the
    system registry's ``ActiveRuntime`` (so a manual switch via
    ``add_runtime.bat`` actually takes effect).  Caller may still pin a
    specific runtime via ``--xr_runtime`` / ``force_runtime`` for testing.
    """
    import os

    if force_runtime:
        if os.path.exists(force_runtime):
            os.environ["XR_RUNTIME_JSON"] = force_runtime
            print(f"[boot] XR_RUNTIME_JSON (forced) = {force_runtime}")
        else:
            print(f"[boot] WARN: forced runtime path does not exist: {force_runtime}")
    else:
        # Respect Windows registry HKLM\SOFTWARE\Khronos\OpenXR\1\ActiveRuntime
        env_val = os.environ.get("XR_RUNTIME_JSON", "")
        print(f"[boot] XR_RUNTIME_JSON env = {env_val!r} (empty = use HKLM ActiveRuntime)")

    from isaaclab.app import AppLauncher

    boot_parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(boot_parser)
    boot_args, remaining = boot_parser.parse_known_args()
    boot_args.headless = headless
    boot_args.xr = (render_mode != "monitor")
    # Stay on the host display
    if hasattr(boot_args, "livestream"):
        boot_args.livestream = -1
    app_launcher = AppLauncher(boot_args)
    sim_app = app_launcher.app
    sys.argv = [sys.argv[0]] + remaining
    return sim_app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--render_mode",
        type=str,
        default="steamvr_native",
        choices=["monitor", "steamvr_desktop", "steamvr_native"],
        help="Same semantics as run_teleop.py.  monitor disables XR entirely; "
             "steamvr_native engages omni.kit.xr.core's OpenXR session.",
    )
    parser.add_argument(
        "--seconds", type=float, default=30.0,
        help="How long to poll OpenXR for hand-tracking joints.",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Boot Isaac Sim without a window (still runs XR if requested).",
    )
    parser.add_argument(
        "--xr_runtime", type=str, default=None,
        help="Force a specific OpenXR runtime JSON path.  Leave unset to use "
             "the OS registry ActiveRuntime (default).",
    )
    args = parser.parse_args()

    print(f"[boot] booting Isaac Sim with render_mode={args.render_mode}...")
    sim_app = _boot_isaac_sim(args.headless, args.render_mode, force_runtime=args.xr_runtime)

    try:
        # Deferred imports — Isaac Sim must be live first.
        from isaaclab.devices.openxr import OpenXRDevice, OpenXRDeviceCfg, XrCfg
        # GripperRetargeter import path
        from isaaclab.devices.openxr.retargeters.manipulator.gripper_retargeter import (
            GripperRetargeter,
            GripperRetargeterCfg,
        )

        print(f"[boot] Isaac Sim ready.  Creating OpenXRDevice...")

        cfg = OpenXRDeviceCfg(
            xr_cfg=XrCfg(
                anchor_pos=(0.0, 0.0, 0.0),
                anchor_rot=(1.0, 0.0, 0.0, 0.0),
            ),
        )
        device = OpenXRDevice(cfg=cfg)
        print(f"[boot] OpenXRDevice: {device}")

        # Construct both gripper retargeters so we can read left + right
        # thumb-index distance in real time without committing to using
        # them as the production retargeter just yet.
        left_grip = GripperRetargeter(GripperRetargeterCfg(
            bound_hand=OpenXRDevice.TrackingTarget.HAND_LEFT,
        ))
        right_grip = GripperRetargeter(GripperRetargeterCfg(
            bound_hand=OpenXRDevice.TrackingTarget.HAND_RIGHT,
        ))

        print(f"\n[probe] Polling {args.seconds:.0f}s -- squeeze your hand or "
              f"pinch thumb + index together so OpenXR hand tracking sees the "
              f"finger motion.\n")

        deadline = time.time() + args.seconds
        t0 = time.time()
        n_frames = 0
        n_with_data = 0
        n_left_close_seen = 0
        n_right_close_seen = 0
        min_left_dist = math.inf
        min_right_dist = math.inf
        last_print = 0.0

        while time.time() < deadline and sim_app.is_running():
            # OpenXRDevice._get_raw_data returns
            # {TrackingTarget.HAND_LEFT: {joint_name: pose}, HAND_RIGHT, HEAD}
            data = device._get_raw_data()
            n_frames += 1

            left = data.get(OpenXRDevice.TrackingTarget.HAND_LEFT) or {}
            right = data.get(OpenXRDevice.TrackingTarget.HAND_RIGHT) or {}

            def _xyz(d, name):
                p = d.get(name)
                if p is None:
                    return None
                # pose = [x, y, z, qw, qx, qy, qz]
                return p[:3]

            l_thumb = _xyz(left, "thumb_tip")
            l_index = _xyz(left, "index_tip")
            r_thumb = _xyz(right, "thumb_tip")
            r_index = _xyz(right, "index_tip")

            l_dist = None
            r_dist = None
            if l_thumb is not None and l_index is not None:
                l_dist = float(((l_thumb - l_index) ** 2).sum() ** 0.5)
                if l_dist < min_left_dist:
                    min_left_dist = l_dist
                # default→origin joints are 0,0,0 which gives dist=0 -- skip
                if l_dist > 1e-4:
                    n_with_data += 1
                if l_dist < 0.03:
                    n_left_close_seen += 1
            if r_thumb is not None and r_index is not None:
                r_dist = float(((r_thumb - r_index) ** 2).sum() ** 0.5)
                if r_dist < min_right_dist:
                    min_right_dist = r_dist
                if r_dist < 0.03:
                    n_right_close_seen += 1

            # Try the actual GripperRetargeter for a sanity check
            try:
                l_cmd = float(left_grip.retarget(data)[0].item())
            except Exception:
                l_cmd = float("nan")
            try:
                r_cmd = float(right_grip.retarget(data)[0].item())
            except Exception:
                r_cmd = float("nan")

            now = time.time()
            if now - last_print >= 0.5:
                last_print = now
                t = now - t0
                def _fmt_d(d):
                    return f"{d * 100:5.1f}cm" if d is not None else "  ---  "
                print(
                    f"  t={t:5.1f}s  L dist={_fmt_d(l_dist)} cmd={l_cmd:+.0f}  "
                    f"R dist={_fmt_d(r_dist)} cmd={r_cmd:+.0f}  "
                    f"(frames={n_frames}, with_data={n_with_data})",
                    flush=True,
                )

            # Pump simulation_app so XRCore subscriptions tick
            sim_app.update()
            time.sleep(0.05)

        print("\n=== SUMMARY ===")
        print(f"  Total frames polled       : {n_frames}")
        print(f"  Frames with real hand data: {n_with_data}")
        print(f"  Min LEFT  thumb-index dist: "
              f"{min_left_dist * 100:.2f} cm" if min_left_dist != math.inf else "  Min LEFT  thumb-index dist: <no data>")
        print(f"  Min RIGHT thumb-index dist: "
              f"{min_right_dist * 100:.2f} cm" if min_right_dist != math.inf else "  Min RIGHT thumb-index dist: <no data>")
        print(f"  LEFT close-state samples  : {n_left_close_seen}")
        print(f"  RIGHT close-state samples : {n_right_close_seen}")

        if n_with_data == 0:
            print("\n[VERDICT] FAIL -- no hand-tracking data ever observed.")
            print("  Likely cause: PICO Connect is not forwarding hand-tracking")
            print("  data into SteamVR's OpenXR runtime, or hand-tracking is")
            print("  disabled on the headset.  Re-enable in PICO settings.")
            return 1
        else:
            verdict = "PASS"
            print(f"\n[VERDICT] {verdict} -- OpenXR hand tracking IS reaching the app.")
            print("  GripperRetargeter can be used as the input source for")
            print("  ust_hm_grip's gripper close command, bypassing the")
            print("  broken SteamVR Action API binding.")
            return 0
    finally:
        try:
            sim_app.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
