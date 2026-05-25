"""Isaac Sim XR + pyopenxr piggyback probe.

Strategy
--------
The standalone headless Action API probe in
``diagnose_pyopenxr_controller.py`` succeeded at instance/session/binding
setup but actions never went ``is_active=True``.  Root cause: SteamVR's
OpenXR runtime only delivers action data to sessions in state
``XR_SESSION_STATE_FOCUSED``, and a headless session can never reach
FOCUSED (no graphics binding -> no VISIBLE -> no FOCUSED).

This piggyback variant tries to exploit that **Isaac Sim's omni.kit.xr.core
already maintains a FOCUSED XR session** when booted with ``--render_mode
steamvr_native``.  In the same process we create our own pyopenxr
``XrInstance`` + headless session in a worker thread.  SteamVR's runtime
may (or may not) share input state across instances within the same
process; this script tells us empirically.

Approach
--------
1. Boot Isaac Sim with the XR experience kit (same as run_teleop).
2. After ``sim_app.update()`` ticks have advanced enough that Isaac
   Sim's OpenXR session reached FOCUSED state, in the SAME thread:
   * Create our pyopenxr instance with Action API extensions.
   * Build action set + trigger/grip actions + suggested binding
     (oculus_touch profile is the one SteamVR accepts for PICO).
   * Attach action set to a fresh headless session.
   * Drive session through events to begin_session.
3. Inside the simulation main loop, call xrSyncActions and
   xrGetActionStateFloat once per Isaac Sim frame and log results.

If actions report is_active=True even briefly, we have a working
input path -- a production sampler can replace the broken SteamVR
Action API channel.

Usage::

    & C:\\Users\\pjwpy\\miniconda3\\envs\\ust\\python.exe -X utf8 `
        -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_piggyback `
        --render_mode steamvr_native --seconds 60
"""

from __future__ import annotations

import argparse
import ctypes
import math
import sys
import time
from pathlib import Path
from typing import Dict, Optional

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))

try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _boot_isaac_sim(render_mode: str, headless: bool):
    """Boot Isaac Sim with the XR experience kit so omni.kit.xr.core
    drives an OpenXR session through its full focus lifecycle."""
    from isaaclab.app import AppLauncher

    boot_parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(boot_parser)
    boot_args, remaining = boot_parser.parse_known_args()
    boot_args.headless = headless
    boot_args.xr = (render_mode != "monitor")
    if hasattr(boot_args, "livestream"):
        boot_args.livestream = -1
    sys.argv = [sys.argv[0]] + remaining
    return AppLauncher(boot_args).app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render_mode", default="steamvr_native",
                        choices=["monitor", "steamvr_desktop", "steamvr_native"])
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--rate", type=float, default=3.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    print(f"[boot] Isaac Sim with render_mode={args.render_mode}, headless={args.headless}")
    sim_app = _boot_isaac_sim(args.render_mode, args.headless)

    try:
        import xr

        # Wait a few ticks so Isaac Sim has time to create its own XR
        # session and reach FOCUSED -- gives us a better chance of
        # SteamVR sharing input across instances.
        print("[boot] Pumping Isaac Sim updates 30 times to settle XR session...")
        for _ in range(30):
            sim_app.update()
        print("[boot] Isaac Sim ready.")

        # ----- Our pyopenxr instance ------------------------------------
        print("\n[pyopenxr] Create instance with Action API + headless...")
        app_info = xr.ApplicationInfo(
            application_name="ust_hm_grip_piggyback",
            application_version=1,
            engine_name="ust_hm_grip",
            engine_version=1,
            api_version=xr.Version(1, 0, 0),
        )
        instance = xr.create_instance(xr.InstanceCreateInfo(
            application_info=app_info,
            enabled_extension_names=["XR_KHR_D3D11_enable", "XR_MND_headless"],
        ))
        print(f"  [OK] instance.")

        system_id = xr.get_system(instance, xr.SystemGetInfo(
            form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))
        print(f"  system_id={system_id!r}")

        session = xr.create_session(instance, xr.SessionCreateInfo(
            system_id=system_id, next=None))
        print(f"  [OK] session.")

        # Action set + actions
        action_set = xr.create_action_set(instance, xr.ActionSetCreateInfo(
            action_set_name="ust_teleop",
            localized_action_set_name="UST Teleop Controllers",
            priority=0,
        ))
        path_left = xr.string_to_path(instance, "/user/hand/left")
        path_right = xr.string_to_path(instance, "/user/hand/right")
        actions = {}
        for name in ("trigger", "grip"):
            actions[name] = xr.create_action(action_set, xr.ActionCreateInfo(
                action_name=name,
                localized_action_name=f"Controller {name}",
                action_type=xr.ActionType.FLOAT_INPUT,
                count_subaction_paths=2,
                subaction_paths=[path_left, path_right],
            ))

        ip_path = xr.string_to_path(instance, "/interaction_profiles/oculus/touch_controller")
        bindings = [
            xr.ActionSuggestedBinding(action=actions["trigger"],
                                      binding=xr.string_to_path(instance,
                                          "/user/hand/left/input/trigger/value")),
            xr.ActionSuggestedBinding(action=actions["trigger"],
                                      binding=xr.string_to_path(instance,
                                          "/user/hand/right/input/trigger/value")),
            xr.ActionSuggestedBinding(action=actions["grip"],
                                      binding=xr.string_to_path(instance,
                                          "/user/hand/left/input/squeeze/value")),
            xr.ActionSuggestedBinding(action=actions["grip"],
                                      binding=xr.string_to_path(instance,
                                          "/user/hand/right/input/squeeze/value")),
        ]
        xr.suggest_interaction_profile_bindings(instance,
            xr.InteractionProfileSuggestedBinding(
                interaction_profile=ip_path,
                count_suggested_bindings=len(bindings),
                suggested_bindings=bindings,
            ))
        print("  [OK] oculus_touch suggested bindings.")
        xr.attach_session_action_sets(session, xr.SessionActionSetsAttachInfo(
            count_action_sets=1, action_sets=[action_set]))
        print("  [OK] action set attached.")

        # Drive session -> READY
        ready = False
        last_state = -1
        for _ in range(300):
            sim_app.update()
            try:
                evt = xr.poll_event(instance)
                if evt is None:
                    time.sleep(0.01); continue
                if evt.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                    s = ctypes.cast(ctypes.byref(evt),
                                    ctypes.POINTER(xr.EventDataSessionStateChanged)).contents
                    if int(s.state) != last_state:
                        print(f"  [our session] state -> {s.state}", flush=True)
                        last_state = int(s.state)
                    if s.state == xr.SessionState.READY:
                        ready = True
                        break
            except Exception:
                time.sleep(0.01); continue
        try:
            xr.begin_session(session, xr.SessionBeginInfo(
                primary_view_configuration_type=xr.ViewConfigurationType.PRIMARY_STEREO))
            print("  [OK] begin_session.")
        except Exception as exc:
            print(f"  [WARN] begin_session: {type(exc).__name__}: {exc}")

        # ----- Poll loop in lockstep with Isaac Sim's main loop ---------
        print(f"\n[probe] Polling for {args.seconds:.0f}s.  Squeeze trigger and grip.")
        print("        Isaac Sim must be in XR mode with HMD on for FOCUSED state.\n")

        deadline = time.time() + args.seconds
        period = 1.0 / max(0.5, args.rate)
        last_print = 0.0
        n = 0
        max_seen = {"L_trig": 0.0, "L_grip": 0.0, "R_trig": 0.0, "R_grip": 0.0}
        any_active = False
        focused_seen = False

        def _drain_events():
            nonlocal focused_seen
            for _ in range(20):
                try:
                    evt = xr.poll_event(instance)
                    if evt is None: return
                    if evt.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                        s = ctypes.cast(ctypes.byref(evt),
                                        ctypes.POINTER(xr.EventDataSessionStateChanged)).contents
                        print(f"  (event) our SessionState -> {s.state}", flush=True)
                        if int(s.state) == int(xr.SessionState.FOCUSED):
                            focused_seen = True
                    elif evt.type == xr.StructureType.EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                        print(f"  (event) interaction profile changed", flush=True)
                except Exception:
                    return

        while time.time() < deadline and sim_app.is_running():
            sim_app.update()  # critical: Isaac Sim XR session keeps ticking

            now = time.time()
            if now - last_print < period:
                continue
            last_print = now
            _drain_events()

            # Frame loop on our session
            try:
                fs = xr.wait_frame(session, xr.FrameWaitInfo())
                display_time = int(fs.predicted_display_time)
                xr.begin_frame(session, xr.FrameBeginInfo())
            except Exception:
                display_time = time.perf_counter_ns()

            try:
                xr.sync_actions(session, xr.ActionsSyncInfo(
                    count_active_action_sets=1,
                    active_action_sets=[xr.ActiveActionSet(
                        action_set=action_set, subaction_path=xr.NULL_PATH)],
                ))
            except Exception as exc:
                if n == 0:
                    print(f"  [warn] sync_actions: {type(exc).__name__}: {exc}")

            def _read(action, sub):
                try:
                    st = xr.get_action_state_float(session, xr.ActionStateGetInfo(
                        action=action, subaction_path=sub))
                    return float(st.current_state), bool(st.is_active)
                except Exception:
                    return 0.0, False

            l_t, l_t_act = _read(actions["trigger"], path_left)
            l_g, l_g_act = _read(actions["grip"],    path_left)
            r_t, r_t_act = _read(actions["trigger"], path_right)
            r_g, r_g_act = _read(actions["grip"],    path_right)
            max_seen["L_trig"] = max(max_seen["L_trig"], l_t)
            max_seen["L_grip"] = max(max_seen["L_grip"], l_g)
            max_seen["R_trig"] = max(max_seen["R_trig"], r_t)
            max_seen["R_grip"] = max(max_seen["R_grip"], r_g)
            if any([l_t_act, l_g_act, r_t_act, r_g_act]):
                any_active = True

            n += 1
            print(f"  t={now - (deadline - args.seconds):5.1f}s "
                  f"L_trig={l_t:.2f}({int(l_t_act)}) L_grip={l_g:.2f}({int(l_g_act)}) | "
                  f"R_trig={r_t:.2f}({int(r_t_act)}) R_grip={r_g:.2f}({int(r_g_act)})  "
                  f"focused={focused_seen}", flush=True)

            try:
                xr.end_frame(session, xr.FrameEndInfo(
                    display_time=display_time,
                    environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE))
            except Exception:
                pass

        # ----- Summary -----
        print("\n" + "=" * 72)
        print("SUMMARY (piggyback)")
        print("=" * 72)
        print(f"  Polls                : {n}")
        print(f"  Our FOCUSED observed : {focused_seen}")
        print(f"  Any action active    : {any_active}")
        for k, v in max_seen.items():
            print(f"  max {k:6s}           : {v:.3f}")
        if any_active and max(max_seen.values()) > 0.05:
            print("\n[VERDICT] PASS -- piggyback works.")
            ret = 0
        elif focused_seen and any_active:
            print("\n[VERDICT] PARTIAL -- focused + active but values are 0.")
            ret = 0
        elif focused_seen:
            print("\n[VERDICT] FAIL -- our session reached FOCUSED but actions stayed inactive.")
            print("  PICO Connect routes input to Isaac Sim's instance only;")
            print("  same-process secondary instance does not receive it.")
            ret = 1
        else:
            print("\n[VERDICT] FAIL -- our session never reached FOCUSED.")
            print("  SteamVR refuses to focus a secondary headless session while")
            print("  Isaac Sim's instance already owns the HMD focus.")
            ret = 1

        try: xr.end_session(session)
        except Exception: pass
        try: xr.destroy_session(session)
        except Exception: pass
        try: xr.destroy_instance(instance)
        except Exception: pass
        return ret
    finally:
        try: sim_app.close()
        except Exception: pass


if __name__ == "__main__":
    sys.exit(main())
