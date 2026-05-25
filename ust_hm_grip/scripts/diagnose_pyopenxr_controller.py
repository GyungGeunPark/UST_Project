"""Pyopenxr Action API probe for PICO controller trigger/grip.

Why this is the RIGHT diagnostic
--------------------------------
The earlier hand-tracking probes (Phase 1 / 1.5 / pyopenxr_session)
all failed.  The user pointed out that those failures were **expected**
because PICO Connect's hand tracking is OFF in their environment --
they are using controllers exclusively.

The actual unexplored channel is OpenXR's **Action API for controllers**:

  * ``xrCreateAction(FLOAT_INPUT)`` for trigger / grip
  * ``xrSuggestInteractionProfileBindings`` with a PICO controller profile
    path -- e.g. ``/interaction_profiles/bytedance/pico_neo3_controller``
  * ``xrAttachSessionActionSets`` + ``xrSyncActions`` in a frame loop
  * ``xrGetActionStateFloat`` to read live trigger/grip values

This path is **completely independent** of:
  * SteamVR's Action System Personal Binding commit (which is broken
    for our app -- see option A blocked)
  * SteamVR Action Manifest registration (also broken)
  * Hand tracking extension (off by user choice)

It uses OpenXR's own action binding registry, which the OpenXR runtime
maintains separately from SteamVR's binding UI.  If PICO's SteamVR
add-on populates the OpenXR controller channel (it should -- this is
how Quest/Vive/Index work with SteamVR/OpenXR), trigger and grip
values will flow into our app via this API.

Strategy
--------
Try several interaction profile paths in sequence and report which
one accepts our suggested binding without ``PathUnsupported`` error.

Usage::

    & C:\\Users\\pjwpy\\miniconda3\\envs\\ust\\python.exe -X utf8 `
        -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_controller `
        --seconds 18
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


# PICO 4 Ultra/4S controllers may be reported under several interaction
# profile paths depending on SteamVR / PICO Connect version.  We try
# each in turn until one succeeds.
INTERACTION_PROFILE_CANDIDATES = [
    # Bytedance is PICO's parent company; standard PICO profiles
    "/interaction_profiles/bytedance/pico_neo3_controller",
    "/interaction_profiles/bytedance/pico4_controller",
    "/interaction_profiles/bytedance/pico4s_controller",
    # PICO Connect may also expose these
    "/interaction_profiles/oculus/touch_controller",
    "/interaction_profiles/khr/simple_controller",
    "/interaction_profiles/htc/vive_controller",
]


def _explain_exc(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=18.0)
    parser.add_argument("--rate", type=float, default=5.0,
                        help="poll rate in Hz")
    args = parser.parse_args()

    import xr

    print("=" * 72)
    print(f"pyopenxr CONTROLLER action probe (pyopenxr {getattr(xr, '__version__', '?')})")
    print("=" * 72)

    # ----- Instance -----------------------------------------------------
    print("\n[step 1] Create instance with D3D11 + headless...")
    app_info = xr.ApplicationInfo(
        application_name="ust_hm_grip_ctrl_probe",
        application_version=1,
        engine_name="ust_hm_grip",
        engine_version=1,
        api_version=xr.Version(1, 0, 0),
    )
    try:
        instance = xr.create_instance(xr.InstanceCreateInfo(
            application_info=app_info,
            enabled_extension_names=["XR_KHR_D3D11_enable", "XR_MND_headless"],
        ))
        print("  [OK] instance created.")
    except Exception as exc:
        print(f"  [FATAL] {_explain_exc(exc)}")
        return 2

    try:
        # Runtime info
        try:
            props = xr.get_instance_properties(instance)
            rn = props.runtime_name
            if isinstance(rn, bytes):
                rn = rn.decode("utf-8", errors="replace")
            print(f"  runtime: {rn!r} v{props.runtime_version.major}."
                  f"{props.runtime_version.minor}.{props.runtime_version.patch}")
        except Exception:
            pass

        # System
        try:
            system_id = xr.get_system(instance, xr.SystemGetInfo(
                form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))
            print(f"  system_id={system_id!r}")
        except Exception as exc:
            print(f"  [FATAL] get_system: {_explain_exc(exc)}")
            return 1

        # Session
        try:
            session = xr.create_session(instance, xr.SessionCreateInfo(
                system_id=system_id, next=None))
            print("  [OK] session created (headless).")
        except Exception as exc:
            print(f"  [FATAL] create_session: {_explain_exc(exc)}")
            return 1

        # ----- Action Set + Actions -----------------------------------
        print("\n[step 2] Create action set + per-side trigger/grip actions...")
        action_set = xr.create_action_set(instance, xr.ActionSetCreateInfo(
            action_set_name="ust_teleop",
            localized_action_set_name="UST Teleop Controllers",
            priority=0,
        ))

        # Subaction paths so we can bind per-hand
        path_left = xr.string_to_path(instance, "/user/hand/left")
        path_right = xr.string_to_path(instance, "/user/hand/right")
        subactions = [path_left, path_right]

        # Create actions: trigger (float) and grip (float) per hand, plus
        # a grip-pose for placing pose verification.
        actions = {}
        for name, atype in [
            ("trigger", xr.ActionType.FLOAT_INPUT),
            ("grip",    xr.ActionType.FLOAT_INPUT),
        ]:
            a = xr.create_action(action_set, xr.ActionCreateInfo(
                action_name=name,
                localized_action_name=f"Controller {name}",
                action_type=atype,
                count_subaction_paths=len(subactions),
                subaction_paths=subactions,
            ))
            actions[name] = a
            print(f"  [OK] action '{name}' created with subactions L+R")

        # ----- Suggest bindings for each candidate interaction profile -----
        print("\n[step 3] Try suggested bindings for each interaction profile...")
        accepted_profile = None
        for profile in INTERACTION_PROFILE_CANDIDATES:
            try:
                ip_path = xr.string_to_path(instance, profile)
            except Exception as exc:
                print(f"  [skip] {profile}: string_to_path failed ({exc})")
                continue
            # Suggested bindings: paths follow OpenXR spec
            #   trigger value: .../input/trigger/value (analog 0-1)
            #   grip value   : .../input/squeeze/value (analog 0-1, "squeeze" is OpenXR's term for grip)
            try:
                trig_left = xr.string_to_path(instance, "/user/hand/left/input/trigger/value")
                trig_right = xr.string_to_path(instance, "/user/hand/right/input/trigger/value")
                squeeze_left = xr.string_to_path(instance, "/user/hand/left/input/squeeze/value")
                squeeze_right = xr.string_to_path(instance, "/user/hand/right/input/squeeze/value")
            except Exception as exc:
                print(f"  [skip] {profile}: subpath resolution failed ({exc})")
                continue

            bindings = [
                xr.ActionSuggestedBinding(action=actions["trigger"], binding=trig_left),
                xr.ActionSuggestedBinding(action=actions["trigger"], binding=trig_right),
                xr.ActionSuggestedBinding(action=actions["grip"],    binding=squeeze_left),
                xr.ActionSuggestedBinding(action=actions["grip"],    binding=squeeze_right),
            ]
            try:
                xr.suggest_interaction_profile_bindings(instance,
                    xr.InteractionProfileSuggestedBinding(
                        interaction_profile=ip_path,
                        count_suggested_bindings=len(bindings),
                        suggested_bindings=bindings,
                    ))
                accepted_profile = profile
                print(f"  [OK] runtime accepted: {profile}")
                # Some runtimes accept multiple profiles -- keep trying so
                # we can also bind oculus_touch as a compatibility fallback.
            except Exception as exc:
                msg = str(exc)
                short = msg[:80] + ("..." if len(msg) > 80 else "")
                print(f"  [reject] {profile}: {type(exc).__name__}: {short}")

        if accepted_profile is None:
            print("\n  No interaction profile was accepted -- this OpenXR runtime")
            print("  does not advertise any PICO/Oculus/SimpleController profile.")
            print("  Cannot proceed with Action API path.")
            return 1

        # ----- Attach action set to session -----
        print(f"\n[step 4] xrAttachSessionActionSets...")
        try:
            xr.attach_session_action_sets(session, xr.SessionActionSetsAttachInfo(
                count_action_sets=1,
                action_sets=[action_set],
            ))
            print("  [OK] attached.")
        except Exception as exc:
            print(f"  [FATAL] {_explain_exc(exc)}")
            return 1

        # ----- Drive session state to READY then BEGIN -----
        print("\n[step 5] Drive session -> READY -> begin_session...")
        ready = False
        for _ in range(200):
            try:
                evt = xr.poll_event(instance)
                if evt is None:
                    time.sleep(0.01); continue
                if evt.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                    import ctypes as _ct
                    s = _ct.cast(_ct.byref(evt),
                                 _ct.POINTER(xr.EventDataSessionStateChanged)).contents
                    print(f"  state -> {s.state}")
                    if s.state == xr.SessionState.READY:
                        ready = True
                        break
            except Exception as exc:
                if "EventUnavailable" in type(exc).__name__ or "Result" in type(exc).__name__:
                    time.sleep(0.01); continue
                print(f"  [WARN] poll_event: {exc}"); time.sleep(0.05)
        if not ready:
            print("  [WARN] never observed READY -- begin_session may fail.")
        try:
            xr.begin_session(session, xr.SessionBeginInfo(
                primary_view_configuration_type=xr.ViewConfigurationType.PRIMARY_STEREO,
            ))
            print("  [OK] begin_session.")
        except Exception as exc:
            print(f"  [WARN] begin_session: {exc}")

        # ----- Poll loop with sync_actions -----
        print(f"\n[step 6] Polling controller actions for {args.seconds:.0f}s -- ")
        print("          squeeze trigger and grip on both controllers.\n")

        n_polls = 0
        max_seen = {"L_trig": 0.0, "L_grip": 0.0, "R_trig": 0.0, "R_grip": 0.0}
        any_active = False
        deadline = time.time() + args.seconds
        period = 1.0 / max(0.5, float(args.rate))
        last_print = 0.0
        t_start_polls = time.time()

        def _drain_events():
            for _ in range(20):
                try:
                    evt = xr.poll_event(instance)
                    if evt is None: return
                    if evt.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                        s = xr.cast(xr.byref(evt),
                                    xr.POINTER(xr.EventDataSessionStateChanged)).contents
                        print(f"  (event) SessionState -> {s.state}", flush=True)
                    elif evt.type == xr.StructureType.EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                        print(f"  (event) interaction profile changed", flush=True)
                except Exception:
                    return

        while time.time() < deadline:
            now = time.time()
            if now - last_print < period:
                time.sleep(0.005); continue
            last_print = now
            _drain_events()

            # frame loop
            try:
                fs = xr.wait_frame(session, xr.FrameWaitInfo())
                display_time = int(fs.predicted_display_time)
                xr.begin_frame(session, xr.FrameBeginInfo())
            except Exception:
                display_time = time.perf_counter_ns()

            # sync actions
            try:
                xr.sync_actions(session, xr.ActionsSyncInfo(
                    count_active_action_sets=1,
                    active_action_sets=[xr.ActiveActionSet(
                        action_set=action_set,
                        subaction_path=xr.NULL_PATH,
                    )],
                ))
            except Exception as exc:
                if n_polls == 0:
                    print(f"  [warn] sync_actions: {exc}")

            def _read(action, sub):
                try:
                    st = xr.get_action_state_float(session, xr.ActionStateGetInfo(
                        action=action, subaction_path=sub,
                    ))
                    return float(st.current_state), bool(st.is_active)
                except Exception as exc:
                    if n_polls == 0:
                        print(f"  [warn] get_action_state_float: {exc}")
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

            n_polls += 1
            if n_polls % 3 == 1:
                print(f"  t={now - t_start_polls:5.1f}s "
                      f"L_trig={l_t:.2f}({int(l_t_act)}) L_grip={l_g:.2f}({int(l_g_act)}) | "
                      f"R_trig={r_t:.2f}({int(r_t_act)}) R_grip={r_g:.2f}({int(r_g_act)})",
                      flush=True)

            # close frame
            try:
                xr.end_frame(session, xr.FrameEndInfo(
                    display_time=display_time,
                    environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
                ))
            except Exception:
                pass

        # ----- Summary -----
        print("\n" + "=" * 72)
        print("SUMMARY")
        print("=" * 72)
        print(f"  Total polls            : {n_polls}")
        print(f"  Accepted profile       : {accepted_profile}")
        print(f"  Any action is_active   : {any_active}")
        print(f"  Max values seen        :")
        for k, v in max_seen.items():
            print(f"    {k:>6} = {v:.3f}")

        if any_active and max(max_seen.values()) > 0.05:
            print("\n[VERDICT] PASS -- OpenXR Action API delivers real PICO")
            print("  controller trigger/grip values.  Direct pyopenxr controller")
            print("  binding bypasses SteamVR's broken Personal Binding pipeline.")
            ret = 0
        elif any_active:
            print("\n[VERDICT] PARTIAL -- actions report is_active=True but all")
            print("  values stayed at 0.  User likely did not squeeze; re-run.")
            ret = 0
        else:
            print("\n[VERDICT] FAIL -- actions never went active.")
            print("  PICO Connect's prism driver does not bridge controller")
            print("  input to the OpenXR Action API on this runtime.")
            print("  Direct pyopenxr controller binding is also dead.")
            ret = 1

        # cleanup
        try: xr.end_session(session)
        except Exception: pass
        try: xr.destroy_action_set(action_set)
        except Exception: pass
        try: xr.destroy_session(session)
        except Exception: pass
        return ret
    finally:
        try: xr.destroy_instance(instance)
        except Exception: pass


if __name__ == "__main__":
    sys.exit(main())
