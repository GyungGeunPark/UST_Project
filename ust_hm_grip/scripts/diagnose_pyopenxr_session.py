"""Pyopenxr headless session + hand tracking probe.

Phase 2 of the option-B-proper investigation.  After
``diagnose_pyopenxr_probe`` confirmed that the active OpenXR runtime
(SteamVR/OpenXR 2.15.6) supports both ``XR_EXT_hand_tracking`` and
``XR_MND_headless``, this script attempts to:

  1. Create an ``xr.Instance`` with both extensions enabled.
  2. Get system + headless session (no graphics binding).
  3. Begin a minimal frame loop (xrWaitFrame / xrBeginFrame / xrEndFrame).
  4. Create an ``XrHandTrackerEXT`` for each hand.
  5. Poll ``xrLocateHandJointsEXT`` for ~15 seconds while the user
     pinches thumb + index fingers together.
  6. Report whether real joint data is observed (position_valid set,
     non-zero positions, thumb-index distance variation).

The verdict tells us whether the SteamVR OpenXR runtime is actually
delivering PICO hand-tracking data, independent of Isaac Sim's wrapper.

This script is destructive only in that it creates a parallel OpenXR
session.  If Isaac Sim's omni.kit.xr.core is already running with the
HMD claimed, this script may fail (or coexist via headless mode --
SteamVR's behaviour with concurrent sessions is empirically unstable).

Usage::

    & C:\\Users\\pjwpy\\miniconda3\\envs\\ust\\python.exe -X utf8 `
        -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_session --seconds 15
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List

try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _explain_exc(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15.0,
                        help="Listening window for joint data.")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="Poll rate Hz (default 10).")
    args = parser.parse_args()

    import xr

    print("=" * 70)
    print(f"pyopenxr session probe (pyopenxr {getattr(xr, '__version__', '?')})")
    print("=" * 70)

    # ----- Instance -----------------------------------------------------
    print("\n[step 1] Creating xr.Instance with hand_tracking + headless...")
    app_info = xr.ApplicationInfo(
        application_name="ust_hm_grip_session_probe",
        application_version=1,
        engine_name="ust_hm_grip",
        engine_version=1,
        api_version=xr.Version(1, 0, 0),
    )
    try:
        instance = xr.create_instance(
            xr.InstanceCreateInfo(
                application_info=app_info,
                enabled_extension_names=[
                    "XR_KHR_D3D11_enable",
                    "XR_EXT_hand_tracking",
                    "XR_MND_headless",
                ],
            )
        )
        print(f"  [OK] Instance: {instance}")
    except Exception as exc:
        print(f"  [FATAL] create_instance: {_explain_exc(exc)}")
        return 2

    try:
        # ----- System ---------------------------------------------------
        print("\n[step 2] xrGetSystem (HEAD_MOUNTED_DISPLAY)...")
        try:
            system_id = xr.get_system(
                instance,
                xr.SystemGetInfo(form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY),
            )
            print(f"  [OK] system_id obtained: {system_id!r}")
        except Exception as exc:
            print(f"  [FATAL] get_system: {_explain_exc(exc)}")
            print("    HMD may be asleep -- wake the PICO and start PCVR.")
            return 1

        # ----- Headless session (no graphics binding) -------------------
        print("\n[step 3] xrCreateSession (XR_MND_headless, no graphics binding)...")
        try:
            session = xr.create_session(
                instance,
                xr.SessionCreateInfo(
                    system_id=system_id,
                    next=None,   # headless = no graphics binding chained in
                ),
            )
            print(f"  [OK] Session: {session}")
        except Exception as exc:
            print(f"  [FATAL] create_session: {_explain_exc(exc)}")
            print("    SteamVR may require a graphics binding even with headless.")
            return 1

        # ----- Reference space (LOCAL) ---------------------------------
        print("\n[step 4] xrCreateReferenceSpace (LOCAL)...")
        try:
            space = xr.create_reference_space(
                session,
                xr.ReferenceSpaceCreateInfo(
                    reference_space_type=xr.ReferenceSpaceType.LOCAL,
                    pose_in_reference_space=xr.Posef(
                        orientation=xr.Quaternionf(x=0, y=0, z=0, w=1),
                        position=xr.Vector3f(x=0, y=0, z=0),
                    ),
                ),
            )
            print(f"  [OK] Reference space (LOCAL) created.")
        except Exception as exc:
            print(f"  [FATAL] create_reference_space: {_explain_exc(exc)}")
            return 1

        # ----- xrBeginSession (after state -> READY) -------------------
        # OpenXR state machine: IDLE -> READY -> SYNCHRONIZED -> VISIBLE -> FOCUSED.
        # We must poll events until we see READY before xrBeginSession is legal.
        print("\n[step 5a] Polling events until SessionState=READY...")
        ready = False
        for _ in range(200):  # ~2 s budget
            try:
                evt = xr.poll_event(instance)
                if evt is None:
                    time.sleep(0.01)
                    continue
                # poll_event returns an EventDataBuffer; the actual event type is in evt.type
                if evt.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                    # cast EventDataBuffer -> EventDataSessionStateChanged
                    state_evt = xr.cast(
                        xr.byref(evt),
                        xr.POINTER(xr.EventDataSessionStateChanged),
                    ).contents
                    print(f"  state change -> {state_evt.state}")
                    if state_evt.state == xr.SessionState.READY:
                        ready = True
                        break
            except Exception as exc:
                # pyopenxr's poll_event raises EventUnavailable when queue is empty;
                # this is normal -- treat as "no event" and continue.
                if "EventUnavailable" in type(exc).__name__:
                    time.sleep(0.01)
                    continue
                if "ResultException" in type(exc).__name__ or "Result" in type(exc).__name__:
                    time.sleep(0.01)
                    continue
                print(f"  [WARN] poll_event: {_explain_exc(exc)}")
                time.sleep(0.05)
        if not ready:
            print("  [WARN] never reached SessionState.READY -- attempting begin_session anyway.")
        else:
            print("  [OK] SessionState.READY observed.")

        print("\n[step 5b] xrBeginSession...")
        try:
            xr.begin_session(
                session,
                xr.SessionBeginInfo(
                    primary_view_configuration_type=xr.ViewConfigurationType.PRIMARY_STEREO,
                ),
            )
            print(f"  [OK] Session BEGIN issued.")
        except Exception as exc:
            print(f"  [WARN] begin_session: {_explain_exc(exc)}")
            print("    Continuing -- some runtimes accept locate without strict frame loop.")

        # ----- Hand trackers --------------------------------------------
        print("\n[step 6] xrCreateHandTrackerEXT (left + right)...")
        hand_trackers = {}
        try:
            for side, hand_enum in (
                ("left", xr.HandEXT.LEFT),
                ("right", xr.HandEXT.RIGHT),
            ):
                ht = xr.create_hand_tracker_ext(
                    session,
                    xr.HandTrackerCreateInfoEXT(
                        hand=hand_enum,
                        hand_joint_set=xr.HandJointSetEXT.DEFAULT,
                    ),
                )
                hand_trackers[side] = ht
                print(f"  [OK] {side} hand_tracker created: {ht}")
        except Exception as exc:
            print(f"  [FATAL] create_hand_tracker_ext: {_explain_exc(exc)}")
            print("    The runtime advertised XR_EXT_hand_tracking but rejected")
            print("    creating an actual tracker.  Likely PICO Connect is not")
            print("    forwarding hand-tracking through this OpenXR path.")
            try:
                xr.destroy_session(session)
            except Exception:
                pass
            return 1

        # ----- Poll loop --------------------------------------------------
        print(f"\n[step 7] Polling hand joints for {args.seconds:.0f}s.  Squeeze")
        print(f"          thumb + index together (pinch) repeatedly.")
        print(f"          NOTE: open / closed pinch gives 5cm <-> 1cm distance change.\n")

        # Pre-allocate JointLocationEXT arrays (26 joints per hand)
        # pyopenxr exposes XR_HAND_JOINT_COUNT_EXT = 26
        n_joints = xr.HAND_JOINT_COUNT_EXT

        deadline = time.time() + args.seconds
        period = 1.0 / max(1.0, args.rate)
        last_print = 0.0
        n_polls = 0
        n_left_valid = 0
        n_right_valid = 0
        min_left_dist = math.inf
        max_left_dist = -math.inf
        min_right_dist = math.inf
        max_right_dist = -math.inf

        # The display_time for xrLocateHandJointsEXT can come from a
        # synchronized xrWaitFrame; in a pure headless session we use
        # the current performance counter converted to XrTime via
        # xrConvertWin32PerformanceCounterToTimeKHR (not enabled).
        # Workaround: use display_time=0 (some runtimes accept) or
        # query system time via the extension we forgot to enable.
        # Try with 0 first; if that returns invalid, switch to a
        # cumulative counter so each frame advances.
        last_print = 0.0
        t0 = time.perf_counter_ns()

        # Drain any pending session state events; if state advances to
        # SYNCHRONIZED / VISIBLE / FOCUSED we'll log it.
        def _drain_events():
            for _ in range(20):
                try:
                    evt = xr.poll_event(instance)
                    if evt is None:
                        return
                    if evt.type == xr.StructureType.EVENT_DATA_SESSION_STATE_CHANGED:
                        s = xr.cast(
                            xr.byref(evt),
                            xr.POINTER(xr.EventDataSessionStateChanged),
                        ).contents.state
                        print(f"  (event) SessionState -> {s}", flush=True)
                except Exception as exc:
                    if "EventUnavailable" in type(exc).__name__ or "Result" in type(exc).__name__:
                        return
                    return

        while time.time() < deadline:
            now = time.time()
            if now - last_print < period:
                time.sleep(0.005)
                continue
            last_print = now

            _drain_events()

            # Critical: must call xrWaitFrame + xrBeginFrame for predicted
            # display_time even in headless mode.  SteamVR rejects locate
            # calls outside a frame as XR_ERROR_VALIDATION_FAILURE.
            try:
                frame_state = xr.wait_frame(session, xr.FrameWaitInfo())
                display_time = int(frame_state.predicted_display_time)
                xr.begin_frame(session, xr.FrameBeginInfo())
            except Exception as exc:
                # If wait/begin fail (e.g. session state not yet RUNNING)
                # fall back to a stamp from our perf counter so we can
                # still see if anything responds.
                display_time = time.perf_counter_ns() - t0
                if display_time <= 0:
                    display_time = 1
                if n_polls == 0:
                    print(f"  [warn] wait/begin_frame: {_explain_exc(exc)}")

            results = {}
            for side, ht in hand_trackers.items():
                try:
                    locate_info = xr.HandJointsLocateInfoEXT(
                        base_space=space,
                        time=display_time,
                    )
                    locations = xr.locate_hand_joints_ext(ht, locate_info)
                    # locations is a wrapped structure: .is_active + .joint_locations (array of 26)
                    is_active = bool(locations.is_active)
                    joints = locations.joint_locations  # list-like of XrHandJointLocationEXT
                    results[side] = (is_active, joints)
                except Exception as exc:
                    results[side] = (False, None)
                    # Print first time only to avoid spam
                    if n_polls == 0:
                        print(f"  [poll-fail {side}] {_explain_exc(exc)}")

            # Extract thumb_tip (joint 5) and index_tip (joint 10) per OpenXR spec:
            #  0 PALM, 1 WRIST, 2 THUMB_METACARPAL, 3 PROXIMAL, 4 DISTAL, 5 TIP
            #  6 INDEX_METACARPAL, 7 PROXIMAL, 8 INTERMEDIATE, 9 DISTAL, 10 TIP
            THUMB_TIP = 5
            INDEX_TIP = 10
            POSITION_VALID = xr.SPACE_LOCATION_POSITION_VALID_BIT

            def _xyz(joints, idx):
                j = joints[idx]
                if not (j.location_flags & POSITION_VALID):
                    return None
                p = j.pose.position
                return (float(p.x), float(p.y), float(p.z))

            n_polls += 1
            l_active, l_joints = results.get("left", (False, None))
            r_active, r_joints = results.get("right", (False, None))
            l_dist = r_dist = None
            if l_active and l_joints is not None:
                l_t = _xyz(l_joints, THUMB_TIP)
                l_i = _xyz(l_joints, INDEX_TIP)
                if l_t and l_i:
                    n_left_valid += 1
                    l_dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(l_t, l_i)))
                    min_left_dist = min(min_left_dist, l_dist)
                    max_left_dist = max(max_left_dist, l_dist)
            if r_active and r_joints is not None:
                r_t = _xyz(r_joints, THUMB_TIP)
                r_i = _xyz(r_joints, INDEX_TIP)
                if r_t and r_i:
                    n_right_valid += 1
                    r_dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(r_t, r_i)))
                    min_right_dist = min(min_right_dist, r_dist)
                    max_right_dist = max(max_right_dist, r_dist)

            def _fmt(d): return "  ----" if d is None else f"{d * 100:5.1f}cm"
            print(
                f"  t={now - (deadline - args.seconds):5.1f}s "
                f"L_active={int(l_active)} dist={_fmt(l_dist)} | "
                f"R_active={int(r_active)} dist={_fmt(r_dist)} | "
                f"polls={n_polls}",
                flush=True,
            )

            # Close the frame — even headless sessions must call xrEndFrame
            try:
                xr.end_frame(
                    session,
                    xr.FrameEndInfo(
                        display_time=display_time,
                        environment_blend_mode=xr.EnvironmentBlendMode.OPAQUE,
                        # No layers in headless mode
                    ),
                )
            except Exception as exc:
                if n_polls == 1:
                    print(f"  [warn] end_frame: {_explain_exc(exc)}")

        # ----- Summary --------------------------------------------------
        print("\n" + "=" * 70)
        print("SUMMARY")
        print("=" * 70)
        print(f"  Total polls           : {n_polls}")
        print(f"  Left  joint reads     : {n_left_valid}  (min/max dist: "
              f"{(min_left_dist if min_left_dist != math.inf else float('nan')) * 100:.2f}/"
              f"{(max_left_dist if max_left_dist != -math.inf else float('nan')) * 100:.2f} cm)")
        print(f"  Right joint reads     : {n_right_valid} (min/max dist: "
              f"{(min_right_dist if min_right_dist != math.inf else float('nan')) * 100:.2f}/"
              f"{(max_right_dist if max_right_dist != -math.inf else float('nan')) * 100:.2f} cm)")

        if n_left_valid > 0 or n_right_valid > 0:
            l_var = (max_left_dist - min_left_dist) if max_left_dist > -math.inf else 0.0
            r_var = (max_right_dist - min_right_dist) if max_right_dist > -math.inf else 0.0
            print(f"\n[VERDICT] PASS -- hand-tracking joint data IS reaching the app.")
            print(f"  Left  variation : {l_var * 100:.2f} cm  (>2 cm = pinch detected)")
            print(f"  Right variation : {r_var * 100:.2f} cm")
            print("  Next step: implement openxr_hand_sampler.py as a")
            print("  background-thread replacement for the broken Action API path.")
            ret = 0
        else:
            print("\n[VERDICT] FAIL -- xrLocateHandJointsEXT returned no valid")
            print("  joints across the full window.  The OpenXR runtime accepted")
            print("  the hand tracker, but PICO Connect's streaming layer is not")
            print("  publishing hand-tracking data to this OpenXR session.")
            print("  Direct pyopenxr approach is also dead for this combination.")
            ret = 1

        # Cleanup
        for ht in hand_trackers.values():
            try:
                xr.destroy_hand_tracker_ext(ht)
            except Exception:
                pass
        try:
            xr.end_session(session)
        except Exception:
            pass
        try:
            xr.destroy_space(space)
        except Exception:
            pass
        try:
            xr.destroy_session(session)
        except Exception:
            pass
        return ret
    finally:
        try:
            xr.destroy_instance(instance)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
