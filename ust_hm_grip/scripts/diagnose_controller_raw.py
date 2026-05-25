"""Controller-input diagnostic -- probes BOTH legacy and Action Manifest paths.

The pre-9.29 version of this script tried to "bypass the Action Manifest"
by calling ``IVRSystem::getControllerState()`` directly.  The premise was:
if the legacy API returns 0, SteamVR is not receiving button data at all,
so binding configuration cannot be the problem.

That premise is **wrong** for PICO Touch via Virtual Desktop / PICO Connect.
These drivers only populate the modern SteamVR Input action system; the
legacy controller-state struct stays empty regardless of how hard the user
squeezes.  Fix: probe **both** paths and let the Action API have the final
word.

* Legacy ``getControllerState()`` is still printed (column ``L_*``) for
  backwards-compat / Vive Wand / Index detection, but its silence is now
  treated as informational, not diagnostic.
* Action Manifest API (``getAnalogActionData`` / ``getDigitalActionData``,
  column ``A_*``) is authoritative.  9.39 adds the ``bActive`` flag (a0/a1)
  per channel so the user can distinguish "Personal Binding never applied
  to our app" (a0 forever) from "user is at rest" (a1 with value 0).

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw

Run for ~10 s, squeezing trigger and grip alternately on each hand.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


# ---- helpers ----------------------------------------------------------

def _generate_runtime_manifest(static_path: str) -> str:
    """Materialise a runtime manifest with binary_path_windows = current python."""
    with open(static_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    py_exe = os.path.abspath(sys.executable)
    for app in manifest.get("applications", []):
        app["binary_path_windows"] = py_exe
    runtime = os.path.join(
        os.path.dirname(static_path), "manifest.runtime.vrmanifest"
    )
    with open(runtime, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    return runtime


def _decode_buttons(mask: int) -> str:
    names = {
        0: "System", 1: "Menu", 2: "Grip", 7: "A",
        32: "Trigger", 33: "Touchpad",
    }
    pressed = [name for bit, name in names.items() if mask & (1 << bit)]
    return ",".join(pressed) if pressed else "-"


def _get_string_prop(system, idx: int, prop) -> str:
    try:
        return system.getStringTrackedDeviceProperty(idx, prop)
    except Exception:
        return ""


# ---- main -------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seconds", type=float, default=10.0,
        help="How long to listen (default 10 s).",
    )
    parser.add_argument(
        "--rate", type=float, default=10.0,
        help="Print rate in Hz (default 10).",
    )
    args = parser.parse_args()

    try:
        import openvr  # type: ignore
    except ImportError:
        print("FATAL -- openvr (pyopenvr) not installed in this Python env.")
        print("  Activate the 'ust' conda env and try again.")
        return 2

    # ---- openvr.init -------------------------------------------------
    try:
        system = openvr.init(openvr.VRApplication_Other)
    except Exception as exc:
        print(f"FATAL -- openvr.init failed: {type(exc).__name__}: {exc}")
        print("  Is SteamVR running?  Launch Steam > Library > SteamVR first.")
        return 2

    try:
        # ---- manifest registration ---------------------------------
        actions_path = str(
            ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions" / "actions.json"
        )
        manifest_static = str(
            ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions" / "manifest.vrmanifest"
        )
        runtime_manifest = _generate_runtime_manifest(manifest_static)
        app_key = "ust.teleop.gr1t2_gripper"
        try:
            apps = openvr.VRApplications()
        except Exception as exc:
            print(f"FATAL -- VRApplications interface unavailable: {exc}")
            return 2
        # 9.43: force re-read of disk manifest by removing any cached
        # registration first.  AddApplicationManifest is a no-op when
        # the app_key is already registered (returns AppKeyAlreadyExists),
        # so SteamVR keeps the manifest snapshot from a previous run --
        # including stale default_bindings URLs and stale binding-file
        # contents.  Calling remove first forces SteamVR to re-read the
        # current actions.json + bindings_*.json on the next add.  See
        # memory.md section 10.51.
        for path_to_clear in (manifest_static, runtime_manifest):
            try:
                apps.removeApplicationManifest(path_to_clear)
            except Exception:
                pass
        for is_temp in (False, True):
            try:
                apps.addApplicationManifest(runtime_manifest, is_temp)
            except Exception as exc:
                msg = str(exc)
                if "AppKeyAlreadyExists" not in msg and "already" not in msg.lower():
                    print(
                        f"[SteamVRSampler][WARN] addApplicationManifest "
                        f"(temp={is_temp}): {exc}"
                    )
        print(
            f"[SteamVRSampler] persistent manifest registration OK "
            f"({runtime_manifest!r}, app_key={app_key!r})."
        )
        try:
            apps.identifyApplication(os.getpid(), app_key)
            print(
                f"[SteamVRSampler] identifyApplication OK "
                f"(pid={os.getpid()}, app_key={app_key!r})."
            )
        except Exception as exc:
            print(f"[SteamVRSampler][WARN] identifyApplication: {exc}")

        # ---- Action set + handles ----------------------------------
        try:
            vri = openvr.VRInput()
            vri.setActionManifestPath(actions_path)
        except Exception as exc:
            print(f"FATAL -- VRInput.setActionManifestPath failed: {exc}")
            return 2
        try:
            h_action_set = vri.getActionSetHandle("/actions/teleop")
        except Exception as exc:
            print(f"FATAL -- getActionSetHandle('/actions/teleop'): {exc}")
            return 2

        handles: Dict[str, int] = {}
        for path in (
            "/actions/teleop/in/trigger_left",
            "/actions/teleop/in/trigger_right",
            "/actions/teleop/in/grip_left",
            "/actions/teleop/in/grip_right",
            "/actions/teleop/in/menu_left",
            "/actions/teleop/in/menu_right",
        ):
            try:
                handles[path] = vri.getActionHandle(path)
            except Exception as exc:
                print(f"[WARN] getActionHandle({path!r}): {exc}")

        # ---- Inventory --------------------------------------------
        print("=" * 70)
        print("Controller probe -- legacy API + Action Manifest API "
              "(post-9.29 rewrite, 9.39 bActive instrumentation)")
        print("=" * 70)
        print("Tracker inventory (TrackedDeviceClass_GenericTracker):")
        for idx in range(openvr.k_unMaxTrackedDeviceCount):
            try:
                cls = system.getTrackedDeviceClass(idx)
            except Exception:
                continue
            if cls == openvr.TrackedDeviceClass_GenericTracker:
                serial = _get_string_prop(system, idx, openvr.Prop_SerialNumber_String)
                print(
                    f"  idx={idx:>2}  serial={serial!r:<40} -> "
                    f"NOT bound (add to tracker_binding.json if needed)"
                )

        probed_controllers: List[Tuple[int, str, str, str]] = []
        for idx in range(openvr.k_unMaxTrackedDeviceCount):
            try:
                cls = system.getTrackedDeviceClass(idx)
            except Exception:
                continue
            if cls == openvr.TrackedDeviceClass_Controller:
                serial = _get_string_prop(system, idx, openvr.Prop_SerialNumber_String)
                ctype = _get_string_prop(system, idx, openvr.Prop_ControllerType_String)
                role_int = system.getControllerRoleForTrackedDeviceIndex(idx)
                role_map = {1: "Left", 2: "Right", 0: "Invalid", 3: "OptOut"}
                role = role_map.get(int(role_int), str(int(role_int)))
                probed_controllers.append((idx, role, ctype, serial))
                print(
                    f"  idx={idx:>2}  role={role:<7} type={ctype:<25} "
                    f"serial={serial!r}"
                )

        if not probed_controllers:
            print("  (no controllers visible)")
            print()
            print("FAIL -- no Controller class devices visible to OpenVR.")
            print("  Pair the PICO controllers in PICO OS and ensure PICO Connect /")
            print("  Virtual Desktop is forwarding controller input to SteamVR.")
            return 1

        # ---- Listening loop ---------------------------------------
        print(f"Listening for {args.seconds:.0f} s -- squeeze trigger and grip "
              f"alternately on each hand.")
        print("Columns:")
        print("  L_*       -- legacy IVRSystem.getControllerState()  (Vive Wand path)")
        print("  A_*(a0/1) -- Action Manifest API value(bActive flag)")
        print("  L_* silent + A_*(a1) is EXPECTED for PICO/VD-Touch.")
        print("  A_*(a0) on every channel = Personal Binding NOT applied (memory.md 10.47).")
        print()

        vrs = openvr.VRSystem() if False else system  # alias for clarity
        deadline = time.time() + args.seconds
        last_print = 0.0
        legacy_seen = False
        legacy_call_ok_seen = False
        action_seen = False
        any_active = False
        period = 1.0 / max(0.5, float(args.rate))

        while time.time() < deadline:
            now = time.time()
            if now - last_print < period:
                time.sleep(0.01)
                continue
            last_print = now

            try:
                vri.updateActionState([
                    openvr.VRActiveActionSet_t(
                        ulActionSet=h_action_set,
                        ulRestrictedToDevice=openvr.k_ulInvalidInputValueHandle,
                    )
                ])
            except Exception:
                pass

            action_vals: Dict[str, Dict[str, float]] = {}
            for side in ("left", "right"):
                def _read_analog(path: str) -> Tuple[float, bool]:
                    h = handles.get(path)
                    if h is None:
                        return 0.0, False
                    try:
                        d = vri.getAnalogActionData(h)
                        return float(d.x), bool(getattr(d, "bActive", False))
                    except Exception:
                        return 0.0, False

                def _read_digital(path: str) -> Tuple[bool, bool]:
                    h = handles.get(path)
                    if h is None:
                        return False, False
                    try:
                        d = vri.getDigitalActionData(h)
                        return bool(d.bState), bool(getattr(d, "bActive", False))
                    except Exception:
                        return False, False

                trig, trig_a = _read_analog(f"/actions/teleop/in/trigger_{side}")
                grip, grip_a = _read_analog(f"/actions/teleop/in/grip_{side}")
                menu, menu_a = _read_digital(f"/actions/teleop/in/menu_{side}")
                action_vals[side] = {
                    "trigger": trig, "grip": grip, "menu": float(menu),
                    "trigger_active": float(trig_a),
                    "grip_active": float(grip_a),
                    "menu_active": float(menu_a),
                }
                if trig_a or grip_a or menu_a:
                    any_active = True
                if trig > 0.05 or grip > 0.05 or menu:
                    action_seen = True

            line_parts = [f"t={now - (deadline - args.seconds):4.1f}s"]
            for (idx, role, _ctype, _serial) in probed_controllers:
                ok, state = vrs.getControllerState(idx)
                side = (
                    "left" if role == "Left"
                    else "right" if role == "Right"
                    else None
                )

                if ok:
                    legacy_call_ok_seen = True
                    trig_l = float(state.rAxis[0].x)
                    grip_l = float(state.rAxis[1].x)
                    btn_p = int(state.ulButtonPressed)
                    if trig_l > 0.05 or grip_l > 0.05 or btn_p:
                        legacy_seen = True
                    legacy_str = (
                        f"L_trig={trig_l:.2f} L_grip={grip_l:.2f} "
                        f"btn=0x{btn_p:x}/{_decode_buttons(btn_p)}"
                    )
                else:
                    legacy_str = "L_trig=---  L_grip=---  btn=---"

                if side is not None and side in action_vals:
                    a = action_vals[side]
                    line_parts.append(
                        f"{role}: {legacy_str} | "
                        f"A_trig={a['trigger']:.2f}(a{int(a['trigger_active'])}) "
                        f"A_grip={a['grip']:.2f}(a{int(a['grip_active'])}) "
                        f"A_menu={'Y' if a['menu'] else '-'}(a{int(a['menu_active'])})"
                    )
                else:
                    line_parts.append(f"{role}: {legacy_str}")
            print("  " + "  |  ".join(line_parts))

        # ---- Verdict ----------------------------------------------
        print()
        print("=" * 70)
        print("Verdict:")
        print("  Legacy IVRSystem.getControllerState():")
        if legacy_call_ok_seen:
            print("    returned ok=True at least once (driver populates legacy state)")
        else:
            print("    always returned ok=False")
            print("    -> EXPECTED for PICO Touch via Virtual Desktop / PICO Connect")
            print("       'compatibility mode'.  These drivers only populate the")
            print("       Action Manifest path.  See memory.md section 10.9 for context.")
        print(f"    nonzero input observed: {'YES' if legacy_seen else 'no'}")
        print("  Action Manifest API:")
        print(f"    nonzero input observed: {'YES' if action_seen else 'no'}")
        print(f"    bActive=True observed:  {'YES' if any_active else 'no'}")
        print()

        # 9.39: bActive=False verdict promoted ABOVE the legacy hardware
        # checklist because Personal-Binding-not-applied is the most
        # common cause and indistinguishable from hardware fault without
        # bActive instrumentation.
        if not any_active:
            print("BINDING DIAGNOSIS -- Personal Binding NOT applied to our app.")
            print("  Every Action API handle reported bActive=False for the entire")
            print(f"  {args.seconds:.0f}s window.  This means SteamVR has no Personal Binding")
            print("  routing PICO controller input to 'ust.teleop.gr1t2_gripper'.")
            print("  HARDWARE IS NOT THE PROBLEM here -- SteamVR's Test Controller")
            print("  panel and PICO Connect's controller test both use the controller")
            print("  driver's defaults, NOT our app's per-application binding.")
            print()
            print("  FAST FIX (one of these):")
            print("    PRIMARY FIX -- clear stale Personal Binding from disk:")
            print("      python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding --clear")
            print("      Then RESTART SteamVR (right-click systray icon -> Quit; relaunch")
            print("      from Steam > Library > Tools > SteamVR).  This is the most reliable")
            print("      fix when open_binding_ui + Save Personal Binding did not work --")
            print("      the stale file on disk was overriding the default.  See memory.md")
            print("      section 10.48 for the writeup.")
            print()
            print("    A. python -X utf8 -m ust_ws.ust_hm_grip.scripts.open_binding_ui")
            print("       In the SteamVR dialog, select a Default binding for")
            print("       'UST Teleop GR1T2 Gripper' and click 'Save Personal Binding'.")
            print("    B. Manually: SteamVR > Settings > Controllers > Manage Controller")
            print("       Bindings > 'UST Teleop GR1T2 Gripper' > Active Controller")
            print("       Binding -> 'Default', then 'Save Personal Binding'.")
            print("  Re-run this diagnostic after fixing; you should see (a1) flags")
            print("  on every channel and nonzero values when you squeeze.")
            print("  See memory.md section 10.47 for the full root-cause writeup.")
            return 2

        if action_seen:
            print("OK -- controllers ARE working (Action API observed input).")
            if not legacy_seen:
                print(
                    "  The legacy 'L_*' columns staying silent is not a fault; it's"
                )
                print(
                    "  expected for modern controller emulators (VD Oculus-Touch, PICO"
                )
                print("  Connect compatibility mode).  Action API path is healthy.")
            print()
            print("  If the gripper still doesn't toggle in run_teleop, this is now a")
            print("  binding-mapping issue, not a hardware-input issue:")
            print("    1. Open the binding editor; verify grip_left/grip_right Pull is")
            print("       bound to controller GRIP (not 'Use as Trigger').")
            print("    2. Click 'Save Personal Binding' -- most commonly missed step.")
            return 0

        # any_active=True but no nonzero input -- bindings applied, user at rest.
        print("WARN -- Action API handles ARE bActive=True but no nonzero input")
        print("  was observed during the window.  Bindings are applied, but the")
        print("  user did not press any button during the test, OR the binding")
        print("  routes the wrong source.  Re-run and squeeze trigger/grip; if")
        print("  still 0, open SteamVR Binding Editor and verify trigger_{left,")
        print("  right} Pull is the physical trigger and grip_{left,right} Pull")
        print("  is the physical grip.")
        return 1
    finally:
        try:
            openvr.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
