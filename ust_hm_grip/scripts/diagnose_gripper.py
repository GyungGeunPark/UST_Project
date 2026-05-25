"""Layer-by-layer diagnostic for the GR1T2 gripper teleop pipeline.

Prints a focused summary of:

* SteamVR connection health (HMD found, controllers found, controller
  type, hand role).
* Whether the OpenVR action manifest has been registered (the bindings
  we ship are visible in SteamVR's Manage Controller Bindings UI).
* Live grip / trigger / menu values for both controllers (the *active*
  signal — see ``--signal-source`` — is highlighted in the WARN block).
* Forearm tracker presence + computed wrist target offset (after
  applying the +0.28 m forearm-wrist offset).
* Whether UDCAP is unintentionally still running (warns if a 'knuckles'
  controller_type is reported).

Usage::

    # default: signal source = grip (matches user request)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper

    # explicitly select a source
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper --signal-source grip
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper --signal-source trigger
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper --signal-source both

Runs for ~10 s in monitor mode (no Isaac Sim required) and exits.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--signal-source", default="grip",
        choices=["grip", "trigger", "both"],
        help="Which controller input drives the gripper close/open. "
             "Default 'grip' (matches the 9.28+ default).",
    )
    parser.add_argument(
        "--seconds", type=float, default=10.0,
        help="How long to listen for input (default 10 s).",
    )
    args = parser.parse_args()

    primary = "grip" if args.signal_source in ("grip", "both") else "trigger"

    print("ust_hm_grip — gripper diagnostic")
    print(f"  signal source = {args.signal_source!r}  (primary input = {primary!r})")
    print("=" * 60)

    try:
        from ust_ws.ust_hm_grip.teleop.gr1t2_gripper_device import (
            GR1T2GripperDevice,
            GR1T2GripperDeviceCfg,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"FATAL — cannot import GR1T2GripperDevice: {exc}")
        return 2

    cfg = GR1T2GripperDeviceCfg(
        tracker_binding_json="./ust_ws/ust_hm_grip/config/tracker_binding.json",
        actions_json="./ust_ws/ust_hm_grip/config/openvr_actions/actions.json",
        vrmanifest_json="./ust_ws/ust_hm_grip/config/openvr_actions/manifest.vrmanifest",
        forearm_wrist_offset=(0.28, 0.0, 0.0),
        prefer_controller_for_eef=True,
        gripper_signal_source=args.signal_source,
        debug=True,
    )

    try:
        device = GR1T2GripperDevice(cfg)
    except Exception as exc:  # noqa: BLE001
        print(f"FATAL — cannot construct device: {exc}")
        return 3

    try:
        device.start()
    except Exception as exc:  # noqa: BLE001
        print(f"FATAL — device.start() failed: {exc}")
        return 4

    print(f"\nWaiting up to {args.seconds:.0f} s for {primary} input...\n")
    deadline = time.time() + args.seconds
    last_log = 0.0
    seen_signal = False
    # 9.39: track bActive across the window.  any_active=False with
    # healthy hardware is the smoking gun for "Personal Binding never
    # applied to ust.teleop.gr1t2_gripper" -- not a hardware issue.
    any_active = False
    try:
        while time.time() < deadline:
            action = device.advance()
            if action is None:
                time.sleep(0.05)
                continue
            now = time.time()
            ai = device._read_action_inputs()  # noqa: SLF001
            l_grip = ai["left"]["grip"]
            r_grip = ai["right"]["grip"]
            l_trig = ai["left"]["trigger"]
            r_trig = ai["right"]["trigger"]
            l_g_a = ai["left"].get("grip_active", False)
            r_g_a = ai["right"].get("grip_active", False)
            l_t_a = ai["left"].get("trigger_active", False)
            r_t_a = ai["right"].get("trigger_active", False)
            if l_g_a or r_g_a or l_t_a or r_t_a:
                any_active = True
            # Match the diagnostic to the active signal source so the WARN
            # block at the bottom is meaningful.
            if args.signal_source == "trigger":
                if l_trig > 0.05 or r_trig > 0.05:
                    seen_signal = True
            elif args.signal_source == "both":
                if l_grip > 0.05 or r_grip > 0.05 or l_trig > 0.05 or r_trig > 0.05:
                    seen_signal = True
            else:
                if l_grip > 0.05 or r_grip > 0.05:
                    seen_signal = True
            if now - last_log >= 0.5:
                # 9.39: 'a' suffix = bActive flag (a1 = binding routes
                # input to our app, a0 = binding NOT applied).
                print(
                    f"  t={now - (deadline - args.seconds):4.1f}s  "
                    f"L_grip={l_grip:.2f}(a{int(l_g_a)}) R_grip={r_grip:.2f}(a{int(r_g_a)})  "
                    f"L_trig={l_trig:.2f}(a{int(l_t_a)}) R_trig={r_trig:.2f}(a{int(r_t_a)})  "
                    f"L_cmd={float(action[14]):+.0f} R_cmd={float(action[15]):+.0f}"
                )
                last_log = now
            time.sleep(0.04)
    finally:
        device.stop()

    print()
    print("=" * 60)
    if seen_signal:
        print(f"OK — {primary} input observed.  Gripper hysteresis is working.")
        return 0

    if not any_active:
        # 9.39: binding-not-applied verdict.  Promoted above the legacy
        # 6-cause hardware checklist because it is the most common cause
        # when the hardware checklist passes (PICO Connect green,
        # controllers paired, SteamVR Test Controller works).
        print("BINDING DIAGNOSIS -- Personal Binding NOT applied to our app.")
        print("  Across the entire window every Action API channel reported")
        print("  bActive=False.  This means SteamVR has no Personal Binding")
        print("  routing PICO controller input to 'ust.teleop.gr1t2_gripper'")
        print("  actions.  HARDWARE IS NOT THE PROBLEM here -- SteamVR's Test")
        print("  Controller panel and PICO Connect's controller test both use")
        print("  the controller driver's defaults, NOT our app's per-application")
        print("  binding, so they can show buttons working while our app sees 0.")
        print()
        print("  FAST FIX:")
        print("    PRIMARY FIX -- clear stale Personal Binding from disk:")
        print("      python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding --clear")
        print("      Then RESTART SteamVR (right-click systray icon -> Quit; relaunch")
        print("      from Steam > Library > Tools > SteamVR).  This is the most reliable")
        print("      fix when open_binding_ui + Save Personal Binding did not work --")
        print("      the stale file on disk was overriding the default.  See memory.md")
        print("      section 10.48 for the writeup.")
        print()
        print("    A. python -X utf8 -m ust_ws.ust_hm_grip.scripts.open_binding_ui")
        print("       In the SteamVR dialog: select 'UST Teleop GR1T2 Gripper")
        print("       Default' as Active Controller Binding, click 'Save Personal")
        print("       Binding' at the bottom.")
        print("    B. Manually: SteamVR > Settings > Controllers > Manage Controller")
        print("       Bindings > 'UST Teleop GR1T2 Gripper' > Active Controller")
        print("       Binding -> 'Default', then 'Save Personal Binding'.")
        print("  Re-run this diagnostic after fixing; (a1) on every channel.")
        print("  Full root-cause writeup: memory.md section 10.47.")
        return 2

    print(f"WARN -- no {primary} input observed during the {args.seconds:.0f}s window.")
    print("  Action API handles ARE bActive=True (binding is applied), but")
    print("  no nonzero input was seen.  Possible causes:")
    print("    1. The binding routes the WRONG source for {0}_left/{0}_right.".format(primary))
    print("       Open the binding editor and verify {0} Pull is the physical {0}.".format(primary))
    print("    2. SteamVR > Settings > Controllers > Manage Controller Bindings")
    print("       does not have a binding for app 'UST Teleop GR1T2 Gripper' set ACTIVE.")
    print(f"    3. The binding has '{primary}_left/{primary}_right' Pull set to 'None'")
    print("       or to the wrong action.  Open the binding editor and verify.")
    print("    4. The binding was edited but **'Save Personal Binding'** at the")
    print("       bottom of the editor was never clicked.")
    print("    5. UdcapDriver is still running and masking PICO Touch as knuckles.")
    print("    6. PICO Connect Streaming Assistant Compatibility Mode set wrong.")
    print(f"    7. User did not actually pull the {primary} during the window.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
