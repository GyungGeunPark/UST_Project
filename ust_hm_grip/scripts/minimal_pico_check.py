"""Minimal XRoboToolkit smoke test (no Isaac Sim required).

Verifies that:
    1. ``xrobotoolkit_sdk`` imports cleanly.
    2. ``xrt.init()`` connects to the running PC service.
    3. Controller analog values change when the user squeezes grip/trigger.

Run BEFORE integrating into the Isaac Lab teleop pipeline (research/47 §6).

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.minimal_pico_check --seconds 15
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="XRoboToolkit minimal smoke test")
    parser.add_argument("--seconds", type=float, default=15.0,
                        help="How long to poll (default: 15 s).")
    parser.add_argument("--rate_hz", type=float, default=10.0,
                        help="Poll rate for printing (default: 10 Hz).")
    args = parser.parse_args()

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        print(f"[FATAL] xrobotoolkit_sdk import failed: {exc}", file=sys.stderr)
        print(
            "        See research/47 §5 to build the Python binding.\n"
            "          cd C:\\develop\\IsaacLab\\ust_ws\\XRoboToolkit-PC-Service-Pybind\n"
            "          .\\setup_windows.bat\n"
            "        (Requires MSVC Build Tools; see EXECUTION_GUIDE.md.)",
            file=sys.stderr,
        )
        return 1

    print("[1/3] xrobotoolkit_sdk imported.")
    try:
        xrt.init()
    except Exception as exc:  # noqa: BLE001
        print(f"[FATAL] xrt.init() failed: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        print(
            "        Ensure RoboticsServiceProcess.exe is running:\n"
            "          Get-Process RoboticsServiceProcess\n"
            "        If not:\n"
            "          & 'C:\\develop\\IsaacLab\\ust_ws\\XRoboToolkit-PC-Service.win\\runService.bat'",
            file=sys.stderr,
        )
        return 2

    print("[2/3] xrt.init() OK -- connected to PC service.")
    print(f"[3/3] Polling for {args.seconds:.0f}s at {args.rate_hz:.0f} Hz.")
    print("      Squeeze grip/trigger on either controller -- values should respond.")
    print()

    dt = 1.0 / max(1.0, args.rate_hz)
    deadline = time.perf_counter() + args.seconds
    max_l_grip = 0.0
    max_l_trig = 0.0
    max_r_grip = 0.0
    max_r_trig = 0.0
    sample_count = 0

    while time.perf_counter() < deadline:
        try:
            lt = float(xrt.get_left_trigger())
            rt = float(xrt.get_right_trigger())
            lg = float(xrt.get_left_grip())
            rg = float(xrt.get_right_grip())
            lpose = xrt.get_left_controller_pose()
            try:
                ts = int(xrt.get_time_stamp_ns())
            except Exception:  # noqa: BLE001
                ts = int(time.time_ns())
        except Exception as exc:  # noqa: BLE001
            print(f"  [poll err] {type(exc).__name__}: {exc}")
            time.sleep(dt)
            continue

        max_l_grip = max(max_l_grip, lg)
        max_l_trig = max(max_l_trig, lt)
        max_r_grip = max(max_r_grip, rg)
        max_r_trig = max(max_r_trig, rt)
        sample_count += 1

        if (lg > 0.05 or rg > 0.05 or lt > 0.05 or rt > 0.05
                or sample_count % 10 == 0):
            try:
                lp = [float(v) for v in lpose[:3]]
                pose_str = f"[{lp[0]:+.3f},{lp[1]:+.3f},{lp[2]:+.3f}]"
            except Exception:  # noqa: BLE001
                pose_str = "<no pose>"
            print(
                f"  t={ts:>20d}  "
                f"L: trig={lt:.2f} grip={lg:.2f} pose={pose_str}  "
                f"R: trig={rt:.2f} grip={rg:.2f}"
            )
        time.sleep(dt)

    print()
    print("=" * 60)
    print(f"Summary  ({sample_count} samples)")
    print("=" * 60)
    print(f"  max L_trigger = {max_l_trig:.3f}    L_grip = {max_l_grip:.3f}")
    print(f"  max R_trigger = {max_r_trig:.3f}    R_grip = {max_r_grip:.3f}")

    verdict_input = max(max_l_grip, max_l_trig, max_r_grip, max_r_trig) > 0.5
    if verdict_input:
        print()
        print("PASS -- xrobotoolkit_sdk receives controller analog input.")
        print("   Continue to Step 5 (coord_transforms unit test).")
    else:
        print()
        print("FAIL -- analog values stayed near zero.")
        print("   Check:")
        print("     1. Unity Client APK is running on headset + Direction = Send")
        print("     2. RoboticsServiceProcess.exe is running")
        print("     3. Controller pairing OK (try Settings -> Controllers)")
        try:
            xrt.close()
        except Exception:  # noqa: BLE001
            pass
        return 3

    try:
        xrt.close()
    except Exception:  # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
