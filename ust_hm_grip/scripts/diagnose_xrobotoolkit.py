"""Layered XRoboToolkit pipeline diagnostic (research/47 §11).

Probes from the lowest layer up:
    L1: Windows process state (RoboticsServiceProcess, PICO Connect conflict)
    L2: TCP listen port (gRPC server reachable)
    L3: xrobotoolkit_sdk import + xrt.init() handshake
    L4: Live controller analog data (trigger/grip stream)

Fails fast at the first broken layer with an actionable hint.

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_xrobotoolkit
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from typing import Optional, Tuple


def _l1_probe_processes() -> Tuple[bool, str]:
    try:
        import psutil  # type: ignore[import-not-found]
    except ImportError:
        return False, "psutil not installed (pip install psutil)"
    svc_running = False
    pc_running = False
    for proc in psutil.process_iter(["name"]):
        name = (proc.info["name"] or "").lower()
        if "roboticsserviceprocess" in name:
            svc_running = True
        elif name == "pico connect.exe":
            pc_running = True
    if not svc_running:
        return False, (
            "RoboticsServiceProcess.exe is NOT running.\n"
            "  Start it: & 'C:\\develop\\IsaacLab\\ust_ws\\XRoboToolkit-PC-Service.win\\runService.bat'"
        )
    msg = "RoboticsServiceProcess.exe is running."
    if pc_running:
        msg += ("\n  WARNING: 'Pico Connect.exe' is also running -- "
                "this may cause USB / OpenXR conflicts.  Consider:\n"
                "    taskkill /IM \"Pico Connect.exe\" /F")
    return True, msg


def _l2_probe_port(host: str = "127.0.0.1",
                   candidate_ports=(60061, 50051, 12345, 23306)
                   ) -> Tuple[bool, str]:
    """Try to connect to known PC-Service ports.

    Default listenPort per ``setting.ini`` of the v1.0.0 release is 60061.
    We also probe the other candidate ports documented in the design guide.
    """
    for port in candidate_ports:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True, f"TCP {host}:{port} reachable."
        except (ConnectionRefusedError, socket.timeout, OSError):
            continue
    return False, (
        f"None of {candidate_ports} are listening on {host}.\n"
        "  Check setting.ini for the actual port:\n"
        "    Get-Content 'C:\\develop\\IsaacLab\\ust_ws\\XRoboToolkit-PC-Service.win\\setting.ini'\n"
        "  Or list listening ports owned by RoboticsServiceProcess:\n"
        "    Get-NetTCPConnection -State Listen | Where-Object OwningProcess -EQ \\\n"
        "        (Get-Process RoboticsServiceProcess).Id"
    )


def _l3_probe_sdk_init() -> Tuple[bool, str, Optional[object]]:
    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        return False, (
            f"xrobotoolkit_sdk import failed: {exc}\n"
            "  Build the binding: cd ust_ws\\XRoboToolkit-PC-Service-Pybind && "
            ".\\setup_windows.bat\n"
            "  (Requires MSVC Build Tools; see EXECUTION_GUIDE.md.)"
        ), None
    try:
        xrt.init()
    except Exception as exc:  # noqa: BLE001
        return False, (
            f"xrt.init() raised {type(exc).__name__}: {exc}\n"
            "  PC service is running but gRPC handshake failed.  "
            "Check Windows Firewall for RoboticsServiceProcess inbound."
        ), None
    return True, "xrt.init() succeeded.", xrt


def _l4_probe_live_data(xrt, seconds: float = 8.0) -> Tuple[bool, str]:
    """Poll for ``seconds`` and report whether trigger/grip ever moved."""
    print(f"[L4] Polling for {seconds:.0f}s -- squeeze controllers...")
    deadline = time.perf_counter() + seconds
    max_l_trig = max_l_grip = max_r_trig = max_r_grip = 0.0
    samples = 0
    while time.perf_counter() < deadline:
        try:
            max_l_trig = max(max_l_trig, float(xrt.get_left_trigger()))
            max_l_grip = max(max_l_grip, float(xrt.get_left_grip()))
            max_r_trig = max(max_r_trig, float(xrt.get_right_trigger()))
            max_r_grip = max(max_r_grip, float(xrt.get_right_grip()))
            samples += 1
        except Exception as exc:  # noqa: BLE001
            return False, f"poll error: {type(exc).__name__}: {exc}"
        time.sleep(0.05)
    summary = (
        f"  L_trig max={max_l_trig:.3f}  L_grip max={max_l_grip:.3f}\n"
        f"  R_trig max={max_r_trig:.3f}  R_grip max={max_r_grip:.3f}\n"
        f"  samples={samples}"
    )
    threshold = 0.3
    moved = max(max_l_trig, max_l_grip, max_r_trig, max_r_grip) >= threshold
    if not moved:
        return False, (
            f"{summary}\n"
            "  No analog activity above 0.3 detected.  Likely causes:\n"
            "    1. Unity Client APK on headset doesn't have Direction=Send\n"
            "    2. Controller pairing dropped (check headset settings)\n"
            "    3. Tracking data toggle in APK is OFF for Controller channel"
        )
    return True, f"{summary}\n  Controller analog stream LIVE."


def main() -> int:
    parser = argparse.ArgumentParser(description="Layered XRoboToolkit pipeline probe")
    parser.add_argument("--skip_live", action="store_true",
                        help="Skip L4 (live data) probe if you just want plumbing check.")
    parser.add_argument("--seconds", type=float, default=8.0,
                        help="L4 live polling duration (default 8 s).")
    args = parser.parse_args()

    print("=" * 60)
    print("XRoboToolkit pipeline diagnostic")
    print("=" * 60)

    print("\n[L1] Windows process state")
    ok, msg = _l1_probe_processes()
    print(f"  {'OK' if ok else 'FAIL'}: {msg}")
    if not ok:
        return 1

    print("\n[L2] TCP listen port (best-effort sweep)")
    ok, msg = _l2_probe_port()
    print(f"  {'OK' if ok else 'WARN'}: {msg}")
    # L2 failure is not fatal; xrt SDK may use a different transport
    if not ok:
        print("  Continuing anyway -- xrobotoolkit_sdk may use a different IPC.")

    print("\n[L3] xrobotoolkit_sdk init")
    ok, msg, xrt = _l3_probe_sdk_init()
    print(f"  {'OK' if ok else 'FAIL'}: {msg}")
    if not ok:
        return 3

    if not args.skip_live:
        print("\n[L4] Live controller data probe")
        ok, msg = _l4_probe_live_data(xrt, seconds=args.seconds)
        print(f"  {'OK' if ok else 'FAIL'}: {msg}")
        if not ok:
            try:
                xrt.close()
            except Exception:  # noqa: BLE001
                pass
            return 4

    try:
        xrt.close()
    except Exception:  # noqa: BLE001
        pass

    print("\n" + "=" * 60)
    print("ALL LAYERS PASS -- XRoboToolkit pipeline is healthy.")
    print("=" * 60)
    print("\nNext: run the teleop pipeline:\n"
          "  python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop \\\n"
          "      --env_variant robot_only --render_mode monitor --headless \\\n"
          "      --input_backend xrobotoolkit --diag off --steps 30")
    return 0


if __name__ == "__main__":
    sys.exit(main())
