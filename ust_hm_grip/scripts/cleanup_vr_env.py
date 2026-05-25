"""Clean up the local SteamVR / Oculus runtime state for ust_hm_grip teleop.

Usage::

    # Run from a regular PowerShell (kills OVRServer_x64 transiently):
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.cleanup_vr_env

    # Run from an *Administrator* PowerShell to also disable the OVR service
    # so the launcher cannot respawn OVRServer_x64:
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.cleanup_vr_env

    # Also stop SteamVR (vrserver/vrmonitor/vrcompositor/vrwebhelper) so PICO
    # Connect can re-spawn it with a fresh IPC namespace:
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.cleanup_vr_env --restart-steamvr

What this script does
=====================
1. Reports which OpenVR / Oculus / PICO runtime processes are currently
   running.
2. Stops ``OVRServer_x64.exe`` and (with admin) disables the
   ``OVRService`` Windows service so it cannot respawn.
3. Optionally stops SteamVR's vrserver + helpers so the next launch
   gets a clean IPC namespace (use ``--restart-steamvr``).
4. Prints a final snapshot.

This addresses the common ``InitError_IPC_NamespaceUnavailable`` we hit
when ust_hm_grip teleop initialises OpenVR while the Oculus runtime is
also running, or when a previous OpenVR client crashed and left
vrserver holding a stale namespace handle.

The script is intentionally cautious: it never restarts SteamVR or PICO
Connect itself -- the user must re-stream from PICO Connect after this
runs, since only PICO Connect knows how to spawn vrserver via its prism
driver.
"""

from __future__ import annotations

import argparse
import ctypes
import subprocess
import sys
from typing import List


_OVR_PROCS = ("OVRServer_x64", "OVRRedir", "OVRServiceLauncher")
_STEAMVR_PROCS = (
    "vrserver",
    "vrmonitor",
    "vrwebhelper",
    "vrcompositor",
    "vrdashboard",
)


def _is_admin() -> bool:
    try:
        return bool(ctypes.windll.shell32.IsUserAnAdmin())
    except Exception:  # pragma: no cover
        return False


def _running_procs(names: tuple[str, ...]) -> List["psutil.Process"]:
    try:
        import psutil
    except ImportError:
        print(
            "[cleanup_vr_env] psutil not installed in the active env. "
            "Install with `pip install psutil` and retry.",
            file=sys.stderr,
        )
        return []
    out = []
    targets = {n.lower() for n in names}
    for proc in psutil.process_iter(["name", "pid"]):
        n = (proc.info.get("name") or "").lower()
        n_no_ext = n[:-4] if n.endswith(".exe") else n
        if n_no_ext in targets:
            out.append(proc)
    return out


def _print_snapshot(label: str) -> None:
    ovr = _running_procs(_OVR_PROCS)
    svr = _running_procs(_STEAMVR_PROCS)
    print(f"\n=== {label} ===")
    if not ovr and not svr:
        print("  (no OpenVR / Oculus runtime processes running)")
        return
    for p in ovr:
        try:
            print(f"  Oculus  : {p.info['name']:30s} PID={p.info['pid']}")
        except Exception:
            pass
    for p in svr:
        try:
            print(f"  SteamVR : {p.info['name']:30s} PID={p.info['pid']}")
        except Exception:
            pass


def _kill_procs(names: tuple[str, ...], label: str) -> int:
    procs = _running_procs(names)
    killed = 0
    for p in procs:
        try:
            pid = p.info["pid"]
            name = p.info["name"]
            print(f"[cleanup_vr_env] stopping {label} {name} PID={pid}")
            p.terminate()
            try:
                p.wait(timeout=3.0)
            except Exception:
                p.kill()
            killed += 1
        except Exception as exc:  # noqa: BLE001
            print(f"[cleanup_vr_env]   failed to stop PID={p.info.get('pid')}: "
                  f"{type(exc).__name__}: {exc}")
    return killed


def _disable_ovr_service() -> None:
    if not _is_admin():
        print(
            "[cleanup_vr_env] Skipping `OVRService` disable: not running as admin.\n"
            "  Re-run from an Administrator PowerShell to keep OVRServer_x64\n"
            "  permanently dead.  Otherwise the service will respawn it within\n"
            "  seconds and you will hit the IPC conflict again."
        )
        return
    for action, args in (
        ("stop", ["sc", "stop", "OVRService"]),
        ("disable", ["sc", "config", "OVRService", "start=", "disabled"]),
    ):
        try:
            res = subprocess.run(args, capture_output=True, text=True, timeout=15)
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            print(f"[cleanup_vr_env] sc {action} OVRService failed: {exc}")
            continue
        first_line = (res.stdout or res.stderr or "").splitlines()[:1]
        suffix = (": " + first_line[0]) if first_line else ""
        rc = res.returncode
        print(f"[cleanup_vr_env] sc {action} OVRService -> rc={rc}{suffix}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--restart-steamvr", action="store_true",
        help="Also stop SteamVR (vrserver / vrmonitor / vrwebhelper / vrcompositor / "
             "vrdashboard) so the next PICO Connect stream re-spawns it with a "
             "fresh IPC namespace.  Without this flag SteamVR is left untouched.",
    )
    parser.add_argument(
        "--keep-ovr", action="store_true",
        help="Do NOT touch Oculus / Meta runtime processes (use if you actually "
             "use a Quest/Rift headset alongside PICO).",
    )
    args = parser.parse_args()

    print("[cleanup_vr_env] target: clear OpenVR IPC namespace contention "
          "for ust_hm_grip teleop.")
    if not _is_admin():
        print("[cleanup_vr_env] NOTE: not running as Administrator.")
    _print_snapshot("BEFORE")

    if not args.keep_ovr:
        _kill_procs(_OVR_PROCS, "Oculus runtime")
        _disable_ovr_service()

    if args.restart_steamvr:
        _kill_procs(_STEAMVR_PROCS, "SteamVR")
        print(
            "\n[cleanup_vr_env] SteamVR stopped.  Open PICO Connect and click\n"
            "  'PCVR 스트리밍 시작' / 'Start PCVR Streaming' to re-spawn vrserver.\n"
            "  Wait until SteamVR's status window shows 'Ready' before relaunching\n"
            "  ust_hm_grip teleop."
        )

    _print_snapshot("AFTER")
    print(
        "\n[cleanup_vr_env] Done.  Next steps:\n"
        "  1. Make sure PICO Connect is streaming and SteamVR shows 'Ready'.\n"
        "  2. Relaunch teleop:\n"
        "       $env:PYTHONPATH = '.'\n"
        "       python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop \\\n"
        "           --env_variant robot_only --teleop_device pico_gripper \\\n"
        "           --vr_runtime pico_connect --gripper_signal_source grip \\\n"
        "           --render_mode steamvr_native --process_priority high"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
