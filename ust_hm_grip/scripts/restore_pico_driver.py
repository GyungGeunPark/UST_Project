"""Re-register PICO Connect's SteamVR external driver after a settings reset.

Symptom (memory.md section 10.50):
  - SteamVR > Settings > Startup/Shutdown > Manage Add-Ons shows ONLY
    ``prism`` (Valve Steam Link) and ``Gamepad Support`` -- the ``pico``
    entry is GONE.  Both visible Add-Ons are OFF.
  - ``openvr.init`` fails with ``InitError_Init_HmdNotFound`` because no
    HMD-providing driver is enabled.
  - PICO HMD shows "Waiting for connection" indefinitely because
    PICO Connect is running on the PC but SteamVR no longer knows
    about its driver.

Root cause:
  SteamVR's "Reset to Default" button (and certain Add-Ons / OpenXR
  toggle sequences) clears the external-driver registry that
  ``vrpathreg.exe`` maintains.  PICO Connect registers its driver
  there at install time pointing at ``<install>/openvr_driver``; once
  the entry is dropped, SteamVR neither displays nor loads it.

Recovery:
  1. Locate the PICO Connect install (Program Files paths).
  2. Find the ``openvr_driver`` subfolder inside it.
  3. Call ``vrpathreg.exe adddriverexternal <that_path>``.
  4. Verify with ``vrpathreg.exe show``.
  5. Restart SteamVR; ``pico`` should reappear in Manage Add-Ons
     (ON by default) and the HMD should connect again.

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.restore_pico_driver
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.restore_pico_driver --dry-run
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.restore_pico_driver \\
        --driver-path "D:/Custom/PICO Connect/openvr_driver"
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import List, Optional


# stdout hardening (9.38 pattern)
try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _say(msg: str) -> None:
    print(msg, flush=True)


PICO_DRIVER_CANDIDATES = [
    Path(r"C:\Program Files\PICO Connect\openvr_driver"),
    Path(r"C:\Program Files (x86)\PICO Connect\openvr_driver"),
    Path(r"C:\Program Files\PICO\PICO Connect\openvr_driver"),
    Path(r"C:\Program Files (x86)\PICO\PICO Connect\openvr_driver"),
    Path(r"D:\Program Files\PICO Connect\openvr_driver"),
    Path(r"D:\Program Files (x86)\PICO Connect\openvr_driver"),
]

VRPATHREG_CANDIDATES = [
    Path(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"),
    Path(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win32\vrpathreg.exe"),
]


def _find_pico_driver() -> Optional[Path]:
    """Try standard install paths + a winreg lookup as fallback."""
    for c in PICO_DRIVER_CANDIDATES:
        if c.exists() and c.is_dir():
            return c
    if sys.platform == "win32":
        try:
            import winreg  # type: ignore
            for hive_key in (
                (winreg.HKEY_LOCAL_MACHINE,
                 r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall"),
                (winreg.HKEY_CURRENT_USER,
                 r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall"),
            ):
                try:
                    with winreg.OpenKey(hive_key[0], hive_key[1]) as hk:
                        i = 0
                        while True:
                            try:
                                sub = winreg.EnumKey(hk, i)
                            except OSError:
                                break
                            i += 1
                            try:
                                with winreg.OpenKey(hk, sub) as sk:
                                    name = winreg.QueryValueEx(sk, "DisplayName")[0]
                                    if "pico connect" in name.lower():
                                        loc = winreg.QueryValueEx(sk, "InstallLocation")[0]
                                        cand = Path(loc) / "openvr_driver"
                                        if cand.exists():
                                            return cand
                            except (FileNotFoundError, OSError):
                                continue
                except (FileNotFoundError, OSError):
                    continue
        except Exception:  # noqa: BLE001
            pass
    return None


def _find_vrpathreg() -> Optional[Path]:
    for c in VRPATHREG_CANDIDATES:
        if c.exists():
            return c
    if sys.platform == "win32":
        try:
            import winreg  # type: ignore
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Valve\Steam") as k:
                steam_path = winreg.QueryValueEx(k, "SteamPath")[0]
            cand = Path(steam_path) / "steamapps" / "common" / "SteamVR" / "bin" / "win64" / "vrpathreg.exe"
            if cand.exists():
                return cand
        except Exception:  # noqa: BLE001
            pass
    return None


def _vrpathreg_show(vrpath: Path) -> str:
    try:
        res = subprocess.run(
            [str(vrpath), "show"],
            capture_output=True, text=True, timeout=10, check=False,
        )
        return (res.stdout or "") + (res.stderr or "")
    except Exception as exc:  # noqa: BLE001
        return f"vrpathreg show failed: {exc}"


def _vrpathreg_adddriver(vrpath: Path, driver_dir: Path) -> int:
    res = subprocess.run(
        [str(vrpath), "adddriverexternal", str(driver_dir)],
        capture_output=True, text=True, timeout=10, check=False,
    )
    if res.stdout:
        for line in res.stdout.splitlines():
            _say(f"  vrpathreg> {line}")
    if res.stderr:
        for line in res.stderr.splitlines():
            _say(f"  vrpathreg!> {line}")
    return res.returncode


def main() -> int:
    _say("[restore_pico_driver] starting...")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print what would happen without modifying anything.",
    )
    parser.add_argument(
        "--driver-path", type=Path, default=None,
        help="Override the auto-detected PICO Connect openvr_driver path.",
    )
    parser.add_argument(
        "--show-only", action="store_true",
        help="Print current vrpathreg state and exit.",
    )
    args = parser.parse_args()

    if sys.platform != "win32":
        _say("[restore_pico_driver] FAIL: this script only runs on Windows.")
        return 2

    vrpath = _find_vrpathreg()
    if vrpath is None:
        _say("[restore_pico_driver] FAIL: vrpathreg.exe not found.")
        _say("  Standard locations checked:")
        for c in VRPATHREG_CANDIDATES:
            _say(f"    {c}")
        return 1
    _say(f"[restore_pico_driver] vrpathreg found at: {vrpath}")

    _say("")
    _say("[restore_pico_driver] current vrpathreg state:")
    state = _vrpathreg_show(vrpath)
    for line in state.splitlines():
        _say(f"  {line}")
    if args.show_only:
        return 0

    driver = args.driver_path or _find_pico_driver()
    if driver is None:
        _say("")
        _say("[restore_pico_driver] FAIL: PICO Connect openvr_driver folder NOT found.")
        _say("  Tried these paths:")
        for c in PICO_DRIVER_CANDIDATES:
            _say(f"    {c}")
        _say("")
        _say("  Recovery options:")
        _say("    1. Reinstall PICO Connect from PICO's official website,")
        _say("       then re-run this script.")
        _say("    2. Locate the openvr_driver folder manually and pass it via")
        _say("       `--driver-path \"<that_folder>\"`.")
        _say("    3. Use Steam Link instead of PICO Connect (no body tracking):")
        _say("       SteamVR > Manage Add-Ons > prism = ON, then run Steam")
        _say("       Link app on the PICO HMD.")
        return 1
    _say("")
    _say(f"[restore_pico_driver] PICO Connect driver folder: {driver}")

    if str(driver) in state:
        _say("[restore_pico_driver] driver is ALREADY registered.")
        _say("  If 'pico' still does not appear in SteamVR Manage Add-Ons:")
        _say("    1. Restart SteamVR fully.")
        _say("    2. Toggle the entry OFF then ON in Manage Add-Ons.")
        _say("    3. Verify PICO Connect (Windows app) is actually running.")
        return 0

    if args.dry_run:
        _say("")
        _say(f"[restore_pico_driver] DRY-RUN: would call:")
        _say(f"  {vrpath} adddriverexternal {driver}")
        return 0

    _say("")
    _say("[restore_pico_driver] registering driver via vrpathreg adddriverexternal...")
    rc = _vrpathreg_adddriver(vrpath, driver)
    if rc != 0:
        _say(f"[restore_pico_driver] FAIL: vrpathreg returned exit code {rc}.")
        _say("  Try running this script as Administrator if PermissionDenied.")
        return rc

    _say("")
    _say("[restore_pico_driver] post-registration state:")
    state2 = _vrpathreg_show(vrpath)
    for line in state2.splitlines():
        _say(f"  {line}")

    if str(driver) in state2:
        _say("")
        _say("[restore_pico_driver] OK -- driver registered.")
        _say("  Next steps:")
        _say("    1. Make sure SteamVR is fully closed (system tray > Quit).")
        _say("    2. Make sure PICO Connect (Windows app) is running.")
        _say("    3. Pair the headset (PICO Connect > Devices panel).")
        _say("    4. Launch SteamVR (Steam > Library > Tools > SteamVR).")
        _say("    5. SteamVR > Settings > Startup/Shutdown > Manage Add-Ons:")
        _say("       'pico' should now be listed -- set it to ON.")
        _say("    6. Wait for the headset icon to turn green.")
        _say("    7. Re-run diagnose_controller_raw to verify trigger/grip.")
        return 0

    _say("[restore_pico_driver][WARN] registration command succeeded but the")
    _say("  driver path is not in the post-show output.  Check vrpathreg show")
    _say("  manually and consider running this script as Administrator.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
