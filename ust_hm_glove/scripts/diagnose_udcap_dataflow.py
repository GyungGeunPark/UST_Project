"""Standalone UDCAP data-flow diagnostic.

Run this *outside* Isaac Sim — it does not import isaaclab or carb — to verify
each layer of the UDCAP glove => SteamVR => action API pipeline independently.

Usage::

    python -m ust_ws.ust_hm_glove.scripts.diagnose_udcap_dataflow

What it checks (top => bottom of the pipeline):

  1. ``UdcapDriver.exe`` and ``UDCAP_overlay.exe`` process state.
     The user-space ``UdcapDriver.exe`` is what reads the gloves over USB/BT
     and forwards finger sensor frames to the SteamVR driver via a named
     pipe.  If it is not running, the SteamVR driver creates virtual
     knuckles controllers (with pose taken from the underlying oculus_touch
     stream) but receives ZERO finger / trigger / grip data — every action
     value reads 0.

  2. ``UdcapDriver.dll.config`` — communication settings + VMC export.
     Confirms whether the user's UDCAP config has VMC broadcasting enabled
     (Path B fallback for our retargeter), and prints the port.

  3. ``SteamVR/logs/vrserver.txt`` for ``udcap: Received named pipe data``
     occurrences.  The presence of these messages (and their freshness)
     proves that ``UdcapDriver.exe`` is actively forwarding glove sensor
     data to SteamVR.  Their absence is a hard failure signal.

  4. UDP port 39539 — listen briefly for VMC ``/VMC/Ext/Bone/Pos`` packets.
     If UDCAP's VMC export is enabled and the gloves are actively read, we
     should see hand-bone OSC packets within a few seconds.

The script prints a short verdict at the end with suggested next steps.
"""

from __future__ import annotations

import json
import os
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from xml.etree import ElementTree as ET


UDCAP_DRIVER_DIR = Path(r"C:\Program Files\UdcapDriver")
UDCAP_DRIVER_EXE = UDCAP_DRIVER_DIR / "UdcapDriver.exe"
UDCAP_OVERLAY_EXE = UDCAP_DRIVER_DIR / "udcap" / "bin" / "win64" / "UDCAP_overlay.exe"
UDCAP_CONFIG_XML = UDCAP_DRIVER_DIR / "UdcapDriver.dll.config"
UDCAP_PROFILE_JSON = UDCAP_DRIVER_DIR / "udcap" / "resources" / "input" / "UDCAP_profile.json"
STEAMVR_LOG = Path(r"C:\Program Files (x86)\Steam\logs\vrserver.txt")

VMC_DEFAULT_PORT = 39539
VMC_LISTEN_SECONDS = 6.0


def _hr(label: str) -> None:
    print()
    print(f"-- {label} " + "-" * max(0, 70 - len(label)))


def check_processes() -> Dict[str, Optional[int]]:
    """Return ``{exe_name: pid_or_None}`` for the two UDCAP executables.

    Uses ``wmic`` for Windows-only process enumeration; falls back to
    ``tasklist`` if ``wmic`` is missing on newer Windows builds.
    """
    out: Dict[str, Optional[int]] = {"UdcapDriver.exe": None, "UDCAP_overlay.exe": None}
    try:
        result = subprocess.run(
            ["tasklist", "/FO", "CSV", "/NH"],
            capture_output=True, text=True, timeout=10, check=False,
        )
        for line in result.stdout.splitlines():
            for exe in out:
                if exe.lower() in line.lower():
                    parts = [p.strip().strip('"') for p in line.split(",")]
                    if len(parts) >= 2 and parts[1].isdigit():
                        out[exe] = int(parts[1])
                    break
    except FileNotFoundError:
        print("  [WARN] tasklist not found — cannot enumerate processes.")
    return out


def parse_udcap_config(path: Path) -> Dict[str, str]:
    """Pull the relevant <add key=...> entries from UdcapDriver.dll.config."""
    if not path.exists():
        return {}
    wanted = {
        "VMC_State", "VMC_IP", "VMC_PORT",
        "OSC_State", "OSC_IP", "OSC_PORT",
        "IsSteamOpen", "Controller_Priority", "Controller_Module",
        "L_DeadZone", "R_DeadZone",
        "L_TriggerOn", "R_TriggerOn",
        "L_GrabOn", "R_GrabOn",
        "L_TriggerValue_Min", "R_TriggerValue_Min",
    }
    out: Dict[str, str] = {}
    try:
        tree = ET.parse(str(path))
        for node in tree.iter("add"):
            key = node.get("key", "")
            val = node.get("value", "")
            if key in wanted:
                out[key] = val
    except ET.ParseError as exc:
        print(f"  [WARN] Could not parse {path}: {exc}")
    return out


def parse_udcap_steamvr_settings(path: Path) -> Dict[str, str]:
    """Read driver_UDCAP / communication_* sections from default.vrsettings."""
    if not path.exists():
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:  # noqa: BLE001
        print(f"  [WARN] Could not parse {path}: {exc}")
        return {}
    out = {}
    for section in ("driver_UDCAP", "communication_serial",
                    "communication_btserial", "communication_namedpipe"):
        if section in data:
            for k, v in data[section].items():
                out[f"{section}.{k}"] = str(v)
    return out


def scan_vrserver_log(path: Path, lookback_lines: int = 5000) -> Dict[str, object]:
    """Count ``udcap: Received named pipe data`` and other UDCAP signals."""
    result: Dict[str, object] = {
        "named_pipe_data_count": 0,
        "last_named_pipe_data_ts": None,
        "udcap_loaded": False,
        "tracking_updates": 0,
        "warnings_count": 0,
        "exists": path.exists(),
    }
    if not path.exists():
        return result
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
    except Exception as exc:  # noqa: BLE001
        print(f"  [WARN] Could not read {path}: {exc}")
        return result
    tail = lines[-lookback_lines:] if len(lines) > lookback_lines else lines
    npd_re = re.compile(r"udcap: UDCAP Server Info: Received named pipe data")
    loaded_re = re.compile(r"Loaded server driver udcap")
    track_re = re.compile(r"udcap: Controller that .* hand is tracking from has been updated")
    ts_re = re.compile(r"^(\w+ \w+ \d+ \d{4} \d+:\d+:\d+\.\d+)")
    for line in tail:
        if npd_re.search(line):
            result["named_pipe_data_count"] = int(result["named_pipe_data_count"]) + 1
            m = ts_re.match(line)
            if m:
                result["last_named_pipe_data_ts"] = m.group(1)
        if loaded_re.search(line):
            result["udcap_loaded"] = True
        if track_re.search(line):
            result["tracking_updates"] = int(result["tracking_updates"]) + 1
        if "[Warning]" in line and ("UDCAP" in line or "udcap" in line):
            result["warnings_count"] = int(result["warnings_count"]) + 1
    return result


def listen_for_vmc(port: int, seconds: float) -> Tuple[int, List[str]]:
    """Bind UDP ``port`` and count incoming OSC packets for ``seconds``.

    Returns ``(packet_count, sample_addresses)`` where ``sample_addresses``
    contains up to 5 unique OSC addresses observed (e.g.,
    ``/VMC/Ext/Bone/Pos``).  Empty list means port was bound but no packets
    arrived — suggests UDCAP is not broadcasting.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", port))
    except OSError as exc:
        print(f"  [WARN] bind UDP {port} failed: {exc}")
        sock.close()
        return -1, []
    sock.settimeout(0.25)
    deadline = time.perf_counter() + seconds
    count = 0
    samples: List[str] = []
    try:
        while time.perf_counter() < deadline:
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            count += 1
            # OSC address is a null-terminated padded string at the start.
            try:
                end = data.index(b"\x00")
                addr = data[:end].decode("ascii", errors="replace")
                if addr and addr not in samples and len(samples) < 5:
                    samples.append(addr)
            except ValueError:
                pass
    finally:
        sock.close()
    return count, samples


def main() -> int:
    print("=" * 76)
    print("UDCAP data-flow diagnostic")
    print("=" * 76)

    # -- 1. Process state -------------------------------------------------
    _hr("1. UDCAP user-space processes")
    procs = check_processes()
    main_running = procs["UdcapDriver.exe"] is not None
    overlay_running = procs["UDCAP_overlay.exe"] is not None
    for exe, pid in procs.items():
        status = f"PID {pid}" if pid is not None else "NOT RUNNING"
        marker = " [OK]" if pid is not None else " [X]"
        print(f"  {marker} {exe:25s} {status}")
    print()
    if not main_running:
        print(
            "  => UdcapDriver.exe is NOT running.  This is the user-space app that\n"
            "    actually reads the gloves over USB/BT and forwards data to the\n"
            "    SteamVR driver via a named pipe.  Without it, no finger/trigger/\n"
            "    grip data ever reaches SteamVR — every action value reads 0.\n"
            "    Launch it from the Start Menu (search 'UdcapDriver') or run:\n"
            "      \"C:\\Program Files\\UdcapDriver\\UdcapDriver.exe\""
        )
    if not overlay_running:
        print("  (UDCAP_overlay.exe missing too — both should auto-start with SteamVR.)")

    # -- 2. UDCAP config ---------------------------------------------------
    _hr("2. UdcapDriver.dll.config (user-space app settings)")
    cfg = parse_udcap_config(UDCAP_CONFIG_XML)
    if not cfg:
        print(f"  [WARN] {UDCAP_CONFIG_XML} not found or unparseable.")
    else:
        for k in sorted(cfg):
            print(f"    {k:30s} = {cfg[k]}")
    vmc_enabled = (cfg.get("VMC_State", "False").lower() == "true")
    vmc_port = int(cfg.get("VMC_PORT", VMC_DEFAULT_PORT))
    vmc_ip = cfg.get("VMC_IP", "127.0.0.1")
    print()
    if vmc_enabled:
        print(f"  [OK] VMC export is ENABLED — broadcasting to {vmc_ip}:{vmc_port}.")
        print("    Our retargeter can consume this via --path_b_port "
              f"{vmc_port} (Path B fallback).")
    else:
        print("  [X] VMC export is DISABLED.  To enable Path B fallback, open\n"
              "    UdcapDriver.exe => Settings => Streaming => toggle VMC ON.")
    if cfg.get("Controller_Priority", "false").lower() == "true":
        print(
            "  [!] Controller_Priority=True — UDCAP is configured to prefer the\n"
            "    underlying physical controller (e.g., the Pico/Quest controller)\n"
            "    over the glove sensors.  If gloves never override the controller\n"
            "    inputs, try setting this to False in the UdcapDriver UI."
        )

    # -- 3. SteamVR driver settings + log ---------------------------------
    _hr("3. SteamVR vrserver.txt — driver activation + named pipe traffic")
    log_info = scan_vrserver_log(STEAMVR_LOG)
    if not log_info["exists"]:
        print(f"  [WARN] {STEAMVR_LOG} does not exist — has SteamVR ever run?")
    else:
        print(f"    udcap driver loaded:        {log_info['udcap_loaded']}")
        print(f"    tracking-source updates:    {log_info['tracking_updates']}")
        print(f"    UDCAP-related warnings:     {log_info['warnings_count']}")
        print(f"    'Received named pipe data': {log_info['named_pipe_data_count']}")
        if log_info["last_named_pipe_data_ts"]:
            print(f"    last pipe-data timestamp:   {log_info['last_named_pipe_data_ts']}")
        print()
        npc = int(log_info["named_pipe_data_count"])
        if npc == 0:
            print(
                "  [X] NO 'Received named pipe data' messages.  UdcapDriver.exe is\n"
                "    not forwarding glove sensor frames to the SteamVR driver.\n"
                "    Likely causes:\n"
                "      • Gloves are not powered on / not paired / USB unplugged\n"
                "      • UdcapDriver.exe is running but no glove is connected\n"
                "      • The named-pipe channel is disabled in default.vrsettings"
            )
        elif npc < 10:
            print(
                f"  [!] Only {npc} 'Received named pipe data' messages — UdcapDriver\n"
                f"    sent something briefly (probably an init handshake) but is not\n"
                f"    streaming continuously.  Move your fingers; if the count does\n"
                f"    not grow on a fresh run, the gloves are not feeding sensor data."
            )
        else:
            print(f"  [OK] {npc} pipe-data messages — UdcapDriver IS forwarding to SteamVR.")

    # -- 4. VMC live test -------------------------------------------------
    _hr(f"4. VMC live test — listening on UDP {vmc_port} for {VMC_LISTEN_SECONDS:.0f}s")
    print("  Now is the time to MOVE YOUR FINGERS — VMC sends one packet per bone\n"
          "  per glove update tick (typically 30–60 Hz for hand bones).")
    count, samples = listen_for_vmc(vmc_port, VMC_LISTEN_SECONDS)
    if count == -1:
        print(
            f"  [!] Could not bind UDP {vmc_port} — another process already owns it.\n"
            f"    That process IS receiving VMC data, but we cannot peek.  Stop\n"
            f"    any other VMC consumer (Virtual Motion Capture, OSC apps, …)\n"
            f"    and re-run this script."
        )
    elif count == 0:
        if vmc_enabled:
            print(
                "  [X] 0 packets in {:.0f}s.  UDCAP claims VMC is enabled but is\n"
                "    not actually broadcasting.  This usually means the gloves\n"
                "    are not connected to UdcapDriver.exe.".format(VMC_LISTEN_SECONDS)
            )
        else:
            print(
                "  (VMC is disabled in UdcapDriver.dll.config — no packets expected.\n"
                "   Enable VMC in UdcapDriver UI to use Path B fallback.)"
            )
    else:
        print(
            f"  [OK] {count} VMC packets received in {VMC_LISTEN_SECONDS:.0f}s.\n"
            f"    Sample OSC addresses observed: {samples}\n"
            f"    Re-run teleop with: --path_b_port {vmc_port}"
        )

    # -- verdict ----------------------------------------------------------
    _hr("VERDICT")
    pipe_count = int(log_info.get("named_pipe_data_count", 0))
    if not main_running:
        print("  Critical: UdcapDriver.exe must be running.  Start it and re-test.")
        print("  Without it, the SteamVR driver creates virtual knuckles controllers")
        print("  but never receives finger sensor data — no binding work can fix this.")
    elif pipe_count <= 1 and count <= 0:
        print("  UdcapDriver.exe is running but sent <=1 named pipe message AND VMC")
        print("  is not broadcasting.  TWO possible causes (CHECK THE UDCAP UI FIRST):")
        print()
        print("  (A) MOST LIKELY -- gloves are 'Not Calibrated'.  UDCAP refuses to")
        print("      forward finger data on either named pipe or VMC until you run")
        print("      calibration.  Look for a 'Not Calibration / Please Calibration'")
        print("      banner in the UDCAP system-tray widget; press F1 (or click the")
        print("      Calibration(F1) button) and follow the open / fist / per-finger")
        print("      sequence for both hands.  This is by far the most common cause")
        print("      when UI > Devices shows 'Connected' with high FPS but no data")
        print("      reaches SteamVR.")
        print()
        print("  (B) Gloves are not actually connected.  Verify in UDCAP UI > Devices")
        print("      that both gloves show 'Connected' with battery > 0% and FPS > 30.")
        print("      If they don't, fix pairing / USB / battery first.")
        print()
        print("  After calibration, re-run this diagnostic: pipe_count should grow")
        print("  past 100 within seconds and the VMC live test should receive packets.")
    elif pipe_count > 10:
        print("  UdcapDriver IS forwarding finger frames to SteamVR.  If the teleop")
        print("  app still sees zero action values, the issue is binding routing")
        print("  inside SteamVR — re-run teleop with the latest 9.3 fix and check")
        print("  Manage Controller Bindings => UST Teleop GR1T2 Fourier in SteamVR UI.")
    elif count > 0:
        print("  VMC IS broadcasting.  The fastest fix is to bypass the SteamVR Input")
        print(f"  layer entirely: re-run teleop with --path_b_port {vmc_port}.")
    else:
        print("  Mixed signals — see individual sections above for details.")
    print("=" * 76)
    return 0


if __name__ == "__main__":
    sys.exit(main())
