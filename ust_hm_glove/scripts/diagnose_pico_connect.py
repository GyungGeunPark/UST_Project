"""Diagnose the PICO Connect -> SteamVR -> PC pipeline.

Layered probe to verify that PICO Connect (prism driver) is the active
tracker source for the 'PICO Connect -> SteamVR -> PC -> Isaac Lab'
pipeline introduced in 9.37.

Layers (probed top-down):

    1. PICO Connect Streaming Service process is alive.
    2. SteamVR has the prism driver loaded (vrpathreg query).
    3. SteamVR Manage Add-Ons state for prism / udcap / VD Streamer.
    4. SteamVR HMD identity matches PICO (or VD's reuse of the Oculus
       protocol when VD is still active).
    5. PICO Motion Trackers visible via OpenVR.
    6. tracker_binding_pico_connect.json maps every PICO tracker to a
       valid role.

Each failed layer prints actionable next steps.  Pass ``--json`` to emit
a machine-readable summary instead of the human report.

Usage::

    python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect
    python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect --json
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional


# ── Defaults / constants ─────────────────────────────────────────────────
DEFAULT_BINDING_JSON = Path(
    "ust_ws/ust_hm_glove/config/tracker_binding_pico_connect.json"
)
PICO_CONNECT_PROCESS_NAMES = (
    "PICO Connect.exe",
    "PICOConnectService.exe",
    "PICOStreamingService.exe",
    "PICOStreamingAssistant.exe",
)
# 9.39 — corrects the historic mislabel (memory.md §10.43 / gotcha #29):
#   * `pico`  = PICO Connect's external openvr driver (PICO Inc.)
#   * `prism` = Steam Link's bundled driver (Valve)
# Both forms must be searched because (a) different PICO Connect versions
# put the folder under different names and (b) Steam Link is a valid
# alternative streaming layer for the same HMD.  Earlier versions of this
# script only searched `prism` and reported FALSE FAILures when the
# (correctly-installed) `pico` folder was the actual provider.
KNOWN_PICO_DRIVER_NAMES = ("pico", "prism", "pico_connect")
# 9.40 — VD's actual driver folder name in vrpathreg show is `VirtualDesktop`
# (per user's vrpathreg show output 2026-05-10).  The legacy name
# `oculus_virtualdesktop` was for older VD versions.  Search BOTH so the
# HMD-provider-conflict detection (gotcha #29) actually fires.
KNOWN_VD_DRIVER_NAMES = ("virtualdesktop", "virtual_desktop", "oculus_virtualdesktop")
KNOWN_GLOVE_DRIVER_NAMES = ("udcap",)
# 9.40 — drivers that act as HMD providers (redirectsDisplay=true in
# their driver.vrdrivermanifest).  When more than ONE of these is
# registered with vrpathreg, vrserver cannot decide which driver owns
# the HMD and openvr.init() blocks indefinitely (gotcha #29).
HMD_PROVIDER_KEYWORDS = (
    "pico",
    "prism",
    "virtualdesktop",
    "virtual_desktop",
    "oculus_virtualdesktop",
    "alvr",
)
# Filesystem probe paths.  PICO Connect 10.x installs the driver under
# `C:\Program Files\PICO Connect\openvr_driver\` regardless of whether
# vrpathreg sees it -- newer Connect versions auto-load the driver
# in-process at SteamVR launch time and skip vrpathreg adddriver.
PICO_CONNECT_DRIVER_INSTALL_DIRS = (
    Path(r"C:\Program Files\PICO Connect\openvr_driver"),
    Path(r"C:\Program Files (x86)\PICO Connect\openvr_driver"),
    Path(r"C:\Program Files\Streaming Assistant\driver"),  # legacy PICO Streaming Assistant
)
STEAMVR_BUNDLED_DRIVERS_DIR = Path(
    r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers"
)
PICO_SERIAL_PREFIXES = ("pmt_", "picobt_", "pico_motion_tracker_")
PICO_MANUFACTURERS = ("pico", "pico immersive pte. ltd.")
PICO_MODELS = ("pico motion tracker", "pmt", "pico body tracker")


# ── Layer probes ─────────────────────────────────────────────────────────


def probe_pico_connect_process() -> Dict[str, Any]:
    """Layer 1: is PICO Connect Streaming Service running on the PC?"""
    if sys.platform != "win32":
        return {"ok": False, "reason": "Non-Windows platform; PICO Connect is Windows-only."}
    try:
        res = subprocess.run(
            ["tasklist", "/FO", "CSV", "/NH"],
            capture_output=True, text=True, timeout=5, check=False,
        )
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "reason": f"tasklist failed: {type(exc).__name__}: {exc}"}
    running: Dict[str, Optional[int]] = {name: None for name in PICO_CONNECT_PROCESS_NAMES}
    for line in res.stdout.splitlines():
        low = line.lower()
        for name in PICO_CONNECT_PROCESS_NAMES:
            if name.lower() in low:
                parts = [p.strip().strip('"') for p in line.split(",")]
                if len(parts) >= 2 and parts[1].isdigit():
                    running[name] = int(parts[1])
                break
    alive = {n: pid for n, pid in running.items() if pid is not None}
    return {
        "ok": bool(alive),
        "alive": alive,
        "checked": list(PICO_CONNECT_PROCESS_NAMES),
        "reason": (
            ""
            if alive
            else "PICO Connect Streaming Service is not running. "
                 "Launch 'PICO Connect' from the Start Menu, sign in, and "
                 "start streaming the headset."
        ),
    }


def _vrpathreg_show_full() -> str:
    """Run ``vrpathreg show`` and return the entire stdout for transparency."""
    candidates = [
        Path(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"),
        Path(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win32\vrpathreg.exe"),
    ]
    vrpath = next((p for p in candidates if p.exists()), None)
    if vrpath is None:
        return ""
    try:
        res = subprocess.run(
            [str(vrpath), "show"],
            capture_output=True, text=True, timeout=5, check=False,
        )
    except Exception:  # noqa: BLE001
        return ""
    return res.stdout


def _list_steamvr_drivers(raw_output: str) -> List[str]:
    """Layer 2 helper: filter vrpathreg show output for known driver lines."""
    drivers: List[str] = []
    for line in raw_output.splitlines():
        low = line.lower().strip()
        # Lines look like:  "External Drivers: ..." or
        # "C:\Program Files\PICO Connect\openvr_driver\"
        for name in (KNOWN_PICO_DRIVER_NAMES + KNOWN_VD_DRIVER_NAMES + KNOWN_GLOVE_DRIVER_NAMES):
            if name in low:
                drivers.append(line.strip())
                break
    return drivers


def _probe_pico_driver_filesystem() -> Dict[str, Any]:
    """9.39 — direct filesystem probe.  PICO Connect 10.x auto-loads the
    driver in-process at SteamVR launch time and skips ``vrpathreg
    adddriver`` -- so the driver folder may be present and functional
    even when ``vrpathreg show`` does NOT list it.  Treat the install
    dir's existence (with a ``driver.vrdrivermanifest`` inside) as
    proof-of-installation independent of the vrpathreg registry.
    """
    for d in PICO_CONNECT_DRIVER_INSTALL_DIRS:
        if not d.exists():
            continue
        # Look for an inner driver folder containing a manifest -- the
        # actual driver layout is usually
        #   <openvr_driver>/<driver_name>/driver.vrdrivermanifest
        # but some legacy installs put the manifest at the root.
        manifests: List[Path] = list(d.rglob("driver.vrdrivermanifest"))
        if manifests:
            inner_names = sorted({m.parent.name for m in manifests})
            return {
                "found": True,
                "install_dir": str(d),
                "inner_driver_names": inner_names,
                "manifests": [str(m) for m in manifests],
            }
    return {"found": False, "install_dir": None, "inner_driver_names": [], "manifests": []}


def _probe_steamvr_bundled_prism() -> bool:
    """9.39 — Steam Link's `prism` driver lives bundled inside SteamVR."""
    return (STEAMVR_BUNDLED_DRIVERS_DIR / "prism").exists()


def _detect_hmd_providers(raw_vrpathreg: str) -> List[Dict[str, str]]:
    """9.40 — parse vrpathreg show output and return EVERY external driver
    line whose name OR path contains an HMD-provider keyword.

    Returns a list of ``{"name": str, "path": str, "matched": str}``.
    When this list has length >= 2, vrserver has an HMD-provider conflict
    that prevents openvr.init() from completing — the user must disable
    all but one in SteamVR > Settings > Manage Add-Ons (or use
    ``vrpathreg removedriver <path>``).  See gotcha #29.
    """
    providers: List[Dict[str, str]] = []
    in_external_block = False
    for line in raw_vrpathreg.splitlines():
        low = line.lower()
        if "external drivers" in low:
            in_external_block = True
            continue
        if not in_external_block:
            continue
        stripped = line.strip()
        if not stripped:
            # Blank line ends the External Drivers block.  Defensive: stop
            # parsing so we don't pick up unrelated trailing content.
            in_external_block = False
            continue
        # Lines look like:  "pico : C:/Program Files/PICO Connect/openvr_driver/"
        # The driver name is before ":" and the path is after.  Some entries
        # use Windows backslashes; others use forward slashes.  We also
        # handle lines that omit the name and only show the path.
        if ":" in stripped:
            # Split only on the FIRST colon — paths contain "C:".  Driver
            # name is short (no colon) and comes before the first " : ".
            sep = stripped.find(" : ")
            if sep == -1:
                # No " : " separator -> assume the whole stripped line is a path
                name = ""
                path = stripped
            else:
                name = stripped[:sep].strip()
                path = stripped[sep + 3:].strip()
        else:
            name = ""
            path = stripped
        name_low = name.lower()
        path_low = path.lower()
        for kw in HMD_PROVIDER_KEYWORDS:
            if kw in name_low or kw in path_low:
                providers.append({"name": name, "path": path, "matched": kw})
                break
    return providers


def probe_steamvr_drivers() -> Dict[str, Any]:
    """Layer 2: which SteamVR drivers are registered AND/OR installed?

    9.39 — combines three signals:
      1. ``vrpathreg show`` text-search for known driver names
      2. PICO Connect driver folder filesystem existence (PICO Connect
         10.x bypasses vrpathreg; folder = installed)
      3. Steam Link's bundled `prism` folder under SteamVR install
    A driver is considered "available" if ANY of the three says so.
    """
    raw = _vrpathreg_show_full()
    drivers = _list_steamvr_drivers(raw)
    pico_fs = _probe_pico_driver_filesystem()
    has_steamlink_prism = _probe_steamvr_bundled_prism()
    hmd_providers = _detect_hmd_providers(raw)

    has_pico_in_vrpathreg = any(
        any(name in d.lower() for name in ("pico", "prism", "pico_connect"))
        for d in drivers
    )
    has_pico_filesystem = pico_fs["found"]
    has_pico_anywhere = has_pico_in_vrpathreg or has_pico_filesystem
    has_vd = any(name in d.lower() for d in drivers for name in KNOWN_VD_DRIVER_NAMES)

    if not raw and not pico_fs["found"]:
        return {
            "ok": False,
            "drivers_found": drivers,
            "vrpathreg_raw": raw,
            "pico_filesystem": pico_fs,
            "steamlink_prism_bundled": has_steamlink_prism,
            "hmd_providers": hmd_providers,
            "reason": (
                "Could not enumerate SteamVR drivers (vrpathreg.exe missing) AND "
                "PICO Connect driver folder not found.  Either SteamVR is not "
                "installed at the default Steam path OR PICO Connect was never "
                "installed.  Check Steam > Library > SteamVR > Manage Add-Ons "
                "in the UI."
            ),
        }

    # 9.40 — HMD provider conflict (gotcha #29).  This is the dominant
    # cause of openvr.init() hang once PICO Connect is properly installed.
    # When the user's vrpathreg lists pico + VirtualDesktop simultaneously,
    # vrserver cannot decide which driver owns the HMD and openvr.init
    # blocks indefinitely waiting for the IPC handshake.  Treat this as
    # a hard FAIL and emit ready-to-paste removal commands.
    if len(hmd_providers) >= 2:
        names = [p["name"] or p["matched"] for p in hmd_providers]
        # Ranked policy (memory.md §10.43):
        #   pico  > prism > VirtualDesktop > oculus_virtualdesktop > alvr
        # Keep the first match by this ranking; suggest removing all others.
        priority = ("pico", "prism", "virtualdesktop", "virtual_desktop",
                    "oculus_virtualdesktop", "alvr")
        sorted_providers = sorted(
            hmd_providers,
            key=lambda p: priority.index(p["matched"])
            if p["matched"] in priority else 99,
        )
        keeper = sorted_providers[0]
        to_remove = sorted_providers[1:]
        removal_cmds = "\n".join(
            f"          vrpathreg removedriver \"{p['path']}\""
            for p in to_remove
        )
        return {
            "ok": False,
            "drivers_found": drivers,
            "vrpathreg_raw": raw,
            "pico_filesystem": pico_fs,
            "pico_in_vrpathreg": has_pico_in_vrpathreg,
            "steamlink_prism_bundled": has_steamlink_prism,
            "virtual_desktop": has_vd,
            "hmd_providers": hmd_providers,
            "hmd_conflict": True,
            "reason": (
                f"HMD PROVIDER CONFLICT (gotcha #29): {len(hmd_providers)} "
                f"HMD-providing drivers are registered with vrpathreg.\n"
                f"Detected providers: {', '.join(names)}.\n\n"
                f"This is the most common cause of openvr.init() hang -- "
                f"vrserver cannot decide which driver owns the HMD, so the "
                f"client IPC handshake never completes.\n\n"
                f"FIX (recommended -- keep {keeper['name'] or keeper['matched']!r} as primary):\n"
                f"  Option A) SteamVR > Settings > Manage Add-Ons UI:\n"
                f"            Toggle OFF every HMD provider EXCEPT "
                f"{keeper['name'] or keeper['matched']!r}.\n"
                f"  Option B) PowerShell (one-shot, requires SteamVR closed):\n"
                f"          $vrpath = "
                f"\"C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR\\bin\\win64\\vrpathreg.exe\"\n"
                f"{removal_cmds}\n"
                f"          (Run each line; use the path string verbatim.)\n\n"
                f"After removal, restart SteamVR and re-run this diagnose."
            ),
        }

    if has_pico_anywhere:
        reason = ""
    else:
        reason = (
            "Neither PICO Connect's openvr_driver folder (filesystem probe) NOR "
            "any 'pico'/'prism' entry in vrpathreg show was found.  PICO Connect "
            "10.x normally installs the driver under "
            "C:\\Program Files\\PICO Connect\\openvr_driver\\ -- if that "
            "directory is missing, reinstall PICO Connect."
        )

    return {
        "ok": has_pico_anywhere,
        "drivers_found": drivers,
        "vrpathreg_raw": raw,
        "pico_filesystem": pico_fs,
        "pico_in_vrpathreg": has_pico_in_vrpathreg,
        "steamlink_prism_bundled": has_steamlink_prism,
        "virtual_desktop": has_vd,
        "hmd_providers": hmd_providers,
        "hmd_conflict": False,
        "reason": reason,
    }


def probe_openvr_devices(init_timeout_sec: float = 15.0) -> Dict[str, Any]:
    """Layers 4+5: HMD identity and tracker inventory via openvr.

    9.38 — uses the SteamVRSampler watchdog so a missing SteamVR /
    unstreamed HMD surfaces as a TimeoutError after ``init_timeout_sec``
    seconds instead of blocking the entire diagnose run indefinitely
    (silent hang reported by user against 9.37 build).  See gotcha #30.
    """
    try:
        import openvr
    except ImportError as exc:
        return {"ok": False, "reason": f"openvr not installed: {exc}"}
    # Reuse the watchdog so the failure mode matches what the live
    # teleop entry-point produces.  Falls back to direct openvr.init
    # if the import path can't be resolved (e.g. running this script
    # without setting PYTHONPATH=. from the repo root).
    try:
        from ust_ws.ust_hm_glove.teleop.vr_sampler import (
            _init_openvr_with_timeout,
        )
        system = _init_openvr_with_timeout(timeout_sec=init_timeout_sec)
    except TimeoutError as exc:
        return {
            "ok": False,
            "timeout": True,
            "reason": (
                f"openvr.init timed out after {init_timeout_sec:.0f}s.\n"
                f"\n"
                f"*** ROOT CAUSE PRIORITY ORDER (per memory.md gotcha #29 "
                f"+ ValveSoftware/openvr#1719 + arvivr.zendesk PICO 4 Ultra):\n"
                f"\n"
                f"  1) HMD PROVIDER CONFLICT (most common!): multiple HMD "
                f"drivers (pico + VirtualDesktop + prism + ...) are "
                f"registered in vrpathreg -- vrserver cannot decide which "
                f"owns the HMD and the IPC handshake never completes.\n"
                f"     -> Check Layer 2 output above; if it lists more than "
                f"1 HMD provider, see the ready-to-paste removal commands "
                f"in the Layer 2 [FAIL] reason.  This must be cleared "
                f"BEFORE openvr.init can ever succeed.\n"
                f"\n"
                f"  2) HMD STANDBY (yellow icon in SteamVR status strip): "
                f"the HMD is registered but proximity sensor is inactive.\n"
                f"     -> Put the headset on or hold your hand over the "
                f"proximity sensor, move it slightly to wake the IMU, then "
                f"watch the SteamVR status strip until the HMD icon turns "
                f"GREEN.  Re-run this diagnose -- Layer 3-5 should complete "
                f"in <2 seconds.\n"
                f"\n"
                f"  3) SteamVR (vrserver.exe) not running at all -- launch "
                f"Steam > Library > Tools > SteamVR.\n"
                f"\n"
                f"  4) SteamVR in 'Starting up...' dialog awaiting user "
                f"input -- click through it.\n"
                f"\n"
                f"Watchdog detail: {exc}"
            ),
        }
    except ImportError:
        # Fallback when running outside the package layout.
        try:
            system = openvr.init(openvr.VRApplication_Other)
        except Exception as exc:  # noqa: BLE001
            return {
                "ok": False,
                "reason": (
                    f"openvr.init failed: {type(exc).__name__}: {exc}.  "
                    "SteamVR is probably not running; launch SteamVR first."
                ),
            }
    except Exception as exc:  # noqa: BLE001
        return {
            "ok": False,
            "reason": (
                f"openvr.init failed: {type(exc).__name__}: {exc}.  "
                "SteamVR is probably not running; launch SteamVR first."
            ),
        }
    try:
        devices: List[Dict[str, Any]] = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            cls = system.getTrackedDeviceClass(i)
            if cls == openvr.TrackedDeviceClass_Invalid:
                continue
            try:
                serial = system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_SerialNumber_String
                )
            except Exception:
                serial = ""
            try:
                manufacturer = system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_ManufacturerName_String
                )
            except Exception:
                manufacturer = ""
            try:
                model = system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_ModelNumber_String
                )
            except Exception:
                model = ""
            devices.append(
                {
                    "idx": i,
                    "class": int(cls),
                    "serial": serial,
                    "manufacturer": manufacturer,
                    "model": model,
                }
            )
        hmd_rows = [d for d in devices if d["class"] == int(openvr.TrackedDeviceClass_HMD)]
        tracker_rows = [
            d for d in devices if d["class"] == int(openvr.TrackedDeviceClass_GenericTracker)
        ]
        # PICO classification matches enumerate_trackers.py logic.
        pico_trackers: List[Dict[str, Any]] = []
        for d in tracker_rows:
            sn_low = d["serial"].lower()
            mn_low = d["manufacturer"].lower()
            md_low = d["model"].lower()
            if (
                any(sn_low.startswith(pfx) for pfx in PICO_SERIAL_PREFIXES)
                or mn_low in PICO_MANUFACTURERS
                or any(p in md_low for p in PICO_MODELS)
            ):
                pico_trackers.append(d)
        return {
            "ok": bool(pico_trackers) and bool(hmd_rows),
            "hmd": hmd_rows,
            "trackers": tracker_rows,
            "pico_trackers": pico_trackers,
            "reason": (
                ""
                if (pico_trackers and hmd_rows)
                else (
                    "No PICO Motion Trackers visible in OpenVR. "
                    "Verify PICO Connect Devices panel shows the trackers paired "
                    "and 'streaming' before retrying."
                )
            ),
        }
    finally:
        try:
            openvr.shutdown()
        except Exception:
            pass


def probe_binding_file(binding_path: Path) -> Dict[str, Any]:
    """Layer 6: does tracker_binding_pico_connect.json look filled in?"""
    if not binding_path.exists():
        return {
            "ok": False,
            "reason": (
                f"{binding_path} not found.  Run "
                f"`python -X utf8 -m ust_ws.ust_hm_glove.scripts.enumerate_trackers "
                f"--out {binding_path}` while PICO Connect is streaming "
                f"to generate the template."
            ),
        }
    try:
        data = json.loads(binding_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "reason": f"Failed to parse {binding_path}: {exc}"}
    trackers = data.get("trackers", {})
    todo_keys = [
        sn
        for sn, info in trackers.items()
        if str(info.get("role", "")).lower().startswith("todo")
        or sn.lower().startswith("pmt_replace_me")
    ]
    return {
        "ok": bool(trackers) and not todo_keys,
        "trackers": list(trackers.keys()),
        "todo_keys": todo_keys,
        "reason": (
            ""
            if trackers and not todo_keys
            else (
                "tracker_binding_pico_connect.json still has placeholder/TODO "
                f"entries: {todo_keys}.  Edit the file and assign each PICO "
                f"Motion Tracker serial to one of: waist, left_forearm, "
                f"right_forearm, left_ankle, right_ankle (matching where you "
                f"physically wear the tracker)."
            )
        ),
    }


# ── Reporter ─────────────────────────────────────────────────────────────


def render_report(report: Dict[str, Any]) -> str:
    out: List[str] = []
    layers = (
        ("1. PICO Connect Streaming Service", report["process"]),
        ("2. SteamVR drivers (pico / prism / udcap; vrpathreg + filesystem)", report["drivers"]),
        ("3-5. OpenVR devices (HMD + PICO trackers)", report["openvr"]),
        ("6. tracker_binding_pico_connect.json", report["binding"]),
    )
    out.append("=" * 72)
    out.append(" PICO Connect -> SteamVR -> PC -> Isaac Lab pipeline diagnosis")
    out.append("=" * 72)
    overall_ok = True
    for title, layer in layers:
        ok = bool(layer.get("ok"))
        overall_ok &= ok
        mark = "[ OK ]" if ok else "[FAIL]"
        out.append(f"\n{mark} {title}")
        if not ok and layer.get("reason"):
            for line in str(layer["reason"]).splitlines():
                out.append(f"        {line}")
    proc = report["process"]
    if proc.get("alive"):
        out.append(f"\n  Streaming processes alive: {proc['alive']}")
    drv = report["drivers"]
    if drv.get("drivers_found"):
        out.append("\n  Detected SteamVR drivers (vrpathreg show grep):")
        for d in drv["drivers_found"]:
            out.append(f"    - {d}")
    pico_fs = drv.get("pico_filesystem", {})
    if pico_fs.get("found"):
        out.append(
            f"\n  PICO Connect driver folder (filesystem probe): {pico_fs['install_dir']!r}"
        )
        if pico_fs.get("inner_driver_names"):
            out.append(
                f"    inner driver folder(s): {pico_fs['inner_driver_names']}"
            )
        out.append(
            "    note: PICO Connect 10.x auto-loads this driver in-process at "
            "SteamVR launch -- it may NOT appear in vrpathreg show output, "
            "and that is FINE."
        )
    elif drv.get("vrpathreg_raw"):
        out.append(
            "\n  PICO Connect driver folder NOT found at any known path:"
        )
        for d in PICO_CONNECT_DRIVER_INSTALL_DIRS:
            out.append(f"    - {d}  (missing)")
    if drv.get("steamlink_prism_bundled"):
        out.append(
            "\n  Steam Link `prism` driver (bundled): installed under SteamVR/drivers/prism"
        )
    if drv.get("virtual_desktop") and drv.get("pico_in_vrpathreg"):
        out.append(
            "\n  WARNING: pico/prism AND oculus_virtualdesktop are both registered.\n"
            "    For the 9.37 PICO Connect pipeline, set 'Virtual Desktop\n"
            "    Streamer (Quest)' OFF in SteamVR Add-Ons to avoid duplicate\n"
            "    HMD/controller injection (see gotcha #29)."
        )
    raw = drv.get("vrpathreg_raw", "")
    if raw:
        out.append("\n  --- vrpathreg show (full output) ---")
        for line in raw.splitlines():
            out.append(f"    {line}")
        out.append("  --- end vrpathreg show ---")
    ovr = report["openvr"]
    if ovr.get("hmd"):
        out.append("\n  HMD:")
        for h in ovr["hmd"]:
            out.append(
                f"    serial={h['serial']!r} manufacturer={h['manufacturer']!r} "
                f"model={h['model']!r}"
            )
    if ovr.get("pico_trackers"):
        out.append(f"\n  PICO Motion Trackers ({len(ovr['pico_trackers'])}):")
        for t in ovr["pico_trackers"]:
            out.append(
                f"    serial={t['serial']!r} model={t['model']!r}"
            )
    elif ovr.get("trackers"):
        out.append(
            f"\n  Generic trackers seen ({len(ovr['trackers'])}) but none classified as PICO:"
        )
        for t in ovr["trackers"]:
            out.append(
                f"    serial={t['serial']!r} manufacturer={t['manufacturer']!r} "
                f"model={t['model']!r}"
            )
    bnd = report["binding"]
    if bnd.get("trackers"):
        out.append(
            f"\n  Binding file lists {len(bnd['trackers'])} tracker entries; "
            f"placeholders remaining: {len(bnd.get('todo_keys', []))}."
        )
    out.append("")
    out.append("=" * 72)
    out.append(
        f" Overall: {'PIPELINE READY' if overall_ok else 'NOT READY -- see [FAIL] lines above'}"
    )
    out.append("=" * 72)
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binding",
        type=Path,
        default=DEFAULT_BINDING_JSON,
        help="Path to tracker_binding_pico_connect.json (default: %(default)s).",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of a report.")
    parser.add_argument(
        "--openvr-timeout",
        type=float,
        default=15.0,
        help="9.38 -- watchdog timeout (seconds) for the OpenVR layer probe. "
             "If openvr.init() does not respond within this window the "
             "probe marks Layer 3-5 as FAIL with a 'SteamVR not running' "
             "reason instead of hanging the diagnose run indefinitely. "
             "Default 15s (lower than the teleop default 30s because the "
             "diagnose script is meant to be re-runnable quickly).  Set 0 "
             "to disable.",
    )
    args = parser.parse_args()

    # Force line-buffered stdout so the per-layer progress prints below
    # are not block-buffered (which made the 'silent hang' impossible to
    # localise on the user's Korean Windows console -- gotcha #30).
    try:
        sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        pass

    print("=" * 72, flush=True)
    print(" PICO Connect -> SteamVR -> PC -> Isaac Lab pipeline diagnosis", flush=True)
    print("=" * 72, flush=True)

    report: Dict[str, Any] = {}

    # Each probe is run with a visible 'running...' / 'done' marker so
    # the user can tell which layer hung (or how long each step took).
    def _run(label: str, fn) -> Dict[str, Any]:
        import time as _t
        t0 = _t.perf_counter()
        print(f"  -> {label} probing ...", flush=True)
        try:
            result = fn()
        except BaseException as exc:  # noqa: BLE001 — visibility above all
            elapsed = _t.perf_counter() - t0
            print(
                f"     {label} CRASHED in {elapsed:.1f}s: "
                f"{type(exc).__name__}: {exc}",
                flush=True,
            )
            return {"ok": False, "reason": f"probe crashed: {exc}"}
        elapsed = _t.perf_counter() - t0
        ok = bool(result.get("ok"))
        mark = "[ OK ]" if ok else "[FAIL]"
        print(f"     {label} {mark} in {elapsed:.1f}s", flush=True)
        return result

    report["process"] = _run("Layer 1 (PICO Connect process)", probe_pico_connect_process)
    report["drivers"] = _run("Layer 2 (SteamVR drivers)", probe_steamvr_drivers)
    # Layers 3-5 are the slow / hangy one — pass the configurable timeout.
    report["openvr"] = _run(
        f"Layer 3-5 (OpenVR HMD+trackers, timeout={args.openvr_timeout:.0f}s)",
        lambda: probe_openvr_devices(init_timeout_sec=float(args.openvr_timeout)),
    )
    report["binding"] = _run(
        "Layer 6 (tracker_binding json)",
        lambda: probe_binding_file(args.binding.resolve()),
    )

    report["overall_ok"] = all(layer.get("ok") for layer in report.values() if isinstance(layer, dict))

    print("", flush=True)
    if args.json:
        print(json.dumps(report, indent=2, default=str), flush=True)
    else:
        print(render_report(report), flush=True)
    return 0 if report["overall_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
