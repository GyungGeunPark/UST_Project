"""Diagnose the PICO Connect -> SteamVR -> Isaac Lab pipeline (grip track).

Layered probe to verify that PICO Connect (prism driver) is the active
tracker source for the 'PICO Connect -> SteamVR -> PC -> Isaac Lab'
pipeline introduced in 9.37 for the ust_hm_grip teleop track.

Layers (probed top-down):

    1. PICO Connect Streaming Service process is alive.
    2. SteamVR has the prism driver loaded (vrpathreg query).
    3. SteamVR Manage Add-Ons state for prism / udcap / VD Streamer.
    4. SteamVR HMD identity matches PICO (or VD's reuse of the Oculus
       protocol when VD is still active).
    5. PICO Motion Trackers visible via OpenVR.
    6. tracker_binding_pico_connect.json maps every PICO tracker the grip
       track needs to a valid role (left_forearm / right_forearm strictly,
       waist optionally).

Each failed layer prints actionable next steps.  Pass ``--json`` to emit
a machine-readable summary instead of the human report.

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pico_connect
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pico_connect --json

Companion to ``scripts/diagnose_controller_raw.py`` (which probes the
PICO controller buttons via the Action API) and
``scripts/diagnose_gripper.py`` (which probes the resolved gripper
trigger/grip values along the same code path as run_teleop).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ── stdout hardening (9.38) ─────────────────────────────────────────────
# Force line buffering so progress messages flush even when stdout is
# redirected (CI, log capture, pytest -s, etc.).
try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _say(msg: str) -> None:
    """Print a progress line with explicit flush."""
    print(msg, flush=True)


# ── Defaults / constants ─────────────────────────────────────────────────
DEFAULT_BINDING_JSON = Path(
    "ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json"
)
PICO_CONNECT_PROCESS_NAMES = (
    "PICO Connect.exe",
    "PICOConnectService.exe",
    "PICOStreamingService.exe",
    "PICOStreamingAssistant.exe",
)
KNOWN_PICO_DRIVER_NAMES = ("prism",)
KNOWN_VD_DRIVER_NAMES = ("oculus_virtualdesktop",)
KNOWN_GLOVE_DRIVER_NAMES = ("udcap",)
PICO_SERIAL_PREFIXES = ("pmt_", "picobt_", "pico_motion_tracker_")
PICO_MANUFACTURERS = ("pico", "pico immersive pte. ltd.")
PICO_MODELS = ("pico motion tracker", "pmt", "pico body tracker")

# Roles that the grip retargeter may consume.  waist is OPTIONAL (only
# the WaistEnabled env variant uses it); the two forearm trackers are the
# fallback wrist-EEF source (controller pose is primary).
GRIP_ROLES_REQUIRED = ("left_forearm", "right_forearm")
GRIP_ROLES_OPTIONAL = ("waist",)


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


def _list_steamvr_drivers() -> List[str]:
    """Layer 2 helper: enumerate SteamVR drivers via vrpathreg.exe."""
    candidates = [
        Path(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"),
        Path(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win32\vrpathreg.exe"),
    ]
    vrpath = next((p for p in candidates if p.exists()), None)
    if vrpath is None:
        return []
    try:
        res = subprocess.run(
            [str(vrpath), "show"],
            capture_output=True, text=True, timeout=5, check=False,
        )
    except Exception:  # noqa: BLE001
        return []
    drivers: List[str] = []
    for line in res.stdout.splitlines():
        low = line.lower().strip()
        for name in (KNOWN_PICO_DRIVER_NAMES + KNOWN_VD_DRIVER_NAMES + KNOWN_GLOVE_DRIVER_NAMES):
            if name in low:
                drivers.append(line.strip())
                break
    return drivers


def probe_steamvr_drivers() -> Dict[str, Any]:
    """Layer 2: which SteamVR drivers are registered?"""
    drivers = _list_steamvr_drivers()
    if not drivers:
        return {
            "ok": False,
            "drivers_found": drivers,
            "reason": (
                "Could not enumerate SteamVR drivers via vrpathreg.exe. "
                "Either SteamVR is not installed at the default Steam path "
                "or the runtime probe could not be located.  Check Steam > "
                "Library > SteamVR > Manage Add-Ons in the UI as a manual "
                "fallback."
            ),
        }
    has_prism = any("prism" in d.lower() for d in drivers)
    has_vd = any(name in d.lower() for d in drivers for name in KNOWN_VD_DRIVER_NAMES)
    has_udcap = any(name in d.lower() for d in drivers for name in KNOWN_GLOVE_DRIVER_NAMES)
    return {
        "ok": has_prism,
        "drivers_found": drivers,
        "prism": has_prism,
        "virtual_desktop": has_vd,
        "udcap": has_udcap,
        "reason": (
            ""
            if has_prism
            else "prism (PICO Connect) driver is NOT registered with SteamVR. "
                 "Install PICO Connect (it ships the driver), then in SteamVR "
                 "go to Settings > Manage Add-Ons and ensure prism is ON."
        ),
    }


def _vrserver_running() -> Optional[bool]:
    """Return True/False if we can determine, None on uncertainty.

    Quick non-blocking check for the SteamVR core process.  Used to
    short-circuit ``probe_openvr_devices`` so we never block in
    ``openvr.init`` for 30+ seconds when SteamVR is not even up.
    """
    if sys.platform != "win32":
        return None
    try:
        res = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq vrserver.exe", "/FO", "CSV", "/NH"],
            capture_output=True, text=True, timeout=5, check=False,
        )
    except Exception:  # noqa: BLE001
        return None
    return "vrserver.exe" in (res.stdout or "").lower()


def _init_openvr_with_timeout(openvr_module, timeout_s: float) -> Tuple[Any, Optional[BaseException]]:
    """Call openvr.init on a daemon thread and join with timeout."""
    box: Dict[str, Any] = {"system": None, "exc": None, "done": False}

    def _runner() -> None:
        try:
            box["system"] = openvr_module.init(openvr_module.VRApplication_Other)
        except BaseException as exc:  # noqa: BLE001
            box["exc"] = exc
        finally:
            box["done"] = True

    th = threading.Thread(target=_runner, name="openvr-init", daemon=True)
    th.start()
    th.join(timeout=timeout_s)
    if not box["done"]:
        return None, TimeoutError(
            f"openvr.init() did not return within {timeout_s:.0f}s"
        )
    return box["system"], box["exc"]


def probe_openvr_devices(init_timeout_s: float = 60.0) -> Dict[str, Any]:
    """Layers 4+5: HMD identity and tracker inventory via openvr.

    9.38: pre-checks SteamVR via vrserver.exe and wraps openvr.init in a
    watchdog so this layer never blocks the diagnose script for more
    than ``init_timeout_s`` seconds.
    """
    # Pre-check: don't waste time inside openvr.init when SteamVR is dead.
    running = _vrserver_running()
    if running is False:
        return {
            "ok": False,
            "reason": (
                "SteamVR (vrserver.exe) is not running.  Launch SteamVR "
                "from Steam > Library > Tools > SteamVR before running "
                "this diagnostic, otherwise openvr.init would block for "
                "30+ seconds while OpenVR tries to auto-start it."
            ),
        }
    _say("[diagnose_pico_connect] probe layer 3-5: importing openvr...")
    try:
        import openvr
    except ImportError as exc:
        return {"ok": False, "reason": f"openvr not installed: {exc}"}
    _say(
        f"[diagnose_pico_connect] probe layer 3-5: calling openvr.init "
        f"(timeout={init_timeout_s:.0f}s)..."
    )
    t0 = time.time()
    system, init_exc = _init_openvr_with_timeout(openvr, init_timeout_s)
    if isinstance(init_exc, TimeoutError):
        return {
            "ok": False,
            "reason": (
                f"openvr.init() did not return within {init_timeout_s:.0f}s.  "
                "SteamVR may be hung or two HMD-redirecting drivers are "
                "fighting (gotcha #29).  Close SteamVR fully, ensure only "
                "ONE of prism / Virtual Desktop Streamer is enabled, then "
                "relaunch SteamVR and retry."
            ),
        }
    if init_exc is not None:
        return {
            "ok": False,
            "reason": (
                f"openvr.init failed: {type(init_exc).__name__}: {init_exc}.  "
                "SteamVR is probably not running; launch SteamVR first."
            ),
        }
    _say(f"[diagnose_pico_connect] openvr.init OK ({time.time()-t0:.1f}s).")
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
        controller_rows = [
            d for d in devices if d["class"] == int(openvr.TrackedDeviceClass_Controller)
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
        # For the grip track, the controllers are essential — they drive
        # the gripper open/close action via /input/grip/value.
        return {
            "ok": bool(hmd_rows) and bool(controller_rows),
            "hmd": hmd_rows,
            "trackers": tracker_rows,
            "pico_trackers": pico_trackers,
            "controllers": controller_rows,
            "reason": (
                ""
                if hmd_rows and controller_rows
                else (
                    "Required devices missing for grip track.  Need >=1 HMD "
                    "and >=1 PICO Touch controller visible to OpenVR.  "
                    "Verify PICO Connect Devices panel shows the headset + "
                    "both controllers paired and 'streaming' before retrying."
                )
            ),
        }
    finally:
        try:
            openvr.shutdown()
        except Exception:
            pass


def probe_binding_file(binding_path: Path) -> Dict[str, Any]:
    """Layer 6: does tracker_binding_pico_connect.json look filled in?

    For the grip track the only STRICTLY required roles are
    ``left_forearm`` and ``right_forearm``.  ``waist`` is optional (used
    only by the WaistEnabled env variant).  Leg roles are not consumed.
    """
    if not binding_path.exists():
        return {
            "ok": False,
            "reason": (
                f"{binding_path} not found.  Run "
                f"`python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers "
                f"--out {binding_path}` while PICO Connect is streaming "
                f"to generate the template."
            ),
        }
    try:
        data = json.loads(binding_path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "reason": f"Failed to parse {binding_path}: {exc}"}
    trackers = data.get("trackers", {})

    # Aggregate filled/required-role coverage.
    todo_keys = [
        sn
        for sn, info in trackers.items()
        if str(info.get("role", "")).lower().startswith("todo")
        or sn.lower().startswith("pmt_replace_me")
    ]
    roles_present = {
        str(info.get("role", "")).lower()
        for info in trackers.values()
        if str(info.get("role", "")).strip()
    }
    missing_required = [r for r in GRIP_ROLES_REQUIRED if r not in roles_present]

    ok = bool(trackers) and not todo_keys and not missing_required
    if missing_required:
        reason = (
            f"tracker_binding_pico_connect.json is missing required roles for "
            f"the grip track: {missing_required}.  At minimum bind one PICO "
            f"Motion Tracker to 'left_forearm' and another to 'right_forearm' "
            f"(these provide the wrist-EEF fallback when the controller pose "
            f"is briefly unavailable)."
        )
    elif todo_keys:
        reason = (
            f"tracker_binding_pico_connect.json still has placeholder/TODO "
            f"entries: {todo_keys}.  Edit the file and assign each PICO "
            f"Motion Tracker serial to one of: left_forearm, right_forearm, "
            f"waist (matching where you physically wear the tracker).  Leg "
            f"slots may stay at role=\"\" -- the grip retargeter ignores them."
        )
    elif not trackers:
        reason = (
            "tracker_binding_pico_connect.json has an empty 'trackers' map.  "
            "Run enumerate_trackers.py to populate it."
        )
    else:
        reason = ""
    return {
        "ok": ok,
        "trackers": list(trackers.keys()),
        "todo_keys": todo_keys,
        "roles_present": sorted(roles_present),
        "missing_required": missing_required,
        "reason": reason,
    }


# ── Reporter ─────────────────────────────────────────────────────────────


def render_report(report: Dict[str, Any]) -> str:
    out: List[str] = []
    layers = (
        ("1. PICO Connect Streaming Service", report["process"]),
        ("2. SteamVR drivers (prism / vd / udcap)", report["drivers"]),
        ("3-5. OpenVR devices (HMD + PICO trackers + controllers)", report["openvr"]),
        ("6. tracker_binding_pico_connect.json", report["binding"]),
    )
    out.append("=" * 72)
    out.append(" PICO Connect -> SteamVR -> PC -> Isaac Lab pipeline diagnosis")
    out.append(" (ust_hm_grip / 16-D gripper track)")
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
        out.append("\n  Detected SteamVR drivers:")
        for d in drv["drivers_found"]:
            out.append(f"    - {d}")
        if drv.get("virtual_desktop") and drv.get("prism"):
            out.append(
                "\n  WARNING: prism AND oculus_virtualdesktop are both registered.\n"
                "    For the 9.37 PICO Connect pipeline, set 'Virtual Desktop\n"
                "    Streamer (Quest)' OFF in SteamVR Add-Ons to avoid duplicate\n"
                "    HMD/controller injection (CLAUDE.md gotcha #29)."
            )
        if drv.get("udcap"):
            out.append(
                "\n  NOTE: udcap (UDCAP gloves) driver is registered.  Not\n"
                "    required for the grip track; safe to leave ON only if you\n"
                "    also use the glove track from the same SteamVR install."
            )
    ovr = report["openvr"]
    if ovr.get("hmd"):
        out.append("\n  HMD:")
        for h in ovr["hmd"]:
            out.append(
                f"    serial={h['serial']!r} manufacturer={h['manufacturer']!r} "
                f"model={h['model']!r}"
            )
    if ovr.get("controllers"):
        out.append(f"\n  Controllers ({len(ovr['controllers'])}):")
        for c in ovr["controllers"]:
            out.append(
                f"    serial={c['serial']!r} model={c['model']!r}"
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
            f"placeholders remaining: {len(bnd.get('todo_keys', []))}.  "
            f"Roles present: {bnd.get('roles_present', [])}."
        )
        if bnd.get("missing_required"):
            out.append(
                f"  Missing required roles: {bnd['missing_required']}"
            )
    out.append("")
    out.append("=" * 72)
    out.append(
        f" Overall: {'PIPELINE READY' if overall_ok else 'NOT READY -- see [FAIL] lines above'}"
    )
    out.append("=" * 72)
    return "\n".join(out)


def main() -> int:
    _say("[diagnose_pico_connect] starting...")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binding",
        type=Path,
        default=DEFAULT_BINDING_JSON,
        help="Path to tracker_binding_pico_connect.json (default: %(default)s).",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of a report.")
    parser.add_argument(
        "--init-timeout",
        type=float,
        default=60.0,
        help=(
            "Max seconds to wait for openvr.init() inside layer 3-5 (default "
            "60s).  Bump to 120 if SteamVR cold-starts slowly on your rig."
        ),
    )
    args = parser.parse_args()

    _say("[diagnose_pico_connect] probe layer 1: PICO Connect Streaming Service...")
    process = probe_pico_connect_process()
    _say("[diagnose_pico_connect] probe layer 2: SteamVR driver registry (vrpathreg)...")
    drivers = probe_steamvr_drivers()
    openvr_layer = probe_openvr_devices(init_timeout_s=float(args.init_timeout))
    _say("[diagnose_pico_connect] probe layer 6: tracker_binding_pico_connect.json...")
    binding = probe_binding_file(args.binding.resolve())

    report: Dict[str, Any] = {
        "process": process,
        "drivers": drivers,
        "openvr": openvr_layer,
        "binding": binding,
    }
    report["overall_ok"] = all(layer.get("ok") for layer in report.values() if isinstance(layer, dict))

    if args.json:
        print(json.dumps(report, indent=2, default=str))
    else:
        print(render_report(report))
    return 0 if report["overall_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
