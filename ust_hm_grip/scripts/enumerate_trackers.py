"""Enumerate SteamVR tracked devices and dump their serial numbers (grip track).

Use once per PICO Connect / Virtual Desktop session: with the PICO 4 Ultra
headset streaming and the trackers active, run this script to list every
GenericTracker serial that SteamVR reports and to get a starting point for
``config/tracker_binding_pico_connect.json``.

Output:
    console table + JSON dump.  Pass ``--out <path>`` to write the JSON
    template directly; the user still has to assign the role fields for
    PICO trackers (waist / left_forearm / right_forearm — leg slots are
    unused by the grip retargeter and may stay with role="").

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers \\
        --out ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json

This is the grip-track counterpart of
``ust_hm_glove/scripts/enumerate_trackers.py``.  The auto-mapping logic is
shared verbatim; only the docstring, default output path, and grip-track-
specific role policy differ.

9.38 hardening (no-output diagnosis):
    * stdout is forced to line-buffered so progress messages flush
      immediately even when running under a redirected pipe.
    * Every long-running step is announced BEFORE it starts so the user
      sees activity instead of an apparently-frozen prompt.
    * The script pre-checks ``vrserver.exe`` (SteamVR core) via tasklist
      and aborts with an actionable message when SteamVR is not running
      -- previously the script would silently block inside ``openvr.init``
      while OpenVR auto-started SteamVR (which can take 30+ seconds and
      sometimes hangs forever if SteamVR is in a bad state).
    * ``openvr.init`` is wrapped in a watchdog thread with a configurable
      timeout (``--init-timeout``, default 60s).  When the timeout fires
      the script exits with a clear "init hung" message instead of
      blocking the user's terminal indefinitely.
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
from typing import Any, Dict, Optional, Tuple


# ── stdout hardening ────────────────────────────────────────────────────
# Force line buffering so each _say() shows up on screen as it happens,
# even when the script is launched under a redirected stdout (CI, log
# capture, pytest -s, etc.).  Best-effort -- only available on Python 3.7+
# and only when stdout is a real text stream.
try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _say(msg: str) -> None:
    """Print a progress line with explicit flush so the user sees every
    step even if the underlying stdout would otherwise block-buffer."""
    print(msg, flush=True)


# ── SteamVR pre-check ───────────────────────────────────────────────────
def _steamvr_running() -> Optional[bool]:
    """Return True/False if we can determine, None on uncertainty.

    Detected by listing the ``vrserver.exe`` process.  Avoids the long
    hang inside ``openvr.init`` when SteamVR isn't even up yet.  On
    non-Windows we return None (no decision).
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
    out = (res.stdout or "").lower()
    return "vrserver.exe" in out


# ── openvr.init watchdog ────────────────────────────────────────────────
def _init_openvr_with_timeout(openvr_module, timeout_s: float) -> Tuple[Any, Optional[BaseException]]:
    """Call ``openvr.init`` on a daemon thread and join with timeout.

    Returns ``(system, None)`` on success, ``(None, exc)`` on failure
    where ``exc`` is the originating exception, or ``(None, TimeoutError)``
    when the call did not return within ``timeout_s`` seconds.
    """
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


def main() -> int:
    _say("[enumerate_trackers] starting...")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Write tracker_binding_pico_connect.json template to this path.",
    )
    parser.add_argument(
        "--init-timeout",
        type=float,
        default=60.0,
        help=(
            "Max seconds to wait for openvr.init() before aborting (default "
            "60s).  Bump to 120 if SteamVR cold-starts slowly on your rig."
        ),
    )
    parser.add_argument(
        "--skip-steamvr-check",
        action="store_true",
        help=(
            "Skip the vrserver.exe pre-check and call openvr.init() "
            "directly.  Use only if the pre-check misreports SteamVR as "
            "not running."
        ),
    )
    args = parser.parse_args()

    # ── Pre-check: is SteamVR even up? ─────────────────────────────────
    if not args.skip_steamvr_check:
        _say("[enumerate_trackers] checking whether vrserver.exe (SteamVR core) is running...")
        running = _steamvr_running()
        if running is False:
            _say("")
            _say("[enumerate_trackers] FAIL: SteamVR (vrserver.exe) is NOT running.")
            _say("")
            _say("  Without SteamVR, openvr.init() would silently block for "
                 "30+ seconds (and may hang forever) while OpenVR tries to "
                 "auto-start SteamVR.  Aborting early.")
            _say("")
            _say("  Fix:")
            _say("    1) Open Steam (the desktop app).")
            _say("    2) Library -> Tools -> SteamVR -> Launch.  Wait until")
            _say("       the SteamVR status window says 'Ready' and the")
            _say("       headset icon turns green/blue.")
            _say("    3) Make sure PICO Connect is running and the headset")
            _say("       is paired (for the PICO Connect -> SteamVR pipeline,")
            _say("       SteamVR > Manage Add-Ons must show prism = ON).")
            _say("    4) Re-run this command.")
            _say("")
            _say("  If SteamVR really IS running but this check is wrong,")
            _say("  re-run with `--skip-steamvr-check` to bypass the probe.")
            return 2
        if running is True:
            _say("[enumerate_trackers] vrserver.exe found.  OK.")
        else:
            _say("[enumerate_trackers] could not determine SteamVR state; proceeding anyway.")

    # ── Import openvr lazily so we get progress before any potential
    #    DLL-load delay or import error.
    _say("[enumerate_trackers] importing openvr (pyopenvr)...")
    try:
        import openvr  # noqa: F401
    except ImportError as exc:
        _say(f"[enumerate_trackers] FAIL: pyopenvr is not installed ({exc}).")
        _say("  Install it inside the same Python env:  pip install openvr")
        return 1
    except Exception as exc:  # noqa: BLE001
        _say(f"[enumerate_trackers] FAIL: openvr import error ({type(exc).__name__}: {exc}).")
        _say("  This usually means the OpenVR DLL could not be loaded.  Check that")
        _say("  SteamVR is installed in its standard location and that the")
        _say("  conda env has not blocked the openvr_api.dll path.")
        return 1
    _say("[enumerate_trackers] openvr imported OK.")

    DEVICE_CLASS_NAMES = {
        openvr.TrackedDeviceClass_Invalid: "Invalid",
        openvr.TrackedDeviceClass_HMD: "HMD",
        openvr.TrackedDeviceClass_Controller: "Controller",
        openvr.TrackedDeviceClass_GenericTracker: "GenericTracker",
        openvr.TrackedDeviceClass_TrackingReference: "TrackingReference",
        openvr.TrackedDeviceClass_DisplayRedirect: "DisplayRedirect",
    }

    def get_string_prop(system, idx: int, prop) -> str:
        try:
            return system.getStringTrackedDeviceProperty(idx, prop)
        except Exception:  # noqa: BLE001
            return ""

    # ── openvr.init with watchdog ──────────────────────────────────────
    _say(
        f"[enumerate_trackers] calling openvr.init(VRApplication_Other) "
        f"(timeout={args.init_timeout:.0f}s)..."
    )
    t0 = time.time()
    system, init_exc = _init_openvr_with_timeout(openvr, args.init_timeout)
    if isinstance(init_exc, TimeoutError):
        _say(f"[enumerate_trackers] FAIL: {init_exc}")
        _say("  Possible causes (in priority order):")
        _say("    1) SteamVR is in a hung state -- close it from the system tray")
        _say("       and from Task Manager (vrserver.exe + vrcompositor.exe +")
        _say("       vrmonitor.exe), then relaunch via Steam.")
        _say("    2) Two HMD-redirecting drivers fighting (e.g. prism + VD).")
        _say("       SteamVR > Manage Add-Ons: keep ONLY ONE of prism / Virtual")
        _say("       Desktop Streamer enabled (CLAUDE.md gotcha #29).")
        _say("    3) SteamVR is starting fresh and needs more than --init-timeout")
        _say("       seconds.  Re-run with `--init-timeout 120`.")
        return 3
    if init_exc is not None:
        _say(f"[enumerate_trackers] FAIL: openvr.init raised {type(init_exc).__name__}: {init_exc}")
        _say("  Is SteamVR running and PICO Connect / Virtual Desktop streaming?")
        return 1
    elapsed = time.time() - t0
    _say(f"[enumerate_trackers] openvr.init OK ({elapsed:.1f}s).")

    try:
        _say("[enumerate_trackers] querying tracked devices...")
        poses = system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        rows = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            cls = system.getTrackedDeviceClass(i)
            if cls == openvr.TrackedDeviceClass_Invalid:
                continue
            cls_name = DEVICE_CLASS_NAMES.get(cls, str(cls))
            serial = get_string_prop(system, i, openvr.Prop_SerialNumber_String)
            model = get_string_prop(system, i, openvr.Prop_ModelNumber_String)
            manufacturer = get_string_prop(system, i, openvr.Prop_ManufacturerName_String)
            valid = bool(poses[i].bPoseIsValid) if i < len(poses) else False
            rows.append(
                {
                    "idx": i,
                    "class": cls_name,
                    "serial": serial,
                    "manufacturer": manufacturer,
                    "model": model,
                    "pose_valid": valid,
                }
            )

        _say(
            f"{'idx':>3}  {'class':<18}  {'serial':<28}  {'manuf':<16}  "
            f"{'model':<24}  {'valid':<5}"
        )
        _say("-" * 96)
        for r in rows:
            _say(
                f"{r['idx']:>3}  {r['class']:<18}  {r['serial']:<28}  "
                f"{r['manufacturer']:<16}  {r['model']:<24}  {str(r['pose_valid']):<5}"
            )

        trackers = {r["serial"]: r for r in rows if r["class"] == "GenericTracker"}
        hmd_rows = [r for r in rows if r["class"] == "HMD"]
        _say(f"\nFound {len(trackers)} generic trackers.")

        # ── VD body-segment -> internal role auto-map ─────────────────────
        vd_segment_to_role = {
            "hips":                 ("waist",         "TrackerRole_Waist"),
            "left_arm_lower":       ("left_forearm",  "TrackerRole_LeftElbow"),
            "right_arm_lower":      ("right_forearm", "TrackerRole_RightElbow"),
            "left_lower_leg":       ("",              "ai_inferred"),
            "right_lower_leg":      ("",              "ai_inferred"),
            "chest":                ("",              "ai_inferred"),
            "left_arm_upper":       ("",              "ai_inferred"),
            "right_arm_upper":      ("",              "ai_inferred"),
            "left_foot_transverse": ("",              "ai_inferred"),
            "right_foot_transverse":("",              "ai_inferred"),
        }

        # ── PICO Connect (prism) tracker auto-map ─────────────────────────
        pico_serial_prefixes = ("pmt_", "picobt_", "pico_motion_tracker_")
        pico_manufacturers = ("pico", "pico immersive pte. ltd.")
        pico_models = ("pico motion tracker", "pmt", "pico body tracker")
        pico_tracker_count = 0
        pico_serials = []
        for r in rows:
            if r["class"] != "GenericTracker":
                continue
            sn_low = r["serial"].lower()
            mn_low = r["manufacturer"].lower()
            md_low = r["model"].lower()
            if (
                any(sn_low.startswith(pfx) for pfx in pico_serial_prefixes)
                or mn_low in pico_manufacturers
                or any(p in md_low for p in pico_models)
            ):
                pico_tracker_count += 1
                pico_serials.append(r["serial"])

        vd_segment_count = sum(
            1 for r in rows
            if r["class"] == "GenericTracker" and r["serial"] in vd_segment_to_role
        )

        # ── Diagnostic warnings ────────────────────────────────────────────
        warnings = []
        if not hmd_rows:
            warnings.append(
                "No HMD detected.  Is SteamVR running with an actively streaming "
                "headset (PICO via PICO Connect, or Quest via VD / Meta Link)?"
            )

        if len(trackers) == 0:
            warnings.append(
                "0 GenericTrackers visible.  For the grip track only the LEFT/RIGHT\n"
                "forearm trackers are STRICTLY required (waist optional).  Pipelines:\n"
                "  A) PICO Connect (prism driver) -- 9.37 default for full-body teleop:\n"
                "     1) PICO Connect Streaming Service running on PC, headset paired.\n"
                "     2) PICO Motion Trackers powered, paired, charged.\n"
                "     3) SteamVR > Manage Add-Ons:\n"
                "          prism                            ON   (PICO Connect)\n"
                "          Virtual Desktop Streamer (Quest) OFF  (avoid driver conflict)\n"
                "          udcap                            OFF  (gloves not used in grip)\n"
                "     4) Re-launch SteamVR after toggling add-ons.\n"
                "  B) Virtual Desktop (legacy 9.36 setup):\n"
                "     1) Pico OS >= 5.14 AND tracker mode = 'Enhanced Forearm' (5 trackers).\n"
                "     2) Every tracker is powered on and paired to the headset.\n"
                "     3) Virtual Desktop Streamer (Windows) has:\n"
                "          OPTIONS -> 'Forward tracking to SteamVR' ON\n"
                "          OPTIONS -> 'Full body tracking'           ON\n"
                "     4) SteamVR add-ons: VD Streamer (Quest) ON, prism OFF.\n"
                "     5) Restart VD Streamer + reconnect the Pico's VD client."
            )

        if pico_tracker_count > 0 and vd_segment_count > 0:
            warnings.append(
                "MIXED runtime detected: prism (PICO Connect) and Virtual\n"
                "Desktop body-segment trackers are BOTH visible in SteamVR.\n"
                "  PICO trackers: {}\n"
                "  VD segments:   {}\n"
                "Pick one pipeline:\n"
                "  * 9.37 default (PICO Connect): SteamVR > Manage Add-Ons,\n"
                "    set 'Virtual Desktop Streamer (Quest)' OFF and prism ON,\n"
                "    then re-run with --vr_runtime pico_connect.\n"
                "  * Legacy (Virtual Desktop): set prism OFF and VD Streamer ON,\n"
                "    then re-run with --vr_runtime virtual_desktop.".format(
                    pico_serials,
                    [
                        r["serial"] for r in rows
                        if r["class"] == "GenericTracker"
                        and r["serial"] in vd_segment_to_role
                    ],
                )
            )

        if warnings:
            _say("\n" + "=" * 72)
            _say("DIAGNOSTICS")
            _say("=" * 72)
            for i, w in enumerate(warnings, 1):
                _say(f"\n[{i}] {w}")
            _say("")

        tracker_template: Dict[str, Any] = {}
        auto_mapped = 0
        pico_tagged = 0
        for sn in trackers:
            sn_low = sn.lower()
            tracker_row = trackers[sn]
            mn_low = tracker_row.get("manufacturer", "").lower()
            md_low = tracker_row.get("model", "").lower()
            if sn in vd_segment_to_role:
                role, svr_role = vd_segment_to_role[sn]
                tracker_template[sn] = {"role": role, "steamvr_role": svr_role}
                if role:
                    auto_mapped += 1
            elif (
                any(sn_low.startswith(pfx) for pfx in pico_serial_prefixes)
                or mn_low in pico_manufacturers
                or any(p in md_low for p in pico_models)
            ):
                tracker_template[sn] = {
                    "role": "TODO_pico",
                    "steamvr_role": "TODO_pico",
                }
                pico_tagged += 1
            else:
                tracker_template[sn] = {"role": "TODO", "steamvr_role": "TODO"}

        if pico_tagged > 0 and pico_tagged >= len(vd_segment_to_role):
            file_comment = (
                "Auto-generated by enumerate_trackers.py (grip track).  PICO Connect "
                "(prism driver) detected.  Replace each 'TODO_pico' with the role you "
                "PHYSICALLY WEAR that tracker as.  For the grip track only "
                "left_forearm / right_forearm are strictly required (waist optional, "
                "legs unused).  Set unused entries to role=\"\" so the SteamVRSampler "
                "skips them."
            )
        else:
            file_comment = (
                "Auto-generated (grip track).  Virtual Desktop Full Body segments were "
                "auto-mapped.  For LHR-* physical trackers, replace each 'TODO' with "
                "one of: waist, left_forearm, right_forearm.  Leg slots are unused by "
                "the grip retargeter -- leave role=\"\" or remove them entirely.  PICO "
                "Connect Motion Trackers carry the 'TODO_pico' tag instead (see "
                "scripts/diagnose_pico_connect.py)."
            )

        template: Dict[str, Any] = {
            "_comment": file_comment,
            "hmd": "auto",
            "trackers": tracker_template,
        }
        if auto_mapped:
            _say(
                f"\nAuto-mapped {auto_mapped} VD body segment(s) to grip-track roles "
                "(waist / *_forearm).  Remaining segments were left as role=\"\"."
            )
        if pico_tagged:
            _say(
                f"\nDetected {pico_tagged} PICO Motion Tracker(s) via PICO Connect."
                "\n  Roles tagged TODO_pico -- edit the JSON to mark each tracker"
                "\n  as waist / left_forearm / right_forearm according to where you"
                "\n  physically wear it.  Leg slots are unused in the grip track."
            )

        if args.out is not None:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(template, indent=2), encoding="utf-8")
            _say(f"Wrote template to {args.out}")
        else:
            _say("\nJSON template:\n")
            _say(json.dumps(template, indent=2))
    finally:
        try:
            openvr.shutdown()
        except Exception:  # noqa: BLE001
            pass

    _say("[enumerate_trackers] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
