"""Enumerate SteamVR tracked devices and dump their serial numbers.

Use once per Virtual Desktop session: with the Pico 4 Ultra headset
streaming (Enhanced Forearm mode, 5 trackers active), run this script to
list every LHR-* serial that SteamVR reports and to get a starting point
for ``config/tracker_binding.json``.

Output:
    console table + JSON dump.  Pass ``--out <path>`` to write the
    JSON template directly; the user still has to assign the five
    ``role`` fields.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

try:
    import openvr
except ImportError as exc:  # pragma: no cover — helpful message on bare env
    print("[enumerate_trackers] pyopenvr is not installed: pip install openvr")
    raise SystemExit(1) from exc


DEVICE_CLASS_NAMES = {
    openvr.TrackedDeviceClass_Invalid: "Invalid",
    openvr.TrackedDeviceClass_HMD: "HMD",
    openvr.TrackedDeviceClass_Controller: "Controller",
    openvr.TrackedDeviceClass_GenericTracker: "GenericTracker",
    openvr.TrackedDeviceClass_TrackingReference: "TrackingReference",
    openvr.TrackedDeviceClass_DisplayRedirect: "DisplayRedirect",
}


# 9.41 -- map the OpenVR init error codes that show up most often when
# the user's pipeline is partially configured.  Reference:
# https://github.com/ValveSoftware/openvr/blob/master/headers/openvr.h
# (search for ``EVRInitError``).  We only enumerate the codes that have
# specific actionable remediation steps; everything else falls through
# to the generic "see diagnose_pico_connect" message.
_OPENVR_INIT_ERROR_HINTS = {
    "VRInitError_Init_HmdNotFound": (
        "SteamVR is running but it cannot find an HMD provider.  Most "
        "common causes:\n"
        "  - PICO Connect is installed but not actively streaming the "
        "HMD (only the WiFi handshake completed).  Put the headset on, "
        "go to PICO Connect's main UI, and ensure 'Stream PCVR' / 'Start "
        "Streaming' is engaged.\n"
        "  - PICO 4 Ultra is in standby (yellow icon in SteamVR status "
        "strip).  Hold your hand over the proximity sensor inside the "
        "headset and wait until the icon turns GREEN.\n"
        "  - vrpathreg has the pico driver registered but the driver "
        "load failed (e.g. PICO Connect re-install pending).  Reinstall "
        "PICO Connect or re-add the driver:\n"
        "       vrpathreg adddriver \"C:\\Program Files\\PICO Connect\\openvr_driver\""
    ),
    "VRInitError_Init_HmdNotFoundPresenceFailed": (
        "Same as HmdNotFound above -- SteamVR's presence check timed out."
    ),
    "VRInitError_Init_PathRegistryNotFound": (
        "openvrpaths.vrpath registry file not found.  Reinstall SteamVR."
    ),
    "VRInitError_IPC_ServerInitFailed": (
        "vrserver IPC handshake failed.  Most common cause: HMD provider "
        "conflict (multiple HMD drivers registered -- gotcha #29).  Run:\n"
        "    python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect\n"
        "and follow its Layer 2 [FAIL] removal commands."
    ),
    "VRInitError_IPC_ConnectFailed": (
        "Could not connect to vrserver IPC.  SteamVR may be in a stale "
        "state.  Stop the process group and restart:\n"
        "    Get-Process vrserver, vrmonitor, vrwebhelper, vrcompositor "
        "-ErrorAction SilentlyContinue | Stop-Process -Force\n"
        "Then relaunch SteamVR via PICO Connect."
    ),
    "VRInitError_Init_VRDashboardNotFound": (
        "VR dashboard initialisation failed.  Restart SteamVR."
    ),
}


def _print_remediation_checklist() -> None:
    """Print a one-shot diagnose-then-decide checklist."""
    print(
        "\n[enumerate_trackers] Recommended next step:\n"
        "  python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect\n"
        "It runs a 6-layer probe and prints exact remediation commands "
        "for whichever layer fails."
    )


def _report_openvr_init_error(exc) -> None:
    """Print a verbose, actionable report for an openvr.OpenVRError."""
    # OpenVRError sometimes has an empty str() representation -- ALWAYS
    # include the type name and the .args tuple so the user has SOMETHING
    # to grep for / paste into a bug report.
    err_type = type(exc).__name__
    err_repr = repr(exc) if str(exc) == "" else str(exc)
    print(f"[enumerate_trackers] openvr.init failed: {err_type}: {err_repr}")
    if exc.args:
        print(f"  args: {exc.args!r}")
    # Many OpenVR error subclasses encode the EVRInitError name in their
    # class name (e.g. ``ApplicationError_AppKeyAlreadyExists``).  Look up
    # the class name in our hint table and print the matching guidance.
    hint = _OPENVR_INIT_ERROR_HINTS.get(err_type)
    if hint is not None:
        print(f"\n[enumerate_trackers] Likely cause + fix:\n  {hint}")
    else:
        print(
            f"\n[enumerate_trackers] Error class {err_type!r} has no specific "
            "remediation entry yet.  Falling back to the layered diagnose."
        )
    _print_remediation_checklist()


def get_string_prop(system, idx: int, prop) -> str:
    try:
        return system.getStringTrackedDeviceProperty(idx, prop)
    except Exception:  # noqa: BLE001
        return ""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Write tracker_binding.json template to this path.",
    )
    args = parser.parse_args()

    # 9.41 -- use the SteamVRSampler watchdog (9.38) so a missing SteamVR
    # / unstreamed HMD surfaces as a TimeoutError after 15s instead of
    # blocking forever.  Falls back to direct openvr.init when the package
    # path is unavailable (e.g. running this script outside the repo).
    try:
        from ust_ws.ust_hm_glove.teleop.vr_sampler import (
            _init_openvr_with_timeout,
        )
        system = _init_openvr_with_timeout(timeout_sec=15.0)
    except TimeoutError as exc:
        print(f"[enumerate_trackers] openvr.init TIMEOUT (>15s):\n{exc}")
        print(
            "\n[enumerate_trackers] Run the layered diagnostic FIRST:\n"
            "  python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect\n"
            "Most common cause is multiple HMD drivers registered (gotcha #29) -- "
            "the diagnose script will print a ready-to-paste vrpathreg removedriver "
            "command."
        )
        return 1
    except ImportError:
        # Stand-alone fallback when ust_ws.ust_hm_glove can't be imported.
        try:
            system = openvr.init(openvr.VRApplication_Other)
        except openvr.OpenVRError as exc:
            _report_openvr_init_error(exc)
            return 1
    except openvr.OpenVRError as exc:
        _report_openvr_init_error(exc)
        return 1
    except Exception as exc:  # noqa: BLE001 -- visibility above all
        print(f"[enumerate_trackers] openvr.init raised {type(exc).__name__}: {exc!r}")
        _print_remediation_checklist()
        return 1

    try:
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

        print(
            f"{'idx':>3}  {'class':<18}  {'serial':<28}  {'manuf':<16}  {'model':<24}  {'valid':<5}"
        )
        print("-" * 96)
        for r in rows:
            print(
                f"{r['idx']:>3}  {r['class']:<18}  {r['serial']:<28}  "
                f"{r['manufacturer']:<16}  {r['model']:<24}  {str(r['pose_valid']):<5}"
            )

        trackers = {r["serial"]: r for r in rows if r["class"] == "GenericTracker"}
        hmd_rows = [r for r in rows if r["class"] == "HMD"]
        print(f"\nFound {len(trackers)} generic trackers.")

        # ── VD body-segment -> internal role auto-map ─────────────────────
        # Virtual Desktop's "Full body tracking" forwarder emits 10 VRChat-
        # style body segments as Vive Trackers.  Map the five physical Pico
        # trackers to our internal role names and explicitly mark the other
        # five as role="" so SteamVRSampler skips them.
        vd_segment_to_role = {
            "hips":                 ("waist",         "TrackerRole_Waist"),
            "left_arm_lower":       ("left_forearm",  "TrackerRole_LeftElbow"),
            "right_arm_lower":      ("right_forearm", "TrackerRole_RightElbow"),
            "left_lower_leg":       ("left_ankle",    "TrackerRole_LeftFoot"),
            "right_lower_leg":      ("right_ankle",   "TrackerRole_RightFoot"),
            "chest":                ("",              "ai_inferred"),
            "left_arm_upper":       ("",              "ai_inferred"),
            "right_arm_upper":      ("",              "ai_inferred"),
            "left_foot_transverse": ("",              "ai_inferred"),
            "right_foot_transverse":("",              "ai_inferred"),
        }

        # ── PICO Connect (prism) tracker auto-map ─────────────────────────
        # PICO Motion Trackers stream in via the prism driver.  Unlike VD,
        # PICO does NOT pre-assign body-part roles to each tracker -- the
        # user must inspect physical mounting.  We therefore leave roles as
        # 'TODO' for serials matching the PICO conventions so the human
        # operator fills them in.
        pico_serial_prefixes = ("pmt_", "picobt_", "pico_motion_tracker_")
        pico_manufacturers = ("pico", "pico immersive pte. ltd.")
        pico_models = ("pico motion tracker", "pmt", "pico body tracker")
        # Detect whether prism (PICO Connect) is the active tracker source
        # by counting how many trackers identify as PICO devices.  Mixed
        # setups (VD + PICO) trigger a separate warning below.
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

        # Detect whether VD body-segment forwarder is active (any tracker
        # serial matching the VRChat naming).
        vd_segment_count = sum(
            1 for r in rows
            if r["class"] == "GenericTracker" and r["serial"] in vd_segment_to_role
        )

        # ── Diagnostic warnings ────────────────────────────────────────────
        #
        # NOTE on "Meta Quest 3" HMD identity for Pico users:
        #   Virtual Desktop streams Pico 4 Ultra through its `oculus_virtualdesktop`
        #   SteamVR driver, which reuses the Oculus/OVR protocol surface.  As a
        #   result SteamVR reports the HMD as "Oculus Meta Quest 3" EVEN WHEN the
        #   physical headset is a Pico.  This is normal VD behaviour, not a bug.
        #   Do not chase the HMD name; only the tracker count matters here.
        warnings = []
        if not hmd_rows:
            warnings.append(
                "No HMD detected.  Is SteamVR running with an actively streaming "
                "headset (Pico via Virtual Desktop, or Quest via VD / Meta Link)?"
            )

        if len(trackers) == 0:
            warnings.append(
                "0 GenericTrackers visible.  Two supported pipelines:\n"
                "  A) PICO Connect (prism driver) -- 9.37 default for full-body teleop:\n"
                "     1) PICO Connect Streaming Service running on PC, headset paired.\n"
                "     2) PICO Motion Trackers powered, paired, charged.\n"
                "     3) SteamVR > Manage Add-Ons:\n"
                "          prism                            ON   (PICO Connect)\n"
                "          Virtual Desktop Streamer (Quest) OFF  (avoid driver conflict)\n"
                "          udcap                            ON   (only if using gloves)\n"
                "     4) Re-launch SteamVR after toggling add-ons.\n"
                "  B) Virtual Desktop (legacy 9.36 setup):\n"
                "     1) Pico OS >= 5.14 AND tracker mode = 'Enhanced Forearm' (5 trackers).\n"
                "     2) Every tracker is powered on and paired to the headset.\n"
                "     3) Virtual Desktop Streamer (Windows) has:\n"
                "          OPTIONS -> 'Forward tracking to SteamVR' ON\n"
                "          OPTIONS -> 'Full body tracking'           ON\n"
                "     4) SteamVR add-ons: VD Streamer (Quest) ON, prism OFF, udcap ON.\n"
                "     5) Restart VD Streamer + reconnect the Pico's VD client."
            )
        elif len(trackers) < 5:
            warnings.append(
                f"Only {len(trackers)} tracker(s) visible.  Enhanced Forearm /"
                " full-body modes typically expose 5.  Check that every tracker"
                " is powered, paired, and visible in SteamVR's 'Manage Vive"
                " Trackers' UI (or PICO Connect's Devices panel for the prism"
                " pipeline)."
            )

        # 9.37 -- detect mixed PICO Connect + Virtual Desktop setup.  Both
        # drivers can be active simultaneously and inject overlapping
        # tracker sets; this is a common cause of double-tracker artefacts
        # and the wrist EEF jumping between sources frame-to-frame.
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
            print("\n" + "=" * 72)
            print("DIAGNOSTICS")
            print("=" * 72)
            for i, w in enumerate(warnings, 1):
                print(f"\n[{i}] {w}")
            print()

        tracker_template: Dict[str, Any] = {}
        auto_mapped = 0
        pico_tagged = 0
        for sn in trackers:
            sn_low = sn.lower()
            tracker_row = trackers[sn]
            mn_low = tracker_row.get("manufacturer", "").lower()
            md_low = tracker_row.get("model", "").lower()
            if sn in vd_segment_to_role:
                # Virtual Desktop body segment: fully auto-mapped.
                role, svr_role = vd_segment_to_role[sn]
                tracker_template[sn] = {"role": role, "steamvr_role": svr_role}
                if role:
                    auto_mapped += 1
            elif (
                any(sn_low.startswith(pfx) for pfx in pico_serial_prefixes)
                or mn_low in pico_manufacturers
                or any(p in md_low for p in pico_models)
            ):
                # PICO Motion Tracker via PICO Connect (prism driver).
                # Roles cannot be inferred automatically -- the same SKU is
                # worn on waist / forearm / ankle depending on the user's
                # mounting choice.  Emit a TODO and a comment hint.
                tracker_template[sn] = {
                    "role": "TODO_pico",
                    "steamvr_role": "TODO_pico",
                }
                pico_tagged += 1
            else:
                # Physical LHR-* Vive Trackers (e.g. Tundra TrackStrap):
                # user must fill the role.
                tracker_template[sn] = {"role": "TODO", "steamvr_role": "TODO"}

        # Pick a comment that matches the dominant tracker source so the
        # written file's docstring is contextual.
        if pico_tagged > 0 and pico_tagged >= len(vd_segment_to_role):
            file_comment = (
                "Auto-generated by enumerate_trackers.py.  PICO Connect (prism "
                "driver) detected.  Replace each 'TODO_pico' with the role you "
                "PHYSICALLY WEAR that tracker as: waist / left_forearm / "
                "right_forearm / left_ankle / right_ankle.  steamvr_role may "
                "stay as TODO_pico unless you also want to broadcast a SteamVR "
                "role hint -- the SteamVRSampler keys off 'role' alone."
            )
        else:
            file_comment = (
                "Auto-generated.  Virtual Desktop Full Body segments were "
                "auto-mapped (hips/*_arm_lower/*_lower_leg).  For LHR-* "
                "physical trackers, replace each 'TODO' with one of: waist, "
                "left_ankle, right_ankle, left_forearm, right_forearm.  PICO "
                "Connect Motion Trackers carry the 'TODO_pico' tag instead "
                "(see scripts/diagnose_pico_connect.py)."
            )

        template: Dict[str, Any] = {
            "_comment": file_comment,
            "hmd": "auto",
            "trackers": tracker_template,
        }
        if auto_mapped:
            print(
                f"\nAuto-mapped {auto_mapped} VD body segment(s) to internal roles "
                "(waist / *_forearm / *_ankle).  Remaining AI-inferred segments "
                "(chest / upper_arm / foot_transverse) were disabled with role=\"\"."
            )
        if pico_tagged:
            print(
                f"\nDetected {pico_tagged} PICO Motion Tracker(s) via PICO Connect."
                "\n  Roles tagged TODO_pico -- edit the JSON to mark each tracker"
                "\n  as waist / left_forearm / right_forearm / left_ankle / right_ankle"
                "\n  according to where you physically wear it."
            )

        if args.out is not None:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(template, indent=2), encoding="utf-8")
            print(f"Wrote template to {args.out}")
        else:
            print("\nJSON template:\n")
            print(json.dumps(template, indent=2))
    finally:
        openvr.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
