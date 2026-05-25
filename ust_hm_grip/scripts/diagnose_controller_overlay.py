"""Action API diagnostic that takes input focus via VRApplication_Overlay.

THE problem this solves
-----------------------

``diagnose_controller_raw.py`` initialises OpenVR as
``VRApplication_Other``.  In OpenVR's app-type model ``Other`` is a
background daemon: SteamVR will NEVER route controller actions to a
background app, regardless of how perfectly the manifest is registered
or how the binding files are structured.

Concrete symptom (9.44 root-cause):
  * Every Action API channel returns bActive=False forever.
  * SteamVR's Dashboard / Desktop View Pointer remains the focus owner,
    so the trigger fires its dashboard binding (= LMB) and the grip
    fires the dashboard middle-click instead of our binding.
  * The user sees "trigger acts as mouse click, grip acts as wheel
    click" while pointing at the desktop -- the smoking gun.

Production teleop avoids this trap by booting Isaac Sim as a
``VRApplication_Scene`` app via ``--render_mode steamvr_native``;
that scene app becomes the focus owner and our manifest's bindings
activate.  Monitor-mode teleop hits the same trap as the diagnostic.

This script is a 30-second validator that bypasses the Isaac Sim
boot:

  1. ``openvr.init(VRApplication_Overlay)`` so we are eligible to
     own input focus (Overlay apps can; Other apps cannot).
  2. Force-unregister + re-register our manifest (9.43 pattern) so
     SteamVR re-reads disk-side actions.json + bindings_*.json.
  3. Create a small overlay, position it in front of the user, show
     it.  This is enough to make us the active focus owner.
  4. Probe Action API trigger / grip / menu for both controllers
     for ``--seconds`` seconds, with bActive flag.

If this script reports ``(a1)`` flags + nonzero values when squeezing
but ``diagnose_controller_raw`` does not, the binding pipeline is
correct -- the only thing missing in the production flow is
``--render_mode steamvr_native`` (or any other path that makes Isaac
Sim the scene app).

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_overlay
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _say(msg: str) -> None:
    print(msg, flush=True)


APP_KEY = "ust.teleop.gr1t2_gripper"
CFG_DIR = ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions"
ACTIONS_PATH = CFG_DIR / "actions.json"
STATIC_MANIFEST = CFG_DIR / "manifest.vrmanifest"
RUNTIME_MANIFEST = CFG_DIR / "manifest.runtime.vrmanifest"


def _generate_runtime_manifest() -> Path:
    with open(STATIC_MANIFEST, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    py_exe = os.path.abspath(sys.executable)
    for app in manifest.get("applications", []):
        app["binary_path_windows"] = py_exe
    with open(RUNTIME_MANIFEST, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    return RUNTIME_MANIFEST


def main() -> int:
    _say("[overlay_diag] starting...")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()

    try:
        import openvr  # type: ignore
    except ImportError:
        _say("FATAL -- pyopenvr not installed.  pip install openvr.")
        return 1

    # ---- Init as Overlay (KEY FIX) ----------------------------------
    _say("[overlay_diag] openvr.init(VRApplication_Overlay) -- so we can be"
         " an input focus owner.  Background-type apps (VRApplication_Other,"
         " used by diagnose_controller_raw) are NOT eligible to receive"
         " action data; that's why this script exists.")
    try:
        system = openvr.init(openvr.VRApplication_Overlay)
    except Exception as exc:
        _say(f"FATAL -- openvr.init(VRApplication_Overlay) failed: {exc}")
        return 1

    try:
        # ---- Manifest re-register (9.43 pattern) --------------------
        try:
            apps = openvr.VRApplications()
        except Exception as exc:
            _say(f"FATAL -- VRApplications interface unavailable: {exc}")
            return 1
        runtime = _generate_runtime_manifest()
        for path_to_clear in (str(STATIC_MANIFEST.resolve()),
                              str(runtime.resolve())):
            try:
                apps.removeApplicationManifest(path_to_clear)
            except Exception:
                pass
        try:
            apps.addApplicationManifest(str(runtime.resolve()), False)
        except Exception as exc:
            msg = str(exc)
            if "AppKeyAlreadyExists" not in msg and "already" not in msg.lower():
                _say(f"[overlay_diag][WARN] addApplicationManifest: {exc}")
        try:
            apps.addApplicationManifest(str(runtime.resolve()), True)
        except Exception:
            pass
        try:
            apps.identifyApplication(os.getpid(), APP_KEY)
            _say(f"[overlay_diag] identifyApplication OK "
                 f"(pid={os.getpid()}, app_key={APP_KEY!r}).")
        except Exception as exc:
            _say(f"[overlay_diag][WARN] identifyApplication: {exc}")

        # ---- Action manifest path ----------------------------------
        try:
            vri = openvr.VRInput()
            vri.setActionManifestPath(str(ACTIONS_PATH.resolve()))
        except Exception as exc:
            _say(f"FATAL -- VRInput.setActionManifestPath: {exc}")
            return 1
        try:
            h_action_set = vri.getActionSetHandle("/actions/teleop")
        except Exception as exc:
            _say(f"FATAL -- getActionSetHandle: {exc}")
            return 1

        handles: Dict[str, int] = {}
        for path in (
            "/actions/teleop/in/trigger_left",
            "/actions/teleop/in/trigger_right",
            "/actions/teleop/in/grip_left",
            "/actions/teleop/in/grip_right",
            "/actions/teleop/in/menu_left",
            "/actions/teleop/in/menu_right",
        ):
            try:
                handles[path] = vri.getActionHandle(path)
            except Exception as exc:
                _say(f"[overlay_diag][WARN] getActionHandle({path!r}): {exc}")

        # ---- Create overlay to GRAB FOCUS --------------------------
        # An Overlay-type app gets focus when its overlay is shown +
        # the user's gaze is on it OR when there is no scene app
        # currently competing for focus.  Showing the overlay also
        # signals SteamVR to route action data to us.
        try:
            ovr = openvr.VROverlay()
        except Exception as exc:
            _say(f"FATAL -- VROverlay interface unavailable: {exc}")
            return 1

        try:
            overlay_handle = ovr.createOverlay(
                f"{APP_KEY}.diag",
                "UST Diagnose Overlay",
            )
        except Exception as exc:
            _say(f"FATAL -- createOverlay: {exc}")
            return 1

        # Position roughly 1m in front of the standing user (identity
        # rotation at +Z = -1.0).  This is the canonical "I'm the
        # active overlay" placement.
        try:
            ovr.setOverlayWidthInMeters(overlay_handle, 0.4)
        except Exception:
            pass
        try:
            mat = openvr.HmdMatrix34_t()
            mat[0][0] = 1.0
            mat[1][1] = 1.0
            mat[2][2] = 1.0
            mat[2][3] = -1.0  # 1m forward (-Z)
            ovr.setOverlayTransformAbsolute(
                overlay_handle,
                openvr.TrackingUniverseStanding,
                mat,
            )
        except Exception as exc:
            _say(f"[overlay_diag][WARN] setOverlayTransformAbsolute: {exc}")
        try:
            ovr.showOverlay(overlay_handle)
            _say("[overlay_diag] overlay shown.  We should now be eligible "
                 "for input focus.")
        except Exception as exc:
            _say(f"[overlay_diag][WARN] showOverlay: {exc}")

        # Brief settle so SteamVR commits focus to us
        time.sleep(2.0)

        # ---- Probe loop --------------------------------------------
        _say("")
        _say(f"[overlay_diag] Listening for {args.seconds:.0f}s.  Squeeze "
             f"trigger / grip on each controller.")
        _say("Columns:  A_*(a0/a1)  -- value(bActive flag).")
        _say("If a1 + nonzero appear: BINDING WORKS.  Production teleop in")
        _say("--render_mode steamvr_native will see the same input.")
        _say("If still a0: the binding pipeline is misconfigured at a deeper")
        _say("level (rare; check vrserver.txt log for 'Binding load' errors).")
        _say("")

        deadline = time.time() + args.seconds
        last_print = 0.0
        period = 1.0 / max(0.5, float(args.rate))
        any_active = False
        any_nonzero = False
        while time.time() < deadline:
            now = time.time()
            if now - last_print < period:
                time.sleep(0.01)
                continue
            last_print = now
            try:
                vri.updateActionState([
                    openvr.VRActiveActionSet_t(
                        ulActionSet=h_action_set,
                        ulRestrictedToDevice=openvr.k_ulInvalidInputValueHandle,
                    )
                ])
            except Exception:
                pass
            row = []
            for side in ("Left ", "Right"):
                key = side.strip().lower()
                vals = {}
                for chan in ("trigger", "grip"):
                    h = handles.get(f"/actions/teleop/in/{chan}_{key}")
                    if h is None:
                        vals[chan] = (0.0, False)
                        continue
                    try:
                        d = vri.getAnalogActionData(h)
                        v = float(d.x)
                        a = bool(getattr(d, "bActive", False))
                    except Exception:
                        v, a = 0.0, False
                    vals[chan] = (v, a)
                    if a:
                        any_active = True
                    if v > 0.05:
                        any_nonzero = True
                row.append(
                    f"{side}: trig={vals['trigger'][0]:.2f}"
                    f"(a{int(vals['trigger'][1])}) "
                    f"grip={vals['grip'][0]:.2f}"
                    f"(a{int(vals['grip'][1])})"
                )
            elapsed = now - (deadline - args.seconds)
            _say(f"  t={elapsed:4.1f}s  " + "   |   ".join(row))

        try:
            ovr.hideOverlay(overlay_handle)
        except Exception:
            pass

        _say("")
        _say("=" * 70)
        _say("Verdict (overlay mode):")
        _say(f"  any bActive=True observed: {'YES' if any_active else 'no'}")
        _say(f"  any nonzero value observed: {'YES' if any_nonzero else 'no'}")
        if any_active and any_nonzero:
            _say("")
            _say("OK -- binding pipeline is healthy.")
            _say("  In production: run --render_mode steamvr_native so Isaac")
            _say("  Sim is the scene app and our actions activate.  monitor")
            _say("  mode does NOT receive Action API data because Isaac Sim")
            _say("  in monitor mode is not a SteamVR scene app.")
            return 0
        if any_active and not any_nonzero:
            _say("")
            _say("PARTIAL -- binding is applied (a1) but no input observed.")
            _say("  Either you didn't squeeze, or the binding routes the")
            _say("  wrong source.  Open SteamVR Binding Editor and verify.")
            return 1
        _say("")
        _say("FAIL -- still no bActive=True even in overlay mode.")
        _say("  This is rare.  Check (in priority order):")
        _say("   1. SteamVR is running with the headset paired (vrserver.txt")
        _say("      should NOT mention 'no HMD').")
        _say("   2. PICO Connect is streaming and prism driver is ON.")
        _say("   3. Manage Controller Bindings > UST Teleop GR1T2 Gripper")
        _say("      > Active Controller Binding = Default + Save Personal Binding.")
        _say("   4. <Steam>/logs/vrserver.txt grep for 'ust.teleop.gr1t2_gripper'")
        _say("      to see if SteamVR even attempted to load our binding.")
        return 2
    finally:
        try:
            openvr.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
