"""Open SteamVR's "Manage Controller Bindings" UI focused on our app.

When ``diagnose_controller_raw`` / ``diagnose_gripper`` report the 9.39
bActive=False verdict (every Action API channel inactive even though the
PICO controllers and PICO Connect streaming are healthy), it means
SteamVR has not applied any Personal Binding to ``ust.teleop.gr1t2_gripper``.
The user has to open SteamVR's Binding Editor for our app, pick a Default
binding, and click "Save Personal Binding".

Doing that manually requires navigating SteamVR > Settings > Controllers >
Manage Controller Bindings > scrolling to find our app -- which most users
get wrong on the first try because SteamVR can show 30+ apps, and our
``ust.teleop.gr1t2_gripper`` row is easy to miss.

This helper takes the shortcut: it registers our manifest (same as the
runtime device) and then calls ``IVRInput::OpenBindingUI`` with our app
key.  SteamVR opens the Binding Editor focused on our app, on the desktop
mirror window, ready for the user to click "Save Personal Binding".

Usage::

    $env:PYTHONPATH = "."
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.open_binding_ui

The script blocks for ``--wait`` seconds (default 30) so SteamVR has time
to render the dialog before this script's openvr.shutdown() tears down
the OpenVR session.
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


# ── stdout hardening (same pattern as 9.38 enumerate_trackers) ──────────
try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _say(msg: str) -> None:
    print(msg, flush=True)


def _vrserver_running() -> Optional[bool]:
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


def _generate_runtime_manifest(static_manifest_path: Path) -> Path:
    """Materialise a runtime ``.vrmanifest`` alongside the static one with
    ``binary_path_windows`` set to the current Python interpreter, mirroring
    what ``GR1T2GripperDevice.start`` does so the registered app entry
    matches the live PID.  See ``teleop/gr1t2_gripper_device.py`` /
    memory.md §10.39 for rationale.
    """
    with open(static_manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    py_exe = os.path.abspath(sys.executable)
    for app in manifest.get("applications", []):
        app["binary_path_windows"] = py_exe
    runtime_path = static_manifest_path.with_name("manifest.runtime.vrmanifest")
    with open(runtime_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    return runtime_path


def main() -> int:
    _say("[open_binding_ui] starting...")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--app-key",
        type=str,
        default="ust.teleop.gr1t2_gripper",
        help="Application key registered by ust_hm_grip (default: %(default)s).",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path(
            "ust_ws/ust_hm_grip/config/openvr_actions/manifest.vrmanifest"
        ),
        help="Path to the static manifest.vrmanifest (default: %(default)s).",
    )
    parser.add_argument(
        "--show-on-desktop",
        action="store_true",
        default=True,
        help="Show the Binding Editor on the desktop monitor (default).",
    )
    parser.add_argument(
        "--in-headset",
        action="store_true",
        help="Show the Binding Editor in the headset instead of the desktop.",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=30.0,
        help="Seconds to keep the OpenVR session alive after opening the UI "
             "so SteamVR has time to render and the user has time to interact "
             "(default 30).",
    )
    parser.add_argument(
        "--init-timeout",
        type=float,
        default=60.0,
        help="Max seconds to wait for openvr.init() before aborting (default 60).",
    )
    parser.add_argument(
        "--skip-steamvr-check",
        action="store_true",
        help="Skip the vrserver.exe pre-check.",
    )
    args = parser.parse_args()

    show_on_desktop = bool(args.show_on_desktop and not args.in_headset)

    # ── SteamVR pre-check ─────────────────────────────────────────────
    if not args.skip_steamvr_check:
        _say("[open_binding_ui] checking whether vrserver.exe (SteamVR core) is running...")
        running = _vrserver_running()
        if running is False:
            _say("")
            _say("[open_binding_ui] FAIL: SteamVR (vrserver.exe) is NOT running.")
            _say("  Launch SteamVR first (Steam > Library > Tools > SteamVR), wait")
            _say("  until the headset icon turns green, then re-run this command.")
            return 2
        if running is True:
            _say("[open_binding_ui] vrserver.exe found.  OK.")
        else:
            _say("[open_binding_ui] could not determine SteamVR state; proceeding.")

    # ── openvr import + init ──────────────────────────────────────────
    _say("[open_binding_ui] importing openvr (pyopenvr)...")
    try:
        import openvr
    except ImportError as exc:
        _say(f"[open_binding_ui] FAIL: pyopenvr is not installed ({exc}).")
        _say("  pip install openvr")
        return 1

    _say(
        f"[open_binding_ui] calling openvr.init(VRApplication_Other) "
        f"(timeout={args.init_timeout:.0f}s)..."
    )
    t0 = time.time()
    system, init_exc = _init_openvr_with_timeout(openvr, args.init_timeout)
    if isinstance(init_exc, TimeoutError):
        _say(f"[open_binding_ui] FAIL: {init_exc}")
        return 3
    if init_exc is not None:
        _say(f"[open_binding_ui] FAIL: openvr.init raised {type(init_exc).__name__}: {init_exc}")
        return 1
    _say(f"[open_binding_ui] openvr.init OK ({time.time()-t0:.1f}s).")

    try:
        # ── Manifest registration (idempotent) ────────────────────────
        # SteamVR lazily loads the action manifest only after an app is
        # registered with a known binary path; without this step,
        # OpenBindingUI for our app_key is silently rejected.
        manifest = args.manifest.resolve()
        if not manifest.exists():
            _say(
                f"[open_binding_ui] FAIL: manifest not found at {manifest}.\n"
                "  Run scripts/run_teleop.py once first to bootstrap the "
                "config, or pass --manifest explicitly."
            )
            return 1
        _say(f"[open_binding_ui] generating runtime manifest from {manifest}...")
        runtime_manifest = _generate_runtime_manifest(manifest)
        _say(f"[open_binding_ui] runtime manifest: {runtime_manifest}")

        try:
            apps = openvr.VRApplications()
        except Exception as exc:  # noqa: BLE001
            _say(f"[open_binding_ui] FAIL: VRApplications interface unavailable: {exc}")
            return 1

        # Persist + per-session register, swallowing AppKeyAlreadyExists.
        try:
            apps.addApplicationManifest(str(runtime_manifest), False)
            _say(
                f"[open_binding_ui] persistent manifest registration OK "
                f"({str(runtime_manifest)!r}, app_key={args.app_key!r})."
            )
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            if "AppKeyAlreadyExists" in msg or "already" in msg.lower():
                _say("[open_binding_ui] persistent manifest already registered (OK).")
            else:
                _say(f"[open_binding_ui][WARN] addApplicationManifest persistent: {exc}")
        try:
            apps.addApplicationManifest(str(runtime_manifest), True)
            _say("[open_binding_ui] current-session manifest registration OK.")
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            if "AppKeyAlreadyExists" in msg or "already" in msg.lower():
                _say("[open_binding_ui] current-session manifest already registered (OK).")
            else:
                _say(f"[open_binding_ui][WARN] addApplicationManifest session: {exc}")

        try:
            apps.identifyApplication(os.getpid(), args.app_key)
            _say(
                f"[open_binding_ui] identifyApplication OK "
                f"(pid={os.getpid()}, app_key={args.app_key!r})."
            )
        except Exception as exc:  # noqa: BLE001
            _say(f"[open_binding_ui][WARN] identifyApplication: {exc}")

        # ── OpenBindingUI ─────────────────────────────────────────────
        try:
            vri = openvr.VRInput()
        except Exception as exc:  # noqa: BLE001
            _say(f"[open_binding_ui] FAIL: VRInput interface unavailable: {exc}")
            return 1

        # OpenBindingUI(app_key, action_set_handle, device_handle, show_on_desktop)
        # - action_set_handle = invalid -> SteamVR shows the full app's binding
        # - device_handle     = invalid -> not restricted to a single device
        # - show_on_desktop   = True    -> open the dialog on the PC monitor
        invalid_handle = openvr.k_ulInvalidActionSetHandle
        invalid_input = openvr.k_ulInvalidInputValueHandle
        _say(
            f"[open_binding_ui] calling OpenBindingUI(app_key={args.app_key!r}, "
            f"show_on_desktop={show_on_desktop})..."
        )
        try:
            vri.openBindingUI(
                args.app_key,
                invalid_handle,
                invalid_input,
                bool(show_on_desktop),
            )
        except Exception as exc:  # noqa: BLE001
            _say(f"[open_binding_ui] FAIL: openBindingUI raised: {type(exc).__name__}: {exc}")
            _say(
                "  This usually means SteamVR rejected the app_key (manifest\n"
                "  not yet visible in the registry).  Run scripts/run_teleop.py\n"
                "  ONCE to fully bootstrap the manifest, then retry this script."
            )
            return 1
        _say("[open_binding_ui] OpenBindingUI dispatched.  Look for the SteamVR")
        _say("  Binding Editor window on your desktop (or in-headset if --in-headset).")
        _say("")
        _say("  Steps to take in the dialog:")
        _say("    1. 'Active Controller Binding' section -> select")
        _say("       'UST Teleop GR1T2 Gripper Default' (or any binding that")
        _say("       has trigger/grip Pull mappings).")
        _say("    2. (If a stale Personal Binding shows up) click 'Reset to")
        _say("       Default' first so the new bindings_pico.json mode takes")
        _say("       effect (memory.md §10.40 / 9.32 grip-mode change).")
        _say("    3. Click 'Save Personal Binding' at the BOTTOM of the dialog.")
        _say("       This is the most-commonly-missed step.")
        _say("    4. Close the dialog.")
        _say("")
        _say(f"  Holding the OpenVR session alive for {args.wait:.0f}s so the")
        _say("  Binding Editor can finish rendering before openvr.shutdown().")
        _say("  Press Ctrl-C to release immediately once you're done.")
        try:
            time.sleep(args.wait)
        except KeyboardInterrupt:
            _say("\n[open_binding_ui] released early by Ctrl-C.")
        _say("[open_binding_ui] After saving, verify with:")
        _say("    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw")
        _say("  Every Action API channel should now show (a1) and respond when")
        _say("  you squeeze the trigger / grip on the PICO controllers.")
    finally:
        try:
            openvr.shutdown()
        except Exception:  # noqa: BLE001
            pass

    _say("[open_binding_ui] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
