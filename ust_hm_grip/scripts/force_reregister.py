"""Force-unregister and re-register the ust_hm_grip manifest with SteamVR.

THE problem this solves
-----------------------

SteamVR's application registry caches manifest content at registration
time, indexed by ``app_key``.  Calling ``IVRApplications::AddApplicationManifest``
again with the same ``app_key`` returns ``VRApplicationError_AppKeyAlreadyExists``
and is **a no-op** -- SteamVR keeps using the previously cached manifest
contents, including ``default_bindings`` and ``action_manifest_path``.

Symptom: you change ``actions.json`` / ``bindings_*.json`` on disk, but
the Action API keeps returning ``bActive=False`` because SteamVR is
still resolving against the old manifest snapshot.  No matter how many
times you toggle Default/Custom in Manage Controller Bindings, no
matter how many Personal Bindings you save, the cached binding URL
points at a file that no longer matches the current ``controller_type``.

This is exactly the failure mode in memory.md sections 10.42 / 10.49.
The 9.42 per-controller_type split fixes the disk-side data; this 9.43
helper forces SteamVR to discard its cached snapshot and re-read it.

Mechanism
---------

1. ``openvr.init`` (lightweight session, VRApplication_Other).
2. ``apps.removeApplicationManifest(<path>)`` for both the static and
   runtime manifest paths -- ignoring ``AppKeyDoesNotExist``.
3. Generate a fresh ``manifest.runtime.vrmanifest`` with the current
   ``sys.executable`` baked in (same as ``GR1T2GripperDevice.start``).
4. ``apps.addApplicationManifest(runtime_path, False)`` -- this time
   SteamVR has no cached entry, so it actually reads the file.
5. ``apps.identifyApplication(pid, app_key)`` to verify the registry
   accepted the new manifest.
6. Print the registry list before & after so the user can see the
   re-registration succeeded.
7. ``openvr.shutdown``.

After running this, **restart SteamVR** so any in-memory binding lookup
state is also flushed.  The next time you launch a teleop / diagnostic
script, SteamVR resolves the binding from the new on-disk files.

Usage::

    python -X utf8 -m ust_ws.ust_hm_grip.scripts.force_reregister
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.force_reregister --dry-run
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


# stdout hardening (9.38 pattern)
try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _say(msg: str) -> None:
    print(msg, flush=True)


APP_KEY = "ust.teleop.gr1t2_gripper"
CFG_DIR = ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions"
STATIC_MANIFEST = CFG_DIR / "manifest.vrmanifest"
RUNTIME_MANIFEST = CFG_DIR / "manifest.runtime.vrmanifest"


def _generate_runtime_manifest() -> Path:
    """Materialise manifest.runtime.vrmanifest next to the static one with
    binary_path_windows = current python interpreter."""
    with open(STATIC_MANIFEST, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    py_exe = os.path.abspath(sys.executable)
    for app in manifest.get("applications", []):
        app["binary_path_windows"] = py_exe
    with open(RUNTIME_MANIFEST, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    return RUNTIME_MANIFEST


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


def main() -> int:
    _say("[force_reregister] starting...")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true",
                        help="Print plan without modifying anything.")
    parser.add_argument("--init-timeout", type=float, default=60.0)
    args = parser.parse_args()

    if not STATIC_MANIFEST.exists():
        _say(f"[force_reregister] FAIL: {STATIC_MANIFEST} not found.")
        return 2
    if not (CFG_DIR / "actions.json").exists():
        _say(f"[force_reregister] FAIL: actions.json not found in {CFG_DIR}.")
        return 2

    _say(f"[force_reregister] paths:")
    _say(f"  app_key          = {APP_KEY}")
    _say(f"  static manifest  = {STATIC_MANIFEST}")
    _say(f"  runtime manifest = {RUNTIME_MANIFEST}")

    try:
        import openvr
    except ImportError as exc:
        _say(f"[force_reregister] FAIL: pyopenvr not installed ({exc})")
        return 1

    _say("")
    _say("[force_reregister] calling openvr.init(VRApplication_Other)...")
    t0 = time.time()
    system, init_exc = _init_openvr_with_timeout(openvr, args.init_timeout)
    if init_exc is not None or system is None:
        _say(f"[force_reregister] FAIL: openvr.init -> {init_exc}")
        return 1
    _say(f"[force_reregister] openvr.init OK ({time.time() - t0:.1f}s).")

    try:
        try:
            apps = openvr.VRApplications()
        except Exception as exc:  # noqa: BLE001
            _say(f"[force_reregister] FAIL: VRApplications interface: {exc}")
            return 1

        # ---- pre: list registry entries that mention our app_key
        _say("")
        _say("[force_reregister] BEFORE -- SteamVR application registry entries "
             f"matching {APP_KEY!r}:")
        try:
            n_apps = apps.getApplicationCount()
        except Exception:  # noqa: BLE001
            n_apps = 0
        before_paths = []
        for i in range(int(n_apps)):
            try:
                k = apps.getApplicationKeyByIndex(i)
            except Exception:  # noqa: BLE001
                continue
            if isinstance(k, bytes):
                k = k.decode("utf-8", errors="replace")
            if k == APP_KEY:
                _say(f"  [{i:>3}] {k}")
                before_paths.append(k)
        if not before_paths:
            _say("  (none)")

        # ---- step 1: removeApplicationManifest for known paths
        candidates = [
            str(STATIC_MANIFEST.resolve()),
            str(RUNTIME_MANIFEST.resolve()) if RUNTIME_MANIFEST.exists() else None,
        ]
        candidates = [c for c in candidates if c]
        _say("")
        _say("[force_reregister] removing existing manifest registrations:")
        for path in candidates:
            if args.dry_run:
                _say(f"  (dry) would removeApplicationManifest({path!r})")
                continue
            try:
                apps.removeApplicationManifest(path)
                _say(f"  REMOVED  {path}")
            except Exception as exc:  # noqa: BLE001
                msg = str(exc)
                if "NotInstalled" in msg or "DoesNotExist" in msg.lower() or "not installed" in msg.lower():
                    _say(f"  (was not registered) {path}")
                else:
                    _say(f"  FAIL  {path}  ({type(exc).__name__}: {exc})")

        # ---- step 2: regenerate runtime manifest with fresh binary path
        if args.dry_run:
            _say(f"\n[force_reregister] (dry) would regenerate {RUNTIME_MANIFEST}")
        else:
            _say("")
            _say(f"[force_reregister] regenerating runtime manifest...")
            runtime = _generate_runtime_manifest()
            _say(f"  {runtime}  (binary_path_windows = {sys.executable})")

        # ---- step 3: addApplicationManifest fresh
        if args.dry_run:
            _say(f"\n[force_reregister] (dry) would addApplicationManifest({RUNTIME_MANIFEST!s}, False)")
            return 0

        _say("")
        _say("[force_reregister] adding manifest fresh (persistent):")
        try:
            apps.addApplicationManifest(str(RUNTIME_MANIFEST.resolve()), False)
            _say(f"  ADDED  {RUNTIME_MANIFEST}  (persistent)")
        except Exception as exc:  # noqa: BLE001
            _say(f"  FAIL  addApplicationManifest persistent: {exc}")
            return 1

        # Also temp registration for current session
        try:
            apps.addApplicationManifest(str(RUNTIME_MANIFEST.resolve()), True)
            _say(f"  ADDED  {RUNTIME_MANIFEST}  (current session)")
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            if "AppKeyAlreadyExists" in msg or "already" in msg.lower():
                _say(f"  (current-session add already in place)")
            else:
                _say(f"  WARN  addApplicationManifest temp: {exc}")

        # ---- step 4: identifyApplication to verify
        try:
            apps.identifyApplication(os.getpid(), APP_KEY)
            _say(f"  identifyApplication OK  (pid={os.getpid()}, app_key={APP_KEY!r})")
        except Exception as exc:  # noqa: BLE001
            _say(f"  FAIL  identifyApplication: {exc}")
            return 1

        # ---- step 5: report after-state
        _say("")
        _say("[force_reregister] AFTER -- registry now contains:")
        try:
            n_apps2 = apps.getApplicationCount()
        except Exception:  # noqa: BLE001
            n_apps2 = 0
        for i in range(int(n_apps2)):
            try:
                k = apps.getApplicationKeyByIndex(i)
            except Exception:
                continue
            if isinstance(k, bytes):
                k = k.decode("utf-8", errors="replace")
            if k == APP_KEY:
                _say(f"  [{i:>3}] {k}  <-- ours, freshly registered")

        _say("")
        _say("[force_reregister] OK -- manifest re-read by SteamVR.")
        _say("")
        _say("CRITICAL NEXT STEPS:")
        _say("  1. Quit SteamVR fully (system tray > Quit SteamVR).")
        _say("  2. Wait 5s for vrserver.exe / vrcompositor.exe to exit.")
        _say("  3. Launch SteamVR again (Steam > Library > Tools > SteamVR).")
        _say("  4. SteamVR > Settings > Controllers > Manage Controller Bindings:")
        _say("     - Dropdown -> 'UST Teleop GR1T2 Gripper'.")
        _say("     - Active Controller Binding -> 'Default' (NOT Custom).")
        _say("     - Click 'Save Personal Binding' at the bottom of the dialog.")
        _say("     - Close.")
        _say("  5. Re-run diagnose_controller_raw -- expect (a1) flags + nonzero")
        _say("     when squeezing trigger / grip.")
    finally:
        try:
            openvr.shutdown()
        except Exception:  # noqa: BLE001
            pass

    _say("")
    _say("[force_reregister] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
