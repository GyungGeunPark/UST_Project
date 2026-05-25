"""Redirect SteamVR's per-app ``CurrentURL_steamvrinput`` to our disk binding.

Background
----------
SteamVR stores the *currently-active* Personal Binding for each registered
application (identified by ``app_key`` in the .vrmanifest) inside
``%STEAMVR_CONFIG%/steamvr.vrsettings`` under a key of the form::

    "<app_key>" : {
        "<controller_type>_<openvr_controller_id>_CurrentURL_steamvrinput"  : "<url>",
        "<controller_type>_<openvr_controller_id>_PreviousURL_steamvrinput" : "<url>",
        "<controller_type>_<openvr_controller_id>_AutosaveURL_steamvrinput" : "<url>",
        ...
    }

The URL may be a Steam Workshop URL (``vr-input-workshop://<id>``) or a
local file URL (``file:///<absolute path to *.json>``).

Why this script
---------------
In session #10 we identified that SteamVR's ``vrwebui_shared.js`` "Save
Personal Binding" flow silently fails when ``m_sInteractionProfile`` is
``null`` (see ``patch_steamvr_binding_bug.py``).  After applying the
patch, the commit *succeeds* -- but it commits whatever URL was already
selected in the SteamVR binding editor, which for users browsing the
Workshop is a Workshop URL, not our local file.

SteamVR's UI does not expose a "Import from URL" or "Load from File"
menu in the current 2.15.6 build of the binding editor, so the only
reliable way to point the runtime at our on-disk binding file is to
edit ``steamvr.vrsettings`` directly while SteamVR is offline, then
restart.

This script does exactly that:

  * **idempotent**: re-running with the same target URL is a no-op
  * **reversible**: ``--revert`` restores from the most recent backup
  * **safe**:
      - refuses to write while SteamVR is running (locks the file +
        the process will overwrite on shutdown)
      - always creates a timestamped backup before writing
      - validates JSON syntax before *and* after edit

Usage
-----

    # apply (default app: ust.teleop.gr1t2_gripper, default file: bindings_pico_controller.json)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.redirect_steamvr_binding_url

    # check only (does not modify)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.redirect_steamvr_binding_url --check

    # revert (restore latest backup)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.redirect_steamvr_binding_url --revert

    # override the disk binding path
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.redirect_steamvr_binding_url \\
        --binding "C:\\custom\\bindings_pico_controller.json"

    # override the app_key (if you forked the manifest)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.redirect_steamvr_binding_url \\
        --app-key ust.teleop.gr1t2_gripper_v2

Notes
-----
* ``AutosaveURL_steamvrinput`` is DELETED when we redirect, so SteamVR
  does not try to restore from a stale Workshop autosave on next launch.
* ``PreviousURL_steamvrinput`` is preserved (it only affects the "Undo"
  arrow in the binding editor, not the runtime).
* You should still run ``patch_steamvr_binding_bug.py`` first -- without
  the patch, any *user* edit in the binding editor will still silently
  fail, so the moment the user touches the UI the redirect can be lost.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional


# Default paths.  All overridable via CLI.
DEFAULT_STEAMVR_CONFIG = Path(
    r"C:\Program Files (x86)\Steam\config\steamvr.vrsettings"
)
DEFAULT_APP_KEY = "ust.teleop.gr1t2_gripper"
# This is the ust_hm_grip package's controller_type=pico_controller binding.
DEFAULT_BINDING = Path(
    r"C:\develop\IsaacLab\ust_ws\ust_hm_grip\config\openvr_actions"
    r"\bindings_pico_controller.json"
)
# Controller-type prefix that SteamVR uses inside the per-app block.
# For PICO 4 / 4 Ultra controllers reported as ``pico_controller`` with
# OpenVR controller id ``250820``, the prefix is ``pico_controller_250820_``.
# A different controller (e.g. ``pico_neo3_controller_NNNNN``) would have
# a different prefix; --controller-prefix overrides.
DEFAULT_CONTROLLER_PREFIX = "pico_controller_250820_"

# SteamVR process names whose presence makes the file unsafe to edit.
STEAMVR_PROCS = (
    "vrserver",
    "vrcompositor",
    "vrmonitor",
    "vrwebhelper",
    "vrdashboard",
    "vrstartup",
    "steamvr",
)


def _stamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _to_file_url(path: Path) -> str:
    """Convert ``C:\\foo\\bar.json`` -> ``file:///C:/foo/bar.json``.

    SteamVR accepts forward-slash form; the leading ``file:///`` (three
    slashes) is required for Windows absolute paths.
    """
    p = path.resolve()
    # SteamVR's resolver expects forward slashes after ``file:///``.
    posix = str(p).replace("\\", "/")
    # On Windows ``str(p)`` is e.g. ``C:/foo/bar.json``; prepend ``file:///``.
    return f"file:///{posix}"


def _steamvr_running() -> list[str]:
    """Return a list of running SteamVR process names (best-effort)."""
    try:
        out = subprocess.check_output(
            ["tasklist", "/FO", "CSV", "/NH"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return []
    running: list[str] = []
    for line in out.splitlines():
        # CSV: "image.exe","pid","Console","1","12345 K"
        if not line or not line.startswith('"'):
            continue
        name = line.split('","', 1)[0].lstrip('"').lower()
        if name.endswith(".exe"):
            name = name[:-4]
        if name in STEAMVR_PROCS:
            running.append(name)
    return running


def _backup(path: Path) -> Path:
    bak = path.with_suffix(path.suffix + f".bak.ust.{_stamp()}")
    shutil.copy2(path, bak)
    return bak


def _latest_backup(path: Path) -> Optional[Path]:
    bks = sorted(
        path.parent.glob(path.name + ".bak.ust.*"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return bks[0] if bks else None


def _load(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _dump(path: Path, obj: dict) -> None:
    # SteamVR's own writer uses 3-space indent and a literal space before
    # each ``:`` separator.  We mimic that so diffs against SteamVR-written
    # files stay minimal; SteamVR also accepts standard ``": "`` so this
    # is purely cosmetic.
    text = json.dumps(obj, indent=3, separators=(",", " : "), sort_keys=True)
    # ensure final newline
    if not text.endswith("\n"):
        text += "\n"
    with path.open("w", encoding="utf-8") as f:
        f.write(text)


def _find_keys(block: dict, prefix: str) -> dict[str, Optional[str]]:
    """Pick out the binding-URL keys from a per-app block."""
    return {
        "current":  f"{prefix}CurrentURL_steamvrinput",
        "previous": f"{prefix}PreviousURL_steamvrinput",
        "autosave": f"{prefix}AutosaveURL_steamvrinput",
        "need_autosave": f"{prefix}NeedToUpdateAutosave_steamvrinput",
    }


def _status(cfg: dict, app_key: str, prefix: str, target_url: str) -> None:
    block = cfg.get(app_key)
    if block is None:
        print(f"  [--] app block missing: {app_key}")
        return
    keys = _find_keys(block, prefix)
    cur = block.get(keys["current"])
    prev = block.get(keys["previous"])
    auto = block.get(keys["autosave"])
    print(f"  app_key  : {app_key}")
    print(f"  current  : {cur}")
    print(f"  previous : {prev}")
    print(f"  autosave : {auto}")
    if cur == target_url:
        print(f"  [OK ] CurrentURL already points at target: {target_url}")
    else:
        print(f"  [REDIR] CurrentURL needs update -> {target_url}")


def _purge(
    config_path: Path,
    app_key: str,
    prefix: str,
) -> int:
    """Delete ALL ``<prefix>*URL_steamvrinput`` keys for ``app_key``.

    With every URL key gone, SteamVR's binding loader falls through to
    the action-manifest's ``default_bindings`` entry, which (for our
    project) resolves to ``bindings_pico_controller.json`` next to
    ``actions.json``.  This is the cleanest way to disengage a stale
    Workshop binding that SteamVR keeps auto-restoring.
    """
    try:
        cfg = _load(config_path)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[FAIL] load {config_path}: {exc}")
        return 2

    block = cfg.get(app_key)
    if block is None:
        print(f"[OK ] no app block to purge: {app_key}")
        return 0

    # Find all keys that start with the prefix AND look like binding-state.
    purge_targets = [
        k for k in list(block.keys())
        if k.startswith(prefix) and (
            "URL_steamvrinput" in k
            or "NeedToUpdateAutosave_steamvrinput" in k
        )
    ]
    if not purge_targets:
        print(f"[OK ] no matching URL keys under {app_key} (prefix={prefix!r})")
        return 0

    print(f"[info] purging {len(purge_targets)} key(s) under {app_key!r}:")
    for k in purge_targets:
        print(f"         - {k} = {block[k]!r}")

    bak = _backup(config_path)
    for k in purge_targets:
        del block[k]
    # If the per-app block is now empty, drop it entirely too.
    if not block:
        print(f"[info] app block is empty after purge -- removing the block.")
        del cfg[app_key]

    try:
        _dump(config_path, cfg)
    except OSError as exc:
        try:
            shutil.copy2(bak, config_path)
        except OSError:
            pass
        print(f"[FAIL] write {config_path}: {exc} (restored backup {bak.name})")
        return 2

    print(f"[OK ] purged (backup -> {bak.name})")
    print("       SteamVR will fall back to action-manifest default_bindings on next launch.")
    return 0


def _apply(
    config_path: Path,
    app_key: str,
    prefix: str,
    target_url: str,
    dry_run: bool,
) -> int:
    try:
        cfg = _load(config_path)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[FAIL] load {config_path}: {exc}")
        return 2

    block = cfg.get(app_key)
    if block is None:
        # Create a minimal block.  SteamVR will fill in the rest on next launch.
        print(f"[info] app block missing -- creating: {app_key}")
        block = {}
        cfg[app_key] = block

    keys = _find_keys(block, prefix)
    cur = block.get(keys["current"])
    if cur == target_url:
        print(f"[OK ] already redirected: {keys['current']} = {target_url}")
        return 0

    print(f"[info] CurrentURL change:")
    print(f"         from: {cur}")
    print(f"         to  : {target_url}")

    # Stage the change in-memory.
    block[keys["current"]] = target_url
    # Remove stale autosave -- otherwise SteamVR may try to roll back to it
    # the first time the user opens the binding editor.
    if keys["autosave"] in block:
        print(f"[info] dropping stale {keys['autosave']} = {block[keys['autosave']]}")
        del block[keys["autosave"]]
    # Always mark "no pending autosave" so SteamVR does not nag.
    block[keys["need_autosave"]] = False

    if dry_run:
        print("[DRY] would write redirect (use without --check to apply).")
        return 0

    bak = _backup(config_path)
    try:
        _dump(config_path, cfg)
    except OSError as exc:
        # restore from backup if write fails
        try:
            shutil.copy2(bak, config_path)
        except OSError:
            pass
        print(f"[FAIL] write {config_path}: {exc} (restored backup {bak.name})")
        return 2

    # Re-load and confirm.
    try:
        re_cfg = _load(config_path)
        re_cur = re_cfg.get(app_key, {}).get(keys["current"])
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[FAIL] re-load after write: {exc}")
        return 2
    if re_cur != target_url:
        print(f"[FAIL] post-write check: CurrentURL is {re_cur!r}, expected {target_url!r}")
        return 2
    print(f"[OK ] redirected (backup -> {bak.name})")
    return 0


def _revert(config_path: Path) -> int:
    bak = _latest_backup(config_path)
    if bak is None:
        print(f"[--] no backup found for {config_path}")
        return 1
    try:
        shutil.copy2(bak, config_path)
    except OSError as exc:
        print(f"[FAIL] revert {config_path}: {exc}")
        return 2
    print(f"[OK ] reverted {config_path.name} from {bak.name}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--steamvr-config", type=Path, default=DEFAULT_STEAMVR_CONFIG,
                    help="Path to steamvr.vrsettings (default: %(default)s)")
    ap.add_argument("--app-key", default=DEFAULT_APP_KEY,
                    help="OpenVR application key (default: %(default)s)")
    ap.add_argument("--controller-prefix", default=DEFAULT_CONTROLLER_PREFIX,
                    help="Controller-type prefix incl. trailing underscore "
                         "(default: %(default)s)")
    ap.add_argument("--binding", type=Path, default=DEFAULT_BINDING,
                    help="Local binding JSON file (default: %(default)s)")
    ap.add_argument("--url", default=None,
                    help="Override target URL.  If omitted, derived from --binding "
                         "as file:///<absolute path>.")
    ap.add_argument("--check", action="store_true",
                    help="Show current/desired state without modifying.")
    ap.add_argument("--revert", action="store_true",
                    help="Restore from the most recent ust-style backup.")
    ap.add_argument("--purge", action="store_true",
                    help="DELETE all *URL_steamvrinput + NeedToUpdateAutosave "
                         "keys under <app_key>.  After this, SteamVR falls back "
                         "to action-manifest default_bindings on next launch.  "
                         "Use this when SteamVR keeps reverting to a stale "
                         "Workshop binding.")
    ap.add_argument("--allow-running", action="store_true",
                    help="Override the safety check that refuses to write "
                         "while SteamVR is running.  Dangerous.")
    args = ap.parse_args()

    cfg_path: Path = args.steamvr_config
    if not cfg_path.exists():
        print(f"[FATAL] steamvr.vrsettings not found: {cfg_path}")
        return 2

    # Build target URL.
    if args.url is not None:
        target_url = args.url
    else:
        if not args.binding.exists():
            print(f"[FATAL] binding file not found: {args.binding}")
            return 2
        target_url = _to_file_url(args.binding)

    print(f"[info] steamvr.vrsettings : {cfg_path}")
    print(f"[info] app_key            : {args.app_key}")
    print(f"[info] controller prefix  : {args.controller_prefix}")
    print(f"[info] target URL         : {target_url}")
    print()

    if args.revert:
        return _revert(cfg_path)

    if args.purge:
        # Purge mutates the file -- enforce the same SteamVR-offline guard.
        running = _steamvr_running()
        if running and not args.allow_running:
            print("[FATAL] SteamVR is currently running -- the file is held open")
            print(f"        and any edit will be lost.  Running: {running}")
            print("        Quit SteamVR first (tray -> 'Exit SteamVR'), or pass")
            print("        --allow-running to override (NOT recommended).")
            return 3
        return _purge(cfg_path, args.app_key, args.controller_prefix)

    # --check is read-only -- safe even while SteamVR is running.
    if args.check:
        try:
            cfg = _load(cfg_path)
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[FAIL] load {cfg_path}: {exc}")
            return 2
        _status(cfg, args.app_key, args.controller_prefix, target_url)
        return 0

    # Mutating path -- refuse if SteamVR is running.
    running = _steamvr_running()
    if running and not args.allow_running:
        print("[FATAL] SteamVR is currently running -- the file is held open")
        print(f"        and any edit will be lost.  Running: {running}")
        print("        Quit SteamVR first (tray -> 'Exit SteamVR'), or pass")
        print("        --allow-running to override (NOT recommended).")
        return 3

    rc = _apply(cfg_path, args.app_key, args.controller_prefix, target_url,
                dry_run=False)
    if rc == 0:
        print()
        print("[done] redirect applied.")
        print("       NEXT STEPS:")
        print("         1. Launch SteamVR (PICO Connect PCVR reconnect will")
        print("            spawn vrserver automatically).")
        print("         2. SteamVR Settings > Controllers > Manage Controller")
        print("            Bindings -> 'UST Teleop GR1T2 Gripper' should now")
        print("            show our local file binding (not a Workshop entry).")
        print("         3. Verify Action API channels via:")
        print("              python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw --seconds 10")
        print("            Expect: A_trig=0.xx(a1) / A_grip=0.xx(a1) when squeezing.")
        print()
        print("       NOTE: SteamVR may rewrite this file when it shuts down.")
        print("             If the redirect is lost, re-run this script.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
