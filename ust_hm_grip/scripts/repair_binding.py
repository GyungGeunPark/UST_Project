"""Repair SteamVR Personal Bindings for ``ust.teleop.gr1t2_gripper``.

Background -- 9.40 escalation of the 9.39 fast-fix.
================================================================

The 9.39 ``open_binding_ui`` helper relies on the user clicking "Save
Personal Binding" inside SteamVR's Binding Editor.  When the user does
that and the diagnostic STILL shows ``bActive=False`` on every channel,
the most likely remaining cause is a **stale Personal Binding file**
sitting on disk from before the 9.32 grip-mode rewrite.  SteamVR loads
that zombie file in preference to our default ``bindings_pico.json``
and reports ``bActive=False`` because the zombie's input sources don't
exist on the modern ``pico_controller`` profile.

OpenVR has no API for deleting a Personal Binding -- the user has to
do it manually OR we delete the file directly from disk.  This script
takes the disk-direct route.

What it does
------------

1. Scans every known SteamVR Personal Binding directory:
     * ``%LOCALAPPDATA%\\openvr\\input\\``
     * ``<Steam>\\config\\steamvr_input\\``
2. Filters files whose name starts with ``binding_<app_key>_``.
3. ``--list`` (default): prints what's there + diagnoses each file
   (empty / has trigger Pull mappings / has stale force_sensor / etc.)
4. ``--clear``: deletes every match (with backup to ``*.bak`` first).
   After this + a SteamVR restart, our app's default binding from
   ``config/openvr_actions/bindings_pico.json`` will be auto-applied
   because no Personal Binding shadow exists anymore.
5. ``--write-default``: writes a fresh Personal Binding file built
   from our shipped ``bindings_pico.json`` for the requested
   controller_type, so the binding is active even before SteamVR
   restarts.  Use after ``--clear`` when you can't restart SteamVR.

Usage::

    $env:PYTHONPATH = "."
    # 1) See what's there
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding

    # 2) Clear stale Personal Bindings (RECOMMENDED first try)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding --clear

    # 3) (Optional) Force-write a fresh Personal Binding
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding --write-default

    # 4) Restart SteamVR
    # 5) Verify
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw
    # Expect (a1) flags + nonzero values when squeezing.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ── stdout hardening (9.38 pattern) ─────────────────────────────────────
try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


def _say(msg: str) -> None:
    print(msg, flush=True)


# ── Constants ────────────────────────────────────────────────────────────
APP_KEY = "ust.teleop.gr1t2_gripper"
CONTROLLER_TYPES = (
    "pico_controller",
    "pico_neo3_controller",
    "pico4_controller",
    "pico_phoenix_controller",
    "oculus_touch",
    "knuckles",
)
ROOT = Path(__file__).resolve().parents[3]
BINDINGS_PICO_PATH = (
    ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions" / "bindings_pico.json"
)


# ── Path discovery ──────────────────────────────────────────────────────
def _find_local_openvr_input() -> Optional[Path]:
    """``%LOCALAPPDATA%\\openvr\\input\\`` -- SteamVR Personal Bindings.

    This is the primary location SteamVR writes when the user clicks
    "Save Personal Binding" in the Binding Editor.
    """
    local = os.environ.get("LOCALAPPDATA")
    if not local:
        return None
    p = Path(local) / "openvr" / "input"
    return p if p.exists() else None


def _find_steamvr_install() -> Optional[Path]:
    """Locate the SteamVR install directory.

    Tries the standard Steam locations.  Reads ``HKCU\\Software\\Valve\\Steam``
    on Windows for non-default installs.
    """
    candidates = [
        Path(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR"),
        Path(r"D:\Steam\steamapps\common\SteamVR"),
        Path(r"E:\Steam\steamapps\common\SteamVR"),
    ]
    if sys.platform == "win32":
        try:
            import winreg  # type: ignore
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Software\Valve\Steam") as k:
                steam_path = winreg.QueryValueEx(k, "SteamPath")[0]
            steam_root = Path(steam_path)
            candidates.insert(
                0, steam_root / "steamapps" / "common" / "SteamVR"
            )
            # Also: read libraryfolders.vdf for non-default install drives.
            libfolders = steam_root / "steamapps" / "libraryfolders.vdf"
            if libfolders.exists():
                txt = libfolders.read_text(encoding="utf-8", errors="ignore")
                # crude vdf scrape -- look for "path" fields
                for line in txt.splitlines():
                    if '"path"' in line.lower():
                        parts = line.split('"')
                        if len(parts) >= 4:
                            lib = Path(parts[3].replace("\\\\", "\\"))
                            candidates.append(
                                lib / "steamapps" / "common" / "SteamVR"
                            )
        except Exception:  # noqa: BLE001
            pass
    for p in candidates:
        if p.exists():
            return p
    return None


def _find_steamvr_input_dir() -> Optional[Path]:
    """``<Steam>\\config\\steamvr_input\\`` -- SteamVR's per-user binding cache."""
    steam_vr = _find_steamvr_install()
    if steam_vr is None:
        return None
    # SteamVR is at <Steam>\steamapps\common\SteamVR.
    # config dir is at <Steam>\config\steamvr_input.
    steam_root = steam_vr.parent.parent.parent
    p = steam_root / "config" / "steamvr_input"
    return p if p.exists() else None


def find_personal_binding_files() -> List[Path]:
    """Return every Personal Binding file SteamVR knows about for our app."""
    locations = [
        _find_local_openvr_input(),
        _find_steamvr_input_dir(),
    ]
    found: List[Path] = []
    for loc in locations:
        if loc is None:
            continue
        try:
            matches = sorted(loc.glob(f"binding_{APP_KEY}_*.json"))
        except Exception:  # noqa: BLE001
            matches = []
        for m in matches:
            if m.is_file():
                found.append(m)
    return found


def inspect_steamvr_appconfig() -> Dict[str, Any]:
    """9.41: detect ``Active Controller Binding = Custom`` selection.

    SteamVR stores per-app per-controller_type active-binding selections
    in ``<Steam>\\config\\steamvr.vrsettings`` under the
    ``"steamvr"."launcher" .. "bindings"`` tree, and ALSO in
    ``%LOCALAPPDATA%\\openvr\\openvrpaths.vrpath``-relative
    ``appconfig.json``.  When the user clicks "Custom" in the Manage
    Controller Bindings dialog, SteamVR creates an empty Custom slot
    and marks it active EVEN IF the slot has no input mappings.  This
    silently shadows the default and produces bActive=False forever.

    This probe scans the relevant config files for any reference to
    our app_key + Custom and surfaces it so the user knows to click
    "Default" in the Manage Controller Bindings UI (or run --clear).

    Returns ``{found: bool, locations: [Path], hints: [str]}``.
    """
    out: Dict[str, Any] = {"found": False, "locations": [], "hints": []}
    candidates: List[Path] = []
    steam_vr = _find_steamvr_install()
    if steam_vr is not None:
        steam_root = steam_vr.parent.parent.parent
        candidates.append(steam_root / "config" / "steamvr.vrsettings")
    local = os.environ.get("LOCALAPPDATA")
    if local:
        candidates.append(Path(local) / "openvr" / "appconfig.json")
        candidates.append(Path(local) / "openvr" / "input" / "appconfig.json")
    for c in candidates:
        if not c.exists():
            continue
        try:
            text = c.read_text(encoding="utf-8", errors="replace")
        except Exception:  # noqa: BLE001
            continue
        if APP_KEY not in text:
            continue
        out["locations"].append(str(c))
        # Heuristic: look for "ust.teleop.gr1t2_gripper" near "custom"
        # (case-insensitive).  SteamVR vrsettings is JSON-ish; greedy
        # search is sufficient for surfacing the symptom.
        low = text.lower()
        idx = low.find(APP_KEY.lower())
        while idx >= 0:
            window = low[max(0, idx - 200):idx + 600]
            if "custom" in window:
                out["found"] = True
                out["hints"].append(
                    f"  in {c.name}: 'custom' appears within 200/600 chars "
                    f"of '{APP_KEY}' -- likely 'Active Controller Binding = "
                    f"Custom' is set for our app.  Open SteamVR > Settings > "
                    f"Controllers > Manage Controller Bindings > select "
                    f"'UST Teleop GR1T2 Gripper' from the dropdown, and "
                    f"click the 'Default' button next to 'Custom'."
                )
                break
            idx = low.find(APP_KEY.lower(), idx + 1)
    return out


# ── Inspection ──────────────────────────────────────────────────────────
def inspect_binding_file(path: Path) -> Dict[str, Any]:
    """Return a structured summary of a Personal Binding file.

    Surfaces:
      * size, mtime
      * presence of trigger / grip / menu sources
      * whether grip is bound as ``trigger`` (post-9.32) or stale ``force_sensor``
      * whether ``inputs.pull.output`` points at a known action
    """
    info: Dict[str, Any] = {
        "path": str(path),
        "size": path.stat().st_size,
        "mtime": time.strftime(
            "%Y-%m-%d %H:%M:%S", time.localtime(path.stat().st_mtime)
        ),
        "has_trigger": False,
        "has_grip": False,
        "has_menu": False,
        "grip_mode": None,
        "stale": False,
        "empty": False,
        "parse_error": "",
    }
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        info["parse_error"] = f"{type(exc).__name__}: {exc}"
        return info
    sources = (
        data.get("bindings", {})
        .get("/actions/teleop", {})
        .get("sources", [])
    )
    if not sources:
        info["empty"] = True
        return info
    for s in sources:
        path_attr = s.get("path", "")
        mode = s.get("mode", "")
        if path_attr.endswith("/input/trigger"):
            info["has_trigger"] = True
        elif path_attr.endswith("/input/grip"):
            info["has_grip"] = True
            info["grip_mode"] = mode
            if mode == "force_sensor":
                # Pre-9.32 stale binding -- force_sensor mode doesn't exist
                # on modern pico_controller / oculus_touch profiles, so this
                # entry will return bActive=False forever.
                info["stale"] = True
        elif path_attr.endswith("/input/application_menu"):
            info["has_menu"] = True
    if not (info["has_trigger"] and info["has_grip"]):
        info["stale"] = True
    return info


# ── Mutation ────────────────────────────────────────────────────────────
def clear_binding(path: Path, dry_run: bool = False) -> Dict[str, Any]:
    """Move ``path`` to ``path.bak`` (or just print under --dry-run)."""
    backup = path.with_suffix(path.suffix + ".bak")
    if dry_run:
        return {"action": "would-clear", "path": str(path), "backup": str(backup)}
    # If a previous backup exists, rotate it.
    if backup.exists():
        try:
            backup.unlink()
        except Exception:  # noqa: BLE001
            pass
    try:
        shutil.move(str(path), str(backup))
        return {"action": "cleared", "path": str(path), "backup": str(backup)}
    except Exception as exc:  # noqa: BLE001
        return {
            "action": "failed",
            "path": str(path),
            "error": f"{type(exc).__name__}: {exc}",
        }


def write_default_binding(
    target_dir: Path, controller_type: str, dry_run: bool = False
) -> Dict[str, Any]:
    """Write a fresh Personal Binding for ``controller_type`` from our
    shipped ``bindings_pico.json``.

    SteamVR's Personal Binding format is the same as the default binding
    file but with a top-level ``name`` / ``description`` / ``category``.
    """
    if not BINDINGS_PICO_PATH.exists():
        return {
            "action": "failed",
            "error": f"shipped bindings file not found: {BINDINGS_PICO_PATH}",
        }
    try:
        default = json.loads(BINDINGS_PICO_PATH.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {"action": "failed", "error": f"parse {BINDINGS_PICO_PATH}: {exc}"}
    personal = {
        "name": "UST Teleop GR1T2 Gripper Default (forced by repair_binding)",
        "description": (
            "Auto-generated by repair_binding.py to override an empty / stale "
            "Personal Binding.  Restart SteamVR after writing this file so "
            "the Action API picks it up."
        ),
        "category": "vrgame",
        "options": {},
        "controller_type": controller_type,
        "bindings": default.get("bindings", {}),
    }
    target_dir.mkdir(parents=True, exist_ok=True)
    out = target_dir / f"binding_{APP_KEY}_{controller_type}.json"
    if dry_run:
        return {
            "action": "would-write",
            "path": str(out),
            "controller_type": controller_type,
        }
    try:
        out.write_text(
            json.dumps(personal, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return {
            "action": "wrote",
            "path": str(out),
            "controller_type": controller_type,
        }
    except Exception as exc:  # noqa: BLE001
        return {
            "action": "failed",
            "path": str(out),
            "error": f"{type(exc).__name__}: {exc}",
        }


# ── Reporters ───────────────────────────────────────────────────────────
def render_inspect(info: Dict[str, Any]) -> str:
    p = info["path"]
    if info["parse_error"]:
        return f"  {p}\n      [PARSE ERROR] {info['parse_error']}"
    parts = []
    parts.append(f"  {p}")
    parts.append(
        f"      size={info['size']}B  mtime={info['mtime']}  "
        f"trigger={'y' if info['has_trigger'] else 'n'}  "
        f"grip={'y' if info['has_grip'] else 'n'} "
        f"(mode={info['grip_mode']!r})  "
        f"menu={'y' if info['has_menu'] else 'n'}"
    )
    if info["empty"]:
        parts.append("      VERDICT: EMPTY -- shadows the default binding, "
                     "shows bActive=False forever.  Recommend: --clear")
    elif info["stale"]:
        parts.append(
            "      VERDICT: STALE / INCOMPLETE -- missing trigger or grip "
            "Pull source, or grip uses force_sensor (pre-9.32).  "
            "Recommend: --clear"
        )
    else:
        parts.append("      VERDICT: OK -- has trigger + grip Pull mappings.  "
                     "If diagnose still shows bActive=False, this binding may "
                     "not be the active one (SteamVR is using a different "
                     "controller_type).  Try --clear and re-run.")
    return "\n".join(parts)


# ── Main ────────────────────────────────────────────────────────────────
def main() -> int:
    _say("[repair_binding] starting...")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--list", action="store_true",
        help="(default) Inspect existing Personal Binding files and print a "
             "diagnosis without modifying anything.",
    )
    parser.add_argument(
        "--clear", action="store_true",
        help="Move every matching Personal Binding to *.bak.  After a "
             "SteamVR restart, our app's default binding will auto-apply.",
    )
    parser.add_argument(
        "--write-default", action="store_true",
        help="After --clear (or standalone), write a fresh Personal Binding "
             "file built from config/openvr_actions/bindings_pico.json for "
             "each --controller-type.  Forces the binding to be present "
             "even before SteamVR restart.",
    )
    parser.add_argument(
        "--controller-type", action="append", default=None,
        help="Which controller_type values to write defaults for.  Can be "
             "passed multiple times.  Default: pico_controller.",
    )
    parser.add_argument(
        "--target",
        choices=("local", "steam", "both"),
        default="local",
        help="Where to write the fresh Personal Binding.  'local' = "
             "%%LOCALAPPDATA%%\\openvr\\input\\ (default, SteamVR's primary "
             "lookup), 'steam' = <Steam>/config/steamvr_input/, 'both'.",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print what would happen without modifying any files.",
    )
    args = parser.parse_args()

    # Default to --list when no action flag is set.
    if not (args.list or args.clear or args.write_default):
        args.list = True

    target_types = args.controller_type or ["pico_controller"]

    local_dir = _find_local_openvr_input()
    steam_dir = _find_steamvr_input_dir()
    _say(f"[repair_binding] paths:")
    _say(f"  LOCALAPPDATA openvr/input  = {local_dir}")
    _say(f"  Steam config/steamvr_input = {steam_dir}")
    _say(f"  app_key                    = {APP_KEY}")
    _say(f"  shipped bindings           = {BINDINGS_PICO_PATH}")
    _say("")

    files = find_personal_binding_files()
    # 9.41: even with NO Personal Binding file on disk, SteamVR can still
    # have "Active Controller Binding = Custom" recorded in
    # steamvr.vrsettings, which produces the same bActive=False symptom.
    # Surface that case explicitly.
    appconfig = inspect_steamvr_appconfig()
    if appconfig["found"]:
        _say("[repair_binding][!!] SteamVR appconfig contains 'custom' "
             "near our app_key.")
        for h in appconfig["hints"]:
            _say(h)
        _say("")
        _say("  IMMEDIATE FIX (no script needed):")
        _say("    1. Open SteamVR > Settings > Controllers >")
        _say("       Manage Controller Bindings.")
        _say("    2. Dropdown -> select 'UST Teleop GR1T2 Gripper'.")
        _say("    3. Active Controller Binding row: click 'Default' button.")
        _say("       (Currently 'Custom' is selected -- that empty Custom")
        _say("       slot is shadowing our default binding.)")
        _say("    4. Close the dialog.  Re-run diagnose_controller_raw.")
        _say("")
        _say("  ALTERNATIVE: re-run this script with --clear and RESTART SteamVR.")
        _say("")

    if not files:
        _say("[repair_binding] No existing Personal Binding *files* for this "
             "app_key.")
        _say(f"  SteamVR will auto-apply the default binding from")
        _say(f"  {BINDINGS_PICO_PATH}")
        _say("  on next session -- UNLESS the SteamVR appconfig still has")
        _say("  Active Controller Binding = Custom selected (see [!!] above).")
        _say("  Run with --write-default to force-write a Personal Binding.")
    else:
        _say(f"[repair_binding] found {len(files)} Personal Binding file(s) "
             f"for app_key={APP_KEY!r}:")
        for f in files:
            info = inspect_binding_file(f)
            _say(render_inspect(info))
        _say("")

    # ---- --clear -----------------------------------------------------
    if args.clear:
        if not files:
            _say("[repair_binding] --clear: no Personal Binding files to clear.")
        else:
            _say("[repair_binding] --clear: moving each file to *.bak ...")
            for f in files:
                result = clear_binding(f, dry_run=args.dry_run)
                if result["action"] == "cleared":
                    _say(f"  CLEARED  {result['path']}  -> {result['backup']}")
                elif result["action"] == "would-clear":
                    _say(f"  (dry)    {result['path']}  -> {result['backup']}")
                else:
                    _say(f"  FAILED   {result['path']}  ({result.get('error')})")
            _say("")
            _say("[repair_binding] After RESTART SteamVR our default binding")
            _say("  should auto-apply.  Steps:")
            _say("    1. Right-click SteamVR systray icon -> Quit SteamVR.")
            _say("    2. Wait 5s for vrserver.exe to exit fully.")
            _say("    3. Steam > Library > Tools > SteamVR -> Launch.")
            _say("    4. Re-run diagnose_controller_raw.  Expect (a1) flags.")

    # ---- --write-default --------------------------------------------
    if args.write_default:
        _say(
            f"[repair_binding] --write-default: writing fresh Personal "
            f"Binding(s) for controller_type(s) {target_types}..."
        )
        targets: List[Path] = []
        if args.target in ("local", "both") and local_dir is not None:
            targets.append(local_dir)
        if args.target in ("steam", "both") and steam_dir is not None:
            targets.append(steam_dir)
        if not targets:
            _say(
                "[repair_binding][FAIL] no writable target directory found.  "
                "Is SteamVR installed?"
            )
            return 1
        for tdir in targets:
            for ct in target_types:
                result = write_default_binding(tdir, ct, dry_run=args.dry_run)
                if result["action"] == "wrote":
                    _say(f"  WROTE    {result['path']}")
                elif result["action"] == "would-write":
                    _say(f"  (dry)    {result['path']}")
                else:
                    _say(
                        f"  FAILED   {result.get('path', tdir)}  "
                        f"({result.get('error')})"
                    )
        _say("")
        _say("[repair_binding] After writing, RESTART SteamVR for the new")
        _say("Personal Binding to be picked up by the Action API.")

    _say("[repair_binding] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
