"""Patch the SteamVR "Save Personal Binding" silent-fail bug.

Background
----------
SteamVR's controller binding editor calls
``this.m_sInteractionProfile.trim()`` in the minified JS that backs the
"Manage Controller Bindings" web UI.  When ``m_sInteractionProfile`` is
``null`` / ``undefined`` (happens transiently when the editor first
loads our app's binding page), the unguarded ``.trim()`` throws a
``TypeError``, the save flow stops, and the UI hangs on a blank page --
the binding never commits.  This is a 6+ year-old, well-documented
SteamVR bug:

  * https://steamcommunity.com/app/250820/discussions/3/596271068722815083/
  * https://github.com/ValveSoftware/openvr/issues/1659
  * https://steamcommunity.com/app/250820/discussions/3/4847652627856593674/

In our 10th session that bug manifested as ``bActive=False`` forever
on every Action API channel of ``ust.teleop.gr1t2_gripper``, blocking
the entire controller-input pipeline (research/45 §1, §5).

This script patches both minified files where the bug lives:

  * ``...\SteamVR\resources\webinterface\dashboard\controllerbindingui.js``
  * ``...\SteamVR\resources\webinterface\dashboard\chunk~bd288592d.js``

The patch is a surgical string replacement:

  -  this.m_sInteractionProfile.trim()
  +  (this.m_sInteractionProfile||"").trim()

It is **idempotent** (re-running is a no-op once patched), **reversible**
via ``--revert`` (restores from timestamped backup), and **safe against
checksums** (SteamVR does not appear to integrity-check the JS bundle).

Caveats
-------
* Every SteamVR update will overwrite these files.  After updating
  SteamVR, re-run this script.  Consider adding a startup hook in
  ``run_teleop.py`` if you want it auto-applied.
* If Steam runs in offline mode or as administrator, the workaround
  may behave differently; see the Steam Community thread referenced
  above for adjacent fixes.

Usage
-----

    # apply (default)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.patch_steamvr_binding_bug

    # check only
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.patch_steamvr_binding_bug --check

    # revert (restore latest backup)
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.patch_steamvr_binding_bug --revert

    # custom SteamVR path
    python -X utf8 -m ust_ws.ust_hm_grip.scripts.patch_steamvr_binding_bug \
        --steamvr "D:\\Steam\\steamapps\\common\\SteamVR"
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from pathlib import Path
from typing import List, Tuple


# Default SteamVR install location.  --steamvr overrides.
DEFAULT_STEAMVR = r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR"

# Files known to contain the bug.  These are minified webpack chunks;
# their exact filenames may change after a SteamVR update -- if so,
# this script falls back to scanning the dashboard directory.
KNOWN_TARGETS = [
    "resources/webinterface/dashboard/controllerbindingui.js",
    "resources/webinterface/dashboard/chunk~bd288592d.js",
]

# Patch contract.
ORIGINAL = "this.m_sInteractionProfile.trim()"
PATCHED  = '(this.m_sInteractionProfile||"").trim()'

# Marker string that uniquely identifies a file we have patched.  This
# string must NOT contain the literal ``ORIGINAL`` (otherwise ``_is_patched``
# would always return False because of the banner itself, and every re-run
# would re-patch + duplicate the banner).  We therefore describe the
# patch in plain English rather than embedding the JS snippet.
PATCH_MARKER = "ust_hm_grip_steamvr_binding_bug_patch_marker_v1"
PATCH_BANNER = (
    f"/* {PATCH_MARKER} -- null-safe trim on m_sInteractionProfile. "
    f"See ust_hm_grip/scripts/patch_steamvr_binding_bug.py */ "
)


def _scan_for_bug(steamvr_root: Path) -> List[Path]:
    """Return absolute paths to .js files under SteamVR/resources/webinterface/dashboard/
    that contain the buggy pattern.  Falls back from KNOWN_TARGETS when
    SteamVR rebundles its chunks."""
    dash = steamvr_root / "resources" / "webinterface" / "dashboard"
    hits: List[Path] = []
    if not dash.exists():
        print(f"[FATAL] dashboard dir not found: {dash}")
        return hits
    for js in dash.glob("*.js"):
        try:
            text = js.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if ORIGINAL in text:
            hits.append(js)
    return hits


def _is_patched(text: str) -> bool:
    """A file counts as patched when our marker banner is present AND
    no raw bug pattern remains (the banner itself contains no JS code,
    only the marker string -- see PATCH_MARKER docstring)."""
    return PATCH_MARKER in text and ORIGINAL not in text


def _stamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _backup(path: Path) -> Path:
    """Create a timestamped backup next to the file.  Returns backup path."""
    bak = path.with_suffix(path.suffix + f".bak.ust.{_stamp()}")
    shutil.copy2(path, bak)
    return bak


def _apply_one(path: Path, dry_run: bool) -> Tuple[bool, str]:
    """Patch one file.  Returns ``(changed, message)``."""
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except OSError as exc:
        return False, f"[FAIL] read {path}: {exc}"

    if _is_patched(text):
        return False, f"[OK ] already patched: {path.name}"

    if ORIGINAL not in text:
        return False, f"[--] no bug pattern in: {path.name} (SteamVR rebundled?)"

    n = text.count(ORIGINAL)
    new_text = text.replace(ORIGINAL, PATCHED)
    # Prepend a single banner comment so the patch is greppable.  Only
    # do this once per file (idempotent).  The marker uniquely identifies
    # our patch and never contains the raw bug string.
    if PATCH_MARKER not in new_text:
        new_text = PATCH_BANNER + new_text

    if dry_run:
        return True, f"[DRY] would patch {n} occurrence(s) in: {path.name}"

    bak = _backup(path)
    try:
        path.write_text(new_text, encoding="utf-8")
    except OSError as exc:
        # restore from backup if write fails partway
        try:
            shutil.copy2(bak, path)
        except OSError:
            pass
        return False, f"[FAIL] write {path}: {exc} (restored backup {bak.name})"
    return True, f"[OK ] patched {n} occurrence(s) in {path.name} (backup -> {bak.name})"


def _latest_backup(path: Path) -> Path | None:
    """Find the most recent ust-style backup next to ``path``."""
    bks = sorted(
        path.parent.glob(path.name + ".bak.ust.*"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    return bks[0] if bks else None


def _revert_one(path: Path) -> Tuple[bool, str]:
    """Restore ``path`` from its latest ust-style backup."""
    bak = _latest_backup(path)
    if bak is None:
        return False, f"[--] no backup found for: {path.name}"
    try:
        shutil.copy2(bak, path)
    except OSError as exc:
        return False, f"[FAIL] revert {path}: {exc}"
    return True, f"[OK ] reverted {path.name} from {bak.name}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--steamvr", default=DEFAULT_STEAMVR,
                    help="SteamVR install root (default: %(default)s)")
    ap.add_argument("--check", action="store_true",
                    help="Only report status, do not modify files.")
    ap.add_argument("--revert", action="store_true",
                    help="Restore from the most recent backup.")
    args = ap.parse_args()

    steamvr_root = Path(args.steamvr)
    if not steamvr_root.exists():
        print(f"[FATAL] SteamVR install not found at: {steamvr_root}")
        print("        Use --steamvr to point at a different install.")
        return 2

    print(f"[info] SteamVR root: {steamvr_root}")

    # First try the known targets; if both contain neither the bug nor
    # the patched form, fall back to a scan.
    targets: List[Path] = []
    for rel in KNOWN_TARGETS:
        p = steamvr_root / rel
        if p.exists():
            targets.append(p)
        else:
            print(f"[warn] known target missing: {rel}")
    if not targets:
        print("[info] known targets all missing -- scanning dashboard dir...")
        targets = _scan_for_bug(steamvr_root)

    # When applying (not check / revert), broaden to include files that
    # are already patched too so we report them as OK.
    if args.check or not args.revert:
        scan = _scan_for_bug(steamvr_root)
        for s in scan:
            if s not in targets:
                targets.append(s)

    if not targets:
        print("[result] No candidate JS files found.  SteamVR may have")
        print("         rebundled.  Run 'grep -rln m_sInteractionProfile' under")
        print(f"         {steamvr_root} to find the new file, then update")
        print("         KNOWN_TARGETS in this script.")
        return 1

    print(f"[info] candidate files: {len(targets)}")
    for t in targets:
        print(f"         {t}")
    print()

    any_changed = False
    if args.revert:
        for t in targets:
            ok, msg = _revert_one(t)
            print(f"  {msg}")
            any_changed |= ok
    elif args.check:
        for t in targets:
            try:
                text = t.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                print(f"  [FAIL] read {t.name}")
                continue
            if _is_patched(text):
                print(f"  [OK ] already patched: {t.name}")
            elif ORIGINAL in text:
                n = text.count(ORIGINAL)
                print(f"  [BUG] {n} occurrence(s) of raw bug pattern in: {t.name}")
            else:
                print(f"  [--] no bug pattern found in: {t.name}")
    else:
        for t in targets:
            ok, msg = _apply_one(t, dry_run=False)
            print(f"  {msg}")
            any_changed |= ok

    print()
    if args.check:
        print("[done] check complete.  Run without --check to apply the patch.")
        return 0
    if args.revert:
        print("[done] revert complete." if any_changed else
              "[done] nothing to revert (no backups found).")
        return 0

    if any_changed:
        print("[done] patch applied.")
        print("       NEXT STEPS:")
        print("         1. Quit SteamVR completely (vrserver + vrcompositor + vrmonitor).")
        print("         2. Re-launch SteamVR (PICO Connect will respawn it).")
        print("         3. Open SteamVR Settings > Controllers > Manage Controller Bindings")
        print("            -> 'UST Teleop GR1T2 Gripper' -> Default")
        print("            -> click 'Save Personal Binding' (should now succeed without a blank page).")
        print("         4. Verify via:")
        print("              python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw --seconds 8")
        print("            Expect: A_trig=0.xx(a1) / A_grip=0.xx(a1) when squeezing.")
        print()
        print("       WARNING: every SteamVR auto-update will OVERWRITE these files.")
        print("                Re-run this script after each SteamVR update.")
    else:
        print("[done] no changes needed (already patched, or pattern not found).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
