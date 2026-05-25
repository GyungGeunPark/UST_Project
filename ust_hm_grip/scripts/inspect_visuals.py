"""Dump the visual prim subtree under each Robotiq base_link in the
built USD AND under the Robotiq stock USD for comparison.

We want to see exactly what the visuals prim contains — references,
inlined meshes, child prims, etc.  The user's screenshot shows the
visuals prim has a child named ``left_thigh_r`` (a GR1T2 thigh roll
mesh, not a Robotiq mesh), so something in the Sdf.CopySpec /
Stage.Flatten chain is grafting GR1T2 meshes into the gripper's
visual hierarchy.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))


def _boot_isaac_sim():
    from isaaclab.app import AppLauncher
    boot_parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(boot_parser)
    boot_args, remaining = boot_parser.parse_known_args()
    boot_args.headless = True
    app_launcher = AppLauncher(boot_args)
    sim_app = app_launcher.app
    sys.argv = [sys.argv[0]] + remaining
    return sim_app


def _dump_subtree(stage, root_path: str, label: str, depth_limit: int = 10):
    from pxr import Usd, Sdf  # noqa
    sys.stdout.write(f"\n========== {label} ==========\n")
    sys.stdout.write(f"  root: {root_path}\n")
    root_prim = stage.GetPrimAtPath(root_path)
    if not (root_prim and root_prim.IsValid()):
        sys.stdout.write(f"  MISSING — prim does not exist\n")
        sys.stdout.flush()
        return
    for prim in Usd.PrimRange(root_prim):
        depth = len(str(prim.GetPath()).strip("/").split("/")) - \
                len(str(root_path).strip("/").split("/"))
        indent = "  " + ("  " * depth)
        name = prim.GetName()
        tn = prim.GetTypeName() or "(no-type)"
        apis = list(prim.GetAppliedSchemas())
        sys.stdout.write(f"{indent}{name}  : {tn}")
        if apis:
            sys.stdout.write(f"  [APIs: {', '.join(apis)}]")
        sys.stdout.write("\n")

        # Show references / payloads / inherits
        try:
            stack = prim.GetPrimStack()
        except Exception:
            stack = []
        for spec in stack:
            try:
                refs = list(spec.referenceList.GetAddedOrExplicitItems())
                pays = list(spec.payloadList.GetAddedOrExplicitItems())
                inhs = list(spec.inheritPathList.GetAddedOrExplicitItems())
            except Exception:
                refs = pays = inhs = []
            for r in refs:
                sys.stdout.write(f"{indent}    [ref] -> {r.assetPath!r} @ {r.primPath}\n")
            for p in pays:
                sys.stdout.write(f"{indent}    [payload] -> {p.assetPath!r} @ {p.primPath}\n")
            for inh in inhs:
                sys.stdout.write(f"{indent}    [inherit] -> {inh}\n")
        sys.stdout.flush()


def main() -> int:
    sim_app = _boot_isaac_sim()
    try:
        from pxr import Usd, Sdf  # noqa
        built = str(_REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd")
        stock = str(_REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "robotiq" / "Robotiq_2F_85_edit.usd")

        # --- BUILT USD ---
        built_stage = Usd.Stage.Open(built)
        if built_stage:
            root = built_stage.GetDefaultPrim().GetPath()
            for side in ("left", "right"):
                _dump_subtree(
                    built_stage,
                    str(root.AppendChild(f"{side}_robotiq_arg2f_85").AppendChild("base_link").AppendChild("visuals")),
                    f"BUILT — {side} gripper base_link/visuals",
                )

        # --- STOCK USD ---
        stock_stage = Usd.Stage.Open(stock)
        if stock_stage:
            _dump_subtree(
                stock_stage,
                "/Robotiq_2F_85/Robotiq_2F_85/base_link/visuals",
                "STOCK — Robotiq_2F_85/Robotiq_2F_85/base_link/visuals",
            )
            # Also look at the /Meshes scope
            sys.stdout.write("\n========== STOCK /Meshes scope (top level) ==========\n")
            meshes = stock_stage.GetPrimAtPath("/Meshes")
            if meshes:
                for child in meshes.GetChildren():
                    sys.stdout.write(f"  /Meshes/{child.GetName()}  : {child.GetTypeName() or '(no-type)'}\n")
                sys.stdout.flush()
            else:
                sys.stdout.write("  no /Meshes top-level prim\n")
                sys.stdout.flush()

        # Also check what's at /Meshes in the BUILT stage
        sys.stdout.write("\n========== BUILT /Meshes (top level) ==========\n")
        if built_stage:
            meshes = built_stage.GetPrimAtPath("/Meshes")
            if meshes:
                for child in meshes.GetChildren():
                    sys.stdout.write(f"  /Meshes/{child.GetName()}\n")
            else:
                sys.stdout.write("  no /Meshes prim in built USD\n")

        # Check what's at /GR1T2.../left_thigh_roll_link (the suspected mesh source)
        sys.stdout.write("\n========== BUILT — left_thigh_roll_link ==========\n")
        if built_stage:
            root = built_stage.GetDefaultPrim().GetPath()
            _dump_subtree(built_stage, str(root.AppendChild("left_thigh_roll_link")),
                          "BUILT — left_thigh_roll_link subtree", depth_limit=3)

        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
