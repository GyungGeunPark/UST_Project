"""Find where the /Meshes scope is defined in the Robotiq stock USD,
and what its children look like.  Also check the flattened layer's
top-level prims to confirm what Stage.Flatten() actually produces.
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


def main() -> int:
    sim_app = _boot_isaac_sim()
    try:
        from pxr import Usd, Sdf  # noqa
        stock = str(_REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "robotiq" / "Robotiq_2F_85_edit.usd")

        stage = Usd.Stage.Open(stock)
        if not stage:
            sys.stdout.write("FAIL — cannot open stock\n")
            return 1
        sys.stdout.write(f"\n========== STOCK Robotiq stage top-level prims ==========\n")
        for prim in stage.GetPseudoRoot().GetChildren():
            sys.stdout.write(f"  {prim.GetPath()}  : {prim.GetTypeName() or '(no-type)'}\n")
        sys.stdout.flush()

        # Walk /Meshes if it exists in the COMPOSED stage
        sys.stdout.write(f"\n========== STOCK /Meshes (composed) ==========\n")
        meshes = stage.GetPrimAtPath("/Meshes")
        if meshes:
            sys.stdout.write(f"  /Meshes exists: {meshes.GetTypeName()}\n")
            for child in meshes.GetChildren():
                sys.stdout.write(f"    /Meshes/{child.GetName()}  : {child.GetTypeName() or '(no-type)'}\n")
                for grandchild in child.GetChildren():
                    sys.stdout.write(f"      .../{grandchild.GetName()}  : {grandchild.GetTypeName() or '(no-type)'}\n")
        else:
            sys.stdout.write("  /Meshes does NOT exist in composed stage\n")
        sys.stdout.flush()

        # Flatten and check top-level prims
        flat = stage.Flatten()
        sys.stdout.write(f"\n========== FLATTENED stock layer top-level prims ==========\n")
        for prim_spec in flat.rootPrims:
            sys.stdout.write(f"  /{prim_spec.name}  : {prim_spec.typeName or '(no-type)'}\n")
        sys.stdout.flush()

        # Find /Flattened_Prototype_* and show what's inside
        sys.stdout.write(f"\n========== FLATTENED layer prototypes ==========\n")
        for prim_spec in flat.rootPrims:
            if "Flattened_Prototype" in prim_spec.name:
                sys.stdout.write(f"  /{prim_spec.name}\n")
                for child_spec in prim_spec.nameChildren:
                    sys.stdout.write(f"    .../{child_spec.name}  : {child_spec.typeName or '(no-type)'}\n")
                    for gc in child_spec.nameChildren:
                        sys.stdout.write(f"      .../{gc.name}  : {gc.typeName or '(no-type)'}\n")
        sys.stdout.flush()

        # Resolve what the visuals references look like in the flattened layer
        sys.stdout.write(f"\n========== FLATTENED — Robotiq_2F_85/Robotiq_2F_85/base_link/visuals ==========\n")
        visuals_spec = flat.GetPrimAtPath("/Robotiq_2F_85/Robotiq_2F_85/base_link/visuals")
        if visuals_spec:
            sys.stdout.write(f"  spec: {visuals_spec.path}, typeName={visuals_spec.typeName}\n")
            sys.stdout.write(f"  has_children: {[c.name for c in visuals_spec.nameChildren]}\n")
            sys.stdout.write(f"  references: {visuals_spec.referenceList.GetAddedOrExplicitItems()}\n")
            sys.stdout.write(f"  instanceable: {visuals_spec.GetInfo('instanceable') if 'instanceable' in visuals_spec.ListInfoKeys() else 'unset'}\n")
        sys.stdout.flush()

        # FULL flattened layer dump for diagnosis (first 100 prim specs)
        sys.stdout.write(f"\n========== FLATTENED prims under Robotiq_2F_85/Robotiq_2F_85/ (first 30 children with refs/instanceable info) ==========\n")
        root_spec = flat.GetPrimAtPath("/Robotiq_2F_85/Robotiq_2F_85")
        if root_spec:
            count = 0
            stack = [root_spec]
            while stack and count < 60:
                spec = stack.pop(0)
                refs = spec.referenceList.GetAddedOrExplicitItems()
                inst = spec.GetInfo('instanceable') if 'instanceable' in spec.ListInfoKeys() else None
                sys.stdout.write(f"  {spec.path}  : {spec.typeName or '(no-type)'}")
                if refs:
                    sys.stdout.write(f"  refs={refs}")
                if inst is not None:
                    sys.stdout.write(f"  inst={inst}")
                sys.stdout.write("\n")
                for child in spec.nameChildren:
                    stack.append(child)
                count += 1
        sys.stdout.flush()
        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
