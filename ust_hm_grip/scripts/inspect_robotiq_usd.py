"""Inspect the Robotiq 2F-85 stock USD downloaded from Isaac Sim 5.1 S3.

Prints the prim tree, joint table, MimicJointAPI status, and any pending
references / payloads so we can write ``build_robotiq_usd.py`` against the
real prim paths rather than guessing.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))


def _boot_isaac_sim() -> "object":
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
        import argparse as _ap

        parser = _ap.ArgumentParser()
        parser.add_argument(
            "--usd",
            default=str(
                _REPO_ROOT
                / "ust_ws"
                / "ust_hm_grip"
                / "isaac_file"
                / "robotiq"
                / "Robotiq_2F_85_edit.usd"
            ),
        )
        args = parser.parse_args()

        from pxr import Usd, UsdGeom, UsdPhysics, Sdf  # noqa

        print(f"[inspect_robotiq] opening {args.usd!r}", flush=True)
        stage = Usd.Stage.Open(args.usd)
        if not stage:
            print("FAIL — cannot open", flush=True)
            return 1

        default_prim = stage.GetDefaultPrim()
        print(f"[inspect_robotiq] default prim: {default_prim.GetPath() if default_prim else None}", flush=True)

        # --- prim tree ---
        print("\n=== Prim tree (path : typeName : APIs) ===", flush=True)
        for prim in stage.Traverse():
            apis = ", ".join(s for s in prim.GetAppliedSchemas())
            if apis:
                apis = " [" + apis + "]"
            print(f"  {prim.GetPath()} : {prim.GetTypeName() or '(none)'}{apis}", flush=True)

        # --- payload / reference targets (read-only; do NOT mutate) ---
        print("\n=== Payloads / References ===", flush=True)
        for prim in stage.Traverse():
            try:
                stack = prim.GetPrimStack()
            except Exception:
                continue
            for spec in stack:
                try:
                    p_items = spec.payloadList.GetAddedOrExplicitItems()
                    r_items = spec.referenceList.GetAddedOrExplicitItems()
                except Exception:
                    continue
                for item in p_items:
                    print(f"  payload  on {prim.GetPath()} -> {item.assetPath} @ {item.primPath}")
                for item in r_items:
                    print(f"  reference on {prim.GetPath()} -> {item.assetPath} @ {item.primPath}")

        # --- variant sets ---
        print("\n=== VariantSets ===")
        for prim in stage.Traverse():
            vs_names = prim.GetVariantSets().GetNames()
            if vs_names:
                for name in vs_names:
                    vs = prim.GetVariantSet(name)
                    cur = vs.GetVariantSelection()
                    opts = vs.GetVariantNames()
                    print(f"  {prim.GetPath()} : {name} = {cur!r}  (options: {opts})")

        # --- joints ---
        print("\n=== Joints (type : path : body0 -> body1 : mimic?) ===")
        for prim in stage.Traverse():
            tn = prim.GetTypeName()
            if "Joint" in str(tn):
                body0 = prim.GetRelationship("physics:body0").GetTargets() if prim.GetRelationship("physics:body0") else []
                body1 = prim.GetRelationship("physics:body1").GetTargets() if prim.GetRelationship("physics:body1") else []
                mimic_apis = [s for s in prim.GetAppliedSchemas() if "Mimic" in s]
                mimic = f"MimicJoint={mimic_apis}" if mimic_apis else "no-mimic"
                print(f"  {tn} : {prim.GetPath()} : {list(body0)} -> {list(body1)} : {mimic}")
                # mimic detail
                for attr_name in (
                    "physxMimicJoint:referenceJoint",
                    "physxMimicJoint:gearing",
                    "physxMimicJoint:offset",
                ):
                    attr = prim.GetAttribute(attr_name)
                    if attr and attr.IsValid() and attr.HasAuthoredValue():
                        val = attr.Get()
                        # Resolve relationships printed as paths if applicable
                        print(f"      {attr_name} = {val!r}")
                # also try relationship form for referenceJoint
                rel = prim.GetRelationship("physxMimicJoint:referenceJoint")
                if rel and rel.GetTargets():
                    print(f"      physxMimicJoint:referenceJoint -> {list(rel.GetTargets())!r}")

        # --- ArticulationRootAPI ---
        print("\n=== ArticulationRootAPI on prims ===")
        for prim in stage.Traverse():
            if UsdPhysics.ArticulationRootAPI(prim):
                print(f"  {prim.GetPath()}")

        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
