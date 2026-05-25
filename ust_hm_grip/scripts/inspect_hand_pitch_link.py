"""Inspect what's actually inside ``{side}_hand_pitch_link`` in the built USD.

The user sees an "object inside the gripper" — likely the wrist's residual
hand visual mesh (the GR1T2 stock USD has the back-of-hand geometry attached
to the hand_pitch_link visuals).  We want to know:
  1. Does ``hand_pitch_link`` have visual / mesh children?
  2. Does it have collision children?
  3. Is it a rigid body / has PhysicsRigidBodyAPI (i.e., physics-critical)?
  4. Is it a parent of joints (i.e., kinematic chain endpoint)?

If it's purely a visual hand mesh, we can strip the visuals safely; the
articulation chain (wrist_yaw → wrist_roll → wrist_pitch → hand_pitch_link)
keeps the BODY intact, and the FixedJoint to the Robotiq base_link still
anchors the gripper to the correct articulation node.
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
        from pxr import Usd, Sdf, UsdPhysics  # noqa

        built = str(_REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd")
        stage = Usd.Stage.Open(built)
        if not stage:
            sys.stdout.write("FAIL — cannot open built USD\n")
            return 1
        root = stage.GetDefaultPrim().GetPath()

        for side in ("left", "right"):
            wrist_path = root.AppendChild(f"{side}_hand_pitch_link")
            wrist = stage.GetPrimAtPath(wrist_path)
            sys.stdout.write(f"\n========== {side}_hand_pitch_link ==========\n")
            if not wrist or not wrist.IsValid():
                sys.stdout.write("  (does not exist)\n")
                continue
            apis = list(wrist.GetAppliedSchemas())
            sys.stdout.write(f"  applied APIs   : {apis}\n")
            sys.stdout.write(f"  has Rigid Body : {UsdPhysics.RigidBodyAPI(wrist) is not None}\n")
            sys.stdout.write(f"  has Mass API   : {UsdPhysics.MassAPI(wrist) is not None}\n")
            sys.stdout.write(f"  has Collision  : {UsdPhysics.CollisionAPI(wrist) is not None}\n")
            sys.stdout.write(f"  has Articulation root : {UsdPhysics.ArticulationRootAPI(wrist) is not None}\n")

            # Walk children — show every prim under hand_pitch_link
            sys.stdout.write(f"\n  children (recursive):\n")
            for prim in Usd.PrimRange(wrist):
                if prim == wrist:
                    continue
                depth = len(str(prim.GetPath()).strip("/").split("/")) - \
                        len(str(wrist_path).strip("/").split("/"))
                indent = "    " + ("  " * depth)
                name = prim.GetName()
                tn = prim.GetTypeName() or "(none)"
                apis_c = list(prim.GetAppliedSchemas())
                sys.stdout.write(f"{indent}{name} : {tn}")
                if apis_c:
                    sys.stdout.write(f"  [APIs: {', '.join(apis_c)}]")
                sys.stdout.write("\n")

            # Find any joints that reference hand_pitch_link as body
            sys.stdout.write(f"\n  joints that reference {side}_hand_pitch_link as body:\n")
            wrist_path_str = str(wrist_path)
            for prim in stage.Traverse():
                if "Joint" not in str(prim.GetTypeName()):
                    continue
                for rel_name in ("physics:body0", "physics:body1"):
                    rel = prim.GetRelationship(rel_name)
                    if rel and rel.IsValid():
                        for tgt in rel.GetTargets():
                            if str(tgt) == wrist_path_str:
                                sys.stdout.write(f"    {prim.GetTypeName()} {prim.GetName()} ({rel_name})\n")

            sys.stdout.flush()

        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
