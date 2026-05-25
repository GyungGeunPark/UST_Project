"""Inspect lead joint physics properties to find what blocks lead close motion.

12th session: lead joint stalls at ~2-3° regardless of PD tuning (K=200/D=40
or K=800/D=40).  PD math says it should reach 99% of target in 5τ.  The fact
that it doesn't move means a physical constraint is blocking it.

Probable culprits:
- physics:lowerLimit / upperLimit too narrow
- physics:jointEnabled false
- jointDriveType wrong
- An additional FixedJoint somewhere
- physics:friction set too high
- joint axis flipped
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))


def _boot_isaac_sim():
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(parser)
    boot_args, remaining = parser.parse_known_args()
    boot_args.headless = True
    AppLauncher(boot_args).app
    sys.argv = [sys.argv[0]] + remaining


def main() -> int:
    _boot_isaac_sim()
    from pxr import Usd, UsdPhysics, PhysxSchema  # noqa

    usd_path = str(
        _REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd"
    )
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"FAIL — cannot open {usd_path}")
        return 1

    print(f"[inspect] opened {usd_path}")
    print("=" * 80)

    interesting = [
        "left_finger_joint", "right_finger_joint",
        "left_right_outer_knuckle_joint", "right_right_outer_knuckle_joint",
        "left_right_inner_finger_joint", "right_right_inner_finger_joint",
        "left_right_inner_finger_knuckle_joint",
        "left_left_inner_finger_knuckle_joint",
        "left_left_inner_finger_joint", "right_left_inner_finger_joint",
    ]

    for prim in stage.Traverse():
        name = prim.GetName()
        if name not in interesting:
            continue
        path = str(prim.GetPath())
        print(f"\n── {name}")
        print(f"   path: {path}")
        print(f"   type: {prim.GetTypeName()}")
        for attr_name in [
            "physics:lowerLimit",
            "physics:upperLimit",
            "physics:axis",
            "physics:jointEnabled",
            "physics:body0",
            "physics:body1",
            "physics:localPos0",
            "physics:localPos1",
            "physics:excludeFromArticulation",
            "drive:angular:physics:targetPosition",
            "drive:angular:physics:stiffness",
            "drive:angular:physics:damping",
            "drive:angular:physics:maxForce",
            "drive:angular:physics:driveType",
            "physxJoint:jointFriction",
            "physxJoint:armature",
            "physxJoint:maxJointVelocity",
        ]:
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid() and attr.HasAuthoredValue():
                val = attr.Get()
                print(f"   {attr_name:55s} = {val}")
        for schema in prim.GetAppliedSchemas():
            if "Mimic" in schema or "Joint" in schema or "Drive" in schema:
                print(f"   schema: {schema}")
        # body0 / body1 are relationships, not attributes
        for rel_name in ("physics:body0", "physics:body1"):
            rel = prim.GetRelationship(rel_name)
            if rel and rel.IsValid():
                targets = rel.GetTargets()
                if targets:
                    print(f"   {rel_name:55s} → {[str(t) for t in targets]}")
        # Mimic referenceJoint
        for axis in ("rotX", "rotY", "rotZ"):
            ref_rel = prim.GetRelationship(f"physxMimicJoint:{axis}:referenceJoint")
            if ref_rel and ref_rel.IsValid():
                targets = ref_rel.GetTargets()
                if targets:
                    print(f"   mimic[{axis}].refJoint     → {[str(t) for t in targets]}")
            ref_target = prim.GetAttribute(f"physxMimicJoint:{axis}:referenceJointAxis")
            gear = prim.GetAttribute(f"physxMimicJoint:{axis}:gearing")
            offset = prim.GetAttribute(f"physxMimicJoint:{axis}:offset")
            if ref_target and ref_target.IsValid() and ref_target.HasAuthoredValue():
                print(f"   mimic[{axis}].refAxis      = {ref_target.Get()}")
            if gear and gear.IsValid() and gear.HasAuthoredValue():
                print(f"   mimic[{axis}].gearing      = {gear.Get()}")
            if offset and offset.IsValid() and offset.HasAuthoredValue():
                print(f"   mimic[{axis}].offset       = {offset.Get()}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
