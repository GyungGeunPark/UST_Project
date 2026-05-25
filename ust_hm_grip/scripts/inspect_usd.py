"""Inspect a GR1T2 USD file's joint + link inventory.

When ``build_gripper_usd.py`` runs but the resulting USD still has zero
``*_gripper_finger_*_joint`` matches at runtime, the most likely cause is
that the build script could not find the wrist link to attach the
gripper to (it logs a WARNING and skips silently).  This helper opens
the USD and prints every joint name + every RigidBody link path so the
user can see what's actually inside.

Usage::

    ./isaaclab.bat -p ust_ws/ust_hm_grip/scripts/inspect_usd.py
    ./isaaclab.bat -p ust_ws/ust_hm_grip/scripts/inspect_usd.py \\
        --usd "C:/develop/IsaacLab/ust_ws/ust_hm_grip/isaac_file/GR1T2_with_gripper.usd"
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def _boot_isaac_sim():
    """Boot Isaac Sim headless so pxr / UsdPhysics schemas register."""
    from isaaclab.app import AppLauncher  # type: ignore
    boot_parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(boot_parser)
    boot_args, remaining = boot_parser.parse_known_args()
    boot_args.headless = True
    app_launcher = AppLauncher(boot_args)
    sys.argv = [sys.argv[0]] + remaining
    return app_launcher.app


def main() -> int:
    sim_app = _boot_isaac_sim()
    try:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--usd",
            default=str(
                Path(__file__).resolve().parents[1]
                / "isaac_file"
                / "GR1T2_with_gripper.usd"
            ),
            help="USD to inspect (default: ust_hm_grip's GR1T2_with_gripper.usd)",
        )
        args = parser.parse_args()

        from pxr import Usd, UsdPhysics  # type: ignore

        path = os.path.abspath(args.usd)
        print(f"[inspect_usd] opening {path!r}...")
        if not os.path.exists(path):
            print(f"[inspect_usd] FAIL: file does not exist.")
            return 1

        stage = Usd.Stage.Open(path)
        if stage is None:
            print(f"[inspect_usd] FAIL: USD failed to open.")
            return 1

        rigid_bodies = []
        # USD physics joint type names carry a 'Physics' prefix (and some
        # builds use 'Physx' for the PhysX-specific schema).  Accept both.
        joints = {
            "PrismaticJoint": [],
            "PhysicsPrismaticJoint": [],
            "PhysxPrismaticJoint": [],
            "RevoluteJoint": [],
            "PhysicsRevoluteJoint": [],
            "PhysxRevoluteJoint": [],
            "FixedJoint": [],
            "PhysicsFixedJoint": [],
            "PhysxFixedJoint": [],
            "SphericalJoint": [],
            "PhysicsSphericalJoint": [],
            "Joint": [],
            "PhysicsJoint": [],
        }
        wrist_candidates = []
        gripper_candidates = []
        for prim in stage.Traverse():
            name = prim.GetName()
            type_name = prim.GetTypeName()
            path_str = str(prim.GetPath())
            if (("wrist" in name.lower() or "hand" in name.lower())
                    and "link" in name.lower()):
                wrist_candidates.append(path_str)
            if "gripper" in name.lower() or "finger" in name.lower():
                gripper_candidates.append(path_str)
            if UsdPhysics.RigidBodyAPI(prim):
                rigid_bodies.append(path_str)
            if type_name in joints:
                joints[type_name].append(path_str)

        print()
        print("=" * 72)
        print(f" Wrist-related Xforms / links ({len(wrist_candidates)}):")
        print("=" * 72)
        for p in wrist_candidates:
            print(f"  {p}")
        if not wrist_candidates:
            print("  (none -- build_gripper_usd.py cannot attach gripper)")

        print()
        print("=" * 72)
        print(f" Gripper / finger-related prims ({len(gripper_candidates)}):")
        print("=" * 72)
        for p in gripper_candidates:
            print(f"  {p}")
        if not gripper_candidates:
            print("  (none -- gripper attachment failed silently)")

        print()
        print("=" * 72)
        print(f" Rigid body links (first 30 of {len(rigid_bodies)}):")
        print("=" * 72)
        for p in rigid_bodies[:30]:
            print(f"  {p}")
        if len(rigid_bodies) > 30:
            print(f"  ... and {len(rigid_bodies) - 30} more")

        print()
        print("=" * 72)
        print(" Joints by type:")
        print("=" * 72)
        for jtype, paths in joints.items():
            if not paths:
                continue
            print(f"  {jtype}: {len(paths)} joints")
            for p in paths[:20]:
                print(f"    {p}")
            if len(paths) > 20:
                print(f"    ... and {len(paths) - 20} more")

        # Check for the specific gripper joints the env_cfg expects.
        # 9.45 follow-up: scan ALL prims (any type), not just those whose
        # GetTypeName matches a hard-coded 'Joint' alias.  USD physics
        # joint types carry the 'Physics'/'Physx' schema prefix, so a
        # simple substring filter on the prim path is more robust.
        expected_gripper_joints = [
            "left_gripper_finger_left_joint",
            "left_gripper_finger_right_joint",
            "right_gripper_finger_left_joint",
            "right_gripper_finger_right_joint",
        ]
        all_joint_names = []
        for prim in stage.Traverse():
            n = prim.GetName()
            tn = prim.GetTypeName()
            # Anything that looks like a joint -- either by name suffix
            # or by Physics/Physx schema type.
            if n.endswith("_joint") or "Joint" in tn:
                all_joint_names.append(n)
        print()
        print("=" * 72)
        print(" Expected env_cfg joints presence check:")
        print("=" * 72)
        missing = []
        for j in expected_gripper_joints:
            present = j in all_joint_names
            mark = "[ OK ]" if present else "[MISS]"
            print(f"  {mark} {j}")
            if not present:
                missing.append(j)
        if missing:
            print()
            print("  -> USD is incomplete.  Re-run build_gripper_usd.py and check")
            print("     its console output for 'WARNING -- *_wrist_pitch_link not found'.")
            print("     If you see that warning, the stock GR1T2 USD uses a different")
            print("     wrist link naming convention; pass `--source` to override.")
        return 0 if not missing else 2
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
