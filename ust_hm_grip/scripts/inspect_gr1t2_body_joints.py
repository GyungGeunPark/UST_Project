"""Inspect GR1T2 USD for head/neck/elbow/leg joint + link names.

Phase D needs: head pose target, elbow position targets, optionally pelvis.
This script enumerates body joints + links so we can pick the right names
for Pink IK FrameTask / PositionTask + pink_controlled_joint_names.
"""
from __future__ import annotations
import argparse, sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))


def _boot_isaac_sim():
    from isaaclab.app import AppLauncher
    p = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(p)
    b, r = p.parse_known_args()
    b.headless = True
    AppLauncher(b).app
    sys.argv = [sys.argv[0]] + r


def main() -> int:
    _boot_isaac_sim()
    from pxr import Usd
    usd_path = str(
        _REPO_ROOT / "ust_ws/ust_hm_grip/isaac_file/GR1T2_with_robotiq.usd"
    )
    stage = Usd.Stage.Open(usd_path)
    print(f"[inspect] {usd_path}\n")

    joints = []
    links = []
    for prim in stage.Traverse():
        ty = prim.GetTypeName()
        name = prim.GetName()
        if ty in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsFixedJoint"):
            joints.append((name, ty, str(prim.GetPath())))
        elif "Xform" in str(ty) and prim.HasAttribute("physics:rigidBodyEnabled"):
            links.append((name, str(prim.GetPath())))
        elif name.endswith("_link"):
            links.append((name, str(prim.GetPath())))

    print("=== Joints (filtered: head/neck/elbow/wrist/hip/knee/ankle/waist/spine) ===")
    for n, ty, p in joints:
        if any(k in n for k in [
            "head", "neck", "elbow", "wrist", "hip", "knee", "ankle",
            "waist", "spine", "shoulder",
        ]):
            print(f"  {ty:25s} {n}  ← {p}")
    print()

    print("=== Links (filtered: head/neck/elbow/wrist/hip/knee/ankle/waist/spine/torso) ===")
    seen = set()
    for n, p in links:
        if any(k in n for k in [
            "head", "neck", "elbow", "wrist", "hip", "knee", "ankle",
            "waist", "spine", "torso", "shoulder",
        ]):
            if n in seen:
                continue
            seen.add(n)
            print(f"  link {n}  ← {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
