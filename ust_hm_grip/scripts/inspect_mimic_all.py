"""Dump ALL physxMimicJoint:* properties of every Robotiq follower joint."""
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
    p = str(_REPO_ROOT / "ust_ws/ust_hm_grip/isaac_file/GR1T2_with_robotiq.usd")
    stage = Usd.Stage.Open(p)
    target_joints = [
        "left_finger_joint", "right_finger_joint",
        "left_right_outer_knuckle_joint",
        "left_right_inner_finger_joint",
        "left_right_inner_finger_knuckle_joint",
        "left_left_inner_finger_knuckle_joint",
        "left_left_inner_finger_joint",
    ]
    for prim in stage.Traverse():
        if prim.GetName() not in target_joints:
            continue
        print(f"\n=== {prim.GetName()} ===")
        for prop in prim.GetProperties():
            n = prop.GetName()
            if "mimic" in n.lower() or "drive:angular" in n.lower():
                if hasattr(prop, "GetTargets"):
                    tgts = prop.GetTargets()
                    print(f"  REL  {n:60s} → {[str(t) for t in tgts]}")
                else:
                    val = prop.Get() if (prop.HasAuthoredValue() if hasattr(prop, 'HasAuthoredValue') else True) else "(default)"
                    print(f"  ATTR {n:60s} = {val}")
        # All applied schemas
        for s in prim.GetAppliedSchemas():
            print(f"  SCHEMA: {s}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
