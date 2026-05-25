"""Verify ALL Robotiq gripper bodies have correct visual mesh subtrees
in the built USD (no GR1T2 mesh contamination from instancing collision).
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
        built = str(_REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd")
        stage = Usd.Stage.Open(built)
        if not stage:
            sys.stdout.write("FAIL — cannot open\n")
            return 1
        root = stage.GetDefaultPrim().GetPath()

        all_ok = True
        for side in ("left", "right"):
            sys.stdout.write(f"\n========== {side.upper()} GRIPPER bodies ==========\n")
            container = root.AppendChild(f"{side}_robotiq_arg2f_85")
            for body in (
                "base_link",
                "left_outer_knuckle", "right_outer_knuckle",
                "left_outer_finger", "right_outer_finger",
                "left_inner_finger", "right_inner_finger",
                "left_inner_knuckle", "right_inner_knuckle",
            ):
                body_path = container.AppendChild(body)
                visuals = stage.GetPrimAtPath(body_path.AppendChild("visuals"))
                if not (visuals and visuals.IsValid()):
                    sys.stdout.write(f"  [FAIL] {body}: no visuals prim\n")
                    all_ok = False
                    continue
                children = list(visuals.GetChildren())
                if not children:
                    # might still have references — check
                    stack = visuals.GetPrimStack()
                    refs = []
                    for spec in stack:
                        try:
                            refs.extend(spec.referenceList.GetAddedOrExplicitItems())
                        except Exception:
                            pass
                    if refs:
                        # this is the bug — visuals reference a prototype that
                        # may or may not contain Robotiq meshes
                        for r in refs:
                            sys.stdout.write(f"  [BAD ] {body}/visuals references {r.primPath} (instancing leak)\n")
                            all_ok = False
                    else:
                        sys.stdout.write(f"  [WARN] {body}/visuals has no children AND no references\n")
                else:
                    # children should be ``Defeatured_2F_85_PAD_OPEN_*step_01``
                    names = [c.GetName() for c in children]
                    is_robotiq = any("Defeatured_2F_85" in n for n in names)
                    tag = "OK  " if is_robotiq else "BAD "
                    if not is_robotiq:
                        all_ok = False
                    sys.stdout.write(f"  [{tag}] {body}/visuals children: {names}\n")
                sys.stdout.flush()

        sys.stdout.write("\n=== VERDICT: " + ("PASS — all visuals contain Robotiq meshes"
                                                 if all_ok
                                                 else "FAIL — some visuals are wrong") + " ===\n")
        return 0 if all_ok else 1
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
