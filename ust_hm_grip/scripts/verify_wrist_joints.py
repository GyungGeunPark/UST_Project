"""Verify that wrist_pitch_joint and robotiq_attach_fixed_joint still
reference {side}_hand_pitch_link after the hand-geometry strip.
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
        from pxr import Usd, UsdPhysics, Sdf  # noqa
        usd = str(_REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd")
        stage = Usd.Stage.Open(usd)
        root = stage.GetDefaultPrim().GetPath()

        # Look up specific joints we care about
        joints_to_check = [
            "/GR1T2_fourier_hand_6dof/left_wrist_pitch_joint",
            "/GR1T2_fourier_hand_6dof/left_robotiq_attach_fixed_joint",
            "/GR1T2_fourier_hand_6dof/right_wrist_pitch_joint",
            "/GR1T2_fourier_hand_6dof/right_robotiq_attach_fixed_joint",
        ]

        for jp in joints_to_check:
            prim = stage.GetPrimAtPath(jp)
            sys.stdout.write(f"\n{jp}\n")
            if not (prim and prim.IsValid()):
                # Search by name pattern across the stage
                name = jp.split("/")[-1]
                found = []
                for p in stage.Traverse():
                    if p.GetName() == name:
                        found.append(p.GetPath())
                if found:
                    sys.stdout.write(f"  not at expected path; found elsewhere: {found}\n")
                    if found:
                        prim = stage.GetPrimAtPath(found[0])
                else:
                    sys.stdout.write(f"  MISSING — joint not found anywhere\n")
                    continue
            sys.stdout.write(f"  type: {prim.GetTypeName()}\n")
            for rel_name in ("physics:body0", "physics:body1"):
                rel = prim.GetRelationship(rel_name)
                if rel and rel.IsValid():
                    targets = list(rel.GetTargets())
                    sys.stdout.write(f"  {rel_name}: {[str(t) for t in targets]}\n")
            sys.stdout.flush()

        # Also check hand_pitch_link survives
        sys.stdout.write("\n--- hand_pitch_link survival check ---\n")
        for side in ("left", "right"):
            wp = root.AppendChild(f"{side}_hand_pitch_link")
            p = stage.GetPrimAtPath(wp)
            sys.stdout.write(f"  {wp}: ")
            if p and p.IsValid():
                apis = list(p.GetAppliedSchemas())
                children = [c.GetName() for c in p.GetChildren()]
                sys.stdout.write(f"OK  APIs={apis}  children={children}\n")
            else:
                sys.stdout.write("MISSING\n")

        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
