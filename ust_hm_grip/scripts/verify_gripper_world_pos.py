"""Quick verifier: compute the WORLD positions of TCP, base_link, and
outer_knuckle bodies for both grippers in the rebuilt USD, and report
the gripper's fingertip direction (TCP - base_link) in world space.

This is just enough to confirm that the per-side rotation correction is
correctly authored without booting the physics sim.
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
        from pxr import Usd, UsdGeom, Sdf  # noqa
        usd_path = str(_REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd")
        stage = Usd.Stage.Open(usd_path)
        root = stage.GetDefaultPrim().GetPath()
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())

        sys.stdout.write("\n========== gripper world positions ==========\n")
        for side in ("left", "right"):
            wrist = stage.GetPrimAtPath(root.AppendChild(f"{side}_hand_pitch_link"))
            container = stage.GetPrimAtPath(root.AppendChild(f"{side}_robotiq_arg2f_85"))
            base_link = stage.GetPrimAtPath(container.GetPath().AppendChild("base_link"))
            tcp = stage.GetPrimAtPath(root.AppendChild(f"{side}_gripper_tcp_link"))

            wt = xc.GetLocalToWorldTransform(wrist).ExtractTranslation()
            bt = xc.GetLocalToWorldTransform(base_link).ExtractTranslation()
            tt = xc.GetLocalToWorldTransform(tcp).ExtractTranslation()
            # gripper's fingertip direction in world = TCP - base_link
            direction = (tt[0] - bt[0], tt[1] - bt[1], tt[2] - bt[2])
            sys.stdout.write(
                f"\n[{side.upper()}]\n"
                f"  wrist world     : ({wt[0]:+.4f}, {wt[1]:+.4f}, {wt[2]:+.4f})\n"
                f"  base_link world : ({bt[0]:+.4f}, {bt[1]:+.4f}, {bt[2]:+.4f})\n"
                f"  TCP world       : ({tt[0]:+.4f}, {tt[1]:+.4f}, {tt[2]:+.4f})\n"
                f"  fingertip dir   : ({direction[0]:+.4f}, {direction[1]:+.4f}, {direction[2]:+.4f})\n"
            )
            sys.stdout.flush()

            # Also show container rotation
            c_world = xc.GetLocalToWorldTransform(container)
            sys.stdout.write(
                f"  container rot.  : "
            )
            for i in range(3):
                sys.stdout.write(
                    f"row{i}=({c_world[i][0]:+.3f},{c_world[i][1]:+.3f},{c_world[i][2]:+.3f}) "
                )
            sys.stdout.write("\n")
            sys.stdout.flush()

        sys.stdout.write("\nExpected outcomes:\n")
        sys.stdout.write("  LEFT  fingertip direction ≈ (0, +0.15, 0)  (along +Y, away from body)\n")
        sys.stdout.write("  RIGHT fingertip direction ≈ (0, -0.15, 0)  (along -Y, away from body)\n")
        sys.stdout.flush()
        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
