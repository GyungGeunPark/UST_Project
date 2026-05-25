"""Inspect the GR1T2 wrist link frame orientation in the *stock* USD.

We want to know:
  - At bind pose, what's the world rotation of ``{side}_hand_pitch_link``?
  - Where is each ``L_thumb_proximal_link`` / ``L_index_proximal_link``
    in world relative to the wrist?  This tells us which local axis
    is "palm-out" (the direction the fingertips point) and which is
    "up" (back of the hand).

Compares the LEFT and RIGHT wrist independently.
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
        from pxr import Usd, UsdGeom, Gf  # noqa
        import os

        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
        gr1t2_usd = (
            f"{ISAAC_NUCLEUS_DIR}/Robots/FourierIntelligence/GR-1/"
            f"GR1T2_fourier_hand_6dof/GR1T2_fourier_hand_6dof.usd"
        )
        print(f"[inspect_wrist] opening GR1T2 stock USD: {gr1t2_usd!r}", flush=True)
        stage = Usd.Stage.Open(gr1t2_usd)
        if not stage:
            print("[inspect_wrist] FAIL — cannot open", flush=True)
            return 1

        default = stage.GetDefaultPrim()
        root = default.GetPath()
        print(f"[inspect_wrist] default prim: {root}", flush=True)

        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        for side, cap in (("left", "L"), ("right", "R")):
            print(f"\n========== {side.upper()} ARM ==========", flush=True)
            wrist_path = root.AppendChild(f"{side}_hand_pitch_link")
            wrist_prim = stage.GetPrimAtPath(wrist_path)
            if not wrist_prim or not wrist_prim.IsValid():
                print(f"  no prim at {wrist_path}", flush=True)
                continue
            wrist_world = xc.GetLocalToWorldTransform(wrist_prim)
            wq = wrist_world.ExtractRotationQuat()
            wt = wrist_world.ExtractTranslation()
            print(f"  wrist world  : translate=({wt[0]:+.4f},{wt[1]:+.4f},{wt[2]:+.4f})", flush=True)
            print(f"                 rotation_quat (real, im_x, im_y, im_z) = "
                  f"({wq.GetReal():+.4f},{wq.GetImaginary()[0]:+.4f},"
                  f"{wq.GetImaginary()[1]:+.4f},{wq.GetImaginary()[2]:+.4f})", flush=True)

            # Compute wrist local axes in world.  GetLocalToWorldTransform
            # uses ROW vectors with the matrix on the right, so the rows of
            # the upper 3x3 are the local axes expressed in world.
            m = wrist_world  # Matrix4d
            wx = (m[0][0], m[0][1], m[0][2])
            wy = (m[1][0], m[1][1], m[1][2])
            wz = (m[2][0], m[2][1], m[2][2])
            print(f"  wrist +X axis in world: ({wx[0]:+.4f},{wx[1]:+.4f},{wx[2]:+.4f})", flush=True)
            print(f"  wrist +Y axis in world: ({wy[0]:+.4f},{wy[1]:+.4f},{wy[2]:+.4f})", flush=True)
            print(f"  wrist +Z axis in world: ({wz[0]:+.4f},{wz[1]:+.4f},{wz[2]:+.4f})", flush=True)

            # Find a fingertip-area link to determine palm-out direction
            for finger_link_name in (
                f"{cap}_index_proximal_link",
                f"{cap}_middle_proximal_link",
                f"{cap}_thumb_proximal_link",
            ):
                fp = root.AppendChild(finger_link_name)
                fprim = stage.GetPrimAtPath(fp)
                if fprim and fprim.IsValid():
                    fw = xc.GetLocalToWorldTransform(fprim)
                    ft = fw.ExtractTranslation()
                    delta = (ft[0] - wt[0], ft[1] - wt[1], ft[2] - wt[2])
                    print(f"  {finger_link_name:>25s} world = ({ft[0]:+.4f},{ft[1]:+.4f},{ft[2]:+.4f})", flush=True)
                    print(f"  delta from wrist = ({delta[0]:+.4f},{delta[1]:+.4f},{delta[2]:+.4f})  "
                          f"(this is the palm-out direction in world)", flush=True)
        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
