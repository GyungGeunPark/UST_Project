"""Verify the Robotiq gripper holds the open pose under the build fix.

Boots the ``robot_only`` env, applies the idle action (which encodes the
open command on both grippers via ``BinaryJointPositionAction``), steps
the sim for 60 frames (0.5s at 120 Hz, enough for the overdamped PD
to settle), and reports per-joint final angles.

PASS = every Robotiq joint within ±10° of 0 after settle.  A real
       failure (mimic + drive both broken) yields 30°-90° drift on
       multiple joints, producing the chain-like deformation seen
       before the fix.  10° tolerance accounts for asymmetric
       gravitational PD steady-state error on the right gripper —
       the right wrist's world orientation in GR1T2 T-pose projects
       gravity onto the gripper's inner_finger joint axes differently
       than the left, giving ~5-7° persistent offset against the
       finite-stiffness drives.  The offset is well below any visual
       threshold for the gripper shape.
"""
from __future__ import annotations

import argparse
import math
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
        import torch  # noqa: F401

        from ust_ws.ust_hm_grip.kitchen_sorting_gr1t2_gripper_env_cfg import (
            KitchenSortingGR1T2GripperRobotOnlyEnvCfg,
            ROBOTIQ_ALL_JOINTS_LEFT,
            ROBOTIQ_ALL_JOINTS_RIGHT,
        )
        from isaaclab.envs import ManagerBasedRLEnv

        cfg = KitchenSortingGR1T2GripperRobotOnlyEnvCfg()
        cfg.scene.num_envs = 1
        env = ManagerBasedRLEnv(cfg)

        idle = cfg.idle_action.unsqueeze(0).to(env.device)
        env.reset()

        all_joint_names = list(ROBOTIQ_ALL_JOINTS_LEFT) + list(ROBOTIQ_ALL_JOINTS_RIGHT)
        robot = env.scene["robot"]
        joint_name_to_idx = {n: i for i, n in enumerate(robot.joint_names)}
        joint_indices = [joint_name_to_idx[j] for j in all_joint_names if j in joint_name_to_idx]
        missing = [j for j in all_joint_names if j not in joint_name_to_idx]
        for m in missing:
            print(f"[test_robotiq_pose] WARNING — joint '{m}' not found in articulation")
        present_names = [j for j in all_joint_names if j in joint_name_to_idx]

        print(f"\n[test_robotiq_pose] tracking {len(joint_indices)} robotiq joints over 30 steps "
              f"(idle action = both grippers commanded OPEN, target=0 rad).\n", flush=True)

        n_steps = 60
        history = []
        for s in range(n_steps):
            step_out = env.step(idle)
            history.append(robot.data.joint_pos[0].detach().cpu().tolist())

        # Print compact snapshot rows
        snapshot_steps = [0, 4, 9, 29, 59]
        print(f"{'step':>5} | " + " | ".join(f"{n[-22:]:>22}" for n in present_names), flush=True)
        for s in snapshot_steps:
            if s < n_steps:
                row = history[s]
                print(f"{s:>5} | " + " | ".join(
                    f"{math.degrees(row[i]):>+20.3f}°" for i in joint_indices
                ), flush=True)

        final = history[-1]
        print(f"\n[test_robotiq_pose] final joint state (n_steps={n_steps}):", flush=True)
        all_pass = True
        tolerance_deg = 10.0
        for jn, i in zip(present_names, joint_indices):
            v_deg = math.degrees(final[i])
            ok = abs(v_deg) < tolerance_deg
            tag = "OK  " if ok else "FAIL"
            print(f"  [{tag}] {jn:<45} = {v_deg:+9.3f}°  (tol ±{tolerance_deg}°)", flush=True)
            if not ok:
                all_pass = False

        verdict = "PASS" if all_pass else "FAIL"
        print(f"\n[test_robotiq_pose] VERDICT: {verdict} "
              + ("— gripper holds open pose under the build fix"
                 if all_pass else "— some joints drifted; mimic/drive still broken"),
              flush=True)
        return 0 if all_pass else 1
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
