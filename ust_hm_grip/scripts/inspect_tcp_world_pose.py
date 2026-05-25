"""Inspect actual left/right gripper TCP world + base_link pose when
the robot is in default joint state.

User report: even after calibration, left wrist twists backward; left
idle pose (-0.20, 0, 1.05) in base_link frame looks suspicious because
IL +X=forward means -0.20 X is BEHIND robot.  Measure the real TCP
pose so the env_cfg DEFAULT_*_POS can be re-anchored to the true
T-pose (= what the robot reaches when no Pink IK target overrides).
"""
from __future__ import annotations
import argparse, sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))


def _boot():
    from isaaclab.app import AppLauncher
    p = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(p)
    b, r = p.parse_known_args()
    b.headless = True
    AppLauncher(b).app
    sys.argv = [sys.argv[0]] + r


def main() -> int:
    _boot()
    import numpy as np
    import torch

    from ust_ws.ust_hm_grip.teleop import _osqp_compat
    _osqp_compat.apply()
    from ust_ws.ust_hm_grip.teleop import _pink_hand_dim_zero_patch
    _pink_hand_dim_zero_patch.apply()

    from isaaclab.envs import ManagerBasedRLEnv
    from ust_ws.ust_hm_grip.kitchen_sorting_gr1t2_gripper_env_cfg import (
        KitchenSortingGR1T2GripperRobotOnlyEnvCfg,
    )

    cfg = KitchenSortingGR1T2GripperRobotOnlyEnvCfg()
    cfg.scene.num_envs = 1
    cfg.sim.device = "cuda:0"
    env = ManagerBasedRLEnv(cfg=cfg)
    env.reset()

    robot = env.scene["robot"]
    body_names = list(robot.data.body_names)
    print(f"[inspect] articulation has {len(body_names)} bodies")

    candidates = [
        "base_link",
        "left_gripper_tcp_link",
        "right_gripper_tcp_link",
        "left_hand_pitch_link",
        "right_hand_pitch_link",
        "head_pitch_link",
        "waist_roll_link",
    ]
    found = {}
    for name in candidates:
        if name in body_names:
            found[name] = body_names.index(name)
        else:
            print(f"[inspect] missing: {name}")

    # Run one step so body state populates
    idle = cfg.idle_action.unsqueeze(0).to(env.device)
    for _ in range(5):
        env.step(idle)

    pos_w = robot.data.body_pos_w[0].detach().cpu().numpy()
    quat_w = robot.data.body_quat_w[0].detach().cpu().numpy()

    print("\n=== World-frame body poses at default + idle action ===")
    for name, idx in found.items():
        p = pos_w[idx]
        q = quat_w[idx]
        print(f"  {name:30s} idx={idx:3d}  pos_w=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  quat_w=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})")

    # Compute TCP pose in base_link frame (manual transform)
    if "base_link" in found:
        from ust_ws.ust_hm_grip.teleop import coord_transforms as ct
        bl_pos = pos_w[found["base_link"]]
        bl_quat = quat_w[found["base_link"]]
        bl_quat_inv = ct.quat_conjugate(bl_quat)
        print("\n=== TCP pose in base_link frame (what env_cfg DEFAULT_*_POS should be) ===")
        for tcp_name in ("left_gripper_tcp_link", "right_gripper_tcp_link"):
            if tcp_name not in found:
                continue
            tcp_pos_w = pos_w[found[tcp_name]]
            tcp_quat_w = quat_w[found[tcp_name]]
            # pos in base_link frame: rotate (tcp_w - bl_w) by inv(bl_quat)
            delta_w = tcp_pos_w - bl_pos
            tcp_pos_bl = ct.quat_rotate_vec(bl_quat_inv, delta_w)
            tcp_quat_bl = ct.quat_multiply(bl_quat_inv, tcp_quat_w)
            print(f"  {tcp_name}:")
            print(f"     pos_in_base_link  = ({tcp_pos_bl[0]:+.4f}, {tcp_pos_bl[1]:+.4f}, {tcp_pos_bl[2]:+.4f})")
            print(f"     quat_in_base_link = ({tcp_quat_bl[0]:+.4f}, {tcp_quat_bl[1]:+.4f}, {tcp_quat_bl[2]:+.4f}, {tcp_quat_bl[3]:+.4f})")

    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
