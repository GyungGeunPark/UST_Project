#!/usr/bin/env python3
"""
Run the G1 Kitchen Sorting environment (no teleop, just visualization).

Usage:
    # Basic environment test
    python ust_ws/ust_260220/scripts/run_env.py --num_envs 1

    # With cameras (RTX PRO 6000 - 720p at 60fps)
    python ust_ws/ust_260220/scripts/run_env.py --num_envs 1 --enable_cameras

    # Multi-env training mode (RTX PRO 6000 - up to 16 envs)
    python ust_ws/ust_260220/scripts/run_env.py --num_envs 16 --headless
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))  # import ust_ws.ust_260220

# Import pinocchio before AppLauncher to force the use of the version installed
# by IsaacLab and not the one installed by Isaac Sim.
# pinocchio is required by the Pink IK controllers.
import pinocchio  # noqa: F401

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="G1 Kitchen Sorting Environment")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")
parser.add_argument("--task", type=str, default=None,
                    help="Task ID (default: auto-select based on flags)")
parser.add_argument("--disable_fabric", action="store_true", help="Disable Fabric API")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- After app launch ---
import torch
import gymnasium as gym

import ust_ws.ust_260220  # noqa: F401

from isaaclab.envs import ManagerBasedRLEnv


def main():
    """Main function."""
    # Auto-select task based on flags
    if args_cli.task:
        task_id = args_cli.task
    elif args_cli.num_envs > 1:
        task_id = "Isaac-KitchenSorting-G1-InspireFTP-Train-v0"
    else:
        task_id = "Isaac-KitchenSorting-G1-InspireFTP-v0"

    env_cfg_cls = gym.spec(task_id).kwargs["env_cfg_entry_point"]
    env_cfg = env_cfg_cls()
    env_cfg.scene.num_envs = args_cli.num_envs

    env = ManagerBasedRLEnv(cfg=env_cfg)

    print("\n" + "=" * 60)
    print("  G1 Kitchen Sorting Environment")
    print(f"  Task: {task_id}")
    print("  Objects: mug, plate, bowl, can, bottle, apple, sponge, teddy")
    print("  Bins: kitchen (left), food (center), misc (right)")
    print(f"  Table: 0.9x0.6m at {env_cfg.scene.table.spawn.size[2]}m height")
    print(f"  Action dim: {env.action_space.shape}")
    print(f"  Num envs: {env.num_envs}")
    print(f"  GPU: RTX PRO 6000")
    print("=" * 60 + "\n")

    obs, info = env.reset()
    idle_action = env_cfg.idle_action.unsqueeze(0).repeat(env.num_envs, 1).to(env.device)

    step = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            obs, rewards, terminated, truncated, info = env.step(idle_action)
            step += 1

            if step % 200 == 0:
                print(f"Step {step}, Reward: {rewards.mean().item():.3f}")

            if terminated.any() or truncated.any():
                print(f"[Step {step}] Episode done. Resetting...")
                obs, info = env.reset()

    env.close()


if __name__ == "__main__":
    main()
