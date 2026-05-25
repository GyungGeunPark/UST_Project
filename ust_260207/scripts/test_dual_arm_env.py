#!/usr/bin/env python3
"""Test script for dual arm UST environment.

Verifies that the environment loads correctly with dual arm configuration.
"""

from __future__ import annotations

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from isaaclab.app import AppLauncher

parser = __import__("argparse").ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args([])
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
from isaaclab.envs import ManagerBasedEnv
from ust_config.ust_teleop_env_cfg import USTMobileManipulatorTrainEnvCfg


def main():
    print("\n" + "=" * 60)
    print("  UST Dual Arm Environment Test")
    print("=" * 60 + "\n")

    # Use training config (no sensors needed, simple scene)
    env_cfg = USTMobileManipulatorTrainEnvCfg()
    env_cfg.scene.num_envs = 2

    print("[1] Creating environment...")
    env = ManagerBasedEnv(cfg=env_cfg)
    print(f"    Device: {env.device}")
    print(f"    Num envs: {env.num_envs}")
    print(f"    Action dim: {env.action_manager.total_action_dim}")

    # Simple config: 4 wheels + 4 right arm (direct) + 1 right gripper + 4 left arm (direct) + 1 left gripper = 14
    # IK config: 4 wheels + 6 right arm (IK) + 1 right gripper + 6 left arm (IK) + 1 left gripper = 18
    expected_action_dim = 14  # USTSimpleActionsCfg (training env)
    actual_action_dim = env.action_manager.total_action_dim
    assert actual_action_dim == expected_action_dim, (
        f"Action dim mismatch: expected {expected_action_dim}, got {actual_action_dim}"
    )
    print(f"    [OK] Action dim = {actual_action_dim} (simple config)")

    print("\n[2] Resetting environment...")
    obs, info = env.reset()

    if isinstance(obs, dict) and "policy" in obs:
        obs_tensor = obs["policy"]
        print(f"    Observation shape: {obs_tensor.shape}")
    else:
        print(f"    Observation type: {type(obs)}")

    print("\n[3] Running 100 steps with zero action...")
    for i in range(100):
        action = torch.zeros(env.num_envs, actual_action_dim, device=env.device)
        obs, info = env.step(action)
        if (i + 1) % 50 == 0:
            print(f"    Step {i + 1} OK")

    print("\n[4] Running 100 steps with random action...")
    for i in range(100):
        action = torch.randn(env.num_envs, actual_action_dim, device=env.device) * 0.1
        obs, info = env.step(action)
        if (i + 1) % 50 == 0:
            print(f"    Step {i + 1} OK")

    print("\n[5] Checking joint info...")
    robot = env.scene["robot"]
    joint_names = robot.joint_names
    print(f"    Total joints: {len(joint_names)}")
    for jn in joint_names:
        print(f"      - {jn}")

    # Verify dual arm joint names
    expected_joints = [
        "wheel_left_front_joint", "wheel_right_front_joint",
        "wheel_left_rear_joint", "wheel_right_rear_joint",
        "right_joint1", "right_joint2", "right_joint3", "right_joint4",
        "right_gripper_left_joint", "right_gripper_right_joint",
        "left_joint1", "left_joint2", "left_joint3", "left_joint4",
        "left_gripper_left_joint", "left_gripper_right_joint",
    ]
    for ej in expected_joints:
        assert ej in joint_names, f"Missing joint: {ej}"
    print(f"    [OK] All {len(expected_joints)} expected joints found")

    print("\n[6] Cleanup...")
    env.close()

    print("\n" + "=" * 60)
    print("  ALL TESTS PASSED!")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
    simulation_app.close()
