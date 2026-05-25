"""G1 Kitchen Sorting Environment - Unitree G1 with INSPIRE 5-finger hand."""

import gymnasium as gym
import os

from . import kitchen_sorting_env_cfg  # noqa: F401

##
# Register Gym environments.
##

# Base environment (VR teleoperation, no cameras)
gym.register(
    id="Isaac-KitchenSorting-G1-InspireFTP-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": kitchen_sorting_env_cfg.KitchenSortingG1EnvCfg,
    },
    disable_env_checker=True,
)

# Vision environment (with cameras, requires --enable_cameras)
gym.register(
    id="Isaac-KitchenSorting-G1-InspireFTP-Vision-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": kitchen_sorting_env_cfg.KitchenSortingG1VisionEnvCfg,
    },
    disable_env_checker=True,
)

# Training environment (multi-env, no cameras)
gym.register(
    id="Isaac-KitchenSorting-G1-InspireFTP-Train-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": kitchen_sorting_env_cfg.KitchenSortingG1TrainEnvCfg,
    },
    disable_env_checker=True,
)

# VR teleoperation environment (long episodes, CloudXR optimized)
gym.register(
    id="Isaac-KitchenSorting-G1-InspireFTP-VR-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": kitchen_sorting_env_cfg.KitchenSortingG1VREnvCfg,
    },
    disable_env_checker=True,
)

# Data collection environment (cameras + medium episodes)
gym.register(
    id="Isaac-KitchenSorting-G1-InspireFTP-DataCollect-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": kitchen_sorting_env_cfg.KitchenSortingG1DataCollectEnvCfg,
    },
    disable_env_checker=True,
)

# Monitor teleoperation environment (no XR, PC screen only)
gym.register(
    id="Isaac-KitchenSorting-G1-InspireFTP-Monitor-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": kitchen_sorting_env_cfg.KitchenSortingG1MonitorEnvCfg,
    },
    disable_env_checker=True,
)

# USD scene environment (ust_human1.usd custom scene + PICO VR teleop)
gym.register(
    id="Isaac-KitchenSorting-G1-InspireFTP-USD-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": kitchen_sorting_env_cfg.KitchenSortingG1USDEnvCfg,
    },
    disable_env_checker=True,
)
