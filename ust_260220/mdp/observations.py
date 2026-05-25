"""Observation functions for G1 Kitchen Sorting task."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# -- List of all kitchen objects in the scene --
# NOTE: These MUST match the variable names in KitchenSortingSceneCfg
KITCHEN_OBJECTS = ["mug", "plate", "bowl", "can", "bottle", "apple", "sponge", "teddy"]
BIN_NAMES = ["bin_kitchen", "bin_food", "bin_misc"]

# Object-to-bin category mapping (for observation helpers)
OBJECT_CATEGORIES = {
    "mug": "kitchen", "plate": "kitchen", "bowl": "kitchen",
    "can": "food", "bottle": "food", "apple": "food",
    "sponge": "misc", "teddy": "misc",
}


def get_eef_pos(env: ManagerBasedRLEnv, link_name: str) -> torch.Tensor:
    """Get end-effector position relative to environment origin."""
    body_pos_w = env.scene["robot"].data.body_pos_w
    eef_idx = env.scene["robot"].data.body_names.index(link_name)
    eef_pos = body_pos_w[:, eef_idx] - env.scene.env_origins
    return eef_pos


def get_eef_quat(env: ManagerBasedRLEnv, link_name: str) -> torch.Tensor:
    """Get end-effector quaternion."""
    body_quat_w = env.scene["robot"].data.body_quat_w
    eef_idx = env.scene["robot"].data.body_names.index(link_name)
    eef_quat = body_quat_w[:, eef_idx]
    return eef_quat


def get_robot_joint_state(
    env: ManagerBasedRLEnv,
    joint_names: list[str],
) -> torch.Tensor:
    """Get specified robot joint positions."""
    indexes, _ = env.scene["robot"].find_joints(joint_names)
    indexes = torch.tensor(indexes, dtype=torch.long)
    robot_joint_states = env.scene["robot"].data.joint_pos[:, indexes]
    return robot_joint_states


def get_all_robot_link_state(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get all robot link states."""
    body_pos_w = env.scene["robot"].data.body_link_state_w[:, :, :]
    return body_pos_w


def object_positions(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get concatenated positions of all sorting objects relative to env origin.

    Returns: (num_envs, num_objects * 3) tensor with XYZ for each object.
    """
    positions = []
    for obj_name in KITCHEN_OBJECTS:
        try:
            obj = env.scene[obj_name]
            pos = obj.data.root_pos_w - env.scene.env_origins
            positions.append(pos)
        except KeyError:
            # Object not in scene (maybe a subset is used)
            pass
    if not positions:
        return torch.zeros(env.num_envs, 3, device=env.device)
    return torch.cat(positions, dim=1)


def object_rotations(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get concatenated quaternions of all sorting objects.

    Returns: (num_envs, num_objects * 4) tensor with quat for each object.
    """
    rotations = []
    for obj_name in KITCHEN_OBJECTS:
        try:
            obj = env.scene[obj_name]
            rot = obj.data.root_quat_w
            rotations.append(rot)
        except KeyError:
            pass
    if not rotations:
        return torch.zeros(env.num_envs, 4, device=env.device)
    return torch.cat(rotations, dim=1)


def bin_positions(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get concatenated positions of all bins relative to env origin.

    Returns: (num_envs, 9) tensor with XYZ for 3 bins.
    """
    positions = []
    for bin_name in BIN_NAMES:
        try:
            obj = env.scene[bin_name]
            pos = obj.data.root_pos_w - env.scene.env_origins
            positions.append(pos)
        except KeyError:
            pass
    if not positions:
        return torch.zeros(env.num_envs, 9, device=env.device)
    return torch.cat(positions, dim=1)


def eef_to_nearest_object(
    env: ManagerBasedRLEnv,
    left_eef_link_name: str = "left_wrist_yaw_link",
    right_eef_link_name: str = "right_wrist_yaw_link",
) -> torch.Tensor:
    """Get distance vectors from both EEFs to all objects.

    Returns: (num_envs, num_objects * 6) - [left_dx,left_dy,left_dz, right_dx,right_dy,right_dz] per object.
    """
    body_pos_w = env.scene["robot"].data.body_pos_w
    left_idx = env.scene["robot"].data.body_names.index(left_eef_link_name)
    right_idx = env.scene["robot"].data.body_names.index(right_eef_link_name)
    left_eef_pos = body_pos_w[:, left_idx] - env.scene.env_origins
    right_eef_pos = body_pos_w[:, right_idx] - env.scene.env_origins

    deltas = []
    for obj_name in KITCHEN_OBJECTS:
        try:
            obj = env.scene[obj_name]
            obj_pos = obj.data.root_pos_w - env.scene.env_origins
            deltas.append(obj_pos - left_eef_pos)
            deltas.append(obj_pos - right_eef_pos)
        except KeyError:
            pass
    if not deltas:
        return torch.zeros(env.num_envs, 6, device=env.device)
    return torch.cat(deltas, dim=1)
