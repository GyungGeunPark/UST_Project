"""Termination functions for G1 Kitchen Sorting task."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from ..constants import TABLE_SURFACE_Z

# Object-to-bin category mapping
OBJECT_BIN_MAP = {
    "mug": "bin_kitchen",
    "plate": "bin_kitchen",
    "bowl": "bin_kitchen",
    "can": "bin_food",
    "bottle": "bin_food",
    "apple": "bin_food",
    "sponge": "bin_misc",
    "teddy": "bin_misc",
}


def object_in_correct_bin(
    env: ManagerBasedRLEnv,
    object_name: str,
    bin_name: str,
    threshold: float = 0.12,
) -> torch.Tensor:
    """Check if an object is within threshold distance of its correct bin (XY only).

    Returns: (num_envs,) boolean tensor.
    """
    try:
        obj: RigidObject = env.scene[object_name]
        target_bin: RigidObject = env.scene[bin_name]
    except KeyError:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    obj_pos = obj.data.root_pos_w - env.scene.env_origins
    bin_pos = target_bin.data.root_pos_w - env.scene.env_origins

    dx = torch.abs(obj_pos[:, 0] - bin_pos[:, 0])
    dy = torch.abs(obj_pos[:, 1] - bin_pos[:, 1])

    in_bin = torch.logical_and(dx < threshold, dy < threshold)
    return in_bin


def all_objects_sorted(
    env: ManagerBasedRLEnv,
    threshold: float = 0.12,
) -> torch.Tensor:
    """Check if ALL objects are in their correct bins.

    Returns: (num_envs,) boolean tensor, True when all objects are correctly sorted.
    """
    all_done = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)

    for obj_name, bin_name in OBJECT_BIN_MAP.items():
        in_bin = object_in_correct_bin(env, obj_name, bin_name, threshold)
        all_done = torch.logical_and(all_done, in_bin)

    return all_done


def count_sorted_objects(
    env: ManagerBasedRLEnv,
    threshold: float = 0.12,
) -> torch.Tensor:
    """Count how many objects are correctly sorted.

    Returns: (num_envs,) float tensor with count of sorted objects.
    """
    count = torch.zeros(env.num_envs, dtype=torch.float32, device=env.device)

    for obj_name, bin_name in OBJECT_BIN_MAP.items():
        in_bin = object_in_correct_bin(env, obj_name, bin_name, threshold)
        count += in_bin.float()

    return count


def any_object_dropped(
    env: ManagerBasedRLEnv,
    minimum_height: float = TABLE_SURFACE_Z - 0.15,  # below table surface minus margin
) -> torch.Tensor:
    """Check if any object has fallen below minimum height.

    Returns: (num_envs,) boolean tensor.
    """
    any_dropped = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    for obj_name in OBJECT_BIN_MAP.keys():
        try:
            obj: RigidObject = env.scene[obj_name]
            obj_z = obj.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
            dropped = obj_z < minimum_height
            any_dropped = torch.logical_or(any_dropped, dropped)
        except KeyError:
            pass

    return any_dropped
