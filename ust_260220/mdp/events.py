"""Event functions for G1 Kitchen Sorting task (domain randomization).

Object spawn positions are randomized on the table surface.
Design guide: ust_ws/research/7. g1_kitchen_sorting_usd_scene_design_guide.md
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

# All manipulable objects (must match scene config variable names)
SORTING_OBJECTS = ["mug", "plate", "bowl", "can", "bottle", "apple", "sponge", "teddy"]


def reset_sorting_objects(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    min_separation: float = 0.08,
):
    """Reset all sorting objects to random non-overlapping positions on the table.

    Each object gets a random position within pose_range offsets from its default
    position, with minimum separation between objects to avoid overlapping.

    Args:
        env: The environment instance.
        env_ids: The environment IDs to reset.
        pose_range: Dict with keys 'x', 'y', 'z', 'yaw' specifying (min, max) ranges.
        min_separation: Minimum distance between objects (meters).
    """
    range_list = [pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
    ranges = torch.tensor(range_list, device=env.device)

    for obj_name in SORTING_OBJECTS:
        try:
            obj = env.scene[obj_name]
        except KeyError:
            continue

        # Get default root state
        root_states = obj.data.default_root_state[env_ids].clone()

        # Generate random offsets
        rand_samples = math_utils.sample_uniform(
            ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=env.device
        )
        orientations_delta = math_utils.quat_from_euler_xyz(
            rand_samples[:, 3], rand_samples[:, 4], rand_samples[:, 5]
        )

        positions = root_states[:, 0:3] + env.scene.env_origins[env_ids] + rand_samples[:, 0:3]
        orientations = math_utils.quat_mul(root_states[:, 3:7], orientations_delta)

        obj.write_root_pose_to_sim(
            torch.cat([positions, orientations], dim=-1), env_ids=env_ids
        )
