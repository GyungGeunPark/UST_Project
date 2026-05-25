"""Patch for Isaac Lab's PinkInverseKinematicsAction when hand_joint_dim=0.

Problem
-------
``isaaclab/envs/mdp/actions/pink_task_space_actions.py:200`` extracts the
hand-joint slice with::

    self._target_hand_joint_positions = actions[:, -self.hand_joint_dim:]

When ``hand_joint_dim`` is zero (the gripper-only configuration in
``ust_hm_grip``, where the binary gripper is driven by a separate
``BinaryJointPositionAction`` rather than by Pink IK), Python evaluates
``actions[:, -0:]`` as ``actions[:, 0:]`` — i.e. **the entire actions
tensor**, not an empty slice.  Then ``apply_actions`` runs::

    all_joint_positions = torch.cat((ik_joint_positions,
                                      self._target_hand_joint_positions),
                                     dim=1)

so the cat produces ``(num_envs, ik_dim + action_dim)`` rows.  For the
robot_only/waist variant that means (1, 17 + 14) = (1, 31), and
``set_joint_position_target(target, joint_ids)`` then explodes with
``RuntimeError: shape mismatch: value tensor of shape [31] cannot be
broadcast to indexing result of shape [1, 17]``.

The main loop's ``except KeyboardInterrupt`` swallowed that traceback
until we widened the catch in ``run_teleop.py``; the symptom you saw was
``Simulation App Shutting Down`` ~3 seconds after the first env.step.

Fix
---
Wrap ``PinkInverseKinematicsAction.process_actions`` so that when
``hand_joint_dim == 0`` we explicitly produce a zero-width slice
``actions[:, 0:0]`` and call the rest of the original logic.  The patch
is idempotent.

This is upstream Isaac Lab code and the right long-term fix is a PR
that replaces line 200 with an ``if self.hand_joint_dim > 0`` guard, but
until that lands we ship the workaround locally.
"""

from __future__ import annotations

_PATCHED_FLAG = "_ust_hm_grip_hand_dim_zero_patched"


def apply() -> None:
    """Idempotently patch ``PinkInverseKinematicsAction.process_actions``."""
    try:
        from isaaclab.envs.mdp.actions.pink_task_space_actions import (
            PinkInverseKinematicsAction,
        )
    except ImportError:
        # isaaclab.envs is loaded lazily after AppLauncher; defer until then.
        return

    if getattr(PinkInverseKinematicsAction, _PATCHED_FLAG, False):
        return

    original = PinkInverseKinematicsAction.process_actions

    def process_actions(self, actions):  # type: ignore[no-redef]
        if self.hand_joint_dim == 0:
            # Reproduce the original body but with the slicing bug fixed.
            self._raw_actions[:] = actions
            # ``actions[:, -0:]`` returns the entire tensor; use a zero-width
            # slice instead so the cat in apply_actions stays the right shape.
            self._target_hand_joint_positions = actions[:, 0:0]
            self.base_link_frame_in_world_rf = self._get_base_link_frame_transform()
            controlled_frame_poses = self._extract_controlled_frame_poses(actions)
            transformed_poses = self._transform_poses_to_base_link_frame(
                controlled_frame_poses
            )
            self._set_task_targets(transformed_poses)
            return
        return original(self, actions)

    PinkInverseKinematicsAction.process_actions = process_actions
    setattr(PinkInverseKinematicsAction, _PATCHED_FLAG, True)
    print(
        "[ust_hm_grip] _pink_hand_dim_zero_patch applied "
        "(works around hand_joint_dim==0 slicing bug in PinkInverseKinematicsAction).",
        flush=True,
    )
