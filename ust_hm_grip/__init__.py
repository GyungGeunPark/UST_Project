"""ust_hm_grip — GR1T2 + 2-finger gripper teleop on PICO 4 Ultra (option B).

This package implements the option B migration described in
``research/36. gripper_elbow_tracker_pico_controller_migration_design_guide.md``.

Public modules:

* ``teleop.gr1t2_gripper_retargeter``  — pure-math 16-D retargeter
* ``teleop.gr1t2_gripper_device``      — SteamVR + PICO controller device
* ``kitchen_sorting_gr1t2_gripper_env_cfg`` — Pink IK arms + 2 BinaryGripperAction
* ``isaac_file.build_gripper_usd``     — USD post-processor (run inside Isaac Sim)
* ``scripts.run_teleop``               — main entry point
* ``scripts.smoke_test``               — Isaac-Sim-free regression script

Gym IDs registered by :func:`register_envs_now`:

* ``Isaac-KitchenSorting-GR1T2-Gripper-v0``
* ``Isaac-KitchenSorting-GR1T2-Gripper-WaistEnabled-v0``
* ``Isaac-KitchenSorting-GR1T2-Gripper-Monitor-v0``
* ``Isaac-KitchenSorting-GR1T2-Gripper-Vision-v0``
* ``Isaac-KitchenSorting-GR1T2-Gripper-VR-v0``
* ``Isaac-KitchenSorting-GR1T2-Gripper-DataCollect-v0``
* ``Isaac-KitchenSorting-GR1T2-Gripper-RobotOnly-v0``
"""

from __future__ import annotations

import os
import traceback


_REGISTERED = False


def register_envs_now() -> None:
    """Register the gripper-equipped Kitchen Sorting envs with Gymnasium.

    Lazy because importing :mod:`kitchen_sorting_gr1t2_gripper_env_cfg`
    pulls in ``isaaclab`` + ``pink`` + ``carb`` which need Isaac Sim's
    AppLauncher to have run.  Calling this before AppLauncher has
    initialised Isaac Sim raises an ImportError that we catch and write
    to ``config/last_import_error.log`` (mirroring the
    ust_fourier_260421 pattern).
    """
    global _REGISTERED
    if _REGISTERED:
        return

    try:
        import gymnasium as gym

        from .kitchen_sorting_gr1t2_gripper_env_cfg import (  # noqa: F401
            KitchenSortingGR1T2GripperDataCollectEnvCfg,
            KitchenSortingGR1T2GripperEnvCfg,
            KitchenSortingGR1T2GripperMonitorEnvCfg,
            KitchenSortingGR1T2GripperRobotOnlyEnvCfg,
            KitchenSortingGR1T2GripperVREnvCfg,
            KitchenSortingGR1T2GripperVisionEnvCfg,
            KitchenSortingGR1T2GripperWaistEnvCfg,
        )

        registrations = [
            (
                "Isaac-KitchenSorting-GR1T2-Gripper-v0",
                KitchenSortingGR1T2GripperEnvCfg,
            ),
            (
                "Isaac-KitchenSorting-GR1T2-Gripper-WaistEnabled-v0",
                KitchenSortingGR1T2GripperWaistEnvCfg,
            ),
            (
                "Isaac-KitchenSorting-GR1T2-Gripper-Monitor-v0",
                KitchenSortingGR1T2GripperMonitorEnvCfg,
            ),
            (
                "Isaac-KitchenSorting-GR1T2-Gripper-Vision-v0",
                KitchenSortingGR1T2GripperVisionEnvCfg,
            ),
            (
                "Isaac-KitchenSorting-GR1T2-Gripper-VR-v0",
                KitchenSortingGR1T2GripperVREnvCfg,
            ),
            (
                "Isaac-KitchenSorting-GR1T2-Gripper-DataCollect-v0",
                KitchenSortingGR1T2GripperDataCollectEnvCfg,
            ),
            (
                "Isaac-KitchenSorting-GR1T2-Gripper-RobotOnly-v0",
                KitchenSortingGR1T2GripperRobotOnlyEnvCfg,
            ),
        ]
        for env_id, cfg_cls in registrations:
            if env_id in gym.registry:
                continue
            gym.register(
                id=env_id,
                entry_point="isaaclab.envs:ManagerBasedRLEnv",
                kwargs={"env_cfg_entry_point": cfg_cls},
                disable_env_checker=True,
            )
        _REGISTERED = True
    except Exception:
        log_path = os.path.join(
            os.path.dirname(__file__), "config", "last_import_error.log"
        )
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        with open(log_path, "w", encoding="utf-8") as f:
            traceback.print_exc(file=f)
        # Re-raise so callers (e.g. run_teleop.py) see a real traceback.
        raise


# Best-effort registration on import: if the consumer has already booted
# Isaac Sim (via AppLauncher) before importing this package, registration
# succeeds silently.  Otherwise the exception is logged and the consumer
# is expected to call ``register_envs_now()`` after AppLauncher.
try:
    register_envs_now()
except Exception:
    pass
