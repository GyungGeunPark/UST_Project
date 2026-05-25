"""``ust_ws.ust_hm_glove`` — Fourier GR1T2 + 6-DoF hand migration target.

Parallel to ``ust_ws.ust_hm_glove`` (Windows/SteamVR G1 port) but for
Fourier GR1T2 + 6-DoF Fourier hand.

Importing this package registers six Gym environments that reuse the
``ust_ws.ust_260220`` kitchen-sorting scene & MDP but swap in the GR1T2
articulation and the 36D Pink IK action layout.

Environment IDs:

* ``Isaac-KitchenSorting-GR1T2-Fourier-v0``                 — base
* ``Isaac-KitchenSorting-GR1T2-Fourier-WaistEnabled-v0``    — Pink IK + waist
* ``Isaac-KitchenSorting-GR1T2-Fourier-Monitor-v0``         — PC-only view
* ``Isaac-KitchenSorting-GR1T2-Fourier-VR-v0``              — 1 h VR episode
* ``Isaac-KitchenSorting-GR1T2-Fourier-Vision-v0``          — cameras
* ``Isaac-KitchenSorting-GR1T2-Fourier-DataCollect-v0``     — vision + 90 s episode
"""

from __future__ import annotations


# The env_cfg module imports ``carb``, ``pink.tasks`` and Isaac Lab —
# packages only available inside a configured Isaac Sim / Isaac Lab
# environment.  Importing this package from a plain Python interpreter
# (e.g. running the smoke test, pytest unit tests, CI byte-compile) must
# still succeed so that the ``ust_hm_glove.teleop`` subpackage can
# be exercised without Isaac Sim.  We therefore register the Gym
# environments lazily; failures simply leave the registry untouched.


def _announce(msg: str, *, err: bool = False) -> None:
    """Print to both stdout and stderr with forced flushing.

    Isaac Sim's carb layer can buffer/redirect one stream while leaving
    the other alive, so we write to both deliberately.  No-op on any
    error (stdout/stderr may not exist in very early Kit bootstrap).
    """
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.write(msg + "\n")
            stream.flush()
        except Exception:
            pass


def _write_log(msg: str, append: bool = False) -> None:
    """Last-resort channel: write to ``config/last_import_error.log``."""
    try:
        from pathlib import Path

        log_path = Path(__file__).parent / "config" / "last_import_error.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(log_path, "a" if append else "w", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass


def _register() -> bool:
    """Register the GR1T2 Gym environments.

    Returns True on full success, False if anything went wrong.  All
    failures are announced loudly on stdout + stderr + a rotating log
    file at ``config/last_import_error.log``.
    """
    try:
        import gymnasium as gym  # type: ignore
    except Exception as exc:
        _announce(f"[ust_hm_glove] gymnasium unavailable: {exc}", err=True)
        return False

    try:
        import carb  # type: ignore  # noqa: F401
        inside_isaac = True
    except Exception:
        inside_isaac = False

    try:
        from .kitchen_sorting_gr1t2_env_cfg import (
            KitchenSortingGR1T2DataCollectEnvCfg,
            KitchenSortingGR1T2EnvCfg,
            KitchenSortingGR1T2MonitorEnvCfg,
            KitchenSortingGR1T2RobotOnlyEnvCfg,
            KitchenSortingGR1T2VREnvCfg,
            KitchenSortingGR1T2VisionEnvCfg,
            KitchenSortingGR1T2WaistEnvCfg,
        )
    except BaseException as exc:
        import os
        import traceback

        if inside_isaac or os.environ.get("UST_FOURIER_VERBOSE_IMPORT"):
            header = (
                "=" * 72 + "\n"
                "[ust_hm_glove] FAILED to import kitchen_sorting_gr1t2_env_cfg\n"
                "  Gym environments will NOT be registered (consumers will hit "
                "gymnasium.error.NameNotFound).\n"
                "  Root cause follows:\n" + "=" * 72
            )
            _announce(header, err=True)
            import io
            buf = io.StringIO()
            traceback.print_exc(file=buf)
            _announce(buf.getvalue(), err=True)
            _write_log(header + "\n" + buf.getvalue())
            _announce(
                f"[ust_hm_glove] traceback also written to "
                f"{(__import__('pathlib').Path(__file__).parent / 'config' / 'last_import_error.log')}",
                err=True,
            )
        return False

    # Direct class-map (no string-based indirection; makes failures
    # obvious instead of silent KeyErrors).
    specs = (
        ("Isaac-KitchenSorting-GR1T2-Fourier-v0", KitchenSortingGR1T2EnvCfg),
        ("Isaac-KitchenSorting-GR1T2-Fourier-WaistEnabled-v0", KitchenSortingGR1T2WaistEnvCfg),
        ("Isaac-KitchenSorting-GR1T2-Fourier-Monitor-v0", KitchenSortingGR1T2MonitorEnvCfg),
        ("Isaac-KitchenSorting-GR1T2-Fourier-VR-v0", KitchenSortingGR1T2VREnvCfg),
        ("Isaac-KitchenSorting-GR1T2-Fourier-Vision-v0", KitchenSortingGR1T2VisionEnvCfg),
        ("Isaac-KitchenSorting-GR1T2-Fourier-DataCollect-v0", KitchenSortingGR1T2DataCollectEnvCfg),
        ("Isaac-KitchenSorting-GR1T2-Fourier-RobotOnly-v0", KitchenSortingGR1T2RobotOnlyEnvCfg),
    )

    registered_ids = []
    skipped_ids = []
    failed_ids = []

    for env_id, env_cfg_cls in specs:
        if env_id in gym.registry:
            skipped_ids.append(env_id)
            continue
        try:
            gym.register(
                id=env_id,
                entry_point="isaaclab.envs:ManagerBasedRLEnv",
                kwargs={"env_cfg_entry_point": env_cfg_cls},
                disable_env_checker=True,
            )
            registered_ids.append(env_id)
        except BaseException as exc:
            failed_ids.append((env_id, repr(exc)))

    if inside_isaac:
        _announce(
            f"[ust_hm_glove] gym registry summary — "
            f"registered={len(registered_ids)} skipped={len(skipped_ids)} "
            f"failed={len(failed_ids)}"
        )
        if failed_ids:
            for env_id, err in failed_ids:
                _announce(f"  ✗ {env_id}: {err}", err=True)
            return False

    return True


# Register at package import time.  The function is idempotent — calling
# it again from ``run_teleop.py`` as a safety net is harmless.
_registered = _register()


def register_envs_now() -> bool:
    """Public helper to re-run registration (useful from entry-point scripts
    that want to surface a summary after Isaac Sim is fully booted)."""
    return _register()


__all__ = [
    "_registered",
    "register_envs_now",
]
