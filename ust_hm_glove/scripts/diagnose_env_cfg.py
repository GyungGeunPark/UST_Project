"""Standalone env_cfg import diagnostic.

Boots Isaac Sim the same way ``run_teleop.py`` does, then attempts the
``kitchen_sorting_gr1t2_env_cfg`` import **line-by-line**, printing each
upstream package's import status to both stdout and stderr with forced
flushing.  Use this to pinpoint which dependency in the env_cfg chain
raises during Isaac Sim startup when ``_register()`` otherwise returns
False silently.

Run::

    python -m ust_ws.ust_hm_glove.scripts.diagnose_env_cfg
"""

from __future__ import annotations

import argparse
import os
import sys
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.environ.setdefault("IPC_IGNORE_VERSION", "1")


try:
    import pinocchio  # noqa: F401
except Exception:
    pass

try:
    import h5py  # noqa: F401
except Exception:
    pass

# osqp ↔ qpsolvers SolverStatus shim (see _osqp_compat module docstring).
try:
    from ust_ws.ust_hm_glove.teleop import _osqp_compat  # noqa: F401
    _osqp_compat.apply()
except Exception:
    pass


def _say(msg: str, *, err: bool = False) -> None:
    """Print to BOTH stdout and stderr so at least one channel reaches the user."""
    out = sys.stderr if err else sys.stdout
    out.write(msg + "\n")
    out.flush()
    # Mirror to the other stream too (carb may redirect one but not both).
    other = sys.stdout if err else sys.stderr
    try:
        other.write(msg + "\n")
        other.flush()
    except Exception:
        pass


def _probe(label: str, code: str, ns: dict) -> bool:
    _say(f"[diagnose] trying: {label}")
    try:
        exec(code, ns)
    except BaseException as exc:  # noqa: BLE001
        _say(f"[diagnose] FAILED : {label} → {type(exc).__name__}: {exc}", err=True)
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()
        return False
    _say(f"[diagnose] OK     : {label}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="GR1T2 env_cfg import diagnostic")
    # Let AppLauncher add its own args (headless, device, etc.)
    from isaaclab.app import AppLauncher  # type: ignore
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    # Force headless for this diagnostic.
    args.headless = True

    app_launcher = AppLauncher(args)
    app = app_launcher.app

    _say("=" * 70)
    _say("GR1T2 env_cfg import diagnostic — Isaac Sim booted successfully.")
    _say("=" * 70)

    ns: dict = {}
    probes = [
        ("carb", "import carb"),
        ("pink.tasks.DampingTask+FrameTask", "from pink.tasks import DampingTask, FrameTask"),
        ("isaaclab.controllers.pink_ik", "from isaaclab.controllers.pink_ik import NullSpacePostureTask, PinkIKControllerCfg"),
        ("isaaclab.controllers.utils", "import isaaclab.controllers.utils as ControllerUtils"),
        ("isaaclab.sim", "import isaaclab.sim as sim_utils"),
        ("isaaclab.assets.ArticulationCfg", "from isaaclab.assets import ArticulationCfg"),
        ("isaaclab.devices.device_base.DevicesCfg", "from isaaclab.devices.device_base import DevicesCfg"),
        ("isaaclab.devices.openxr", "from isaaclab.devices.openxr import ManusViveCfg, OpenXRDeviceCfg, XrCfg"),
        ("GR1T2RetargeterCfg", "from isaaclab.devices.openxr.retargeters.humanoid.fourier.gr1t2_retargeter import GR1T2RetargeterCfg"),
        ("isaaclab.envs.ManagerBasedRLEnvCfg", "from isaaclab.envs import ManagerBasedRLEnvCfg"),
        ("PinkInverseKinematicsActionCfg", "from isaaclab.envs.mdp.actions.pink_actions_cfg import PinkInverseKinematicsActionCfg"),
        ("isaaclab.utils.configclass", "from isaaclab.utils import configclass"),
        ("isaaclab_assets.robots.fourier.GR1T2_HIGH_PD_CFG", "from isaaclab_assets.robots.fourier import GR1T2_HIGH_PD_CFG"),
        ("ust_260220 kitchen_sorting_env_cfg scene/obs/rewards/etc", (
            "from ust_ws.ust_260220.kitchen_sorting_env_cfg import ("
            " EventCfg, KitchenSortingSceneCfg, KitchenSortingUSDSceneCfg,"
            " KitchenSortingVisionSceneCfg, ObservationsCfg, RewardsCfg,"
            " TerminationsCfg, USDEventCfg, USDObservationsCfg, USDRewardsCfg,"
            " USDTerminationsCfg)"
        )),
        ("ust_hm_glove.teleop.gr1t2_udcap_device (Cfg)", "from ust_ws.ust_hm_glove.teleop.gr1t2_udcap_device import GR1T2FourierUDCAPDeviceCfg"),
        ("kitchen_sorting_gr1t2_env_cfg module", "import ust_ws.ust_hm_glove.kitchen_sorting_gr1t2_env_cfg as cfg_mod"),
        ("KitchenSortingGR1T2EnvCfg class", "from ust_ws.ust_hm_glove.kitchen_sorting_gr1t2_env_cfg import KitchenSortingGR1T2EnvCfg"),
        ("KitchenSortingGR1T2WaistEnvCfg class", "from ust_ws.ust_hm_glove.kitchen_sorting_gr1t2_env_cfg import KitchenSortingGR1T2WaistEnvCfg"),
    ]

    failures = 0
    for label, code in probes:
        if not _probe(label, code, ns):
            failures += 1
            _say(
                f"[diagnose] → stopping at first failure ({label}).\n"
                f"[diagnose] Fix the above exception and re-run.",
                err=True,
            )
            break

    _say("=" * 70)
    if failures == 0:
        _say("[diagnose] ALL IMPORTS PASSED.")
        _say("[diagnose] Now trying gym registration side-effect…")
        _probe("ust_hm_glove package (register envs)", "import ust_ws.ust_hm_glove", ns)
        try:
            import gymnasium as gym
            gr1t2_ids = sorted(k for k in gym.registry.keys() if "GR1T2" in k)
            _say(f"[diagnose] Registered GR1T2 env IDs: {gr1t2_ids}")
            if not gr1t2_ids:
                _say(
                    "[diagnose] NO env IDs registered — check __init__.py::_register() logic.",
                    err=True,
                )
                failures = 1
        except Exception as exc:
            _say(f"[diagnose] gym registry probe failed: {exc}", err=True)
            failures = 1
    else:
        _say(f"[diagnose] FAILED — {failures} import error(s). See traceback above.", err=True)
    _say("=" * 70)

    app.close()
    return 0 if failures == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
