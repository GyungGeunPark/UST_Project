"""Bootstrap helper to load ``ust_hm_glove.fourier_hand_mapper`` module.

The package's ``__init__.py`` pulls in ``gr1t2_retargeter`` which requires
``torch`` + ``isaaclab``.  Layer-1 unit tests should not need either.
This helper does a *direct file* load so the mapper is usable from a bare
Python interpreter (numpy only).

When running inside the full ``ust`` conda environment (Isaac Lab + torch
present) the standard package import path is used because it is faster
and shares state with the production module.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import types
from pathlib import Path
from typing import Any


_REPO_ROOT = Path(__file__).resolve().parents[2]   # .../IsaacLab
_MAPPER_PATH = _REPO_ROOT / "ust_ws" / "ust_hm_glove" / "teleop" / "fourier_hand_mapper.py"


def load_fourier_hand_mapper() -> types.ModuleType:
    """Return the ``fourier_hand_mapper`` module, package-import preferred.

    Raises
    ------
    FileNotFoundError
        When the production module is missing from disk.
    """
    # Prefer the standard package path when it loads cleanly.
    try:
        from ust_ws.ust_hm_glove.teleop import fourier_hand_mapper as _m
        return _m
    except Exception:                                     # noqa: BLE001
        pass

    if not _MAPPER_PATH.is_file():
        raise FileNotFoundError(
            f"fourier_hand_mapper.py not found at {_MAPPER_PATH}.  "
            "Cannot run ust_hm_glove.validation without ust_hm_glove."
        )

    spec = importlib.util.spec_from_file_location(
        "ust_hm_glove.validation._fourier_hand_mapper_direct", str(_MAPPER_PATH)
    )
    if spec is None or spec.loader is None:               # pragma: no cover
        raise ImportError(f"Cannot create spec for {_MAPPER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def repo_root() -> Path:
    """Absolute path to the IsaacLab repo root (parent of ``ust_ws``)."""
    return _REPO_ROOT


def package_root() -> Path:
    """Absolute path to ``ust_ws/ust_hm_glove/validation``."""
    return Path(__file__).resolve().parent
