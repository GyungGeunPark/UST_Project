"""Validate the Fourier dex-retargeting YAML / URDF pipeline.

Runs *without* Isaac Sim.  Loads both ``fourier_*_dexpilot.yml`` files,
invokes the DexPilot optimizer on a handful of synthetic fingertip
configurations and prints summary statistics for each.  Intended as a
quick health check before a VR session.

A mandatory argument is the URDF directory (i.e. where
``GR1_T2_left_hand.urdf`` / ``GR1_T2_right_hand.urdf`` live).  The URDFs
must be generated via Isaac Lab's ``ControllerUtils.convert_usd_to_urdf``
beforehand — this script does not launch Isaac Sim.

Usage::

    python -m ust_ws.ust_hm_glove.scripts.validate_fourier_dex \
        --urdf_dir C:/tmp/fourier_hand_urdfs
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


_POSE_TPOSE = np.array(
    [
        [0.00, 0.00, 0.00],   # wrist
        [0.03, 0.04, -0.02],  # thumb tip
        [0.10, 0.01, 0.00],   # index tip
        [0.10, -0.01, 0.00],  # middle tip
        [0.09, -0.03, 0.00],  # ring tip
        [0.08, -0.05, 0.00],  # pinky tip
    ],
    dtype=np.float64,
)


_POSE_FIST = np.array(
    [
        [0.00, 0.00, 0.00],
        [0.04, 0.02, 0.01],
        [0.05, 0.00, -0.02],
        [0.05, -0.02, -0.02],
        [0.05, -0.03, -0.02],
        [0.05, -0.04, -0.02],
    ],
    dtype=np.float64,
)


_POSE_PINCH = np.array(
    [
        [0.00, 0.00, 0.00],
        [0.06, 0.00, -0.01],
        [0.07, 0.00, -0.01],  # index tip meets thumb tip
        [0.10, -0.02, 0.00],
        [0.09, -0.03, 0.00],
        [0.08, -0.05, 0.00],
    ],
    dtype=np.float64,
)


_CASES = {
    "t-pose": _POSE_TPOSE,
    "fist": _POSE_FIST,
    "pinch": _POSE_PINCH,
}


def _check_yaml(
    yaml_path: Path,
    urdf_dir: Path,
) -> Dict[str, Any]:
    from dex_retargeting.retargeting_config import RetargetingConfig  # type: ignore

    RetargetingConfig.set_default_urdf_dir(str(urdf_dir))
    cfg = RetargetingConfig.load_from_file(str(yaml_path))
    optimizer = cfg.build()

    target_joints = list(getattr(optimizer, "target_joint_names", []))
    results: List[Dict[str, Any]] = []
    for label, tips in _CASES.items():
        try:
            q = np.asarray(optimizer.retarget(tips), dtype=np.float64).reshape(-1)
            results.append(
                {
                    "case": label,
                    "status": "ok",
                    "n_joints": int(q.size),
                    "min": float(q.min()),
                    "max": float(q.max()),
                    "any_nan": bool(np.any(np.isnan(q))),
                }
            )
        except Exception as exc:  # noqa: BLE001
            results.append({"case": label, "status": f"fail: {exc!r}"})
    return {
        "yaml": str(yaml_path),
        "n_target_joints": len(target_joints),
        "target_joints": target_joints,
        "cases": results,
    }


def _parse_args():
    p = argparse.ArgumentParser(description="Fourier dex-retargeting pipeline validator")
    p.add_argument(
        "--urdf_dir",
        type=str,
        required=True,
        help="Directory containing GR1_T2_{left,right}_hand.urdf (from convert_usd_to_urdf).",
    )
    p.add_argument(
        "--left_yaml",
        type=str,
        default=str(
            ROOT / "ust_ws" / "ust_hm_glove" / "config" / "dex_retargeting" / "fourier_left_dexpilot.yml"
        ),
    )
    p.add_argument(
        "--right_yaml",
        type=str,
        default=str(
            ROOT / "ust_ws" / "ust_hm_glove" / "config" / "dex_retargeting" / "fourier_right_dexpilot.yml"
        ),
    )
    return p.parse_args()


def main() -> int:
    try:
        import dex_retargeting  # noqa: F401 — import for version check
    except Exception as exc:  # noqa: BLE001
        print(f"[validate_fourier_dex] FAIL — dex-retargeting missing: {exc}")
        return 1

    args = _parse_args()
    urdf_dir = Path(args.urdf_dir).resolve()
    if not urdf_dir.is_dir():
        print(f"[validate_fourier_dex] FAIL — --urdf_dir not a directory: {urdf_dir}")
        return 1

    failed = 0
    for side, yaml_path in (("left", args.left_yaml), ("right", args.right_yaml)):
        path = Path(yaml_path)
        if not path.is_file():
            print(f"[validate_fourier_dex] FAIL — missing YAML: {path}")
            failed += 1
            continue
        try:
            report = _check_yaml(path, urdf_dir)
        except Exception as exc:  # noqa: BLE001
            print(f"[validate_fourier_dex] FAIL {side}: {exc!r}")
            failed += 1
            continue
        ok_cases = sum(1 for c in report["cases"] if c.get("status") == "ok")
        total = len(report["cases"])
        print(
            f"[validate_fourier_dex] {side.upper():<5s}  YAML={path.name}  "
            f"target_joints={report['n_target_joints']}  cases={ok_cases}/{total} ok"
        )
        for c in report["cases"]:
            status = c.get("status")
            if status == "ok":
                print(
                    f"    - {c['case']:<7s} n={c['n_joints']:<2d}  "
                    f"min={c['min']:+.3f}  max={c['max']:+.3f}  nan={c['any_nan']}"
                )
            else:
                print(f"    - {c['case']:<7s} {status}")
        if ok_cases != total:
            failed += 1

    print(f"[validate_fourier_dex] {'OK' if failed == 0 else 'FAILED'}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
