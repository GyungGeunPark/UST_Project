"""Per-finger isolation tester.

Drives the FourierHandMapper through 10 isolated single-finger fixtures
(``L_index_only``, ``L_middle_only``, …, ``R_thumb_only``) and reports
which OTHER finger slots showed crosstalk above threshold.

Used to verify per-finger isolation after any mapper change.

Usage::

    python -m ust_ws.ust_hm_glove.validation.scripts.run_per_finger_isolation \\
        [--threshold 0.05]

Output (verbose) example::

    L_index_only:
      target finger Δ = 1.332 rad ✓
      crosstalk:
        - L_middle_prox 0.000  ✓ clean
        - R_index_prox  0.000  ✓ clean
        ...
      isolation: 22/22 slots clean

This is Layer-1 — runs in seconds without Isaac Sim.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Dict, List, Tuple

import numpy as np


_PER_SIDE_LABELS = (
    "index_prox", "middle_prox", "pinky_prox", "ring_prox", "thumb_yaw",
    "index_int",  "middle_int",  "pinky_int",  "ring_int",  "thumb_pitch",
    "thumb_dist",
)

_FINGERS = ("Index", "Middle", "Ring", "Little")
_THUMB = "Thumb"
_FINGER_PARTS = ("Proximal", "Intermediate", "Distal")
_THUMB_PARTS = ("Proximal", "Intermediate", "Distal")


def _build_isolated_pose(target_side: str, target_finger: str,
                         flex_deg: float = 80.0,
                         thumb_yaw_deg: float = 30.0,
                         thumb_pitch_deg: float = 50.0) -> Dict[str, Tuple[float, float, float, float]]:
    """All bones identity except the chosen finger, which is flexed."""
    pose: Dict[str, Tuple[float, float, float, float]] = {}
    for side in ("Left", "Right"):
        for finger in _FINGERS:
            for part in _FINGER_PARTS:
                pose[f"{side}{finger}{part}"] = (0.0, 0.0, 0.0, 1.0)
        for part in _THUMB_PARTS:
            pose[f"{side}{_THUMB}{part}"] = (0.0, 0.0, 0.0, 1.0)
    # Flex the target finger
    half = math.radians(flex_deg) * 0.5
    if target_finger == "Thumb":
        # Thumb gets yaw on Proximal, pitch on Intermediate.
        ya = math.radians(thumb_yaw_deg) * 0.5
        pa = math.radians(thumb_pitch_deg) * 0.5
        pose[f"{target_side}{_THUMB}Proximal"] = (0.0, 0.0, math.sin(ya), math.cos(ya))
        pose[f"{target_side}{_THUMB}Intermediate"] = (math.sin(pa), 0.0, 0.0, math.cos(pa))
    else:
        for part in _FINGER_PARTS:
            pose[f"{target_side}{target_finger}{part}"] = (
                math.sin(half), 0.0, 0.0, math.cos(half),
            )
    return pose


def _check_isolation(side: str, finger: str, threshold: float) -> Tuple[bool, List[str]]:
    """Run mapper on isolated pose; return (target_active, list of crosstalk slots)."""
    from ust_ws.ust_hm_glove.validation._bootstrap import load_fourier_hand_mapper
    fhm = load_fourier_hand_mapper()
    mapper = fhm.FourierHandMapper(
        proximal_scale=2.5, thumb_scale=2.5,
        use_tanh=True, vmc_subtract_rest=False,
    )
    pose = _build_isolated_pose(side, finger)
    L = np.asarray(mapper.map_hand_vmc(pose, is_right=False), dtype=np.float64)
    R = np.asarray(mapper.map_hand_vmc(pose, is_right=True),  dtype=np.float64)

    # For each slot in (Left, Right), determine whether it should be active.
    is_left = (side == "Left")
    slot_targets: List[Tuple[str, float, bool]] = []   # (label, value, expected_active)
    for s_name, vec in (("L", L), ("R", R)):
        for j, lbl in enumerate(_PER_SIDE_LABELS):
            v = float(vec[j])
            slot_label = f"{s_name}_{lbl}"
            # Expected active only on the target side AND for the target finger
            on_target_side = (s_name == "L" and is_left) or (s_name == "R" and not is_left)
            if not on_target_side:
                expected_active = False
            else:
                if finger == "Index":
                    expected_active = lbl in ("index_prox", "index_int")
                elif finger == "Middle":
                    expected_active = lbl in ("middle_prox", "middle_int")
                elif finger == "Ring":
                    expected_active = lbl in ("ring_prox", "ring_int")
                elif finger == "Little":
                    expected_active = lbl in ("pinky_prox", "pinky_int")
                elif finger == "Thumb":
                    expected_active = lbl in ("thumb_yaw", "thumb_pitch", "thumb_dist")
                else:
                    expected_active = False
            slot_targets.append((slot_label, v, expected_active))

    # Crosstalk: any expected_active=False slot with |v| > threshold.
    crosstalk = [
        f"{lbl} {v:+.3f}" for (lbl, v, exp) in slot_targets
        if not exp and abs(v) > threshold
    ]
    target_max = max(
        (abs(v) for (_, v, exp) in slot_targets if exp),
        default=0.0,
    )
    return target_max > 0.3, crosstalk


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--threshold", type=float, default=0.05,
                    help="Crosstalk tolerance per slot (rad). Default 0.05 (~3°).")
    args = ap.parse_args()

    sides = ("Left", "Right")
    fingers = ("Index", "Middle", "Ring", "Little", "Thumb")
    n_pass, n_fail = 0, 0
    for side in sides:
        for finger in fingers:
            label = f"{side[0]}_{finger.lower()}_only"
            target_ok, crosstalk = _check_isolation(side, finger, args.threshold)
            if target_ok and not crosstalk:
                print(f"  PASS  {label:<18} (target ≥0.3 rad, no crosstalk)")
                n_pass += 1
            else:
                print(f"  FAIL  {label:<18}")
                if not target_ok:
                    print(f"        target finger barely activated")
                for ct in crosstalk:
                    print(f"        crosstalk → {ct}")
                n_fail += 1
    print()
    print(f"=== TOTAL: {n_pass} pass, {n_fail} fail ===")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
