"""Extract 5 fingertip positions from a 31-bone SteamVR Skeletal pose.

Reference: `research/28 …` §2.3.1 / §5.4(b).  The Skeletal 2.0 bone order
(see Valve's ``steamvr_skeleton_hand.json``) is::

    0  root               1  wrist
    2  thumb0             3  thumb1      4  thumb2     5  thumb3       (tip)
    6  index0    7  index1    8  index2    9  index3   10 index4       (tip)
    11 middle0  12 middle1   13 middle2   14 middle3   15 middle4      (tip)
    16 ring0    17 ring1     18 ring2     19 ring3     20 ring4        (tip)
    21 pinky0   22 pinky1    23 pinky2    24 pinky3    25 pinky4       (tip)
    26 aux_thumb  27 aux_index  28 aux_middle  29 aux_ring  30 aux_pinky

When the sampler queries ``VRSkeletalTransformSpace_Model`` the returned
bone positions are expressed in the hand's *model* frame — i.e. the wrist
is at the origin with +X pointing down the middle finger.  This is
exactly the input format that ``dex-retargeting``'s DexPilot optimizer
expects.

For consistency with our retargeting code we return an ``(5, 3)`` array
ordered as ``[thumb, index, middle, ring, pinky]``.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


# (thumb3, index4, middle4, ring4, pinky4) — model-frame tip bones.
STEAMVR_TIP_BONE_INDICES: Sequence[int] = (5, 10, 15, 20, 25)


__all__ = [
    "STEAMVR_TIP_BONE_INDICES",
    "extract_fingertips_wrist_frame",
    "extract_fingertips_with_wrist",
]


def _require_31x7(bones: np.ndarray) -> np.ndarray:
    arr = np.asarray(bones, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(
            f"expected (N, 7) bone array (pos + wxyz quat), got {arr.shape}"
        )
    return arr


def extract_fingertips_wrist_frame(
    bones_model_frame: np.ndarray,
    tip_indices: Sequence[int] = STEAMVR_TIP_BONE_INDICES,
) -> np.ndarray:
    """Return fingertip positions (metres) in the hand-model (wrist) frame.

    Parameters
    ----------
    bones_model_frame:
        ``(31, 7)`` array from ``SteamVRSampler``.  Columns are
        ``[px, py, pz, qw, qx, qy, qz]``; only the positions are used.
    tip_indices:
        Bone indices to pick up as fingertips.  Defaults to the last bone of
        each finger chain.

    Returns
    -------
    ``(5, 3)`` array ordered thumb → pinky.  If the input has fewer bones
    than requested, the missing rows are filled with zeros.
    """
    arr = _require_31x7(bones_model_frame)
    tips = np.zeros((len(tip_indices), 3), dtype=np.float64)
    for out_row, idx in enumerate(tip_indices):
        if 0 <= idx < arr.shape[0]:
            tips[out_row] = arr[idx, :3]
    return tips


def extract_fingertips_with_wrist(
    bones_model_frame: np.ndarray,
    tip_indices: Sequence[int] = STEAMVR_TIP_BONE_INDICES,
) -> np.ndarray:
    """Same as `extract_fingertips_wrist_frame` but also returns the wrist
    bone (row 0 of the output) so DexPilot-style optimizers that require a
    fixed origin still have a reference point."""
    arr = _require_31x7(bones_model_frame)
    rows = [np.zeros(3, dtype=np.float64)]  # wrist at origin by convention
    for idx in tip_indices:
        if 0 <= idx < arr.shape[0]:
            rows.append(arr[idx, :3])
        else:
            rows.append(np.zeros(3, dtype=np.float64))
    return np.asarray(rows, dtype=np.float64)
