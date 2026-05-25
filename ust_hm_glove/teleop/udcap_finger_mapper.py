"""UDCAP VMC finger mapper (Path B fallback).

Mirrors ``ust_260220/teleop/udcap_finger_mapper.py``.  The mapping reduces
UDCAP VMC bone quaternions to the Inspire 12-DOF / hand channel layout
used by the existing Pink IK hand action (see `g1_retargeter.py`).

The Pink IK env expects a flat 24D vector (left + right) when the
retargeter is driving Inspire FTP — the convenience
``pack_24d(left12, right12)`` helper is provided so the device and
retargeter agree on the concatenation order.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Tuple

import numpy as np


__all__ = [
    "UDCAPFingerMapper",
    "INSPIRE_JOINT_DIM",
    "THUMB_BEND",
    "THUMB_ROTATION",
    "INDEX_BEND",
    "INDEX_SPREAD",
    "MIDDLE_BEND",
    "MIDDLE_SPREAD",
    "RING_BEND",
    "RING_SPREAD",
    "LITTLE_BEND",
    "LITTLE_SPREAD",
    "THUMB_OPPOSITION",
    "WRIST_ROTATION",
    "pack_24d",
]


INSPIRE_JOINT_DIM = 12

THUMB_BEND = 0
THUMB_ROTATION = 1
INDEX_BEND = 2
INDEX_SPREAD = 3
MIDDLE_BEND = 4
MIDDLE_SPREAD = 5
RING_BEND = 6
RING_SPREAD = 7
LITTLE_BEND = 8
LITTLE_SPREAD = 9
THUMB_OPPOSITION = 10
WRIST_ROTATION = 11


def _quat_to_bend(qx: float, qy: float, qz: float, qw: float) -> float:
    angle = 2.0 * math.acos(max(-1.0, min(1.0, abs(qw))))
    return min(angle / math.pi, 1.0)


def _quat_to_spread(qx: float, qy: float, qz: float, qw: float) -> float:
    spread = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return max(-1.0, min(1.0, spread / (math.pi / 4)))


def _quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> float:
    rot = math.atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz))
    return max(-1.0, min(1.0, rot / (math.pi / 2)))


class UDCAPFingerMapper:
    """Map VMC bone quaternions to Inspire 12D per hand (numpy-first)."""

    def __init__(self, bend_scale: float = 1.0, spread_scale: float = 0.5):
        self.bend_scale = float(bend_scale)
        self.spread_scale = float(spread_scale)

    # ── primary entry: VMC (UDCAP Path B) ──
    def map_hand(
        self,
        bones: Dict[str, Tuple[float, float, float, float]],
        is_right: bool,
    ) -> np.ndarray:
        out = np.zeros(INSPIRE_JOINT_DIM, dtype=np.float32)
        side = "Right" if is_right else "Left"

        # Thumb supports both naming conventions; see fourier_hand_mapper.py
        # ``map_hand_vmc`` for the rationale.  UDCAP 0.1.8.2 uses Unity
        # convention (ThumbProximal/Intermediate/Distal); some other VMC
        # broadcasters use the anatomy convention (ThumbMetacarpal/
        # Proximal/Distal).  Probe ThumbIntermediate to disambiguate.
        thumb_inter = bones.get(f"{side}ThumbIntermediate")
        if thumb_inter is not None:
            # Unity convention: Proximal=CMC (rotation/opposition),
            # Intermediate=MCP (bend), Distal=IP.
            thumb_cmc = bones.get(f"{side}ThumbProximal")
            thumb_mcp = thumb_inter
        else:
            # Anatomy convention: Metacarpal=CMC, Proximal=MCP, Distal=IP.
            thumb_cmc = bones.get(f"{side}ThumbMetacarpal")
            thumb_mcp = bones.get(f"{side}ThumbProximal")
        if thumb_mcp is not None:
            out[THUMB_BEND] = _quat_to_bend(*thumb_mcp) * self.bend_scale
        if thumb_cmc is not None:
            out[THUMB_ROTATION] = _quat_to_rotation(*thumb_cmc)
            out[THUMB_OPPOSITION] = _quat_to_bend(*thumb_cmc) * self.bend_scale

        for finger, bend_ix, spread_ix in (
            ("Index", INDEX_BEND, INDEX_SPREAD),
            ("Middle", MIDDLE_BEND, MIDDLE_SPREAD),
            ("Ring", RING_BEND, RING_SPREAD),
            ("Little", LITTLE_BEND, LITTLE_SPREAD),
        ):
            prox = bones.get(f"{side}{finger}Proximal")
            if prox is None:
                continue
            out[bend_ix] = _quat_to_bend(*prox) * self.bend_scale
            out[spread_ix] = _quat_to_spread(*prox) * self.spread_scale
        return out

    # ── fallback: SteamVR Skeletal 31-bone (model space) ──
    def map_from_skeletal(
        self,
        bones_model_frame: np.ndarray,
        is_right: bool,
        finger_curls: Optional[Iterable[float]] = None,
    ) -> np.ndarray:
        """Derive the 12D vector from the SteamVR skeletal bones.

        Uses per-finger proximal bone orientations (indices 2, 6, 11, 16, 21)
        for bend/spread values and ``fingerCurls`` as a sanity bias on the
        thumb/opposition channels.
        """
        arr = np.asarray(bones_model_frame, dtype=np.float64)
        out = np.zeros(INSPIRE_JOINT_DIM, dtype=np.float32)
        finger_proximals = {
            "Thumb": 2,   # thumb0
            "Index": 6,
            "Middle": 11,
            "Ring": 16,
            "Little": 21,
        }
        if arr.shape[0] < max(finger_proximals.values()) + 1:
            return out

        def _row_to_quat(row: np.ndarray) -> Tuple[float, float, float, float]:
            # bones_model_frame rows are [px, py, pz, qw, qx, qy, qz]
            return float(row[4]), float(row[5]), float(row[6]), float(row[3])

        qx, qy, qz, qw = _row_to_quat(arr[finger_proximals["Thumb"]])
        out[THUMB_BEND] = _quat_to_bend(qx, qy, qz, qw) * self.bend_scale
        out[THUMB_ROTATION] = _quat_to_rotation(qx, qy, qz, qw)
        out[THUMB_OPPOSITION] = _quat_to_bend(qx, qy, qz, qw) * self.bend_scale

        for finger, bend_ix, spread_ix in (
            ("Index", INDEX_BEND, INDEX_SPREAD),
            ("Middle", MIDDLE_BEND, MIDDLE_SPREAD),
            ("Ring", RING_BEND, RING_SPREAD),
            ("Little", LITTLE_BEND, LITTLE_SPREAD),
        ):
            qx, qy, qz, qw = _row_to_quat(arr[finger_proximals[finger]])
            out[bend_ix] = _quat_to_bend(qx, qy, qz, qw) * self.bend_scale
            out[spread_ix] = _quat_to_spread(qx, qy, qz, qw) * self.spread_scale

        if finger_curls is not None:
            curls = list(finger_curls)
            if len(curls) >= 5:
                # Blend the animation-derived curl into the bend channel —
                # helps when model-frame quats are noisy near full fist.
                out[THUMB_BEND] = 0.5 * out[THUMB_BEND] + 0.5 * float(curls[0])
                out[INDEX_BEND] = 0.5 * out[INDEX_BEND] + 0.5 * float(curls[1])
                out[MIDDLE_BEND] = 0.5 * out[MIDDLE_BEND] + 0.5 * float(curls[2])
                out[RING_BEND] = 0.5 * out[RING_BEND] + 0.5 * float(curls[3])
                out[LITTLE_BEND] = 0.5 * out[LITTLE_BEND] + 0.5 * float(curls[4])

        # Suppress the rare NaN that slips in when UDCAP sends a zero quat.
        out = np.nan_to_num(out, nan=0.0, posinf=1.0, neginf=-1.0)
        _ = is_right  # reserved for future side-specific tweaks (mirror thumb)
        return out


def pack_24d(left_12: np.ndarray, right_12: np.ndarray) -> np.ndarray:
    """Concatenate left then right 12D vectors into a single 24D numpy array."""
    left = np.asarray(left_12, dtype=np.float32).reshape(-1)
    right = np.asarray(right_12, dtype=np.float32).reshape(-1)
    if left.size != INSPIRE_JOINT_DIM or right.size != INSPIRE_JOINT_DIM:
        raise ValueError(
            f"expected two length-{INSPIRE_JOINT_DIM} vectors, got {left.size} and {right.size}"
        )
    return np.concatenate([left, right], axis=0)
