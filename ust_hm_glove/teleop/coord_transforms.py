"""Coordinate transforms between SteamVR and Isaac Lab conventions.

Reference: `research/28. SteamVR 휴머노이드 텔레오퍼레이션 구현 가이드` §5.2, §5.3.

Conventions
-----------
SteamVR world (TrackingUniverseStanding):
    +X right, +Y up, -Z forward   (right-handed, Y-up)
Isaac Lab / USD / ROS REP-103:
    +X forward, +Y left, +Z up    (right-handed, Z-up)

The transform between the two bases is a fixed rotation matrix
``R_svr2il = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]`` which maps a SteamVR
position ``(sx, sy, sz)`` to Isaac Lab ``(-sz, -sx, sy)``.

The forearm-to-wrist recovery (§5.2) uses the fact that the Pico Enhanced
Forearm tracker sits roughly at the elbow; the wrist is ``offset_local``
metres along the tracker's local +X axis.
"""

from __future__ import annotations

import math
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np


__all__ = [
    "R_SVR2IL",
    "R_SVR2IL_QUAT_WXYZ",
    "svr_to_isaaclab",
    "svr_to_isaaclab_array",
    "forearm_to_wrist",
    "quat_wxyz_to_matrix",
    "matrix_to_quat_wxyz",
    "quat_multiply",
    "quat_rotate_vec",
    "normalise_quat",
]


# SteamVR -> Isaac Lab basis change (column = SVR axis expressed in IL coords).
R_SVR2IL: np.ndarray = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
)


def _vec3(v: Any) -> np.ndarray:
    arr = np.asarray(v, dtype=np.float64).reshape(-1)
    if arr.size < 3:
        raise ValueError(f"expected length-3 position, got {arr.size}")
    return arr[:3]


def _quat_wxyz(q: Any) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64).reshape(-1)
    if arr.size < 4:
        raise ValueError(f"expected length-4 quaternion, got {arr.size}")
    return arr[:4]


def normalise_quat(q: Sequence[float]) -> np.ndarray:
    """Normalise a quaternion in place-safe fashion; returns a new array."""
    arr = _quat_wxyz(q)
    n = float(np.linalg.norm(arr))
    if n < 1e-12:
        out = np.zeros(4, dtype=np.float64)
        out[0] = 1.0
        return out
    return arr / n


def quat_wxyz_to_matrix(q: Sequence[float]) -> np.ndarray:
    """Convert an (w, x, y, z) quaternion to a 3x3 rotation matrix."""
    w, x, y, z = normalise_quat(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to an (w, x, y, z) quaternion."""
    m = np.asarray(R, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return normalise_quat(np.array([w, x, y, z]))


def quat_multiply(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    """Hamilton product (w, x, y, z)."""
    aw, ax, ay, az = _quat_wxyz(a)
    bw, bx, by, bz = _quat_wxyz(b)
    return normalise_quat(
        np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ],
            dtype=np.float64,
        )
    )


def quat_rotate_vec(q: Sequence[float], v: Sequence[float]) -> np.ndarray:
    """Apply rotation ``q`` (wxyz) to 3-vector ``v``."""
    return quat_wxyz_to_matrix(q) @ _vec3(v)


# ── SteamVR <-> Isaac Lab ───────────────────────────────────────────────────
# Expressing SVR basis vectors in the Isaac Lab frame gives R_SVR2IL.
# The corresponding (wxyz) quaternion is precomputed once.
R_SVR2IL_QUAT_WXYZ: np.ndarray = matrix_to_quat_wxyz(R_SVR2IL)


def svr_to_isaaclab(
    pos: Sequence[float], quat_wxyz: Sequence[float]
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert a SteamVR pose (pos, wxyz quat) to Isaac Lab coordinates.

    Applies both a basis change of the position and a left/right conjugation
    of the orientation so that roll/pitch/yaw read naturally in the target
    frame.

    Parameters
    ----------
    pos:
        3-vector in SteamVR world coordinates (metres).
    quat_wxyz:
        4-vector (w, x, y, z) representing the tracker's orientation.

    Returns
    -------
    (pos_il, quat_il):
        Pose expressed in Isaac Lab / ROS REP-103 coordinates.  The returned
        quaternion remains ``(w, x, y, z)``.
    """
    p = _vec3(pos)
    # ROS_X = -SVR_Z, ROS_Y = -SVR_X, ROS_Z = SVR_Y  (see doc 28 §5.3)
    p_il = np.array([-p[2], -p[0], p[1]], dtype=np.float64)

    # q_out = q_frame * q_device * q_frame^-1
    q_frame = R_SVR2IL_QUAT_WXYZ
    q_frame_inv = np.array([q_frame[0], -q_frame[1], -q_frame[2], -q_frame[3]])
    q_il = quat_multiply(quat_multiply(q_frame, quat_wxyz), q_frame_inv)
    return p_il, q_il


def svr_to_isaaclab_array(poses: np.ndarray) -> np.ndarray:
    """Vectorised `svr_to_isaaclab` for an (N, 7) array (pos3 + quat wxyz)."""
    poses = np.asarray(poses, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1] != 7:
        raise ValueError(f"expected (N, 7) array, got {poses.shape}")
    out = np.empty_like(poses)
    for i in range(poses.shape[0]):
        p, q = svr_to_isaaclab(poses[i, :3], poses[i, 3:7])
        out[i, :3] = p
        out[i, 3:7] = q
    return out


# ── Forearm -> wrist recovery ───────────────────────────────────────────────


def forearm_to_wrist(
    forearm_pose: Mapping[str, Any],
    forearm_to_wrist_offset: Sequence[float] = (0.12, 0.0, 0.0),
    glove_rel_quat_wxyz: Optional[Sequence[float]] = None,
) -> dict:
    """Synthesize a 6-DoF wrist pose from a forearm tracker pose.

    The UDCAP VR Glove has no absolute positional tracking; the Enhanced
    Forearm tracker sits near the elbow and its local +X points toward the
    wrist.  The wrist position is therefore ``T_forearm * offset_local``.
    The wrist rotation defaults to the forearm rotation but may be composed
    with a glove-relative rotation (wxyz).

    Parameters
    ----------
    forearm_pose:
        Mapping with keys ``"pos"`` (3,) and ``"quat"`` (4,), quaternion in
        (w, x, y, z).  ``SteamVRSampler`` snapshots use this layout.
    forearm_to_wrist_offset:
        Offset from tracker to wrist in the tracker's local frame.  Typical
        values are 0.10–0.15 m along +X, calibrated per user.
    glove_rel_quat_wxyz:
        Optional relative rotation from the SteamVR Skeletal wrist bone;
        when provided, the output wrist quaternion is
        ``q_forearm * q_glove``.

    Returns
    -------
    dict with ``"pos"`` (np.ndarray (3,)) and ``"quat"`` (np.ndarray (4,) wxyz).
    """
    pos = _vec3(forearm_pose["pos"])
    quat = _quat_wxyz(forearm_pose["quat"])
    offset = _vec3(forearm_to_wrist_offset)

    R = quat_wxyz_to_matrix(quat)
    wrist_pos = pos + R @ offset

    if glove_rel_quat_wxyz is None:
        wrist_quat = normalise_quat(quat)
    else:
        wrist_quat = quat_multiply(quat, glove_rel_quat_wxyz)

    return {"pos": wrist_pos, "quat": wrist_quat}
