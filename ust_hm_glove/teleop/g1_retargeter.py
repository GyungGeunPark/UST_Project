"""SteamVR snapshot → G1 Inspire-FTP Pink IK 38D action.

This is the Windows/SteamVR port of ``ust_260220/teleop/g1_retargeter.py``.
The coordinate conversion ``XRT Z-in → USD Z-out`` is replaced by
``SteamVR Y-up → Isaac Lab Z-up`` (see ``coord_transforms.svr_to_isaaclab``),
and the raw input is the ``SteamVRSampler`` snapshot instead of an
``XRTFrame``.

The 38D output layout is unchanged:

    [0:3]    left  EEF position  (Isaac Lab frame, metres)
    [3:7]    left  EEF quaternion (wxyz)
    [7:10]   right EEF position
    [10:14]  right EEF quaternion (wxyz)
    [14:38]  24 hand joints  (Pink IK: L proximal 5, R proximal 5,
                              L intermediate 5, R intermediate 5,
                              thumb intermediate/distal 4)

Sources per side (priority):

    1. ``snap["trackers"][f"{side}_forearm"]`` + forearm-to-wrist offset
       → full 6-DoF wrist pose from the Pico Enhanced Forearm tracker.
    2. ``snap["controllers"][side]["pose"]`` — controller fallback.
    3. Idle pose from ``DEFAULT_*_POS/QUAT``.

Finger joints:

    Primary — ``dex-retargeting`` DexPilot solver per side (``YAML`` cfg).
              The solver returns joints in URDF order; we slice the first 12
              channels as Inspire bend/spread proxies.
    Fallback — ``UDCAPFingerMapper.map_from_skeletal`` on the 31-bone model
              data, or ``map_hand`` on a VMC dict when the sampler is
              skeletal-disabled and a ``VMCHandReceiver`` is attached.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import torch

from . import coord_transforms as ct
from .fingertip_extractor import extract_fingertips_wrist_frame
from .udcap_finger_mapper import (
    INSPIRE_JOINT_DIM,
    UDCAPFingerMapper,
)


__all__ = [
    "G1SteamVRRetargeter",
    "DEFAULT_LEFT_POS",
    "DEFAULT_LEFT_QUAT",
    "DEFAULT_RIGHT_POS",
    "DEFAULT_RIGHT_QUAT",
]


# ── G1 idle pose (pelvis frame, Pink IK target) — matches ust_260220 ──────
DEFAULT_LEFT_POS: Tuple[float, float, float] = (-0.1487, 0.2038, 1.0952)
DEFAULT_LEFT_QUAT: Tuple[float, float, float, float] = (0.707, 0.0, 0.0, 0.707)
DEFAULT_RIGHT_POS: Tuple[float, float, float] = (0.1487, 0.2038, 1.0952)
DEFAULT_RIGHT_QUAT: Tuple[float, float, float, float] = (0.707, 0.0, 0.0, 0.707)


def _is_zero_pose(pose: Optional[Mapping[str, Any]]) -> bool:
    if pose is None:
        return True
    pos = np.asarray(pose.get("pos", (0.0, 0.0, 0.0)), dtype=np.float64)
    quat = np.asarray(pose.get("quat", (1.0, 0.0, 0.0, 0.0)), dtype=np.float64)
    if np.any(np.isnan(pos)) or np.any(np.isnan(quat)):
        return True
    if np.allclose(pos, 0.0, atol=1e-6) and abs(abs(quat[0]) - 1.0) < 1e-6:
        if np.allclose(quat[1:], 0.0, atol=1e-6):
            return True
    return False


@dataclass
class _HandSolverHandle:
    """Optional dex-retargeting handle (right/left), graceful fallback."""

    optimizer: Any = None
    num_dof: int = 0


class G1SteamVRRetargeter:
    """SteamVR snapshot → 38D Pink IK action (pure math)."""

    ACTION_DIM = 38
    FINGER_DIM_PER_HAND = 12

    def __init__(
        self,
        position_scale: float = 1.0,
        rotation_scale: float = 1.0,
        gripper_threshold: float = 0.5,
        body_pos_offset: Sequence[float] = (0.0, 0.0, 0.0),
        controller_pos_offset: Sequence[float] = (0.0, 0.0, 0.0),
        forearm_wrist_offset: Sequence[float] = (0.12, 0.0, 0.0),
        left_hand_retarget_yaml: Optional[str] = None,
        right_hand_retarget_yaml: Optional[str] = None,
        dex_urdf_dir: Optional[str] = None,
        use_waist_origin: bool = True,
        subtract_waist_z: bool = False,
        freeze_orientation: bool = False,
        debug: bool = False,
    ) -> None:
        self.position_scale = float(position_scale)
        self.rotation_scale = float(rotation_scale)
        self.gripper_threshold = float(gripper_threshold)
        self.body_pos_offset = np.asarray(body_pos_offset, dtype=np.float64)
        self.controller_pos_offset = np.asarray(controller_pos_offset, dtype=np.float64)
        self.forearm_wrist_offset = np.asarray(forearm_wrist_offset, dtype=np.float64)
        # When True, subtract the waist tracker X/Y from wrist positions so
        # the EEF target is expressed in the user's pelvis frame (Pink IK
        # 규약과 일치).  The original ust_260220 code assumed the PICO body
        # origin coincided with the G1 pelvis — on SteamVR/VD the play-area
        # origin is arbitrary, so we recover a pelvis-relative target by
        # using the physical waist tracker as moving origin.  Z stays
        # floor-anchored (SteamVR floor = IL floor), which matches the G1
        # idle target Z = 1.09 m.
        self.use_waist_origin = bool(use_waist_origin)
        # Further subtract waist Z — only needed if your Pink IK cfg uses a
        # pelvis-joint-local Z (not floor-anchored).  Leave False for the
        # default ust_260220 Pink IK setup.
        self.subtract_waist_z = bool(subtract_waist_z)
        # Diagnostic: ignore the forearm tracker orientation and emit the
        # G1 idle quaternion instead.  Useful to decouple "IK fails due to
        # unreachable wrist orientation" from "IK fails due to unreachable
        # position".  If the robot starts tracking positions with this flag
        # on, the issue is the forearm→wrist orientation mapping.
        self.freeze_orientation = bool(freeze_orientation)
        self.debug = bool(debug)

        self.finger_mapper = UDCAPFingerMapper()

        # dex-retargeting is optional; falls back to UDCAP mapper on failure.
        self._dex_left = self._build_dex_solver(left_hand_retarget_yaml, dex_urdf_dir)
        self._dex_right = self._build_dex_solver(right_hand_retarget_yaml, dex_urdf_dir)

        # Debug / diagnostics
        self._frames = 0
        self._source_left = "default"
        self._source_right = "default"
        self._finger_source_left = "idle"
        self._finger_source_right = "idle"

    # ── dex-retargeting plumbing ───────────────────────────────────────────
    @staticmethod
    def _build_dex_solver(yaml_path: Optional[str], urdf_dir: Optional[str]) -> _HandSolverHandle:
        if yaml_path is None:
            return _HandSolverHandle()
        try:
            from dex_retargeting.retargeting_config import RetargetingConfig
        except Exception:  # noqa: BLE001 — optional dep
            return _HandSolverHandle()
        try:
            if urdf_dir is not None:
                RetargetingConfig.set_default_urdf_dir(urdf_dir)
            cfg = RetargetingConfig.load_from_file(yaml_path)
            optimizer = cfg.build()
            num_dof = int(getattr(optimizer, "n_target", 0)) or len(
                getattr(optimizer, "target_joint_names", [])
            )
            return _HandSolverHandle(optimizer=optimizer, num_dof=num_dof)
        except Exception as exc:  # noqa: BLE001
            print(f"[G1SteamVRRetargeter] dex-retargeting solver unavailable ({exc}); using UDCAP fallback.")
            return _HandSolverHandle()

    # ── public API ─────────────────────────────────────────────────────────
    def retarget(
        self,
        snapshot: Mapping[str, Any],
        udcap_bones: Optional[Dict[str, Tuple[float, float, float, float]]] = None,
    ) -> torch.Tensor:
        """Return a 38D Pink IK action for ``snapshot``.

        ``udcap_bones`` is the raw VMC dict from ``VMCHandReceiver``; when
        skeletal data is available in the sampler snapshot we prefer it and
        treat ``udcap_bones`` as an additional fallback.
        """
        action = torch.zeros(self.ACTION_DIM, dtype=torch.float32)

        left_pos, left_quat, src_l = self._resolve_eef_target(snapshot, side="left")
        right_pos, right_quat, src_r = self._resolve_eef_target(snapshot, side="right")
        self._source_left = src_l
        self._source_right = src_r

        action[0:3] = torch.from_numpy(left_pos.astype(np.float32))
        action[3:7] = torch.from_numpy(left_quat.astype(np.float32))
        action[7:10] = torch.from_numpy(right_pos.astype(np.float32))
        action[10:14] = torch.from_numpy(right_quat.astype(np.float32))

        hand_24 = self._resolve_hand_joints(snapshot, udcap_bones)
        action[14:38] = torch.from_numpy(hand_24.astype(np.float32))

        self._frames += 1
        if self.debug and self._frames % 60 == 0:
            n_trk = len(snapshot.get("trackers") or {})
            print(
                f"[Retarget #{self._frames}] L={src_l}/{self._finger_source_left} "
                f"R={src_r}/{self._finger_source_right} | trackers={n_trk} | "
                f"L_pos=({left_pos[0]:.3f},{left_pos[1]:.3f},{left_pos[2]:.3f}) "
                f"R_pos=({right_pos[0]:.3f},{right_pos[1]:.3f},{right_pos[2]:.3f})"
            )
        return action

    # ── EEF resolution ─────────────────────────────────────────────────────
    def _user_pelvis_origin_il(
        self, snapshot: Mapping[str, Any]
    ) -> Optional[np.ndarray]:
        """Return the user's pelvis origin in Isaac Lab coordinates.

        Uses the ``waist`` tracker (physical Pico Enhanced Forearm waist
        belt).  Returns ``None`` if absent / invalid so the caller can fall
        back to the old behaviour.
        """
        if not self.use_waist_origin:
            return None
        trackers = snapshot.get("trackers") or {}
        waist = trackers.get("waist")
        if _is_zero_pose(waist):
            return None
        waist_pos_il, _ = ct.svr_to_isaaclab(waist["pos"], waist["quat"])
        # Default: keep Z from SteamVR floor (both coincide with IL floor).
        # When subtract_waist_z=True, also subtract Z so the result is fully
        # pelvis-local.
        if not self.subtract_waist_z:
            return np.array([waist_pos_il[0], waist_pos_il[1], 0.0], dtype=np.float64)
        return waist_pos_il.astype(np.float64)

    def _resolve_eef_target(
        self,
        snapshot: Mapping[str, Any],
        side: str,
    ) -> Tuple[np.ndarray, np.ndarray, str]:
        trackers = snapshot.get("trackers") or {}
        forearm = trackers.get(f"{side}_forearm")
        origin_il = self._user_pelvis_origin_il(snapshot)

        if not _is_zero_pose(forearm):
            wrist = ct.forearm_to_wrist(forearm, self.forearm_wrist_offset)
            pos_il, quat_il = ct.svr_to_isaaclab(wrist["pos"], wrist["quat"])
            if origin_il is not None:
                pos_il = pos_il - origin_il  # express wrist in user pelvis frame
            pos_il = pos_il * self.position_scale + self.body_pos_offset
            if self.freeze_orientation:
                idle_q = DEFAULT_LEFT_QUAT if side == "left" else DEFAULT_RIGHT_QUAT
                quat_il = np.asarray(idle_q, dtype=np.float64)
            return pos_il, quat_il, "forearm"

        controllers = snapshot.get("controllers") or {}
        ctrl = controllers.get(side)
        if ctrl is not None and not _is_zero_pose(ctrl.get("pose")):
            pose = ctrl["pose"]
            pos_il, quat_il = ct.svr_to_isaaclab(pose["pos"], pose["quat"])
            if origin_il is not None:
                pos_il = pos_il - origin_il
            pos_il = pos_il * self.position_scale + self.controller_pos_offset
            return pos_il, quat_il, "controller"

        if side == "left":
            return (
                np.asarray(DEFAULT_LEFT_POS, dtype=np.float64),
                np.asarray(DEFAULT_LEFT_QUAT, dtype=np.float64),
                "default",
            )
        return (
            np.asarray(DEFAULT_RIGHT_POS, dtype=np.float64),
            np.asarray(DEFAULT_RIGHT_QUAT, dtype=np.float64),
            "default",
        )

    # ── hand resolution ────────────────────────────────────────────────────
    def _resolve_hand_joints(
        self,
        snapshot: Mapping[str, Any],
        udcap_bones: Optional[Dict[str, Tuple[float, float, float, float]]],
    ) -> np.ndarray:
        left_12 = np.zeros(INSPIRE_JOINT_DIM, dtype=np.float32)
        right_12 = np.zeros(INSPIRE_JOINT_DIM, dtype=np.float32)

        hands = snapshot.get("hands") or {}
        left_hand = hands.get("left")
        right_hand = hands.get("right")

        if left_hand is not None and self._dex_left.optimizer is not None:
            try:
                tips = extract_fingertips_wrist_frame(left_hand["bones"])
                left_12 = self._dex_to_inspire12(self._dex_left, tips)
                self._finger_source_left = "dex"
            except Exception:
                left_hand_via_dex = False
            else:
                left_hand_via_dex = True
        else:
            left_hand_via_dex = False

        if right_hand is not None and self._dex_right.optimizer is not None:
            try:
                tips = extract_fingertips_wrist_frame(right_hand["bones"])
                right_12 = self._dex_to_inspire12(self._dex_right, tips)
                self._finger_source_right = "dex"
            except Exception:
                right_hand_via_dex = False
            else:
                right_hand_via_dex = True
        else:
            right_hand_via_dex = False

        if not left_hand_via_dex:
            if left_hand is not None:
                left_12 = self.finger_mapper.map_from_skeletal(
                    left_hand["bones"], is_right=False,
                    finger_curls=left_hand.get("fingerCurls"),
                )
                self._finger_source_left = "skeletal"
            elif udcap_bones:
                left_12 = self.finger_mapper.map_hand(udcap_bones, is_right=False)
                self._finger_source_left = "vmc"
            else:
                self._finger_source_left = "idle"

        if not right_hand_via_dex:
            if right_hand is not None:
                right_12 = self.finger_mapper.map_from_skeletal(
                    right_hand["bones"], is_right=True,
                    finger_curls=right_hand.get("fingerCurls"),
                )
                self._finger_source_right = "skeletal"
            elif udcap_bones:
                right_12 = self.finger_mapper.map_hand(udcap_bones, is_right=True)
                self._finger_source_right = "vmc"
            else:
                self._finger_source_right = "idle"

        return self._inspire12_to_24joints(right_12, left_12)

    @staticmethod
    def _dex_to_inspire12(handle: _HandSolverHandle, fingertips_wrist_frame: np.ndarray) -> np.ndarray:
        """Run a DexPilot optimizer on fingertip positions and down-project
        the resulting joint target to the 12 Inspire bend/spread channels.

        The YAML shipped in ``config/dex_retargeting/`` is expected to use
        the DexPilot vector-type solver whose targets are fingertip
        positions.  We call ``retarget`` with a ``(5, 3)`` array and
        transparently handle either ``(5, 3)`` or ``(6, 3)`` inputs.
        """
        if handle.optimizer is None:
            return np.zeros(INSPIRE_JOINT_DIM, dtype=np.float32)
        try:
            q = handle.optimizer.retarget(np.asarray(fingertips_wrist_frame, dtype=np.float64))
        except Exception:  # noqa: BLE001 — solver not converged / shape mismatch
            return np.zeros(INSPIRE_JOINT_DIM, dtype=np.float32)
        q = np.asarray(q, dtype=np.float32).reshape(-1)
        if q.size >= INSPIRE_JOINT_DIM:
            return q[:INSPIRE_JOINT_DIM]
        padded = np.zeros(INSPIRE_JOINT_DIM, dtype=np.float32)
        padded[: q.size] = q
        return padded

    @staticmethod
    def _inspire12_to_24joints(right_12: np.ndarray, left_12: np.ndarray) -> np.ndarray:
        """Pink IK 24-joint expansion (identical to ust_260220 g1_retargeter)."""
        j = np.zeros(24, dtype=np.float32)
        # Left proximal (5)
        j[0] = left_12[2]; j[1] = left_12[4]; j[2] = left_12[8]; j[3] = left_12[6]; j[4] = left_12[1]
        # Right proximal (5)
        j[5] = right_12[2]; j[6] = right_12[4]; j[7] = right_12[8]; j[8] = right_12[6]; j[9] = right_12[1]
        # Left intermediate (5)
        j[10] = left_12[2]; j[11] = left_12[4]; j[12] = left_12[8]; j[13] = left_12[6]; j[14] = left_12[0]
        # Right intermediate (5)
        j[15] = right_12[2]; j[16] = right_12[4]; j[17] = right_12[8]; j[18] = right_12[6]; j[19] = right_12[0]
        # Thumb intermediate / distal (4)
        j[20] = left_12[10]; j[21] = right_12[10]; j[22] = left_12[0]; j[23] = right_12[0]
        return j

    def get_source_info(self) -> Dict[str, str]:
        return {
            "left_eef": self._source_left,
            "right_eef": self._source_right,
            "left_finger": self._finger_source_left,
            "right_finger": self._finger_source_right,
        }

    def reset(self) -> None:
        self._frames = 0
        self._source_left = "default"
        self._source_right = "default"
        self._finger_source_left = "idle"
        self._finger_source_right = "idle"
