"""SteamVR snapshot → GR1T2 + 2-finger gripper Pink IK 16D action.

This is the **option B** retargeter that replaces the 22-DoF Fourier hand
with a 2-DoF parallel gripper per side.  The action layout collapses to::

    [0:3]    left wrist position   (base_link frame, metres)
    [3:7]    left wrist quaternion (wxyz)
    [7:10]   right wrist position
    [10:14]  right wrist quaternion (wxyz, with optional 180° Z correction)
    [14]     left gripper command   (-1 = close, +1 = open)
    [15]     right gripper command  (-1 = close, +1 = open)

The wrist EEF target is taken from the **PICO controller pose** (preferred,
since the user holds the controller in hand), with a fallback to the
``left_forearm`` / ``right_forearm`` tracker when the controller pose is
unavailable.  The forearm tracker is now expected to be worn at the elbow
("Forearm Tracking Enhanced" mode in PICO Connect), so the
``forearm_wrist_offset`` should be ~0.28 m (typical adult forearm length)
rather than the 0.12 m used when the tracker was on the wrist itself.

Gripper command source (selectable via ``gripper_signal_source`` cfg):

    * ``"grip"``    — PICO controller grip pull only (default, 9.28+).
    * ``"trigger"`` — PICO controller trigger pull only (legacy 9.27).
    * ``"both"``    — Logical OR of grip + trigger (either closes).

Hysteresis is applied so a slowly-modulated value near the threshold
does not cause rapid open/close chattering.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np
import torch

# Reuse the hardware-generic helpers from the Windows G1 port.
from ust_ws.ust_hm_grip.teleop import coord_transforms as ct


__all__ = [
    "GR1T2GripperSteamVRRetargeter",
    "GR1T2GripperRetargeterCfg",
    "DEFAULT_LEFT_POS",
    "DEFAULT_LEFT_QUAT",
    "DEFAULT_RIGHT_POS",
    "DEFAULT_RIGHT_QUAT",
    "ACTION_DIM",
    "GRIPPER_OPEN",
    "GRIPPER_CLOSE",
]


# ── Action dimensions ──────────────────────────────────────────────────
ACTION_DIM: int = 7 + 7 + 2     # 16

# Gripper binary commands.  Matches BinaryJointPositionAction's convention
# where ``actions < 0`` triggers the close command and otherwise the open
# command.  We send unambiguous ±1.0 values so any noise stays well clear
# of the threshold.
GRIPPER_OPEN: float = +1.0
GRIPPER_CLOSE: float = -1.0


# ── Idle pose (base_link frame, gripper_tcp_link target) ──────────────
# 13th-bis session, part 8 — measured directly from the actual T-pose
# (default joint state) of the assembled robot using
# scripts/inspect_tcp_world_pose.py / runtime [tcp_diag] log.  The
# previous values (-/+0.20, 0.00, +1.05, quat=0.707,0,0.707,0) were
# legacy from a pre-Robotiq-attach setup; they put the wrist target ~1 m
# above the pelvis (above the robot's head) and were L/R asymmetric on
# the X axis (LEFT idle pos = behind the robot, X=-0.20), causing Pink
# IK to twist the left arm backwards to reach an impossible target.
#
# Measured pose (T-pose, default joint state):
#   * Both wrist TCPs sit ~23 cm to either side of the spine (Y = ±0.229)
#   * Both 23 cm BELOW base_link (Z = -0.235) — gripper hangs at hip level
#   * Almost zero forward offset (X = +0.003)
#   * Both palm-down convention: quat = (0, 0, 1, 0) — 180° about Y, which
#     is exactly the Ry(180°) gripper attach orientation baked into
#     build_robotiq_usd.py §3.12.
#
# Anchoring idle to these values means the calibrated wrist target
# (idle_q * delta_q) matches the robot's natural T-pose at calibration —
# Pink IK has zero error → robot stays still until user moves the
# controller.
DEFAULT_LEFT_POS: Tuple[float, float, float] = (0.003, 0.229, -0.235)
DEFAULT_LEFT_QUAT: Tuple[float, float, float, float] = (0.0, 0.0, 1.0, 0.0)
DEFAULT_RIGHT_POS: Tuple[float, float, float] = (0.003, -0.229, -0.235)
DEFAULT_RIGHT_QUAT: Tuple[float, float, float, float] = (0.0, 0.0, 1.0, 0.0)


# 180° rotation about +Z (wxyz).  Matches the GR1T2Retargeter's right-wrist
# correction so the OpenXR palm frame maps to the URDF control frame.
_Z180_WXYZ: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


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
class GR1T2GripperRetargeterCfg:
    """Construction parameters for :class:`GR1T2GripperSteamVRRetargeter`."""

    position_scale: float = 1.0
    rotation_scale: float = 1.0

    # 13th session, part 10 — per-axis wrist position scale (multiplies
    # the controller delta from zero before adding to idle_pos).  Users
    # report Z (up/down) tracking feels weak; bumping Z to 1.5 amplifies
    # vertical motion.  X (forward/back) and Y (left/right) stay 1.0 so
    # natural reaching distance doesn't change.  Applied in the
    # ``controller_pose_zero`` calibration branch only — the legacy
    # waist-origin path keeps the existing ``position_scale``.
    wrist_pos_scale_per_axis: Tuple[float, float, float] = (1.0, 1.0, 1.5)

    body_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    controller_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Forearm-tracker offset along the local +X axis (from tracker origin to
    # wrist).  9.27 default 0.12 m matched a wrist-mounted tracker; with the
    # PICO "Forearm Tracking Enhanced" calibration the tracker now sits on
    # the elbow / proximal forearm so the offset must grow to a typical
    # adult forearm length of ~0.28 m.  Override per-user via the device
    # config or a calibration script.
    forearm_wrist_offset: Tuple[float, float, float] = (0.28, 0.0, 0.0)

    # Express EEF targets in the robot's base_link frame by subtracting the
    # waist tracker XY.  Pink IK expects base_link-relative targets.
    use_waist_origin: bool = True
    subtract_waist_z: bool = False

    # Diagnostic: freeze wrist orientation to idle quaternion (position-only
    # IK).  Lets you separate "IK fails due to bad orientation" from "bad
    # position" during bring-up.
    freeze_orientation: bool = False

    # Apply the 180° Z rotation to the right wrist quaternion so the
    # controller pose lands in GR1T2's URDF control frame.
    # 13th-bis session, part 8 — Z180 hack disabled.  Was needed when
    # idle quats had a 90° about Y convention; with the measured palm-
    # down (0, 0, 1, 0) idle quaternion used by both sides, the right
    # wrist no longer needs an extra Z180 flip — both sides target
    # palm-down naturally.  Left True only if a downstream env wants
    # the legacy convention.
    right_wrist_z180: bool = False

    # The hand the user actually grips with is the controller, so prefer
    # the controller pose for the wrist target by default.  Forearm
    # tracker remains as a fallback for the case where the controller is
    # temporarily unavailable (out of view / battery / paired but idle).
    prefer_controller_for_eef: bool = True
    controller_to_wrist_offset: Tuple[float, float, float] = (0.0, 0.0, -0.05)

    # Gripper hysteresis thresholds.  Close when the chosen signal crosses
    # ``gripper_close_threshold`` rising, re-open when it falls below
    # ``gripper_open_threshold``.  The deadband prevents chattering when
    # the user holds the input near a single value.
    gripper_close_threshold: float = 0.6
    gripper_open_threshold: float = 0.4

    # 9.28: Which controller input drives gripper close/open.
    #   "grip"    — grip pull only (default; matches user request 2026-05-XX
    #               that close/open should be on grip, not trigger).
    #   "trigger" — trigger pull only (legacy 9.27 behaviour).
    #   "both"    — logical OR (either grip or trigger closes the gripper).
    # The previous ``use_grip_as_close`` field is kept for backward-compat
    # but is now ignored; ``gripper_signal_source`` is authoritative.
    gripper_signal_source: str = "grip"
    # Deprecated.  Left intact so existing calls don't break, but the new
    # ``gripper_signal_source`` field is what _resolve_gripper looks at.
    use_grip_as_close: bool = True

    # When True, force the wrist EEF target to the idle pose regardless
    # of forearm tracker / controller state.  Used by the `--ignore_arms`
    # flag for the no-tracker rig.
    disable_arm_tracking: bool = False

    # 13th session: When True, the sampler's snapshot poses are already
    # expressed in the Isaac Lab base_link frame (e.g.
    # ``XRoboSampler._pose_to_pq`` applies ``ct.xr_to_isaaclab`` at read
    # time).  In that case the retargeter must NOT re-apply
    # ``svr_to_isaaclab`` — the double-transform mangles positions and
    # quaternions because R_SVR2IL * R_XR2IL != I.
    # The SteamVR backend keeps this False (default) so its raw-OpenVR
    # snapshot still goes through svr_to_isaaclab once.
    pose_in_il_frame: bool = False

    # 13th-bis session: Startup calibration zero for the controllers.
    # When non-None, the retargeter switches to "delta-from-zero" mode
    # and the wrist target becomes::
    #
    #     wrist_target = idle_wrist_pos + (raw_controller_pos - zero_pos)
    #
    # so the robot starts in idle T-pose at startup regardless of where
    # the user's hands happen to be.  Setting this dict mid-run causes
    # the robot to "re-zero" to the current hand position.
    #
    # Layout: ``{"left": {"pos": (3,), "quat": (4,)}, "right": {...}}``
    # When the dict is missing a side, that side falls back to the
    # legacy waist-origin-subtract path.
    controller_pose_zero: Optional[Dict[str, Dict[str, Any]]] = None

    # 13th session, part 12-13 — independent position / orientation
    # sources for the wrist EEF target.  User wants controller to drive
    # ONLY gripper open/close (via buttons); arm reach + wrist rotation
    # both come from the body skeleton wrist tracker.
    #
    #   wrist_pos_source  = "wrist_tracker" (default) | "controller"
    #   wrist_quat_source = "wrist_tracker" (default) | "controller"
    #
    # Setting both to "wrist_tracker" makes the controller pose entirely
    # NON-driving — only its buttons (trigger / grip) reach the
    # retargeter.  When the chosen source is unavailable (e.g. body
    # skeleton not streaming) the channel falls back to the IDLE value
    # (delta = 0) — it does NOT silently switch to controller, which
    # would re-introduce the very coupling we removed.
    wrist_pos_source: str = "controller"
    wrist_quat_source: str = "controller"
    # Startup zero for body wrist trackers (mirror of
    # ``controller_pose_zero`` but for the body skeleton wrist).
    # Layout: ``{"left": {"pos": (3,), "quat": (4,)}, "right": {...}}``.
    # Quat populated when ``wrist_quat_source="wrist_tracker"`` so
    # the SMPL wrist orientation can be deltaed against the user's
    # startup wrist orientation just like the controller path.
    wrist_pose_zero: Optional[Dict[str, Dict[str, Any]]] = None

    # Override the idle defaults (e.g. after running calibrate_idle_pose.py).
    idle_left_pos: Tuple[float, float, float] = DEFAULT_LEFT_POS
    idle_left_quat: Tuple[float, float, float, float] = DEFAULT_LEFT_QUAT
    idle_right_pos: Tuple[float, float, float] = DEFAULT_RIGHT_POS
    idle_right_quat: Tuple[float, float, float, float] = DEFAULT_RIGHT_QUAT

    debug: bool = False


class GR1T2GripperSteamVRRetargeter:
    """SteamVR snapshot → 16D Pink IK action for GR1T2 + parallel gripper.

    Pure math, no Isaac Sim dependency — safe to import in unit tests.
    """

    ACTION_DIM = ACTION_DIM
    GRIPPER_OPEN = GRIPPER_OPEN
    GRIPPER_CLOSE = GRIPPER_CLOSE

    def __init__(
        self,
        cfg: Optional[GR1T2GripperRetargeterCfg] = None,
        **kwargs: Any,
    ) -> None:
        if cfg is None:
            cfg = GR1T2GripperRetargeterCfg(**kwargs)
        elif kwargs:
            raise TypeError(
                "pass either a GR1T2GripperRetargeterCfg or keyword args, not both"
            )
        self.cfg = cfg

        # Diagnostics / instrumentation
        self._frames = 0
        self._source_left = "default"
        self._source_right = "default"

        # Hysteresis state — start with "open" so the first frame doesn't
        # accidentally clamp empty space.
        self._gripper_state_left = GRIPPER_OPEN
        self._gripper_state_right = GRIPPER_OPEN

        # Per-side max trigger / grip seen (for diagnostic logs)
        self._max_trigger: Dict[str, float] = {"left": 0.0, "right": 0.0}
        self._max_grip: Dict[str, float] = {"left": 0.0, "right": 0.0}

    # ── public API ─────────────────────────────────────────────────────
    def retarget(
        self,
        snapshot: Mapping[str, Any],
        action_inputs: Optional[Mapping[str, Mapping[str, Any]]] = None,
    ) -> torch.Tensor:
        """Return a 16D action for ``snapshot``.

        Parameters
        ----------
        snapshot:
            Latest ``SteamVRSampler`` snapshot (hmd, trackers, controllers).
        action_inputs:
            ``{"left": {"trigger": float, "grip": float, ...}, "right": ...}``
            mapping with action-API analog values.  Pass ``None`` to fall
            back to legacy ``getControllerState()`` button state in the
            snapshot.
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

        # Gripper commands
        gripper_l = self._resolve_gripper(
            snapshot, action_inputs, side="left",
        )
        gripper_r = self._resolve_gripper(
            snapshot, action_inputs, side="right",
        )
        action[14] = float(gripper_l)
        action[15] = float(gripper_r)

        self._frames += 1
        if self.cfg.debug and (self._frames == 1 or self._frames % 20 == 0):
            self._print_periodic_log(action, src_l, src_r)
        return action

    def get_source_info(self) -> Dict[str, Any]:
        return {
            "left_eef": self._source_left,
            "right_eef": self._source_right,
            "left_gripper": "close" if self._gripper_state_left < 0 else "open",
            "right_gripper": "close" if self._gripper_state_right < 0 else "open",
            "max_trigger_left": self._max_trigger["left"],
            "max_trigger_right": self._max_trigger["right"],
            "max_grip_left": self._max_grip["left"],
            "max_grip_right": self._max_grip["right"],
        }

    def reset(self) -> None:
        self._frames = 0
        self._source_left = "default"
        self._source_right = "default"
        self._gripper_state_left = GRIPPER_OPEN
        self._gripper_state_right = GRIPPER_OPEN
        self._max_trigger = {"left": 0.0, "right": 0.0}
        self._max_grip = {"left": 0.0, "right": 0.0}

    # ── EEF resolution ─────────────────────────────────────────────────
    def _user_pelvis_origin_il(
        self, snapshot: Mapping[str, Any]
    ) -> Optional[np.ndarray]:
        if not self.cfg.use_waist_origin:
            return None
        trackers = snapshot.get("trackers") or {}
        waist = trackers.get("waist")
        if _is_zero_pose(waist):
            return None
        if self.cfg.pose_in_il_frame:
            waist_pos_il = np.asarray(waist["pos"], dtype=np.float64)
        else:
            waist_pos_il, _ = ct.svr_to_isaaclab(waist["pos"], waist["quat"])
        if not self.cfg.subtract_waist_z:
            return np.array([waist_pos_il[0], waist_pos_il[1], 0.0], dtype=np.float64)
        return waist_pos_il.astype(np.float64)

    def _resolve_eef_target(
        self,
        snapshot: Mapping[str, Any],
        side: str,
    ) -> Tuple[np.ndarray, np.ndarray, str]:
        if self.cfg.disable_arm_tracking:
            if side == "left":
                return (
                    np.asarray(self.cfg.idle_left_pos, dtype=np.float64),
                    np.asarray(self.cfg.idle_left_quat, dtype=np.float64),
                    "default(disabled)",
                )
            return (
                np.asarray(self.cfg.idle_right_pos, dtype=np.float64),
                np.asarray(self.cfg.idle_right_quat, dtype=np.float64),
                "default(disabled)",
            )

        trackers = snapshot.get("trackers") or {}
        forearm = trackers.get(f"{side}_forearm")
        controllers = snapshot.get("controllers") or {}
        ctrl = controllers.get(side)
        origin_il = self._user_pelvis_origin_il(snapshot)

        cfg_offset = (
            np.asarray(self.cfg.body_pos_offset, dtype=np.float64)
            if not isinstance(self.cfg.body_pos_offset, np.ndarray)
            else self.cfg.body_pos_offset
        )
        ctrl_offset = (
            np.asarray(self.cfg.controller_pos_offset, dtype=np.float64)
            if not isinstance(self.cfg.controller_pos_offset, np.ndarray)
            else self.cfg.controller_pos_offset
        )

        def _from_forearm() -> Tuple[np.ndarray, np.ndarray, str]:
            wrist = ct.forearm_to_wrist(forearm, self.cfg.forearm_wrist_offset)
            if self.cfg.pose_in_il_frame:
                pos_il = np.asarray(wrist["pos"], dtype=np.float64)
                quat_il = np.asarray(wrist["quat"], dtype=np.float64)
            else:
                pos_il, quat_il = ct.svr_to_isaaclab(wrist["pos"], wrist["quat"])
            if origin_il is not None:
                pos_il = pos_il - origin_il
            pos_il = pos_il * self.cfg.position_scale + cfg_offset
            quat_il = self._apply_orientation(quat_il, side)
            return pos_il, quat_il, "forearm"

        def _from_controller() -> Tuple[np.ndarray, np.ndarray, str]:
            pose = ctrl["pose"]
            wrist = ct.forearm_to_wrist(pose, self.cfg.controller_to_wrist_offset)
            if self.cfg.pose_in_il_frame:
                pos_il = np.asarray(wrist["pos"], dtype=np.float64)
                quat_il = np.asarray(wrist["quat"], dtype=np.float64)
            else:
                pos_il, quat_il = ct.svr_to_isaaclab(wrist["pos"], wrist["quat"])

            # 13th-bis session — startup wrist calibration.  When
            # ``controller_pose_zero[side]`` is set, the wrist target
            # is computed as a delta from the user's startup hand
            # position, anchored at the robot's idle wrist pose.  This
            # decouples the user's posture-at-startup from the robot's
            # idle wrist target — robot starts in T-pose even if the
            # user is seated with arms at their sides, and movement
            # tracks 1:1.
            cal = self.cfg.controller_pose_zero
            if cal is not None and side in cal and cal[side] is not None:
                zero_pos = np.asarray(cal[side]["pos"], dtype=np.float64)
                zero_quat = np.asarray(cal[side]["quat"], dtype=np.float64)
                idle_pos = (
                    np.asarray(self.cfg.idle_left_pos, dtype=np.float64)
                    if side == "left"
                    else np.asarray(self.cfg.idle_right_pos, dtype=np.float64)
                )
                idle_q = (
                    np.asarray(self.cfg.idle_left_quat, dtype=np.float64)
                    if side == "left"
                    else np.asarray(self.cfg.idle_right_quat, dtype=np.float64)
                )
                axis_scale = np.asarray(
                    self.cfg.wrist_pos_scale_per_axis, dtype=np.float64
                )

                trackers_local = snapshot.get("trackers") or {}
                wt = trackers_local.get(f"{side}_wrist")
                wzero = (self.cfg.wrist_pose_zero or {}).get(side)

                # ── POSITION SOURCE ───────────────────────────────────
                # 13th session, part 13 — when wrist_pos_source is
                # "wrist_tracker" but the tracker is unavailable, fall
                # back to IDLE (delta=0).  We do NOT fall back to
                # controller pos because that re-introduces the
                # coupling the user explicitly removed.
                pos_source_used = "idle"
                delta_pos_il = np.zeros(3, dtype=np.float64)
                if self.cfg.wrist_pos_source == "wrist_tracker":
                    if (
                        wt is not None
                        and not _is_zero_pose(wt)
                        and wzero is not None
                        and wzero.get("pos") is not None
                    ):
                        wt_pos = np.asarray(wt["pos"], dtype=np.float64)
                        if not self.cfg.pose_in_il_frame:
                            wt_pos, _ = ct.svr_to_isaaclab(wt_pos, wt["quat"])
                        wt_zero = np.asarray(wzero["pos"], dtype=np.float64)
                        delta_pos_il = wt_pos - wt_zero
                        pos_source_used = "wrist_tracker"
                    # else: stay at idle (delta=0).  No controller
                    # fallback — controller must NOT influence position.
                else:
                    # "controller" position source — legacy path.
                    raw_pos = np.asarray(pose["pos"], dtype=np.float64)
                    if self.cfg.pose_in_il_frame:
                        raw_pos_il = raw_pos
                    else:
                        raw_pos_il, _ = ct.svr_to_isaaclab(raw_pos, pose["quat"])
                    delta_pos_il = raw_pos_il - zero_pos
                    pos_source_used = "controller_pos"

                pos_il = (
                    idle_pos
                    + delta_pos_il * axis_scale * self.cfg.position_scale
                    + ctrl_offset
                )
                self._pos_source_last = {
                    **getattr(self, "_pos_source_last", {}),
                    side: pos_source_used,
                }

                # ── ORIENTATION SOURCE ────────────────────────────────
                # 13th session, part 13 — independent quat source.  When
                # ``wrist_quat_source == "wrist_tracker"`` use SMPL
                # L/R_Wrist orientation; controller quat ignored.
                # Fallback (tracker missing) → idle quat, NOT controller.
                quat_source_used = "idle"
                if self.cfg.wrist_quat_source == "wrist_tracker":
                    if (
                        wt is not None
                        and not _is_zero_pose(wt)
                        and wzero is not None
                        and wzero.get("quat") is not None
                    ):
                        wt_quat = np.asarray(wt["quat"], dtype=np.float64)
                        if not self.cfg.pose_in_il_frame:
                            _, wt_quat = ct.svr_to_isaaclab(
                                np.zeros(3, dtype=np.float64), wt_quat
                            )
                        wt_zero_q = np.asarray(wzero["quat"], dtype=np.float64)
                        delta_q = ct.quat_multiply(
                            wt_quat, ct.quat_conjugate(wt_zero_q)
                        )
                        quat_il = ct.quat_multiply(delta_q, idle_q)
                        quat_source_used = "wrist_tracker"
                    else:
                        quat_il = idle_q
                        quat_source_used = "idle_fallback"
                else:
                    # "controller" quat source — legacy delta-from-zero.
                    delta_q = ct.quat_multiply(
                        quat_il, ct.quat_conjugate(zero_quat)
                    )
                    quat_il = ct.quat_multiply(delta_q, idle_q)
                    quat_source_used = "controller_quat"

                # Right-side Z180 convention adjustment (post-source).
                if side == "right" and self.cfg.right_wrist_z180:
                    quat_il = ct.quat_multiply(quat_il, _Z180_WXYZ)
                if self.cfg.freeze_orientation:
                    quat_il = idle_q
                    quat_source_used = "freeze_idle"
                self._quat_source_last = {
                    **getattr(self, "_quat_source_last", {}),
                    side: quat_source_used,
                }
            else:
                if origin_il is not None:
                    pos_il = pos_il - origin_il
                pos_il = pos_il * self.cfg.position_scale + ctrl_offset
                quat_il = self._apply_orientation(quat_il, side)
            return pos_il, quat_il, "controller"

        # 13th session, part 14 — Source-based dispatch.  Position and
        # orientation each have an INDEPENDENT source.  When the source
        # is the body wrist tracker, the controller path is NOT
        # consulted at all (even before ``controller_pose_zero`` is
        # captured).  Previously the non-cal branch of ``_from_controller``
        # ran when the cal hadn't been captured yet, silently feeding
        # raw controller pose into the wrist target during the startup
        # window — that's exactly the coupling the user reported.
        pos_src = (self.cfg.wrist_pos_source or "controller").lower()
        quat_src = (self.cfg.wrist_quat_source or "controller").lower()

        # If BOTH sources are controller, keep the legacy router so
        # forearm tracker can still fall back when controllers are absent.
        if pos_src == "controller" and quat_src == "controller":
            if self.cfg.prefer_controller_for_eef:
                if ctrl is not None and not _is_zero_pose(ctrl.get("pose")):
                    return _from_controller()
                if not _is_zero_pose(forearm):
                    return _from_forearm()
            else:
                if not _is_zero_pose(forearm):
                    return _from_forearm()
                if ctrl is not None and not _is_zero_pose(ctrl.get("pose")):
                    return _from_controller()

            if side == "left":
                return (
                    np.asarray(self.cfg.idle_left_pos, dtype=np.float64),
                    np.asarray(self.cfg.idle_left_quat, dtype=np.float64),
                    "default",
                )
            return (
                np.asarray(self.cfg.idle_right_pos, dtype=np.float64),
                np.asarray(self.cfg.idle_right_quat, dtype=np.float64),
                "default",
            )

        # Otherwise: at least one source is wrist_tracker.  Compose pos
        # and quat from independent sources, idle for missing data.
        # Controller path is ONLY consulted for axes whose source is
        # explicitly "controller".
        return self._compose_eef_target(snapshot, side, pos_src, quat_src,
                                         ctrl_offset=ctrl_offset)

    def _compose_eef_target(
        self,
        snapshot: Mapping[str, Any],
        side: str,
        pos_src: str,
        quat_src: str,
        ctrl_offset: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, str]:
        """Source-based pos/quat composer (part 14).

        Each axis is computed from its declared source, independently.
        When the declared source is missing data, the axis snaps to its
        idle value — there is NO silent fallback to the controller.
        This guarantees the controller pose CANNOT drive the wrist EEF
        target unless its source is explicitly configured.
        """
        idle_pos = (
            np.asarray(self.cfg.idle_left_pos, dtype=np.float64)
            if side == "left"
            else np.asarray(self.cfg.idle_right_pos, dtype=np.float64)
        )
        idle_q = (
            np.asarray(self.cfg.idle_left_quat, dtype=np.float64)
            if side == "left"
            else np.asarray(self.cfg.idle_right_quat, dtype=np.float64)
        )
        axis_scale = np.asarray(
            self.cfg.wrist_pos_scale_per_axis, dtype=np.float64
        )
        trackers = snapshot.get("trackers") or {}
        wt = trackers.get(f"{side}_wrist")
        wzero = (self.cfg.wrist_pose_zero or {}).get(side)
        ctrls = snapshot.get("controllers") or {}
        ctrl = ctrls.get(side)
        cal = self.cfg.controller_pose_zero
        czero = (cal or {}).get(side)

        # ── POS ─────────────────────────────────────────────────────
        pos_source_used = "idle"
        delta_pos_il = np.zeros(3, dtype=np.float64)
        if pos_src == "wrist_tracker":
            if (wt is not None and not _is_zero_pose(wt)
                    and wzero is not None and wzero.get("pos") is not None):
                wt_pos = np.asarray(wt["pos"], dtype=np.float64)
                if not self.cfg.pose_in_il_frame:
                    wt_pos, _ = ct.svr_to_isaaclab(wt_pos, wt["quat"])
                wt_zero = np.asarray(wzero["pos"], dtype=np.float64)
                delta_pos_il = wt_pos - wt_zero
                pos_source_used = "wrist_tracker"
        elif pos_src == "controller":
            if (ctrl is not None and not _is_zero_pose(ctrl.get("pose"))
                    and czero is not None and czero.get("pos") is not None):
                ctrl_pose = ctrl["pose"]
                raw_pos = np.asarray(ctrl_pose["pos"], dtype=np.float64)
                if self.cfg.pose_in_il_frame:
                    raw_pos_il = raw_pos
                else:
                    raw_pos_il, _ = ct.svr_to_isaaclab(raw_pos, ctrl_pose["quat"])
                zero_pos = np.asarray(czero["pos"], dtype=np.float64)
                delta_pos_il = raw_pos_il - zero_pos
                pos_source_used = "controller"

        pos_il = (
            idle_pos
            + delta_pos_il * axis_scale * self.cfg.position_scale
            + ctrl_offset
        )

        # ── QUAT ────────────────────────────────────────────────────
        quat_source_used = "idle"
        quat_il = idle_q.copy()
        if quat_src == "wrist_tracker":
            if (wt is not None and not _is_zero_pose(wt)
                    and wzero is not None and wzero.get("quat") is not None):
                wt_quat = np.asarray(wt["quat"], dtype=np.float64)
                if not self.cfg.pose_in_il_frame:
                    _, wt_quat = ct.svr_to_isaaclab(
                        np.zeros(3, dtype=np.float64), wt_quat
                    )
                wt_zero_q = np.asarray(wzero["quat"], dtype=np.float64)
                delta_q = ct.quat_multiply(
                    wt_quat, ct.quat_conjugate(wt_zero_q)
                )
                quat_il = ct.quat_multiply(delta_q, idle_q)
                quat_source_used = "wrist_tracker"
        elif quat_src == "controller":
            if (ctrl is not None and not _is_zero_pose(ctrl.get("pose"))
                    and czero is not None and czero.get("quat") is not None):
                ctrl_pose = ctrl["pose"]
                raw_quat = np.asarray(ctrl_pose["quat"], dtype=np.float64)
                if not self.cfg.pose_in_il_frame:
                    _, raw_quat = ct.svr_to_isaaclab(
                        np.zeros(3, dtype=np.float64), raw_quat
                    )
                zero_quat = np.asarray(czero["quat"], dtype=np.float64)
                delta_q = ct.quat_multiply(
                    raw_quat, ct.quat_conjugate(zero_quat)
                )
                quat_il = ct.quat_multiply(delta_q, idle_q)
                quat_source_used = "controller"

        # Right-side Z180 convention adjustment.
        if side == "right" and self.cfg.right_wrist_z180:
            quat_il = ct.quat_multiply(quat_il, _Z180_WXYZ)
        if self.cfg.freeze_orientation:
            quat_il = idle_q
            quat_source_used = "freeze_idle"

        # Stash diagnostics for callers that want to surface in logs.
        self._pos_source_last = {
            **getattr(self, "_pos_source_last", {}),
            side: pos_source_used,
        }
        self._quat_source_last = {
            **getattr(self, "_quat_source_last", {}),
            side: quat_source_used,
        }
        return pos_il, quat_il, f"pos={pos_source_used}/quat={quat_source_used}"

    def _apply_orientation(self, quat_il: np.ndarray, side: str) -> np.ndarray:
        if self.cfg.freeze_orientation:
            idle_q = self.cfg.idle_left_quat if side == "left" else self.cfg.idle_right_quat
            return np.asarray(idle_q, dtype=np.float64)
        if side == "right" and self.cfg.right_wrist_z180:
            quat_il = ct.quat_multiply(quat_il, _Z180_WXYZ)
        return quat_il.astype(np.float64)

    # ── Gripper resolution ─────────────────────────────────────────────
    def _resolve_gripper(
        self,
        snapshot: Mapping[str, Any],
        action_inputs: Optional[Mapping[str, Mapping[str, Any]]],
        side: str,
    ) -> float:
        """Map grip / trigger → ±1 with hysteresis.  Returns the new state."""
        trigger, grip = self._read_trigger_grip(snapshot, action_inputs, side)
        # Track running max (purely diagnostic)
        self._max_trigger[side] = max(self._max_trigger[side], trigger)
        self._max_grip[side] = max(self._max_grip[side], grip)

        # 9.28: source selection — grip (default), trigger, or both.
        src = (self.cfg.gripper_signal_source or "grip").lower()
        if src == "trigger":
            signal = trigger
        elif src == "both":
            signal = max(trigger, grip)
        else:
            # "grip" (default) and any unknown value fall back to grip-only.
            signal = grip

        # Hysteresis: rising edge through close_threshold → close, falling
        # edge through open_threshold → open.  Anything in between keeps
        # the previous state.
        prev = self._gripper_state_left if side == "left" else self._gripper_state_right
        if signal >= self.cfg.gripper_close_threshold:
            new = GRIPPER_CLOSE
        elif signal <= self.cfg.gripper_open_threshold:
            new = GRIPPER_OPEN
        else:
            new = prev
        if side == "left":
            self._gripper_state_left = new
        else:
            self._gripper_state_right = new
        return new

    @staticmethod
    def _read_trigger_grip(
        snapshot: Mapping[str, Any],
        action_inputs: Optional[Mapping[str, Mapping[str, Any]]],
        side: str,
    ) -> Tuple[float, float]:
        """Prefer action-API values; fall back to legacy controller buttons."""
        if action_inputs:
            ai = action_inputs.get(side) or {}
            t = float(ai.get("trigger", 0.0))
            g = float(ai.get("grip", 0.0))
            if t > 1e-3 or g > 1e-3:
                return t, g
        controllers = snapshot.get("controllers") or {}
        ctrl = controllers.get(side)
        if isinstance(ctrl, dict):
            buttons = ctrl.get("buttons") or {}
            t = float(buttons.get("trigger", 0.0))
            g = float(buttons.get("grip", 0.0))
            return t, g
        return 0.0, 0.0

    # ── Diagnostics ────────────────────────────────────────────────────
    def _print_periodic_log(
        self,
        action: torch.Tensor,
        src_l: str,
        src_r: str,
    ) -> None:
        if self._frames == 1:
            print(
                "[GR1T2Gripper #1 first-call] 16D action vector "
                f"(L={src_l}, R={src_r})"
            )
            l_pos = action[0:3].tolist()
            r_pos = action[7:10].tolist()
            print(
                f"  L_wrist: pos=({l_pos[0]:+.3f},{l_pos[1]:+.3f},{l_pos[2]:+.3f}) "
                f"gripper={float(action[14]):+.0f}"
            )
            print(
                f"  R_wrist: pos=({r_pos[0]:+.3f},{r_pos[1]:+.3f},{r_pos[2]:+.3f}) "
                f"gripper={float(action[15]):+.0f}"
            )
            return
        l_pos = action[0:3].tolist()
        r_pos = action[7:10].tolist()
        # 9.28: pick the running-max metric that matches the active signal
        # source so the user sees the value that actually drives gripping.
        src = (self.cfg.gripper_signal_source or "grip").lower()
        if src == "trigger":
            mL = self._max_trigger["left"]
            mR = self._max_trigger["right"]
            tag = "max_trig"
        elif src == "both":
            mL = max(self._max_trigger["left"], self._max_grip["left"])
            mR = max(self._max_trigger["right"], self._max_grip["right"])
            tag = "max_either"
        else:
            mL = self._max_grip["left"]
            mR = self._max_grip["right"]
            tag = "max_grip"
        print(
            f"[GR1T2Gripper #{self._frames}] L={src_l} R={src_r} | "
            f"L_pos=({l_pos[0]:.3f},{l_pos[1]:.3f},{l_pos[2]:.3f}) "
            f"R_pos=({r_pos[0]:.3f},{r_pos[1]:.3f},{r_pos[2]:.3f}) | "
            f"L_cmd={float(action[14]):+.0f} R_cmd={float(action[15]):+.0f} | "
            f"{tag} L={mL:.2f} R={mR:.2f}"
        )
