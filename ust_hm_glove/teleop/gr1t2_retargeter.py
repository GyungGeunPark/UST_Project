"""SteamVR snapshot → Fourier GR1T2 Pink IK 36D action.

This is the GR1T2 counterpart of ``ust_hm_glove.teleop.g1_retargeter``.
The overall structure (forearm → wrist, SVR → Isaac Lab coordinates,
priority A/B/C sourcing) is unchanged; the differences are:

* **36D action** (vs 38D for G1) — see :data:`ACTION_DIM`.
* **Hand joint layout is 11 per side / 22 total** (Fourier 6-DoF hand's
  6 drivers + 5 mimic), ordered per Isaac Lab's
  ``pickplace_gr1t2_env_cfg.py`` ``hand_joint_names`` global order.
* **Pink IK target frame is the palm** (``*_hand_pitch_link``) so all
  seven arm joints contribute to EEF orientation — no more 5-DoF-arm
  artefact.
* **Right wrist quaternion carries a 180° Z-axis correction** matching
  the internal ``GR1T2Retargeter._retarget_abs`` behaviour; UDCAP wrists
  therefore land in the same URDF control frame.
* **Base link is ``base_link``** (not ``pelvis``) — the ``use_waist_origin``
  logic subtracts the hips tracker XY (Z optional) so the target lands
  in the robot's ``base_link`` frame just as Pink IK expects.

The output layout (length 36):

    [0:3]    left wrist position   (base_link frame, metres)
    [3:7]    left wrist quaternion (wxyz)
    [7:10]   right wrist position
    [10:14]  right wrist quaternion (wxyz)
    [14:36]  22 hand joints (see
             :func:`fourier_hand_mapper.pack_22d` / Isaac Lab global order)

Finger sourcing (per side):

    Primary — ``dex-retargeting`` DexPilot solver loaded from a Fourier
              YAML.  Input is the fingertip positions in the hand model
              frame; output is the 11-joint target for that side.
    Fallback — :class:`fourier_hand_mapper.FourierHandMapper` on either
              UDCAP VMC bones or the raw SteamVR 31-bone Skeletal data.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import torch

# Reuse the hardware-generic helpers from the Windows G1 port.
from ust_ws.ust_hm_glove.teleop import coord_transforms as ct
from ust_ws.ust_hm_glove.teleop.fingertip_extractor import (
    extract_fingertips_wrist_frame,
)

from .fourier_hand_mapper import (
    FOURIER_JOINT_DIM,
    FOURIER_TOTAL_HAND_JOINTS,
    FourierHandMapper,
    pack_22d,
)


__all__ = [
    "GR1T2FourierSteamVRRetargeter",
    "GR1T2FourierRetargeterCfg",
    "DEFAULT_LEFT_POS",
    "DEFAULT_LEFT_QUAT",
    "DEFAULT_RIGHT_POS",
    "DEFAULT_RIGHT_QUAT",
    "ACTION_DIM",
    "HAND_DIM_PER_SIDE",
    "HAND_DIM_TOTAL",
]


# ── Action dimensions ──────────────────────────────────────────────────
HAND_DIM_PER_SIDE = FOURIER_JOINT_DIM          # 11
HAND_DIM_TOTAL = FOURIER_TOTAL_HAND_JOINTS     # 22
ACTION_DIM: int = 7 + 7 + HAND_DIM_TOTAL       # 36


# ── Idle pose (base_link frame, palm-link target) ──────────────────────
# These defaults reproduce the approximate T-pose wrist placement used by
# ``PickPlaceGR1T2EnvCfg`` when the robot is freshly spawned at
# ``pos=(0,0,0.93), rot=(0.7071,0,0,0.7071)``.  Users should re-measure via
# ``scripts/calibrate_gr1t2_idle_pose.py`` and replace these values with
# the calibrated result to match their URDF/USD exactly.
DEFAULT_LEFT_POS: Tuple[float, float, float] = (-0.20, 0.00, 1.05)
DEFAULT_LEFT_QUAT: Tuple[float, float, float, float] = (0.707, 0.0, 0.707, 0.0)
DEFAULT_RIGHT_POS: Tuple[float, float, float] = (0.20, 0.00, 1.05)
DEFAULT_RIGHT_QUAT: Tuple[float, float, float, float] = (0.707, 0.0, 0.707, 0.0)


# 180° rotation about +Z (wxyz).  Isaac Lab's GR1T2Retargeter applies
# ``z_axis_rot_quat = [0, 0, 0, 1]`` (xyzw) to the right wrist so that the
# OpenXR palm frame maps to the USD control frame.  We replicate that here.
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
class _HandSolverHandle:
    """Optional ``dex-retargeting`` optimizer handle, gracefully missing."""

    optimizer: Any = None
    num_dof: int = 0

    @property
    def available(self) -> bool:
        return self.optimizer is not None


@dataclass
class GR1T2FourierRetargeterCfg:
    """Construction parameters for :class:`GR1T2FourierSteamVRRetargeter`."""

    position_scale: float = 1.0
    rotation_scale: float = 1.0
    gripper_threshold: float = 0.5

    body_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    controller_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    forearm_wrist_offset: Tuple[float, float, float] = (0.12, 0.0, 0.0)

    # dex-retargeting YAML paths; when missing the DexPilot solver is
    # disabled and finger joints come from ``FourierHandMapper``.
    left_hand_retarget_yaml: Optional[str] = None
    right_hand_retarget_yaml: Optional[str] = None
    dex_urdf_dir: Optional[str] = None

    # Express EEF targets in the robot's base_link frame by subtracting the
    # waist tracker XY.  Pink IK expects base_link-relative targets.
    use_waist_origin: bool = True
    subtract_waist_z: bool = False

    # Diagnostic: freeze wrist orientation to idle quaternion (position-only
    # IK).  Matches the G1 diagnostic flag for consistency.
    freeze_orientation: bool = False

    # Apply the 180° Z rotation to the right wrist quaternion so the UDCAP
    # target lands in GR1T2's URDF control frame.  Leave True unless your
    # Pink IK env is configured to expect the OpenXR convention verbatim.
    right_wrist_z180: bool = True

    # 9.14 — when True, the controller pose is preferred over the forearm
    # tracker for the wrist EEF target.  Default False = legacy behaviour
    # (forearm tracker first, controller fallback).  Recommended ON for
    # users on Virtual Desktop Full Body Tracking, whose AI-inferred
    # ``arm_lower`` tracker badly compresses overhead motion (~30 cm
    # SVR-Z range observed even when user raises arm fully overhead).
    # Touch / knuckles controllers are tracked directly by the headset
    # and therefore reflect the wrist pose accurately.
    prefer_controller_for_eef: bool = False
    # When ``prefer_controller_for_eef`` is True, this offset is added in
    # the controller's local frame to convert "controller body" to
    # "wrist".  Touch is held with the body roughly between the thumb
    # and index finger; the wrist sits ~5 cm in the -Z direction
    # (toward the user's body).
    controller_to_wrist_offset: Tuple[float, float, float] = (0.0, 0.0, -0.05)

    # ── finger fallback mapper tuning ──
    hand_proximal_scale: float = 1.0
    hand_thumb_scale: float = 1.0
    # 9.12 fix — when True, FourierHandMapper uses tanh-shaped
    # amplification (sensitive at small curls, smooth saturation near
    # joint limit) instead of the legacy linear path which hard-clips
    # once raw*scale > 1.  Strongly recommended for UDCAP gloves whose
    # left/right sensors deliver up to ~30% asymmetric raw signal —
    # linear scale guarantees one hand flat-lines at the joint limit
    # before the other reaches mid-range.  See memory.md §10.21.
    hand_use_tanh_amplification: bool = True
    # 9.14 — VMC bone REST POSE calibration.  9.18 default lowered 30 → 10.
    hand_vmc_subtract_rest: bool = True
    hand_vmc_rest_frames: int = 10

    # When SteamVR skeletal data is unavailable (UDCAP driver does not
    # emit Skeletal Input 2.0 or the binding didn't match the controller
    # profile), fall back to driving the Fourier hand from the controller
    # trigger/grip analog axes.  This at least gives the user a binary
    # pinch / power-grip gesture for pick-and-place teleop.  Safe to leave
    # enabled — if all higher-priority sources are available the trigger
    # fallback is never consulted.
    enable_button_grip_fallback: bool = True
    # Treat trigger+grip as thumb-opposition trigger (pinch gesture) when
    # either axis exceeds this.
    button_grip_pinch_threshold: float = 0.3

    # 9.26 fix: when True, the wrist EEF target is FORCED to idle pose
    # regardless of forearm tracker / controller state.  Use this when the
    # user has neither real wrist trackers NOR wants the controllers to
    # drive the wrist (the typical "no-tracker rig + intentional T-pose
    # arms" use case).  9.25 routed `--ignore_trackers` through
    # `prefer_controller=True` which made the controller drive the wrist
    # instead — the OPPOSITE of the user's intent ("팔은 트래커가 있을
    # 때만 움직여야 한다").  9.26 introduces this dedicated flag so
    # `--ignore_trackers` can lock the arm at idle without conflating
    # with the controller path.  See memory.md §10.34.
    disable_arm_tracking: bool = False

    # 9.23 fix: single-pole low-pass on the 22D finger output.  The env
    # step rate is 20 Hz (decimation 6 over physics 120 Hz) but UDCAP
    # broadcasts at ~140 Hz and SteamVR samples at 120 Hz — only 1 of
    # every ~7 source frames actually reaches the action manager.
    # ``map_hand_*`` is invoked stateless every advance, so any fast
    # finger motion shows up as jagged 0↔limit jumps in the [GR1T2Retarget
    # #N] logs (e.g. l_idx swinging -1.26 → -0.00 → -1.26 across 40
    # frames in the 9.22 user video).  This was the missing analog of the
    # ``WaistEstimator``/``HeadEstimator`` ``low_pass_alpha`` filtering
    # that had been wired into both of those estimators since 9.13/9.18
    # but never extended to the finger output.  ``alpha = 0.4`` smooths
    # the typical 1-frame outlier without adding perceptible lag.  Set to
    # 1.0 to disable (legacy 9.22 behaviour) or to 0.2 for very strong
    # smoothing.  See memory.md §10.31.
    finger_low_pass_alpha: float = 0.4

    debug: bool = False

    # Overridable idle defaults (e.g. after running calibrate_gr1t2_idle_pose.py).
    idle_left_pos: Tuple[float, float, float] = DEFAULT_LEFT_POS
    idle_left_quat: Tuple[float, float, float, float] = DEFAULT_LEFT_QUAT
    idle_right_pos: Tuple[float, float, float] = DEFAULT_RIGHT_POS
    idle_right_quat: Tuple[float, float, float, float] = DEFAULT_RIGHT_QUAT


class GR1T2FourierSteamVRRetargeter:
    """SteamVR snapshot → 36D Pink IK action for GR1T2 (pure math)."""

    ACTION_DIM = ACTION_DIM
    HAND_DIM_PER_SIDE = HAND_DIM_PER_SIDE
    HAND_DIM_TOTAL = HAND_DIM_TOTAL

    def __init__(
        self,
        cfg: Optional[GR1T2FourierRetargeterCfg] = None,
        **kwargs: Any,
    ) -> None:
        if cfg is None:
            cfg = GR1T2FourierRetargeterCfg(**kwargs)
        elif kwargs:
            raise TypeError(
                "pass either a GR1T2FourierRetargeterCfg or keyword args, not both"
            )
        self.cfg = cfg

        self.finger_mapper = FourierHandMapper(
            proximal_scale=cfg.hand_proximal_scale,
            thumb_scale=cfg.hand_thumb_scale,
            use_tanh=cfg.hand_use_tanh_amplification,
            vmc_subtract_rest=cfg.hand_vmc_subtract_rest,
            vmc_rest_frames=cfg.hand_vmc_rest_frames,
        )

        self._dex_left = self._build_dex_solver(
            cfg.left_hand_retarget_yaml, cfg.dex_urdf_dir
        )
        self._dex_right = self._build_dex_solver(
            cfg.right_hand_retarget_yaml, cfg.dex_urdf_dir
        )

        # Diagnostics / instrumentation
        self._frames = 0
        self._source_left = "default"
        self._source_right = "default"
        self._finger_source_left = "idle"
        self._finger_source_right = "idle"
        # Per-finger min/max tracker for the periodic debug print.  Lets us
        # immediately tell whether finger targets are constant (min == max,
        # the symptom of frozen retargeter output) or actually varying.
        # 9.17 — track ALL 5 finger drivers per side (not just index/thumb).
        # When user reports "fingers don't match" we need to see whether
        # middle/ring/pinky are responding too.  Slot indices below match
        # pack_22d() output:
        #   slot 0 = L_index_proximal, 1 = L_middle, 2 = L_pinky, 3 = L_ring, 4 = L_thumb_yaw
        #   slot 5 = R_index_proximal, 6 = R_middle, 7 = R_pinky, 8 = R_ring, 9 = R_thumb_yaw
        self._finger_track: Dict[str, Tuple[float, float]] = {
            f"{side}_{name}": (float("inf"), float("-inf"))
            for side in ("l", "r")
            for name in ("idx", "mid", "pky", "rng", "thb")
        }
        # Per-tracker SVR-frame min/max: lets the user see whether the raw
        # forearm/waist tracker is actually moving (vs the retargeter's
        # post-conversion output).  9.13 — added because Virtual Desktop AI
        # body tracking can yield trackers whose position drifts only ~5cm
        # even when the user moves their arm overhead, and without this
        # readout it's impossible to distinguish "tracker not moving" from
        # "conversion bug".
        self._svr_pose_track: Dict[str, Tuple[float, float]] = {
            "l_arm_z": (float("inf"), float("-inf")),
            "r_arm_z": (float("inf"), float("-inf")),
            "waist_pitch": (float("inf"), float("-inf")),
        }
        # 9.16 — one-shot finger-scale recommendation flag.
        self._scale_advice_emitted: bool = False
        # 9.18 — one-shot stuck-finger warning flag.
        self._stuck_finger_warned: bool = False
        # 9.23 — previous frame's 22D finger output for the EMA filter.
        # ``None`` until the first ``retarget()`` call so the initial
        # frame is passed through verbatim (no lag at session start).
        self._prev_finger_22: Optional[np.ndarray] = None

    # ── dex-retargeting plumbing ────────────────────────────────────────
    @staticmethod
    def _build_dex_solver(
        yaml_path: Optional[str], urdf_dir: Optional[str]
    ) -> _HandSolverHandle:
        if not yaml_path:
            return _HandSolverHandle()
        try:
            from dex_retargeting.retargeting_config import RetargetingConfig  # type: ignore
        except Exception:  # noqa: BLE001
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
            print(
                f"[GR1T2FourierRetargeter] dex-retargeting solver unavailable ({exc});"
                f" falling back to FourierHandMapper."
            )
            return _HandSolverHandle()

    # ── public API ─────────────────────────────────────────────────────
    def retarget(
        self,
        snapshot: Mapping[str, Any],
        udcap_bones: Optional[Dict[str, Tuple[float, float, float, float]]] = None,
        action_inputs: Optional[Mapping[str, Mapping[str, Any]]] = None,
    ) -> torch.Tensor:
        """Return a 36D Pink IK action for ``snapshot``.

        Parameters
        ----------
        snapshot:
            Latest ``SteamVRSampler`` snapshot (hmd, trackers, hands,
            controllers).
        udcap_bones:
            Optional VMC bone dict (Path B).  Used as a finger fallback
            when skeletal isn't available.
        action_inputs:
            Optional ``{"left": {...}, "right": {...}}`` mapping, each
            inner dict carrying ``trigger`` (float), ``grip`` (float),
            ``finger_curls`` (list[5] thumb/index/middle/ring/pinky).
            This is how ``GR1T2FourierUDCAPDevice`` forwards the action-
            API values it reads for knuckles emulators that only emit
            per-finger curl / trigger / grip through the modern action
            system (not legacy ``getControllerState()``).  Pass ``None``
            when the caller can't supply action-API values; the
            retargeter will fall back to legacy button state in the
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

        hand_22 = self._resolve_hand_joints(snapshot, udcap_bones, action_inputs)
        # 9.23 fix: single-pole low-pass on the 22D finger output to
        # suppress 20 Hz env-step aliasing of the 140 Hz UDCAP stream.
        # ``alpha == 1.0`` disables the filter (legacy behaviour); the
        # first frame always passes through verbatim so cold-start
        # behaviour is unchanged.  This is the missing finger analog of
        # the existing ``WaistEstimator``/``HeadEstimator`` low-pass
        # filtering — see memory.md §10.31 for the derivation and the
        # measured before/after on l_idx_proximal jagged motion.
        a = float(self.cfg.finger_low_pass_alpha)
        if 0.0 < a < 1.0 and self._prev_finger_22 is not None:
            hand_22 = (a * hand_22 + (1.0 - a) * self._prev_finger_22).astype(np.float32)
        self._prev_finger_22 = hand_22.copy()
        action[14 : 14 + self.HAND_DIM_TOTAL] = torch.from_numpy(hand_22.astype(np.float32))

        self._frames += 1
        # Dump the entire 22D finger action vector ONCE on the first call so
        # that the user can verify (a) the values are not all zero, (b) what
        # order of magnitude to expect, even if the teleop session is too
        # short to ever reach frame 20.  This is the highest-priority
        # diagnostic when fingers don't move on the robot.
        if self.cfg.debug and self._frames == 1:
            print(
                "[GR1T2Retarget #1 first-call] 22D finger vector "
                f"(source L={self._finger_source_left}, R={self._finger_source_right}):"
            )
            for i, name in enumerate((
                "L_idx_prox", "L_mid_prox", "L_pky_prox", "L_rng_prox", "L_thb_yaw",
                "R_idx_prox", "R_mid_prox", "R_pky_prox", "R_rng_prox", "R_thb_yaw",
                "L_idx_int",  "L_mid_int",  "L_pky_int",  "L_rng_int",  "L_thb_pit",
                "R_idx_int",  "R_mid_int",  "R_pky_int",  "R_rng_int",  "R_thb_pit",
                "L_thb_dist", "R_thb_dist",
            )):
                print(f"  [{i:2d}] {name:11s} = {float(action[14 + i]):+.4f} rad")
        # Print every 20 frames (~1 s @ 20 Hz teleop) instead of 60 so that a
        # short teleop session — even one that the user terminates after a
        # couple of seconds — still produces enough samples to diagnose
        # whether finger values are varying.
        if self.cfg.debug and self._frames % 20 == 0:
            n_trk = len(snapshot.get("trackers") or {})
            # Update raw-SVR tracker tracking (so the periodic log shows
            # whether trackers themselves are moving — independent of
            # the conversion / waist-origin pipeline).
            trackers = snapshot.get("trackers") or {}
            l_arm_pose = trackers.get("left_forearm")
            r_arm_pose = trackers.get("right_forearm")
            waist_pose = trackers.get("waist")
            if l_arm_pose and l_arm_pose.get("pos") is not None:
                z = float(l_arm_pose["pos"][1])  # SVR Y = up
                lo, hi = self._svr_pose_track["l_arm_z"]
                self._svr_pose_track["l_arm_z"] = (min(lo, z), max(hi, z))
            if r_arm_pose and r_arm_pose.get("pos") is not None:
                z = float(r_arm_pose["pos"][1])
                lo, hi = self._svr_pose_track["r_arm_z"]
                self._svr_pose_track["r_arm_z"] = (min(lo, z), max(hi, z))
            if waist_pose and waist_pose.get("quat") is not None:
                wq = waist_pose["quat"]
                # rough pitch from quat: 2*(w*y - z*x)
                pitch_sin = 2.0 * (float(wq[0]) * float(wq[2]) - float(wq[3]) * float(wq[1]))
                pitch_sin = max(-1.0, min(1.0, pitch_sin))
                import math as _m
                pitch_deg = _m.degrees(_m.asin(pitch_sin))
                lo, hi = self._svr_pose_track["waist_pitch"]
                self._svr_pose_track["waist_pitch"] = (min(lo, pitch_deg), max(hi, pitch_deg))
            # 9.17 — pull all 5 driver joints per side from action[14:36].
            # Slots in pack_22d order:
            #   0 = L_index_proximal, 1 = L_middle_proximal,
            #   2 = L_pinky_proximal, 3 = L_ring_proximal,
            #   4 = L_thumb_proximal_yaw,
            #   5..9 = R_index/middle/pinky/ring/thumb_yaw.
            slot_map = {
                "l_idx": 0, "l_mid": 1, "l_pky": 2, "l_rng": 3, "l_thb": 4,
                "r_idx": 5, "r_mid": 6, "r_pky": 7, "r_rng": 8, "r_thb": 9,
            }
            now = {
                key: float(action[14 + slot])
                for key, slot in slot_map.items()
            }
            # Update running min/max for every finger.
            for key, val in now.items():
                lo, hi = self._finger_track[key]
                self._finger_track[key] = (min(lo, val), max(hi, val))

            la = self._svr_pose_track["l_arm_z"]
            ra = self._svr_pose_track["r_arm_z"]
            wp = self._svr_pose_track["waist_pitch"]
            la_range = (la[1] - la[0]) if la[0] != float("inf") else 0.0
            ra_range = (ra[1] - ra[0]) if ra[0] != float("inf") else 0.0
            wp_range = (wp[1] - wp[0]) if wp[0] != float("inf") else 0.0

            def _fmt(key: str) -> str:
                lo, hi = self._finger_track[key]
                rng = hi - lo if lo != float("inf") else 0.0
                return f"{key}={now[key]:+.2f}[{lo:+.2f},{hi:+.2f}]Δ{rng:.2f}"

            print(
                f"[GR1T2Retarget #{self._frames}] "
                f"L={src_l}/{self._finger_source_left} "
                f"R={src_r}/{self._finger_source_right} | trackers={n_trk} | "
                f"L_pos=({left_pos[0]:.3f},{left_pos[1]:.3f},{left_pos[2]:.3f}) "
                f"R_pos=({right_pos[0]:.3f},{right_pos[1]:.3f},{right_pos[2]:.3f}) | "
                f"raw_SVR_arm_Z range: L={la_range:+.3f}m R={ra_range:+.3f}m "
                f"waist_pitch range={wp_range:+.1f}deg"
            )
            print(
                f"  L: {_fmt('l_idx')} {_fmt('l_mid')} {_fmt('l_pky')} "
                f"{_fmt('l_rng')} {_fmt('l_thb')}"
            )
            print(
                f"  R: {_fmt('r_idx')} {_fmt('r_mid')} {_fmt('r_pky')} "
                f"{_fmt('r_rng')} {_fmt('r_thb')}"
            )

        # 9.18 — stuck-finger diagnostic.  After 200 frames (~10 s),
        # check each finger's running min/max range.  If a finger has
        # Δ < 0.05 rad (~3°) it's effectively frozen — likely UDCAP
        # glove sensor not transmitting that bone OR rest cal absorbed
        # all motion.  Print a one-shot warning naming the stuck fingers.
        if (
            self.cfg.debug
            and not self._stuck_finger_warned
            and self._frames >= 200
        ):
            stuck = []
            for key, (lo, hi) in self._finger_track.items():
                if lo == float("inf"):
                    continue
                if (hi - lo) < 0.05:
                    stuck.append(key)
            if stuck:
                self._stuck_finger_warned = True
                print(
                    f"[GR1T2Retarget][stuck-finger-warn] After {self._frames} "
                    f"frames the following fingers have variation < 0.05 rad "
                    f"(~3°), suggesting UDCAP is not broadcasting them "
                    f"meaningfully OR rest cal absorbed all the motion:\n"
                    f"  Stuck: {stuck}\n"
                    f"  → Try: (a) restart with the user holding open hand "
                    f"perfectly still during the first 0.5 s; (b) "
                    f"--vmc_subtract_rest false (skip rest cal entirely); "
                    f"(c) check UDCAP UI per-finger heatmap to confirm the "
                    f"glove sensor is reading that finger."
                )

        # 9.16 — once-only finger-scale recommendation after ~30 s of
        # teleop.  Looks at L/R index max curl reached; if either side
        # is far below the joint limit (1.57 rad full curl), recommend
        # scaling up.  If asymmetric, recommend per-side tuning.
        if (
            self.cfg.debug
            and not self._scale_advice_emitted
            and self._frames >= 600
        ):
            self._scale_advice_emitted = True
            l_max = abs(self._finger_track["l_idx"][0])  # most-negative magnitude
            r_max = abs(self._finger_track["r_idx"][0])
            limit = 1.57
            l_pct = 100.0 * l_max / limit
            r_pct = 100.0 * r_max / limit
            asymmetry = abs(l_max - r_max)
            # 9.17 — also report middle/ring/pinky reachability so the user
            # can detect the case "index works but other fingers don't".
            def _pct(key: str) -> float:
                rng = self._finger_track[key]
                if rng[0] == float("inf"):
                    return 0.0
                return 100.0 * abs(rng[0]) / limit
            advice_lines = [
                "[GR1T2Retarget][finger-scale-advice] After ~30 s of teleop:",
                f"  L_idx max curl reached = {l_max:.2f} rad ({l_pct:.0f}% of joint limit)",
                f"  R_idx max curl reached = {r_max:.2f} rad ({r_pct:.0f}% of joint limit)",
                f"  Per-finger max %: "
                f"L(idx={_pct('l_idx'):.0f}, mid={_pct('l_mid'):.0f}, "
                f"pky={_pct('l_pky'):.0f}, rng={_pct('l_rng'):.0f}) "
                f"R(idx={_pct('r_idx'):.0f}, mid={_pct('r_mid'):.0f}, "
                f"pky={_pct('r_pky'):.0f}, rng={_pct('r_rng'):.0f})",
            ]
            # Detect the "only index moves" pattern.
            l_others = max(_pct("l_mid"), _pct("l_pky"), _pct("l_rng"))
            r_others = max(_pct("r_mid"), _pct("r_pky"), _pct("r_rng"))
            if l_pct > 50 and l_others < 20:
                advice_lines.append(
                    "  → LEFT hand: index moves but middle/ring/pinky stuck. "
                    "Likely UDCAP glove sensor not transmitting those bones, "
                    "OR rest pose absorbed all motion.  Try restart with the "
                    "user holding open hand still during cal window."
                )
            if r_pct > 50 and r_others < 20:
                advice_lines.append(
                    "  → RIGHT hand: index moves but middle/ring/pinky stuck "
                    "(same diagnosis as above for the right glove)."
                )
            if max(l_pct, r_pct) < 60.0:
                cur = self.cfg.hand_proximal_scale
                suggest = max(2.0, cur * (limit / max(l_max, r_max, 0.1)))
                advice_lines.append(
                    f"  → Both hands under 60% of full curl.  Bump "
                    f"--finger_proximal_scale from {cur:.1f} to ~{suggest:.1f} "
                    f"(or further if user expected fuller curl)."
                )
            elif asymmetry > 0.4:
                advice_lines.append(
                    f"  → L/R asymmetry {asymmetry:.2f} rad (~{asymmetry*180/3.14159:.0f}°). "
                    f"This is the typical UDCAP RSSI imbalance.  Tanh "
                    f"amplification (default ON) helps; if still uneven, "
                    f"bump --finger_proximal_scale to amplify the weaker "
                    f"side, then accept the stronger side may saturate."
                )
            elif min(l_pct, r_pct) > 90.0:
                advice_lines.append(
                    f"  → Both hands at 90%+ of full curl.  Possibly "
                    f"saturating; if mid-fist already maxes out, "
                    f"--finger_proximal_scale lower (1.5-2.0) gives more "
                    f"dynamic range."
                )
            else:
                advice_lines.append(
                    "  → Curl range looks reasonable; no scale change needed."
                )
            print("\n".join(advice_lines))
        return action

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
        self._finger_track = {
            "l_idx": (float("inf"), float("-inf")),
            "r_idx": (float("inf"), float("-inf")),
            "l_thb": (float("inf"), float("-inf")),
            "r_thb": (float("inf"), float("-inf")),
        }
        self._svr_pose_track = {
            "l_arm_z": (float("inf"), float("-inf")),
            "r_arm_z": (float("inf"), float("-inf")),
            "waist_pitch": (float("inf"), float("-inf")),
        }
        self._scale_advice_emitted = False
        self._stuck_finger_warned = False
        # 9.23: forget the previous finger output so the next call's
        # initial frame passes through verbatim (no carry-over from a
        # prior session).
        self._prev_finger_22 = None

    # ── EEF resolution ─────────────────────────────────────────────────
    def _user_pelvis_origin_il(
        self, snapshot: Mapping[str, Any]
    ) -> Optional[np.ndarray]:
        """Return the user's pelvis origin in Isaac Lab coordinates.

        Uses the ``waist`` tracker for the planar (X, Y) and optionally Z
        offset.  Returns ``None`` if the tracker is missing so EEF targets
        remain in raw world coordinates.
        """
        if not self.cfg.use_waist_origin:
            return None
        trackers = snapshot.get("trackers") or {}
        waist = trackers.get("waist")
        if _is_zero_pose(waist):
            return None
        waist_pos_il, _ = ct.svr_to_isaaclab(waist["pos"], waist["quat"])
        if not self.cfg.subtract_waist_z:
            return np.array([waist_pos_il[0], waist_pos_il[1], 0.0], dtype=np.float64)
        return waist_pos_il.astype(np.float64)

    def _resolve_eef_target(
        self,
        snapshot: Mapping[str, Any],
        side: str,
    ) -> Tuple[np.ndarray, np.ndarray, str]:
        # 9.26 fix: force idle pose when arm tracking is intentionally
        # disabled (e.g. no-tracker rig where the user wants the arms to
        # stay in T-pose).  Bypasses both forearm and controller paths so
        # the robot's wrists stay at the idle target indefinitely.
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
            pos_il, quat_il = ct.svr_to_isaaclab(wrist["pos"], wrist["quat"])
            if origin_il is not None:
                pos_il = pos_il - origin_il
            pos_il = pos_il * self.cfg.position_scale + cfg_offset
            quat_il = self._apply_orientation(quat_il, side)
            return pos_il, quat_il, "forearm"

        def _from_controller() -> Tuple[np.ndarray, np.ndarray, str]:
            pose = ctrl["pose"]
            # Apply controller→wrist offset in the controller's local frame
            # before SVR→IL conversion.  Reuses the forearm_to_wrist helper
            # by faking a "forearm pose" — same math (rotate offset by quat,
            # add to pos) — just with a different offset constant.
            wrist = ct.forearm_to_wrist(pose, self.cfg.controller_to_wrist_offset)
            pos_il, quat_il = ct.svr_to_isaaclab(wrist["pos"], wrist["quat"])
            if origin_il is not None:
                pos_il = pos_il - origin_il
            pos_il = pos_il * self.cfg.position_scale + ctrl_offset
            quat_il = self._apply_orientation(quat_il, side)
            return pos_il, quat_il, "controller"

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

    def _apply_orientation(self, quat_il: np.ndarray, side: str) -> np.ndarray:
        """Apply the diagnostic freeze and the right-wrist 180° Z correction."""
        if self.cfg.freeze_orientation:
            idle_q = self.cfg.idle_left_quat if side == "left" else self.cfg.idle_right_quat
            return np.asarray(idle_q, dtype=np.float64)
        if side == "right" and self.cfg.right_wrist_z180:
            quat_il = ct.quat_multiply(quat_il, _Z180_WXYZ)
        return quat_il.astype(np.float64)

    # ── hand resolution ────────────────────────────────────────────────
    def _resolve_hand_joints(
        self,
        snapshot: Mapping[str, Any],
        udcap_bones: Optional[Dict[str, Tuple[float, float, float, float]]],
        action_inputs: Optional[Mapping[str, Mapping[str, Any]]] = None,
    ) -> np.ndarray:
        """Resolve the 22 hand joints with a clear priority chain::

            1. DexPilot             (requires Fourier hand URDF + skeletal fingertips)
            2. Skeletal bones       (31-bone SteamVR Skeletal)
            3. SteamVR finger-curl  (5 per-hand curls via action API — primary
                                     for UDCAP and other Index emulators that
                                     emit per-finger curl but no skeleton)
            4. VMC bones            (Path B, OSC-delivered VMC)
            5. Trigger/grip         (action-API preferred, legacy as fallback —
                                     binary pinch gesture)
            6. Idle / zero

        When UDCAP's SteamVR driver does not implement Skeletal Input 2.0
        (the common case for LucidVR-family gloves), skeletal stays
        ``bActive=False`` and we fall through to finger-curl (step 3).
        That path reads the Valve Index ``/user/hand/*/input/finger/*``
        values that UDCAP does emit, so the user gets 5-DoF-per-hand
        control without a Skeletal-capable driver.
        """
        left_11 = np.zeros(HAND_DIM_PER_SIDE, dtype=np.float32)
        right_11 = np.zeros(HAND_DIM_PER_SIDE, dtype=np.float32)

        hands = snapshot.get("hands") or {}
        left_hand = hands.get("left")
        right_hand = hands.get("right")
        controllers = snapshot.get("controllers") or {}
        ctrl_left = controllers.get("left")
        ctrl_right = controllers.get("right")

        left_resolved = False
        right_resolved = False

        # 1. DexPilot (when Fourier hand URDF available and skeletal data present).
        if left_hand is not None and self._dex_left.available:
            try:
                tips = extract_fingertips_wrist_frame(left_hand["bones"])
                left_11 = self._dex_to_fourier11(self._dex_left, tips)
                self._finger_source_left = "dex"
                left_resolved = True
            except Exception:  # noqa: BLE001
                pass

        if right_hand is not None and self._dex_right.available:
            try:
                tips = extract_fingertips_wrist_frame(right_hand["bones"])
                right_11 = self._dex_to_fourier11(self._dex_right, tips)
                self._finger_source_right = "dex"
                right_resolved = True
            except Exception:  # noqa: BLE001
                pass

        # 2. SteamVR skeletal → Fourier mapper
        if not left_resolved and left_hand is not None:
            left_11 = self.finger_mapper.map_hand_skeletal(
                left_hand["bones"],
                is_right=False,
                finger_curls=left_hand.get("fingerCurls"),
            )
            self._finger_source_left = "skeletal"
            left_resolved = True
        if not right_resolved and right_hand is not None:
            right_11 = self.finger_mapper.map_hand_skeletal(
                right_hand["bones"],
                is_right=True,
                finger_curls=right_hand.get("fingerCurls"),
            )
            self._finger_source_right = "skeletal"
            right_resolved = True

        # 3. SteamVR per-finger curl actions (primary source for UDCAP).
        #    Treats a (side) source as providing data only when at least
        #    one of the five curls is above a small deadband — otherwise
        #    the chain falls through to the legacy / button fallback so
        #    resting hands don't lock into the finger-curl path.
        if action_inputs:
            left_act = action_inputs.get("left") or {}
            right_act = action_inputs.get("right") or {}
            left_curls = list(left_act.get("finger_curls") or ())
            right_curls = list(right_act.get("finger_curls") or ())
            if (
                not left_resolved
                and len(left_curls) >= 5
                and any(float(c) > 1e-3 for c in left_curls[:5])
            ):
                left_11 = self.finger_mapper.map_from_finger_curls(left_curls[:5])
                self._finger_source_left = "finger_action"
                left_resolved = True
            if (
                not right_resolved
                and len(right_curls) >= 5
                and any(float(c) > 1e-3 for c in right_curls[:5])
            ):
                right_11 = self.finger_mapper.map_from_finger_curls(right_curls[:5])
                self._finger_source_right = "finger_action"
                right_resolved = True

        # 4. VMC OSC (Path B)
        if not left_resolved and udcap_bones:
            left_11 = self.finger_mapper.map_hand_vmc(udcap_bones, is_right=False)
            self._finger_source_left = "vmc"
            left_resolved = True
        if not right_resolved and udcap_bones:
            right_11 = self.finger_mapper.map_hand_vmc(udcap_bones, is_right=True)
            self._finger_source_right = "vmc"
            right_resolved = True

        # 5. Controller trigger/grip → binary-grip gesture.
        #    Prefer action-API values (knuckles-compatible) over the
        #    sampler's legacy ``getControllerState()`` path which returns
        #    zero for most knuckles emulators.
        def _pick_trigger_grip(side: str, legacy_ctrl: Any) -> Tuple[float, float]:
            if action_inputs:
                act = (action_inputs.get(side) or {}) if action_inputs else {}
                t, g = float(act.get("trigger", 0.0)), float(act.get("grip", 0.0))
                if t > 1e-3 or g > 1e-3:
                    return t, g
            buttons = (legacy_ctrl.get("buttons") or {}) if isinstance(legacy_ctrl, dict) else {}
            return float(buttons.get("trigger", 0.0)), float(buttons.get("grip", 0.0))

        if self.cfg.enable_button_grip_fallback:
            if not left_resolved:
                t, g = _pick_trigger_grip("left", ctrl_left)
                if t > 1e-3 or g > 1e-3:
                    thumb_touch = (
                        t > self.cfg.button_grip_pinch_threshold
                        or g > self.cfg.button_grip_pinch_threshold
                    )
                    left_11 = self.finger_mapper.map_from_controller_buttons(
                        trigger=t, grip=g, thumb_touch=thumb_touch,
                    )
                    self._finger_source_left = "button"
                    left_resolved = True
            if not right_resolved:
                t, g = _pick_trigger_grip("right", ctrl_right)
                if t > 1e-3 or g > 1e-3:
                    thumb_touch = (
                        t > self.cfg.button_grip_pinch_threshold
                        or g > self.cfg.button_grip_pinch_threshold
                    )
                    right_11 = self.finger_mapper.map_from_controller_buttons(
                        trigger=t, grip=g, thumb_touch=thumb_touch,
                    )
                    self._finger_source_right = "button"
                    right_resolved = True

        # 6. Idle — keep the default zero vector
        if not left_resolved:
            self._finger_source_left = "idle"
        if not right_resolved:
            self._finger_source_right = "idle"

        return pack_22d(left_11, right_11)

    @staticmethod
    def _dex_to_fourier11(
        handle: _HandSolverHandle,
        fingertips_wrist_frame: np.ndarray,
    ) -> np.ndarray:
        """Run the DexPilot optimizer and down-project the result to 11 DoF.

        The Fourier dex-retargeting YAML declares 11 ``target_joint_names``
        in the canonical (proximal-drivers, intermediate-mimic, distal)
        order; we truncate/pad the solver output to the first 11 elements.
        """
        if not handle.available:
            return np.zeros(HAND_DIM_PER_SIDE, dtype=np.float32)
        try:
            q = handle.optimizer.retarget(
                np.asarray(fingertips_wrist_frame, dtype=np.float64)
            )
        except Exception:  # noqa: BLE001
            return np.zeros(HAND_DIM_PER_SIDE, dtype=np.float32)
        q = np.asarray(q, dtype=np.float32).reshape(-1)
        out = np.zeros(HAND_DIM_PER_SIDE, dtype=np.float32)
        n = min(q.size, HAND_DIM_PER_SIDE)
        out[:n] = q[:n]
        return out
