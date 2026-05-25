"""UDCAP/VMC/Skeletal → Fourier 6-DoF hand (11-joint) fallback mapper.

The primary finger retargeting path on GR1T2 is
``dex-retargeting.DexPilot`` driven by the SteamVR Skeletal fingertip
positions (see ``GR1T2FourierSteamVRRetargeter``).  DexPilot requires:

* a valid Fourier hand URDF (USD → URDF conversion inside Isaac Lab), and
* a running ``dex-retargeting`` installation.

When either is missing this module takes over.  It maps the UDCAP glove
data (VMC bone quaternions or raw SteamVR 31-bone Skeletal) to the
**11-joint Fourier hand layout** declared in Isaac Lab's
``pickplace_gr1t2_env_cfg.py``:

    per hand::
        0  {L,R}_index_proximal_joint           (MCP pitch — driver)
        1  {L,R}_middle_proximal_joint          (MCP pitch — driver)
        2  {L,R}_pinky_proximal_joint           (MCP pitch — driver)
        3  {L,R}_ring_proximal_joint            (MCP pitch — driver)
        4  {L,R}_thumb_proximal_yaw_joint       (thumb opposition — driver)
        5  {L,R}_index_intermediate_joint       (PIP  — mimic)
        6  {L,R}_middle_intermediate_joint      (PIP  — mimic)
        7  {L,R}_pinky_intermediate_joint       (PIP  — mimic)
        8  {L,R}_ring_intermediate_joint        (PIP  — mimic)
        9  {L,R}_thumb_proximal_pitch_joint     (thumb flex — driver)
       10  {L,R}_thumb_distal_joint             (DIP — mimic)

The intermediate/distal "mimic" joints are mechanically linked on real
hardware; in simulation we emit reasonable command values and let Isaac
Sim's mimic constraints (if declared in USD) re-clamp them.

``pack_22d(left_11, right_11)`` arranges two per-side vectors into the
global 22-joint order used by Isaac Lab Pink IK
``hand_joint_names`` (``L_idx_prox, L_mid_prox, L_pnk_prox, L_rng_prox,
L_thb_yaw,  R_idx_prox, ... , L_thb_dist, R_thb_dist``).
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np


__all__ = [
    "FourierHandMapper",
    "FOURIER_JOINT_DIM",
    "FOURIER_TOTAL_HAND_JOINTS",
    "PACK_22D_SIGNS",
    "pack_22d",
]


#: Number of commanded joints per Fourier hand (6 drivers + 5 mimic).
FOURIER_JOINT_DIM = 11
#: Total hand joints across both hands (L + R).
FOURIER_TOTAL_HAND_JOINTS = 22

# Per-side joint indices (documented above).
IDX_INDEX_PROX = 0
IDX_MIDDLE_PROX = 1
IDX_PINKY_PROX = 2
IDX_RING_PROX = 3
IDX_THUMB_YAW = 4
IDX_INDEX_INT = 5
IDX_MIDDLE_INT = 6
IDX_PINKY_INT = 7
IDX_RING_INT = 8
IDX_THUMB_PITCH = 9
IDX_THUMB_DIST = 10

# Approximate URDF joint limits (radians).  These follow Fourier Hand 6-DoF
# published specs and Isaac Sim joint ranges; for the intermediate/distal
# mimic joints we reuse the driver limit because the mimic ratio already
# keeps them within bounds.
_PROXIMAL_MAX = 1.57                         # ~90 deg MCP flex
_THUMB_YAW_RANGE: Tuple[float, float] = (-0.5, 1.0)
_THUMB_PITCH_MAX = 1.2


# ── quaternion → scalar extraction helpers (VMC xyz-w convention) ──────────
def _quat_to_bend(qx: float, qy: float, qz: float, qw: float) -> float:
    """Angle magnitude normalised to ``[0, 1]``."""
    angle = 2.0 * math.acos(max(-1.0, min(1.0, abs(qw))))
    return float(max(0.0, min(1.0, angle / math.pi)))


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Z-axis rotation (yaw) — bounded to ``[-1, 1]``."""
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return float(max(-1.0, min(1.0, yaw / (math.pi / 2.0))))


def _quat_to_pitch(qx: float, qy: float, qz: float, qw: float) -> float:
    """X-axis rotation (pitch) — bounded to ``[-1, 1]``."""
    s = 2.0 * (qw * qx + qy * qz)
    c = 1.0 - 2.0 * (qx * qx + qy * qy)
    pitch = math.atan2(s, c)
    return float(max(-1.0, min(1.0, pitch / (math.pi / 2.0))))


class FourierHandMapper:
    """Map UDCAP (VMC or SteamVR Skeletal) data to Fourier 11-joint vectors.

    Parameters
    ----------
    proximal_scale:
        Scales the raw ``[0, 1]`` curl/bend values into radians by
        ``proximal_scale * _PROXIMAL_MAX``.  Leave at 1.0 unless the
        user's measured curl maxes out below the URDF limit.
    thumb_scale:
        Same, for thumb flexion.
    intermediate_mimic:
        Linear coefficient applied from proximal -> intermediate.  The
        Fourier mechanical linkage is roughly 1.0 (curling the MCP fully
        also flexes PIP fully) — we use 0.85 to leave headroom.
    distal_mimic:
        Linear coefficient applied from ``thumb_proximal_pitch`` to
        ``thumb_distal``.
    use_tanh:
        2026-04-26 9.12 amplification curve.  When True, the per-finger
        scaling uses ``out = limit * tanh(raw * scale)`` instead of the
        linear ``out = raw * scale * limit``.  Tanh keeps the small-curl
        derivative ≈ scale (good amplification near rest) but saturates
        smoothly at ``limit`` instead of hard-clipping once raw*scale > 1.
        Critical when one glove's raw signal is ~30% weaker than the
        other (UDCAP RSSI L/R imbalance): linear scaling makes the
        stronger side flat-line at the joint limit while the weaker
        side is still in mid-range.  Default ON.
    """

    def __init__(
        self,
        proximal_scale: float = 1.0,
        thumb_scale: float = 1.0,
        intermediate_mimic: float = 0.85,
        distal_mimic: float = 0.75,
        use_tanh: bool = True,
        vmc_subtract_rest: bool = True,
        vmc_rest_frames: int = 10,
    ) -> None:
        self.proximal_scale = float(proximal_scale)
        self.thumb_scale = float(thumb_scale)
        self.intermediate_mimic = float(intermediate_mimic)
        self.distal_mimic = float(distal_mimic)
        self.use_tanh = bool(use_tanh)

        # 9.14 — per-bone REST pose calibration for VMC source.  UDCAP's
        # broadcast bones are NOT identity at the user's open-hand rest:
        # observed pinky/ring proximal at ~16° offset, thumb yaw at ~14°
        # offset even when the user's hand is fully relaxed.  Without
        # subtracting that rest the mapper outputs static -0.27 rad
        # baselines on those joints, and the user-visible curl span is
        # (max - rest) instead of (max - 0).  Capture the *averaged* quat
        # over the first N frames as the per-bone rest, then return the
        # relative rotation in subsequent frames.
        self.vmc_subtract_rest = bool(vmc_subtract_rest)
        self.vmc_rest_frames = max(1, int(vmc_rest_frames))
        self._vmc_rest_quats: Dict[str, np.ndarray] = {}
        self._vmc_rest_count: Dict[str, int] = {}
        self._vmc_rest_calibrated: bool = False

    def reset_vmc_rest(self) -> None:
        """Forget the captured per-bone rest pose.  Call this when the user
        wants to re-zero the hand mapping (e.g. via a hotkey)."""
        self._vmc_rest_quats.clear()
        self._vmc_rest_count.clear()
        self._vmc_rest_calibrated = False

    def _vmc_quat_relative_to_rest(
        self, bone_name: str, quat_xyzw: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        """During the first ``vmc_rest_frames`` calls, accumulate ``bone_name``
        quats; from then on return the input quat composed with the inverse
        of the averaged rest quat.

        ``quat_xyzw`` is in (qx, qy, qz, qw) order to match the rest of
        this module.  Internal calculations use (w, x, y, z) — the math
        helpers from ``waist_estimator`` are reused.
        """
        if not self.vmc_subtract_rest:
            return quat_xyzw

        qx, qy, qz, qw = quat_xyzw
        q_in_wxyz = np.array([qw, qx, qy, qz], dtype=np.float64)
        # Normalise — VMC sources occasionally emit non-unit quats; without
        # this the relative computation never returns identity when input
        # equals the rest pose.
        n = float(np.linalg.norm(q_in_wxyz))
        if n > 1e-9:
            q_in_wxyz = q_in_wxyz / n

        if not self._vmc_rest_calibrated:
            count = self._vmc_rest_count.get(bone_name, 0)
            if count == 0:
                self._vmc_rest_quats[bone_name] = q_in_wxyz.copy()
                self._vmc_rest_count[bone_name] = 1
            else:
                # Accumulate; we'll average at the calibration moment.
                # Hemisphere consistency: flip if dot < 0.
                stored = self._vmc_rest_quats[bone_name]
                if float(np.dot(stored, q_in_wxyz)) < 0.0:
                    q_in_wxyz = -q_in_wxyz
                self._vmc_rest_quats[bone_name] = stored + q_in_wxyz
                self._vmc_rest_count[bone_name] = count + 1

            # Once any bone has reached the threshold, finalize all
            # collected rests by averaging + normalising.
            if self._vmc_rest_count[bone_name] >= self.vmc_rest_frames:
                for name, accum in list(self._vmc_rest_quats.items()):
                    n = max(1, self._vmc_rest_count.get(name, 1))
                    avg = accum / float(n)
                    norm = float(np.linalg.norm(avg))
                    if norm < 1e-9:
                        avg = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                    else:
                        avg = avg / norm
                    self._vmc_rest_quats[name] = avg
                self._vmc_rest_calibrated = True

            # During calibration phase, return identity-equivalent quat
            # (no relative rotation yet).
            return (0.0, 0.0, 0.0, 1.0)

        rest = self._vmc_rest_quats.get(bone_name)
        if rest is None:
            # Bone first observed after calibration window; treat its
            # current pose as its rest (identity output).
            self._vmc_rest_quats[bone_name] = q_in_wxyz.copy()
            return (0.0, 0.0, 0.0, 1.0)

        # q_rel = q * conj(rest)  (rotates "from rest into current").
        rw, rx, ry, rz = rest
        rest_conj = np.array([rw, -rx, -ry, -rz], dtype=np.float64)
        cw, cx, cy, cz = q_in_wxyz
        # Hamilton product (w,x,y,z): a * b
        aw, ax, ay, az = cw, cx, cy, cz
        bw, bx, by, bz = rest_conj
        rel_w = aw * bw - ax * bx - ay * by - az * bz
        rel_x = aw * bx + ax * bw + ay * bz - az * by
        rel_y = aw * by - ax * bz + ay * bw + az * bx
        rel_z = aw * bz + ax * by - ay * bx + az * bw
        return (float(rel_x), float(rel_y), float(rel_z), float(rel_w))

    # ── amplification helper ────────────────────────────────────────────
    def _amplify(self, raw01: float, scale: float, limit: float) -> float:
        """Map raw curl in [0, 1] to a joint angle in [0, limit].

        With ``use_tanh=False`` this is the legacy linear path:
        ``raw * scale * limit`` (clamps later via ``_clamp_array``).

        With ``use_tanh=True`` (9.12 default) this uses
        ``limit * tanh(raw * scale)``.  At small ``raw`` the tangent is
        ``scale * raw`` (same amplification as linear); as ``raw * scale``
        grows the output asymptotes to ``limit`` smoothly.  This avoids
        the hard-clip flat-line we observed at ``--finger_proximal_scale
        4.0`` where ``L_idx_proximal`` saturated within frame 20.
        """
        raw = max(0.0, min(1.0, float(raw01)))
        if self.use_tanh:
            return float(limit * math.tanh(raw * float(scale)))
        return float(raw * float(scale) * limit)

    # ── primary entry: VMC (UDCAP Path B) ────────────────────────────────
    def map_hand_vmc(
        self,
        bones: Dict[str, Tuple[float, float, float, float]],
        is_right: bool,
    ) -> np.ndarray:
        """Return an (11,) vector derived from VMC bone quaternions.

        Bone naming conventions in the wild differ for the thumb:

          * **Unity HumanBodyBones (UDCAP, VRM, MocapForAll …):**
            ``{Left,Right}Thumb{Proximal,Intermediate,Distal}``
            — three Proximal/Intermediate/Distal segments like every
            other finger.
          * **Anatomy (some VMC sources):**
            ``{Left,Right}Thumb{Metacarpal,Proximal,Distal}``

        We accept both.  Probe ``ThumbIntermediate``: if present, the
        broadcaster uses the Unity convention and we map
        ``Proximal=CMC (yaw)``, ``Intermediate=MCP (pitch)``,
        ``Distal=IP``.  Otherwise we fall back to the anatomy convention
        with ``Metacarpal=CMC``, ``Proximal=MCP``, ``Distal=IP``.

        Verified against UDCAP 0.1.8.2 broadcast — bone-name sniffer
        observed the Unity convention (LeftThumbProximal/Intermediate/
        Distal, no Metacarpal channel).  See memory.md §10.16.

        Each quaternion is ``(qx, qy, qz, qw)`` (Unity xyz-w order).
        Missing bones default to "no rotation" (0).
        """
        out = np.zeros(FOURIER_JOINT_DIM, dtype=np.float32)
        side = "Right" if is_right else "Left"

        # Four fingers (MCP + PIP) — both conventions agree here.
        finger_pairs: Sequence[Tuple[str, int, int]] = (
            ("Index", IDX_INDEX_PROX, IDX_INDEX_INT),
            ("Middle", IDX_MIDDLE_PROX, IDX_MIDDLE_INT),
            ("Ring", IDX_RING_PROX, IDX_RING_INT),
            ("Little", IDX_PINKY_PROX, IDX_PINKY_INT),
        )
        for finger, prox_idx, int_idx in finger_pairs:
            prox_name = f"{side}{finger}Proximal"
            inter_name = f"{side}{finger}Intermediate"
            prox = bones.get(prox_name)
            inter = bones.get(inter_name)
            if prox is not None:
                prox_rel = self._vmc_quat_relative_to_rest(prox_name, prox)
                out[prox_idx] = self._amplify(
                    _quat_to_bend(*prox_rel), self.proximal_scale, _PROXIMAL_MAX
                )
            if inter is not None:
                # Use VMC intermediate quat when present; else leave mimic to the
                # post-processing branch.
                inter_rel = self._vmc_quat_relative_to_rest(inter_name, inter)
                out[int_idx] = self._amplify(
                    _quat_to_bend(*inter_rel), self.proximal_scale, _PROXIMAL_MAX
                )

        # Thumb: probe convention by ThumbIntermediate presence.
        thumb_inter = bones.get(f"{side}ThumbIntermediate")
        thumb_inter_name = f"{side}ThumbIntermediate"
        if thumb_inter is not None:
            # Unity convention — Proximal is CMC, Intermediate is MCP.
            thumb_cmc = bones.get(f"{side}ThumbProximal")  # opposition (yaw)
            thumb_cmc_name = f"{side}ThumbProximal"
            thumb_mcp = thumb_inter                         # flex     (pitch)
            thumb_mcp_name = thumb_inter_name
        else:
            # Anatomy convention — Metacarpal is CMC, Proximal is MCP.
            thumb_cmc = bones.get(f"{side}ThumbMetacarpal")
            thumb_cmc_name = f"{side}ThumbMetacarpal"
            thumb_mcp = bones.get(f"{side}ThumbProximal")
            thumb_mcp_name = f"{side}ThumbProximal"
        thumb_dist = bones.get(f"{side}ThumbDistal")
        thumb_dist_name = f"{side}ThumbDistal"

        if thumb_cmc is not None:
            thumb_cmc_rel = self._vmc_quat_relative_to_rest(thumb_cmc_name, thumb_cmc)
            # 9.19 fix (C8): UDCAP encodes thumb opposition on the X axis
            # (qx Δ ≈ 0.38 dominant over qy/qz at 0.15-0.19) — see
            # research/34. §3.1 C8 + per-bone raw-quat diagnostic on
            # ust_hm_glove.validation/recorded.  The previous _quat_to_yaw call
            # extracted Z-axis Euler yaw, throwing away ~70% of the
            # thumb opposition signal.  _quat_to_pitch extracts X-axis
            # rotation and recovers the full magnitude.
            yaw = _quat_to_pitch(*thumb_cmc_rel)
            # 9.15 fix: previously the formula
            #   out = yaw*(hi-lo)/2 + (lo+hi)/2
            # mapped yaw=0 to the MIDPOINT of the URDF range (e.g. for
            # range [-0.5, +1.0] this gave +0.25 even at rest, then
            # PACK_22D_SIGNS[4]=-1 produced the persistent -0.25 we
            # observed in [GR1T2Retarget #20] L_thb_yaw_joint = -0.250
            # rad sticking across 460 frames.  The thumb opposition
            # joint should be 0 (neutral) when no relative motion.
            # Map yaw in [-1, 0] linearly into [range_lo, 0] and
            # yaw in [0, +1] linearly into [0, range_hi].
            if yaw >= 0.0:
                thumb_yaw_out = yaw * _THUMB_YAW_RANGE[1]
            else:
                # 9.20 fix (C10): GR1T2 USD allows thumb opposition only
                # in one direction (URDF [-1.74, 0]).  Negative yaw_norm
                # would map to a negative thumb_yaw_out which after
                # PACK_22D_SIGNS[4]=-1 becomes a positive packed value
                # exceeding the URDF max → PhysX clamps to 0 → spurious
                # tracking error spikes (observed up to 0.086 rad on R
                # thumb during ust_hm_glove.validation Layer-2 replay).  Truncate
                # extension-direction motion to 0; opposition direction
                # alone is what the robot can physically reproduce.
                thumb_yaw_out = 0.0
            out[IDX_THUMB_YAW] = _clamp(
                thumb_yaw_out, _THUMB_YAW_RANGE[0], _THUMB_YAW_RANGE[1],
            )
        if thumb_mcp is not None:
            thumb_mcp_rel = self._vmc_quat_relative_to_rest(thumb_mcp_name, thumb_mcp)
            out[IDX_THUMB_PITCH] = self._amplify(
                _quat_to_bend(*thumb_mcp_rel), self.thumb_scale, _THUMB_PITCH_MAX
            )
        if thumb_dist is not None:
            thumb_dist_rel = self._vmc_quat_relative_to_rest(thumb_dist_name, thumb_dist)
            out[IDX_THUMB_DIST] = self._amplify(
                _quat_to_bend(*thumb_dist_rel), self.thumb_scale, _THUMB_PITCH_MAX
            )

        self._fill_mimic_inplace(out)
        return _clamp_array(out)

    # ── fallback: SteamVR Skeletal 31-bone ──────────────────────────────
    def map_hand_skeletal(
        self,
        bones_model_frame: np.ndarray,
        is_right: bool,
        finger_curls: Optional[Iterable[float]] = None,
    ) -> np.ndarray:
        """Derive the 11-vector from the Skeletal 31-bone model-frame array.

        ``bones_model_frame`` rows are ``[px, py, pz, qw, qx, qy, qz]``
        (see ``vr_sampler.py``).  ``finger_curls`` is the ``(5,)`` tuple
        ``(thumb, index, middle, ring, pinky)`` reported by OpenVR
        ``SkeletalSummary``; it is blended (50/50) with proximal bend when
        present because the Skeletal quaternions can be jittery near
        fingertip.
        """
        arr = np.asarray(bones_model_frame, dtype=np.float64)
        out = np.zeros(FOURIER_JOINT_DIM, dtype=np.float32)

        # SteamVR proximal-bone indices: thumb0=2, index0=6, middle0=11,
        # ring0=16, pinky0=21 — the bones on which bend is most reliable.
        prox_indices = {
            "thumb": 2,
            "index": 6,
            "middle": 11,
            "ring": 16,
            "pinky": 21,
        }
        if arr.shape[0] < max(prox_indices.values()) + 1:
            return out

        def _row_quat_xyzw(row: np.ndarray) -> Tuple[float, float, float, float]:
            # Columns 3=qw, 4=qx, 5=qy, 6=qz → return (qx, qy, qz, qw).
            return float(row[4]), float(row[5]), float(row[6]), float(row[3])

        # Four fingers
        for fname, prox_idx, int_idx in (
            ("index", IDX_INDEX_PROX, IDX_INDEX_INT),
            ("middle", IDX_MIDDLE_PROX, IDX_MIDDLE_INT),
            ("ring", IDX_RING_PROX, IDX_RING_INT),
            ("pinky", IDX_PINKY_PROX, IDX_PINKY_INT),
        ):
            qx, qy, qz, qw = _row_quat_xyzw(arr[prox_indices[fname]])
            bend01 = _quat_to_bend(qx, qy, qz, qw)
            out[prox_idx] = self._amplify(bend01, self.proximal_scale, _PROXIMAL_MAX)
            # Intermediate bone (proximal + 1 or + 2) is often very noisy;
            # leave to mimic post-process.  (We still blend curls below.)

        # Thumb.  Use thumb0 quaternion for yaw (opposition) and reserve
        # finger_curls[0] for pitch (flex) — that channel is the most
        # reliable from SteamVR.
        qx, qy, qz, qw = _row_quat_xyzw(arr[prox_indices["thumb"]])
        # 9.19 fix (C8): see map_hand_vmc note above.  Skeletal source
        # likewise encodes thumb opposition primarily on X axis; use
        # _quat_to_pitch instead of _quat_to_yaw.
        yaw_norm = _quat_to_pitch(qx, qy, qz, qw)
        if yaw_norm >= 0.0:
            thumb_yaw_out = yaw_norm * _THUMB_YAW_RANGE[1]
        else:
            # 9.20 fix (C10): see map_hand_vmc note above.  Skeletal
            # source likewise truncates extension-direction yaw to 0
            # because GR1T2 USD only allows opposition direction.
            thumb_yaw_out = 0.0
        out[IDX_THUMB_YAW] = _clamp(
            thumb_yaw_out, _THUMB_YAW_RANGE[0], _THUMB_YAW_RANGE[1],
        )

        # Blend curls into driver channels (thumb pitch uses the explicit
        # curl because the quat-derived pitch suffers from atan2 wrap).
        if finger_curls is not None:
            curls = list(finger_curls)
            if len(curls) >= 5:
                # 5 values: thumb, index, middle, ring, pinky.
                thumb_pitch = float(max(0.0, min(1.0, curls[0])))
                out[IDX_THUMB_PITCH] = self._amplify(
                    thumb_pitch, self.thumb_scale, _THUMB_PITCH_MAX
                )

                # Blend 50/50 with quat-derived bend so a static user at rest
                # (curls=0) still moves proximals when they gesture.
                for prox_idx, curl_idx in (
                    (IDX_INDEX_PROX, 1),
                    (IDX_MIDDLE_PROX, 2),
                    (IDX_RING_PROX, 3),
                    (IDX_PINKY_PROX, 4),
                ):
                    curl = float(max(0.0, min(1.0, curls[curl_idx])))
                    curl_amped = self._amplify(curl, self.proximal_scale, _PROXIMAL_MAX)
                    out[prox_idx] = 0.5 * out[prox_idx] + 0.5 * curl_amped

        self._fill_mimic_inplace(out)
        return _clamp_array(out)

    # ── Per-finger curl (SteamVR action) — primary source for UDCAP ──────
    def map_from_finger_curls(
        self,
        curls_5: Sequence[float],
    ) -> np.ndarray:
        """Drive the Fourier hand from 5 per-finger curl values.

        Intended to consume the Valve-Index-profile per-finger curl inputs
        (``/user/hand/{l,r}/input/finger/{thumb,index,middle,ring,pinky}``)
        that UDCAP / LucidVR-family drivers emit even when they do not
        implement SteamVR Skeletal Input 2.0.  This is the preferred
        finger-source path for those gloves — more articulate than the
        trigger/grip binary-grip fallback, less brittle than skeletal
        (which is typically inactive for those drivers).

        Parameters
        ----------
        curls_5:
            Iterable of exactly 5 floats in ``[0, 1]`` in order
            ``(thumb, index, middle, ring, pinky)``.  Values outside the
            range are clamped.  Missing or None values are treated as 0.

        Returns
        -------
        (11,) float32 vector in :data:`FOURIER_JOINT_DIM` layout.  The
        four finger proximal drivers track their corresponding curl; the
        thumb proximal_pitch tracks thumb curl; thumb yaw engages
        (opposition) proportionally to thumb curl for pinch postures.
        Intermediate and distal joints follow the standard mimic rule.
        """
        vals = [
            float(max(0.0, min(1.0, float(c) if c is not None else 0.0)))
            for c in list(curls_5)[:5]
        ] + [0.0] * max(0, 5 - len(list(curls_5)))
        thumb, index, middle, ring, pinky = vals[:5]

        out = np.zeros(FOURIER_JOINT_DIM, dtype=np.float32)
        out[IDX_INDEX_PROX] = self._amplify(index, self.proximal_scale, _PROXIMAL_MAX)
        out[IDX_MIDDLE_PROX] = self._amplify(middle, self.proximal_scale, _PROXIMAL_MAX)
        out[IDX_RING_PROX] = self._amplify(ring, self.proximal_scale, _PROXIMAL_MAX)
        out[IDX_PINKY_PROX] = self._amplify(pinky, self.proximal_scale, _PROXIMAL_MAX)
        # Thumb pitch = thumb flexion along MCP.
        out[IDX_THUMB_PITCH] = self._amplify(thumb, self.thumb_scale, _THUMB_PITCH_MAX)
        # Thumb yaw (opposition) opens toward palm when the thumb curls —
        # lets the user form a pinch with the index when both are curled.
        out[IDX_THUMB_YAW] = _clamp(
            _THUMB_YAW_RANGE[0]
            + (_THUMB_YAW_RANGE[1] - _THUMB_YAW_RANGE[0]) * thumb,
            _THUMB_YAW_RANGE[0],
            _THUMB_YAW_RANGE[1],
        )
        self._fill_mimic_inplace(out)
        return _clamp_array(out)

    # ── controller-button grip fallback ──────────────────────────────────
    def map_from_controller_buttons(
        self,
        trigger: float,
        grip: float,
        thumb_touch: bool = False,
    ) -> np.ndarray:
        """Last-resort finger map built from controller trigger/grip axes.

        When the UDCAP SteamVR driver does not emit Skeletal Input 2.0
        data (see the OpenVR skeletal probe in ``GR1T2FourierUDCAPDevice``)
        the user's finger positions are still partially observable via
        the Valve-Index-compatible trigger and grip analog channels
        that UDCAP *does* emit — those two values drive a conservative
        binary-grip gesture so at least pick-and-place teleoperation
        remains possible.

        Semantics:
          * ``trigger`` (0..1) — index + middle finger curl (precision
            pinch).  Values < 0.1 keep the index open; > 0.5 yields a
            full curl.
          * ``grip``    (0..1) — ring + pinky curl (power grip).  Also
            feeds back a mild boost to index/middle so the user can
            "close the whole fist" by squeezing hard.
          * ``thumb_touch`` — bool.  When ``True`` the thumb flexes and
            opposes toward the index tip, producing a pinch posture.

        Returns an (11,) vector in :data:`FOURIER_JOINT_DIM` layout.
        """
        t = float(max(0.0, min(1.0, trigger)))
        g = float(max(0.0, min(1.0, grip)))
        # Precision fingers (index, middle) — dominated by trigger, grip
        # adds a small amount.
        precision = self._amplify(
            min(1.0, 0.8 * t + 0.3 * g), self.proximal_scale, _PROXIMAL_MAX
        )
        # Power fingers (ring, pinky) — dominated by grip.
        power = self._amplify(g, self.proximal_scale, _PROXIMAL_MAX)

        out = np.zeros(FOURIER_JOINT_DIM, dtype=np.float32)
        out[IDX_INDEX_PROX] = precision
        out[IDX_MIDDLE_PROX] = precision
        out[IDX_RING_PROX] = power
        out[IDX_PINKY_PROX] = power

        if thumb_touch or t > 0.4 or g > 0.4:
            # Oppose the thumb toward the palm for pinch/grip postures.
            out[IDX_THUMB_YAW] = 0.6   # yaw toward palm
            out[IDX_THUMB_PITCH] = self._amplify(
                min(1.0, max(t, g) * 1.1), self.thumb_scale, _THUMB_PITCH_MAX
            )
        # mimic intermediates + distal filled by the standard rule
        self._fill_mimic_inplace(out)
        return _clamp_array(out)

    # ── internal ─────────────────────────────────────────────────────────
    def _fill_mimic_inplace(self, out: np.ndarray) -> None:
        """Fill intermediate / distal slots when they are still zero."""
        for prox, inter in (
            (IDX_INDEX_PROX, IDX_INDEX_INT),
            (IDX_MIDDLE_PROX, IDX_MIDDLE_INT),
            (IDX_RING_PROX, IDX_RING_INT),
            (IDX_PINKY_PROX, IDX_PINKY_INT),
        ):
            if abs(float(out[inter])) < 1e-6:
                out[inter] = out[prox] * self.intermediate_mimic
        if abs(float(out[IDX_THUMB_DIST])) < 1e-6:
            out[IDX_THUMB_DIST] = out[IDX_THUMB_PITCH] * self.distal_mimic


# ── module-level helpers ───────────────────────────────────────────────


def _clamp(v: float, lo: float, hi: float) -> float:
    return float(max(lo, min(hi, v)))


def _clamp_array(out: np.ndarray) -> np.ndarray:
    """Clamp the 11-vec to URDF-safe ranges."""
    # drivers
    for idx in (IDX_INDEX_PROX, IDX_MIDDLE_PROX, IDX_PINKY_PROX, IDX_RING_PROX):
        out[idx] = _clamp(float(out[idx]), 0.0, _PROXIMAL_MAX)
    # mimic intermediates follow drivers; still clamp to ±max
    for idx in (IDX_INDEX_INT, IDX_MIDDLE_INT, IDX_PINKY_INT, IDX_RING_INT):
        out[idx] = _clamp(float(out[idx]), 0.0, _PROXIMAL_MAX)
    out[IDX_THUMB_YAW] = _clamp(float(out[IDX_THUMB_YAW]), *_THUMB_YAW_RANGE)
    out[IDX_THUMB_PITCH] = _clamp(float(out[IDX_THUMB_PITCH]), 0.0, _THUMB_PITCH_MAX)
    out[IDX_THUMB_DIST] = _clamp(float(out[IDX_THUMB_DIST]), 0.0, _THUMB_PITCH_MAX)
    return np.nan_to_num(out, nan=0.0, posinf=_PROXIMAL_MAX, neginf=0.0)


# ── global 22-joint packer (Isaac Lab hand_joint_names order) ──────────


#: Per-slot sign for the 22-D action vector (2026-04-26 9.9 fix).
#:
#: GR1T2's USD has the per-finger flexion in the NEGATIVE direction for
#: every finger except the thumb pitch (and the thumb distal, which
#: follows the thumb pitch direction).  Joint limit ranges as observed
#: from ``robot.data.joint_pos_limits`` at runtime::
#:
#:     proximal index/middle/ring/pinky : [-1.570, +0.000]   -> NEGATE
#:     thumb_proximal_yaw (opposition)   : [-1.740, +0.000]  -> NEGATE
#:     thumb_proximal_pitch (flexion)    : [+0.000, +1.220]  -> KEEP
#:     intermediate (mimic of proximal)  : assumed negative  -> NEGATE
#:     thumb_distal (mimic of pitch)     : assumed positive  -> KEEP
#:
#: ``FourierHandMapper`` outputs positive curl magnitudes (0..1.57 rad);
#: PhysX clamps any out-of-range target to the closest limit, so an un-
#: signed +0.5 sent to a [-1.57, 0] joint becomes 0 -- the joint never
#: moves.  This sign vector applies the per-slot flip when packing into
#: the canonical action layout.  Verified by ``[FingerCmp]`` showing
#: ``act_tgt=+0.214 jpt=+0.214 pos=-0.000`` (target reaches articulation
#: but joint stays at 0).  See memory.md sec 10.18.
PACK_22D_SIGNS: Tuple[float, ...] = (
    # [ 0:10] proximal drivers (index/middle/pinky/ring/thumb_yaw, L then R)
    -1.0, -1.0, -1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0,
    # [10:14] L intermediates (index/middle/pinky/ring) -- mimic proximal
    -1.0, -1.0, -1.0, -1.0,
    # [14] L_thumb_proximal_pitch -- positive direction
    +1.0,
    # [15:19] R intermediates -- mimic proximal
    -1.0, -1.0, -1.0, -1.0,
    # [19] R_thumb_proximal_pitch -- positive direction
    +1.0,
    # [20] L_thumb_distal, [21] R_thumb_distal -- follow thumb pitch (positive)
    +1.0, +1.0,
)


def pack_22d(left_11: np.ndarray, right_11: np.ndarray) -> np.ndarray:
    """Assemble two per-side vectors into the 22-dim Isaac Lab order.

    The order below matches ``pickplace_gr1t2_env_cfg.py:136-159``::

        [ 0] L_index_proximal
        [ 1] L_middle_proximal
        [ 2] L_pinky_proximal
        [ 3] L_ring_proximal
        [ 4] L_thumb_proximal_yaw
        [ 5] R_index_proximal
        [ 6] R_middle_proximal
        [ 7] R_pinky_proximal
        [ 8] R_ring_proximal
        [ 9] R_thumb_proximal_yaw
        [10] L_index_intermediate
        [11] L_middle_intermediate
        [12] L_pinky_intermediate
        [13] L_ring_intermediate
        [14] L_thumb_proximal_pitch
        [15] R_index_intermediate
        [16] R_middle_intermediate
        [17] R_pinky_intermediate
        [18] R_ring_intermediate
        [19] R_thumb_proximal_pitch
        [20] L_thumb_distal
        [21] R_thumb_distal

    ``left_11`` / ``right_11`` are expected in
    :data:`FOURIER_JOINT_DIM` layout.  Per-slot sign is applied via
    :data:`PACK_22D_SIGNS` so the output matches the GR1T2 USD's
    finger-flexion direction (negative for proximal/intermediate/
    thumb_yaw, positive for thumb_pitch / thumb_distal).
    """
    left = np.asarray(left_11, dtype=np.float32).reshape(-1)
    right = np.asarray(right_11, dtype=np.float32).reshape(-1)
    if left.size != FOURIER_JOINT_DIM or right.size != FOURIER_JOINT_DIM:
        raise ValueError(
            f"expected two length-{FOURIER_JOINT_DIM} vectors, "
            f"got {left.size} and {right.size}"
        )
    out = np.zeros(FOURIER_TOTAL_HAND_JOINTS, dtype=np.float32)
    # L/R proximals interleaved
    out[0] = left[IDX_INDEX_PROX]
    out[1] = left[IDX_MIDDLE_PROX]
    out[2] = left[IDX_PINKY_PROX]
    out[3] = left[IDX_RING_PROX]
    out[4] = left[IDX_THUMB_YAW]
    out[5] = right[IDX_INDEX_PROX]
    out[6] = right[IDX_MIDDLE_PROX]
    out[7] = right[IDX_PINKY_PROX]
    out[8] = right[IDX_RING_PROX]
    out[9] = right[IDX_THUMB_YAW]
    # L/R intermediates interleaved; slot 14 = L_thumb_proximal_pitch,
    # slot 19 = R_thumb_proximal_pitch.
    out[10] = left[IDX_INDEX_INT]
    out[11] = left[IDX_MIDDLE_INT]
    out[12] = left[IDX_PINKY_INT]
    out[13] = left[IDX_RING_INT]
    out[14] = left[IDX_THUMB_PITCH]
    out[15] = right[IDX_INDEX_INT]
    out[16] = right[IDX_MIDDLE_INT]
    out[17] = right[IDX_PINKY_INT]
    out[18] = right[IDX_RING_INT]
    out[19] = right[IDX_THUMB_PITCH]
    # thumb distals (L, R)
    out[20] = left[IDX_THUMB_DIST]
    out[21] = right[IDX_THUMB_DIST]
    # Apply per-slot sign so the unsigned curl magnitudes from the
    # mapper land in the joint's actual flexion direction.
    out *= np.asarray(PACK_22D_SIGNS, dtype=np.float32)
    return out
