"""ust_hm_glove teleop package — UDCAP VR-glove driven Pico HMD teleoperation.

Merged from the pre-9.36 layout where shared modules lived in
``ust_ws.ust_hm_glove.teleop`` and Fourier-GR1T2-specific code lived in
``ust_ws.ust_hm_glove.teleop``.  Post-9.36 both sets are co-located here
so the entire UDCAP-glove track is self-contained under one package.

Module overview
---------------
Hardware-generic (originally ust_hm_glove.teleop):
    vr_sampler                — pyopenvr 120 Hz sampler thread
    coord_transforms          — SteamVR ↔ Isaac Lab coord conversions
    fingertip_extractor       — SteamVR Skeletal 26-bone tip extraction
    vmc_receiver              — VMC OSC receiver (UDP 39539)
    udcap_finger_mapper       — UDCAP / VMC / Skeletal → Inspire 12-joint
    g1_retargeter             — G1 + Inspire 5-finger retargeter (legacy)
    pico_udcap_device         — G1 device (legacy)

Fourier GR1T2 (originally ust_hm_glove.teleop):
    gr1t2_retargeter          — GR1T2 + Fourier 6-DoF hand 36D retargeter
    fourier_hand_mapper       — UDCAP / VMC / Skeletal → Fourier 11-joint
    waist_estimator           — hips tracker → waist Euler
    gr1t2_udcap_device        — GR1T2 + UDCAP DeviceBase
    head_estimator            — HMD-driven head joint follow

OSQP / qpsolvers compat:
    _osqp_compat              — qpsolvers 4.x ↔ osqp 0.6 shim
"""

# ── Hardware-generic exports (was ust_hm_glove.teleop) ─────────────
from .coord_transforms import (
    svr_to_isaaclab,
    forearm_to_wrist,
    quat_wxyz_to_matrix,
    matrix_to_quat_wxyz,
    quat_multiply,
)
from .fingertip_extractor import (
    STEAMVR_TIP_BONE_INDICES,
    extract_fingertips_wrist_frame,
)
from .udcap_finger_mapper import (
    UDCAPFingerMapper,
    INSPIRE_JOINT_DIM,
)

# ── Fourier GR1T2 exports (was ust_hm_glove.teleop) ────────────
from .gr1t2_retargeter import (  # noqa: F401
    GR1T2FourierSteamVRRetargeter,
    GR1T2FourierRetargeterCfg,
    DEFAULT_LEFT_POS,
    DEFAULT_LEFT_QUAT,
    DEFAULT_RIGHT_POS,
    DEFAULT_RIGHT_QUAT,
    ACTION_DIM,
    HAND_DIM_PER_SIDE,
    HAND_DIM_TOTAL,
)
from .fourier_hand_mapper import (  # noqa: F401
    FourierHandMapper,
    FOURIER_JOINT_DIM,
    FOURIER_TOTAL_HAND_JOINTS,
    pack_22d,
)
from .waist_estimator import WaistEstimator, WaistEstimate  # noqa: F401
from .gr1t2_udcap_device import (  # noqa: F401
    GR1T2FourierUDCAPDevice,
    GR1T2FourierUDCAPDeviceCfg,
    GR1T2InterventionInterface,
)

# ── Lazy modules (heavy runtime deps) ──────────────────────────────────
# vr_sampler / vmc_receiver / pico_udcap_device / g1_retargeter pull in
# openvr / python-osc / Isaac Sim — leave them for explicit imports so
# offline smoke tests don't hit those deps.

__all__ = [
    # hardware-generic
    "svr_to_isaaclab",
    "forearm_to_wrist",
    "quat_wxyz_to_matrix",
    "matrix_to_quat_wxyz",
    "quat_multiply",
    "STEAMVR_TIP_BONE_INDICES",
    "extract_fingertips_wrist_frame",
    "UDCAPFingerMapper",
    "INSPIRE_JOINT_DIM",
    # legacy G1 (lazy-imported in scripts)
    "G1SteamVRRetargeter",
    "SteamVRSampler",
    "SteamVRSnapshot",
    "VMCHandReceiver",
    "PicoUDCAPDevice",
    "PicoUDCAPDeviceCfg",
    # Fourier GR1T2
    "GR1T2FourierSteamVRRetargeter",
    "GR1T2FourierRetargeterCfg",
    "DEFAULT_LEFT_POS",
    "DEFAULT_LEFT_QUAT",
    "DEFAULT_RIGHT_POS",
    "DEFAULT_RIGHT_QUAT",
    "ACTION_DIM",
    "HAND_DIM_PER_SIDE",
    "HAND_DIM_TOTAL",
    "FourierHandMapper",
    "FOURIER_JOINT_DIM",
    "FOURIER_TOTAL_HAND_JOINTS",
    "pack_22d",
    "WaistEstimator",
    "WaistEstimate",
    "GR1T2FourierUDCAPDevice",
    "GR1T2FourierUDCAPDeviceCfg",
    "GR1T2InterventionInterface",
]
