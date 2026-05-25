"""``GR1T2FourierUDCAPDevice`` — Windows/SteamVR teleop device for Isaac Lab.

Drop-in replacement for the G1 ``PicoUDCAPDevice`` when the robot is
Fourier GR1T2 + 6-DoF Fourier hand.  Produces the 36D Pink IK action
tensor expected by ``KitchenSortingGR1T2EnvCfg``.

The device re-uses the hardware-generic modules from
``ust_ws.ust_hm_glove.teleop`` (``SteamVRSampler``, ``VMCHandReceiver``)
so the Pico 4 Ultra + Virtual Desktop + UDCAP VR Glove stack remains
untouched.  Only the retargeter differs.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple

import numpy as np
import torch


# ── Optional Isaac Lab imports ────────────────────────────────────────
try:
    from isaaclab.devices.device_base import DeviceBase, DeviceCfg  # type: ignore

    _ISAACLAB_AVAILABLE = True
except Exception:  # noqa: BLE001 — missing carb, standalone unit-test mode
    _ISAACLAB_AVAILABLE = False

    class DeviceBase:  # type: ignore[override]
        """Minimal stand-in so this module imports without Isaac Sim."""

        def __init__(self, retargeters: Optional[Iterable[Any]] = None) -> None:
            self._retargeters = list(retargeters or [])

        def __str__(self) -> str:
            return self.__class__.__name__

        def reset(self) -> None:  # pragma: no cover — runtime override
            raise NotImplementedError

        def add_callback(self, key: Any, func: Any) -> None:  # pragma: no cover
            raise NotImplementedError

        def _get_raw_data(self) -> Any:
            raise NotImplementedError

        def advance(self) -> Any:
            raise NotImplementedError

    @dataclass
    class DeviceCfg:  # type: ignore[override]
        sim_device: str = "cpu"
        retargeters: list = field(default_factory=list)


from ust_ws.ust_hm_glove.teleop.vr_sampler import SteamVRSampler
# NOTE: the G1 ``PICOInterventionInterface`` is purely button debouncing,
# so we subclass it verbatim at the end of this module.
from ust_ws.ust_hm_glove.teleop.pico_udcap_device import PICOInterventionInterface

from .gr1t2_retargeter import (
    ACTION_DIM,
    GR1T2FourierRetargeterCfg,
    GR1T2FourierSteamVRRetargeter,
)
from .head_estimator import HeadEstimate, HeadEstimator
from .waist_estimator import WaistEstimate, WaistEstimator


__all__ = [
    "GR1T2FourierUDCAPDevice",
    "GR1T2FourierUDCAPDeviceCfg",
    "GR1T2InterventionInterface",
]


@dataclass
class GR1T2FourierUDCAPDeviceCfg(DeviceCfg):
    """Configuration for :class:`GR1T2FourierUDCAPDevice`.

    All paths may be absolute or repo-root relative (``./ust_ws/...``);
    the device normalises them on construction.
    """

    # Hardware / VR config
    tracker_binding_json: str = "./ust_ws/ust_hm_glove/config/tracker_binding.json"
    actions_json: str = "./ust_ws/ust_hm_glove/config/openvr_actions/actions.json"
    # SteamVR application manifest — required for default_bindings to be
    # auto-applied by SteamVR (memory.md §10.10).  Must declare the same
    # ``app_key`` as actions.json (here ``ust.teleop.fourier_gr1t2``).
    vrmanifest_json: str = (
        "./ust_ws/ust_hm_glove/config/openvr_actions/manifest.vrmanifest"
    )
    app_key: str = "ust.teleop.fourier_gr1t2"

    # dex-retargeting YAMLs (optional; fall back to FourierHandMapper when absent)
    left_hand_retarget_yaml: Optional[str] = (
        "./ust_ws/ust_hm_glove/config/dex_retargeting/fourier_left_dexpilot.yml"
    )
    right_hand_retarget_yaml: Optional[str] = (
        "./ust_ws/ust_hm_glove/config/dex_retargeting/fourier_right_dexpilot.yml"
    )
    dex_urdf_dir: Optional[str] = None

    forearm_wrist_offset: Tuple[float, float, float] = (0.12, 0.0, 0.0)
    position_scale: float = 1.0
    rotation_scale: float = 1.0
    gripper_threshold: float = 0.5
    body_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    controller_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Idle pose (GR1T2 base_link frame).  Keep ``None`` to use the defaults
    # declared in :mod:`gr1t2_retargeter`.  Override after running
    # ``scripts/calibrate_gr1t2_idle_pose.py`` for your USD.
    idle_left_pos: Optional[Tuple[float, float, float]] = None
    idle_left_quat: Optional[Tuple[float, float, float, float]] = None
    idle_right_pos: Optional[Tuple[float, float, float]] = None
    idle_right_quat: Optional[Tuple[float, float, float, float]] = None

    use_waist_origin: bool = True
    subtract_waist_z: bool = False
    freeze_orientation: bool = False
    right_wrist_z180: bool = True

    # Waist DoF (Pink IK null-space posture target for WaistEnabled envs).
    enable_waist_dof: bool = False
    waist_source: str = "hips_tracker"
    waist_gain: Tuple[float, float, float] = (1.0, 0.5, 0.3)
    waist_low_pass_alpha: float = 0.3

    # Sampler
    sampler_rate_hz: float = 120.0
    # Watchdog timeout for openvr.init() inside SteamVRSampler.start().
    # When SteamVR / PICO Connect / VD is not running the call would
    # otherwise block the entire teleop entry-point indefinitely (silent
    # hang right after "generated runtime manifest" log line).  9.38.
    sampler_init_timeout_sec: float = 30.0
    # Skeleton 2.0 (SteamVR Skeletal Input).  Default ON in 9.37 — this is
    # now the PRIMARY finger source per user request.  When the UDCAP
    # SteamVR driver implements ``CreateSkeletonComponent`` (Skeletal Input
    # 2.0) it forwards a 31-bone hand skeleton at ~120 Hz which we read via
    # ``getSkeletalBoneData`` in ``vr_sampler._read_hand``.  The retargeter
    # then prefers either DexPilot (when the Fourier hand URDF is loaded)
    # or ``map_hand_skeletal`` over any other finger source.  See
    # ``_probe_openvr_skeletal`` for runtime diagnostics that report
    # ``bActive`` / tracking level / bone count for each side.
    enable_skeletal: bool = True

    # Path B (VMC) fallback — DISABLED BY DEFAULT in 9.37.  Pre-9.37 default
    # was 39539 because UDCAP historically returned ``bActive=False`` for
    # Skeletal Input 2.0 even when it advertised support, forcing the user
    # onto VMC OSC bone broadcasts.  Per user request the system is now
    # migrated to Skeleton 2.0 as the primary path; VMC stays available as
    # an explicit fallback when Skeletal Input cannot be enabled in UDCAP
    # (open the UDCAP tray UI → Settings → Output → enable "VMC" and "OSC"
    # then pass ``--path_b_port 39539`` to re-arm this fallback).  The
    # retargeter priority chain (gr1t2_retargeter._resolve_hand_joints) is
    # unchanged — Skeletal beats VMC whenever both are present.
    path_b_port: int = 0

    # Hand mapper tuning (2026-04-26 9.10/9.11/9.12 fix)
    # --------------------------------------------------
    # Default scale changed in 9.12 from 4.0 -> 2.5 because video review
    # of the 4.0 / 6.0 runs showed L_idx_proximal hitting -1.570 (the
    # joint's limit, full fist) within frame 20 and STAYING there for
    # the remainder of the session.  A user mid-fist already saturated
    # the joint -> upper-half resolution lost.  2.5 keeps ~80% of full
    # fist within the joint range, leaving the user dynamic range
    # mostly intact while still amplifying the under-reported UDCAP
    # quat magnitudes.
    #
    # Companion: `non_linear_curl=True` applies a tanh-shaped curve so
    # small motions still amplify generously but the tail asymptotes
    # to the joint limit smoothly (avoids the abrupt clip we saw at
    # scale=4 / 6).
    #
    # CLI tuning still works:
    #   --finger_proximal_scale 1.5  (less sensitive, no saturation)
    #   --finger_proximal_scale 4.0  (legacy 9.11 default)
    #   --finger_proximal_scale 6.0  (max amplification, often saturates)
    hand_proximal_scale: float = 2.5
    hand_thumb_scale: float = 2.5
    # 9.12 fix: when True, applies ``out = limit * tanh(scale*raw / limit)``
    # so the output approaches but never exceeds the joint limit smoothly.
    # Recommended ON for UDCAP because the glove sensor is asymmetric
    # (left vs right) and a hard linear scale guarantees one hand
    # over-saturates while the other is still in mid-range.
    hand_use_tanh_amplification: bool = True
    # 9.14 fix: per-bone REST POSE calibration for VMC source.  UDCAP
    # broadcasts non-identity rest quats for pinky/ring/thumb (~16°
    # offset observed even at user open hand), creating a static curl
    # baseline.  When True the mapper averages the first
    # ``hand_vmc_rest_frames`` frames to capture the rest pose, then
    # outputs only the relative rotation.  User starts at zero curl and
    # full motion range is preserved.  See memory.md §10.23.
    hand_vmc_subtract_rest: bool = True
    # 9.18: lowered 30 → 10 (~0.5 s) so a brief still moment captures
    # the rest pose; longer windows let the user's small fidgeting
    # contaminate the average and absorb subsequent motion.
    hand_vmc_rest_frames: int = 10

    # 9.14/9.16 fix: prefer the controller pose (Touch/knuckles) over the
    # ``*_forearm`` tracker for the wrist EEF target.  Default False:
    # use the wrist-mounted physical Vive tracker when available
    # (matches the user's actual rig — they wear trackers on the
    # forearms / wrists, not Touch controllers in the hands).  Set
    # True only when there is no wrist tracker AND the controllers
    # are held in hand AND UDCAP Space Plan is properly configured
    # (see UDCAP CONFIGURATION CHECK warning).
    prefer_controller_for_eef: bool = False
    controller_to_wrist_offset: Tuple[float, float, float] = (0.0, 0.0, -0.05)

    # 9.14 fix: averaged zero-cal window for WaistEstimator.  Single-
    # frame zero (9.13) was unreliable with VD AI hips tracker which
    # has noise on the first sample; averaging 30 frames (~1.5 s @
    # 20 Hz) gives a stable rest pose.
    waist_zero_cal_frames: int = 30
    # 9.15 fix: per-axis deadband for WaistEstimator.  Default pitch
    # deadband 0.3 rad (17°) absorbs Virtual Desktop AI body tracker
    # noise (observed waist_pitch raw range up to 111° even when user
    # is standing perfectly still).  yaw/roll deadband default 0 to
    # preserve normal turn / lean responsiveness.
    waist_deadband_rad: Tuple[float, float, float] = (0.0, 0.3, 0.0)

    # 9.18 fix: HeadEstimator drives the robot's head_yaw/pitch/roll
    # joints from the user's HMD orientation.  Replaces the 9.16
    # "HMD-follow viewport camera" feature (which tracked the camera
    # to the HMD; the user wanted the robot's head to track instead).
    enable_head_follow_hmd: bool = True
    head_gain: Tuple[float, float, float] = (1.0, 0.7, 0.5)
    head_low_pass_alpha: float = 0.4
    head_zero_cal_frames: int = 15
    head_deadband_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Controller button fallback (for when UDCAP does not emit Skeletal
    # Input 2.0 — degrades finger control to a binary pinch/grip gesture).
    enable_button_grip_fallback: bool = True
    button_grip_pinch_threshold: float = 0.3

    # 9.26 fix: when True, the retargeter forces the arm EEF targets to
    # the idle pose (T-pose) regardless of forearm tracker / controller
    # state.  Used by `--ignore_trackers true` so the user's no-tracker
    # rig (PICO HMD + UDCAP + 2 controllers) keeps the arms still while
    # only fingers track.  See memory.md §10.34.
    disable_arm_tracking: bool = False

    # 9.23 fix: single-pole low-pass alpha applied to the 22D finger
    # output BEFORE it goes into the action tensor.  UDCAP broadcasts at
    # ~140 Hz and SteamVR samples at 120 Hz, but env.step runs at only
    # 20 Hz (decimation 6 over physics 120 Hz), so without smoothing the
    # 1-of-7 picked source frame produces visible 0↔limit jumps when the
    # user closes/opens their hand naturally.  ``alpha = 0.4`` is the
    # missing analog of ``waist_low_pass_alpha`` / ``head_low_pass_alpha``
    # from the existing Waist/Head estimators.  Set to 1.0 to disable
    # (legacy behaviour) or to 0.2 for very strong damping.  See
    # memory.md §10.31.
    finger_low_pass_alpha: float = 0.4

    debug: bool = True


class GR1T2FourierUDCAPDevice(DeviceBase):
    """SteamVR + UDCAP + Fourier retargeter → 36D action, ``DeviceBase`` API."""

    ACTION_DIM = ACTION_DIM

    def __init__(self, cfg: GR1T2FourierUDCAPDeviceCfg):
        super().__init__()
        self.cfg = cfg
        self._sampler: Optional[SteamVRSampler] = None
        self._vmc = None
        self._started = False
        self._advance_count = 0
        self._first_advance_logged = False
        # Action-handle cache — populated in ``start()`` after sampler has
        # loaded the manifest.  Read each frame via ``getAnalogActionData``.
        # Action-based reads are required because the sampler's legacy
        # ``getControllerState()`` path returns zeros for most knuckles
        # emulators (UDCAP / LucidVR / Phantom-style drivers).
        self._action_handles: Dict[str, Any] = {}
        # Runtime watchdog: if every action value (trigger / grip / 10 finger
        # curls) stays at 0 for this many advance() calls in a row AFTER the
        # first frame, print a one-shot warning explaining the most likely
        # cause (UdcapDriver.exe not forwarding glove data to SteamVR).  The
        # threshold corresponds to ~10 s at 20 Hz teleop rate.
        self._zero_streak_advances = 0
        self._zero_streak_warned = False
        # 9.15: frozen-controller-Z watchdog (UDCAP Space Plan misconfig).
        self._ctrl_z_track: Dict[str, Tuple[float, float]] = {
            "left": (float("inf"), float("-inf")),
            "right": (float("inf"), float("-inf")),
        }
        self._frozen_z_warned = False

        self._tracker_binding = self._load_tracker_binding(
            self._absolutise(cfg.tracker_binding_json)
        )

        retargeter_cfg = GR1T2FourierRetargeterCfg(
            position_scale=cfg.position_scale,
            rotation_scale=cfg.rotation_scale,
            gripper_threshold=cfg.gripper_threshold,
            body_pos_offset=cfg.body_pos_offset,
            controller_pos_offset=cfg.controller_pos_offset,
            forearm_wrist_offset=cfg.forearm_wrist_offset,
            left_hand_retarget_yaml=self._absolutise_optional(cfg.left_hand_retarget_yaml),
            right_hand_retarget_yaml=self._absolutise_optional(cfg.right_hand_retarget_yaml),
            dex_urdf_dir=self._absolutise_optional(cfg.dex_urdf_dir),
            use_waist_origin=cfg.use_waist_origin,
            subtract_waist_z=cfg.subtract_waist_z,
            freeze_orientation=cfg.freeze_orientation,
            right_wrist_z180=cfg.right_wrist_z180,
            hand_proximal_scale=cfg.hand_proximal_scale,
            hand_thumb_scale=cfg.hand_thumb_scale,
            hand_use_tanh_amplification=cfg.hand_use_tanh_amplification,
            hand_vmc_subtract_rest=cfg.hand_vmc_subtract_rest,
            hand_vmc_rest_frames=cfg.hand_vmc_rest_frames,
            prefer_controller_for_eef=cfg.prefer_controller_for_eef,
            controller_to_wrist_offset=cfg.controller_to_wrist_offset,
            enable_button_grip_fallback=cfg.enable_button_grip_fallback,
            button_grip_pinch_threshold=cfg.button_grip_pinch_threshold,
            finger_low_pass_alpha=cfg.finger_low_pass_alpha,
            disable_arm_tracking=cfg.disable_arm_tracking,
            debug=cfg.debug,
        )
        if cfg.idle_left_pos is not None:
            retargeter_cfg.idle_left_pos = cfg.idle_left_pos
        if cfg.idle_left_quat is not None:
            retargeter_cfg.idle_left_quat = cfg.idle_left_quat
        if cfg.idle_right_pos is not None:
            retargeter_cfg.idle_right_pos = cfg.idle_right_pos
        if cfg.idle_right_quat is not None:
            retargeter_cfg.idle_right_quat = cfg.idle_right_quat
        self._retargeter = GR1T2FourierSteamVRRetargeter(retargeter_cfg)

        self._waist: Optional[WaistEstimator]
        if cfg.enable_waist_dof:
            self._waist = WaistEstimator(
                source=cfg.waist_source,
                gain=cfg.waist_gain,
                low_pass_alpha=cfg.waist_low_pass_alpha,
                zero_cal_frames=cfg.waist_zero_cal_frames,
                deadband_rad=cfg.waist_deadband_rad,
            )
        else:
            self._waist = None

        # 9.18 — HeadEstimator: drives the robot's head joints from HMD pose.
        if cfg.enable_head_follow_hmd:
            self._head = HeadEstimator(
                gain=cfg.head_gain,
                low_pass_alpha=cfg.head_low_pass_alpha,
                zero_cal_frames=cfg.head_zero_cal_frames,
                deadband_rad=cfg.head_deadband_rad,
            )
        else:
            self._head = None

    # ── path helpers ───────────────────────────────────────────────────
    @staticmethod
    def _absolutise(path: str) -> str:
        p = Path(path)
        if not p.is_absolute():
            p = Path.cwd() / p
        return str(p.resolve())

    @classmethod
    def _absolutise_optional(cls, path: Optional[str]) -> Optional[str]:
        if path is None:
            return None
        return cls._absolutise(path)

    @staticmethod
    def _load_tracker_binding(path: str) -> Dict[str, Dict[str, str]]:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        trackers = data.get("trackers", data)
        return {
            str(sn): {str(k): str(v) for k, v in info.items()}
            for sn, info in trackers.items()
        }

    @staticmethod
    def _generate_runtime_manifest(static_manifest_path: str) -> str:
        """Materialise a runtime ``.vrmanifest`` next to the static template
        with ``binary_path_windows`` pointed at the *current* Python
        interpreter (``sys.executable``).

        SteamVR validates ``binary_path_windows`` as a path RELATIVE to the
        manifest file's directory.  When the static template ships
        ``"python.exe"`` SteamVR resolves it to
        ``<config>/openvr_actions/python.exe``, which doesn't exist.  Then
        ``vrserver.txt`` logs ``App ust.teleop.fourier_gr1t2 binary_path
        ... is invalid. Skipping`` and the entire application entry is
        silently dropped — leaving ``identifyApplication`` to raise
        ``ApplicationError_UnknownApplication`` (memory.md §10.12).

        Writing an absolute path that *exists* (the conda-env python.exe)
        makes SteamVR accept the entry.  We regenerate this file every
        launch so ``sys.executable`` stays in sync with the active env.
        """
        import sys as _sys  # local import keeps ``sys`` out of module namespace

        with open(static_manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)

        py_exe = os.path.abspath(_sys.executable)
        for app in manifest.get("applications", []):
            app["binary_path_windows"] = py_exe

        runtime_path = os.path.join(
            os.path.dirname(static_manifest_path),
            "manifest.runtime.vrmanifest",
        )
        with open(runtime_path, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)
        return runtime_path

    # ── lifecycle ──────────────────────────────────────────────────────
    def start(self) -> None:
        if self._started:
            return

        # Materialise a runtime manifest with sys.executable as the binary
        # path; SteamVR drops application entries whose binary_path_windows
        # cannot be resolved relative to the manifest directory.  See the
        # docstring on ``_generate_runtime_manifest`` for the rationale.
        static_manifest = self._absolutise(self.cfg.vrmanifest_json)
        try:
            runtime_manifest = self._generate_runtime_manifest(static_manifest)
            if self.cfg.debug:
                print(
                    f"[GR1T2FourierUDCAPDevice] generated runtime manifest "
                    f"{runtime_manifest!r} with binary_path_windows pointing "
                    "at the active interpreter."
                )
        except Exception as exc:  # noqa: BLE001 — fall back to static
            print(
                f"[GR1T2FourierUDCAPDevice] runtime manifest generation "
                f"failed ({type(exc).__name__}: {exc}); falling back to "
                f"static manifest at {static_manifest!r}."
            )
            runtime_manifest = static_manifest

        self._sampler = SteamVRSampler(
            tracker_binding=self._tracker_binding,
            actions_json_path=self._absolutise(self.cfg.actions_json),
            rate_hz=self.cfg.sampler_rate_hz,
            enable_skeletal=self.cfg.enable_skeletal,
            vrmanifest_path=runtime_manifest,
            stale_vrmanifest_paths=(static_manifest,)
            if runtime_manifest != static_manifest else (),
            app_key=self.cfg.app_key,
            init_timeout_sec=self.cfg.sampler_init_timeout_sec,
        )
        self._sampler.start()

        if self.cfg.path_b_port and self.cfg.path_b_port > 0:
            try:
                from ust_ws.ust_hm_glove.teleop.vmc_receiver import VMCHandReceiver

                self._vmc = VMCHandReceiver(port=int(self.cfg.path_b_port))
                self._vmc.start()
            except Exception as exc:  # noqa: BLE001
                print(f"[GR1T2FourierUDCAPDevice] VMC fallback disabled ({exc}).")
                self._vmc = None

        if self._waist is not None:
            self._waist.reset()

        # Set up action handles for trigger/grip/finger-curl — must come
        # AFTER sampler.start() because sampler's ``setActionManifestPath``
        # is what registers the manifest with SteamVR.  Getting handles
        # for actions that didn't exist in the old manifest used to be a
        # silent no-op; now that actions.json ships the 4 trigger/grip +
        # 10 finger-curl actions, the handles should resolve.
        self._setup_action_handles()

        self._started = True
        if self.cfg.debug:
            print(
                f"[GR1T2FourierUDCAPDevice] started — actions='{self.cfg.actions_json}' "
                f"binding={len(self._tracker_binding)} trackers "
                f"skeletal={self.cfg.enable_skeletal} path_b={bool(self._vmc)} "
                f"waist_dof={bool(self._waist)} "
                f"action_handles={len(self._action_handles)} "
                f"app_key={self.cfg.app_key!r} "
                f"vrmanifest='{self.cfg.vrmanifest_json}'"
            )
            # Give the sampler a few frames to populate before probing —
            # otherwise action handles may not yet be bound.
            import time as _t
            deadline = _t.perf_counter() + 2.0
            while (self._sampler.frame_count < 5
                   and _t.perf_counter() < deadline):
                _t.sleep(0.05)
            self._probe_openvr_inventory()
            if self.cfg.enable_skeletal:
                self._probe_openvr_skeletal()
            self._probe_action_values()
            # The above probes describe what SteamVR returns to us.  Equally
            # important is whether glove sensor data even reaches SteamVR in
            # the first place — that depends on the user-space UdcapDriver
            # process forwarding frames over a named pipe.  We can detect the
            # process state cheaply via tasklist; a missing process is a
            # complete-pipeline failure no binding work can recover from.
            self._probe_udcap_processes()

    # ── Action-based input handles ─────────────────────────────────────
    _FINGER_NAMES: Tuple[str, ...] = ("thumb", "index", "middle", "ring", "pinky")

    def _setup_action_handles(self) -> None:
        """Cache OpenVR action handles for trigger/grip/finger curls.

        Safe no-op if the sampler's ``_vr_input`` is not available or a
        specific handle cannot be resolved (most likely cause: the action
        isn't declared in actions.json).
        """
        sampler = self._sampler
        if sampler is None or getattr(sampler, "_vr_input", None) is None:
            return
        vi = sampler._vr_input
        wanted = [
            "/actions/teleop/in/trigger_left",
            "/actions/teleop/in/trigger_right",
            "/actions/teleop/in/grip_left",
            "/actions/teleop/in/grip_right",
        ]
        for side in ("left", "right"):
            for name in self._FINGER_NAMES:
                wanted.append(f"/actions/teleop/in/finger_{name}_{side}")
        for path in wanted:
            try:
                self._action_handles[path] = vi.getActionHandle(path)
            except Exception as exc:  # noqa: BLE001
                if self.cfg.debug:
                    print(
                        f"[GR1T2FourierUDCAPDevice] getActionHandle({path!r}) failed: {exc}"
                    )

    def _read_analog_action(self, path: str) -> float:
        """Read a vector1 action value or return 0.0 on any failure."""
        handle = self._action_handles.get(path)
        if handle is None or self._sampler is None:
            return 0.0
        vi = getattr(self._sampler, "_vr_input", None)
        if vi is None:
            return 0.0
        try:
            data = vi.getAnalogActionData(handle)
        except Exception:
            return 0.0
        # VRAnalogActionData_t.x is the scalar value for vector1.
        try:
            return float(getattr(data, "x", 0.0))
        except Exception:
            return 0.0

    def _read_action_inputs(self) -> Dict[str, Any]:
        """Return ``{"left": {...}, "right": {...}}`` with keys
        ``trigger``, ``grip``, ``finger_curls`` (list of 5 floats in
        thumb/index/middle/ring/pinky order).

        Any individual missing action resolves to 0.0.  Suitable for
        passing straight to the retargeter which treats an all-zero
        curl tuple as "no input" and skips the finger-curl source.
        """
        out: Dict[str, Any] = {}
        for side in ("left", "right"):
            trigger = self._read_analog_action(f"/actions/teleop/in/trigger_{side}")
            grip = self._read_analog_action(f"/actions/teleop/in/grip_{side}")
            curls = [
                self._read_analog_action(f"/actions/teleop/in/finger_{n}_{side}")
                for n in self._FINGER_NAMES
            ]
            out[side] = {"trigger": trigger, "grip": grip, "finger_curls": curls}
        return out

    def _probe_udcap_processes(self) -> None:
        """Print whether ``UdcapDriver.exe`` and ``UDCAP_overlay.exe`` are alive.

        ``UdcapDriver.exe`` is the user-space app that reads the gloves over
        USB / Bluetooth and forwards finger sensor frames to the SteamVR
        driver via a named pipe.  Without it running, the SteamVR driver
        creates virtual knuckles controllers (with pose taken from the
        underlying oculus_touch / pico stream) but receives ZERO finger /
        trigger / grip data — every action value reads 0 regardless of
        binding work.  See memory.md §10.14 for the full investigation.
        """
        import subprocess as _sp
        wanted = ("UdcapDriver.exe", "UDCAP_overlay.exe")
        running: Dict[str, Optional[int]] = {name: None for name in wanted}
        try:
            res = _sp.run(
                ["tasklist", "/FO", "CSV", "/NH"],
                capture_output=True, text=True, timeout=5, check=False,
            )
            for line in res.stdout.splitlines():
                low = line.lower()
                for name in wanted:
                    if name.lower() in low:
                        parts = [p.strip().strip('"') for p in line.split(",")]
                        if len(parts) >= 2 and parts[1].isdigit():
                            running[name] = int(parts[1])
                        break
        except Exception as exc:  # noqa: BLE001
            print(
                "[GR1T2FourierUDCAPDevice] --- UDCAP user-space process probe ---\n"
                f"  tasklist failed ({type(exc).__name__}: {exc}); skipping check."
            )
            return
        print("[GR1T2FourierUDCAPDevice] --- UDCAP user-space process probe ---")
        for name in wanted:
            pid = running[name]
            mark = "✓" if pid is not None else "✗"
            stat = f"PID {pid}" if pid is not None else "NOT RUNNING"
            print(f"  {mark} {name:25s} {stat}")
        if running["UdcapDriver.exe"] is None:
            print(
                "  → UdcapDriver.exe is NOT running.  This is the user-space app\n"
                "    that bridges the gloves to the SteamVR driver.  Without it\n"
                "    no finger/trigger/grip data flows, regardless of binding\n"
                "    correctness.  Launch it from the Start Menu (search\n"
                "    'UdcapDriver') or manually:\n"
                "      \"C:\\Program Files\\UdcapDriver\\UdcapDriver.exe\"\n"
                "    For a deeper inspection of the data path run:\n"
                "      python -m ust_ws.ust_hm_glove.scripts.diagnose_udcap_dataflow"
            )

    def _probe_action_values(self) -> None:
        """One-shot dump of the resolved action handles + current values."""
        snapshot = self._read_action_inputs()
        print("[GR1T2FourierUDCAPDevice] --- OpenVR action values probe ---")
        for side, vals in snapshot.items():
            curls = vals["finger_curls"]
            print(
                f"  {side}: trigger={vals['trigger']:.3f} grip={vals['grip']:.3f} "
                f"finger_curls=[thumb={curls[0]:.2f} index={curls[1]:.2f} "
                f"middle={curls[2]:.2f} ring={curls[3]:.2f} pinky={curls[4]:.2f}]"
            )
        any_nonzero = any(
            vals["trigger"] > 0.01 or vals["grip"] > 0.01 or max(vals["finger_curls"]) > 0.01
            for vals in snapshot.values()
        )
        if any_nonzero:
            print("  → action-based input is live — fingers WILL move when you squeeze.")
        else:
            print(
                "  → all action values 0.0.  Either the user is fully at rest OR the binding\n"
                "    didn't match.  Squeeze the trigger now; if values are still 0, check\n"
                "    SteamVR > Settings > Controllers > Manage Controller Bindings > UST Teleop\n"
                "    and confirm the Index-profile binding named 'UST Teleop — Index profile\n"
                "    + per-finger curl' is active, then relaunch."
            )

    # ── OpenVR diagnostic probes (one-shot after start()) ───────────────
    def _probe_openvr_inventory(self) -> None:
        """Dump every tracked device with its class, serial, and (for
        controllers) ``Prop_ControllerType_String`` + hand role.

        This is the only reliable way to tell whether UDCAP's glove is
        being reported as ``knuckles`` (matches our binding) vs some
        other profile that silently bypasses our ``bindings_index.json``
        routing for skeletal input.
        """
        sampler = self._sampler
        if sampler is None or getattr(sampler, "_vr_system", None) is None:
            return
        try:
            import openvr
        except Exception:
            return
        system = sampler._vr_system
        cls_names = {
            int(openvr.TrackedDeviceClass_Invalid): "Invalid",
            int(openvr.TrackedDeviceClass_HMD): "HMD",
            int(openvr.TrackedDeviceClass_Controller): "Controller",
            int(openvr.TrackedDeviceClass_GenericTracker): "Tracker",
            int(openvr.TrackedDeviceClass_TrackingReference): "Reference",
        }
        role_names = {
            int(openvr.TrackedControllerRole_LeftHand): "Left",
            int(openvr.TrackedControllerRole_RightHand): "Right",
        }
        print("[GR1T2FourierUDCAPDevice] --- OpenVR device inventory ---")
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            try:
                cls = int(system.getTrackedDeviceClass(i))
            except Exception:
                continue
            if cls == int(openvr.TrackedDeviceClass_Invalid):
                continue
            cls_name = cls_names.get(cls, f"cls={cls}")
            try:
                serial = system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_SerialNumber_String
                )
            except Exception:
                serial = "?"
            extra = ""
            if cls == int(openvr.TrackedDeviceClass_Controller):
                try:
                    ctype = system.getStringTrackedDeviceProperty(
                        i, openvr.Prop_ControllerType_String
                    )
                except Exception as exc:
                    ctype = f"<err: {exc}>"
                try:
                    role = int(system.getControllerRoleForTrackedDeviceIndex(i))
                except Exception:
                    role = -1
                try:
                    rmodel = system.getStringTrackedDeviceProperty(
                        i, openvr.Prop_RenderModelName_String
                    )
                except Exception:
                    rmodel = "?"
                extra = (
                    f" controller_type={ctype!r} "
                    f"role={role_names.get(role, str(role))} "
                    f"render_model={rmodel!r}"
                )
            print(f"  idx={i:2d} cls={cls_name:11s} serial={serial!r:30s}{extra}")

        # 9.15 — detect UDCAP virtual-knuckles + Pico/Quest Touch coexistence
        # and warn about Space Plan misconfiguration.  When UDCAP injects
        # virtual knuckles with role=Left/Right while the user's actual
        # Touch controllers carry role=Invalid (=0), the sampler picks
        # the UDCAP knuckles -- which under "Space Plan = Vive Tracker
        # 3.0" mode synthesize a static / non-moving wrist pose because
        # there is no real Vive tracker to anchor on.
        knuckles_serials: list[str] = []
        touch_serials: list[str] = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            try:
                if int(system.getTrackedDeviceClass(i)) != int(
                    openvr.TrackedDeviceClass_Controller
                ):
                    continue
                ctype = system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_ControllerType_String
                )
                serial = system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_SerialNumber_String
                )
            except Exception:
                continue
            if "knuckles" in str(ctype).lower():
                knuckles_serials.append(str(serial))
            elif "oculus_touch" in str(ctype).lower() or "touch" in str(ctype).lower():
                touch_serials.append(str(serial))
        if knuckles_serials and touch_serials:
            print(
                "[GR1T2FourierUDCAPDevice] *** UDCAP CONFIGURATION CHECK ***\n"
                f"  UDCAP virtual knuckles detected: {knuckles_serials}\n"
                f"  Real Touch controllers also present: {touch_serials}\n"
                "  ──> Sampler will pick the UDCAP knuckles because they\n"
                "      claim role=Left/Right (Touch controllers report\n"
                "      role=Invalid).  If your robot's wrist target Z stays\n"
                "      CONSTANT across many frames (visible in the\n"
                "      [GR1T2Retarget #N] L_pos / R_pos lines as the third\n"
                "      coordinate), UDCAP is generating a fake static pose\n"
                "      and you must fix the UDCAP setting:\n"
                "        UDCAP UI > Settings > Controller > Space Orientation\n"
                "        Space Plan = 'Vive Tracker 3.0' is WRONG when you\n"
                "        do not have a real Vive Tracker.  Change to a\n"
                "        Space Plan that matches your hardware (Index\n"
                "        Knuckles, Quest Touch, etc.) OR zero out the\n"
                "        Left/Right Offset Position + Degrees so UDCAP\n"
                "        does not displace the controller pose.\n"
                "      Additionally:  UDCAP UI > Settings > General >\n"
                "        Controller_Priority -> Low  (so the actual\n"
                "        Pico/Quest controllers are not masked)."
            )

    def _probe_openvr_skeletal(self) -> None:
        """Call the OpenVR skeletal API directly (outside of sampler's
        silent ``except``) so that a real ImportError / "action not
        bound" exception surfaces.

        We report, per side:
          * action handle (from sampler),
          * ``getSkeletalTrackingLevel`` (0=Estimated, 1=Partial, 2=Full),
          * ``getBoneCount`` (31 for Index profile),
          * ``getSkeletalActionData`` (bActive + activeOrigin).

        When skeletal binding is set up correctly, ``bActive == True``
        and ``activeOrigin`` != 0.  When UDCAP's driver does not
        implement skeletal input, ``bActive == False`` even though
        trigger/grip still work — that is the actionable signal.
        """
        sampler = self._sampler
        if sampler is None or getattr(sampler, "_vr_input", None) is None:
            return
        try:
            import openvr
        except Exception:
            return
        vi = sampler._vr_input
        handles = getattr(sampler, "_h_skeletons", {}) or {}
        print("[GR1T2FourierUDCAPDevice] --- OpenVR skeletal probe ---")
        if not handles:
            print("  (sampler has no skeletal action handles; enable_skeletal=False?)")
            return
        level_names = {0: "Estimated", 1: "Partial", 2: "Full"}
        for side, h in handles.items():
            print(f"  {side}: action_handle=0x{int(h):x}")
            # 1. Action state: is the action bound to any origin?
            try:
                data = vi.getSkeletalActionData(h)
                # VRSkeletalActionData_t has bActive + activeOrigin
                print(
                    f"    getSkeletalActionData: bActive={bool(data.bActive)} "
                    f"activeOrigin=0x{int(data.activeOrigin):x}"
                )
            except Exception as exc:
                print(f"    getSkeletalActionData FAILED: {type(exc).__name__}: {exc}")
            # 2. Tracking level (requires binding)
            try:
                lvl = int(vi.getSkeletalTrackingLevel(h))
                print(
                    f"    getSkeletalTrackingLevel={lvl} ({level_names.get(lvl, '?')})"
                )
            except Exception as exc:
                print(f"    getSkeletalTrackingLevel FAILED: {type(exc).__name__}: {exc}")
            # 3. Bone count (31 for hand skeletal)
            try:
                n = int(vi.getBoneCount(h))
                print(f"    getBoneCount={n}")
            except Exception as exc:
                print(f"    getBoneCount FAILED: {type(exc).__name__}: {exc}")
        print(
            "  → Interpretation:\n"
            "    • all three succeed → UDCAP emits skeletal, sampler bug. Look at vr_sampler._read_hand.\n"
            "    • getSkeletalActionData succeeds with bActive=False → binding file didn't match the\n"
            "      controller profile (see controller_type above; add that profile to bindings_index.json).\n"
            "    • all three raise 'action not bound' → SteamVR hasn't loaded the manifest. Check that\n"
            "      actions_json path is absolute on disk and restart SteamVR so it re-scans bindings.\n"
            "    • getBoneCount raises 'invalid operation' with bActive=True → UDCAP driver does NOT\n"
            "      implement skeletal protocol. Use --path_b_port 39539 (VMC) or the new controller-\n"
            "      button grip fallback (enable_button_grip_fallback=True, default ON)."
        )

    def stop(self) -> None:
        if self._sampler is not None:
            self._sampler.stop()
            self._sampler = None
        if self._vmc is not None:
            self._vmc.stop()
            self._vmc = None
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    # ── DeviceBase ─────────────────────────────────────────────────────
    def reset(self) -> None:
        self._retargeter.reset()
        if self._waist is not None:
            self._waist.reset()
        if self._head is not None:
            self._head.reset()
        self._advance_count = 0
        self._first_advance_logged = False
        self._zero_streak_advances = 0
        self._zero_streak_warned = False
        self._ctrl_z_track = {
            "left": (float("inf"), float("-inf")),
            "right": (float("inf"), float("-inf")),
        }
        self._frozen_z_warned = False

    def head_estimate(self) -> Optional["HeadEstimate"]:
        """Return the latest HMD-derived head joint targets, or None if
        head tracking is disabled / no HMD pose has arrived yet."""
        if self._head is None or self._sampler is None:
            return None
        snap = self._sampler.snapshot()
        if not snap or snap.get("frame_count", 0) == 0:
            return None
        return self._head.estimate(snap)

    def add_callback(self, key: Any, func: Any) -> None:  # pragma: no cover
        pass

    def _get_raw_data(self) -> Optional[Dict[str, Any]]:
        if self._sampler is None:
            return None
        return self._sampler.snapshot()

    def advance(self) -> Optional[torch.Tensor]:
        """Return the latest 36D action (or ``None`` before first frame)."""
        if not self._started:
            self.start()
        snap = self._get_raw_data()
        if snap is None or snap.get("frame_count", 0) == 0:
            return None

        udcap_bones = self._vmc.latest_bones() if self._vmc is not None else None
        # Read trigger/grip + per-finger curls via the SteamVR Input action
        # API.  UDCAP / LucidVR-family knuckles emulators don't populate the
        # legacy ``getControllerState()`` path that the sampler reads, so
        # without this the button-grip fallback would see 0.00 and never
        # fire.  The finger-curl channel is the primary source for gloves
        # that emit per-finger values via /user/hand/*/input/finger/* but
        # don't implement Skeletal Input 2.0.
        action_inputs = self._read_action_inputs()
        action = self._retargeter.retarget(
            snap,
            udcap_bones=udcap_bones,
            action_inputs=action_inputs,
        )
        self._advance_count += 1

        # Runtime watchdog: track how long every action input has been zero.
        # If we see ~10 s of all-zero inputs after the first frame, surface a
        # one-shot warning that points at the most likely cause (UdcapDriver
        # not forwarding glove data).  Pose comes from the controllers/
        # trackers, so the arm continues to track — only fingers stay frozen.
        if not self._zero_streak_warned:
            all_zero = True
            for v in action_inputs.values():
                if (v.get("trigger", 0.0) > 1e-3
                        or v.get("grip", 0.0) > 1e-3
                        or any(c > 1e-3 for c in v.get("finger_curls", ()) or ())):
                    all_zero = False
                    break
            if all_zero:
                self._zero_streak_advances += 1
            else:
                self._zero_streak_advances = 0
            # 200 advance() calls ≈ 10 s at the typical 20 Hz teleop step.
            if self._zero_streak_advances >= 200:
                self._zero_streak_warned = True
                vmc_status = "DISABLED" if self._vmc is None else (
                    "OK -- bones flowing" if udcap_bones else
                    "ENABLED but receiving NO packets on UDP "
                    f"{self.cfg.path_b_port}"
                )
                print(
                    "[GR1T2FourierUDCAPDevice] WARNING -- no SteamVR Input action\n"
                    "  (trigger / grip / per-finger curl) has been non-zero for ~10 s.\n"
                    f"  VMC fallback: {vmc_status}\n"
                    "  Most likely cause (priority order):\n"
                    "    1. *** UDCAP NOT CALIBRATED ***  Open the UDCAP system-tray\n"
                    "       widget; if you see 'Not Calibration / Please Calibration'\n"
                    "       press F1 (or click Calibration(F1)) and follow the open /\n"
                    "       fist / per-finger sequence for both hands.  UDCAP refuses\n"
                    "       to emit finger data on any channel until calibrated.\n"
                    "    2. Controller_Priority is High in UDCAP General settings,\n"
                    "       letting the underlying Pico/Quest controller mask glove\n"
                    "       inputs.  Switch to Low or power off the physical controller.\n"
                    "    3. Glove not actually connected -- UDCAP UI > Devices should\n"
                    "       show both gloves Connected with FPS > 30.\n"
                    "    4. UDCAP's VMC broadcast disabled or on a different port.  In\n"
                    "       the UDCAP tray UI, open Settings > Output and confirm VMC is\n"
                    "       enabled with port 39539 (or pass --path_b_port <your-port>).\n"
                    "  For a layer-by-layer inspection run:\n"
                    "    python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_udcap_dataflow\n"
                    "  Arm tracking continues from forearm trackers; this warning only\n"
                    "  affects FINGER control."
                )

        # 9.13: periodic VMC packet count log so the user can confirm at a
        # glance whether bone data is actually arriving on the VMC port.
        # Surfaces "VMC enabled but UDCAP not broadcasting" cases that the
        # 10 s zero-streak warning would otherwise mask.
        if self.cfg.debug and self._vmc is not None and self._advance_count % 100 == 0:
            n_bones = len(udcap_bones) if udcap_bones else 0
            print(
                f"[GR1T2FourierUDCAPDevice][advance #{self._advance_count}] "
                f"VMC port={self.cfg.path_b_port} bones_received={n_bones}"
                f"{' (waiting for first packet)' if n_bones == 0 else ''}"
            )

        # 9.15: detect frozen controller-Z (the symptom of UDCAP Space Plan
        # = "Vive Tracker 3.0" mode generating a static fake controller
        # pose).  Track the controller pose Z over advance() calls; if it
        # stays within a tiny epsilon for ~10 s, surface a one-shot
        # warning pointing at the UDCAP setting.
        if not self._frozen_z_warned:
            ctrls = snap.get("controllers") or {}
            for side in ("left", "right"):
                c = ctrls.get(side)
                if c is None or not isinstance(c, dict):
                    continue
                pose = c.get("pose")
                if not pose:
                    continue
                z = float(pose["pos"][1])  # SVR Y = up
                lo, hi = self._ctrl_z_track[side]
                self._ctrl_z_track[side] = (min(lo, z), max(hi, z))
            # After ~10 s (200 advances), if either side's Z range is
            # < 2 mm, the UDCAP-fake-controller diagnosis is highly
            # likely.  We tolerate the FIRST 60 advances (~3 s) so the
            # initial cal frames don't dominate.
            if self._advance_count >= 200:
                ranges = {}
                for side in ("left", "right"):
                    lo, hi = self._ctrl_z_track[side]
                    ranges[side] = max(0.0, hi - lo) if lo != float("inf") else 0.0
                if max(ranges["left"], ranges["right"]) < 0.002:
                    self._frozen_z_warned = True
                    print(
                        "[GR1T2FourierUDCAPDevice] *** FROZEN CONTROLLER Z DETECTED ***\n"
                        f"  Over the past {self._advance_count} advances, controller pose\n"
                        f"  Z range: L={ranges['left']*1000:.1f} mm, R={ranges['right']*1000:.1f} mm.\n"
                        "  This means the wrist EEF target NEVER changes height even\n"
                        "  if the user raises their arm overhead.  Most likely cause:\n"
                        "    UDCAP UI > Settings > Controller > Space Orientation\n"
                        "    Space Plan = 'Vive Tracker 3.0' is set, but no real\n"
                        "    Vive Tracker is in use.  UDCAP synthesises a fake static\n"
                        "    knuckles pose anchored to nothing.  Change Space Plan\n"
                        "    to match your hardware (Index / Quest Touch / Pico) OR\n"
                        "    zero out the Position + Degrees offsets in that screen.\n"
                        "  Additionally: UDCAP UI > Settings > General >\n"
                        "    Controller_Priority -> Low (so Touch is not masked)."
                    )

        if self.cfg.debug and not self._first_advance_logged:
            info = self._retargeter.get_source_info()
            trks = snap.get("trackers") or {}
            hands = snap.get("hands") or {}
            ctrls = snap.get("controllers") or {}

            def _fmt(p):
                if p is None:
                    return "None"
                pp = p["pos"]
                return f"({pp[0]:+.3f},{pp[1]:+.3f},{pp[2]:+.3f})"

            def _fmt_hand(h):
                """Explain the sampler's view of skeletal hand data so the
                user can tell "no UDCAP glove" apart from "OpenVR action
                unbound" apart from "sampler crashed silently"."""
                if h is None:
                    return "None (sampler returned no skeletal data)"
                bones = h.get("bones")
                curls = h.get("fingerCurls")
                splays = h.get("fingerSplays")
                bshape = tuple(bones.shape) if bones is not None else None
                cshape = tuple(curls.shape) if curls is not None else None
                sshape = tuple(splays.shape) if splays is not None else None
                return (
                    f"bones={bshape} fingerCurls={cshape} "
                    f"fingerSplays={sshape} "
                    f"curls_sum={float(curls.sum()) if curls is not None else 'n/a'}"
                )

            def _fmt_ctrl(c):
                if c is None:
                    return "None"
                pose = c.get("pose")
                buttons = c.get("buttons") or {}
                return (
                    f"pose={_fmt(pose)} "
                    f"trigger={buttons.get('trigger', 0):.2f} "
                    f"grip={buttons.get('grip', 0):.2f} "
                    f"menu={buttons.get('menu', False)}"
                )

            def _fmt_act(side):
                v = action_inputs.get(side) or {}
                curls = v.get("finger_curls") or [0.0] * 5
                return (
                    f"trigger={v.get('trigger', 0):.2f} grip={v.get('grip', 0):.2f} "
                    f"curls=[thb={curls[0]:.2f} idx={curls[1]:.2f} mid={curls[2]:.2f} "
                    f"rng={curls[3]:.2f} pnk={curls[4]:.2f}]"
                )

            print(
                "[GR1T2FourierUDCAPDevice] --- base_link-frame diagnostic ---\n"
                f"  SteamVR world: waist={_fmt(trks.get('waist'))} "
                f"left_forearm={_fmt(trks.get('left_forearm'))} "
                f"right_forearm={_fmt(trks.get('right_forearm'))}\n"
                f"  use_waist_origin={self.cfg.use_waist_origin} "
                f"subtract_waist_z={self.cfg.subtract_waist_z}\n"
                f"  L_EEF target: {[f'{v:+.3f}' for v in action[0:3].tolist()]}\n"
                f"  R_EEF target: {[f'{v:+.3f}' for v in action[7:10].tolist()]}\n"
                f"  sources={info} "
                f"nonzero_fingers={(action[14:36].abs() > 1e-4).sum().item()}/22\n"
                f"  --- hand-skeletal diagnostic ---\n"
                f"  hands.left : {_fmt_hand(hands.get('left'))}\n"
                f"  hands.right: {_fmt_hand(hands.get('right'))}\n"
                f"  ctrls.left  (legacy): {_fmt_ctrl(ctrls.get('left'))}\n"
                f"  ctrls.right (legacy): {_fmt_ctrl(ctrls.get('right'))}\n"
                f"  --- action-API input diagnostic (preferred path for knuckles) ---\n"
                f"  left  actions: {_fmt_act('left')}\n"
                f"  right actions: {_fmt_act('right')}\n"
                f"  → If `action curls` are all 0 while legacy `ctrls.*.trigger` is also 0,\n"
                f"    either the user is at rest OR the SteamVR binding didn't apply.\n"
                f"    Squeeze the trigger during teleop; if the log later shows nonzero\n"
                f"    `curls` or `trigger`, the action path works.  If not:\n"
                f"    SteamVR → Settings → Controllers → Manage Controller Bindings →\n"
                f"    UST Teleop, activate the Index-profile binding named\n"
                f"    'UST Teleop — Index profile + per-finger curl', then relaunch."
            )
            self._first_advance_logged = True

        return action

    # ── diagnostics / accessors ────────────────────────────────────────
    @property
    def is_connected(self) -> bool:
        if self._sampler is None:
            return False
        return self._sampler.frame_count > 0

    def snapshot(self) -> Optional[Dict[str, Any]]:
        return self._get_raw_data()

    def get_controller_data(self) -> Optional[Dict[str, Any]]:
        snap = self._get_raw_data()
        if snap is None:
            return None
        ctrl = snap.get("controllers") or {"left": None, "right": None}
        if ctrl.get("left") is None and ctrl.get("right") is None:
            return None
        return ctrl

    def get_source_info(self) -> Dict[str, str]:
        return self._retargeter.get_source_info()

    def get_waist_estimate(self) -> Optional[WaistEstimate]:
        """Return the current waist Euler estimate or ``None`` when disabled."""
        if self._waist is None:
            return None
        snap = self._get_raw_data()
        if snap is None:
            return None
        return self._waist.estimate(snap)


class GR1T2InterventionInterface(PICOInterventionInterface):
    """Button debouncing intervention trigger for HG-DAgger.

    Identical semantics to the G1 ``PICOInterventionInterface`` — both
    grips pressed → intervention; both triggers → resume; both menu
    buttons → reset.  Subclassed to keep the ``ust_hm_glove`` API
    self-contained when the rest of the pipeline imports from this
    package only.
    """

    def __init__(
        self,
        device: GR1T2FourierUDCAPDevice,
        grip_threshold: float = 0.8,
        trigger_threshold: float = 0.8,
        debounce_frames: int = 3,
    ) -> None:
        super().__init__(
            device=device,  # duck-typed against PicoUDCAPDevice
            grip_threshold=grip_threshold,
            trigger_threshold=trigger_threshold,
            debounce_frames=debounce_frames,
        )
