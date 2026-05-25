"""`PicoUDCAPDevice` — Windows/SteamVR teleop device for Isaac Lab.

Mirrors the interface of ``PICOFullBodyTeleopDevice`` (``ust_260220``) so
that ``run_teleop.py`` / ``record_demos.py`` can swap implementations with
minimal edits.  Produces the same 38D Pink IK action tensor.

Unlike the Linux predecessor this implementation is fully in-process:

  * ``SteamVRSampler`` runs a 120 Hz pyopenvr thread and publishes the
    latest snapshot of (HMD, 5 trackers, both-hand skeletal) under a lock.
  * ``VMCHandReceiver`` is started only when ``path_b_port`` is set, acting
    as a fallback for the UDCAP SteamVR Add-on.
  * ``G1SteamVRRetargeter`` converts the snapshot to the 38D action.

The device exposes ``advance() -> torch.Tensor`` plus convenience
accessors used by ``PICOInterventionInterface`` (button state) and the
demo recorder (latest snapshot / diagnostic counters).

Because Isaac Lab's ``DeviceBase`` lives under ``isaaclab.devices`` which
in turn imports ``carb`` from Isaac Sim at module import time, we only
require it at run time.  The dataclass ``PicoUDCAPDeviceCfg`` subclasses
``isaaclab.devices.DeviceCfg`` when Isaac Lab is available; otherwise it
falls back to a plain dataclass so unit tests can import this module.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np
import torch


# ── Optional Isaac Lab imports ─────────────────────────────────────────────
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


from .coord_transforms import svr_to_isaaclab
from .g1_retargeter import G1SteamVRRetargeter
from .vr_sampler import SteamVRSampler


__all__ = ["PicoUDCAPDeviceCfg", "PicoUDCAPDevice", "PICOInterventionInterface"]


@dataclass
class PicoUDCAPDeviceCfg(DeviceCfg):
    """Configuration for :class:`PicoUDCAPDevice`.

    All paths may be absolute or repo-root relative (``./ust_ws/...``);
    the device normalises them on construction.
    """

    tracker_binding_json: str = "./ust_ws/ust_hm_glove/config/tracker_binding.json"
    actions_json: str = "./ust_ws/ust_hm_glove/config/openvr_actions/actions.json"
    forearm_wrist_offset: Tuple[float, float, float] = (0.12, 0.0, 0.0)
    left_hand_retarget_yaml: Optional[str] = (
        "./ust_ws/ust_hm_glove/config/dex_retargeting/inspire_left_dexpilot.yml"
    )
    right_hand_retarget_yaml: Optional[str] = (
        "./ust_ws/ust_hm_glove/config/dex_retargeting/inspire_right_dexpilot.yml"
    )
    dex_urdf_dir: Optional[str] = None

    position_scale: float = 1.0
    rotation_scale: float = 1.0
    gripper_threshold: float = 0.5
    body_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    controller_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Express EEF targets in the user's pelvis frame by subtracting the
    # waist tracker position.  REQUIRED for Pink IK because its target is
    # pelvis-relative, but the raw SteamVR world coordinates depend on
    # wherever the user happens to stand in the play area.
    use_waist_origin: bool = True
    # Only subtract waist Z if the IK cfg expects a pelvis-joint-local Z
    # (floor-anchored Z such as Pink IK's is handled with False).
    subtract_waist_z: bool = False
    # Diagnostic: freeze wrist orientation at idle so Pink IK only has to
    # solve position (use this to split "IK fails due to bad orientation"
    # from "IK fails due to bad position").
    freeze_orientation: bool = False

    sampler_rate_hz: float = 120.0
    # Watchdog timeout for openvr.init() inside SteamVRSampler.start().
    # Mirrors GR1T2FourierUDCAPDeviceCfg.sampler_init_timeout_sec — see
    # 9.38 / memory.md §10.45 for the silent-hang failure mode.
    sampler_init_timeout_sec: float = 30.0
    # Skeleton 2.0 (SteamVR Skeletal Input) — primary finger source in 9.37.
    # When True, ``vr_sampler`` queries the 31-bone hand skeleton at
    # ``sampler_rate_hz`` and feeds it into the retargeter; the resulting
    # action takes precedence over VMC and trigger/grip fallbacks.
    enable_skeletal: bool = True

    # Path B (VMC) fallback — kept available for setups where the UDCAP
    # SteamVR driver does not implement Skeletal Input 2.0.  Default 0
    # (disabled) in 9.37 to match the GR1T2 device and the user-requested
    # Skeleton 2.0 migration.  Set to 39539 to re-enable VMC.
    path_b_port: int = 0

    debug: bool = True


class PicoUDCAPDevice(DeviceBase):
    """Isaac Lab DeviceBase: SteamVR + UDCAP + G1 retargeter → 38D action."""

    ACTION_DIM = G1SteamVRRetargeter.ACTION_DIM

    def __init__(self, cfg: PicoUDCAPDeviceCfg):
        super().__init__()
        self.cfg = cfg
        self._sampler: Optional[SteamVRSampler] = None
        self._vmc = None
        self._started = False
        self._advance_count = 0
        self._first_advance_logged = False

        self._tracker_binding = self._load_tracker_binding(
            self._absolutise(cfg.tracker_binding_json)
        )

        left_yaml = self._absolutise(cfg.left_hand_retarget_yaml) if cfg.left_hand_retarget_yaml else None
        right_yaml = self._absolutise(cfg.right_hand_retarget_yaml) if cfg.right_hand_retarget_yaml else None
        dex_urdf_dir = self._absolutise(cfg.dex_urdf_dir) if cfg.dex_urdf_dir else None

        self._retargeter = G1SteamVRRetargeter(
            position_scale=cfg.position_scale,
            rotation_scale=cfg.rotation_scale,
            gripper_threshold=cfg.gripper_threshold,
            body_pos_offset=cfg.body_pos_offset,
            controller_pos_offset=cfg.controller_pos_offset,
            forearm_wrist_offset=cfg.forearm_wrist_offset,
            left_hand_retarget_yaml=left_yaml,
            right_hand_retarget_yaml=right_yaml,
            dex_urdf_dir=dex_urdf_dir,
            use_waist_origin=cfg.use_waist_origin,
            subtract_waist_z=cfg.subtract_waist_z,
            freeze_orientation=cfg.freeze_orientation,
            debug=cfg.debug,
        )

    # ── path helpers ──────────────────────────────────────────────────────
    @staticmethod
    def _absolutise(path: str) -> str:
        p = Path(path)
        if not p.is_absolute():
            p = Path.cwd() / p
        return str(p.resolve())

    @staticmethod
    def _load_tracker_binding(path: str) -> Dict[str, Dict[str, str]]:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        trackers = data.get("trackers", data)
        # trackers: {serial: {"role": ..., "steamvr_role": ...}}
        return {str(sn): {str(k): str(v) for k, v in info.items()} for sn, info in trackers.items()}

    # ── lifecycle ─────────────────────────────────────────────────────────
    def start(self) -> None:
        if self._started:
            return
        self._sampler = SteamVRSampler(
            tracker_binding=self._tracker_binding,
            actions_json_path=self._absolutise(self.cfg.actions_json),
            rate_hz=self.cfg.sampler_rate_hz,
            enable_skeletal=self.cfg.enable_skeletal,
            init_timeout_sec=self.cfg.sampler_init_timeout_sec,
        )
        self._sampler.start()

        if self.cfg.path_b_port and self.cfg.path_b_port > 0:
            try:
                from .vmc_receiver import VMCHandReceiver  # noqa: WPS433

                self._vmc = VMCHandReceiver(port=int(self.cfg.path_b_port))
                self._vmc.start()
            except Exception as exc:  # noqa: BLE001
                print(f"[PicoUDCAPDevice] VMC fallback disabled ({exc}).")
                self._vmc = None

        self._started = True
        if self.cfg.debug:
            print(
                f"[PicoUDCAPDevice] started — actions='{self.cfg.actions_json}' "
                f"binding={len(self._tracker_binding)} trackers "
                f"skeletal={self.cfg.enable_skeletal} path_b={bool(self._vmc)}"
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

    # ── DeviceBase ────────────────────────────────────────────────────────
    def reset(self) -> None:
        self._retargeter.reset()
        self._advance_count = 0
        self._first_advance_logged = False

    def add_callback(self, key: Any, func: Any) -> None:  # pragma: no cover — not used
        pass

    def _get_raw_data(self) -> Optional[Dict[str, Any]]:
        if self._sampler is None:
            return None
        return self._sampler.snapshot()

    def advance(self) -> Optional[torch.Tensor]:
        """Return the latest 38D Pink IK action (or ``None`` before first frame)."""
        if not self._started:
            self.start()
        snap = self._get_raw_data()
        if snap is None or snap.get("frame_count", 0) == 0:
            return None

        udcap_bones = self._vmc.latest_bones() if self._vmc is not None else None
        action = self._retargeter.retarget(snap, udcap_bones=udcap_bones)
        self._advance_count += 1

        if self.cfg.debug and not self._first_advance_logged:
            info = self._retargeter.get_source_info()
            # Dump raw + computed coords so the user can sanity-check the
            # pelvis-frame conversion.
            trks = snap.get("trackers") or {}
            waist = trks.get("waist")
            lf = trks.get("left_forearm")
            rf = trks.get("right_forearm")
            def _fmt(p):
                if p is None:
                    return "None"
                pp = p["pos"]
                return f"({pp[0]:+.3f},{pp[1]:+.3f},{pp[2]:+.3f})"
            print(
                "[PicoUDCAPDevice] --- pelvis-frame diagnostic ---\n"
                f"  SteamVR world: waist={_fmt(waist)} left_forearm={_fmt(lf)} right_forearm={_fmt(rf)}\n"
                f"  use_waist_origin={self.cfg.use_waist_origin} subtract_waist_z={self.cfg.subtract_waist_z}\n"
                f"  L_EEF target (pelvis frame): {[f'{v:+.3f}' for v in action[0:3].tolist()]} "
                f"(idle: [-0.149, +0.204, +1.095])\n"
                f"  R_EEF target (pelvis frame): {[f'{v:+.3f}' for v in action[7:10].tolist()]} "
                f"(idle: [+0.149, +0.204, +1.095])\n"
                f"  sources={info} nonzero_fingers={(action[14:38].abs() > 1e-4).sum().item()}/24"
            )
            print(
                f"[PicoUDCAPDevice] first action: "
                f"L_EEF={action[0:3].tolist()} L_QUAT={action[3:7].tolist()} "
                f"R_EEF={action[7:10].tolist()} R_QUAT={action[10:14].tolist()} "
                f"sources={info} nonzero_fingers={(action[14:38].abs() > 1e-4).sum().item()}/24"
            )
            self._first_advance_logged = True

        return action

    # ── diagnostics ───────────────────────────────────────────────────────
    @property
    def is_connected(self) -> bool:
        """True when the sampler has produced at least one frame."""
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


# ── Intervention interface — button debouncer on top of the device ────────


class PICOInterventionInterface:
    """HG-DAgger-style intervention/resume trigger from UDCAP/Index buttons.

    Mirrors ``ust_260220.teleop.pico_intervention.PICOInterventionInterface``
    but reads state from :class:`PicoUDCAPDevice` instead of the old TCP
    device.  The UDCAP driver emulates Valve Index controllers, so the
    "trigger" and "grip" axes are the ones reported by SteamVR.
    """

    def __init__(
        self,
        device: PicoUDCAPDevice,
        grip_threshold: float = 0.8,
        trigger_threshold: float = 0.8,
        debounce_frames: int = 3,
    ):
        self.device = device
        self.grip_threshold = float(grip_threshold)
        self.trigger_threshold = float(trigger_threshold)
        self.debounce_frames = int(debounce_frames)
        self._grip_count = 0
        self._trigger_count = 0
        self._reset_count = 0

    def _controllers(self) -> Optional[Dict[str, Any]]:
        data = self.device.get_controller_data()
        if not data:
            return None
        if data.get("left") is None and data.get("right") is None:
            return None
        return data

    def check_intervention_trigger(self) -> bool:
        data = self._controllers()
        if data is None:
            self._grip_count = 0
            return False
        lg = self._grip_of(data.get("left"))
        rg = self._grip_of(data.get("right"))
        if lg > self.grip_threshold and rg > self.grip_threshold:
            self._grip_count += 1
        else:
            self._grip_count = 0
        return self._grip_count >= self.debounce_frames

    def check_resume_trigger(self) -> bool:
        data = self._controllers()
        if data is None:
            self._trigger_count = 0
            return False
        lt = self._trigger_of(data.get("left"))
        rt = self._trigger_of(data.get("right"))
        if lt > self.trigger_threshold and rt > self.trigger_threshold:
            self._trigger_count += 1
        else:
            self._trigger_count = 0
        return self._trigger_count >= self.debounce_frames

    def check_reset_trigger(self) -> bool:
        data = self._controllers()
        if data is None:
            self._reset_count = 0
            return False
        lm = self._menu_of(data.get("left"))
        rm = self._menu_of(data.get("right"))
        if lm and rm:
            self._reset_count += 1
        else:
            self._reset_count = 0
        return self._reset_count >= self.debounce_frames

    @staticmethod
    def _trigger_of(side_state: Optional[Dict[str, Any]]) -> float:
        if not side_state:
            return 0.0
        buttons = side_state.get("buttons") or {}
        return float(buttons.get("trigger", 0.0))

    @staticmethod
    def _grip_of(side_state: Optional[Dict[str, Any]]) -> float:
        if not side_state:
            return 0.0
        buttons = side_state.get("buttons") or {}
        return float(buttons.get("grip", 0.0))

    @staticmethod
    def _menu_of(side_state: Optional[Dict[str, Any]]) -> bool:
        if not side_state:
            return False
        buttons = side_state.get("buttons") or {}
        return bool(buttons.get("menu", False))

    def reset(self) -> None:
        self._grip_count = 0
        self._trigger_count = 0
        self._reset_count = 0
