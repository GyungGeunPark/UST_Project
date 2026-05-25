"""``GR1T2GripperDevice`` — UDCAP-free PICO controller teleop device.

This is the **option B** drop-in replacement for ``GR1T2FourierUDCAPDevice``
when:

* The robot's hands are replaced by a 2-finger parallel gripper (USD-side
  change is performed by ``isaac_file/build_gripper_usd.py`` before runtime).
* The user wears motion trackers on the **elbows / proximal forearm**
  (PICO Connect "Forearm Tracking Enhanced" calibration mode).
* The user holds **PICO Touch controllers** in both hands and the trigger
  drives the gripper open/close state.
* UDCAP gloves and the Windows mini-PC are **decommissioned** — VMC OSC,
  the SteamVR Skeletal Input 2.0 path, and per-finger curl actions all go
  away.

Compared to ``GR1T2FourierUDCAPDevice`` the differences are:

* No ``VMCHandReceiver`` (``path_b_port = 0`` always).
* No ``UDCAP / UDCAP_overlay`` process probe.
* No frozen-controller-Z watchdog (UDCAP-specific failure mode).
* No skeletal probe (PICO controllers don't emit Skeletal 2.0).
* Action handles narrowed to ``trigger_*`` + ``grip_*`` + ``menu_*``.
* The retargeter is the new 16-D :class:`GR1T2GripperSteamVRRetargeter`.

The ``GR1T2InterventionInterface`` from the legacy device is replaced by
:class:`GripperInterventionInterface` whose intervention/resume buttons are
``menu`` (no longer ``trigger``, since the trigger is now reserved for
gripper control).
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


from ust_ws.ust_hm_grip.teleop.vr_sampler import SteamVRSampler

from .gr1t2_gripper_retargeter import (
    ACTION_DIM,
    GR1T2GripperRetargeterCfg,
    GR1T2GripperSteamVRRetargeter,
)


__all__ = [
    "GR1T2GripperDevice",
    "GR1T2GripperDeviceCfg",
    "GripperInterventionInterface",
]


@dataclass
class GR1T2GripperDeviceCfg(DeviceCfg):
    """Configuration for :class:`GR1T2GripperDevice`."""

    # Hardware / VR config
    tracker_binding_json: str = "./ust_ws/ust_hm_grip/config/tracker_binding.json"
    actions_json: str = "./ust_ws/ust_hm_grip/config/openvr_actions/actions.json"
    vrmanifest_json: str = (
        "./ust_ws/ust_hm_grip/config/openvr_actions/manifest.vrmanifest"
    )
    app_key: str = "ust.teleop.gr1t2_gripper"

    # Forearm-tracker offset.  Default 0.28 m matches the elbow-mounted
    # tracker placement that PICO Connect's "Forearm Tracking Enhanced"
    # mode expects.  Override per-user via a calibration script if needed.
    forearm_wrist_offset: Tuple[float, float, float] = (0.28, 0.0, 0.0)

    position_scale: float = 1.0
    rotation_scale: float = 1.0
    # 13th session, part 10 — per-axis wrist position scale (X, Y, Z).
    # Default (1.0, 1.0, 1.5) amplifies vertical motion which users
    # report tracks weakly under 1:1 mapping.
    wrist_pos_scale_per_axis: Tuple[float, float, float] = (1.0, 1.0, 1.5)
    # 13th session, part 12 — split position vs orientation source.
    #   "controller"    — wrist EEF position driven by controller pos
    #                     (legacy; controller drives both pos and quat)
    #   "wrist_tracker" — wrist EEF position driven by body skeleton
    #                     SMPL L_Wrist/R_Wrist (xrobo_sampler
    #                     ``trackers["{side}_wrist"]``).  Controller
    #                     quat still drives wrist orientation.
    # Default ``wrist_tracker`` per user request: controller in hand
    # rotates the gripper, but the arm reach point comes from the
    # body skeleton wrist estimate (more anatomically accurate).
    wrist_pos_source: str = "wrist_tracker"
    # 13th session, part 15 — wrist EEF orientation source.
    # Research finding: PICO's Motion Tracker calibration app uses the
    # SMPL 24-joint body for arm+wrist POSITION but overlays the
    # CONTROLLER pose for fine wrist ROTATION (the SMPL wrist joints
    # idx 20/21 carry AI-smoothed orientation that does not capture
    # fine controller wrist motion — confirmed by NVIDIA SONIC/GR00T
    # ZMQ v3 protocol which also overrides wrist quat from controller).
    # See research/48 (XRoboToolkit body fusion report).
    #
    # Default split therefore matches the cal-app appearance:
    #   pos  ← body wrist tracker (SMPL L/R_Wrist)
    #   quat ← controller (delta-from-zero)
    # Controller pos is still ignored.  Controller buttons drive gripper.
    wrist_quat_source: str = "controller"
    body_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    controller_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Idle pose (override after calibration).
    idle_left_pos: Optional[Tuple[float, float, float]] = None
    idle_left_quat: Optional[Tuple[float, float, float, float]] = None
    idle_right_pos: Optional[Tuple[float, float, float]] = None
    idle_right_quat: Optional[Tuple[float, float, float, float]] = None

    use_waist_origin: bool = True
    subtract_waist_z: bool = False
    freeze_orientation: bool = False
    right_wrist_z180: bool = False  # 13th-bis part 8: see retargeter cfg

    # The PICO controller is held in hand → controller pose is the most
    # accurate wrist target.  The elbow-mounted forearm tracker is only
    # consulted as a fallback (e.g. if the controller is briefly out of
    # the headset's view).
    prefer_controller_for_eef: bool = True
    controller_to_wrist_offset: Tuple[float, float, float] = (0.0, 0.0, -0.05)

    # Gripper hysteresis thresholds.
    gripper_close_threshold: float = 0.6
    gripper_open_threshold: float = 0.4
    # 9.28: which controller input drives close/open ("grip" default,
    # "trigger", or "both").  Forwarded to the retargeter.
    gripper_signal_source: str = "grip"
    # Deprecated -- kept for backward compat.  Ignored by the retargeter
    # when ``gripper_signal_source`` is set.
    use_grip_as_close: bool = True

    # When True, force the wrist EEF target to the idle pose regardless of
    # tracker / controller state.  Mirrors the 9.26 ``--ignore_trackers``
    # flag (CLAUDE.md gotcha #17).
    disable_arm_tracking: bool = False

    # Sampler
    sampler_rate_hz: float = 120.0

    debug: bool = True

    # ─── Backend selection (research/47) ─────────────────────────────
    input_backend: str = "openvr"
    """Which input pipeline to use.  Choices:
        * "openvr" — legacy SteamVR Action API (requires Personal Binding).
        * "xrobotoolkit" — PICO official XRoboToolkit gRPC (research/47).
    """

    # ─── XRoboToolkit-specific ──────────────────────────────────────
    xrt_enable_body: bool = False
    """When True with input_backend=xrobotoolkit, populate snapshot
    ``trackers`` from the PICO 24-joint body mocap stream.  Requires
    PICO Motion Trackers + headset calibration."""

    xrt_enable_hand: bool = False
    """When True with input_backend=xrobotoolkit, populate snapshot
    ``hands`` from the OpenXR 26-joint hand tracking stream.  Unused
    by the current 16-D retargeter; reserved for future dexterous-hand
    upgrades."""


class GR1T2GripperDevice(DeviceBase):
    """SteamVR + PICO controller + 16-D gripper retargeter.

    Public API mirrors ``GR1T2FourierUDCAPDevice`` so the rest of the
    Isaac Lab stack (env action manager, demo recorder) can swap modules
    with a single import change.
    """

    ACTION_DIM = ACTION_DIM

    def __init__(self, cfg: GR1T2GripperDeviceCfg):
        super().__init__()
        self.cfg = cfg
        self._sampler: Optional[SteamVRSampler] = None
        self._started = False
        self._advance_count = 0
        self._first_advance_logged = False
        self._action_handles: Dict[str, Any] = {}

        self._tracker_binding = self._load_tracker_binding(
            self._absolutise(cfg.tracker_binding_json)
        )

        # 13th session — when running on the XRoboToolkit backend, the
        # XRoboSampler already converts every pose to Isaac Lab base_link
        # frame (via ``coord_transforms.xr_to_isaaclab``).  The retargeter
        # must therefore SKIP the legacy ``svr_to_isaaclab`` call to avoid
        # a double-transform that mangles positions and quaternions.
        _pose_in_il = (cfg.input_backend or "openvr").lower() == "xrobotoolkit"

        retargeter_cfg = GR1T2GripperRetargeterCfg(
            position_scale=cfg.position_scale,
            rotation_scale=cfg.rotation_scale,
            wrist_pos_scale_per_axis=tuple(cfg.wrist_pos_scale_per_axis),
            wrist_pos_source=str(cfg.wrist_pos_source),
            wrist_quat_source=str(cfg.wrist_quat_source),
            body_pos_offset=cfg.body_pos_offset,
            controller_pos_offset=cfg.controller_pos_offset,
            forearm_wrist_offset=cfg.forearm_wrist_offset,
            use_waist_origin=cfg.use_waist_origin,
            subtract_waist_z=cfg.subtract_waist_z,
            freeze_orientation=cfg.freeze_orientation,
            right_wrist_z180=cfg.right_wrist_z180,
            prefer_controller_for_eef=cfg.prefer_controller_for_eef,
            controller_to_wrist_offset=cfg.controller_to_wrist_offset,
            gripper_close_threshold=cfg.gripper_close_threshold,
            gripper_open_threshold=cfg.gripper_open_threshold,
            use_grip_as_close=cfg.use_grip_as_close,
            gripper_signal_source=cfg.gripper_signal_source,
            disable_arm_tracking=cfg.disable_arm_tracking,
            pose_in_il_frame=_pose_in_il,
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
        self._retargeter = GR1T2GripperSteamVRRetargeter(retargeter_cfg)

    # ── path helpers ───────────────────────────────────────────────────
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
        return {
            str(sn): {str(k): str(v) for k, v in info.items()}
            for sn, info in trackers.items()
        }

    @staticmethod
    def _generate_runtime_manifest(static_manifest_path: str) -> str:
        """Materialise a runtime ``.vrmanifest`` next to the static template
        with ``binary_path_windows`` pointed at the *current* Python
        interpreter (``sys.executable``).

        See the analogous helper in ``gr1t2_udcap_device.py`` for the
        rationale; SteamVR drops application entries whose binary path
        cannot be resolved relative to the manifest directory.
        """
        import sys as _sys

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

        backend = (self.cfg.input_backend or "openvr").lower()

        # ── XRoboToolkit branch (research/47) ──────────────────────────
        if backend == "xrobotoolkit":
            from ust_ws.ust_hm_grip.teleop.xrobo_sampler import XRoboSampler
            if self.cfg.debug:
                print(
                    "[GR1T2GripperDevice] backend=xrobotoolkit — skipping "
                    "SteamVR application registration."
                )
            self._sampler = XRoboSampler(
                rate_hz=self.cfg.sampler_rate_hz,
                enable_body=self.cfg.xrt_enable_body,
                enable_hand=self.cfg.xrt_enable_hand,
                debug=self.cfg.debug,
            )
            self._sampler.start()
            self._started = True

            if self.cfg.debug:
                import time as _t
                deadline = _t.perf_counter() + 2.0
                while (self._sampler.frame_count < 5
                       and _t.perf_counter() < deadline):
                    _t.sleep(0.05)
                self._probe_action_values_xrt()
            return

        static_manifest = self._absolutise(self.cfg.vrmanifest_json)
        try:
            runtime_manifest = self._generate_runtime_manifest(static_manifest)
            if self.cfg.debug:
                print(
                    f"[GR1T2GripperDevice] generated runtime manifest "
                    f"{runtime_manifest!r}."
                )
        except Exception as exc:  # noqa: BLE001
            print(
                f"[GR1T2GripperDevice] runtime manifest generation failed "
                f"({type(exc).__name__}: {exc}); using static manifest."
            )
            runtime_manifest = static_manifest

        self._sampler = SteamVRSampler(
            tracker_binding=self._tracker_binding,
            actions_json_path=self._absolutise(self.cfg.actions_json),
            rate_hz=self.cfg.sampler_rate_hz,
            enable_skeletal=False,            # PICO controllers don't emit Skeletal 2.0
            vrmanifest_path=runtime_manifest,
            stale_vrmanifest_paths=(static_manifest,)
            if runtime_manifest != static_manifest else (),
            app_key=self.cfg.app_key,
        )
        self._sampler.start()

        self._setup_action_handles()
        self._started = True

        if self.cfg.debug:
            print(
                f"[GR1T2GripperDevice] started — actions='{self.cfg.actions_json}' "
                f"binding={len(self._tracker_binding)} trackers "
                f"action_handles={len(self._action_handles)} "
                f"app_key={self.cfg.app_key!r}"
            )
            import time as _t
            deadline = _t.perf_counter() + 2.0
            while (self._sampler.frame_count < 5
                   and _t.perf_counter() < deadline):
                _t.sleep(0.05)
            self._probe_openvr_inventory()
            self._probe_action_values()

    # ── Action-based input handles ─────────────────────────────────────
    def _setup_action_handles(self) -> None:
        sampler = self._sampler
        if sampler is None or getattr(sampler, "_vr_input", None) is None:
            return
        vi = sampler._vr_input
        wanted = [
            "/actions/teleop/in/trigger_left",
            "/actions/teleop/in/trigger_right",
            "/actions/teleop/in/grip_left",
            "/actions/teleop/in/grip_right",
            "/actions/teleop/in/menu_left",
            "/actions/teleop/in/menu_right",
        ]
        for path in wanted:
            try:
                self._action_handles[path] = vi.getActionHandle(path)
            except Exception as exc:  # noqa: BLE001
                if self.cfg.debug:
                    print(
                        f"[GR1T2GripperDevice] getActionHandle({path!r}) failed: {exc}"
                    )

    def _read_analog_action(self, path: str) -> Tuple[float, bool]:
        """Return ``(value, bActive)`` for an analog action.

        9.39: ``bActive`` is now exposed.  When SteamVR has not applied a
        Personal Binding for our app, every analog/digital action stays
        at ``bActive=False`` and the value is forced to 0.  Surfacing
        ``bActive`` lets us distinguish "user is at rest" from "binding
        never reached our app" -- the latter being the gotcha that
        masquerades as the former.
        """
        handle = self._action_handles.get(path)
        if handle is None or self._sampler is None:
            return 0.0, False
        vi = getattr(self._sampler, "_vr_input", None)
        if vi is None:
            return 0.0, False
        try:
            data = vi.getAnalogActionData(handle)
        except Exception:
            return 0.0, False
        try:
            value = float(getattr(data, "x", 0.0))
        except Exception:
            value = 0.0
        try:
            active = bool(getattr(data, "bActive", False))
        except Exception:
            active = False
        return value, active

    def _read_digital_action(self, path: str) -> Tuple[bool, bool]:
        """Return ``(state, bActive)`` for a digital action.  See
        :meth:`_read_analog_action` for the rationale on ``bActive``.
        """
        handle = self._action_handles.get(path)
        if handle is None or self._sampler is None:
            return False, False
        vi = getattr(self._sampler, "_vr_input", None)
        if vi is None:
            return False, False
        try:
            data = vi.getDigitalActionData(handle)
        except Exception:
            return False, False
        try:
            state = bool(getattr(data, "bState", False))
        except Exception:
            state = False
        try:
            active = bool(getattr(data, "bActive", False))
        except Exception:
            active = False
        return state, active

    def _read_action_inputs(self) -> Dict[str, Any]:
        """Return ``{side: {trigger, grip, menu, *_active}}`` for both hands.

        Backwards compatible: callers that only consume ``trigger`` /
        ``grip`` / ``menu`` keep working (the retargeter does this).
        New ``*_active`` fields surface the per-action ``bActive`` state
        for diagnostics.  When all six ``*_active`` flags are False, the
        Personal Binding is missing for our app -- run
        ``scripts/open_binding_ui.py`` to fix it (9.39).
        """
        backend = (self.cfg.input_backend or "openvr").lower()

        # ── XRoboToolkit branch (research/47) ──────────────────────────
        if backend == "xrobotoolkit":
            # XRoboToolkit doesn't have an "active" concept — analog values
            # are always live when the controller is paired.  Pull straight
            # from the snapshot's controllers.buttons dict.
            snap = self._get_raw_data() or {}
            ctrls = snap.get("controllers") or {}
            out: Dict[str, Any] = {}
            for side in ("left", "right"):
                c = ctrls.get(side) or {}
                btn = c.get("buttons") or {}
                out[side] = {
                    "trigger": float(btn.get("trigger", 0.0)),
                    "grip":    float(btn.get("grip", 0.0)),
                    "menu":    bool(btn.get("menu", False)),
                    "trigger_active": True,
                    "grip_active":    True,
                    "menu_active":    True,
                }
            return out

        out: Dict[str, Any] = {}
        for side in ("left", "right"):
            trig_v, trig_act = self._read_analog_action(f"/actions/teleop/in/trigger_{side}")
            grip_v, grip_act = self._read_analog_action(f"/actions/teleop/in/grip_{side}")
            menu_v, menu_act = self._read_digital_action(f"/actions/teleop/in/menu_{side}")
            out[side] = {
                "trigger": trig_v,
                "grip": grip_v,
                "menu": menu_v,
                "trigger_active": trig_act,
                "grip_active": grip_act,
                "menu_active": menu_act,
            }
        return out

    def _probe_action_values(self) -> None:
        snapshot = self._read_action_inputs()
        print("[GR1T2GripperDevice] --- OpenVR action values probe ---")
        for side, vals in snapshot.items():
            # 9.39: surface bActive per channel so the user can tell
            # "binding missing" (all active=False) from "user at rest"
            # (active=True, value=0).
            print(
                f"  {side}: trigger={vals['trigger']:.3f} (active={vals['trigger_active']})  "
                f"grip={vals['grip']:.3f} (active={vals['grip_active']})  "
                f"menu={vals['menu']} (active={vals['menu_active']})"
            )
        any_nonzero = any(
            vals["trigger"] > 0.01 or vals["grip"] > 0.01 or vals["menu"]
            for vals in snapshot.values()
        )
        all_inactive = not any(
            vals["trigger_active"] or vals["grip_active"] or vals["menu_active"]
            for vals in snapshot.values()
        )
        if any_nonzero:
            print("  → action-based input is live — gripper WILL toggle when you squeeze.")
            return
        if all_inactive:
            # 9.39: definitive diagnosis.  Every action handle has
            # bActive=False -> SteamVR did not apply a Personal Binding
            # for our app (NOT a "user is at rest" case).  Hardware /
            # streaming layer can be 100% healthy and this still happens
            # if the user has never opened SteamVR > Manage Controller
            # Bindings for ust.teleop.gr1t2_gripper, or has a stale empty
            # Personal Binding from before 9.32.
            print(
                "  → ALL action handles report bActive=False.\n"
                "    DIAGNOSIS: SteamVR has NOT applied any Personal Binding for our\n"
                "    app 'UST Teleop GR1T2 Gripper'.  This is not a 'user at rest'\n"
                "    case -- the Action API is reporting that no binding currently\n"
                "    routes input to our app's actions, regardless of what you press.\n"
                "    FAST FIX (one of these):\n"
                "      A. Run the helper that opens the Binding UI focused on us:\n"
                "           python -X utf8 -m ust_ws.ust_hm_grip.scripts.open_binding_ui\n"
                "         Then in the dialog: pick 'UST Teleop GR1T2 Gripper Default',\n"
                "         click 'Save Personal Binding' at the bottom, close the dialog,\n"
                "         and re-run this diagnostic.\n"
                "      B. Manually: SteamVR > Settings > Controllers > Manage Controller\n"
                "         Bindings > 'UST Teleop GR1T2 Gripper' > Active Controller\n"
                "         Binding -> 'Custom' or 'Default', then 'Save Personal Binding'.\n"
                "    NOTE: SteamVR's own Test Controller WORKS even when our binding\n"
                "    is missing -- the test panel uses the controller driver's default,\n"
                "    not per-app Personal Bindings.  Don't be fooled by Test Controller\n"
                "    showing trigger/grip activity (memory.md §10.47)."
            )
            return
        # Mixed: some actions active, some not.  Likely a partial binding.
        src = (self.cfg.gripper_signal_source or "grip").lower()
        primary = "grip" if src in ("grip", "both") else "trigger"
        print(
            "  → some action handles bActive=True with value 0.  Either user is at\n"
            f"    rest OR the {primary} channel is the unbound one.  Squeeze the {primary}\n"
            "    now; if values stay 0 with active=True:\n"
            "      1. SteamVR > Settings > Controllers > Manage Controller Bindings\n"
            "         > select 'UST Teleop GR1T2 Gripper' app, click 'Edit Binding'.\n"
            "      2. Make sure {0}_left/{0}_right are bound to controller {0} as\n"
            "         **mode = 'Trigger', input = 'Pull'** (NOT 'Force Sensor').\n"
            "         Per memory.md §10.40 the Force Sensor mode is silent under\n"
            "         Virtual Desktop's Oculus-Touch emulation.\n"
            "      3. If a Personal Binding from before 9.32 is active, click\n"
            "         'Reset to Default' first so the new file's mode is picked up.\n"
            "      4. Click 'Save Personal Binding' at the bottom (easy to miss!).\n"
            "      5. Back in 'Manage Controller Bindings', confirm your saved\n"
            "         binding is the ACTIVE one (not a different one).".format(primary)
        )

    def _probe_action_values_xrt(self) -> None:
        """XRoboToolkit-specific diagnostic probe (analogous to
        :meth:`_probe_action_values` for OpenVR).  Verifies that the
        snapshot reports nonzero analog values when the user actuates
        the controllers.
        """
        if self._sampler is None:
            return
        snap = self._sampler.snapshot()
        ctrls = snap.get("controllers") or {}
        print("[GR1T2GripperDevice] --- XRoboToolkit channel probe ---")
        for side in ("left", "right"):
            c = ctrls.get(side) or {}
            btn = c.get("buttons") or {}
            pose = c.get("pose") or {}
            pos = pose.get("pos")
            pos_str = (
                f"({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})"
                if pos is not None else "<no pose>"
            )
            print(
                f"  {side}: trigger={btn.get('trigger', 0.0):.3f}  "
                f"grip={btn.get('grip', 0.0):.3f}  "
                f"menu={btn.get('menu', False)}  "
                f"pose={pos_str}"
            )
        any_nonzero = any(
            ((ctrls.get(side, {}) or {}).get("buttons", {}) or {}).get("trigger", 0.0) > 0.01
            or ((ctrls.get(side, {}) or {}).get("buttons", {}) or {}).get("grip", 0.0) > 0.01
            for side in ("left", "right")
        )
        if any_nonzero:
            print("  → XRoboToolkit channel LIVE — gripper will toggle when "
                  "you squeeze.")
        else:
            print(
                "  → Analog values at rest.  Squeeze grip/trigger now.\n"
                "  If values stay 0:\n"
                "    1. Verify Unity Client APK on headset has Direction=Send\n"
                "    2. Verify RoboticsServiceProcess.exe is running\n"
                "    3. Run scripts/diagnose_xrobotoolkit.py for a layered probe"
            )

    def _probe_openvr_inventory(self) -> None:
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
        print("[GR1T2GripperDevice] --- OpenVR device inventory ---")
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
                extra = (
                    f" controller_type={ctype!r} "
                    f"role={role_names.get(role, str(role))}"
                )
            print(f"  idx={i:2d} cls={cls_name:11s} serial={serial!r:30s}{extra}")

        # 9.27 → migrated: warn when a UDCAP "knuckles" controller is still
        # active even though the user is supposed to have decommissioned it.
        knuckles_serials: list[str] = []
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
        if knuckles_serials:
            print(
                "[GR1T2GripperDevice] *** UDCAP STILL RUNNING ***\n"
                f"  Detected knuckles emulator(s): {knuckles_serials}\n"
                "  This package assumes the UDCAP / Windows mini-PC stack has been\n"
                "  decommissioned.  The knuckles controllers may mask PICO Touch\n"
                "  inputs.  Stop the UdcapDriver process or unplug the gloves."
            )

    def stop(self) -> None:
        if self._sampler is not None:
            self._sampler.stop()
            self._sampler = None
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    # ── DeviceBase ─────────────────────────────────────────────────────
    def reset(self) -> None:
        self._retargeter.reset()
        self._advance_count = 0
        self._first_advance_logged = False

    def add_callback(self, key: Any, func: Any) -> None:  # pragma: no cover
        pass

    def _get_raw_data(self) -> Optional[Dict[str, Any]]:
        if self._sampler is None:
            return None
        return self._sampler.snapshot()

    def advance(self) -> Optional[torch.Tensor]:
        """Return the latest 16-D action (or ``None`` before first frame)."""
        if not self._started:
            self.start()
        snap = self._get_raw_data()
        if snap is None or snap.get("frame_count", 0) == 0:
            return None

        action_inputs = self._read_action_inputs()
        action = self._retargeter.retarget(snap, action_inputs=action_inputs)
        self._advance_count += 1

        if self.cfg.debug and not self._first_advance_logged:
            self._first_advance_logged = True
            self._log_first_advance(snap, action_inputs, action)
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

    def get_source_info(self) -> Dict[str, Any]:
        return self._retargeter.get_source_info()

    # ── Helpers ────────────────────────────────────────────────────────
    @staticmethod
    def _fmt(p: Optional[Mapping[str, Any]]) -> str:
        if p is None:
            return "None"
        pp = p["pos"]
        return f"({pp[0]:+.3f},{pp[1]:+.3f},{pp[2]:+.3f})"

    def _log_first_advance(
        self,
        snap: Mapping[str, Any],
        action_inputs: Mapping[str, Mapping[str, Any]],
        action: torch.Tensor,
    ) -> None:
        trks = snap.get("trackers") or {}
        ctrls = snap.get("controllers") or {}

        def _fmt_act(side: str) -> str:
            v = action_inputs.get(side) or {}
            return (
                f"trigger={v.get('trigger', 0):.2f} grip={v.get('grip', 0):.2f} "
                f"menu={v.get('menu', False)}"
            )

        def _fmt_ctrl(c: Optional[Mapping[str, Any]]) -> str:
            if c is None:
                return "None"
            pose = c.get("pose")
            return f"pose={self._fmt(pose)}"

        print(
            "[GR1T2GripperDevice] --- base_link-frame diagnostic ---\n"
            f"  SteamVR world: waist={self._fmt(trks.get('waist'))} "
            f"left_forearm={self._fmt(trks.get('left_forearm'))} "
            f"right_forearm={self._fmt(trks.get('right_forearm'))}\n"
            f"  L_EEF target: {[f'{v:+.3f}' for v in action[0:3].tolist()]}\n"
            f"  R_EEF target: {[f'{v:+.3f}' for v in action[7:10].tolist()]}\n"
            f"  L_gripper={float(action[14]):+.0f} R_gripper={float(action[15]):+.0f}\n"
            f"  ctrls.left : {_fmt_ctrl(ctrls.get('left'))}\n"
            f"  ctrls.right: {_fmt_ctrl(ctrls.get('right'))}\n"
            f"  --- action-API input ---\n"
            f"  left  actions: {_fmt_act('left')}\n"
            f"  right actions: {_fmt_act('right')}\n"
            f"  → Squeeze the {('grip' if (self.cfg.gripper_signal_source or 'grip').lower() in ('grip', 'both') else 'trigger')};"
            f" the next periodic log should show\n"
            f"    L_cmd / R_cmd flip from +1 to -1 above the close threshold "
            f"(source={self.cfg.gripper_signal_source!r})."
        )


class GripperInterventionInterface:
    """HG-DAgger intervention trigger using the controller MENU buttons.

    The original :class:`PICOInterventionInterface` overloaded the trigger
    and grip for intervention/resume signals.  In the gripper migration the
    trigger is reserved for gripper open/close, so we move the intervention
    semantics to the menu button:

        * Both ``menu_left`` AND ``menu_right`` pressed → toggle intervention
          (rising edge only).

    Subclasses can override :meth:`on_toggle` to wire into the env's
    intervention manager.
    """

    def __init__(
        self,
        device: GR1T2GripperDevice,
        debounce_frames: int = 3,
    ) -> None:
        self._device = device
        self._debounce_frames = max(1, int(debounce_frames))
        self._counter = 0
        self._is_intervening = False
        self._latched = False

    def update(self) -> None:
        action_inputs = self._device._read_action_inputs()  # noqa: SLF001
        both_menu = (
            bool(action_inputs.get("left", {}).get("menu"))
            and bool(action_inputs.get("right", {}).get("menu"))
        )
        if both_menu:
            self._counter += 1
            if (
                not self._latched
                and self._counter >= self._debounce_frames
            ):
                self._latched = True
                self._is_intervening = not self._is_intervening
                self.on_toggle(self._is_intervening)
        else:
            self._counter = 0
            self._latched = False

    @property
    def is_intervening(self) -> bool:
        return self._is_intervening

    def on_toggle(self, intervening: bool) -> None:
        """Override in a subclass to plug into the env's intervention manager."""
        print(f"[GripperIntervention] toggled → intervening={intervening}")
