"""SteamVR in-process sampler (pyopenvr).

Reference: `research/28. SteamVR …` §4.1.  Runs a background thread at
``rate_hz`` that enumerates tracked devices, queries HMD + tracker poses and
both-hand skeletal data, and exposes the latest snapshot under a lock.

The snapshot layout is::

    {
        "timestamp":   float (perf_counter seconds),
        "hmd":         {"pos": (3,), "quat": (4, wxyz)} | None,
        "trackers":    {role: {"pos": (3,), "quat": (4, wxyz)}},   # serial-bound
        "hands":       {
            "left":  {"bones": (31, 7), "fingerCurls": (5,), "fingerSplays": (4,)} | None,
            "right": …,
        },
        "controllers": {"left": {...}, "right": {...}} | None,   # button state
    }

This module is used from ``pico_udcap_device.py``.  It requires the
``openvr`` (``pyopenvr``) pip package; importing is only enforced when the
sampler is instantiated.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Mapping, Optional

import numpy as np


__all__ = ["SteamVRSampler", "SteamVRSnapshot", "SAMPLER_STATE_IDLE", "SAMPLER_STATE_RUNNING"]


SAMPLER_STATE_IDLE = "idle"
SAMPLER_STATE_RUNNING = "running"

# 9.38 — openvr.init() has no built-in timeout; if SteamVR is not running
# (or PICO Connect / Steam Link / VD has not yet exposed the HMD provider),
# the call blocks indefinitely with no diagnostic.  Wrap it in a watchdog
# thread so the failure mode is actionable.  See memory.md §10.45.
_DEFAULT_OPENVR_INIT_TIMEOUT_SEC = 30.0


def _init_openvr_with_timeout(timeout_sec: float = _DEFAULT_OPENVR_INIT_TIMEOUT_SEC):
    """Call ``openvr.init(VRApplication_Other)`` with a watchdog timeout.

    On success returns the IVRSystem handle.  On timeout raises
    ``TimeoutError`` with the most likely root causes listed.  Any
    exception raised inside ``openvr.init`` is re-raised on the caller
    thread verbatim.

    The worker is a daemon thread — if ``openvr.init`` is genuinely
    stuck in native code it cannot be cancelled, but the daemon flag
    ensures it dies with the process rather than blocking shutdown.
    """
    import openvr  # noqa: WPS433 — runtime import isolates optional dep

    result: Dict[str, Any] = {"system": None, "exc": None}

    def _target() -> None:
        try:
            result["system"] = openvr.init(openvr.VRApplication_Other)
        except BaseException as exc:  # noqa: BLE001 — propagate everything
            result["exc"] = exc

    t = threading.Thread(target=_target, name="openvr-init-watchdog", daemon=True)
    t.start()
    t.join(timeout=float(timeout_sec))

    if t.is_alive():
        raise TimeoutError(
            f"openvr.init(VRApplication_Other) blocked for >"
            f"{timeout_sec:.0f}s without responding.\n"
            f"Most likely causes (priority order):\n"
            f"  1) SteamVR (vrserver.exe) is not running -- start SteamVR "
            f"first (Steam > Library > SteamVR > Play, or PICO Connect "
            f"will auto-launch it when the HMD streams).\n"
            f"  2) PICO Connect (or Steam Link / Virtual Desktop) is not "
            f"streaming the HMD -- SteamVR is waiting for the HMD provider "
            f"to initialize.  Put the headset on and start streaming.\n"
            f"  3) SteamVR is in a 'Starting up...' dialog awaiting user "
            f"input -- click through it.\n"
            f"  4) Multiple HMD drivers ON simultaneously (pico + prism + "
            f"VD) -- HMD double-emit conflict; enable EXACTLY ONE in "
            f"SteamVR > Settings > Manage Add-Ons.  See gotcha #29.\n"
            f"\n"
            f"Run the 6-layer probe for a definitive diagnosis:\n"
            f"  python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect"
        )

    if result["exc"] is not None:
        raise result["exc"]

    return result["system"]


@dataclass
class SteamVRSnapshot:
    """Typed view over the raw snapshot dict (optional convenience wrapper)."""

    timestamp: float = 0.0
    hmd: Optional[Dict[str, np.ndarray]] = None
    trackers: Dict[str, Dict[str, np.ndarray]] = field(default_factory=dict)
    hands: Dict[str, Optional[Dict[str, np.ndarray]]] = field(
        default_factory=lambda: {"left": None, "right": None}
    )
    controllers: Dict[str, Optional[Dict[str, Any]]] = field(
        default_factory=lambda: {"left": None, "right": None}
    )
    # Diagnostic counters
    frame_count: int = 0

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "SteamVRSnapshot":
        return cls(
            timestamp=float(d.get("timestamp", 0.0)),
            hmd=d.get("hmd"),
            trackers=dict(d.get("trackers", {})),
            hands=dict(d.get("hands", {"left": None, "right": None})),
            controllers=dict(d.get("controllers", {"left": None, "right": None})),
            frame_count=int(d.get("frame_count", 0)),
        )


def _pose_matrix_to_pq(m: Any) -> Dict[str, np.ndarray]:
    """Convert an OpenVR ``HmdMatrix34_t`` to ``{"pos": (3,), "quat": (4,) wxyz}``."""
    # m.m is a 3x4 C array; extract position column 3, rotation cols 0..2.
    r = np.array(
        [[m.m[i][j] for j in range(3)] for i in range(3)],
        dtype=np.float64,
    )
    pos = np.array([m.m[0][3], m.m[1][3], m.m[2][3]], dtype=np.float64)
    # Rotation matrix -> wxyz quaternion
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    q /= max(float(np.linalg.norm(q)), 1e-12)
    return {"pos": pos, "quat": q}


class SteamVRSampler:
    """Background pyopenvr sampler thread.

    The thread connects with ``VRApplication_Other`` so it does not fight
    Isaac Sim's own OpenXR runtime (§8.3 of doc 28).

    Parameters
    ----------
    tracker_binding:
        Mapping ``serial -> {"role": str, "steamvr_role": str}`` loaded from
        ``tracker_binding.json``.  Only devices whose serial is in this map
        are exposed under ``snapshot()["trackers"][role]``.
    actions_json_path:
        Absolute path to the OpenVR action manifest (``actions.json``).
    rate_hz:
        Target sampling rate.  Actual rate is clamped by the time spent in
        the pyopenvr calls; 120 Hz is the design point.
    enable_skeletal:
        When False the hand dictionary is never populated.  Useful for
        tracker-only diagnostics where the UDCAP driver is not yet loaded.
    """

    BONES_PER_HAND = 31  # SteamVR Skeletal 2.0

    def __init__(
        self,
        tracker_binding: Mapping[str, Mapping[str, str]],
        actions_json_path: str,
        rate_hz: float = 120.0,
        enable_skeletal: bool = True,
        action_set: str = "/actions/teleop",
        skeleton_actions: Mapping[str, str] = (
            ("left", "/actions/teleop/in/skeleton_left"),
            ("right", "/actions/teleop/in/skeleton_right"),
        ),
        vrmanifest_path: Optional[str] = None,
        app_key: Optional[str] = None,
        stale_vrmanifest_paths: Iterable[str] = (),
        init_timeout_sec: float = _DEFAULT_OPENVR_INIT_TIMEOUT_SEC,
    ):
        self._serial_to_role: Dict[str, str] = {
            sn: info.get("role", sn) for sn, info in tracker_binding.items()
        }
        self._actions_json_path = os.path.abspath(actions_json_path)
        self._vrmanifest_path = (
            os.path.abspath(vrmanifest_path) if vrmanifest_path else None
        )
        # Paths SteamVR may have cached from a previous (broken) run that
        # share our app_key.  We purge these via removeApplicationManifest
        # before re-adding so the cleanup is automatic.
        self._stale_vrmanifest_paths = tuple(
            os.path.abspath(p) for p in stale_vrmanifest_paths
        )
        self._app_key = app_key
        self._rate_hz = max(10.0, float(rate_hz))
        self._dt_target = 1.0 / self._rate_hz
        self._enable_skeletal = bool(enable_skeletal)
        self._action_set_name = action_set
        self._skeleton_actions = dict(skeleton_actions)
        # Watchdog timeout for openvr.init().  See _init_openvr_with_timeout
        # for the failure-mode rationale.  9.38 / memory.md §10.45.
        self._init_timeout_sec = max(1.0, float(init_timeout_sec))

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._state = SAMPLER_STATE_IDLE
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0
        self._latest: Dict[str, Any] = {
            "timestamp": 0.0,
            "hmd": None,
            "trackers": {},
            "hands": {"left": None, "right": None},
            "controllers": {"left": None, "right": None},
            "frame_count": 0,
        }

        self._openvr = None  # populated on start()
        self._vr_system = None
        self._vr_input = None
        self._h_action_set = None
        self._h_skeletons: Dict[str, int] = {}

    # ── lifecycle ──────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._state == SAMPLER_STATE_RUNNING:
            return
        import openvr  # noqa: WPS433 — runtime import isolates optional dep

        self._openvr = openvr
        # 9.38 — wrap openvr.init in a watchdog thread so a missing
        # SteamVR / unstreamed HMD surfaces as a TimeoutError with
        # actionable hints instead of a silent infinite hang.
        print(
            f"[SteamVRSampler] connecting to SteamVR via openvr.init "
            f"(VRApplication_Other, timeout={self._init_timeout_sec:.0f}s) ..."
        )
        _t0 = time.perf_counter()
        self._vr_system = _init_openvr_with_timeout(self._init_timeout_sec)
        print(
            f"[SteamVRSampler] openvr.init OK in {time.perf_counter() - _t0:.2f}s."
        )

        # ── Application registration (CRITICAL for action bindings) ──────
        # Without this block SteamVR refuses to honour ``default_bindings``
        # in actions.json — the app is anonymous ``python.exe`` to it,
        # never appears in Settings → Controllers → Manage Controller
        # Bindings, and ``getSkeletalActionData`` returns ``bActive=False``
        # because the binding-resolver IPC has no app to look up.  See
        # memory.md §10.10 (9th-fix).
        #
        # The openvr-python binding RAISES on error (returns ``None`` on
        # success) — we have to swallow ``ApplicationError_AppKeyAlready
        # Exists`` (idempotent re-registration) and let everything else
        # surface as a print so the failure mode is visible.
        if self._vrmanifest_path and self._app_key:
            try:
                apps = openvr.VRApplications()
            except Exception as exc:  # noqa: BLE001
                apps = None
                print(
                    f"[SteamVRSampler] VRApplications() failed: "
                    f"{type(exc).__name__}: {exc}."
                )
            if apps is not None:
                # ``openvr-python`` raises a typed subclass of
                # ``OpenVRError`` for each error code; ``AppKeyAlreadyExists``
                # is the idempotent re-registration path and lives on the
                # nested ``error_code`` submodule.
                from openvr import error_code as _ec  # noqa: WPS433

                # Purge any stale manifest registrations from prior runs
                # (e.g. the static template that had a bad binary path
                # which got silently dropped — its app_key still squats in
                # SteamVR's registry and conflicts with our new runtime
                # manifest).  removeApplicationManifest is a no-op if the
                # path is unknown.
                for stale in self._stale_vrmanifest_paths:
                    try:
                        apps.removeApplicationManifest(stale)
                        print(
                            f"[SteamVRSampler] purged stale manifest "
                            f"registration {stale!r}."
                        )
                    except Exception:  # noqa: BLE001
                        pass  # Wasn't registered — nothing to clean up.

                def _add_manifest(temporary: bool, label: str) -> bool:
                    """Wrap addApplicationManifest with idempotent error handling."""
                    try:
                        apps.addApplicationManifest(self._vrmanifest_path, temporary)
                        print(
                            f"[SteamVRSampler] {label} manifest registration OK "
                            f"({self._vrmanifest_path!r}, app_key={self._app_key!r})."
                        )
                        return True
                    except _ec.ApplicationError_AppKeyAlreadyExists:
                        return True  # Already loaded in this scope — fine.
                    except _ec.OpenVRError as exc:
                        print(
                            f"[SteamVRSampler] {label} addApplicationManifest "
                            f"failed: {type(exc).__name__}: {exc}."
                        )
                        return False
                    except Exception as exc:  # noqa: BLE001
                        print(
                            f"[SteamVRSampler] {label} addApplicationManifest "
                            f"raised {type(exc).__name__}: {exc}."
                        )
                        return False

                # CRITICAL: SteamVR's ``bTemporary=False`` registers the
                # manifest in the persistent registry but DOES NOT load the
                # applications into the running session — that only happens
                # on the next SteamVR restart.  Calling ``identifyApplication``
                # immediately after therefore fails with
                # ``ApplicationError_UnknownApplication`` (memory.md §10.10).
                #
                # The fix is two-step:
                #   1. ``temporary=False`` — persist for future SteamVR
                #      sessions so the binding stays editable in the
                #      Manage Controller Bindings UI long-term.
                #   2. ``temporary=True``  — load into the CURRENT session
                #      so ``identifyApplication`` and binding resolution
                #      succeed without requiring a SteamVR restart.
                #
                # Both calls are idempotent (AppKeyAlreadyExists is silently
                # absorbed).  We require the *temporary* one to succeed for
                # the current launch to function; the persistent one is
                # best-effort.
                _add_manifest(False, "persistent")
                if not _add_manifest(True, "current-session"):
                    print(
                        "[SteamVRSampler] could not load manifest into the "
                        "running SteamVR session; identifyApplication will "
                        "almost certainly fail and bindings will not apply."
                    )

                # Diagnostic: confirm SteamVR actually accepted our
                # application entry (it can silently drop the entry if the
                # binary_path_windows is unresolvable, etc.).  The presence
                # of our app_key in this list — and a True from
                # ``isApplicationInstalled`` — is the definitive signal
                # that addApplicationManifest worked.
                self._dump_app_registry(apps)

                # identifyApplication tells SteamVR which registered app
                # this PID corresponds to.  Required every launch — it is
                # NOT persistent across runs.
                try:
                    apps.identifyApplication(int(os.getpid()), self._app_key)
                    print(
                        f"[SteamVRSampler] identifyApplication OK "
                        f"(pid={os.getpid()}, app_key={self._app_key!r})."
                    )
                except Exception as exc:  # noqa: BLE001
                    print(
                        f"[SteamVRSampler] identifyApplication(pid={os.getpid()}, "
                        f"app_key={self._app_key!r}) raised "
                        f"{type(exc).__name__}: {exc}."
                    )

        self._vr_input = openvr.VRInput()
        self._vr_input.setActionManifestPath(self._actions_json_path)
        self._h_action_set = self._vr_input.getActionSetHandle(self._action_set_name)
        if self._enable_skeletal:
            for side, path in self._skeleton_actions.items():
                self._h_skeletons[side] = self._vr_input.getActionHandle(path)

        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="SteamVRSampler", daemon=True)
        self._thread.start()
        self._state = SAMPLER_STATE_RUNNING

    def _dump_app_registry(self, apps) -> None:
        """Print the SteamVR-known applications list, marking our app_key.

        Useful when ``addApplicationManifest`` returns success but
        ``identifyApplication`` then fails with ``UnknownApplication`` —
        in that case the list will NOT contain our app_key, proving that
        SteamVR silently rejected the application entry inside the
        manifest (typically because ``binary_path_windows`` could not be
        resolved relative to the manifest directory).
        """
        if self._app_key is None:
            return
        try:
            count = int(apps.getApplicationCount())
        except Exception as exc:  # noqa: BLE001
            print(f"[SteamVRSampler] getApplicationCount failed: {exc}")
            return
        print(f"[SteamVRSampler] --- SteamVR app registry ({count} apps) ---")
        found = False
        related: list[str] = []
        for i in range(count):
            try:
                key_raw = apps.getApplicationKeyByIndex(i)
            except Exception:
                continue
            key = (
                key_raw.decode("utf-8")
                if isinstance(key_raw, (bytes, bytearray))
                else str(key_raw)
            )
            if key == self._app_key:
                found = True
                print(f"  [{i:3d}] {key}   <-- ours")
            elif "ust" in key.lower() or "fourier" in key.lower() or "teleop" in key.lower():
                related.append(f"  [{i:3d}] {key}   <-- related?")
        for line in related:
            print(line)
        if found:
            print(f"  → app_key {self._app_key!r} IS registered. identifyApplication should succeed.")
        else:
            print(
                f"  → app_key {self._app_key!r} is NOT in the registry.\n"
                "    addApplicationManifest accepted the file but rejected the\n"
                "    application entry inside.  Most common cause: invalid\n"
                "    binary_path_windows.  Check vrserver.txt for a line like\n"
                "    'binary_path ... is invalid. Skipping'."
            )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._openvr is not None and self._vr_system is not None:
            try:
                self._openvr.shutdown()
            except Exception:
                pass
        self._state = SAMPLER_STATE_IDLE

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    # ── access ─────────────────────────────────────────────────────────────

    def snapshot(self) -> Dict[str, Any]:
        """Return a shallow copy of the latest snapshot.  Thread-safe."""
        with self._lock:
            return {
                "timestamp": self._latest["timestamp"],
                "hmd": self._latest["hmd"],
                "trackers": dict(self._latest["trackers"]),
                "hands": dict(self._latest["hands"]),
                "controllers": dict(self._latest["controllers"]),
                "frame_count": self._latest["frame_count"],
            }

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def is_running(self) -> bool:
        return self._state == SAMPLER_STATE_RUNNING

    # ── internals ──────────────────────────────────────────────────────────

    def _enumerate_devices(self):
        ovr = self._openvr
        k_max = ovr.k_unMaxTrackedDeviceCount
        poses = self._vr_system.getDeviceToAbsoluteTrackingPose(
            ovr.TrackingUniverseStanding, 0, k_max
        )
        device_info = []  # list[(idx, cls, serial, pose_valid, pose)]
        for i in range(k_max):
            cls = self._vr_system.getTrackedDeviceClass(i)
            if cls == ovr.TrackedDeviceClass_Invalid:
                continue
            try:
                serial = self._vr_system.getStringTrackedDeviceProperty(
                    i, ovr.Prop_SerialNumber_String
                )
            except Exception:
                serial = ""
            pose = poses[i]
            device_info.append((i, cls, serial, bool(pose.bPoseIsValid), pose))
        return device_info

    def _read_controller_buttons(self, device_idx: int) -> Dict[str, Any]:
        ovr = self._openvr
        try:
            state = self._vr_system.getControllerState(device_idx)
        except Exception:
            return {"trigger": 0.0, "grip": 0.0, "menu": False, "raw": None}
        # state is a VRControllerState_t; button fields are bitmasks.
        buttons = getattr(state, "ulButtonPressed", 0)
        axes = getattr(state, "rAxis", None)
        trigger = 0.0
        grip = 0.0
        if axes is not None:
            # Axis 1 is the trigger on most controllers; axis 2 grip.
            try:
                trigger = float(axes[1].x)
                grip = float(axes[2].x)
            except Exception:
                pass
        menu_mask = 1 << ovr.k_EButton_ApplicationMenu
        return {
            "trigger": trigger,
            "grip": grip,
            "menu": bool(buttons & menu_mask),
            "raw_buttons": int(buttons),
        }

    def _read_hand(self, side: str):
        """Return {"bones", "fingerCurls", "fingerSplays"} or None."""
        if not self._enable_skeletal or side not in self._h_skeletons:
            return None
        ovr = self._openvr
        try:
            bones = self._vr_input.getSkeletalBoneData(
                self._h_skeletons[side],
                ovr.VRSkeletalTransformSpace_Model,
                ovr.VRSkeletalMotionRange_WithoutController,
            )
            summary = self._vr_input.getSkeletalSummaryData(
                self._h_skeletons[side],
                ovr.VRSummaryType_FromAnimation,
            )
        except Exception:
            return None

        rows = []
        for b in bones:
            p = b.position
            q = b.orientation
            rows.append(
                (
                    float(p.v[0]), float(p.v[1]), float(p.v[2]),
                    float(q.w), float(q.x), float(q.y), float(q.z),  # wxyz
                )
            )
        arr = np.array(rows, dtype=np.float32)
        if arr.shape[0] < self.BONES_PER_HAND:
            pad = np.zeros((self.BONES_PER_HAND - arr.shape[0], 7), dtype=np.float32)
            pad[:, 3] = 1.0  # identity wxyz
            arr = np.vstack([arr, pad])
        return {
            "bones": arr[: self.BONES_PER_HAND],
            "fingerCurls": np.array(list(summary.flFingerCurl), dtype=np.float32),
            "fingerSplays": np.array(list(summary.flFingerSplay), dtype=np.float32),
        }

    def _loop(self) -> None:
        ovr = self._openvr
        while not self._stop.is_set():
            t_start = time.perf_counter()
            try:
                devices = self._enumerate_devices()
            except Exception:
                time.sleep(0.05)
                continue

            snap_hmd = None
            snap_trackers: Dict[str, Dict[str, np.ndarray]] = {}
            snap_controllers: Dict[str, Optional[Dict[str, Any]]] = {"left": None, "right": None}

            for i, cls, serial, valid, pose in devices:
                if not valid:
                    continue
                if cls == ovr.TrackedDeviceClass_HMD:
                    snap_hmd = _pose_matrix_to_pq(pose.mDeviceToAbsoluteTracking)
                elif cls == ovr.TrackedDeviceClass_GenericTracker:
                    role = self._serial_to_role.get(serial)
                    if role:
                        snap_trackers[role] = _pose_matrix_to_pq(
                            pose.mDeviceToAbsoluteTracking
                        )
                elif cls == ovr.TrackedDeviceClass_Controller:
                    hand_role = self._vr_system.getControllerRoleForTrackedDeviceIndex(i)
                    side = None
                    if hand_role == ovr.TrackedControllerRole_LeftHand:
                        side = "left"
                    elif hand_role == ovr.TrackedControllerRole_RightHand:
                        side = "right"
                    if side is not None:
                        pose_pq = _pose_matrix_to_pq(pose.mDeviceToAbsoluteTracking)
                        buttons = self._read_controller_buttons(i)
                        snap_controllers[side] = {
                            "pose": pose_pq,
                            "buttons": buttons,
                        }

            snap_hands = {"left": None, "right": None}
            if self._enable_skeletal and self._h_skeletons:
                try:
                    self._vr_input.updateActionState(
                        [
                            ovr.VRActiveActionSet_t(
                                ulActionSet=self._h_action_set,
                                ulRestrictedToDevice=ovr.k_ulInvalidInputValueHandle,
                            )
                        ]
                    )
                    for side in self._h_skeletons:
                        snap_hands[side] = self._read_hand(side)
                except Exception:
                    pass

            self._frame_count += 1
            with self._lock:
                self._latest = {
                    "timestamp": time.perf_counter(),
                    "hmd": snap_hmd,
                    "trackers": snap_trackers,
                    "hands": snap_hands,
                    "controllers": snap_controllers,
                    "frame_count": self._frame_count,
                }

            elapsed = time.perf_counter() - t_start
            sleep_left = self._dt_target - elapsed
            if sleep_left > 0:
                # Short sleep preserves 120 Hz; clamp in case of clock jitter.
                time.sleep(min(sleep_left, self._dt_target))
