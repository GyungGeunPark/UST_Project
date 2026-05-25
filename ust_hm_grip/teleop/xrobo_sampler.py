"""XRoboToolkit gRPC sampler — drop-in replacement for SteamVRSampler.

Background thread that polls ``xrobotoolkit_sdk`` at ``rate_hz`` and exposes
the latest snapshot under a lock.  The snapshot layout matches
:class:`SteamVRSampler` so :class:`GR1T2GripperSteamVRRetargeter` works
unchanged when this backend is selected.

Coordinate transforms:
    * Positions: XRoboToolkit world (X right, Y up, Z in) → Isaac Lab
      base-link (X forward, Y left, Z up).  See coord_transforms.xr_to_isaaclab.
    * Quaternions: xyzw → wxyz reordering, then similarity transform.

Required environment:
    * ``XRoboToolkit-PC-Service`` running on the PC (RoboticsServiceProcess.exe).
    * ``XRoboToolkit-Unity-Client`` APK active on the headset with Direction=Send.
    * ``xrobotoolkit_sdk`` Python module installed in the active interpreter.

Usage::

    sampler = XRoboSampler(rate_hz=120.0, enable_body=False)
    sampler.start()
    ...
    snap = sampler.snapshot()
    sampler.stop()
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

import numpy as np

from . import coord_transforms as ct


__all__ = ["XRoboSampler", "SAMPLER_STATE_IDLE", "SAMPLER_STATE_RUNNING"]


SAMPLER_STATE_IDLE = "idle"
SAMPLER_STATE_RUNNING = "running"


# PICO Body 24-joint indices (SMPL-compatible).  Maps to ust_hm_grip roles.
#
# 13th session — ankles INTENTIONALLY OMITTED.  User reported (live test)
# that the PICO body 24-joint stream's ankle channels drift / wobble even
# while seated, and the gripper retargeter doesn't consume ankle data
# anyway.  Excluding them keeps the snapshot's ``trackers`` dict clean
# and prevents any future role-table lookup from accidentally picking up
# noisy ankle positions.  If a later env_cfg needs them, set
# ``XRoboSampler.body_tracker_role_map`` explicitly.
_DEFAULT_BODY_ROLE_MAP: Dict[int, str] = {
    0:  "waist",          # Pelvis
    18: "left_forearm",   # LEFT_ELBOW (forearm role expects elbow position)
    19: "right_forearm",  # RIGHT_ELBOW
    # 13th session, part 11 — wrist trackers (SMPL L_Wrist, R_Wrist).
    # Surfaced so the retargeter / TRACK diagnostic can show body-skeleton
    # wrist pose alongside the controller pose.  Useful when the user
    # wants to (a) compare controller-driven wrist target against the
    # body-skeleton-estimated wrist (debug consistency), or (b) drive
    # robot wrists from the body skeleton instead of the controllers
    # (set ``prefer_controller_for_eef=False`` and ``forearm`` falls
    # back to body-skeleton; future Phase D++ can wire wrist directly).
    20: "left_wrist",     # LEFT_WRIST
    21: "right_wrist",    # RIGHT_WRIST
    # Ankles (7=LEFT_ANKLE, 8=RIGHT_ANKLE) deliberately excluded.
}


class XRoboSampler:
    """Background xrobotoolkit_sdk poller.

    Public API matches :class:`SteamVRSampler` (``start`` / ``stop`` /
    ``snapshot`` / ``frame_count`` / ``is_running``) so the device class
    can swap backends without conditional logic on the sampler side.
    """

    def __init__(
        self,
        rate_hz: float = 120.0,
        enable_body: bool = False,
        enable_hand: bool = False,
        body_tracker_role_map: Optional[Dict[int, str]] = None,
        debug: bool = False,
    ) -> None:
        self._rate_hz = max(10.0, float(rate_hz))
        self._dt_target = 1.0 / self._rate_hz
        self._enable_body = bool(enable_body)
        self._enable_hand = bool(enable_hand)
        self._body_role_map = dict(body_tracker_role_map or _DEFAULT_BODY_ROLE_MAP)
        self._debug = bool(debug)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._state = SAMPLER_STATE_IDLE
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0
        self._latest: Dict[str, Any] = self._empty_snapshot()

        self._xrt = None  # populated on start()

    # ── lifecycle ──────────────────────────────────────────────────────

    @staticmethod
    def _empty_snapshot() -> Dict[str, Any]:
        return {
            "timestamp": 0.0,
            "hmd": None,
            "trackers": {},
            "hands": {"left": None, "right": None},
            "controllers": {"left": None, "right": None},
            "frame_count": 0,
        }

    def start(self) -> None:
        if self._state == SAMPLER_STATE_RUNNING:
            return
        try:
            import xrobotoolkit_sdk as xrt  # noqa: WPS433
        except ImportError as exc:
            raise RuntimeError(
                "xrobotoolkit_sdk is not installed.  Build the binding:\n"
                "  cd C:\\develop\\IsaacLab\\ust_ws\\XRoboToolkit-PC-Service-Pybind\n"
                "  python -m pip install pybind11 numpy\n"
                "  python setup.py install\n"
                "and make sure XRoboToolkit-PC-Service is installed too."
            ) from exc
        self._xrt = xrt

        try:
            xrt.init()
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"xrt.init() failed: {type(exc).__name__}: {exc}\n"
                "  Likely causes:\n"
                "    1. RoboticsServiceProcess.exe is not running.  Start it:\n"
                "       & 'C:\\develop\\IsaacLab\\ust_ws\\XRoboToolkit-PC-Service.win\\runService.bat'\n"
                "    2. PICO Connect is holding the PICO USB.  Kill it:\n"
                "       taskkill /IM \"Pico Connect.exe\" /F\n"
                "    3. setting.ini port is blocked by Windows Firewall.\n"
                "    Run scripts/diagnose_xrobotoolkit.py for a layered probe."
            ) from exc

        if self._debug:
            print("[XRoboSampler] xrt.init() OK; spawning poll thread "
                  f"at {self._rate_hz:.1f} Hz.")
            # 13th session — probe which XR data channels are populated
            # so users immediately see whether HMD / body / PMT toggles
            # are ON in the Unity Client APK.  Each unanswered channel
            # is a tracking source that won't reach the retargeter.
            try:
                hp = np.asarray(xrt.get_headset_pose(), dtype=np.float64)
                hmd_live = not bool(np.allclose(hp, 0.0, atol=1e-6))
                body_avail = bool(xrt.is_body_data_available())
                n_pmt = int(xrt.num_motion_data_available())
                serials = list(xrt.get_motion_tracker_serial_numbers() or [])
                # 13th session, part 15 — body_avail True means Unity APK
                # is in "Full body tracking" mode (PICO Motion Tracker
                # → Mode = Full body).  n_pmt > 0 means APK is in "Object"
                # mode (raw PMT poses, max 3).  These modes are mutually
                # exclusive at the APK level; "None" = neither.
                if body_avail and n_pmt == 0:
                    apk_mode = "Full body tracking (SMPL 24-joint AI fusion ON)"
                elif n_pmt > 0 and not body_avail:
                    apk_mode = (
                        f"Object mode (raw PMT only, n={n_pmt}); "
                        f"to get SMPL fusion switch APK → 'Full body tracking'"
                    )
                elif body_avail and n_pmt > 0:
                    apk_mode = "BOTH (unusual — APK build streams both)"
                else:
                    apk_mode = (
                        "None — body data inactive.  Set APK Tracking → "
                        "PICO Motion Tracker → Mode = 'Full body tracking', "
                        "and ensure ≥2 PMTs paired + calibrated via PICO "
                        "Motion Tracker app (PUI ≥ 5.11.0)."
                    )
                print(
                    "[XRoboSampler] channel probe:\n"
                    f"  HMD pose:                   {'LIVE' if hmd_live else 'ZERO (wear headset + APK Head toggle ON)'}\n"
                    f"  Body skeleton (24-joint):   {'AVAILABLE' if body_avail else 'OFF (APK Body toggle = ON to enable)'}\n"
                    f"  Independent PMT (Motion Tracker): n={n_pmt}, serials={serials or '[]'}\n"
                    f"  APK tracking mode:          {apk_mode}\n"
                    f"  Body role map (enable_body={self._enable_body}): {self._body_role_map if self._enable_body else '(disabled — pass --xrt_enable_body True)'}"
                )
            except Exception as exc:  # noqa: BLE001
                print(f"[XRoboSampler] channel probe failed: {type(exc).__name__}: {exc}")

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name="XRoboSampler", daemon=True,
        )
        self._thread.start()
        self._state = SAMPLER_STATE_RUNNING

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._xrt is not None:
            try:
                self._xrt.close()
            except Exception:  # noqa: BLE001
                pass
            self._xrt = None
        self._state = SAMPLER_STATE_IDLE

    def __enter__(self) -> "XRoboSampler":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # ── access ─────────────────────────────────────────────────────────

    def snapshot(self) -> Dict[str, Any]:
        """Return a shallow copy of the latest snapshot.  Thread-safe."""
        with self._lock:
            return {
                "timestamp":   self._latest["timestamp"],
                "hmd":         self._latest["hmd"],
                "trackers":    dict(self._latest["trackers"]),
                "hands":       dict(self._latest["hands"]),
                "controllers": dict(self._latest["controllers"]),
                "frame_count": self._latest["frame_count"],
            }

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def is_running(self) -> bool:
        return self._state == SAMPLER_STATE_RUNNING

    # ── internal: pose helpers ─────────────────────────────────────────

    @staticmethod
    def _is_zero_pose(pose_xyzw: Any) -> bool:
        """Heuristic: XR SDK returns near-zero arrays when data not ready."""
        if pose_xyzw is None:
            return True
        arr = np.asarray(pose_xyzw, dtype=np.float64).reshape(-1)
        if arr.size < 7:
            return True
        p = arr[:3]
        q = arr[3:7]
        if np.any(np.isnan(p)) or np.any(np.isnan(q)):
            return True
        return bool(np.allclose(p, 0.0, atol=1e-6) and np.allclose(q, 0.0, atol=1e-6))

    def _pose_to_pq(self, pose_xyzw: Any) -> Dict[str, np.ndarray]:
        """Convert XR (7,) [x,y,z,qx,qy,qz,qw] → Isaac Lab {"pos","quat" wxyz}."""
        arr = np.asarray(pose_xyzw, dtype=np.float64).reshape(-1)
        pos_il, quat_il_wxyz = ct.xr_to_isaaclab(arr[:3], arr[3:7])
        return {"pos": pos_il, "quat": quat_il_wxyz}

    # ── internal: per-channel readers ──────────────────────────────────

    def _read_menu_button(self, side: str) -> bool:
        """Read PICO menu (system) button.  Returns False if the SDK build
        predates the get_*_menu_button API (graceful fallback)."""
        xrt = self._xrt
        if xrt is None:
            return False
        fn_name = f"get_{side}_menu_button"
        fn = getattr(xrt, fn_name, None)
        if fn is None:
            return False
        try:
            return bool(fn())
        except Exception:  # noqa: BLE001
            return False

    def _read_face_button(self, name: str) -> bool:
        """Read PICO face button by name ('A', 'B', 'X', 'Y').

        PICO Touch convention:
          * A, B = right controller (lower, upper)
          * X, Y = left controller (lower, upper)

        Graceful fallback if the SDK build doesn't expose the API.
        """
        xrt = self._xrt
        if xrt is None:
            return False
        fn = getattr(xrt, f"get_{name}_button", None)
        if fn is None:
            return False
        try:
            return bool(fn())
        except Exception:  # noqa: BLE001
            return False

    def _read_hmd(self) -> Optional[Dict[str, np.ndarray]]:
        try:
            pose = np.asarray(self._xrt.get_headset_pose(), dtype=np.float64)
        except Exception:  # noqa: BLE001
            return None
        if self._is_zero_pose(pose):
            return None
        return self._pose_to_pq(pose)

    def _read_controller(self, side: str) -> Optional[Dict[str, Any]]:
        xrt = self._xrt
        try:
            if side == "left":
                pose = np.asarray(xrt.get_left_controller_pose(), dtype=np.float64)
                trigger = float(xrt.get_left_trigger())
                grip = float(xrt.get_left_grip())
                menu_btn = self._read_menu_button("left")
                # PICO convention: left = X (lower), Y (upper)
                face_low = self._read_face_button("X")
                face_high = self._read_face_button("Y")
            elif side == "right":
                pose = np.asarray(xrt.get_right_controller_pose(), dtype=np.float64)
                trigger = float(xrt.get_right_trigger())
                grip = float(xrt.get_right_grip())
                menu_btn = self._read_menu_button("right")
                # PICO convention: right = A (lower), B (upper)
                face_low = self._read_face_button("A")
                face_high = self._read_face_button("B")
            else:
                return None
        except Exception:  # noqa: BLE001
            return None

        if self._is_zero_pose(pose):
            # Controller is on but pose stream not yet populated; still
            # surface analog values so the gripper can be commanded even
            # without spatial tracking (e.g. cold-start race condition).
            pose_pq = {
                "pos":  np.array([0.0, 0.0, 0.0], dtype=np.float64),
                "quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            }
        else:
            pose_pq = self._pose_to_pq(pose)

        # 13th-bis session — face buttons (A/B on right, X/Y on left).
        # Used for runtime re-calibration trigger (`A` press = re-zero
        # head/waist/wrist) and reserved for future intervention hooks.
        button_low_name  = "a" if side == "right" else "x"
        button_high_name = "b" if side == "right" else "y"
        return {
            "pose": pose_pq,
            "buttons": {
                "trigger": max(0.0, min(1.0, trigger)),
                "grip":    max(0.0, min(1.0, grip)),
                "menu":    bool(menu_btn),
                button_low_name:  bool(face_low),
                button_high_name: bool(face_high),
                "raw_buttons": 0,
            },
        }

    def _read_body_joints(self) -> Dict[str, Dict[str, np.ndarray]]:
        """Map PICO body 24-joint poses → ust_hm_grip role-keyed trackers."""
        out: Dict[str, Dict[str, np.ndarray]] = {}
        if not self._enable_body or not self._body_role_map:
            return out
        xrt = self._xrt
        try:
            if not bool(xrt.is_body_data_available()):
                return out
            poses = np.asarray(xrt.get_body_joints_pose(), dtype=np.float64)
        except Exception:  # noqa: BLE001
            return out
        if poses.ndim != 2 or poses.shape[1] != 7:
            return out
        for idx, role in self._body_role_map.items():
            if 0 <= idx < poses.shape[0]:
                pose_xyzw = poses[idx]
                if self._is_zero_pose(pose_xyzw):
                    continue
                out[role] = self._pose_to_pq(pose_xyzw)
        return out

    # ── main poll loop ─────────────────────────────────────────────────

    def _loop(self) -> None:
        while not self._stop.is_set():
            t_start = time.perf_counter()
            try:
                snap_hmd      = self._read_hmd()
                snap_left     = self._read_controller("left")
                snap_right    = self._read_controller("right")
                snap_trackers = self._read_body_joints()
            except Exception:  # noqa: BLE001
                # Keep the thread alive even on transient errors.
                self._stop.wait(0.05)
                continue

            self._frame_count += 1
            with self._lock:
                self._latest = {
                    "timestamp":   time.perf_counter(),
                    "hmd":         snap_hmd,
                    "trackers":    snap_trackers,
                    "hands":       {"left": None, "right": None},
                    "controllers": {"left": snap_left, "right": snap_right},
                    "frame_count": self._frame_count,
                }

            elapsed = time.perf_counter() - t_start
            sleep_left = self._dt_target - elapsed
            if sleep_left > 0:
                # Use Event.wait so stop() can break out of long sleeps
                self._stop.wait(sleep_left)
