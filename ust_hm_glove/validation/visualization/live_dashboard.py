"""rerun.io live dashboard for finger-precision telemetry.

Logs every frame's :class:`FourierHandMapper` 11D output, packed 22D
output, robot joint targets/actuals (when available), and per-bone VMC
quaternion magnitude — all on a single time axis so the user can scrub
through and see exactly where the precision is lost.

The dashboard is *purely additive* — call :meth:`FingerDashboard.push_frame`
once per simulation step from any of Layer 2/3/4 entry points.

Graceful fallback when ``rerun-sdk`` is not installed: every method
becomes a no-op and prints a one-time warning, so the host script does
not need to wrap each call in a try/except.

Usage::

    from ust_ws.ust_hm_glove.validation.visualization.live_dashboard import FingerDashboard
    dash = FingerDashboard(spawn=True)        # opens rerun viewer
    while running:
        dash.push_frame(t, vmc_bones, left_11, right_11, packed_22,
                        target_22=target, actual_22=actual)

Architecture::

    finger/L_idx_prox/target,actual         scalar series
    finger/.../...
    vmc/LeftIndexProximal/bend_rad          scalar series (per-bone)
    delta/L_idx_prox/Δrange                 scalar series (rolling max - min)
    summary/source_left, summary/source_right   text logs
"""

from __future__ import annotations

import math
import sys
import warnings
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np


_PER_SIDE_LABELS = (
    "index_prox", "middle_prox", "pinky_prox", "ring_prox", "thumb_yaw",
    "index_int",  "middle_int",  "pinky_int",  "ring_int",  "thumb_pitch",
    "thumb_dist",
)

_PACKED_22_LABELS = (
    "L_idx_prox", "L_mid_prox", "L_pky_prox", "L_rng_prox", "L_thb_yaw",
    "R_idx_prox", "R_mid_prox", "R_pky_prox", "R_rng_prox", "R_thb_yaw",
    "L_idx_int",  "L_mid_int",  "L_pky_int",  "L_rng_int",  "L_thb_pitch",
    "R_idx_int",  "R_mid_int",  "R_pky_int",  "R_rng_int",  "R_thb_pitch",
    "L_thb_dist", "R_thb_dist",
)


def _import_rerun():
    try:
        import rerun as rr            # type: ignore
        return rr
    except ImportError:
        return None


class FingerDashboard:
    """Stateful rerun.io logger.

    Parameters
    ----------
    name
        rerun application id (becomes the recording name).
    spawn
        Whether to spawn a viewer window (default True).
    save_path
        If given, save the recording to this .rrd file as well.
    """

    def __init__(
        self,
        *,
        name: str = "ust_finger_validation",
        spawn: bool = True,
        save_path: Optional[str] = None,
    ) -> None:
        self._rr = _import_rerun()
        self._enabled = self._rr is not None
        self._track: Dict[str, Tuple[float, float]] = {}      # min, max per slot
        if not self._enabled:
            warnings.warn(
                "rerun-sdk not installed — dashboard is a no-op.  "
                "pip install rerun-sdk to enable live visualization.",
                RuntimeWarning,
                stacklevel=2,
            )
            return
        rr = self._rr
        rr.init(name, spawn=spawn)
        if save_path is not None:
            rr.save(save_path)

    # ── per-frame entry point ─────────────────────────────────────────

    def push_frame(
        self,
        t_seconds: float,
        *,
        vmc_bones: Optional[Mapping[str, Sequence[float]]] = None,
        left_11: Optional[Sequence[float]] = None,
        right_11: Optional[Sequence[float]] = None,
        packed_22: Optional[Sequence[float]] = None,
        target_22: Optional[Sequence[float]] = None,
        actual_22: Optional[Sequence[float]] = None,
        source_left: Optional[str] = None,
        source_right: Optional[str] = None,
    ) -> None:
        if not self._enabled:
            return
        rr = self._rr
        rr.set_time_seconds("teleop_clock", float(t_seconds))

        # 1. Per-side 11D mapper output
        for side_label, vec in (("L", left_11), ("R", right_11)):
            if vec is None:
                continue
            for j, name in enumerate(_PER_SIDE_LABELS):
                if j >= len(vec):
                    break
                rr.log(f"finger/{side_label}_{name}/raw", rr.Scalar(float(vec[j])))

        # 2. Packed 22D — running min/max for Δrange
        if packed_22 is not None:
            for j, label in enumerate(_PACKED_22_LABELS):
                if j >= len(packed_22):
                    break
                v = float(packed_22[j])
                rr.log(f"packed/{label}", rr.Scalar(v))
                lo, hi = self._track.get(label, (v, v))
                lo, hi = min(lo, v), max(hi, v)
                self._track[label] = (lo, hi)
                rr.log(f"delta/{label}", rr.Scalar(hi - lo))

        # 3. Target vs actual (Layer 2 / 4)
        if target_22 is not None:
            for j, label in enumerate(_PACKED_22_LABELS):
                if j >= len(target_22):
                    break
                rr.log(f"joint/{label}/target", rr.Scalar(float(target_22[j])))
        if actual_22 is not None:
            for j, label in enumerate(_PACKED_22_LABELS):
                if j >= len(actual_22):
                    break
                rr.log(f"joint/{label}/actual", rr.Scalar(float(actual_22[j])))

        # 4. VMC bone bend magnitude heatmap
        if vmc_bones is not None:
            for bone_name, q in vmc_bones.items():
                if len(q) < 4:
                    continue
                qw = float(q[3])
                # _quat_to_bend equivalent — magnitude in [0, π]
                bend_rad = 2.0 * math.acos(max(-1.0, min(1.0, abs(qw))))
                rr.log(f"vmc/{bone_name}/bend_rad", rr.Scalar(bend_rad))

        # 5. Source labels
        if source_left:
            rr.log("summary/source_left", rr.TextLog(source_left))
        if source_right:
            rr.log("summary/source_right", rr.TextLog(source_right))

    # ── 3-D user hand skeleton (optional) ─────────────────────────────

    def push_user_hand_3d(
        self,
        side: str,                       # "left" or "right"
        joint_positions_w: np.ndarray,   # (N, 3) world-frame positions
        *,
        radii: float = 0.005,
        color: Optional[Tuple[int, int, int]] = None,
    ) -> None:
        """Show the user's hand skeleton in rerun's 3D space view."""
        if not self._enabled:
            return
        rr = self._rr
        if color is None:
            color = (0, 200, 200) if side == "left" else (200, 100, 0)
        rr.log(
            f"world/user_hand_{side}",
            rr.Points3D(np.asarray(joint_positions_w, dtype=np.float32),
                        colors=[color] * len(joint_positions_w),
                        radii=radii),
        )

    # ── shutdown ──────────────────────────────────────────────────────

    def close(self) -> None:
        # rerun's recording flushes automatically on process exit; nothing to
        # do here, but the method exists so tests / runners can `with` it.
        pass

    # support `with FingerDashboard() as dash:` form
    def __enter__(self) -> "FingerDashboard":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ── matplotlib fallback (no rerun) ────────────────────────────────────


class _MatplotlibFallback:
    """Minimal stand-in if rerun is unavailable — buffers data and shows a
    plot at close().  Not real-time but at least gives a visualization.
    """

    def __init__(self) -> None:
        self._t: list[float] = []
        self._packed: list[list[float]] = []

    def push_frame(self, t_seconds, **kwargs) -> None:
        p = kwargs.get("packed_22")
        if p is None:
            return
        self._t.append(float(t_seconds))
        self._packed.append([float(x) for x in p])

    def close(self) -> None:
        if not self._t:
            return
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
        except Exception as exc:                          # noqa: BLE001
            print(f"[FingerDashboard][fallback] matplotlib unavailable: {exc}",
                  file=sys.stderr)
            return
        arr = np.asarray(self._packed)
        fig, ax = plt.subplots(figsize=(12, 5))
        for j, label in enumerate(_PACKED_22_LABELS[:22]):
            ax.plot(self._t, arr[:, j], label=label, linewidth=0.8)
        ax.set_xlabel("time (s)")
        ax.set_ylabel("joint angle (rad)")
        ax.set_title("packed_22 over time (fallback view)")
        ax.legend(loc="upper right", fontsize=6, ncol=4)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()


def make_dashboard(*, name: str = "ust_finger_validation",
                   spawn: bool = True,
                   save_path: Optional[str] = None,
                   prefer_fallback: bool = False):
    """Factory that returns either a :class:`FingerDashboard` or the
    matplotlib fallback, depending on installed packages and ``prefer_fallback``.
    """
    if not prefer_fallback and _import_rerun() is not None:
        return FingerDashboard(name=name, spawn=spawn, save_path=save_path)
    return _MatplotlibFallback()
