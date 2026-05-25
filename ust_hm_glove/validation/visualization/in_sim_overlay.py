"""Isaac Lab in-sim VisualizationMarkers overlay for the user's hand.

Renders 32 spheres next to the robot (one per VMC finger bone position
estimate) so the operator can compare the user's hand pose against the
robot's hand pose visually in the same Isaac Sim viewport.

Calls into ``isaaclab.markers.VisualizationMarkers`` — only loadable
inside an Isaac Sim runtime.  When invoked outside that context the
class becomes a no-op (logs a one-time warning).

Layer-3 (visual replay) entry point.

Usage (inside Isaac Sim app)::

    from ust_ws.ust_hm_glove.validation.visualization.in_sim_overlay import UserHandOverlay
    overlay = UserHandOverlay(num_joints_per_hand=16)

    # in env.step loop
    overlay.update(
        positions_left_w=user_hand_left_xyz,    # (N, 3) world frame
        positions_right_w=user_hand_right_xyz,
    )

When VMC has no per-joint position data (only quaternions), the overlay
can synthesize sphere positions by composing a fixed bone-length
kinematic chain — see :meth:`UserHandOverlay.fk_from_quats_and_origin`.
"""

from __future__ import annotations

import math
import warnings
from typing import Mapping, Optional, Sequence, Tuple

import numpy as np


_NUM_FINGERS = 5
_DEFAULT_BONE_LEN_M = 0.030  # 3 cm per phalanx (rough adult hand)


def _try_isaaclab_imports():
    """Return (sim_utils, VisualizationMarkers, VisualizationMarkersCfg) or None."""
    try:
        import isaaclab.sim as sim_utils                                  # type: ignore
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # type: ignore
        return sim_utils, VisualizationMarkers, VisualizationMarkersCfg
    except Exception:
        return None


class UserHandOverlay:
    """Visualization-marker pair (left / right user hand spheres)."""

    def __init__(
        self,
        *,
        prim_path_root: str = "/Visuals/UstUserHandOverlay",
        sphere_radius: float = 0.008,
        left_color: Tuple[float, float, float] = (0.0, 0.7, 0.9),
        right_color: Tuple[float, float, float] = (0.95, 0.55, 0.1),
    ) -> None:
        self._enabled = False
        imports = _try_isaaclab_imports()
        if imports is None:
            warnings.warn(
                "isaaclab not importable — UserHandOverlay is a no-op.  "
                "Run inside Isaac Sim to enable in-sim visualization.",
                RuntimeWarning,
                stacklevel=2,
            )
            self._left_markers = None
            self._right_markers = None
            return

        sim_utils, VisualizationMarkers, VisualizationMarkersCfg = imports
        try:
            cfg_left = VisualizationMarkersCfg(
                prim_path=f"{prim_path_root}/left",
                markers={
                    "joint": sim_utils.SphereCfg(
                        radius=sphere_radius,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=left_color
                        ),
                    ),
                },
            )
            cfg_right = VisualizationMarkersCfg(
                prim_path=f"{prim_path_root}/right",
                markers={
                    "joint": sim_utils.SphereCfg(
                        radius=sphere_radius,
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=right_color
                        ),
                    ),
                },
            )
            self._left_markers = VisualizationMarkers(cfg_left)
            self._right_markers = VisualizationMarkers(cfg_right)
            self._enabled = True
        except Exception as exc:                                  # noqa: BLE001
            warnings.warn(f"VisualizationMarkers construction failed: {exc!r}.  "
                          "UserHandOverlay disabled.",
                          RuntimeWarning, stacklevel=2)
            self._left_markers = None
            self._right_markers = None

    @property
    def enabled(self) -> bool:
        return self._enabled

    def update(
        self,
        *,
        positions_left_w: Optional[np.ndarray] = None,
        positions_right_w: Optional[np.ndarray] = None,
    ) -> None:
        """Update sphere positions.  ``positions_*_w`` are (N, 3) numpy arrays."""
        if not self._enabled:
            return
        try:
            import torch                                                   # type: ignore
        except ImportError:
            return
        if positions_left_w is not None and self._left_markers is not None:
            arr = np.asarray(positions_left_w, dtype=np.float32)
            self._left_markers.visualize(translations=torch.from_numpy(arr))
        if positions_right_w is not None and self._right_markers is not None:
            arr = np.asarray(positions_right_w, dtype=np.float32)
            self._right_markers.visualize(translations=torch.from_numpy(arr))

    # ── helper: compose finger spheres from VMC quats + bone lengths ──

    @staticmethod
    def fk_from_quats_and_origin(
        bone_quats: Mapping[str, Sequence[float]],
        *,
        side: str,
        wrist_origin_w: np.ndarray,
        bone_length_m: float = _DEFAULT_BONE_LEN_M,
    ) -> np.ndarray:
        """Forward-kinematics a coarse 16-sphere skeleton from VMC bone quats.

        This is a *rough* approximation — UDCAP's VMC payload only carries
        per-bone orientations relative to a parent, not positions, so we
        assume a fixed bone length per phalanx.  Good enough for visual
        overlay; not a substitute for Skeletal Input 2.0 positions.

        Returns (16, 3) world-frame positions::

            wrist + 5 fingers × (proximal, intermediate, distal-tip)
        """
        out: list[np.ndarray] = [np.asarray(wrist_origin_w, dtype=np.float64)]
        # finger root offsets (right-handed +Y forward)
        finger_root_offsets = {
            "Thumb":  np.array([+0.045, +0.020, 0.0]),
            "Index":  np.array([+0.085, +0.030, 0.0]),
            "Middle": np.array([+0.090, +0.005, 0.0]),
            "Ring":   np.array([+0.085, -0.020, 0.0]),
            "Little": np.array([+0.075, -0.045, 0.0]),
        }
        side_factor = 1.0 if side.lower().startswith("l") else -1.0
        side_label = "Left" if side.lower().startswith("l") else "Right"

        for finger, root_offset in finger_root_offsets.items():
            base = np.asarray(wrist_origin_w, dtype=np.float64) + root_offset * np.array([1.0, side_factor, 1.0])
            current_dir = np.array([1.0, 0.0, 0.0])
            current = base.copy()
            parts = ("Proximal", "Intermediate", "Distal")
            for part in parts:
                bone_name = f"{side_label}{finger}{part}"
                q = bone_quats.get(bone_name, (0.0, 0.0, 0.0, 1.0))
                qx, qy, qz, qw = (float(x) for x in q)
                # Rotate the running direction by this quat (around X axis primarily).
                # We extract the bend angle; assume it pulls the direction down.
                bend_rad = 2.0 * math.acos(max(-1.0, min(1.0, abs(qw))))
                # Apply a simple pitch-down by `bend_rad` in the YZ plane.
                pitch = bend_rad
                rot = np.array([
                    [math.cos(pitch),  0.0, math.sin(pitch)],
                    [0.0,              1.0, 0.0],
                    [-math.sin(pitch), 0.0, math.cos(pitch)],
                ])
                current_dir = rot @ current_dir
                current = current + bone_length_m * current_dir
                out.append(current.copy())
        return np.asarray(out, dtype=np.float32)
