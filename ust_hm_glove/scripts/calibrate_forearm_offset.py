"""Measure the forearm-tracker → wrist offset.

Ask the user to strike a clean T-pose and hold still.  Over ``--samples``
frames we average the forearm tracker world pose, and assuming the wrist
is at the same height as the shoulder and offset laterally by the lower-
arm length (typed in ``--lower_arm_m``), compute the offset expressed in
the tracker's local frame.

Writes a JSON with the per-side offsets plus session metadata (timestamps,
serials).  Feed the result into ``PicoUDCAPDeviceCfg.forearm_wrist_offset``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict

import numpy as np

try:
    import openvr  # noqa: F401  (import used indirectly via SteamVRSampler)
except ImportError:
    print("[calibrate_forearm_offset] pyopenvr missing: pip install openvr")
    raise

from ust_ws.ust_hm_glove.teleop.coord_transforms import (
    quat_wxyz_to_matrix,
    svr_to_isaaclab,
)
from ust_ws.ust_hm_glove.teleop.vr_sampler import SteamVRSampler


def load_binding(path: Path) -> Dict[str, Dict[str, str]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    trackers = data.get("trackers", data)
    return {str(sn): {str(k): str(v) for k, v in info.items()} for sn, info in trackers.items()}


def _avg_pose(samples):
    if not samples:
        return None
    poses = np.stack([np.concatenate([s["pos"], s["quat"]]) for s in samples], axis=0)
    avg_pos = poses[:, :3].mean(axis=0)
    # Quaternion averaging: normalise sum (OK for small spread)
    avg_quat = poses[:, 3:7].sum(axis=0)
    avg_quat /= max(float(np.linalg.norm(avg_quat)), 1e-12)
    return {"pos": avg_pos, "quat": avg_quat}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binding",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "config" / "tracker_binding.json",
    )
    parser.add_argument(
        "--actions",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "config" / "openvr_actions" / "actions.json",
    )
    parser.add_argument("--samples", type=int, default=240, help="Frames to average (~2 s @ 120 Hz)")
    parser.add_argument("--warmup", type=int, default=60)
    parser.add_argument(
        "--lower_arm_m",
        type=float,
        default=0.27,
        help="Measured distance from tracker strap to wrist (metres). 0.10–0.15 is typical.",
    )
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    binding = load_binding(args.binding)
    sampler = SteamVRSampler(
        tracker_binding=binding,
        actions_json_path=str(args.actions),
        enable_skeletal=False,  # tracker-only measurement is sufficient
    )
    sampler.start()
    print("[calibrate] Warming up — stand in T-pose with both forearms horizontal, palms forward.")
    try:
        # warmup
        for _ in range(args.warmup):
            _ = sampler.snapshot()
            time.sleep(1.0 / 120.0)

        left_samples = []
        right_samples = []
        print(f"[calibrate] Capturing {args.samples} frames (~{args.samples/120:.1f} s).  Hold still.")
        for _ in range(args.samples):
            snap = sampler.snapshot()
            trk = snap.get("trackers") or {}
            left = trk.get("left_forearm")
            right = trk.get("right_forearm")
            if left is not None:
                left_samples.append(left)
            if right is not None:
                right_samples.append(right)
            time.sleep(1.0 / 120.0)

        left_avg = _avg_pose(left_samples)
        right_avg = _avg_pose(right_samples)

        def to_offset(side_avg):
            if side_avg is None:
                return None
            # Local +X of the tracker expressed in tracker frame:
            # offset = (lower_arm_m, 0, 0).  Nothing to invert in this direction.
            return (float(args.lower_arm_m), 0.0, 0.0)

        result = {
            "timestamp": time.time(),
            "lower_arm_m": args.lower_arm_m,
            "samples_used": {
                "left": len(left_samples),
                "right": len(right_samples),
            },
            "left_forearm_offset": to_offset(left_avg),
            "right_forearm_offset": to_offset(right_avg),
            "left_avg_world_pos": (
                left_avg["pos"].tolist() if left_avg is not None else None
            ),
            "right_avg_world_pos": (
                right_avg["pos"].tolist() if right_avg is not None else None
            ),
        }

        # Preview the wrist position the device will see
        if left_avg is not None:
            R = quat_wxyz_to_matrix(left_avg["quat"])
            wrist_world = left_avg["pos"] + R @ np.array([args.lower_arm_m, 0.0, 0.0])
            wrist_il, _ = svr_to_isaaclab(wrist_world, left_avg["quat"])
            result["preview_left_wrist_isaaclab"] = wrist_il.tolist()
        if right_avg is not None:
            R = quat_wxyz_to_matrix(right_avg["quat"])
            wrist_world = right_avg["pos"] + R @ np.array([args.lower_arm_m, 0.0, 0.0])
            wrist_il, _ = svr_to_isaaclab(wrist_world, right_avg["quat"])
            result["preview_right_wrist_isaaclab"] = wrist_il.tolist()

        out_text = json.dumps(result, indent=2)
        print("\n[calibrate] Result:\n" + out_text)
        if args.out is not None:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(out_text, encoding="utf-8")
            print(f"[calibrate] Wrote {args.out}")
        else:
            print("\n[calibrate] Pass --out <path> to persist this result.")
    finally:
        sampler.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
