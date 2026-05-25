"""Layer-2 headless replay — feed a JSONL VMC fixture into Isaac Lab and
record the robot's joint response to HDF5.

Pipeline::

    JSONL VMC packets
        → group_into_frames        (replay_vmc helper)
        → FourierHandMapper        (per-frame 22D)
        → robot.set_joint_position_target() on hand joints
        → env.step()                physics integration
        → robot.data.joint_pos      sample
        → HDF5 dump

The HDF5 schema is consumed by ``tools.analyze_replay_hdf5``.

Important
---------
* Requires Isaac Lab + isaaclab_tasks + the ust_hm_glove env_cfg.
* Pass ``--headless`` to suppress the renderer entirely (CI mode).
* Without ``--env-id`` this defaults to the GR1T2 RobotOnly env.

Usage::

    python -m ust_ws.ust_hm_glove.validation.scripts.run_replay_headless \\
        --replay ust_ws/ust_hm_glove/validation/tests/golden/full_fist.vmc.jsonl \\
        --output results/headless_full_fist.hdf5 \\
        --steps 200 \\
        --headless
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np


def _bootstrap_app(headless: bool) -> tuple:
    """Launch Isaac Sim and return (sim_app, env, robot, hand_joint_ids)."""
    # Isaac Lab requires IPC_IGNORE_VERSION before AppLauncher import.
    os.environ.setdefault("IPC_IGNORE_VERSION", "1")
    # h5py preload to avoid hdf5.dll mismatch (memory.md §3.8).
    try:
        import h5py                       # noqa: F401
    except ImportError:
        pass
    from isaaclab.app import AppLauncher                                # type: ignore
    parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(parser)
    args, _ = parser.parse_known_args(["--headless"] if headless else [])
    app = AppLauncher(args)
    sim_app = app.app

    import gymnasium as gym                                              # noqa: F401
    from ust_ws.ust_hm_glove import kitchen_sorting_gr1t2_env_cfg  # noqa: F401
    from ust_ws.ust_hm_glove.kitchen_sorting_gr1t2_env_cfg import (
        KitchenSortingGR1T2RobotOnlyEnvCfg,
    )

    env_id = "Isaac-KitchenSorting-GR1T2-Fourier-RobotOnly-v0"
    # 9.19 fix: ManagerBasedRLEnv requires explicit cfg= in gym.make.
    # The gym registration only stores env_cfg_entry_point as metadata;
    # we must instantiate it ourselves.
    env_cfg = KitchenSortingGR1T2RobotOnlyEnvCfg()
    # Force single environment for headless replay.
    try:
        env_cfg.scene.num_envs = 1
        env_cfg.scene.env_spacing = 0.0
    except Exception:                                  # noqa: BLE001
        pass
    env = gym.make(env_id, cfg=env_cfg)
    obs, _ = env.reset()
    robot = env.unwrapped.scene["robot"]
    hand_joint_ids = _resolve_hand_joint_ids(robot)
    return sim_app, env, robot, hand_joint_ids


def _resolve_hand_joint_ids(robot) -> List[int]:
    """Match FOURIER_HAND_JOINT_NAMES to the articulation's joint_names list."""
    from ust_ws.ust_hm_glove.kitchen_sorting_gr1t2_env_cfg import (
        FOURIER_HAND_JOINT_NAMES,
    )
    all_names = list(robot.data.joint_names)
    out: List[int] = []
    for n in FOURIER_HAND_JOINT_NAMES:
        if n in all_names:
            out.append(all_names.index(n))
        else:
            print(f"[run_replay_headless][WARN] hand joint missing: {n}",
                  file=sys.stderr)
    return out


# ── idle arm action helper ───────────────────────────────────────────


def build_idle_arm_14() -> np.ndarray:
    """Return the 14D idle arm portion of the 36D Pink IK action.

    Layout per gr1t2_retargeter::
        [0:3]   left wrist position  (base_link frame, metres)
        [3:7]   left wrist quaternion (wxyz)
        [7:10]  right wrist position
        [10:14] right wrist quaternion (wxyz)

    The all-zero quaternion ``(0,0,0,0)`` is mathematically invalid (zero
    norm) and causes Pink IK's OSQP solver to build a non-PSD KKT matrix
    every step, producing ``"The problem seems to be non-convex"`` +
    ``"Workspace allocation error!"`` repeatedly.  Filling 14D with the
    documented idle T-pose targets (DEFAULT_LEFT_POS/QUAT,
    DEFAULT_RIGHT_POS/QUAT from gr1t2_retargeter) keeps the QP convex
    and the arms in a stable T-pose while only finger joints animate.
    """
    try:
        # Prefer the production retargeter's defaults (kept in sync with
        # the env_cfg's IDLE_ACTION).
        from ust_ws.ust_hm_glove.teleop.gr1t2_retargeter import (
            DEFAULT_LEFT_POS, DEFAULT_LEFT_QUAT,
            DEFAULT_RIGHT_POS, DEFAULT_RIGHT_QUAT,
        )
    except Exception:                                  # noqa: BLE001
        # Hard-coded fallback (matches research/35. and current code).
        DEFAULT_LEFT_POS = (-0.20, 0.00, 1.05)
        DEFAULT_LEFT_QUAT = (0.707, 0.0, 0.707, 0.0)
        DEFAULT_RIGHT_POS = (0.20, 0.00, 1.05)
        DEFAULT_RIGHT_QUAT = (0.707, 0.0, 0.707, 0.0)
    return np.asarray(
        list(DEFAULT_LEFT_POS) + list(DEFAULT_LEFT_QUAT)
        + list(DEFAULT_RIGHT_POS) + list(DEFAULT_RIGHT_QUAT),
        dtype=np.float32,
    )


# ── frame→hand action helper ─────────────────────────────────────────


def frames_to_hand_actions(records, *,
                           proximal_scale: float = 2.5,
                           thumb_scale: float = 2.5,
                           use_tanh: bool = True,
                           vmc_subtract_rest: bool = False,
                           vmc_rest_frames: int = 30,
                           frame_window_us: int = 30_000) -> np.ndarray:
    """Drive replay_vmc.feed_to_mapper_jsonl logic offline → return (T, 22)."""
    from ust_ws.ust_hm_glove.validation._bootstrap import load_fourier_hand_mapper
    from ust_ws.ust_hm_glove.validation.tools import replay_vmc
    fhm = load_fourier_hand_mapper()
    mapper = fhm.FourierHandMapper(
        proximal_scale=proximal_scale, thumb_scale=thumb_scale,
        use_tanh=use_tanh, vmc_subtract_rest=vmc_subtract_rest,
        vmc_rest_frames=vmc_rest_frames,
    )
    out: list[np.ndarray] = []
    for frame_bones in replay_vmc.group_into_frames(records, frame_window_us=frame_window_us):
        L = mapper.map_hand_vmc(frame_bones, is_right=False)
        R = mapper.map_hand_vmc(frame_bones, is_right=True)
        out.append(np.asarray(fhm.pack_22d(L, R), dtype=np.float32))
    if not out:
        return np.zeros((0, 22), dtype=np.float32)
    return np.stack(out, axis=0)


# ── main ─────────────────────────────────────────────────────────────


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--replay", type=Path, required=True, help="JSONL VMC fixture")
    p.add_argument("--output", type=Path, required=True, help="HDF5 output")
    p.add_argument("--steps", type=int, default=200,
                   help="Number of env.step() calls (cycles through the frames if shorter)")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--proximal-scale", type=float, default=2.5)
    p.add_argument("--thumb-scale", type=float, default=2.5)
    p.add_argument("--no-tanh", action="store_true")
    p.add_argument("--subtract-rest", action="store_true")
    p.add_argument("--rest-frames", type=int, default=30)
    p.add_argument("--frame-window-us", type=int, default=30_000)
    args = p.parse_args()

    if not args.replay.exists():
        print(f"[run_replay_headless][ERROR] {args.replay} not found", file=sys.stderr)
        return 1

    print(f"[run_replay_headless] launching Isaac Lab (headless={args.headless})…")
    try:
        sim_app, env, robot, hand_joint_ids = _bootstrap_app(args.headless)
    except Exception as exc:                                          # noqa: BLE001
        print(f"[run_replay_headless][ERROR] Isaac Lab bootstrap failed: {exc}",
              file=sys.stderr)
        return 2

    from ust_ws.ust_hm_glove.validation.tools import replay_vmc

    print(f"[run_replay_headless] loading {args.replay}…")
    records = list(replay_vmc.iter_records(args.replay))
    actions_22 = frames_to_hand_actions(
        records,
        proximal_scale=args.proximal_scale,
        thumb_scale=args.thumb_scale,
        use_tanh=not args.no_tanh,
        vmc_subtract_rest=args.subtract_rest,
        vmc_rest_frames=args.rest_frames,
        frame_window_us=args.frame_window_us,
    )
    if actions_22.shape[0] == 0:
        print("[run_replay_headless][ERROR] zero frames — bad fixture?", file=sys.stderr)
        return 3
    print(f"[run_replay_headless] {actions_22.shape[0]} mapper frames; "
          f"running for {args.steps} env steps")

    import torch                                                          # type: ignore

    # 9.19 fix: idle arm action with valid wrist target — without this
    # the Pink IK QP solver builds a non-PSD KKT matrix every step and
    # spams "The problem seems to be non-convex" + "Workspace allocation
    # error!".  See run_replay_headless.build_idle_arm_14 docstring.
    idle_arm_14 = build_idle_arm_14()
    print(f"[run_replay_headless] idle arm 14D: {idle_arm_14.tolist()}")

    # Warm up the QP with idle action (no fingers) for a few steps so
    # the solver settles before we start animating fingers.
    warmup_action = np.zeros((1, 36), dtype=np.float32)
    warmup_action[:, :14] = idle_arm_14
    warmup_tensor = torch.tensor(warmup_action, dtype=torch.float32, device=robot.device)
    for w in range(5):
        env.step(warmup_tensor)
    print("[run_replay_headless] QP warmup complete (5 idle steps)")

    targets, actuals = [], []
    timestamps = []
    n_frames = actions_22.shape[0]
    for step in range(args.steps):
        # 9.20 fix: clamp to last frame instead of wrapping back to frame 0.
        # The previous `step % n_frames` wrap caused a target discontinuity
        # at the loop boundary (e.g. step 1000: full fist → rest pose in
        # one tick) → PhysX took 30+ steps to catch up → spurious 0.9 rad
        # tracking error spikes that polluted the analyze_replay_hdf5 max
        # statistics.  Clamping holds the last commanded pose, which is
        # what a real teleop session would do at end-of-recording.
        if step < n_frames:
            frame = actions_22[step]
        else:
            frame = actions_22[-1]
        # 9.19 fix: action_36 = [idle arm 14D | mapper finger 22D]
        # Previously sent all-zero arm portion which gave Pink IK a zero
        # quaternion (norm 0, mathematically invalid) and OSQP failed
        # every step.
        action_36 = np.zeros((1, 36), dtype=np.float32)
        action_36[0, :14] = idle_arm_14
        action_36[:, 14:36] = frame
        env.step(torch.tensor(action_36, dtype=torch.float32, device=robot.device))

        # Sample target / actual joint pos
        jp_target = robot.data.joint_pos_target[0, hand_joint_ids].detach().cpu().numpy()
        jp_actual = robot.data.joint_pos[0, hand_joint_ids].detach().cpu().numpy()
        targets.append(jp_target)
        actuals.append(jp_actual)
        timestamps.append(step * env.unwrapped.physics_dt)
        if step % 50 == 0:
            print(f"  step {step:3d}  err.max={float(np.abs(jp_target - jp_actual).max()):.3f}")
        if step + 1 == actions_22.shape[0]:
            n_loops += 1

    targets_arr = np.stack(targets, axis=0)
    actuals_arr = np.stack(actuals, axis=0)
    timestamps_arr = np.asarray(timestamps, dtype=np.float64)

    # Joint limits
    limits = robot.data.joint_pos_limits[0, hand_joint_ids].detach().cpu().numpy()  # (22, 2)

    # Write HDF5
    print(f"[run_replay_headless] writing {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    import h5py
    with h5py.File(args.output, "w") as f:
        f.create_dataset("/timestamps_s", data=timestamps_arr)
        f.create_dataset("/joint_pos_target", data=targets_arr)
        f.create_dataset("/joint_pos_actual", data=actuals_arr)
        f.create_dataset("/joint_pos_limits", data=limits)
        f.create_dataset("/joint_names",
                         data=np.asarray([n.encode("utf-8") for n in
                                          [robot.data.joint_names[i] for i in hand_joint_ids]]))
        meta = f.create_group("/metadata")
        meta.attrs["replay_source"] = str(args.replay)
        meta.attrs["steps"] = int(args.steps)
        meta.attrs["proximal_scale"] = float(args.proximal_scale)
        meta.attrs["thumb_scale"] = float(args.thumb_scale)
        meta.attrs["use_tanh"] = bool(not args.no_tanh)
        meta.attrs["vmc_subtract_rest"] = bool(args.subtract_rest)
        meta.attrs["vmc_rest_frames"] = int(args.rest_frames)
    print(f"[run_replay_headless] done — {len(timestamps)} steps written.")
    sim_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
