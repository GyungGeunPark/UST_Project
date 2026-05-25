"""Calibrate the GR1T2 idle wrist pose for ``GR1T2FourierSteamVRRetargeter``.

Launches Isaac Sim with the stock ``KitchenSortingGR1T2MonitorEnvCfg``
environment, lets PhysX settle for a few hundred steps with the idle
action, and then reads the palm-link (``left_hand_pitch_link`` /
``right_hand_pitch_link``) world pose relative to ``base_link`` so the
user can paste the measured ``pos + quat_wxyz`` tuples into
:data:`ust_ws.ust_hm_glove.teleop.gr1t2_retargeter.DEFAULT_LEFT_POS`
etc.

Outputs both:

* a JSON file at ``--output`` (default ``config/gr1t2_idle_pose.json``);
* a terminal block of Python literals ready to paste into the retargeter.

Usage::

    python -m ust_ws.ust_hm_glove.scripts.calibrate_gr1t2_idle_pose \
        --headless --settle_steps 300 \
        --output ust_ws/ust_hm_glove/config/gr1t2_idle_pose.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


try:
    import pinocchio  # noqa: F401
except Exception:  # pragma: no cover
    pass

try:
    import h5py  # noqa: F401
except Exception:  # pragma: no cover
    pass


from isaaclab.app import AppLauncher


def _parse_args():
    p = argparse.ArgumentParser(description="GR1T2 idle palm pose calibrator")
    p.add_argument("--settle_steps", type=int, default=300)
    p.add_argument(
        "--output",
        type=str,
        default="ust_ws/ust_hm_glove/config/gr1t2_idle_pose.json",
    )
    p.add_argument(
        "--env_variant",
        type=str,
        default="monitor",
        choices=["base", "waist_enabled", "monitor"],
    )
    AppLauncher.add_app_launcher_args(p)
    return p.parse_args()


def _pose_in_base_link(env, env_idx: int, link_name: str):
    """Return ``(pos_xyz, quat_wxyz)`` of ``link_name`` in the base_link frame."""
    robot = env.scene["robot"]
    body_ids, body_names = robot.find_bodies(link_name)
    base_ids, _ = robot.find_bodies("base_link")
    if len(body_ids) == 0 or len(base_ids) == 0:
        raise RuntimeError(f"Could not resolve {link_name} or base_link in robot.")

    # world poses
    world_pos = robot.data.body_pos_w[env_idx]    # (num_bodies, 3)
    world_quat = robot.data.body_quat_w[env_idx]  # (num_bodies, 4) wxyz
    import torch

    # World -> base_link transform
    b_pos = world_pos[base_ids[0]]
    b_quat = world_quat[base_ids[0]]
    w_pos = world_pos[body_ids[0]]
    w_quat = world_quat[body_ids[0]]

    # Relative pose: q_rel = conj(b_quat) * w_quat ; p_rel = R(conj(b_quat)) * (w_pos - b_pos)
    def _conj(q):
        return torch.stack([q[0], -q[1], -q[2], -q[3]])

    def _mul(a, b):
        aw, ax, ay, az = a[0], a[1], a[2], a[3]
        bw, bx, by, bz = b[0], b[1], b[2], b[3]
        return torch.stack([
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ])

    def _rot(q, v):
        qv = torch.stack([torch.tensor(0.0, device=v.device), v[0], v[1], v[2]])
        return _mul(_mul(q, qv), _conj(q))[1:]

    q_rel = _mul(_conj(b_quat), w_quat)
    dp = w_pos - b_pos
    p_rel = _rot(_conj(b_quat), dp)

    return p_rel.cpu().numpy(), q_rel.cpu().numpy()


def main() -> int:
    args = _parse_args()
    app_launcher = AppLauncher(args)
    app = app_launcher.app

    import gymnasium as gym
    import torch

    import ust_ws.ust_hm_glove  # noqa: F401

    env_id = {
        "base": "Isaac-KitchenSorting-GR1T2-Fourier-v0",
        "waist_enabled": "Isaac-KitchenSorting-GR1T2-Fourier-WaistEnabled-v0",
        "monitor": "Isaac-KitchenSorting-GR1T2-Fourier-Monitor-v0",
    }[args.env_variant]

    env_cfg_cls = gym.spec(env_id).kwargs["env_cfg_entry_point"]
    env_cfg = env_cfg_cls()
    env_cfg.scene.num_envs = 1

    from isaaclab.envs import ManagerBasedRLEnv
    env = ManagerBasedRLEnv(cfg=env_cfg)

    print(f"[calibrate] settling for {args.settle_steps} steps…")
    obs, _info = env.reset()
    idle_action = env_cfg.idle_action.unsqueeze(0).repeat(env.num_envs, 1).to(env.device)
    for _ in range(args.settle_steps):
        obs, _r, term, trunc, _info = env.step(idle_action)
        if term.any() or trunc.any():
            obs, _info = env.reset()

    left_pos, left_quat = _pose_in_base_link(env, 0, "left_hand_pitch_link")
    right_pos, right_quat = _pose_in_base_link(env, 0, "right_hand_pitch_link")

    payload = {
        "left_hand_pitch_link": {
            "pos": [float(v) for v in left_pos],
            "quat_wxyz": [float(v) for v in left_quat],
        },
        "right_hand_pitch_link": {
            "pos": [float(v) for v in right_pos],
            "quat_wxyz": [float(v) for v in right_quat],
        },
        "env_id": env_id,
        "settle_steps": args.settle_steps,
    }

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"[calibrate] wrote {out_path}")

    print("\n-- Paste into gr1t2_retargeter.py --")
    print(
        f"DEFAULT_LEFT_POS  = ({left_pos[0]:+.4f}, {left_pos[1]:+.4f}, {left_pos[2]:+.4f})"
    )
    print(
        f"DEFAULT_LEFT_QUAT = ({left_quat[0]:+.4f}, {left_quat[1]:+.4f}, "
        f"{left_quat[2]:+.4f}, {left_quat[3]:+.4f})"
    )
    print(
        f"DEFAULT_RIGHT_POS  = ({right_pos[0]:+.4f}, {right_pos[1]:+.4f}, {right_pos[2]:+.4f})"
    )
    print(
        f"DEFAULT_RIGHT_QUAT = ({right_quat[0]:+.4f}, {right_quat[1]:+.4f}, "
        f"{right_quat[2]:+.4f}, {right_quat[3]:+.4f})"
    )

    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
