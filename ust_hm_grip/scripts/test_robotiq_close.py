"""End-to-end verification that PICO grip → gripper close lands on
joint position targets.

This is the missing companion of ``test_robotiq_pose.py`` (which only
exercises the OPEN command).  We:

1. Reset the env in robot_only variant.
2. Step the idle action (16-D, grippers +1 = OPEN) for 60 frames so the
   Robotiq joints settle near 0 rad.  Sanity check that the open command
   produces close-to-zero joint positions.
3. Send a CLOSE action on BOTH grippers (idle action with [14]=[15]=-1)
   for 120 more frames.  Verify the lead joint (left/right_finger_joint)
   reaches ``ROBOTIQ_CLOSE_RAD`` (+0.785) ± tolerance and that each
   follower reaches ``gearing * ROBOTIQ_CLOSE_RAD`` ± tolerance.
4. Send an asymmetric action: close LEFT only ([14]=-1, [15]=+1) for 120
   frames.  Verify left stays closed, right returns to 0.

This exercises the full PICO grip → 16-D action → Action Manager →
BinaryJointPositionAction → set_joint_position_target chain.  If any
wiring is wrong (action slot order, signal sign, missing actuator,
broken mimic), this test fails with a clear per-joint diagnostic.

PASS criteria (with stiffness=10 / damping=80 implicit PD):
- After close command, lead joint within ±10° of close target.
- After open command, all 12 joints within ±10° of 0.

The ±10° tolerance matches ``test_robotiq_pose.py``'s settle envelope
(gravity-induced steady-state offset against the deliberately-low
stiffness chosen to dodge the Issue #3347 wrist-rotation artifact at
high stiffness).
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT))


def _boot_isaac_sim() -> "object":
    from isaaclab.app import AppLauncher

    boot_parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(boot_parser)
    boot_args, remaining = boot_parser.parse_known_args()
    boot_args.headless = True
    app_launcher = AppLauncher(boot_args)
    sim_app = app_launcher.app
    sys.argv = [sys.argv[0]] + remaining
    return sim_app


def main() -> int:
    sim_app = _boot_isaac_sim()
    try:
        import torch

        from ust_ws.ust_hm_grip.kitchen_sorting_gr1t2_gripper_env_cfg import (
            KitchenSortingGR1T2GripperRobotOnlyEnvCfg,
            ROBOTIQ_ALL_JOINTS_LEFT,
            ROBOTIQ_ALL_JOINTS_RIGHT,
            ROBOTIQ_CLOSE_RAD,
            _ROBOTIQ_LEFT_JOINTS_WITH_GEAR,
            _ROBOTIQ_RIGHT_JOINTS_WITH_GEAR,
        )
        from isaaclab.envs import ManagerBasedRLEnv

        cfg = KitchenSortingGR1T2GripperRobotOnlyEnvCfg()
        cfg.scene.num_envs = 1
        env = ManagerBasedRLEnv(cfg)

        idle = cfg.idle_action.unsqueeze(0).to(env.device)  # shape (1, 16)
        env.reset()

        robot = env.scene["robot"]
        joint_name_to_idx = {n: i for i, n in enumerate(robot.joint_names)}

        # ── DIAG -- expose action_manager internals ──────────────────
        am = env.action_manager
        print("\n[diag] Action Manager terms (resolved):", flush=True)
        for term_name in am.active_terms:
            term = am.get_term(term_name)
            print(f"  {term_name}: action_dim={term.action_dim} "
                  f"joint_names={getattr(term, '_joint_names', None)} "
                  f"joint_ids={getattr(term, '_joint_ids', None)}", flush=True)
            if hasattr(term, "_close_command"):
                print(f"    open_command  = {term._open_command.detach().cpu().tolist()}",
                      flush=True)
                print(f"    close_command = {term._close_command.detach().cpu().tolist()}",
                      flush=True)

        gear_left = {name: gear for name, gear in _ROBOTIQ_LEFT_JOINTS_WITH_GEAR}
        gear_right = {name: gear for name, gear in _ROBOTIQ_RIGHT_JOINTS_WITH_GEAR}

        all_joints = list(ROBOTIQ_ALL_JOINTS_LEFT) + list(ROBOTIQ_ALL_JOINTS_RIGHT)
        joint_indices = {j: joint_name_to_idx[j] for j in all_joints
                         if j in joint_name_to_idx}
        missing = [j for j in all_joints if j not in joint_indices]
        for m in missing:
            print(f"[test_robotiq_close] WARNING -- joint '{m}' missing")
            return 2

        # Tolerance budget (9th session, lead-only drive architecture):
        #   * Lead joint has an explicit BinaryJointPositionAction
        #     position target -> tight tol (TOL_LEAD_DEG).
        #   * Followers have K=0 actuators and are carried by the
        #     4-bar linkage + PhysxMimicJointAPI.  Their final angles
        #     depend on linkage geometry; in steady state they should
        #     approach gear*lead but can drift more under PhysX
        #     solver tolerance.  Looser tol (TOL_FOLLOWER_DEG).
        #   * For OPEN command both lead and followers are at rest near
        #     0deg with only gravity drift -> tight tol on all.
        TOL_LEAD_DEG = 10.0
        TOL_FOLLOWER_DEG = 25.0
        TOL_OPEN_DEG = 10.0
        N_SETTLE_OPEN = 60
        # K=200 / D=20 on lead -> slow-pole rate K/D=10 -> tau=0.1s.
        # 240 frames at 120 Hz = 2.0 s = 20*tau -- well-settled.
        N_SETTLE_CLOSE = 240

        # ────────────────────────────────────────────────────────────────
        # Phase 1 -- idle (both OPEN), confirm joints settle to 0
        # ────────────────────────────────────────────────────────────────
        print(f"\n[Phase 1] idle action (both grippers OPEN), "
              f"{N_SETTLE_OPEN} steps", flush=True)
        for _ in range(N_SETTLE_OPEN):
            env.step(idle)

        q = robot.data.joint_pos[0].detach().cpu().tolist()
        phase1_pass = True
        for j in all_joints:
            v_deg = math.degrees(q[joint_indices[j]])
            ok = abs(v_deg) < TOL_OPEN_DEG
            if not ok:
                phase1_pass = False
                print(f"  [FAIL] {j:<45} = {v_deg:+9.3f}deg (open target = 0)",
                      flush=True)
        if phase1_pass:
            print(f"  [OK ] all 12 joints within +-{TOL_OPEN_DEG}deg of 0 (open target)",
                  flush=True)

        # ────────────────────────────────────────────────────────────────
        # Phase 2 -- both grippers CLOSE, verify lead + followers reach
        #             gear * 0.785 rad target
        # ────────────────────────────────────────────────────────────────
        close_both = idle.clone()
        close_both[0, 14] = -1.0  # left gripper close
        close_both[0, 15] = -1.0  # right gripper close
        print(f"\n[Phase 2] both grippers CLOSE (action[14,15] = -1, -1), "
              f"{N_SETTLE_CLOSE} steps", flush=True)
        for s in range(N_SETTLE_CLOSE):
            env.step(close_both)
            if s == 0 or s == 5 or s == 30:
                lg = am.get_term("left_gripper_action")
                rg = am.get_term("right_gripper_action")
                print(f"  [diag step {s}] raw L={lg._raw_actions[0].tolist()} "
                      f"raw R={rg._raw_actions[0].tolist()} "
                      f"processed_L={lg._processed_actions[0].tolist()} "
                      f"processed_R={rg._processed_actions[0].tolist()}", flush=True)
                jp = robot.data.joint_pos[0].detach().cpu().tolist()
                jpt = robot.data.joint_pos_target[0].detach().cpu().tolist()
                lj = joint_indices["left_finger_joint"]
                rj = joint_indices["right_finger_joint"]
                print(f"  [diag step {s}] L_finger_joint pos="
                      f"{math.degrees(jp[lj]):+.2f}deg target="
                      f"{math.degrees(jpt[lj]):+.2f}deg | "
                      f"R_finger_joint pos={math.degrees(jp[rj]):+.2f}deg "
                      f"target={math.degrees(jpt[rj]):+.2f}deg", flush=True)

        q = robot.data.joint_pos[0].detach().cpu().tolist()
        phase2_pass = True
        lead_joints = {"left_finger_joint", "right_finger_joint"}
        for j in all_joints:
            gear = gear_left.get(j, gear_right.get(j, 1.0))
            target_rad = gear * ROBOTIQ_CLOSE_RAD
            target_deg = math.degrees(target_rad)
            v_deg = math.degrees(q[joint_indices[j]])
            err_deg = v_deg - target_deg
            tol = TOL_LEAD_DEG if j in lead_joints else TOL_FOLLOWER_DEG
            ok = abs(err_deg) < tol
            tag = "OK  " if ok else "FAIL"
            role = "LEAD    " if j in lead_joints else "FOLLOWER"
            print(f"  [{tag}] {role} {j:<45} = {v_deg:+9.3f}deg "
                  f"(target gear*{ROBOTIQ_CLOSE_RAD:.3f}={target_deg:+9.3f}deg, "
                  f"err={err_deg:+8.3f}deg, tol +-{tol}deg)", flush=True)
            if not ok:
                phase2_pass = False

        # ────────────────────────────────────────────────────────────────
        # Phase 3 -- asymmetric: close LEFT, open RIGHT.  Verifies
        #             independent L/R action slot wiring.
        # ────────────────────────────────────────────────────────────────
        close_left_only = idle.clone()
        close_left_only[0, 14] = -1.0  # left close
        close_left_only[0, 15] = +1.0  # right open (back to 0)
        print(f"\n[Phase 3] LEFT close + RIGHT open (action[14,15] = -1, +1), "
              f"{N_SETTLE_CLOSE} steps", flush=True)
        for _ in range(N_SETTLE_CLOSE):
            env.step(close_left_only)

        q = robot.data.joint_pos[0].detach().cpu().tolist()
        phase3_pass = True
        for j in ROBOTIQ_ALL_JOINTS_LEFT:
            gear = gear_left[j]
            target_deg = math.degrees(gear * ROBOTIQ_CLOSE_RAD)
            v_deg = math.degrees(q[joint_indices[j]])
            err_deg = v_deg - target_deg
            tol = TOL_LEAD_DEG if j in lead_joints else TOL_FOLLOWER_DEG
            ok = abs(err_deg) < tol
            tag = "OK  " if ok else "FAIL"
            print(f"  [{tag}] LEFT  {j:<40} = {v_deg:+9.3f}deg "
                  f"(target={target_deg:+9.3f}deg, err={err_deg:+8.3f}deg, "
                  f"tol +-{tol}deg)",
                  flush=True)
            if not ok:
                phase3_pass = False
        for j in ROBOTIQ_ALL_JOINTS_RIGHT:
            target_deg = 0.0
            v_deg = math.degrees(q[joint_indices[j]])
            err_deg = v_deg - target_deg
            ok = abs(err_deg) < TOL_OPEN_DEG
            tag = "OK  " if ok else "FAIL"
            print(f"  [{tag}] RIGHT {j:<40} = {v_deg:+9.3f}deg "
                  f"(target=0deg [open], err={err_deg:+8.3f}deg, "
                  f"tol +-{TOL_OPEN_DEG}deg)",
                  flush=True)
            if not ok:
                phase3_pass = False

        # ────────────────────────────────────────────────────────────────
        # Summary
        # ────────────────────────────────────────────────────────────────
        all_pass = phase1_pass and phase2_pass and phase3_pass
        print("\n" + "=" * 72, flush=True)
        print(f"  Phase 1 (open):                 {'PASS' if phase1_pass else 'FAIL'}",
              flush=True)
        print(f"  Phase 2 (close both):           {'PASS' if phase2_pass else 'FAIL'}",
              flush=True)
        print(f"  Phase 3 (close left, open right): "
              f"{'PASS' if phase3_pass else 'FAIL'}", flush=True)
        print("=" * 72, flush=True)
        verdict = "PASS" if all_pass else "FAIL"
        print(f"\n[test_robotiq_close] VERDICT: {verdict}", flush=True)
        if all_pass:
            print("  Pipeline OK: PICO grip retargeter sign convention "
                  "(GRIPPER_CLOSE=-1) drives the Robotiq lead joint to "
                  f"+{math.degrees(ROBOTIQ_CLOSE_RAD):.1f}deg and each "
                  "follower to gear * close, independently per side.",
                  flush=True)
        return 0 if all_pass else 1
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
