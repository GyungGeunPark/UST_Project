"""Layer-1 diagnostic for research/33 Cause #2 / #3.

Reads the GR1T2 USD that Isaac Lab spawns and prints the
``physics:maxJointVelocity`` and ``drive:angular:physics:maxForce``
attributes baked into every L_*/R_* finger joint.  Lets the user verify
whether the finger-tracking lag is rooted in a low USD-baked velocity
cap (in which case the 9.27 ``velocity_limit_sim=50`` cfg override
fixes it) or whether the USD already permits 50+ rad/s (in which case
the cfg override is just defensive).

Usage::

    ./isaaclab.sh -p ust_ws/ust_hm_glove/scripts/diagnose_finger_actuator_limits.py

This script is *headless* — it does NOT launch Isaac Sim, only opens
the USD file via the Pixar USD library that Isaac Lab depends on.

Reference: research/33 §2.2 (velocity_limit silent-ignore in
ImplicitActuator) and §4.1 (layer-by-layer diagnostic protocol).
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional


def _resolve_usd_path() -> Optional[Path]:
    """Locate the GR1T2 fourier hand USD that Isaac Lab spawns.

    Mirrors the resolution logic in ``isaaclab_assets.robots.fourier``
    but works without booting Isaac Sim.
    """
    try:
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # type: ignore
    except Exception as exc:  # noqa: BLE001
        print(f"[diagnose] could not import ISAAC_NUCLEUS_DIR ({exc!r}); "
              f"specify USD path on the command line: "
              f"./isaaclab.sh -p {Path(__file__).name} <path/to/gr1t2.usd>")
        return None

    rel = "Robots/FourierIntelligence/GR-1/GR1T2_fourier_hand_6dof/GR1T2_fourier_hand_6dof.usd"
    candidate = Path(ISAAC_NUCLEUS_DIR) / rel
    if not candidate.exists():
        print(f"[diagnose] USD not found at {candidate} -- this is normal "
              f"if your ISAAC_NUCLEUS_DIR is the Nucleus URL form.  Pass the "
              f"file path explicitly:  ./isaaclab.sh -p {Path(__file__).name} <path>")
        return None
    return candidate


def _print_finger_joint_limits(usd_path: Path) -> int:
    """Open USD via Pixar USD lib and dump per-finger-joint limits."""
    try:
        from pxr import Usd, UsdPhysics  # type: ignore
    except ImportError as exc:
        print(f"[diagnose] pxr.Usd unavailable ({exc!r}).  Run via "
              "isaaclab.sh -p, not bare python.")
        return 1

    print(f"[diagnose] Opening {usd_path} ...")
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        print(f"[diagnose] FAILED to open {usd_path}")
        return 2

    rows = []
    for prim in stage.Traverse():
        name = prim.GetName()
        if not (name.startswith("L_") or name.startswith("R_")):
            continue
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue

        max_vel_attr = prim.GetAttribute("physics:maxJointVelocity")
        max_vel = max_vel_attr.Get() if max_vel_attr.IsValid() else None

        # Drive max force (effort limit) lives under
        # ``drive:angular:physics:maxForce`` for a revolute joint.
        max_force_attr = prim.GetAttribute("drive:angular:physics:maxForce")
        max_force = max_force_attr.Get() if max_force_attr.IsValid() else None

        rows.append((name, max_vel, max_force))

    if not rows:
        print("[diagnose] No L_*/R_* joints found in the stage.  USD path correct?")
        return 3

    print()
    print(f"  {'joint':<40s}  {'maxJointVelocity':>16s}  {'maxForce':>12s}")
    print(f"  {'-' * 40}  {'-' * 16}  {'-' * 12}")
    for name, mv, mf in sorted(rows):
        mv_s = "<unset>" if mv is None else f"{mv:.2f}"
        mf_s = "<unset>" if mf is None else f"{mf:.2f}"
        print(f"  {name:<40s}  {mv_s:>16s}  {mf_s:>12s}")

    # 9.27 expected behaviour: research/33 §2.2 says we want >= 50 rad/s
    # for natural finger motion (12-20 rad/s peak).  Flag any joint that
    # ships a tighter cap.
    suspect = [r for r in rows if r[1] is not None and r[1] < 50.0]
    print()
    if suspect:
        print(f"[diagnose] {len(suspect)} joint(s) have maxJointVelocity < 50 rad/s:")
        for name, mv, _ in suspect:
            print(f"  - {name}: {mv:.2f} rad/s")
        print()
        print("[diagnose] CAUSE #2 CONFIRMED.  The USD bakes a velocity cap "
              "below natural finger speed.  9.27 cfg override "
              "``velocity_limit_sim=50`` lifts this at runtime.  No code "
              "change required if 9.27 is applied.")
    else:
        cap_min = min((r[1] for r in rows if r[1] is not None), default=None)
        if cap_min is None:
            print("[diagnose] No maxJointVelocity baked into USD finger joints "
                  "(unset / inherited).  PhysX defaults to ~100 rad/s.  "
                  "The 9.27 ``velocity_limit_sim=50`` override is therefore "
                  "DEFENSIVE -- it doesn't lift a real bottleneck.  Re-run "
                  "live and look for residual lag from Cause #4 (render) or "
                  "Cause #5 (process priority).")
        else:
            print(f"[diagnose] All finger joints permit >= 50 rad/s "
                  f"(min observed = {cap_min:.2f}).  The USD itself is not "
                  f"the bottleneck; 9.27 cfg override is defensive.")
    return 0


def main() -> int:
    if len(sys.argv) > 1:
        usd_path: Optional[Path] = Path(sys.argv[1])
        if not usd_path.exists():
            print(f"[diagnose] argv[1]={usd_path} not found.")
            return 1
    else:
        usd_path = _resolve_usd_path()
        if usd_path is None:
            return 1
    return _print_finger_joint_limits(usd_path)


if __name__ == "__main__":
    raise SystemExit(main())
