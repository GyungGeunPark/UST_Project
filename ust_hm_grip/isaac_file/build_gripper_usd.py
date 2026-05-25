"""Materialise ``GR1T2_with_gripper.usd`` from the stock GR1T2 USD.

This script must be run **inside an Isaac Sim Python environment** so the
``pxr`` USD library is available.  Outside of Isaac Sim it will fail with
``ImportError: pxr``.

What it does
============
1. Opens the stock ``GR1T2_fourier_hand_6dof.usd`` from the Isaac Sim
   asset cache (the same path that ``GR1T2_HIGH_PD_CFG`` uses).
2. **Removes** the 6-DoF Fourier hand prim chain (``L_*`` and ``R_*``
   joints + bodies) — those bodies become orphans and are dropped.
3. **Adds** a ``{side}_gripper_base_link`` Xform under each
   ``{side}_wrist_pitch_link`` via a fixed joint (rigid offset of
   +5 cm along the wrist's local Z so the gripper sits at the palm
   surface).
4. **Adds** two prismatic joints per side:
   * ``{side}_gripper_finger_left_joint``  (axis +Y, range [0, 0.04])
   * ``{side}_gripper_finger_right_joint`` (axis -Y, range [0, 0.04])
   So both fingers translate in opposite Y to open/close together.
5. **Adds** simple finger boxes (visual + collision) attached to each
   prismatic joint's child link.
6. Saves to ``ust_ws/ust_hm_grip/isaac_file/GR1T2_with_gripper.usd``.

Usage
=====
::

    # Inside an Isaac Sim Python env (or via isaaclab.bat):
    ./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_gripper_usd.py

The script is idempotent — re-running it re-creates the gripper from a
clean copy of the stock USD, so any tuning changes to the geometry below
take effect on the next run.
"""

from __future__ import annotations

import os
import sys
import argparse
from pathlib import Path

# Resolve repo root so absolute paths work regardless of CWD.
_REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_PATH = _REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_gripper.usd"


# ---------------------------------------------------------------------------
# Isaac Sim 부팅 — pxr / UsdPhysics / DriveAPI / PrismaticJoint schema 가
# 등록되려면 SimulationApp 이 한 번 떠 있어야 한다.  argparse 보다 먼저
# AppLauncher 를 띄워야 하므로, 여기서 sys.argv 를 일시적으로 가로채
# AppLauncher 전용 args 만 파싱한다.  (`--headless` 강제: GUI 불필요)
# ---------------------------------------------------------------------------
def _boot_isaac_sim() -> "object":
    """Boot Isaac Sim headless and return the SimulationApp handle.

    Must be called BEFORE any ``from pxr import ...`` statement.  The conda
    env Python (``isaaclab.bat -p`` path) does NOT register the pxr USD
    libraries until ``isaacsim`` is imported and SimulationApp launched.
    """
    try:
        # AppLauncher must own its own argparse instance, otherwise it pulls
        # build_gripper_usd's --output / --source / --keep_fourier_hand into
        # its parser and aborts.  We pre-strip our flags from sys.argv,
        # let AppLauncher consume the remainder, then restore.
        from isaaclab.app import AppLauncher  # type: ignore
    except ImportError as exc:
        print(
            "[build_gripper_usd] FATAL — cannot import isaaclab.app.AppLauncher.\n"
            "  This script must be run inside an Isaac Lab Python environment.\n"
            "  Try:  ./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_gripper_usd.py\n"
            f"  Underlying error: {exc}"
        )
        sys.exit(2)

    # Build a separate parser with only AppLauncher's flags so our own
    # --output / --source / --keep_fourier_hand survive untouched in sys.argv.
    boot_parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(boot_parser)
    boot_args, remaining = boot_parser.parse_known_args()
    # Force headless — this script doesn't need a GUI.
    boot_args.headless = True

    app_launcher = AppLauncher(boot_args)
    sim_app = app_launcher.app

    # Restore sys.argv so the script's own argparse below sees only its own
    # flags (the AppLauncher ones have been consumed and removed).
    sys.argv = [sys.argv[0]] + remaining

    # Verify pxr now importable.
    try:
        from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf  # noqa: F401
    except ImportError as exc:
        print(
            "[build_gripper_usd] FATAL — SimulationApp launched but pxr still "
            f"not importable: {exc}"
        )
        sim_app.close()
        sys.exit(3)

    return sim_app


def _resolve_stock_usd_path() -> str:
    """Return the absolute path to the stock GR1T2 USD."""
    try:
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
    except ImportError:
        # Fallback: query the env var that AppLauncher sets.
        nucleus = os.environ.get("ISAAC_NUCLEUS_DIR")
        if not nucleus:
            print(
                "[build_gripper_usd] FATAL — cannot resolve ISAAC_NUCLEUS_DIR.\n"
                "  Either run inside Isaac Lab Python (isaaclab imports work)\n"
                "  or set ISAAC_NUCLEUS_DIR to the omniverse://localhost/NVIDIA/Assets/Isaac/X.Y/Isaac path."
            )
            sys.exit(3)
    else:
        nucleus = ISAAC_NUCLEUS_DIR
    return f"{nucleus}/Robots/FourierIntelligence/GR-1/GR1T2_fourier_hand_6dof/GR1T2_fourier_hand_6dof.usd"


def _setup_default_prim(stage) -> None:
    """Ensure the stage has a defaultPrim so referencing works downstream."""
    from pxr import Sdf  # type: ignore

    if not stage.GetDefaultPrim():
        # GR1T2 stock USD's defaultPrim is "GR1T2_fourier_hand_6dof".
        for top in stage.GetPseudoRoot().GetChildren():
            if top.GetTypeName() == "Xform":
                stage.SetDefaultPrim(top)
                return


def _remove_fourier_hand(stage) -> None:
    """Strip the 22-DoF Fourier hand joints from the stage.

    The stock USD has joints named L_index_proximal_joint, L_middle_..., etc.
    along with the corresponding rigid-body links.  This routine deletes
    every prim whose name matches the Fourier-hand pattern.  Visual / collision
    sub-prims attached to those bodies are dropped along with their parent.
    """
    from pxr import Sdf  # type: ignore

    fourier_prefixes = (
        "L_thumb_", "L_index_", "L_middle_", "L_pinky_", "L_ring_",
        "R_thumb_", "R_index_", "R_middle_", "R_pinky_", "R_ring_",
    )
    to_delete = []
    for prim in stage.Traverse():
        name = prim.GetName()
        if any(name.startswith(p) for p in fourier_prefixes):
            to_delete.append(prim.GetPath())
    # Delete deepest paths first so we don't invalidate child paths.
    to_delete.sort(key=lambda p: -len(p.pathString))
    for path in to_delete:
        stage.RemovePrim(path)
    print(f"[build_gripper_usd] removed {len(to_delete)} Fourier-hand prims.")


def _attach_gripper_to_wrist(stage, side: str) -> None:
    """Attach a 2-finger gripper to ``{side}_hand_pitch_link``.

    9.45: search BOTH naming conventions (env_cfg convention AND Fourier
    convention) and fail loudly when no wrist link is found.
    9.47: place gripper prims at the articulation root (sibling of all
    GR1T2 links) instead of nesting them under the wrist -- PhysX
    rejects nested RigidBody hierarchies in articulations.
    9.48: position each gripper prim at its true REST world pose
    (computed via UsdGeom.XformCache), not at the local-offset value.
    The fixed/prismatic joints declare their anchor via localPos0 in the
    parent's frame, so the URDF converter's
        parent_world * localPos0  ==  child_world * localPos1
    consistency check requires child_world (the gripper prim's world
    transform) to actually equal the joint anchor's world position.
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf  # type: ignore

    default_prim = stage.GetDefaultPrim()
    root_path = default_prim.GetPath() if default_prim else Sdf.Path("/")

    # Generate candidate names.  9.45 inspect_usd revealed GR1T2's actual
    # link convention is ``{side}_hand_pitch_link`` (not ``_wrist_``).  The
    # joint connecting ``*_hand_roll_link`` -> ``*_hand_pitch_link`` is
    # called ``*_wrist_pitch_joint`` (URDF convention: joint named after
    # the DoF, link named after the segment), which is why earlier
    # versions of this script searched for the wrong prim name and
    # silently skipped attachment.  Now we try multiple conventions.
    cap_prefix = "L" if side == "left" else "R"
    candidate_names = [
        f"{side}_hand_pitch_link",       # GR1T2 actual: left_hand_pitch_link
        f"{cap_prefix}_hand_pitch_link",  # capital variant: L_hand_pitch_link
        f"{side}_wrist_pitch_link",      # legacy guess: left_wrist_pitch_link
        f"{cap_prefix}_wrist_pitch_link",  # legacy capital: L_wrist_pitch_link
        f"{side}_hand_pitch",            # suffix-dropped variants
        f"{cap_prefix}_hand_pitch",
        f"{side}_wrist_pitch",
        f"{cap_prefix}_wrist_pitch",
    ]
    wrist_path = None
    for cand in candidate_names:
        cand_path = root_path.AppendPath(cand)
        if stage.GetPrimAtPath(cand_path):
            wrist_path = cand_path
            print(f"[build_gripper_usd] {side} wrist link found at root level: {cand!r}")
            break
    if wrist_path is None:
        # Search the whole stage for any prim whose name matches a candidate.
        for prim in stage.Traverse():
            if prim.GetName() in candidate_names:
                wrist_path = prim.GetPath()
                print(
                    f"[build_gripper_usd] {side} wrist link found via traverse: "
                    f"{prim.GetName()!r} at {wrist_path}"
                )
                break
    if wrist_path is None:
        # Diagnostic: enumerate every prim whose name contains 'wrist'.
        wrist_like = []
        for prim in stage.Traverse():
            n = prim.GetName().lower()
            if "wrist" in n:
                wrist_like.append(prim.GetName())
        raise RuntimeError(
            f"[build_gripper_usd] FATAL -- could not find {side} wrist link.\n"
            f"  Tried: {candidate_names}\n"
            f"  All wrist-related prim names in this stage: {wrist_like}\n"
            f"  Override with --source <other.usd> if the stock USD uses a\n"
            f"  different convention, or rename the wrist link in the source\n"
            f"  USD.  Silent skip would have produced an output USD without\n"
            f"  any gripper joints, crashing the teleop later (memory.md\n"
            f"  section 10.52)."
        )

    # 9.47: Gripper prims must be SIBLINGS of all other articulation links
    # (direct children of the articulation root, e.g. /GR1T2_fourier_hand_6dof/),
    # NOT nested under the wrist link.  PhysX rejects nested RigidBodyAPI
    # in articulations -- "Rigid Body ... missing xformstack reset when
    # child of another enabled rigid body in hierarchy" -- which then
    # silently corrupts joint body0/body1 resolution and reports
    # "no bodies defined" at PhysicsUSD::CreateJoint.  The fixed joint
    # below still attaches the gripper base to the wrist via body0/body1
    # references (joints refer to bodies by absolute path, independent
    # of USD hierarchy).
    # 9.48: compute the wrist's REST world transform so we can place the
    # base_link at the joint's anchor world pose.  USD's relative-Xform
    # convention means the wrist's local Xform is relative to its parent
    # (the kinematic chain), and we need wrist_world to compute the
    # gripper world pose statically.
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    wrist_prim = stage.GetPrimAtPath(wrist_path)
    wrist_world = xform_cache.GetLocalToWorldTransform(wrist_prim)
    # base_link world pose = wrist_world @ Translate(0, 0, 0.05) (offset
    # along wrist's local +Z by 5 cm so the gripper sits at the palm).
    base_world = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, 0.05)) * wrist_world

    base_path = root_path.AppendChild(f"{side}_gripper_base_link")
    base_prim = UsdGeom.Xform.Define(stage, base_path).GetPrim()
    # Set base_link's USD transform to the computed REST world pose
    # using a single matrix Xform op so we don't lose orientation.
    UsdGeom.Xformable(base_prim).ClearXformOpOrder()
    base_xform_op = UsdGeom.Xformable(base_prim).AddTransformOp()
    base_xform_op.Set(base_world)
    UsdPhysics.RigidBodyAPI.Apply(base_prim)
    UsdPhysics.MassAPI.Apply(base_prim)
    base_prim.GetAttribute("physics:mass").Set(0.05)

    # Visual proxy — a small box so the user can see where the gripper is.
    base_vis = UsdGeom.Cube.Define(stage, base_path.AppendChild("visual"))
    base_vis.CreateSizeAttr(0.04)
    base_vis.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.02))

    # Fixed joint wrist -> gripper_base.  Place at the articulation root
    # (sibling of the bodies it connects), not under either body, so it
    # follows the same flat hierarchy convention as the rest of the
    # GR1T2 articulation joints (which live at /.../joints/...).
    fj_path = root_path.AppendChild(f"{side}_gripper_attach_fixed_joint")
    fj = UsdPhysics.FixedJoint.Define(stage, fj_path)
    fj.CreateBody0Rel().SetTargets([wrist_path])
    fj.CreateBody1Rel().SetTargets([base_path])
    fj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.05))      # wrist +Z 5 cm
    fj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # Finger links + prismatic joints.  Two fingers translate in opposite +Y/-Y.
    # 9.47: finger links live at the articulation root level (sibling of
    # base_link and all GR1T2 bodies), not nested under base_link.  See
    # base_link comment above for the rationale.
    for finger_side, sign in (("left", +1.0), ("right", -1.0)):
        finger_link_path = root_path.AppendChild(
            f"{side}_gripper_finger_{finger_side}_link"
        )
        finger_link = UsdGeom.Xform.Define(stage, finger_link_path).GetPrim()
        # 9.48: compute the finger link's REST world pose as
        # base_world @ Translate(0, sign*0.02, 0) -- prismatic joint's
        # rest anchor in the base's local frame, transformed to world
        # so the URDF transform-consistency check
        #   parent_world * localPos0 == child_world * localPos1
        # succeeds with localPos0=(0, sign*0.02, 0) and localPos1=(0,0,0).
        finger_world = (
            Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, sign * 0.02, 0.0))
            * base_world
        )
        UsdGeom.Xformable(finger_link).ClearXformOpOrder()
        finger_xform_op = UsdGeom.Xformable(finger_link).AddTransformOp()
        finger_xform_op.Set(finger_world)
        UsdPhysics.RigidBodyAPI.Apply(finger_link)
        UsdPhysics.MassAPI.Apply(finger_link)
        finger_link.GetAttribute("physics:mass").Set(0.02)

        # Visual + collision: a thin box 0.06 m long, 0.01 m wide, 0.04 m tall.
        finger_vis = UsdGeom.Cube.Define(stage, finger_link_path.AppendChild("visual"))
        finger_vis.CreateSizeAttr(1.0)
        # Scale to a thin finger shape and translate so finger sticks out
        # toward +Z (the gripping direction).
        UsdGeom.XformCommonAPI(finger_vis).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.05))
        UsdGeom.XformCommonAPI(finger_vis).SetScale(Gf.Vec3f(0.01, 0.04, 0.06))
        UsdPhysics.CollisionAPI.Apply(finger_vis.GetPrim())

        # Prismatic joint base -> finger_link.  Place at the articulation
        # root level (sibling of bodies it connects), same hierarchy
        # convention as the wrist fixed joint above.
        pj_path = root_path.AppendChild(
            f"{side}_gripper_finger_{finger_side}_joint"
        )
        pj = UsdPhysics.PrismaticJoint.Define(stage, pj_path)
        pj.CreateBody0Rel().SetTargets([base_path])
        pj.CreateBody1Rel().SetTargets([finger_link_path])
        pj.CreateAxisAttr("Y")
        pj.CreateLowerLimitAttr(0.0)
        pj.CreateUpperLimitAttr(0.04)
        # Each finger sits at sign * 0.02 m local Y when fully open.
        pj.CreateLocalPos0Attr(Gf.Vec3f(0.0, sign * 0.02, 0.0))
        pj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        pj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        pj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        # Drive: position-controlled with stiffness/damping that match the
        # ImplicitActuator override in kitchen_sorting_gr1t2_gripper_env_cfg.
        drive = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateMaxForceAttr(200.0)
        drive.CreateTargetPositionAttr(0.04)
        drive.CreateDampingAttr(100.0)
        drive.CreateStiffnessAttr(2000.0)

def main() -> None:
    sim_app = _boot_isaac_sim()
    try:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--output", "-o",
            default=str(DEFAULT_OUTPUT_PATH),
            help=f"Output USD path (default: {DEFAULT_OUTPUT_PATH})",
        )
        parser.add_argument(
            "--source",
            default=None,
            help="Source GR1T2 USD path; defaults to ISAAC_NUCLEUS_DIR copy.",
        )
        parser.add_argument(
            "--keep_fourier_hand",
            action="store_true",
            help="Don't strip the Fourier hand prims (debugging).",
        )
        args = parser.parse_args()

        from pxr import Usd, UsdGeom  # type: ignore

        src = args.source or _resolve_stock_usd_path()
        out = os.path.abspath(args.output)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        print(f"[build_gripper_usd] source = {src!r}")
        print(f"[build_gripper_usd] output = {out!r}")

        src_stage = Usd.Stage.Open(src)
        if not src_stage:
            print(f"[build_gripper_usd] FATAL -- cannot open source USD {src!r}.")
            sys.exit(4)

        flat_layer = src_stage.Flatten()
        out_stage = Usd.Stage.Open(flat_layer)
        out_stage.GetRootLayer().Export(out)

        stage = Usd.Stage.Open(out)
        _setup_default_prim(stage)

        if not args.keep_fourier_hand:
            _remove_fourier_hand(stage)

        for side in ("left", "right"):
            _attach_gripper_to_wrist(stage, side)

        stage.GetRootLayer().Save()
        print(
            f"[build_gripper_usd] DONE -- gripper-equipped GR1T2 USD written to {out!r}."
        )
        print(
            "  Next: launch run_teleop.py -- the env_cfg auto-resolves to this path."
        )
    finally:
        sim_app.close()


if __name__ == "__main__":
    main()
