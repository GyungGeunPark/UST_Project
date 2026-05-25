"""Materialise ``GR1T2_with_robotiq.usd`` from stock GR1T2 + Robotiq 2F-85.

This is the **option A** counterpart of ``build_gripper_usd.py`` (option B,
custom box-finger gripper).  See ``ust_ws/research/43.
robotiq_2f85_optionA_migration_design_guide.md`` for the design.

What it does
============
1. Opens the stock GR1T2 USD from Isaac Sim cache.
2. Removes the 6-DoF Fourier hand prims (same as 9.45 build_gripper_usd).
3. **For each side (left, right)**:
   a. Copies the Robotiq 2F-85 subtree (``/Robotiq_2F_85/Robotiq_2F_85``,
      from the local stock cache) under
      ``/<gr1t2>/<side>_robotiq_arg2f_85`` via ``Sdf.CopySpec``.
   b. Removes the gripper's ``PhysicsArticulationRootAPI`` so the merged
      asset has exactly one articulation root (the GR1T2's).
   c. Anchors the gripper base to ``{side}_hand_pitch_link`` via a
      ``UsdPhysics.FixedJoint`` (with optional translation / rotation to
      match the GR1T2 wrist's palm-out axis to the Robotiq mount-face axis).
   d. Adds a ``{side}_gripper_tcp_link`` Xform fixed-jointed to the
      Robotiq ``base_link`` at +0.150 m along the gripper's local +Z
      (TCP midpoint between fingertip pads when closed) — used as the
      Pink IK target.
4. Saves to ``ust_ws/ust_hm_grip/isaac_file/GR1T2_with_robotiq.usd``.

Source assets are pre-downloaded into
``ust_ws/ust_hm_grip/isaac_file/robotiq/`` (Robotiq_2F_85_edit.usd +
configuration/ + payloads/ + parts/).

Usage
=====
::

    ./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_robotiq_usd.py
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_PATH = _REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd"
DEFAULT_ROBOTIQ_USD = (
    _REPO_ROOT
    / "ust_ws"
    / "ust_hm_grip"
    / "isaac_file"
    / "robotiq"
    / "Robotiq_2F_85_edit.usd"
)

# +Z offset (in robotiq base-link local frame) from the mount face to the
# tip midpoint when the gripper is closed.  Robotiq datasheet body height
# 162.8 mm minus fingertip pad thickness; community implementations use
# ~0.150 m.  Calibrate after first visual inspection.
GRIPPER_TCP_OFFSET_Z = 0.150
# Translation along wrist +Z for attaching the gripper mount face.  Zero
# = mount face flush with wrist link origin.
GRIPPER_ATTACH_OFFSET_Z = 0.0


def _wrist_to_gripper_rotation(side: str):
    """180° rotation around the wrist's local Y axis (= world Y axis
    at bind pose, since both GR1T2 wrists have identity world rotation
    per ``scripts/inspect_wrist_frame.py``).

    Effect on the Robotiq's local axes in the wrist frame:
      gripper +X → wrist -X (forward → backward)
      gripper +Y → wrist +Y (unchanged — Y is the rotation axis)
      gripper +Z → wrist -Z (up → down; the fingertip-out direction
                              flips, so the gripper hangs from the
                              wrist with fingertips pointing down)

    At bind pose, this puts both grippers' fingertips along world -Z
    (down) — the natural palm-down posture matching the original
    Fourier hand's L_/R_*_proximal_link directions (see
    ``inspect_wrist_frame.py`` output: index/middle delta from wrist
    is dominated by -Z).

    The user explicitly directed "쳐리퍼들을 월드 좌표 Y축 회전을
    180도 회전" — both grippers, same rotation, world Y axis.  Since
    the wrist frame coincides with world frame at bind pose, the
    rotation can be authored once in the wrist local frame and applied
    identically to both sides.
    """
    from pxr import Gf  # type: ignore
    if side not in ("left", "right"):
        raise ValueError(f"unknown side {side!r}; expected 'left' or 'right'")
    return Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), 180.0)


def _rotation_to_quatf(rot):
    """Convert a ``Gf.Rotation`` to a ``Gf.Quatf`` (real, im_x, im_y, im_z)."""
    from pxr import Gf  # type: ignore
    q = rot.GetQuat()
    im = q.GetImaginary()
    return Gf.Quatf(float(q.GetReal()), float(im[0]), float(im[1]), float(im[2]))


def _boot_isaac_sim() -> "object":
    """Boot Isaac Sim headless and return the SimulationApp handle."""
    try:
        from isaaclab.app import AppLauncher  # type: ignore
    except ImportError as exc:
        print(
            "[build_robotiq_usd] FATAL — cannot import isaaclab.app.AppLauncher.\n"
            f"  Underlying error: {exc}"
        )
        sys.exit(2)

    boot_parser = argparse.ArgumentParser(add_help=False)
    AppLauncher.add_app_launcher_args(boot_parser)
    boot_args, remaining = boot_parser.parse_known_args()
    boot_args.headless = True
    app_launcher = AppLauncher(boot_args)
    sim_app = app_launcher.app
    sys.argv = [sys.argv[0]] + remaining

    try:
        from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf  # noqa: F401
    except ImportError as exc:
        print(f"[build_robotiq_usd] FATAL — pxr unavailable: {exc}")
        sim_app.close()
        sys.exit(3)

    return sim_app


def _resolve_stock_gr1t2_usd_path() -> str:
    try:
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
    except ImportError:
        nucleus = os.environ.get("ISAAC_NUCLEUS_DIR")
        if not nucleus:
            print("[build_robotiq_usd] FATAL — cannot resolve ISAAC_NUCLEUS_DIR.")
            sys.exit(3)
    else:
        nucleus = ISAAC_NUCLEUS_DIR
    return f"{nucleus}/Robots/FourierIntelligence/GR-1/GR1T2_fourier_hand_6dof/GR1T2_fourier_hand_6dof.usd"


def _setup_default_prim(stage) -> None:
    from pxr import Sdf  # type: ignore

    if not stage.GetDefaultPrim():
        for top in stage.GetPseudoRoot().GetChildren():
            if top.GetTypeName() == "Xform":
                stage.SetDefaultPrim(top)
                return


def _remove_fourier_hand(stage) -> None:
    """Strip the 22-DoF Fourier hand joints + bodies — same as 9.45."""
    fourier_prefixes = (
        "L_thumb_", "L_index_", "L_middle_", "L_pinky_", "L_ring_",
        "R_thumb_", "R_index_", "R_middle_", "R_pinky_", "R_ring_",
    )
    to_delete = []
    for prim in stage.Traverse():
        name = prim.GetName()
        if any(name.startswith(p) for p in fourier_prefixes):
            to_delete.append(prim.GetPath())
    to_delete.sort(key=lambda p: -len(p.pathString))
    for path in to_delete:
        stage.RemovePrim(path)
    print(f"[build_robotiq_usd] removed {len(to_delete)} Fourier-hand prims.")


def _strip_hand_pitch_link_geometry(stage, side: str) -> None:
    """Remove the residual GR1T2 wrist-back visual + collision meshes
    inside ``{side}_hand_pitch_link``.

    The GR1T2 stock USD carries a hand-shaped visual + collision under
    ``hand_pitch_link/visuals`` and ``hand_pitch_link/collisions`` —
    originally these depicted the back of the Fourier hand.  After we
    bolt the Robotiq 2F-85 on top, that hand mesh is co-located with
    (i.e. INSIDE) the gripper base_link and shows up between the
    finger pads, looking like an "object stuck in the gripper".

    The mesh children are pure scene geometry:
      * ``visuals/...``     — render-only Mesh prims, no joints or APIs
        beyond UsdGeom display data
      * ``collisions/...``  — PhysX collision shapes that approximated
        the original hand; they don't terminate any joint, and the
        Robotiq's own collision shapes already cover the same volume
        once the gripper is attached.

    What stays untouched (these ARE physics-critical):
      * ``hand_pitch_link`` itself      — the rigid body that the
        wrist_pitch_joint terminates at (kinematic chain node).
      * ``hand_pitch_link/end_effector_link`` — the EEF frame child
        (no visual, just an Xform reference) used by various code
        paths to find the hand tip.
      * The body's ``PhysicsRigidBodyAPI`` / ``PhysicsMassAPI`` —
        without these, the articulation can't include this segment
        and the wrist joints wouldn't propagate to the gripper.

    Hand rotation (wrist_yaw / wrist_roll / wrist_pitch joints) is
    entirely a function of the articulation chain — removing visuals
    and collision meshes has zero effect on the joint values.
    """
    wrist_path = _find_wrist_path(stage, side)
    for child_name in ("visuals", "collisions"):
        child_path = wrist_path.AppendChild(child_name)
        if stage.GetPrimAtPath(child_path):
            stage.RemovePrim(child_path)
            print(
                f"[build_robotiq_usd]   stripped {wrist_path.name}/{child_name} "
                f"(GR1T2 wrist's residual hand geometry — collision still "
                f"provided by the Robotiq base_link mesh)"
            )


def _find_wrist_path(stage, side):
    """Same convention as build_gripper_usd 9.45 — match against multiple
    candidate names, fail loudly on miss."""
    default_prim = stage.GetDefaultPrim()
    root_path = default_prim.GetPath()
    cap_prefix = "L" if side == "left" else "R"
    candidates = [
        f"{side}_hand_pitch_link",
        f"{cap_prefix}_hand_pitch_link",
        f"{side}_wrist_pitch_link",
        f"{cap_prefix}_wrist_pitch_link",
    ]
    for cand in candidates:
        cand_path = root_path.AppendPath(cand)
        if stage.GetPrimAtPath(cand_path):
            return cand_path
    for prim in stage.Traverse():
        if prim.GetName() in candidates:
            return prim.GetPath()
    raise RuntimeError(
        f"[build_robotiq_usd] FATAL — cannot find {side} wrist link.\n"
        f"  Candidates tried: {candidates}"
    )


def _attach_robotiq(stage, side: str, robotiq_layer, robotiq_layer_path: str) -> None:
    """Attach a Robotiq 2F-85 subtree to ``{side}_hand_pitch_link``.

    Uses ``Sdf.CopySpec`` to import the gripper articulation subtree from
    ``Robotiq_2F_85_edit.usd``'s ``/Robotiq_2F_85/Robotiq_2F_85`` into the
    humanoid USD as a new prim ``{side}_robotiq_arg2f_85``.  Then:
      - removes that prim's ``PhysicsArticulationRootAPI`` (humanoid keeps
        the only articulation root)
      - adds a ``UsdPhysics.FixedJoint`` (wrist body → gripper base_link)
      - adds a ``{side}_gripper_tcp_link`` fixed-jointed to gripper
        base_link at +GRIPPER_TCP_OFFSET_Z along local Z (Pink IK target)
      - widens outer-knuckle joint limits to symmetric ±47° (stock is
        [0°, 47°], but ``gearing=-1`` followers need negative angles)
      - re-applies ``PhysxMimicJointAPI`` schemas + adds explicit drives
        on all 6 joints so the linkage works even if mimic constraint
        registration fails under the Isaac Sim 5.1 known-issue (some
        follower links not tracking the lead).  The Isaac Lab actuator
        cfg overrides these stiffness/damping at runtime.
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf  # type: ignore

    default_prim = stage.GetDefaultPrim()
    root_path = default_prim.GetPath()
    wrist_path = _find_wrist_path(stage, side)
    print(f"[build_robotiq_usd] {side} wrist link: {wrist_path}")

    # 0) Strip the GR1T2 wrist's residual hand-shaped visual + collision
    #    meshes.  These appeared between the gripper finger pads in the
    #    user's screenshot ("an object inside the gripper").  Removing
    #    them is purely cosmetic / collision-shape cleanup — the rigid
    #    body and articulation node stay, so hand rotation is unaffected.
    #    See ``_strip_hand_pitch_link_geometry`` for the full rationale.
    _strip_hand_pitch_link_geometry(stage, side)

    # 1) Compute wrist_world up-front — we'll use it for both the gripper
    #    container's transform op AND the TCP placement.
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    wrist_prim = stage.GetPrimAtPath(wrist_path)
    wrist_world = xform_cache.GetLocalToWorldTransform(wrist_prim)
    wrist_translation = wrist_world.ExtractTranslation()

    # 1a) Apply per-side orientation correction (design-guide #43 §7.2):
    #     the gripper's fingertip direction (gripper local +Z) is rotated
    #     to align with the arm's continuation direction in world space —
    #     wrist +Y for LEFT, wrist -Y for RIGHT.  Without this, identity
    #     attachment makes the gripper point "up" from the wrist in bind
    #     pose (gripper +Z = wrist +Z = world +Z) rather than outward
    #     along the arm.  See ``_wrist_to_gripper_rotation`` docstring.
    gripper_rotation = _wrist_to_gripper_rotation(side)

    # 2) FLATTEN-AND-COPY the Robotiq subtree into our stage so we can
    #    rename joints with a per-side prefix.  References preserve the
    #    source's prim names, which makes both grippers share leaf names
    #    like ``finger_joint`` and crashes the USD→URDF converter with
    #    "joint 'Joints_finger_joint' is not unique."
    dst_path = root_path.AppendChild(f"{side}_robotiq_arg2f_85")
    if stage.GetPrimAtPath(dst_path):
        stage.RemovePrim(dst_path)

    robotiq_stage = Usd.Stage.Open(robotiq_layer_path)
    # CRITICAL: turn off instancing on every prim in the Robotiq stage
    # BEFORE flattening.  The stock USD marks every ``visuals`` prim as
    # ``instanceable=True``, so ``Stage.Flatten()`` collapses the unique
    # mesh data into ``/Flattened_Prototype_N`` top-level prims that the
    # individual visuals reference.  ``Sdf.CopySpec`` only copies the
    # subtree under ``/Robotiq_2F_85/Robotiq_2F_85`` — the prototypes
    # at the source layer's root are left behind.  In the merged USD
    # those references resolve against whatever happens to live at
    # ``/Flattened_Prototype_N`` in the destination stage, which is a
    # GR1T2 body part's mesh prototype (the visible bug: the gripper
    # visual renders as a thigh-roll mesh).  Un-instancing makes
    # Flatten() inline the mesh data into the visuals prim itself,
    # so each visuals prim becomes self-contained and CopySpec carries
    # the actual gripper meshes across.
    for instanceable_prim in robotiq_stage.Traverse():
        if instanceable_prim.IsInstanceable():
            instanceable_prim.SetInstanceable(False)
    robotiq_flat_layer = robotiq_stage.Flatten()
    src_path = Sdf.Path("/Robotiq_2F_85/Robotiq_2F_85")
    Sdf.CopySpec(
        robotiq_flat_layer, src_path,
        stage.GetRootLayer(), dst_path,
    )
    dst_prim = stage.GetPrimAtPath(dst_path)

    # 3) Set the container's WORLD transform.  In bind pose, wrist.world
    #    has identity rotation (verified by inspect_wrist_frame.py),
    #    so the container's world transform = gripper_rotation applied,
    #    then translated to the wrist's world position.  The base_link
    #    is at identity within the container, so it inherits this same
    #    world transform — its world rotation = gripper_rotation, which
    #    must equal the FixedJoint constraint below for the URDF
    #    converter's "joint transforms are consistent" check to pass.
    container_world = Gf.Matrix4d()
    container_world.SetRotate(gripper_rotation)
    container_world.SetTranslateOnly(wrist_translation)
    UsdGeom.Xformable(dst_prim).ClearXformOpOrder()
    UsdGeom.Xformable(dst_prim).AddTransformOp().Set(container_world)

    # 4) Rename every joint under <dst>/Joints/* to add a "<side>_"
    #    prefix, and rewrite each follower's PhysxMimicJointAPI
    #    referenceJoint rel-target so it points at the renamed lead.
    joints_scope = stage.GetPrimAtPath(dst_path.AppendChild("Joints"))
    rename_map = {}
    if joints_scope.IsValid():
        for child in list(joints_scope.GetChildren()):
            old_name = child.GetName()
            new_name = f"{side}_{old_name}"
            old_path = child.GetPath()
            new_path = old_path.GetParentPath().AppendChild(new_name)
            Sdf.CopySpec(
                stage.GetRootLayer(), old_path,
                stage.GetRootLayer(), new_path,
            )
            stage.RemovePrim(old_path)
            rename_map[old_path] = new_path

    # 4a) Rewrite PhysxMimicJointAPI:rotX/rotZ referenceJoint rel-targets
    #     to point at the renamed lead.  Stock USD references the lead
    #     by path; our CopySpec preserved the old paths so we need to
    #     update them.
    joints_scope = stage.GetPrimAtPath(dst_path.AppendChild("Joints"))
    if joints_scope.IsValid():
        for child in joints_scope.GetChildren():
            for rel_name in (
                "physics:body0", "physics:body1",
                "physxMimicJoint:rotX:referenceJoint",
                "physxMimicJoint:rotY:referenceJoint",
                "physxMimicJoint:rotZ:referenceJoint",
                "physxMimicJoint:transX:referenceJoint",
                "physxMimicJoint:transY:referenceJoint",
                "physxMimicJoint:transZ:referenceJoint",
            ):
                rel = child.GetRelationship(rel_name)
                if rel and rel.IsValid():
                    new_targets = []
                    changed = False
                    for tgt in rel.GetTargets():
                        new_tgt = rename_map.get(Sdf.Path(tgt), tgt)
                        if new_tgt != tgt:
                            changed = True
                        new_targets.append(new_tgt)
                    if changed:
                        rel.SetTargets(new_targets)

    # 4b) FixedJoint under outer_finger (left_outer_finger/FixedJoint and
    #     right_outer_finger/FixedJoint) also have body0/body1 rel —
    #     they were already copied as children of those bodies and use
    #     paths within the same subtree, so they don't reference joints
    #     by path; no rewrite needed.  Sanity-check by listing.

    # 4c) Widen outer-knuckle joint limits from stock [0°, 47°] to
    #     symmetric [-47°, 47°].  The mimic-follower outer_knuckle on
    #     the side opposite to the lead (``{side}_right_outer_knuckle_joint``)
    #     has ``gearing=-1`` — when the lead goes to +47°, this follower
    #     needs to go to -47°.  Stock USD's [0, 47] clamps it to 0,
    #     breaking the parallel-grasp linkage.  The 5 follower 4-bar
    #     joints (``*_inner_finger_*_joint``) already have full ±180°
    #     range in stock so they don't need this fix.
    joints_scope = stage.GetPrimAtPath(dst_path.AppendChild("Joints"))
    if joints_scope.IsValid():
        for child in joints_scope.GetChildren():
            jn = child.GetName()
            if jn.endswith("right_outer_knuckle_joint"):
                low_attr = child.GetAttribute("physics:lowerLimit")
                if low_attr and low_attr.IsValid():
                    low_attr.Set(-47.0)
                    print(
                        f"[build_robotiq_usd]   widened limits on {jn} to "
                        f"[-47, 47] (was [0, 47]); follower w/ gearing=-1"
                    )

    # 4d) Re-apply PhysxMimicJointAPI + add explicit drives on all 6
    #     revolute joints per side.  Reasons:
    #       1. Stock USD has drives on the followers stripped
    #          (``delete apiSchemas = ["PhysicsDriveAPI:angular"]``) —
    #          followers rely solely on the mimic constraint.  Under
    #          Isaac Sim 5.1's known issue ("some follower links don't
    #          track the lead"), the unconstrained followers float free
    #          and gravity deforms the gripper into a chain-like mess.
    #       2. Adding drives back gives Isaac Lab's actuator cfg a
    #          handle to apply per-joint stiffness/damping at runtime
    #          via ``ImplicitActuatorCfg(joint_names_expr=[...])``.
    #       3. Re-applying PhysxMimicJointAPI is defensive: even though
    #          the schema is already in ``GetAppliedSchemas()`` after
    #          CopySpec, re-applying ensures PhysX's constraint
    #          registration runs against the renamed joint paths.
    try:
        from pxr import PhysxSchema  # type: ignore
        _have_physx_schema = True
    except ImportError:
        _have_physx_schema = False
        print(
            "[build_robotiq_usd]   WARNING: pxr.PhysxSchema unavailable; "
            "skipping explicit re-apply of PhysxMimicJointAPI"
        )

    joints_scope = stage.GetPrimAtPath(dst_path.AppendChild("Joints"))
    if joints_scope.IsValid():
        for child in joints_scope.GetChildren():
            # Add PhysicsDriveAPI:angular if absent, then author
            # zero-target / placeholder defaults.  Isaac Lab's
            # ImplicitActuatorCfg.stiffness/damping override these at
            # startup, but PhysX needs the drive API present so the
            # override has something to bind to.  ``maxForce`` is the
            # USD-level effort cap; Isaac Lab's ``effort_limit_sim``
            # cannot exceed it, so we raise this to 500 N·m here to
            # let env_cfg's lead actuator deliver enough torque to
            # overcome the 4-bar linkage + PhysxMimicJointAPI static
            # resistance.  (12th session: with maxForce=50 the lead
            # joint hard-stalled at ~3° vs +45° target — see
            # test_robotiq_close.py diagnostic.)
            drive_api = UsdPhysics.DriveAPI.Apply(child, "angular")
            target_attr = child.GetAttribute("drive:angular:physics:targetPosition")
            if not (target_attr and target_attr.IsValid()
                    and target_attr.HasAuthoredValue()):
                drive_api.CreateTargetPositionAttr(0.0)
            stiff_attr = child.GetAttribute("drive:angular:physics:stiffness")
            if stiff_attr and stiff_attr.IsValid():
                stiff_attr.Set(50.0)        # overridden by env_cfg per-side
            damp_attr = child.GetAttribute("drive:angular:physics:damping")
            if damp_attr and damp_attr.IsValid():
                damp_attr.Set(5.0)          # overridden by env_cfg per-side
            maxf_attr = child.GetAttribute("drive:angular:physics:maxForce")
            if maxf_attr and maxf_attr.IsValid():
                maxf_attr.Set(500.0)        # 12th session: raised from 50 N·m

            # 12th session — stock Robotiq USD bakes
            # ``physxJoint:maxJointVelocity = 146.46`` (deg/s = 2.56 rad/s)
            # on the lead `finger_joint` (followers have 10000 deg/s).
            # Even at full cap the lead should close in ~0.3 s, but the
            # implicit-solver interaction with a 1e-4 kg·m² armature and
            # PhysxMimicJointAPI on six closed-loop followers causes a
            # hard stall around 2-3°.  Raising maxJointVelocity to match
            # the followers (10000 deg/s) and bumping armature to 0.01
            # (100× original) gives the LCP solver enough numerical
            # inertia to converge.  test_robotiq_close.py + visual
            # rendering both confirm the gripper now fully closes.
            mvel_attr = child.GetAttribute("physxJoint:maxJointVelocity")
            if mvel_attr and mvel_attr.IsValid():
                mvel_attr.Set(10000.0)
            arm_attr = child.GetAttribute("physxJoint:armature")
            if arm_attr and arm_attr.IsValid():
                arm_attr.Set(0.01)
            elif _have_physx_schema:
                # author armature if absent (followers have it; verify)
                try:
                    PhysxSchema.PhysxJointAPI.Apply(child)
                    child.CreateAttribute(
                        "physxJoint:armature",
                        Sdf.ValueTypeNames.Float,
                    ).Set(0.01)
                except Exception:  # noqa: BLE001
                    pass

            # Re-apply PhysxMimicJointAPI on followers (defensive — the
            # API is preserved through CopySpec but re-applying ensures
            # PhysX's constraint registration is called against the
            # renamed joint path).
            if _have_physx_schema:
                for axis in ("rotX", "rotZ"):
                    api_name = f"PhysxMimicJointAPI:{axis}"
                    if api_name in child.GetAppliedSchemas():
                        # Force a re-author by removing then re-adding;
                        # noop semantically if PhysX caches by joint path.
                        child.RemoveAppliedSchema(api_name)
                        PhysxSchema.PhysxMimicJointAPI.Apply(child, axis)
            print(f"[build_robotiq_usd]   {side}/{child.GetName()}: drive + mimic re-authored")

    # 4e) Set mass + inertia on base_link and outer_knuckles.  Stock
    #     Robotiq USD applies ``PhysicsRigidBodyAPI`` but not
    #     ``PhysicsMassAPI`` on these bodies — PhysX then logs
    #     ``possibly invalid inertia tensor … and a negative mass``
    #     and substitutes a small-sphere approximation.  Falling back
    #     to a 1-kg sphere on base_link skews the reaction torques
    #     against the wrist FixedJoint and the outer_knuckle revolute
    #     joints, contributing to the chain-like deformation seen
    #     before the fix.  Values follow the Robotiq 2F-85 datasheet
    #     total mass of 0.925 kg (see design-guide #43 §2.3), with
    #     the bulk on base_link and small distributions on the
    #     outer_knuckles symmetric to the inner_knuckles' 0.027 kg.
    body_mass_map = {
        "base_link":           (0.600, (0.001, 0.001, 0.001)),
        "left_outer_knuckle":  (0.050, (5e-5, 5e-5, 5e-5)),
        "right_outer_knuckle": (0.050, (5e-5, 5e-5, 5e-5)),
    }
    for body_name, (mass_kg, diag_inertia) in body_mass_map.items():
        body_prim = stage.GetPrimAtPath(dst_path.AppendChild(body_name))
        if not (body_prim and body_prim.IsValid()):
            continue
        mass_api = UsdPhysics.MassAPI.Apply(body_prim)
        mass_api.CreateMassAttr(mass_kg)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag_inertia))
        print(
            f"[build_robotiq_usd]   {side}/{body_name}: mass={mass_kg} kg, "
            f"diagInertia={diag_inertia}"
        )

    # 3) Remove the gripper's ArticulationRootAPI — single root must live
    #    on the humanoid, not on the gripper subtree.
    if dst_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        dst_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    # Also recurse — Isaac Sim stock applies it inside the nested
    # /Robotiq_2F_85/Robotiq_2F_85 only, but be safe.
    for sub in Usd.PrimRange(dst_prim):
        if sub.HasAPI(UsdPhysics.ArticulationRootAPI):
            sub.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            print(
                f"[build_robotiq_usd]   removed ArticulationRootAPI from "
                f"{sub.GetPath()}"
            )

    # 4) FixedJoint anchoring gripper base to wrist body.  Gripper base
    #    is the referenced subtree's base_link.
    #
    #    Frame alignment: with localRot0 = gripper_rotation (wrist-side
    #    of the joint, rotates the wrist frame into the gripper frame)
    #    and localRot1 = identity (gripper-side), the FixedJoint's two
    #    anchor frames coincide when base_link's world rotation =
    #    wrist's world rotation * gripper_rotation.  This matches the
    #    container.world rotation set above (= identity * gripper_rotation
    #    in bind pose), so the URDF converter's joint-consistency check
    #    passes.  At runtime, base_link tracks the wrist with the
    #    constant relative rotation = gripper_rotation.
    base_link_path = dst_path.AppendChild("base_link")
    fj_path = root_path.AppendChild(f"{side}_robotiq_attach_fixed_joint")
    fj = UsdPhysics.FixedJoint.Define(stage, fj_path)
    fj.CreateBody0Rel().SetTargets([wrist_path])
    fj.CreateBody1Rel().SetTargets([base_link_path])
    fj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, GRIPPER_ATTACH_OFFSET_Z))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fj.CreateLocalRot0Attr(_rotation_to_quatf(gripper_rotation))
    fj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # 5) TCP frame — fixed-jointed child of base_link at +0.150 m local Z.
    #    Used as Pink IK FrameTask target.  In bind pose:
    #      base_link.world.rotation = gripper_rotation
    #      base_link.world.translation = wrist_translation
    #      TCP.world = base_link.world * Translate(0,0,GRIPPER_TCP_OFFSET_Z)
    #    Apply gripper_rotation to (0,0,+TCP_offset): the 180° around
    #    (1,1,0)/√2 axis flips Z, so the offset becomes (0,0,-TCP_offset)
    #    in the world frame.  TCP.world.translation =
    #    wrist_translation + (0, 0, -GRIPPER_TCP_OFFSET_Z).
    rotation_matrix4d = Gf.Matrix4d().SetRotate(gripper_rotation)
    tcp_offset_local = Gf.Vec3d(0.0, 0.0, GRIPPER_TCP_OFFSET_Z)
    tcp_offset_world = rotation_matrix4d.TransformDir(tcp_offset_local)
    tcp_world = Gf.Matrix4d()
    tcp_world.SetRotate(gripper_rotation)  # same orientation as base_link
    tcp_world.SetTranslateOnly(Gf.Vec3d(
        wrist_translation[0] + tcp_offset_world[0],
        wrist_translation[1] + tcp_offset_world[1],
        wrist_translation[2] + tcp_offset_world[2],
    ))

    tcp_path = root_path.AppendChild(f"{side}_gripper_tcp_link")
    tcp_prim = UsdGeom.Xform.Define(stage, tcp_path).GetPrim()
    UsdGeom.Xformable(tcp_prim).ClearXformOpOrder()
    UsdGeom.Xformable(tcp_prim).AddTransformOp().Set(tcp_world)
    UsdPhysics.RigidBodyAPI.Apply(tcp_prim)
    UsdPhysics.MassAPI.Apply(tcp_prim)
    tcp_prim.GetAttribute("physics:mass").Set(1.0e-4)

    tcp_fj_path = root_path.AppendChild(f"{side}_gripper_tcp_fixed_joint")
    tcp_fj = UsdPhysics.FixedJoint.Define(stage, tcp_fj_path)
    tcp_fj.CreateBody0Rel().SetTargets([base_link_path])
    tcp_fj.CreateBody1Rel().SetTargets([tcp_path])
    tcp_fj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, GRIPPER_TCP_OFFSET_Z))
    tcp_fj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    tcp_fj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    tcp_fj.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    print(
        f"[build_robotiq_usd]   {side} attached: wrist={wrist_path.name} → "
        f"base_link, TCP at +{GRIPPER_TCP_OFFSET_Z*1000:.0f} mm Z"
    )


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
            "--robotiq",
            default=str(DEFAULT_ROBOTIQ_USD),
            help=f"Local Robotiq 2F-85 edit.usd (default: {DEFAULT_ROBOTIQ_USD})",
        )
        parser.add_argument(
            "--keep_fourier_hand",
            action="store_true",
            help="Don't strip the Fourier hand prims (debugging).",
        )
        args = parser.parse_args()

        from pxr import Usd  # type: ignore

        src = args.source or _resolve_stock_gr1t2_usd_path()
        out = os.path.abspath(args.output)
        robotiq_usd = os.path.abspath(args.robotiq)
        if not os.path.exists(robotiq_usd):
            print(
                f"[build_robotiq_usd] FATAL — Robotiq USD not found at "
                f"{robotiq_usd!r}.\n"
                f"  Download from Isaac Sim 5.1 S3:\n"
                f"  Assets/Isaac/5.1/Isaac/Robots/Robotiq/2F-85/"
            )
            sys.exit(4)

        os.makedirs(os.path.dirname(out), exist_ok=True)
        print(f"[build_robotiq_usd] gr1t2  = {src!r}")
        print(f"[build_robotiq_usd] robotiq = {robotiq_usd!r}")
        print(f"[build_robotiq_usd] output  = {out!r}")

        # 1) Flatten GR1T2 stock USD to our editable output
        src_stage = Usd.Stage.Open(src)
        if not src_stage:
            print(f"[build_robotiq_usd] FATAL — cannot open {src!r}.")
            sys.exit(5)
        flat = src_stage.Flatten()
        out_stage = Usd.Stage.Open(flat)
        out_stage.GetRootLayer().Export(out)

        stage = Usd.Stage.Open(out)
        _setup_default_prim(stage)
        if not args.keep_fourier_hand:
            _remove_fourier_hand(stage)

        # 2) Reference the Robotiq stock USD on each wrist.  The
        #    Robotiq_2F_85_edit.usd already has Physx_Mimic variant
        #    selected by default — when we reference it, the variant
        #    selection follows.
        for side in ("left", "right"):
            _attach_robotiq(stage, side, None, robotiq_usd)

        stage.GetRootLayer().Save()
        print(f"[build_robotiq_usd] DONE — wrote {out!r}.")
        print("  Next: run inspect_usd_robotiq.py to verify the attached USD,")
        print("        then build env_cfg to use GR1T2_with_robotiq.usd.")
    finally:
        sim_app.close()


if __name__ == "__main__":
    main()
