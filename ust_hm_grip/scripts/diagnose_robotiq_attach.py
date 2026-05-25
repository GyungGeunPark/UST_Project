"""Diagnose Robotiq attachment issues in GR1T2_with_robotiq.usd.

Prints:
1. body0/body1 rel targets and their validity (do the prims exist?)
2. PhysxMimicJointAPI applied schemas + referenceJoint rel targets and validity
3. Container xform + base_link xform + outer_knuckle xform (rest pose)
4. The FixedJoint between wrist link and gripper base_link
5. Whether `ArticulationRootAPI` is properly applied (should be only on root)
"""
from __future__ import annotations

import argparse
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


def _path_exists(stage, path) -> str:
    from pxr import Sdf  # type: ignore
    p = Sdf.Path(str(path))
    prim = stage.GetPrimAtPath(p)
    return "OK" if (prim and prim.IsValid()) else "MISSING"


def main() -> int:
    sim_app = _boot_isaac_sim()
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--usd",
            default=str(
                _REPO_ROOT / "ust_ws" / "ust_hm_grip" / "isaac_file" / "GR1T2_with_robotiq.usd"
            ),
        )
        args = parser.parse_args()

        from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf  # noqa

        print(f"[diag] opening {args.usd!r}", flush=True)
        stage = Usd.Stage.Open(args.usd)
        if not stage:
            print("FAIL — cannot open", flush=True)
            return 1

        default_prim = stage.GetDefaultPrim()
        root_path = default_prim.GetPath()
        print(f"[diag] default prim path: {root_path}", flush=True)

        # Walk gripper subtrees
        for side in ("left", "right"):
            container_path = root_path.AppendChild(f"{side}_robotiq_arg2f_85")
            container_prim = stage.GetPrimAtPath(container_path)
            print(f"\n========== {side.upper()} GRIPPER ==========")
            print(f"container path: {container_path}  [{_path_exists(stage, container_path)}]")
            if not container_prim.IsValid():
                continue

            xc = UsdGeom.XformCache(Usd.TimeCode.Default())
            container_world = xc.GetLocalToWorldTransform(container_prim)
            print(f"container.world translate: {container_world.ExtractTranslation()}")
            print(f"container.world rotation (quat): {container_world.ExtractRotationQuat()}")

            # Wrist link
            wrist_path = root_path.AppendChild(f"{side}_hand_pitch_link")
            if not stage.GetPrimAtPath(wrist_path):
                # try GR1T2-prefixed
                wrist_path = root_path.AppendChild(f"GR1T2_fourier_hand_6dof_{side}_hand_pitch_link")
            wrist_prim = stage.GetPrimAtPath(wrist_path)
            print(f"wrist path:        {wrist_path}  [{_path_exists(stage, wrist_path)}]")
            if wrist_prim and wrist_prim.IsValid():
                wrist_world = xc.GetLocalToWorldTransform(wrist_prim)
                print(f"wrist.world translate: {wrist_world.ExtractTranslation()}")

            # Container's xform ops
            xformable = UsdGeom.Xformable(container_prim)
            ops = xformable.GetOrderedXformOps()
            print(f"container.xformOpOrder: {[op.GetOpName() for op in ops]}")
            for op in ops:
                print(f"  op {op.GetOpName()} = {op.Get()}")

            # base_link in container
            base_link_path = container_path.AppendChild("base_link")
            base_prim = stage.GetPrimAtPath(base_link_path)
            print(f"base_link path:    {base_link_path}  [{_path_exists(stage, base_link_path)}]")
            if base_prim and base_prim.IsValid():
                bw = xc.GetLocalToWorldTransform(base_prim)
                print(f"base_link.world translate: {bw.ExtractTranslation()}")
                print(f"base_link.applied APIs: {list(base_prim.GetAppliedSchemas())}")

            # outer_knuckles
            for body_name in ("left_outer_knuckle", "right_outer_knuckle",
                              "left_outer_finger", "right_outer_finger",
                              "left_inner_finger", "right_inner_finger",
                              "left_inner_knuckle", "right_inner_knuckle"):
                bp = container_path.AppendChild(body_name)
                bprim = stage.GetPrimAtPath(bp)
                print(f"  {body_name}: [{_path_exists(stage, bp)}]"
                      + (f" APIs={list(bprim.GetAppliedSchemas())[:3]}..." if bprim and bprim.IsValid() else ""))

            # joints
            joints_path = container_path.AppendChild("Joints")
            joints_scope = stage.GetPrimAtPath(joints_path)
            print(f"\njoints scope:      {joints_path}  [{_path_exists(stage, joints_path)}]")
            if joints_scope and joints_scope.IsValid():
                for jc in joints_scope.GetChildren():
                    jp = jc.GetPath()
                    jt = jc.GetTypeName()
                    apis = jc.GetAppliedSchemas()
                    mimic_apis = [a for a in apis if "Mimic" in a]
                    print(f"  joint: {jt} {jp.name}")
                    print(f"     APIs: {list(apis)}")
                    # body0 / body1
                    for rel_name in ("physics:body0", "physics:body1"):
                        rel = jc.GetRelationship(rel_name)
                        if rel and rel.IsValid():
                            tgts = list(rel.GetTargets())
                            tgt_status = [(t, _path_exists(stage, t)) for t in tgts]
                            print(f"     {rel_name}: {tgt_status}")
                    # mimic rels
                    for axis in ("rotX", "rotY", "rotZ", "transX", "transY", "transZ"):
                        rel_name = f"physxMimicJoint:{axis}:referenceJoint"
                        rel = jc.GetRelationship(rel_name)
                        if rel and rel.IsValid():
                            tgts = list(rel.GetTargets())
                            if tgts:
                                tgt_status = [(t, _path_exists(stage, t)) for t in tgts]
                                print(f"     {rel_name}: {tgt_status}")
                                for attr_suffix in ("gearing", "offset", "dampingRatio", "naturalFrequency"):
                                    attr_name = f"physxMimicJoint:{axis}:{attr_suffix}"
                                    attr = jc.GetAttribute(attr_name)
                                    if attr and attr.IsValid() and attr.HasAuthoredValue():
                                        print(f"        {attr_suffix}: {attr.Get()}")
                    # limits
                    lower = jc.GetAttribute("physics:lowerLimit")
                    upper = jc.GetAttribute("physics:upperLimit")
                    if lower and lower.HasAuthoredValue():
                        print(f"     limits: [{lower.Get()}, {upper.Get()}]")
                    drive_target = jc.GetAttribute("drive:angular:physics:targetPosition")
                    if drive_target and drive_target.HasAuthoredValue():
                        stiff = jc.GetAttribute("drive:angular:physics:stiffness")
                        damp = jc.GetAttribute("drive:angular:physics:damping")
                        maxf = jc.GetAttribute("drive:angular:physics:maxForce")
                        print(f"     drive: target={drive_target.Get()} stiff={stiff.Get() if stiff else None}"
                              f" damp={damp.Get() if damp else None} maxF={maxf.Get() if maxf else None}")
                    # also localPos / localRot
                    for attr_name in ("physics:localPos0", "physics:localPos1",
                                       "physics:localRot0", "physics:localRot1",
                                       "physics:axis"):
                        a = jc.GetAttribute(attr_name)
                        if a and a.IsValid() and a.HasAuthoredValue():
                            print(f"     {attr_name}: {a.Get()}")

            # FixedJoints under outer_finger
            for sub_body in ("left_outer_finger", "right_outer_finger"):
                for child in stage.GetPrimAtPath(container_path.AppendChild(sub_body)).GetChildren():
                    if "Joint" in str(child.GetTypeName()):
                        cp = child.GetPath()
                        print(f"\n   nested joint under {sub_body}: {cp}")
                        for rel_name in ("physics:body0", "physics:body1"):
                            rel = child.GetRelationship(rel_name)
                            if rel and rel.IsValid():
                                tgts = list(rel.GetTargets())
                                tgt_status = [(t, _path_exists(stage, t)) for t in tgts]
                                print(f"      {rel_name}: {tgt_status}")

            # attach FixedJoint
            attach_path = root_path.AppendChild(f"{side}_robotiq_attach_fixed_joint")
            attach = stage.GetPrimAtPath(attach_path)
            print(f"\nattach joint:      {attach_path}  [{_path_exists(stage, attach_path)}]")
            if attach and attach.IsValid():
                for rel_name in ("physics:body0", "physics:body1"):
                    rel = attach.GetRelationship(rel_name)
                    if rel and rel.IsValid():
                        tgts = list(rel.GetTargets())
                        tgt_status = [(t, _path_exists(stage, t)) for t in tgts]
                        print(f"  {rel_name}: {tgt_status}")
                for attr_name in ("physics:localPos0", "physics:localPos1",
                                   "physics:localRot0", "physics:localRot1"):
                    a = attach.GetAttribute(attr_name)
                    if a and a.IsValid() and a.HasAuthoredValue():
                        print(f"  {attr_name}: {a.Get()}")

            # TCP frame
            tcp_path = root_path.AppendChild(f"{side}_gripper_tcp_link")
            tcp = stage.GetPrimAtPath(tcp_path)
            print(f"\ntcp link path:     {tcp_path}  [{_path_exists(stage, tcp_path)}]")
            if tcp and tcp.IsValid():
                tcp_world = xc.GetLocalToWorldTransform(tcp)
                print(f"  tcp.world translate: {tcp_world.ExtractTranslation()}")

        # ArticulationRoot
        print("\n========== ArticulationRootAPI ==========")
        for prim in stage.Traverse():
            if UsdPhysics.ArticulationRootAPI(prim):
                print(f"  {prim.GetPath()}")

        return 0
    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
