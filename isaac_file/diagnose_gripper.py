#!/usr/bin/env python3
"""Diagnose gripper joint issues in the UST robot USD.

Checks:
1. Joint type and properties for all gripper joints
2. DriveAPI presence and settings
3. MimicJointAPI presence (which may interfere with direct control)
4. apiSchemas composition
5. Joint limits
"""

from pxr import Usd, UsdPhysics, Sdf, Gf

USD_PATH = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_fixed.usd"
ROBOT_PATH = "/World/Robot"

def inspect_joint(stage, joint_path, layer):
    """Inspect a single joint thoroughly."""
    prim = stage.GetPrimAtPath(joint_path)
    if not prim:
        print(f"  [NOT FOUND] {joint_path}")
        return

    print(f"\n{'='*70}")
    print(f"JOINT: {joint_path}")
    print(f"  Type: {prim.GetTypeName()}")

    # Applied schemas (composed)
    schemas = list(prim.GetAppliedSchemas())
    print(f"  Applied Schemas (composed): {schemas}")

    # Check layer-level apiSchemas override
    spec = layer.GetPrimAtPath(joint_path)
    if spec:
        api_info = spec.GetInfo("apiSchemas")
        print(f"  Layer apiSchemas override: {api_info}")
        if api_info:
            print(f"    isExplicit: {api_info.isExplicit}")
            print(f"    explicitItems: {api_info.explicitItems}")
            print(f"    prependedItems: {api_info.prependedItems}")
            print(f"    appendedItems: {api_info.appendedItems}")
            print(f"    deletedItems: {api_info.deletedItems}")

    # Joint attributes
    if prim.HasAPI(UsdPhysics.RevoluteJoint):
        print(f"  Joint API: RevoluteJoint")
        joint = UsdPhysics.RevoluteJoint(prim)
        print(f"    axis: {joint.GetAxisAttr().Get()}")
    elif prim.HasAPI(UsdPhysics.PrismaticJoint):
        print(f"  Joint API: PrismaticJoint")
        joint = UsdPhysics.PrismaticJoint(prim)
        print(f"    axis: {joint.GetAxisAttr().Get()}")

    # Check if it's a PrismaticJoint type
    if prim.GetTypeName() == "PhysicsPrismaticJoint":
        print(f"  TYPE is PhysicsPrismaticJoint")
    elif prim.GetTypeName() == "PhysicsRevoluteJoint":
        print(f"  TYPE is PhysicsRevoluteJoint")

    # Joint limits
    limit_api = UsdPhysics.LimitAPI(prim, "transY") if "Prismatic" in prim.GetTypeName() else None
    if limit_api:
        low = limit_api.GetLowAttr().Get()
        high = limit_api.GetHighAttr().Get()
        print(f"  Limits (transY): low={low}, high={high}")

    limit_api2 = UsdPhysics.LimitAPI(prim, "transX")
    if limit_api2:
        low = limit_api2.GetLowAttr().Get()
        high = limit_api2.GetHighAttr().Get()
        print(f"  Limits (transX): low={low}, high={high}")

    # Drive API
    for axis in ["transX", "transY", "transZ", "rotX", "rotY", "rotZ", "linear", "angular"]:
        drive = UsdPhysics.DriveAPI(prim, axis)
        if drive:
            type_attr = drive.GetTypeAttr()
            stiffness_attr = drive.GetStiffnessAttr()
            damping_attr = drive.GetDampingAttr()
            max_force_attr = drive.GetMaxForceAttr()
            target_pos = drive.GetTargetPositionAttr()
            target_vel = drive.GetTargetVelocityAttr()

            if type_attr and type_attr.Get() is not None:
                print(f"  DriveAPI ({axis}):")
                print(f"    type: {type_attr.Get()}")
                print(f"    stiffness: {stiffness_attr.Get() if stiffness_attr else None}")
                print(f"    damping: {damping_attr.Get() if damping_attr else None}")
                print(f"    maxForce: {max_force_attr.Get() if max_force_attr else None}")
                print(f"    targetPosition: {target_pos.Get() if target_pos else None}")
                print(f"    targetVelocity: {target_vel.Get() if target_vel else None}")

    # Mimic Joint properties
    for axis in ["transX", "transY", "rotX", "rotY", "rotZ"]:
        ref_rel_name = f"physxMimicJoint:{axis}:referenceJoint"
        ref_rel = prim.GetRelationship(ref_rel_name)
        if ref_rel and ref_rel.GetTargets():
            print(f"  MimicJoint ({axis}):")
            print(f"    referenceJoint: {ref_rel.GetTargets()}")
            gearing_attr = prim.GetAttribute(f"physxMimicJoint:{axis}:gearing")
            offset_attr = prim.GetAttribute(f"physxMimicJoint:{axis}:offset")
            if gearing_attr:
                print(f"    gearing: {gearing_attr.Get()}")
            if offset_attr:
                print(f"    offset: {offset_attr.Get()}")

    # Body relationships
    for rel in prim.GetRelationships():
        targets = rel.GetTargets()
        if targets:
            print(f"  Rel '{rel.GetName()}': {targets}")

    # All attributes for completeness
    print(f"  --- All attributes ---")
    for attr in prim.GetAttributes():
        val = attr.Get()
        if val is not None:
            print(f"    {attr.GetName()} = {val}")


def main():
    print(f"Opening: {USD_PATH}")
    stage = Usd.Stage.Open(USD_PATH)
    layer = Sdf.Layer.FindOrOpen(USD_PATH)

    if not stage:
        print("ERROR: Could not open stage")
        return

    # Find all gripper joints
    gripper_joints = []
    for side in ["right", "left"]:
        for finger in ["left", "right"]:
            # Check both possible paths
            paths = [
                f"/World/Robot/open_manipulator_x_{side}/joints/{side}_gripper_{finger}_joint",
                f"/World/Robot/wapple_pi/base/open_manipulator_x_{side}/joints/{side}_gripper_{finger}_joint",
            ]
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                if prim:
                    gripper_joints.append(path)
                    break
            else:
                # Also try to find by name
                print(f"  [SEARCH] Looking for {side}_gripper_{finger}_joint...")
                for prim in stage.TraverseAll():
                    if prim.GetName() == f"{side}_gripper_{finger}_joint":
                        gripper_joints.append(str(prim.GetPath()))
                        print(f"    Found at: {prim.GetPath()}")
                        break

    print(f"\nFound {len(gripper_joints)} gripper joints:")
    for j in gripper_joints:
        print(f"  {j}")

    # Inspect each joint
    for joint_path in gripper_joints:
        inspect_joint(stage, joint_path, layer)

    # Also check what Isaac Lab's find_joints would resolve
    print(f"\n{'='*70}")
    print("JOINT NAME RESOLUTION CHECK")
    print(f"{'='*70}")

    # Simulate what find_joints does - look for all joints in the articulation
    print("\nAll joints in articulation:")
    root_prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/wapple_pi")
    if root_prim:
        for prim in Usd.PrimRange(root_prim):
            if prim.IsA(UsdPhysics.Joint):
                print(f"  {prim.GetPath()} [{prim.GetTypeName()}]")

    # Check open_manipulator arm joint hierarchies
    for side in ["right", "left"]:
        arm_path = f"{ROBOT_PATH}/open_manipulator_x_{side}"
        arm_prim = stage.GetPrimAtPath(arm_path)
        if arm_prim:
            print(f"\nArm assembly: {arm_path}")
            for prim in Usd.PrimRange(arm_prim):
                if prim.IsA(UsdPhysics.Joint):
                    print(f"  {prim.GetPath()} [{prim.GetTypeName()}]")


if __name__ == "__main__":
    main()
