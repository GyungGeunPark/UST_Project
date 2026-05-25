#!/usr/bin/env python3
"""Fix gripper_right_joint to support direct position control.

Problem:
- gripper_left_joint has PhysicsDriveAPI:linear → Isaac Lab can set position targets
- gripper_right_joint has PhysxMimicJointAPI:transY but NO DriveAPI → cannot be directly controlled
- PhysxMimicJointAPI doesn't work at runtime (known PhysX issue)
- Result: only one finger moves when gripping

Fix:
1. Add PhysicsDriveAPI:linear to *_gripper_right_joint in arm physics USD files
2. Clear empty SdfTokenListOp() apiSchemas overrides in main USD (so schemas propagate)
3. Remove PhysxMimicJointAPI:transY from gripper_right_joint (since we're driving directly)

Applied to:
- open_manipulator_x_right/configuration/open_manipulator_x_physics.usd
- open_manipulator_x_left/configuration/open_manipulator_x_physics.usd
- ust_project1_fixed.usd (clear apiSchemas overrides)
"""

from pxr import Usd, UsdPhysics, Sdf, Gf, Vt


def fix_arm_physics_usd(side: str):
    """Add DriveAPI:linear to gripper_right_joint in arm physics USD."""
    physics_path = f"/workspace/isaaclab/ust_ws/isaac_file/open_manipulator_x_{side}/configuration/open_manipulator_x_physics.usd"
    print(f"\n{'='*60}")
    print(f"Fixing arm physics USD: {side}")
    print(f"File: {physics_path}")
    print(f"{'='*60}")

    layer = Sdf.Layer.FindOrOpen(physics_path)
    if not layer:
        print(f"  ERROR: Could not open {physics_path}")
        return False

    joint_path = f"/open_manipulator_x/joints/{side}_gripper_right_joint"
    spec = layer.GetPrimAtPath(joint_path)
    if not spec:
        print(f"  ERROR: Joint spec not found: {joint_path}")
        return False

    # Step 1: Check current apiSchemas
    api = spec.GetInfo("apiSchemas")
    print(f"  Current apiSchemas: {api}")

    # Step 2: Replace apiSchemas - remove MimicJointAPI, add DriveAPI
    # Current: PhysicsJointStateAPI:linear, PhysxJointAPI, PhysxMimicJointAPI:transY
    # New:     PhysicsJointStateAPI:linear, PhysxJointAPI, PhysicsDriveAPI:linear
    new_api = Sdf.TokenListOp()
    new_api.prependedItems = [
        "PhysicsJointStateAPI:linear",
        "PhysxJointAPI",
        "PhysicsDriveAPI:linear",
    ]
    spec.SetInfo("apiSchemas", new_api)
    print(f"  [FIX] Set apiSchemas: {new_api.prependedItems}")

    # Step 3: Add DriveAPI:linear properties (matching gripper_left_joint)
    drive_props = {
        "drive:linear:physics:type": ("Token", "force"),
        "drive:linear:physics:stiffness": ("Float", 0.00719817029312253),
        "drive:linear:physics:damping": ("Float", 2.8792680950573413e-06),
        "drive:linear:physics:maxForce": ("Float", 1000.0),
        "drive:linear:physics:targetPosition": ("Float", 0.0),
        "drive:linear:physics:targetVelocity": ("Float", 0.0),
    }

    for prop_name, (type_name, default_val) in drive_props.items():
        existing = spec.properties.get(prop_name)
        if existing:
            existing.default = default_val
            print(f"  [UPDATE] {prop_name} = {default_val}")
        else:
            if type_name == "Token":
                attr = Sdf.AttributeSpec(spec, prop_name, Sdf.ValueTypeNames.Token)
            elif type_name == "Float":
                attr = Sdf.AttributeSpec(spec, prop_name, Sdf.ValueTypeNames.Float)
            attr.default = default_val
            print(f"  [ADD] {prop_name} = {default_val}")

    # Step 4: Add maxJointVelocity (matching gripper_left_joint)
    max_vel_prop = spec.properties.get("physxJoint:maxJointVelocity")
    if not max_vel_prop:
        attr = Sdf.AttributeSpec(spec, "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float)
        attr.default = 4.8
        print(f"  [ADD] physxJoint:maxJointVelocity = 4.8")

    # Step 5: Remove mimic joint properties (no longer needed since we drive directly)
    mimic_props_to_remove = [
        "physxMimicJoint:transY:gearing",
        "physxMimicJoint:transY:offset",
        "physxMimicJoint:transY:dampingRatio",
        "physxMimicJoint:transY:naturalFrequency",
        "physxMimicJoint:transY:referenceJoint",
    ]
    for prop_name in mimic_props_to_remove:
        prop = spec.properties.get(prop_name)
        if prop:
            spec.RemoveProperty(prop)
            print(f"  [REMOVE] {prop_name}")

    # Save
    layer.Save()
    print(f"  [SAVED] {physics_path}")
    return True


def fix_main_usd_apiSchemas():
    """Clear empty SdfTokenListOp() overrides on gripper joints in main USD."""
    usd_path = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_fixed.usd"
    print(f"\n{'='*60}")
    print(f"Fixing main USD apiSchemas overrides")
    print(f"File: {usd_path}")
    print(f"{'='*60}")

    layer = Sdf.Layer.FindOrOpen(usd_path)
    if not layer:
        print(f"  ERROR: Could not open {usd_path}")
        return False

    fixed_count = 0
    for side in ["right", "left"]:
        for finger in ["left", "right"]:
            joint_path = f"/World/Robot/open_manipulator_x_{side}/joints/{side}_gripper_{finger}_joint"
            spec = layer.GetPrimAtPath(joint_path)
            if not spec:
                print(f"  [SKIP] {joint_path} not found in main layer")
                continue

            api = spec.GetInfo("apiSchemas")
            if api:
                # Check if it's empty (blocking the reference's schemas)
                has_items = (api.explicitItems or api.prependedItems or api.appendedItems)
                if not has_items:
                    spec.ClearInfo("apiSchemas")
                    print(f"  [FIX] Cleared empty apiSchemas override: {side}_gripper_{finger}_joint")
                    fixed_count += 1
                else:
                    print(f"  [OK] {side}_gripper_{finger}_joint has schemas: {api.prependedItems or api.explicitItems}")
            else:
                print(f"  [OK] {side}_gripper_{finger}_joint has no apiSchemas override")

    # Save
    if fixed_count > 0:
        layer.Save()
        print(f"  [SAVED] {usd_path} ({fixed_count} overrides cleared)")
    else:
        print(f"  [OK] No overrides to clear")

    return True


def verify_fix():
    """Verify the fix by checking composed schemas."""
    print(f"\n{'='*60}")
    print(f"VERIFICATION")
    print(f"{'='*60}")

    # Check arm physics USDs
    for side in ["right", "left"]:
        physics_path = f"/workspace/isaaclab/ust_ws/isaac_file/open_manipulator_x_{side}/configuration/open_manipulator_x_physics.usd"
        layer = Sdf.Layer.FindOrOpen(physics_path)
        if layer:
            joint_path = f"/open_manipulator_x/joints/{side}_gripper_right_joint"
            spec = layer.GetPrimAtPath(joint_path)
            if spec:
                api = spec.GetInfo("apiSchemas")
                print(f"\n  {side} arm physics USD - gripper_right_joint:")
                print(f"    apiSchemas: {api.prependedItems if api else 'None'}")

                # Check for drive properties
                has_drive = False
                has_mimic = False
                for prop in spec.properties:
                    if "drive:linear" in prop.name:
                        has_drive = True
                    if "physxMimicJoint" in prop.name:
                        has_mimic = True
                print(f"    Has DriveAPI props: {has_drive}")
                print(f"    Has MimicJoint props: {has_mimic}")

    # Check main USD composed view
    usd_path = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_fixed.usd"
    stage = Usd.Stage.Open(usd_path)
    if stage:
        print(f"\n  Main USD composed view:")
        for side in ["right", "left"]:
            for finger in ["left", "right"]:
                joint_path = f"/World/Robot/open_manipulator_x_{side}/joints/{side}_gripper_{finger}_joint"
                prim = stage.GetPrimAtPath(joint_path)
                if prim:
                    schemas = list(prim.GetAppliedSchemas())
                    print(f"    {side}_gripper_{finger}_joint: {schemas}")

                    # Check drive
                    drive_type = prim.GetAttribute("drive:linear:physics:type")
                    if drive_type and drive_type.Get():
                        print(f"      DriveAPI: type={drive_type.Get()}, "
                              f"stiffness={prim.GetAttribute('drive:linear:physics:stiffness').Get()}, "
                              f"maxForce={prim.GetAttribute('drive:linear:physics:maxForce').Get()}")


def main():
    # Fix arm physics USDs
    fix_arm_physics_usd("right")
    fix_arm_physics_usd("left")

    # Fix main USD apiSchemas overrides
    fix_main_usd_apiSchemas()

    # Verify
    verify_fix()

    print(f"\n{'='*60}")
    print(f"FIX COMPLETE!")
    print(f"{'='*60}")
    print(f"\nBoth gripper fingers now have PhysicsDriveAPI:linear.")
    print(f"Isaac Lab's BinaryJointPositionAction can directly control both fingers.")
    print(f"The mimic joint properties have been removed (no longer needed).")


if __name__ == "__main__":
    main()
