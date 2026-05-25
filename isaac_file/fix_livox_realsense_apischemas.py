#!/usr/bin/env python3
"""Fix livox mid360 and realsense2 camera apiSchemas overrides.

The main USD (ust_project1_fixed.usd) has empty SdfTokenListOp() overrides
on livox_mid360 and realsense2_camera rigid body prims. This blocks RigidBodyAPI
from propagating through USD composition, making the FixedJoint chain inert
(livox/realsense float in space instead of being attached to the robot).

Fix: ClearInfo("apiSchemas") on all affected prims so the referenced USD's
RigidBodyAPI properly propagates.

Usage:
    python3 fix_livox_realsense_apischemas.py
"""

from pxr import Sdf


USD_PATH = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_fixed.usd"
ROBOT = "/World/Robot"


def fix_apiSchemas_overrides(layer):
    """Clear empty apiSchemas overrides on livox and realsense prims."""

    # All prims that need apiSchemas override cleared
    prims_to_fix = [
        # Livox mid360 rigid body prims
        f"{ROBOT}/livox_mid360/world",
        f"{ROBOT}/livox_mid360/oasis_300",
        f"{ROBOT}/livox_mid360/livox_base",
        f"{ROBOT}/livox_mid360/livox",
        # Realsense2 camera rigid body prims
        f"{ROBOT}/realsense2_camera/base_link",
        f"{ROBOT}/realsense2_camera/camera_link",
    ]

    fixed_count = 0

    for prim_path in prims_to_fix:
        spec = layer.GetPrimAtPath(prim_path)
        if not spec:
            print(f"  [SKIP] {prim_path} not found in layer")
            continue

        api = spec.GetInfo("apiSchemas")

        if not api:
            print(f"  [OK] {prim_path}: no apiSchemas override (good)")
            continue

        # Check if it's an empty override (SdfTokenListOp() with no items)
        is_empty = False

        if api.isExplicit and not api.explicitItems:
            is_empty = True
        elif not api.isExplicit:
            has_items = (api.explicitItems or api.prependedItems or api.appendedItems
                         or api.deletedItems or api.orderedItems)
            if not has_items:
                is_empty = True

        if is_empty:
            spec.ClearInfo("apiSchemas")
            print(f"  [FIX] Cleared empty apiSchemas override on {prim_path}")
            fixed_count += 1
        else:
            print(f"  [OK] {prim_path}: apiSchemas has items: {api}")

    return fixed_count


def scan_all_prims_for_empty_apischemas(layer):
    """Scan ALL prims in the layer for empty apiSchemas overrides.

    This catches any other prims we might have missed.
    """
    print("\n=== Scanning ALL prims for empty apiSchemas overrides ===")

    def _scan_prim_spec(spec, path_str):
        found = []
        api = spec.GetInfo("apiSchemas")
        if api:
            is_empty = False
            if api.isExplicit and not api.explicitItems:
                is_empty = True
            elif not api.isExplicit:
                has_items = (api.explicitItems or api.prependedItems or api.appendedItems
                             or api.deletedItems or api.orderedItems)
                if not has_items:
                    is_empty = True

            if is_empty:
                found.append(path_str)

        # Recurse into children
        for child_name in spec.nameChildren:
            child_spec = spec.nameChildren[child_name]
            child_path = f"{path_str}/{child_name}" if path_str != "/" else f"/{child_name}"
            found.extend(_scan_prim_spec(child_spec, child_path))

        return found

    root = layer.pseudoRoot
    empty_prims = []
    for child_name in root.nameChildren:
        child_spec = root.nameChildren[child_name]
        empty_prims.extend(_scan_prim_spec(child_spec, f"/{child_name}"))

    if empty_prims:
        print(f"  Found {len(empty_prims)} prims with empty apiSchemas overrides:")
        for p in empty_prims:
            print(f"    - {p}")
    else:
        print("  No remaining prims with empty apiSchemas overrides (all clean!)")

    return empty_prims


def main():
    print(f"Opening USD: {USD_PATH}")
    layer = Sdf.Layer.FindOrOpen(USD_PATH)
    if not layer:
        print(f"ERROR: Could not open {USD_PATH}")
        return

    # Fix known prims
    print("\n=== FIX: Livox mid360 & Realsense2 apiSchemas ===")
    fixed_count = fix_apiSchemas_overrides(layer)
    print(f"\nFixed {fixed_count} prims")

    # Scan for any remaining empty apiSchemas overrides
    remaining = scan_all_prims_for_empty_apischemas(layer)

    # Fix any remaining ones found by scan
    if remaining:
        print(f"\n=== FIX: Clearing {len(remaining)} additional empty apiSchemas overrides ===")
        for prim_path in remaining:
            spec = layer.GetPrimAtPath(prim_path)
            if spec:
                spec.ClearInfo("apiSchemas")
                print(f"  [FIX] Cleared {prim_path}")
                fixed_count += 1

    # Save
    if fixed_count > 0:
        layer.Save()
        print(f"\nSaved {USD_PATH} with {fixed_count} total fixes")
    else:
        print("\nNo fixes needed - all apiSchemas look correct")

    # Final verification scan
    remaining_after = scan_all_prims_for_empty_apischemas(layer)
    if not remaining_after:
        print("\n✓ Verification passed: no empty apiSchemas overrides remain")


if __name__ == "__main__":
    main()
