#!/usr/bin/env python3
"""Fix visual/physics mismatch in UST robot USD.

Issue: Visual meshes (waffle_pi_base, tire meshes) are siblings of the
physics 'base' prim, not children. When the base RigidBody moves during
simulation, the visual meshes stay in place causing a visual/physics split.

Fix: Reparent visual meshes to be children of the 'base' RigidBody prim
using Sdf layer API (required for reliable USDC edits).
"""

from pxr import Usd, UsdGeom, Sdf, Gf

USD_PATH = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_fixed.usd"
WAPPLE_PATH = "/World/Robot/wapple_pi"
BASE_PATH = f"{WAPPLE_PATH}/base"

# Visual mesh prims to reparent under base
VISUAL_MESHES = [
    "waffle_pi_base",
    "left_tire_1",
    "left_tire_2",
    "right_tire_1",
    "right_tire_2",
]


def remove_child_spec(parent_spec, child_name):
    """Remove a child prim spec from parent using nameChildren list manipulation."""
    # In Sdf Python API, we remove by editing the nameChildren list
    children = parent_spec.nameChildren
    for i, child in enumerate(children):
        if child.name == child_name:
            del parent_spec.nameChildren[child.name]
            return True
    return False


def reparent_visual_meshes():
    """Reparent visual meshes from wapple_pi/ to wapple_pi/base/ using Sdf layer API."""
    print(f"Opening: {USD_PATH}")

    # Use Sdf.Layer directly to avoid stage composition overhead
    layer = Sdf.Layer.FindOrOpen(USD_PATH)
    if not layer:
        print(f"ERROR: Could not open {USD_PATH}")
        return

    wapple_spec = layer.GetPrimAtPath(WAPPLE_PATH)
    base_spec = layer.GetPrimAtPath(BASE_PATH)

    if not wapple_spec:
        print(f"ERROR: {WAPPLE_PATH} not found in layer")
        return
    if not base_spec:
        print(f"ERROR: {BASE_PATH} not found in layer")
        return

    # First clean up any partial previous run (visuals container may exist)
    visuals_path = f"{BASE_PATH}/visuals"
    existing_visuals = layer.GetPrimAtPath(visuals_path)
    if existing_visuals:
        # Check what's already there
        print(f"  [INFO] visuals container already exists, checking contents...")
        for child in existing_visuals.nameChildren:
            print(f"    Already has: {child.name}")

    # Create a "visuals" container under base for organization
    if not existing_visuals:
        existing_visuals = Sdf.PrimSpec(base_spec, "visuals", Sdf.SpecifierDef, "Xform")
        print(f"  [CREATE] Xform -> {visuals_path}")

    visuals_spec = existing_visuals

    for mesh_name in VISUAL_MESHES:
        src_path = f"{WAPPLE_PATH}/{mesh_name}"
        dst_path = f"{visuals_path}/{mesh_name}"

        src_spec = layer.GetPrimAtPath(src_path)
        dst_spec = layer.GetPrimAtPath(dst_path)

        # If already at destination but also still at source, remove source
        if dst_spec and src_spec:
            print(f"  [CLEANUP] {mesh_name} exists in both src and dst, removing src")
            del wapple_spec.nameChildren[mesh_name]
            continue

        # If already moved
        if dst_spec and not src_spec:
            print(f"  [SKIP] {mesh_name} already at {dst_path}")
            continue

        # If source doesn't exist
        if not src_spec:
            print(f"  [SKIP] {mesh_name} not found at {src_path}")
            continue

        # Copy then remove original
        success = Sdf.CopySpec(layer, src_path, layer, dst_path)
        if success:
            del wapple_spec.nameChildren[mesh_name]
            print(f"  [MOVE] {src_path} -> {dst_path}")
        else:
            print(f"  [ERROR] Failed to copy {src_path} -> {dst_path}")

    # Save
    print(f"\nSaving: {USD_PATH}")
    layer.Save()
    print("DONE!")

    # Verify using composed stage
    print("\n=== VERIFICATION ===")
    verify_stage = Usd.Stage.Open(USD_PATH)

    base_prim = verify_stage.GetPrimAtPath(BASE_PATH)
    print(f"\nChildren of {BASE_PATH}:")
    for child in base_prim.GetChildren():
        cpath = str(child.GetPath())
        print(f"  {cpath} ({child.GetTypeName()})")
        if "visuals" in str(child.GetName()):
            for vchild in child.GetChildren():
                print(f"    {vchild.GetPath()} ({vchild.GetTypeName()})")

    wapple_prim = verify_stage.GetPrimAtPath(WAPPLE_PATH)
    print(f"\nChildren of {WAPPLE_PATH}:")
    for child in wapple_prim.GetChildren():
        cpath = str(child.GetPath())
        print(f"  {cpath} ({child.GetTypeName()})")

    # Verify transforms preserved
    print("\n=== Transform check ===")
    for mesh_name in VISUAL_MESHES:
        new_path = f"{visuals_path}/{mesh_name}"
        prim = verify_stage.GetPrimAtPath(new_path)
        if prim.IsValid():
            xf = UsdGeom.Xformable(prim)
            ops = xf.GetOrderedXformOps()
            translate_val = None
            for op in ops:
                if op.GetName() == "xformOp:translate":
                    translate_val = op.Get()
            print(f"  {mesh_name}: translate={translate_val}")
        else:
            print(f"  {mesh_name}: NOT FOUND at {new_path}")


if __name__ == "__main__":
    reparent_visual_meshes()
