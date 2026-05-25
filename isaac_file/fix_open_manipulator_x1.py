
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

def find_robot_prims(stage):
    """
    Search for open_manipulator_x1, worldBody, and link2 prims in the stage hierarchy.
    Returns tuple of (robot_root_path, worldbody_path, link2_path) or (None, None, None) if not found.
    """
    robot_root_path = None
    worldbody_path = None
    link2_path = None

    # Search through the entire stage
    for prim in Usd.PrimRange.AllPrims(stage.GetPseudoRoot()):
        prim_name = prim.GetName()
        prim_path = str(prim.GetPath())

        # Look for open_manipulator_x1
        if prim_name == "open_manipulator_x1":
            robot_root_path = prim_path
            print(f"Found robot root at: {robot_root_path}")

            # Look for worldBody and link2 under this robot
            for child in prim.GetChildren():
                child_name = child.GetName()
                if child_name == "worldBody":
                    worldbody_path = str(child.GetPath())
                    print(f"Found worldBody at: {worldbody_path}")
                elif child_name == "link2":
                    link2_path = str(child.GetPath())
                    print(f"Found link2 at: {link2_path}")

    return robot_root_path, worldbody_path, link2_path

def fix_robot_hierarchy():
    """
    Fixes the hierarchy of the open_manipulator_x1 robot by:
    1. Renaming worldBody to link1
    2. Making link2 (and all its children) a child of the new link1
    3. Setting articulation root correctly
    """
    stage = omni.usd.get_context().get_stage()

    print("=== Starting Robot Hierarchy Fix ===")
    print()
    print("Current structure:")
    print("  open_manipulator_x1/")
    print("    ├── link2 (has link2,3,4,5, grippers)")
    print("    └── worldBody (has link1 mesh)")
    print()
    print("Target structure:")
    print("  open_manipulator_x1/")
    print("    └── link1 (renamed from worldBody)")
    print("        └── link2 (moved here with all children)")
    print()

    # --- 1. FIND ROBOT PRIMS AUTOMATICALLY ---
    print("--- Step 1: Searching for robot components ---")
    robot_root_path, worldbody_path, link2_path = find_robot_prims(stage)

    if not robot_root_path or not worldbody_path or not link2_path:
        print("Error: Could not find required components in the stage.")
        print(f"Found robot root: {robot_root_path}")
        print(f"Found worldBody: {worldbody_path}")
        print(f"Found link2: {link2_path}")
        return

    robot_root_prim = stage.GetPrimAtPath(robot_root_path)
    worldbody_prim = stage.GetPrimAtPath(worldbody_path)
    link2_prim = stage.GetPrimAtPath(link2_path)

    if not robot_root_prim.IsValid() or not worldbody_prim.IsValid() or not link2_prim.IsValid():
        print(f"Error: Invalid prims found.")
        return

    print(f"✓ Successfully located all components")
    print()

    # --- 2. CHECK AND LOAD PAYLOADS IF NEEDED ---
    print("--- Step 2: Checking for payloads ---")
    payloads_found = False

    for prim in Usd.PrimRange.AllPrims(robot_root_prim):
        if prim.HasPayload():
            print(f"  Loading payload for: {prim.GetPath()}")
            prim.Load()
            payloads_found = True

    if payloads_found:
        print("✓ Payloads loaded successfully")
    else:
        print("✓ No payloads found (this is normal)")
    print()

    # --- 3. CREATE NEW LINK1 AND COPY WORLDBODY CONTENT ---
    print("--- Step 3: Creating new link1 from worldBody ---")

    new_link1_path = f"{robot_root_path}/link1"
    layer = stage.GetRootLayer()

    try:
        # Instead of renaming, we copy the worldBody spec to link1
        # This works even with payloads
        print(f"  Creating link1 at: {new_link1_path}")

        with Sdf.ChangeBlock():
            # Copy worldBody to link1
            Sdf.CopySpec(layer, worldbody_path, layer, new_link1_path)

        link1_prim = stage.GetPrimAtPath(new_link1_path)
        if link1_prim.IsValid():
            print(f"✓ Successfully created link1 from worldBody")
            print(f"  New path: {new_link1_path}")

            # Now delete the original worldBody
            print(f"  Removing original worldBody...")
            with Sdf.ChangeBlock():
                edit = Sdf.BatchNamespaceEdit()
                edit.Add(worldbody_path, Sdf.Path.emptyPath)
                if not layer.Apply(edit):
                    print("  Warning: Could not remove worldBody, trying alternative method...")
                    # Alternative: use stage delete
                    stage.RemovePrim(worldbody_path)

            print(f"✓ Original worldBody removed")
        else:
            print("Error: Failed to create link1 from worldBody")
            return
    except Exception as e:
        print(f"Error during link1 creation: {e}")
        print("Trying alternative method using DefinePrim...")
        try:
            # Alternative: Create new link1 and copy properties manually
            link1_prim = stage.DefinePrim(new_link1_path, worldbody_prim.GetTypeName())

            # Copy all attributes
            for attr in worldbody_prim.GetAttributes():
                new_attr = link1_prim.CreateAttribute(attr.GetName(), attr.GetTypeName())
                new_attr.Set(attr.Get())

            # Copy all relationships
            for rel in worldbody_prim.GetRelationships():
                new_rel = link1_prim.CreateRelationship(rel.GetName())
                new_rel.SetTargets(rel.GetTargets())

            # Copy children
            for child in worldbody_prim.GetChildren():
                child_name = child.GetName()
                new_child_path = f"{new_link1_path}/{child_name}"
                with Sdf.ChangeBlock():
                    Sdf.CopySpec(layer, str(child.GetPath()), layer, new_child_path)

            # Remove old worldBody
            stage.RemovePrim(worldbody_path)

            print(f"✓ Successfully created link1 using alternative method")
        except Exception as e2:
            print(f"Error: All methods failed: {e2}")
            return
    print()

    # --- 4. REPARENT LINK2 TO BE A CHILD OF LINK1 ---
    print("--- Step 4: Moving link2 (and all children) under link1 ---")

    new_link2_path = f"{new_link1_path}/link2"
    print(f"  Moving '{link2_path}' to '{new_link2_path}'...")

    try:
        omni.kit.commands.execute('MovePrim',
            path_from=link2_path,
            path_to=new_link2_path)

        link2_prim = stage.GetPrimAtPath(new_link2_path)
        if link2_prim.IsValid():
            print("✓ Successfully moved link2 under link1")
            print(f"  All children (link2,3,4,5, grippers) moved together")
        else:
            print("Error: Failed to move link2")
            return
    except Exception as e:
        print(f"Error during reparenting: {e}")
        return
    print()

    # --- 5. REMOVE ALL EXISTING ARTICULATION ROOT APIS ---
    print("--- Step 5: Cleaning up Articulation Root APIs ---")
    removed_count = 0

    # Check all prims under the robot
    for prim in Usd.PrimRange.AllPrims(robot_root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"  Removing ArticulationRootAPI from '{prim.GetPath()}'")
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            removed_count += 1

    if removed_count > 0:
        print(f"✓ Removed {removed_count} ArticulationRootAPI instances")
    else:
        print("✓ No existing ArticulationRootAPI found")
    print()

    # --- 6. APPLY ARTICULATION ROOT TO LINK1 ---
    print("--- Step 6: Applying ArticulationRootAPI to link1 ---")
    link1_prim = stage.GetPrimAtPath(new_link1_path)

    if not link1_prim.IsValid():
        print(f"Error: link1 prim at '{new_link1_path}' is not valid")
        return

    UsdPhysics.ArticulationRootAPI.Apply(link1_prim)
    print(f"✓ Applied ArticulationRootAPI to '{new_link1_path}'")
    print()

    # --- 7. VERIFY THE NEW HIERARCHY ---
    print("--- Step 7: Verifying new hierarchy ---")
    link1_prim = stage.GetPrimAtPath(new_link1_path)
    link2_prim = stage.GetPrimAtPath(new_link2_path)

    if link1_prim.IsValid() and link2_prim.IsValid():
        print("✓ Hierarchy verified:")
        print(f"  ├── link1: {new_link1_path}")
        print(f"  └── link2: {new_link2_path}")

        # Count children under link2
        child_count = 0
        for child in link2_prim.GetChildren():
            child_count += 1

        print(f"  link2 has {child_count} children (link2,3,4,5, grippers, etc.)")
    else:
        print("Warning: Could not verify hierarchy")
    print()

    # --- 8. SAVE THE FIXED STAGE ---
    output_path = "/home/isaac/ust_ws/isaac_file/ust_project1_fixed.usd"
    print("--- Step 8: Saving fixed USD file ---")
    print(f"Saving to: {output_path}")

    try:
        omni.usd.get_context().save_as_stage(output_path)
        print("✓ Save complete!")
        print()
        print("=" * 70)
        print("SUCCESS! Robot hierarchy has been fixed!")
        print("=" * 70)
        print()
        print("CHANGES MADE:")
        print("  1. Created link1 from worldBody (copied and removed original)")
        print("  2. Moved link2 (and all children) under link1")
        print("  3. Applied ArticulationRootAPI to link1")
        print()
        print("NEW STRUCTURE:")
        print("  /World/Robot/open_manipulator_x1/")
        print("    └── link1 (ArticulationRoot)")
        print("        └── link2")
        print("            ├── link2")
        print("            ├── link3")
        print("            ├── link4")
        print("            ├── link5")
        print("            ├── gripper_left_link")
        print("            ├── gripper_right_link")
        print("            └── end_effector_target")
        print()
        print(f"Saved to: {output_path}")
        print()
        print("NEXT STEPS:")
        print("  1. Close the current USD file in Isaac Sim")
        print("  2. Open 'ust_project1_fixed.usd'")
        print("  3. Open Lula Robot Description Editor")
        print("  4. Select 'link1' as the Robot Prim")
        print("  5. All joints should now be visible in the Joints tab!")
        print()
        print("=" * 70)
    except Exception as e:
        print(f"Error saving file: {e}")
        print("The changes may still be in memory. Try saving manually.")

# To run this script:
# 1. Open your original 'ust_project1.usd' file in Isaac Sim.
# 2. Open the Script Editor (Window > Script Editor).
# 3. Copy and paste the entire content of this file into the editor.
# 4. Click 'Run'.
# 5. A new file 'ust_project1_fixed.usd' will be created with the corrected hierarchy.
if __name__ == "__main__":
    fix_robot_hierarchy()
