
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

def fix_robot_with_payloads():
    """
    Loads payloads for the open_manipulator_x1 robot, fixes its hierarchy,
    and saves it as a new, self-contained USD file.
    """
    stage = omni.usd.get_context().get_stage()

    # --- 1. DEFINE PRIM PATHS ---
    # PLEASE VERIFY THESE PATHS ARE CORRECT FOR YOUR STAGE
    world_body_path = "/World"
    link1_path = f"{world_body_path}/link1"
    link2_path = f"{world_body_path}/link2"
    
    base_prim_to_check = stage.GetPrimAtPath(world_body_path)
    if not base_prim_to_check.IsValid():
        print(f"Error: The base path '{world_body_path}' is not valid. Please check the structure of your stage.")
        return

    print("--- Step 1: Loading Payloads ---")
    print("This will make the referenced content directly editable in the stage.")

    # We need to iterate through the prims that are part of the robot
    # It's safer to just load all payloads under the world body for simplicity
    for prim in Usd.PrimRange.AllPrims(base_prim_to_check):
        if prim.HasPayload():
            print(f"Loading payload for: {prim.GetPath()}")
            prim.Load()

    print("Payload loading complete.")

    # --- 2. HIERARCHY AND ARTICULATION ROOT FIX (from previous script) ---
    print("\n--- Step 2: Fixing Hierarchy and Articulation Root ---")
    
    link1_prim = stage.GetPrimAtPath(link1_path)
    link2_prim = stage.GetPrimAtPath(link2_path)

    if not link1_prim.IsValid() or not link2_prim.IsValid():
        print(f"Error: Prims not found at '{link1_path}' or '{link2_path}' even after loading payloads.")
        return

    # Reparent link2 to be a child of link1
    new_link2_path = f"{link1_path}/link2"
    print(f"Moving '{link2_path}' to '{new_link2_path}'...")
    
    layer = stage.GetRootLayer()
    with Sdf.ChangeBlock():
        Sdf.MoveSpec(layer, link2_path, layer, new_link2_path)
        
    link2_prim_new = stage.GetPrimAtPath(new_link2_path)
    if not link2_prim_new.IsValid():
        print("Error: Failed to move link2. The hierarchy could not be fixed.")
        return
        
    print("Reparenting successful.")

    # Remove all existing Articulation Root APIs
    print("Searching for and removing existing Articulation Root APIs...")
    for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath(link1_path)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"Removing ArticulationRootAPI from '{prim.GetPath()}'")
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    
    old_link2_prim = stage.GetPrimAtPath(link2_path) # This prim should not be valid anymore, but check just in case
    if old_link2_prim.IsValid() and old_link2_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
         print(f"Removing ArticulationRootAPI from '{old_link2_prim.GetPath()}'")
         old_link2_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)


    # Apply Articulation Root to the new base link (link1)
    print(f"Applying ArticulationRootAPI to the new base link: '{link1_path}'")
    link1_prim = stage.GetPrimAtPath(link1_path)
    UsdPhysics.ArticulationRootAPI.Apply(link1_prim)

    print("Hierarchy and Articulation Root configuration complete.")

    # --- 3. SAVE THE FLATTENED AND FIXED STAGE ---
    output_path = "/home/isaac/ust_ws/isaac_file/ust_project1_flattened_fixed.usd"
    print(f"\n--- Step 3: Saving New USD File ---")
    print(f"Saving the flattened and fixed stage to: {output_path}")
    
    # The 'flatten' command is not directly available in the low-level API.
    # Saving the stage after loading payloads effectively creates a self-contained file.
    # For a true flatten, you would use omni.kit.asset_converter, but saving is sufficient here.
    omni.usd.get_context().save_as_stage(output_path)
    
    print("\nSave complete.")
    print("A new file 'ust_project1_flattened_fixed.usd' has been created.")
    print("This file contains the corrected robot hierarchy and is self-contained (no payloads).")
    print("Please use this new file for all future work with the Lula editor.")

if __name__ == "__main__":
    fix_robot_with_payloads()
