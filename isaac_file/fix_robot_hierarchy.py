
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

def fix_robot_hierarchy():
    """
    Fixes the hierarchy of the open_manipulator_x1 robot by reparenting links
    and setting the articulation root correctly.
    """
    stage = omni.usd.get_context().get_stage()

    # --- 1. DEFINE PRIM PATHS ---
    # PLEASE VERIFY THESE PATHS ARE CORRECT FOR YOUR STAGE
    world_body_path = "/World"  # Assuming the parent of link1 and link2 is /World
    link1_path = f"{world_body_path}/link1"
    link2_path = f"{world_body_path}/link2"
    
    link1_prim = stage.GetPrimAtPath(link1_path)
    link2_prim = stage.GetPrimAtPath(link2_path)

    if not link1_prim.IsValid() or not link2_prim.IsValid():
        print(f"Error: Prims not found at '{link1_path}' or '{link2_path}'.")
        print("Please verify the prim paths at the beginning of the script.")
        return

    print(f"Found link1 at: {link1_path}")
    print(f"Found link2 at: {link2_path}")

    # --- 2. REPARENT LINK2 TO BE A CHILD OF LINK1 ---
    new_link2_path = f"{link1_path}/link2"
    print(f"Moving '{link2_path}' to '{new_link2_path}'...")

    # omni.kit.commands.execute('MovePrim',
    #     path_from=link2_path,
    #     path_to=new_link2_path)
    
    # Using Sdf is more robust for hierarchy changes
    layer = stage.GetRootLayer()
    with Sdf.ChangeBlock():
        Sdf.MoveSpec(layer, link2_path, layer, new_link2_path)
        
    # Get the new prim for link2 after moving
    link2_prim = stage.GetPrimAtPath(new_link2_path)
    if not link2_prim.IsValid():
        print("Error: Failed to move link2. The hierarchy could not be fixed.")
        return
        
    print("Reparenting successful.")

    # --- 3. REMOVE ALL EXISTING ARTICULATION ROOT APIS ---
    print("Searching for and removing existing Articulation Root APIs...")
    for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath(link1_path)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"Removing ArticulationRootAPI from '{prim.GetPath()}'")
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

    # Also check the original link2 path in case it had one
    old_link2_prim = stage.GetPrimAtPath(link2_path)
    if old_link2_prim.IsValid() and old_link2_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
         print(f"Removing ArticulationRootAPI from '{old_link2_prim.GetPath()}'")
         old_link2_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)


    # --- 4. APPLY ARTICULATION ROOT TO THE NEW BASE LINK (LINK1) ---
    print(f"Applying ArticulationRootAPI to the new base link: '{link1_path}'")
    link1_prim = stage.GetPrimAtPath(link1_path)
    UsdPhysics.ArticulationRootAPI.Apply(link1_prim)

    # --- 5. ADDRESS THE MISSING LINK5 JOINT ---
    # The 'link5' joint not appearing is likely due to the broken hierarchy
    # or an issue in the original USD file. After fixing the hierarchy, 
    # it should appear in the Lula editor.
    #
    # If it still doesn't appear, you need to verify that a joint connecting
    # link4 and link5 exists in the USD file. You can't "create" a joint
    # purely in the Lula editor; it only configures existing ones.
    #
    # To check for a joint, you would look for a prim of type 'PhysicsRevoluteJoint'
    # (or similar) that connects 'link4' and 'link5'.
    link4_path = f"{new_link2_path}/link3/link4" # Example path
    link5_path = f"{link4_path}/link5" # Example path
    
    print("\n--- Joint Verification ---")
    print("After running this script, please check the Lula Robot Description Editor again.")
    print("The 'link5' joint should now be available for configuration.")
    print("If not, the joint definition between link4 and link5 is likely missing or incorrect in your original USD file.")
    
    # --- 6. SAVE THE FIXED STAGE ---
    output_path = "/home/isaac/ust_ws/isaac_file/ust_project1_fixed.usd"
    print(f"\nSaving the fixed stage to: {output_path}")
    omni.usd.get_context().save_as_stage(output_path)
    print("Save complete.")
    print("\nPlease open the new file 'ust_project1_fixed.usd' and use it for your Lula setup.")


# To run this script:
# 1. Open your original 'ust_project1.usd' file in Isaac Sim.
# 2. Open the Script Editor (Window > Script Editor).
# 3. Copy and paste the entire content of this file into the editor.
# 4. Click 'Run'.
# 5. A new file 'ust_project1_fixed.usd' will be created with the corrected hierarchy.
if __name__ == "__main__":
    fix_robot_hierarchy()
