#!/usr/bin/env python3
"""Check arm physics USD files for gripper joint DriveAPI and MimicAPI schemas."""

from pxr import Usd, UsdPhysics, Sdf

for side in ["right", "left"]:
    physics_path = f"/workspace/isaaclab/ust_ws/isaac_file/open_manipulator_x_{side}/configuration/open_manipulator_x_physics.usd"
    print(f"\n{'='*60}")
    print(f"ARM PHYSICS USD: {side}")
    print(f"{'='*60}")

    layer = Sdf.Layer.FindOrOpen(physics_path)
    if not layer:
        print(f"  ERROR: Could not open {physics_path}")
        continue

    # Traverse all prims in the layer
    def traverse_layer(spec, indent=0):
        prefix = "  " * indent
        if spec.name and "gripper" in spec.name.lower():
            print(f"{prefix}{spec.path}")
            print(f"{prefix}  specifier: {spec.specifier}")
            api = spec.GetInfo("apiSchemas")
            if api:
                print(f"{prefix}  apiSchemas: {api}")
                print(f"{prefix}    isExplicit: {api.isExplicit}")
                print(f"{prefix}    prependedItems: {api.prependedItems}")
                print(f"{prefix}    explicitItems: {api.explicitItems}")
            else:
                print(f"{prefix}  apiSchemas: (none)")

            # Check properties
            for prop in spec.properties:
                print(f"{prefix}  prop: {prop.name} = {prop.default}")

            # Check relationships
            for prop in spec.properties:
                if isinstance(prop, Sdf.RelationshipSpec):
                    print(f"{prefix}  rel: {prop.name} -> {prop.targetPathList}")

        for child in spec.nameChildren:
            traverse_layer(child, indent + 1)

    traverse_layer(layer.pseudoRoot)

    # Also open as stage to check composed view
    stage = Usd.Stage.Open(physics_path)
    if stage:
        print(f"\n  Composed view:")
        for prim in stage.TraverseAll():
            if "gripper" in prim.GetName().lower():
                schemas = list(prim.GetAppliedSchemas())
                print(f"  {prim.GetPath()}: schemas={schemas}")
                for attr in prim.GetAttributes():
                    val = attr.Get()
                    if val is not None and "drive" in attr.GetName().lower():
                        print(f"    {attr.GetName()} = {val}")
