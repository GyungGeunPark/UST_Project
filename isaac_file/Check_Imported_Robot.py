# 임포트된 로봇 확인 스크립트
# Isaac Sim Script Editor에서 실행

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

# 현재 스테이지 가져오기
stage = omni.usd.get_context().get_stage()

print("=" * 70)
print("Checking Imported Robot Structure")
print("=" * 70)

# /World 하위의 모든 프림 검색
world_prim = stage.GetPrimAtPath("/World")
if not world_prim.IsValid():
    print("\n✗ /World prim not found!")
else:
    print("\n✓ /World prim exists")
    print("\nSearching for robot...")

    # /World의 자식들 확인
    found_robot = False
    for child in world_prim.GetChildren():
        child_path = str(child.GetPath())
        child_name = child.GetName()

        print(f"\nFound: {child_path}")
        print(f"  Type: {child.GetTypeName()}")

        # open_manipulator 관련 프림 찾기
        if "manipulator" in child_name.lower() or "open_manipulator" in child_name.lower():
            found_robot = True
            robot_prim = child

            print(f"\n{'=' * 70}")
            print(f"✓ ROBOT FOUND: {child_path}")
            print(f"{'=' * 70}")

            # 로봇의 계층 구조 출력
            print("\nRobot Hierarchy:")

            def print_hierarchy(prim, indent=0):
                prefix = "  " * indent + "└─ " if indent > 0 else ""
                prim_type = prim.GetTypeName()

                # ArticulationRoot 확인
                has_articulation = prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                articulation_mark = " [ArticulationRoot]" if has_articulation else ""

                # Joint 확인
                is_joint = prim.IsA(UsdPhysics.Joint)
                joint_mark = " [Joint]" if is_joint else ""

                print(f"{prefix}{prim.GetName()} ({prim_type}){articulation_mark}{joint_mark}")

                # 최대 5 레벨까지만 출력
                if indent < 5:
                    for child in prim.GetChildren():
                        print_hierarchy(child, indent + 1)

            print_hierarchy(robot_prim)

            # 메쉬 파일 확인
            print("\n" + "=" * 70)
            print("Checking Mesh Files:")
            print("=" * 70)

            mesh_count = 0
            for prim in Usd.PrimRange(robot_prim):
                if prim.IsA(UsdGeom.Mesh):
                    mesh_count += 1
                    print(f"\n  Mesh {mesh_count}: {prim.GetPath()}")

                    # 메쉬 속성 확인
                    mesh = UsdGeom.Mesh(prim)
                    points = mesh.GetPointsAttr().Get()
                    if points:
                        print(f"    ✓ Has geometry ({len(points)} vertices)")
                    else:
                        print(f"    ✗ No geometry data!")

            if mesh_count == 0:
                print("\n  ✗ WARNING: No mesh geometry found!")
                print("  This means mesh files were not loaded correctly.")
                print("  You need to fix the package paths.")
            else:
                print(f"\n  ✓ Total meshes found: {mesh_count}")

            # Physics 확인
            print("\n" + "=" * 70)
            print("Checking Physics Setup:")
            print("=" * 70)

            # ArticulationRoot 확인
            if robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                print("  ✓ Robot has ArticulationRootAPI")
            else:
                print("  ✗ Robot missing ArticulationRootAPI")

            # Joint 개수 확인
            joint_count = 0
            for prim in Usd.PrimRange(robot_prim):
                if prim.IsA(UsdPhysics.Joint):
                    joint_count += 1

            print(f"  ✓ Found {joint_count} joints")

            break

    if not found_robot:
        print("\n✗ No robot found under /World")
        print("\nAll children of /World:")
        for child in world_prim.GetChildren():
            print(f"  - {child.GetPath()}")

print("\n" + "=" * 70)
print("Analysis Complete")
print("=" * 70)
