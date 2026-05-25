# Isaac Sim URDF Import Script (모든 버전 호환)
# Script Editor에서 실행하세요

import omni.kit.commands
import omni.usd
from pxr import UsdPhysics

# URDF 파일 경로
urdf_path = "/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf"

# 메쉬 파일이 있는 패키지 경로
mesh_base_path = "/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description"

print("=" * 70)
print("Starting URDF Import for Open Manipulator X")
print("=" * 70)
print(f"\nURDF file: {urdf_path}")
print(f"Mesh base: {mesh_base_path}")
print()

# 방법 1: URDFParseAndImportFile 명령 사용 (Isaac Sim 2022.2 이상)
try:
    print("Method 1: Trying URDFParseAndImportFile command...")

    # Import config 딕셔너리로 설정
    result = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config={
            'merge_fixed_joints': False,
            'convex_decomp': False,
            'import_inertia_tensor': True,
            'fix_base': True,
            'make_default_prim': True,
            'self_collision': True,
            'create_physics_scene': True,
            'distance_scale': 1.0
        },
        dest_path="",
        package_map={
            "open_manipulator_description": mesh_base_path
        }
    )

    if result[0]:  # result is tuple (success, prim_path)
        print(f"\n✓ SUCCESS! Import completed!")
        print(f"✓ Robot imported at: {result[1]}")
        print(f"\nCheck the Stage window to see your robot!")
        print(f"Save your scene: File > Save As...")
    else:
        raise Exception("Import returned False")

except Exception as e1:
    print(f"Method 1 failed: {e1}")
    print("\nMethod 2: Trying URDFCreateImportConfig command...")

    # 방법 2: URDFCreateImportConfig + URDFParseFile 사용
    try:
        # Import config 생성
        config_result = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config = config_result[1]

        # Config 설정
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.self_collision = True
        import_config.create_physics_scene = True
        import_config.distance_scale = 1.0

        # URDF 파싱 및 임포트
        result = omni.kit.commands.execute(
            "URDFParseFile",
            urdf_path=urdf_path,
            import_config=import_config,
            dest_path="",
            package_map={
                "open_manipulator_description": mesh_base_path
            }
        )

        print(f"\n✓ SUCCESS! Import completed!")
        print(f"✓ Robot imported")
        print(f"\nCheck the Stage window to see your robot!")

    except Exception as e2:
        print(f"Method 2 failed: {e2}")
        print("\nMethod 3: Trying simple URDF import...")

        # 방법 3: 가장 기본적인 방법
        try:
            result = omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=urdf_path,
                import_config=None,  # Use defaults
                dest_path="/World/open_manipulator_x"
            )

            print(f"\n✓ SUCCESS! Import completed with default settings!")
            print(f"\nCheck the Stage window to see your robot!")

        except Exception as e3:
            print(f"Method 3 failed: {e3}")
            print("\n" + "=" * 70)
            print("ALL METHODS FAILED")
            print("=" * 70)
            print("\nPlease try the following:")
            print("1. Check if URDF extension is enabled:")
            print("   Window > Extensions > Search 'urdf' > Enable all")
            print()
            print("2. Check Isaac Sim version:")
            print("   Help > About")
            print()
            print("3. Try GUI method:")
            print("   Isaac Utils > URDF Importer")
            print()
            print("Error details:")
            print(f"  Error 1: {e1}")
            print(f"  Error 2: {e2}")
            print(f"  Error 3: {e3}")

print("\n" + "=" * 70)
