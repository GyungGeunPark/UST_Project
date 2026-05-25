# Isaac Sim URDF Import Script v2
# 이 버전은 Isaac Sim의 실제 명령 시그니처에 맞춰 수정됨

import omni.kit.commands
import omni.usd
from pxr import Sdf, UsdGeom
import os

# URDF 파일 경로
urdf_path = "/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf"

# 메쉬 파일 디렉토리
mesh_dir = "/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/meshes/open_manipulator_x"

print("=" * 70)
print("URDF Import v2 - Using correct command signatures")
print("=" * 70)
print(f"\nURDF file: {urdf_path}")
print(f"Mesh dir: {mesh_dir}")

# URDF 파일에서 package:// 경로를 실제 파일 경로로 변환
print("\nStep 1: Preparing URDF with absolute mesh paths...")

with open(urdf_path, 'r') as f:
    urdf_content = f.read()

# package://open_manipulator_description를 실제 경로로 치환
urdf_content_fixed = urdf_content.replace(
    'package://open_manipulator_description/meshes',
    '/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/meshes'
)

# 임시 URDF 파일 생성
temp_urdf_path = "/home/isaac/ust_ws/isaac_file/temp_open_manipulator_x.urdf"
with open(temp_urdf_path, 'w') as f:
    f.write(urdf_content_fixed)

print(f"✓ Created temporary URDF with absolute paths: {temp_urdf_path}")

# 방법 1: dest_path만 사용
try:
    print("\nStep 2: Importing URDF...")

    result = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=temp_urdf_path,
        dest_path="/World/open_manipulator_x"
    )

    print(f"\n{'=' * 70}")
    print("✓ SUCCESS! Robot imported!")
    print(f"{'=' * 70}")
    print(f"\nRobot path: /World/open_manipulator_x")
    print("\nNext steps:")
    print("1. Check the Stage window - you should see the robot hierarchy")
    print("2. Click Play button to test physics")
    print("3. Save your scene: File > Save As...")
    print("   Suggested name: open_manipulator_x_from_urdf.usd")
    print()
    print("Then you can configure Lula Robot Description Editor!")

except Exception as e:
    print(f"\n✗ Import failed: {e}")
    print("\nTrying alternative method...")

    # 대안: URDFParseFile 사용
    try:
        result = omni.kit.commands.execute(
            "URDFParseFile",
            urdf_path=temp_urdf_path
        )
        print("\n✓ SUCCESS with URDFParseFile!")

    except Exception as e2:
        print(f"✗ Alternative method also failed: {e2}")
        print("\nThe robot might still be imported. Check /World in Stage window.")

print("\n" + "=" * 70)
