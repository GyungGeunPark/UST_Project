#!/usr/bin/env python3
"""UST Project USD Fix Script.

ust_project1.usd의 물리 구조를 수정합니다:
1. wapple_pi에 RigidBody, Collision, ArticulationRoot 추가
2. 바퀴 4개 (front/rear, left/right) RevoluteJoint 추가
3. 서브어셈블리를 wapple_pi/base에 FixedJoint로 연결
4. 각 서브어셈블리의 독립 ArticulationRoot 제거 (단일 root)
5. 결과를 ust_project1_fixed.usd로 저장
"""

from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt

INPUT_USD = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1.usd"
OUTPUT_USD = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_fixed.usd"

# TurtleBot3 Waffle Pi 물리 파라미터
WHEEL_RADIUS = 0.033  # m
WHEEL_WIDTH = 0.018   # m
WHEEL_MASS = 0.05     # kg
BASE_MASS = 1.8       # kg (TurtleBot3 본체)

# 바퀴 4개 위치 (로봇 로컬 좌표, base 기준)
# 기존 타이어: left_tire_1 (0.065, 0.141, 0.021), right_tire_1 (0, -0.140, ~0)
# 4바퀴 배치: front/rear × left/right
WHEEL_POSITIONS = {
    "wheel_left_front": Gf.Vec3d(0.065, 0.141, 0.0),
    "wheel_right_front": Gf.Vec3d(0.065, -0.141, 0.0),
    "wheel_left_rear": Gf.Vec3d(-0.065, 0.141, 0.0),
    "wheel_right_rear": Gf.Vec3d(-0.065, -0.141, 0.0),
}

ROBOT_PATH = "/World/Robot"
WAPPLE_PATH = f"{ROBOT_PATH}/wapple_pi"
OMX_PATH = f"{ROBOT_PATH}/open_manipulator_x"
CAM_PATH = f"{ROBOT_PATH}/realsense2_camera"
LIDAR_PATH = f"{ROBOT_PATH}/livox_mid360"


def add_rigid_body(stage, prim_path, mass=None):
    """prim에 RigidBodyAPI 추가."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"  [SKIP] Prim not found: {prim_path}")
        return
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
        print(f"  [ADD] RigidBodyAPI -> {prim_path}")
    if mass is not None:
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(mass)
        print(f"  [ADD] Mass={mass} -> {prim_path}")


def add_collision_box(stage, parent_path, name, size, offset=Gf.Vec3d(0, 0, 0)):
    """Box collision geometry 추가."""
    col_path = f"{parent_path}/collisions"
    col_prim = stage.GetPrimAtPath(col_path)
    if not col_prim.IsValid():
        col_prim = UsdGeom.Xform.Define(stage, col_path).GetPrim()

    box_path = f"{col_path}/{name}"
    box_prim = stage.GetPrimAtPath(box_path)
    if not box_prim.IsValid():
        cube = UsdGeom.Cube.Define(stage, box_path)
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(size[0], size[1], size[2]))
        cube.AddTranslateOp().Set(Gf.Vec3d(offset[0], offset[1], offset[2]))

        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        print(f"  [ADD] Collision box -> {box_path}")


def add_collision_cylinder(stage, parent_path, name, radius, height, offset=Gf.Vec3d(0, 0, 0)):
    """Cylinder collision geometry 추가."""
    col_path = f"{parent_path}/collisions"
    col_prim = stage.GetPrimAtPath(col_path)
    if not col_prim.IsValid():
        UsdGeom.Xform.Define(stage, col_path)

    cyl_path = f"{col_path}/{name}"
    cyl_prim = stage.GetPrimAtPath(cyl_path)
    if not cyl_prim.IsValid():
        cyl = UsdGeom.Cylinder.Define(stage, cyl_path)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(height)
        cyl.CreateAxisAttr("Y")  # 바퀴: Y축이 회전축
        cyl.AddTranslateOp().Set(Gf.Vec3d(offset[0], offset[1], offset[2]))

        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        print(f"  [ADD] Collision cylinder -> {cyl_path}")


def create_wheel_link(stage, wheel_path, position):
    """바퀴 링크 생성 (Xform + RigidBody + Collision)."""
    wheel_xform = UsdGeom.Xform.Define(stage, wheel_path)
    wheel_xform.AddTranslateOp().Set(position)

    # RigidBody + Mass
    wheel_prim = wheel_xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(wheel_prim)
    mass_api = UsdPhysics.MassAPI.Apply(wheel_prim)
    mass_api.CreateMassAttr(WHEEL_MASS)

    # Collision (cylinder)
    add_collision_cylinder(
        stage, wheel_path, "wheel_collision",
        radius=WHEEL_RADIUS, height=WHEEL_WIDTH
    )

    print(f"  [CREATE] Wheel link -> {wheel_path}")


def create_revolute_joint(stage, joint_path, body0_path, body1_path, axis="Y",
                          local_pos0=Gf.Vec3f(0, 0, 0), local_pos1=Gf.Vec3f(0, 0, 0)):
    """RevoluteJoint 생성."""
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

    # body 연결
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])

    # 축 설정
    joint.CreateAxisAttr(axis)

    # 로컬 위치
    joint.CreateLocalPos0Attr(local_pos0)
    joint.CreateLocalPos1Attr(local_pos1)
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    # 무한 회전 (limit 없음)
    # RevoluteJoint에 limit을 설정하지 않으면 기본적으로 무제한

    # Drive (velocity control)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)  # velocity control: stiffness=0
    drive.CreateDampingAttr(10.0)   # damping for velocity control
    drive.CreateMaxForceAttr(100.0)

    print(f"  [CREATE] RevoluteJoint -> {joint_path}")
    print(f"           body0={body0_path}, body1={body1_path}")


def create_fixed_joint(stage, joint_path, body0_path, body1_path,
                       local_pos0=Gf.Vec3f(0, 0, 0), local_pos1=Gf.Vec3f(0, 0, 0)):
    """FixedJoint 생성."""
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])
    joint.CreateLocalPos0Attr(local_pos0)
    joint.CreateLocalPos1Attr(local_pos1)
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    print(f"  [CREATE] FixedJoint -> {joint_path}")
    print(f"           body0={body0_path}, body1={body1_path}")


def remove_articulation_root(stage, prim_path):
    """prim에서 ArticulationRootAPI 제거."""
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        print(f"  [REMOVE] ArticulationRootAPI from {prim_path}")


def fix_joint_body0(stage, joint_path, new_body0_path):
    """Joint의 body0를 수정."""
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        print(f"  [SKIP] Joint not found: {joint_path}")
        return
    rel = prim.GetRelationship("physics:body0")
    if rel:
        rel.SetTargets([Sdf.Path(new_body0_path)])
        print(f"  [FIX] body0 -> {new_body0_path} for {joint_path}")


def main():
    print(f"Opening: {INPUT_USD}")
    stage = Usd.Stage.Open(INPUT_USD)

    # ================================================================
    # STEP 1: wapple_pi base에 물리 속성 추가
    # ================================================================
    print("\n=== STEP 1: Add physics to wapple_pi base ===")

    # wapple_pi를 RigidBody로 설정하지 않고,
    # 내부에 base 링크를 만들어서 ArticulationRoot + RigidBody 설정
    base_path = f"{WAPPLE_PATH}/base"
    base_prim = stage.GetPrimAtPath(base_path)
    if not base_prim.IsValid():
        UsdGeom.Xform.Define(stage, base_path)
        print(f"  [CREATE] Xform -> {base_path}")

    # ArticulationRoot는 base에 설정
    base_prim = stage.GetPrimAtPath(base_path)
    if not base_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(base_prim)
        print(f"  [ADD] ArticulationRootAPI -> {base_path}")

    # RigidBody + Mass
    add_rigid_body(stage, base_path, mass=BASE_MASS)

    # Base collision (box approximation: 약 28cm x 30cm x 14cm)
    add_collision_box(stage, base_path, "base_collision",
                      size=(0.28, 0.30, 0.14), offset=Gf.Vec3d(0, 0, 0.07))

    # waffle_pi_base 비주얼을 base 아래로 이동시키진 않고,
    # 대신 waffle_pi_base에 대한 참조를 유지 (비주얼은 wapple_pi 아래 그대로)

    # ================================================================
    # STEP 2: 바퀴 4개 생성
    # ================================================================
    print("\n=== STEP 2: Create 4 wheel links ===")

    for wheel_name, pos in WHEEL_POSITIONS.items():
        wheel_path = f"{WAPPLE_PATH}/{wheel_name}"
        # 기존 타이어 비주얼 prim이 있으면 건너뜀 (직접 생성)
        existing = stage.GetPrimAtPath(wheel_path)
        if existing.IsValid():
            print(f"  [EXISTS] {wheel_path} - adding physics")
            prim = existing
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI.Apply(prim)
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr(WHEEL_MASS)
            add_collision_cylinder(stage, wheel_path, "wheel_collision",
                                   radius=WHEEL_RADIUS, height=WHEEL_WIDTH)
        else:
            create_wheel_link(stage, wheel_path, pos)

    # ================================================================
    # STEP 3: 바퀴 RevoluteJoint 4개 생성
    # ================================================================
    print("\n=== STEP 3: Create 4 wheel revolute joints ===")

    joints_path = f"{WAPPLE_PATH}/joints"
    joints_prim = stage.GetPrimAtPath(joints_path)
    if not joints_prim.IsValid():
        UsdGeom.Scope.Define(stage, joints_path)
        print(f"  [CREATE] Scope -> {joints_path}")

    for wheel_name, pos in WHEEL_POSITIONS.items():
        joint_name = f"{wheel_name}_joint"
        joint_path = f"{joints_path}/{joint_name}"

        # 기존 joint가 있으면 삭제 후 재생성
        existing_joint = stage.GetPrimAtPath(joint_path)
        if existing_joint.IsValid():
            stage.RemovePrim(joint_path)

        create_revolute_joint(
            stage,
            joint_path=joint_path,
            body0_path=base_path,
            body1_path=f"{WAPPLE_PATH}/{wheel_name}",
            axis="Y",  # Y축 회전 (좌우 방향)
            local_pos0=Gf.Vec3f(float(pos[0]), float(pos[1]), float(pos[2])),
            local_pos1=Gf.Vec3f(0, 0, 0),
        )

    # ================================================================
    # STEP 4: 서브어셈블리의 ArticulationRoot 제거
    # ================================================================
    print("\n=== STEP 4: Remove sub-assembly ArticulationRoots ===")

    # 각 서브어셈블리의 root_joint에 있는 ArticulationRoot 제거
    for sub_path in [OMX_PATH, CAM_PATH, LIDAR_PATH]:
        root_joint_path = f"{sub_path}/root_joint"
        remove_articulation_root(stage, root_joint_path)

        # 각 서브어셈블리 내부의 다른 ArticulationRoot도 제거
        sub_prim = stage.GetPrimAtPath(sub_path)
        if sub_prim.IsValid():
            for prim in Usd.PrimRange(sub_prim):
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    path = str(prim.GetPath())
                    if path != base_path:  # wapple_pi/base는 유지
                        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                        print(f"  [REMOVE] ArticulationRootAPI from {path}")

    # ================================================================
    # STEP 5: 서브어셈블리 root_joint의 body0를 wapple_pi/base로 수정
    # ================================================================
    print("\n=== STEP 5: Fix sub-assembly root_joint body0 ===")

    # realsense2_camera: body0 -> wapple_pi/base (기존: wapple_pi)
    fix_joint_body0(stage, f"{CAM_PATH}/root_joint", base_path)

    # livox_mid360: body0 -> wapple_pi/base (기존: wapple_pi)
    fix_joint_body0(stage, f"{LIDAR_PATH}/root_joint", base_path)

    # open_manipulator_x: body0 -> wapple_pi/base (기존: empty!)
    fix_joint_body0(stage, f"{OMX_PATH}/root_joint", base_path)

    # ================================================================
    # STEP 6: open_manipulator_x 내부 RigidBody 확인
    # ================================================================
    print("\n=== STEP 6: Verify open_manipulator_x RigidBodies ===")

    omx_prim = stage.GetPrimAtPath(OMX_PATH)
    if omx_prim.IsValid():
        for prim in Usd.PrimRange(omx_prim):
            path = str(prim.GetPath())
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                print(f"  [OK] RigidBody: {path}")

    # ================================================================
    # STEP 7: defaultPrim 설정
    # ================================================================
    print("\n=== STEP 7: Set defaultPrim ===")

    layer = stage.GetRootLayer()
    layer.defaultPrim = "World"
    print(f"  [SET] defaultPrim = 'World'")

    # ================================================================
    # SAVE
    # ================================================================
    print(f"\n=== Saving to: {OUTPUT_USD} ===")
    stage.GetRootLayer().Export(OUTPUT_USD)
    print("DONE!")

    # ================================================================
    # VERIFY
    # ================================================================
    print("\n=== VERIFICATION ===")
    verify_stage = Usd.Stage.Open(OUTPUT_USD)
    robot = verify_stage.GetPrimAtPath(ROBOT_PATH)

    art_roots = []
    joints = []
    rigid_bodies = []

    for prim in Usd.PrimRange(robot):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_roots.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(str(prim.GetPath()))
        if "Joint" in prim.GetTypeName():
            joints.append(str(prim.GetPath()))

    print(f"\nArticulationRoots ({len(art_roots)}):")
    for p in art_roots:
        print(f"  {p}")

    print(f"\nRigidBodies ({len(rigid_bodies)}):")
    for p in rigid_bodies:
        print(f"  {p}")

    print(f"\nJoints ({len(joints)}):")
    for p in joints:
        prim = verify_stage.GetPrimAtPath(p)
        b0 = prim.GetRelationship("physics:body0").GetTargets() if prim.GetRelationship("physics:body0") else []
        b1 = prim.GetRelationship("physics:body1").GetTargets() if prim.GetRelationship("physics:body1") else []
        print(f"  {p} ({prim.GetTypeName()})")
        print(f"    body0: {b0}")
        print(f"    body1: {b1}")


if __name__ == "__main__":
    main()
