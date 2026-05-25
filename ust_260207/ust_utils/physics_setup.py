# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""물리 속성 설정 유틸리티.

all_dev.md 스펙의 create_wheel_material(), set_mass() 등 구현.
USD 파일에 이미 설정된 경우 이 모듈은 불필요합니다.

사용법 (Isaac Sim Script Editor 또는 standalone script):
    from ust_utils.physics_setup import apply_physics_properties
    apply_physics_properties(stage)
"""

from __future__ import annotations


def create_wheel_material(stage, friction: float = 0.8):
    """바퀴용 물리 재질 생성 (all_dev.md 스펙).

    Args:
        stage: USD Stage
        friction: 마찰 계수

    Returns:
        생성된 재질 경로
    """
    from pxr import UsdShade, UsdPhysics

    material_path = "/World/Materials/WheelMaterial"
    UsdShade.Material.Define(stage, material_path)
    material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))
    material.CreateStaticFrictionAttr().Set(friction)
    material.CreateDynamicFrictionAttr().Set(friction)
    material.CreateRestitutionAttr().Set(0.1)
    return material_path


def set_mass(stage, prim_path: str, mass: float, com: tuple = None):
    """질량 및 무게중심 설정 (all_dev.md 스펙).

    Args:
        stage: USD Stage
        prim_path: Prim 경로
        mass: 질량 (kg)
        com: 무게중심 (x, y, z) 또는 None
    """
    from pxr import UsdPhysics, Gf

    mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(prim_path))
    mass_api.CreateMassAttr().Set(mass)
    if com:
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*com))


def setup_velocity_drive(stage, joint_path: str, damping: float = 1e4,
                         max_force: float = 1e6):
    """속도 제어용 Angular Drive 설정 (all_dev.md 스펙).

    Args:
        stage: USD Stage
        joint_path: 조인트 Prim 경로
        damping: 댐핑 값
        max_force: 최대 힘

    Returns:
        생성된 Drive API
    """
    from pxr import UsdPhysics

    joint_prim = stage.GetPrimAtPath(joint_path)
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateDampingAttr().Set(damping)
    drive.CreateStiffnessAttr().Set(0)
    drive.CreateMaxForceAttr().Set(max_force)
    drive.CreateTargetVelocityAttr().Set(0)
    return drive


def apply_physics_properties(stage, robot_path: str = "/World/Robot"):
    """로봇에 물리 속성 일괄 적용.

    TurtleBot3 Waffle Pi 기준 질량 및 마찰력을 설정합니다.
    USD 파일에 이미 설정된 경우 덮어씁니다.

    Args:
        stage: USD Stage
        robot_path: 로봇 루트 경로
    """
    # 바퀴 재질
    wheel_material = create_wheel_material(stage, friction=0.8)

    # 베이스 질량 설정 (TurtleBot3 기준)
    set_mass(stage, f"{robot_path}/base_link", mass=1.5, com=(0, 0, -0.02))

    # 바퀴 질량
    for wheel in ["wheel_left_link", "wheel_right_link"]:
        set_mass(stage, f"{robot_path}/{wheel}", mass=0.1)

    # 암 링크 질량 (OpenMANIPULATOR-X 기준)
    arm_masses = {
        "link1": 0.082,
        "link2": 0.098,
        "link3": 0.088,
        "link4": 0.051,
        "link5": 0.035,
    }
    for link_name, mass_val in arm_masses.items():
        try:
            set_mass(stage, f"{robot_path}/{link_name}", mass=mass_val)
        except Exception:
            pass  # 링크가 없으면 무시

    print("[PhysicsSetup] Properties applied.")
