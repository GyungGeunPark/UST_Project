#!/usr/bin/env python3
"""
USD 파일에 추가 wheel 링크를 복사하는 스크립트

이 스크립트는 Isaac Sim에서 실행되어야 합니다.
실행 방법:
1. Isaac Sim을 실행
2. Script Editor를 열기 (Window -> Script Editor)
3. 이 파일의 내용을 붙여넣고 실행
"""

import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf


def duplicate_wheel(stage, source_wheel_path, new_wheel_name, new_position, new_joint_name=None):
    """
    기존 wheel을 복사하여 새로운 wheel을 생성합니다.

    Args:
        stage: USD Stage
        source_wheel_path: 복사할 원본 wheel의 경로 (예: "/World/turtlebot3_waffle_pi/base/wheel_left_link")
        new_wheel_name: 새로운 wheel의 이름 (예: "wheel_front_left_link")
        new_position: 새로운 wheel의 위치 (x, y, z) 튜플
        new_joint_name: 새로운 조인트 이름 (선택 사항)

    Returns:
        새로 생성된 wheel 프림
    """
    # 원본 wheel 프림 가져오기
    source_prim = stage.GetPrimAtPath(source_wheel_path)
    if not source_prim.IsValid():
        print(f"오류: {source_wheel_path} 경로에 프림이 없습니다.")
        return None

    # 부모 경로 계산
    parent_path = source_wheel_path.rsplit("/", 1)[0]
    new_wheel_path = f"{parent_path}/{new_wheel_name}"

    # 새로운 경로에 이미 프림이 있는지 확인
    if stage.GetPrimAtPath(new_wheel_path).IsValid():
        print(f"경고: {new_wheel_path}에 이미 프림이 존재합니다.")
        return stage.GetPrimAtPath(new_wheel_path)

    # 원본 프림 복사
    # USD에서 프림을 복사하는 방법: 새 프림 생성 후 속성 복사
    parent_prim = stage.GetPrimAtPath(parent_path)
    new_prim = stage.DefinePrim(new_wheel_path, source_prim.GetTypeName())

    # 모든 속성 복사
    for attr in source_prim.GetAttributes():
        attr_name = attr.GetName()
        attr_value = attr.Get()

        if new_prim.HasAttribute(attr_name):
            new_attr = new_prim.GetAttribute(attr_name)
        else:
            new_attr = new_prim.CreateAttribute(attr_name, attr.GetTypeName())

        new_attr.Set(attr_value)

    # 모든 하위 프림 복사 (재귀적으로)
    def copy_prim_hierarchy(src_prim, dst_prim):
        for child in src_prim.GetChildren():
            child_name = child.GetName()
            dst_child_path = dst_prim.GetPath().AppendChild(child_name)
            dst_child = stage.DefinePrim(dst_child_path, child.GetTypeName())

            # 속성 복사
            for attr in child.GetAttributes():
                attr_name = attr.GetName()
                attr_value = attr.Get()

                if dst_child.HasAttribute(attr_name):
                    dst_attr = dst_child.GetAttribute(attr_name)
                else:
                    dst_attr = dst_child.CreateAttribute(attr_name, attr.GetTypeName())

                dst_attr.Set(attr_value)

            # 재귀적으로 하위 프림 복사
            copy_prim_hierarchy(child, dst_child)

    copy_prim_hierarchy(source_prim, new_prim)

    # 위치 설정
    xformable = UsdGeom.Xformable(new_prim)
    if xformable:
        # 기존 transform 연산 가져오기
        ops = xformable.GetOrderedXformOps()

        # translate 연산 찾기 또는 생성
        translate_op = None
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break

        if translate_op is None:
            translate_op = xformable.AddTranslateOp()

        # 새 위치 설정
        translate_op.Set(Gf.Vec3d(*new_position))

    # 조인트 이름 변경 (있는 경우)
    if new_joint_name:
        joint_prim = stage.GetPrimAtPath(f"{new_wheel_path}/joint")
        if joint_prim.IsValid():
            # 조인트 이름은 prim 이름을 바꿔야 함
            # USD에서는 직접 이름 변경이 어려우므로, 속성만 업데이트
            print(f"조인트 이름 변경은 수동으로 해야 합니다: {new_joint_name}")

    print(f"성공적으로 wheel을 복사했습니다: {new_wheel_path}")
    return new_prim


def add_extra_wheels():
    """
    turtlebot3_waffle_pi에 추가 wheel 2개를 추가합니다.
    """
    # USD Stage 가져오기
    stage = omni.usd.get_context().get_stage()

    if not stage:
        print("오류: USD Stage를 가져올 수 없습니다.")
        return

    # 로봇의 base 경로 (실제 프로젝트에 맞게 수정)
    robot_base_path = "/World/turtlebot3_waffle_pi/base"

    # 기존 wheel 경로
    left_wheel_path = f"{robot_base_path}/wheel_left_link"
    right_wheel_path = f"{robot_base_path}/wheel_right_link"

    # 기존 wheel이 있는지 확인
    if not stage.GetPrimAtPath(left_wheel_path).IsValid():
        print(f"오류: {left_wheel_path}를 찾을 수 없습니다.")
        print("robot_base_path를 확인하세요.")
        return

    # 원본 wheel 위치 가져오기
    left_prim = stage.GetPrimAtPath(left_wheel_path)
    left_xform = UsdGeom.Xformable(left_prim)
    left_translate = None
    for op in left_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            left_translate = op.Get()
            break

    right_prim = stage.GetPrimAtPath(right_wheel_path)
    right_xform = UsdGeom.Xformable(right_prim)
    right_translate = None
    for op in right_xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            right_translate = op.Get()
            break

    print(f"기존 왼쪽 wheel 위치: {left_translate}")
    print(f"기존 오른쪽 wheel 위치: {right_translate}")

    # 새로운 wheel 위치 계산 (예: 앞쪽에 추가)
    # 원본 위치에서 x축으로 0.1m 앞으로 이동
    if left_translate:
        new_left_pos = (left_translate[0] + 0.15, left_translate[1], left_translate[2])
    else:
        new_left_pos = (0.15, 0.144, 0.033)

    if right_translate:
        new_right_pos = (right_translate[0] + 0.15, right_translate[1], right_translate[2])
    else:
        new_right_pos = (0.15, -0.144, 0.033)

    print(f"\n새로운 wheel 추가:")
    print(f"  - wheel_front_left_link 위치: {new_left_pos}")
    print(f"  - wheel_front_right_link 위치: {new_right_pos}")

    # 왼쪽 wheel 복사
    new_left_wheel = duplicate_wheel(
        stage=stage,
        source_wheel_path=left_wheel_path,
        new_wheel_name="wheel_front_left_link",
        new_position=new_left_pos,
        new_joint_name="wheel_front_left_joint"
    )

    # 오른쪽 wheel 복사
    new_right_wheel = duplicate_wheel(
        stage=stage,
        source_wheel_path=right_wheel_path,
        new_wheel_name="wheel_front_right_link",
        new_position=new_right_pos,
        new_joint_name="wheel_front_right_joint"
    )

    print("\n완료! 2개의 wheel이 추가되었습니다.")
    print("주의: 물리 시뮬레이션을 위해 조인트와 actuator도 추가로 설정해야 할 수 있습니다.")


if __name__ == "__main__":
    # Isaac Sim에서 실행
    add_extra_wheels()
