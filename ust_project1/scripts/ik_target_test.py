# Script Editor에서 실행
# 먼저 로봇이 스테이지에 로드되어 있고, Play 상태여야 함

import numpy as np
from pxr import Usd, UsdGeom
import omni.usd
import omni.timeline

# Isaac Sim 버전에 따른 import (구버전: omni.isaac.core)
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver
)

# 경로 설정 (자신의 경로로 수정!)
ROBOT_PRIM_PATH = "/World/Robot/open_manipulator_x"  # 스테이지의 로봇 경로
ROBOT_DESCRIPTION_PATH = "/home/isaac/ust_ws/ust_project1/config/open_x1_des.yaml"
URDF_PATH = "/home/isaac/ust_ws/ust_project1/config/open_manipulator_x.urdf"
END_EFFECTOR_FRAME = "end_effector_link"
ROBOT_NAME = "open_manipulator_x"

# 시뮬레이션이 실행 중인지 확인
timeline = omni.timeline.get_timeline_interface()
if not timeline.is_playing():
    print("⚠️ 먼저 Play 버튼을 눌러 시뮬레이션을 시작하세요!")
else:
    # World 인스턴스 가져오기 또는 생성
    world = World.instance()
    if world is None:
        world = World()

    # 이미 scene에 추가된 로봇이 있는지 확인
    robot = world.scene.get_object(ROBOT_NAME)
    if robot is None:
        # 없으면 새로 추가
        robot = world.scene.add(Articulation(prim_path=ROBOT_PRIM_PATH, name=ROBOT_NAME))
        # 물리 핸들 초기화를 위해 reset 호출
        world.reset()
    else:
        print(f"기존 로봇 '{ROBOT_NAME}'을 재사용합니다.")

    # Lula Solver 초기화
    kinematics_solver = LulaKinematicsSolver(
        robot_description_path=ROBOT_DESCRIPTION_PATH,
        urdf_path=URDF_PATH
    )

    # 사용 가능한 프레임 확인
    print("사용 가능한 프레임:", kinematics_solver.get_all_frame_names())

    # ArticulationKinematicsSolver 초기화
    art_solver = ArticulationKinematicsSolver(
        robot,
        kinematics_solver,
        END_EFFECTOR_FRAME
    )

    # 현재 엔드이펙터 위치 먼저 확인
    ee_pos, ee_rot = art_solver.compute_end_effector_pose()
    print(f"현재 EE 위치: {ee_pos}")

    # 테스트: OpenMANIPULATOR-X 작업 공간 내의 안전한 위치로 IK 계산
    # OpenMANIPULATOR-X 작업 범위: 약 0.1m ~ 0.38m (베이스에서)
    # 더 안전한 타겟 위치 설정 (로봇 전방, 낮은 높이)
    target_position = np.array([0.2, 0.0, 0.15])  # x=전방, y=중앙, z=높이

    print(f"타겟 위치: {target_position}")

    action, success = art_solver.compute_inverse_kinematics(
        target_position,
        target_orientation=None  # 4축 로봇은 orientation 생략
    )

    if success and action is not None:
        print(f"IK 성공! 조인트 위치: {action.joint_positions}")
        robot.apply_action(action)
    else:
        print("IK 실패 - 타겟이 작업 공간 외부이거나 도달 불가능한 위치입니다.")
        print("다른 타겟 위치를 시도해보세요.")

        # 대안: 현재 위치 근처에서 작은 변화 시도
        if ee_pos is not None:
            alt_target = np.array([ee_pos[0] + 0.02, ee_pos[1], ee_pos[2]])
            print(f"대안 타겟 시도: {alt_target}")
            action2, success2 = art_solver.compute_inverse_kinematics(
                alt_target,
                target_orientation=None
            )
            if success2 and action2 is not None:
                print(f"대안 IK 성공! 조인트 위치: {action2.joint_positions}")
                robot.apply_action(action2)
            else:
                print("대안 IK도 실패. robot_description.yaml 설정을 확인하세요.")
