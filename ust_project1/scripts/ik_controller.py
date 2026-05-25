"""
OpenMANIPULATOR-X IK Controller for Isaac Sim
상시 사용을 위한 프로덕션용 IK 컨트롤러

사용법:
    1. Isaac Sim에서 로봇이 로드된 상태에서 Play 버튼 클릭
    2. Script Editor에서 이 스크립트 실행
    3. IKController 인스턴스를 생성하여 사용

예시:
    controller = IKController()
    controller.move_to_position([0.2, 0.0, 0.15])
"""

import numpy as np
from typing import Optional, Tuple, List
import omni.timeline

# Isaac Sim imports (구버전: omni.isaac.core)
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver
)


class IKController:
    """
    OpenMANIPULATOR-X IK 컨트롤러

    4축 매니퓰레이터를 위한 역운동학 컨트롤러입니다.
    위치 기반 제어만 지원합니다 (orientation 제어 불가).

    Attributes:
        robot: Isaac Sim Articulation 객체
        is_initialized: 초기화 완료 여부
        end_effector_frame: 사용 중인 엔드이펙터 프레임 이름
    """

    # 기본 설정
    DEFAULT_ROBOT_PRIM_PATH = "/World/Robot/open_manipulator_x"
    DEFAULT_ROBOT_NAME = "open_manipulator_x"
    DEFAULT_CONFIG_DIR = "/home/isaac/ust_ws/ust_project1/config"
    DEFAULT_ROBOT_DESC = "open_x1_des.yaml"
    DEFAULT_URDF = "open_manipulator_x.urdf"
    DEFAULT_EE_FRAME = "end_effector_link"

    # 작업 공간 제한 (미터)
    WORKSPACE_MIN_RADIUS = 0.08  # 최소 도달 거리
    WORKSPACE_MAX_RADIUS = 0.38  # 최대 도달 거리
    WORKSPACE_MIN_HEIGHT = -0.05  # 최소 높이 (베이스 기준)
    WORKSPACE_MAX_HEIGHT = 0.40  # 최대 높이

    def __init__(
        self,
        robot_prim_path: str = None,
        config_dir: str = None,
        robot_description_file: str = None,
        urdf_file: str = None,
        end_effector_frame: str = None,
        auto_initialize: bool = True
    ):
        """
        IK 컨트롤러 초기화

        Args:
            robot_prim_path: 스테이지 내 로봇 Prim 경로
            config_dir: 설정 파일 디렉토리
            robot_description_file: Lula robot description yaml 파일명
            urdf_file: URDF 파일명
            end_effector_frame: 엔드이펙터 프레임 이름
            auto_initialize: 자동 초기화 여부
        """
        # 경로 설정
        self._robot_prim_path = robot_prim_path or self.DEFAULT_ROBOT_PRIM_PATH
        self._config_dir = config_dir or self.DEFAULT_CONFIG_DIR
        self._robot_desc_file = robot_description_file or self.DEFAULT_ROBOT_DESC
        self._urdf_file = urdf_file or self.DEFAULT_URDF
        self._ee_frame = end_effector_frame or self.DEFAULT_EE_FRAME

        # 내부 상태
        self._world: Optional[World] = None
        self._robot: Optional[Articulation] = None
        self._kinematics_solver: Optional[LulaKinematicsSolver] = None
        self._art_solver: Optional[ArticulationKinematicsSolver] = None
        self._is_initialized = False
        self._available_frames: List[str] = []

        if auto_initialize:
            self.initialize()

    @property
    def robot(self) -> Optional[Articulation]:
        """로봇 Articulation 객체"""
        return self._robot

    @property
    def is_initialized(self) -> bool:
        """초기화 완료 여부"""
        return self._is_initialized

    @property
    def end_effector_frame(self) -> str:
        """현재 사용 중인 엔드이펙터 프레임"""
        return self._ee_frame

    @property
    def available_frames(self) -> List[str]:
        """사용 가능한 프레임 목록"""
        return self._available_frames.copy()

    def initialize(self) -> bool:
        """
        컨트롤러 초기화

        Returns:
            성공 여부
        """
        # 시뮬레이션 상태 확인
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            print("[IKController] 경고: 시뮬레이션이 Play 상태가 아닙니다.")
            print("[IKController] Play 버튼을 누른 후 다시 시도하세요.")
            return False

        try:
            # World 인스턴스 가져오기
            self._world = World.instance()
            if self._world is None:
                self._world = World()

            # 로봇 가져오기 (이미 scene에 있으면 재사용)
            self._robot = self._world.scene.get_object(self.DEFAULT_ROBOT_NAME)
            if self._robot is None:
                self._robot = self._world.scene.add(
                    Articulation(
                        prim_path=self._robot_prim_path,
                        name=self.DEFAULT_ROBOT_NAME
                    )
                )

            # 항상 reset 호출하여 Physics View 생성 보장
            self._world.reset()

            # 파일 경로 구성
            robot_desc_path = f"{self._config_dir}/{self._robot_desc_file}"
            urdf_path = f"{self._config_dir}/{self._urdf_file}"

            # Lula Kinematics Solver 초기화
            self._kinematics_solver = LulaKinematicsSolver(
                robot_description_path=robot_desc_path,
                urdf_path=urdf_path
            )

            # 사용 가능한 프레임 확인
            self._available_frames = self._kinematics_solver.get_all_frame_names()
            print(f"[IKController] 사용 가능한 프레임: {self._available_frames}")

            # 엔드이펙터 프레임 검증
            if self._ee_frame not in self._available_frames:
                print(f"[IKController] 경고: '{self._ee_frame}' 프레임을 찾을 수 없음")
                # fallback: 마지막 프레임 사용
                self._ee_frame = self._available_frames[-1]
                print(f"[IKController] '{self._ee_frame}'을 엔드이펙터로 사용")

            # Articulation Kinematics Solver 초기화
            self._art_solver = ArticulationKinematicsSolver(
                self._robot,
                self._kinematics_solver,
                self._ee_frame
            )

            # IK 설정 최적화 (버전에 따라 메서드가 없을 수 있음)
            try:
                if hasattr(self._kinematics_solver, 'set_max_iterations'):
                    self._kinematics_solver.set_max_iterations(200)
            except Exception:
                pass  # 해당 메서드가 없는 버전에서는 무시

            self._is_initialized = True
            print(f"[IKController] 초기화 완료")
            print(f"[IKController] 엔드이펙터: {self._ee_frame}")

            return True

        except Exception as e:
            print(f"[IKController] 초기화 실패: {e}")
            self._is_initialized = False
            return False

    def _ensure_robot_initialized(self) -> bool:
        """
        로봇 Articulation이 완전히 초기화되었는지 확인

        Returns:
            초기화 완료 여부
        """
        if not self._is_initialized or self._robot is None:
            return False

        try:
            # 조인트 위치를 읽어서 초기화 상태 확인
            joint_pos = self._robot.get_joint_positions()
            return joint_pos is not None and len(joint_pos) > 0
        except Exception:
            return False

    def get_end_effector_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        현재 엔드이펙터 위치와 방향 가져오기

        Returns:
            (position, orientation) 튜플
            position: [x, y, z] numpy array
            orientation: [qw, qx, qy, qz] quaternion numpy array
        """
        if not self._is_initialized:
            print("[IKController] 초기화되지 않음")
            return None, None

        if not self._ensure_robot_initialized():
            print("[IKController] 로봇 Articulation이 아직 초기화되지 않았습니다.")
            print("[IKController] 시뮬레이션이 한 스텝 이상 실행된 후 다시 시도하세요.")
            return None, None

        try:
            return self._art_solver.compute_end_effector_pose()
        except Exception as e:
            print(f"[IKController] FK 계산 실패: {e}")
            return None, None

    def is_position_reachable(self, position: np.ndarray) -> bool:
        """
        지정된 위치가 작업 공간 내에 있는지 확인

        Args:
            position: [x, y, z] 타겟 위치

        Returns:
            도달 가능 여부
        """
        if not self._is_initialized:
            return False

        try:
            # 로봇 베이스 위치 가져오기
            robot_base_pos, _ = self._robot.get_world_pose()

            # XY 평면에서의 거리 계산
            dx = position[0] - robot_base_pos[0]
            dy = position[1] - robot_base_pos[1]
            horizontal_distance = np.sqrt(dx**2 + dy**2)

            # 높이 계산
            height = position[2] - robot_base_pos[2]

            # 작업 공간 검사
            if horizontal_distance < self.WORKSPACE_MIN_RADIUS:
                return False
            if horizontal_distance > self.WORKSPACE_MAX_RADIUS:
                return False
            if height < self.WORKSPACE_MIN_HEIGHT:
                return False
            if height > self.WORKSPACE_MAX_HEIGHT:
                return False

            return True
        except Exception:
            return False

    def compute_ik(
        self,
        target_position: np.ndarray,
        check_workspace: bool = True
    ) -> Tuple[Optional[np.ndarray], bool]:
        """
        역운동학 계산 (조인트 위치 반환)

        Args:
            target_position: [x, y, z] 타겟 위치
            check_workspace: 작업 공간 검사 수행 여부

        Returns:
            (joint_positions, success) 튜플
            joint_positions: 조인트 위치 배열 (실패 시 None)
            success: IK 성공 여부
        """
        if not self._is_initialized:
            print("[IKController] 초기화되지 않음")
            return None, False

        if not self._ensure_robot_initialized():
            print("[IKController] 로봇이 아직 초기화되지 않았습니다.")
            return None, False

        target = np.array(target_position)

        # 작업 공간 검사
        if check_workspace and not self.is_position_reachable(target):
            print(f"[IKController] 타겟이 작업 공간 외부: {target}")
            return None, False

        try:
            # 로봇 베이스 위치 업데이트
            robot_base_pos, robot_base_ori = self._robot.get_world_pose()
            self._kinematics_solver.set_robot_base_pose(robot_base_pos, robot_base_ori)

            # IK 계산 (4축 로봇: orientation=None)
            action, success = self._art_solver.compute_inverse_kinematics(
                target,
                target_orientation=None  # 위치만 제어
            )

            if success and action is not None:
                return action.joint_positions, True
            else:
                return None, False
        except Exception as e:
            print(f"[IKController] IK 계산 실패: {e}")
            return None, False

    def move_to_position(
        self,
        target_position: np.ndarray,
        check_workspace: bool = True
    ) -> bool:
        """
        지정된 위치로 로봇 이동

        Args:
            target_position: [x, y, z] 타겟 위치
            check_workspace: 작업 공간 검사 수행 여부

        Returns:
            성공 여부
        """
        if not self._is_initialized:
            print("[IKController] 초기화되지 않음")
            return False

        if not self._ensure_robot_initialized():
            print("[IKController] 로봇이 아직 초기화되지 않았습니다.")
            return False

        target = np.array(target_position)

        # 작업 공간 검사
        if check_workspace and not self.is_position_reachable(target):
            print(f"[IKController] 타겟이 작업 공간 외부: {target}")
            return False

        try:
            # 로봇 베이스 위치 업데이트
            robot_base_pos, robot_base_ori = self._robot.get_world_pose()
            self._kinematics_solver.set_robot_base_pose(robot_base_pos, robot_base_ori)

            # IK 계산
            action, success = self._art_solver.compute_inverse_kinematics(
                target,
                target_orientation=None
            )

            if success and action is not None:
                self._robot.apply_action(action)
                return True
            else:
                print(f"[IKController] IK 실패: {target}")
                return False
        except Exception as e:
            print(f"[IKController] 이동 실패: {e}")
            return False

    def move_to_joint_positions(self, joint_positions: np.ndarray) -> bool:
        """
        지정된 조인트 위치로 로봇 이동 (직접 조인트 제어)

        Args:
            joint_positions: 조인트 위치 배열 [joint1, joint2, joint3, joint4]

        Returns:
            성공 여부
        """
        if not self._is_initialized:
            print("[IKController] 초기화되지 않음")
            return False

        if not self._ensure_robot_initialized():
            print("[IKController] 로봇이 아직 초기화되지 않았습니다.")
            return False

        try:
            from omni.isaac.core.utils.types import ArticulationAction

            action = ArticulationAction(joint_positions=np.array(joint_positions))
            self._robot.apply_action(action)
            return True
        except Exception as e:
            print(f"[IKController] 조인트 이동 실패: {e}")
            return False

    def get_joint_positions(self) -> Optional[np.ndarray]:
        """
        현재 조인트 위치 가져오기

        Returns:
            조인트 위치 배열
        """
        if not self._is_initialized:
            return None

        if not self._ensure_robot_initialized():
            return None

        try:
            return self._robot.get_joint_positions()
        except Exception as e:
            print(f"[IKController] 조인트 위치 읽기 실패: {e}")
            return None

    def go_home(self) -> bool:
        """
        홈 위치로 이동 (기본 자세)

        Returns:
            성공 여부
        """
        # 기본 조인트 위치 (robot_description.yaml의 default_q)
        home_position = np.array([0.0, 0.0, 0.0, 0.0])
        return self.move_to_joint_positions(home_position)

    def go_ready(self) -> bool:
        """
        Ready 자세로 이동 (작업 준비 자세)

        Returns:
            성공 여부
        """
        # 작업하기 좋은 자세
        ready_position = np.array([0.0, -0.5, 0.5, 0.0])
        return self.move_to_joint_positions(ready_position)


# =============================================================================
# 전역 인스턴스 (편의를 위해)
# =============================================================================
_controller_instance: Optional[IKController] = None


def get_controller() -> IKController:
    """
    전역 IK 컨트롤러 인스턴스 가져오기

    Returns:
        IKController 인스턴스
    """
    global _controller_instance
    if _controller_instance is None or not _controller_instance.is_initialized:
        _controller_instance = IKController()
    return _controller_instance


def move_to(x: float, y: float, z: float) -> bool:
    """
    간편 함수: 지정된 위치로 이동

    Args:
        x, y, z: 타겟 좌표 (미터)

    Returns:
        성공 여부
    """
    controller = get_controller()
    return controller.move_to_position(np.array([x, y, z]))


def get_ee_position() -> Optional[np.ndarray]:
    """
    간편 함수: 현재 엔드이펙터 위치 가져오기

    Returns:
        [x, y, z] position array
    """
    controller = get_controller()
    pos, _ = controller.get_end_effector_pose()
    return pos


# =============================================================================
# 테스트/데모용 코드
# =============================================================================
if __name__ == "__main__" or True:  # Script Editor에서 실행 시
    print("=" * 50)
    print("OpenMANIPULATOR-X IK Controller")
    print("=" * 50)

    # 컨트롤러 초기화
    controller = IKController()

    if controller.is_initialized:
        print("\n[IKController] 컨트롤러가 초기화되었습니다.")
        print("[IKController] 시뮬레이션이 몇 스텝 실행된 후 아래 명령을 사용하세요:")
        print("\n사용 예시:")
        print("  controller.move_to_position([0.2, 0.0, 0.15])")
        print("  controller.go_home()")
        print("  controller.go_ready()")
        print("  pos, rot = controller.get_end_effector_pose()")
        print("  joints = controller.get_joint_positions()")
        print("\n주의: FK/IK 기능은 시뮬레이션이 실행 중일 때만 동작합니다.")
