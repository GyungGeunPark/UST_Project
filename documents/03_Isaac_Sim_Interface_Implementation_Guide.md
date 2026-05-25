# Isaac Sim LLM Robot Control - Isaac Sim Interface Implementation Guide

## 1. 문서 개요

### 1.1 목적
본 문서는 Isaac Sim 환경에서 로봇을 제어하기 위한 인터페이스 계층의 상세 구현 가이드를 제공합니다. Articulation Controller, IK 솔버, 모바일 베이스, 그리퍼 제어를 포함합니다.

### 1.2 범위
- `robot_controller.py`: 메인 로봇 컨트롤러
- `mobile_base.py`: 4륜 구동 모바일 베이스 제어
- `manipulator.py`: 매니퓰레이터 IK 제어
- `gripper.py`: 그리퍼 제어
- `ik_solver.py`: IK 솔버 래퍼

### 1.3 사전 요구사항
- Isaac Sim 4.2.0 이상
- Python 3.10
- Lula Robot Description 파일 (.yaml)
- 로봇 URDF/USD 파일

---

## 2. 개요: Isaac Sim Robot Control API

### 2.1 핵심 개념

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Isaac Sim Robot Control                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────┐    ┌──────────────────┐    ┌───────────────┐ │
│  │   USD Stage      │───▶│   Articulation   │───▶│ PhysX Joint   │ │
│  │   (Robot Prim)   │    │   (Python API)   │    │ (Simulation)  │ │
│  └──────────────────┘    └──────────────────┘    └───────────────┘ │
│           │                       │                                 │
│           │                       ▼                                 │
│           │              ┌──────────────────┐                      │
│           │              │ Articulation     │                      │
│           │              │ Action           │                      │
│           │              │ - joint_positions│                      │
│           │              │ - joint_velocities│                     │
│           │              │ - joint_efforts  │                      │
│           │              └──────────────────┘                      │
│           │                       │                                 │
│           │                       ▼                                 │
│           │              ┌──────────────────┐                      │
│           └─────────────▶│ apply_action()   │                      │
│                          └──────────────────┘                      │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 제어 모드

| 제어 모드 | Stiffness | Damping | 사용 사례 |
|----------|-----------|---------|----------|
| Position Control | 높음 (1000+) | 낮음 (100) | 매니퓰레이터 조인트 |
| Velocity Control | 0 | >0 (50+) | 모바일 베이스 휠 |
| Effort Control | 0 | 0 | 힘/토크 직접 제어 |

---

## 3. robot_controller.py - 메인 로봇 컨트롤러

### 3.1 목적
Isaac Sim 환경에서 로봇 제어를 위한 통합 인터페이스를 제공합니다.

### 3.2 상세 구현

```python
# isaac_interface/robot_controller.py

import numpy as np
from typing import Optional, Tuple, Dict, List
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class RobotConfig:
    """로봇 설정"""
    # 기본 정보
    name: str = "mobile_manipulator"
    prim_path: str = "/World/Robot"

    # 파일 경로
    urdf_path: str = ""
    lula_description_path: str = ""

    # 휠 설정
    wheel_joint_indices: List[int] = None
    wheel_radius: float = 0.1
    wheel_base_width: float = 0.5
    wheel_base_length: float = 0.6
    max_wheel_velocity: float = 10.0

    # 매니퓰레이터 설정
    arm_joint_indices: List[int] = None
    end_effector_frame: str = "tool0"

    # 그리퍼 설정
    gripper_joint_indices: List[int] = None
    gripper_open_position: float = 0.04
    gripper_close_position: float = 0.0
    gripper_grasp_force: float = 10.0

    # 제어 파라미터
    position_stiffness: float = 1000.0
    position_damping: float = 100.0
    velocity_damping: float = 50.0

    def __post_init__(self):
        if self.wheel_joint_indices is None:
            self.wheel_joint_indices = [0, 1, 2, 3]
        if self.arm_joint_indices is None:
            self.arm_joint_indices = [4, 5, 6, 7, 8, 9]
        if self.gripper_joint_indices is None:
            self.gripper_joint_indices = [10, 11]

    @classmethod
    def from_yaml(cls, config: dict) -> 'RobotConfig':
        """YAML 설정에서 생성"""
        robot = config.get("robot", {})
        files = config.get("files", {})
        joints = config.get("joints", {})
        control = config.get("control", {})

        return cls(
            name=robot.get("name", "mobile_manipulator"),
            prim_path=robot.get("prim_path", "/World/Robot"),
            urdf_path=files.get("urdf_path", ""),
            lula_description_path=files.get("lula_description_path", ""),
            wheel_joint_indices=joints.get("wheel", {}).get("indices", [0, 1, 2, 3]),
            wheel_radius=joints.get("wheel", {}).get("radius", 0.1),
            wheel_base_width=joints.get("wheel", {}).get("base_width", 0.5),
            wheel_base_length=joints.get("wheel", {}).get("base_length", 0.6),
            max_wheel_velocity=joints.get("wheel", {}).get("max_velocity", 10.0),
            arm_joint_indices=joints.get("arm", {}).get("indices", [4, 5, 6, 7, 8, 9]),
            end_effector_frame=joints.get("arm", {}).get("end_effector_frame", "tool0"),
            gripper_joint_indices=joints.get("gripper", {}).get("indices", [10, 11]),
            gripper_open_position=joints.get("gripper", {}).get("open_position", 0.04),
            gripper_close_position=joints.get("gripper", {}).get("close_position", 0.0),
            gripper_grasp_force=joints.get("gripper", {}).get("grasp_force", 10.0),
            position_stiffness=control.get("position_stiffness", 1000.0),
            position_damping=control.get("position_damping", 100.0),
            velocity_damping=control.get("velocity_damping", 50.0)
        )


class IsaacRobotController:
    """Isaac Sim 로봇 통합 컨트롤러"""

    def __init__(self, config: RobotConfig):
        self.config = config

        # Isaac Sim 객체들 (initialize()에서 설정)
        self._articulation = None
        self._articulation_controller = None

        # 서브 컨트롤러들
        self._mobile_base: Optional['MobileBaseController'] = None
        self._manipulator: Optional['ManipulatorController'] = None
        self._gripper: Optional['GripperController'] = None

        # 상태
        self._initialized = False
        self._joint_names: List[str] = []
        self._num_dof = 0

    def initialize(self, world=None):
        """Isaac Sim 환경에서 로봇 초기화

        Args:
            world: Isaac Sim World 객체 (None이면 현재 스테이지에서 로드)
        """
        try:
            # Isaac Sim imports (런타임에 임포트)
            from isaacsim.core.articulations import Articulation
            from isaacsim.core.utils.types import ArticulationAction
            from pxr import UsdPhysics

            logger.info(f"Initializing robot at {self.config.prim_path}")

            # Articulation 생성
            self._articulation = Articulation(self.config.prim_path)

            if world:
                world.scene.add(self._articulation)

            # 초기화 대기
            self._articulation.initialize()

            # 조인트 정보 수집
            self._num_dof = self._articulation.num_dof
            self._joint_names = self._articulation.dof_names

            logger.info(f"Robot initialized with {self._num_dof} DOF")
            logger.info(f"Joint names: {self._joint_names}")

            # 제어 모드 설정
            self._configure_control_modes()

            # 서브 컨트롤러 초기화
            self._init_subcontrollers()

            self._initialized = True
            logger.info("Robot controller initialization complete")

        except Exception as e:
            logger.error(f"Failed to initialize robot: {e}")
            raise

    def _configure_control_modes(self):
        """조인트별 제어 모드 설정"""
        # 휠: 속도 제어
        for idx in self.config.wheel_joint_indices:
            if idx < self._num_dof:
                self._articulation.set_gains(
                    kps=np.array([0.0]),
                    kds=np.array([self.config.velocity_damping]),
                    joint_indices=np.array([idx])
                )

        # 매니퓰레이터: 위치 제어
        for idx in self.config.arm_joint_indices:
            if idx < self._num_dof:
                self._articulation.set_gains(
                    kps=np.array([self.config.position_stiffness]),
                    kds=np.array([self.config.position_damping]),
                    joint_indices=np.array([idx])
                )

        # 그리퍼: 위치 제어
        for idx in self.config.gripper_joint_indices:
            if idx < self._num_dof:
                self._articulation.set_gains(
                    kps=np.array([self.config.position_stiffness]),
                    kds=np.array([self.config.position_damping]),
                    joint_indices=np.array([idx])
                )

    def _init_subcontrollers(self):
        """서브 컨트롤러 초기화"""
        # 모바일 베이스 컨트롤러
        self._mobile_base = MobileBaseController(
            articulation=self._articulation,
            wheel_indices=self.config.wheel_joint_indices,
            wheel_radius=self.config.wheel_radius,
            wheel_base_width=self.config.wheel_base_width,
            max_velocity=self.config.max_wheel_velocity
        )

        # 매니퓰레이터 컨트롤러
        self._manipulator = ManipulatorController(
            articulation=self._articulation,
            arm_indices=self.config.arm_joint_indices,
            end_effector_frame=self.config.end_effector_frame,
            lula_description_path=self.config.lula_description_path,
            urdf_path=self.config.urdf_path
        )

        # 그리퍼 컨트롤러
        self._gripper = GripperController(
            articulation=self._articulation,
            gripper_indices=self.config.gripper_joint_indices,
            open_position=self.config.gripper_open_position,
            close_position=self.config.gripper_close_position
        )

    @property
    def mobile_base(self) -> 'MobileBaseController':
        """모바일 베이스 컨트롤러"""
        return self._mobile_base

    @property
    def manipulator(self) -> 'ManipulatorController':
        """매니퓰레이터 컨트롤러"""
        return self._manipulator

    @property
    def gripper(self) -> 'GripperController':
        """그리퍼 컨트롤러"""
        return self._gripper

    def move_to_position(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None
    ) -> bool:
        """엔드이펙터를 목표 위치로 이동

        Args:
            target_position: [x, y, z] 목표 위치 (meters)
            target_orientation: [qw, qx, qy, qz] 목표 방향 (optional)

        Returns:
            bool: IK 성공 여부
        """
        if not self._initialized:
            logger.error("Robot not initialized")
            return False

        return self._manipulator.move_to_pose(target_position, target_orientation)

    def move_base(self, linear_vel: float, angular_vel: float):
        """모바일 베이스 속도 제어

        Args:
            linear_vel: 선속도 (m/s), 양수=전진
            angular_vel: 각속도 (rad/s), 양수=좌회전
        """
        if not self._initialized:
            logger.error("Robot not initialized")
            return

        self._mobile_base.set_velocity(linear_vel, angular_vel)

    def control_gripper(self, action: str):
        """그리퍼 제어

        Args:
            action: "open" 또는 "close"
        """
        if not self._initialized:
            logger.error("Robot not initialized")
            return

        if action == "open":
            self._gripper.open()
        elif action == "close":
            self._gripper.close()

    def get_end_effector_position(self) -> np.ndarray:
        """현재 엔드이펙터 위치 반환"""
        if not self._initialized:
            return np.zeros(3)

        return self._manipulator.get_end_effector_pose()[0]

    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """현재 엔드이펙터 포즈 반환 (위치, 방향)"""
        if not self._initialized:
            return np.zeros(3), np.array([1, 0, 0, 0])

        return self._manipulator.get_end_effector_pose()

    def get_joint_positions(self) -> np.ndarray:
        """현재 모든 조인트 위치 반환"""
        if not self._initialized:
            return np.zeros(self._num_dof)

        return self._articulation.get_joint_positions()

    def get_joint_velocities(self) -> np.ndarray:
        """현재 모든 조인트 속도 반환"""
        if not self._initialized:
            return np.zeros(self._num_dof)

        return self._articulation.get_joint_velocities()

    def set_joint_positions(
        self,
        positions: np.ndarray,
        joint_indices: Optional[np.ndarray] = None
    ):
        """조인트 위치 직접 설정"""
        if not self._initialized:
            return

        from isaacsim.core.utils.types import ArticulationAction

        action = ArticulationAction(
            joint_positions=positions,
            joint_indices=joint_indices
        )
        self._articulation.apply_action(action)

    def set_joint_velocities(
        self,
        velocities: np.ndarray,
        joint_indices: Optional[np.ndarray] = None
    ):
        """조인트 속도 직접 설정"""
        if not self._initialized:
            return

        from isaacsim.core.utils.types import ArticulationAction

        action = ArticulationAction(
            joint_velocities=velocities,
            joint_indices=joint_indices
        )
        self._articulation.apply_action(action)

    def emergency_stop(self):
        """비상 정지 - 모든 조인트 정지"""
        if not self._initialized:
            return

        logger.warning("Emergency stop activated!")

        # 모든 조인트 속도 0으로 설정
        zero_velocities = np.zeros(self._num_dof)
        self.set_joint_velocities(zero_velocities)

        # 모든 조인트 힘 0으로 설정
        from isaacsim.core.utils.types import ArticulationAction
        action = ArticulationAction(
            joint_efforts=np.zeros(self._num_dof)
        )
        self._articulation.apply_action(action)

    def reset_to_default(self):
        """기본 포즈로 리셋"""
        if not self._initialized:
            return

        # 기본 조인트 위치 (0으로 리셋)
        default_positions = np.zeros(self._num_dof)
        self._articulation.set_joint_positions(default_positions)
        self._articulation.set_joint_velocities(np.zeros(self._num_dof))

    def get_robot_state(self) -> Dict:
        """로봇 전체 상태 반환"""
        return {
            "initialized": self._initialized,
            "num_dof": self._num_dof,
            "joint_positions": self.get_joint_positions().tolist(),
            "joint_velocities": self.get_joint_velocities().tolist(),
            "end_effector_position": self.get_end_effector_position().tolist(),
            "gripper_state": self._gripper.get_state().value if self._gripper else "unknown",
            "base_odometry": self._mobile_base.get_odometry() if self._mobile_base else None
        }
```

---

## 4. mobile_base.py - 모바일 베이스 제어

### 4.1 목적
4륜 구동 모바일 베이스의 속도 제어와 오도메트리 계산을 담당합니다.

### 4.2 상세 구현

```python
# isaac_interface/mobile_base.py

import numpy as np
from typing import Optional, List, Dict
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class DriveType(Enum):
    """구동 방식"""
    DIFFERENTIAL = "differential"    # 차동 구동 (2륜 또는 4륜)
    MECANUM = "mecanum"              # 메카넘 휠 (전방향)
    OMNI = "omni"                    # 옴니휠


@dataclass
class Odometry:
    """오도메트리 데이터"""
    x: float = 0.0           # 위치 X (m)
    y: float = 0.0           # 위치 Y (m)
    theta: float = 0.0       # 방향 (rad)
    linear_velocity: float = 0.0    # 선속도 (m/s)
    angular_velocity: float = 0.0   # 각속도 (rad/s)
    timestamp: float = 0.0

    def to_dict(self) -> Dict:
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "timestamp": self.timestamp
        }


class MobileBaseController:
    """모바일 베이스 컨트롤러"""

    def __init__(
        self,
        articulation,
        wheel_indices: List[int],
        wheel_radius: float = 0.1,
        wheel_base_width: float = 0.5,
        wheel_base_length: float = 0.6,
        max_velocity: float = 10.0,
        drive_type: DriveType = DriveType.DIFFERENTIAL
    ):
        """
        Args:
            articulation: Isaac Sim Articulation 객체
            wheel_indices: 휠 조인트 인덱스 [FL, FR, RL, RR]
            wheel_radius: 휠 반지름 (m)
            wheel_base_width: 좌우 휠 간 거리 (m)
            wheel_base_length: 전후 휠 간 거리 (m)
            max_velocity: 최대 휠 속도 (rad/s)
            drive_type: 구동 방식
        """
        self._articulation = articulation
        self._wheel_indices = np.array(wheel_indices)
        self._wheel_radius = wheel_radius
        self._wheel_base_width = wheel_base_width
        self._wheel_base_length = wheel_base_length
        self._max_velocity = max_velocity
        self._drive_type = drive_type

        # 오도메트리
        self._odometry = Odometry()
        self._last_wheel_positions: Optional[np.ndarray] = None
        self._last_time: float = 0.0

        # 현재 명령 속도
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0

        logger.info(f"Mobile base initialized: {drive_type.value}, "
                   f"wheel_radius={wheel_radius}, width={wheel_base_width}")

    def set_velocity(self, linear: float, angular: float):
        """기본 속도 명령 설정

        Args:
            linear: 선속도 (m/s), 양수=전진, 음수=후진
            angular: 각속도 (rad/s), 양수=좌회전, 음수=우회전
        """
        self._cmd_linear = linear
        self._cmd_angular = angular

        # 휠 속도 계산
        wheel_velocities = self._compute_wheel_velocities(linear, angular)

        # 속도 제한
        wheel_velocities = np.clip(wheel_velocities, -self._max_velocity, self._max_velocity)

        # 적용
        self._apply_wheel_velocities(wheel_velocities)

        logger.debug(f"Base velocity command: linear={linear:.2f}, angular={angular:.2f}")

    def set_velocity_omni(self, vx: float, vy: float, omega: float):
        """전방향 속도 명령 (메카넘/옴니휠 전용)

        Args:
            vx: X 방향 속도 (m/s)
            vy: Y 방향 속도 (m/s)
            omega: 회전 속도 (rad/s)
        """
        if self._drive_type not in [DriveType.MECANUM, DriveType.OMNI]:
            logger.warning("Omni velocity only supported for mecanum/omni drive")
            return

        wheel_velocities = self._compute_mecanum_velocities(vx, vy, omega)
        wheel_velocities = np.clip(wheel_velocities, -self._max_velocity, self._max_velocity)
        self._apply_wheel_velocities(wheel_velocities)

    def stop(self):
        """정지"""
        self.set_velocity(0.0, 0.0)

    def _compute_wheel_velocities(self, linear: float, angular: float) -> np.ndarray:
        """차동 구동 휠 속도 계산

        차동 구동 운동학:
        v_left = (v - omega * L/2) / r
        v_right = (v + omega * L/2) / r

        Args:
            linear: 선속도 (m/s)
            angular: 각속도 (rad/s)

        Returns:
            [v_fl, v_fr, v_rl, v_rr] 휠 속도 (rad/s)
        """
        r = self._wheel_radius
        L = self._wheel_base_width

        v_left = (linear - angular * L / 2) / r
        v_right = (linear + angular * L / 2) / r

        # 4륜: 좌측 휠 동일, 우측 휠 동일
        return np.array([v_left, v_right, v_left, v_right])

    def _compute_mecanum_velocities(
        self,
        vx: float,
        vy: float,
        omega: float
    ) -> np.ndarray:
        """메카넘 휠 속도 계산

        메카넘 휠 운동학 (45도 롤러):
        v_fl = (vx - vy - (L+W)/2 * omega) / r
        v_fr = (vx + vy + (L+W)/2 * omega) / r
        v_rl = (vx + vy - (L+W)/2 * omega) / r
        v_rr = (vx - vy + (L+W)/2 * omega) / r

        Args:
            vx: X 방향 속도 (전진)
            vy: Y 방향 속도 (측면)
            omega: 회전 속도

        Returns:
            [v_fl, v_fr, v_rl, v_rr] 휠 속도 (rad/s)
        """
        r = self._wheel_radius
        L = self._wheel_base_length
        W = self._wheel_base_width

        k = (L + W) / 2

        v_fl = (vx - vy - k * omega) / r
        v_fr = (vx + vy + k * omega) / r
        v_rl = (vx + vy - k * omega) / r
        v_rr = (vx - vy + k * omega) / r

        return np.array([v_fl, v_fr, v_rl, v_rr])

    def _apply_wheel_velocities(self, velocities: np.ndarray):
        """휠 속도 적용"""
        from isaacsim.core.utils.types import ArticulationAction

        action = ArticulationAction(
            joint_velocities=velocities,
            joint_indices=self._wheel_indices
        )
        self._articulation.apply_action(action)

    def update_odometry(self, dt: float):
        """오도메트리 업데이트 (매 시뮬레이션 스텝마다 호출)

        Args:
            dt: 시간 간격 (초)
        """
        if dt <= 0:
            return

        # 현재 휠 위치 읽기
        all_positions = self._articulation.get_joint_positions()
        wheel_positions = all_positions[self._wheel_indices]

        if self._last_wheel_positions is not None:
            # 휠 변위 계산
            delta_positions = wheel_positions - self._last_wheel_positions

            # 평균 좌/우 변위
            delta_left = (delta_positions[0] + delta_positions[2]) / 2
            delta_right = (delta_positions[1] + delta_positions[3]) / 2

            # 선/각 변위 계산
            delta_linear = self._wheel_radius * (delta_left + delta_right) / 2
            delta_angular = self._wheel_radius * (delta_right - delta_left) / self._wheel_base_width

            # 오도메트리 적분
            self._odometry.theta += delta_angular
            self._odometry.x += delta_linear * np.cos(self._odometry.theta)
            self._odometry.y += delta_linear * np.sin(self._odometry.theta)

            # 속도 계산
            self._odometry.linear_velocity = delta_linear / dt
            self._odometry.angular_velocity = delta_angular / dt

        self._last_wheel_positions = wheel_positions.copy()
        self._odometry.timestamp += dt

    def get_odometry(self) -> Odometry:
        """현재 오도메트리 반환"""
        return self._odometry

    def reset_odometry(self):
        """오도메트리 초기화"""
        self._odometry = Odometry()
        self._last_wheel_positions = None

    def get_wheel_velocities(self) -> np.ndarray:
        """현재 휠 속도 반환"""
        all_velocities = self._articulation.get_joint_velocities()
        return all_velocities[self._wheel_indices]


class DifferentialDriveKinematics:
    """차동 구동 운동학 유틸리티"""

    @staticmethod
    def forward_kinematics(
        v_left: float,
        v_right: float,
        wheel_radius: float,
        wheel_base: float
    ) -> tuple[float, float]:
        """순운동학: 휠 속도 → 로봇 속도

        Args:
            v_left: 좌측 휠 속도 (rad/s)
            v_right: 우측 휠 속도 (rad/s)
            wheel_radius: 휠 반지름 (m)
            wheel_base: 휠 간 거리 (m)

        Returns:
            (linear_velocity, angular_velocity)
        """
        linear = wheel_radius * (v_left + v_right) / 2
        angular = wheel_radius * (v_right - v_left) / wheel_base
        return linear, angular

    @staticmethod
    def inverse_kinematics(
        linear: float,
        angular: float,
        wheel_radius: float,
        wheel_base: float
    ) -> tuple[float, float]:
        """역운동학: 로봇 속도 → 휠 속도

        Args:
            linear: 선속도 (m/s)
            angular: 각속도 (rad/s)
            wheel_radius: 휠 반지름 (m)
            wheel_base: 휠 간 거리 (m)

        Returns:
            (v_left, v_right) 휠 속도 (rad/s)
        """
        v_left = (linear - angular * wheel_base / 2) / wheel_radius
        v_right = (linear + angular * wheel_base / 2) / wheel_radius
        return v_left, v_right
```

---

## 5. manipulator.py - 매니퓰레이터 제어

### 5.1 목적
IK 솔버를 사용하여 매니퓰레이터 엔드이펙터의 포즈 기반 제어를 제공합니다.

### 5.2 상세 구현

```python
# isaac_interface/manipulator.py

import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class JointLimits:
    """조인트 제한"""
    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray


class ManipulatorController:
    """매니퓰레이터 IK 기반 컨트롤러"""

    def __init__(
        self,
        articulation,
        arm_indices: List[int],
        end_effector_frame: str = "tool0",
        lula_description_path: str = "",
        urdf_path: str = ""
    ):
        """
        Args:
            articulation: Isaac Sim Articulation 객체
            arm_indices: 매니퓰레이터 조인트 인덱스
            end_effector_frame: 엔드이펙터 프레임 이름
            lula_description_path: Lula robot description YAML 경로
            urdf_path: URDF 파일 경로
        """
        self._articulation = articulation
        self._arm_indices = np.array(arm_indices)
        self._end_effector_frame = end_effector_frame
        self._lula_description_path = lula_description_path
        self._urdf_path = urdf_path

        # IK 솔버 (초기화 시 설정)
        self._ik_solver = None
        self._kinematics_solver = None

        # 조인트 제한
        self._joint_limits: Optional[JointLimits] = None

        # 초기화
        self._init_ik_solver()

    def _init_ik_solver(self):
        """IK 솔버 초기화"""
        try:
            from isaacsim.robot_motion.motion_generation import (
                ArticulationKinematicsSolver,
                LulaKinematicsSolver
            )

            if self._lula_description_path and self._urdf_path:
                # Lula IK 솔버 초기화
                self._lula_solver = LulaKinematicsSolver(
                    robot_description_path=self._lula_description_path,
                    urdf_path=self._urdf_path
                )

                self._kinematics_solver = ArticulationKinematicsSolver(
                    self._articulation,
                    self._lula_solver,
                    end_effector_frame_name=self._end_effector_frame
                )

                logger.info("Lula IK solver initialized successfully")
            else:
                logger.warning("Lula description or URDF path not provided, "
                             "IK solver not initialized")

        except ImportError as e:
            logger.error(f"Failed to import IK solver modules: {e}")
        except Exception as e:
            logger.error(f"Failed to initialize IK solver: {e}")

    def move_to_pose(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None
    ) -> bool:
        """엔드이펙터를 목표 포즈로 이동

        Args:
            target_position: [x, y, z] 목표 위치 (meters)
            target_orientation: [qw, qx, qy, qz] 목표 방향 (quaternion)

        Returns:
            bool: IK 성공 여부
        """
        if self._kinematics_solver is None:
            logger.error("IK solver not initialized")
            return False

        try:
            # IK 계산
            action, success = self._kinematics_solver.compute_inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation
            )

            if success:
                # 조인트 액션 적용
                self._articulation.apply_action(action)
                logger.debug(f"IK solution found, applying action")
                return True
            else:
                logger.warning(f"IK solution not found for position {target_position}")
                return False

        except Exception as e:
            logger.error(f"IK computation error: {e}")
            return False

    def move_joints(self, positions: np.ndarray):
        """조인트 위치 직접 제어

        Args:
            positions: 매니퓰레이터 조인트 위치 배열
        """
        from isaacsim.core.utils.types import ArticulationAction

        action = ArticulationAction(
            joint_positions=positions,
            joint_indices=self._arm_indices
        )
        self._articulation.apply_action(action)

    def move_joints_velocity(self, velocities: np.ndarray):
        """조인트 속도 직접 제어

        Args:
            velocities: 매니퓰레이터 조인트 속도 배열
        """
        from isaacsim.core.utils.types import ArticulationAction

        action = ArticulationAction(
            joint_velocities=velocities,
            joint_indices=self._arm_indices
        )
        self._articulation.apply_action(action)

    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """현재 엔드이펙터 포즈 반환

        Returns:
            (position, orientation): 위치 [x,y,z], 방향 [qw,qx,qy,qz]
        """
        if self._kinematics_solver is None:
            logger.warning("IK solver not initialized, returning zeros")
            return np.zeros(3), np.array([1, 0, 0, 0])

        try:
            # Forward Kinematics 계산
            position, orientation = self._kinematics_solver.compute_end_effector_pose()
            return position, orientation

        except Exception as e:
            logger.error(f"FK computation error: {e}")
            return np.zeros(3), np.array([1, 0, 0, 0])

    def get_arm_joint_positions(self) -> np.ndarray:
        """매니퓰레이터 조인트 위치 반환"""
        all_positions = self._articulation.get_joint_positions()
        return all_positions[self._arm_indices]

    def get_arm_joint_velocities(self) -> np.ndarray:
        """매니퓰레이터 조인트 속도 반환"""
        all_velocities = self._articulation.get_joint_velocities()
        return all_velocities[self._arm_indices]

    def compute_ik(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None
    ) -> Optional[np.ndarray]:
        """IK만 계산 (적용하지 않음)

        Args:
            target_position: 목표 위치
            target_orientation: 목표 방향

        Returns:
            조인트 위치 배열 또는 None (실패 시)
        """
        if self._kinematics_solver is None:
            return None

        try:
            action, success = self._kinematics_solver.compute_inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation
            )

            if success:
                return action.joint_positions
            return None

        except Exception as e:
            logger.error(f"IK computation error: {e}")
            return None

    def compute_fk(
        self,
        joint_positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Forward Kinematics 계산

        Args:
            joint_positions: 조인트 위치 배열

        Returns:
            (position, orientation)
        """
        if self._lula_solver is None:
            return np.zeros(3), np.array([1, 0, 0, 0])

        try:
            # 임시로 조인트 위치 설정 후 FK 계산
            # (실제 구현은 Lula API에 따라 다를 수 있음)
            position, orientation = self._lula_solver.compute_forward_kinematics(
                joint_positions,
                self._end_effector_frame
            )
            return position, orientation

        except Exception as e:
            logger.error(f"FK computation error: {e}")
            return np.zeros(3), np.array([1, 0, 0, 0])

    def is_position_reachable(self, position: np.ndarray) -> bool:
        """위치가 도달 가능한지 확인

        Args:
            position: 확인할 위치 [x, y, z]

        Returns:
            bool: 도달 가능 여부
        """
        return self.compute_ik(position) is not None


class TrajectoryGenerator:
    """궤적 생성 유틸리티"""

    @staticmethod
    def linear_interpolation(
        start: np.ndarray,
        end: np.ndarray,
        num_points: int = 10
    ) -> np.ndarray:
        """선형 보간 궤적 생성

        Args:
            start: 시작 위치
            end: 끝 위치
            num_points: 경유점 개수

        Returns:
            (num_points, 3) 크기의 궤적 배열
        """
        t = np.linspace(0, 1, num_points)
        trajectory = np.outer(1 - t, start) + np.outer(t, end)
        return trajectory

    @staticmethod
    def trapezoidal_velocity_profile(
        start: np.ndarray,
        end: np.ndarray,
        max_velocity: float,
        acceleration: float,
        dt: float = 0.01
    ) -> Tuple[np.ndarray, np.ndarray]:
        """사다리꼴 속도 프로파일 궤적 생성

        Args:
            start: 시작 위치
            end: 끝 위치
            max_velocity: 최대 속도
            acceleration: 가속도
            dt: 시간 간격

        Returns:
            (positions, velocities): 위치/속도 궤적
        """
        distance = np.linalg.norm(end - start)
        direction = (end - start) / distance if distance > 0 else np.zeros_like(start)

        # 가속/감속 시간
        t_accel = max_velocity / acceleration

        # 가속 거리
        d_accel = 0.5 * acceleration * t_accel**2

        if 2 * d_accel > distance:
            # 최대 속도에 도달하지 못하는 경우
            t_accel = np.sqrt(distance / acceleration)
            t_cruise = 0
            t_total = 2 * t_accel
            actual_max_vel = acceleration * t_accel
        else:
            # 정상 사다리꼴 프로파일
            d_cruise = distance - 2 * d_accel
            t_cruise = d_cruise / max_velocity
            t_total = 2 * t_accel + t_cruise
            actual_max_vel = max_velocity

        # 시간 배열
        t = np.arange(0, t_total + dt, dt)
        positions = []
        velocities = []

        for ti in t:
            if ti < t_accel:
                # 가속 구간
                v = acceleration * ti
                d = 0.5 * acceleration * ti**2
            elif ti < t_accel + t_cruise:
                # 등속 구간
                v = actual_max_vel
                d = d_accel + actual_max_vel * (ti - t_accel)
            else:
                # 감속 구간
                t_decel = ti - t_accel - t_cruise
                v = actual_max_vel - acceleration * t_decel
                d = distance - 0.5 * acceleration * (t_total - ti)**2

            positions.append(start + direction * d)
            velocities.append(direction * v)

        return np.array(positions), np.array(velocities)
```

---

## 6. gripper.py - 그리퍼 제어

### 6.1 목적
2-finger 그리퍼의 열기/닫기 제어를 담당합니다.

### 6.2 상세 구현

```python
# isaac_interface/gripper.py

import numpy as np
from typing import List, Optional
from enum import Enum
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


class GripperState(Enum):
    """그리퍼 상태"""
    OPEN = "open"
    CLOSED = "closed"
    MOVING = "moving"
    GRASPING = "grasping"
    UNKNOWN = "unknown"


@dataclass
class GripperStatus:
    """그리퍼 상태 정보"""
    state: GripperState
    position: float  # 현재 위치 (0=닫힘, max=열림)
    force: float     # 현재 힘
    is_object_detected: bool


class GripperController:
    """그리퍼 컨트롤러"""

    def __init__(
        self,
        articulation,
        gripper_indices: List[int],
        open_position: float = 0.04,
        close_position: float = 0.0,
        grasp_force: float = 10.0,
        move_speed: float = 0.1
    ):
        """
        Args:
            articulation: Isaac Sim Articulation 객체
            gripper_indices: 그리퍼 조인트 인덱스 (예: [10, 11] for 2-finger)
            open_position: 열린 상태 위치 (m)
            close_position: 닫힌 상태 위치 (m)
            grasp_force: 파지 힘 (N)
            move_speed: 이동 속도 (m/s)
        """
        self._articulation = articulation
        self._gripper_indices = np.array(gripper_indices)
        self._open_position = open_position
        self._close_position = close_position
        self._grasp_force = grasp_force
        self._move_speed = move_speed

        # 상태
        self._target_position = open_position
        self._state = GripperState.UNKNOWN
        self._is_grasping = False

        # 물체 감지 임계값
        self._force_threshold = grasp_force * 0.5
        self._position_tolerance = 0.001  # 1mm

        logger.info(f"Gripper initialized: indices={gripper_indices}, "
                   f"open={open_position}, close={close_position}")

    def open(self):
        """그리퍼 열기"""
        self._target_position = self._open_position
        self._is_grasping = False
        self._set_position(self._open_position)
        self._state = GripperState.MOVING
        logger.debug("Gripper opening")

    def close(self):
        """그리퍼 닫기"""
        self._target_position = self._close_position
        self._set_position(self._close_position)
        self._state = GripperState.MOVING
        logger.debug("Gripper closing")

    def grasp(self, force: Optional[float] = None):
        """파지 동작 (힘 제어 포함)

        Args:
            force: 파지 힘 (None이면 기본값 사용)
        """
        grasp_force = force if force is not None else self._grasp_force

        # 힘 제어 모드로 전환하여 닫기
        self._target_position = self._close_position
        self._is_grasping = True
        self._state = GripperState.MOVING

        # 위치 명령 + 힘 제한
        self._set_position_with_force_limit(self._close_position, grasp_force)
        logger.debug(f"Gripper grasping with force {grasp_force}N")

    def set_position(self, position: float):
        """그리퍼 위치 직접 설정

        Args:
            position: 목표 위치 (0 ~ open_position)
        """
        position = np.clip(position, self._close_position, self._open_position)
        self._target_position = position
        self._set_position(position)
        self._state = GripperState.MOVING

    def _set_position(self, position: float):
        """내부: 조인트 위치 설정"""
        from isaacsim.core.utils.types import ArticulationAction

        # 모든 그리퍼 조인트에 동일한 위치 설정
        # (대칭 그리퍼의 경우 반대 부호 필요할 수 있음)
        num_fingers = len(self._gripper_indices)

        if num_fingers == 2:
            # 2-finger 대칭 그리퍼
            positions = np.array([position, position])
        else:
            positions = np.full(num_fingers, position)

        action = ArticulationAction(
            joint_positions=positions,
            joint_indices=self._gripper_indices
        )
        self._articulation.apply_action(action)

    def _set_position_with_force_limit(self, position: float, max_force: float):
        """힘 제한이 있는 위치 제어"""
        from isaacsim.core.utils.types import ArticulationAction

        num_fingers = len(self._gripper_indices)
        positions = np.full(num_fingers, position)
        efforts = np.full(num_fingers, max_force)

        # 위치 명령과 힘 제한 동시 적용
        action = ArticulationAction(
            joint_positions=positions,
            joint_indices=self._gripper_indices
        )
        self._articulation.apply_action(action)

        # 힘 제한 설정 (별도 API 필요)
        # self._articulation.set_max_efforts(efforts, self._gripper_indices)

    def get_state(self) -> GripperState:
        """현재 그리퍼 상태 반환"""
        self._update_state()
        return self._state

    def get_status(self) -> GripperStatus:
        """상세 상태 정보 반환"""
        self._update_state()

        current_pos = self.get_position()
        current_force = self.get_force()

        return GripperStatus(
            state=self._state,
            position=current_pos,
            force=current_force,
            is_object_detected=self._is_object_detected()
        )

    def get_position(self) -> float:
        """현재 그리퍼 위치 반환"""
        all_positions = self._articulation.get_joint_positions()
        gripper_positions = all_positions[self._gripper_indices]
        return np.mean(np.abs(gripper_positions))

    def get_force(self) -> float:
        """현재 그리퍼 힘 반환"""
        all_efforts = self._articulation.get_joint_efforts()
        gripper_efforts = all_efforts[self._gripper_indices]
        return np.mean(np.abs(gripper_efforts))

    def _update_state(self):
        """상태 업데이트"""
        current_pos = self.get_position()
        current_force = self.get_force()

        # 이동 완료 확인
        if abs(current_pos - self._target_position) < self._position_tolerance:
            if self._is_grasping and self._is_object_detected():
                self._state = GripperState.GRASPING
            elif current_pos >= self._open_position - self._position_tolerance:
                self._state = GripperState.OPEN
            elif current_pos <= self._close_position + self._position_tolerance:
                self._state = GripperState.CLOSED
        else:
            self._state = GripperState.MOVING

    def _is_object_detected(self) -> bool:
        """물체 감지 여부 (힘 기반)"""
        current_force = self.get_force()
        current_pos = self.get_position()

        # 닫히는 중인데 힘이 임계값 이상이면 물체 감지
        return (
            self._target_position == self._close_position and
            current_force > self._force_threshold and
            current_pos > self._close_position + self._position_tolerance * 2
        )

    def is_grasping(self) -> bool:
        """파지 중인지 확인"""
        return self._state == GripperState.GRASPING

    def get_aperture(self) -> float:
        """그리퍼 개구부 크기 반환 (열린 정도, 0~1)"""
        current_pos = self.get_position()
        range_val = self._open_position - self._close_position
        if range_val == 0:
            return 0.0
        return (current_pos - self._close_position) / range_val


class ParallelGripperController(GripperController):
    """평행 그리퍼 컨트롤러 (미러 조인트)"""

    def _set_position(self, position: float):
        """미러 조인트용 위치 설정"""
        from isaacsim.core.utils.types import ArticulationAction

        # 평행 그리퍼: 양쪽이 반대 방향으로 움직임
        positions = np.array([position, -position])

        action = ArticulationAction(
            joint_positions=positions,
            joint_indices=self._gripper_indices
        )
        self._articulation.apply_action(action)
```

---

## 7. ik_solver.py - IK 솔버 래퍼

### 7.1 목적
여러 IK 솔버를 통합하여 일관된 인터페이스를 제공합니다.

### 7.2 상세 구현

```python
# isaac_interface/ik_solver.py

import numpy as np
from typing import Optional, Tuple, Protocol
from abc import ABC, abstractmethod
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class IKSolverType(Enum):
    """IK 솔버 타입"""
    LULA = "lula"
    CUROBO = "curobo"
    IKFAST = "ikfast"


class IKSolverInterface(ABC):
    """IK 솔버 인터페이스"""

    @abstractmethod
    def solve(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        seed_configuration: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], bool]:
        """IK 계산

        Args:
            target_position: 목표 위치 [x, y, z]
            target_orientation: 목표 방향 [qw, qx, qy, qz]
            seed_configuration: 초기 조인트 설정

        Returns:
            (joint_positions, success): 조인트 위치와 성공 여부
        """
        pass

    @abstractmethod
    def forward_kinematics(
        self,
        joint_positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """FK 계산

        Args:
            joint_positions: 조인트 위치

        Returns:
            (position, orientation): 엔드이펙터 포즈
        """
        pass


class LulaIKSolver(IKSolverInterface):
    """Lula IK 솔버 래퍼"""

    def __init__(
        self,
        robot_description_path: str,
        urdf_path: str,
        end_effector_frame: str
    ):
        self._end_effector_frame = end_effector_frame
        self._solver = None

        try:
            from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

            self._solver = LulaKinematicsSolver(
                robot_description_path=robot_description_path,
                urdf_path=urdf_path
            )
            logger.info("Lula IK solver initialized")

        except ImportError as e:
            logger.error(f"Failed to import Lula: {e}")
        except Exception as e:
            logger.error(f"Failed to initialize Lula: {e}")

    def solve(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        seed_configuration: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], bool]:
        """Lula IK 계산"""
        if self._solver is None:
            return None, False

        try:
            joint_positions = self._solver.compute_inverse_kinematics(
                frame_name=self._end_effector_frame,
                warm_start=seed_configuration,
                position=target_position,
                orientation=target_orientation
            )

            if joint_positions is not None:
                return joint_positions, True
            return None, False

        except Exception as e:
            logger.error(f"Lula IK error: {e}")
            return None, False

    def forward_kinematics(
        self,
        joint_positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Lula FK 계산"""
        if self._solver is None:
            return np.zeros(3), np.array([1, 0, 0, 0])

        try:
            position, orientation = self._solver.compute_forward_kinematics(
                frame_name=self._end_effector_frame,
                joint_positions=joint_positions
            )
            return position, orientation

        except Exception as e:
            logger.error(f"Lula FK error: {e}")
            return np.zeros(3), np.array([1, 0, 0, 0])


class CuRoboIKSolver(IKSolverInterface):
    """cuRobo IK 솔버 래퍼 (GPU 가속)"""

    def __init__(
        self,
        robot_config_path: str,
        world_config_path: Optional[str] = None,
        tensor_args: Optional[dict] = None
    ):
        self._solver = None
        self._motion_gen = None

        try:
            # cuRobo imports
            from curobo.types.robot import RobotConfig
            from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
            from curobo.types.math import Pose

            # 설정 로드
            robot_config = RobotConfig.from_yaml(robot_config_path)

            ik_config = IKSolverConfig.load_from_robot_config(
                robot_config,
                world_coll_checker=None if world_config_path is None else ...,
                tensor_args=tensor_args
            )

            self._solver = IKSolver(ik_config)
            self._Pose = Pose

            logger.info("cuRobo IK solver initialized")

        except ImportError as e:
            logger.error(f"Failed to import cuRobo: {e}")
        except Exception as e:
            logger.error(f"Failed to initialize cuRobo: {e}")

    def solve(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        seed_configuration: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], bool]:
        """cuRobo IK 계산"""
        if self._solver is None:
            return None, False

        try:
            # Pose 생성
            if target_orientation is None:
                target_orientation = np.array([1, 0, 0, 0])

            pose = self._Pose(
                position=target_position,
                quaternion=target_orientation
            )

            # IK 계산
            result = self._solver.solve_single(pose)

            if result.success.item():
                return result.solution.cpu().numpy().flatten(), True
            return None, False

        except Exception as e:
            logger.error(f"cuRobo IK error: {e}")
            return None, False

    def solve_batch(
        self,
        target_positions: np.ndarray,
        target_orientations: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """배치 IK 계산 (GPU 병렬 처리)

        Args:
            target_positions: (N, 3) 목표 위치들
            target_orientations: (N, 4) 목표 방향들

        Returns:
            (joint_positions, success_flags): (N, DOF), (N,)
        """
        if self._solver is None:
            n = len(target_positions)
            return np.zeros((n, 6)), np.zeros(n, dtype=bool)

        try:
            if target_orientations is None:
                target_orientations = np.tile([1, 0, 0, 0], (len(target_positions), 1))

            poses = self._Pose(
                position=target_positions,
                quaternion=target_orientations
            )

            results = self._solver.solve_batch(poses)

            return (
                results.solution.cpu().numpy(),
                results.success.cpu().numpy()
            )

        except Exception as e:
            logger.error(f"cuRobo batch IK error: {e}")
            n = len(target_positions)
            return np.zeros((n, 6)), np.zeros(n, dtype=bool)

    def forward_kinematics(
        self,
        joint_positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """cuRobo FK 계산"""
        if self._solver is None:
            return np.zeros(3), np.array([1, 0, 0, 0])

        try:
            # cuRobo FK 구현
            state = self._solver.compute_kinematics(joint_positions)
            return state.ee_position.cpu().numpy(), state.ee_quaternion.cpu().numpy()

        except Exception as e:
            logger.error(f"cuRobo FK error: {e}")
            return np.zeros(3), np.array([1, 0, 0, 0])


class IKSolverFactory:
    """IK 솔버 팩토리"""

    @staticmethod
    def create(
        solver_type: IKSolverType,
        **kwargs
    ) -> IKSolverInterface:
        """IK 솔버 생성

        Args:
            solver_type: 솔버 타입
            **kwargs: 솔버별 설정

        Returns:
            IKSolverInterface 구현체
        """
        if solver_type == IKSolverType.LULA:
            return LulaIKSolver(
                robot_description_path=kwargs.get("robot_description_path", ""),
                urdf_path=kwargs.get("urdf_path", ""),
                end_effector_frame=kwargs.get("end_effector_frame", "tool0")
            )
        elif solver_type == IKSolverType.CUROBO:
            return CuRoboIKSolver(
                robot_config_path=kwargs.get("robot_config_path", ""),
                world_config_path=kwargs.get("world_config_path"),
                tensor_args=kwargs.get("tensor_args")
            )
        else:
            raise ValueError(f"Unknown solver type: {solver_type}")


class IKSolverWithFallback:
    """폴백을 지원하는 IK 솔버"""

    def __init__(
        self,
        primary: IKSolverInterface,
        fallback: Optional[IKSolverInterface] = None
    ):
        self._primary = primary
        self._fallback = fallback

    def solve(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        seed_configuration: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], bool]:
        """IK 계산 (실패시 폴백)"""
        # 주 솔버 시도
        result, success = self._primary.solve(
            target_position, target_orientation, seed_configuration
        )

        if success:
            return result, True

        # 폴백 솔버 시도
        if self._fallback is not None:
            logger.info("Primary IK failed, trying fallback solver")
            return self._fallback.solve(
                target_position, target_orientation, seed_configuration
            )

        return None, False
```

---

## 8. 스크립트: Standalone 실행

### 8.1 run_standalone.py

```python
# scripts/run_standalone.py
"""Isaac Sim Standalone 실행 스크립트"""

import sys
import os

# Isaac Sim 경로 추가 (환경에 맞게 수정)
ISAAC_SIM_PATH = os.environ.get("ISAAC_SIM_PATH", "/isaac-sim")
sys.path.append(f"{ISAAC_SIM_PATH}/exts")

from isaacsim import SimulationApp

# SimulationApp 생성 (가장 먼저 해야 함)
CONFIG = {
    "headless": False,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracedLighting"
}
simulation_app = SimulationApp(CONFIG)

# Isaac Sim 모듈 임포트 (SimulationApp 생성 후)
import carb
from isaacsim.core import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
import numpy as np

# 프로젝트 모듈 임포트
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from isaac_interface.robot_controller import IsaacRobotController, RobotConfig
from core.control_manager import LLMRobotControlManager


def setup_scene(world: World, robot_usd_path: str, robot_prim_path: str):
    """씬 설정"""
    # 바닥 추가
    world.scene.add_default_ground_plane()

    # 로봇 USD 로드
    add_reference_to_stage(
        usd_path=robot_usd_path,
        prim_path=robot_prim_path
    )

    print(f"Robot loaded at {robot_prim_path}")


async def run_simulation(config: dict):
    """메인 시뮬레이션 루프"""
    # World 생성
    world = World(stage_units_in_meters=1.0)
    await world.initialize_simulation_context_async()

    # 씬 설정
    robot_usd = config.get("robot_usd_path", "")
    robot_prim = config.get("robot_prim_path", "/World/Robot")
    setup_scene(world, robot_usd, robot_prim)

    # 로봇 컨트롤러 초기화
    robot_config = RobotConfig.from_yaml(config)
    robot_controller = IsaacRobotController(robot_config)
    robot_controller.initialize(world)

    # LLM 제어 매니저 초기화
    control_manager = LLMRobotControlManager(config)
    control_manager.set_robot_controller(robot_controller)

    # 웹 서버 시작 (별도 스레드)
    from web.server import start_server
    import threading
    server_thread = threading.Thread(
        target=start_server,
        args=(control_manager, config.get("server", {})),
        daemon=True
    )
    server_thread.start()

    print("Simulation ready. Web UI available at http://localhost:8000")

    # 시뮬레이션 루프
    world.reset()
    while simulation_app.is_running():
        await world.step_async()

        # 모바일 베이스 오도메트리 업데이트
        dt = world.get_physics_dt()
        robot_controller.mobile_base.update_odometry(dt)


def main():
    import asyncio
    import yaml

    # 설정 로드
    config_path = os.path.join(
        os.path.dirname(__file__),
        "..", "config", "robot_config.yaml"
    )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # 시뮬레이션 실행
    asyncio.ensure_future(run_simulation(config))

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
```

---

## 9. 설정 파일 템플릿

### 9.1 robot_config.yaml

```yaml
# config/robot_config.yaml

# 로봇 기본 정보
robot:
  name: "mobile_manipulator"
  prim_path: "/World/Robot"

# 파일 경로
files:
  urdf_path: "assets/robot/robot.urdf"
  usd_path: "assets/robot/robot.usd"
  lula_description_path: "assets/robot/robot_description.yaml"

# 조인트 설정
joints:
  # 모바일 베이스 (4륜)
  wheel:
    indices: [0, 1, 2, 3]        # FL, FR, RL, RR
    names: ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
    radius: 0.1                  # meters
    base_width: 0.5              # meters (좌우 거리)
    base_length: 0.6             # meters (전후 거리)
    max_velocity: 10.0           # rad/s

  # 매니퓰레이터 (6 DOF)
  arm:
    indices: [4, 5, 6, 7, 8, 9]
    names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    end_effector_frame: "tool0"

  # 그리퍼 (2-finger)
  gripper:
    indices: [10, 11]
    names: ["finger_left", "finger_right"]
    open_position: 0.04          # meters
    close_position: 0.0
    grasp_force: 10.0            # Newtons

# 제어 파라미터
control:
  position_stiffness: 1000.0
  position_damping: 100.0
  velocity_damping: 50.0

# 시뮬레이션 설정
simulation:
  physics_dt: 0.0166667          # 60Hz
  rendering_dt: 0.0166667

# 서버 설정
server:
  host: "0.0.0.0"
  port: 8000
```

---

## 10. 변경 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|----------|--------|
| 1.0 | 2025-12-14 | 초기 작성 | Claude Code |

---

**문서 끝**
