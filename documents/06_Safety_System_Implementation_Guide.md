# Isaac Sim LLM Robot Control - Safety System Implementation Guide

## 1. 문서 개요

### 1.1 목적
본 문서는 로봇 제어 시스템의 안전 기능 구현 가이드를 제공합니다. 5단계 안전 시스템, 비상 정지, 충돌 방지, 작업 공간 제한을 포함합니다.

### 1.2 범위
- 5단계 안전 시스템 아키텍처
- 비상 정지 시스템 (`emergency_stop.py`)
- 작업 공간 검증 (`workspace_validator.py`)
- 충돌 감지 (`collision_checker.py`)
- 속도/가속도 제한
- 실시간 모니터링

---

## 2. 5단계 안전 시스템 아키텍처

### 2.1 안전 계층 구조

```
┌─────────────────────────────────────────────────────────────────────┐
│                    5-Layer Safety Architecture                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Layer 1: LLM Safety Instructions (시스템 프롬프트)                   │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ • 안전 지침이 포함된 시스템 프롬프트                            │    │
│  │ • 작업 공간 경계 정보 제공                                     │    │
│  │ • 위험 동작 금지 지시                                          │    │
│  │ • 확인 요청 규칙                                               │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              ▼                                       │
│  Layer 2: JSON Schema Validation (파라미터 제한)                     │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ • Function Calling 스키마로 값 범위 제한                        │    │
│  │ • 거리: 0.1 ~ 100 cm                                           │    │
│  │ • 속도: 0.1 ~ 2.0 배                                           │    │
│  │ • 선속도: -1.0 ~ 1.0 m/s                                       │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              ▼                                       │
│  Layer 3: Command Validation (명령 검증)                             │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ • 작업 공간 경계 검사                                           │    │
│  │ • 속도/가속도 제한 확인                                         │    │
│  │ • 충돌 경로 검사                                                │    │
│  │ • 자기 충돌 검사                                                │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              ▼                                       │
│  Layer 4: User Confirmation (선택적)                                 │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ • 위험 동작 시 사용자 확인 요청                                  │    │
│  │ • 큰 이동 거리 확인                                             │    │
│  │ • 비정상 패턴 감지                                              │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                              ▼                                       │
│  Layer 5: Emergency Stop (비상 정지)                                 │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ • 즉시 모든 동작 정지                                           │    │
│  │ • 키보드 단축키 (ESC)                                          │    │
│  │ • 웹 UI 버튼                                                   │    │
│  │ • 프로그래밍 방식 호출                                          │    │
│  │ • 자동 트리거 (이상 감지)                                       │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 안전 검사 흐름

```
User Command
     │
     ▼
┌─────────────────┐
│ Layer 1: LLM    │ ── 시스템 프롬프트 안전 지침
│ Safety Prompt   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Layer 2: Schema │ ── JSON 스키마 범위 검사
│ Validation      │
└────────┬────────┘
         │ Schema OK?
         ├──── No ──▶ Reject Command
         │
         ▼
┌─────────────────┐
│ Layer 3: Logic  │ ── 작업공간/충돌/속도 검사
│ Validation      │
└────────┬────────┘
         │ Valid?
         ├──── No ──▶ Reject Command
         │
         ▼
┌─────────────────┐
│ Layer 4: User   │ ── 위험 동작 확인 요청
│ Confirmation    │    (선택적)
└────────┬────────┘
         │ Confirmed?
         ├──── No ──▶ Cancel Command
         │
         ▼
┌─────────────────┐
│ Execute Command │
└────────┬────────┘
         │
         ▼ (실행 중)
┌─────────────────┐
│ Layer 5:        │ ── 실시간 모니터링
│ Runtime Monitor │    이상 감지 시 E-Stop
└─────────────────┘
```

---

## 3. 비상 정지 시스템

### 3.1 상세 구현

```python
# safety/emergency_stop.py

import asyncio
import time
from typing import Optional, Callable, List, Dict
from enum import Enum
from dataclasses import dataclass
import logging
import threading

logger = logging.getLogger(__name__)


class EmergencyStopReason(Enum):
    """비상 정지 사유"""
    USER_TRIGGERED = "user_triggered"           # 사용자 수동 정지
    WORKSPACE_VIOLATION = "workspace_violation" # 작업 공간 이탈
    COLLISION_DETECTED = "collision_detected"   # 충돌 감지
    VELOCITY_EXCEEDED = "velocity_exceeded"     # 과속
    COMMUNICATION_LOST = "communication_lost"   # 통신 두절
    SYSTEM_ERROR = "system_error"               # 시스템 오류
    WATCHDOG_TIMEOUT = "watchdog_timeout"       # 워치독 타임아웃


@dataclass
class EmergencyStopEvent:
    """비상 정지 이벤트"""
    timestamp: float
    reason: EmergencyStopReason
    details: str
    position: Optional[List[float]] = None
    velocity: Optional[List[float]] = None


class EmergencyStopSystem:
    """비상 정지 시스템"""

    def __init__(self, config: Dict = None):
        self.config = config or {}

        # 상태
        self._is_stopped = False
        self._stop_reason: Optional[EmergencyStopReason] = None
        self._stop_events: List[EmergencyStopEvent] = []

        # 콜백
        self._on_stop_callbacks: List[Callable] = []
        self._on_reset_callbacks: List[Callable] = []

        # 로봇 컨트롤러 참조
        self._robot_controller = None

        # 워치독
        self._watchdog_enabled = self.config.get("watchdog_enabled", True)
        self._watchdog_timeout = self.config.get("watchdog_timeout", 5.0)
        self._last_heartbeat = time.time()
        self._watchdog_thread: Optional[threading.Thread] = None

        # 키보드 입력 감지 (선택적)
        self._keyboard_listener = None

        logger.info("Emergency stop system initialized")

    def set_robot_controller(self, controller):
        """로봇 컨트롤러 설정"""
        self._robot_controller = controller

    def register_on_stop(self, callback: Callable):
        """비상 정지 콜백 등록"""
        self._on_stop_callbacks.append(callback)

    def register_on_reset(self, callback: Callable):
        """리셋 콜백 등록"""
        self._on_reset_callbacks.append(callback)

    def trigger(
        self,
        reason: EmergencyStopReason = EmergencyStopReason.USER_TRIGGERED,
        details: str = ""
    ):
        """비상 정지 트리거

        Args:
            reason: 정지 사유
            details: 상세 정보
        """
        if self._is_stopped:
            logger.warning("Emergency stop already active")
            return

        logger.critical(f"🛑 EMERGENCY STOP: {reason.value} - {details}")

        self._is_stopped = True
        self._stop_reason = reason

        # 현재 위치/속도 기록
        position = None
        velocity = None
        if self._robot_controller:
            try:
                position = self._robot_controller.get_end_effector_position().tolist()
                velocity = self._robot_controller.get_joint_velocities().tolist()
            except:
                pass

        # 이벤트 기록
        event = EmergencyStopEvent(
            timestamp=time.time(),
            reason=reason,
            details=details,
            position=position,
            velocity=velocity
        )
        self._stop_events.append(event)

        # 로봇 정지 실행
        self._execute_stop()

        # 콜백 호출
        for callback in self._on_stop_callbacks:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Emergency stop callback error: {e}")

    def _execute_stop(self):
        """실제 로봇 정지 실행"""
        if self._robot_controller:
            try:
                # 모든 조인트 속도 0
                self._robot_controller.emergency_stop()
                logger.info("Robot stopped successfully")
            except Exception as e:
                logger.error(f"Failed to stop robot: {e}")

    def reset(self) -> bool:
        """비상 정지 해제

        Returns:
            bool: 리셋 성공 여부
        """
        if not self._is_stopped:
            logger.info("Emergency stop not active")
            return True

        # 안전 조건 확인
        if not self._check_safe_to_reset():
            logger.warning("Cannot reset: safety conditions not met")
            return False

        logger.info("Resetting emergency stop")
        self._is_stopped = False
        self._stop_reason = None

        # 콜백 호출
        for callback in self._on_reset_callbacks:
            try:
                callback()
            except Exception as e:
                logger.error(f"Reset callback error: {e}")

        return True

    def _check_safe_to_reset(self) -> bool:
        """리셋 안전 조건 확인"""
        if not self._robot_controller:
            return True

        try:
            # 속도가 충분히 낮은지 확인
            velocities = self._robot_controller.get_joint_velocities()
            max_velocity = max(abs(v) for v in velocities)

            if max_velocity > 0.01:  # 거의 정지 상태
                logger.warning(f"Robot still moving: max velocity = {max_velocity}")
                return False

            return True

        except Exception as e:
            logger.error(f"Safety check error: {e}")
            return False

    @property
    def is_stopped(self) -> bool:
        """비상 정지 상태"""
        return self._is_stopped

    @property
    def stop_reason(self) -> Optional[EmergencyStopReason]:
        """정지 사유"""
        return self._stop_reason

    def get_events(self) -> List[EmergencyStopEvent]:
        """정지 이벤트 이력"""
        return self._stop_events.copy()

    # ==========================================
    # Watchdog
    # ==========================================

    def start_watchdog(self):
        """워치독 시작"""
        if not self._watchdog_enabled:
            return

        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            daemon=True
        )
        self._watchdog_thread.start()
        logger.info(f"Watchdog started (timeout: {self._watchdog_timeout}s)")

    def heartbeat(self):
        """워치독 하트비트"""
        self._last_heartbeat = time.time()

    def _watchdog_loop(self):
        """워치독 루프"""
        while True:
            time.sleep(1.0)

            if self._is_stopped:
                continue

            elapsed = time.time() - self._last_heartbeat
            if elapsed > self._watchdog_timeout:
                logger.error(f"Watchdog timeout: {elapsed:.1f}s since last heartbeat")
                self.trigger(
                    EmergencyStopReason.WATCHDOG_TIMEOUT,
                    f"No heartbeat for {elapsed:.1f}s"
                )

    # ==========================================
    # 키보드 비상 정지 (선택적)
    # ==========================================

    def setup_keyboard_stop(self, key: str = "escape"):
        """키보드 비상 정지 설정"""
        try:
            from pynput import keyboard

            def on_press(pressed_key):
                if pressed_key == keyboard.Key.escape:
                    self.trigger(EmergencyStopReason.USER_TRIGGERED, "Keyboard ESC pressed")

            self._keyboard_listener = keyboard.Listener(on_press=on_press)
            self._keyboard_listener.start()
            logger.info("Keyboard emergency stop enabled (press ESC)")

        except ImportError:
            logger.warning("pynput not installed, keyboard stop disabled")


class SafetyMonitor:
    """실시간 안전 모니터"""

    def __init__(
        self,
        emergency_system: EmergencyStopSystem,
        workspace_bounds: Dict,
        velocity_limits: Dict
    ):
        self.emergency_system = emergency_system
        self.workspace_bounds = workspace_bounds
        self.velocity_limits = velocity_limits

        # 모니터링 상태
        self._is_monitoring = False
        self._monitor_task: Optional[asyncio.Task] = None

        # 임계값
        self._position_margin = 0.02  # 2cm 마진
        self._velocity_warning_threshold = 0.8  # 80% 임계

    async def start_monitoring(self, robot_controller, interval: float = 0.02):
        """모니터링 시작 (50Hz)"""
        self._is_monitoring = True
        logger.info("Safety monitoring started")

        while self._is_monitoring:
            try:
                # 위치 확인
                position = robot_controller.get_end_effector_position()
                if not self._check_workspace(position):
                    self.emergency_system.trigger(
                        EmergencyStopReason.WORKSPACE_VIOLATION,
                        f"Position {position} outside workspace"
                    )

                # 속도 확인
                velocities = robot_controller.get_joint_velocities()
                max_velocity = max(abs(v) for v in velocities)
                if max_velocity > self.velocity_limits.get("max_joint_velocity", 10.0):
                    self.emergency_system.trigger(
                        EmergencyStopReason.VELOCITY_EXCEEDED,
                        f"Velocity {max_velocity} exceeds limit"
                    )

                # 하트비트
                self.emergency_system.heartbeat()

                await asyncio.sleep(interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Safety monitor error: {e}")
                await asyncio.sleep(1.0)

    def stop_monitoring(self):
        """모니터링 중지"""
        self._is_monitoring = False
        logger.info("Safety monitoring stopped")

    def _check_workspace(self, position) -> bool:
        """작업 공간 확인"""
        bounds = self.workspace_bounds.get("bounds", {})
        min_bounds = bounds.get("min", [-1, -1, 0])
        max_bounds = bounds.get("max", [1, 1, 1.5])

        margin = self._position_margin

        return (
            min_bounds[0] + margin <= position[0] <= max_bounds[0] - margin and
            min_bounds[1] + margin <= position[1] <= max_bounds[1] - margin and
            min_bounds[2] + margin <= position[2] <= max_bounds[2] - margin
        )
```

---

## 4. 작업 공간 검증

### 4.1 상세 구현

```python
# safety/workspace_validator.py

import numpy as np
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class BoundaryViolationType(Enum):
    """경계 위반 유형"""
    NONE = "none"
    X_MIN = "x_min"
    X_MAX = "x_max"
    Y_MIN = "y_min"
    Y_MAX = "y_max"
    Z_MIN = "z_min"
    Z_MAX = "z_max"
    MULTIPLE = "multiple"


@dataclass
class WorkspaceCheckResult:
    """작업 공간 검사 결과"""
    is_valid: bool
    violation_type: BoundaryViolationType = BoundaryViolationType.NONE
    distance_to_boundary: float = float('inf')
    nearest_valid_point: Optional[np.ndarray] = None
    message: str = ""


class WorkspaceBounds:
    """작업 공간 경계"""

    def __init__(self, config: Dict):
        bounds = config.get("bounds", {})
        self.min = np.array(bounds.get("min", [-1.0, -1.0, 0.0]))
        self.max = np.array(bounds.get("max", [1.0, 1.0, 1.5]))

        safety = config.get("safety", {})
        self.margin = safety.get("workspace_margin", 0.05)

        # 내부 경계 (마진 적용)
        self.inner_min = self.min + self.margin
        self.inner_max = self.max - self.margin

        logger.info(f"Workspace bounds: {self.min} to {self.max} (margin: {self.margin})")

    def contains(self, point: np.ndarray, use_margin: bool = True) -> bool:
        """점이 경계 내에 있는지 확인"""
        if use_margin:
            min_b, max_b = self.inner_min, self.inner_max
        else:
            min_b, max_b = self.min, self.max

        return np.all(point >= min_b) and np.all(point <= max_b)

    def clamp(self, point: np.ndarray, use_margin: bool = True) -> np.ndarray:
        """점을 경계 내로 제한"""
        if use_margin:
            return np.clip(point, self.inner_min, self.inner_max)
        return np.clip(point, self.min, self.max)

    def distance_to_boundary(self, point: np.ndarray) -> float:
        """경계까지의 최소 거리 (음수 = 외부)"""
        dist_to_min = point - self.inner_min
        dist_to_max = self.inner_max - point

        return min(np.min(dist_to_min), np.min(dist_to_max))

    def get_center(self) -> np.ndarray:
        """중심점"""
        return (self.min + self.max) / 2

    def get_size(self) -> np.ndarray:
        """크기"""
        return self.max - self.min


class WorkspaceValidator:
    """작업 공간 검증기"""

    def __init__(self, config: Dict):
        self.bounds = WorkspaceBounds(config)

        # 추가 제한 영역 (예: 장애물)
        self.exclusion_zones: List[Dict] = config.get("exclusion_zones", [])

        # 경고 거리 (경계에 가까울 때)
        self.warning_distance = config.get("warning_distance", 0.1)

    def check_point(self, point: np.ndarray) -> WorkspaceCheckResult:
        """단일 점 검사"""
        point = np.asarray(point)

        # 기본 경계 검사
        if self.bounds.contains(point):
            distance = self.bounds.distance_to_boundary(point)

            if distance < self.warning_distance:
                return WorkspaceCheckResult(
                    is_valid=True,
                    distance_to_boundary=distance,
                    message=f"Warning: close to boundary ({distance:.3f}m)"
                )

            return WorkspaceCheckResult(
                is_valid=True,
                distance_to_boundary=distance
            )

        # 위반 유형 결정
        violation = self._determine_violation_type(point)
        nearest = self.bounds.clamp(point)
        distance = -np.linalg.norm(point - nearest)

        return WorkspaceCheckResult(
            is_valid=False,
            violation_type=violation,
            distance_to_boundary=distance,
            nearest_valid_point=nearest,
            message=f"Position {point} violates {violation.value} boundary"
        )

    def check_trajectory(
        self,
        start: np.ndarray,
        end: np.ndarray,
        num_samples: int = 10
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """궤적 검사

        Args:
            start: 시작점
            end: 끝점
            num_samples: 샘플 수

        Returns:
            (valid, first_violation_point): 유효 여부와 첫 위반 지점
        """
        start = np.asarray(start)
        end = np.asarray(end)

        for t in np.linspace(0, 1, num_samples):
            point = start + t * (end - start)
            result = self.check_point(point)

            if not result.is_valid:
                return False, point

        return True, None

    def check_sphere(
        self,
        center: np.ndarray,
        radius: float
    ) -> WorkspaceCheckResult:
        """구 영역 검사 (충돌 검사용)"""
        center = np.asarray(center)

        # 구의 모든 극점이 경계 내에 있는지 확인
        for axis in range(3):
            for direction in [-1, 1]:
                point = center.copy()
                point[axis] += direction * radius

                if not self.bounds.contains(point):
                    return WorkspaceCheckResult(
                        is_valid=False,
                        message=f"Sphere extends outside workspace"
                    )

        return WorkspaceCheckResult(is_valid=True)

    def _determine_violation_type(self, point: np.ndarray) -> BoundaryViolationType:
        """위반 유형 결정"""
        violations = []

        if point[0] < self.bounds.inner_min[0]:
            violations.append(BoundaryViolationType.X_MIN)
        if point[0] > self.bounds.inner_max[0]:
            violations.append(BoundaryViolationType.X_MAX)
        if point[1] < self.bounds.inner_min[1]:
            violations.append(BoundaryViolationType.Y_MIN)
        if point[1] > self.bounds.inner_max[1]:
            violations.append(BoundaryViolationType.Y_MAX)
        if point[2] < self.bounds.inner_min[2]:
            violations.append(BoundaryViolationType.Z_MIN)
        if point[2] > self.bounds.inner_max[2]:
            violations.append(BoundaryViolationType.Z_MAX)

        if len(violations) == 0:
            return BoundaryViolationType.NONE
        if len(violations) == 1:
            return violations[0]
        return BoundaryViolationType.MULTIPLE

    def suggest_safe_target(
        self,
        current: np.ndarray,
        desired: np.ndarray
    ) -> np.ndarray:
        """안전한 목표 위치 제안"""
        current = np.asarray(current)
        desired = np.asarray(desired)

        if self.bounds.contains(desired):
            return desired

        # 경계에서 가장 가까운 유효 점
        clamped = self.bounds.clamp(desired)

        # 현재 위치에서 clamped 방향으로 조금 안쪽
        direction = clamped - current
        distance = np.linalg.norm(direction)

        if distance < 0.01:
            return current

        direction = direction / distance
        safe_distance = max(0, distance - self.bounds.margin)

        return current + direction * safe_distance


class JointLimitsValidator:
    """조인트 제한 검증기"""

    def __init__(self, config: Dict):
        joints = config.get("joints", {}).get("arm", {})

        self.lower_limits = np.array(joints.get("lower_limits", [-np.pi] * 6))
        self.upper_limits = np.array(joints.get("upper_limits", [np.pi] * 6))
        self.velocity_limits = np.array(joints.get("velocity_limits", [2.0] * 6))
        self.effort_limits = np.array(joints.get("effort_limits", [100.0] * 6))

    def check_positions(self, positions: np.ndarray) -> Tuple[bool, str]:
        """조인트 위치 제한 확인"""
        positions = np.asarray(positions)

        if np.any(positions < self.lower_limits):
            violated = np.where(positions < self.lower_limits)[0]
            return False, f"Joint(s) {violated} below lower limit"

        if np.any(positions > self.upper_limits):
            violated = np.where(positions > self.upper_limits)[0]
            return False, f"Joint(s) {violated} above upper limit"

        return True, ""

    def check_velocities(self, velocities: np.ndarray) -> Tuple[bool, str]:
        """조인트 속도 제한 확인"""
        velocities = np.asarray(velocities)

        if np.any(np.abs(velocities) > self.velocity_limits):
            violated = np.where(np.abs(velocities) > self.velocity_limits)[0]
            return False, f"Joint(s) {violated} velocity exceeded"

        return True, ""

    def clamp_positions(self, positions: np.ndarray) -> np.ndarray:
        """조인트 위치 클램핑"""
        return np.clip(positions, self.lower_limits, self.upper_limits)

    def clamp_velocities(self, velocities: np.ndarray) -> np.ndarray:
        """조인트 속도 클램핑"""
        return np.clip(velocities, -self.velocity_limits, self.velocity_limits)
```

---

## 5. 충돌 감지

### 5.1 상세 구현

```python
# safety/collision_checker.py

import numpy as np
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class CollisionType(Enum):
    """충돌 유형"""
    NONE = "none"
    SELF_COLLISION = "self_collision"
    ENVIRONMENT = "environment"
    GROUND = "ground"
    OBSTACLE = "obstacle"


@dataclass
class CollisionResult:
    """충돌 검사 결과"""
    has_collision: bool
    collision_type: CollisionType = CollisionType.NONE
    collision_point: Optional[np.ndarray] = None
    distance: float = float('inf')
    link_pair: Optional[Tuple[str, str]] = None
    message: str = ""


class CollisionChecker:
    """충돌 검사기"""

    def __init__(self, config: Dict):
        self.config = config
        safety = config.get("safety", {})

        # 검사 활성화 설정
        self.self_collision_enabled = safety.get("self_collision_check", True)
        self.env_collision_enabled = safety.get("environment_collision_check", True)

        # 안전 거리
        self.min_distance = safety.get("collision_min_distance", 0.02)  # 2cm

        # 간단한 링크 충돌 행렬 (self-collision)
        # True = 충돌 가능, False = 인접하여 무시
        self._collision_matrix = self._build_collision_matrix()

        # 환경 장애물 (구 또는 박스로 근사)
        self.obstacles: List[Dict] = []

        logger.info("Collision checker initialized")

    def _build_collision_matrix(self) -> np.ndarray:
        """자기 충돌 행렬 생성"""
        # 6-DOF 매니퓰레이터 가정
        # 인접한 링크 쌍은 제외
        n_links = 7  # base + 6 joints
        matrix = np.ones((n_links, n_links), dtype=bool)

        # 대각선과 인접 요소 제외
        for i in range(n_links):
            matrix[i, i] = False
            if i > 0:
                matrix[i, i-1] = False
                matrix[i-1, i] = False

        return matrix

    def check_self_collision(
        self,
        joint_positions: np.ndarray,
        link_transforms: Optional[List[np.ndarray]] = None
    ) -> CollisionResult:
        """자기 충돌 검사

        Args:
            joint_positions: 조인트 위치
            link_transforms: 링크 변환 행렬 (없으면 계산)

        Returns:
            CollisionResult
        """
        if not self.self_collision_enabled:
            return CollisionResult(has_collision=False)

        # 여기에 실제 자기 충돌 검사 구현
        # 간단한 구현: 링크를 캡슐로 근사하여 거리 계산

        # 플레이스홀더
        return CollisionResult(has_collision=False)

    def check_environment_collision(
        self,
        position: np.ndarray,
        radius: float = 0.05
    ) -> CollisionResult:
        """환경 충돌 검사

        Args:
            position: 검사 위치
            radius: 충돌 체 반경

        Returns:
            CollisionResult
        """
        if not self.env_collision_enabled:
            return CollisionResult(has_collision=False)

        # 바닥 충돌 검사
        if position[2] - radius < 0:
            return CollisionResult(
                has_collision=True,
                collision_type=CollisionType.GROUND,
                collision_point=np.array([position[0], position[1], 0]),
                distance=position[2] - radius,
                message="Ground collision detected"
            )

        # 장애물 충돌 검사
        for obstacle in self.obstacles:
            result = self._check_obstacle_collision(position, radius, obstacle)
            if result.has_collision:
                return result

        return CollisionResult(has_collision=False)

    def check_trajectory_collision(
        self,
        start: np.ndarray,
        end: np.ndarray,
        radius: float = 0.05,
        num_samples: int = 20
    ) -> CollisionResult:
        """궤적 충돌 검사"""
        start = np.asarray(start)
        end = np.asarray(end)

        for t in np.linspace(0, 1, num_samples):
            point = start + t * (end - start)
            result = self.check_environment_collision(point, radius)

            if result.has_collision:
                return result

        return CollisionResult(has_collision=False)

    def _check_obstacle_collision(
        self,
        position: np.ndarray,
        radius: float,
        obstacle: Dict
    ) -> CollisionResult:
        """장애물 충돌 검사"""
        obs_type = obstacle.get("type", "sphere")

        if obs_type == "sphere":
            return self._check_sphere_collision(position, radius, obstacle)
        elif obs_type == "box":
            return self._check_box_collision(position, radius, obstacle)

        return CollisionResult(has_collision=False)

    def _check_sphere_collision(
        self,
        position: np.ndarray,
        radius: float,
        obstacle: Dict
    ) -> CollisionResult:
        """구 장애물 충돌 검사"""
        obs_center = np.array(obstacle["center"])
        obs_radius = obstacle["radius"]

        distance = np.linalg.norm(position - obs_center) - obs_radius - radius

        if distance < self.min_distance:
            return CollisionResult(
                has_collision=True,
                collision_type=CollisionType.OBSTACLE,
                collision_point=obs_center,
                distance=distance,
                message=f"Collision with sphere obstacle at {obs_center}"
            )

        return CollisionResult(has_collision=False, distance=distance)

    def _check_box_collision(
        self,
        position: np.ndarray,
        radius: float,
        obstacle: Dict
    ) -> CollisionResult:
        """박스 장애물 충돌 검사 (AABB)"""
        obs_min = np.array(obstacle["min"])
        obs_max = np.array(obstacle["max"])

        # 박스 확장 (radius 고려)
        expanded_min = obs_min - radius
        expanded_max = obs_max + radius

        if np.all(position >= expanded_min) and np.all(position <= expanded_max):
            return CollisionResult(
                has_collision=True,
                collision_type=CollisionType.OBSTACLE,
                distance=0,
                message="Collision with box obstacle"
            )

        # 최소 거리 계산
        closest = np.clip(position, obs_min, obs_max)
        distance = np.linalg.norm(position - closest) - radius

        return CollisionResult(has_collision=False, distance=distance)

    def add_obstacle(self, obstacle: Dict):
        """장애물 추가"""
        self.obstacles.append(obstacle)
        logger.info(f"Obstacle added: {obstacle}")

    def remove_obstacle(self, index: int):
        """장애물 제거"""
        if 0 <= index < len(self.obstacles):
            removed = self.obstacles.pop(index)
            logger.info(f"Obstacle removed: {removed}")

    def clear_obstacles(self):
        """모든 장애물 제거"""
        self.obstacles.clear()
        logger.info("All obstacles cleared")


class MotionSafetyValidator:
    """모션 안전 검증기 (속도/가속도)"""

    def __init__(self, config: Dict):
        limits = config.get("velocity_limits", {})
        manip = limits.get("manipulator", {})
        base = limits.get("base", {})

        # 매니퓰레이터 제한
        self.max_linear_velocity = manip.get("max_linear", 0.5)
        self.max_angular_velocity = manip.get("max_angular", 1.0)
        self.max_linear_acceleration = manip.get("max_acceleration", 2.0)
        self.max_angular_acceleration = manip.get("max_angular_acceleration", 5.0)

        # 모바일 베이스 제한
        self.max_base_linear = base.get("max_linear", 1.0)
        self.max_base_angular = base.get("max_angular", 1.5)

    def check_velocity(
        self,
        velocity: float,
        is_angular: bool = False
    ) -> Tuple[bool, str]:
        """속도 제한 확인"""
        limit = self.max_angular_velocity if is_angular else self.max_linear_velocity

        if abs(velocity) > limit:
            return False, f"Velocity {velocity:.3f} exceeds limit {limit}"

        return True, ""

    def check_acceleration(
        self,
        current_velocity: float,
        target_velocity: float,
        dt: float,
        is_angular: bool = False
    ) -> Tuple[bool, str]:
        """가속도 제한 확인"""
        acceleration = (target_velocity - current_velocity) / dt

        limit = self.max_angular_acceleration if is_angular else self.max_linear_acceleration

        if abs(acceleration) > limit:
            return False, f"Acceleration {acceleration:.3f} exceeds limit {limit}"

        return True, ""

    def limit_velocity_change(
        self,
        current_velocity: float,
        target_velocity: float,
        dt: float,
        is_angular: bool = False
    ) -> float:
        """가속도 제한에 맞춰 목표 속도 제한"""
        limit = self.max_angular_acceleration if is_angular else self.max_linear_acceleration

        max_change = limit * dt
        change = target_velocity - current_velocity
        limited_change = np.clip(change, -max_change, max_change)

        return current_velocity + limited_change
```

---

## 6. 통합 안전 검증기

### 6.1 종합 검증 클래스

```python
# safety/integrated_validator.py

import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import logging

from .workspace_validator import WorkspaceValidator, WorkspaceCheckResult
from .collision_checker import CollisionChecker, CollisionResult, MotionSafetyValidator
from .emergency_stop import EmergencyStopSystem, EmergencyStopReason

logger = logging.getLogger(__name__)


@dataclass
class SafetyValidationResult:
    """종합 안전 검증 결과"""
    is_safe: bool
    workspace_result: Optional[WorkspaceCheckResult] = None
    collision_result: Optional[CollisionResult] = None
    velocity_check: Optional[Tuple[bool, str]] = None
    warnings: List[str] = None
    suggested_correction: Optional[Dict] = None

    def __post_init__(self):
        if self.warnings is None:
            self.warnings = []


class IntegratedSafetyValidator:
    """통합 안전 검증기"""

    def __init__(self, config: Dict):
        self.config = config

        # 서브 검증기들
        self.workspace_validator = WorkspaceValidator(config.get("workspace", {}))
        self.collision_checker = CollisionChecker(config.get("workspace", {}))
        self.motion_validator = MotionSafetyValidator(config.get("workspace", {}))

        # 비상 정지 시스템 참조
        self.emergency_system: Optional[EmergencyStopSystem] = None

        # 검증 설정
        safety = config.get("workspace", {}).get("safety", {})
        self.require_all_checks = safety.get("require_all_checks", True)
        self.auto_emergency_on_critical = safety.get("auto_emergency_on_critical", True)

    def set_emergency_system(self, system: EmergencyStopSystem):
        """비상 정지 시스템 설정"""
        self.emergency_system = system

    def validate_command(
        self,
        target_position: np.ndarray,
        current_position: np.ndarray,
        current_velocity: Optional[np.ndarray] = None,
        speed_factor: float = 1.0
    ) -> SafetyValidationResult:
        """명령 종합 검증

        Args:
            target_position: 목표 위치
            current_position: 현재 위치
            current_velocity: 현재 속도 (선택)
            speed_factor: 속도 배율

        Returns:
            SafetyValidationResult
        """
        result = SafetyValidationResult(is_safe=True)

        # 1. 작업 공간 검사
        workspace_check = self.workspace_validator.check_point(target_position)
        result.workspace_result = workspace_check

        if not workspace_check.is_valid:
            result.is_safe = False
            result.suggested_correction = {
                "type": "position",
                "suggested": workspace_check.nearest_valid_point.tolist()
            }
            logger.warning(f"Workspace violation: {workspace_check.message}")

            if self.auto_emergency_on_critical:
                self._trigger_emergency(
                    EmergencyStopReason.WORKSPACE_VIOLATION,
                    workspace_check.message
                )

            return result

        elif workspace_check.distance_to_boundary < 0.1:
            result.warnings.append(f"Close to boundary: {workspace_check.distance_to_boundary:.3f}m")

        # 2. 충돌 검사
        collision_check = self.collision_checker.check_trajectory_collision(
            current_position, target_position
        )
        result.collision_result = collision_check

        if collision_check.has_collision:
            result.is_safe = False
            logger.warning(f"Collision detected: {collision_check.message}")

            if self.auto_emergency_on_critical:
                self._trigger_emergency(
                    EmergencyStopReason.COLLISION_DETECTED,
                    collision_check.message
                )

            return result

        # 3. 속도 검사
        distance = np.linalg.norm(target_position - current_position)
        estimated_velocity = distance * speed_factor  # 간단한 추정

        velocity_ok, velocity_msg = self.motion_validator.check_velocity(estimated_velocity)
        result.velocity_check = (velocity_ok, velocity_msg)

        if not velocity_ok:
            result.warnings.append(velocity_msg)
            # 속도 초과는 경고만 (자동 제한)

        return result

    def validate_realtime(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray
    ) -> SafetyValidationResult:
        """실시간 상태 검증 (모니터링용)"""
        result = SafetyValidationResult(is_safe=True)

        # 작업 공간 검사
        workspace_check = self.workspace_validator.check_point(current_position)
        result.workspace_result = workspace_check

        if not workspace_check.is_valid:
            result.is_safe = False
            self._trigger_emergency(
                EmergencyStopReason.WORKSPACE_VIOLATION,
                workspace_check.message
            )

        # 속도 검사
        max_velocity = np.max(np.abs(current_velocity))
        velocity_ok, velocity_msg = self.motion_validator.check_velocity(max_velocity)
        result.velocity_check = (velocity_ok, velocity_msg)

        if not velocity_ok:
            result.is_safe = False
            self._trigger_emergency(
                EmergencyStopReason.VELOCITY_EXCEEDED,
                velocity_msg
            )

        return result

    def _trigger_emergency(self, reason: EmergencyStopReason, details: str):
        """비상 정지 트리거"""
        if self.emergency_system and self.auto_emergency_on_critical:
            self.emergency_system.trigger(reason, details)
```

---

## 7. 테스트

### 7.1 안전 시스템 테스트

```python
# tests/test_safety.py

import pytest
import numpy as np
from unittest.mock import Mock

from safety.emergency_stop import EmergencyStopSystem, EmergencyStopReason
from safety.workspace_validator import WorkspaceValidator, WorkspaceBounds
from safety.collision_checker import CollisionChecker


class TestEmergencyStop:

    @pytest.fixture
    def system(self):
        return EmergencyStopSystem()

    def test_trigger_stop(self, system):
        assert not system.is_stopped
        system.trigger(EmergencyStopReason.USER_TRIGGERED, "Test")
        assert system.is_stopped
        assert system.stop_reason == EmergencyStopReason.USER_TRIGGERED

    def test_reset(self, system):
        system.trigger()
        assert system.reset()
        assert not system.is_stopped

    def test_callback(self, system):
        callback = Mock()
        system.register_on_stop(callback)
        system.trigger()
        callback.assert_called_once()


class TestWorkspaceValidator:

    @pytest.fixture
    def validator(self):
        config = {
            "bounds": {"min": [-1, -1, 0], "max": [1, 1, 1.5]},
            "safety": {"workspace_margin": 0.05}
        }
        return WorkspaceValidator(config)

    def test_valid_point(self, validator):
        result = validator.check_point(np.array([0, 0.5, 0.5]))
        assert result.is_valid

    def test_invalid_point(self, validator):
        result = validator.check_point(np.array([2, 0, 0]))
        assert not result.is_valid
        assert result.nearest_valid_point is not None

    def test_trajectory_check(self, validator):
        start = np.array([0, 0.5, 0.5])
        end = np.array([0.5, 0.5, 0.5])
        valid, point = validator.check_trajectory(start, end)
        assert valid

        end_invalid = np.array([2, 0, 0])
        valid, point = validator.check_trajectory(start, end_invalid)
        assert not valid


class TestCollisionChecker:

    @pytest.fixture
    def checker(self):
        config = {"safety": {"collision_min_distance": 0.02}}
        return CollisionChecker(config)

    def test_ground_collision(self, checker):
        result = checker.check_environment_collision(
            np.array([0, 0, 0.01]),
            radius=0.05
        )
        assert result.has_collision

    def test_no_collision(self, checker):
        result = checker.check_environment_collision(
            np.array([0, 0, 1.0]),
            radius=0.05
        )
        assert not result.has_collision

    def test_obstacle_collision(self, checker):
        checker.add_obstacle({
            "type": "sphere",
            "center": [0.5, 0.5, 0.5],
            "radius": 0.1
        })

        result = checker.check_environment_collision(
            np.array([0.5, 0.5, 0.5]),
            radius=0.05
        )
        assert result.has_collision
```

---

## 8. 변경 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|----------|--------|
| 1.0 | 2025-12-14 | 초기 작성 | Claude Code |

---

**문서 끝**
