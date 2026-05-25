# Isaac Sim LLM 로봇 제어 연구 보고서

## 1. 개요

본 문서는 Unity에서 구현된 LLM 기반 로봇 제어 시스템(LLMRobotControl)을 Isaac Sim 환경으로 포팅하기 위한 종합 연구 보고서입니다. 4륜 구동 모바일 베이스와 매니퓰레이터가 장착된 로봇을 대상으로 합니다.

---

## 2. 기존 Unity LLMRobotControl 시스템 분석

### 2.1 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                     사용자 인터페이스 계층                        │
│  ┌───────────────┐      ┌──────────────────────┐               │
│  │ HTML UI       │      │ Unity UI (TextMeshPro)│               │
│  │ - text_input  │      │ - 입력 필드            │               │
│  │ - button_input│      │ - 피드백 디스플레이     │               │
│  └───────┬───────┘      └──────────┬───────────┘               │
└──────────┼──────────────────────────┼────────────────────────────┘
           │                          │
┌──────────▼──────────────────────────▼──────────────┐
│            LLMRobotControlManager                   │
│  - 명령 오케스트레이션                                │
│  - 사용자 확인 워크플로우                              │
│  - 명령 캐싱                                         │
└──────────┬──────────────────────────┬──────────────┘
           │                          │
     ┌─────▼─────┐             ┌─────▼──────┐
     │ OpenAI    │             │ Command    │
     │ Client    │             │ Validator  │
     └─────┬─────┘             └─────┬──────┘
           │                         │
           ▼                         ▼
     ┌──────────────────────────────────┐
     │     IKRobotController            │
     │  - 부드러운 움직임 보간             │
     │  - Bio IK 통합                    │
     └──────────────────────────────────┘
```

### 2.2 핵심 컴포넌트 (13개 스크립트)

| 파일 | 목적 | Isaac Sim 포팅 필요 |
|------|------|---------------------|
| `RobotCommand.cs` | 명령 데이터 구조 | ✅ Python 클래스로 변환 |
| `RobotControlConfig.cs` | 설정 시스템 | ✅ YAML 또는 Python Config로 변환 |
| `OpenAIClient.cs` | OpenAI API 통신 | ✅ Python requests/aiohttp로 변환 |
| `OpenAIResponseParser.cs` | LLM 응답 파싱 | ✅ Python JSON 파싱으로 변환 |
| `CommandValidator.cs` | 안전 검증 | ✅ Python 클래스로 변환 |
| `IKRobotController.cs` | IK 제어 | ✅ Isaac Sim Articulation API로 변환 |
| `LLMRobotControlManager.cs` | 메인 오케스트레이터 | ✅ Python 비동기 시스템으로 변환 |
| `WebUIBridge.cs` | HTTP 서버/UI 통합 | ✅ FastAPI/Flask로 변환 |
| `PerformanceMonitor.cs` | 성능 모니터링 | 선택적 |
| `EmergencyStopSystem.cs` | 비상 정지 | ✅ 필수 |

### 2.3 OpenAI Function Calling 스키마

```json
{
  "name": "move_robot_ik",
  "description": "로봇의 IK 타겟 이동",
  "parameters": {
    "type": "object",
    "properties": {
      "movement_type": {
        "type": "string",
        "enum": ["relative", "absolute"]
      },
      "direction": {
        "type": "string",
        "enum": ["forward", "backward", "left", "right", "up", "down"]
      },
      "distance": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 100.0
      },
      "position": {
        "type": "object",
        "properties": {
          "x": { "type": "number" },
          "y": { "type": "number" },
          "z": { "type": "number" }
        }
      },
      "speed": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 2.0,
        "default": 1.0
      },
      "duration": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 10.0,
        "default": 2.0
      }
    },
    "required": ["movement_type"]
  }
}
```

### 2.4 안전 시스템 (5단계)

1. **LLM 지침**: 시스템 프롬프트에 안전 지침 포함
2. **JSON 스키마**: 구조화된 출력으로 매개변수 유형 적용
3. **명령 검증**: 작업 공간 경계, 충돌 감지
4. **사용자 확인**: 선택적 human-in-the-loop
5. **비상 정지**: 즉시 정지 기능

---

## 3. Isaac Sim 로봇 제어 API 연구

### 3.1 Articulation Controller API

Isaac Sim의 Articulation Controller는 조인트 위치, 속도, 힘을 제어하는 저수준 컨트롤러입니다.

#### 핵심 메서드

```python
from isaacsim.core.utils.types import ArticulationAction
import numpy as np

# 위치 명령 생성
action = ArticulationAction(
    joint_positions=np.array([0.0, 0.5, 1.0, ...]),
    joint_indices=np.array([0, 1, 2, ...])  # 선택적
)

# 액션 적용
robot_articulation.apply_action(action)

# 직접 조인트 명령
robot.set_joint_positions([[...]])  # 위치 제어
robot.set_joint_velocities([[...]])  # 속도 제어
robot.set_joint_efforts([[...]])    # 힘/토크 제어
```

#### 제어 모드 설정

| 제어 모드 | Stiffness | Damping |
|----------|-----------|---------|
| 위치 제어 | 높음 | 낮음 |
| 속도 제어 | 0 | >0 |
| 힘 제어 | 0 | 0 |

### 3.2 IK 솔버 옵션

| 솔루션 | 실시간 성능 | 구현 난이도 |
|--------|------------|------------|
| **Lula IK** | Sub-ms (CPU) | ⭐⭐⭐⭐ |
| **cuRobo** | 37,000 IK/초 (GPU) | ⭐⭐⭐ |
| **IKFast** | ~4 μs (해석적) | ⭐⭐ |
| **Isaac Lab Diff IK** | <1 ms (GPU 배치) | ⭐⭐⭐⭐ |

### 3.3 모바일 베이스 + 매니퓰레이터 제어

```python
# 모바일 베이스 속도 제어 (4륜)
base_velocities = np.array([v_fl, v_fr, v_rl, v_rr])
robot.set_joint_velocities(base_velocities, joint_indices=wheel_indices)

# 매니퓰레이터 위치 제어
arm_positions = np.array([j1, j2, j3, j4, j5, j6])
robot.set_joint_positions(arm_positions, joint_indices=arm_indices)
```

---

## 4. LLM 로봇 제어 통합 패턴 연구

### 4.1 주요 프레임워크 비교 (2024-2025)

| 프레임워크 | 특징 | 출처 |
|-----------|------|------|
| **ELLMER** | GPT-4 + RAG, 긴 horizon 작업 | [Nature Machine Intelligence](https://www.nature.com/articles/s42256-025-01005-x) |
| **OpenVLA** | 7B VLA 모델, Isaac Sim 통합 | [GitHub](https://github.com/openvla/openvla) |
| **Ada (MIT)** | 자연어로 59-89% 작업 정확도 향상 | [MIT News](https://news.mit.edu/2024/natural-language-boosts-llm-performance-coding-planning-robotics-0501) |
| **CLEAR** | 자연어 + 컴퓨터 비전 통합 | [ACM/IEEE HRI 2024](https://dl.acm.org/doi/10.1145/3610978.3640671) |

### 4.2 권장 아키텍처: 하이브리드 접근

```
┌─────────────────────────────────────────────────────────────────┐
│                      사용자 인터페이스                            │
│    ┌──────────────────┐    ┌────────────────────────┐          │
│    │  Web UI          │    │  음성 인터페이스 (선택)   │          │
│    │  (FastAPI)       │    │                        │          │
│    └────────┬─────────┘    └────────────────────────┘          │
└─────────────┼─────────────────────────────────────────────────────┘
              │
┌─────────────▼─────────────────────────────────────────────────────┐
│              LLM Robot Controller (Python)                       │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │  OpenAI/Anthropic API Client (async)                        │ │
│  │  - Function Calling (move_robot, grip, navigate)            │ │
│  │  - 시스템 프롬프트 (로봇 상태, 작업 공간 정보)                    │ │
│  └─────────────────────────────────────────────────────────────┘ │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │  Command Validator                                          │ │
│  │  - 작업 공간 경계 검사                                         │ │
│  │  - 충돌 감지 (cuRobo/RMPflow)                                 │ │
│  │  - 속도/가속도 제한                                            │ │
│  └─────────────────────────────────────────────────────────────┘ │
└──────────────────────────────┬────────────────────────────────────┘
                               │
┌──────────────────────────────▼────────────────────────────────────┐
│              Isaac Sim Robot Interface                            │
│  ┌───────────────────────┐  ┌────────────────────────────────┐  │
│  │  Mobile Base Control  │  │  Manipulator Control           │  │
│  │  - Velocity commands  │  │  - Lula IK / cuRobo            │  │
│  │  - Odometry feedback  │  │  - Joint position/velocity     │  │
│  └───────────────────────┘  └────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
```

### 4.3 ROS2 통합 옵션

**방법 1: Isaac Sim ROS2 Bridge (권장)**

```python
# OmniGraph ROS2 노드 활성화
# /isaac_joint_states 토픽 발행
# /isaac_joint_commands 토픽 구독
```

**방법 2: topic_based_ros2_control**

```python
# ros2_control 하드웨어 인터페이스 사용
# MoveIt2 통합 가능
```

**왕복 지연 시간**: Python 토픽 기준 10-20ms

---

## 5. 권장 구현 아키텍처

### 5.1 파일 구조

```
isaac_llm_robot_control/
├── config/
│   ├── robot_config.yaml         # 로봇 설정
│   ├── workspace_config.yaml     # 작업 공간 경계
│   └── llm_config.yaml           # LLM API 설정
├── core/
│   ├── __init__.py
│   ├── robot_command.py          # 명령 데이터 클래스
│   ├── llm_client.py             # OpenAI/Anthropic 클라이언트
│   ├── response_parser.py        # LLM 응답 파싱
│   ├── command_validator.py      # 안전 검증
│   └── control_manager.py        # 메인 오케스트레이터
├── isaac_interface/
│   ├── __init__.py
│   ├── robot_controller.py       # Isaac Sim 로봇 제어
│   ├── mobile_base.py            # 모바일 베이스 제어
│   ├── manipulator.py            # 매니퓰레이터 제어
│   └── ik_solver.py              # IK 솔버 래퍼
├── web/
│   ├── __init__.py
│   ├── server.py                 # FastAPI 웹 서버
│   ├── static/                   # HTML/CSS/JS
│   └── templates/
├── utils/
│   ├── __init__.py
│   ├── safety.py                 # 비상 정지
│   └── performance_monitor.py    # 성능 모니터링
├── scripts/
│   ├── run_standalone.py         # Isaac Sim 스탠드얼론 실행
│   └── run_with_ros2.py          # ROS2 통합 실행
├── tests/
│   └── ...
└── requirements.txt
```

### 5.2 핵심 클래스 설계

#### 5.2.1 RobotCommand 데이터 클래스

```python
# core/robot_command.py
from dataclasses import dataclass
from typing import Optional, Literal
import numpy as np

@dataclass
class RobotCommand:
    movement_type: Literal["relative", "absolute", "mobile_base"]
    direction: Optional[str] = None  # forward, backward, left, right, up, down
    distance: float = 0.0            # cm 단위
    absolute_position: Optional[np.ndarray] = None  # [x, y, z]
    speed: float = 1.0               # 0.1 - 2.0
    duration: float = 2.0            # 0.1 - 10.0 초

    # 모바일 베이스 전용
    linear_velocity: float = 0.0     # m/s
    angular_velocity: float = 0.0    # rad/s

    # 그리퍼 전용
    gripper_action: Optional[Literal["open", "close"]] = None
```

#### 5.2.2 LLM 클라이언트

```python
# core/llm_client.py
import asyncio
import openai
from typing import Callable, Optional
import json

class LLMClient:
    def __init__(self, config: dict):
        self.client = openai.AsyncOpenAI(api_key=config["api_key"])
        self.model = config.get("model", "gpt-4-turbo")
        self.timeout = config.get("timeout", 30)
        self.last_call_time = 0
        self.min_interval = config.get("min_interval", 1.0)

        # Function calling 정의
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "move_manipulator",
                    "description": "매니퓰레이터 엔드이펙터 이동",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "movement_type": {"type": "string", "enum": ["relative", "absolute"]},
                            "direction": {"type": "string", "enum": ["forward", "backward", "left", "right", "up", "down"]},
                            "distance": {"type": "number", "minimum": 0.1, "maximum": 100.0},
                            "position": {
                                "type": "object",
                                "properties": {"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}}
                            },
                            "speed": {"type": "number", "minimum": 0.1, "maximum": 2.0, "default": 1.0}
                        },
                        "required": ["movement_type"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "move_mobile_base",
                    "description": "모바일 베이스 이동",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "linear_velocity": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                            "angular_velocity": {"type": "number", "minimum": -1.5, "maximum": 1.5},
                            "duration": {"type": "number", "minimum": 0.1, "maximum": 10.0}
                        },
                        "required": ["linear_velocity"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "control_gripper",
                    "description": "그리퍼 열기/닫기",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {"type": "string", "enum": ["open", "close"]}
                        },
                        "required": ["action"]
                    }
                }
            }
        ]

    async def send_command(self, user_message: str, system_prompt: str) -> dict:
        """LLM에 명령 전송 및 Function Call 결과 반환"""
        response = await self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            tools=self.tools,
            tool_choice="auto",
            temperature=0.1,
            max_tokens=200
        )

        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            return {
                "function": tool_call.function.name,
                "arguments": json.loads(tool_call.function.arguments)
            }
        return {"error": "No function call generated"}
```

#### 5.2.3 Isaac Sim 로봇 컨트롤러

```python
# isaac_interface/robot_controller.py
from isaacsim import SimulationApp

class IsaacRobotController:
    def __init__(self, robot_prim_path: str, config: dict):
        self.robot_prim_path = robot_prim_path
        self.config = config
        self.articulation = None
        self.ik_solver = None

    def initialize(self, stage):
        """Isaac Sim 스테이지에서 로봇 초기화"""
        from isaacsim.core.articulations import Articulation
        from isaacsim.robot_motion.motion_generation import (
            ArticulationKinematicsSolver,
            LulaKinematicsSolver
        )

        self.articulation = Articulation(self.robot_prim_path)

        # Lula IK 솔버 초기화
        self.lula_solver = LulaKinematicsSolver(
            robot_description_path=self.config["lula_description_path"],
            urdf_path=self.config["urdf_path"]
        )

        self.ik_solver = ArticulationKinematicsSolver(
            self.articulation,
            self.lula_solver,
            end_effector_frame_name=self.config["end_effector_frame"]
        )

    def move_to_position(self, target_position: np.ndarray,
                         target_orientation: np.ndarray = None) -> bool:
        """IK를 사용하여 엔드이펙터 위치 이동"""
        action, success = self.ik_solver.compute_inverse_kinematics(
            target_position,
            target_orientation
        )

        if success:
            self.articulation.apply_action(action)
        return success

    def move_base(self, linear_vel: float, angular_vel: float):
        """모바일 베이스 속도 제어"""
        # 4륜 구동 속도 계산
        wheel_velocities = self._compute_wheel_velocities(linear_vel, angular_vel)
        self.articulation.set_joint_velocities(
            wheel_velocities,
            joint_indices=self.config["wheel_joint_indices"]
        )

    def control_gripper(self, action: str):
        """그리퍼 제어"""
        gripper_position = self.config["gripper_open"] if action == "open" else self.config["gripper_close"]
        self.articulation.set_joint_positions(
            [gripper_position],
            joint_indices=self.config["gripper_joint_indices"]
        )

    def _compute_wheel_velocities(self, linear_vel, angular_vel) -> np.ndarray:
        """차동 구동 또는 메카넘/옴니 휠 속도 계산"""
        wheel_base = self.config["wheel_base"]
        wheel_radius = self.config["wheel_radius"]

        # 차동 구동 예시 (4륜)
        left_vel = (linear_vel - angular_vel * wheel_base / 2) / wheel_radius
        right_vel = (linear_vel + angular_vel * wheel_base / 2) / wheel_radius

        return np.array([left_vel, right_vel, left_vel, right_vel])

    def get_end_effector_position(self) -> np.ndarray:
        """현재 엔드이펙터 위치 반환"""
        return self.lula_solver.compute_forward_kinematics(
            self.articulation.get_joint_positions()
        )

    def emergency_stop(self):
        """비상 정지"""
        num_joints = self.articulation.num_dof
        self.articulation.set_joint_velocities(np.zeros(num_joints))
        self.articulation.set_joint_efforts(np.zeros(num_joints))
```

#### 5.2.4 메인 컨트롤 매니저

```python
# core/control_manager.py
import asyncio
from typing import Optional
from .llm_client import LLMClient
from .command_validator import CommandValidator
from .robot_command import RobotCommand
from ..isaac_interface.robot_controller import IsaacRobotController

class LLMRobotControlManager:
    def __init__(self, config: dict):
        self.config = config
        self.llm_client = LLMClient(config["llm"])
        self.validator = CommandValidator(config["workspace"])
        self.robot_controller: Optional[IsaacRobotController] = None

        # 명령 캐시
        self.command_cache = {}
        self.max_cache_size = config.get("max_cache_size", 50)

        # 상태
        self.is_moving = False
        self.emergency_stopped = False

        # 콜백
        self.on_command_received = None
        self.on_command_validated = None
        self.on_command_failed = None
        self.on_movement_completed = None

    def set_robot_controller(self, controller: IsaacRobotController):
        """Isaac Sim 로봇 컨트롤러 설정"""
        self.robot_controller = controller

    async def process_command(self, user_command: str) -> dict:
        """자연어 명령 처리"""
        if self.is_moving:
            return {"error": "Robot is currently moving"}

        if self.emergency_stopped:
            return {"error": "Emergency stop active. Reset required."}

        # 콜백 호출
        if self.on_command_received:
            self.on_command_received(user_command)

        # 캐시 확인
        cache_key = user_command.lower().strip()
        if cache_key in self.command_cache:
            return await self._execute_cached_command(cache_key)

        # LLM 호출
        system_prompt = self._build_system_prompt()
        try:
            llm_result = await self.llm_client.send_command(user_command, system_prompt)
        except Exception as e:
            if self.on_command_failed:
                self.on_command_failed(str(e))
            return {"error": f"LLM API error: {e}"}

        if "error" in llm_result:
            return llm_result

        # 명령 파싱
        command = self._parse_llm_result(llm_result)

        # 검증
        current_pos = self.robot_controller.get_end_effector_position()
        validation_result = self.validator.validate(command, current_pos)

        if not validation_result["is_valid"]:
            if self.on_command_failed:
                self.on_command_failed(validation_result["error"])
            return {"error": validation_result["error"]}

        # 캐시 저장
        self._cache_command(cache_key, command)

        # 실행
        return await self._execute_command(command)

    async def _execute_command(self, command: RobotCommand) -> dict:
        """명령 실행"""
        self.is_moving = True

        try:
            if command.movement_type == "mobile_base":
                self.robot_controller.move_base(
                    command.linear_velocity,
                    command.angular_velocity
                )
                await asyncio.sleep(command.duration)
                self.robot_controller.move_base(0, 0)  # 정지

            elif command.gripper_action:
                self.robot_controller.control_gripper(command.gripper_action)

            else:
                # 매니퓰레이터 이동
                if command.movement_type == "relative":
                    target_pos = self._calculate_relative_position(command)
                else:
                    target_pos = command.absolute_position

                success = self.robot_controller.move_to_position(target_pos)
                if not success:
                    return {"error": "IK solution not found"}

                # 이동 완료 대기 (간단한 구현)
                await asyncio.sleep(command.duration / command.speed)

            if self.on_movement_completed:
                self.on_movement_completed(command)

            return {"success": True, "command": command}

        finally:
            self.is_moving = False

    def emergency_stop(self):
        """비상 정지"""
        self.emergency_stopped = True
        self.is_moving = False
        if self.robot_controller:
            self.robot_controller.emergency_stop()

    def reset_emergency_stop(self):
        """비상 정지 해제"""
        self.emergency_stopped = False

    def _build_system_prompt(self) -> str:
        """시스템 프롬프트 생성"""
        workspace = self.config["workspace"]
        return f"""당신은 로봇 제어 시스템입니다.

작업 공간 경계:
- X: {workspace['min'][0]} ~ {workspace['max'][0]} m
- Y: {workspace['min'][1]} ~ {workspace['max'][1]} m
- Z: {workspace['min'][2]} ~ {workspace['max'][2]} m

좌표 시스템:
- 앞/뒤: Z축 (+앞, -뒤)
- 좌/우: X축 (-좌, +우)
- 위/아래: Y축 (+위, -아래)

거리 단위: 센티미터 (cm)

사용 가능한 함수:
1. move_manipulator: 매니퓰레이터 엔드이펙터 이동
2. move_mobile_base: 모바일 베이스 이동
3. control_gripper: 그리퍼 열기/닫기

항상 안전을 최우선으로 하고, 작업 공간 내에서만 이동하세요.
"""

    def _parse_llm_result(self, result: dict) -> RobotCommand:
        """LLM 결과를 RobotCommand로 변환"""
        func_name = result["function"]
        args = result["arguments"]

        if func_name == "move_manipulator":
            return RobotCommand(
                movement_type=args.get("movement_type", "relative"),
                direction=args.get("direction"),
                distance=args.get("distance", 0),
                absolute_position=np.array([args["position"]["x"], args["position"]["y"], args["position"]["z"]]) if "position" in args else None,
                speed=args.get("speed", 1.0)
            )
        elif func_name == "move_mobile_base":
            return RobotCommand(
                movement_type="mobile_base",
                linear_velocity=args.get("linear_velocity", 0),
                angular_velocity=args.get("angular_velocity", 0),
                duration=args.get("duration", 2.0)
            )
        elif func_name == "control_gripper":
            return RobotCommand(
                movement_type="gripper",
                gripper_action=args["action"]
            )
```

### 5.3 웹 UI 서버

```python
# web/server.py
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import asyncio

app = FastAPI()
control_manager = None  # 전역 컨트롤 매니저

@app.post("/api/command")
async def process_command(request: dict):
    """자연어 명령 처리 API"""
    command = request.get("command", "")
    result = await control_manager.process_command(command)
    return result

@app.post("/api/emergency_stop")
async def emergency_stop():
    """비상 정지 API"""
    control_manager.emergency_stop()
    return {"status": "emergency_stopped"}

@app.post("/api/reset")
async def reset():
    """비상 정지 해제 API"""
    control_manager.reset_emergency_stop()
    return {"status": "reset"}

@app.get("/api/status")
async def get_status():
    """로봇 상태 API"""
    return {
        "is_moving": control_manager.is_moving,
        "emergency_stopped": control_manager.emergency_stopped,
        "position": control_manager.robot_controller.get_end_effector_position().tolist()
    }

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """실시간 상태 업데이트용 WebSocket"""
    await websocket.accept()
    while True:
        status = await get_status()
        await websocket.send_json(status)
        await asyncio.sleep(0.1)  # 10Hz 업데이트
```

---

## 6. OpenVLA 통합 옵션 (고급)

### 6.1 OpenVLA 개요

프로젝트에 이미 OpenVLA가 설치되어 있습니다 (`ust_ws/openvla/`). OpenVLA는 7B 파라미터 VLA(Vision-Language-Action) 모델로, 이미지와 자연어 명령을 입력받아 직접 로봇 액션을 출력합니다.

### 6.2 OpenVLA vs GPT-4 Function Calling 비교

| 특성 | GPT-4 Function Calling | OpenVLA |
|------|----------------------|---------|
| 입력 | 텍스트만 | 이미지 + 텍스트 |
| 출력 | 구조화된 명령 | 직접 액션 벡터 |
| 학습 | 불필요 | 파인튜닝 필요 |
| 추론 속도 | 1-3초 (API) | ~6Hz (로컬) |
| VRAM | 없음 | 15GB |
| 일반화 | 높음 | 중간 (파인튜닝 필요) |

### 6.3 하이브리드 접근 권장

```
고수준 계획: GPT-4/Claude (자연어 → 고수준 작업 분해)
저수준 제어: OpenVLA (이미지 + 작업 → 액션)
```

---

## 7. 구현 로드맵

### Phase 1: 기본 인프라 (1-2주)

1. **프로젝트 구조 생성**
   - [ ] Python 패키지 구조 설정
   - [ ] 설정 파일 (YAML) 생성
   - [ ] 의존성 관리 (requirements.txt)

2. **핵심 클래스 구현**
   - [ ] RobotCommand 데이터 클래스
   - [ ] LLMClient (OpenAI API)
   - [ ] CommandValidator

3. **Isaac Sim 인터페이스**
   - [ ] 로봇 Articulation 래퍼
   - [ ] 조인트 인덱스 매핑
   - [ ] 기본 위치/속도 제어 테스트

### Phase 2: IK 및 모션 제어 (1-2주)

1. **IK 솔버 통합**
   - [ ] Lula IK 설정 (robot_description.yaml)
   - [ ] IK 테스트 및 튜닝

2. **모바일 베이스 제어**
   - [ ] 4륜 구동 속도 계산
   - [ ] Odometry 피드백

3. **그리퍼 제어**
   - [ ] 열기/닫기 위치 설정

### Phase 3: LLM 통합 (1주)

1. **Function Calling 구현**
   - [ ] 3개 함수 정의 (manipulator, base, gripper)
   - [ ] 시스템 프롬프트 최적화

2. **명령 처리 파이프라인**
   - [ ] 비동기 처리 (asyncio)
   - [ ] 명령 캐싱
   - [ ] 에러 처리

### Phase 4: 안전 시스템 (1주)

1. **검증 시스템**
   - [ ] 작업 공간 경계 검사
   - [ ] 충돌 감지 (RMPflow 또는 cuRobo)
   - [ ] 속도/가속도 제한

2. **비상 정지**
   - [ ] 키보드 단축키
   - [ ] 웹 UI 버튼
   - [ ] 자동 복구 로직

### Phase 5: 웹 UI (1주)

1. **FastAPI 서버**
   - [ ] REST API 엔드포인트
   - [ ] WebSocket 실시간 업데이트

2. **프론트엔드**
   - [ ] 텍스트 입력 UI
   - [ ] 버튼 입력 UI
   - [ ] 상태 표시

### Phase 6: 테스트 및 최적화 (1주)

1. **테스트**
   - [ ] 단위 테스트
   - [ ] 통합 테스트
   - [ ] 시뮬레이션 테스트

2. **최적화**
   - [ ] 응답 시간 측정
   - [ ] 캐싱 효과 분석
   - [ ] 문서화

---

## 8. 참고 자료

### 8.1 공식 문서

- [Isaac Sim Articulation Controller](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/articulation_controller.html)
- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/main/index.html)
- [Isaac Sim ROS2 Integration](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_simulation_control.html)
- [Lula Robot Description](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/tutorial_configure_manipulator.html)

### 8.2 연구 논문

- [OpenVLA: An Open-Source Vision-Language-Action Model](https://arxiv.org/abs/2406.09246) - Stanford, 2024
- [ELLMER: Embodied Large Language Model-Enabled Robot](https://www.nature.com/articles/s42256-025-01005-x) - Nature Machine Intelligence, 2025
- [Large Language Models for Robotics: Opportunities, Challenges, and Perspectives](https://www.sciencedirect.com/science/article/pii/S2949855424000613) - ScienceDirect, 2024

### 8.3 GitHub 리포지토리

- [OpenVLA](https://github.com/openvla/openvla)
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)
- [Awesome LLM Robotics](https://github.com/GT-RIPL/Awesome-LLM-Robotics)
- [isaac-ros2-control-sample](https://github.com/hijimasa/isaac-ros2-control-sample)

---

## 9. 결론

Unity에서 구현된 LLMRobotControl 시스템은 잘 구조화되어 있으며, Isaac Sim으로의 포팅이 충분히 가능합니다. 핵심 아키텍처(명령 파싱 → 검증 → 실행)를 유지하면서 다음 사항을 변경해야 합니다:

1. **언어**: C# → Python
2. **로봇 제어**: Unity Animation Rigging/Bio IK → Isaac Sim Articulation API + Lula IK
3. **웹 서버**: Unity HTTP Server → FastAPI/Flask
4. **비동기 처리**: Unity Coroutine → Python asyncio

권장 순서:
1. 먼저 GPT-4 Function Calling 기반 시스템 구현 (빠른 프로토타이핑)
2. 안정화 후 OpenVLA 통합 고려 (고급 비전 기반 제어)

---

**문서 버전**: 1.0
**작성일**: 2025-12-14
**작성자**: Claude Code Research Agent
