# Isaac Sim LLM Robot Control - Core Module Implementation Guide

## 1. 문서 개요

### 1.1 목적
본 문서는 Core Control Layer의 핵심 모듈들에 대한 상세 구현 가이드를 제공합니다. 각 모듈의 구현 방법, 코드 예시, 테스트 방법을 포함합니다.

### 1.2 범위
- `robot_command.py`: 명령 데이터 구조
- `llm_client.py`: LLM API 클라이언트
- `response_parser.py`: LLM 응답 파싱
- `command_validator.py`: 명령 검증
- `control_manager.py`: 메인 오케스트레이터

---

## 2. robot_command.py - 명령 데이터 구조

### 2.1 목적
로봇 명령을 나타내는 타입 안전한 데이터 구조를 정의합니다.

### 2.2 상세 구현

```python
# core/robot_command.py

from dataclasses import dataclass, field
from typing import Optional, Literal, List
from enum import Enum
import numpy as np
import uuid
import time


class CommandType(Enum):
    """명령 유형"""
    MANIPULATOR = "manipulator"
    MOBILE_BASE = "mobile_base"
    GRIPPER = "gripper"
    COMPOSITE = "composite"  # 복합 명령


class MovementType(Enum):
    """이동 유형"""
    RELATIVE = "relative"
    ABSOLUTE = "absolute"


class Direction(Enum):
    """이동 방향"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"


class GripperAction(Enum):
    """그리퍼 동작"""
    OPEN = "open"
    CLOSE = "close"


@dataclass
class Position3D:
    """3D 위치"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @classmethod
    def from_numpy(cls, arr: np.ndarray) -> 'Position3D':
        return cls(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    def __add__(self, other: 'Position3D') -> 'Position3D':
        return Position3D(
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z
        )


@dataclass
class Orientation:
    """쿼터니언 방향"""
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

    def to_numpy(self) -> np.ndarray:
        return np.array([self.qw, self.qx, self.qy, self.qz])

    @classmethod
    def from_numpy(cls, arr: np.ndarray) -> 'Orientation':
        return cls(qw=float(arr[0]), qx=float(arr[1]),
                   qy=float(arr[2]), qz=float(arr[3]))


@dataclass
class RobotCommand:
    """로봇 명령 데이터 클래스"""

    # 기본 식별 정보
    command_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: float = field(default_factory=time.time)
    command_type: CommandType = CommandType.MANIPULATOR

    # 매니퓰레이터 명령 필드
    movement_type: Optional[MovementType] = None
    direction: Optional[Direction] = None
    distance: float = 0.0  # cm 단위
    absolute_position: Optional[Position3D] = None
    orientation: Optional[Orientation] = None
    speed: float = 1.0  # 0.1 ~ 2.0 배속

    # 모바일 베이스 명령 필드
    linear_velocity: float = 0.0   # m/s
    angular_velocity: float = 0.0  # rad/s
    duration: float = 2.0          # seconds

    # 그리퍼 명령 필드
    gripper_action: Optional[GripperAction] = None

    # 메타데이터
    raw_user_input: Optional[str] = None
    confidence: float = 1.0

    def to_dict(self) -> dict:
        """딕셔너리로 변환"""
        return {
            "command_id": self.command_id,
            "timestamp": self.timestamp,
            "command_type": self.command_type.value,
            "movement_type": self.movement_type.value if self.movement_type else None,
            "direction": self.direction.value if self.direction else None,
            "distance": self.distance,
            "absolute_position": {
                "x": self.absolute_position.x,
                "y": self.absolute_position.y,
                "z": self.absolute_position.z
            } if self.absolute_position else None,
            "speed": self.speed,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "duration": self.duration,
            "gripper_action": self.gripper_action.value if self.gripper_action else None
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'RobotCommand':
        """딕셔너리에서 생성"""
        cmd = cls()
        cmd.command_id = data.get("command_id", str(uuid.uuid4()))
        cmd.timestamp = data.get("timestamp", time.time())

        if data.get("command_type"):
            cmd.command_type = CommandType(data["command_type"])

        if data.get("movement_type"):
            cmd.movement_type = MovementType(data["movement_type"])

        if data.get("direction"):
            cmd.direction = Direction(data["direction"])

        cmd.distance = data.get("distance", 0.0)
        cmd.speed = data.get("speed", 1.0)

        if data.get("absolute_position"):
            pos = data["absolute_position"]
            cmd.absolute_position = Position3D(
                x=pos.get("x", 0),
                y=pos.get("y", 0),
                z=pos.get("z", 0)
            )

        cmd.linear_velocity = data.get("linear_velocity", 0.0)
        cmd.angular_velocity = data.get("angular_velocity", 0.0)
        cmd.duration = data.get("duration", 2.0)

        if data.get("gripper_action"):
            cmd.gripper_action = GripperAction(data["gripper_action"])

        return cmd

    def get_relative_offset(self) -> Optional[Position3D]:
        """상대 이동 오프셋 계산 (cm -> m 변환)"""
        if self.movement_type != MovementType.RELATIVE or not self.direction:
            return None

        distance_m = self.distance / 100.0  # cm to meters

        # 좌표계: X(좌우), Y(상하), Z(앞뒤)
        direction_map = {
            Direction.FORWARD: Position3D(z=distance_m),
            Direction.BACKWARD: Position3D(z=-distance_m),
            Direction.LEFT: Position3D(x=-distance_m),
            Direction.RIGHT: Position3D(x=distance_m),
            Direction.UP: Position3D(y=distance_m),
            Direction.DOWN: Position3D(y=-distance_m),
        }

        return direction_map.get(self.direction, Position3D())

    def validate_basic(self) -> tuple[bool, Optional[str]]:
        """기본 유효성 검사"""
        # 속도 범위 검사
        if not 0.1 <= self.speed <= 2.0:
            return False, f"Speed must be between 0.1 and 2.0, got {self.speed}"

        # 거리 범위 검사
        if self.distance < 0 or self.distance > 100:
            return False, f"Distance must be between 0 and 100cm, got {self.distance}"

        # 지속 시간 검사
        if self.duration < 0.1 or self.duration > 10.0:
            return False, f"Duration must be between 0.1 and 10.0s, got {self.duration}"

        # 명령 타입별 필수 필드 검사
        if self.command_type == CommandType.MANIPULATOR:
            if not self.movement_type:
                return False, "Manipulator command requires movement_type"
            if self.movement_type == MovementType.RELATIVE and not self.direction:
                return False, "Relative movement requires direction"
            if self.movement_type == MovementType.ABSOLUTE and not self.absolute_position:
                return False, "Absolute movement requires position"

        elif self.command_type == CommandType.GRIPPER:
            if not self.gripper_action:
                return False, "Gripper command requires gripper_action"

        return True, None


@dataclass
class CommandResult:
    """명령 실행 결과"""
    success: bool
    command_id: str
    message: str = ""
    error_code: Optional[str] = None
    execution_time: float = 0.0
    final_position: Optional[Position3D] = None

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "command_id": self.command_id,
            "message": self.message,
            "error_code": self.error_code,
            "execution_time": self.execution_time,
            "final_position": {
                "x": self.final_position.x,
                "y": self.final_position.y,
                "z": self.final_position.z
            } if self.final_position else None
        }


# Factory functions
def create_manipulator_relative_command(
    direction: str,
    distance: float,
    speed: float = 1.0
) -> RobotCommand:
    """상대 이동 매니퓰레이터 명령 생성"""
    return RobotCommand(
        command_type=CommandType.MANIPULATOR,
        movement_type=MovementType.RELATIVE,
        direction=Direction(direction),
        distance=distance,
        speed=speed
    )


def create_manipulator_absolute_command(
    x: float, y: float, z: float,
    speed: float = 1.0
) -> RobotCommand:
    """절대 위치 매니퓰레이터 명령 생성"""
    return RobotCommand(
        command_type=CommandType.MANIPULATOR,
        movement_type=MovementType.ABSOLUTE,
        absolute_position=Position3D(x=x, y=y, z=z),
        speed=speed
    )


def create_base_command(
    linear_velocity: float,
    angular_velocity: float = 0.0,
    duration: float = 2.0
) -> RobotCommand:
    """모바일 베이스 명령 생성"""
    return RobotCommand(
        command_type=CommandType.MOBILE_BASE,
        linear_velocity=linear_velocity,
        angular_velocity=angular_velocity,
        duration=duration
    )


def create_gripper_command(action: str) -> RobotCommand:
    """그리퍼 명령 생성"""
    return RobotCommand(
        command_type=CommandType.GRIPPER,
        gripper_action=GripperAction(action)
    )
```

### 2.3 사용 예시

```python
# 상대 이동 명령 생성
cmd1 = create_manipulator_relative_command("forward", 10.0, speed=1.5)
print(cmd1.to_dict())

# 절대 위치 명령 생성
cmd2 = create_manipulator_absolute_command(0.5, 0.3, 0.4)
print(cmd2.get_relative_offset())  # None (절대 위치 명령)

# 기본 유효성 검사
is_valid, error = cmd1.validate_basic()
print(f"Valid: {is_valid}, Error: {error}")
```

---

## 3. llm_client.py - LLM API 클라이언트

### 3.1 목적
OpenAI/Anthropic API와의 비동기 통신을 처리하고, Function Calling을 통해 구조화된 로봇 명령을 생성합니다.

### 3.2 상세 구현

```python
# core/llm_client.py

import asyncio
import json
import time
from typing import Optional, Dict, List, Any, AsyncIterator
from dataclasses import dataclass, field
import logging

# OpenAI 클라이언트
try:
    from openai import AsyncOpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

# Anthropic 클라이언트
try:
    from anthropic import AsyncAnthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False

logger = logging.getLogger(__name__)


@dataclass
class LLMConfig:
    """LLM 클라이언트 설정"""
    provider: str = "openai"  # openai | anthropic
    api_key: str = ""
    model: str = "gpt-4-turbo"
    temperature: float = 0.1
    max_tokens: int = 200
    timeout: float = 30.0
    min_interval: float = 1.0  # API 호출 간 최소 간격
    max_retries: int = 3
    retry_delay: float = 2.0


@dataclass
class LLMResult:
    """LLM 호출 결과"""
    success: bool
    function_name: Optional[str] = None
    arguments: Optional[Dict[str, Any]] = None
    raw_response: Optional[str] = None
    error: Optional[str] = None
    latency: float = 0.0


class ToolDefinition:
    """Function Calling 도구 정의"""

    @staticmethod
    def get_robot_tools() -> List[Dict]:
        """로봇 제어용 Function Calling 도구 정의"""
        return [
            {
                "type": "function",
                "function": {
                    "name": "move_manipulator",
                    "description": "매니퓰레이터 엔드이펙터를 이동합니다. 상대 이동(방향+거리) 또는 절대 위치로 이동 가능합니다.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "movement_type": {
                                "type": "string",
                                "enum": ["relative", "absolute"],
                                "description": "이동 유형: relative(상대), absolute(절대)"
                            },
                            "direction": {
                                "type": "string",
                                "enum": ["forward", "backward", "left", "right", "up", "down"],
                                "description": "상대 이동 방향 (movement_type이 relative일 때 필수)"
                            },
                            "distance": {
                                "type": "number",
                                "minimum": 0.1,
                                "maximum": 100.0,
                                "description": "이동 거리 (센티미터, cm)"
                            },
                            "position": {
                                "type": "object",
                                "properties": {
                                    "x": {"type": "number", "description": "X 좌표 (미터)"},
                                    "y": {"type": "number", "description": "Y 좌표 (미터)"},
                                    "z": {"type": "number", "description": "Z 좌표 (미터)"}
                                },
                                "description": "절대 위치 (movement_type이 absolute일 때 필수)"
                            },
                            "speed": {
                                "type": "number",
                                "minimum": 0.1,
                                "maximum": 2.0,
                                "default": 1.0,
                                "description": "이동 속도 배율 (1.0이 기본)"
                            }
                        },
                        "required": ["movement_type"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "move_mobile_base",
                    "description": "모바일 베이스(4륜 구동)를 이동합니다. 선속도와 각속도로 제어합니다.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "linear_velocity": {
                                "type": "number",
                                "minimum": -1.0,
                                "maximum": 1.0,
                                "description": "선속도 (m/s). 양수=전진, 음수=후진"
                            },
                            "angular_velocity": {
                                "type": "number",
                                "minimum": -1.5,
                                "maximum": 1.5,
                                "default": 0.0,
                                "description": "각속도 (rad/s). 양수=좌회전, 음수=우회전"
                            },
                            "duration": {
                                "type": "number",
                                "minimum": 0.1,
                                "maximum": 10.0,
                                "default": 2.0,
                                "description": "이동 지속 시간 (초)"
                            }
                        },
                        "required": ["linear_velocity"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "control_gripper",
                    "description": "그리퍼를 열거나 닫습니다.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": ["open", "close"],
                                "description": "그리퍼 동작: open(열기) 또는 close(닫기)"
                            }
                        },
                        "required": ["action"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "stop_robot",
                    "description": "로봇의 모든 움직임을 즉시 정지합니다.",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            }
        ]


class BaseLLMClient:
    """LLM 클라이언트 기본 클래스"""

    def __init__(self, config: LLMConfig):
        self.config = config
        self.tools = ToolDefinition.get_robot_tools()
        self.last_call_time = 0.0
        self._call_count = 0

    async def _rate_limit(self):
        """Rate limiting 적용"""
        elapsed = time.time() - self.last_call_time
        if elapsed < self.config.min_interval:
            await asyncio.sleep(self.config.min_interval - elapsed)
        self.last_call_time = time.time()

    async def send_command(
        self,
        user_message: str,
        system_prompt: str,
        context: Optional[List[Dict]] = None
    ) -> LLMResult:
        """명령 전송 (서브클래스에서 구현)"""
        raise NotImplementedError

    def get_call_count(self) -> int:
        """API 호출 횟수 반환"""
        return self._call_count


class OpenAIClient(BaseLLMClient):
    """OpenAI API 클라이언트"""

    def __init__(self, config: LLMConfig):
        super().__init__(config)
        if not OPENAI_AVAILABLE:
            raise ImportError("openai package not installed")
        self.client = AsyncOpenAI(api_key=config.api_key)

    async def send_command(
        self,
        user_message: str,
        system_prompt: str,
        context: Optional[List[Dict]] = None
    ) -> LLMResult:
        """OpenAI API로 명령 전송"""
        await self._rate_limit()

        messages = [{"role": "system", "content": system_prompt}]

        # 이전 컨텍스트 추가
        if context:
            messages.extend(context)

        messages.append({"role": "user", "content": user_message})

        start_time = time.time()

        for attempt in range(self.config.max_retries):
            try:
                response = await asyncio.wait_for(
                    self.client.chat.completions.create(
                        model=self.config.model,
                        messages=messages,
                        tools=self.tools,
                        tool_choice="auto",
                        temperature=self.config.temperature,
                        max_tokens=self.config.max_tokens
                    ),
                    timeout=self.config.timeout
                )

                self._call_count += 1
                latency = time.time() - start_time

                message = response.choices[0].message

                # Function call 결과 파싱
                if message.tool_calls:
                    tool_call = message.tool_calls[0]
                    return LLMResult(
                        success=True,
                        function_name=tool_call.function.name,
                        arguments=json.loads(tool_call.function.arguments),
                        raw_response=message.content,
                        latency=latency
                    )

                # 일반 텍스트 응답 (함수 호출 없음)
                return LLMResult(
                    success=False,
                    raw_response=message.content,
                    error="No function call in response",
                    latency=latency
                )

            except asyncio.TimeoutError:
                logger.warning(f"API timeout (attempt {attempt + 1}/{self.config.max_retries})")
                if attempt < self.config.max_retries - 1:
                    await asyncio.sleep(self.config.retry_delay)

            except Exception as e:
                logger.error(f"API error: {e}")
                if attempt < self.config.max_retries - 1:
                    await asyncio.sleep(self.config.retry_delay)
                else:
                    return LLMResult(
                        success=False,
                        error=str(e),
                        latency=time.time() - start_time
                    )

        return LLMResult(
            success=False,
            error="Max retries exceeded",
            latency=time.time() - start_time
        )


class AnthropicClient(BaseLLMClient):
    """Anthropic API 클라이언트"""

    def __init__(self, config: LLMConfig):
        super().__init__(config)
        if not ANTHROPIC_AVAILABLE:
            raise ImportError("anthropic package not installed")
        self.client = AsyncAnthropic(api_key=config.api_key)

    def _convert_tools_to_anthropic(self) -> List[Dict]:
        """OpenAI 형식 도구를 Anthropic 형식으로 변환"""
        anthropic_tools = []
        for tool in self.tools:
            if tool["type"] == "function":
                func = tool["function"]
                anthropic_tools.append({
                    "name": func["name"],
                    "description": func["description"],
                    "input_schema": func["parameters"]
                })
        return anthropic_tools

    async def send_command(
        self,
        user_message: str,
        system_prompt: str,
        context: Optional[List[Dict]] = None
    ) -> LLMResult:
        """Anthropic API로 명령 전송"""
        await self._rate_limit()

        messages = []

        # 이전 컨텍스트 추가
        if context:
            for msg in context:
                if msg["role"] in ["user", "assistant"]:
                    messages.append(msg)

        messages.append({"role": "user", "content": user_message})

        start_time = time.time()

        for attempt in range(self.config.max_retries):
            try:
                response = await asyncio.wait_for(
                    self.client.messages.create(
                        model=self.config.model,
                        max_tokens=self.config.max_tokens,
                        system=system_prompt,
                        messages=messages,
                        tools=self._convert_tools_to_anthropic()
                    ),
                    timeout=self.config.timeout
                )

                self._call_count += 1
                latency = time.time() - start_time

                # Tool use 결과 파싱
                for block in response.content:
                    if block.type == "tool_use":
                        return LLMResult(
                            success=True,
                            function_name=block.name,
                            arguments=block.input,
                            latency=latency
                        )

                # 텍스트 응답만 있는 경우
                text_content = "".join(
                    block.text for block in response.content
                    if block.type == "text"
                )
                return LLMResult(
                    success=False,
                    raw_response=text_content,
                    error="No tool use in response",
                    latency=latency
                )

            except asyncio.TimeoutError:
                logger.warning(f"API timeout (attempt {attempt + 1})")
                if attempt < self.config.max_retries - 1:
                    await asyncio.sleep(self.config.retry_delay)

            except Exception as e:
                logger.error(f"API error: {e}")
                if attempt < self.config.max_retries - 1:
                    await asyncio.sleep(self.config.retry_delay)
                else:
                    return LLMResult(
                        success=False,
                        error=str(e),
                        latency=time.time() - start_time
                    )

        return LLMResult(
            success=False,
            error="Max retries exceeded",
            latency=time.time() - start_time
        )


def create_llm_client(config: LLMConfig) -> BaseLLMClient:
    """설정에 따라 적절한 LLM 클라이언트 생성"""
    if config.provider == "openai":
        return OpenAIClient(config)
    elif config.provider == "anthropic":
        return AnthropicClient(config)
    else:
        raise ValueError(f"Unknown provider: {config.provider}")


class SystemPromptBuilder:
    """시스템 프롬프트 생성기"""

    def __init__(self, workspace_config: Dict):
        self.workspace = workspace_config

    def build(
        self,
        current_position: Optional[tuple] = None,
        additional_context: str = ""
    ) -> str:
        """시스템 프롬프트 생성"""
        bounds = self.workspace.get("bounds", {})
        min_bounds = bounds.get("min", [-1, -1, 0])
        max_bounds = bounds.get("max", [1, 1, 1.5])

        prompt = f"""당신은 로봇 제어 시스템입니다. 사용자의 자연어 명령을 로봇 제어 함수 호출로 변환합니다.

## 작업 공간 정보
작업 공간 경계:
- X축 (좌우): {min_bounds[0]}m ~ {max_bounds[0]}m
- Y축 (상하): {min_bounds[1]}m ~ {max_bounds[1]}m
- Z축 (앞뒤): {min_bounds[2]}m ~ {max_bounds[2]}m

## 좌표계
- 앞/뒤: Z축 (+Z = 앞, -Z = 뒤)
- 좌/우: X축 (-X = 좌, +X = 우)
- 위/아래: Y축 (+Y = 위, -Y = 아래)

## 단위
- 거리: 센티미터 (cm) 단위로 입력받아 처리합니다
- 속도: m/s (선속도), rad/s (각속도)

## 사용 가능한 함수
1. **move_manipulator**: 매니퓰레이터 엔드이펙터 이동
   - relative: 방향과 거리로 상대 이동 (예: "앞으로 10cm")
   - absolute: 절대 좌표로 이동 (예: "X=0.5, Y=0.3, Z=0.4 위치로")

2. **move_mobile_base**: 모바일 베이스 이동
   - linear_velocity: 선속도 (양수=전진, 음수=후진)
   - angular_velocity: 각속도 (양수=좌회전, 음수=우회전)
   - duration: 이동 시간

3. **control_gripper**: 그리퍼 열기/닫기

4. **stop_robot**: 모든 움직임 정지

## 안전 지침
- 항상 작업 공간 경계 내에서만 이동하세요
- 급격한 속도 변경을 피하세요
- 불확실한 명령에는 확인을 요청하세요
"""

        if current_position:
            prompt += f"\n## 현재 상태\n현재 엔드이펙터 위치: X={current_position[0]:.3f}m, Y={current_position[1]:.3f}m, Z={current_position[2]:.3f}m\n"

        if additional_context:
            prompt += f"\n## 추가 컨텍스트\n{additional_context}\n"

        return prompt
```

### 3.3 사용 예시

```python
import asyncio
from core.llm_client import create_llm_client, LLMConfig, SystemPromptBuilder

async def main():
    # 설정 생성
    config = LLMConfig(
        provider="openai",
        api_key="your-api-key",
        model="gpt-4-turbo",
        temperature=0.1
    )

    # 클라이언트 생성
    client = create_llm_client(config)

    # 시스템 프롬프트 생성
    workspace = {"bounds": {"min": [-1, -1, 0], "max": [1, 1, 1.5]}}
    prompt_builder = SystemPromptBuilder(workspace)
    system_prompt = prompt_builder.build(current_position=(0.5, 0.3, 0.4))

    # 명령 전송
    result = await client.send_command(
        user_message="앞으로 10센티미터 이동해줘",
        system_prompt=system_prompt
    )

    if result.success:
        print(f"Function: {result.function_name}")
        print(f"Arguments: {result.arguments}")
    else:
        print(f"Error: {result.error}")

asyncio.run(main())
```

---

## 4. command_validator.py - 명령 검증기

### 4.1 목적
로봇 명령의 안전성을 검증하고, 위험한 명령을 사전에 차단합니다.

### 4.2 상세 구현

```python
# core/command_validator.py

from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from enum import Enum
import numpy as np
import logging

from .robot_command import RobotCommand, CommandType, MovementType, Position3D

logger = logging.getLogger(__name__)


class ValidationErrorCode(Enum):
    """검증 에러 코드"""
    NONE = "none"
    OUT_OF_WORKSPACE = "out_of_workspace"
    VELOCITY_EXCEEDED = "velocity_exceeded"
    ACCELERATION_EXCEEDED = "acceleration_exceeded"
    COLLISION_DETECTED = "collision_detected"
    INVALID_COMMAND = "invalid_command"
    SELF_COLLISION = "self_collision"


@dataclass
class WorkspaceBounds:
    """작업 공간 경계"""
    min_x: float = -1.0
    max_x: float = 1.0
    min_y: float = -1.0
    max_y: float = 1.0
    min_z: float = 0.0
    max_z: float = 1.5
    margin: float = 0.05  # 안전 마진

    def contains(self, position: np.ndarray) -> bool:
        """위치가 작업 공간 내에 있는지 확인 (마진 적용)"""
        return (
            self.min_x + self.margin <= position[0] <= self.max_x - self.margin and
            self.min_y + self.margin <= position[1] <= self.max_y - self.margin and
            self.min_z + self.margin <= position[2] <= self.max_z - self.margin
        )

    def clamp(self, position: np.ndarray) -> np.ndarray:
        """위치를 작업 공간 내로 제한"""
        return np.array([
            np.clip(position[0], self.min_x + self.margin, self.max_x - self.margin),
            np.clip(position[1], self.min_y + self.margin, self.max_y - self.margin),
            np.clip(position[2], self.min_z + self.margin, self.max_z - self.margin)
        ])

    @classmethod
    def from_config(cls, config: dict) -> 'WorkspaceBounds':
        """설정 딕셔너리에서 생성"""
        bounds = config.get("bounds", {})
        min_vals = bounds.get("min", [-1, -1, 0])
        max_vals = bounds.get("max", [1, 1, 1.5])
        return cls(
            min_x=min_vals[0], max_x=max_vals[0],
            min_y=min_vals[1], max_y=max_vals[1],
            min_z=min_vals[2], max_z=max_vals[2],
            margin=config.get("safety", {}).get("workspace_margin", 0.05)
        )


@dataclass
class VelocityLimits:
    """속도 제한"""
    # 매니퓰레이터
    max_linear_velocity: float = 0.5       # m/s
    max_angular_velocity: float = 1.0      # rad/s
    max_acceleration: float = 2.0          # m/s^2

    # 모바일 베이스
    max_base_linear: float = 1.0           # m/s
    max_base_angular: float = 1.5          # rad/s

    @classmethod
    def from_config(cls, config: dict) -> 'VelocityLimits':
        """설정 딕셔너리에서 생성"""
        manip = config.get("velocity_limits", {}).get("manipulator", {})
        base = config.get("velocity_limits", {}).get("base", {})
        return cls(
            max_linear_velocity=manip.get("max_linear", 0.5),
            max_angular_velocity=manip.get("max_angular", 1.0),
            max_acceleration=manip.get("max_acceleration", 2.0),
            max_base_linear=base.get("max_linear", 1.0),
            max_base_angular=base.get("max_angular", 1.5)
        )


@dataclass
class ValidationResult:
    """검증 결과"""
    is_valid: bool
    error_code: ValidationErrorCode = ValidationErrorCode.NONE
    error_message: Optional[str] = None
    warnings: List[str] = field(default_factory=list)
    computed_target: Optional[np.ndarray] = None
    suggested_target: Optional[np.ndarray] = None  # 유효한 대안 위치

    def add_warning(self, warning: str):
        self.warnings.append(warning)


class CollisionChecker:
    """충돌 검사 인터페이스 (선택적)"""

    def __init__(self, config: dict):
        self.enabled = config.get("safety", {}).get("environment_collision_check", False)
        self.self_collision_check = config.get("safety", {}).get("self_collision_check", True)
        # 실제 구현에서는 cuRobo 또는 RMPflow 연동

    def check_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        num_points: int = 10
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """궤적 충돌 검사"""
        if not self.enabled:
            return True, None

        # 여기에 실제 충돌 검사 로직 구현
        # cuRobo 또는 RMPflow 사용
        return True, None

    def check_self_collision(self, joint_positions: np.ndarray) -> bool:
        """자기 충돌 검사"""
        if not self.self_collision_check:
            return False
        # 여기에 실제 자기 충돌 검사 로직 구현
        return False


class CommandValidator:
    """명령 검증기"""

    def __init__(self, config: dict):
        self.workspace = WorkspaceBounds.from_config(config)
        self.velocity_limits = VelocityLimits.from_config(config)
        self.collision_checker = CollisionChecker(config)

    def validate(
        self,
        command: RobotCommand,
        current_position: np.ndarray,
        current_velocity: Optional[np.ndarray] = None
    ) -> ValidationResult:
        """명령 종합 검증"""

        # 1. 기본 유효성 검사
        basic_valid, basic_error = command.validate_basic()
        if not basic_valid:
            return ValidationResult(
                is_valid=False,
                error_code=ValidationErrorCode.INVALID_COMMAND,
                error_message=basic_error
            )

        # 명령 타입별 검증
        if command.command_type == CommandType.MANIPULATOR:
            return self._validate_manipulator_command(command, current_position, current_velocity)
        elif command.command_type == CommandType.MOBILE_BASE:
            return self._validate_base_command(command)
        elif command.command_type == CommandType.GRIPPER:
            return ValidationResult(is_valid=True)  # 그리퍼는 추가 검증 불필요
        else:
            return ValidationResult(is_valid=True)

    def _validate_manipulator_command(
        self,
        command: RobotCommand,
        current_position: np.ndarray,
        current_velocity: Optional[np.ndarray]
    ) -> ValidationResult:
        """매니퓰레이터 명령 검증"""

        result = ValidationResult(is_valid=True)

        # 목표 위치 계산
        if command.movement_type == MovementType.RELATIVE:
            offset = command.get_relative_offset()
            if offset is None:
                return ValidationResult(
                    is_valid=False,
                    error_code=ValidationErrorCode.INVALID_COMMAND,
                    error_message="Failed to compute relative offset"
                )
            target_pos = current_position + offset.to_numpy()
        else:
            if command.absolute_position is None:
                return ValidationResult(
                    is_valid=False,
                    error_code=ValidationErrorCode.INVALID_COMMAND,
                    error_message="Absolute position not specified"
                )
            target_pos = command.absolute_position.to_numpy()

        result.computed_target = target_pos

        # 2. 작업 공간 경계 검사
        if not self.workspace.contains(target_pos):
            suggested = self.workspace.clamp(target_pos)
            return ValidationResult(
                is_valid=False,
                error_code=ValidationErrorCode.OUT_OF_WORKSPACE,
                error_message=f"Target position {target_pos} is outside workspace bounds",
                computed_target=target_pos,
                suggested_target=suggested
            )

        # 3. 속도 제한 검사
        distance = np.linalg.norm(target_pos - current_position)
        estimated_velocity = (distance * command.speed) / command.duration

        if estimated_velocity > self.velocity_limits.max_linear_velocity:
            result.add_warning(
                f"Estimated velocity {estimated_velocity:.2f} m/s exceeds recommended limit. "
                f"Consider reducing speed or increasing duration."
            )

        # 4. 가속도 제한 검사 (현재 속도가 있는 경우)
        if current_velocity is not None:
            current_speed = np.linalg.norm(current_velocity)
            acceleration = abs(estimated_velocity - current_speed) / 0.1  # 100ms 기준
            if acceleration > self.velocity_limits.max_acceleration:
                result.add_warning(
                    f"High acceleration detected. Consider smoother transition."
                )

        # 5. 충돌 검사
        collision_free, collision_point = self.collision_checker.check_trajectory(
            current_position, target_pos
        )
        if not collision_free:
            return ValidationResult(
                is_valid=False,
                error_code=ValidationErrorCode.COLLISION_DETECTED,
                error_message=f"Potential collision detected at {collision_point}",
                computed_target=target_pos
            )

        return result

    def _validate_base_command(self, command: RobotCommand) -> ValidationResult:
        """모바일 베이스 명령 검증"""

        result = ValidationResult(is_valid=True)

        # 선속도 제한 검사
        if abs(command.linear_velocity) > self.velocity_limits.max_base_linear:
            return ValidationResult(
                is_valid=False,
                error_code=ValidationErrorCode.VELOCITY_EXCEEDED,
                error_message=f"Linear velocity {command.linear_velocity} exceeds limit {self.velocity_limits.max_base_linear}"
            )

        # 각속도 제한 검사
        if abs(command.angular_velocity) > self.velocity_limits.max_base_angular:
            return ValidationResult(
                is_valid=False,
                error_code=ValidationErrorCode.VELOCITY_EXCEEDED,
                error_message=f"Angular velocity {command.angular_velocity} exceeds limit {self.velocity_limits.max_base_angular}"
            )

        return result

    def check_emergency_conditions(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray
    ) -> Tuple[bool, Optional[str]]:
        """비상 상황 검사 (실시간 모니터링용)"""

        # 작업 공간 이탈 검사
        if not self.workspace.contains(current_position):
            return True, "Robot position outside workspace bounds"

        # 과속 검사
        speed = np.linalg.norm(current_velocity)
        if speed > self.velocity_limits.max_linear_velocity * 1.5:  # 150% 초과시
            return True, f"Excessive velocity detected: {speed:.2f} m/s"

        return False, None
```

### 4.3 사용 예시

```python
from core.command_validator import CommandValidator, ValidationResult
from core.robot_command import create_manipulator_relative_command
import numpy as np

# 설정
config = {
    "bounds": {"min": [-1, -1, 0], "max": [1, 1, 1.5]},
    "safety": {"workspace_margin": 0.05},
    "velocity_limits": {
        "manipulator": {"max_linear": 0.5, "max_acceleration": 2.0},
        "base": {"max_linear": 1.0, "max_angular": 1.5}
    }
}

# 검증기 생성
validator = CommandValidator(config)

# 명령 생성
command = create_manipulator_relative_command("forward", 20.0, speed=1.0)

# 현재 위치
current_pos = np.array([0.5, 0.3, 0.4])

# 검증
result = validator.validate(command, current_pos)

if result.is_valid:
    print(f"Command valid! Target: {result.computed_target}")
    if result.warnings:
        print(f"Warnings: {result.warnings}")
else:
    print(f"Command rejected: {result.error_message}")
    if result.suggested_target is not None:
        print(f"Suggested target: {result.suggested_target}")
```

---

## 5. control_manager.py - 메인 컨트롤 매니저

### 5.1 목적
전체 제어 흐름을 오케스트레이션하고, 각 컴포넌트를 조율합니다.

### 5.2 상세 구현

```python
# core/control_manager.py

import asyncio
from typing import Optional, Dict, Callable, List, Any
from enum import Enum
from dataclasses import dataclass, field
import time
import logging

from .robot_command import (
    RobotCommand, CommandResult, CommandType, MovementType,
    Position3D, GripperAction
)
from .llm_client import (
    BaseLLMClient, LLMConfig, LLMResult,
    create_llm_client, SystemPromptBuilder
)
from .command_validator import CommandValidator, ValidationResult

logger = logging.getLogger(__name__)


class ControllerState(Enum):
    """컨트롤러 상태"""
    INITIALIZING = "initializing"
    IDLE = "idle"
    PROCESSING = "processing"
    MOVING = "moving"
    EMERGENCY_STOPPED = "emergency_stopped"
    ERROR = "error"


class EventType(Enum):
    """이벤트 타입"""
    COMMAND_RECEIVED = "command_received"
    COMMAND_VALIDATED = "command_validated"
    COMMAND_REJECTED = "command_rejected"
    MOVEMENT_STARTED = "movement_started"
    MOVEMENT_COMPLETED = "movement_completed"
    MOVEMENT_FAILED = "movement_failed"
    EMERGENCY_STOP = "emergency_stop"
    RESET = "reset"
    STATE_CHANGED = "state_changed"


@dataclass
class ControllerStatus:
    """컨트롤러 상태 정보"""
    state: ControllerState
    is_moving: bool
    emergency_stopped: bool
    current_position: List[float]
    current_orientation: List[float]
    joint_positions: List[float]
    gripper_state: str
    last_command_id: Optional[str]
    last_error: Optional[str]
    uptime: float
    command_count: int


class CommandCache:
    """명령 캐시"""

    def __init__(self, max_size: int = 100, ttl: float = 3600):
        self.max_size = max_size
        self.ttl = ttl
        self._cache: Dict[str, tuple[RobotCommand, float]] = {}

    def get(self, key: str) -> Optional[RobotCommand]:
        """캐시에서 명령 조회"""
        key = key.lower().strip()
        if key in self._cache:
            command, timestamp = self._cache[key]
            if time.time() - timestamp < self.ttl:
                return command
            else:
                del self._cache[key]
        return None

    def put(self, key: str, command: RobotCommand):
        """캐시에 명령 저장"""
        key = key.lower().strip()

        # 캐시 크기 제한
        if len(self._cache) >= self.max_size:
            # 가장 오래된 항목 제거
            oldest_key = min(self._cache.keys(), key=lambda k: self._cache[k][1])
            del self._cache[oldest_key]

        self._cache[key] = (command, time.time())

    def clear(self):
        """캐시 초기화"""
        self._cache.clear()


class LLMRobotControlManager:
    """LLM 로봇 제어 매니저"""

    def __init__(self, config: Dict):
        self.config = config
        self._start_time = time.time()
        self._command_count = 0

        # 상태
        self._state = ControllerState.INITIALIZING
        self._is_moving = False
        self._emergency_stopped = False
        self._last_command_id: Optional[str] = None
        self._last_error: Optional[str] = None

        # 컴포넌트 초기화
        llm_config = LLMConfig(
            provider=config.get("llm", {}).get("provider", "openai"),
            api_key=config.get("llm", {}).get("api_key", ""),
            model=config.get("llm", {}).get("model", "gpt-4-turbo"),
            temperature=config.get("llm", {}).get("temperature", 0.1),
            max_tokens=config.get("llm", {}).get("max_tokens", 200),
            timeout=config.get("llm", {}).get("timeout", 30),
            min_interval=config.get("llm", {}).get("min_interval", 1.0)
        )
        self._llm_client = create_llm_client(llm_config)
        self._validator = CommandValidator(config.get("workspace", {}))
        self._prompt_builder = SystemPromptBuilder(config.get("workspace", {}))

        # 캐시
        cache_config = config.get("cache", {})
        self._cache = CommandCache(
            max_size=cache_config.get("max_size", 100),
            ttl=cache_config.get("ttl", 3600)
        )

        # 로봇 컨트롤러 (외부에서 설정)
        self._robot_controller = None

        # 이벤트 콜백
        self._callbacks: Dict[EventType, List[Callable]] = {
            event: [] for event in EventType
        }

        # 현재 처리 중인 작업
        self._current_task: Optional[asyncio.Task] = None

        self._set_state(ControllerState.IDLE)

    def set_robot_controller(self, controller):
        """로봇 컨트롤러 설정"""
        self._robot_controller = controller
        logger.info("Robot controller connected")

    def register_callback(self, event: EventType, callback: Callable):
        """이벤트 콜백 등록"""
        self._callbacks[event].append(callback)

    def _emit_event(self, event: EventType, data: Any = None):
        """이벤트 발생"""
        for callback in self._callbacks[event]:
            try:
                callback(event, data)
            except Exception as e:
                logger.error(f"Callback error for {event}: {e}")

    def _set_state(self, new_state: ControllerState):
        """상태 변경"""
        old_state = self._state
        self._state = new_state
        logger.debug(f"State changed: {old_state} -> {new_state}")
        self._emit_event(EventType.STATE_CHANGED, {
            "old_state": old_state,
            "new_state": new_state
        })

    async def process_command(self, user_input: str) -> CommandResult:
        """자연어 명령 처리 메인 메서드"""

        # 상태 검사
        if self._emergency_stopped:
            return CommandResult(
                success=False,
                command_id="",
                message="Emergency stop is active. Please reset first.",
                error_code="EMERGENCY_STOPPED"
            )

        if self._is_moving:
            return CommandResult(
                success=False,
                command_id="",
                message="Robot is currently moving. Please wait.",
                error_code="BUSY"
            )

        if self._robot_controller is None:
            return CommandResult(
                success=False,
                command_id="",
                message="Robot controller not connected.",
                error_code="NO_CONTROLLER"
            )

        self._set_state(ControllerState.PROCESSING)
        self._emit_event(EventType.COMMAND_RECEIVED, {"input": user_input})

        try:
            # 1. 캐시 확인
            cached_command = self._cache.get(user_input)
            if cached_command:
                logger.info(f"Using cached command for: {user_input}")
                return await self._execute_command(cached_command)

            # 2. LLM 호출
            current_pos = self._get_current_position()
            system_prompt = self._prompt_builder.build(
                current_position=tuple(current_pos) if current_pos is not None else None
            )

            llm_result = await self._llm_client.send_command(
                user_message=user_input,
                system_prompt=system_prompt
            )

            if not llm_result.success:
                self._last_error = llm_result.error
                self._set_state(ControllerState.IDLE)
                return CommandResult(
                    success=False,
                    command_id="",
                    message=f"LLM error: {llm_result.error}",
                    error_code="LLM_ERROR"
                )

            # 3. 명령 파싱
            command = self._parse_llm_result(llm_result, user_input)

            # 4. 검증
            validation_result = self._validator.validate(command, current_pos)

            if not validation_result.is_valid:
                self._emit_event(EventType.COMMAND_REJECTED, {
                    "command": command,
                    "reason": validation_result.error_message
                })
                self._set_state(ControllerState.IDLE)
                return CommandResult(
                    success=False,
                    command_id=command.command_id,
                    message=f"Validation failed: {validation_result.error_message}",
                    error_code=validation_result.error_code.value
                )

            self._emit_event(EventType.COMMAND_VALIDATED, {
                "command": command,
                "target": validation_result.computed_target
            })

            # 5. 캐시 저장
            self._cache.put(user_input, command)

            # 6. 실행
            return await self._execute_command(command)

        except Exception as e:
            logger.error(f"Error processing command: {e}")
            self._last_error = str(e)
            self._set_state(ControllerState.ERROR)
            return CommandResult(
                success=False,
                command_id="",
                message=f"Internal error: {e}",
                error_code="INTERNAL_ERROR"
            )

    async def _execute_command(self, command: RobotCommand) -> CommandResult:
        """명령 실행"""
        self._is_moving = True
        self._last_command_id = command.command_id
        self._set_state(ControllerState.MOVING)
        self._emit_event(EventType.MOVEMENT_STARTED, {"command": command})

        start_time = time.time()

        try:
            if command.command_type == CommandType.MANIPULATOR:
                success = await self._execute_manipulator_command(command)
            elif command.command_type == CommandType.MOBILE_BASE:
                success = await self._execute_base_command(command)
            elif command.command_type == CommandType.GRIPPER:
                success = await self._execute_gripper_command(command)
            else:
                success = False

            execution_time = time.time() - start_time
            self._command_count += 1

            if success:
                self._emit_event(EventType.MOVEMENT_COMPLETED, {
                    "command": command,
                    "execution_time": execution_time
                })
                final_pos = self._get_current_position()
                return CommandResult(
                    success=True,
                    command_id=command.command_id,
                    message="Command executed successfully",
                    execution_time=execution_time,
                    final_position=Position3D.from_numpy(final_pos) if final_pos is not None else None
                )
            else:
                self._emit_event(EventType.MOVEMENT_FAILED, {
                    "command": command,
                    "reason": "Execution failed"
                })
                return CommandResult(
                    success=False,
                    command_id=command.command_id,
                    message="Command execution failed",
                    error_code="EXECUTION_FAILED",
                    execution_time=execution_time
                )

        finally:
            self._is_moving = False
            self._set_state(ControllerState.IDLE)

    async def _execute_manipulator_command(self, command: RobotCommand) -> bool:
        """매니퓰레이터 명령 실행"""
        current_pos = self._get_current_position()

        if command.movement_type == MovementType.RELATIVE:
            offset = command.get_relative_offset()
            target_pos = current_pos + offset.to_numpy()
        else:
            target_pos = command.absolute_position.to_numpy()

        # IK 기반 이동
        success = self._robot_controller.move_to_position(
            target_pos,
            target_orientation=None  # 기본 방향 유지
        )

        if success:
            # 이동 완료 대기
            move_time = command.duration / command.speed
            await asyncio.sleep(move_time)

        return success

    async def _execute_base_command(self, command: RobotCommand) -> bool:
        """모바일 베이스 명령 실행"""
        self._robot_controller.move_base(
            command.linear_velocity,
            command.angular_velocity
        )

        await asyncio.sleep(command.duration)

        # 정지
        self._robot_controller.move_base(0, 0)
        return True

    async def _execute_gripper_command(self, command: RobotCommand) -> bool:
        """그리퍼 명령 실행"""
        if command.gripper_action == GripperAction.OPEN:
            self._robot_controller.control_gripper("open")
        else:
            self._robot_controller.control_gripper("close")

        await asyncio.sleep(0.5)  # 그리퍼 동작 대기
        return True

    def _parse_llm_result(self, result: LLMResult, user_input: str) -> RobotCommand:
        """LLM 결과를 RobotCommand로 변환"""
        args = result.arguments or {}

        if result.function_name == "move_manipulator":
            movement_type = MovementType(args.get("movement_type", "relative"))

            if movement_type == MovementType.RELATIVE:
                from .robot_command import Direction
                return RobotCommand(
                    command_type=CommandType.MANIPULATOR,
                    movement_type=movement_type,
                    direction=Direction(args["direction"]) if "direction" in args else None,
                    distance=args.get("distance", 10.0),
                    speed=args.get("speed", 1.0),
                    raw_user_input=user_input
                )
            else:
                pos = args.get("position", {})
                return RobotCommand(
                    command_type=CommandType.MANIPULATOR,
                    movement_type=movement_type,
                    absolute_position=Position3D(
                        x=pos.get("x", 0),
                        y=pos.get("y", 0),
                        z=pos.get("z", 0)
                    ),
                    speed=args.get("speed", 1.0),
                    raw_user_input=user_input
                )

        elif result.function_name == "move_mobile_base":
            return RobotCommand(
                command_type=CommandType.MOBILE_BASE,
                linear_velocity=args.get("linear_velocity", 0),
                angular_velocity=args.get("angular_velocity", 0),
                duration=args.get("duration", 2.0),
                raw_user_input=user_input
            )

        elif result.function_name == "control_gripper":
            return RobotCommand(
                command_type=CommandType.GRIPPER,
                gripper_action=GripperAction(args["action"]),
                raw_user_input=user_input
            )

        elif result.function_name == "stop_robot":
            # 정지 명령은 즉시 비상 정지 호출
            self.emergency_stop()
            return RobotCommand(
                command_type=CommandType.MANIPULATOR,
                raw_user_input=user_input
            )

        raise ValueError(f"Unknown function: {result.function_name}")

    def _get_current_position(self) -> Optional[np.ndarray]:
        """현재 엔드이펙터 위치 조회"""
        if self._robot_controller:
            return self._robot_controller.get_end_effector_position()
        return None

    def emergency_stop(self):
        """비상 정지"""
        logger.warning("EMERGENCY STOP activated!")
        self._emergency_stopped = True
        self._is_moving = False
        self._set_state(ControllerState.EMERGENCY_STOPPED)

        if self._robot_controller:
            self._robot_controller.emergency_stop()

        # 현재 작업 취소
        if self._current_task and not self._current_task.done():
            self._current_task.cancel()

        self._emit_event(EventType.EMERGENCY_STOP, {
            "timestamp": time.time()
        })

    def reset(self):
        """비상 정지 해제"""
        if self._emergency_stopped:
            logger.info("Emergency stop reset")
            self._emergency_stopped = False
            self._last_error = None
            self._set_state(ControllerState.IDLE)
            self._emit_event(EventType.RESET, {
                "timestamp": time.time()
            })

    def get_status(self) -> ControllerStatus:
        """현재 상태 조회"""
        current_pos = self._get_current_position()
        joint_pos = []
        gripper_state = "unknown"

        if self._robot_controller:
            joint_pos = self._robot_controller.get_joint_positions().tolist()
            gripper_state = self._robot_controller.gripper.get_state().value

        return ControllerStatus(
            state=self._state,
            is_moving=self._is_moving,
            emergency_stopped=self._emergency_stopped,
            current_position=current_pos.tolist() if current_pos is not None else [0, 0, 0],
            current_orientation=[1, 0, 0, 0],  # TODO: 실제 방향
            joint_positions=joint_pos,
            gripper_state=gripper_state,
            last_command_id=self._last_command_id,
            last_error=self._last_error,
            uptime=time.time() - self._start_time,
            command_count=self._command_count
        )

    def clear_cache(self):
        """명령 캐시 초기화"""
        self._cache.clear()
        logger.info("Command cache cleared")
```

### 5.3 사용 예시

```python
import asyncio
from core.control_manager import LLMRobotControlManager, EventType

# 설정
config = {
    "llm": {
        "provider": "openai",
        "api_key": "your-api-key",
        "model": "gpt-4-turbo"
    },
    "workspace": {
        "bounds": {"min": [-1, -1, 0], "max": [1, 1, 1.5]},
        "velocity_limits": {
            "manipulator": {"max_linear": 0.5}
        }
    },
    "cache": {"max_size": 100, "ttl": 3600}
}

# 매니저 생성
manager = LLMRobotControlManager(config)

# 이벤트 콜백 등록
def on_movement_completed(event, data):
    print(f"Movement completed! Time: {data['execution_time']:.2f}s")

manager.register_callback(EventType.MOVEMENT_COMPLETED, on_movement_completed)

# 로봇 컨트롤러 연결 (실제 환경에서)
# manager.set_robot_controller(robot_controller)

# 명령 처리
async def main():
    result = await manager.process_command("앞으로 10센티미터 이동해줘")
    print(f"Result: {result.success}, {result.message}")

    # 상태 확인
    status = manager.get_status()
    print(f"State: {status.state}, Position: {status.current_position}")

asyncio.run(main())
```

---

## 6. 테스트 가이드

### 6.1 단위 테스트 예시

```python
# tests/test_robot_command.py

import pytest
import numpy as np
from core.robot_command import (
    RobotCommand, CommandType, MovementType, Direction,
    Position3D, create_manipulator_relative_command
)


class TestRobotCommand:

    def test_create_relative_command(self):
        cmd = create_manipulator_relative_command("forward", 10.0, speed=1.5)
        assert cmd.command_type == CommandType.MANIPULATOR
        assert cmd.movement_type == MovementType.RELATIVE
        assert cmd.direction == Direction.FORWARD
        assert cmd.distance == 10.0
        assert cmd.speed == 1.5

    def test_relative_offset_forward(self):
        cmd = create_manipulator_relative_command("forward", 10.0)
        offset = cmd.get_relative_offset()
        assert offset is not None
        assert offset.z == 0.1  # 10cm = 0.1m
        assert offset.x == 0.0
        assert offset.y == 0.0

    def test_basic_validation_speed_limit(self):
        cmd = RobotCommand(
            command_type=CommandType.MANIPULATOR,
            movement_type=MovementType.RELATIVE,
            direction=Direction.FORWARD,
            distance=10.0,
            speed=3.0  # 제한 초과
        )
        is_valid, error = cmd.validate_basic()
        assert not is_valid
        assert "Speed" in error

    def test_to_dict_and_from_dict(self):
        original = create_manipulator_relative_command("up", 15.0, speed=1.2)
        data = original.to_dict()
        restored = RobotCommand.from_dict(data)
        assert restored.direction == original.direction
        assert restored.distance == original.distance
        assert restored.speed == original.speed


class TestCommandValidator:

    def test_workspace_bounds_check(self):
        from core.command_validator import CommandValidator

        config = {
            "bounds": {"min": [-1, -1, 0], "max": [1, 1, 1.5]},
            "safety": {"workspace_margin": 0.05},
            "velocity_limits": {"manipulator": {"max_linear": 0.5}}
        }
        validator = CommandValidator(config)

        # 유효한 명령
        cmd = create_manipulator_relative_command("forward", 10.0)
        current_pos = np.array([0.0, 0.5, 0.5])
        result = validator.validate(cmd, current_pos)
        assert result.is_valid

        # 작업 공간 이탈 명령
        cmd2 = create_manipulator_relative_command("forward", 200.0)  # 2m
        result2 = validator.validate(cmd2, current_pos)
        assert not result2.is_valid
        assert result2.error_code.value == "out_of_workspace"
```

### 6.2 통합 테스트 예시

```python
# tests/test_integration.py

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock
from core.control_manager import LLMRobotControlManager
from core.llm_client import LLMResult


@pytest.fixture
def mock_robot_controller():
    controller = Mock()
    controller.get_end_effector_position.return_value = np.array([0.0, 0.3, 0.5])
    controller.get_joint_positions.return_value = np.zeros(6)
    controller.move_to_position.return_value = True
    controller.gripper.get_state.return_value = Mock(value="open")
    return controller


@pytest.fixture
def manager(mock_robot_controller):
    config = {
        "llm": {"provider": "openai", "api_key": "test"},
        "workspace": {"bounds": {"min": [-1, -1, 0], "max": [1, 1, 1.5]}},
        "cache": {"max_size": 10}
    }
    mgr = LLMRobotControlManager(config)
    mgr.set_robot_controller(mock_robot_controller)
    return mgr


@pytest.mark.asyncio
async def test_full_command_flow(manager, mocker):
    # LLM 응답 모킹
    mock_llm_result = LLMResult(
        success=True,
        function_name="move_manipulator",
        arguments={
            "movement_type": "relative",
            "direction": "forward",
            "distance": 10.0,
            "speed": 1.0
        }
    )
    mocker.patch.object(
        manager._llm_client,
        'send_command',
        new_callable=AsyncMock,
        return_value=mock_llm_result
    )

    # 명령 실행
    result = await manager.process_command("앞으로 10cm 이동")

    assert result.success
    assert result.command_id is not None
```

---

## 7. 변경 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|----------|--------|
| 1.0 | 2025-12-14 | 초기 작성 | Claude Code |

---

**문서 끝**
