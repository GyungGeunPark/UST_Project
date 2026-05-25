# Isaac Sim LLM Robot Control - LLM Integration Implementation Guide

## 1. 문서 개요

### 1.1 목적
본 문서는 OpenAI GPT-4 및 Anthropic Claude API를 활용한 LLM 통합의 상세 구현 가이드를 제공합니다. Function Calling을 통한 구조화된 로봇 명령 생성에 중점을 둡니다.

### 1.2 범위
- OpenAI Function Calling 구현
- Anthropic Tool Use 구현
- 시스템 프롬프트 설계
- 명령 파싱 및 변환
- 에러 처리 및 재시도 로직
- 비용 최적화 전략

---

## 2. Function Calling 개요

### 2.1 동작 원리

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Function Calling Flow                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  User Input                                                          │
│  "앞으로 10cm 이동해줘"                                                │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                    System Prompt                             │    │
│  │  + Robot Context (workspace, current position)              │    │
│  │  + Available Functions (move_manipulator, move_base, etc.)  │    │
│  │  + Safety Instructions                                       │    │
│  └─────────────────────────────────────────────────────────────┘    │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                    LLM (GPT-4/Claude)                        │    │
│  │                                                              │    │
│  │  Input Analysis → Intent Detection → Parameter Extraction   │    │
│  │                                                              │    │
│  └─────────────────────────────────────────────────────────────┘    │
│       │                                                              │
│       ▼                                                              │
│  Function Call Response                                              │
│  {                                                                   │
│    "name": "move_manipulator",                                      │
│    "arguments": {                                                    │
│      "movement_type": "relative",                                   │
│      "direction": "forward",                                        │
│      "distance": 10.0,                                              │
│      "speed": 1.0                                                   │
│    }                                                                 │
│  }                                                                   │
│       │                                                              │
│       ▼                                                              │
│  RobotCommand 생성 → 검증 → 실행                                      │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 지원하는 LLM 모델

| Provider | Model | Function Calling | 권장 사용 |
|----------|-------|------------------|----------|
| OpenAI | gpt-4-turbo | ✅ 네이티브 | 프로덕션 |
| OpenAI | gpt-4o | ✅ 네이티브 | 프로덕션 |
| OpenAI | gpt-3.5-turbo | ✅ 네이티브 | 개발/테스트 |
| Anthropic | claude-3-opus | ✅ Tool Use | 복잡한 추론 |
| Anthropic | claude-3-sonnet | ✅ Tool Use | 프로덕션 |
| Anthropic | claude-3-haiku | ✅ Tool Use | 빠른 응답 |

---

## 3. Function Definition 상세

### 3.1 move_manipulator 함수

```python
MOVE_MANIPULATOR_FUNCTION = {
    "type": "function",
    "function": {
        "name": "move_manipulator",
        "description": """
매니퓰레이터의 엔드이펙터를 이동시킵니다.

사용 사례:
- "앞으로 10cm 이동" → relative, forward, 10
- "위로 5센치" → relative, up, 5
- "오른쪽으로 20cm" → relative, right, 20
- "X=0.5, Y=0.3, Z=0.4 위치로 이동" → absolute, position

좌표계:
- Z축: 전진(+) / 후진(-)
- X축: 우(+) / 좌(-)
- Y축: 상(+) / 하(-)

거리 단위: 센티미터(cm)
""",
        "parameters": {
            "type": "object",
            "properties": {
                "movement_type": {
                    "type": "string",
                    "enum": ["relative", "absolute"],
                    "description": "이동 유형. relative: 현재 위치 기준 상대 이동, absolute: 절대 좌표 이동"
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
                    "required": ["x", "y", "z"],
                    "description": "절대 위치 좌표 (movement_type이 absolute일 때 필수)"
                },
                "speed": {
                    "type": "number",
                    "minimum": 0.1,
                    "maximum": 2.0,
                    "default": 1.0,
                    "description": "이동 속도 배율. 1.0이 기본 속도, 2.0이 2배속"
                },
                "duration": {
                    "type": "number",
                    "minimum": 0.5,
                    "maximum": 10.0,
                    "default": 2.0,
                    "description": "이동 소요 시간 (초)"
                }
            },
            "required": ["movement_type"]
        }
    }
}
```

### 3.2 move_mobile_base 함수

```python
MOVE_MOBILE_BASE_FUNCTION = {
    "type": "function",
    "function": {
        "name": "move_mobile_base",
        "description": """
모바일 베이스(4륜 구동)를 이동시킵니다.

사용 사례:
- "앞으로 가" → linear_velocity: 0.3
- "뒤로 천천히" → linear_velocity: -0.2
- "왼쪽으로 회전" → angular_velocity: 0.5
- "오른쪽으로 돌아" → angular_velocity: -0.5
- "앞으로 가면서 왼쪽으로" → linear: 0.3, angular: 0.3

속도 단위:
- 선속도: m/s (양수=전진, 음수=후진)
- 각속도: rad/s (양수=좌회전, 음수=우회전)
""",
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
}
```

### 3.3 control_gripper 함수

```python
CONTROL_GRIPPER_FUNCTION = {
    "type": "function",
    "function": {
        "name": "control_gripper",
        "description": """
그리퍼를 열거나 닫습니다.

사용 사례:
- "그리퍼 열어" → action: "open"
- "집어" → action: "close"
- "놔" → action: "open"
- "잡아" → action: "close"
""",
        "parameters": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["open", "close"],
                    "description": "그리퍼 동작"
                }
            },
            "required": ["action"]
        }
    }
}
```

### 3.4 stop_robot 함수

```python
STOP_ROBOT_FUNCTION = {
    "type": "function",
    "function": {
        "name": "stop_robot",
        "description": """
로봇의 모든 움직임을 즉시 정지합니다.

사용 사례:
- "멈춰" → stop
- "정지" → stop
- "스톱" → stop
- "그만" → stop
""",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": []
        }
    }
}
```

### 3.5 전체 도구 정의

```python
# core/llm_tools.py

from typing import List, Dict

def get_robot_control_tools() -> List[Dict]:
    """로봇 제어용 Function Calling 도구 정의 반환"""
    return [
        MOVE_MANIPULATOR_FUNCTION,
        MOVE_MOBILE_BASE_FUNCTION,
        CONTROL_GRIPPER_FUNCTION,
        STOP_ROBOT_FUNCTION
    ]

def get_tools_for_anthropic() -> List[Dict]:
    """Anthropic API 형식으로 도구 변환"""
    tools = get_robot_control_tools()
    anthropic_tools = []

    for tool in tools:
        if tool["type"] == "function":
            func = tool["function"]
            anthropic_tools.append({
                "name": func["name"],
                "description": func["description"],
                "input_schema": func["parameters"]
            })

    return anthropic_tools
```

---

## 4. 시스템 프롬프트 설계

### 4.1 프롬프트 템플릿

```python
# core/prompts.py

from typing import Optional, Tuple, Dict
from string import Template

SYSTEM_PROMPT_TEMPLATE = Template("""당신은 산업용 로봇 제어 시스템입니다. 사용자의 자연어 명령을 정확하게 분석하여 적절한 로봇 제어 함수를 호출합니다.

## 로봇 구성
- **매니퓰레이터**: 6자유도 로봇 팔, IK 기반 엔드이펙터 위치 제어
- **모바일 베이스**: 4륜 차동 구동, 속도 제어
- **그리퍼**: 2-finger 평행 그리퍼, 열기/닫기

## 작업 공간 정보
작업 공간 경계 (안전 영역):
- X축 (좌우): ${min_x}m ~ ${max_x}m
- Y축 (상하): ${min_y}m ~ ${max_y}m
- Z축 (앞뒤): ${min_z}m ~ ${max_z}m

## 좌표계 규약
```
        Y (위)
        │
        │    Z (앞)
        │   /
        │  /
        │ /
        └──────── X (오른쪽)
```
- **전진/후진**: Z축 (+Z = 앞, -Z = 뒤)
- **좌우**: X축 (-X = 왼쪽, +X = 오른쪽)
- **상하**: Y축 (+Y = 위, -Y = 아래)

## 단위 체계
- **거리**: 센티미터(cm) 입력 → 내부적으로 미터(m) 변환
- **선속도**: m/s
- **각속도**: rad/s
- **시간**: 초(s)

## 현재 로봇 상태
${current_state}

## 안전 지침
1. **작업 공간 준수**: 모든 이동은 작업 공간 경계 내에서만 수행
2. **속도 제한**: 급격한 속도 변경 금지
3. **확인 요청**: 불명확한 명령은 확인 요청
4. **우선 순위**: 정지 명령은 최우선 처리

## 명령 해석 가이드라인
- "조금" = 5cm, "많이" = 20cm, 지정하지 않으면 = 10cm
- "빨리" = speed 1.5, "천천히" = speed 0.5
- "앞/전진" = forward, "뒤/후진" = backward
- "위/올려" = up, "아래/내려" = down
- "왼쪽" = left, "오른쪽" = right

## 응답 규칙
1. 반드시 제공된 함수 중 하나를 호출
2. 파라미터는 스키마 범위 내에서 지정
3. 모호한 경우 기본값 사용
4. 여러 동작이 필요하면 가장 중요한 하나만 선택
""")


class PromptBuilder:
    """시스템 프롬프트 빌더"""

    def __init__(self, workspace_config: Dict):
        self.workspace = workspace_config
        bounds = workspace_config.get("bounds", {})
        self.min_bounds = bounds.get("min", [-1, -1, 0])
        self.max_bounds = bounds.get("max", [1, 1, 1.5])

    def build(
        self,
        current_position: Optional[Tuple[float, float, float]] = None,
        current_state: Optional[str] = None
    ) -> str:
        """시스템 프롬프트 생성

        Args:
            current_position: 현재 엔드이펙터 위치 (x, y, z)
            current_state: 추가 상태 정보

        Returns:
            완성된 시스템 프롬프트
        """
        state_info = "정보 없음"
        if current_position:
            x, y, z = current_position
            state_info = f"""- 현재 엔드이펙터 위치: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m
- 상태: 대기 중"""

        if current_state:
            state_info += f"\n- 추가 정보: {current_state}"

        return SYSTEM_PROMPT_TEMPLATE.substitute(
            min_x=self.min_bounds[0],
            max_x=self.max_bounds[0],
            min_y=self.min_bounds[1],
            max_y=self.max_bounds[1],
            min_z=self.min_bounds[2],
            max_z=self.max_bounds[2],
            current_state=state_info
        )

    def build_with_context(
        self,
        current_position: Optional[Tuple[float, float, float]],
        recent_commands: List[str],
        error_history: List[str]
    ) -> str:
        """컨텍스트가 포함된 프롬프트 생성"""
        base_prompt = self.build(current_position)

        context_parts = []

        if recent_commands:
            context_parts.append("## 최근 명령 이력")
            for cmd in recent_commands[-5:]:
                context_parts.append(f"- {cmd}")

        if error_history:
            context_parts.append("\n## 최근 오류")
            for err in error_history[-3:]:
                context_parts.append(f"- {err}")

        if context_parts:
            base_prompt += "\n\n" + "\n".join(context_parts)

        return base_prompt
```

### 4.2 프롬프트 최적화 팁

```python
# core/prompt_optimization.py

"""
프롬프트 최적화 전략
"""

# 1. 토큰 효율성을 위한 간결한 프롬프트 (개발/테스트용)
MINIMAL_PROMPT = """Robot controller. Commands: move_manipulator, move_mobile_base, control_gripper, stop_robot.
Workspace: X[-1,1] Y[-1,1.5] Z[0,1.5] meters.
Coordinate: +Z=forward, +X=right, +Y=up.
Units: distance in cm, velocity in m/s."""

# 2. Few-shot 예제 포함 프롬프트
FEW_SHOT_EXAMPLES = """
예제 명령과 응답:

User: "앞으로 10센치"
→ move_manipulator(movement_type="relative", direction="forward", distance=10)

User: "천천히 뒤로 가"
→ move_mobile_base(linear_velocity=-0.2, duration=2.0)

User: "그리퍼 열어"
→ control_gripper(action="open")

User: "멈춰"
→ stop_robot()
"""

# 3. 에러 방지 지침
ERROR_PREVENTION_INSTRUCTIONS = """
주의사항:
- distance는 반드시 숫자로 (예: "10", not "10cm")
- direction은 정확히 6개 중 하나: forward, backward, left, right, up, down
- speed는 0.1~2.0 범위
- 동시 동작 요청 시 매니퓰레이터 우선
"""


def select_prompt_strategy(
    complexity: str = "standard",
    include_examples: bool = False
) -> str:
    """복잡도에 따른 프롬프트 전략 선택

    Args:
        complexity: "minimal" | "standard" | "detailed"
        include_examples: Few-shot 예제 포함 여부
    """
    if complexity == "minimal":
        return MINIMAL_PROMPT
    elif complexity == "detailed":
        prompt = SYSTEM_PROMPT_TEMPLATE.safe_substitute(
            min_x=-1, max_x=1, min_y=-1, max_y=1.5,
            min_z=0, max_z=1.5, current_state="대기 중"
        )
        if include_examples:
            prompt += "\n" + FEW_SHOT_EXAMPLES
        prompt += "\n" + ERROR_PREVENTION_INSTRUCTIONS
        return prompt
    else:  # standard
        return SYSTEM_PROMPT_TEMPLATE.safe_substitute(
            min_x=-1, max_x=1, min_y=-1, max_y=1.5,
            min_z=0, max_z=1.5, current_state="대기 중"
        )
```

---

## 5. 응답 파싱 및 변환

### 5.1 OpenAI 응답 파서

```python
# core/response_parser.py

import json
from typing import Dict, Optional, Any
from dataclasses import dataclass
from enum import Enum
import logging

from .robot_command import (
    RobotCommand, CommandType, MovementType, Direction,
    Position3D, GripperAction
)

logger = logging.getLogger(__name__)


class ParseErrorCode(Enum):
    """파싱 에러 코드"""
    NONE = "none"
    NO_FUNCTION_CALL = "no_function_call"
    INVALID_JSON = "invalid_json"
    MISSING_REQUIRED_FIELD = "missing_required_field"
    INVALID_FIELD_VALUE = "invalid_field_value"
    UNKNOWN_FUNCTION = "unknown_function"


@dataclass
class ParseResult:
    """파싱 결과"""
    success: bool
    command: Optional[RobotCommand] = None
    error_code: ParseErrorCode = ParseErrorCode.NONE
    error_message: Optional[str] = None
    raw_response: Optional[str] = None


class OpenAIResponseParser:
    """OpenAI API 응답 파서"""

    @staticmethod
    def parse(response: Any) -> ParseResult:
        """OpenAI ChatCompletion 응답 파싱

        Args:
            response: OpenAI API 응답 객체

        Returns:
            ParseResult
        """
        try:
            message = response.choices[0].message

            # Function call 확인
            if not message.tool_calls:
                return ParseResult(
                    success=False,
                    error_code=ParseErrorCode.NO_FUNCTION_CALL,
                    error_message="No function call in response",
                    raw_response=message.content
                )

            tool_call = message.tool_calls[0]
            function_name = tool_call.function.name

            # Arguments 파싱
            try:
                arguments = json.loads(tool_call.function.arguments)
            except json.JSONDecodeError as e:
                return ParseResult(
                    success=False,
                    error_code=ParseErrorCode.INVALID_JSON,
                    error_message=f"Failed to parse arguments: {e}",
                    raw_response=tool_call.function.arguments
                )

            # 함수별 명령 생성
            return OpenAIResponseParser._create_command(function_name, arguments)

        except Exception as e:
            logger.error(f"Parse error: {e}")
            return ParseResult(
                success=False,
                error_code=ParseErrorCode.INVALID_JSON,
                error_message=str(e)
            )

    @staticmethod
    def _create_command(function_name: str, arguments: Dict) -> ParseResult:
        """함수명과 인자로 RobotCommand 생성"""

        try:
            if function_name == "move_manipulator":
                return OpenAIResponseParser._parse_move_manipulator(arguments)
            elif function_name == "move_mobile_base":
                return OpenAIResponseParser._parse_move_base(arguments)
            elif function_name == "control_gripper":
                return OpenAIResponseParser._parse_gripper(arguments)
            elif function_name == "stop_robot":
                return OpenAIResponseParser._parse_stop()
            else:
                return ParseResult(
                    success=False,
                    error_code=ParseErrorCode.UNKNOWN_FUNCTION,
                    error_message=f"Unknown function: {function_name}"
                )
        except Exception as e:
            return ParseResult(
                success=False,
                error_code=ParseErrorCode.INVALID_FIELD_VALUE,
                error_message=str(e)
            )

    @staticmethod
    def _parse_move_manipulator(args: Dict) -> ParseResult:
        """move_manipulator 함수 파싱"""

        # 필수 필드 확인
        movement_type_str = args.get("movement_type")
        if not movement_type_str:
            return ParseResult(
                success=False,
                error_code=ParseErrorCode.MISSING_REQUIRED_FIELD,
                error_message="movement_type is required"
            )

        movement_type = MovementType(movement_type_str)
        command = RobotCommand(command_type=CommandType.MANIPULATOR)
        command.movement_type = movement_type
        command.speed = args.get("speed", 1.0)
        command.duration = args.get("duration", 2.0)

        if movement_type == MovementType.RELATIVE:
            # 상대 이동: direction과 distance 필요
            direction_str = args.get("direction")
            if not direction_str:
                return ParseResult(
                    success=False,
                    error_code=ParseErrorCode.MISSING_REQUIRED_FIELD,
                    error_message="direction is required for relative movement"
                )

            command.direction = Direction(direction_str)
            command.distance = args.get("distance", 10.0)

        else:  # ABSOLUTE
            # 절대 이동: position 필요
            position = args.get("position")
            if not position:
                return ParseResult(
                    success=False,
                    error_code=ParseErrorCode.MISSING_REQUIRED_FIELD,
                    error_message="position is required for absolute movement"
                )

            command.absolute_position = Position3D(
                x=position.get("x", 0),
                y=position.get("y", 0),
                z=position.get("z", 0)
            )

        return ParseResult(success=True, command=command)

    @staticmethod
    def _parse_move_base(args: Dict) -> ParseResult:
        """move_mobile_base 함수 파싱"""

        linear_velocity = args.get("linear_velocity")
        if linear_velocity is None:
            return ParseResult(
                success=False,
                error_code=ParseErrorCode.MISSING_REQUIRED_FIELD,
                error_message="linear_velocity is required"
            )

        command = RobotCommand(
            command_type=CommandType.MOBILE_BASE,
            linear_velocity=float(linear_velocity),
            angular_velocity=float(args.get("angular_velocity", 0.0)),
            duration=float(args.get("duration", 2.0))
        )

        return ParseResult(success=True, command=command)

    @staticmethod
    def _parse_gripper(args: Dict) -> ParseResult:
        """control_gripper 함수 파싱"""

        action = args.get("action")
        if not action:
            return ParseResult(
                success=False,
                error_code=ParseErrorCode.MISSING_REQUIRED_FIELD,
                error_message="action is required"
            )

        command = RobotCommand(
            command_type=CommandType.GRIPPER,
            gripper_action=GripperAction(action)
        )

        return ParseResult(success=True, command=command)

    @staticmethod
    def _parse_stop() -> ParseResult:
        """stop_robot 함수 파싱"""
        # 정지 명령은 특별한 타입으로 처리
        command = RobotCommand(
            command_type=CommandType.MANIPULATOR,  # 임시
            # 실제로는 emergency_stop 호출
        )
        command._is_stop_command = True  # 특별 플래그

        return ParseResult(success=True, command=command)


class AnthropicResponseParser:
    """Anthropic API 응답 파서"""

    @staticmethod
    def parse(response: Any) -> ParseResult:
        """Anthropic Messages 응답 파싱"""
        try:
            # Tool use block 찾기
            for block in response.content:
                if block.type == "tool_use":
                    function_name = block.name
                    arguments = block.input

                    return OpenAIResponseParser._create_command(
                        function_name, arguments
                    )

            # Tool use가 없는 경우
            text_content = ""
            for block in response.content:
                if block.type == "text":
                    text_content += block.text

            return ParseResult(
                success=False,
                error_code=ParseErrorCode.NO_FUNCTION_CALL,
                error_message="No tool use in response",
                raw_response=text_content
            )

        except Exception as e:
            logger.error(f"Anthropic parse error: {e}")
            return ParseResult(
                success=False,
                error_code=ParseErrorCode.INVALID_JSON,
                error_message=str(e)
            )
```

### 5.2 명령어 추출 유틸리티

```python
# core/command_extraction.py

import re
from typing import Optional, Tuple, List
from dataclasses import dataclass


@dataclass
class ExtractedIntent:
    """추출된 의도"""
    action: str  # move, grip, stop
    target: Optional[str] = None  # manipulator, base, gripper
    direction: Optional[str] = None
    distance: Optional[float] = None
    speed: Optional[str] = None  # fast, slow, normal


class CommandExtractor:
    """자연어에서 명령 의도 추출 (전처리용)"""

    # 방향 패턴
    DIRECTION_PATTERNS = {
        "forward": ["앞", "전진", "앞으로", "forward"],
        "backward": ["뒤", "후진", "뒤로", "backward", "back"],
        "left": ["왼", "좌", "왼쪽", "left"],
        "right": ["오른", "우", "오른쪽", "right"],
        "up": ["위", "올", "위로", "up"],
        "down": ["아래", "내", "내려", "down"]
    }

    # 거리 패턴
    DISTANCE_PATTERN = re.compile(r'(\d+(?:\.\d+)?)\s*(?:cm|센티|센치|센티미터)?')

    # 속도 패턴
    SPEED_PATTERNS = {
        "fast": ["빨리", "빠르게", "fast", "quickly"],
        "slow": ["천천히", "느리게", "slow", "slowly"]
    }

    # 정지 패턴
    STOP_PATTERNS = ["멈춰", "정지", "스톱", "stop", "그만"]

    # 그리퍼 패턴
    GRIPPER_OPEN_PATTERNS = ["열어", "open", "놔", "펴"]
    GRIPPER_CLOSE_PATTERNS = ["닫아", "close", "잡아", "집어"]

    @classmethod
    def extract(cls, text: str) -> ExtractedIntent:
        """텍스트에서 의도 추출"""
        text_lower = text.lower()

        # 정지 명령 확인
        for pattern in cls.STOP_PATTERNS:
            if pattern in text_lower:
                return ExtractedIntent(action="stop")

        # 그리퍼 명령 확인
        for pattern in cls.GRIPPER_OPEN_PATTERNS:
            if pattern in text_lower:
                return ExtractedIntent(action="grip", target="gripper", direction="open")

        for pattern in cls.GRIPPER_CLOSE_PATTERNS:
            if pattern in text_lower:
                return ExtractedIntent(action="grip", target="gripper", direction="close")

        # 이동 명령 분석
        intent = ExtractedIntent(action="move")

        # 방향 추출
        for direction, patterns in cls.DIRECTION_PATTERNS.items():
            for pattern in patterns:
                if pattern in text_lower:
                    intent.direction = direction
                    break
            if intent.direction:
                break

        # 거리 추출
        distance_match = cls.DISTANCE_PATTERN.search(text)
        if distance_match:
            intent.distance = float(distance_match.group(1))

        # 속도 추출
        for speed, patterns in cls.SPEED_PATTERNS.items():
            for pattern in patterns:
                if pattern in text_lower:
                    intent.speed = speed
                    break
            if intent.speed:
                break

        # 대상 추론
        if "바퀴" in text_lower or "베이스" in text_lower or "주행" in text_lower:
            intent.target = "base"
        else:
            intent.target = "manipulator"

        return intent
```

---

## 6. 에러 처리 및 재시도

### 6.1 재시도 로직

```python
# core/retry_handler.py

import asyncio
import random
from typing import TypeVar, Callable, Any
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)

T = TypeVar('T')


class RetryStrategy(Enum):
    """재시도 전략"""
    FIXED = "fixed"              # 고정 간격
    EXPONENTIAL = "exponential"  # 지수 백오프
    JITTER = "jitter"            # 랜덤 지터


@dataclass
class RetryConfig:
    """재시도 설정"""
    max_retries: int = 3
    initial_delay: float = 1.0
    max_delay: float = 30.0
    strategy: RetryStrategy = RetryStrategy.EXPONENTIAL
    jitter_factor: float = 0.1


class RetryHandler:
    """재시도 핸들러"""

    def __init__(self, config: RetryConfig):
        self.config = config

    def calculate_delay(self, attempt: int) -> float:
        """재시도 지연 시간 계산"""
        if self.config.strategy == RetryStrategy.FIXED:
            delay = self.config.initial_delay

        elif self.config.strategy == RetryStrategy.EXPONENTIAL:
            delay = self.config.initial_delay * (2 ** attempt)

        else:  # JITTER
            base_delay = self.config.initial_delay * (2 ** attempt)
            jitter = base_delay * self.config.jitter_factor * random.random()
            delay = base_delay + jitter

        return min(delay, self.config.max_delay)

    async def execute_with_retry(
        self,
        func: Callable[..., T],
        *args,
        **kwargs
    ) -> T:
        """재시도 로직으로 함수 실행"""
        last_exception = None

        for attempt in range(self.config.max_retries):
            try:
                if asyncio.iscoroutinefunction(func):
                    return await func(*args, **kwargs)
                else:
                    return func(*args, **kwargs)

            except RateLimitError as e:
                last_exception = e
                delay = self.calculate_delay(attempt)
                logger.warning(f"Rate limit hit, retrying in {delay:.1f}s (attempt {attempt + 1})")
                await asyncio.sleep(delay)

            except TimeoutError as e:
                last_exception = e
                delay = self.calculate_delay(attempt)
                logger.warning(f"Timeout, retrying in {delay:.1f}s (attempt {attempt + 1})")
                await asyncio.sleep(delay)

            except APIError as e:
                if e.is_retryable:
                    last_exception = e
                    delay = self.calculate_delay(attempt)
                    logger.warning(f"Retryable API error, retrying in {delay:.1f}s")
                    await asyncio.sleep(delay)
                else:
                    raise

        raise last_exception or Exception("Max retries exceeded")


class RateLimitError(Exception):
    """Rate limit 에러"""
    pass


class APIError(Exception):
    """API 에러"""
    def __init__(self, message: str, is_retryable: bool = False):
        super().__init__(message)
        self.is_retryable = is_retryable
```

### 6.2 폴백 처리

```python
# core/fallback_handler.py

from typing import Optional, List, Dict, Any
from dataclasses import dataclass
import logging

from .robot_command import RobotCommand, CommandType

logger = logging.getLogger(__name__)


@dataclass
class FallbackResult:
    """폴백 결과"""
    success: bool
    command: Optional[RobotCommand] = None
    fallback_used: str = ""
    message: str = ""


class FallbackHandler:
    """LLM 실패 시 폴백 처리"""

    # 간단한 패턴 매칭 규칙
    SIMPLE_PATTERNS = {
        "앞으로": ("move_manipulator", {"movement_type": "relative", "direction": "forward", "distance": 10}),
        "뒤로": ("move_manipulator", {"movement_type": "relative", "direction": "backward", "distance": 10}),
        "위로": ("move_manipulator", {"movement_type": "relative", "direction": "up", "distance": 10}),
        "아래로": ("move_manipulator", {"movement_type": "relative", "direction": "down", "distance": 10}),
        "왼쪽": ("move_manipulator", {"movement_type": "relative", "direction": "left", "distance": 10}),
        "오른쪽": ("move_manipulator", {"movement_type": "relative", "direction": "right", "distance": 10}),
        "열어": ("control_gripper", {"action": "open"}),
        "닫아": ("control_gripper", {"action": "close"}),
        "멈춰": ("stop_robot", {}),
        "정지": ("stop_robot", {}),
    }

    @classmethod
    def try_fallback(cls, user_input: str, error_message: str) -> FallbackResult:
        """폴백 시도

        Args:
            user_input: 원본 사용자 입력
            error_message: LLM 에러 메시지

        Returns:
            FallbackResult
        """
        user_input_lower = user_input.lower().strip()

        # 패턴 매칭 시도
        for pattern, (func_name, args) in cls.SIMPLE_PATTERNS.items():
            if pattern in user_input_lower:
                command = cls._create_command_from_pattern(func_name, args)
                if command:
                    return FallbackResult(
                        success=True,
                        command=command,
                        fallback_used="pattern_matching",
                        message=f"Used pattern matching for '{pattern}'"
                    )

        # 폴백 실패
        return FallbackResult(
            success=False,
            message=f"No fallback available. Original error: {error_message}"
        )

    @classmethod
    def _create_command_from_pattern(cls, func_name: str, args: Dict) -> Optional[RobotCommand]:
        """패턴에서 명령 생성"""
        try:
            from .response_parser import OpenAIResponseParser
            result = OpenAIResponseParser._create_command(func_name, args)
            return result.command if result.success else None
        except Exception as e:
            logger.error(f"Fallback command creation failed: {e}")
            return None
```

---

## 7. 비용 최적화

### 7.1 토큰 사용량 추적

```python
# core/token_tracker.py

import tiktoken
from typing import Dict, List
from dataclasses import dataclass, field
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)


@dataclass
class TokenUsage:
    """토큰 사용량"""
    prompt_tokens: int = 0
    completion_tokens: int = 0
    total_tokens: int = 0
    estimated_cost: float = 0.0


@dataclass
class UsageStats:
    """사용 통계"""
    total_requests: int = 0
    total_prompt_tokens: int = 0
    total_completion_tokens: int = 0
    total_cost: float = 0.0
    requests_per_minute: float = 0.0
    average_latency: float = 0.0


class TokenTracker:
    """토큰 사용량 추적기"""

    # 모델별 가격 (USD per 1K tokens)
    PRICING = {
        "gpt-4-turbo": {"input": 0.01, "output": 0.03},
        "gpt-4o": {"input": 0.005, "output": 0.015},
        "gpt-3.5-turbo": {"input": 0.0005, "output": 0.0015},
        "claude-3-opus": {"input": 0.015, "output": 0.075},
        "claude-3-sonnet": {"input": 0.003, "output": 0.015},
        "claude-3-haiku": {"input": 0.00025, "output": 0.00125},
    }

    def __init__(self, model: str = "gpt-4-turbo"):
        self.model = model
        self._usage_history: List[Dict] = []
        self._start_time = datetime.now()

        # 토크나이저 초기화
        try:
            self._encoding = tiktoken.encoding_for_model(model)
        except:
            self._encoding = tiktoken.get_encoding("cl100k_base")

    def count_tokens(self, text: str) -> int:
        """텍스트의 토큰 수 계산"""
        return len(self._encoding.encode(text))

    def estimate_cost(self, prompt_tokens: int, completion_tokens: int) -> float:
        """비용 추정"""
        pricing = self.PRICING.get(self.model, {"input": 0.01, "output": 0.03})
        input_cost = (prompt_tokens / 1000) * pricing["input"]
        output_cost = (completion_tokens / 1000) * pricing["output"]
        return input_cost + output_cost

    def record_usage(self, prompt_tokens: int, completion_tokens: int, latency: float):
        """사용량 기록"""
        cost = self.estimate_cost(prompt_tokens, completion_tokens)

        self._usage_history.append({
            "timestamp": datetime.now(),
            "prompt_tokens": prompt_tokens,
            "completion_tokens": completion_tokens,
            "cost": cost,
            "latency": latency
        })

        logger.debug(f"Token usage: {prompt_tokens}+{completion_tokens}={prompt_tokens+completion_tokens}, "
                    f"cost: ${cost:.4f}")

    def get_stats(self) -> UsageStats:
        """통계 조회"""
        if not self._usage_history:
            return UsageStats()

        total_prompt = sum(u["prompt_tokens"] for u in self._usage_history)
        total_completion = sum(u["completion_tokens"] for u in self._usage_history)
        total_cost = sum(u["cost"] for u in self._usage_history)
        avg_latency = sum(u["latency"] for u in self._usage_history) / len(self._usage_history)

        # 분당 요청 수 계산
        elapsed = (datetime.now() - self._start_time).total_seconds() / 60
        rpm = len(self._usage_history) / elapsed if elapsed > 0 else 0

        return UsageStats(
            total_requests=len(self._usage_history),
            total_prompt_tokens=total_prompt,
            total_completion_tokens=total_completion,
            total_cost=total_cost,
            requests_per_minute=rpm,
            average_latency=avg_latency
        )

    def get_recent_usage(self, minutes: int = 60) -> List[Dict]:
        """최근 사용량 조회"""
        cutoff = datetime.now() - timedelta(minutes=minutes)
        return [u for u in self._usage_history if u["timestamp"] > cutoff]
```

### 7.2 캐싱 전략

```python
# core/command_cache.py

import hashlib
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass
from collections import OrderedDict
import logging

logger = logging.getLogger(__name__)


@dataclass
class CacheEntry:
    """캐시 항목"""
    command: Any  # RobotCommand
    timestamp: float
    hit_count: int = 0


class LRUCache:
    """LRU 캐시"""

    def __init__(self, max_size: int = 100):
        self.max_size = max_size
        self._cache: OrderedDict[str, CacheEntry] = OrderedDict()

    def _make_key(self, text: str) -> str:
        """캐시 키 생성"""
        normalized = text.lower().strip()
        return hashlib.md5(normalized.encode()).hexdigest()

    def get(self, text: str) -> Optional[Any]:
        """캐시에서 조회"""
        key = self._make_key(text)

        if key in self._cache:
            entry = self._cache[key]
            entry.hit_count += 1

            # LRU 업데이트
            self._cache.move_to_end(key)

            logger.debug(f"Cache hit for '{text[:20]}...' (hits: {entry.hit_count})")
            return entry.command

        return None

    def put(self, text: str, command: Any):
        """캐시에 저장"""
        key = self._make_key(text)

        # 크기 제한 확인
        if len(self._cache) >= self.max_size:
            oldest_key = next(iter(self._cache))
            del self._cache[oldest_key]

        self._cache[key] = CacheEntry(
            command=command,
            timestamp=time.time()
        )

        logger.debug(f"Cached command for '{text[:20]}...'")

    def clear(self):
        """캐시 비우기"""
        self._cache.clear()

    def get_stats(self) -> Dict:
        """캐시 통계"""
        total_hits = sum(e.hit_count for e in self._cache.values())
        return {
            "size": len(self._cache),
            "max_size": self.max_size,
            "total_hits": total_hits,
            "fill_rate": len(self._cache) / self.max_size
        }


class SemanticCache:
    """의미 기반 캐시 (유사 명령 매칭)"""

    def __init__(self, similarity_threshold: float = 0.9):
        self.threshold = similarity_threshold
        self._cache: Dict[str, CacheEntry] = {}

    def _normalize(self, text: str) -> str:
        """텍스트 정규화"""
        import re

        text = text.lower().strip()

        # 숫자 정규화 (특정 숫자는 보존)
        text = re.sub(r'\b(\d+)\s*(cm|센티|센치)\b', r'\1cm', text)

        # 불필요한 문자 제거
        text = re.sub(r'[^\w\s]', '', text)

        return text

    def _similarity(self, text1: str, text2: str) -> float:
        """텍스트 유사도 계산 (간단한 Jaccard)"""
        words1 = set(text1.split())
        words2 = set(text2.split())

        intersection = len(words1 & words2)
        union = len(words1 | words2)

        return intersection / union if union > 0 else 0

    def get(self, text: str) -> Optional[Any]:
        """유사한 명령 검색"""
        normalized = self._normalize(text)

        best_match = None
        best_similarity = 0

        for cached_text, entry in self._cache.items():
            sim = self._similarity(normalized, cached_text)
            if sim > best_similarity:
                best_similarity = sim
                best_match = entry

        if best_similarity >= self.threshold and best_match:
            best_match.hit_count += 1
            logger.debug(f"Semantic cache hit (similarity: {best_similarity:.2f})")
            return best_match.command

        return None

    def put(self, text: str, command: Any):
        """캐시에 저장"""
        normalized = self._normalize(text)
        self._cache[normalized] = CacheEntry(
            command=command,
            timestamp=time.time()
        )
```

---

## 8. 테스트 가이드

### 8.1 LLM 클라이언트 테스트

```python
# tests/test_llm_client.py

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch

from core.llm_client import OpenAIClient, LLMConfig, LLMResult
from core.llm_tools import get_robot_control_tools


class TestOpenAIClient:

    @pytest.fixture
    def config(self):
        return LLMConfig(
            provider="openai",
            api_key="test-key",
            model="gpt-4-turbo",
            temperature=0.1
        )

    @pytest.fixture
    def client(self, config):
        with patch('core.llm_client.AsyncOpenAI'):
            return OpenAIClient(config)

    @pytest.mark.asyncio
    async def test_send_command_success(self, client):
        # Mock response
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message.tool_calls = [Mock()]
        mock_response.choices[0].message.tool_calls[0].function.name = "move_manipulator"
        mock_response.choices[0].message.tool_calls[0].function.arguments = '{"movement_type": "relative", "direction": "forward", "distance": 10}'

        client.client.chat.completions.create = AsyncMock(return_value=mock_response)

        result = await client.send_command("앞으로 10cm", "test prompt")

        assert result.success
        assert result.function_name == "move_manipulator"
        assert result.arguments["direction"] == "forward"

    @pytest.mark.asyncio
    async def test_send_command_no_function_call(self, client):
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message.tool_calls = None
        mock_response.choices[0].message.content = "I cannot understand"

        client.client.chat.completions.create = AsyncMock(return_value=mock_response)

        result = await client.send_command("random text", "test prompt")

        assert not result.success
        assert "No function call" in result.error


class TestResponseParser:

    def test_parse_move_manipulator_relative(self):
        from core.response_parser import OpenAIResponseParser

        args = {
            "movement_type": "relative",
            "direction": "forward",
            "distance": 15.0,
            "speed": 1.5
        }

        result = OpenAIResponseParser._create_command("move_manipulator", args)

        assert result.success
        assert result.command.direction.value == "forward"
        assert result.command.distance == 15.0
        assert result.command.speed == 1.5

    def test_parse_move_base(self):
        from core.response_parser import OpenAIResponseParser

        args = {
            "linear_velocity": 0.5,
            "angular_velocity": 0.2,
            "duration": 3.0
        }

        result = OpenAIResponseParser._create_command("move_mobile_base", args)

        assert result.success
        assert result.command.linear_velocity == 0.5
        assert result.command.angular_velocity == 0.2

    def test_parse_missing_required_field(self):
        from core.response_parser import OpenAIResponseParser

        args = {"speed": 1.0}  # movement_type 누락

        result = OpenAIResponseParser._create_command("move_manipulator", args)

        assert not result.success
        assert "required" in result.error_message.lower()
```

---

## 9. 변경 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|----------|--------|
| 1.0 | 2025-12-14 | 초기 작성 | Claude Code |

---

**문서 끝**
