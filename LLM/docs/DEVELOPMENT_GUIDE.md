# Isaac Sim LLM Robot Control - 개발 가이드

시스템을 확장하거나 수정하려는 개발자를 위한 가이드입니다.

---

## 목차

1. [프로젝트 구조](#1-프로젝트-구조)
2. [핵심 컴포넌트](#2-핵심-컴포넌트)
3. [데이터 흐름](#3-데이터-흐름)
4. [확장 방법](#4-확장-방법)
5. [테스트](#5-테스트)
6. [디버깅](#6-디버깅)

---

## 1. 프로젝트 구조

```
LLM/
├── config/                 # 설정 파일 (YAML)
│   ├── robot_config.yaml      # 로봇 하드웨어 설정
│   ├── workspace_config.yaml  # 작업 공간 및 안전 설정
│   ├── llm_config.yaml        # LLM API 설정
│   └── server_config.yaml     # 웹 서버 설정
│
├── core/                   # 핵심 비즈니스 로직
│   ├── __init__.py
│   ├── robot_command.py       # 명령 데이터 구조
│   ├── llm_client.py          # LLM API 클라이언트
│   ├── llm_tools.py           # Function Calling 정의
│   ├── prompts.py             # 시스템 프롬프트
│   ├── response_parser.py     # LLM 응답 파싱
│   ├── command_validator.py   # 명령 검증
│   └── control_manager.py     # 메인 오케스트레이터
│
├── isaac_interface/        # Isaac Sim 인터페이스
│   ├── __init__.py
│   ├── robot_controller.py    # 통합 로봇 컨트롤러
│   ├── mobile_base.py         # 모바일 베이스 제어
│   ├── manipulator.py         # 매니퓰레이터 제어
│   ├── gripper.py             # 그리퍼 제어
│   └── ik_solver.py           # IK 솔버 래퍼
│
├── safety/                 # 안전 시스템
│   ├── __init__.py
│   ├── emergency_stop.py      # 비상 정지 시스템
│   ├── workspace_validator.py # 작업 공간 검증
│   └── collision_checker.py   # 충돌 감지
│
├── web/                    # 웹 서버
│   ├── __init__.py
│   ├── server.py              # FastAPI 애플리케이션
│   ├── api/
│   │   └── routes.py          # API 스키마
│   └── static/                # 프론트엔드
│       ├── index.html
│       ├── css/style.css
│       └── js/main.js
│
├── utils/                  # 유틸리티
│   ├── __init__.py
│   ├── config_loader.py       # 설정 로더
│   └── logging_config.py      # 로깅 설정
│
├── scripts/                # 실행 스크립트
│   ├── run_standalone.py      # 독립 실행
│   └── run_with_isaac.py      # Isaac Sim 연동
│
├── tests/                  # 테스트
│   └── test_core.py
│
├── docs/                   # 문서
├── requirements.txt
└── README.md
```

---

## 2. 핵심 컴포넌트

### 2.1 ControlManager

메인 오케스트레이터로 모든 컴포넌트를 조율합니다.

```python
# core/control_manager.py

class ControlManager:
    """명령 처리 흐름:
    1. 자연어 명령 수신
    2. LLM으로 Function Call 생성
    3. 응답 파싱
    4. 명령 검증
    5. 로봇 제어 실행
    """

    async def process_command(self, command_text: str) -> CommandResult:
        # 1. 시스템 프롬프트 생성
        system_prompt = get_system_prompt(self.config, ...)

        # 2. LLM 호출
        llm_response = await self._llm_client.process_command(
            command_text, system_prompt, tools
        )

        # 3. 응답 파싱
        parse_result = self._parser.parse(llm_response, command_text)

        # 4. 명령 검증
        validation = self._validator.validate(command, current_position)

        # 5. 실행
        return await self._execute_command(command)
```

### 2.2 LLMClient

LLM API 호출을 담당합니다.

```python
# core/llm_client.py

class LLMClient:
    """지원 프로바이더:
    - OpenAI (GPT-4, GPT-4o)
    - Anthropic (Claude)
    """

    async def process_command(
        self,
        command: str,
        system_prompt: str,
        tools: List[Dict]
    ) -> Dict[str, Any]:
        # 캐시 확인
        if cached := self._cache.get(command):
            return cached

        # API 호출
        if self.provider == "openai":
            result = await self._call_openai(command, system_prompt, tools)
        else:
            result = await self._call_anthropic(command, system_prompt, tools)

        # 캐시 저장
        self._cache.set(command, result)
        return result
```

### 2.3 RobotCommand

명령 데이터 구조입니다.

```python
# core/robot_command.py

@dataclass
class RobotCommand:
    command_id: str
    original_text: str
    command_type: CommandType
    status: CommandStatus
    function_name: str
    parameters: Dict[str, Any]
    created_at: float
    started_at: Optional[float]
    completed_at: Optional[float]
    error_message: Optional[str]

class CommandType(Enum):
    MOVE_MANIPULATOR = "move_manipulator"
    MOVE_MOBILE_BASE = "move_mobile_base"
    CONTROL_GRIPPER = "control_gripper"
    STOP_ROBOT = "stop_robot"
```

### 2.4 RobotController

로봇 하드웨어 인터페이스입니다.

```python
# isaac_interface/robot_controller.py

class RobotController:
    """서브시스템 조율:
    - MobileBaseController
    - ManipulatorController
    - GripperController
    - IKSolverWrapper
    """

    async def move_relative(
        self,
        direction: str,
        distance: float,
        speed: float = 1.0
    ) -> bool:
        # 목표 위치 계산
        target = self._calculate_target(direction, distance)

        # IK 풀이
        joint_targets = self._ik_solver.solve(target)

        # 실행
        return await self._manipulator.move_to_position(target, speed)
```

---

## 3. 데이터 흐름

### 3.1 명령 처리 흐름

```
사용자 입력: "앞으로 10cm"
        │
        ▼
┌───────────────────┐
│    Web Server     │  POST /api/command
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│  ControlManager   │
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│    LLMClient      │  → OpenAI/Anthropic API
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│  ResponseParser   │  Function Call 추출
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ CommandValidator  │  안전 검증
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│  RobotController  │  로봇 제어
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│   Isaac Sim       │  물리 시뮬레이션
└───────────────────┘
```

### 3.2 안전 검증 흐름

```
명령 수신
    │
    ▼
┌───────────────────┐
│ Layer 1: LLM      │  시스템 프롬프트 안전 지침
│ Safety Prompt     │
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ Layer 2: Schema   │  JSON 스키마 범위 검사
│ Validation        │  (거리: 0.1~100cm)
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ Layer 3: Logic    │  작업공간/충돌/속도 검사
│ Validation        │
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ Layer 4: User     │  위험 동작 확인 (선택적)
│ Confirmation      │
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ Layer 5: Runtime  │  실시간 모니터링
│ Monitor           │  이상 시 E-Stop
└───────────────────┘
```

---

## 4. 확장 방법

### 4.1 새 로봇 명령 추가

**Step 1: Tool 정의 추가**

```python
# core/llm_tools.py

def get_tool_definitions():
    return [
        # ... 기존 도구
        {
            "type": "function",
            "function": {
                "name": "rotate_wrist",
                "description": "Rotate the wrist joint",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "angle": {
                            "type": "number",
                            "minimum": -180,
                            "maximum": 180,
                            "description": "Rotation angle in degrees"
                        },
                        "speed": {
                            "type": "number",
                            "minimum": 0.1,
                            "maximum": 2.0,
                            "default": 1.0
                        }
                    },
                    "required": ["angle"]
                }
            }
        }
    ]
```

**Step 2: CommandType 추가**

```python
# core/robot_command.py

class CommandType(Enum):
    # ... 기존 타입
    ROTATE_WRIST = "rotate_wrist"
```

**Step 3: 파서 매핑 추가**

```python
# core/response_parser.py

class LLMResponseParser:
    FUNCTION_MAP = {
        # ... 기존 매핑
        "rotate_wrist": CommandType.ROTATE_WRIST,
    }
```

**Step 4: 실행 핸들러 추가**

```python
# core/control_manager.py

async def _execute_command(self, command: RobotCommand):
    # ... 기존 코드
    elif command.command_type == CommandType.ROTATE_WRIST:
        result = await self._execute_rotate_wrist(command)
    # ...

async def _execute_rotate_wrist(self, command: RobotCommand):
    params = command.parameters
    angle = params.get("angle", 0)
    speed = params.get("speed", 1.0)

    if self._robot_controller:
        success = await self._robot_controller.rotate_wrist(angle, speed)
    else:
        await asyncio.sleep(0.5)
        success = True

    return CommandResult(
        success=success,
        command_id=command.command_id,
        message="Wrist rotation completed" if success else "Failed"
    )
```

### 4.2 새 로봇 지원

**Step 1: 설정 파일 생성**

```yaml
# config/robot_config.yaml

robot:
  name: "ur5"
  prim_path: "/World/ur5"

joints:
  arm:
    names:
      - "shoulder_pan_joint"
      - "shoulder_lift_joint"
      - "elbow_joint"
      - "wrist_1_joint"
      - "wrist_2_joint"
      - "wrist_3_joint"
    indices: [0, 1, 2, 3, 4, 5]

  gripper:
    names:
      - "finger_joint"
    indices: [6]
```

**Step 2: 컨트롤러 수정 (필요시)**

```python
# isaac_interface/robot_controller.py

class RobotController:
    def initialize(self, world, articulation):
        # 로봇 타입에 따른 초기화
        robot_name = self.robot_config.get("name", "")

        if robot_name == "ur5":
            # UR5 특화 설정
            pass
        elif robot_name == "stretch":
            # Stretch 설정
            pass
```

### 4.3 커스텀 안전 규칙 추가

```python
# safety/workspace_validator.py

class WorkspaceValidator:
    def __init__(self, config: Dict):
        # ... 기존 코드
        self.custom_rules = config.get("custom_rules", [])

    def check_point(self, point: np.ndarray) -> WorkspaceCheckResult:
        # 기본 검사
        result = self._basic_check(point)

        # 커스텀 규칙 검사
        for rule in self.custom_rules:
            if not self._check_custom_rule(point, rule):
                return WorkspaceCheckResult(
                    is_valid=False,
                    message=f"Custom rule violation: {rule['name']}"
                )

        return result
```

---

## 5. 테스트

### 5.1 단위 테스트 실행

```bash
# 모든 테스트
pytest tests/ -v

# 특정 테스트
pytest tests/test_core.py -v

# 커버리지 포함
pytest tests/ --cov=core --cov-report=html
```

### 5.2 테스트 작성 예시

```python
# tests/test_new_feature.py

import pytest
from core.control_manager import ControlManager

class TestNewFeature:
    @pytest.fixture
    def manager(self):
        config = {
            "llm": {"provider": "openai"},
            "workspace": {"bounds": {"min": [-1, -1, 0], "max": [1, 1, 1.5]}}
        }
        return ControlManager(config)

    @pytest.mark.asyncio
    async def test_new_command(self, manager):
        # Mock LLM client
        manager._llm_client = MockLLMClient()

        result = await manager.process_command("새 명령")

        assert result.success
        assert result.message == "Expected message"
```

### 5.3 통합 테스트

```python
# tests/test_integration.py

import pytest
import httpx

@pytest.mark.asyncio
async def test_api_command():
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "http://localhost:8000/api/command",
            json={"command": "앞으로 10cm"}
        )

        assert response.status_code == 200
        data = response.json()
        assert data["success"]
```

---

## 6. 디버깅

### 6.1 로깅 설정

```python
# 디버그 로깅 활성화
import logging
logging.getLogger().setLevel(logging.DEBUG)

# 특정 모듈 로깅
logging.getLogger("core.llm_client").setLevel(logging.DEBUG)
```

### 6.2 LLM 응답 디버깅

```python
# core/llm_client.py

async def _call_openai(self, ...):
    response = await self._client.chat.completions.create(...)

    # 디버그 출력
    logger.debug(f"LLM Response: {response}")
    logger.debug(f"Tool calls: {response.choices[0].message.tool_calls}")

    return result
```

### 6.3 명령 추적

```python
# core/control_manager.py

async def process_command(self, command_text: str):
    logger.info(f"=== Command Start: {command_text} ===")

    # 각 단계 로깅
    logger.debug(f"System prompt: {system_prompt[:200]}...")
    logger.debug(f"LLM response: {llm_response}")
    logger.debug(f"Parsed command: {command.to_dict()}")
    logger.debug(f"Validation: {validation}")

    result = await self._execute_command(command)

    logger.info(f"=== Command End: {result.success} ===")
    return result
```

### 6.4 성능 프로파일링

```python
import time

async def process_command(self, command_text: str):
    timings = {}

    t0 = time.time()
    llm_response = await self._llm_client.process_command(...)
    timings["llm_call"] = time.time() - t0

    t0 = time.time()
    parse_result = self._parser.parse(...)
    timings["parsing"] = time.time() - t0

    t0 = time.time()
    result = await self._execute_command(command)
    timings["execution"] = time.time() - t0

    logger.info(f"Timings: {timings}")
```

---

## 추가 리소스

- **아키텍처 문서**: `../documents/01_System_Architecture_Design.md`
- **API 명세**: `../documents/07_API_Specification.md`
- **안전 시스템**: `../documents/06_Safety_System_Implementation_Guide.md`

---

**문서 버전**: 1.0
**최종 수정**: 2025-12-14
