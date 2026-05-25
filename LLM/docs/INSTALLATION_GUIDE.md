# Isaac Sim LLM Robot Control - 설치 및 적용 가이드

## 목차

1. [시스템 요구사항](#1-시스템-요구사항)
2. [설치 방법](#2-설치-방법)
3. [환경 설정](#3-환경-설정)
4. [실행 방법](#4-실행-방법)
5. [Isaac Sim 연동](#5-isaac-sim-연동)
6. [웹 UI 사용법](#6-웹-ui-사용법)
7. [API 사용법](#7-api-사용법)
8. [커스터마이징](#8-커스터마이징)
9. [문제 해결](#9-문제-해결)

---

## 1. 시스템 요구사항

### 필수 요구사항

| 항목 | 요구사항 |
|------|---------|
| Python | 3.10 이상 |
| OS | Ubuntu 20.04/22.04, Windows 10/11 |
| RAM | 최소 8GB (16GB 권장) |
| 네트워크 | LLM API 호출을 위한 인터넷 연결 |

### 선택적 요구사항 (Isaac Sim 연동 시)

| 항목 | 요구사항 |
|------|---------|
| NVIDIA GPU | RTX 2070 이상 |
| VRAM | 최소 8GB |
| Isaac Sim | 2023.1.0 이상 |
| CUDA | 11.8 이상 |

### LLM API 키

다음 중 하나 이상의 API 키가 필요합니다:

- **OpenAI API Key**: GPT-4, GPT-4o 사용
- **Anthropic API Key**: Claude 사용

---

## 2. 설치 방법

### 2.1 저장소 구조 확인

```bash
cd /workspace/isaaclab/ust_ws/LLM
```

디렉토리 구조:
```
LLM/
├── config/           # 설정 파일
├── core/             # 핵심 모듈
├── isaac_interface/  # Isaac Sim 인터페이스
├── safety/           # 안전 시스템
├── web/              # 웹 서버
├── utils/            # 유틸리티
├── scripts/          # 실행 스크립트
├── tests/            # 테스트
├── docs/             # 문서
└── requirements.txt  # 의존성
```

### 2.2 Python 의존성 설치

```bash
# 가상환경 생성 (권장)
python -m venv venv
source venv/bin/activate  # Linux/Mac
# 또는
.\venv\Scripts\activate  # Windows

# 의존성 설치
pip install -r requirements.txt
```

### 2.3 의존성 목록

```
fastapi>=0.100.0      # 웹 프레임워크
uvicorn[standard]     # ASGI 서버
websockets>=11.0      # WebSocket 지원
openai>=1.0.0         # OpenAI API
anthropic>=0.18.0     # Anthropic API
pyyaml>=6.0           # YAML 설정
pydantic>=2.0         # 데이터 검증
numpy>=1.24.0         # 수치 연산
httpx>=0.24.0         # HTTP 클라이언트
```

---

## 3. 환경 설정

### 3.1 API 키 설정

#### 방법 1: 환경 변수 (권장)

```bash
# Linux/Mac - ~/.bashrc 또는 ~/.zshrc에 추가
export OPENAI_API_KEY="sk-your-openai-api-key"
export ANTHROPIC_API_KEY="sk-ant-your-anthropic-key"

# 적용
source ~/.bashrc
```

```powershell
# Windows PowerShell
$env:OPENAI_API_KEY = "sk-your-openai-api-key"
$env:ANTHROPIC_API_KEY = "sk-ant-your-anthropic-key"
```

#### 방법 2: 설정 파일

`config/llm_config.yaml` 수정:

```yaml
provider: "openai"  # 또는 "anthropic"

openai:
  api_key: "sk-your-api-key"  # 직접 입력 (비권장)
  # 또는 환경변수 참조
  api_key: "${OPENAI_API_KEY}"
  model: "gpt-4o"
  temperature: 0.1
  max_tokens: 1024
```

### 3.2 로봇 설정

`config/robot_config.yaml`:

```yaml
robot:
  name: "stretch"
  prim_path: "/World/stretch"  # Isaac Sim에서의 경로

joints:
  arm:
    indices: [2, 3, 4, 5, 6, 7]
    end_effector_frame: "link_gripper"

  gripper:
    indices: [8, 9]
    open_position: 0.04
    close_position: 0.0
```

### 3.3 작업 공간 설정

`config/workspace_config.yaml`:

```yaml
bounds:
  min: [-2.0, -2.0, 0.0]  # meters [x, y, z]
  max: [2.0, 2.0, 1.5]

velocity_limits:
  manipulator:
    max_linear: 0.5       # m/s
    max_angular: 1.0      # rad/s

safety:
  workspace_margin: 0.05  # 경계로부터의 안전 마진
  self_collision_check: true
  environment_collision_check: true
```

### 3.4 서버 설정

`config/server_config.yaml`:

```yaml
server:
  host: "0.0.0.0"
  port: 8000

websocket:
  status_broadcast_hz: 10  # 상태 업데이트 빈도

logging:
  level: "INFO"  # DEBUG, INFO, WARNING, ERROR
  file: "logs/robot_control.log"
```

---

## 4. 실행 방법

### 4.1 독립 실행 모드 (Isaac Sim 없이)

테스트 및 개발 목적으로 Isaac Sim 없이 실행:

```bash
cd /workspace/isaaclab/ust_ws/LLM

# 기본 실행
python scripts/run_standalone.py

# 옵션 지정
python scripts/run_standalone.py --port 8080 --debug

# CLI 모드 (웹 서버 없이)
python scripts/run_standalone.py --no-web
```

**실행 옵션:**

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--port` | 웹 서버 포트 | 8000 |
| `--host` | 웹 서버 호스트 | 0.0.0.0 |
| `--config-dir` | 설정 디렉토리 경로 | config/ |
| `--no-web` | 웹 서버 없이 CLI 모드 | False |
| `--debug` | 디버그 로깅 활성화 | False |

### 4.2 CLI 모드 사용

`--no-web` 옵션으로 실행 시:

```
Enter command: 앞으로 10cm
Processing: 앞으로 10cm
Success: Manipulator move completed (simulation)
Final position: [0.6, 0.3, 0.5]

Enter command: status
Robot Status:
  State: completed
  Position: [0.6, 0.3, 0.5]
  Gripper: open
  Emergency Stopped: False

Enter command: quit
```

---

## 5. Isaac Sim 연동

### 5.1 Isaac Sim Python으로 실행

```bash
# Isaac Sim Python 경로 (예시)
~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh scripts/run_with_isaac.py

# 옵션
~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh scripts/run_with_isaac.py \
  --port 8000 \
  --usd-path /path/to/your/scene.usd
```

### 5.2 Isaac Sim에서 직접 실행

Isaac Sim Script Editor에서:

```python
import sys
sys.path.append("/workspace/isaaclab/ust_ws/LLM")

from utils.config_loader import load_config
from core.control_manager import ControlManager
from isaac_interface.robot_controller import RobotController
from web.server import start_server

# 설정 로드
config = load_config()

# 로봇 컨트롤러 초기화
robot_controller = RobotController(config)
robot_controller.initialize(world, robot_articulation)

# 컨트롤 매니저 생성
control_manager = ControlManager(config)
control_manager.set_robot_controller(robot_controller)

# 웹 서버 시작 (별도 스레드)
import threading
server_thread = threading.Thread(
    target=start_server,
    args=(control_manager, {"port": 8000})
)
server_thread.daemon = True
server_thread.start()
```

### 5.3 커스텀 로봇 연동

다른 로봇을 사용하려면:

1. `config/robot_config.yaml` 수정
2. `isaac_interface/robot_controller.py`에서 조인트 매핑 수정

```yaml
# config/robot_config.yaml
robot:
  name: "my_robot"
  prim_path: "/World/my_robot"

joints:
  arm:
    names:
      - "joint1"
      - "joint2"
      # ... 로봇의 조인트 이름
    indices: [0, 1, 2, 3, 4, 5]
```

---

## 6. 웹 UI 사용법

### 6.1 접속

브라우저에서 접속:
```
http://localhost:8000
```

### 6.2 UI 구성

```
┌─────────────────────────────────────────────────────────────────┐
│  Isaac Sim Robot Control                    [● Connected]       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────┐  ┌─────────────────────────────┐  │
│  │ Command Input           │  │ Robot Status                │  │
│  │                         │  │                             │  │
│  │ [___________________]   │  │ State: Idle                 │  │
│  │ [Send]                  │  │ Position X: 0.500           │  │
│  │                         │  │ Position Y: 0.300           │  │
│  │ Quick: [앞으로] [뒤로]  │  │ Position Z: 0.500           │  │
│  │        [열기] [닫기]    │  │ Gripper: open               │  │
│  └─────────────────────────┘  │ Commands: 42                │  │
│                               └─────────────────────────────┘  │
│  ┌─────────────────────────┐                                   │
│  │ Direction Control       │  ┌─────────────────────────────┐  │
│  │        [↑]              │  │ Command Log                 │  │
│  │    [←] [■] [→]          │  │                             │  │
│  │        [↓]              │  │ 10:30:15 앞으로 10cm: OK    │  │
│  │                         │  │ 10:30:12 그리퍼 열어: OK    │  │
│  │ [⬆ Forward] [⬇ Back]    │  │ 10:30:05 위로 5cm: OK       │  │
│  │                         │  │                             │  │
│  │ Distance: [====10====]  │  └─────────────────────────────┘  │
│  └─────────────────────────┘                                   │
│                                                                 │
│  ┌─────────────────────────┐                                   │
│  │ [  EMERGENCY STOP  ]    │  [ Reset ]                        │
│  └─────────────────────────┘                                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 6.3 명령어 예시

**한국어 명령:**
```
앞으로 10cm              # 전진
뒤로 20cm               # 후진
왼쪽으로 5cm            # 좌측 이동
오른쪽으로 5cm          # 우측 이동
위로 10cm               # 상승
아래로 10cm             # 하강
그리퍼 열어             # 그리퍼 열기
그리퍼 닫아             # 그리퍼 닫기
정지                    # 모든 동작 정지
```

**영어 명령:**
```
move forward 10cm
move backward 20cm
open gripper
close gripper
stop
```

---

## 7. API 사용법

### 7.1 REST API

#### 명령 전송

```bash
# 자연어 명령
curl -X POST http://localhost:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"command": "앞으로 10cm"}'

# 응답
{
  "success": true,
  "command_id": "550e8400-e29b-41d4-a716-446655440000",
  "message": "Command executed successfully",
  "execution_time": 0.52,
  "final_position": {"x": 0.6, "y": 0.3, "z": 0.5}
}
```

#### 빠른 명령

```bash
curl -X POST http://localhost:8000/api/quick_command \
  -H "Content-Type: application/json" \
  -d '{"action": "forward", "value": 15}'
```

#### 상태 조회

```bash
curl http://localhost:8000/api/status

# 응답
{
  "connected": true,
  "state": "idle",
  "is_moving": false,
  "emergency_stopped": false,
  "position": {"x": 0.5, "y": 0.3, "z": 0.5},
  "gripper_state": "open",
  "command_count": 42
}
```

#### 비상 정지

```bash
curl -X POST http://localhost:8000/api/emergency_stop

# 리셋
curl -X POST http://localhost:8000/api/reset
```

### 7.2 WebSocket API

```javascript
const ws = new WebSocket('ws://localhost:8000/ws');

ws.onopen = () => {
    console.log('Connected');

    // 명령 전송
    ws.send(JSON.stringify({
        type: 'command',
        command: '앞으로 10cm'
    }));
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);

    switch (data.type) {
        case 'status':
            // 10Hz 상태 업데이트
            console.log('Position:', data.data.position);
            break;
        case 'command_result':
            console.log('Result:', data.data);
            break;
        case 'emergency_stop':
            console.log('Emergency stopped!');
            break;
    }
};
```

### 7.3 Python 클라이언트

```python
import requests
import json

BASE_URL = "http://localhost:8000"

def send_command(command: str):
    """명령 전송"""
    response = requests.post(
        f"{BASE_URL}/api/command",
        json={"command": command}
    )
    return response.json()

def get_status():
    """상태 조회"""
    response = requests.get(f"{BASE_URL}/api/status")
    return response.json()

# 사용 예시
result = send_command("앞으로 10cm")
print(f"Success: {result['success']}")

status = get_status()
print(f"Position: {status['position']}")
```

---

## 8. 커스터마이징

### 8.1 새 명령 추가

`core/llm_tools.py`에 새 함수 정의 추가:

```python
# core/llm_tools.py

def get_tool_definitions():
    return [
        # ... 기존 도구들
        {
            "type": "function",
            "function": {
                "name": "rotate_gripper",
                "description": "Rotate the gripper to a specific angle",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "angle": {
                            "type": "number",
                            "minimum": -180,
                            "maximum": 180,
                            "description": "Rotation angle in degrees"
                        }
                    },
                    "required": ["angle"]
                }
            }
        }
    ]
```

### 8.2 프롬프트 수정

`core/prompts.py`에서 시스템 프롬프트 수정:

```python
SYSTEM_PROMPT_BASE = """You are a robot control assistant...

## 추가 지침
- 새로운 기능에 대한 설명
- 특수 상황 처리 방법
"""
```

### 8.3 LLM 모델 변경

`config/llm_config.yaml`:

```yaml
provider: "anthropic"  # OpenAI에서 변경

anthropic:
  api_key: "${ANTHROPIC_API_KEY}"
  model: "claude-sonnet-4-20250514"  # 또는 다른 모델
  temperature: 0.1
```

### 8.4 안전 제한 조정

`config/workspace_config.yaml`:

```yaml
velocity_limits:
  manipulator:
    max_linear: 0.3       # 더 느리게
    max_angular: 0.5

safety:
  workspace_margin: 0.1   # 더 큰 마진
```

---

## 9. 문제 해결

### 9.1 일반적인 오류

#### LLM API 오류

```
Error: LLM client not initialized
```

**해결:**
1. API 키가 올바르게 설정되었는지 확인
2. 환경 변수 확인: `echo $OPENAI_API_KEY`
3. 네트워크 연결 확인

#### 포트 충돌

```
Error: Address already in use
```

**해결:**
```bash
# 다른 포트 사용
python scripts/run_standalone.py --port 8080

# 또는 기존 프로세스 종료
lsof -i :8000
kill -9 <PID>
```

#### 명령 파싱 실패

```
Error: No function call in response
```

**해결:**
- 명령을 더 명확하게 작성
- 지원되는 명령 형식 확인
- LLM temperature 낮추기 (config에서 0.1 이하)

### 9.2 Isaac Sim 연동 오류

#### ImportError

```
Error: Isaac Sim not found
```

**해결:**
- Isaac Sim Python으로 실행 확인
- 경로 확인: `which python.sh`

#### 로봇 미발견

```
Warning: Robot not found at /World/stretch
```

**해결:**
1. USD 씬에 로봇이 로드되었는지 확인
2. `robot_config.yaml`의 `prim_path` 확인
3. Isaac Sim에서 로봇 경로 확인

### 9.3 성능 문제

#### 느린 응답

- LLM 캐시 활성화 확인 (`llm_config.yaml`)
- 네트워크 지연 확인
- 더 빠른 모델 사용 (gpt-4o-mini)

#### 메모리 부족

- 로그 파일 크기 제한 확인
- WebSocket 클라이언트 수 제한

### 9.4 로그 확인

```bash
# 로그 파일 확인
tail -f logs/robot_control.log

# 디버그 모드 실행
python scripts/run_standalone.py --debug
```

---

## 추가 리소스

- **API 명세서**: `docs/07_API_Specification.md`
- **아키텍처 문서**: `docs/01_System_Architecture_Design.md`
- **안전 시스템**: `docs/06_Safety_System_Implementation_Guide.md`

---

**문서 버전**: 1.0
**최종 수정**: 2025-12-14
