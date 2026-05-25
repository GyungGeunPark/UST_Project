# Isaac Sim LLM Robot Control - API Specification

## 1. 문서 개요

### 1.1 목적
본 문서는 Isaac Sim LLM Robot Control 시스템의 API 명세를 정의합니다. REST API, WebSocket API, 내부 인터페이스를 포함합니다.

### 1.2 API 버전
- **버전**: v1.0.0
- **기본 URL**: `http://localhost:8000`
- **WebSocket URL**: `ws://localhost:8000/ws`

---

## 2. REST API

### 2.1 공통 사항

#### 인증
현재 버전은 인증을 요구하지 않습니다. 프로덕션 환경에서는 API Key 또는 JWT 인증을 추가하세요.

#### 응답 형식
모든 응답은 JSON 형식입니다.

```json
{
    "success": true,
    "data": { ... },
    "error": null,
    "timestamp": 1702544400.123
}
```

#### HTTP 상태 코드
| 코드 | 설명 |
|------|------|
| 200 | 성공 |
| 400 | 잘못된 요청 |
| 404 | 리소스 없음 |
| 500 | 서버 오류 |
| 503 | 서비스 불가 (컨트롤러 미연결) |

---

### 2.2 Command API

#### POST /api/command
자연어 명령을 처리합니다.

**Request**
```http
POST /api/command HTTP/1.1
Content-Type: application/json

{
    "command": "앞으로 10cm 이동해줘",
    "require_confirmation": false
}
```

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| command | string | ✅ | 자연어 명령 (1-500자) |
| require_confirmation | boolean | ❌ | 실행 전 확인 요청 (기본: false) |

**Response - Success (200)**
```json
{
    "success": true,
    "command_id": "550e8400-e29b-41d4-a716-446655440000",
    "message": "Command executed successfully",
    "result": {
        "function": "move_manipulator",
        "parameters": {
            "movement_type": "relative",
            "direction": "forward",
            "distance": 10.0,
            "speed": 1.0
        }
    },
    "execution_time": 2.35,
    "final_position": {
        "x": 0.500,
        "y": 0.300,
        "z": 0.500
    }
}
```

**Response - Validation Error (400)**
```json
{
    "success": false,
    "command_id": "550e8400-e29b-41d4-a716-446655440001",
    "message": "Target position outside workspace bounds",
    "error_code": "OUT_OF_WORKSPACE",
    "suggested_position": {
        "x": 0.95,
        "y": 0.30,
        "z": 0.50
    }
}
```

**Response - Service Unavailable (503)**
```json
{
    "success": false,
    "message": "Robot is currently moving",
    "error_code": "BUSY"
}
```

---

#### POST /api/quick_command
사전 정의된 빠른 명령을 실행합니다.

**Request**
```http
POST /api/quick_command HTTP/1.1
Content-Type: application/json

{
    "action": "forward",
    "value": 15.0
}
```

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| action | string | ✅ | 명령 타입 (아래 참조) |
| value | number | ❌ | 거리(cm) 또는 속도 (기본: 10) |

**유효한 action 값:**
| Action | 설명 | value 용도 |
|--------|------|-----------|
| forward | 전진 | 거리 (cm) |
| backward | 후진 | 거리 (cm) |
| left | 좌측 이동 | 거리 (cm) |
| right | 우측 이동 | 거리 (cm) |
| up | 상승 | 거리 (cm) |
| down | 하강 | 거리 (cm) |
| grip_open | 그리퍼 열기 | 무시 |
| grip_close | 그리퍼 닫기 | 무시 |
| stop | 정지 | 무시 |

**Response**
`POST /api/command`와 동일

---

### 2.3 Status API

#### GET /api/status
현재 로봇 상태를 조회합니다.

**Request**
```http
GET /api/status HTTP/1.1
```

**Response (200)**
```json
{
    "connected": true,
    "state": "idle",
    "is_moving": false,
    "emergency_stopped": false,
    "position": {
        "x": 0.500,
        "y": 0.300,
        "z": 0.400
    },
    "orientation": {
        "qw": 1.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0
    },
    "joint_positions": [0.0, 0.5, 1.0, 0.2, 0.3, 0.1],
    "gripper_state": "open",
    "last_command_id": "550e8400-e29b-41d4-a716-446655440000",
    "last_error": null,
    "uptime": 3600.5,
    "command_count": 42
}
```

**state 필드 값:**
| 값 | 설명 |
|-----|------|
| idle | 대기 중 |
| processing | 명령 처리 중 |
| moving | 이동 중 |
| emergency_stopped | 비상 정지 상태 |
| error | 오류 상태 |

**gripper_state 필드 값:**
| 값 | 설명 |
|-----|------|
| open | 열림 |
| closed | 닫힘 |
| moving | 동작 중 |
| grasping | 파지 중 |

---

#### GET /api/health
서버 헬스 체크를 수행합니다.

**Request**
```http
GET /api/health HTTP/1.1
```

**Response (200)**
```json
{
    "status": "healthy",
    "control_manager_connected": true,
    "websocket_clients": 3,
    "uptime": 3600.5,
    "version": "1.0.0"
}
```

---

### 2.4 Emergency API

#### POST /api/emergency_stop
비상 정지를 활성화합니다.

**Request**
```http
POST /api/emergency_stop HTTP/1.1
Content-Type: application/json

{}
```

**Response (200)**
```json
{
    "status": "emergency_stopped",
    "timestamp": 1702544400.123
}
```

---

#### POST /api/reset
비상 정지를 해제합니다.

**Request**
```http
POST /api/reset HTTP/1.1
Content-Type: application/json

{}
```

**Response (200)**
```json
{
    "status": "reset",
    "timestamp": 1702544405.456
}
```

**Response - Cannot Reset (400)**
```json
{
    "status": "failed",
    "message": "Cannot reset: robot still moving",
    "current_velocity": 0.05
}
```

---

### 2.5 Statistics API

#### GET /api/stats
시스템 통계를 조회합니다.

**Request**
```http
GET /api/stats HTTP/1.1
```

**Response (200)**
```json
{
    "total_commands": 150,
    "successful_commands": 142,
    "failed_commands": 8,
    "uptime_seconds": 7200.0,
    "websocket_clients": 2,
    "llm_stats": {
        "total_requests": 150,
        "total_prompt_tokens": 45000,
        "total_completion_tokens": 3000,
        "estimated_cost_usd": 0.54,
        "average_latency_ms": 450
    },
    "cache_stats": {
        "size": 45,
        "max_size": 100,
        "hit_rate": 0.32,
        "total_hits": 48
    }
}
```

---

#### POST /api/clear_cache
명령 캐시를 초기화합니다.

**Request**
```http
POST /api/clear_cache HTTP/1.1
```

**Response (200)**
```json
{
    "status": "cache_cleared",
    "cleared_entries": 45
}
```

---

## 3. WebSocket API

### 3.1 연결

**URL**: `ws://localhost:8000/ws`

**연결 예시 (JavaScript)**
```javascript
const ws = new WebSocket('ws://localhost:8000/ws');

ws.onopen = () => {
    console.log('Connected');
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    console.log('Message:', data);
};

ws.onclose = () => {
    console.log('Disconnected');
};
```

---

### 3.2 Server → Client 메시지

#### Status Update
10Hz로 브로드캐스트되는 상태 업데이트

```json
{
    "type": "status",
    "data": {
        "state": "idle",
        "is_moving": false,
        "emergency_stopped": false,
        "position": [0.5, 0.3, 0.4],
        "timestamp": 1702544400.123
    }
}
```

---

#### Command Result
명령 실행 결과

```json
{
    "type": "command_result",
    "data": {
        "success": true,
        "command_id": "550e8400-e29b-41d4-a716-446655440000",
        "message": "Command executed successfully",
        "execution_time": 2.35
    }
}
```

---

#### Emergency Stop Event
비상 정지 이벤트

```json
{
    "type": "emergency_stop",
    "data": {
        "reason": "user_triggered",
        "details": "Manual emergency stop",
        "timestamp": 1702544400.123,
        "position": [0.5, 0.3, 0.4]
    }
}
```

---

#### Error Event
오류 이벤트

```json
{
    "type": "error",
    "data": {
        "code": "IK_FAILED",
        "message": "IK solution not found for target position",
        "timestamp": 1702544400.123
    }
}
```

---

### 3.3 Client → Server 메시지

#### Ping
연결 유지 핑

```json
{
    "type": "ping"
}
```

**응답:**
```json
{
    "type": "pong"
}
```

---

#### Command
WebSocket을 통한 명령 전송

```json
{
    "type": "command",
    "command": "앞으로 10cm"
}
```

---

## 4. Error Codes

### 4.1 일반 오류

| 코드 | 설명 |
|------|------|
| INTERNAL_ERROR | 내부 서버 오류 |
| INVALID_REQUEST | 잘못된 요청 |
| NOT_CONNECTED | 컨트롤러 미연결 |

### 4.2 명령 오류

| 코드 | 설명 |
|------|------|
| BUSY | 로봇 이동 중 |
| EMERGENCY_STOPPED | 비상 정지 상태 |
| LLM_ERROR | LLM API 오류 |
| PARSE_ERROR | 명령 파싱 실패 |
| NO_FUNCTION_CALL | LLM 함수 호출 없음 |

### 4.3 검증 오류

| 코드 | 설명 |
|------|------|
| OUT_OF_WORKSPACE | 작업 공간 이탈 |
| VELOCITY_EXCEEDED | 속도 초과 |
| COLLISION_DETECTED | 충돌 감지 |
| IK_FAILED | IK 솔루션 없음 |
| INVALID_COMMAND | 잘못된 명령 |

### 4.4 안전 오류

| 코드 | 설명 |
|------|------|
| WORKSPACE_VIOLATION | 작업 공간 위반 |
| SELF_COLLISION | 자기 충돌 |
| WATCHDOG_TIMEOUT | 워치독 타임아웃 |

---

## 5. LLM Function Calling Schema

### 5.1 move_manipulator

```json
{
    "name": "move_manipulator",
    "description": "매니퓰레이터 엔드이펙터 이동",
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
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "z": {"type": "number"}
                }
            },
            "speed": {
                "type": "number",
                "minimum": 0.1,
                "maximum": 2.0,
                "default": 1.0
            }
        },
        "required": ["movement_type"]
    }
}
```

### 5.2 move_mobile_base

```json
{
    "name": "move_mobile_base",
    "description": "모바일 베이스 이동",
    "parameters": {
        "type": "object",
        "properties": {
            "linear_velocity": {
                "type": "number",
                "minimum": -1.0,
                "maximum": 1.0
            },
            "angular_velocity": {
                "type": "number",
                "minimum": -1.5,
                "maximum": 1.5,
                "default": 0.0
            },
            "duration": {
                "type": "number",
                "minimum": 0.1,
                "maximum": 10.0,
                "default": 2.0
            }
        },
        "required": ["linear_velocity"]
    }
}
```

### 5.3 control_gripper

```json
{
    "name": "control_gripper",
    "description": "그리퍼 제어",
    "parameters": {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": ["open", "close"]
            }
        },
        "required": ["action"]
    }
}
```

### 5.4 stop_robot

```json
{
    "name": "stop_robot",
    "description": "모든 동작 정지",
    "parameters": {
        "type": "object",
        "properties": {},
        "required": []
    }
}
```

---

## 6. 설정 파일 스키마

### 6.1 robot_config.yaml

```yaml
# Robot Configuration Schema

robot:
  name: string              # 로봇 이름
  prim_path: string         # USD Prim 경로

files:
  urdf_path: string         # URDF 파일 경로
  usd_path: string          # USD 파일 경로
  lula_description_path: string  # Lula 설명 파일

joints:
  wheel:
    indices: array[int]     # 휠 조인트 인덱스
    radius: float           # 휠 반지름 (m)
    base_width: float       # 베이스 너비 (m)
    max_velocity: float     # 최대 속도 (rad/s)

  arm:
    indices: array[int]     # 팔 조인트 인덱스
    end_effector_frame: string  # 엔드이펙터 프레임

  gripper:
    indices: array[int]     # 그리퍼 조인트 인덱스
    open_position: float    # 열림 위치 (m)
    close_position: float   # 닫힘 위치 (m)

control:
  position_stiffness: float # 위치 강성
  position_damping: float   # 위치 감쇠
```

### 6.2 workspace_config.yaml

```yaml
# Workspace Configuration Schema

bounds:
  min: array[float, 3]      # 최소 경계 [x, y, z]
  max: array[float, 3]      # 최대 경계 [x, y, z]

velocity_limits:
  manipulator:
    max_linear: float       # 최대 선속도 (m/s)
    max_angular: float      # 최대 각속도 (rad/s)
    max_acceleration: float # 최대 가속도 (m/s^2)
  base:
    max_linear: float       # 최대 선속도
    max_angular: float      # 최대 각속도

safety:
  workspace_margin: float   # 경계 마진 (m)
  self_collision_check: boolean
  environment_collision_check: boolean
  collision_min_distance: float
```

### 6.3 llm_config.yaml

```yaml
# LLM Configuration Schema

provider: string            # "openai" | "anthropic"

openai:
  api_key: string           # API 키 (환경변수 지원: ${ENV_VAR})
  model: string             # 모델 ID
  temperature: float        # 0.0 - 2.0
  max_tokens: int           # 최대 토큰
  timeout: float            # 타임아웃 (초)

anthropic:
  api_key: string
  model: string

rate_limit:
  min_interval: float       # 최소 호출 간격 (초)
  max_retries: int          # 최대 재시도
  retry_delay: float        # 재시도 지연 (초)

cache:
  enabled: boolean
  max_size: int             # 최대 캐시 항목
  ttl: float                # TTL (초)
```

---

## 7. 사용 예시

### 7.1 Python 클라이언트

```python
import requests
import websocket
import json

BASE_URL = "http://localhost:8000"

# REST API 예시
def send_command(command: str) -> dict:
    response = requests.post(
        f"{BASE_URL}/api/command",
        json={"command": command}
    )
    return response.json()

# 상태 조회
def get_status() -> dict:
    response = requests.get(f"{BASE_URL}/api/status")
    return response.json()

# 비상 정지
def emergency_stop():
    response = requests.post(f"{BASE_URL}/api/emergency_stop")
    return response.json()

# WebSocket 예시
def on_message(ws, message):
    data = json.loads(message)
    if data["type"] == "status":
        print(f"Position: {data['data']['position']}")

def start_websocket():
    ws = websocket.WebSocketApp(
        "ws://localhost:8000/ws",
        on_message=on_message
    )
    ws.run_forever()

# 사용 예시
result = send_command("앞으로 10cm")
print(f"Success: {result['success']}")
```

### 7.2 JavaScript 클라이언트

```javascript
const BASE_URL = 'http://localhost:8000';

// REST API
async function sendCommand(command) {
    const response = await fetch(`${BASE_URL}/api/command`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command })
    });
    return response.json();
}

async function getStatus() {
    const response = await fetch(`${BASE_URL}/api/status`);
    return response.json();
}

// WebSocket
const ws = new WebSocket('ws://localhost:8000/ws');

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);

    switch (data.type) {
        case 'status':
            console.log('Position:', data.data.position);
            break;
        case 'command_result':
            console.log('Command result:', data.data);
            break;
        case 'emergency_stop':
            console.log('Emergency stop!', data.data.reason);
            break;
    }
};

// 명령 전송
ws.send(JSON.stringify({
    type: 'command',
    command: '앞으로 10cm'
}));
```

### 7.3 cURL 예시

```bash
# 명령 전송
curl -X POST http://localhost:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"command": "앞으로 10cm"}'

# 상태 조회
curl http://localhost:8000/api/status

# 비상 정지
curl -X POST http://localhost:8000/api/emergency_stop

# 리셋
curl -X POST http://localhost:8000/api/reset

# 빠른 명령
curl -X POST http://localhost:8000/api/quick_command \
  -H "Content-Type: application/json" \
  -d '{"action": "forward", "value": 15}'
```

---

## 8. 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-12-14 | 초기 API 명세 |

---

**문서 끝**
