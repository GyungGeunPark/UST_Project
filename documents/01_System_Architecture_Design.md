# Isaac Sim LLM Robot Control - System Architecture Design

## 1. 문서 개요

### 1.1 목적
본 문서는 Isaac Sim 환경에서 LLM 기반 로봇 제어 시스템의 전체 아키텍처를 정의합니다. Unity LLMRobotControl 시스템을 Isaac Sim으로 포팅하기 위한 설계 청사진을 제공합니다.

### 1.2 범위
- 4륜 구동 모바일 베이스 + 매니퓰레이터 로봇 제어
- LLM(GPT-4/Claude) Function Calling 기반 자연어 명령 처리
- 웹 UI를 통한 사용자 인터페이스
- 실시간 안전 시스템

### 1.3 대상 독자
- 시스템 개발자
- 로봇 엔지니어
- 프로젝트 관리자

---

## 2. 시스템 아키텍처 개요

### 2.1 High-Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              USER INTERFACE LAYER                                │
│  ┌────────────────────┐  ┌─────────────────────┐  ┌──────────────────────┐     │
│  │   Web Browser      │  │  Mobile App (선택)   │  │  Voice Interface     │     │
│  │   (React/Vue)      │  │                     │  │  (선택)              │     │
│  └─────────┬──────────┘  └─────────┬───────────┘  └──────────┬───────────┘     │
└────────────┼──────────────────────┼──────────────────────────┼──────────────────┘
             │ HTTP/WebSocket       │                          │
             ▼                      ▼                          ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              WEB SERVER LAYER                                    │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                        FastAPI Application                                │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────┐  │  │
│  │  │ REST API        │  │ WebSocket       │  │ Static File Server      │  │  │
│  │  │ /api/command    │  │ /ws             │  │ /static                 │  │  │
│  │  │ /api/status     │  │ (실시간 상태)    │  │ (HTML/CSS/JS)          │  │  │
│  │  │ /api/emergency  │  │                 │  │                         │  │  │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────┬─────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           CORE CONTROL LAYER                                     │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                    LLMRobotControlManager                                 │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  ┌────────────────┐  │  │
│  │  │ Command     │  │ State       │  │ Callback   │  │ Command        │  │  │
│  │  │ Queue       │  │ Machine     │  │ Manager    │  │ Cache          │  │  │
│  │  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  └───────┬────────┘  │  │
│  │         │                │               │                  │           │  │
│  └─────────┼────────────────┼───────────────┼──────────────────┼───────────┘  │
│            │                │               │                  │              │
│  ┌─────────▼────────────────▼───────────────▼──────────────────▼───────────┐  │
│  │                       Processing Pipeline                                │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  ┌─────────────────┐ │  │
│  │  │ LLM Client  │─▶│ Response    │─▶│ Command    │─▶│ Motion          │ │  │
│  │  │ (OpenAI/    │  │ Parser      │  │ Validator  │  │ Executor        │ │  │
│  │  │ Anthropic)  │  │             │  │            │  │                 │ │  │
│  │  └─────────────┘  └─────────────┘  └────────────┘  └─────────────────┘ │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────┬─────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         ISAAC SIM INTERFACE LAYER                                │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                       IsaacRobotController                                │  │
│  │  ┌──────────────────┐  ┌───────────────────┐  ┌────────────────────┐    │  │
│  │  │ MobileBase       │  │ Manipulator       │  │ Gripper            │    │  │
│  │  │ Controller       │  │ Controller        │  │ Controller         │    │  │
│  │  │ ───────────────  │  │ ────────────────  │  │ ─────────────────  │    │  │
│  │  │ • velocity_cmd   │  │ • Lula IK Solver  │  │ • open/close       │    │  │
│  │  │ • odometry       │  │ • cuRobo (선택)   │  │ • position_control │    │  │
│  │  │ • wheel_control  │  │ • trajectory_gen  │  │                    │    │  │
│  │  └────────┬─────────┘  └─────────┬─────────┘  └──────────┬─────────┘    │  │
│  │           │                      │                       │              │  │
│  └───────────┼──────────────────────┼───────────────────────┼──────────────┘  │
│              │                      │                       │                 │
│  ┌───────────▼──────────────────────▼───────────────────────▼──────────────┐  │
│  │                    Articulation Controller                               │  │
│  │  ┌───────────────────────────────────────────────────────────────────┐  │  │
│  │  │ Joint Position Control | Joint Velocity Control | Joint Effort    │  │  │
│  │  └───────────────────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────┬─────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                            ISAAC SIM RUNTIME                                     │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                      Physics Simulation (PhysX)                           │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐ │  │
│  │  │ Robot USD   │  │ Environment │  │ Collision   │  │ Rendering       │ │  │
│  │  │ Model       │  │ USD Scene   │  │ Detection   │  │ (RTX/Ray Trace) │ │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────┘ │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────┘

                              CROSS-CUTTING CONCERNS
┌─────────────────────────────────────────────────────────────────────────────────┐
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  ┌─────────────────┐ │
│  │ Safety        │  │ Logging       │  │ Configuration │  │ Performance     │ │
│  │ System        │  │ System        │  │ Management    │  │ Monitor         │ │
│  │ ────────────  │  │ ────────────  │  │ ────────────  │  │ ──────────────  │ │
│  │ • E-Stop      │  │ • Structured  │  │ • YAML files  │  │ • Latency       │ │
│  │ • Workspace   │  │ • File/Console│  │ • Hot reload  │  │ • Throughput    │ │
│  │ • Collision   │  │ • Telemetry   │  │ • Validation  │  │ • Resource      │ │
│  └───────────────┘  └───────────────┘  └───────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 계층 설명

| 계층 | 책임 | 주요 기술 |
|------|------|----------|
| User Interface Layer | 사용자 입력 수신 및 피드백 표시 | HTML5, JavaScript, WebSocket |
| Web Server Layer | HTTP/WS 요청 처리, API 제공 | FastAPI, Uvicorn |
| Core Control Layer | 명령 처리, LLM 연동, 검증 | Python asyncio, OpenAI API |
| Isaac Sim Interface Layer | 로봇 제어 추상화 | Isaac Sim Python API |
| Isaac Sim Runtime | 물리 시뮬레이션 | PhysX, USD |

---

## 3. 컴포넌트 상세 설계

### 3.1 Core Control Layer 컴포넌트

#### 3.1.1 LLMRobotControlManager
**책임**: 전체 제어 흐름 오케스트레이션

```
┌─────────────────────────────────────────────────────────────────┐
│                    LLMRobotControlManager                        │
├─────────────────────────────────────────────────────────────────┤
│ Attributes:                                                      │
│   - llm_client: LLMClient                                       │
│   - validator: CommandValidator                                  │
│   - robot_controller: IsaacRobotController                      │
│   - command_cache: Dict[str, RobotCommand]                      │
│   - state: ControllerState (IDLE|PROCESSING|MOVING|E_STOP)      │
│   - event_callbacks: Dict[str, List[Callable]]                  │
├─────────────────────────────────────────────────────────────────┤
│ Methods:                                                         │
│   + async process_command(user_input: str) -> CommandResult     │
│   + async execute_command(command: RobotCommand) -> bool        │
│   + emergency_stop() -> None                                    │
│   + reset() -> None                                             │
│   + register_callback(event: str, callback: Callable) -> None   │
│   + get_status() -> ControllerStatus                            │
│   - _build_system_prompt() -> str                               │
│   - _parse_llm_response(response: dict) -> RobotCommand         │
│   - _cache_command(key: str, command: RobotCommand) -> None     │
└─────────────────────────────────────────────────────────────────┘
```

#### 3.1.2 LLMClient
**책임**: LLM API와의 통신

```
┌─────────────────────────────────────────────────────────────────┐
│                         LLMClient                                │
├─────────────────────────────────────────────────────────────────┤
│ Attributes:                                                      │
│   - client: AsyncOpenAI | AsyncAnthropic                        │
│   - model: str                                                   │
│   - tools: List[ToolDefinition]                                 │
│   - timeout: float                                               │
│   - retry_config: RetryConfig                                   │
├─────────────────────────────────────────────────────────────────┤
│ Methods:                                                         │
│   + async send_command(message: str, context: str) -> LLMResult │
│   + async stream_command(message: str) -> AsyncIterator         │
│   + set_tools(tools: List[ToolDefinition]) -> None              │
│   - _handle_rate_limit() -> None                                │
│   - _parse_tool_call(response) -> dict                          │
└─────────────────────────────────────────────────────────────────┘
```

#### 3.1.3 CommandValidator
**책임**: 명령 안전성 검증

```
┌─────────────────────────────────────────────────────────────────┐
│                      CommandValidator                            │
├─────────────────────────────────────────────────────────────────┤
│ Attributes:                                                      │
│   - workspace_bounds: WorkspaceBounds                           │
│   - velocity_limits: VelocityLimits                             │
│   - collision_checker: Optional[CollisionChecker]               │
├─────────────────────────────────────────────────────────────────┤
│ Methods:                                                         │
│   + validate(cmd: RobotCommand, current_pos: np.ndarray)        │
│       -> ValidationResult                                        │
│   + check_workspace_bounds(target: np.ndarray) -> bool          │
│   + check_velocity_limits(velocity: float) -> bool              │
│   + check_collision(trajectory: np.ndarray) -> bool             │
│   - _compute_target_position(cmd: RobotCommand,                 │
│       current: np.ndarray) -> np.ndarray                        │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Isaac Sim Interface Layer 컴포넌트

#### 3.2.1 IsaacRobotController
**책임**: Isaac Sim 로봇 제어 추상화

```
┌─────────────────────────────────────────────────────────────────┐
│                    IsaacRobotController                          │
├─────────────────────────────────────────────────────────────────┤
│ Attributes:                                                      │
│   - articulation: Articulation                                  │
│   - mobile_base: MobileBaseController                           │
│   - manipulator: ManipulatorController                          │
│   - gripper: GripperController                                  │
│   - config: RobotConfig                                         │
├─────────────────────────────────────────────────────────────────┤
│ Methods:                                                         │
│   + initialize(stage) -> None                                   │
│   + move_to_position(target: np.ndarray, orient: np.ndarray)    │
│       -> bool                                                    │
│   + move_base(linear: float, angular: float) -> None            │
│   + control_gripper(action: str) -> None                        │
│   + get_end_effector_pose() -> Tuple[np.ndarray, np.ndarray]   │
│   + get_joint_positions() -> np.ndarray                         │
│   + emergency_stop() -> None                                    │
└─────────────────────────────────────────────────────────────────┘
```

#### 3.2.2 MobileBaseController
**책임**: 4륜 구동 모바일 베이스 제어

```
┌─────────────────────────────────────────────────────────────────┐
│                   MobileBaseController                           │
├─────────────────────────────────────────────────────────────────┤
│ Attributes:                                                      │
│   - wheel_joint_indices: List[int]                              │
│   - wheel_radius: float                                          │
│   - wheel_base: float                                            │
│   - max_linear_velocity: float                                   │
│   - max_angular_velocity: float                                  │
│   - odometry: Odometry                                           │
├─────────────────────────────────────────────────────────────────┤
│ Methods:                                                         │
│   + set_velocity(linear: float, angular: float) -> None         │
│   + stop() -> None                                               │
│   + get_odometry() -> Odometry                                  │
│   - _compute_wheel_velocities(linear, angular) -> np.ndarray    │
│   - _differential_drive(linear, angular) -> np.ndarray          │
│   - _mecanum_drive(vx, vy, omega) -> np.ndarray                 │
└─────────────────────────────────────────────────────────────────┘
```

#### 3.2.3 ManipulatorController
**책임**: 매니퓰레이터 IK 기반 제어

```
┌─────────────────────────────────────────────────────────────────┐
│                   ManipulatorController                          │
├─────────────────────────────────────────────────────────────────┤
│ Attributes:                                                      │
│   - arm_joint_indices: List[int]                                │
│   - lula_solver: LulaKinematicsSolver                           │
│   - ik_solver: ArticulationKinematicsSolver                     │
│   - end_effector_frame: str                                      │
│   - joint_limits: JointLimits                                    │
├─────────────────────────────────────────────────────────────────┤
│ Methods:                                                         │
│   + move_to_pose(position: np.ndarray, orientation: np.ndarray) │
│       -> bool                                                    │
│   + move_joints(positions: np.ndarray) -> None                  │
│   + get_end_effector_pose() -> Tuple[np.ndarray, np.ndarray]   │
│   + compute_ik(target_pos, target_orient) -> Optional[np.ndarray]│
│   + compute_fk(joint_positions) -> Tuple[np.ndarray, np.ndarray]│
└─────────────────────────────────────────────────────────────────┘
```

#### 3.2.4 GripperController
**책임**: 그리퍼 제어

```
┌─────────────────────────────────────────────────────────────────┐
│                     GripperController                            │
├─────────────────────────────────────────────────────────────────┤
│ Attributes:                                                      │
│   - gripper_joint_indices: List[int]                            │
│   - open_position: float                                         │
│   - close_position: float                                        │
│   - grasp_force: float                                           │
├─────────────────────────────────────────────────────────────────┤
│ Methods:                                                         │
│   + open() -> None                                               │
│   + close() -> None                                              │
│   + set_position(position: float) -> None                       │
│   + get_state() -> GripperState                                 │
│   + is_grasping() -> bool                                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. 데이터 흐름

### 4.1 명령 처리 시퀀스 다이어그램

```
User        WebUI       FastAPI     ControlManager    LLMClient    Validator    RobotController
  │           │            │              │               │            │              │
  │ "앞으로    │            │              │               │            │              │
  │  10cm"    │            │              │               │            │              │
  ├──────────▶│            │              │               │            │              │
  │           │ POST       │              │               │            │              │
  │           │ /api/cmd   │              │               │            │              │
  │           ├───────────▶│              │               │            │              │
  │           │            │ process_cmd  │               │            │              │
  │           │            ├─────────────▶│               │            │              │
  │           │            │              │ send_command  │            │              │
  │           │            │              ├──────────────▶│            │              │
  │           │            │              │               │            │              │
  │           │            │              │  OpenAI API   │            │              │
  │           │            │              │◀──────────────┤            │              │
  │           │            │              │               │            │              │
  │           │            │              │ Function Call Result       │              │
  │           │            │              │ {move_manipulator,         │              │
  │           │            │              │  direction: forward,       │              │
  │           │            │              │  distance: 10}             │              │
  │           │            │              │               │            │              │
  │           │            │              │ validate      │            │              │
  │           │            │              ├───────────────┼───────────▶│              │
  │           │            │              │               │            │              │
  │           │            │              │◀──────────────┼────────────┤              │
  │           │            │              │ ValidationResult: OK       │              │
  │           │            │              │               │            │              │
  │           │            │              │ move_to_position           │              │
  │           │            │              ├───────────────┼────────────┼─────────────▶│
  │           │            │              │               │            │              │
  │           │            │              │               │            │   IK Solve   │
  │           │            │              │               │            │   Apply      │
  │           │            │              │               │            │   Action     │
  │           │            │              │               │            │              │
  │           │            │              │◀──────────────┼────────────┼──────────────┤
  │           │            │              │  Movement Complete         │              │
  │           │            │◀─────────────┤               │            │              │
  │           │◀───────────┤ {success: true}              │            │              │
  │◀──────────┤            │              │               │            │              │
  │ "완료"     │            │              │               │            │              │
```

### 4.2 상태 전이 다이어그램

```
                    ┌─────────┐
                    │  INIT   │
                    └────┬────┘
                         │ initialize()
                         ▼
           ┌────────────────────────────┐
           │                            │
           ▼                            │
    ┌─────────────┐    process_cmd()   │
    │    IDLE     │◀───────────────────┘
    └──────┬──────┘
           │ process_command()
           ▼
    ┌─────────────┐
    │ PROCESSING  │──────────────────────┐
    └──────┬──────┘                      │
           │ validation_passed           │ validation_failed
           ▼                             │
    ┌─────────────┐                      │
    │   MOVING    │                      │
    └──────┬──────┘                      │
           │ movement_complete           │
           │                             │
           ▼                             │
    ┌─────────────┐◀─────────────────────┘
    │    IDLE     │
    └─────────────┘


    Emergency Stop Path (from any state):

    ┌─────────────┐
    │  ANY STATE  │
    └──────┬──────┘
           │ emergency_stop()
           ▼
    ┌─────────────┐
    │  E_STOPPED  │
    └──────┬──────┘
           │ reset()
           ▼
    ┌─────────────┐
    │    IDLE     │
    └─────────────┘
```

---

## 5. 인터페이스 정의

### 5.1 내부 인터페이스

#### 5.1.1 RobotCommand 데이터 구조

```python
@dataclass
class RobotCommand:
    """로봇 명령 데이터 구조"""
    command_id: str                                    # UUID
    timestamp: float                                   # Unix timestamp
    command_type: CommandType                          # MANIPULATOR | BASE | GRIPPER

    # 매니퓰레이터 명령
    movement_type: Optional[MovementType] = None       # RELATIVE | ABSOLUTE
    direction: Optional[Direction] = None              # FORWARD | BACKWARD | LEFT | RIGHT | UP | DOWN
    distance: float = 0.0                              # cm
    absolute_position: Optional[np.ndarray] = None     # [x, y, z] in meters
    orientation: Optional[np.ndarray] = None           # [qw, qx, qy, qz]
    speed: float = 1.0                                 # 0.1 - 2.0

    # 모바일 베이스 명령
    linear_velocity: float = 0.0                       # m/s
    angular_velocity: float = 0.0                      # rad/s
    duration: float = 0.0                              # seconds

    # 그리퍼 명령
    gripper_action: Optional[GripperAction] = None     # OPEN | CLOSE
```

#### 5.1.2 ValidationResult 데이터 구조

```python
@dataclass
class ValidationResult:
    """명령 검증 결과"""
    is_valid: bool
    error_code: Optional[ErrorCode] = None
    error_message: Optional[str] = None
    warnings: List[str] = field(default_factory=list)
    computed_target: Optional[np.ndarray] = None
```

#### 5.1.3 ControllerStatus 데이터 구조

```python
@dataclass
class ControllerStatus:
    """컨트롤러 상태 정보"""
    state: ControllerState
    is_moving: bool
    emergency_stopped: bool
    current_position: np.ndarray
    current_orientation: np.ndarray
    joint_positions: np.ndarray
    gripper_state: GripperState
    last_command_id: Optional[str]
    last_error: Optional[str]
    uptime: float
```

### 5.2 외부 인터페이스 (REST API)

#### 5.2.1 POST /api/command
```yaml
Request:
  Content-Type: application/json
  Body:
    command: string          # 자연어 명령
    require_confirmation: bool  # 선택: 사용자 확인 필요 여부

Response:
  200 OK:
    success: boolean
    command_id: string
    result:
      action: string
      parameters: object
    message: string

  400 Bad Request:
    error: string
    code: string

  503 Service Unavailable:
    error: "Robot is currently moving"
```

#### 5.2.2 POST /api/emergency_stop
```yaml
Request:
  Content-Type: application/json
  Body: {} (empty)

Response:
  200 OK:
    status: "emergency_stopped"
    timestamp: float
```

#### 5.2.3 GET /api/status
```yaml
Response:
  200 OK:
    state: string              # IDLE | PROCESSING | MOVING | E_STOPPED
    is_moving: boolean
    emergency_stopped: boolean
    position:
      x: float
      y: float
      z: float
    orientation:
      qw: float
      qx: float
      qy: float
      qz: float
    gripper_state: string      # OPEN | CLOSED | MOVING
    last_error: string | null
```

#### 5.2.4 WebSocket /ws
```yaml
Connection: Upgrade to WebSocket

Server -> Client Messages (10Hz):
  type: "status"
  data:
    state: string
    is_moving: boolean
    position: [x, y, z]
    timestamp: float

  type: "command_result"
  data:
    command_id: string
    success: boolean
    message: string

  type: "error"
  data:
    code: string
    message: string
```

---

## 6. 설정 관리

### 6.1 설정 파일 구조

```
config/
├── robot_config.yaml       # 로봇 하드웨어 설정
├── workspace_config.yaml   # 작업 공간 경계 및 제한
├── llm_config.yaml         # LLM API 설정
└── server_config.yaml      # 웹 서버 설정
```

### 6.2 robot_config.yaml 스키마

```yaml
# 로봇 기본 설정
robot:
  name: "mobile_manipulator"
  prim_path: "/World/Robot"

# URDF/USD 파일 경로
files:
  urdf_path: "assets/robot/robot.urdf"
  lula_description_path: "assets/robot/robot_description.yaml"

# 조인트 설정
joints:
  # 모바일 베이스
  wheel:
    indices: [0, 1, 2, 3]          # FL, FR, RL, RR
    radius: 0.1                     # meters
    base_width: 0.5                 # meters
    base_length: 0.6                # meters
    max_velocity: 10.0              # rad/s

  # 매니퓰레이터
  arm:
    indices: [4, 5, 6, 7, 8, 9]    # 6 DOF
    end_effector_frame: "tool0"

  # 그리퍼
  gripper:
    indices: [10, 11]               # 2-finger
    open_position: 0.04             # meters
    close_position: 0.0
    grasp_force: 10.0               # Newtons

# 제어 파라미터
control:
  position_stiffness: 1000.0
  position_damping: 100.0
  velocity_damping: 50.0
```

### 6.3 workspace_config.yaml 스키마

```yaml
# 작업 공간 경계 (meters)
workspace:
  bounds:
    min: [-1.0, -1.0, 0.0]
    max: [1.0, 1.0, 1.5]

# 속도 제한
velocity_limits:
  manipulator:
    max_linear: 0.5               # m/s
    max_angular: 1.0              # rad/s
    max_acceleration: 2.0         # m/s^2
  base:
    max_linear: 1.0               # m/s
    max_angular: 1.5              # rad/s

# 안전 마진
safety:
  workspace_margin: 0.05          # meters
  self_collision_check: true
  environment_collision_check: true
```

### 6.4 llm_config.yaml 스키마

```yaml
# LLM 제공자 설정
provider: "openai"                  # openai | anthropic

# OpenAI 설정
openai:
  api_key: "${OPENAI_API_KEY}"      # 환경 변수 참조
  model: "gpt-4-turbo"
  temperature: 0.1
  max_tokens: 200
  timeout: 30

# Anthropic 설정 (대안)
anthropic:
  api_key: "${ANTHROPIC_API_KEY}"
  model: "claude-3-sonnet-20240229"

# Rate limiting
rate_limit:
  min_interval: 1.0                 # seconds
  max_retries: 3
  retry_delay: 2.0                  # seconds

# 캐싱
cache:
  enabled: true
  max_size: 100
  ttl: 3600                         # seconds
```

---

## 7. 의존성 관리

### 7.1 Python 의존성

```
# requirements.txt

# Isaac Sim (시스템에서 제공)
# isaacsim >= 4.2.0

# LLM API
openai>=1.0.0
anthropic>=0.8.0

# 웹 프레임워크
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
websockets>=12.0

# 데이터 처리
numpy>=1.24.0
pydantic>=2.0.0
pyyaml>=6.0

# 비동기 처리
asyncio
aiohttp>=3.9.0

# 유틸리티
python-dotenv>=1.0.0
structlog>=23.0.0

# 테스트
pytest>=7.0.0
pytest-asyncio>=0.21.0
```

### 7.2 버전 호환성 매트릭스

| Component | Minimum Version | Recommended | Notes |
|-----------|-----------------|-------------|-------|
| Python | 3.10 | 3.10 | Isaac Sim 요구사항 |
| Isaac Sim | 4.2.0 | 4.5.0 | Lula IK 포함 |
| CUDA | 11.8 | 12.1 | GPU 가속 |
| OpenAI API | v1 | v1 | Function calling |

---

## 8. 배포 구성

### 8.1 디렉토리 구조

```
isaac_llm_robot_control/
├── config/
│   ├── robot_config.yaml
│   ├── workspace_config.yaml
│   ├── llm_config.yaml
│   └── server_config.yaml
├── core/
│   ├── __init__.py
│   ├── robot_command.py
│   ├── llm_client.py
│   ├── response_parser.py
│   ├── command_validator.py
│   └── control_manager.py
├── isaac_interface/
│   ├── __init__.py
│   ├── robot_controller.py
│   ├── mobile_base.py
│   ├── manipulator.py
│   ├── gripper.py
│   └── ik_solver.py
├── web/
│   ├── __init__.py
│   ├── server.py
│   ├── api/
│   │   ├── __init__.py
│   │   ├── routes.py
│   │   └── websocket.py
│   ├── static/
│   │   ├── index.html
│   │   ├── css/
│   │   │   └── style.css
│   │   └── js/
│   │       └── main.js
│   └── templates/
├── safety/
│   ├── __init__.py
│   ├── emergency_stop.py
│   ├── collision_checker.py
│   └── workspace_validator.py
├── utils/
│   ├── __init__.py
│   ├── config_loader.py
│   ├── logging.py
│   └── performance_monitor.py
├── scripts/
│   ├── run_standalone.py
│   ├── run_with_ros2.py
│   └── setup_robot.py
├── tests/
│   ├── __init__.py
│   ├── test_llm_client.py
│   ├── test_validator.py
│   ├── test_robot_controller.py
│   └── test_integration.py
├── assets/
│   └── robot/
│       ├── robot.urdf
│       └── robot_description.yaml
├── requirements.txt
├── setup.py
└── README.md
```

### 8.2 실행 방식

#### 8.2.1 Standalone 모드 (권장)
```bash
# Isaac Sim Python 환경에서 실행
./python.sh scripts/run_standalone.py
```

#### 8.2.2 ROS2 통합 모드
```bash
# ROS2 환경 활성화 후 실행
source /opt/ros/humble/setup.bash
./python.sh scripts/run_with_ros2.py
```

---

## 9. 확장성 고려사항

### 9.1 수평 확장
- 웹 서버는 여러 인스턴스로 확장 가능 (로드 밸런서 사용)
- LLM 요청은 비동기로 처리되어 동시 사용자 지원

### 9.2 기능 확장 포인트
1. **새로운 LLM 제공자 추가**: `LLMClient` 인터페이스 구현
2. **새로운 로봇 타입 지원**: `IsaacRobotController` 서브클래스 생성
3. **추가 안전 검사**: `CommandValidator`에 검증 로직 추가
4. **커스텀 Function Calling**: `tools` 정의 확장

### 9.3 OpenVLA 통합 경로
```
Phase 1: GPT-4 Function Calling (현재 설계)
    ↓
Phase 2: 하이브리드 (GPT-4 + OpenVLA)
    - 고수준 계획: GPT-4
    - 저수준 제어: OpenVLA
    ↓
Phase 3: 순수 OpenVLA (선택)
    - 종단간 비전-언어-액션
```

---

## 10. 변경 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|----------|--------|
| 1.0 | 2025-12-14 | 초기 아키텍처 설계 | Claude Code |

---

**문서 끝**
