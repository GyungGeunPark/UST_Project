# Isaac Sim LLM Robot Control - Web UI Implementation Guide

## 1. 문서 개요

### 1.1 목적
본 문서는 FastAPI 기반 웹 서버와 실시간 UI의 상세 구현 가이드를 제공합니다. REST API, WebSocket, 프론트엔드 구현을 포함합니다.

### 1.2 범위
- FastAPI 서버 구현
- REST API 엔드포인트
- WebSocket 실시간 통신
- HTML/CSS/JavaScript 프론트엔드
- 상태 모니터링 대시보드

---

## 2. 서버 아키텍처

### 2.1 전체 구조

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Web Server Layer                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    FastAPI Application                       │   │
│  │                                                              │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │   │
│  │  │ REST Router  │  │ WebSocket    │  │ Static Files     │  │   │
│  │  │              │  │ Manager      │  │ Handler          │  │   │
│  │  │ /api/*       │  │ /ws          │  │ /static/*        │  │   │
│  │  └──────┬───────┘  └──────┬───────┘  └──────────────────┘  │   │
│  │         │                 │                                 │   │
│  │         └────────┬────────┘                                 │   │
│  │                  │                                          │   │
│  │         ┌────────▼────────┐                                 │   │
│  │         │ Request Handler │                                 │   │
│  │         │                 │                                 │   │
│  │         │ - Authentication│                                 │   │
│  │         │ - Rate Limiting │                                 │   │
│  │         │ - CORS          │                                 │   │
│  │         └────────┬────────┘                                 │   │
│  │                  │                                          │   │
│  └──────────────────┼──────────────────────────────────────────┘   │
│                     │                                               │
│          ┌──────────▼──────────┐                                   │
│          │ Control Manager     │                                   │
│          │ Interface           │                                   │
│          └─────────────────────┘                                   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. FastAPI 서버 구현

### 3.1 메인 서버 모듈

```python
# web/server.py

import asyncio
from typing import Optional, Dict, List
from contextlib import asynccontextmanager
import logging

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
import uvicorn

logger = logging.getLogger(__name__)

# 전역 상태 (애플리케이션 수준)
app_state: Dict = {
    "control_manager": None,
    "websocket_clients": [],
    "status_broadcast_task": None
}


@asynccontextmanager
async def lifespan(app: FastAPI):
    """애플리케이션 수명주기 관리"""
    logger.info("Starting web server...")

    # 상태 브로드캐스트 태스크 시작
    app_state["status_broadcast_task"] = asyncio.create_task(
        broadcast_status_loop()
    )

    yield

    # 정리
    logger.info("Shutting down web server...")
    if app_state["status_broadcast_task"]:
        app_state["status_broadcast_task"].cancel()

    # WebSocket 연결 종료
    for client in app_state["websocket_clients"]:
        await client.close()


# FastAPI 앱 생성
app = FastAPI(
    title="Isaac Sim LLM Robot Control API",
    description="자연어 기반 로봇 제어 시스템",
    version="1.0.0",
    lifespan=lifespan
)

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 프로덕션에서는 특정 도메인으로 제한
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def set_control_manager(manager):
    """컨트롤 매니저 설정"""
    app_state["control_manager"] = manager
    logger.info("Control manager connected to web server")


# ============================================================
# REST API Endpoints
# ============================================================

@app.get("/", response_class=HTMLResponse)
async def root():
    """메인 페이지"""
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <meta http-equiv="refresh" content="0; url=/static/index.html">
    </head>
    <body>
        <p>Redirecting to <a href="/static/index.html">control panel</a>...</p>
    </body>
    </html>
    """


@app.get("/api/health")
async def health_check():
    """헬스 체크"""
    return {
        "status": "healthy",
        "control_manager_connected": app_state["control_manager"] is not None,
        "websocket_clients": len(app_state["websocket_clients"])
    }


@app.post("/api/command")
async def process_command(request: dict):
    """자연어 명령 처리 API

    Request Body:
    {
        "command": "앞으로 10cm 이동해줘",
        "require_confirmation": false
    }
    """
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    command = request.get("command", "")
    if not command:
        raise HTTPException(status_code=400, detail="Command is required")

    try:
        result = await manager.process_command(command)
        return result.to_dict()
    except Exception as e:
        logger.error(f"Command processing error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/emergency_stop")
async def emergency_stop():
    """비상 정지 API"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    manager.emergency_stop()

    # 모든 WebSocket 클라이언트에게 알림
    await broadcast_message({
        "type": "emergency_stop",
        "timestamp": asyncio.get_event_loop().time()
    })

    return {"status": "emergency_stopped"}


@app.post("/api/reset")
async def reset():
    """비상 정지 해제 API"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    manager.reset()
    return {"status": "reset"}


@app.get("/api/status")
async def get_status():
    """로봇 상태 조회 API"""
    manager = app_state["control_manager"]
    if manager is None:
        return {
            "connected": False,
            "state": "disconnected"
        }

    status = manager.get_status()
    return {
        "connected": True,
        "state": status.state.value,
        "is_moving": status.is_moving,
        "emergency_stopped": status.emergency_stopped,
        "position": {
            "x": status.current_position[0],
            "y": status.current_position[1],
            "z": status.current_position[2]
        },
        "gripper_state": status.gripper_state,
        "last_error": status.last_error,
        "uptime": status.uptime,
        "command_count": status.command_count
    }


@app.get("/api/stats")
async def get_stats():
    """통계 정보 조회"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    status = manager.get_status()
    return {
        "total_commands": status.command_count,
        "uptime_seconds": status.uptime,
        "websocket_clients": len(app_state["websocket_clients"]),
        "cache_stats": manager._cache.get_stats() if hasattr(manager, '_cache') else {}
    }


@app.post("/api/clear_cache")
async def clear_cache():
    """명령 캐시 초기화"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    manager.clear_cache()
    return {"status": "cache_cleared"}


# ============================================================
# WebSocket Endpoints
# ============================================================

class WebSocketManager:
    """WebSocket 연결 관리자"""

    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        app_state["websocket_clients"].append(websocket)
        logger.info(f"WebSocket client connected. Total: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        if websocket in app_state["websocket_clients"]:
            app_state["websocket_clients"].remove(websocket)
        logger.info(f"WebSocket client disconnected. Total: {len(self.active_connections)}")

    async def broadcast(self, message: dict):
        """모든 클라이언트에게 메시지 브로드캐스트"""
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                disconnected.append(connection)

        for conn in disconnected:
            self.disconnect(conn)


ws_manager = WebSocketManager()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 엔드포인트"""
    await ws_manager.connect(websocket)

    try:
        while True:
            # 클라이언트로부터 메시지 수신
            data = await websocket.receive_json()

            if data.get("type") == "ping":
                await websocket.send_json({"type": "pong"})

            elif data.get("type") == "command":
                # WebSocket을 통한 명령 처리
                command = data.get("command", "")
                manager = app_state["control_manager"]

                if manager:
                    result = await manager.process_command(command)
                    await websocket.send_json({
                        "type": "command_result",
                        "data": result.to_dict()
                    })

    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        ws_manager.disconnect(websocket)


async def broadcast_message(message: dict):
    """전역 브로드캐스트"""
    await ws_manager.broadcast(message)


async def broadcast_status_loop():
    """상태 브로드캐스트 루프 (10Hz)"""
    while True:
        try:
            manager = app_state["control_manager"]
            if manager and ws_manager.active_connections:
                status = manager.get_status()
                await ws_manager.broadcast({
                    "type": "status",
                    "data": {
                        "state": status.state.value,
                        "is_moving": status.is_moving,
                        "emergency_stopped": status.emergency_stopped,
                        "position": status.current_position,
                        "timestamp": asyncio.get_event_loop().time()
                    }
                })
            await asyncio.sleep(0.1)  # 10Hz

        except asyncio.CancelledError:
            break
        except Exception as e:
            logger.error(f"Status broadcast error: {e}")
            await asyncio.sleep(1)


# ============================================================
# Static Files
# ============================================================

# 정적 파일 마운트 (HTML, CSS, JS)
import os
static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    app.mount("/static", StaticFiles(directory=static_dir), name="static")


# ============================================================
# Server Startup
# ============================================================

def start_server(control_manager=None, config: dict = None):
    """서버 시작"""
    if control_manager:
        set_control_manager(control_manager)

    config = config or {}
    host = config.get("host", "0.0.0.0")
    port = config.get("port", 8000)

    uvicorn.run(
        app,
        host=host,
        port=port,
        log_level="info"
    )


def run_server_async(control_manager=None, config: dict = None):
    """비동기 서버 실행 (기존 이벤트 루프에서)"""
    if control_manager:
        set_control_manager(control_manager)

    config = config or {}
    host = config.get("host", "0.0.0.0")
    port = config.get("port", 8000)

    uvconfig = uvicorn.Config(
        app,
        host=host,
        port=port,
        log_level="info"
    )
    server = uvicorn.Server(uvconfig)
    return server.serve()


if __name__ == "__main__":
    start_server()
```

### 3.2 API 라우터 분리 (선택적)

```python
# web/api/routes.py

from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
from pydantic import BaseModel, Field

router = APIRouter(prefix="/api", tags=["Robot Control"])


# Request/Response Models
class CommandRequest(BaseModel):
    """명령 요청"""
    command: str = Field(..., description="자연어 명령", min_length=1, max_length=500)
    require_confirmation: bool = Field(False, description="실행 전 확인 요청")


class CommandResponse(BaseModel):
    """명령 응답"""
    success: bool
    command_id: str
    message: str
    error_code: Optional[str] = None
    execution_time: Optional[float] = None


class StatusResponse(BaseModel):
    """상태 응답"""
    connected: bool
    state: str
    is_moving: bool
    emergency_stopped: bool
    position: dict
    gripper_state: str
    last_error: Optional[str]


class QuickCommandRequest(BaseModel):
    """빠른 명령 요청"""
    action: str = Field(..., description="명령 타입", pattern="^(forward|backward|left|right|up|down|grip_open|grip_close|stop)$")
    value: Optional[float] = Field(10.0, description="거리(cm) 또는 속도")


@router.post("/command", response_model=CommandResponse)
async def process_command(request: CommandRequest):
    """자연어 명령 처리"""
    # 실제 구현은 server.py에서 가져옴
    pass


@router.post("/quick_command", response_model=CommandResponse)
async def quick_command(request: QuickCommandRequest):
    """빠른 명령 (버튼용)"""
    action_map = {
        "forward": f"앞으로 {request.value}cm",
        "backward": f"뒤로 {request.value}cm",
        "left": f"왼쪽으로 {request.value}cm",
        "right": f"오른쪽으로 {request.value}cm",
        "up": f"위로 {request.value}cm",
        "down": f"아래로 {request.value}cm",
        "grip_open": "그리퍼 열어",
        "grip_close": "그리퍼 닫아",
        "stop": "정지"
    }

    command = action_map.get(request.action, "")
    # process_command 호출...
    pass


@router.get("/status", response_model=StatusResponse)
async def get_status():
    """상태 조회"""
    pass


@router.post("/emergency_stop")
async def emergency_stop():
    """비상 정지"""
    pass


@router.post("/reset")
async def reset():
    """리셋"""
    pass
```

---

## 4. 프론트엔드 구현

### 4.1 HTML 구조

```html
<!-- web/static/index.html -->
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Isaac Sim Robot Control</title>
    <link rel="stylesheet" href="css/style.css">
</head>
<body>
    <div class="container">
        <!-- 헤더 -->
        <header class="header">
            <h1>🤖 Isaac Sim Robot Control</h1>
            <div class="connection-status" id="connection-status">
                <span class="status-dot"></span>
                <span class="status-text">연결 중...</span>
            </div>
        </header>

        <!-- 메인 컨텐츠 -->
        <main class="main-content">
            <!-- 왼쪽: 제어 패널 -->
            <section class="control-panel">
                <!-- 텍스트 입력 -->
                <div class="input-section">
                    <h2>🎤 음성/텍스트 명령</h2>
                    <div class="command-input-wrapper">
                        <input type="text"
                               id="command-input"
                               placeholder="명령을 입력하세요 (예: 앞으로 10cm)"
                               autocomplete="off">
                        <button id="send-btn" class="primary-btn">전송</button>
                    </div>
                    <div class="quick-commands">
                        <span class="hint">빠른 명령:</span>
                        <button class="quick-cmd" data-cmd="앞으로 10cm">앞으로</button>
                        <button class="quick-cmd" data-cmd="뒤로 10cm">뒤로</button>
                        <button class="quick-cmd" data-cmd="그리퍼 열어">열기</button>
                        <button class="quick-cmd" data-cmd="그리퍼 닫아">닫기</button>
                    </div>
                </div>

                <!-- 방향 버튼 -->
                <div class="direction-section">
                    <h2>🎮 방향 제어</h2>
                    <div class="direction-pad">
                        <div class="row">
                            <button class="dir-btn" data-action="up">↑<br>위</button>
                        </div>
                        <div class="row">
                            <button class="dir-btn" data-action="left">←<br>좌</button>
                            <button class="dir-btn center" data-action="stop">⏹<br>정지</button>
                            <button class="dir-btn" data-action="right">→<br>우</button>
                        </div>
                        <div class="row">
                            <button class="dir-btn" data-action="down">↓<br>아래</button>
                        </div>
                    </div>
                    <div class="forward-backward">
                        <button class="dir-btn wide" data-action="forward">⬆ 전진</button>
                        <button class="dir-btn wide" data-action="backward">⬇ 후진</button>
                    </div>
                    <div class="distance-slider">
                        <label>이동 거리: <span id="distance-value">10</span>cm</label>
                        <input type="range" id="distance-slider" min="1" max="50" value="10">
                    </div>
                </div>

                <!-- 그리퍼 제어 -->
                <div class="gripper-section">
                    <h2>✋ 그리퍼 제어</h2>
                    <div class="gripper-controls">
                        <button class="gripper-btn open" data-action="grip_open">
                            <span class="icon">🖐</span>
                            <span>열기</span>
                        </button>
                        <button class="gripper-btn close" data-action="grip_close">
                            <span class="icon">✊</span>
                            <span>닫기</span>
                        </button>
                    </div>
                </div>

                <!-- 비상 정지 -->
                <div class="emergency-section">
                    <button id="emergency-btn" class="emergency-btn">
                        🛑 비상 정지
                    </button>
                    <button id="reset-btn" class="reset-btn" disabled>
                        🔄 리셋
                    </button>
                </div>
            </section>

            <!-- 오른쪽: 상태 및 로그 -->
            <section class="status-panel">
                <!-- 로봇 상태 -->
                <div class="robot-status">
                    <h2>📊 로봇 상태</h2>
                    <div class="status-grid">
                        <div class="status-item">
                            <span class="label">상태</span>
                            <span class="value" id="robot-state">-</span>
                        </div>
                        <div class="status-item">
                            <span class="label">위치 X</span>
                            <span class="value" id="pos-x">-</span>
                        </div>
                        <div class="status-item">
                            <span class="label">위치 Y</span>
                            <span class="value" id="pos-y">-</span>
                        </div>
                        <div class="status-item">
                            <span class="label">위치 Z</span>
                            <span class="value" id="pos-z">-</span>
                        </div>
                        <div class="status-item">
                            <span class="label">그리퍼</span>
                            <span class="value" id="gripper-state">-</span>
                        </div>
                        <div class="status-item">
                            <span class="label">명령 수</span>
                            <span class="value" id="command-count">0</span>
                        </div>
                    </div>
                </div>

                <!-- 명령 로그 -->
                <div class="command-log">
                    <h2>📝 명령 로그</h2>
                    <div class="log-container" id="log-container">
                        <!-- 로그 항목들이 여기에 추가됨 -->
                    </div>
                </div>
            </section>
        </main>

        <!-- 푸터 -->
        <footer class="footer">
            <span>Isaac Sim LLM Robot Control v1.0</span>
            <span id="uptime">가동시간: --:--:--</span>
        </footer>
    </div>

    <script src="js/main.js"></script>
</body>
</html>
```

### 4.2 CSS 스타일

```css
/* web/static/css/style.css */

:root {
    --primary-color: #2196F3;
    --primary-dark: #1976D2;
    --success-color: #4CAF50;
    --danger-color: #f44336;
    --warning-color: #FF9800;
    --bg-color: #1a1a2e;
    --card-bg: #16213e;
    --text-color: #eee;
    --text-secondary: #aaa;
    --border-radius: 8px;
}

* {
    box-sizing: border-box;
    margin: 0;
    padding: 0;
}

body {
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background-color: var(--bg-color);
    color: var(--text-color);
    min-height: 100vh;
}

.container {
    max-width: 1400px;
    margin: 0 auto;
    padding: 20px;
}

/* Header */
.header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 15px 20px;
    background: var(--card-bg);
    border-radius: var(--border-radius);
    margin-bottom: 20px;
}

.header h1 {
    font-size: 1.5rem;
}

.connection-status {
    display: flex;
    align-items: center;
    gap: 8px;
}

.status-dot {
    width: 12px;
    height: 12px;
    border-radius: 50%;
    background: var(--warning-color);
}

.status-dot.connected {
    background: var(--success-color);
}

.status-dot.disconnected {
    background: var(--danger-color);
}

/* Main Content */
.main-content {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 20px;
}

@media (max-width: 900px) {
    .main-content {
        grid-template-columns: 1fr;
    }
}

/* Panels */
.control-panel, .status-panel {
    display: flex;
    flex-direction: column;
    gap: 20px;
}

.control-panel > div, .status-panel > div {
    background: var(--card-bg);
    padding: 20px;
    border-radius: var(--border-radius);
}

h2 {
    font-size: 1.1rem;
    margin-bottom: 15px;
    color: var(--text-color);
}

/* Input Section */
.command-input-wrapper {
    display: flex;
    gap: 10px;
    margin-bottom: 10px;
}

#command-input {
    flex: 1;
    padding: 12px 15px;
    border: 1px solid #444;
    border-radius: var(--border-radius);
    background: #0f0f1a;
    color: var(--text-color);
    font-size: 1rem;
}

#command-input:focus {
    outline: none;
    border-color: var(--primary-color);
}

.primary-btn {
    padding: 12px 25px;
    background: var(--primary-color);
    color: white;
    border: none;
    border-radius: var(--border-radius);
    cursor: pointer;
    font-weight: bold;
    transition: background 0.2s;
}

.primary-btn:hover {
    background: var(--primary-dark);
}

.primary-btn:disabled {
    background: #555;
    cursor: not-allowed;
}

.quick-commands {
    display: flex;
    gap: 8px;
    align-items: center;
    flex-wrap: wrap;
}

.hint {
    color: var(--text-secondary);
    font-size: 0.9rem;
}

.quick-cmd {
    padding: 6px 12px;
    background: #333;
    border: 1px solid #444;
    color: var(--text-color);
    border-radius: 4px;
    cursor: pointer;
    font-size: 0.85rem;
}

.quick-cmd:hover {
    background: #444;
}

/* Direction Pad */
.direction-pad {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 5px;
    margin-bottom: 15px;
}

.direction-pad .row {
    display: flex;
    gap: 5px;
}

.dir-btn {
    width: 70px;
    height: 70px;
    background: #333;
    border: 2px solid #444;
    color: var(--text-color);
    border-radius: var(--border-radius);
    cursor: pointer;
    font-size: 0.9rem;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    transition: all 0.2s;
}

.dir-btn:hover {
    background: #444;
    border-color: var(--primary-color);
}

.dir-btn:active {
    background: var(--primary-color);
}

.dir-btn.center {
    background: var(--danger-color);
}

.dir-btn.center:hover {
    background: #d32f2f;
}

.dir-btn.wide {
    width: 150px;
    height: 50px;
}

.forward-backward {
    display: flex;
    gap: 10px;
    justify-content: center;
    margin-bottom: 15px;
}

.distance-slider {
    text-align: center;
}

.distance-slider input[type="range"] {
    width: 100%;
    margin-top: 10px;
}

/* Gripper Controls */
.gripper-controls {
    display: flex;
    gap: 15px;
    justify-content: center;
}

.gripper-btn {
    flex: 1;
    padding: 20px;
    background: #333;
    border: 2px solid #444;
    color: var(--text-color);
    border-radius: var(--border-radius);
    cursor: pointer;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
    transition: all 0.2s;
}

.gripper-btn .icon {
    font-size: 2rem;
}

.gripper-btn:hover {
    border-color: var(--primary-color);
    background: #444;
}

.gripper-btn.open:active {
    background: var(--success-color);
}

.gripper-btn.close:active {
    background: var(--warning-color);
}

/* Emergency Section */
.emergency-section {
    display: flex;
    gap: 15px;
}

.emergency-btn {
    flex: 2;
    padding: 20px;
    background: var(--danger-color);
    color: white;
    border: none;
    border-radius: var(--border-radius);
    font-size: 1.2rem;
    font-weight: bold;
    cursor: pointer;
    transition: all 0.2s;
}

.emergency-btn:hover {
    background: #d32f2f;
    transform: scale(1.02);
}

.emergency-btn.active {
    animation: pulse 1s infinite;
}

@keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.7; }
}

.reset-btn {
    flex: 1;
    padding: 20px;
    background: #333;
    color: var(--text-color);
    border: 2px solid #444;
    border-radius: var(--border-radius);
    font-size: 1rem;
    cursor: pointer;
}

.reset-btn:disabled {
    opacity: 0.5;
    cursor: not-allowed;
}

.reset-btn:not(:disabled):hover {
    background: #444;
    border-color: var(--success-color);
}

/* Status Grid */
.status-grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 15px;
}

.status-item {
    background: #0f0f1a;
    padding: 15px;
    border-radius: var(--border-radius);
}

.status-item .label {
    display: block;
    font-size: 0.85rem;
    color: var(--text-secondary);
    margin-bottom: 5px;
}

.status-item .value {
    font-size: 1.2rem;
    font-weight: bold;
    font-family: monospace;
}

/* Command Log */
.log-container {
    height: 300px;
    overflow-y: auto;
    background: #0f0f1a;
    border-radius: var(--border-radius);
    padding: 10px;
}

.log-entry {
    padding: 8px 10px;
    margin-bottom: 5px;
    border-radius: 4px;
    font-size: 0.9rem;
    border-left: 3px solid #444;
}

.log-entry.success {
    border-left-color: var(--success-color);
    background: rgba(76, 175, 80, 0.1);
}

.log-entry.error {
    border-left-color: var(--danger-color);
    background: rgba(244, 67, 54, 0.1);
}

.log-entry.pending {
    border-left-color: var(--warning-color);
    background: rgba(255, 152, 0, 0.1);
}

.log-entry .time {
    font-size: 0.75rem;
    color: var(--text-secondary);
}

.log-entry .message {
    margin-top: 3px;
}

/* Footer */
.footer {
    display: flex;
    justify-content: space-between;
    padding: 15px 20px;
    margin-top: 20px;
    background: var(--card-bg);
    border-radius: var(--border-radius);
    font-size: 0.9rem;
    color: var(--text-secondary);
}
```

### 4.3 JavaScript 로직

```javascript
// web/static/js/main.js

class RobotControlUI {
    constructor() {
        // WebSocket
        this.ws = null;
        this.wsRetryCount = 0;
        this.maxRetries = 5;

        // 상태
        this.isConnected = false;
        this.isEmergencyStopped = false;
        this.currentDistance = 10;

        // DOM 요소
        this.elements = {
            connectionStatus: document.getElementById('connection-status'),
            statusDot: document.querySelector('.status-dot'),
            statusText: document.querySelector('.status-text'),
            commandInput: document.getElementById('command-input'),
            sendBtn: document.getElementById('send-btn'),
            emergencyBtn: document.getElementById('emergency-btn'),
            resetBtn: document.getElementById('reset-btn'),
            distanceSlider: document.getElementById('distance-slider'),
            distanceValue: document.getElementById('distance-value'),
            logContainer: document.getElementById('log-container'),
            // 상태 표시
            robotState: document.getElementById('robot-state'),
            posX: document.getElementById('pos-x'),
            posY: document.getElementById('pos-y'),
            posZ: document.getElementById('pos-z'),
            gripperState: document.getElementById('gripper-state'),
            commandCount: document.getElementById('command-count'),
            uptime: document.getElementById('uptime')
        };

        this.init();
    }

    init() {
        this.connectWebSocket();
        this.bindEvents();
        this.startStatusPolling();
    }

    // =====================================================
    // WebSocket
    // =====================================================

    connectWebSocket() {
        const wsUrl = `ws://${window.location.host}/ws`;

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                console.log('WebSocket connected');
                this.setConnectionStatus(true);
                this.wsRetryCount = 0;
            };

            this.ws.onclose = () => {
                console.log('WebSocket disconnected');
                this.setConnectionStatus(false);
                this.scheduleReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };

            this.ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                this.handleWebSocketMessage(data);
            };

        } catch (error) {
            console.error('Failed to connect WebSocket:', error);
            this.scheduleReconnect();
        }
    }

    scheduleReconnect() {
        if (this.wsRetryCount < this.maxRetries) {
            this.wsRetryCount++;
            const delay = Math.min(1000 * Math.pow(2, this.wsRetryCount), 30000);
            console.log(`Reconnecting in ${delay}ms...`);
            setTimeout(() => this.connectWebSocket(), delay);
        }
    }

    handleWebSocketMessage(data) {
        switch (data.type) {
            case 'status':
                this.updateStatus(data.data);
                break;
            case 'command_result':
                this.handleCommandResult(data.data);
                break;
            case 'emergency_stop':
                this.handleEmergencyStop();
                break;
            case 'pong':
                // 핑-퐁 응답
                break;
        }
    }

    // =====================================================
    // 이벤트 바인딩
    // =====================================================

    bindEvents() {
        // 명령 전송
        this.elements.sendBtn.addEventListener('click', () => this.sendCommand());
        this.elements.commandInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') this.sendCommand();
        });

        // 빠른 명령 버튼
        document.querySelectorAll('.quick-cmd').forEach(btn => {
            btn.addEventListener('click', () => {
                const cmd = btn.dataset.cmd;
                this.elements.commandInput.value = cmd;
                this.sendCommand();
            });
        });

        // 방향 버튼
        document.querySelectorAll('.dir-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const action = btn.dataset.action;
                this.sendDirectionCommand(action);
            });
        });

        // 그리퍼 버튼
        document.querySelectorAll('.gripper-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const action = btn.dataset.action;
                this.sendGripperCommand(action);
            });
        });

        // 거리 슬라이더
        this.elements.distanceSlider.addEventListener('input', (e) => {
            this.currentDistance = parseInt(e.target.value);
            this.elements.distanceValue.textContent = this.currentDistance;
        });

        // 비상 정지
        this.elements.emergencyBtn.addEventListener('click', () => this.emergencyStop());

        // 리셋
        this.elements.resetBtn.addEventListener('click', () => this.reset());
    }

    // =====================================================
    // 명령 전송
    // =====================================================

    async sendCommand() {
        const command = this.elements.commandInput.value.trim();
        if (!command) return;

        this.addLogEntry(command, 'pending', '전송 중...');
        this.elements.commandInput.value = '';
        this.elements.sendBtn.disabled = true;

        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command })
            });

            const result = await response.json();

            if (result.success) {
                this.addLogEntry(command, 'success', '완료');
            } else {
                this.addLogEntry(command, 'error', result.message || '실패');
            }

        } catch (error) {
            this.addLogEntry(command, 'error', `오류: ${error.message}`);
        } finally {
            this.elements.sendBtn.disabled = false;
        }
    }

    sendDirectionCommand(action) {
        const commands = {
            'up': `위로 ${this.currentDistance}cm`,
            'down': `아래로 ${this.currentDistance}cm`,
            'left': `왼쪽으로 ${this.currentDistance}cm`,
            'right': `오른쪽으로 ${this.currentDistance}cm`,
            'forward': `앞으로 ${this.currentDistance}cm`,
            'backward': `뒤로 ${this.currentDistance}cm`,
            'stop': '정지'
        };

        const cmd = commands[action];
        if (cmd) {
            this.elements.commandInput.value = cmd;
            this.sendCommand();
        }
    }

    sendGripperCommand(action) {
        const commands = {
            'grip_open': '그리퍼 열어',
            'grip_close': '그리퍼 닫아'
        };

        const cmd = commands[action];
        if (cmd) {
            this.elements.commandInput.value = cmd;
            this.sendCommand();
        }
    }

    async emergencyStop() {
        try {
            const response = await fetch('/api/emergency_stop', { method: 'POST' });
            const result = await response.json();

            if (result.status === 'emergency_stopped') {
                this.handleEmergencyStop();
            }
        } catch (error) {
            console.error('Emergency stop error:', error);
            this.addLogEntry('비상 정지', 'error', error.message);
        }
    }

    async reset() {
        try {
            const response = await fetch('/api/reset', { method: 'POST' });
            const result = await response.json();

            if (result.status === 'reset') {
                this.isEmergencyStopped = false;
                this.elements.emergencyBtn.classList.remove('active');
                this.elements.resetBtn.disabled = true;
                this.addLogEntry('시스템', 'success', '리셋 완료');
            }
        } catch (error) {
            console.error('Reset error:', error);
        }
    }

    // =====================================================
    // UI 업데이트
    // =====================================================

    setConnectionStatus(connected) {
        this.isConnected = connected;
        this.elements.statusDot.className = `status-dot ${connected ? 'connected' : 'disconnected'}`;
        this.elements.statusText.textContent = connected ? '연결됨' : '연결 끊김';
    }

    updateStatus(status) {
        // 상태 업데이트
        this.elements.robotState.textContent = this.translateState(status.state);

        // 위치 업데이트
        if (status.position) {
            this.elements.posX.textContent = status.position[0]?.toFixed(3) || '-';
            this.elements.posY.textContent = status.position[1]?.toFixed(3) || '-';
            this.elements.posZ.textContent = status.position[2]?.toFixed(3) || '-';
        }

        // 비상 정지 상태 확인
        if (status.emergency_stopped && !this.isEmergencyStopped) {
            this.handleEmergencyStop();
        }
    }

    translateState(state) {
        const translations = {
            'idle': '대기',
            'processing': '처리 중',
            'moving': '이동 중',
            'emergency_stopped': '비상 정지',
            'error': '오류'
        };
        return translations[state] || state;
    }

    handleCommandResult(result) {
        if (result.success) {
            this.addLogEntry(
                result.command_id?.substring(0, 8) || 'cmd',
                'success',
                result.message
            );
        } else {
            this.addLogEntry(
                result.command_id?.substring(0, 8) || 'cmd',
                'error',
                result.message
            );
        }
    }

    handleEmergencyStop() {
        this.isEmergencyStopped = true;
        this.elements.emergencyBtn.classList.add('active');
        this.elements.resetBtn.disabled = false;
        this.addLogEntry('시스템', 'error', '🛑 비상 정지 활성화');
    }

    addLogEntry(command, status, message) {
        const entry = document.createElement('div');
        entry.className = `log-entry ${status}`;

        const time = new Date().toLocaleTimeString();
        entry.innerHTML = `
            <div class="time">${time}</div>
            <div class="message"><strong>${command}</strong>: ${message}</div>
        `;

        this.elements.logContainer.insertBefore(entry, this.elements.logContainer.firstChild);

        // 최대 100개 로그 유지
        while (this.elements.logContainer.children.length > 100) {
            this.elements.logContainer.removeChild(this.elements.logContainer.lastChild);
        }
    }

    // =====================================================
    // 상태 폴링
    // =====================================================

    startStatusPolling() {
        setInterval(async () => {
            if (!this.isConnected) return;

            try {
                const response = await fetch('/api/status');
                const status = await response.json();

                if (status.connected) {
                    this.elements.gripperState.textContent = status.gripper_state || '-';
                    this.elements.commandCount.textContent = status.command_count || 0;

                    if (status.uptime) {
                        this.elements.uptime.textContent = `가동시간: ${this.formatUptime(status.uptime)}`;
                    }
                }
            } catch (error) {
                // 조용히 실패
            }
        }, 2000); // 2초마다 폴링
    }

    formatUptime(seconds) {
        const h = Math.floor(seconds / 3600);
        const m = Math.floor((seconds % 3600) / 60);
        const s = Math.floor(seconds % 60);
        return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }
}

// 앱 시작
document.addEventListener('DOMContentLoaded', () => {
    window.robotUI = new RobotControlUI();
});
```

---

## 5. 테스트

### 5.1 API 테스트

```python
# tests/test_web_api.py

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, AsyncMock

from web.server import app, set_control_manager


@pytest.fixture
def mock_manager():
    manager = Mock()
    manager.process_command = AsyncMock(return_value=Mock(
        to_dict=lambda: {"success": True, "command_id": "test", "message": "OK"}
    ))
    manager.get_status = Mock(return_value=Mock(
        state=Mock(value="idle"),
        is_moving=False,
        emergency_stopped=False,
        current_position=[0, 0, 0],
        gripper_state="open",
        last_error=None,
        uptime=100,
        command_count=10
    ))
    return manager


@pytest.fixture
def client(mock_manager):
    set_control_manager(mock_manager)
    return TestClient(app)


def test_health_check(client):
    response = client.get("/api/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"


def test_process_command(client, mock_manager):
    response = client.post("/api/command", json={"command": "앞으로 10cm"})
    assert response.status_code == 200
    assert response.json()["success"] == True


def test_process_command_empty(client):
    response = client.post("/api/command", json={"command": ""})
    assert response.status_code == 400


def test_get_status(client):
    response = client.get("/api/status")
    assert response.status_code == 200
    assert "state" in response.json()


def test_emergency_stop(client, mock_manager):
    response = client.post("/api/emergency_stop")
    assert response.status_code == 200
    assert response.json()["status"] == "emergency_stopped"
    mock_manager.emergency_stop.assert_called_once()
```

---

## 6. 변경 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|----------|--------|
| 1.0 | 2025-12-14 | 초기 작성 | Claude Code |

---

**문서 끝**
