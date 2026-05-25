# FastAPI Web Server for Isaac Sim LLM Robot Control

import asyncio
import os
import logging
from typing import Dict, List, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
import uvicorn

logger = logging.getLogger(__name__)

# Global state
app_state: Dict = {
    "control_manager": None,
    "websocket_clients": [],
    "status_broadcast_task": None
}


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager"""
    logger.info("Starting web server...")

    # Start status broadcast task
    app_state["status_broadcast_task"] = asyncio.create_task(
        broadcast_status_loop()
    )

    yield

    # Cleanup
    logger.info("Shutting down web server...")
    if app_state["status_broadcast_task"]:
        app_state["status_broadcast_task"].cancel()
        try:
            await app_state["status_broadcast_task"]
        except asyncio.CancelledError:
            pass

    # Close WebSocket connections
    for client in app_state["websocket_clients"]:
        try:
            await client.close()
        except Exception:
            pass


# Create FastAPI app
app = FastAPI(
    title="Isaac Sim LLM Robot Control API",
    description="Natural language robot control system",
    version="1.0.0",
    lifespan=lifespan
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def set_control_manager(manager):
    """Set the control manager

    Args:
        manager: ControlManager instance
    """
    app_state["control_manager"] = manager
    logger.info("Control manager connected to web server")


# ============================================================
# REST API Endpoints
# ============================================================

@app.get("/", response_class=HTMLResponse)
async def root():
    """Root redirect to control panel"""
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
    """Health check endpoint"""
    return {
        "status": "healthy",
        "control_manager_connected": app_state["control_manager"] is not None,
        "websocket_clients": len(app_state["websocket_clients"]),
        "version": "1.0.0"
    }


@app.post("/api/command")
async def process_command(request: dict):
    """Process natural language command

    Request Body:
    {
        "command": "Move forward 10cm",
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


@app.post("/api/quick_command")
async def quick_command(request: dict):
    """Process quick command (button-based)

    Request Body:
    {
        "action": "forward",
        "value": 10.0
    }
    """
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    action = request.get("action", "")
    value = request.get("value", 10.0)

    # Map action to natural language command
    action_map = {
        "forward": f"앞으로 {value}cm",
        "backward": f"뒤로 {value}cm",
        "left": f"왼쪽으로 {value}cm",
        "right": f"오른쪽으로 {value}cm",
        "up": f"위로 {value}cm",
        "down": f"아래로 {value}cm",
        "grip_open": "그리퍼 열어",
        "grip_close": "그리퍼 닫아",
        "stop": "정지"
    }

    command = action_map.get(action, "")
    if not command:
        raise HTTPException(status_code=400, detail=f"Unknown action: {action}")

    try:
        result = await manager.process_command(command)
        return result.to_dict()
    except Exception as e:
        logger.error(f"Quick command error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/emergency_stop")
async def emergency_stop():
    """Trigger emergency stop"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    manager.emergency_stop()

    # Notify all WebSocket clients
    await broadcast_message({
        "type": "emergency_stop",
        "data": {
            "reason": "user_triggered",
            "timestamp": asyncio.get_event_loop().time()
        }
    })

    return {"status": "emergency_stopped"}


@app.post("/api/reset")
async def reset():
    """Reset from emergency stop"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    manager.reset()
    return {"status": "reset"}


@app.get("/api/status")
async def get_status():
    """Get robot status"""
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
    """Get system statistics"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    stats = manager.get_stats()
    return {
        "total_commands": stats.get("command_count", 0),
        "uptime_seconds": stats.get("uptime", 0),
        "websocket_clients": len(app_state["websocket_clients"]),
        "llm_stats": stats.get("llm_stats", {}),
        "cache_stats": stats.get("cache_stats", {})
    }


@app.post("/api/clear_cache")
async def clear_cache():
    """Clear command cache"""
    manager = app_state["control_manager"]
    if manager is None:
        raise HTTPException(status_code=503, detail="Control manager not connected")

    manager.clear_cache()
    return {"status": "cache_cleared"}


# ============================================================
# WebSocket
# ============================================================

class WebSocketManager:
    """WebSocket connection manager"""

    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        app_state["websocket_clients"].append(websocket)
        logger.info(f"WebSocket connected. Total: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        if websocket in app_state["websocket_clients"]:
            app_state["websocket_clients"].remove(websocket)
        logger.info(f"WebSocket disconnected. Total: {len(self.active_connections)}")

    async def broadcast(self, message: dict):
        """Broadcast to all clients"""
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
    """WebSocket endpoint"""
    await ws_manager.connect(websocket)

    try:
        while True:
            data = await websocket.receive_json()

            if data.get("type") == "ping":
                await websocket.send_json({"type": "pong"})

            elif data.get("type") == "command":
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
    """Broadcast message to all clients"""
    await ws_manager.broadcast(message)


async def broadcast_status_loop():
    """Status broadcast loop (10Hz)"""
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

# Mount static files
static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    app.mount("/static", StaticFiles(directory=static_dir), name="static")


# ============================================================
# Server Startup
# ============================================================

def start_server(control_manager=None, config: dict = None):
    """Start the server (blocking)

    Args:
        control_manager: ControlManager instance
        config: Server configuration
    """
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
    """Get server coroutine for async execution

    Args:
        control_manager: ControlManager instance
        config: Server configuration

    Returns:
        Server coroutine
    """
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
