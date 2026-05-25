# API Routes for Robot Control

from fastapi import APIRouter, HTTPException
from typing import Optional
from pydantic import BaseModel, Field

router = APIRouter(prefix="/api", tags=["Robot Control"])


# Request/Response Models

class CommandRequest(BaseModel):
    """Command request model"""
    command: str = Field(..., description="Natural language command", min_length=1, max_length=500)
    require_confirmation: bool = Field(False, description="Require confirmation before execution")


class QuickCommandRequest(BaseModel):
    """Quick command request model"""
    action: str = Field(
        ...,
        description="Command type",
        pattern="^(forward|backward|left|right|up|down|grip_open|grip_close|stop)$"
    )
    value: Optional[float] = Field(10.0, description="Distance (cm) or speed")


class CommandResponse(BaseModel):
    """Command response model"""
    success: bool
    command_id: str
    message: str
    error_code: Optional[str] = None
    execution_time: Optional[float] = None


class StatusResponse(BaseModel):
    """Status response model"""
    connected: bool
    state: str
    is_moving: bool
    emergency_stopped: bool
    position: dict
    gripper_state: str
    last_error: Optional[str]


class StatsResponse(BaseModel):
    """Statistics response model"""
    total_commands: int
    uptime_seconds: float
    websocket_clients: int
    llm_stats: dict
    cache_stats: dict


# Route handlers would be implemented in server.py
# This file provides the schema definitions for documentation
