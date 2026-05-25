# Core data structures for robot commands

import uuid
import time
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, field
from enum import Enum


class CommandStatus(Enum):
    """Command execution status"""
    PENDING = "pending"
    PROCESSING = "processing"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class CommandType(Enum):
    """Command type classification"""
    MOVE_MANIPULATOR = "move_manipulator"
    MOVE_MOBILE_BASE = "move_mobile_base"
    CONTROL_GRIPPER = "control_gripper"
    STOP_ROBOT = "stop_robot"
    UNKNOWN = "unknown"


class MovementType(Enum):
    """Movement type for manipulator"""
    RELATIVE = "relative"
    ABSOLUTE = "absolute"


class Direction(Enum):
    """Movement direction"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"


class GripperAction(Enum):
    """Gripper action type"""
    OPEN = "open"
    CLOSE = "close"


@dataclass
class RobotCommand:
    """Robot command data structure"""

    command_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    original_text: str = ""
    command_type: CommandType = CommandType.UNKNOWN
    status: CommandStatus = CommandStatus.PENDING

    # Function call details
    function_name: str = ""
    parameters: Dict[str, Any] = field(default_factory=dict)

    # Timing
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    completed_at: Optional[float] = None

    # Result
    error_message: Optional[str] = None
    error_code: Optional[str] = None

    def start_execution(self):
        """Mark command as executing"""
        self.status = CommandStatus.EXECUTING
        self.started_at = time.time()

    def complete(self):
        """Mark command as completed"""
        self.status = CommandStatus.COMPLETED
        self.completed_at = time.time()

    def fail(self, error_message: str, error_code: str = "UNKNOWN_ERROR"):
        """Mark command as failed"""
        self.status = CommandStatus.FAILED
        self.completed_at = time.time()
        self.error_message = error_message
        self.error_code = error_code

    def cancel(self):
        """Mark command as cancelled"""
        self.status = CommandStatus.CANCELLED
        self.completed_at = time.time()

    @property
    def execution_time(self) -> Optional[float]:
        """Get execution time in seconds"""
        if self.started_at and self.completed_at:
            return self.completed_at - self.started_at
        return None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            "command_id": self.command_id,
            "original_text": self.original_text,
            "command_type": self.command_type.value,
            "status": self.status.value,
            "function_name": self.function_name,
            "parameters": self.parameters,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "completed_at": self.completed_at,
            "execution_time": self.execution_time,
            "error_message": self.error_message,
            "error_code": self.error_code,
        }


@dataclass
class CommandResult:
    """Command execution result"""

    success: bool
    command_id: str
    message: str
    error_code: Optional[str] = None
    execution_time: Optional[float] = None
    final_position: Optional[List[float]] = None
    suggested_position: Optional[List[float]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        result = {
            "success": self.success,
            "command_id": self.command_id,
            "message": self.message,
        }

        if self.error_code:
            result["error_code"] = self.error_code
        if self.execution_time is not None:
            result["execution_time"] = self.execution_time
        if self.final_position:
            result["final_position"] = {
                "x": self.final_position[0],
                "y": self.final_position[1],
                "z": self.final_position[2],
            }
        if self.suggested_position:
            result["suggested_position"] = {
                "x": self.suggested_position[0],
                "y": self.suggested_position[1],
                "z": self.suggested_position[2],
            }

        return result


@dataclass
class RobotState:
    """Robot state data structure"""

    state: CommandStatus = CommandStatus.PENDING
    is_moving: bool = False
    emergency_stopped: bool = False
    current_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    current_orientation: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    joint_positions: List[float] = field(default_factory=list)
    gripper_state: str = "open"
    last_command_id: Optional[str] = None
    last_error: Optional[str] = None
    uptime: float = 0.0
    command_count: int = 0


@dataclass
class MoveManipulatorParams:
    """Parameters for move_manipulator command"""
    movement_type: MovementType = MovementType.RELATIVE
    direction: Optional[Direction] = None
    distance: float = 10.0  # cm
    position: Optional[Dict[str, float]] = None  # for absolute movement
    speed: float = 1.0  # speed multiplier

    @classmethod
    def from_dict(cls, params: Dict[str, Any]) -> "MoveManipulatorParams":
        """Create from dictionary"""
        movement_type = MovementType(params.get("movement_type", "relative"))
        direction = None
        if "direction" in params:
            direction = Direction(params["direction"])

        return cls(
            movement_type=movement_type,
            direction=direction,
            distance=params.get("distance", 10.0),
            position=params.get("position"),
            speed=params.get("speed", 1.0),
        )


@dataclass
class MoveMobileBaseParams:
    """Parameters for move_mobile_base command"""
    linear_velocity: float = 0.0  # m/s
    angular_velocity: float = 0.0  # rad/s
    duration: float = 2.0  # seconds

    @classmethod
    def from_dict(cls, params: Dict[str, Any]) -> "MoveMobileBaseParams":
        """Create from dictionary"""
        return cls(
            linear_velocity=params.get("linear_velocity", 0.0),
            angular_velocity=params.get("angular_velocity", 0.0),
            duration=params.get("duration", 2.0),
        )


@dataclass
class ControlGripperParams:
    """Parameters for control_gripper command"""
    action: GripperAction = GripperAction.OPEN

    @classmethod
    def from_dict(cls, params: Dict[str, Any]) -> "ControlGripperParams":
        """Create from dictionary"""
        return cls(
            action=GripperAction(params.get("action", "open"))
        )
