# LLM Response Parser for Robot Control

import logging
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass

from .robot_command import (
    RobotCommand,
    CommandType,
    CommandStatus,
    MoveManipulatorParams,
    MoveMobileBaseParams,
    ControlGripperParams,
)

logger = logging.getLogger(__name__)


@dataclass
class ParseResult:
    """Result of parsing LLM response"""
    success: bool
    command: Optional[RobotCommand] = None
    error_message: str = ""
    error_code: str = ""


class LLMResponseParser:
    """Parser for LLM responses"""

    FUNCTION_MAP = {
        "move_manipulator": CommandType.MOVE_MANIPULATOR,
        "move_mobile_base": CommandType.MOVE_MOBILE_BASE,
        "control_gripper": CommandType.CONTROL_GRIPPER,
        "stop_robot": CommandType.STOP_ROBOT,
    }

    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}

    def parse(
        self,
        llm_response: Dict[str, Any],
        original_command: str
    ) -> ParseResult:
        """Parse LLM response into a RobotCommand

        Args:
            llm_response: Response from LLM client
            original_command: Original user command text

        Returns:
            ParseResult with parsed command or error
        """
        # Check if LLM call was successful
        if not llm_response.get("success", False):
            error_msg = llm_response.get("error", "Unknown LLM error")
            error_code = llm_response.get("error_code", "LLM_ERROR")
            return ParseResult(
                success=False,
                error_message=error_msg,
                error_code=error_code
            )

        # Get function name and parameters
        function_name = llm_response.get("function_name", "")
        parameters = llm_response.get("parameters", {})

        if not function_name:
            return ParseResult(
                success=False,
                error_message="No function name in LLM response",
                error_code="PARSE_ERROR"
            )

        # Map function to command type
        command_type = self.FUNCTION_MAP.get(function_name, CommandType.UNKNOWN)

        if command_type == CommandType.UNKNOWN:
            return ParseResult(
                success=False,
                error_message=f"Unknown function: {function_name}",
                error_code="PARSE_ERROR"
            )

        # Validate and normalize parameters
        try:
            validated_params = self._validate_parameters(command_type, parameters)
        except ValueError as e:
            return ParseResult(
                success=False,
                error_message=str(e),
                error_code="INVALID_PARAMETERS"
            )

        # Create command
        command = RobotCommand(
            original_text=original_command,
            command_type=command_type,
            status=CommandStatus.PENDING,
            function_name=function_name,
            parameters=validated_params
        )

        return ParseResult(success=True, command=command)

    def _validate_parameters(
        self,
        command_type: CommandType,
        parameters: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Validate and normalize parameters based on command type

        Args:
            command_type: Type of command
            parameters: Raw parameters from LLM

        Returns:
            Validated and normalized parameters

        Raises:
            ValueError: If parameters are invalid
        """
        if command_type == CommandType.MOVE_MANIPULATOR:
            return self._validate_move_manipulator(parameters)
        elif command_type == CommandType.MOVE_MOBILE_BASE:
            return self._validate_move_mobile_base(parameters)
        elif command_type == CommandType.CONTROL_GRIPPER:
            return self._validate_control_gripper(parameters)
        elif command_type == CommandType.STOP_ROBOT:
            return {}  # No parameters needed
        else:
            raise ValueError(f"Unknown command type: {command_type}")

    def _validate_move_manipulator(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Validate move_manipulator parameters"""
        movement_type = params.get("movement_type", "relative")

        if movement_type not in ["relative", "absolute"]:
            raise ValueError(f"Invalid movement_type: {movement_type}")

        result = {
            "movement_type": movement_type,
            "speed": self._clamp(params.get("speed", 1.0), 0.1, 2.0)
        }

        if movement_type == "relative":
            direction = params.get("direction")
            if direction not in ["forward", "backward", "left", "right", "up", "down"]:
                raise ValueError(f"Invalid direction for relative movement: {direction}")

            distance = params.get("distance", 10.0)
            if not isinstance(distance, (int, float)):
                raise ValueError(f"Invalid distance: {distance}")

            result["direction"] = direction
            result["distance"] = self._clamp(float(distance), 0.1, 100.0)

        elif movement_type == "absolute":
            position = params.get("position")
            if not position:
                raise ValueError("Position required for absolute movement")

            if not all(k in position for k in ["x", "y", "z"]):
                raise ValueError("Position must have x, y, z coordinates")

            result["position"] = {
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position["z"])
            }

        return result

    def _validate_move_mobile_base(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Validate move_mobile_base parameters"""
        linear_velocity = params.get("linear_velocity", 0.0)

        if not isinstance(linear_velocity, (int, float)):
            raise ValueError(f"Invalid linear_velocity: {linear_velocity}")

        return {
            "linear_velocity": self._clamp(float(linear_velocity), -1.0, 1.0),
            "angular_velocity": self._clamp(
                float(params.get("angular_velocity", 0.0)), -1.5, 1.5
            ),
            "duration": self._clamp(float(params.get("duration", 2.0)), 0.1, 10.0)
        }

    def _validate_control_gripper(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Validate control_gripper parameters"""
        action = params.get("action")

        if action not in ["open", "close"]:
            raise ValueError(f"Invalid gripper action: {action}")

        return {"action": action}

    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range"""
        return max(min_val, min(max_val, value))

    def extract_target_position(
        self,
        command: RobotCommand,
        current_position: Tuple[float, float, float]
    ) -> Optional[Tuple[float, float, float]]:
        """Extract target position from command

        Args:
            command: Robot command
            current_position: Current end-effector position (x, y, z)

        Returns:
            Target position or None if not applicable
        """
        if command.command_type != CommandType.MOVE_MANIPULATOR:
            return None

        params = command.parameters
        movement_type = params.get("movement_type", "relative")

        if movement_type == "absolute":
            pos = params.get("position", {})
            return (pos.get("x", 0), pos.get("y", 0), pos.get("z", 0))

        elif movement_type == "relative":
            direction = params.get("direction")
            distance_cm = params.get("distance", 10.0)
            distance_m = distance_cm / 100.0  # Convert cm to m

            x, y, z = current_position

            # Calculate new position based on direction
            direction_vectors = {
                "forward": (distance_m, 0, 0),
                "backward": (-distance_m, 0, 0),
                "left": (0, distance_m, 0),
                "right": (0, -distance_m, 0),
                "up": (0, 0, distance_m),
                "down": (0, 0, -distance_m),
            }

            delta = direction_vectors.get(direction, (0, 0, 0))
            return (x + delta[0], y + delta[1], z + delta[2])

        return None
