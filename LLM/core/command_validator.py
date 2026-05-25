# Command Validator for Robot Control

import logging
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass

from .robot_command import RobotCommand, CommandType

logger = logging.getLogger(__name__)


@dataclass
class ValidationResult:
    """Result of command validation"""
    is_valid: bool
    errors: List[str]
    warnings: List[str]
    suggested_correction: Optional[Dict[str, Any]] = None

    @property
    def has_warnings(self) -> bool:
        return len(self.warnings) > 0


class CommandValidator:
    """Validator for robot commands"""

    def __init__(self, config: Dict[str, Any]):
        self.config = config

        # Workspace bounds
        workspace = config.get("workspace", {})
        bounds = workspace.get("bounds", {})
        self.min_bounds = bounds.get("min", [-1.0, -1.0, 0.0])
        self.max_bounds = bounds.get("max", [1.0, 1.0, 1.5])

        # Safety config
        safety = workspace.get("safety", {})
        self.workspace_margin = safety.get("workspace_margin", 0.05)

        # Velocity limits
        velocity_limits = workspace.get("velocity_limits", {})
        manip_limits = velocity_limits.get("manipulator", {})
        self.max_linear_velocity = manip_limits.get("max_linear", 0.5)
        self.max_angular_velocity = manip_limits.get("max_angular", 1.0)

        base_limits = velocity_limits.get("base", {})
        self.max_base_linear = base_limits.get("max_linear", 1.0)
        self.max_base_angular = base_limits.get("max_angular", 1.5)

    def validate(
        self,
        command: RobotCommand,
        current_position: Optional[Tuple[float, float, float]] = None
    ) -> ValidationResult:
        """Validate a robot command

        Args:
            command: Command to validate
            current_position: Current end-effector position

        Returns:
            ValidationResult with errors and warnings
        """
        errors = []
        warnings = []
        suggested_correction = None

        # Validate based on command type
        if command.command_type == CommandType.MOVE_MANIPULATOR:
            result = self._validate_move_manipulator(
                command.parameters, current_position
            )
            errors.extend(result.get("errors", []))
            warnings.extend(result.get("warnings", []))
            if result.get("suggested_correction"):
                suggested_correction = result["suggested_correction"]

        elif command.command_type == CommandType.MOVE_MOBILE_BASE:
            result = self._validate_move_mobile_base(command.parameters)
            errors.extend(result.get("errors", []))
            warnings.extend(result.get("warnings", []))

        elif command.command_type == CommandType.CONTROL_GRIPPER:
            result = self._validate_control_gripper(command.parameters)
            errors.extend(result.get("errors", []))
            warnings.extend(result.get("warnings", []))

        # STOP_ROBOT always valid

        return ValidationResult(
            is_valid=len(errors) == 0,
            errors=errors,
            warnings=warnings,
            suggested_correction=suggested_correction
        )

    def _validate_move_manipulator(
        self,
        params: Dict[str, Any],
        current_position: Optional[Tuple[float, float, float]]
    ) -> Dict[str, Any]:
        """Validate move_manipulator parameters"""
        result = {"errors": [], "warnings": []}

        movement_type = params.get("movement_type")
        speed = params.get("speed", 1.0)

        # Speed validation
        if speed > 1.5:
            result["warnings"].append(f"High speed ({speed}x) - be cautious")

        if movement_type == "relative":
            direction = params.get("direction")
            distance = params.get("distance", 10.0)

            # Distance validation
            if distance > 50.0:
                result["warnings"].append(
                    f"Large movement distance ({distance}cm) - verify target"
                )

            # Workspace boundary check if position known
            if current_position:
                target = self._calculate_target_position(
                    current_position, direction, distance
                )
                bounds_result = self._check_workspace_bounds(target)
                result["errors"].extend(bounds_result.get("errors", []))
                result["warnings"].extend(bounds_result.get("warnings", []))

                if bounds_result.get("suggested_position"):
                    result["suggested_correction"] = {
                        "type": "position",
                        "suggested": bounds_result["suggested_position"]
                    }

        elif movement_type == "absolute":
            position = params.get("position", {})
            target = (position.get("x", 0), position.get("y", 0), position.get("z", 0))

            bounds_result = self._check_workspace_bounds(target)
            result["errors"].extend(bounds_result.get("errors", []))
            result["warnings"].extend(bounds_result.get("warnings", []))

            if bounds_result.get("suggested_position"):
                result["suggested_correction"] = {
                    "type": "position",
                    "suggested": bounds_result["suggested_position"]
                }

        return result

    def _validate_move_mobile_base(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Validate move_mobile_base parameters"""
        result = {"errors": [], "warnings": []}

        linear_velocity = params.get("linear_velocity", 0.0)
        angular_velocity = params.get("angular_velocity", 0.0)
        duration = params.get("duration", 2.0)

        # Velocity checks
        if abs(linear_velocity) > self.max_base_linear:
            result["errors"].append(
                f"Linear velocity {linear_velocity} exceeds limit {self.max_base_linear}"
            )

        if abs(angular_velocity) > self.max_base_angular:
            result["errors"].append(
                f"Angular velocity {angular_velocity} exceeds limit {self.max_base_angular}"
            )

        # Duration warning
        if duration > 5.0:
            result["warnings"].append(
                f"Long duration ({duration}s) - monitor movement"
            )

        return result

    def _validate_control_gripper(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Validate control_gripper parameters"""
        result = {"errors": [], "warnings": []}

        action = params.get("action")
        if action not in ["open", "close"]:
            result["errors"].append(f"Invalid gripper action: {action}")

        return result

    def _calculate_target_position(
        self,
        current: Tuple[float, float, float],
        direction: str,
        distance_cm: float
    ) -> Tuple[float, float, float]:
        """Calculate target position from relative movement"""
        distance_m = distance_cm / 100.0
        x, y, z = current

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

    def _check_workspace_bounds(
        self,
        position: Tuple[float, float, float]
    ) -> Dict[str, Any]:
        """Check if position is within workspace bounds"""
        result = {"errors": [], "warnings": []}
        x, y, z = position

        # Inner bounds with margin
        inner_min = [b + self.workspace_margin for b in self.min_bounds]
        inner_max = [b - self.workspace_margin for b in self.max_bounds]

        # Check bounds
        violations = []
        if x < inner_min[0]:
            violations.append(f"X ({x:.3f}) below minimum ({inner_min[0]})")
        if x > inner_max[0]:
            violations.append(f"X ({x:.3f}) above maximum ({inner_max[0]})")
        if y < inner_min[1]:
            violations.append(f"Y ({y:.3f}) below minimum ({inner_min[1]})")
        if y > inner_max[1]:
            violations.append(f"Y ({y:.3f}) above maximum ({inner_max[1]})")
        if z < inner_min[2]:
            violations.append(f"Z ({z:.3f}) below minimum ({inner_min[2]})")
        if z > inner_max[2]:
            violations.append(f"Z ({z:.3f}) above maximum ({inner_max[2]})")

        if violations:
            result["errors"].append(
                f"Target position outside workspace: {', '.join(violations)}"
            )
            # Calculate nearest valid position
            clamped = (
                max(inner_min[0], min(inner_max[0], x)),
                max(inner_min[1], min(inner_max[1], y)),
                max(inner_min[2], min(inner_max[2], z))
            )
            result["suggested_position"] = list(clamped)

        # Warning if close to boundary
        warning_distance = 0.1
        for i, (val, min_b, max_b) in enumerate(zip(
            [x, y, z], inner_min, inner_max
        )):
            if val - min_b < warning_distance or max_b - val < warning_distance:
                axis = ["X", "Y", "Z"][i]
                result["warnings"].append(
                    f"Position close to {axis} boundary"
                )

        return result

    def validate_trajectory(
        self,
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        num_samples: int = 10
    ) -> ValidationResult:
        """Validate a trajectory between two points"""
        errors = []
        warnings = []

        for i in range(num_samples):
            t = i / (num_samples - 1)
            point = (
                start[0] + t * (end[0] - start[0]),
                start[1] + t * (end[1] - start[1]),
                start[2] + t * (end[2] - start[2])
            )

            bounds_result = self._check_workspace_bounds(point)

            if bounds_result.get("errors"):
                errors.append(
                    f"Trajectory point {i} violates bounds: {point}"
                )
                break

        return ValidationResult(
            is_valid=len(errors) == 0,
            errors=errors,
            warnings=warnings
        )
