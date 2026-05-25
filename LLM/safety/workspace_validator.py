# Workspace Validator

import numpy as np
import logging
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class BoundaryViolationType(Enum):
    """Boundary violation types"""
    NONE = "none"
    X_MIN = "x_min"
    X_MAX = "x_max"
    Y_MIN = "y_min"
    Y_MAX = "y_max"
    Z_MIN = "z_min"
    Z_MAX = "z_max"
    MULTIPLE = "multiple"


@dataclass
class WorkspaceCheckResult:
    """Workspace check result"""
    is_valid: bool
    violation_type: BoundaryViolationType = BoundaryViolationType.NONE
    distance_to_boundary: float = float('inf')
    nearest_valid_point: Optional[np.ndarray] = None
    message: str = ""


class WorkspaceBounds:
    """Workspace boundary definition"""

    def __init__(self, config: Dict):
        """Initialize workspace bounds

        Args:
            config: Workspace configuration
        """
        bounds = config.get("bounds", {})
        self.min = np.array(bounds.get("min", [-1.0, -1.0, 0.0]))
        self.max = np.array(bounds.get("max", [1.0, 1.0, 1.5]))

        safety = config.get("safety", {})
        self.margin = safety.get("workspace_margin", 0.05)

        # Inner bounds with margin
        self.inner_min = self.min + self.margin
        self.inner_max = self.max - self.margin

        logger.info(f"Workspace bounds: {self.min} to {self.max} (margin: {self.margin})")

    def contains(self, point: np.ndarray, use_margin: bool = True) -> bool:
        """Check if point is within bounds

        Args:
            point: Point to check
            use_margin: Whether to use safety margin

        Returns:
            True if within bounds
        """
        if use_margin:
            min_b, max_b = self.inner_min, self.inner_max
        else:
            min_b, max_b = self.min, self.max

        return np.all(point >= min_b) and np.all(point <= max_b)

    def clamp(self, point: np.ndarray, use_margin: bool = True) -> np.ndarray:
        """Clamp point to bounds

        Args:
            point: Point to clamp
            use_margin: Whether to use safety margin

        Returns:
            Clamped point
        """
        if use_margin:
            return np.clip(point, self.inner_min, self.inner_max)
        return np.clip(point, self.min, self.max)

    def distance_to_boundary(self, point: np.ndarray) -> float:
        """Get distance to nearest boundary

        Args:
            point: Point to check

        Returns:
            Distance (negative if outside)
        """
        dist_to_min = point - self.inner_min
        dist_to_max = self.inner_max - point

        return min(np.min(dist_to_min), np.min(dist_to_max))

    def get_center(self) -> np.ndarray:
        """Get workspace center"""
        return (self.min + self.max) / 2

    def get_size(self) -> np.ndarray:
        """Get workspace size"""
        return self.max - self.min


class WorkspaceValidator:
    """Validator for workspace boundaries"""

    def __init__(self, config: Dict):
        """Initialize workspace validator

        Args:
            config: Workspace configuration
        """
        self.bounds = WorkspaceBounds(config)
        self.exclusion_zones: List[Dict] = config.get("exclusion_zones", [])
        self.warning_distance = config.get("warning_distance", 0.1)

    def check_point(self, point: np.ndarray) -> WorkspaceCheckResult:
        """Check if point is valid

        Args:
            point: Point to check

        Returns:
            WorkspaceCheckResult
        """
        point = np.asarray(point)

        # Check basic bounds
        if self.bounds.contains(point):
            distance = self.bounds.distance_to_boundary(point)

            if distance < self.warning_distance:
                return WorkspaceCheckResult(
                    is_valid=True,
                    distance_to_boundary=distance,
                    message=f"Warning: close to boundary ({distance:.3f}m)"
                )

            return WorkspaceCheckResult(
                is_valid=True,
                distance_to_boundary=distance
            )

        # Violation - determine type
        violation = self._determine_violation_type(point)
        nearest = self.bounds.clamp(point)
        distance = -np.linalg.norm(point - nearest)

        return WorkspaceCheckResult(
            is_valid=False,
            violation_type=violation,
            distance_to_boundary=distance,
            nearest_valid_point=nearest,
            message=f"Position {point} violates {violation.value} boundary"
        )

    def check_trajectory(
        self,
        start: np.ndarray,
        end: np.ndarray,
        num_samples: int = 10
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """Check if trajectory is valid

        Args:
            start: Start point
            end: End point
            num_samples: Number of samples to check

        Returns:
            (is_valid, first_violation_point)
        """
        start = np.asarray(start)
        end = np.asarray(end)

        for t in np.linspace(0, 1, num_samples):
            point = start + t * (end - start)
            result = self.check_point(point)

            if not result.is_valid:
                return False, point

        return True, None

    def suggest_safe_target(
        self,
        current: np.ndarray,
        desired: np.ndarray
    ) -> np.ndarray:
        """Suggest safe target position

        Args:
            current: Current position
            desired: Desired position

        Returns:
            Safe target position
        """
        current = np.asarray(current)
        desired = np.asarray(desired)

        if self.bounds.contains(desired):
            return desired

        # Clamp to bounds
        clamped = self.bounds.clamp(desired)

        # Move slightly inside from boundary
        direction = clamped - current
        distance = np.linalg.norm(direction)

        if distance < 0.01:
            return current

        direction = direction / distance
        safe_distance = max(0, distance - self.bounds.margin)

        return current + direction * safe_distance

    def _determine_violation_type(self, point: np.ndarray) -> BoundaryViolationType:
        """Determine violation type"""
        violations = []

        if point[0] < self.bounds.inner_min[0]:
            violations.append(BoundaryViolationType.X_MIN)
        if point[0] > self.bounds.inner_max[0]:
            violations.append(BoundaryViolationType.X_MAX)
        if point[1] < self.bounds.inner_min[1]:
            violations.append(BoundaryViolationType.Y_MIN)
        if point[1] > self.bounds.inner_max[1]:
            violations.append(BoundaryViolationType.Y_MAX)
        if point[2] < self.bounds.inner_min[2]:
            violations.append(BoundaryViolationType.Z_MIN)
        if point[2] > self.bounds.inner_max[2]:
            violations.append(BoundaryViolationType.Z_MAX)

        if len(violations) == 0:
            return BoundaryViolationType.NONE
        if len(violations) == 1:
            return violations[0]
        return BoundaryViolationType.MULTIPLE


class JointLimitsValidator:
    """Validator for joint limits"""

    def __init__(self, config: Dict):
        """Initialize joint limits validator

        Args:
            config: Robot configuration
        """
        joints = config.get("joints", {}).get("arm", {})

        self.lower_limits = np.array(joints.get("lower_limits", [-np.pi] * 6))
        self.upper_limits = np.array(joints.get("upper_limits", [np.pi] * 6))
        self.velocity_limits = np.array(joints.get("velocity_limits", [2.0] * 6))

    def check_positions(self, positions: np.ndarray) -> Tuple[bool, str]:
        """Check joint positions"""
        positions = np.asarray(positions)

        if np.any(positions < self.lower_limits):
            violated = np.where(positions < self.lower_limits)[0]
            return False, f"Joint(s) {violated} below lower limit"

        if np.any(positions > self.upper_limits):
            violated = np.where(positions > self.upper_limits)[0]
            return False, f"Joint(s) {violated} above upper limit"

        return True, ""

    def check_velocities(self, velocities: np.ndarray) -> Tuple[bool, str]:
        """Check joint velocities"""
        velocities = np.asarray(velocities)

        if np.any(np.abs(velocities) > self.velocity_limits):
            violated = np.where(np.abs(velocities) > self.velocity_limits)[0]
            return False, f"Joint(s) {violated} velocity exceeded"

        return True, ""

    def clamp_positions(self, positions: np.ndarray) -> np.ndarray:
        """Clamp positions to limits"""
        return np.clip(positions, self.lower_limits, self.upper_limits)

    def clamp_velocities(self, velocities: np.ndarray) -> np.ndarray:
        """Clamp velocities to limits"""
        return np.clip(velocities, -self.velocity_limits, self.velocity_limits)
