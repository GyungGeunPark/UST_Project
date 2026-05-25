# Collision Checker

import numpy as np
import logging
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class CollisionType(Enum):
    """Collision types"""
    NONE = "none"
    SELF_COLLISION = "self_collision"
    ENVIRONMENT = "environment"
    GROUND = "ground"
    OBSTACLE = "obstacle"


@dataclass
class CollisionResult:
    """Collision check result"""
    has_collision: bool
    collision_type: CollisionType = CollisionType.NONE
    collision_point: Optional[np.ndarray] = None
    distance: float = float('inf')
    link_pair: Optional[Tuple[str, str]] = None
    message: str = ""


class CollisionChecker:
    """Collision checking for robot safety"""

    def __init__(self, config: Dict):
        """Initialize collision checker

        Args:
            config: Workspace/safety configuration
        """
        self.config = config
        safety = config.get("safety", {})

        self.self_collision_enabled = safety.get("self_collision_check", True)
        self.env_collision_enabled = safety.get("environment_collision_check", True)
        self.min_distance = safety.get("collision_min_distance", 0.02)

        # Obstacles
        self.obstacles: List[Dict] = []

        logger.info("Collision checker initialized")

    def check_environment_collision(
        self,
        position: np.ndarray,
        radius: float = 0.05
    ) -> CollisionResult:
        """Check for environment collisions

        Args:
            position: Position to check
            radius: Collision sphere radius

        Returns:
            CollisionResult
        """
        if not self.env_collision_enabled:
            return CollisionResult(has_collision=False)

        # Ground collision
        if position[2] - radius < 0:
            return CollisionResult(
                has_collision=True,
                collision_type=CollisionType.GROUND,
                collision_point=np.array([position[0], position[1], 0]),
                distance=position[2] - radius,
                message="Ground collision detected"
            )

        # Obstacle collisions
        for obstacle in self.obstacles:
            result = self._check_obstacle_collision(position, radius, obstacle)
            if result.has_collision:
                return result

        return CollisionResult(has_collision=False)

    def check_trajectory_collision(
        self,
        start: np.ndarray,
        end: np.ndarray,
        radius: float = 0.05,
        num_samples: int = 20
    ) -> CollisionResult:
        """Check trajectory for collisions

        Args:
            start: Start position
            end: End position
            radius: Collision sphere radius
            num_samples: Number of samples

        Returns:
            CollisionResult for first collision
        """
        start = np.asarray(start)
        end = np.asarray(end)

        for t in np.linspace(0, 1, num_samples):
            point = start + t * (end - start)
            result = self.check_environment_collision(point, radius)

            if result.has_collision:
                return result

        return CollisionResult(has_collision=False)

    def _check_obstacle_collision(
        self,
        position: np.ndarray,
        radius: float,
        obstacle: Dict
    ) -> CollisionResult:
        """Check collision with obstacle"""
        obs_type = obstacle.get("type", "sphere")

        if obs_type == "sphere":
            return self._check_sphere_collision(position, radius, obstacle)
        elif obs_type == "box":
            return self._check_box_collision(position, radius, obstacle)

        return CollisionResult(has_collision=False)

    def _check_sphere_collision(
        self,
        position: np.ndarray,
        radius: float,
        obstacle: Dict
    ) -> CollisionResult:
        """Check collision with sphere obstacle"""
        obs_center = np.array(obstacle["center"])
        obs_radius = obstacle["radius"]

        distance = np.linalg.norm(position - obs_center) - obs_radius - radius

        if distance < self.min_distance:
            return CollisionResult(
                has_collision=True,
                collision_type=CollisionType.OBSTACLE,
                collision_point=obs_center,
                distance=distance,
                message=f"Collision with sphere obstacle at {obs_center}"
            )

        return CollisionResult(has_collision=False, distance=distance)

    def _check_box_collision(
        self,
        position: np.ndarray,
        radius: float,
        obstacle: Dict
    ) -> CollisionResult:
        """Check collision with box obstacle"""
        obs_min = np.array(obstacle["min"])
        obs_max = np.array(obstacle["max"])

        # Expand box by radius
        expanded_min = obs_min - radius
        expanded_max = obs_max + radius

        if np.all(position >= expanded_min) and np.all(position <= expanded_max):
            return CollisionResult(
                has_collision=True,
                collision_type=CollisionType.OBSTACLE,
                distance=0,
                message="Collision with box obstacle"
            )

        # Calculate distance
        closest = np.clip(position, obs_min, obs_max)
        distance = np.linalg.norm(position - closest) - radius

        return CollisionResult(has_collision=False, distance=distance)

    def add_obstacle(self, obstacle: Dict):
        """Add obstacle"""
        self.obstacles.append(obstacle)
        logger.info(f"Obstacle added: {obstacle}")

    def remove_obstacle(self, index: int):
        """Remove obstacle by index"""
        if 0 <= index < len(self.obstacles):
            removed = self.obstacles.pop(index)
            logger.info(f"Obstacle removed: {removed}")

    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        logger.info("All obstacles cleared")


class MotionSafetyValidator:
    """Validate motion parameters"""

    def __init__(self, config: Dict):
        """Initialize motion safety validator

        Args:
            config: Workspace configuration
        """
        limits = config.get("velocity_limits", {})
        manip = limits.get("manipulator", {})
        base = limits.get("base", {})

        # Manipulator limits
        self.max_linear_velocity = manip.get("max_linear", 0.5)
        self.max_angular_velocity = manip.get("max_angular", 1.0)
        self.max_linear_acceleration = manip.get("max_acceleration", 2.0)
        self.max_angular_acceleration = manip.get("max_angular_acceleration", 5.0)

        # Base limits
        self.max_base_linear = base.get("max_linear", 1.0)
        self.max_base_angular = base.get("max_angular", 1.5)

    def check_velocity(
        self,
        velocity: float,
        is_angular: bool = False
    ) -> Tuple[bool, str]:
        """Check velocity limit

        Args:
            velocity: Velocity to check
            is_angular: Whether angular velocity

        Returns:
            (is_valid, message)
        """
        limit = self.max_angular_velocity if is_angular else self.max_linear_velocity

        if abs(velocity) > limit:
            return False, f"Velocity {velocity:.3f} exceeds limit {limit}"

        return True, ""

    def check_acceleration(
        self,
        current_velocity: float,
        target_velocity: float,
        dt: float,
        is_angular: bool = False
    ) -> Tuple[bool, str]:
        """Check acceleration limit

        Args:
            current_velocity: Current velocity
            target_velocity: Target velocity
            dt: Time delta
            is_angular: Whether angular

        Returns:
            (is_valid, message)
        """
        acceleration = (target_velocity - current_velocity) / dt
        limit = self.max_angular_acceleration if is_angular else self.max_linear_acceleration

        if abs(acceleration) > limit:
            return False, f"Acceleration {acceleration:.3f} exceeds limit {limit}"

        return True, ""

    def limit_velocity_change(
        self,
        current_velocity: float,
        target_velocity: float,
        dt: float,
        is_angular: bool = False
    ) -> float:
        """Limit velocity change to respect acceleration limit

        Args:
            current_velocity: Current velocity
            target_velocity: Target velocity
            dt: Time delta
            is_angular: Whether angular

        Returns:
            Limited target velocity
        """
        limit = self.max_angular_acceleration if is_angular else self.max_linear_acceleration

        max_change = limit * dt
        change = target_velocity - current_velocity
        limited_change = np.clip(change, -max_change, max_change)

        return current_velocity + limited_change
