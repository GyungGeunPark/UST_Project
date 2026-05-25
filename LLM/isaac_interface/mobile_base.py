# Mobile Base Controller for Isaac Sim

import logging
import asyncio
import numpy as np
from typing import Dict, Any, Optional, Tuple

logger = logging.getLogger(__name__)


class MobileBaseController:
    """Controller for differential drive mobile base"""

    def __init__(self, articulation, config: Dict[str, Any]):
        """Initialize mobile base controller

        Args:
            articulation: Isaac Sim Articulation object (can be None for standalone)
            config: Wheel configuration
        """
        self._articulation = articulation
        self.config = config

        # Wheel parameters
        self._wheel_indices = config.get("indices", [0, 1])
        self._wheel_radius = config.get("radius", 0.05)
        self._base_width = config.get("base_width", 0.33)
        self._max_velocity = config.get("max_velocity", 10.0)

        # State
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._is_moving = False

        logger.info(f"MobileBaseController initialized (wheels: {self._wheel_indices})")

    async def move(
        self,
        linear_velocity: float,
        angular_velocity: float,
        duration: float
    ) -> bool:
        """Move the mobile base

        Args:
            linear_velocity: Forward/backward velocity in m/s
            angular_velocity: Rotation velocity in rad/s
            duration: Duration in seconds

        Returns:
            True if successful
        """
        logger.info(
            f"Mobile base move: linear={linear_velocity:.3f}, "
            f"angular={angular_velocity:.3f}, duration={duration:.2f}s"
        )

        # Convert to wheel velocities (differential drive kinematics)
        left_wheel_vel, right_wheel_vel = self._compute_wheel_velocities(
            linear_velocity, angular_velocity
        )

        # Clamp to limits
        left_wheel_vel = np.clip(left_wheel_vel, -self._max_velocity, self._max_velocity)
        right_wheel_vel = np.clip(right_wheel_vel, -self._max_velocity, self._max_velocity)

        if self._articulation:
            try:
                self._is_moving = True

                # Set wheel velocities
                velocities = np.zeros(self._articulation.num_dof)
                velocities[self._wheel_indices[0]] = left_wheel_vel
                velocities[self._wheel_indices[1]] = right_wheel_vel

                # Apply velocities over duration
                dt = 0.02  # 50Hz control loop
                steps = int(duration / dt)

                for _ in range(steps):
                    if not self._is_moving:  # Check for stop
                        break
                    self._articulation.set_joint_velocities(velocities)
                    await asyncio.sleep(dt)

                # Stop
                await self.stop()
                return True

            except Exception as e:
                logger.error(f"Mobile base move error: {e}")
                return False

            finally:
                self._is_moving = False
        else:
            # Standalone mode - simulate movement
            self._current_linear = linear_velocity
            self._current_angular = angular_velocity
            await asyncio.sleep(duration)
            self._current_linear = 0.0
            self._current_angular = 0.0
            return True

    def _compute_wheel_velocities(
        self,
        linear: float,
        angular: float
    ) -> Tuple[float, float]:
        """Compute wheel velocities from linear and angular velocities

        Args:
            linear: Linear velocity in m/s
            angular: Angular velocity in rad/s

        Returns:
            (left_wheel_vel, right_wheel_vel) in rad/s
        """
        # Differential drive inverse kinematics
        # v = (v_r + v_l) / 2
        # omega = (v_r - v_l) / L
        # where v_r, v_l are wheel linear velocities

        # Solve for wheel linear velocities
        v_left = linear - (angular * self._base_width / 2)
        v_right = linear + (angular * self._base_width / 2)

        # Convert to angular velocities
        omega_left = v_left / self._wheel_radius
        omega_right = v_right / self._wheel_radius

        return omega_left, omega_right

    async def stop(self):
        """Stop the mobile base"""
        logger.info("Stopping mobile base")
        self._is_moving = False

        if self._articulation:
            try:
                velocities = self._articulation.get_joint_velocities()
                velocities[self._wheel_indices[0]] = 0.0
                velocities[self._wheel_indices[1]] = 0.0
                self._articulation.set_joint_velocities(velocities)
            except Exception as e:
                logger.error(f"Stop error: {e}")

        self._current_linear = 0.0
        self._current_angular = 0.0

    def get_odometry(self) -> Tuple[float, float, float]:
        """Get estimated odometry

        Returns:
            (x, y, theta) estimated pose
        """
        # Simplified - in real implementation would integrate wheel encoders
        return (0.0, 0.0, 0.0)

    @property
    def is_moving(self) -> bool:
        """Check if base is moving"""
        return self._is_moving
