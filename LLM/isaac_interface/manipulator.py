# Manipulator Controller for Isaac Sim

import logging
import asyncio
import numpy as np
from typing import Dict, Any, Optional, Tuple, List

logger = logging.getLogger(__name__)


class ManipulatorController:
    """Controller for robot manipulator arm"""

    def __init__(self, articulation, config: Dict[str, Any]):
        """Initialize manipulator controller

        Args:
            articulation: Isaac Sim Articulation object (can be None for standalone)
            config: Arm configuration
        """
        self._articulation = articulation
        self.config = config

        # Joint parameters
        self._joint_indices = config.get("indices", [2, 3, 4, 5, 6, 7])
        self._end_effector_frame = config.get("end_effector_frame", "link_gripper")

        # Limits
        self._lower_limits = np.array(config.get("lower_limits", [-3.14] * 6))
        self._upper_limits = np.array(config.get("upper_limits", [3.14] * 6))
        self._velocity_limits = np.array(config.get("velocity_limits", [1.0] * 6))

        # State
        self._is_moving = False
        self._current_position = np.array([0.5, 0.3, 0.5])
        self._current_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        logger.info(f"ManipulatorController initialized (joints: {self._joint_indices})")

    async def move_to_position(
        self,
        target_position: Tuple[float, float, float],
        speed: float = 1.0
    ) -> bool:
        """Move end-effector to target position

        Args:
            target_position: (x, y, z) target position in meters
            speed: Speed multiplier (0.1 to 2.0)

        Returns:
            True if successful
        """
        logger.info(f"Moving to position: {target_position} at speed {speed}")

        target = np.array(target_position)

        if self._articulation:
            try:
                self._is_moving = True

                # Get current position
                current = self.get_end_effector_position()
                current = np.array(current)

                # Interpolate path
                duration = np.linalg.norm(target - current) / (0.1 * speed)
                duration = max(0.5, min(5.0, duration))

                dt = 0.02  # 50Hz
                steps = int(duration / dt)

                for i in range(steps):
                    if not self._is_moving:
                        break

                    t = (i + 1) / steps
                    interp_pos = current + t * (target - current)

                    # In real implementation, would use IK and joint control
                    self._current_position = interp_pos

                    await asyncio.sleep(dt)

                self._current_position = target
                return True

            except Exception as e:
                logger.error(f"Move to position error: {e}")
                return False

            finally:
                self._is_moving = False
        else:
            # Standalone mode
            distance = np.linalg.norm(target - self._current_position)
            duration = distance / (0.1 * speed)
            await asyncio.sleep(min(3.0, duration))
            self._current_position = target
            return True

    async def move_joints(
        self,
        joint_positions: List[float],
        speed: float = 1.0
    ) -> bool:
        """Move to specified joint positions

        Args:
            joint_positions: Target joint positions
            speed: Speed multiplier

        Returns:
            True if successful
        """
        logger.info(f"Moving joints to: {joint_positions}")

        if self._articulation:
            try:
                self._is_moving = True

                target = np.array(joint_positions)
                current = self.get_joint_positions()

                # Check limits
                target = np.clip(target, self._lower_limits, self._upper_limits)

                # Interpolate
                max_diff = np.max(np.abs(target - current))
                duration = max_diff / (1.0 * speed)
                duration = max(0.5, min(5.0, duration))

                dt = 0.02
                steps = int(duration / dt)

                for i in range(steps):
                    if not self._is_moving:
                        break

                    t = (i + 1) / steps
                    interp = current + t * (target - current)

                    # Set joint positions
                    all_positions = self._articulation.get_joint_positions()
                    for j, idx in enumerate(self._joint_indices):
                        all_positions[idx] = interp[j]
                    self._articulation.set_joint_positions(all_positions)

                    await asyncio.sleep(dt)

                return True

            except Exception as e:
                logger.error(f"Move joints error: {e}")
                return False

            finally:
                self._is_moving = False
        else:
            await asyncio.sleep(1.0)
            return True

    async def stop(self):
        """Stop manipulator motion"""
        logger.info("Stopping manipulator")
        self._is_moving = False

        if self._articulation:
            try:
                velocities = self._articulation.get_joint_velocities()
                for idx in self._joint_indices:
                    velocities[idx] = 0.0
                self._articulation.set_joint_velocities(velocities)
            except Exception as e:
                logger.error(f"Stop error: {e}")

    def get_end_effector_position(self) -> Tuple[float, float, float]:
        """Get current end-effector position

        Returns:
            (x, y, z) position in meters
        """
        if self._articulation:
            try:
                # Get transform from articulation
                # This is simplified - real implementation would use FK
                return tuple(self._current_position)
            except Exception:
                pass

        return tuple(self._current_position)

    def get_end_effector_orientation(self) -> Tuple[float, float, float, float]:
        """Get current end-effector orientation

        Returns:
            (qw, qx, qy, qz) quaternion
        """
        return tuple(self._current_orientation)

    def get_joint_positions(self) -> np.ndarray:
        """Get current arm joint positions

        Returns:
            Array of joint positions
        """
        if self._articulation:
            try:
                all_positions = self._articulation.get_joint_positions()
                return np.array([all_positions[i] for i in self._joint_indices])
            except Exception:
                pass

        return np.zeros(len(self._joint_indices))

    def get_joint_velocities(self) -> np.ndarray:
        """Get current arm joint velocities

        Returns:
            Array of joint velocities
        """
        if self._articulation:
            try:
                all_velocities = self._articulation.get_joint_velocities()
                return np.array([all_velocities[i] for i in self._joint_indices])
            except Exception:
                pass

        return np.zeros(len(self._joint_indices))

    @property
    def is_moving(self) -> bool:
        """Check if manipulator is moving"""
        return self._is_moving
