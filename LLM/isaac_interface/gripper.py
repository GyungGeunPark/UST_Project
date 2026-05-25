# Gripper Controller for Isaac Sim

import logging
import asyncio
import numpy as np
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class GripperState:
    """Gripper state constants"""
    OPEN = "open"
    CLOSED = "closed"
    MOVING = "moving"
    GRASPING = "grasping"


class GripperController:
    """Controller for robot gripper"""

    def __init__(self, articulation, config: Dict[str, Any]):
        """Initialize gripper controller

        Args:
            articulation: Isaac Sim Articulation object (can be None for standalone)
            config: Gripper configuration
        """
        self._articulation = articulation
        self.config = config

        # Joint parameters
        self._joint_indices = config.get("indices", [8, 9])
        self._open_position = config.get("open_position", 0.04)
        self._close_position = config.get("close_position", 0.0)
        self._max_force = config.get("max_force", 20.0)

        # State
        self._state = GripperState.OPEN
        self._current_position = self._open_position
        self._is_moving = False

        logger.info(f"GripperController initialized (joints: {self._joint_indices})")

    async def open(self) -> bool:
        """Open the gripper

        Returns:
            True if successful
        """
        logger.info("Opening gripper")

        if self._state == GripperState.OPEN:
            return True

        try:
            self._is_moving = True
            self._state = GripperState.MOVING

            if self._articulation:
                await self._move_to_position(self._open_position)
            else:
                # Standalone simulation
                await asyncio.sleep(0.3)

            self._current_position = self._open_position
            self._state = GripperState.OPEN
            return True

        except Exception as e:
            logger.error(f"Gripper open error: {e}")
            return False

        finally:
            self._is_moving = False

    async def close(self) -> bool:
        """Close the gripper

        Returns:
            True if successful
        """
        logger.info("Closing gripper")

        if self._state == GripperState.CLOSED:
            return True

        try:
            self._is_moving = True
            self._state = GripperState.MOVING

            if self._articulation:
                await self._move_to_position(self._close_position)
            else:
                # Standalone simulation
                await asyncio.sleep(0.3)

            self._current_position = self._close_position
            self._state = GripperState.CLOSED
            return True

        except Exception as e:
            logger.error(f"Gripper close error: {e}")
            return False

        finally:
            self._is_moving = False

    async def _move_to_position(self, target: float, duration: float = 0.5):
        """Move gripper to position

        Args:
            target: Target position
            duration: Movement duration
        """
        if not self._articulation:
            return

        start = self._current_position
        dt = 0.02  # 50Hz
        steps = int(duration / dt)

        for i in range(steps):
            if not self._is_moving:
                break

            t = (i + 1) / steps
            position = start + t * (target - start)

            # Set gripper joint positions
            all_positions = self._articulation.get_joint_positions()
            for idx in self._joint_indices:
                all_positions[idx] = position
            self._articulation.set_joint_positions(all_positions)

            await asyncio.sleep(dt)

    def get_state(self) -> str:
        """Get gripper state

        Returns:
            Current gripper state string
        """
        return self._state

    def get_position(self) -> float:
        """Get gripper position

        Returns:
            Current gripper position
        """
        if self._articulation:
            try:
                all_positions = self._articulation.get_joint_positions()
                return all_positions[self._joint_indices[0]]
            except Exception:
                pass

        return self._current_position

    def is_grasping(self) -> bool:
        """Check if gripper is grasping an object

        Returns:
            True if grasping
        """
        # In real implementation, would check force/effort feedback
        return self._state == GripperState.GRASPING

    @property
    def is_moving(self) -> bool:
        """Check if gripper is moving"""
        return self._is_moving

    @property
    def is_open(self) -> bool:
        """Check if gripper is open"""
        return self._state == GripperState.OPEN

    @property
    def is_closed(self) -> bool:
        """Check if gripper is closed"""
        return self._state == GripperState.CLOSED
