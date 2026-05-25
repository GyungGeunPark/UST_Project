# Robot Controller for Isaac Sim

import logging
import asyncio
import numpy as np
from typing import Dict, Any, Optional, Tuple, List

logger = logging.getLogger(__name__)


class RobotController:
    """Main robot controller coordinating all subsystems"""

    def __init__(self, config: Dict[str, Any]):
        """Initialize robot controller

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.robot_config = config.get("robot", {})

        # Subsystem controllers
        self._mobile_base = None
        self._manipulator = None
        self._gripper = None
        self._ik_solver = None

        # Isaac Sim references
        self._articulation = None
        self._world = None

        # State
        self._initialized = False
        self._emergency_stopped = False

        logger.info("RobotController created")

    def initialize(self, world, articulation):
        """Initialize with Isaac Sim objects

        Args:
            world: Isaac Sim World object
            articulation: Robot Articulation object
        """
        self._world = world
        self._articulation = articulation

        # Initialize subsystems
        from .mobile_base import MobileBaseController
        from .manipulator import ManipulatorController
        from .gripper import GripperController
        from .ik_solver import IKSolverWrapper

        # Mobile base
        wheel_config = self.robot_config.get("joints", {}).get("wheel", {})
        self._mobile_base = MobileBaseController(articulation, wheel_config)

        # Manipulator
        arm_config = self.robot_config.get("joints", {}).get("arm", {})
        self._manipulator = ManipulatorController(articulation, arm_config)

        # Gripper
        gripper_config = self.robot_config.get("joints", {}).get("gripper", {})
        self._gripper = GripperController(articulation, gripper_config)

        # IK Solver
        files_config = self.robot_config.get("files", {})
        self._ik_solver = IKSolverWrapper(files_config)

        self._initialized = True
        logger.info("RobotController initialized with Isaac Sim")

    def initialize_standalone(self):
        """Initialize for standalone mode (without Isaac Sim)"""
        from .mobile_base import MobileBaseController
        from .manipulator import ManipulatorController
        from .gripper import GripperController
        from .ik_solver import IKSolverWrapper

        # Create mock controllers
        wheel_config = self.robot_config.get("joints", {}).get("wheel", {})
        self._mobile_base = MobileBaseController(None, wheel_config)

        arm_config = self.robot_config.get("joints", {}).get("arm", {})
        self._manipulator = ManipulatorController(None, arm_config)

        gripper_config = self.robot_config.get("joints", {}).get("gripper", {})
        self._gripper = GripperController(None, gripper_config)

        files_config = self.robot_config.get("files", {})
        self._ik_solver = IKSolverWrapper(files_config)

        self._initialized = True
        logger.info("RobotController initialized in standalone mode")

    # ==========================================
    # Position and State
    # ==========================================

    def get_end_effector_position(self) -> Tuple[float, float, float]:
        """Get current end-effector position

        Returns:
            (x, y, z) position in meters
        """
        if self._manipulator and self._articulation:
            return self._manipulator.get_end_effector_position()
        return (0.5, 0.3, 0.5)  # Default position

    def get_end_effector_orientation(self) -> Tuple[float, float, float, float]:
        """Get current end-effector orientation

        Returns:
            (qw, qx, qy, qz) quaternion
        """
        if self._manipulator and self._articulation:
            return self._manipulator.get_end_effector_orientation()
        return (1.0, 0.0, 0.0, 0.0)

    def get_joint_positions(self) -> List[float]:
        """Get all joint positions

        Returns:
            List of joint positions
        """
        if self._articulation:
            return self._articulation.get_joint_positions().tolist()
        return [0.0] * 10

    def get_joint_velocities(self) -> List[float]:
        """Get all joint velocities

        Returns:
            List of joint velocities
        """
        if self._articulation:
            return self._articulation.get_joint_velocities().tolist()
        return [0.0] * 10

    def get_gripper_state(self) -> str:
        """Get gripper state

        Returns:
            'open', 'closed', or 'moving'
        """
        if self._gripper:
            return self._gripper.get_state()
        return "open"

    # ==========================================
    # Manipulator Control
    # ==========================================

    async def move_relative(
        self,
        direction: str,
        distance: float,
        speed: float = 1.0
    ) -> bool:
        """Move end-effector in a direction

        Args:
            direction: 'forward', 'backward', 'left', 'right', 'up', 'down'
            distance: Distance in meters
            speed: Speed multiplier

        Returns:
            True if successful
        """
        if self._emergency_stopped:
            logger.warning("Cannot move: emergency stopped")
            return False

        if not self._manipulator:
            logger.warning("Manipulator not initialized")
            await asyncio.sleep(0.5)  # Simulate
            return True

        current_pos = self.get_end_effector_position()

        # Calculate target position
        direction_vectors = {
            "forward": (distance, 0, 0),
            "backward": (-distance, 0, 0),
            "left": (0, distance, 0),
            "right": (0, -distance, 0),
            "up": (0, 0, distance),
            "down": (0, 0, -distance),
        }

        delta = direction_vectors.get(direction, (0, 0, 0))
        target_pos = (
            current_pos[0] + delta[0],
            current_pos[1] + delta[1],
            current_pos[2] + delta[2]
        )

        return await self.move_to_position(target_pos, speed)

    async def move_to_position(
        self,
        position: Tuple[float, float, float],
        speed: float = 1.0
    ) -> bool:
        """Move end-effector to absolute position

        Args:
            position: (x, y, z) target position in meters
            speed: Speed multiplier

        Returns:
            True if successful
        """
        if self._emergency_stopped:
            logger.warning("Cannot move: emergency stopped")
            return False

        if not self._manipulator:
            await asyncio.sleep(0.5)  # Simulate
            return True

        try:
            # Get IK solution
            if self._ik_solver:
                joint_targets = self._ik_solver.solve(
                    position,
                    self.get_end_effector_orientation()
                )

                if joint_targets is None:
                    logger.error("IK solution not found")
                    return False

            # Execute motion
            return await self._manipulator.move_to_position(position, speed)

        except Exception as e:
            logger.error(f"Move to position error: {e}")
            return False

    # ==========================================
    # Mobile Base Control
    # ==========================================

    async def move_base(
        self,
        linear_velocity: float,
        angular_velocity: float,
        duration: float
    ) -> bool:
        """Move mobile base

        Args:
            linear_velocity: Linear velocity in m/s
            angular_velocity: Angular velocity in rad/s
            duration: Duration in seconds

        Returns:
            True if successful
        """
        if self._emergency_stopped:
            logger.warning("Cannot move: emergency stopped")
            return False

        if not self._mobile_base:
            await asyncio.sleep(duration)
            return True

        try:
            return await self._mobile_base.move(
                linear_velocity, angular_velocity, duration
            )
        except Exception as e:
            logger.error(f"Mobile base move error: {e}")
            return False

    # ==========================================
    # Gripper Control
    # ==========================================

    async def open_gripper(self) -> bool:
        """Open the gripper

        Returns:
            True if successful
        """
        if self._emergency_stopped:
            return False

        if not self._gripper:
            await asyncio.sleep(0.3)
            return True

        return await self._gripper.open()

    async def close_gripper(self) -> bool:
        """Close the gripper

        Returns:
            True if successful
        """
        if self._emergency_stopped:
            return False

        if not self._gripper:
            await asyncio.sleep(0.3)
            return True

        return await self._gripper.close()

    # ==========================================
    # Safety
    # ==========================================

    async def stop(self):
        """Stop all motion"""
        logger.info("Stopping robot")

        if self._mobile_base:
            await self._mobile_base.stop()

        if self._manipulator:
            await self._manipulator.stop()

    def emergency_stop(self):
        """Emergency stop - immediate halt"""
        logger.critical("EMERGENCY STOP")
        self._emergency_stopped = True

        # Set all velocities to zero immediately
        if self._articulation:
            try:
                num_joints = self._articulation.num_dof
                self._articulation.set_joint_velocities(np.zeros(num_joints))
            except Exception as e:
                logger.error(f"Emergency stop error: {e}")

    def reset_emergency(self):
        """Reset emergency stop state"""
        logger.info("Resetting emergency stop")
        self._emergency_stopped = False

    @property
    def is_emergency_stopped(self) -> bool:
        """Check if emergency stopped"""
        return self._emergency_stopped

    @property
    def is_initialized(self) -> bool:
        """Check if initialized"""
        return self._initialized
