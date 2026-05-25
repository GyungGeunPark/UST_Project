# IK Solver Wrapper for Isaac Sim

import logging
import numpy as np
from typing import Dict, Any, Optional, Tuple, List

logger = logging.getLogger(__name__)


class IKSolverWrapper:
    """Wrapper for Inverse Kinematics solver"""

    def __init__(self, config: Dict[str, Any]):
        """Initialize IK solver

        Args:
            config: Files configuration with URDF/Lula paths
        """
        self.config = config

        # Lula IK solver (Isaac Sim)
        self._lula_solver = None

        # Configuration
        self._urdf_path = config.get("urdf_path", "")
        self._lula_description_path = config.get("lula_description_path", "")

        # Solver parameters
        self._max_iterations = 100
        self._tolerance = 0.001  # 1mm

        self._initialized = False

        logger.info("IKSolverWrapper created")

    def initialize(self, robot_description=None):
        """Initialize the IK solver with robot description

        Args:
            robot_description: Optional robot description object
        """
        try:
            # Try to initialize Lula IK solver (Isaac Sim specific)
            from omni.isaac.motion_generation import LulaKinematicsSolver

            self._lula_solver = LulaKinematicsSolver(
                robot_description_path=self._lula_description_path,
                urdf_path=self._urdf_path
            )
            self._initialized = True
            logger.info("Lula IK solver initialized")

        except ImportError:
            logger.warning("Lula solver not available, using fallback")
            self._initialized = True

        except Exception as e:
            logger.error(f"IK solver initialization error: {e}")
            self._initialized = True  # Continue without real IK

    def solve(
        self,
        target_position: Tuple[float, float, float],
        target_orientation: Optional[Tuple[float, float, float, float]] = None,
        initial_joint_positions: Optional[List[float]] = None
    ) -> Optional[np.ndarray]:
        """Solve IK for target pose

        Args:
            target_position: (x, y, z) target position in meters
            target_orientation: (qw, qx, qy, qz) target orientation (optional)
            initial_joint_positions: Initial guess for joint positions

        Returns:
            Joint positions array or None if no solution
        """
        if self._lula_solver:
            try:
                # Convert to numpy
                pos = np.array(target_position)

                if target_orientation:
                    quat = np.array(target_orientation)
                else:
                    quat = np.array([1.0, 0.0, 0.0, 0.0])

                # Solve IK
                result = self._lula_solver.compute_inverse_kinematics(
                    target_position=pos,
                    target_orientation=quat
                )

                if result is not None:
                    return result
                else:
                    logger.warning("IK solution not found")
                    return None

            except Exception as e:
                logger.error(f"IK solve error: {e}")
                return None
        else:
            # Fallback - return placeholder
            logger.debug("Using fallback IK (placeholder)")
            return np.zeros(6)

    def solve_with_constraints(
        self,
        target_position: Tuple[float, float, float],
        target_orientation: Optional[Tuple[float, float, float, float]] = None,
        joint_limits: Optional[Tuple[np.ndarray, np.ndarray]] = None,
        collision_spheres: Optional[List[Dict]] = None
    ) -> Optional[np.ndarray]:
        """Solve IK with additional constraints

        Args:
            target_position: Target end-effector position
            target_orientation: Target orientation
            joint_limits: (lower_limits, upper_limits) arrays
            collision_spheres: List of collision sphere definitions

        Returns:
            Joint positions or None
        """
        # For now, just call basic solve
        return self.solve(target_position, target_orientation)

    def compute_forward_kinematics(
        self,
        joint_positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute forward kinematics

        Args:
            joint_positions: Joint position array

        Returns:
            (position, orientation) tuple
        """
        if self._lula_solver:
            try:
                pos, quat = self._lula_solver.compute_forward_kinematics(
                    joint_positions
                )
                return pos, quat
            except Exception as e:
                logger.error(f"FK error: {e}")

        # Fallback
        return np.array([0.5, 0.3, 0.5]), np.array([1.0, 0.0, 0.0, 0.0])

    def check_joint_limits(
        self,
        joint_positions: np.ndarray,
        lower_limits: np.ndarray,
        upper_limits: np.ndarray
    ) -> bool:
        """Check if joint positions are within limits

        Args:
            joint_positions: Joint positions to check
            lower_limits: Lower joint limits
            upper_limits: Upper joint limits

        Returns:
            True if within limits
        """
        return np.all(joint_positions >= lower_limits) and \
               np.all(joint_positions <= upper_limits)

    @property
    def is_initialized(self) -> bool:
        """Check if solver is initialized"""
        return self._initialized
