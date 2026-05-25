# Emergency Stop System

import asyncio
import time
import threading
import logging
from typing import Optional, Callable, List, Dict
from enum import Enum
from dataclasses import dataclass

logger = logging.getLogger(__name__)


class EmergencyStopReason(Enum):
    """Emergency stop reasons"""
    USER_TRIGGERED = "user_triggered"
    WORKSPACE_VIOLATION = "workspace_violation"
    COLLISION_DETECTED = "collision_detected"
    VELOCITY_EXCEEDED = "velocity_exceeded"
    COMMUNICATION_LOST = "communication_lost"
    SYSTEM_ERROR = "system_error"
    WATCHDOG_TIMEOUT = "watchdog_timeout"


@dataclass
class EmergencyStopEvent:
    """Emergency stop event data"""
    timestamp: float
    reason: EmergencyStopReason
    details: str
    position: Optional[List[float]] = None
    velocity: Optional[List[float]] = None


class EmergencyStopSystem:
    """Emergency stop system for robot safety"""

    def __init__(self, config: Dict = None):
        """Initialize emergency stop system

        Args:
            config: Configuration dictionary
        """
        self.config = config or {}

        # State
        self._is_stopped = False
        self._stop_reason: Optional[EmergencyStopReason] = None
        self._stop_events: List[EmergencyStopEvent] = []

        # Callbacks
        self._on_stop_callbacks: List[Callable] = []
        self._on_reset_callbacks: List[Callable] = []

        # Robot controller reference
        self._robot_controller = None

        # Watchdog
        self._watchdog_enabled = self.config.get("watchdog_enabled", True)
        self._watchdog_timeout = self.config.get("watchdog_timeout", 5.0)
        self._last_heartbeat = time.time()
        self._watchdog_thread: Optional[threading.Thread] = None
        self._watchdog_running = False

        logger.info("Emergency stop system initialized")

    def set_robot_controller(self, controller):
        """Set robot controller reference

        Args:
            controller: Robot controller instance
        """
        self._robot_controller = controller

    def register_on_stop(self, callback: Callable):
        """Register callback for emergency stop event

        Args:
            callback: Function to call on stop
        """
        self._on_stop_callbacks.append(callback)

    def register_on_reset(self, callback: Callable):
        """Register callback for reset event

        Args:
            callback: Function to call on reset
        """
        self._on_reset_callbacks.append(callback)

    def trigger(
        self,
        reason: EmergencyStopReason = EmergencyStopReason.USER_TRIGGERED,
        details: str = ""
    ):
        """Trigger emergency stop

        Args:
            reason: Stop reason
            details: Additional details
        """
        if self._is_stopped:
            logger.warning("Emergency stop already active")
            return

        logger.critical(f"EMERGENCY STOP: {reason.value} - {details}")

        self._is_stopped = True
        self._stop_reason = reason

        # Get position/velocity if possible
        position = None
        velocity = None
        if self._robot_controller:
            try:
                position = list(self._robot_controller.get_end_effector_position())
                velocity = list(self._robot_controller.get_joint_velocities())
            except Exception:
                pass

        # Record event
        event = EmergencyStopEvent(
            timestamp=time.time(),
            reason=reason,
            details=details,
            position=position,
            velocity=velocity
        )
        self._stop_events.append(event)

        # Execute stop
        self._execute_stop()

        # Call callbacks
        for callback in self._on_stop_callbacks:
            try:
                callback(event)
            except Exception as e:
                logger.error(f"Stop callback error: {e}")

    def _execute_stop(self):
        """Execute the actual stop"""
        if self._robot_controller:
            try:
                self._robot_controller.emergency_stop()
                logger.info("Robot stopped successfully")
            except Exception as e:
                logger.error(f"Failed to stop robot: {e}")

    def reset(self) -> bool:
        """Reset emergency stop

        Returns:
            True if reset successful
        """
        if not self._is_stopped:
            logger.info("Emergency stop not active")
            return True

        # Check safety conditions
        if not self._check_safe_to_reset():
            logger.warning("Cannot reset: safety conditions not met")
            return False

        logger.info("Resetting emergency stop")
        self._is_stopped = False
        self._stop_reason = None

        # Reset robot controller
        if self._robot_controller:
            self._robot_controller.reset_emergency()

        # Call callbacks
        for callback in self._on_reset_callbacks:
            try:
                callback()
            except Exception as e:
                logger.error(f"Reset callback error: {e}")

        return True

    def _check_safe_to_reset(self) -> bool:
        """Check if safe to reset"""
        if not self._robot_controller:
            return True

        try:
            velocities = self._robot_controller.get_joint_velocities()
            max_velocity = max(abs(v) for v in velocities)

            if max_velocity > 0.01:
                logger.warning(f"Robot still moving: max velocity = {max_velocity}")
                return False

            return True

        except Exception as e:
            logger.error(f"Safety check error: {e}")
            return False

    @property
    def is_stopped(self) -> bool:
        """Check if emergency stopped"""
        return self._is_stopped

    @property
    def stop_reason(self) -> Optional[EmergencyStopReason]:
        """Get stop reason"""
        return self._stop_reason

    def get_events(self) -> List[EmergencyStopEvent]:
        """Get stop event history"""
        return self._stop_events.copy()

    # Watchdog

    def start_watchdog(self):
        """Start watchdog timer"""
        if not self._watchdog_enabled:
            return

        self._watchdog_running = True
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            daemon=True
        )
        self._watchdog_thread.start()
        logger.info(f"Watchdog started (timeout: {self._watchdog_timeout}s)")

    def stop_watchdog(self):
        """Stop watchdog timer"""
        self._watchdog_running = False
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=2.0)

    def heartbeat(self):
        """Update watchdog heartbeat"""
        self._last_heartbeat = time.time()

    def _watchdog_loop(self):
        """Watchdog monitoring loop"""
        while self._watchdog_running:
            time.sleep(1.0)

            if self._is_stopped:
                continue

            elapsed = time.time() - self._last_heartbeat
            if elapsed > self._watchdog_timeout:
                logger.error(f"Watchdog timeout: {elapsed:.1f}s since last heartbeat")
                self.trigger(
                    EmergencyStopReason.WATCHDOG_TIMEOUT,
                    f"No heartbeat for {elapsed:.1f}s"
                )


class SafetyMonitor:
    """Real-time safety monitor"""

    def __init__(
        self,
        emergency_system: EmergencyStopSystem,
        workspace_bounds: Dict,
        velocity_limits: Dict
    ):
        """Initialize safety monitor

        Args:
            emergency_system: Emergency stop system
            workspace_bounds: Workspace boundary configuration
            velocity_limits: Velocity limit configuration
        """
        self.emergency_system = emergency_system
        self.workspace_bounds = workspace_bounds
        self.velocity_limits = velocity_limits

        # Thresholds
        self._position_margin = 0.02  # 2cm
        self._velocity_warning_threshold = 0.8

        # State
        self._is_monitoring = False

    async def start_monitoring(self, robot_controller, interval: float = 0.02):
        """Start safety monitoring

        Args:
            robot_controller: Robot controller to monitor
            interval: Monitoring interval in seconds
        """
        self._is_monitoring = True
        logger.info("Safety monitoring started")

        while self._is_monitoring:
            try:
                # Check position
                position = robot_controller.get_end_effector_position()
                if not self._check_workspace(position):
                    self.emergency_system.trigger(
                        EmergencyStopReason.WORKSPACE_VIOLATION,
                        f"Position {position} outside workspace"
                    )

                # Check velocity
                velocities = robot_controller.get_joint_velocities()
                max_velocity = max(abs(v) for v in velocities)
                max_allowed = self.velocity_limits.get(
                    "manipulator", {}
                ).get("max_angular", 10.0)

                if max_velocity > max_allowed:
                    self.emergency_system.trigger(
                        EmergencyStopReason.VELOCITY_EXCEEDED,
                        f"Velocity {max_velocity} exceeds limit {max_allowed}"
                    )

                # Update heartbeat
                self.emergency_system.heartbeat()

                await asyncio.sleep(interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Safety monitor error: {e}")
                await asyncio.sleep(1.0)

    def stop_monitoring(self):
        """Stop safety monitoring"""
        self._is_monitoring = False
        logger.info("Safety monitoring stopped")

    def _check_workspace(self, position) -> bool:
        """Check if position is within workspace"""
        bounds = self.workspace_bounds.get("bounds", {})
        min_bounds = bounds.get("min", [-1, -1, 0])
        max_bounds = bounds.get("max", [1, 1, 1.5])

        margin = self._position_margin

        return (
            min_bounds[0] + margin <= position[0] <= max_bounds[0] - margin and
            min_bounds[1] + margin <= position[1] <= max_bounds[1] - margin and
            min_bounds[2] + margin <= position[2] <= max_bounds[2] - margin
        )
