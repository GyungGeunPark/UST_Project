# Control Manager - Main orchestrator for robot control

import time
import asyncio
import logging
from typing import Dict, Any, Optional, Tuple, Callable
from dataclasses import dataclass

from .robot_command import (
    RobotCommand,
    CommandResult,
    CommandStatus,
    CommandType,
    RobotState,
)
from .llm_client import LLMClient
from .llm_tools import get_tool_definitions
from .prompts import get_system_prompt
from .response_parser import LLMResponseParser
from .command_validator import CommandValidator

logger = logging.getLogger(__name__)


class ControlManager:
    """Main control manager for robot operations"""

    def __init__(self, config: Dict[str, Any]):
        """Initialize control manager

        Args:
            config: Configuration dictionary
        """
        self.config = config
        self._start_time = time.time()

        # State
        self._state = RobotState()
        self._emergency_stopped = False
        self._is_moving = False
        self._command_count = 0

        # Current position (will be updated from robot controller)
        self._current_position = (0.5, 0.3, 0.5)
        self._current_orientation = (1.0, 0.0, 0.0, 0.0)
        self._gripper_state = "open"

        # Components
        self._llm_client: Optional[LLMClient] = None
        self._parser = LLMResponseParser(config)
        self._validator = CommandValidator(config)

        # Robot controller (to be set externally)
        self._robot_controller = None

        # Emergency stop system (to be set externally)
        self._emergency_system = None

        # Callbacks
        self._on_command_start: Optional[Callable] = None
        self._on_command_complete: Optional[Callable] = None
        self._on_error: Optional[Callable] = None

        # Initialize LLM client
        self._init_llm_client()

        logger.info("ControlManager initialized")

    def _init_llm_client(self):
        """Initialize LLM client"""
        llm_config = self.config.get("llm", {})
        try:
            self._llm_client = LLMClient(llm_config)
            logger.info("LLM client initialized")
        except Exception as e:
            logger.error(f"Failed to initialize LLM client: {e}")
            self._llm_client = None

    def set_robot_controller(self, controller):
        """Set the robot controller

        Args:
            controller: Robot controller instance
        """
        self._robot_controller = controller
        logger.info("Robot controller connected to ControlManager")

    def set_emergency_system(self, system):
        """Set the emergency stop system

        Args:
            system: Emergency stop system instance
        """
        self._emergency_system = system
        logger.info("Emergency system connected to ControlManager")

    def set_callbacks(
        self,
        on_command_start: Optional[Callable] = None,
        on_command_complete: Optional[Callable] = None,
        on_error: Optional[Callable] = None
    ):
        """Set callback functions

        Args:
            on_command_start: Called when command starts executing
            on_command_complete: Called when command completes
            on_error: Called on error
        """
        self._on_command_start = on_command_start
        self._on_command_complete = on_command_complete
        self._on_error = on_error

    async def process_command(self, command_text: str) -> CommandResult:
        """Process a natural language command

        Args:
            command_text: Natural language command from user

        Returns:
            CommandResult with execution status
        """
        logger.info(f"Processing command: {command_text}")

        # Check emergency stop
        if self._emergency_stopped:
            return CommandResult(
                success=False,
                command_id="",
                message="Robot is emergency stopped",
                error_code="EMERGENCY_STOPPED"
            )

        # Check if already moving
        if self._is_moving:
            return CommandResult(
                success=False,
                command_id="",
                message="Robot is currently executing another command",
                error_code="BUSY"
            )

        # Check LLM client
        if not self._llm_client:
            return CommandResult(
                success=False,
                command_id="",
                message="LLM client not initialized",
                error_code="NOT_CONNECTED"
            )

        try:
            # Get system prompt with current state
            system_prompt = get_system_prompt(
                self.config,
                include_safety=True,
                include_workspace=True,
                current_position=self._current_position,
                gripper_state=self._gripper_state
            )

            # Get tool definitions
            tools = get_tool_definitions()

            # Call LLM
            llm_response = await self._llm_client.process_command(
                command_text, system_prompt, tools
            )

            # Parse response
            parse_result = self._parser.parse(llm_response, command_text)

            if not parse_result.success:
                return CommandResult(
                    success=False,
                    command_id="",
                    message=parse_result.error_message,
                    error_code=parse_result.error_code
                )

            command = parse_result.command

            # Validate command
            validation_result = self._validator.validate(
                command, self._current_position
            )

            if not validation_result.is_valid:
                error_msg = "; ".join(validation_result.errors)
                return CommandResult(
                    success=False,
                    command_id=command.command_id,
                    message=f"Validation failed: {error_msg}",
                    error_code="VALIDATION_ERROR",
                    suggested_position=validation_result.suggested_correction.get(
                        "suggested"
                    ) if validation_result.suggested_correction else None
                )

            # Log warnings
            for warning in validation_result.warnings:
                logger.warning(f"Command warning: {warning}")

            # Execute command
            return await self._execute_command(command)

        except Exception as e:
            logger.error(f"Command processing error: {e}", exc_info=True)
            if self._on_error:
                self._on_error(str(e))

            return CommandResult(
                success=False,
                command_id="",
                message=f"Processing error: {str(e)}",
                error_code="INTERNAL_ERROR"
            )

    async def _execute_command(self, command: RobotCommand) -> CommandResult:
        """Execute a validated command

        Args:
            command: Validated robot command

        Returns:
            CommandResult with execution status
        """
        command.start_execution()
        self._is_moving = True
        self._command_count += 1

        if self._on_command_start:
            self._on_command_start(command)

        try:
            # Route to appropriate handler
            if command.command_type == CommandType.MOVE_MANIPULATOR:
                result = await self._execute_move_manipulator(command)
            elif command.command_type == CommandType.MOVE_MOBILE_BASE:
                result = await self._execute_move_mobile_base(command)
            elif command.command_type == CommandType.CONTROL_GRIPPER:
                result = await self._execute_control_gripper(command)
            elif command.command_type == CommandType.STOP_ROBOT:
                result = await self._execute_stop(command)
            else:
                result = CommandResult(
                    success=False,
                    command_id=command.command_id,
                    message=f"Unknown command type: {command.command_type}",
                    error_code="UNKNOWN_COMMAND"
                )

            # Update command status
            if result.success:
                command.complete()
            else:
                command.fail(result.message, result.error_code or "EXECUTION_ERROR")

            if self._on_command_complete:
                self._on_command_complete(command, result)

            return result

        except Exception as e:
            logger.error(f"Command execution error: {e}", exc_info=True)
            command.fail(str(e), "EXECUTION_ERROR")

            return CommandResult(
                success=False,
                command_id=command.command_id,
                message=f"Execution error: {str(e)}",
                error_code="EXECUTION_ERROR"
            )

        finally:
            self._is_moving = False

    async def _execute_move_manipulator(
        self, command: RobotCommand
    ) -> CommandResult:
        """Execute move_manipulator command"""
        params = command.parameters

        if not self._robot_controller:
            # Simulation mode - just update position
            target = self._parser.extract_target_position(
                command, self._current_position
            )
            if target:
                self._current_position = target
                await asyncio.sleep(0.5)  # Simulate movement time

            return CommandResult(
                success=True,
                command_id=command.command_id,
                message="Manipulator move completed (simulation)",
                execution_time=command.execution_time,
                final_position=list(self._current_position)
            )

        # Real robot execution
        try:
            movement_type = params.get("movement_type")
            speed = params.get("speed", 1.0)

            if movement_type == "relative":
                direction = params.get("direction")
                distance = params.get("distance", 10.0) / 100.0  # cm to m

                success = await self._robot_controller.move_relative(
                    direction, distance, speed
                )

            elif movement_type == "absolute":
                position = params.get("position", {})
                target = (position.get("x"), position.get("y"), position.get("z"))

                success = await self._robot_controller.move_to_position(
                    target, speed
                )

            else:
                success = False

            if success:
                # Update current position
                self._current_position = self._robot_controller.get_end_effector_position()

                return CommandResult(
                    success=True,
                    command_id=command.command_id,
                    message="Manipulator move completed",
                    execution_time=command.execution_time,
                    final_position=list(self._current_position)
                )
            else:
                return CommandResult(
                    success=False,
                    command_id=command.command_id,
                    message="Manipulator move failed",
                    error_code="EXECUTION_ERROR"
                )

        except Exception as e:
            return CommandResult(
                success=False,
                command_id=command.command_id,
                message=f"Manipulator error: {str(e)}",
                error_code="EXECUTION_ERROR"
            )

    async def _execute_move_mobile_base(
        self, command: RobotCommand
    ) -> CommandResult:
        """Execute move_mobile_base command"""
        params = command.parameters

        if not self._robot_controller:
            # Simulation mode
            await asyncio.sleep(params.get("duration", 2.0))
            return CommandResult(
                success=True,
                command_id=command.command_id,
                message="Mobile base move completed (simulation)",
                execution_time=command.execution_time
            )

        # Real robot execution
        try:
            linear = params.get("linear_velocity", 0.0)
            angular = params.get("angular_velocity", 0.0)
            duration = params.get("duration", 2.0)

            success = await self._robot_controller.move_base(
                linear, angular, duration
            )

            if success:
                return CommandResult(
                    success=True,
                    command_id=command.command_id,
                    message="Mobile base move completed",
                    execution_time=command.execution_time
                )
            else:
                return CommandResult(
                    success=False,
                    command_id=command.command_id,
                    message="Mobile base move failed",
                    error_code="EXECUTION_ERROR"
                )

        except Exception as e:
            return CommandResult(
                success=False,
                command_id=command.command_id,
                message=f"Mobile base error: {str(e)}",
                error_code="EXECUTION_ERROR"
            )

    async def _execute_control_gripper(
        self, command: RobotCommand
    ) -> CommandResult:
        """Execute control_gripper command"""
        params = command.parameters
        action = params.get("action", "open")

        if not self._robot_controller:
            # Simulation mode
            self._gripper_state = "open" if action == "open" else "closed"
            await asyncio.sleep(0.3)
            return CommandResult(
                success=True,
                command_id=command.command_id,
                message=f"Gripper {action} completed (simulation)",
                execution_time=command.execution_time
            )

        # Real robot execution
        try:
            if action == "open":
                success = await self._robot_controller.open_gripper()
            else:
                success = await self._robot_controller.close_gripper()

            if success:
                self._gripper_state = "open" if action == "open" else "closed"
                return CommandResult(
                    success=True,
                    command_id=command.command_id,
                    message=f"Gripper {action} completed",
                    execution_time=command.execution_time
                )
            else:
                return CommandResult(
                    success=False,
                    command_id=command.command_id,
                    message=f"Gripper {action} failed",
                    error_code="EXECUTION_ERROR"
                )

        except Exception as e:
            return CommandResult(
                success=False,
                command_id=command.command_id,
                message=f"Gripper error: {str(e)}",
                error_code="EXECUTION_ERROR"
            )

    async def _execute_stop(self, command: RobotCommand) -> CommandResult:
        """Execute stop_robot command"""
        if self._robot_controller:
            try:
                await self._robot_controller.stop()
            except Exception as e:
                logger.error(f"Stop error: {e}")

        self._is_moving = False

        return CommandResult(
            success=True,
            command_id=command.command_id,
            message="Robot stopped",
            execution_time=command.execution_time
        )

    def emergency_stop(self):
        """Trigger emergency stop"""
        logger.critical("EMERGENCY STOP triggered")
        self._emergency_stopped = True
        self._is_moving = False

        if self._robot_controller:
            try:
                self._robot_controller.emergency_stop()
            except Exception as e:
                logger.error(f"Emergency stop error: {e}")

        if self._emergency_system:
            self._emergency_system.trigger()

    def reset(self):
        """Reset from emergency stop"""
        logger.info("Resetting from emergency stop")
        self._emergency_stopped = False

        if self._emergency_system:
            self._emergency_system.reset()

    def get_status(self) -> RobotState:
        """Get current robot status"""
        # Update state from robot controller if available
        if self._robot_controller:
            try:
                self._current_position = self._robot_controller.get_end_effector_position()
                self._current_orientation = self._robot_controller.get_end_effector_orientation()
                self._gripper_state = self._robot_controller.get_gripper_state()
            except Exception:
                pass

        state = CommandStatus.PENDING
        if self._emergency_stopped:
            state = CommandStatus.FAILED
        elif self._is_moving:
            state = CommandStatus.EXECUTING

        return RobotState(
            state=state,
            is_moving=self._is_moving,
            emergency_stopped=self._emergency_stopped,
            current_position=list(self._current_position),
            current_orientation=list(self._current_orientation),
            gripper_state=self._gripper_state,
            uptime=time.time() - self._start_time,
            command_count=self._command_count
        )

    def clear_cache(self):
        """Clear LLM command cache"""
        if self._llm_client:
            self._llm_client.clear_cache()
            logger.info("Command cache cleared")

    def get_stats(self) -> Dict[str, Any]:
        """Get statistics"""
        stats = {
            "uptime": time.time() - self._start_time,
            "command_count": self._command_count,
            "emergency_stopped": self._emergency_stopped,
        }

        if self._llm_client:
            stats["llm_stats"] = self._llm_client.get_stats()
            stats["cache_stats"] = self._llm_client.get_cache_stats()

        return stats

    @property
    def is_ready(self) -> bool:
        """Check if manager is ready for commands"""
        return (
            self._llm_client is not None and
            not self._emergency_stopped and
            not self._is_moving
        )
