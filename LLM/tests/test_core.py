# Core Module Tests

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.robot_command import (
    RobotCommand, CommandResult, CommandStatus, CommandType
)
from core.response_parser import LLMResponseParser
from core.command_validator import CommandValidator
from core.llm_tools import get_tool_definitions


class TestRobotCommand:
    """Tests for RobotCommand class"""

    def test_create_command(self):
        """Test command creation"""
        cmd = RobotCommand(
            original_text="move forward 10cm",
            command_type=CommandType.MOVE_MANIPULATOR
        )

        assert cmd.command_id is not None
        assert cmd.original_text == "move forward 10cm"
        assert cmd.command_type == CommandType.MOVE_MANIPULATOR
        assert cmd.status == CommandStatus.PENDING

    def test_command_lifecycle(self):
        """Test command status transitions"""
        cmd = RobotCommand()

        assert cmd.status == CommandStatus.PENDING

        cmd.start_execution()
        assert cmd.status == CommandStatus.EXECUTING
        assert cmd.started_at is not None

        cmd.complete()
        assert cmd.status == CommandStatus.COMPLETED
        assert cmd.completed_at is not None
        assert cmd.execution_time is not None

    def test_command_failure(self):
        """Test command failure"""
        cmd = RobotCommand()
        cmd.start_execution()
        cmd.fail("Test error", "TEST_ERROR")

        assert cmd.status == CommandStatus.FAILED
        assert cmd.error_message == "Test error"
        assert cmd.error_code == "TEST_ERROR"

    def test_to_dict(self):
        """Test conversion to dictionary"""
        cmd = RobotCommand(
            original_text="test",
            command_type=CommandType.STOP_ROBOT
        )

        result = cmd.to_dict()

        assert result["original_text"] == "test"
        assert result["command_type"] == "stop_robot"
        assert "command_id" in result


class TestCommandResult:
    """Tests for CommandResult class"""

    def test_success_result(self):
        """Test successful result"""
        result = CommandResult(
            success=True,
            command_id="test-123",
            message="Success",
            execution_time=1.5,
            final_position=[0.5, 0.3, 0.4]
        )

        assert result.success
        assert result.message == "Success"

        d = result.to_dict()
        assert d["success"]
        assert d["final_position"]["x"] == 0.5

    def test_error_result(self):
        """Test error result"""
        result = CommandResult(
            success=False,
            command_id="test-123",
            message="Failed",
            error_code="TEST_ERROR"
        )

        assert not result.success
        assert result.error_code == "TEST_ERROR"


class TestLLMResponseParser:
    """Tests for LLM response parser"""

    def setup_method(self):
        """Setup for each test"""
        self.parser = LLMResponseParser()

    def test_parse_move_manipulator(self):
        """Test parsing move_manipulator response"""
        llm_response = {
            "success": True,
            "function_name": "move_manipulator",
            "parameters": {
                "movement_type": "relative",
                "direction": "forward",
                "distance": 10.0,
                "speed": 1.0
            }
        }

        result = self.parser.parse(llm_response, "move forward 10cm")

        assert result.success
        assert result.command is not None
        assert result.command.command_type == CommandType.MOVE_MANIPULATOR
        assert result.command.parameters["direction"] == "forward"

    def test_parse_control_gripper(self):
        """Test parsing control_gripper response"""
        llm_response = {
            "success": True,
            "function_name": "control_gripper",
            "parameters": {
                "action": "open"
            }
        }

        result = self.parser.parse(llm_response, "open gripper")

        assert result.success
        assert result.command.command_type == CommandType.CONTROL_GRIPPER
        assert result.command.parameters["action"] == "open"

    def test_parse_llm_error(self):
        """Test parsing LLM error response"""
        llm_response = {
            "success": False,
            "error": "API error",
            "error_code": "LLM_ERROR"
        }

        result = self.parser.parse(llm_response, "test")

        assert not result.success
        assert result.error_code == "LLM_ERROR"

    def test_extract_target_position(self):
        """Test target position extraction"""
        cmd = RobotCommand(
            command_type=CommandType.MOVE_MANIPULATOR,
            parameters={
                "movement_type": "relative",
                "direction": "forward",
                "distance": 10.0
            }
        )

        current = (0.5, 0.3, 0.5)
        target = self.parser.extract_target_position(cmd, current)

        assert target is not None
        assert target[0] == 0.6  # forward adds to x


class TestCommandValidator:
    """Tests for command validator"""

    def setup_method(self):
        """Setup for each test"""
        config = {
            "workspace": {
                "bounds": {
                    "min": [-1.0, -1.0, 0.0],
                    "max": [1.0, 1.0, 1.5]
                },
                "safety": {
                    "workspace_margin": 0.05
                },
                "velocity_limits": {
                    "manipulator": {
                        "max_linear": 0.5
                    },
                    "base": {
                        "max_linear": 1.0,
                        "max_angular": 1.5
                    }
                }
            }
        }
        self.validator = CommandValidator(config)

    def test_valid_move_command(self):
        """Test validation of valid move command"""
        cmd = RobotCommand(
            command_type=CommandType.MOVE_MANIPULATOR,
            parameters={
                "movement_type": "relative",
                "direction": "forward",
                "distance": 10.0,
                "speed": 1.0
            }
        )

        result = self.validator.validate(cmd, (0.5, 0.3, 0.5))

        assert result.is_valid

    def test_workspace_violation(self):
        """Test detection of workspace violation"""
        cmd = RobotCommand(
            command_type=CommandType.MOVE_MANIPULATOR,
            parameters={
                "movement_type": "absolute",
                "position": {"x": 2.0, "y": 0.0, "z": 0.5},
                "speed": 1.0
            }
        )

        result = self.validator.validate(cmd)

        assert not result.is_valid
        assert len(result.errors) > 0

    def test_velocity_limit(self):
        """Test velocity limit validation"""
        cmd = RobotCommand(
            command_type=CommandType.MOVE_MOBILE_BASE,
            parameters={
                "linear_velocity": 2.0,  # Exceeds limit
                "angular_velocity": 0.0,
                "duration": 2.0
            }
        )

        result = self.validator.validate(cmd)

        assert not result.is_valid


class TestToolDefinitions:
    """Tests for LLM tool definitions"""

    def test_get_tool_definitions(self):
        """Test getting tool definitions"""
        tools = get_tool_definitions()

        assert len(tools) == 4  # 4 functions defined

        names = [t["function"]["name"] for t in tools]
        assert "move_manipulator" in names
        assert "move_mobile_base" in names
        assert "control_gripper" in names
        assert "stop_robot" in names

    def test_move_manipulator_schema(self):
        """Test move_manipulator schema"""
        tools = get_tool_definitions()
        move_tool = next(t for t in tools if t["function"]["name"] == "move_manipulator")

        params = move_tool["function"]["parameters"]["properties"]
        assert "movement_type" in params
        assert "direction" in params
        assert "distance" in params


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
