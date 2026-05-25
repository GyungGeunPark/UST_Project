# LLM Function/Tool Definitions for Robot Control

from typing import List, Dict, Any


def get_tool_definitions() -> List[Dict[str, Any]]:
    """Get tool definitions for OpenAI function calling"""
    return [
        {
            "type": "function",
            "function": {
                "name": "move_manipulator",
                "description": "Move the robot manipulator's end-effector. Use relative movement for direction-based commands (forward, backward, up, down, left, right) with distance. Use absolute movement for specific coordinate positions.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "movement_type": {
                            "type": "string",
                            "enum": ["relative", "absolute"],
                            "description": "Type of movement: 'relative' for direction-based, 'absolute' for coordinate-based"
                        },
                        "direction": {
                            "type": "string",
                            "enum": ["forward", "backward", "left", "right", "up", "down"],
                            "description": "Direction for relative movement"
                        },
                        "distance": {
                            "type": "number",
                            "minimum": 0.1,
                            "maximum": 100.0,
                            "description": "Distance in centimeters for relative movement"
                        },
                        "position": {
                            "type": "object",
                            "properties": {
                                "x": {"type": "number", "description": "X coordinate in meters"},
                                "y": {"type": "number", "description": "Y coordinate in meters"},
                                "z": {"type": "number", "description": "Z coordinate in meters"}
                            },
                            "description": "Target position for absolute movement"
                        },
                        "speed": {
                            "type": "number",
                            "minimum": 0.1,
                            "maximum": 2.0,
                            "default": 1.0,
                            "description": "Speed multiplier (0.1 = slow, 1.0 = normal, 2.0 = fast)"
                        }
                    },
                    "required": ["movement_type"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "move_mobile_base",
                "description": "Move the mobile robot base. Positive linear velocity moves forward, negative moves backward. Positive angular velocity turns left (counterclockwise), negative turns right (clockwise).",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "linear_velocity": {
                            "type": "number",
                            "minimum": -1.0,
                            "maximum": 1.0,
                            "description": "Linear velocity in m/s. Positive = forward, negative = backward"
                        },
                        "angular_velocity": {
                            "type": "number",
                            "minimum": -1.5,
                            "maximum": 1.5,
                            "default": 0.0,
                            "description": "Angular velocity in rad/s. Positive = left turn, negative = right turn"
                        },
                        "duration": {
                            "type": "number",
                            "minimum": 0.1,
                            "maximum": 10.0,
                            "default": 2.0,
                            "description": "Duration in seconds"
                        }
                    },
                    "required": ["linear_velocity"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "control_gripper",
                "description": "Open or close the robot gripper. Use 'open' to release objects, 'close' to grasp objects.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "action": {
                            "type": "string",
                            "enum": ["open", "close"],
                            "description": "Gripper action: 'open' or 'close'"
                        }
                    },
                    "required": ["action"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "stop_robot",
                "description": "Stop all robot motion immediately. Use this for any stop or halt command.",
                "parameters": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            }
        }
    ]


def get_anthropic_tool_definitions() -> List[Dict[str, Any]]:
    """Get tool definitions for Anthropic function calling"""
    return [
        {
            "name": "move_manipulator",
            "description": "Move the robot manipulator's end-effector. Use relative movement for direction-based commands (forward, backward, up, down, left, right) with distance. Use absolute movement for specific coordinate positions.",
            "input_schema": {
                "type": "object",
                "properties": {
                    "movement_type": {
                        "type": "string",
                        "enum": ["relative", "absolute"],
                        "description": "Type of movement: 'relative' for direction-based, 'absolute' for coordinate-based"
                    },
                    "direction": {
                        "type": "string",
                        "enum": ["forward", "backward", "left", "right", "up", "down"],
                        "description": "Direction for relative movement"
                    },
                    "distance": {
                        "type": "number",
                        "description": "Distance in centimeters for relative movement (0.1 to 100.0)"
                    },
                    "position": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number", "description": "X coordinate in meters"},
                            "y": {"type": "number", "description": "Y coordinate in meters"},
                            "z": {"type": "number", "description": "Z coordinate in meters"}
                        },
                        "description": "Target position for absolute movement"
                    },
                    "speed": {
                        "type": "number",
                        "description": "Speed multiplier (0.1 = slow, 1.0 = normal, 2.0 = fast)"
                    }
                },
                "required": ["movement_type"]
            }
        },
        {
            "name": "move_mobile_base",
            "description": "Move the mobile robot base. Positive linear velocity moves forward, negative moves backward. Positive angular velocity turns left, negative turns right.",
            "input_schema": {
                "type": "object",
                "properties": {
                    "linear_velocity": {
                        "type": "number",
                        "description": "Linear velocity in m/s (-1.0 to 1.0). Positive = forward"
                    },
                    "angular_velocity": {
                        "type": "number",
                        "description": "Angular velocity in rad/s (-1.5 to 1.5). Positive = left turn"
                    },
                    "duration": {
                        "type": "number",
                        "description": "Duration in seconds (0.1 to 10.0)"
                    }
                },
                "required": ["linear_velocity"]
            }
        },
        {
            "name": "control_gripper",
            "description": "Open or close the robot gripper.",
            "input_schema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["open", "close"],
                        "description": "Gripper action"
                    }
                },
                "required": ["action"]
            }
        },
        {
            "name": "stop_robot",
            "description": "Stop all robot motion immediately.",
            "input_schema": {
                "type": "object",
                "properties": {},
                "required": []
            }
        }
    ]


# Command type mapping
FUNCTION_TO_COMMAND_TYPE = {
    "move_manipulator": "MOVE_MANIPULATOR",
    "move_mobile_base": "MOVE_MOBILE_BASE",
    "control_gripper": "CONTROL_GRIPPER",
    "stop_robot": "STOP_ROBOT",
}
