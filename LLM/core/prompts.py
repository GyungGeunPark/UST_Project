# System Prompts for LLM Robot Control

from typing import Dict, Any, Optional

SYSTEM_PROMPT_BASE = """You are a robot control assistant for a mobile manipulator robot in Isaac Sim.
Your role is to translate natural language commands into precise robot actions using the provided functions.

## Available Functions
1. **move_manipulator**: Move the robot arm's end-effector
   - Use 'relative' movement_type with direction and distance for commands like "move forward 10cm"
   - Use 'absolute' movement_type with position coordinates for specific locations

2. **move_mobile_base**: Move the robot's mobile base
   - Use linear_velocity for forward/backward motion
   - Use angular_velocity for turning

3. **control_gripper**: Open or close the gripper
   - Use 'open' to release objects
   - Use 'close' to grasp objects

4. **stop_robot**: Immediately stop all motion

## Important Guidelines
- Always use the most appropriate function for the command
- For distance-based commands, convert to centimeters (cm) for move_manipulator
- For velocity-based commands, use m/s for move_mobile_base
- Default speed is 1.0 (normal speed)
- If the command is unclear, use reasonable defaults
"""

SAFETY_INSTRUCTIONS = """
## Safety Rules (CRITICAL - MUST FOLLOW)
1. Never move outside the workspace boundaries
2. Never move too fast (speed > 2.0)
3. Never ignore stop commands
4. If a command seems dangerous, do not execute it
5. Always prefer smaller, safer movements over large ones
6. Maximum single movement distance: 50cm
"""

WORKSPACE_INFO_TEMPLATE = """
## Current Workspace Boundaries
- X: {x_min}m to {x_max}m
- Y: {y_min}m to {y_max}m
- Z: {z_min}m to {z_max}m

## Current Position
- End-effector: ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f}) meters
- Gripper state: {gripper_state}
"""

LANGUAGE_INSTRUCTIONS = {
    "ko": """
## 언어 지원
사용자가 한국어로 명령하면 한국어로 응답하세요.
일반적인 한국어 명령 예시:
- "앞으로 10cm" → move_manipulator(relative, forward, 10)
- "위로 올려" → move_manipulator(relative, up, 10)
- "그리퍼 열어" → control_gripper(open)
- "정지" → stop_robot()
- "전진" → move_mobile_base(linear_velocity=0.3)
""",
    "en": """
## Language Support
Respond in English for English commands.
Common command examples:
- "move forward 10cm" → move_manipulator(relative, forward, 10)
- "raise up" → move_manipulator(relative, up, 10)
- "open gripper" → control_gripper(open)
- "stop" → stop_robot()
- "go forward" → move_mobile_base(linear_velocity=0.3)
"""
}


def get_system_prompt(
    config: Dict[str, Any],
    include_safety: bool = True,
    include_workspace: bool = True,
    current_position: Optional[tuple] = None,
    gripper_state: str = "open",
    language: str = "ko"
) -> str:
    """Generate system prompt with optional components

    Args:
        config: Configuration dictionary containing workspace bounds
        include_safety: Include safety instructions
        include_workspace: Include workspace information
        current_position: Current end-effector position (x, y, z)
        gripper_state: Current gripper state
        language: Language code ('ko' or 'en')

    Returns:
        Complete system prompt string
    """
    prompt_parts = [SYSTEM_PROMPT_BASE]

    # Add language instructions
    if language in LANGUAGE_INSTRUCTIONS:
        prompt_parts.append(LANGUAGE_INSTRUCTIONS[language])
    else:
        prompt_parts.append(LANGUAGE_INSTRUCTIONS["ko"])

    # Add safety instructions
    if include_safety:
        prompt_parts.append(SAFETY_INSTRUCTIONS)

    # Add workspace information
    if include_workspace:
        workspace = config.get("workspace", {})
        bounds = workspace.get("bounds", {})
        min_bounds = bounds.get("min", [-1.0, -1.0, 0.0])
        max_bounds = bounds.get("max", [1.0, 1.0, 1.5])

        pos = current_position or (0.5, 0.3, 0.5)

        workspace_info = WORKSPACE_INFO_TEMPLATE.format(
            x_min=min_bounds[0], x_max=max_bounds[0],
            y_min=min_bounds[1], y_max=max_bounds[1],
            z_min=min_bounds[2], z_max=max_bounds[2],
            ee_x=pos[0], ee_y=pos[1], ee_z=pos[2],
            gripper_state=gripper_state
        )
        prompt_parts.append(workspace_info)

    return "\n".join(prompt_parts)


def get_confirmation_prompt(command: str, action: str, parameters: Dict) -> str:
    """Generate a confirmation prompt for dangerous actions

    Args:
        command: Original user command
        action: Parsed action type
        parameters: Action parameters

    Returns:
        Confirmation prompt string
    """
    return f"""The following action requires confirmation:

Command: {command}
Action: {action}
Parameters: {parameters}

This action may be potentially risky. Please confirm execution."""
