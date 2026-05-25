# Isaac Sim LLM Robot Control System

A natural language interface for controlling robots in NVIDIA Isaac Sim using Large Language Models (LLM).

## Features

- **Natural Language Control**: Control robots using natural language commands in Korean or English
- **LLM Integration**: Supports OpenAI GPT-4 and Anthropic Claude for command interpretation
- **Web Interface**: Real-time control panel with WebSocket support
- **5-Layer Safety System**: Comprehensive safety validation at multiple levels
- **Modular Architecture**: Clean separation of concerns for easy extension

## Architecture

```
LLM Robot Control System
├── core/               # Core business logic
│   ├── control_manager.py   # Main orchestrator
│   ├── llm_client.py        # LLM API client
│   ├── robot_command.py     # Command data structures
│   ├── response_parser.py   # LLM response parsing
│   └── command_validator.py # Command validation
├── isaac_interface/    # Isaac Sim interface
│   ├── robot_controller.py  # Main robot controller
│   ├── mobile_base.py       # Mobile base control
│   ├── manipulator.py       # Arm control
│   ├── gripper.py           # Gripper control
│   └── ik_solver.py         # IK solver wrapper
├── safety/             # Safety systems
│   ├── emergency_stop.py    # Emergency stop system
│   ├── workspace_validator.py # Workspace validation
│   └── collision_checker.py # Collision detection
├── web/                # Web server
│   ├── server.py            # FastAPI server
│   └── static/              # Frontend files
├── config/             # Configuration files
└── scripts/            # Entry points
```

## Quick Start

### Prerequisites

- Python 3.10+
- OpenAI API key or Anthropic API key
- (Optional) NVIDIA Isaac Sim for full functionality

### Installation

1. Install dependencies:
```bash
cd ust_ws/LLM
pip install -r requirements.txt
```

2. Set environment variables:
```bash
export OPENAI_API_KEY=your-api-key
# or
export ANTHROPIC_API_KEY=your-api-key
```

### Running Standalone (Without Isaac Sim)

```bash
python scripts/run_standalone.py
```

This starts the web server at http://localhost:8000

### Running with Isaac Sim

```bash
~/.local/share/ov/pkg/isaac-sim-*/python.sh scripts/run_with_isaac.py
```

## Configuration

Configuration files are located in `config/`:

- `robot_config.yaml`: Robot parameters (joints, limits)
- `workspace_config.yaml`: Workspace bounds and safety limits
- `llm_config.yaml`: LLM provider settings
- `server_config.yaml`: Web server configuration

## API Reference

### REST API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/command` | POST | Process natural language command |
| `/api/quick_command` | POST | Execute quick command |
| `/api/status` | GET | Get robot status |
| `/api/emergency_stop` | POST | Trigger emergency stop |
| `/api/reset` | POST | Reset from emergency stop |
| `/api/stats` | GET | Get system statistics |

### WebSocket

Connect to `ws://localhost:8000/ws` for real-time updates.

Message types:
- `status`: Robot status updates (10Hz)
- `command_result`: Command execution results
- `emergency_stop`: Emergency stop events

## Supported Commands

### Korean Examples
- "앞으로 10cm 이동해줘"
- "위로 올려"
- "그리퍼 열어"
- "정지"

### English Examples
- "Move forward 10cm"
- "Raise up"
- "Open gripper"
- "Stop"

## Safety System

5-layer safety architecture:

1. **LLM Safety Instructions**: Safety rules in system prompt
2. **JSON Schema Validation**: Parameter range limits
3. **Command Validation**: Workspace and velocity checks
4. **User Confirmation**: Optional confirmation for risky actions
5. **Emergency Stop**: Immediate halt capability

## Development

### Running Tests

```bash
pytest tests/ -v
```

### Project Structure

```
LLM/
├── config/           # YAML configuration files
├── core/             # Core modules
├── isaac_interface/  # Isaac Sim interface
├── safety/           # Safety systems
├── web/              # Web server and frontend
├── utils/            # Utility functions
├── scripts/          # Entry point scripts
├── tests/            # Test files
├── requirements.txt  # Dependencies
└── README.md         # This file
```

## License

MIT License
