# LLM Robot Control System
## Unity Implementation with OpenAI API Integration

A comprehensive system for controlling robots in Unity using Large Language Models (LLMs) via natural language commands. This implementation translates conversational inputs into precise Inverse Kinematics (IK) target movements.

---

## 📋 Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Component Reference](#component-reference)
- [Configuration](#configuration)
- [Usage Examples](#usage-examples)
- [Troubleshooting](#troubleshooting)
- [API Reference](#api-reference)

---

## ✨ Features

- **Natural Language Control**: Command robots using everyday language
- **OpenAI Integration**: Leverages GPT-4/GPT-3.5 for intelligent command parsing
- **IK Target Control**: Precise end-effector positioning via Animation Rigging
- **Multi-Layer Safety**: Workspace bounds, collision detection, user confirmation
- **Web UI Interface**: HTML-based control panels (button and text input)
- **Performance Monitoring**: Real-time metrics and statistics
- **Emergency Stop**: Immediate halt with Space key
- **Command Caching**: Improved response times for repeated commands
- **Coordinate Translation**: Automatic cm-to-Unity-units conversion

### Coordinate System

- **Forward/Backward**: Z-axis (+forward, -backward)
- **Left/Right**: X-axis (-left, +right)
- **Up/Down**: Y-axis (+up, -down)

Example: "Move forward 30cm" → IK target moves +0.3 units on Z-axis

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────┐
│              User Interface Layer                   │
│  ┌───────────────┐      ┌──────────────────────┐  │
│  │ HTML UI       │      │ Unity UI (TextMeshPro)│  │
│  │ - text_input  │      │ - Input Field         │  │
│  │ - button_input│      │ - Feedback Display    │  │
│  └───────┬───────┘      └──────────┬───────────┘  │
└──────────┼──────────────────────────┼──────────────┘
           │                          │
┌──────────▼──────────────────────────▼──────────────┐
│            LLMRobotControlManager                   │
│  - Command orchestration                            │
│  - User confirmation workflow                       │
│  - Command caching                                  │
└──────────┬──────────────────────────┬──────────────┘
           │                          │
     ┌─────▼─────┐             ┌─────▼──────┐
     │ OpenAI    │             │ Command    │
     │ Client    │             │ Validator  │
     └─────┬─────┘             └─────┬──────┘
           │                         │
           ▼                         ▼
     ┌──────────────────────────────────┐
     │     IKRobotController            │
     │  - Smooth movement interpolation │
     │  - Animation Rigging integration │
     └──────────────────────────────────┘
```

---

## 📦 Prerequisites

### Required
- Unity 2021.3 LTS or later
- .NET 4.x equivalent or higher
- OpenAI API Key ([Get one here](https://platform.openai.com/api-keys))

### Recommended Unity Packages
- **TextMeshPro** (Built-in) - For UI text
- **Animation Rigging** (com.unity.animation.rigging) - For IK constraints
- **Newtonsoft Json** (com.unity.nuget.newtonsoft-json) - Optional, for JSON handling

### Optional
- Unity WebGL Export (for web-based deployment)
- Robot model with proper rigging

---

## 🚀 Installation

### Step 1: Install Unity Packages

```
Window → Package Manager
```

Install:
1. **TextMeshPro** (if prompted on first use)
2. **Animation Rigging**
   - Search: "Animation Rigging"
   - Click: Install

### Step 2: Import Scripts

All scripts are located in:
```
Assets/Scripts/LLMRobotControl/
```

Scripts included:
- `RobotCommand.cs` - Data structures
- `RobotControlConfig.cs` - Configuration ScriptableObject
- `OpenAIClient.cs` - API communication
- `OpenAIResponseParser.cs` - Response parsing
- `CommandValidator.cs` - Safety validation
- `IKRobotController.cs` - IK control
- `LLMRobotControlManager.cs` - Main orchestrator
- `WebUIBridge.cs` - HTML UI integration
- `PerformanceMonitor.cs` - Metrics tracking
- `EmergencyStopSystem.cs` - Emergency stop

### Step 3: Create Configuration Asset

```
Assets → Create → Robot Control → Config
```

Name it: `RobotControlConfig`

### Step 4: Configure API Key

**⚠️ IMPORTANT: Never commit API keys to version control!**

1. Select `RobotControlConfig` asset
2. In Inspector, enter your OpenAI API key
3. Configure settings:
   - Model: `gpt-4-turbo` (recommended) or `gpt-3.5-turbo`
   - Workspace bounds
   - Safety constraints

---

## ⚡ Quick Start

### Method 1: Scene Setup (Recommended)

#### 1. Create Robot Control System

```
1. Create empty GameObject: "RobotControlSystem"
2. Add components in order:
   - RobotControlConfig (assign the ScriptableObject)
   - OpenAIClient
   - CommandValidator
   - IKRobotController
   - LLMRobotControlManager
   - WebUIBridge (optional)
   - PerformanceMonitor (optional)
   - EmergencyStopSystem (optional)
```

#### 2. Setup IK Target

```
1. Create empty GameObject: "IK_Target"
2. Position at desired end-effector location
3. Add visual indicator (small sphere) for debugging
4. Assign to IKRobotController → IK Target field
```

#### 3. Setup Animation Rigging (If using Unity Animation Rigging)

```
On your robot GameObject:
1. Add "Rig Builder" component
2. Create child GameObject: "Rig Layer"
3. Add "Rig" component to Rig Layer
4. Add to Rig Builder's Rig List

Create IK Constraint:
1. Create child of Rig Layer: "IK_Constraint"
2. Add "Two Bone IK Constraint" component
3. Configure:
   - Root: Upper arm/leg bone
   - Mid: Lower arm/leg bone
   - Tip: Hand/foot bone
   - Target: IK_Target GameObject
   - Target Position Weight: 1
```

#### 4. Setup UI (Optional - Unity UI)

```
Create Canvas:
1. GameObject → UI → Canvas
2. Add UI elements:
   - Input Field (TMP_InputField) for command input
   - Text (TextMeshProUGUI) for feedback
   - Text (TextMeshProUGUI) for status
   - Button for Confirm
   - Button for Cancel
   - Panel for confirmation dialog
```

#### 5. Connect References

In `LLMRobotControlManager`:
```
- Config → RobotControlConfig asset
- OpenAI Client → OpenAIClient component
- Command Validator → CommandValidator component
- IK Controller → IKRobotController component
- UI components → Respective UI elements
```

In `OpenAIClient` and `CommandValidator`:
```
- Config → RobotControlConfig asset
```

In `IKRobotController`:
```
- Config → RobotControlConfig asset
- IK Target → IK_Target GameObject
- IK Constraint → TwoBoneIKConstraint (if using Animation Rigging)
- Rig → Rig component (if using Animation Rigging)
```

### Method 2: Web UI Interface

#### 1. Enable Web Server

```
WebUIBridge component:
- Enable Web Server: ✓
- Web Server Port: 8080
```

#### 2. Access HTML UI

Run the scene, then open in browser:
- Text Input: `http://localhost:8080/text_input.html`
- Button Input: `http://localhost:8080/button_input.html`

The HTML files from `Assets/Report/` will be served automatically.

---

## 🎮 Usage Examples

### Natural Language Commands

```csharp
// From Unity UI or Web UI:
"Move forward 30cm"
"Go backward 20cm slowly"
"Move left 10cm"
"Raise your hand to 1.5 meters"
"Move to position x=0.5, y=1.0, z=0.3"
"Stop"
```

### Programmatic Control

```csharp
using LLMRobotControl;

public class MyController : MonoBehaviour
{
    [SerializeField] private LLMRobotControlManager controlManager;

    void Start()
    {
        // Process command programmatically
        controlManager.ProcessCommand("Move forward 30cm");

        // Register for events
        controlManager.OnCommandReceived += OnCommand;
        controlManager.OnCommandValidated += OnValidated;
        controlManager.OnCommandFailed += OnFailed;
    }

    void OnCommand(string command)
    {
        Debug.Log($"Command received: {command}");
    }

    void OnValidated(RobotCommand cmd)
    {
        Debug.Log($"Command validated: {cmd}");
    }

    void OnFailed(string error)
    {
        Debug.LogError($"Command failed: {error}");
    }
}
```

### Emergency Stop

```csharp
// Trigger emergency stop
if (Input.GetKeyDown(KeyCode.Space))
{
    controlManager.EmergencyStop();
}

// Or use EmergencyStopSystem component
var emergencySystem = GetComponent<EmergencyStopSystem>();
emergencySystem.ExecuteEmergencyStop();
```

---

## ⚙️ Configuration

### RobotControlConfig Settings

#### OpenAI API
```
API Key: [Your OpenAI API key]
Model: gpt-4-turbo
API Timeout: 30s
```

#### Workspace Bounds
```
Min: (-1, 0, -1)
Max: (1, 2, 1)
```

#### Safety Constraints
```
Max Single Movement: 1.0m (100cm)
Min Distance to Obstacles: 0.2m (20cm)
Obstacle Layer: Obstacles
```

#### Movement Settings
```
Movement Curve: EaseInOut (default)
Position Threshold: 0.01m
```

#### User Interaction
```
Enable User Confirmation: Yes
Confirmation Timeout: 10s
```

#### Performance
```
Min API Call Interval: 1.0s
Max Cache Size: 50
```

#### Coordinate System
```
Centimeters to Unity Units: 0.01 (1cm = 0.01 Unity units)
```

---

## 🔧 Component Reference

### Core Components

#### LLMRobotControlManager
Main orchestrator that coordinates all components.

**Public Methods:**
- `ProcessCommand(string command)` - Process natural language command
- `ConfirmCommand()` - Confirm pending movement
- `CancelCommand()` - Cancel pending movement
- `EmergencyStop()` - Immediate halt
- `GetStatistics()` - Get performance stats

**Events:**
- `OnCommandReceived` - When command is received
- `OnCommandValidated` - When command passes validation
- `OnCommandFailed` - When command fails

#### OpenAIClient
Handles OpenAI API communication.

**Public Methods:**
- `SendChatRequest(string message, Action<string> onSuccess, Action<string> onError)` - Send API request
- `CanMakeCall()` - Check rate limiting
- `GetStats()` - Get API statistics

#### CommandValidator
Validates commands for safety.

**Public Methods:**
- `ValidateCommand(RobotCommand command, Vector3 currentPosition)` - Validate command

**Returns:** `ValidationResult` with:
- `isValid` - Validation passed
- `errorMessage` - Error description
- `safePosition` - Safe alternative position
- `validatedCommand` - The command

#### IKRobotController
Controls IK target movement.

**Public Methods:**
- `MoveToPosition(Vector3 targetPos, float duration, float speed)` - Execute movement
- `StopCurrentMovement()` - Stop current movement
- `IsMoving()` - Check if moving
- `GetCurrentPosition()` - Get IK target position
- `GetDistanceToTarget()` - Distance remaining
- `GetMovementProgress()` - Progress (0-1)

**Events:**
- `OnMovementStarted` - Movement begins
- `OnMovementCompleted` - Movement ends
- `OnMovementFailed` - Movement fails

#### WebUIBridge
Bridge between Unity and HTML UI.

**Public Methods:**
- `ReceiveMessage(string message)` - Receive from web
- `SendMessage(string message)` - Send to web
- `SendCommandFromExternal(string command)` - External API

---

## 🐛 Troubleshooting

### Common Issues

#### 1. API Key Error

**Symptom:** "Configuration error: API Key is not set"

**Solution:**
1. Open `RobotControlConfig` ScriptableObject
2. Paste your OpenAI API key
3. Never commit this file to version control!

#### 2. IK Target Not Moving

**Symptoms:** Command accepted but robot doesn't move

**Diagnosis:**
1. Check Rig Builder weight = 1.0
2. Verify TwoBoneIKConstraint is enabled
3. Ensure constraint has valid bone references
4. Check IK Target is assigned in IKRobotController

**Solution:**
```csharp
// Force rig update
rig.weight = 0f;
yield return null;
rig.weight = 1f;
```

#### 3. API Timeout

**Symptom:** "Request timeout" errors

**Solutions:**
- Increase timeout: Config → API Timeout (try 60s)
- Check internet connection
- Verify OpenAI service status
- Add retry logic (see code examples)

#### 4. Jittery Movement

**Symptom:** Robot movement is not smooth

**Solutions:**
1. Adjust movement curve in Config
2. Use FixedUpdate for physics
3. Increase IK solver iterations
4. Use SmoothDamp interpolation

#### 5. Commands Not Parsed

**Symptom:** LLM returns unexpected responses

**Solutions:**
1. Check system prompt in Config
2. Use more specific commands
3. Try different model (gpt-4 vs gpt-3.5-turbo)
4. Add examples to system prompt

#### 6. Web UI Not Loading

**Symptom:** `http://localhost:8080` not accessible

**Solutions:**
1. Check WebUIBridge enabled
2. Verify port not in use
3. Check firewall settings
4. Try different port
5. Ensure scene is playing

### Debug Mode

Enable verbose logging:
```csharp
// In each component
Debug.Log("[ComponentName] Detailed message");
```

Check Console for:
- `[OpenAIClient]` - API communication
- `[Parser]` - Response parsing
- `[CommandValidator]` - Validation results
- `[IKRobotController]` - Movement execution
- `[ControlManager]` - Orchestration flow

---

## 📚 API Reference

### RobotCommand Structure

```csharp
public class RobotCommand
{
    public string movementType;      // "relative" or "absolute"
    public string direction;         // "forward", "backward", etc.
    public float distance;           // In centimeters
    public Vector3? absolutePosition; // For absolute moves
    public float speed;              // 0.1 - 2.0
    public float duration;           // 0.1 - 10.0 seconds
}
```

### ValidationResult Structure

```csharp
public class ValidationResult
{
    public bool isValid;
    public string errorMessage;
    public Vector3 safePosition;
    public RobotCommand validatedCommand;
}
```

### OpenAI Function Schema

The system uses this function definition:

```json
{
  "name": "move_robot_ik",
  "description": "Move the robot's IK target",
  "parameters": {
    "type": "object",
    "properties": {
      "movement_type": {
        "type": "string",
        "enum": ["relative", "absolute"]
      },
      "direction": {
        "type": "string",
        "enum": ["forward", "backward", "left", "right", "up", "down"]
      },
      "distance": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 100.0
      },
      "position": {
        "type": "object",
        "properties": {
          "x": { "type": "number" },
          "y": { "type": "number" },
          "z": { "type": "number" }
        }
      },
      "speed": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 2.0,
        "default": 1.0
      },
      "duration": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 10.0,
        "default": 2.0
      }
    },
    "required": ["movement_type"]
  }
}
```

---

## 🎯 Best Practices

### 1. Safety First
- Always set appropriate workspace bounds
- Enable user confirmation for critical movements
- Test collision detection thoroughly
- Use emergency stop during development

### 2. Performance Optimization
- Enable command caching
- Set appropriate API call intervals
- Use lower token limits for faster responses
- Consider using gpt-3.5-turbo for speed

### 3. Error Handling
- Always check validation results
- Handle API timeouts gracefully
- Provide clear user feedback
- Log errors for debugging

### 4. Testing
- Start with small movements
- Test edge cases (boundaries, collisions)
- Verify emergency stop works
- Monitor performance metrics

---

## 📝 Example Scene Setup Checklist

- [ ] Import all scripts to `Assets/Scripts/LLMRobotControl/`
- [ ] Install Animation Rigging package
- [ ] Install TextMeshPro
- [ ] Create RobotControlConfig ScriptableObject
- [ ] Add OpenAI API key to config
- [ ] Configure workspace bounds
- [ ] Create RobotControlSystem GameObject
- [ ] Add all required components
- [ ] Create IK_Target GameObject
- [ ] Setup Animation Rigging (if used)
- [ ] Connect all component references
- [ ] Create UI elements (optional)
- [ ] Test with simple command
- [ ] Verify emergency stop works
- [ ] Configure safety constraints
- [ ] Enable web UI (optional)

---

## 📄 License

Based on research document: `LLM_Robot_Control_System_Design.md`

---

## 🤝 Contributing

For issues, improvements, or questions:
1. Check troubleshooting section
2. Review console logs
3. Verify component references
4. Test with minimal scene

---

## 📞 Support

For help with:
- **OpenAI API**: https://platform.openai.com/docs
- **Unity Animation Rigging**: https://docs.unity3d.com/Packages/com.unity.animation.rigging
- **Unity Scripting**: https://docs.unity3d.com/ScriptReference/

---

**Last Updated:** 2025-11-03
**Version:** 1.0
**Unity Version:** 2021.3 LTS+
