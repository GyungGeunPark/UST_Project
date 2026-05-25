# LLM Robot Control - Quick Reference Card

## 🚀 5-Minute Setup

```
1. Tools → Robot Control → Create Config Asset
2. Add OpenAI API key to config
3. Tools → Robot Control → Setup Scene
4. Tools → Robot Control → Validate Setup
5. Press Play → Type command → Done!
```

---

## 📍 File Locations

```
Scripts:        Assets/Scripts/LLMRobotControl/
Config:         Assets/[Your Location]/RobotControlConfig.asset
HTML UI:        Assets/Report/text_input.html, button_input.html
Documentation:  Assets/Scripts/LLMRobotControl/README.md
```

---

## 🎯 Coordinate System

```
Forward  →  +Z axis  →  "Move forward 30cm"  →  (0, 0, +0.3)
Backward →  -Z axis  →  "Move backward 20cm" →  (0, 0, -0.2)
Left     →  -X axis  →  "Move left 10cm"     →  (-0.1, 0, 0)
Right    →  +X axis  →  "Move right 15cm"    →  (+0.15, 0, 0)
Up       →  +Y axis  →  "Move up 5cm"        →  (0, +0.05, 0)
Down     →  -Y axis  →  "Move down 8cm"      →  (0, -0.08, 0)

Conversion: 1 cm = 0.01 Unity units (default)
```

---

## 💬 Command Examples

### Basic Movement
```
"Move forward 30cm"
"Go backward 20cm"
"Move left 10cm slowly"
"Move right 15cm quickly"
"Move up 5cm"
"Stop"
```

### Absolute Positioning
```
"Go to position x=0.5, y=1.0, z=0.3"
"Move to coordinates 0.5, 1.0, 0.3"
"Reach position x=0, y=1.5, z=0"
```

### Complex Commands
```
"Move forward 30cm then move left 10cm"
"Raise your hand to 1.5 meters"
"Move slowly forward for 5 seconds"
```

---

## ⚙️ Configuration Quick Settings

### Essential Settings
```
API Key:           [Your OpenAI API Key] ⚠️ NEVER COMMIT!
Model:             gpt-4-turbo (best) or gpt-3.5-turbo (fast)
API Timeout:       30s
```

### Safety (Default Values)
```
Workspace Min:     (-1, 0, -1)
Workspace Max:     (1, 2, 1)
Max Movement:      1.0m per command
Min Obstacle Dist: 0.2m
```

### Performance
```
API Call Interval: 1.0s minimum
Cache Size:        50 commands
User Confirmation: Enabled (optional)
Confirmation Time: 10s
```

---

## 🎮 Hotkeys

```
Space          Emergency Stop (instant halt)
R              Reset Emergency Stop
```

---

## 🔧 Unity Menu Commands

```
Tools → Robot Control →
  ├─ Setup Scene              (Create complete system)
  ├─ Create Config Asset      (Make configuration)
  ├─ Validate Setup           (Check everything)
  ├─ Open Documentation       (View README.md)
  └─ About                    (Version info)
```

---

## 📦 Required Components Setup

```
RobotControlSystem (GameObject)
  ├─ RobotControlConfig (assigned)
  ├─ OpenAIClient
  ├─ CommandValidator
  ├─ IKRobotController
  ├─ LLMRobotControlManager
  ├─ WebUIBridge (optional)
  ├─ PerformanceMonitor (optional)
  └─ EmergencyStopSystem (optional)

IK_Target (GameObject)
  └─ Visual indicator (sphere)
```

---

## 🌐 Web UI Access

```
Start Scene:
  - Text UI:   http://localhost:8080/text_input.html
  - Button UI: http://localhost:8080/button_input.html

Port: Configurable in WebUIBridge (default: 8080)
```

---

## 🐛 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| "API Key not set" | Add key to RobotControlConfig |
| Robot doesn't move | Check Rig Builder weight = 1.0 |
| Timeout errors | Increase timeout, check internet |
| Web UI 404 | Check HTML files in Report folder |
| Jittery motion | Adjust movement curve in config |
| Parse errors | Try simpler command, check model |

---

## 📊 Component Reference

### Core Flow
```
User Command
  ↓
LLMRobotControlManager (orchestrator)
  ↓
OpenAIClient (API call)
  ↓
OpenAIResponseParser (parse JSON)
  ↓
CommandValidator (safety check)
  ↓
IKRobotController (execute movement)
```

### Key Methods

#### LLMRobotControlManager
```csharp
ProcessCommand(string cmd)      // Main entry point
ConfirmCommand()                // Approve movement
CancelCommand()                 // Cancel movement
EmergencyStop()                 // Immediate halt
GetStatistics()                 // Get metrics
```

#### IKRobotController
```csharp
MoveToPosition(Vector3, float, float)  // Execute move
StopCurrentMovement()                  // Stop
IsMoving()                             // Check status
GetCurrentPosition()                   // Get position
GetMovementProgress()                  // Progress 0-1
```

---

## 📈 Performance Metrics

### Expected Performance
```
API Call:          1-3 seconds (first time)
Cached Command:    < 0.1 seconds
Movement:          0.1-10 seconds (configurable)
Success Rate:      > 95% (valid commands)
Memory Overhead:   < 100 MB
```

### Monitoring
```csharp
// Automatic tracking:
- API response times
- Movement durations
- Success/failure counts
- Cache hit rates
- FPS and memory
```

---

## 🔒 Safety Layers

```
1. LLM Instructions   ← System prompt with guidelines
2. JSON Schema        ← Type validation
3. Command Validator  ← Bounds + collision
4. User Confirmation  ← Human approval
5. Emergency Stop     ← Instant halt
```

---

## 🎨 Visualization (Scene View)

```
Green Cube:        Workspace bounds
Green Sphere:      IK target current position
Red Sphere:        Target destination (when moving)
Yellow Line:       Movement path
```

---

## 📝 Logging Levels

```csharp
// Enable verbose logging:
[OpenAIClient]        // API communication
[Parser]              // Response parsing
[CommandValidator]    // Validation results
[IKRobotController]   // Movement execution
[ControlManager]      // Orchestration flow
[WebUIBridge]         // Web communication
[Performance]         // Metrics updates
[EmergencyStop]       // Stop events
```

---

## 🔗 Important Links

### Documentation
- README.md - Full documentation
- IMPLEMENTATION_SUMMARY.md - Technical details
- LLM_Robot_Control_System_Design.md - Design spec

### External Resources
- OpenAI API: https://platform.openai.com/docs
- Unity Animation Rigging: https://docs.unity3d.com/Packages/com.unity.animation.rigging
- Unity Scripting: https://docs.unity3d.com/ScriptReference/

---

## 💡 Pro Tips

### 1. Start Simple
```
Begin with: "Move forward 10cm"
Not:        "Execute complex trajectory with obstacle avoidance"
```

### 2. Use Caching
```
Repeat common commands for instant execution
Cache automatically stores last 50 commands
```

### 3. Test Safety
```
Always test emergency stop before real deployment
Set conservative workspace bounds initially
```

### 4. Monitor Performance
```
Check PerformanceMonitor display
Review statistics regularly
Optimize based on metrics
```

### 5. Iterate Configuration
```
Start with small workspace
Gradually expand bounds
Tune speed and duration
Adjust safety margins
```

---

## 🎯 Success Checklist

Setup:
- [ ] Config asset created
- [ ] API key added (and NOT committed to git!)
- [ ] Scene setup complete
- [ ] Validation passes
- [ ] IK target assigned

Testing:
- [ ] Simple command works
- [ ] Emergency stop works
- [ ] User confirmation works
- [ ] Web UI accessible
- [ ] Performance acceptable

Safety:
- [ ] Workspace bounds set correctly
- [ ] Collision detection working
- [ ] Emergency stop tested
- [ ] Timeout handling works
- [ ] Error messages clear

---

## 🚨 Emergency Procedures

### If Robot Misbehaves
```
1. Press SPACE immediately (emergency stop)
2. Check Console for errors
3. Tools → Robot Control → Validate Setup
4. Review last command in feedback display
5. Adjust safety constraints if needed
```

### If System Hangs
```
1. Press ESC to exit Play mode
2. Check Console for stack traces
3. Verify API connectivity
4. Review timeout settings
5. Restart Unity if needed
```

---

## 📞 Getting Help

### Debug Steps
```
1. Check Console for error messages
2. Verify all component references
3. Run validation tool
4. Review configuration settings
5. Test with minimal scene
```

### Information to Provide
```
- Unity version
- Error messages from Console
- Configuration settings
- Command that failed
- Validation report
```

---

**Keep this card handy! 📌**

For detailed information, see [README.md](README.md)
