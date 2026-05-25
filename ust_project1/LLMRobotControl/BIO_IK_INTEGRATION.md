# Bio IK Integration Guide

## Overview

The LLM Robot Control System now fully supports **Bio IK** for advanced inverse kinematics solving. This integration allows natural language commands to control robots using Bio IK's powerful solver.

## What Changed

### Files Modified

1. **RobotControlSetupHelper.cs**
   - Now detects Bio IK components in the scene
   - Automatically assigns Bio IK component to IKRobotController
   - Finds and configures Bio IK Position objectives
   - Updated setup dialog to show Bio IK status

2. **IKRobotController.cs**
   - Added Bio IK component references
   - Added Bio IK Position objective support
   - Auto-detection of Position objectives matching IK target
   - Helper methods for Bio IK configuration

## How It Works

### Architecture

```
Natural Language Command
        ↓
OpenAI API (GPT-4)
        ↓
Robot Command Parser
        ↓
Command Validator
        ↓
IKRobotController
        ↓
Bio IK Position Objective
        ↓
Bio IK Solver
        ↓
Robot Movement
```

### Component Integration

The `IKRobotController` now supports three IK methods:

1. **Bio IK** (Recommended)
   - Uses `BioIK.BioIK` component
   - Controls via `Position` objective
   - Advanced solver with multiple constraints
   - Best for complex robot configurations

2. **Unity Animation Rigging** (Optional)
   - Uses `TwoBoneIKConstraint`
   - Good for humanoid characters
   - Simpler setup

3. **Direct IK Target Control** (Fallback)
   - Moves IK target transform directly
   - Works with any IK system
   - Simplest approach

## Setup Instructions

### Automatic Setup (Recommended)

1. **Ensure Bio IK is in Scene**
   - Your robot should have a `BioIK.BioIK` component
   - At least one segment with a `Position` objective

2. **Run Setup Tool**
   ```
   Unity → Tools → Robot Control → Setup Scene
   ```

3. **What Happens**
   - Creates RobotControlSystem GameObject
   - Creates IK_Target GameObject
   - Detects Bio IK component automatically
   - Assigns Bio IK to IKRobotController
   - Finds matching Position objective
   - Displays setup status

4. **Configure Position Objective (if needed)**
   - Select your Bio IK component
   - Find the Position objective
   - Set its Target to `IK_Target` transform
   - Set desired weight (1.0 recommended)

### Manual Setup

If automatic setup doesn't find Bio IK:

1. **Add Components Manually**
   - Create empty GameObject: `RobotControlSystem`
   - Add all required components (see README.md)

2. **Create IK Target**
   ```csharp
   GameObject ikTarget = new GameObject("IK_Target");
   ikTarget.transform.position = /* end-effector position */;
   ```

3. **Assign Bio IK**
   - Select `RobotControlSystem`
   - Find `IKRobotController` component
   - Assign `Bio IK` field → your BioIK.BioIK component
   - Assign `Bio IK Position Objective` → Position objective from Bio IK

4. **Configure Position Objective**
   - In Bio IK component
   - Select Position objective
   - Set Target → `IK_Target`

## Validation

After setup, validate your configuration:

```
Unity → Tools → Robot Control → Validate Setup
```

### Expected Results

```
✓ LLMRobotControlManager present
✓ OpenAIClient present
✓ CommandValidator present
✓ IKRobotController present
✓ Configuration assigned
✓ Configuration valid
✓ IK Target assigned
✓ Bio IK component assigned
✓ Bio IK Position objective assigned
```

## Usage Examples

### Basic Movement

```
User: "Move forward 30cm"

Flow:
1. OpenAI API converts to: {direction: "forward", distance: 30}
2. IKRobotController calculates target position
3. Moves IK_Target to new position
4. Bio IK solver adjusts robot joints
5. Robot moves smoothly to target
```

### Absolute Positioning

```
User: "Go to position x=0.5, y=1.0, z=0.3"

Flow:
1. OpenAI API converts to: {position: {x:0.5, y:1.0, z:0.3}}
2. IKRobotController sets target position
3. Bio IK solver computes joint angles
4. Robot reaches target position
```

## Bio IK Position Objective Configuration

### Recommended Settings

```
Position Objective:
├─ Target: IK_Target (Transform)
├─ Weight: 1.0
├─ Use X: true
├─ Use Y: true
├─ Use Z: true
└─ Motion Type: Instant (or Smooth)
```

### Multiple Objectives

You can have multiple Position objectives:

```
Bio IK Component:
├─ Segment 1 (Right Arm):
│   └─ Position Objective 1: Right_Hand_Target
├─ Segment 2 (Left Arm):
│   └─ Position Objective 2: Left_Hand_Target
└─ Segment 3 (Head):
    └─ Position Objective 3: Head_Target
```

For multi-arm control:
- Create multiple `IKRobotController` instances
- Each controlling different Position objective
- Each with unique IK_Target

## Troubleshooting

### Issue: Bio IK Not Detected

**Symptoms:**
```
[Setup] No Bio IK component found in scene
```

**Solutions:**
1. Ensure `BioIK.BioIK` component exists on robot
2. Robot should be in the active scene
3. Bio IK package is installed

### Issue: Position Objective Not Found

**Symptoms:**
```
[Setup] No Bio IK Position objective found
```

**Solutions:**
1. Add Position objective to Bio IK segment
2. Manually assign in Inspector:
   - Select `RobotControlSystem`
   - `IKRobotController` → `Bio IK Position Objective`

### Issue: Robot Doesn't Move

**Symptoms:**
- Command executes but robot doesn't move
- IK_Target moves but joints don't update

**Solutions:**
1. Check Bio IK enabled and weight > 0
2. Verify Position objective:
   - Target assigned to IK_Target
   - Weight > 0
   - Uses X, Y, Z enabled
3. Check solver settings:
   - Iterations > 10
   - Update enabled

### Issue: Jittery Movement

**Symptoms:**
- Robot shakes during movement
- Unstable joint positions

**Solutions:**
1. Increase Bio IK solver iterations (20-50)
2. Use smooth motion type in Position objective
3. Adjust movement duration in commands
4. Enable damping in Bio IK settings

## API Reference

### IKRobotController Methods

```csharp
// Set Bio IK component
public void SetBioIK(BioIK.BioIK bioIKComponent)

// Set Bio IK Position objective
public void SetBioIKPositionObjective(Position objective)

// Move to position (works with Bio IK)
public void MoveToPosition(Vector3 targetPosition, float duration, float speed)

// Get current position
public Vector3 GetCurrentPosition()

// Check if moving
public bool IsMoving()
```

### Setup Helper Methods

```csharp
// Find Bio IK Position objective
private static Position FindBioIKPositionObjective(BioIK.BioIK bioIK, Transform targetTransform)
```

## Performance Considerations

### Bio IK vs Other Methods

| Method | Setup Complexity | Performance | Flexibility | Best For |
|--------|-----------------|-------------|-------------|----------|
| Bio IK | Medium | Good | Excellent | Complex robots |
| Animation Rigging | Low | Excellent | Good | Humanoids |
| Direct Target | Very Low | Best | Limited | Simple IK |

### Optimization Tips

1. **Solver Iterations**
   - Start with 20 iterations
   - Increase if accuracy needed
   - Decrease for better performance

2. **Update Frequency**
   - Use FixedUpdate for Bio IK
   - Reduce update rate if possible
   - Cache computations

3. **Constraint Count**
   - Minimize active objectives
   - Use appropriate weights
   - Disable unused segments

## Advanced Usage

### Custom Constraints

You can combine Position objectives with other Bio IK constraints:

```csharp
Bio IK Segment:
├─ Position Objective (priority: 1.0)
├─ Orientation Objective (priority: 0.5)
├─ LookAt Objective (priority: 0.3)
└─ Joint Limit Constraint (priority: 1.0)
```

### Dynamic Target Switching

```csharp
// Switch between multiple targets
ikController.GetComponent<IKRobotController>().SetBioIKPositionObjective(objective1);
// ... after some time
ikController.GetComponent<IKRobotController>().SetBioIKPositionObjective(objective2);
```

### Workspace Constraints

Combine with CommandValidator:

```csharp
// In RobotControlConfig
workspaceMin = new Vector3(-1, 0, -1);
workspaceMax = new Vector3(1, 2, 1);

// Position objectives automatically respect workspace
```

## Migration from Hybrid IK

If you were using Hybrid IK before:

1. **Remove Hybrid IK References**
   - Old code searched for `HybridIK_Ex`
   - Now searches for `BioIK.BioIK`

2. **Update Controller Type**
   - Bio IK uses different API
   - Position objectives instead of direct targets

3. **No Code Changes Required**
   - IKRobotController handles both methods
   - Automatic fallback to direct target control

## Testing

### Quick Test

1. **Setup System**
   ```
   Tools → Robot Control → Setup Scene
   ```

2. **Validate**
   ```
   Tools → Robot Control → Validate Setup
   ```

3. **Play Mode**
   - Press Play
   - Type command: `"Move forward 30cm"`
   - Observe Bio IK solving in real-time

### Debug Visualization

Enable in Scene view:
- Green sphere: IK_Target current position
- Red sphere: Target destination
- Yellow line: Movement path
- Bio IK gizmos: Joint angles, segment bounds

## FAQ

**Q: Can I use both Bio IK and Animation Rigging?**
A: Yes, IKRobotController supports both. Bio IK takes priority if assigned.

**Q: Do I need Bio IK for this system to work?**
A: No, it's optional. Direct IK target control works without Bio IK.

**Q: How do I control multiple end-effectors?**
A: Create multiple IKRobotController instances, each with different Position objectives.

**Q: Can I use custom Bio IK objectives?**
A: Yes, but you'll need to modify IKRobotController to support them.

**Q: Performance impact of Bio IK?**
A: Minimal if configured correctly (20-30 iterations). Adjust based on robot complexity.

## References

- [Bio IK Documentation](https://assetstore.unity.com/packages/tools/animation/bio-ik-67819)
- [LLM Robot Control README](README.md)
- [Implementation Summary](IMPLEMENTATION_SUMMARY.md)
- [Quick Reference](QUICK_REFERENCE.md)

## Version History

### Version 1.1 (2025-11-03)
- Added Bio IK integration
- Auto-detection of Bio IK components
- Position objective configuration
- Updated setup and validation tools
- Comprehensive Bio IK documentation

### Version 1.0 (2025-11-03)
- Initial release
- Animation Rigging support only

---

**Bio IK Integration Complete! 🎊**

Your LLM Robot Control System now supports advanced Bio IK solving for precise and natural robot movements.
