#!/usr/bin/env python3
"""Analyze gripper joint geometry to determine correct open/close values."""

from pxr import Usd, UsdPhysics, Sdf, Gf
import os, re

USD_PATH = "/workspace/isaaclab/ust_ws/isaac_file/ust_project1_fixed.usd"
stage = Usd.Stage.Open(USD_PATH)

print("=" * 70)
print("GRIPPER JOINT GEOMETRY ANALYSIS")
print("=" * 70)

for side in ["right", "left"]:
    print(f"\n--- {side.upper()} ARM GRIPPER ---")

    for finger in ["left", "right"]:
        joint_path = f"/World/Robot/open_manipulator_x_{side}/joints/{side}_gripper_{finger}_joint"
        prim = stage.GetPrimAtPath(joint_path)
        if not prim:
            print(f"  NOT FOUND: {joint_path}")
            continue

        axis = prim.GetAttribute("physics:axis").Get()
        localPos0 = prim.GetAttribute("physics:localPos0").Get()
        localRot0 = prim.GetAttribute("physics:localRot0").Get()
        lower = prim.GetAttribute("physics:lowerLimit").Get()
        upper = prim.GetAttribute("physics:upperLimit").Get()

        # Extract quaternion components from Quatf
        w = localRot0.GetReal()
        img = localRot0.GetImaginary()
        x, y, z = img[0], img[1], img[2]

        print(f"\n  {side}_gripper_{finger}_joint:")
        print(f"    axis:      {axis}")
        print(f"    localPos0: ({localPos0[0]:.4f}, {localPos0[1]:.4f}, {localPos0[2]:.4f})")
        print(f"    localRot0: (w={w:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f})")
        print(f"    limits:    [{lower:.4f}, {upper:.4f}]")

        # Compute joint Y-axis in parent frame using quaternion rotation
        q = Gf.Quatd(float(w), float(x), float(y), float(z))
        # Rotate (0,1,0) by quaternion: q * v * q_conjugate
        # For unit quaternion: v' = q * (0 + v) * q*
        # Using matrix approach
        mat = Gf.Matrix3d(q)
        joint_y = mat * Gf.Vec3d(0, 1, 0)
        print(f"    Joint Y in parent frame: ({joint_y[0]:.3f}, {joint_y[1]:.3f}, {joint_y[2]:.3f})")

        # Physical interpretation
        y_pos = localPos0[1]  # Y position of joint mount point on link5

        # For OpenMANIPULATOR-X gripper:
        # - Left finger at y=+0.021 (left side of gripper)
        # - Right finger at y=-0.021 (right side of gripper)
        # - "Close" = both fingers move toward y=0 (center)
        # - "Open" = both fingers move away from y=0

        # Joint motion: positive joint value → motion in +joint_Y direction
        # Joint Y in parent frame:
        #   - If identity rotation: joint +Y = parent +Y
        #   - If 180° around X: joint +Y = parent -Y

        if abs(joint_y[1] - 1.0) < 0.01:
            # Joint Y = Parent +Y
            if y_pos > 0:
                # Left finger: +Y moves AWAY from center = OPEN
                print(f"    → +value = OPEN (finger moves in +Y, away from center)")
                print(f"    → -value = CLOSE (finger moves in -Y, toward center)")
            else:
                # Right finger: +Y moves TOWARD center = CLOSE
                print(f"    → +value = CLOSE (finger moves in +Y, toward center)")
                print(f"    → -value = OPEN (finger moves in -Y, away from center)")
        elif abs(joint_y[1] + 1.0) < 0.01:
            # Joint Y = Parent -Y
            if y_pos > 0:
                # Left finger: -Y moves TOWARD center = CLOSE
                print(f"    → +value = CLOSE (finger moves in -Y, toward center)")
                print(f"    → -value = OPEN (finger moves in +Y, away from center)")
            else:
                # Right finger: -Y moves AWAY from center = OPEN
                print(f"    → +value = OPEN (finger moves in -Y, away from center)")
                print(f"    → -value = CLOSE (finger moves in +Y, toward center)")

print(f"\n{'='*70}")
print("WHAT THE CURRENT CONFIG DOES")
print(f"{'='*70}")

print("""
Current BinaryJointPositionActionCfg:
  OPEN command:
    gripper_left_joint  = +0.019  → LEFT finger: OPEN ✓
    gripper_right_joint = -0.019  → RIGHT finger: ???

  CLOSE command:
    gripper_left_joint  = -0.01   → LEFT finger: CLOSE ✓
    gripper_right_joint = +0.01   → RIGHT finger: ???

From the analysis above:
  right finger localRot0=(0,1,0,0) → 180° around X → joint Y = parent -Y

  RIGHT finger is at y=-0.021:
    +value → joint +Y = parent -Y → further from center = OPEN
    -value → joint -Y = parent +Y → toward center = CLOSE

  So:
    OPEN:  gripper_right_joint = -0.019 → CLOSE (wrong!)
    CLOSE: gripper_right_joint = +0.01  → OPEN (wrong!)

  THE VALUES ARE INVERTED! This is why fingers move opposite to each other.
""")

print(f"\n{'='*70}")
print("CORRECT VALUES")
print(f"{'='*70}")
print("""
Both fingers should use SAME SIGN for same action:

  OPEN command:
    gripper_left_joint  = +0.019  (left finger opens)
    gripper_right_joint = +0.019  (right finger opens - same positive = open)

  CLOSE command:
    gripper_left_joint  = -0.01   (left finger closes)
    gripper_right_joint = -0.01   (right finger closes - same negative = close)

Wait... but the right finger has localRot0=(0,1,0,0) which flips the Y axis.
Let me reconsider.

Actually: for a 180° rotation around X axis:
  Y → -Y
  Z → -Z
  X → X

So for right finger:
  Joint +Y direction = Parent -Y direction
  Right finger at y=-0.021, center at y=0

  Parent -Y is AWAY from center (toward more negative Y) → OPEN
  Parent +Y is TOWARD center → CLOSE

So for RIGHT finger:
  +value → parent -Y → AWAY from center → OPEN
  -value → parent +Y → TOWARD center → CLOSE

And for LEFT finger (identity rotation):
  +value → parent +Y → AWAY from center → OPEN
  -value → parent -Y → TOWARD center → CLOSE

CONCLUSION: BOTH fingers use +positive = OPEN, -negative = CLOSE!
The current config with inverted signs for right finger is WRONG!

CORRECT CONFIG:
  open:  left=+0.019, right=+0.019
  close: left=-0.01,  right=-0.01

Initial positions should also be the same sign:
  left=+0.015, right=+0.015 (both open)
""")

# Also check the URDF for reference
print(f"\n{'='*70}")
print("URDF REFERENCE")
print(f"{'='*70}")
urdf_path = "/workspace/isaaclab/ust_ws/ust_260207/ust_config/open_manipulator_x.urdf"
if os.path.exists(urdf_path):
    with open(urdf_path, 'r') as f:
        content = f.read()
    for match in re.finditer(r'<joint\s+name="(gripper_\w+)"[^>]*>.*?</joint>', content, re.DOTALL):
        name = match.group(1)
        joint_xml = match.group(0)
        print(f"\n  URDF joint: {name}")
        for tag in ['origin', 'axis', 'limit', 'mimic']:
            tag_match = re.search(f'<{tag}[^>]*/>', joint_xml)
            if tag_match:
                print(f"    {tag_match.group(0)}")
