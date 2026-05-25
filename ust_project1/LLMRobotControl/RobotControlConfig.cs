using UnityEngine;

namespace LLMRobotControl
{
    /// <summary>
    /// Configuration ScriptableObject for robot control system
    /// Create via: Assets > Create > Robot Control > Config
    /// </summary>
    [CreateAssetMenu(fileName = "RobotControlConfig", menuName = "Robot Control/Config", order = 1)]
    public class RobotControlConfig : ScriptableObject
    {
        [Header("OpenAI API Settings")]
        [Tooltip("Your OpenAI API key - NEVER commit this to version control!")]
        public string apiKey = "";

        [Tooltip("OpenAI model to use (e.g., gpt-4-turbo, gpt-4, gpt-3.5-turbo)")]
        public string model = "gpt-4-turbo";

        [Tooltip("API request timeout in seconds")]
        [Range(10f, 120f)]
        public float apiTimeout = 30f;

        [Header("Workspace Bounds")]
        [Tooltip("Minimum workspace position (meters)")]
        public Vector3 workspaceMin = new Vector3(-1f, 0f, -1f);

        [Tooltip("Maximum workspace position (meters)")]
        public Vector3 workspaceMax = new Vector3(1f, 2f, 1f);

        [Header("Safety Constraints")]
        [Tooltip("Maximum single movement distance (meters)")]
        [Range(0.1f, 2.0f)]
        public float maxSingleMovement = 1.0f;

        [Tooltip("Minimum distance to maintain from obstacles (meters)")]
        [Range(0.05f, 0.5f)]
        public float minDistanceToObstacles = 0.2f;

        [Tooltip("Layer mask for collision detection")]
        public LayerMask obstacleLayer;

        [Header("Movement Settings")]
        [Tooltip("Movement interpolation curve for smooth motion")]
        public AnimationCurve movementCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);

        [Tooltip("Position threshold for movement completion (meters)")]
        [Range(0.001f, 0.1f)]
        public float positionThreshold = 0.01f;

        [Header("User Interaction")]
        [Tooltip("Require user confirmation before executing movements")]
        public bool enableUserConfirmation = true;

        [Tooltip("Confirmation timeout in seconds")]
        [Range(5f, 60f)]
        public float confirmationTimeout = 10f;

        [Header("Performance")]
        [Tooltip("Minimum interval between API calls (seconds)")]
        [Range(0.5f, 5f)]
        public float minAPICallInterval = 1.0f;

        [Tooltip("Maximum cache size for command caching")]
        [Range(10, 100)]
        public int maxCacheSize = 50;

        [Header("Coordinate System")]
        [Tooltip("Unit conversion: 1 cm = X Unity units")]
        public float centimetersToUnityUnits = 0.01f;

        /// <summary>
        /// Validate configuration settings
        /// </summary>
        public bool Validate(out string errorMessage)
        {
            if (string.IsNullOrEmpty(apiKey))
            {
                errorMessage = "API Key is not set!";
                return false;
            }

            if (string.IsNullOrEmpty(model))
            {
                errorMessage = "Model is not set!";
                return false;
            }

            if (workspaceMin.x >= workspaceMax.x ||
                workspaceMin.y >= workspaceMax.y ||
                workspaceMin.z >= workspaceMax.z)
            {
                errorMessage = "Invalid workspace bounds!";
                return false;
            }

            errorMessage = string.Empty;
            return true;
        }

        /// <summary>
        /// Get the system prompt for OpenAI
        /// </summary>
        public string GetSystemPrompt()
        {
            return $@"You are a robot control AI assistant. Your role is to interpret natural language commands and convert them into precise robot control instructions.

**Robot Coordinate System:**
- Forward/Backward: Z-axis (positive = forward, negative = backward)
- Left/Right: X-axis (positive = right, negative = left)
- Up/Down: Y-axis (positive = up, negative = down)

**Workspace Bounds:**
- X: [{workspaceMin.x:F2}, {workspaceMax.x:F2}] meters
- Y: [{workspaceMin.y:F2}, {workspaceMax.y:F2}] meters
- Z: [{workspaceMin.z:F2}, {workspaceMax.z:F2}] meters

**Available Functions:**
- move_robot_ik: Move the robot's IK target to a new position

**Safety Guidelines:**
- All movements must be within the robot's workspace
- Maximum single movement distance: {maxSingleMovement * 100f}cm
- Movement speed range: 0.1 to 2.0
- Always validate coordinates before execution

**Examples:**
- ""Move forward 30cm"" → move_robot_ik with direction=""forward"", distance=30
- ""Go to position x=0.5, z=0.3"" → move_robot_ik with absolute position
- ""Move left 10cm slowly"" → move_robot_ik with direction=""left"", distance=10, speed=0.5

When the user gives a command, use the move_robot_ik function with appropriate parameters.";
        }
    }
}
