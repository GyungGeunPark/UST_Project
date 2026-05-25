using UnityEngine;

namespace LLMRobotControl
{
    /// <summary>
    /// Validates robot commands for safety and workspace constraints
    /// </summary>
    public class CommandValidator : MonoBehaviour
    {
        [Header("Configuration")]
        [SerializeField] private RobotControlConfig config;

        [Header("Debug Visualization")]
        [SerializeField] private bool showGizmos = true;
        [SerializeField] private Color workspaceColor = new Color(0, 1, 0, 0.3f);
        [SerializeField] private Color collisionColor = new Color(1, 0, 0, 0.5f);

        private void Awake()
        {
            if (config == null)
            {
                Debug.LogError("[CommandValidator] Configuration not assigned!");
            }
        }

        /// <summary>
        /// Validate a robot command for safety
        /// </summary>
        public ValidationResult ValidateCommand(RobotCommand command, Vector3 currentPosition)
        {
            var result = new ValidationResult
            {
                isValid = true,
                safePosition = currentPosition,
                validatedCommand = command
            };

            if (config == null)
            {
                result.isValid = false;
                result.errorMessage = "Configuration not set";
                return result;
            }

            // 1. Validate movement type
            if (string.IsNullOrEmpty(command.movementType))
            {
                result.isValid = false;
                result.errorMessage = "Movement type is required";
                return result;
            }

            // 2. Calculate target position
            Vector3 targetPosition;
            if (command.movementType == "relative")
            {
                if (string.IsNullOrEmpty(command.direction))
                {
                    result.isValid = false;
                    result.errorMessage = "Direction is required for relative movements";
                    return result;
                }

                targetPosition = CalculateRelativePosition(currentPosition, command.direction, command.distance);
            }
            else if (command.movementType == "absolute" && command.absolutePosition.HasValue)
            {
                targetPosition = command.absolutePosition.Value;
            }
            else
            {
                result.isValid = false;
                result.errorMessage = "Invalid movement parameters";
                return result;
            }

            // 3. Check workspace bounds
            if (!IsWithinWorkspace(targetPosition))
            {
                result.isValid = false;
                result.errorMessage = $"Target position {targetPosition} is outside workspace bounds " +
                                     $"[{config.workspaceMin} to {config.workspaceMax}]";
                result.safePosition = ClampToWorkspace(targetPosition);
                Debug.LogWarning($"[CommandValidator] {result.errorMessage}. Clamped to {result.safePosition}");
                return result;
            }

            // 4. Check maximum movement distance
            float movementDistance = Vector3.Distance(currentPosition, targetPosition);
            if (movementDistance > config.maxSingleMovement)
            {
                result.isValid = false;
                result.errorMessage = $"Movement distance {movementDistance:F2}m exceeds maximum {config.maxSingleMovement:F2}m";

                // Calculate safe position along the movement vector
                Vector3 direction = (targetPosition - currentPosition).normalized;
                result.safePosition = currentPosition + direction * config.maxSingleMovement;
                Debug.LogWarning($"[CommandValidator] {result.errorMessage}. Limited to {result.safePosition}");
                return result;
            }

            // 5. Check collision with obstacles
            if (WillCollide(currentPosition, targetPosition))
            {
                result.isValid = false;
                result.errorMessage = "Path to target position would result in collision";
                Debug.LogWarning($"[CommandValidator] Collision detected between {currentPosition} and {targetPosition}");
                return result;
            }

            // 6. Validate speed and duration
            if (command.speed < 0.1f || command.speed > 2.0f)
            {
                result.isValid = false;
                result.errorMessage = $"Speed {command.speed} is outside valid range [0.1, 2.0]";
                return result;
            }

            if (command.duration < 0.1f || command.duration > 10.0f)
            {
                result.isValid = false;
                result.errorMessage = $"Duration {command.duration} is outside valid range [0.1, 10.0]";
                return result;
            }

            // All checks passed
            result.safePosition = targetPosition;
            Debug.Log($"[CommandValidator] Command validated successfully: {currentPosition} → {targetPosition}");
            return result;
        }

        /// <summary>
        /// Calculate relative position from current position and direction
        /// </summary>
        private Vector3 CalculateRelativePosition(Vector3 currentPos, string direction, float distanceCm)
        {
            float distanceUnits = distanceCm * config.centimetersToUnityUnits;

            switch (direction?.ToLower())
            {
                case "forward":
                    return currentPos + Vector3.forward * distanceUnits;
                case "backward":
                    return currentPos + Vector3.back * distanceUnits;
                case "left":
                    return currentPos + Vector3.left * distanceUnits;
                case "right":
                    return currentPos + Vector3.right * distanceUnits;
                case "up":
                    return currentPos + Vector3.up * distanceUnits;
                case "down":
                    return currentPos + Vector3.down * distanceUnits;
                default:
                    Debug.LogWarning($"[CommandValidator] Unknown direction: {direction}");
                    return currentPos;
            }
        }

        /// <summary>
        /// Check if position is within workspace bounds
        /// </summary>
        private bool IsWithinWorkspace(Vector3 position)
        {
            return position.x >= config.workspaceMin.x && position.x <= config.workspaceMax.x &&
                   position.y >= config.workspaceMin.y && position.y <= config.workspaceMax.y &&
                   position.z >= config.workspaceMin.z && position.z <= config.workspaceMax.z;
        }

        /// <summary>
        /// Clamp position to workspace bounds
        /// </summary>
        private Vector3 ClampToWorkspace(Vector3 position)
        {
            return new Vector3(
                Mathf.Clamp(position.x, config.workspaceMin.x, config.workspaceMax.x),
                Mathf.Clamp(position.y, config.workspaceMin.y, config.workspaceMax.y),
                Mathf.Clamp(position.z, config.workspaceMin.z, config.workspaceMax.z)
            );
        }

        /// <summary>
        /// Check for collisions along path
        /// </summary>
        private bool WillCollide(Vector3 from, Vector3 to)
        {
            Vector3 direction = to - from;
            float distance = direction.magnitude;

            // Raycast to check for obstacles
            if (Physics.Raycast(from, direction.normalized, distance, config.obstacleLayer))
            {
                return true;
            }

            // Spherecast for more robust collision detection
            if (Physics.SphereCast(
                from,
                config.minDistanceToObstacles,
                direction.normalized,
                out RaycastHit hit,
                distance,
                config.obstacleLayer))
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Draw workspace bounds in Scene view
        /// </summary>
        private void OnDrawGizmos()
        {
            if (!showGizmos || config == null) return;

            // Draw workspace bounds
            Gizmos.color = workspaceColor;
            Vector3 center = (config.workspaceMin + config.workspaceMax) * 0.5f;
            Vector3 size = config.workspaceMax - config.workspaceMin;
            Gizmos.DrawCube(center, size);

            // Draw wireframe
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(center, size);
        }
    }
}
