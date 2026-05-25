using UnityEngine;
using System.Collections;
using BioIK;

#if UNITY_ANIMATION_RIGGING
using UnityEngine.Animations.Rigging;
#endif

namespace LLMRobotControl
{
    /// <summary>
    /// Controls robot IK target based on validated commands
    /// Supports Bio IK, Unity Animation Rigging package or manual IK control
    /// </summary>
    public class IKRobotController : MonoBehaviour
    {
        [Header("Configuration")]
        [SerializeField] private RobotControlConfig config;

        [Header("IK Target Setup")]
        [Tooltip("The IK target transform to control")]
        [SerializeField] private Transform ikTarget;

        [Header("Bio IK Setup (Optional)")]
        [Tooltip("Optional: Bio IK component for advanced IK solving")]
        [SerializeField] private BioIK.BioIK bioIK;

        [Tooltip("Optional: Bio IK Position objective for end-effector control")]
        [SerializeField] private Position bioIKPositionObjective;

#if UNITY_ANIMATION_RIGGING
        [Tooltip("Optional: Two Bone IK Constraint (requires Animation Rigging package)")]
        [SerializeField] private TwoBoneIKConstraint ikConstraint;

        [Tooltip("Optional: Rig component (requires Animation Rigging package)")]
        [SerializeField] private Rig rig;
#endif

        [Header("Movement Visualization")]
        [SerializeField] private bool showDebugVisuals = true;
        [SerializeField] private LineRenderer pathLineRenderer;
        [SerializeField] private GameObject targetVisualization;

        private Vector3 currentTargetPosition;
        private bool isMoving = false;
        private Coroutine currentMovement;

        // Events
        public System.Action<Vector3> OnMovementStarted;
        public System.Action<Vector3> OnMovementCompleted;
        public System.Action<string> OnMovementFailed;

        private void Awake()
        {
            if (config == null)
            {
                Debug.LogError("[IKRobotController] Configuration not assigned!");
            }

            if (ikTarget == null)
            {
                Debug.LogError("[IKRobotController] IK Target not assigned!");
                return;
            }

            currentTargetPosition = ikTarget.position;

            // Check for Bio IK setup
            if (bioIK != null)
            {
                Debug.Log("[IKRobotController] Bio IK detected and enabled");

                // If position objective not assigned, try to find it
                if (bioIKPositionObjective == null)
                {
                    bioIKPositionObjective = FindBioIKPositionObjective();
                }
            }

#if UNITY_ANIMATION_RIGGING
            // Ensure rig is properly set up
            if (rig != null)
            {
                rig.weight = 1.0f;
                Debug.Log("[IKRobotController] Animation Rigging enabled");
            }
#endif

            // Setup visualization
            SetupVisualization();
        }

        private void Start()
        {
            UpdateTargetVisualization(ikTarget.position);
        }

        /// <summary>
        /// Move IK target to specified position
        /// </summary>
        public void MoveToPosition(Vector3 targetPosition, float duration, float speed)
        {
            if (ikTarget == null)
            {
                Debug.LogError("[IKRobotController] IK Target not assigned!");
                OnMovementFailed?.Invoke("IK Target not assigned");
                return;
            }

            if (isMoving)
            {
                Debug.LogWarning("[IKRobotController] Already moving. Stopping previous movement.");
                StopCurrentMovement();
            }

            currentMovement = StartCoroutine(SmoothMoveToPosition(targetPosition, duration, speed));
        }

        /// <summary>
        /// Stop current movement immediately
        /// </summary>
        public void StopCurrentMovement()
        {
            if (currentMovement != null)
            {
                StopCoroutine(currentMovement);
                currentMovement = null;
            }
            isMoving = false;
        }

        /// <summary>
        /// Smooth movement coroutine with animation curve
        /// </summary>
        private IEnumerator SmoothMoveToPosition(Vector3 targetPosition, float duration, float speed)
        {
            isMoving = true;
            Vector3 startPosition = ikTarget.position;
            float actualDuration = duration / speed;
            float elapsedTime = 0f;

            OnMovementStarted?.Invoke(targetPosition);
            Debug.Log($"[IKRobotController] Moving from {startPosition} to {targetPosition} over {actualDuration:F2}s");

            // Update visualization
            UpdateTargetVisualization(targetPosition);
            DrawPath(startPosition, targetPosition);

            while (elapsedTime < actualDuration)
            {
                elapsedTime += Time.deltaTime;
                float normalizedTime = Mathf.Clamp01(elapsedTime / actualDuration);

                // Use animation curve for smooth motion
                float curveValue = config.movementCurve.Evaluate(normalizedTime);

                ikTarget.position = Vector3.Lerp(startPosition, targetPosition, curveValue);

                yield return null;
            }

            // Ensure we reach exact target position
            ikTarget.position = targetPosition;
            currentTargetPosition = targetPosition;
            isMoving = false;
            currentMovement = null;

            OnMovementCompleted?.Invoke(targetPosition);
            Debug.Log($"[IKRobotController] Movement completed. Final position: {ikTarget.position}");

            // Clear path visualization
            if (pathLineRenderer != null)
            {
                yield return new WaitForSeconds(0.5f);
                pathLineRenderer.enabled = false;
            }
        }

        /// <summary>
        /// Check if robot is currently moving
        /// </summary>
        public bool IsMoving()
        {
            return isMoving;
        }

        /// <summary>
        /// Get current IK target position
        /// </summary>
        public Vector3 GetCurrentPosition()
        {
            return ikTarget != null ? ikTarget.position : Vector3.zero;
        }

        /// <summary>
        /// Get distance to target position
        /// </summary>
        public float GetDistanceToTarget()
        {
            if (ikTarget == null) return 0f;
            return Vector3.Distance(ikTarget.position, currentTargetPosition);
        }

        /// <summary>
        /// Setup visualization components
        /// </summary>
        private void SetupVisualization()
        {
            if (!showDebugVisuals) return;

            // Create path line renderer if not assigned
            if (pathLineRenderer == null)
            {
                GameObject lineObj = new GameObject("PathLineRenderer");
                lineObj.transform.SetParent(transform);
                pathLineRenderer = lineObj.AddComponent<LineRenderer>();
                pathLineRenderer.startWidth = 0.02f;
                pathLineRenderer.endWidth = 0.02f;
                pathLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
                pathLineRenderer.startColor = Color.yellow;
                pathLineRenderer.endColor = Color.red;
                pathLineRenderer.enabled = false;
            }

            // Create target visualization if not assigned
            if (targetVisualization == null && ikTarget != null)
            {
                targetVisualization = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                targetVisualization.name = "TargetVisualization";
                targetVisualization.transform.localScale = Vector3.one * 0.05f;
                targetVisualization.GetComponent<Renderer>().material.color = new Color(0, 1, 0, 0.5f);
                // Remove collider to avoid physics interactions
                Destroy(targetVisualization.GetComponent<Collider>());
            }
        }

        /// <summary>
        /// Update target visualization position
        /// </summary>
        private void UpdateTargetVisualization(Vector3 position)
        {
            if (targetVisualization != null && showDebugVisuals)
            {
                targetVisualization.transform.position = position;
                targetVisualization.SetActive(true);
            }
        }

        /// <summary>
        /// Draw path between current and target position
        /// </summary>
        private void DrawPath(Vector3 start, Vector3 end)
        {
            if (pathLineRenderer != null && showDebugVisuals)
            {
                pathLineRenderer.positionCount = 2;
                pathLineRenderer.SetPosition(0, start);
                pathLineRenderer.SetPosition(1, end);
                pathLineRenderer.enabled = true;
            }
        }

        /// <summary>
        /// Draw gizmos in Scene view
        /// </summary>
        private void OnDrawGizmos()
        {
            if (!showDebugVisuals || ikTarget == null) return;

            // Draw current IK target position
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(ikTarget.position, 0.05f);

            // Draw movement indicator
            if (isMoving)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(ikTarget.position, currentTargetPosition);

                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(currentTargetPosition, 0.08f);
            }

            // Draw position label
            #if UNITY_EDITOR
            UnityEditor.Handles.Label(
                ikTarget.position + Vector3.up * 0.1f,
                $"IK Target\n{ikTarget.position:F2}"
            );
            #endif
        }

        private void OnDestroy()
        {
            // Cleanup
            if (targetVisualization != null)
            {
                Destroy(targetVisualization);
            }
        }

        /// <summary>
        /// Get movement progress (0 to 1)
        /// </summary>
        public float GetMovementProgress()
        {
            if (!isMoving || ikTarget == null) return 1f;

            float totalDistance = Vector3.Distance(ikTarget.position, currentTargetPosition);
            if (totalDistance < config.positionThreshold) return 1f;

            return 1f - (GetDistanceToTarget() / totalDistance);
        }

        /// <summary>
        /// Find Bio IK Position objective in the Bio IK component
        /// </summary>
        private Position FindBioIKPositionObjective()
        {
            if (bioIK == null) return null;

            Position foundPosition = null;
            int positionCount = 0;

            // Search through all segments
            foreach (var segment in bioIK.Segments)
            {
                if (segment == null) continue;

                // Check objectives in this segment
                foreach (var objective in segment.Objectives)
                {
                    if (objective is Position posObjective)
                    {
                        positionCount++;
                        if (foundPosition == null)
                        {
                            foundPosition = posObjective;
                        }
                    }
                }
            }

            if (foundPosition != null)
            {
                Debug.Log($"[IKRobotController] Found {positionCount} Position objective(s) in Bio IK. Using the first one.");
                Debug.LogWarning("[IKRobotController] Please manually verify that the Position objective's target is set to IK_Target.");
                return foundPosition;
            }

            Debug.LogWarning("[IKRobotController] No Bio IK Position objective found in any segment");
            return null;
        }

        /// <summary>
        /// Set Bio IK component reference (for setup scripts)
        /// </summary>
        public void SetBioIK(BioIK.BioIK bioIKComponent)
        {
            bioIK = bioIKComponent;
            if (bioIK != null)
            {
                bioIKPositionObjective = FindBioIKPositionObjective();
                Debug.Log("[IKRobotController] Bio IK component assigned");
            }
        }

        /// <summary>
        /// Set Bio IK Position objective reference (for setup scripts)
        /// </summary>
        public void SetBioIKPositionObjective(Position objective)
        {
            bioIKPositionObjective = objective;
            Debug.Log("[IKRobotController] Bio IK Position objective assigned");
        }
    }
}
