using UnityEngine;

namespace LLMRobotControl
{
    /// <summary>
    /// Emergency stop system for immediate robot halt
    /// </summary>
    public class EmergencyStopSystem : MonoBehaviour
    {
        [Header("Hotkeys")]
        [SerializeField] private KeyCode emergencyStopKey = KeyCode.Space;
        [SerializeField] private KeyCode resetKey = KeyCode.R;

        [Header("Controllers")]
        [SerializeField] private IKRobotController[] robotControllers;
        [SerializeField] private LLMRobotControlManager controlManager;

        [Header("UI")]
        [SerializeField] private GameObject emergencyStopPanel;
        [SerializeField] private TMPro.TextMeshProUGUI statusText;

        [Header("Logging")]
        [SerializeField] private bool logIncidents = true;
        [SerializeField] private string logFilePath = "Logs/emergency_stops.txt";

        private bool isStopped = false;
        private System.DateTime lastStopTime;
        private int stopCount = 0;

        // Events
        public System.Action OnEmergencyStopActivated;
        public System.Action OnEmergencyStopReset;

        private void Update()
        {
            // Check for emergency stop hotkey
            if (Input.GetKeyDown(emergencyStopKey))
            {
                ExecuteEmergencyStop();
            }

            // Check for reset hotkey
            if (Input.GetKeyDown(resetKey) && isStopped)
            {
                ResetEmergencyStop();
            }
        }

        /// <summary>
        /// Execute emergency stop
        /// </summary>
        public void ExecuteEmergencyStop()
        {
            if (isStopped)
            {
                Debug.LogWarning("[EmergencyStop] Already in emergency stop state");
                return;
            }

            Debug.LogWarning("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            Debug.LogWarning("   EMERGENCY STOP ACTIVATED!");
            Debug.LogWarning("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

            isStopped = true;
            lastStopTime = System.DateTime.Now;
            stopCount++;

            // Stop all robot controllers
            if (robotControllers != null)
            {
                foreach (var controller in robotControllers)
                {
                    if (controller != null)
                    {
                        controller.StopCurrentMovement();
                    }
                }
            }

            // Stop control manager
            if (controlManager != null)
            {
                controlManager.EmergencyStop();
            }

            // Show UI
            if (emergencyStopPanel != null)
            {
                emergencyStopPanel.SetActive(true);
            }

            if (statusText != null)
            {
                statusText.text = $"<color=red><b>EMERGENCY STOP ACTIVE</b></color>\n\n" +
                                 $"Time: {lastStopTime:HH:mm:ss}\n" +
                                 $"Stop Count: {stopCount}\n\n" +
                                 $"Press [{resetKey}] to reset";
            }

            // Log incident
            if (logIncidents)
            {
                LogIncident();
            }

            // Invoke event
            OnEmergencyStopActivated?.Invoke();

            // Additional safety measures
            Time.timeScale = 0f; // Pause simulation (optional)
        }

        /// <summary>
        /// Reset emergency stop
        /// </summary>
        public void ResetEmergencyStop()
        {
            if (!isStopped)
            {
                Debug.LogWarning("[EmergencyStop] Not in emergency stop state");
                return;
            }

            Debug.Log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            Debug.Log("   Emergency Stop Reset");
            Debug.Log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

            isStopped = false;

            // Hide UI
            if (emergencyStopPanel != null)
            {
                emergencyStopPanel.SetActive(false);
            }

            // Resume simulation
            Time.timeScale = 1f;

            // Invoke event
            OnEmergencyStopReset?.Invoke();
        }

        /// <summary>
        /// Log emergency stop incident
        /// </summary>
        private void LogIncident()
        {
            try
            {
                string logDirectory = System.IO.Path.Combine(Application.persistentDataPath, "Logs");
                if (!System.IO.Directory.Exists(logDirectory))
                {
                    System.IO.Directory.CreateDirectory(logDirectory);
                }

                string fullPath = System.IO.Path.Combine(Application.persistentDataPath, logFilePath);
                string logEntry = $"[{lastStopTime:yyyy-MM-dd HH:mm:ss}] Emergency Stop #{stopCount}\n" +
                                 $"  Scene: {UnityEngine.SceneManagement.SceneManager.GetActiveScene().name}\n" +
                                 $"  Active Controllers: {(robotControllers != null ? robotControllers.Length : 0)}\n" +
                                 $"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

                System.IO.File.AppendAllText(fullPath, logEntry);
                Debug.Log($"[EmergencyStop] Incident logged to: {fullPath}");
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[EmergencyStop] Failed to log incident: {ex.Message}");
            }
        }

        /// <summary>
        /// Check if system is in emergency stop state
        /// </summary>
        public bool IsStopped()
        {
            return isStopped;
        }

        /// <summary>
        /// Get total stop count
        /// </summary>
        public int GetStopCount()
        {
            return stopCount;
        }

        /// <summary>
        /// Get time of last stop
        /// </summary>
        public System.DateTime GetLastStopTime()
        {
            return lastStopTime;
        }

        private void OnGUI()
        {
            if (isStopped && emergencyStopPanel == null)
            {
                // Fallback UI if no panel assigned
                GUI.color = Color.red;
                GUI.Box(new Rect(Screen.width / 2 - 200, 20, 400, 100), "");

                GUI.color = Color.white;
                GUIStyle style = new GUIStyle(GUI.skin.label);
                style.alignment = TextAnchor.MiddleCenter;
                style.fontSize = 20;
                style.fontStyle = FontStyle.Bold;

                GUI.Label(new Rect(Screen.width / 2 - 200, 30, 400, 30), "EMERGENCY STOP ACTIVE", style);

                style.fontSize = 14;
                style.fontStyle = FontStyle.Normal;
                GUI.Label(new Rect(Screen.width / 2 - 200, 70, 400, 30),
                    $"Press [{resetKey}] to reset", style);
            }
        }
    }
}
