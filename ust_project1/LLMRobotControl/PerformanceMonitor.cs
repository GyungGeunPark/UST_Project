using UnityEngine;
using System.Collections.Generic;

namespace LLMRobotControl
{
    /// <summary>
    /// Monitors and tracks performance metrics for the LLM robot control system
    /// </summary>
    public class PerformanceMonitor : MonoBehaviour
    {
        [Header("Display Settings")]
        [SerializeField] private bool showOnScreenDisplay = true;
        [SerializeField] private bool logToConsole = false;
        [SerializeField] private float updateInterval = 1.0f;

        [Header("UI Display")]
        [SerializeField] private TMPro.TextMeshProUGUI displayText;

        private class PerformanceMetrics
        {
            public float avgAPIResponseTime = 0f;
            public float avgMovementTime = 0f;
            public int totalCommands = 0;
            public int successfulCommands = 0;
            public int failedCommands = 0;
            public List<float> apiResponseTimes = new List<float>();
            public List<float> movementTimes = new List<float>();
            public float minAPIResponseTime = float.MaxValue;
            public float maxAPIResponseTime = 0f;
            public float minMovementTime = float.MaxValue;
            public float maxMovementTime = 0f;
        }

        private PerformanceMetrics metrics = new PerformanceMetrics();
        private float lastUpdateTime = 0f;
        private System.Diagnostics.Stopwatch apiStopwatch;
        private System.Diagnostics.Stopwatch movementStopwatch;

        private void Awake()
        {
            apiStopwatch = new System.Diagnostics.Stopwatch();
            movementStopwatch = new System.Diagnostics.Stopwatch();
        }

        private void Update()
        {
            if (showOnScreenDisplay && Time.time - lastUpdateTime >= updateInterval)
            {
                UpdateDisplay();
                lastUpdateTime = Time.time;
            }
        }

        /// <summary>
        /// Record start of API call
        /// </summary>
        public void StartAPICall()
        {
            apiStopwatch.Restart();
        }

        /// <summary>
        /// Record end of API call
        /// </summary>
        public void EndAPICall(bool success)
        {
            apiStopwatch.Stop();
            float responseTime = (float)apiStopwatch.Elapsed.TotalSeconds;

            metrics.apiResponseTimes.Add(responseTime);
            metrics.totalCommands++;

            if (success)
            {
                metrics.successfulCommands++;
            }
            else
            {
                metrics.failedCommands++;
            }

            // Update statistics
            metrics.avgAPIResponseTime = CalculateAverage(metrics.apiResponseTimes);
            metrics.minAPIResponseTime = Mathf.Min(metrics.minAPIResponseTime, responseTime);
            metrics.maxAPIResponseTime = Mathf.Max(metrics.maxAPIResponseTime, responseTime);

            if (logToConsole)
            {
                Debug.Log($"[Performance] API call: {responseTime:F2}s, Success: {success}");
            }
        }

        /// <summary>
        /// Record start of movement
        /// </summary>
        public void StartMovement()
        {
            movementStopwatch.Restart();
        }

        /// <summary>
        /// Record end of movement
        /// </summary>
        public void EndMovement()
        {
            movementStopwatch.Stop();
            float movementTime = (float)movementStopwatch.Elapsed.TotalSeconds;

            metrics.movementTimes.Add(movementTime);

            // Update statistics
            metrics.avgMovementTime = CalculateAverage(metrics.movementTimes);
            metrics.minMovementTime = Mathf.Min(metrics.minMovementTime, movementTime);
            metrics.maxMovementTime = Mathf.Max(metrics.maxMovementTime, movementTime);

            if (logToConsole)
            {
                Debug.Log($"[Performance] Movement: {movementTime:F2}s");
            }
        }

        /// <summary>
        /// Calculate average from list
        /// </summary>
        private float CalculateAverage(List<float> values)
        {
            if (values.Count == 0) return 0f;
            float sum = 0f;
            foreach (var value in values)
            {
                sum += value;
            }
            return sum / values.Count;
        }

        /// <summary>
        /// Update on-screen display
        /// </summary>
        private void UpdateDisplay()
        {
            if (!showOnScreenDisplay) return;

            string report = GenerateReport();

            if (displayText != null)
            {
                displayText.text = report;
            }
        }

        /// <summary>
        /// Generate performance report
        /// </summary>
        public string GenerateReport()
        {
            float successRate = metrics.totalCommands > 0
                ? (float)metrics.successfulCommands / metrics.totalCommands * 100f
                : 0f;

            return $"<b>Performance Monitor</b>\n" +
                   $"━━━━━━━━━━━━━━━━━━━━\n" +
                   $"<b>Commands:</b>\n" +
                   $"  Total: {metrics.totalCommands}\n" +
                   $"  Success: {metrics.successfulCommands}\n" +
                   $"  Failed: {metrics.failedCommands}\n" +
                   $"  Success Rate: {successRate:F1}%\n" +
                   $"\n<b>API Response Time:</b>\n" +
                   $"  Avg: {metrics.avgAPIResponseTime:F2}s\n" +
                   $"  Min: {(metrics.minAPIResponseTime == float.MaxValue ? 0 : metrics.minAPIResponseTime):F2}s\n" +
                   $"  Max: {metrics.maxAPIResponseTime:F2}s\n" +
                   $"\n<b>Movement Time:</b>\n" +
                   $"  Avg: {metrics.avgMovementTime:F2}s\n" +
                   $"  Min: {(metrics.minMovementTime == float.MaxValue ? 0 : metrics.minMovementTime):F2}s\n" +
                   $"  Max: {metrics.maxMovementTime:F2}s\n" +
                   $"\n<b>System:</b>\n" +
                   $"  FPS: {(1f / Time.deltaTime):F0}\n" +
                   $"  Memory: {(System.GC.GetTotalMemory(false) / 1024f / 1024f):F1} MB";
        }

        /// <summary>
        /// Reset all metrics
        /// </summary>
        public void ResetMetrics()
        {
            metrics = new PerformanceMetrics();
            Debug.Log("[Performance] Metrics reset");
        }

        /// <summary>
        /// Get success rate
        /// </summary>
        public float GetSuccessRate()
        {
            return metrics.totalCommands > 0
                ? (float)metrics.successfulCommands / metrics.totalCommands
                : 0f;
        }

        /// <summary>
        /// Get average API response time
        /// </summary>
        public float GetAverageAPIResponseTime()
        {
            return metrics.avgAPIResponseTime;
        }

        /// <summary>
        /// Get average movement time
        /// </summary>
        public float GetAverageMovementTime()
        {
            return metrics.avgMovementTime;
        }
    }
}
