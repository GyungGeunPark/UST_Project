using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using TMPro;

namespace LLMRobotControl
{
    /// <summary>
    /// Main orchestrator for LLM-based robot control
    /// Coordinates OpenAI API, validation, and IK control
    /// </summary>
    public class LLMRobotControlManager : MonoBehaviour
    {
        [Header("Core Components")]
        [SerializeField] private RobotControlConfig config;
        [SerializeField] private OpenAIClient openAIClient;
        [SerializeField] private CommandValidator commandValidator;
        [SerializeField] private IKRobotController ikController;

        [Header("UI Components (Optional - for Unity UI)")]
        [SerializeField] private TMP_InputField commandInputField;
        [SerializeField] private TextMeshProUGUI feedbackText;
        [SerializeField] private TextMeshProUGUI statusText;
        [SerializeField] private Button confirmButton;
        [SerializeField] private Button cancelButton;
        [SerializeField] private GameObject confirmationPanel;

        [Header("Command Cache")]
        [SerializeField] private bool enableCaching = true;
        private System.Collections.Generic.Dictionary<string, RobotCommand> commandCache;

        private bool waitingForConfirmation = false;
        private RobotCommand pendingCommand;
        private Vector3 pendingTargetPosition;

        // Statistics
        private int totalCommands = 0;
        private int successfulCommands = 0;
        private int failedCommands = 0;

        // Events
        public System.Action<string> OnCommandReceived;
        public System.Action<RobotCommand> OnCommandValidated;
        public System.Action<string> OnCommandFailed;

        private void Awake()
        {
            ValidateComponents();
            InitializeCache();
            SetupUI();
            RegisterEventHandlers();
        }

        private void Start()
        {
            UpdateStatus("Ready");
            ShowFeedback("System initialized. Ready for commands.", Color.green);

            if (config != null && !config.Validate(out string error))
            {
                ShowFeedback($"Configuration Error: {error}", Color.red);
                UpdateStatus("Configuration Error");
            }
        }

        /// <summary>
        /// Process a natural language command from user
        /// </summary>
        public void ProcessCommand(string userCommand)
        {
            if (string.IsNullOrWhiteSpace(userCommand))
            {
                ShowFeedback("Please enter a command", Color.red);
                return;
            }

            if (ikController != null && ikController.IsMoving())
            {
                ShowFeedback("Robot is currently moving. Please wait.", Color.yellow);
                return;
            }

            // Clear input field
            if (commandInputField != null)
            {
                commandInputField.text = "";
            }

            StartCoroutine(ProcessCommandCoroutine(userCommand));
        }

        /// <summary>
        /// Command processing coroutine
        /// </summary>
        private IEnumerator ProcessCommandCoroutine(string userCommand)
        {
            totalCommands++;
            UpdateStatus("Processing command...");
            ShowFeedback($"Command: \"{userCommand}\"", Color.white);
            OnCommandReceived?.Invoke(userCommand);

            Debug.Log($"[ControlManager] Processing command: {userCommand}");

            // Check cache first
            if (enableCaching && commandCache.ContainsKey(userCommand.ToLower()))
            {
                Debug.Log("[ControlManager] Using cached command");
                var cachedCommand = commandCache[userCommand.ToLower()];
                yield return ProcessValidatedCommand(cachedCommand);
                yield break;
            }

            // Check API rate limiting
            if (openAIClient != null && !openAIClient.CanMakeCall())
            {
                ShowFeedback("Rate limited. Please wait before sending another command.", Color.yellow);
                UpdateStatus("Rate Limited");
                failedCommands++;
                yield break;
            }

            // Send to OpenAI API
            bool responseReceived = false;
            string responseData = null;
            string errorMessage = null;

            yield return openAIClient.SendChatRequest(
                userCommand,
                (response) =>
                {
                    responseReceived = true;
                    responseData = response;
                },
                (error) =>
                {
                    responseReceived = true;
                    errorMessage = error;
                }
            );

            if (!responseReceived)
            {
                ShowFeedback("Request timeout", Color.red);
                UpdateStatus("Timeout");
                failedCommands++;
                OnCommandFailed?.Invoke("Request timeout");
                yield break;
            }

            if (errorMessage != null)
            {
                ShowFeedback($"API Error: {errorMessage}", Color.red);
                UpdateStatus("API Error");
                failedCommands++;
                OnCommandFailed?.Invoke(errorMessage);
                yield break;
            }

            // Parse response
            RobotCommand command;
            try
            {
                command = OpenAIResponseParser.ParseFunctionCall(responseData);
                Debug.Log($"[ControlManager] Parsed command: {command}");
            }
            catch (System.Exception ex)
            {
                ShowFeedback($"Parse Error: {ex.Message}", Color.red);
                UpdateStatus("Parse Error");
                failedCommands++;
                OnCommandFailed?.Invoke(ex.Message);
                yield break;
            }

            // Cache the command
            if (enableCaching)
            {
                CacheCommand(userCommand, command);
            }

            // Process the validated command
            yield return ProcessValidatedCommand(command);
        }

        /// <summary>
        /// Validate and execute a robot command
        /// </summary>
        private IEnumerator ProcessValidatedCommand(RobotCommand command)
        {
            // Validate command
            Vector3 currentPosition = ikController.GetCurrentPosition();
            ValidationResult validationResult = commandValidator.ValidateCommand(command, currentPosition);

            if (!validationResult.isValid)
            {
                ShowFeedback($"Validation Failed: {validationResult.errorMessage}", Color.red);
                UpdateStatus("Validation Failed");
                failedCommands++;
                OnCommandFailed?.Invoke(validationResult.errorMessage);

                // Show suggested safe position if available
                if (validationResult.safePosition != currentPosition)
                {
                    ShowFeedback(
                        $"Suggested safe position: {validationResult.safePosition:F2}\n" +
                        $"Current: {currentPosition:F2}",
                        Color.yellow
                    );
                }
                yield break;
            }

            OnCommandValidated?.Invoke(command);

            // User confirmation if enabled
            if (config.enableUserConfirmation)
            {
                pendingCommand = command;
                pendingTargetPosition = validationResult.safePosition;

                ShowFeedback(
                    $"Confirm movement?\n" +
                    $"From: {currentPosition:F2}\n" +
                    $"To: {validationResult.safePosition:F2}\n" +
                    $"Speed: {command.speed:F1}, Duration: {command.duration:F1}s",
                    Color.cyan
                );

                UpdateStatus("Waiting for confirmation");

                if (confirmationPanel != null)
                {
                    confirmationPanel.SetActive(true);
                }

                waitingForConfirmation = true;
                float waitTime = 0f;

                while (waitingForConfirmation && waitTime < config.confirmationTimeout)
                {
                    waitTime += Time.deltaTime;
                    yield return null;
                }

                if (confirmationPanel != null)
                {
                    confirmationPanel.SetActive(false);
                }

                if (waitingForConfirmation)
                {
                    ShowFeedback("Confirmation timeout", Color.red);
                    UpdateStatus("Timeout");
                    failedCommands++;
                    waitingForConfirmation = false;
                    yield break;
                }

                // Check if cancelled
                if (pendingCommand == null)
                {
                    yield break; // Was cancelled
                }
            }

            // Execute movement
            UpdateStatus("Executing movement");
            ikController.MoveToPosition(
                validationResult.safePosition,
                command.duration,
                command.speed
            );

            // Wait for movement to complete
            while (ikController.IsMoving())
            {
                float progress = ikController.GetMovementProgress();
                float remaining = ikController.GetDistanceToTarget();
                UpdateStatus($"Moving... {progress * 100f:F0}% ({remaining:F2}m remaining)");
                yield return null;
            }

            // Movement completed
            successfulCommands++;
            ShowFeedback("Movement completed successfully!", Color.green);
            UpdateStatus($"Ready (Success: {successfulCommands}/{totalCommands})");
            Debug.Log($"[ControlManager] Movement completed. Position: {ikController.GetCurrentPosition()}");
        }

        /// <summary>
        /// Confirm pending command
        /// </summary>
        public void ConfirmCommand()
        {
            if (waitingForConfirmation)
            {
                waitingForConfirmation = false;
                Debug.Log("[ControlManager] Command confirmed by user");
            }
        }

        /// <summary>
        /// Cancel pending command
        /// </summary>
        public void CancelCommand()
        {
            if (waitingForConfirmation)
            {
                waitingForConfirmation = false;
                pendingCommand = null;
                ShowFeedback("Command cancelled by user", Color.yellow);
                UpdateStatus("Cancelled");
                failedCommands++;
                Debug.Log("[ControlManager] Command cancelled by user");
            }
        }

        /// <summary>
        /// Emergency stop - immediately halt all movement
        /// </summary>
        public void EmergencyStop()
        {
            if (ikController != null)
            {
                ikController.StopCurrentMovement();
            }

            StopAllCoroutines();
            waitingForConfirmation = false;
            pendingCommand = null;

            ShowFeedback("EMERGENCY STOP ACTIVATED", Color.red);
            UpdateStatus("Emergency Stop");
            Debug.LogWarning("[ControlManager] Emergency stop activated!");
        }

        /// <summary>
        /// Show feedback message to user
        /// </summary>
        private void ShowFeedback(string message, Color color)
        {
            if (feedbackText != null)
            {
                feedbackText.text = message;
                feedbackText.color = color;
            }

            Debug.Log($"[ControlManager] Feedback: {message}");
        }

        /// <summary>
        /// Update status display
        /// </summary>
        private void UpdateStatus(string status)
        {
            if (statusText != null)
            {
                statusText.text = $"Status: {status}";
            }

            Debug.Log($"[ControlManager] Status: {status}");
        }

        /// <summary>
        /// Initialize command cache
        /// </summary>
        private void InitializeCache()
        {
            if (enableCaching)
            {
                commandCache = new System.Collections.Generic.Dictionary<string, RobotCommand>();
            }
        }

        /// <summary>
        /// Cache a command
        /// </summary>
        private void CacheCommand(string userCommand, RobotCommand command)
        {
            if (commandCache == null) return;

            string key = userCommand.ToLower();
            if (commandCache.ContainsKey(key))
            {
                commandCache[key] = command;
            }
            else
            {
                if (commandCache.Count >= config.maxCacheSize)
                {
                    // Remove oldest entry (simplified - could use LRU)
                    var enumerator = commandCache.Keys.GetEnumerator();
                    if (enumerator.MoveNext())
                    {
                        commandCache.Remove(enumerator.Current);
                    }
                }
                commandCache.Add(key, command);
            }
        }

        /// <summary>
        /// Setup UI components
        /// </summary>
        private void SetupUI()
        {
            if (commandInputField != null)
            {
                commandInputField.onSubmit.AddListener(ProcessCommand);
            }

            if (confirmButton != null)
            {
                confirmButton.onClick.AddListener(ConfirmCommand);
            }

            if (cancelButton != null)
            {
                cancelButton.onClick.AddListener(CancelCommand);
            }

            if (confirmationPanel != null)
            {
                confirmationPanel.SetActive(false);
            }
        }

        /// <summary>
        /// Register event handlers
        /// </summary>
        private void RegisterEventHandlers()
        {
            if (ikController != null)
            {
                ikController.OnMovementCompleted += OnMovementComplete;
                ikController.OnMovementFailed += OnMovementFailure;
            }
        }

        /// <summary>
        /// Handle movement completion
        /// </summary>
        private void OnMovementComplete(Vector3 finalPosition)
        {
            Debug.Log($"[ControlManager] Movement completed at {finalPosition}");
        }

        /// <summary>
        /// Handle movement failure
        /// </summary>
        private void OnMovementFailure(string reason)
        {
            ShowFeedback($"Movement failed: {reason}", Color.red);
            failedCommands++;
        }

        /// <summary>
        /// Validate all required components
        /// </summary>
        private void ValidateComponents()
        {
            if (config == null)
            {
                Debug.LogError("[ControlManager] Configuration not assigned!");
            }

            if (openAIClient == null)
            {
                Debug.LogError("[ControlManager] OpenAI Client not assigned!");
            }

            if (commandValidator == null)
            {
                Debug.LogError("[ControlManager] Command Validator not assigned!");
            }

            if (ikController == null)
            {
                Debug.LogError("[ControlManager] IK Robot Controller not assigned!");
            }
        }

        /// <summary>
        /// Get system statistics
        /// </summary>
        public string GetStatistics()
        {
            float successRate = totalCommands > 0 ? (float)successfulCommands / totalCommands * 100f : 0f;
            return $"Total Commands: {totalCommands}\n" +
                   $"Successful: {successfulCommands}\n" +
                   $"Failed: {failedCommands}\n" +
                   $"Success Rate: {successRate:F1}%\n" +
                   $"Cache Size: {(commandCache != null ? commandCache.Count : 0)}";
        }

        private void Update()
        {
            // Emergency stop hotkey
            if (Input.GetKeyDown(KeyCode.Space))
            {
                EmergencyStop();
            }
        }

        private void OnDestroy()
        {
            // Unregister event handlers
            if (ikController != null)
            {
                ikController.OnMovementCompleted -= OnMovementComplete;
                ikController.OnMovementFailed -= OnMovementFailure;
            }
        }
    }
}
