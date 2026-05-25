using UnityEngine;

namespace LLMRobotControl
{
    /// <summary>
    /// Simple tester script for LLM Robot Control System
    /// Add this to any GameObject to test commands without UI
    /// </summary>
    public class LLMRobotControlTester : MonoBehaviour
    {
        [Header("Control Manager")]
        [SerializeField] private LLMRobotControlManager controlManager;

        [Header("Test Commands")]
        [Tooltip("명령을 입력하고 Test Command 버튼을 누르세요")]
        [TextArea(3, 10)]
        [SerializeField] private string testCommand = "앞으로 30cm 이동";

        [Header("Debug Settings")]
        [SerializeField] private bool enableVerboseLogging = true;

        [Header("Test Buttons")]
        [SerializeField] private bool executeTestCommand = false;
        [SerializeField] private bool testAPIConnection = false;
        [SerializeField] private bool checkSetup = false;

        private void Awake()
        {
            if (controlManager == null)
            {
                controlManager = FindObjectOfType<LLMRobotControlManager>();
                if (controlManager == null)
                {
                    Debug.LogError("[Tester] LLMRobotControlManager not found in scene!");
                }
            }
        }

        private void Update()
        {
            // Keyboard shortcuts
            if (Input.GetKeyDown(KeyCode.T))
            {
                ExecuteTestCommand();
            }

            if (Input.GetKeyDown(KeyCode.C))
            {
                CheckSetup();
            }
        }

        private void OnValidate()
        {
            if (executeTestCommand)
            {
                executeTestCommand = false;
                if (Application.isPlaying)
                {
                    ExecuteTestCommand();
                }
            }

            if (testAPIConnection)
            {
                testAPIConnection = false;
                if (Application.isPlaying)
                {
                    TestAPIConnection();
                }
            }

            if (checkSetup)
            {
                checkSetup = false;
                if (Application.isPlaying)
                {
                    CheckSetup();
                }
            }
        }

        /// <summary>
        /// Execute the test command
        /// </summary>
        public void ExecuteTestCommand()
        {
            if (controlManager == null)
            {
                Debug.LogError("[Tester] Control Manager not assigned!");
                return;
            }

            if (string.IsNullOrEmpty(testCommand))
            {
                Debug.LogWarning("[Tester] Test command is empty!");
                return;
            }

            Debug.Log($"[Tester] ===== 테스트 명령 실행 =====");
            Debug.Log($"[Tester] 명령: {testCommand}");
            Debug.Log($"[Tester] 시간: {System.DateTime.Now:HH:mm:ss}");

            controlManager.ProcessCommand(testCommand);

            Debug.Log($"[Tester] ===== 명령 전송 완료 =====");
        }

        /// <summary>
        /// Test API connection
        /// </summary>
        public void TestAPIConnection()
        {
            Debug.Log("[Tester] ===== API 연결 테스트 =====");

            var openAIClient = FindObjectOfType<OpenAIClient>();
            if (openAIClient == null)
            {
                Debug.LogError("[Tester] OpenAIClient not found!");
                return;
            }

            Debug.Log("[Tester] OpenAIClient found");
            Debug.Log("[Tester] API 호출 가능 여부를 확인하려면 실제 명령을 실행하세요");
        }

        /// <summary>
        /// Check complete setup
        /// </summary>
        public void CheckSetup()
        {
            Debug.Log("[Tester] ===== 설정 확인 =====");

            bool allGood = true;

            // Check Control Manager
            if (controlManager == null)
            {
                Debug.LogError("[Tester] ✗ LLMRobotControlManager NOT FOUND");
                allGood = false;
            }
            else
            {
                Debug.Log("[Tester] ✓ LLMRobotControlManager found");
            }

            // Check OpenAIClient
            var openAIClient = FindObjectOfType<OpenAIClient>();
            if (openAIClient == null)
            {
                Debug.LogError("[Tester] ✗ OpenAIClient NOT FOUND");
                allGood = false;
            }
            else
            {
                Debug.Log("[Tester] ✓ OpenAIClient found");
            }

            // Check CommandValidator
            var validator = FindObjectOfType<CommandValidator>();
            if (validator == null)
            {
                Debug.LogError("[Tester] ✗ CommandValidator NOT FOUND");
                allGood = false;
            }
            else
            {
                Debug.Log("[Tester] ✓ CommandValidator found");
            }

            // Check IKRobotController
            var ikController = FindObjectOfType<IKRobotController>();
            if (ikController == null)
            {
                Debug.LogError("[Tester] ✗ IKRobotController NOT FOUND");
                allGood = false;
            }
            else
            {
                Debug.Log("[Tester] ✓ IKRobotController found");

                // Check IK Target
                var ikTarget = GameObject.Find("IK_Target");
                if (ikTarget == null)
                {
                    Debug.LogWarning("[Tester] ⚠ IK_Target GameObject not found by name");
                }
                else
                {
                    Debug.Log($"[Tester] ✓ IK_Target found at position: {ikTarget.transform.position}");
                }
            }

            // Check Config
            var config = Resources.FindObjectsOfTypeAll<RobotControlConfig>();
            if (config == null || config.Length == 0)
            {
                Debug.LogError("[Tester] ✗ RobotControlConfig NOT FOUND");
                allGood = false;
            }
            else
            {
                Debug.Log($"[Tester] ✓ RobotControlConfig found ({config.Length} instances)");
            }

            // Check Bio IK
            var bioIK = FindObjectOfType<BioIK.BioIK>();
            if (bioIK == null)
            {
                Debug.LogWarning("[Tester] ⚠ Bio IK not found (optional)");
            }
            else
            {
                Debug.Log($"[Tester] ✓ Bio IK found on: {bioIK.gameObject.name}");
            }

            if (allGood)
            {
                Debug.Log("[Tester] ===== ✓ 모든 컴포넌트 확인됨 =====");
            }
            else
            {
                Debug.LogError("[Tester] ===== ✗ 일부 컴포넌트 누락 =====");
            }
        }

        [ContextMenu("Execute Test Command")]
        public void ExecuteTestCommandMenu()
        {
            ExecuteTestCommand();
        }

        [ContextMenu("Check Setup")]
        public void CheckSetupMenu()
        {
            CheckSetup();
        }

        private void OnGUI()
        {
            if (!enableVerboseLogging) return;

            GUILayout.BeginArea(new Rect(10, 10, 400, 200));
            GUILayout.Label("=== LLM Robot Control Tester ===");
            GUILayout.Label($"Control Manager: {(controlManager != null ? "OK" : "MISSING")}");
            GUILayout.Label("Keyboard Shortcuts:");
            GUILayout.Label("  T - Execute Test Command");
            GUILayout.Label("  C - Check Setup");
            GUILayout.Label("  Space - Emergency Stop");
            GUILayout.EndArea();
        }
    }
}
