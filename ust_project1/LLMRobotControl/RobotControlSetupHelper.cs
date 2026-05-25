using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
using System.Linq;
#endif
using BioIK;

namespace LLMRobotControl
{
    /// <summary>
    /// Editor helper script for quick setup of LLM Robot Control System
    /// Usage: Tools → Robot Control → Setup Scene
    /// </summary>
    public class RobotControlSetupHelper
    {
#if UNITY_EDITOR
        [MenuItem("Tools/Robot Control/Setup Scene")]
        public static void SetupScene()
        {
            // Check if system already exists
            if (GameObject.Find("RobotControlSystem") != null)
            {
                if (!EditorUtility.DisplayDialog(
                    "Setup Already Exists",
                    "A RobotControlSystem already exists in the scene. Do you want to create another one?",
                    "Yes", "No"))
                {
                    return;
                }
            }

            // Create main system GameObject
            GameObject systemObj = new GameObject("RobotControlSystem");
            Undo.RegisterCreatedObjectUndo(systemObj, "Create Robot Control System");

            // Add components
            var client = systemObj.AddComponent<OpenAIClient>();
            var validator = systemObj.AddComponent<CommandValidator>();
            var ikController = systemObj.AddComponent<IKRobotController>();
            var manager = systemObj.AddComponent<LLMRobotControlManager>();
            var webBridge = systemObj.AddComponent<WebUIBridge>();
            var perfMonitor = systemObj.AddComponent<PerformanceMonitor>();
            var emergencyStop = systemObj.AddComponent<EmergencyStopSystem>();

            // Try to find config
            RobotControlConfig config = AssetDatabase.FindAssets("t:RobotControlConfig")
                .Select(guid => AssetDatabase.GUIDToAssetPath(guid))
                .Select(path => AssetDatabase.LoadAssetAtPath<RobotControlConfig>(path))
                .FirstOrDefault();

            if (config != null)
            {
                // Assign config using SerializedObject
                var clientSO = new SerializedObject(client);
                clientSO.FindProperty("config").objectReferenceValue = config;
                clientSO.ApplyModifiedProperties();

                var validatorSO = new SerializedObject(validator);
                validatorSO.FindProperty("config").objectReferenceValue = config;
                validatorSO.ApplyModifiedProperties();

                var ikSO = new SerializedObject(ikController);
                ikSO.FindProperty("config").objectReferenceValue = config;
                ikSO.ApplyModifiedProperties();

                var managerSO = new SerializedObject(manager);
                managerSO.FindProperty("config").objectReferenceValue = config;
                managerSO.FindProperty("openAIClient").objectReferenceValue = client;
                managerSO.FindProperty("commandValidator").objectReferenceValue = validator;
                managerSO.FindProperty("ikController").objectReferenceValue = ikController;
                managerSO.ApplyModifiedProperties();

                var webBridgeSO = new SerializedObject(webBridge);
                webBridgeSO.FindProperty("controlManager").objectReferenceValue = manager;
                webBridgeSO.ApplyModifiedProperties();

                var emergencySO = new SerializedObject(emergencyStop);
                emergencySO.FindProperty("controlManager").objectReferenceValue = manager;
                emergencySO.FindProperty("robotControllers").arraySize = 1;
                emergencySO.FindProperty("robotControllers").GetArrayElementAtIndex(0).objectReferenceValue = ikController;
                emergencySO.ApplyModifiedProperties();

                Debug.Log("[Setup] Configuration assigned successfully");
            }
            else
            {
                Debug.LogWarning("[Setup] No RobotControlConfig found. Create one: Assets → Create → Robot Control → Config");
            }

            // Create IK Target
            GameObject ikTarget = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            ikTarget.name = "IK_Target";
            ikTarget.transform.localScale = Vector3.one * 0.1f;
            ikTarget.GetComponent<Renderer>().material.color = new Color(0, 1, 0, 0.5f);
            Object.DestroyImmediate(ikTarget.GetComponent<Collider>()); // Remove collider
            Undo.RegisterCreatedObjectUndo(ikTarget, "Create IK Target");

            // Assign IK target
            var ikControllerSO = new SerializedObject(ikController);
            ikControllerSO.FindProperty("ikTarget").objectReferenceValue = ikTarget.transform;
            ikControllerSO.ApplyModifiedProperties();

            // Try to find and assign Bio IK component
            BioIK.BioIK bioIK = Object.FindObjectOfType<BioIK.BioIK>();
            if (bioIK != null)
            {
                Debug.Log($"[Setup] Found Bio IK component on: {bioIK.gameObject.name}");
                ikControllerSO.Update();
                ikControllerSO.FindProperty("bioIK").objectReferenceValue = bioIK;
                ikControllerSO.ApplyModifiedProperties();

                // Try to find a Position objective that could use this target
                Position positionObjective = FindBioIKPositionObjective(bioIK, ikTarget.transform);
                if (positionObjective == null)
                {
                    Debug.LogWarning("[Setup] No Bio IK Position objective found. You may need to configure it manually.");
                    Debug.LogWarning("[Setup] Or create a new Position objective and assign the IK_Target as its target.");
                }
                else
                {
                    ikControllerSO.Update();
                    ikControllerSO.FindProperty("bioIKPositionObjective").objectReferenceValue = positionObjective;
                    ikControllerSO.ApplyModifiedProperties();
                    Debug.Log("[Setup] Bio IK Position objective assigned successfully");
                }
            }
            else
            {
                Debug.LogWarning("[Setup] No Bio IK component found in scene. Bio IK integration will not be configured.");
                Debug.LogWarning("[Setup] You can manually assign Bio IK component to IKRobotController later.");
            }

            // Select the system
            Selection.activeGameObject = systemObj;

            string bioIKStatus = bioIK != null ? "✓ Bio IK detected and configured" : "⚠ No Bio IK found - manual setup needed";

            EditorUtility.DisplayDialog(
                "Setup Complete",
                "Robot Control System has been set up!\n\n" +
                "Next steps:\n" +
                "1. Create RobotControlConfig if not exists\n" +
                "2. Add your OpenAI API key to the config\n" +
                "3. Configure workspace bounds\n" +
                "4. Assign Bio IK Position objective to IK_Target (if needed)\n" +
                "5. Test with a simple command\n\n" +
                bioIKStatus + "\n\n" +
                "See README.md for detailed instructions.",
                "OK"
            );
        }

        [MenuItem("Tools/Robot Control/Create Config Asset")]
        public static void CreateConfigAsset()
        {
            RobotControlConfig config = ScriptableObject.CreateInstance<RobotControlConfig>();

            string path = EditorUtility.SaveFilePanelInProject(
                "Create Robot Control Config",
                "RobotControlConfig",
                "asset",
                "Create a new Robot Control Configuration asset"
            );

            if (!string.IsNullOrEmpty(path))
            {
                AssetDatabase.CreateAsset(config, path);
                AssetDatabase.SaveAssets();
                AssetDatabase.Refresh();

                Selection.activeObject = config;

                EditorUtility.DisplayDialog(
                    "Config Created",
                    "Robot Control Config asset created!\n\n" +
                    "Don't forget to:\n" +
                    "1. Add your OpenAI API key\n" +
                    "2. Configure workspace bounds\n" +
                    "3. Set safety constraints\n\n" +
                    "⚠️ NEVER commit API keys to version control!",
                    "OK"
                );

                Debug.Log($"[Setup] Config created at: {path}");
            }
        }

        [MenuItem("Tools/Robot Control/Validate Setup")]
        public static void ValidateSetup()
        {
            var system = GameObject.Find("RobotControlSystem");
            if (system == null)
            {
                EditorUtility.DisplayDialog(
                    "Validation Failed",
                    "RobotControlSystem not found in scene.\n\n" +
                    "Run: Tools → Robot Control → Setup Scene",
                    "OK"
                );
                return;
            }

            string report = "=== Robot Control Setup Validation ===\n\n";
            bool allValid = true;

            // Check components
            var manager = system.GetComponent<LLMRobotControlManager>();
            var client = system.GetComponent<OpenAIClient>();
            var validator = system.GetComponent<CommandValidator>();
            var ikController = system.GetComponent<IKRobotController>();

            report += CheckComponent("LLMRobotControlManager", manager, ref allValid);
            report += CheckComponent("OpenAIClient", client, ref allValid);
            report += CheckComponent("CommandValidator", validator, ref allValid);
            report += CheckComponent("IKRobotController", ikController, ref allValid);

            // Check config
            if (manager != null)
            {
                var managerSO = new SerializedObject(manager);
                var configProp = managerSO.FindProperty("config");
                if (configProp.objectReferenceValue != null)
                {
                    report += "✓ Configuration assigned\n";

                    var config = configProp.objectReferenceValue as RobotControlConfig;
                    if (config != null)
                    {
                        if (config.Validate(out string error))
                        {
                            report += "✓ Configuration valid\n";
                        }
                        else
                        {
                            report += $"✗ Configuration error: {error}\n";
                            allValid = false;
                        }
                    }
                }
                else
                {
                    report += "✗ Configuration not assigned\n";
                    allValid = false;
                }
            }

            // Check IK Target
            if (ikController != null)
            {
                var ikSO = new SerializedObject(ikController);
                var targetProp = ikSO.FindProperty("ikTarget");
                if (targetProp.objectReferenceValue != null)
                {
                    report += "✓ IK Target assigned\n";
                }
                else
                {
                    report += "✗ IK Target not assigned\n";
                    allValid = false;
                }

                // Check Bio IK setup
                var bioIKProp = ikSO.FindProperty("bioIK");
                if (bioIKProp.objectReferenceValue != null)
                {
                    report += "✓ Bio IK component assigned\n";

                    var bioIKObjectiveProp = ikSO.FindProperty("bioIKPositionObjective");
                    if (bioIKObjectiveProp.objectReferenceValue != null)
                    {
                        report += "✓ Bio IK Position objective assigned\n";
                    }
                    else
                    {
                        report += "⚠ Bio IK Position objective not assigned (recommended)\n";
                    }
                }
                else
                {
                    report += "⚠ Bio IK not assigned (optional - using direct IK target control)\n";
                }
            }

            report += "\n";
            if (allValid)
            {
                report += "✓ Setup is complete and valid!\n\n";
                report += "You can now test the system.";
            }
            else
            {
                report += "⚠ Setup has issues that need to be fixed.\n\n";
                report += "See console for details.";
            }

            Debug.Log(report);

            EditorUtility.DisplayDialog(
                allValid ? "Validation Passed" : "Validation Issues Found",
                report,
                "OK"
            );
        }

        private static string CheckComponent<T>(string name, T component, ref bool allValid) where T : Component
        {
            if (component != null)
            {
                return $"✓ {name} present\n";
            }
            else
            {
                allValid = false;
                return $"✗ {name} missing\n";
            }
        }

        [MenuItem("Tools/Robot Control/Open Documentation")]
        public static void OpenDocumentation()
        {
            string readmePath = System.IO.Path.Combine(
                Application.dataPath,
                "Scripts/LLMRobotControl/README.md"
            );

            if (System.IO.File.Exists(readmePath))
            {
                Application.OpenURL("file://" + readmePath);
            }
            else
            {
                EditorUtility.DisplayDialog(
                    "Documentation Not Found",
                    $"README.md not found at:\n{readmePath}",
                    "OK"
                );
            }
        }

        [MenuItem("Tools/Robot Control/About")]
        public static void ShowAbout()
        {
            EditorUtility.DisplayDialog(
                "LLM Robot Control System",
                "Version: 1.0\n" +
                "Unity: 2021.3+\n\n" +
                "A comprehensive system for controlling robots using LLMs.\n\n" +
                "Features:\n" +
                "• Natural language control\n" +
                "• OpenAI API integration\n" +
                "• IK target control\n" +
                "• Multi-layer safety\n" +
                "• Web UI interface\n" +
                "• Performance monitoring\n\n" +
                "Based on: LLM_Robot_Control_System_Design.md",
                "OK"
            );
        }

        /// <summary>
        /// Find Bio IK Position objective that could work with the given target
        /// </summary>
        private static Position FindBioIKPositionObjective(BioIK.BioIK bioIK, Transform targetTransform)
        {
            if (bioIK == null) return null;

            Position foundPosition = null;
            int positionCount = 0;

            // Search through all segments for Position objectives
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
                Debug.Log($"[Setup] Found {positionCount} Position objective(s) in Bio IK. Using the first one.");
                Debug.LogWarning($"[Setup] Please manually verify that the Position objective's target transform is set to IK_Target.");
                return foundPosition;
            }

            return null;
        }
#endif
    }
}
