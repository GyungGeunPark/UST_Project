using System;
using System.Collections;
using System.Text;
using UnityEngine;
using UnityEngine.Networking;

namespace LLMRobotControl
{
    /// <summary>
    /// Handles communication with OpenAI API for robot control commands
    /// </summary>
    public class OpenAIClient : MonoBehaviour
    {
        [Header("Configuration")]
        [SerializeField] private RobotControlConfig config;

        private const string API_URL = "https://api.openai.com/v1/chat/completions";
        private float lastCallTime = 0f;

        private void Awake()
        {
            if (config == null)
            {
                Debug.LogError("[OpenAIClient] Configuration not assigned!");
            }
        }

        /// <summary>
        /// Check if we can make an API call (rate limiting)
        /// </summary>
        public bool CanMakeCall()
        {
            if (config == null) return false;
            return Time.time - lastCallTime >= config.minAPICallInterval;
        }

        /// <summary>
        /// Send a chat request to OpenAI API
        /// </summary>
        public IEnumerator SendChatRequest(
            string userMessage,
            Action<string> onSuccess,
            Action<string> onError)
        {
            if (config == null)
            {
                onError?.Invoke("Configuration not set");
                yield break;
            }

            // Validate configuration
            if (!config.Validate(out string validationError))
            {
                onError?.Invoke($"Configuration error: {validationError}");
                yield break;
            }

            // Check rate limiting
            if (!CanMakeCall())
            {
                float waitTime = config.minAPICallInterval - (Time.time - lastCallTime);
                onError?.Invoke($"Rate limited. Wait {waitTime:F1}s");
                yield break;
            }

            lastCallTime = Time.time;

            // Build request
            var request = BuildChatRequest(userMessage);
            string jsonRequest = JsonUtility.ToJson(request);

            // Manual JSON building for complex structures (Unity JsonUtility limitations)
            jsonRequest = BuildManualJsonRequest(userMessage);

            byte[] bodyRaw = Encoding.UTF8.GetBytes(jsonRequest);

            using (UnityWebRequest webRequest = new UnityWebRequest(API_URL, "POST"))
            {
                webRequest.uploadHandler = new UploadHandlerRaw(bodyRaw);
                webRequest.downloadHandler = new DownloadHandlerBuffer();
                webRequest.SetRequestHeader("Content-Type", "application/json");
                webRequest.SetRequestHeader("Authorization", $"Bearer {config.apiKey}");
                webRequest.timeout = (int)config.apiTimeout;

                Debug.Log($"[OpenAIClient] Sending request: {userMessage}");

                yield return webRequest.SendWebRequest();

                if (webRequest.result == UnityWebRequest.Result.Success)
                {
                    Debug.Log($"[OpenAIClient] Response received: {webRequest.downloadHandler.text.Substring(0, Mathf.Min(200, webRequest.downloadHandler.text.Length))}...");
                    onSuccess?.Invoke(webRequest.downloadHandler.text);
                }
                else
                {
                    string errorMsg = $"API Error: {webRequest.error}\n{webRequest.downloadHandler.text}";
                    Debug.LogError($"[OpenAIClient] {errorMsg}");
                    onError?.Invoke(errorMsg);
                }
            }
        }

        /// <summary>
        /// Build the chat request structure
        /// </summary>
        private ChatCompletionRequest BuildChatRequest(string userMessage)
        {
            return new ChatCompletionRequest
            {
                model = config.model,
                messages = new ChatMessage[]
                {
                    new ChatMessage
                    {
                        role = "system",
                        content = config.GetSystemPrompt()
                    },
                    new ChatMessage
                    {
                        role = "user",
                        content = userMessage
                    }
                },
                temperature = 0.1f,
                max_tokens = 200
            };
        }

        /// <summary>
        /// Build JSON request manually due to Unity JsonUtility limitations with nested objects
        /// </summary>
        private string BuildManualJsonRequest(string userMessage)
        {
            string systemPrompt = config.GetSystemPrompt().Replace("\"", "\\\"").Replace("\n", "\\n");
            string userPrompt = userMessage.Replace("\"", "\\\"").Replace("\n", "\\n");

            return $@"{{
  ""model"": ""{config.model}"",
  ""messages"": [
    {{
      ""role"": ""system"",
      ""content"": ""{systemPrompt}""
    }},
    {{
      ""role"": ""user"",
      ""content"": ""{userPrompt}""
    }}
  ],
  ""functions"": [
    {{
      ""name"": ""move_robot_ik"",
      ""description"": ""Move the robot's IK target to a new position based on directional commands or absolute coordinates"",
      ""parameters"": {{
        ""type"": ""object"",
        ""properties"": {{
          ""movement_type"": {{
            ""type"": ""string"",
            ""enum"": [""relative"", ""absolute""],
            ""description"": ""Whether the movement is relative to current position or absolute coordinates""
          }},
          ""direction"": {{
            ""type"": ""string"",
            ""enum"": [""forward"", ""backward"", ""left"", ""right"", ""up"", ""down""],
            ""description"": ""Direction of movement (for relative movements)""
          }},
          ""distance"": {{
            ""type"": ""number"",
            ""minimum"": 0.1,
            ""maximum"": 100.0,
            ""description"": ""Distance to move in centimeters (for relative movements)""
          }},
          ""position"": {{
            ""type"": ""object"",
            ""properties"": {{
              ""x"": {{ ""type"": ""number"" }},
              ""y"": {{ ""type"": ""number"" }},
              ""z"": {{ ""type"": ""number"" }}
            }},
            ""description"": ""Absolute target position in Unity coordinates (for absolute movements)""
          }},
          ""speed"": {{
            ""type"": ""number"",
            ""minimum"": 0.1,
            ""maximum"": 2.0,
            ""default"": 1.0,
            ""description"": ""Movement speed multiplier""
          }},
          ""duration"": {{
            ""type"": ""number"",
            ""minimum"": 0.1,
            ""maximum"": 10.0,
            ""default"": 2.0,
            ""description"": ""Duration of movement in seconds""
          }}
        }},
        ""required"": [""movement_type""]
      }}
    }}
  ],
  ""function_call"": ""auto"",
  ""temperature"": 0.1,
  ""max_tokens"": 200
}}";
        }

        /// <summary>
        /// Get usage statistics
        /// </summary>
        public string GetStats()
        {
            return $"Last API call: {Time.time - lastCallTime:F1}s ago";
        }
    }
}
