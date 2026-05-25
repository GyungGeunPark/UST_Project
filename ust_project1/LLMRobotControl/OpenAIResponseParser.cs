using System;
using System.Text.RegularExpressions;
using UnityEngine;

namespace LLMRobotControl
{
    /// <summary>
    /// Parses OpenAI API responses and extracts robot commands
    /// </summary>
    public static class OpenAIResponseParser
    {
        /// <summary>
        /// Parse OpenAI response JSON and extract robot command
        /// </summary>
        public static RobotCommand ParseFunctionCall(string jsonResponse)
        {
            try
            {
                // Parse the response using Unity's JsonUtility
                ChatCompletionResponse response = JsonUtility.FromJson<ChatCompletionResponse>(jsonResponse);

                if (response == null || response.choices == null || response.choices.Length == 0)
                {
                    throw new Exception("Invalid response format: No choices found");
                }

                var message = response.choices[0].message;
                if (message == null)
                {
                    throw new Exception("Invalid response format: No message found");
                }

                // Check for function call
                var functionCall = message.function_call;
                if (functionCall == null)
                {
                    throw new Exception("No function call in response. The model may have responded with text instead.");
                }

                if (functionCall.name != "move_robot_ik")
                {
                    throw new Exception($"Unexpected function: {functionCall.name}");
                }

                // Parse function arguments
                Debug.Log($"[Parser] Function arguments: {functionCall.arguments}");
                return ParseFunctionArguments(functionCall.arguments);
            }
            catch (Exception ex)
            {
                Debug.LogError($"[OpenAIResponseParser] Failed to parse response: {ex.Message}\nJSON: {jsonResponse}");
                throw;
            }
        }

        /// <summary>
        /// Parse function call arguments JSON
        /// </summary>
        private static RobotCommand ParseFunctionArguments(string argumentsJson)
        {
            try
            {
                // Manual JSON parsing due to Unity JsonUtility limitations
                var command = new RobotCommand();

                // Parse movement_type
                command.movementType = ExtractStringValue(argumentsJson, "movement_type");

                // Parse direction (optional for absolute movements)
                command.direction = ExtractStringValue(argumentsJson, "direction");

                // Parse distance (optional for absolute movements)
                command.distance = ExtractFloatValue(argumentsJson, "distance", 0f);

                // Parse speed
                command.speed = ExtractFloatValue(argumentsJson, "speed", 1.0f);

                // Parse duration
                command.duration = ExtractFloatValue(argumentsJson, "duration", 2.0f);

                // Parse absolute position if present
                if (argumentsJson.Contains("\"position\""))
                {
                    Vector3 pos = ExtractPositionValue(argumentsJson, "position");
                    command.absolutePosition = pos;
                }

                // Validate required fields
                if (string.IsNullOrEmpty(command.movementType))
                {
                    throw new Exception("movement_type is required");
                }

                if (command.movementType == "relative" && string.IsNullOrEmpty(command.direction))
                {
                    throw new Exception("direction is required for relative movements");
                }

                if (command.movementType == "absolute" && !command.absolutePosition.HasValue)
                {
                    throw new Exception("position is required for absolute movements");
                }

                Debug.Log($"[Parser] Parsed command: {command}");
                return command;
            }
            catch (Exception ex)
            {
                Debug.LogError($"[OpenAIResponseParser] Failed to parse arguments: {ex.Message}\nJSON: {argumentsJson}");
                throw;
            }
        }

        /// <summary>
        /// Extract string value from JSON
        /// </summary>
        private static string ExtractStringValue(string json, string key)
        {
            string pattern = $"\\\"{key}\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"";
            var match = Regex.Match(json, pattern);
            return match.Success ? match.Groups[1].Value : string.Empty;
        }

        /// <summary>
        /// Extract float value from JSON
        /// </summary>
        private static float ExtractFloatValue(string json, string key, float defaultValue)
        {
            string pattern = $"\\\"{key}\\\"\\s*:\\s*([0-9.]+)";
            var match = Regex.Match(json, pattern);
            if (match.Success && float.TryParse(match.Groups[1].Value, out float result))
            {
                return result;
            }
            return defaultValue;
        }

        /// <summary>
        /// Extract Vector3 position from JSON
        /// </summary>
        private static Vector3 ExtractPositionValue(string json, string key)
        {
            // Find position object
            string pattern = $"\\\"{key}\\\"\\s*:\\s*{{{{([^}}}}]+)}}}}";
            var match = Regex.Match(json, pattern);

            if (!match.Success)
            {
                return Vector3.zero;
            }

            string positionJson = match.Groups[1].Value;

            float x = ExtractFloatValue(positionJson, "x", 0f);
            float y = ExtractFloatValue(positionJson, "y", 0f);
            float z = ExtractFloatValue(positionJson, "z", 0f);

            return new Vector3(x, y, z);
        }

        /// <summary>
        /// Try to extract error message from OpenAI error response
        /// </summary>
        public static string ExtractErrorMessage(string jsonResponse)
        {
            try
            {
                // Look for error message in response
                string pattern = "\\\"message\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"";
                var match = Regex.Match(jsonResponse, pattern);
                return match.Success ? match.Groups[1].Value : "Unknown error";
            }
            catch
            {
                return "Failed to parse error message";
            }
        }
    }
}
