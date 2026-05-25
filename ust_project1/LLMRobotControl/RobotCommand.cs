using UnityEngine;
using System;

namespace LLMRobotControl
{
    /// <summary>
    /// Data structure representing a robot movement command parsed from LLM
    /// </summary>
    [Serializable]
    public class RobotCommand
    {
        public string movementType;      // "relative" or "absolute"
        public string direction;         // "forward", "backward", "left", "right", "up", "down"
        public float distance;           // Distance in centimeters
        public Vector3? absolutePosition; // For absolute positioning
        public float speed = 1.0f;       // Speed multiplier (0.1 - 2.0)
        public float duration = 2.0f;    // Duration in seconds (0.1 - 10.0)

        public override string ToString()
        {
            if (movementType == "absolute" && absolutePosition.HasValue)
            {
                return $"Move to position {absolutePosition.Value} at speed {speed}";
            }
            else
            {
                return $"Move {direction} {distance}cm at speed {speed} for {duration}s";
            }
        }
    }

    /// <summary>
    /// OpenAI API request message structure
    /// </summary>
    [Serializable]
    public class ChatMessage
    {
        public string role;     // "system", "user", or "assistant"
        public string content;  // Message content
    }

    /// <summary>
    /// OpenAI function definition structure
    /// </summary>
    [Serializable]
    public class FunctionDefinition
    {
        public string name;
        public string description;
        public FunctionParameters parameters;
    }

    [Serializable]
    public class FunctionParameters
    {
        public string type = "object";
        public object properties;
        public string[] required;
    }

    /// <summary>
    /// Complete OpenAI API request structure
    /// </summary>
    [Serializable]
    public class ChatCompletionRequest
    {
        public string model;
        public ChatMessage[] messages;
        public FunctionDefinition[] functions;
        public string function_call;
        public float temperature = 0.1f;
        public int max_tokens = 200;
    }

    /// <summary>
    /// OpenAI API response structure
    /// </summary>
    [Serializable]
    public class ChatCompletionResponse
    {
        public string id;
        public string @object;
        public long created;
        public string model;
        public Choice[] choices;
        public Usage usage;
    }

    [Serializable]
    public class Choice
    {
        public int index;
        public ResponseMessage message;
        public string finish_reason;
    }

    [Serializable]
    public class ResponseMessage
    {
        public string role;
        public string content;
        public FunctionCall function_call;
    }

    [Serializable]
    public class FunctionCall
    {
        public string name;
        public string arguments;
    }

    [Serializable]
    public class Usage
    {
        public int prompt_tokens;
        public int completion_tokens;
        public int total_tokens;
    }

    /// <summary>
    /// Validation result structure
    /// </summary>
    public class ValidationResult
    {
        public bool isValid;
        public string errorMessage;
        public Vector3 safePosition;
        public RobotCommand validatedCommand;

        public ValidationResult()
        {
            isValid = false;
            errorMessage = string.Empty;
            safePosition = Vector3.zero;
        }
    }
}
