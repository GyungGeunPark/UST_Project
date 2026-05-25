using UnityEngine;
using System.Runtime.InteropServices;

namespace LLMRobotControl
{
    /// <summary>
    /// Bridge between Unity and HTML UI interfaces (button_input.html, text_input.html)
    /// Simulates Webots RobotWindow API for command exchange
    /// </summary>
    public class WebUIBridge : MonoBehaviour
    {
        [Header("Control Manager")]
        [SerializeField] private LLMRobotControlManager controlManager;

        [Header("Web Server Settings")]
        [SerializeField] private int webServerPort = 8080;
        [SerializeField] private bool enableWebServer = true;

        [Header("Message Queue")]
        [SerializeField] private int maxQueueSize = 100;

        private System.Collections.Generic.Queue<string> incomingMessages;
        private System.Collections.Generic.Queue<string> outgoingMessages;
        private SimpleHTTPServer httpServer;

        // JavaScript interop (for WebGL builds)
#if UNITY_WEBGL && !UNITY_EDITOR
        [DllImport("__Internal")]
        private static extern void SendToWeb(string message);

        [DllImport("__Internal")]
        private static extern string ReceiveFromWeb();
#endif

        private void Awake()
        {
            incomingMessages = new System.Collections.Generic.Queue<string>();
            outgoingMessages = new System.Collections.Generic.Queue<string>();

            if (controlManager == null)
            {
                controlManager = FindObjectOfType<LLMRobotControlManager>();
                if (controlManager == null)
                {
                    Debug.LogError("[WebUIBridge] Control Manager not found!");
                }
            }
        }

        private void Start()
        {
#if !UNITY_WEBGL || UNITY_EDITOR
            if (enableWebServer)
            {
                StartWebServer();
            }
#endif
            Debug.Log("[WebUIBridge] Web UI Bridge initialized");
        }

        private void Update()
        {
            // Process incoming messages
            ProcessIncomingMessages();

            // Update web UI with status
            UpdateWebUI();
        }

        /// <summary>
        /// Receive message from web UI (simulates robotWindow.send)
        /// </summary>
        public void ReceiveMessage(string message)
        {
            if (string.IsNullOrEmpty(message)) return;

            if (incomingMessages.Count >= maxQueueSize)
            {
                Debug.LogWarning("[WebUIBridge] Message queue full, dropping oldest message");
                incomingMessages.Dequeue();
            }

            incomingMessages.Enqueue(message);
            Debug.Log($"[WebUIBridge] Received from web: {message}");
        }

        /// <summary>
        /// Send message to web UI (simulates robotWindow.receive)
        /// </summary>
        public void SendMessage(string message)
        {
            if (string.IsNullOrEmpty(message)) return;

            if (outgoingMessages.Count >= maxQueueSize)
            {
                Debug.LogWarning("[WebUIBridge] Outgoing queue full, dropping oldest message");
                outgoingMessages.Dequeue();
            }

            outgoingMessages.Enqueue(message);
            Debug.Log($"[WebUIBridge] Sending to web: {message}");

#if UNITY_WEBGL && !UNITY_EDITOR
            SendToWeb(message);
#endif
        }

        /// <summary>
        /// Process incoming messages from web UI
        /// </summary>
        private void ProcessIncomingMessages()
        {
            while (incomingMessages.Count > 0)
            {
                string message = incomingMessages.Dequeue();
                ProcessWebCommand(message);
            }
        }

        /// <summary>
        /// Process command received from web UI
        /// </summary>
        private void ProcessWebCommand(string command)
        {
            if (controlManager == null)
            {
                Debug.LogError("[WebUIBridge] Control Manager not available");
                SendMessage("Error: Control Manager not available");
                return;
            }

            // Parse command format from button_input.html
            // Format: "forward 1 5" or "backward 1 5" or natural language
            string[] parts = command.Split(' ');

            if (parts.Length >= 2 && IsDirectionalCommand(parts[0]))
            {
                // Convert button command format to natural language
                string direction = parts[0];
                string naturalCommand = ConvertButtonCommandToNaturalLanguage(command);
                Debug.Log($"[WebUIBridge] Converted '{command}' to '{naturalCommand}'");
                controlManager.ProcessCommand(naturalCommand);
            }
            else
            {
                // Direct natural language command
                controlManager.ProcessCommand(command);
            }
        }

        /// <summary>
        /// Convert button command to natural language
        /// Example: "forward 1 5" -> "Move forward for 5 seconds at speed 1"
        /// </summary>
        private string ConvertButtonCommandToNaturalLanguage(string buttonCommand)
        {
            string[] parts = buttonCommand.Split(' ');

            if (parts.Length < 1) return buttonCommand;

            string direction = parts[0].ToLower();

            if (parts.Length >= 3)
            {
                // Format: direction speed duration
                string speed = parts[1];
                string duration = parts[2];

                float speedValue = float.Parse(speed);
                float durationValue = float.Parse(duration);

                // Estimate distance based on speed and duration
                // Assume ~10cm per second at speed 1.0
                float estimatedDistance = speedValue * durationValue * 10f;

                return $"Move {direction} {estimatedDistance:F0} centimeters";
            }
            else if (parts.Length >= 2)
            {
                // Format: direction speed
                string speed = parts[1];
                return $"Move {direction} at speed {speed}";
            }
            else
            {
                // Just direction
                return $"Move {direction}";
            }
        }

        /// <summary>
        /// Check if command is a directional command
        /// </summary>
        private bool IsDirectionalCommand(string command)
        {
            string cmd = command.ToLower();
            return cmd == "forward" || cmd == "backward" || cmd == "left" ||
                   cmd == "right" || cmd == "up" || cmd == "down" || cmd == "stop";
        }

        /// <summary>
        /// Update web UI with current status
        /// </summary>
        private void UpdateWebUI()
        {
            // Send status updates periodically
            if (Time.frameCount % 60 == 0) // Every ~1 second at 60fps
            {
                if (controlManager != null)
                {
                    string status = controlManager.GetStatistics();
                    // Don't flood the queue with status updates
                    if (outgoingMessages.Count < 5)
                    {
                        SendMessage(FormatHTMLMessage(status));
                    }
                }
            }
        }

        /// <summary>
        /// Format message for HTML display
        /// </summary>
        private string FormatHTMLMessage(string message)
        {
            return message
                .Replace("<", "&lt;")
                .Replace(">", "&gt;")
                .Replace("\n", "<br>");
        }

        /// <summary>
        /// Start simple HTTP server for web UI
        /// </summary>
        private void StartWebServer()
        {
            try
            {
                httpServer = new SimpleHTTPServer(webServerPort, this);
                Debug.Log($"[WebUIBridge] Web server started on port {webServerPort}");
                Debug.Log($"[WebUIBridge] Access UI at: http://localhost:{webServerPort}/");
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[WebUIBridge] Failed to start web server: {ex.Message}");
            }
        }

        /// <summary>
        /// Get next outgoing message for web UI
        /// </summary>
        public string GetNextOutgoingMessage()
        {
            return outgoingMessages.Count > 0 ? outgoingMessages.Dequeue() : null;
        }

        private void OnDestroy()
        {
            if (httpServer != null)
            {
                httpServer.Stop();
            }
        }

        /// <summary>
        /// Public API for external systems to send commands
        /// </summary>
        public void SendCommandFromExternal(string command)
        {
            ReceiveMessage(command);
        }
    }

    /// <summary>
    /// Simple HTTP server for serving HTML UI and handling WebSocket-like communication
    /// Note: This is a simplified implementation. For production, consider using a proper WebSocket library.
    /// </summary>
    public class SimpleHTTPServer
    {
        private System.Net.HttpListener listener;
        private WebUIBridge bridge;
        private bool isRunning = false;

        public SimpleHTTPServer(int port, WebUIBridge bridge)
        {
            this.bridge = bridge;

            try
            {
                listener = new System.Net.HttpListener();
                listener.Prefixes.Add($"http://localhost:{port}/");
                listener.Start();
                isRunning = true;

                // Start listening in background
                System.Threading.ThreadPool.QueueUserWorkItem(ListenForRequests);

                Debug.Log($"[SimpleHTTPServer] Server started on port {port}");
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[SimpleHTTPServer] Failed to start: {ex.Message}");
                throw;
            }
        }

        private void ListenForRequests(object state)
        {
            while (isRunning && listener != null && listener.IsListening)
            {
                try
                {
                    var context = listener.GetContext();
                    System.Threading.ThreadPool.QueueUserWorkItem(HandleRequest, context);
                }
                catch (System.Exception ex)
                {
                    if (isRunning)
                    {
                        Debug.LogError($"[SimpleHTTPServer] Error: {ex.Message}");
                    }
                }
            }
        }

        private void HandleRequest(object state)
        {
            var context = (System.Net.HttpListenerContext)state;
            var request = context.Request;
            var response = context.Response;

            try
            {
                if (request.HttpMethod == "POST" && request.Url.AbsolutePath == "/command")
                {
                    // Handle command from web UI
                    using (var reader = new System.IO.StreamReader(request.InputStream))
                    {
                        string command = reader.ReadToEnd();
                        bridge.ReceiveMessage(command);

                        string responseText = "{\"status\":\"ok\"}";
                        byte[] buffer = System.Text.Encoding.UTF8.GetBytes(responseText);
                        response.ContentType = "application/json";
                        response.ContentLength64 = buffer.Length;
                        response.OutputStream.Write(buffer, 0, buffer.Length);
                    }
                }
                else if (request.HttpMethod == "GET" && request.Url.AbsolutePath == "/status")
                {
                    // Send status to web UI
                    string message = bridge.GetNextOutgoingMessage() ?? "{\"status\":\"idle\"}";
                    byte[] buffer = System.Text.Encoding.UTF8.GetBytes(message);
                    response.ContentType = "application/json";
                    response.ContentLength64 = buffer.Length;
                    response.OutputStream.Write(buffer, 0, buffer.Length);
                }
                else
                {
                    // Serve static files (HTML)
                    string filePath = request.Url.AbsolutePath.TrimStart('/');
                    ServeStaticFile(response, filePath);
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[SimpleHTTPServer] Request error: {ex.Message}");
            }
            finally
            {
                response.Close();
            }
        }

        private void ServeStaticFile(System.Net.HttpListenerResponse response, string filePath)
        {
            // Look for HTML files in Report folder
            string htmlPath = System.IO.Path.Combine(
                Application.dataPath,
                "Report",
                string.IsNullOrEmpty(filePath) ? "text_input.html" : filePath
            );

            if (System.IO.File.Exists(htmlPath))
            {
                byte[] fileBytes = System.IO.File.ReadAllBytes(htmlPath);
                response.ContentType = "text/html";
                response.ContentLength64 = fileBytes.Length;
                response.OutputStream.Write(fileBytes, 0, fileBytes.Length);
            }
            else
            {
                response.StatusCode = 404;
                byte[] notFound = System.Text.Encoding.UTF8.GetBytes("File not found");
                response.ContentLength64 = notFound.Length;
                response.OutputStream.Write(notFound, 0, notFound.Length);
            }
        }

        public void Stop()
        {
            isRunning = false;
            if (listener != null)
            {
                listener.Stop();
                listener.Close();
            }
        }
    }
}
