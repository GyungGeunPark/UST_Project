// Isaac Sim Robot Control - Main JavaScript

class RobotControlUI {
    constructor() {
        // WebSocket
        this.ws = null;
        this.wsRetryCount = 0;
        this.maxRetries = 5;

        // State
        this.isConnected = false;
        this.isEmergencyStopped = false;
        this.currentDistance = 10;

        // DOM elements
        this.elements = {
            connectionStatus: document.getElementById('connection-status'),
            statusDot: document.querySelector('.status-dot'),
            statusText: document.querySelector('.status-text'),
            commandInput: document.getElementById('command-input'),
            sendBtn: document.getElementById('send-btn'),
            emergencyBtn: document.getElementById('emergency-btn'),
            resetBtn: document.getElementById('reset-btn'),
            distanceSlider: document.getElementById('distance-slider'),
            distanceValue: document.getElementById('distance-value'),
            logContainer: document.getElementById('log-container'),
            // Status display
            robotState: document.getElementById('robot-state'),
            posX: document.getElementById('pos-x'),
            posY: document.getElementById('pos-y'),
            posZ: document.getElementById('pos-z'),
            gripperState: document.getElementById('gripper-state'),
            commandCount: document.getElementById('command-count'),
            uptime: document.getElementById('uptime')
        };

        this.init();
    }

    init() {
        this.connectWebSocket();
        this.bindEvents();
        this.startStatusPolling();
    }

    // =====================================================
    // WebSocket
    // =====================================================

    connectWebSocket() {
        const wsUrl = `ws://${window.location.host}/ws`;

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                console.log('WebSocket connected');
                this.setConnectionStatus(true);
                this.wsRetryCount = 0;
            };

            this.ws.onclose = () => {
                console.log('WebSocket disconnected');
                this.setConnectionStatus(false);
                this.scheduleReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };

            this.ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                this.handleWebSocketMessage(data);
            };

        } catch (error) {
            console.error('Failed to connect WebSocket:', error);
            this.scheduleReconnect();
        }
    }

    scheduleReconnect() {
        if (this.wsRetryCount < this.maxRetries) {
            this.wsRetryCount++;
            const delay = Math.min(1000 * Math.pow(2, this.wsRetryCount), 30000);
            console.log(`Reconnecting in ${delay}ms...`);
            setTimeout(() => this.connectWebSocket(), delay);
        }
    }

    handleWebSocketMessage(data) {
        switch (data.type) {
            case 'status':
                this.updateStatus(data.data);
                break;
            case 'command_result':
                this.handleCommandResult(data.data);
                break;
            case 'emergency_stop':
                this.handleEmergencyStop();
                break;
            case 'pong':
                // Ping-pong response
                break;
        }
    }

    // =====================================================
    // Event Binding
    // =====================================================

    bindEvents() {
        // Command send
        this.elements.sendBtn.addEventListener('click', () => this.sendCommand());
        this.elements.commandInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') this.sendCommand();
        });

        // Quick command buttons
        document.querySelectorAll('.quick-cmd').forEach(btn => {
            btn.addEventListener('click', () => {
                const cmd = btn.dataset.cmd;
                this.elements.commandInput.value = cmd;
                this.sendCommand();
            });
        });

        // Direction buttons
        document.querySelectorAll('.dir-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const action = btn.dataset.action;
                this.sendDirectionCommand(action);
            });
        });

        // Gripper buttons
        document.querySelectorAll('.gripper-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const action = btn.dataset.action;
                this.sendGripperCommand(action);
            });
        });

        // Distance slider
        this.elements.distanceSlider.addEventListener('input', (e) => {
            this.currentDistance = parseInt(e.target.value);
            this.elements.distanceValue.textContent = this.currentDistance;
        });

        // Emergency stop
        this.elements.emergencyBtn.addEventListener('click', () => this.emergencyStop());

        // Reset
        this.elements.resetBtn.addEventListener('click', () => this.reset());
    }

    // =====================================================
    // Command Sending
    // =====================================================

    async sendCommand() {
        const command = this.elements.commandInput.value.trim();
        if (!command) return;

        this.addLogEntry(command, 'pending', 'Sending...');
        this.elements.commandInput.value = '';
        this.elements.sendBtn.disabled = true;

        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command })
            });

            const result = await response.json();

            if (result.success) {
                this.addLogEntry(command, 'success', 'Completed');
            } else {
                this.addLogEntry(command, 'error', result.message || 'Failed');
            }

        } catch (error) {
            this.addLogEntry(command, 'error', `Error: ${error.message}`);
        } finally {
            this.elements.sendBtn.disabled = false;
        }
    }

    sendDirectionCommand(action) {
        const commands = {
            'up': `up ${this.currentDistance}cm`,
            'down': `down ${this.currentDistance}cm`,
            'left': `left ${this.currentDistance}cm`,
            'right': `right ${this.currentDistance}cm`,
            'forward': `forward ${this.currentDistance}cm`,
            'backward': `backward ${this.currentDistance}cm`,
            'stop': 'stop'
        };

        const cmd = commands[action];
        if (cmd) {
            this.elements.commandInput.value = cmd;
            this.sendCommand();
        }
    }

    sendGripperCommand(action) {
        const commands = {
            'grip_open': 'open gripper',
            'grip_close': 'close gripper'
        };

        const cmd = commands[action];
        if (cmd) {
            this.elements.commandInput.value = cmd;
            this.sendCommand();
        }
    }

    async emergencyStop() {
        try {
            const response = await fetch('/api/emergency_stop', { method: 'POST' });
            const result = await response.json();

            if (result.status === 'emergency_stopped') {
                this.handleEmergencyStop();
            }
        } catch (error) {
            console.error('Emergency stop error:', error);
            this.addLogEntry('Emergency Stop', 'error', error.message);
        }
    }

    async reset() {
        try {
            const response = await fetch('/api/reset', { method: 'POST' });
            const result = await response.json();

            if (result.status === 'reset') {
                this.isEmergencyStopped = false;
                this.elements.emergencyBtn.classList.remove('active');
                this.elements.resetBtn.disabled = true;
                this.addLogEntry('System', 'success', 'Reset completed');
            }
        } catch (error) {
            console.error('Reset error:', error);
        }
    }

    // =====================================================
    // UI Updates
    // =====================================================

    setConnectionStatus(connected) {
        this.isConnected = connected;
        this.elements.statusDot.className = `status-dot ${connected ? 'connected' : 'disconnected'}`;
        this.elements.statusText.textContent = connected ? 'Connected' : 'Disconnected';
    }

    updateStatus(status) {
        // State update
        this.elements.robotState.textContent = this.translateState(status.state);

        // Position update
        if (status.position) {
            this.elements.posX.textContent = status.position[0]?.toFixed(3) || '-';
            this.elements.posY.textContent = status.position[1]?.toFixed(3) || '-';
            this.elements.posZ.textContent = status.position[2]?.toFixed(3) || '-';
        }

        // Emergency stop check
        if (status.emergency_stopped && !this.isEmergencyStopped) {
            this.handleEmergencyStop();
        }
    }

    translateState(state) {
        const translations = {
            'pending': 'Idle',
            'processing': 'Processing',
            'executing': 'Moving',
            'completed': 'Idle',
            'failed': 'Error',
            'cancelled': 'Cancelled'
        };
        return translations[state] || state;
    }

    handleCommandResult(result) {
        if (result.success) {
            this.addLogEntry(
                result.command_id?.substring(0, 8) || 'cmd',
                'success',
                result.message
            );
        } else {
            this.addLogEntry(
                result.command_id?.substring(0, 8) || 'cmd',
                'error',
                result.message
            );
        }
    }

    handleEmergencyStop() {
        this.isEmergencyStopped = true;
        this.elements.emergencyBtn.classList.add('active');
        this.elements.resetBtn.disabled = false;
        this.addLogEntry('System', 'error', 'EMERGENCY STOP ACTIVATED');
    }

    addLogEntry(command, status, message) {
        const entry = document.createElement('div');
        entry.className = `log-entry ${status}`;

        const time = new Date().toLocaleTimeString();
        entry.innerHTML = `
            <div class="time">${time}</div>
            <div class="message"><strong>${command}</strong>: ${message}</div>
        `;

        this.elements.logContainer.insertBefore(entry, this.elements.logContainer.firstChild);

        // Keep max 100 log entries
        while (this.elements.logContainer.children.length > 100) {
            this.elements.logContainer.removeChild(this.elements.logContainer.lastChild);
        }
    }

    // =====================================================
    // Status Polling
    // =====================================================

    startStatusPolling() {
        setInterval(async () => {
            if (!this.isConnected) return;

            try {
                const response = await fetch('/api/status');
                const status = await response.json();

                if (status.connected) {
                    this.elements.gripperState.textContent = status.gripper_state || '-';
                    this.elements.commandCount.textContent = status.command_count || 0;

                    if (status.uptime) {
                        this.elements.uptime.textContent = `Uptime: ${this.formatUptime(status.uptime)}`;
                    }
                }
            } catch (error) {
                // Silently fail
            }
        }, 2000); // Poll every 2 seconds
    }

    formatUptime(seconds) {
        const h = Math.floor(seconds / 3600);
        const m = Math.floor((seconds % 3600) / 60);
        const s = Math.floor(seconds % 60);
        return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    }
}

// Start application
document.addEventListener('DOMContentLoaded', () => {
    window.robotUI = new RobotControlUI();
});
