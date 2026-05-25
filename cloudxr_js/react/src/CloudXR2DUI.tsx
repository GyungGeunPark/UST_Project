/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * CloudXR2DUI.tsx - CloudXR 2D User Interface Management
 *
 * This class handles all the HTML form interactions, localStorage persistence,
 * and form validation for the CloudXR React example. It follows the same pattern
 * as the simple example's CloudXRWebUI class, providing a clean separation
 * between UI management and React component logic.
 *
 * Features:
 * - Form field management and localStorage persistence
 * - Proxy configuration based on protocol
 * - Form validation and default value handling
 * - Event listener management
 * - Error handling and logging
 */

import { CloudXRConfig, enableLocalStorage, setupCertificateAcceptanceLink } from '@helpers/utils';

/**
 * 2D UI Management for CloudXR React Example
 * Handles the main user interface for CloudXR streaming, including form management,
 * localStorage persistence, and user interaction controls.
 */
export class CloudXR2DUI {
  /** Button to initiate XR streaming session */
  private startButton!: HTMLButtonElement;
  /** Input field for the CloudXR server IP address */
  private serverIpInput!: HTMLInputElement;
  /** Input field for the CloudXR server port number */
  private portInput!: HTMLInputElement;
  /** Input field for proxy URL configuration */
  private proxyUrlInput!: HTMLInputElement;
  /** Dropdown to select between AR and VR immersive modes */
  private immersiveSelect!: HTMLSelectElement;
  /** Dropdown to select device frame rate (FPS) */
  private deviceFrameRateSelect!: HTMLSelectElement;
  /** Dropdown to select max streaming bitrate (Mbps) */
  private maxStreamingBitrateMbpsSelect!: HTMLSelectElement;
  /** Input field for per-eye width configuration */
  private perEyeWidthInput!: HTMLInputElement;
  /** Input field for per-eye height configuration */
  private perEyeHeightInput!: HTMLInputElement;
  /** Dropdown to select server backend type */
  private serverTypeSelect!: HTMLSelectElement;
  /** Dropdown to select application type */
  private appSelect!: HTMLSelectElement;
  /** Dropdown to select reference space for XR tracking */
  private referenceSpaceSelect!: HTMLSelectElement;
  /** Input for XR reference space X offset (cm) */
  private xrOffsetXInput!: HTMLInputElement;
  /** Input for XR reference space Y offset (cm) */
  private xrOffsetYInput!: HTMLInputElement;
  /** Input for XR reference space Z offset (cm) */
  private xrOffsetZInput!: HTMLInputElement;
  /** Text element displaying proxy configuration help */
  private proxyDefaultText!: HTMLElement;
  /** Error message box element */
  private errorMessageBox!: HTMLElement;
  /** Error message text element */
  private errorMessageText!: HTMLElement;
  /** Certificate acceptance link container */
  private certAcceptanceLink!: HTMLElement;
  /** Certificate acceptance link anchor */
  private certLink!: HTMLAnchorElement;
  /** Flag to track if the 2D UI has been initialized */
  private initialized: boolean = false;

  /** Current form configuration state */
  private currentConfiguration: CloudXRConfig;
  /** Callback function for configuration changes */
  private onConfigurationChange: ((config: CloudXRConfig) => void) | null = null;
  /** Connect button click handler for cleanup */
  private handleConnectClick: ((event: Event) => void) | null = null;
  /** Array to store all event listeners for proper cleanup */
  private eventListeners: Array<{
    element: HTMLElement;
    event: string;
    handler: EventListener;
  }> = [];
  /** Cleanup function for certificate acceptance link */
  private certLinkCleanup: (() => void) | null = null;

  /**
   * Creates a new CloudXR2DUI instance
   * @param onConfigurationChange - Callback function called when configuration changes
   */
  constructor(onConfigurationChange?: (config: CloudXRConfig) => void) {
    this.onConfigurationChange = onConfigurationChange || null;
    this.currentConfiguration = this.getDefaultConfiguration();
  }

  /**
   * Initializes the CloudXR2DUI with all necessary components and event handlers
   */
  public initialize(): void {
    if (this.initialized) {
      return;
    }

    try {
      this.initializeElements();
      this.setupLocalStorage();
      this.setupProxyConfiguration();
      this.setupEventListeners();
      this.updateConfiguration();
      this.setStartButtonState(false, 'CONNECT');
      this.initialized = true;
    } catch (error) {
      // Continue with default values if initialization fails
      this.showError(`Failed to initialize CloudXR2DUI: ${error}`);
    }
  }

  /**
   * Initializes all DOM element references by their IDs
   * Throws an error if any required element is not found
   */
  private initializeElements(): void {
    this.startButton = this.getElement<HTMLButtonElement>('startButton');
    this.serverIpInput = this.getElement<HTMLInputElement>('serverIpInput');
    this.portInput = this.getElement<HTMLInputElement>('portInput');
    this.proxyUrlInput = this.getElement<HTMLInputElement>('proxyUrl');
    this.immersiveSelect = this.getElement<HTMLSelectElement>('immersive');
    this.deviceFrameRateSelect = this.getElement<HTMLSelectElement>('deviceFrameRate');
    this.maxStreamingBitrateMbpsSelect =
      this.getElement<HTMLSelectElement>('maxStreamingBitrateMbps');
    this.perEyeWidthInput = this.getElement<HTMLInputElement>('perEyeWidth');
    this.perEyeHeightInput = this.getElement<HTMLInputElement>('perEyeHeight');
    this.serverTypeSelect = this.getElement<HTMLSelectElement>('serverType');
    this.appSelect = this.getElement<HTMLSelectElement>('app');
    this.referenceSpaceSelect = this.getElement<HTMLSelectElement>('referenceSpace');
    this.xrOffsetXInput = this.getElement<HTMLInputElement>('xrOffsetX');
    this.xrOffsetYInput = this.getElement<HTMLInputElement>('xrOffsetY');
    this.xrOffsetZInput = this.getElement<HTMLInputElement>('xrOffsetZ');
    this.proxyDefaultText = this.getElement<HTMLElement>('proxyDefaultText');
    this.errorMessageBox = this.getElement<HTMLElement>('errorMessageBox');
    this.errorMessageText = this.getElement<HTMLElement>('errorMessageText');
    this.certAcceptanceLink = this.getElement<HTMLElement>('certAcceptanceLink');
    this.certLink = this.getElement<HTMLAnchorElement>('certLink');
  }

  /**
   * Gets a DOM element by ID with type safety
   * @param id - The element ID to find
   * @returns The found element with the specified type
   * @throws Error if element is not found
   */
  private getElement<T extends HTMLElement>(id: string): T {
    const element = document.getElementById(id) as T;
    if (!element) {
      throw new Error(`Element with id '${id}' not found`);
    }
    return element;
  }

  /**
   * Gets the default configuration values
   * @returns Default configuration object
   */
  private getDefaultConfiguration(): CloudXRConfig {
    const useSecure = typeof window !== 'undefined' ? window.location.protocol === 'https:' : false;
    // Default port: HTTP → 49100, HTTPS without proxy → 48322, HTTPS with proxy → 443
    const defaultPort = useSecure ? 48322 : 49100;
    return {
      serverIP: '127.0.0.1',
      port: defaultPort,
      useSecureConnection: useSecure,
      perEyeWidth: 2048,
      perEyeHeight: 1792,
      deviceFrameRate: 90,
      maxStreamingBitrateMbps: 150,
      immersiveMode: 'ar',
      app: 'generic',
      serverType: 'manual',
      proxyUrl: '',
      referenceSpaceType: 'auto',
    };
  }

  /**
   * Enables localStorage persistence for form inputs
   * Automatically saves and restores user preferences
   */
  private setupLocalStorage(): void {
    enableLocalStorage(this.serverTypeSelect, 'serverType');
    enableLocalStorage(this.serverIpInput, 'serverIp');
    enableLocalStorage(this.portInput, 'port');
    enableLocalStorage(this.perEyeWidthInput, 'perEyeWidth');
    enableLocalStorage(this.perEyeHeightInput, 'perEyeHeight');
    enableLocalStorage(this.proxyUrlInput, 'proxyUrl');
    enableLocalStorage(this.deviceFrameRateSelect, 'deviceFrameRate');
    enableLocalStorage(this.maxStreamingBitrateMbpsSelect, 'maxStreamingBitrateMbps');
    enableLocalStorage(this.immersiveSelect, 'immersiveMode');
    enableLocalStorage(this.appSelect, 'app');
    enableLocalStorage(this.referenceSpaceSelect, 'referenceSpace');
    enableLocalStorage(this.xrOffsetXInput, 'xrOffsetX');
    enableLocalStorage(this.xrOffsetYInput, 'xrOffsetY');
    enableLocalStorage(this.xrOffsetZInput, 'xrOffsetZ');
  }

  /**
   * Configures proxy settings based on the current protocol (HTTP/HTTPS)
   * Sets appropriate placeholders and help text for port and proxy URL inputs
   */
  private setupProxyConfiguration(): void {
    // Update port placeholder based on protocol
    if (window.location.protocol === 'https:') {
      this.portInput.placeholder = 'Port (default: 48322, or 443 if proxy URL set)';
    } else {
      this.portInput.placeholder = 'Port (default: 49100)';
    }

    // Set default text and placeholder based on protocol
    if (window.location.protocol === 'https:') {
      this.proxyDefaultText.textContent =
        'Optional: Leave empty for direct WSS connection, or provide URL for proxy routing (e.g., https://proxy.example.com/)';
      this.proxyUrlInput.placeholder = '';
    } else {
      this.proxyDefaultText.textContent = 'Not needed for HTTP - uses direct WS connection';
      this.proxyUrlInput.placeholder = '';
    }
  }

  /**
   * Sets up event listeners for form input changes
   * Handles both input and change events for better compatibility
   */
  private setupEventListeners(): void {
    // Update configuration when form inputs change
    const updateConfig = () => this.updateConfiguration();

    // Helper function to add listeners and store them for cleanup
    const addListener = (element: HTMLElement, event: string, handler: EventListener) => {
      element.addEventListener(event, handler);
      this.eventListeners.push({ element, event, handler });
    };

    // Add event listeners for all form fields
    addListener(this.serverTypeSelect, 'change', updateConfig);
    addListener(this.serverIpInput, 'input', updateConfig);
    addListener(this.serverIpInput, 'change', updateConfig);
    addListener(this.portInput, 'input', updateConfig);
    addListener(this.portInput, 'change', updateConfig);
    addListener(this.perEyeWidthInput, 'input', updateConfig);
    addListener(this.perEyeWidthInput, 'change', updateConfig);
    addListener(this.perEyeHeightInput, 'input', updateConfig);
    addListener(this.perEyeHeightInput, 'change', updateConfig);
    addListener(this.deviceFrameRateSelect, 'change', updateConfig);
    addListener(this.maxStreamingBitrateMbpsSelect, 'change', updateConfig);
    addListener(this.immersiveSelect, 'change', updateConfig);
    addListener(this.appSelect, 'change', updateConfig);
    addListener(this.referenceSpaceSelect, 'change', updateConfig);
    addListener(this.xrOffsetXInput, 'input', updateConfig);
    addListener(this.xrOffsetXInput, 'change', updateConfig);
    addListener(this.xrOffsetYInput, 'input', updateConfig);
    addListener(this.xrOffsetYInput, 'change', updateConfig);
    addListener(this.xrOffsetZInput, 'input', updateConfig);
    addListener(this.xrOffsetZInput, 'change', updateConfig);
    addListener(this.proxyUrlInput, 'input', updateConfig);
    addListener(this.proxyUrlInput, 'change', updateConfig);

    // Set up certificate acceptance link and store cleanup function
    this.certLinkCleanup = setupCertificateAcceptanceLink(
      this.serverIpInput,
      this.portInput,
      this.proxyUrlInput,
      this.certAcceptanceLink,
      this.certLink
    );
  }

  /**
   * Updates the current configuration from form values
   * Calls the configuration change callback if provided
   */
  private updateConfiguration(): void {
    const useSecure = this.getDefaultConfiguration().useSecureConnection;
    const portValue = parseInt(this.portInput.value);
    const hasProxy = this.proxyUrlInput.value.trim().length > 0;

    // Smart default port based on connection type and proxy usage
    let defaultPort = 49100; // HTTP default
    if (useSecure) {
      defaultPort = hasProxy ? 443 : 48322; // HTTPS with proxy → 443, HTTPS without → 48322
    }

    const newConfiguration: CloudXRConfig = {
      serverIP: this.serverIpInput.value || this.getDefaultConfiguration().serverIP,
      port: portValue || defaultPort,
      useSecureConnection: useSecure,
      perEyeWidth:
        parseInt(this.perEyeWidthInput.value) || this.getDefaultConfiguration().perEyeWidth,
      perEyeHeight:
        parseInt(this.perEyeHeightInput.value) || this.getDefaultConfiguration().perEyeHeight,
      deviceFrameRate:
        parseInt(this.deviceFrameRateSelect.value) ||
        this.getDefaultConfiguration().deviceFrameRate,
      maxStreamingBitrateMbps:
        parseInt(this.maxStreamingBitrateMbpsSelect.value) ||
        this.getDefaultConfiguration().maxStreamingBitrateMbps,
      immersiveMode:
        (this.immersiveSelect.value as 'ar' | 'vr') || this.getDefaultConfiguration().immersiveMode,
      app: this.appSelect.value || this.getDefaultConfiguration().app,
      serverType: this.serverTypeSelect.value || this.getDefaultConfiguration().serverType,
      proxyUrl: this.proxyUrlInput.value || this.getDefaultConfiguration().proxyUrl,
      referenceSpaceType:
        (this.referenceSpaceSelect.value as 'auto' | 'local-floor' | 'local' | 'viewer') ||
        this.getDefaultConfiguration().referenceSpaceType,
      // Convert cm from UI into meters for config (respect 0; if invalid, use 0)
      xrOffsetX: (() => {
        const v = parseFloat(this.xrOffsetXInput.value);
        return Number.isFinite(v) ? v / 100 : 0;
      })(),
      xrOffsetY: (() => {
        const v = parseFloat(this.xrOffsetYInput.value);
        return Number.isFinite(v) ? v / 100 : 0;
      })(),
      xrOffsetZ: (() => {
        const v = parseFloat(this.xrOffsetZInput.value);
        return Number.isFinite(v) ? v / 100 : 0;
      })(),
    };

    this.currentConfiguration = newConfiguration;

    // Call the configuration change callback if provided
    if (this.onConfigurationChange) {
      this.onConfigurationChange(newConfiguration);
    }
  }

  /**
   * Gets the current configuration
   * @returns Current configuration object
   */
  public getConfiguration(): CloudXRConfig {
    return { ...this.currentConfiguration };
  }

  /**
   * Sets the start button state
   * @param disabled - Whether the button should be disabled
   * @param text - Text to display on the button
   */
  public setStartButtonState(disabled: boolean, text: string): void {
    if (this.startButton) {
      this.startButton.disabled = disabled;
      this.startButton.innerHTML = text;
    }
  }

  /**
   * Sets up the connect button click handler
   * @param onConnect - Function to call when connect button is clicked
   * @param onError - Function to call when an error occurs
   */
  public setupConnectButtonHandler(
    onConnect: () => Promise<void>,
    onError: (error: Error) => void
  ): void {
    if (this.startButton) {
      // Remove any existing listener
      if (this.handleConnectClick) {
        this.startButton.removeEventListener('click', this.handleConnectClick);
      }

      // Create new handler
      this.handleConnectClick = async () => {
        // Disable button during XR session
        this.setStartButtonState(true, 'CONNECT (starting XR session...)');

        try {
          await onConnect();
        } catch (error) {
          this.setStartButtonState(false, 'CONNECT');
          onError(error as Error);
        }
      };

      // Add the new listener
      this.startButton.addEventListener('click', this.handleConnectClick);
    }
  }

  /**
   * Shows a status message in the UI with a specific type
   * @param message - Message to display
   * @param type - Message type: 'success', 'error', or 'info'
   */
  public showStatus(message: string, type: 'success' | 'error' | 'info'): void {
    if (this.errorMessageText && this.errorMessageBox) {
      this.errorMessageText.textContent = message;
      this.errorMessageBox.className = `error-message-box show ${type}`;
    }
    console[type === 'error' ? 'error' : 'info'](message);
  }

  /**
   * Shows an error message in the UI
   * @param message - Error message to display
   */
  public showError(message: string): void {
    this.showStatus(message, 'error');
  }

  /**
   * Hides the error message
   */
  public hideError(): void {
    if (this.errorMessageBox) {
      this.errorMessageBox.classList.remove('show');
    }
  }

  /**
   * Cleans up event listeners and resources
   * Should be called when the component unmounts
   */
  public cleanup(): void {
    // Remove all stored event listeners
    this.eventListeners.forEach(({ element, event, handler }) => {
      element.removeEventListener(event, handler);
    });
    this.eventListeners = [];

    // Remove CONNECT button listener
    if (this.startButton && this.handleConnectClick) {
      this.startButton.removeEventListener('click', this.handleConnectClick);
      this.handleConnectClick = null;
    }

    // Clean up certificate acceptance link listeners
    if (this.certLinkCleanup) {
      this.certLinkCleanup();
      this.certLinkCleanup = null;
    }
  }
}
