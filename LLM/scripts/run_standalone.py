#!/usr/bin/env python3
"""
Isaac Sim LLM Robot Control - Standalone Entry Point

This script runs the LLM robot control system without Isaac Sim.
Useful for testing the web interface and LLM integration.

Usage:
    python scripts/run_standalone.py
    python scripts/run_standalone.py --port 8080
    python scripts/run_standalone.py --no-web
"""

import argparse
import asyncio
import logging
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.config_loader import load_config
from utils.logging_config import setup_logging
from core.control_manager import ControlManager
from safety.emergency_stop import EmergencyStopSystem
from web.server import start_server, set_control_manager, run_server_async


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Isaac Sim LLM Robot Control - Standalone Mode"
    )
    parser.add_argument(
        "--config-dir",
        type=str,
        default=None,
        help="Configuration directory path"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Web server port (default: 8000)"
    )
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Web server host (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--no-web",
        action="store_true",
        help="Run without web server"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )

    return parser.parse_args()


def main():
    """Main entry point"""
    args = parse_args()

    # Load configuration
    config_dir = args.config_dir or str(Path(__file__).parent.parent / "config")
    config = load_config(config_dir)

    # Setup logging
    log_config = config.get("server", {}).get("logging", {})
    if args.debug:
        log_config["level"] = "DEBUG"
    setup_logging(log_config)

    logger = logging.getLogger(__name__)
    logger.info("Starting Isaac Sim LLM Robot Control (Standalone Mode)")

    # Create emergency stop system
    emergency_config = config.get("server", {}).get("emergency_stop", {})
    emergency_system = EmergencyStopSystem(emergency_config)

    # Create control manager
    control_manager = ControlManager(config)
    control_manager.set_emergency_system(emergency_system)

    logger.info("Control manager initialized")
    logger.info("Running in simulation mode (no Isaac Sim connection)")

    if args.no_web:
        # CLI mode
        run_cli_mode(control_manager)
    else:
        # Web server mode
        server_config = {
            "host": args.host,
            "port": args.port
        }
        logger.info(f"Starting web server on http://{args.host}:{args.port}")
        start_server(control_manager, server_config)


def run_cli_mode(control_manager: ControlManager):
    """Run in CLI mode without web server"""
    logger = logging.getLogger(__name__)
    logger.info("Running in CLI mode. Type 'quit' to exit.")

    async def process_commands():
        while True:
            try:
                command = input("\nEnter command: ").strip()

                if command.lower() in ['quit', 'exit', 'q']:
                    break

                if command.lower() == 'status':
                    status = control_manager.get_status()
                    print(f"\nRobot Status:")
                    print(f"  State: {status.state.value}")
                    print(f"  Position: {status.current_position}")
                    print(f"  Gripper: {status.gripper_state}")
                    print(f"  Emergency Stopped: {status.emergency_stopped}")
                    continue

                if command.lower() == 'stop':
                    control_manager.emergency_stop()
                    print("Emergency stop triggered!")
                    continue

                if command.lower() == 'reset':
                    control_manager.reset()
                    print("Reset completed!")
                    continue

                if not command:
                    continue

                # Process command
                print(f"Processing: {command}")
                result = await control_manager.process_command(command)

                if result.success:
                    print(f"Success: {result.message}")
                    if result.final_position:
                        print(f"Final position: {result.final_position}")
                else:
                    print(f"Failed: {result.message}")
                    if result.error_code:
                        print(f"Error code: {result.error_code}")

            except KeyboardInterrupt:
                print("\nInterrupted")
                break
            except EOFError:
                break
            except Exception as e:
                logger.error(f"Error: {e}")

    asyncio.run(process_commands())


if __name__ == "__main__":
    main()
