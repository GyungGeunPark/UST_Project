#!/usr/bin/env python3
"""
Isaac Sim LLM Robot Control - Isaac Sim Integration Entry Point

This script runs the LLM robot control system with Isaac Sim.
Requires Isaac Sim to be installed and available.

Usage:
    # Run with Isaac Sim Python
    ~/.local/share/ov/pkg/isaac-sim-*/python.sh scripts/run_with_isaac.py
"""

import argparse
import asyncio
import logging
import sys
from pathlib import Path

# Isaac Sim imports (must be first)
try:
    from omni.isaac.kit import SimulationApp

    # Create simulation app
    simulation_app = SimulationApp({"headless": False})

    # Now import other Isaac Sim modules
    import omni
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot

except ImportError as e:
    print(f"Error: Isaac Sim not found. {e}")
    print("Please run this script using Isaac Sim Python:")
    print("  ~/.local/share/ov/pkg/isaac-sim-*/python.sh scripts/run_with_isaac.py")
    sys.exit(1)

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.config_loader import load_config
from utils.logging_config import setup_logging
from core.control_manager import ControlManager
from isaac_interface.robot_controller import RobotController
from safety.emergency_stop import EmergencyStopSystem, SafetyMonitor
from web.server import run_server_async, set_control_manager


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Isaac Sim LLM Robot Control"
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
        help="Web server port"
    )
    parser.add_argument(
        "--usd-path",
        type=str,
        default=None,
        help="USD scene file path"
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run in headless mode"
    )

    return parser.parse_args()


async def main():
    """Main entry point"""
    args = parse_args()

    # Load configuration
    config_dir = args.config_dir or str(Path(__file__).parent.parent / "config")
    config = load_config(config_dir)

    # Setup logging
    log_config = config.get("server", {}).get("logging", {})
    setup_logging(log_config)

    logger = logging.getLogger(__name__)
    logger.info("Starting Isaac Sim LLM Robot Control")

    # Create Isaac Sim world
    world = World()
    await world.initialize_simulation_context_async()

    # Load scene or robot
    robot_config = config.get("robot", {})
    usd_path = args.usd_path or robot_config.get("files", {}).get("usd_path")

    if usd_path:
        logger.info(f"Loading USD: {usd_path}")
        # Load USD file
        omni.usd.get_context().open_stage(usd_path)
        await world.reset_async()

    # Get robot articulation
    prim_path = robot_config.get("prim_path", "/World/stretch")
    robot = world.scene.get_object(prim_path)

    if robot is None:
        logger.warning(f"Robot not found at {prim_path}, creating placeholder")
        # You may want to create or spawn the robot here

    # Create robot controller
    robot_controller = RobotController(config)
    if robot:
        robot_controller.initialize(world, robot)
    else:
        robot_controller.initialize_standalone()

    # Create emergency stop system
    emergency_config = config.get("server", {}).get("emergency_stop", {})
    emergency_system = EmergencyStopSystem(emergency_config)
    emergency_system.set_robot_controller(robot_controller)
    emergency_system.start_watchdog()

    # Create safety monitor
    workspace_config = config.get("workspace", {})
    safety_monitor = SafetyMonitor(
        emergency_system,
        workspace_config,
        workspace_config.get("velocity_limits", {})
    )

    # Create control manager
    control_manager = ControlManager(config)
    control_manager.set_robot_controller(robot_controller)
    control_manager.set_emergency_system(emergency_system)

    logger.info("All systems initialized")

    # Start web server
    server_config = {
        "host": "0.0.0.0",
        "port": args.port
    }
    set_control_manager(control_manager)

    # Create tasks
    web_task = asyncio.create_task(run_server_async(control_manager, server_config))
    monitor_task = asyncio.create_task(
        safety_monitor.start_monitoring(robot_controller)
    )

    logger.info(f"Web server started on http://0.0.0.0:{args.port}")

    # Main simulation loop
    try:
        while simulation_app.is_running():
            world.step(render=True)
            await asyncio.sleep(0.01)  # 100Hz

            # Update heartbeat
            emergency_system.heartbeat()

    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        # Cleanup
        safety_monitor.stop_monitoring()
        emergency_system.stop_watchdog()
        web_task.cancel()
        monitor_task.cancel()

        simulation_app.close()


if __name__ == "__main__":
    asyncio.run(main())
