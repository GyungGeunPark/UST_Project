#!/usr/bin/env python3
# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST ROS2 Bridge Script.

Isaac Sim과 실제 로봇 간의 ROS2 통신 브릿지를 설정합니다.

사용법:
    # Isaac Sim에서 ROS2 퍼블리시 (시뮬레이션 → 실제)
    ./isaaclab.sh -p scripts/run_ros2_bridge.py --mode sim2real

    # ROS2에서 Isaac Sim 제어 (실제 → 시뮬레이션)
    ./isaaclab.sh -p scripts/run_ros2_bridge.py --mode real2sim

    # 양방향 (미러링)
    ./isaaclab.sh -p scripts/run_ros2_bridge.py --mode bidirectional

필요 조건:
    - ROS2 Humble 설치
    - ros2_control 패키지
    - turtlebot3 패키지
    - open_manipulator_x 패키지
"""

from __future__ import annotations

import argparse
import sys
import os
from typing import Optional

# 프로젝트 경로 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ROS2 임포트 시도
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Twist, PoseStamped
    from std_msgs.msg import Float64MultiArray, Bool
    from nav_msgs.msg import Odometry
    ROS2_AVAILABLE = True
except ImportError:
    print("[WARNING] ROS2 not available. Bridge will run in simulation-only mode.")
    ROS2_AVAILABLE = False

# Isaac Lab AppLauncher 설정
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="UST ROS2 Bridge")
parser.add_argument(
    "--mode",
    type=str,
    default="sim2real",
    choices=["sim2real", "real2sim", "bidirectional"],
    help="브릿지 모드",
)
parser.add_argument(
    "--publish_rate",
    type=float,
    default=50.0,
    help="ROS2 퍼블리시 주파수 (Hz)",
)
parser.add_argument(
    "--enable_base",
    action="store_true",
    default=True,
    help="모바일 베이스 제어 활성화",
)
parser.add_argument(
    "--enable_arm",
    action="store_true",
    default=True,
    help="매니퓰레이터 암 제어 활성화",
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import numpy as np
from isaaclab.envs import ManagerBasedEnv

from ust_config.ust_teleop_env_cfg import USTMobileManipulatorTeleopEnvCfg
from ust_controllers.differential_drive_controller import (
    DifferentialDriveController,
    DifferentialDriveOdometry,
)


class USTRobotBridge:
    """Isaac Sim ↔ 실제 로봇 ROS2 브릿지.

    주요 기능:
    - 시뮬레이션 관절 상태 → ROS2 JointState 퍼블리시
    - ROS2 cmd_vel → 시뮬레이션 베이스 제어
    - ROS2 arm_controller/command → 시뮬레이션 암 제어
    - 시뮬레이션 카메라 → ROS2 Image 퍼블리시
    """

    def __init__(
        self,
        env: ManagerBasedEnv,
        mode: str = "sim2real",
        publish_rate: float = 50.0,
        enable_base: bool = True,
        enable_arm: bool = True,
    ):
        """브릿지 초기화.

        Args:
            env: Isaac Lab 환경
            mode: 브릿지 모드
            publish_rate: ROS2 퍼블리시 주파수
            enable_base: 베이스 제어 활성화
            enable_arm: 암 제어 활성화
        """
        self.env = env
        self.mode = mode
        self.publish_rate = publish_rate
        self.enable_base = enable_base
        self.enable_arm = enable_arm

        # 차동 구동 컨트롤러
        self.base_controller = DifferentialDriveController()
        self.odometry = DifferentialDriveOdometry()

        # ROS2 초기화 (가능한 경우)
        self._ros_node: Optional[Node] = None
        self._init_ros2()

        # 외부 명령 버퍼
        self._cmd_vel = (0.0, 0.0)  # (linear, angular)
        self._right_arm_cmd = [0.0, -0.5, 0.3, 0.2]  # 오른쪽 암 초기 관절 위치
        self._left_arm_cmd = [0.0, -0.5, 0.3, 0.2]   # 왼쪽 암 초기 관절 위치
        self._right_gripper_cmd = 0.015  # 오른쪽 그리퍼 열린 상태
        self._left_gripper_cmd = 0.015   # 왼쪽 그리퍼 열린 상태

    def _init_ros2(self):
        """ROS2 노드 초기화."""
        if not ROS2_AVAILABLE:
            return

        rclpy.init()
        self._ros_node = USTBridgeNode(self)
        print("[INFO] ROS2 bridge node initialized.")

    def update(self, obs: dict, action: torch.Tensor) -> torch.Tensor:
        """브릿지 업데이트.

        Args:
            obs: 현재 관측
            action: 제안된 액션 (텔레옵 또는 정책)

        Returns:
            최종 액션 (ROS2 명령 반영)
        """
        if self._ros_node is not None:
            # ROS2 콜백 처리
            rclpy.spin_once(self._ros_node, timeout_sec=0.001)

        if self.mode == "sim2real":
            # 시뮬레이션 상태를 ROS2로 퍼블리시
            self._publish_sim_state(obs)
            return action

        elif self.mode == "real2sim":
            # ROS2 명령을 시뮬레이션에 적용
            return self._apply_ros_commands()

        elif self.mode == "bidirectional":
            # 양방향
            self._publish_sim_state(obs)
            return self._apply_ros_commands()

        return action

    def _publish_sim_state(self, obs: dict):
        """시뮬레이션 상태를 ROS2로 퍼블리시."""
        if self._ros_node is None:
            return

        policy_obs = obs.get("policy", {})

        # 오른쪽 암 관절 상태
        right_arm_pos = None
        right_gripper_pos = None
        if "right_arm_joint_pos" in policy_obs:
            right_arm_pos = policy_obs["right_arm_joint_pos"][0].cpu().numpy()
        if "right_gripper_pos" in policy_obs:
            right_gripper_pos = policy_obs["right_gripper_pos"][0].cpu().numpy()

        # 왼쪽 암 관절 상태
        left_arm_pos = None
        left_gripper_pos = None
        if "left_arm_joint_pos" in policy_obs:
            left_arm_pos = policy_obs["left_arm_joint_pos"][0].cpu().numpy()
        if "left_gripper_pos" in policy_obs:
            left_gripper_pos = policy_obs["left_gripper_pos"][0].cpu().numpy()

        if right_arm_pos is not None and left_arm_pos is not None:
            self._ros_node.publish_joint_state(
                right_arm_positions=right_arm_pos,
                right_gripper_position=right_gripper_pos[0] if right_gripper_pos is not None else 0.015,
                left_arm_positions=left_arm_pos,
                left_gripper_position=left_gripper_pos[0] if left_gripper_pos is not None else 0.015,
            )

        # 오른쪽 EE 포즈
        if "right_ee_pose" in policy_obs:
            right_ee = policy_obs["right_ee_pose"][0].cpu().numpy()
            self._ros_node.publish_ee_pose(position=right_ee[:3], orientation=right_ee[3:], side="right")

        # 왼쪽 EE 포즈
        if "left_ee_pose" in policy_obs:
            left_ee = policy_obs["left_ee_pose"][0].cpu().numpy()
            self._ros_node.publish_ee_pose(position=left_ee[:3], orientation=left_ee[3:], side="left")

    def _apply_ros_commands(self) -> torch.Tensor:
        """ROS2 명령을 액션 텐서로 변환.

        액션 레이아웃 (18D):
        [0:4] wheel velocities (LF, RF, LR, RR)
        [4:10] right arm delta pose (position 3D + rotation 3D)
        [10] right gripper command
        [11:17] left arm delta pose (position 3D + rotation 3D)
        [17] left gripper command
        """
        action = torch.zeros(1, 18, device=self.env.device)

        # 베이스 명령 (cmd_vel → 4 wheel velocities)
        if self.enable_base:
            linear, angular = self._cmd_vel
            left_vel, right_vel = self.base_controller.cmd_vel_to_wheel_velocities(
                linear, angular
            )
            action[0, 0] = left_vel    # wheel_left_front
            action[0, 1] = right_vel   # wheel_right_front
            action[0, 2] = left_vel    # wheel_left_rear
            action[0, 3] = right_vel   # wheel_right_rear

        # 오른쪽 그리퍼 명령
        action[0, 10] = self._right_gripper_cmd

        # 왼쪽 그리퍼 명령
        action[0, 17] = self._left_gripper_cmd

        return action

    def set_cmd_vel(self, linear: float, angular: float):
        """cmd_vel 명령 설정."""
        self._cmd_vel = (linear, angular)

    def set_right_arm_command(self, positions: list):
        """오른쪽 암 관절 명령 설정."""
        self._right_arm_cmd = positions[:4]

    def set_left_arm_command(self, positions: list):
        """왼쪽 암 관절 명령 설정."""
        self._left_arm_cmd = positions[:4]

    def set_right_gripper_command(self, position: float):
        """오른쪽 그리퍼 명령 설정."""
        self._right_gripper_cmd = position

    def set_left_gripper_command(self, position: float):
        """왼쪽 그리퍼 명령 설정."""
        self._left_gripper_cmd = position

    def close(self):
        """브릿지 종료."""
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            rclpy.shutdown()


if ROS2_AVAILABLE:
    class USTBridgeNode(Node):
        """UST ROS2 브릿지 노드."""

        def __init__(self, bridge: USTRobotBridge):
            super().__init__('ust_robot_bridge')
            self.bridge = bridge

            # QoS 프로파일
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

            # 퍼블리셔
            self.joint_state_pub = self.create_publisher(
                JointState, '/joint_states', qos
            )
            self.right_ee_pose_pub = self.create_publisher(
                PoseStamped, '/right_ee_pose', qos
            )
            self.left_ee_pose_pub = self.create_publisher(
                PoseStamped, '/left_ee_pose', qos
            )
            self.odom_pub = self.create_publisher(
                Odometry, '/odom', qos
            )

            # 서브스크라이버
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self._cmd_vel_callback, qos
            )
            self.right_arm_cmd_sub = self.create_subscription(
                Float64MultiArray, '/right_arm_controller/command',
                self._right_arm_cmd_callback, qos
            )
            self.left_arm_cmd_sub = self.create_subscription(
                Float64MultiArray, '/left_arm_controller/command',
                self._left_arm_cmd_callback, qos
            )
            self.right_gripper_cmd_sub = self.create_subscription(
                Float64MultiArray, '/right_gripper_controller/command',
                self._right_gripper_cmd_callback, qos
            )
            self.left_gripper_cmd_sub = self.create_subscription(
                Float64MultiArray, '/left_gripper_controller/command',
                self._left_gripper_cmd_callback, qos
            )

            self.get_logger().info('UST Bridge Node initialized')

        def _cmd_vel_callback(self, msg: Twist):
            """cmd_vel 콜백."""
            self.bridge.set_cmd_vel(msg.linear.x, msg.angular.z)

        def _right_arm_cmd_callback(self, msg: Float64MultiArray):
            """오른쪽 암 명령 콜백."""
            self.bridge.set_right_arm_command(list(msg.data))

        def _left_arm_cmd_callback(self, msg: Float64MultiArray):
            """왼쪽 암 명령 콜백."""
            self.bridge.set_left_arm_command(list(msg.data))

        def _right_gripper_cmd_callback(self, msg: Float64MultiArray):
            """오른쪽 그리퍼 명령 콜백."""
            if len(msg.data) > 0:
                self.bridge.set_right_gripper_command(msg.data[0])

        def _left_gripper_cmd_callback(self, msg: Float64MultiArray):
            """왼쪽 그리퍼 명령 콜백."""
            if len(msg.data) > 0:
                self.bridge.set_left_gripper_command(msg.data[0])

        def publish_joint_state(
            self,
            right_arm_positions: np.ndarray,
            right_gripper_position: float,
            left_arm_positions: np.ndarray,
            left_gripper_position: float,
        ):
            """관절 상태 퍼블리시 (듀얼 암)."""
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.name = [
                'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4',
                'right_gripper_left_joint', 'right_gripper_right_joint',
                'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4',
                'left_gripper_left_joint', 'left_gripper_right_joint',
            ]
            msg.position = (
                list(right_arm_positions) + [right_gripper_position, right_gripper_position]
                + list(left_arm_positions) + [left_gripper_position, left_gripper_position]
            )

            self.joint_state_pub.publish(msg)

        def publish_ee_pose(
            self,
            position: np.ndarray,
            orientation: np.ndarray,
            side: str = "right",
        ):
            """EE 포즈 퍼블리시."""
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            msg.pose.position.x = float(position[0])
            msg.pose.position.y = float(position[1])
            msg.pose.position.z = float(position[2])

            msg.pose.orientation.w = float(orientation[0])
            msg.pose.orientation.x = float(orientation[1])
            msg.pose.orientation.y = float(orientation[2])
            msg.pose.orientation.z = float(orientation[3])

            if side == "right":
                self.right_ee_pose_pub.publish(msg)
            else:
                self.left_ee_pose_pub.publish(msg)


def main():
    """메인 함수."""
    print("\n" + "=" * 50)
    print("     UST ROS2 Bridge")
    print("=" * 50)
    print(f"\n  Mode: {args_cli.mode}")
    print(f"  Publish rate: {args_cli.publish_rate} Hz")
    print(f"  Base control: {args_cli.enable_base}")
    print(f"  Arm control: {args_cli.enable_arm}")
    print("\n" + "=" * 50 + "\n")

    if not ROS2_AVAILABLE:
        print("[WARNING] Running without ROS2 (simulation only mode)")

    # 환경 생성
    print("[INFO] Creating environment...")
    env_cfg = USTMobileManipulatorTeleopEnvCfg()
    env_cfg.scene.num_envs = 1
    env = ManagerBasedEnv(cfg=env_cfg)

    # 브릿지 생성
    bridge = USTRobotBridge(
        env=env,
        mode=args_cli.mode,
        publish_rate=args_cli.publish_rate,
        enable_base=args_cli.enable_base,
        enable_arm=args_cli.enable_arm,
    )

    # 메인 루프
    obs, info = env.reset()
    print("[INFO] Bridge running. Press Ctrl+C to exit.")

    try:
        while simulation_app.is_running():
            with torch.inference_mode():
                # 기본 액션 (제로)
                action = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)

                # 브릿지 업데이트
                action = bridge.update(obs, action)

                # 환경 스텝
                obs, info = env.step(action)

    except KeyboardInterrupt:
        print("\n[INFO] Bridge stopped by user.")

    finally:
        bridge.close()
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
