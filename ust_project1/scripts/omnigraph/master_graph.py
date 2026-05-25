"""
omnigraph/master_graph.py
통합 로봇 제어 마스터 그래프

모든 제어 그래프를 통합 관리합니다:
- IK Controller Graph: 매니퓰레이터 역운동학 제어
- Differential Drive Graph: 모바일 베이스 제어
- Teleoperation Graph: VR 텔레오퍼레이션

Usage:
    from omnigraph import MasterControlGraph

    config = {
        "manipulator_path": "/World/Robot/open_manipulator_x",
        "mobile_base_path": "/World/Robot/MobileBase",
        ...
    }

    master = MasterControlGraph(config)
    results = master.create_all()
    master.save_to_usd("scene_with_graphs.usd")

Author: UST Robotics Project
Date: 2024
"""

from typing import Dict, Optional, Any, List
import os
import yaml

from .ik_graph import IKControllerGraph
from .differential_drive_graph import DifferentialDriveGraph, DifferentialDriveWithLidarGraph
from .teleoperation_graph import TeleoperationGraph
from .graph_builder import OmniGraphBuilder


class MasterControlGraph:
    """
    통합 로봇 제어 마스터 그래프

    세 가지 서브그래프를 관리합니다:
    1. IK Controller Graph - 매니퓰레이터 역운동학 제어
    2. Differential Drive Graph - 모바일 베이스 제어
    3. Teleoperation Graph - VR 텔레오퍼레이션

    모든 그래프는 USD에 저장되어 Play 버튼만 누르면 자동 실행됩니다.

    Attributes:
        config: 로봇 설정 딕셔너리

    Example:
        >>> master = MasterControlGraph(config)
        >>> master.create_all()
        >>> master.save_to_usd("/path/to/scene.usd")
    """

    # 기본 설정 파일 경로
    DEFAULT_CONFIG_PATH = "/home/isaac/ust_ws/ust_project1/config/robot_params.yaml"

    def __init__(self, config: Dict[str, Any] = None):
        """
        MasterControlGraph 초기화

        Args:
            config: 설정 딕셔너리 (None이면 기본 설정 사용)
        """
        self.config = config or self._default_config()

        # 서브그래프 인스턴스
        self._ik_graph: Optional[IKControllerGraph] = None
        self._diff_drive_graph: Optional[DifferentialDriveGraph] = None
        self._teleop_graph: Optional[TeleoperationGraph] = None

        # 그래프 핸들 저장
        self._graph_handles: Dict[str, Any] = {}

        # 생성 결과
        self._creation_results: Dict[str, bool] = {}

    @staticmethod
    def _default_config() -> Dict[str, Any]:
        """기본 설정 반환"""
        return {
            # ========================================
            # 로봇 경로
            # ========================================
            "manipulator_path": "/World/Robot/open_manipulator_x",
            "mobile_base_path": "/World/Robot/MobileBase",
            "chassis_path": "/World/Robot/MobileBase/base_link",
            "ik_target_path": "/World/IK_Target",

            # ========================================
            # IK 설정
            # ========================================
            "config_dir": "/home/isaac/ust_ws/ust_project1/config",
            "robot_desc_file": "open_x1_des.yaml",
            "urdf_file": "open_manipulator_x.urdf",
            "end_effector_frame": "end_effector_link",
            "ik_target_initial_position": [0.2, 0.0, 0.15],

            # ========================================
            # 차동 구동 설정
            # ========================================
            "wheel_radius": 0.05,
            "wheel_distance": 0.3,
            "left_wheel_joint": "left_wheel_joint",
            "right_wheel_joint": "right_wheel_joint",
            "max_linear_velocity": 1.0,
            "max_angular_velocity": 2.0,

            # ========================================
            # 그리퍼 설정
            # ========================================
            "gripper_joint_name": "gripper_left_joint",
            "gripper_min_position": -0.01,
            "gripper_max_position": 0.02,

            # ========================================
            # ROS2 설정
            # ========================================
            "namespace": "",

            # ========================================
            # 텔레오퍼레이션 설정
            # ========================================
            "coordinate_scale": 1.0,
            "coordinate_offset": [0.0, 0.0, 0.0],

            # ========================================
            # 활성화 플래그
            # ========================================
            "enable_ik": True,
            "enable_diff_drive": True,
            "enable_teleoperation": True,
            "enable_mobile_in_teleop": True,
            "enable_gripper_in_teleop": True,
        }

    @classmethod
    def from_yaml(cls, yaml_path: str = None) -> 'MasterControlGraph':
        """
        YAML 설정 파일에서 인스턴스 생성

        Args:
            yaml_path: YAML 파일 경로 (None이면 기본 경로)

        Returns:
            MasterControlGraph 인스턴스
        """
        yaml_path = yaml_path or cls.DEFAULT_CONFIG_PATH

        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                yaml_config = yaml.safe_load(f)

            # YAML 구조를 플랫 딕셔너리로 변환
            config = cls._flatten_yaml_config(yaml_config)
            print(f"[MasterControlGraph] Loaded config from {yaml_path}")
            return cls(config)

        except Exception as e:
            print(f"[MasterControlGraph] Failed to load YAML: {e}")
            print(f"[MasterControlGraph] Using default config")
            return cls()

    @staticmethod
    def _flatten_yaml_config(yaml_config: Dict) -> Dict[str, Any]:
        """YAML 설정을 플랫 딕셔너리로 변환"""
        config = {}

        # robot_paths
        if "robot_paths" in yaml_config:
            rp = yaml_config["robot_paths"]
            config["manipulator_path"] = rp.get("manipulator")
            config["mobile_base_path"] = rp.get("mobile_base")
            config["chassis_path"] = rp.get("chassis")
            config["ik_target_path"] = rp.get("ik_target")

        # ik_config
        if "ik_config" in yaml_config:
            ic = yaml_config["ik_config"]
            config["config_dir"] = ic.get("config_dir")
            config["robot_desc_file"] = ic.get("robot_description")
            config["urdf_file"] = ic.get("urdf")
            config["end_effector_frame"] = ic.get("end_effector_frame")

            if "workspace" in ic:
                ws = ic["workspace"]
                config["workspace_limits"] = ws

        # differential_drive
        if "differential_drive" in yaml_config:
            dd = yaml_config["differential_drive"]
            config["wheel_radius"] = dd.get("wheel_radius")
            config["wheel_distance"] = dd.get("wheel_distance")
            config["left_wheel_joint"] = dd.get("left_wheel_joint")
            config["right_wheel_joint"] = dd.get("right_wheel_joint")
            config["max_linear_velocity"] = dd.get("max_linear_velocity")
            config["max_angular_velocity"] = dd.get("max_angular_velocity")

        # gripper
        if "gripper" in yaml_config:
            gr = yaml_config["gripper"]
            config["gripper_joint_name"] = gr.get("joint_name")
            config["gripper_min_position"] = gr.get("min_position")
            config["gripper_max_position"] = gr.get("max_position")

        # ros2
        if "ros2" in yaml_config:
            r2 = yaml_config["ros2"]
            config["namespace"] = r2.get("namespace", "")

        # teleoperation
        if "teleoperation" in yaml_config:
            tp = yaml_config["teleoperation"]
            if "coordinate_transform" in tp:
                ct = tp["coordinate_transform"]
                config["coordinate_scale"] = ct.get("scale", 1.0)
                config["coordinate_offset"] = ct.get("offset", [0, 0, 0])

        # graphs
        if "graphs" in yaml_config:
            gs = yaml_config["graphs"]
            config["enable_ik"] = gs.get("enable_ik", True)
            config["enable_diff_drive"] = gs.get("enable_diff_drive", True)
            config["enable_teleoperation"] = gs.get("enable_teleoperation", True)

        return config

    def create_ik_graph(self) -> bool:
        """
        IK 제어 그래프 생성

        Returns:
            성공 여부
        """
        if not self.config.get("enable_ik", True):
            print("[MasterControlGraph] IK graph disabled")
            return False

        print("\n" + "=" * 50)
        print("Creating IK Controller Graph")
        print("=" * 50)

        self._ik_graph = IKControllerGraph(
            robot_prim_path=self.config.get("manipulator_path"),
            ik_target_path=self.config.get("ik_target_path"),
            config_dir=self.config.get("config_dir"),
            robot_desc_file=self.config.get("robot_desc_file"),
            urdf_file=self.config.get("urdf_file"),
            end_effector_frame=self.config.get("end_effector_frame"),
            ik_target_initial_position=self.config.get("ik_target_initial_position"),
        )

        handle = self._ik_graph.create()
        if handle:
            self._graph_handles["ik"] = handle
            self._creation_results["ik"] = True
            return True

        self._creation_results["ik"] = False
        return False

    def create_diff_drive_graph(self) -> bool:
        """
        차동 구동 그래프 생성

        Returns:
            성공 여부
        """
        if not self.config.get("enable_diff_drive", True):
            print("[MasterControlGraph] Differential drive graph disabled")
            return False

        print("\n" + "=" * 50)
        print("Creating Differential Drive Graph")
        print("=" * 50)

        self._diff_drive_graph = DifferentialDriveGraph(
            robot_prim_path=self.config.get("mobile_base_path"),
            chassis_prim_path=self.config.get("chassis_path"),
            left_wheel_joint=self.config.get("left_wheel_joint"),
            right_wheel_joint=self.config.get("right_wheel_joint"),
            wheel_radius=self.config.get("wheel_radius"),
            wheel_distance=self.config.get("wheel_distance"),
            namespace=self.config.get("namespace", ""),
            max_linear_velocity=self.config.get("max_linear_velocity"),
            max_angular_velocity=self.config.get("max_angular_velocity"),
        )

        handle = self._diff_drive_graph.create()
        if handle:
            self._graph_handles["diff_drive"] = handle
            self._creation_results["diff_drive"] = True
            return True

        self._creation_results["diff_drive"] = False
        return False

    def create_teleop_graph(self) -> bool:
        """
        텔레오퍼레이션 그래프 생성

        Returns:
            성공 여부
        """
        if not self.config.get("enable_teleoperation", True):
            print("[MasterControlGraph] Teleoperation graph disabled")
            return False

        print("\n" + "=" * 50)
        print("Creating Teleoperation Graph")
        print("=" * 50)

        self._teleop_graph = TeleoperationGraph(
            manipulator_path=self.config.get("manipulator_path"),
            mobile_base_path=self.config.get("mobile_base_path"),
            ik_target_path=self.config.get("ik_target_path"),
            wheel_radius=self.config.get("wheel_radius"),
            wheel_distance=self.config.get("wheel_distance"),
            coordinate_scale=self.config.get("coordinate_scale"),
            coordinate_offset=self.config.get("coordinate_offset"),
            gripper_joint_name=self.config.get("gripper_joint_name"),
            enable_mobile_control=self.config.get("enable_mobile_in_teleop", True),
            enable_gripper_control=self.config.get("enable_gripper_in_teleop", True),
        )

        handle = self._teleop_graph.create()
        if handle:
            self._graph_handles["teleop"] = handle
            self._creation_results["teleop"] = True
            return True

        self._creation_results["teleop"] = False
        return False

    def create_all(self) -> Dict[str, bool]:
        """
        모든 그래프 생성

        Returns:
            각 그래프의 생성 결과 딕셔너리
        """
        print("\n" + "=" * 60)
        print("OmniGraph Teleoperation System Setup")
        print("=" * 60)

        results = {
            "ik": self.create_ik_graph(),
            "diff_drive": self.create_diff_drive_graph(),
            "teleop": self.create_teleop_graph()
        }

        self._print_summary(results)

        return results

    def _print_summary(self, results: Dict[str, bool]):
        """결과 요약 출력"""
        print("\n" + "=" * 60)
        print("Setup Results Summary")
        print("=" * 60)

        total = len(results)
        success = sum(results.values())

        for name, result in results.items():
            status = "[OK]" if result else "[FAILED/DISABLED]"
            print(f"  {status} {name}")

        print("-" * 60)
        print(f"  Total: {success}/{total} graphs created")
        print("=" * 60)

    def save_to_usd(self, output_path: str = None) -> bool:
        """
        현재 스테이지를 USD 파일로 저장

        OmniGraph가 포함된 USD를 저장하면
        다음에 열 때 Play만 눌러도 자동 실행됩니다.

        Args:
            output_path: 저장 경로 (None이면 현재 스테이지 저장)

        Returns:
            성공 여부
        """
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                print("[MasterControlGraph] No stage available")
                return False

            if output_path:
                stage.GetRootLayer().Export(output_path)
                print(f"\n[MasterControlGraph] Scene saved to: {output_path}")
            else:
                stage.GetRootLayer().Save()
                print(f"\n[MasterControlGraph] Scene saved to current file")

            print("[MasterControlGraph] OmniGraph is now embedded in USD!")
            print("[MasterControlGraph] Open this USD and click Play to auto-run.")

            return True

        except Exception as e:
            print(f"[MasterControlGraph] Save failed: {e}")
            return False

    def get_graph_handles(self) -> Dict[str, Any]:
        """생성된 그래프 핸들 반환"""
        return self._graph_handles.copy()

    def get_creation_results(self) -> Dict[str, bool]:
        """생성 결과 반환"""
        return self._creation_results.copy()

    def get_config(self) -> Dict[str, Any]:
        """현재 설정 반환"""
        return self.config.copy()

    def print_next_steps(self):
        """다음 단계 안내 출력"""
        print("\n" + "=" * 60)
        print("Next Steps")
        print("=" * 60)
        print("""
1. USD 파일 저장:
   File → Save As → ust_project1_with_omnigraph.usd

2. Isaac Sim 재시작 후 USD 열기:
   File → Open → ust_project1_with_omnigraph.usd

3. Play 버튼 클릭:
   시뮬레이션 시작 → OmniGraph 자동 실행

4. ROS2 토픽 확인:
   ros2 topic list
   ros2 topic echo /joint_states
   ros2 topic echo /odom

5. 텔레오퍼레이션 테스트:
   - Quest2ROS 앱 실행
   - VR 컨트롤러로 로봇 제어

6. cmd_vel 테스트:
   ros2 topic pub /cmd_vel geometry_msgs/Twist \\
     "{linear: {x: 0.1}, angular: {z: 0.1}}"
""")
        print("=" * 60)


# =============================================================================
# Quick Setup Function
# =============================================================================

def quick_setup(
    manipulator_path: str = "/World/Robot/open_manipulator_x",
    mobile_base_path: str = "/World/Robot/MobileBase",
    output_usd: str = None,
    enable_ik: bool = True,
    enable_diff_drive: bool = True,
    enable_teleop: bool = True
) -> MasterControlGraph:
    """
    빠른 설정 함수

    기본 설정으로 모든 그래프를 빠르게 생성합니다.

    Args:
        manipulator_path: 매니퓰레이터 경로
        mobile_base_path: 모바일 베이스 경로
        output_usd: 저장할 USD 경로
        enable_ik: IK 그래프 활성화
        enable_diff_drive: 차동 구동 그래프 활성화
        enable_teleop: 텔레오퍼레이션 그래프 활성화

    Returns:
        MasterControlGraph 인스턴스

    Example:
        >>> master = quick_setup(
        ...     manipulator_path="/World/Robot/open_manipulator_x",
        ...     output_usd="/home/user/scene.usd"
        ... )
    """
    config = MasterControlGraph._default_config()
    config["manipulator_path"] = manipulator_path
    config["mobile_base_path"] = mobile_base_path
    config["enable_ik"] = enable_ik
    config["enable_diff_drive"] = enable_diff_drive
    config["enable_teleoperation"] = enable_teleop

    master = MasterControlGraph(config)
    master.create_all()

    if output_usd:
        master.save_to_usd(output_usd)

    master.print_next_steps()

    return master
