"""
omnigraph/ik_graph.py
IK Target 추적을 위한 OmniGraph 구성

OpenMANIPULATOR-X 역운동학 제어 그래프를 생성합니다.
IK Target Xform의 위치를 읽어 Lula IK Solver로 조인트 위치를 계산하고
로봇에 적용합니다.

Usage:
    ik_graph = IKControllerGraph()
    ik_graph.create()

Author: UST Robotics Project
Date: 2024
"""

from typing import Optional, Dict, Any, List
from .graph_builder import OmniGraphBuilder, ROS2NodeFactory, ScriptNodeGenerator


class IKControllerGraph:
    """
    OpenMANIPULATOR-X IK 제어 그래프

    IK Target Xform을 읽어 역운동학 계산 후 조인트 제어합니다.
    Script Node를 사용하여 Lula IK Solver를 호출합니다.

    그래프 구조:
        OnPlaybackTick → ReadTargetTransform → IKSolverScript → ArticulationController

    Attributes:
        graph_path: OmniGraph 경로
        robot_prim_path: 로봇 Articulation 경로
        ik_target_path: IK Target Xform 경로
        config_dir: IK 설정 파일 디렉토리

    Example:
        >>> ik_graph = IKControllerGraph(
        ...     robot_prim_path="/World/Robot/open_manipulator_x",
        ...     ik_target_path="/World/IK_Target"
        ... )
        >>> ik_graph.create()
    """

    # 기본 설정
    DEFAULT_GRAPH_PATH = "/World/IK_Controller_Graph"
    DEFAULT_ROBOT_PATH = "/World/Robot/open_manipulator_x"
    DEFAULT_IK_TARGET_PATH = "/World/IK_Target"
    DEFAULT_CONFIG_DIR = "/home/isaac/ust_ws/ust_project1/config"

    # IK 설정 파일
    DEFAULT_ROBOT_DESC = "open_x1_des.yaml"
    DEFAULT_URDF = "open_manipulator_x.urdf"
    DEFAULT_EE_FRAME = "end_effector_link"

    # 작업 공간 제한 (미터)
    WORKSPACE_LIMITS = {
        "min_radius": 0.08,
        "max_radius": 0.38,
        "min_height": -0.05,
        "max_height": 0.40,
    }

    # IK Target 초기 위치
    DEFAULT_IK_TARGET_POSITION = [0.2, 0.0, 0.15]

    def __init__(
        self,
        graph_path: str = None,
        robot_prim_path: str = None,
        ik_target_path: str = None,
        config_dir: str = None,
        robot_desc_file: str = None,
        urdf_file: str = None,
        end_effector_frame: str = None,
        ik_target_initial_position: List[float] = None
    ):
        """
        IKControllerGraph 초기화

        Args:
            graph_path: OmniGraph USD 경로
            robot_prim_path: 로봇 Articulation Prim 경로
            ik_target_path: IK Target Xform 경로
            config_dir: 설정 파일 디렉토리
            robot_desc_file: Lula robot description yaml 파일명
            urdf_file: URDF 파일명
            end_effector_frame: 엔드이펙터 프레임 이름
            ik_target_initial_position: IK Target 초기 위치 [x, y, z]
        """
        self.graph_path = graph_path or self.DEFAULT_GRAPH_PATH
        self.robot_prim_path = robot_prim_path or self.DEFAULT_ROBOT_PATH
        self.ik_target_path = ik_target_path or self.DEFAULT_IK_TARGET_PATH
        self.config_dir = config_dir or self.DEFAULT_CONFIG_DIR
        self.robot_desc_file = robot_desc_file or self.DEFAULT_ROBOT_DESC
        self.urdf_file = urdf_file or self.DEFAULT_URDF
        self.end_effector_frame = end_effector_frame or self.DEFAULT_EE_FRAME
        self.ik_target_initial_position = (
            ik_target_initial_position or self.DEFAULT_IK_TARGET_POSITION
        )

        self._graph_handle = None
        self._builder: Optional[OmniGraphBuilder] = None

    def create(self) -> Optional[Any]:
        """
        IK 제어 그래프 생성

        IK Target Xform을 생성하고 OmniGraph를 구성합니다.

        Returns:
            그래프 핸들 (실패 시 None)
        """
        print(f"[IKControllerGraph] Creating graph at {self.graph_path}")

        # IK Target Xform 생성
        self._create_ik_target()

        # 그래프 빌더 초기화
        self._builder = OmniGraphBuilder(self.graph_path)
        self._builder.remove_existing()

        # 노드 추가
        self._add_nodes()

        # 값 설정
        self._set_values()

        # 연결
        self._create_connections()

        # 그래프 빌드
        self._graph_handle = self._builder.build()

        # Script Node 내용 설정
        if self._graph_handle:
            self._setup_script_node()
            print(f"[IKControllerGraph] Graph created successfully")
        else:
            print(f"[IKControllerGraph] Graph creation failed")

        return self._graph_handle

    def _create_ik_target(self) -> bool:
        """IK Target Xform 생성"""
        return OmniGraphBuilder.create_xform_target(
            self.ik_target_path,
            position=self.ik_target_initial_position
        )

    def _add_nodes(self):
        """그래프 노드 추가"""
        # Note: ReadPrimAttribute는 실행 흐름 노드가 아니므로 execIn/execOut이 없음
        # IK Target 위치는 Script Node 내에서 직접 USD API로 읽음
        self._builder.add_nodes([
            # 실행 트리거
            ("OnPlaybackTick", ROS2NodeFactory.get_node_type("on_playback_tick")),

            # IK 계산 Script Node (IK Target 위치를 내부에서 직접 읽음)
            ("IKSolverScript", ROS2NodeFactory.get_node_type("script_node")),

            # Articulation Controller (버전에 따라 자동 선택)
            ("ArticulationController", ROS2NodeFactory.get_node_type("articulation_controller")),
        ])

    def _set_values(self):
        """노드 속성 값 설정"""
        # Isaac Sim 4.5.0+에서는 usePath 속성이 없음
        # targetPrim을 사용하거나 robotPath만 설정
        self._builder.set_values([
            # Articulation Controller 설정
            # Isaac Sim 4.5.0+: targetPrim 사용
            # Isaac Sim 4.2 이하: robotPath + usePath 사용
            ("ArticulationController.inputs:targetPrim", self.robot_prim_path),
        ])

    def _create_connections(self):
        """노드 연결"""
        self._builder.connect_many([
            # 실행 흐름 (ReadPrimAttribute 제거 - Script Node에서 직접 USD API로 읽음)
            ("OnPlaybackTick.outputs:tick", "IKSolverScript.inputs:execIn"),
            ("IKSolverScript.outputs:execOut", "ArticulationController.inputs:execIn"),
        ])

    def _setup_script_node(self):
        """Script Node에 IK 계산 스크립트 설정"""
        script_content = self._generate_ik_script()

        try:
            import omni.graph.core as og

            script_node_path = f"{self.graph_path}/IKSolverScript"

            # Script Node 입력 속성 설정
            # Note: Script Node의 스크립트 설정은 버전에 따라 다를 수 있음
            try:
                og.Controller.attribute(f"{script_node_path}.inputs:script").set(script_content)
            except Exception:
                # 대안: 노드 내 usePython 또는 다른 속성 시도
                pass

            print(f"[IKControllerGraph] Script node configured")

        except Exception as e:
            print(f"[IKControllerGraph] Script setup warning: {e}")

    def _generate_ik_script(self) -> str:
        """IK 계산 스크립트 생성"""
        return f'''"""
IK Solver Script Node - Auto Generated
Robot: {self.robot_prim_path}
End Effector: {self.end_effector_frame}
Config: {self.config_dir}
"""
import numpy as np
import omni.usd
from pxr import UsdGeom, Gf

# Global state
_initialized = False
_ik_solver = None
_art_solver = None
_robot = None
_ik_target_path = "{self.ik_target_path}"

def get_ik_target_position():
    """IK Target Xform에서 위치 읽기"""
    try:
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(_ik_target_path)
        if prim.IsValid():
            xformable = UsdGeom.Xformable(prim)
            transform = xformable.ComputeLocalToWorldTransform(0)
            translation = transform.ExtractTranslation()
            return np.array([translation[0], translation[1], translation[2]])
    except Exception:
        pass
    return None

def setup(db):
    """Initialize IK solver"""
    global _initialized, _ik_solver, _art_solver, _robot

    if _initialized:
        return True

    try:
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.motion_generation import (
            ArticulationKinematicsSolver,
            LulaKinematicsSolver
        )

        config_dir = "{self.config_dir}"
        robot_path = "{self.robot_prim_path}"
        ee_frame = "{self.end_effector_frame}"

        world = World.instance()
        if world is None:
            return False

        # Get or add robot
        _robot = world.scene.get_object("ik_controlled_robot")
        if _robot is None:
            _robot = world.scene.add(
                Articulation(prim_path=robot_path, name="ik_controlled_robot")
            )
            world.reset()

        # Initialize Lula solver
        _ik_solver = LulaKinematicsSolver(
            robot_description_path=f"{{config_dir}}/{self.robot_desc_file}",
            urdf_path=f"{{config_dir}}/{self.urdf_file}"
        )

        _art_solver = ArticulationKinematicsSolver(
            _robot, _ik_solver, ee_frame
        )

        _initialized = True
        print("[IK Script] Initialized")
        return True

    except Exception as e:
        print(f"[IK Script] Setup error: {{e}}")
        return False

def compute(db):
    """Compute IK and apply"""
    global _initialized, _art_solver, _robot, _ik_solver

    if not _initialized:
        if not setup(db):
            return

    if _art_solver is None or _robot is None:
        return

    try:
        # Get target position from Xform
        target = get_ik_target_position()
        if target is None:
            return

        # Workspace check
        robot_base_pos, robot_base_ori = _robot.get_world_pose()
        dx = target[0] - robot_base_pos[0]
        dy = target[1] - robot_base_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        height = target[2] - robot_base_pos[2]

        # Workspace limits
        if dist < {self.WORKSPACE_LIMITS["min_radius"]} or dist > {self.WORKSPACE_LIMITS["max_radius"]}:
            return
        if height < {self.WORKSPACE_LIMITS["min_height"]} or height > {self.WORKSPACE_LIMITS["max_height"]}:
            return

        # Update robot base pose
        _ik_solver.set_robot_base_pose(robot_base_pos, robot_base_ori)

        # Compute IK
        action, success = _art_solver.compute_inverse_kinematics(
            target, target_orientation=None
        )

        if success and action is not None:
            _robot.apply_action(action)

    except Exception:
        pass
'''

    def get_graph_handle(self):
        """그래프 핸들 반환"""
        return self._graph_handle

    def get_config(self) -> Dict[str, Any]:
        """현재 설정 반환"""
        return {
            "graph_path": self.graph_path,
            "robot_prim_path": self.robot_prim_path,
            "ik_target_path": self.ik_target_path,
            "config_dir": self.config_dir,
            "robot_desc_file": self.robot_desc_file,
            "urdf_file": self.urdf_file,
            "end_effector_frame": self.end_effector_frame,
            "ik_target_initial_position": self.ik_target_initial_position,
            "workspace_limits": self.WORKSPACE_LIMITS,
        }

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> 'IKControllerGraph':
        """설정 딕셔너리에서 인스턴스 생성"""
        return cls(
            graph_path=config.get("graph_path"),
            robot_prim_path=config.get("robot_prim_path"),
            ik_target_path=config.get("ik_target_path"),
            config_dir=config.get("config_dir"),
            robot_desc_file=config.get("robot_desc_file"),
            urdf_file=config.get("urdf_file"),
            end_effector_frame=config.get("end_effector_frame"),
            ik_target_initial_position=config.get("ik_target_initial_position"),
        )

    def __repr__(self) -> str:
        return (
            f"IKControllerGraph("
            f"graph_path='{self.graph_path}', "
            f"robot='{self.robot_prim_path}', "
            f"target='{self.ik_target_path}')"
        )


# =============================================================================
# Alternative: Simple IK Graph (without Script Node)
# =============================================================================

class SimpleIKGraph:
    """
    간단한 IK 그래프 (Script Node 없이)

    외부 Python 스크립트로 IK 계산을 수행하고
    이 그래프는 ArticulationController만 제공합니다.
    """

    def __init__(
        self,
        graph_path: str = "/World/Simple_IK_Graph",
        robot_prim_path: str = "/World/Robot/open_manipulator_x",
        namespace: str = ""
    ):
        self.graph_path = graph_path
        self.robot_prim_path = robot_prim_path
        self.namespace = namespace
        self._graph_handle = None

    def create(self) -> Optional[Any]:
        """그래프 생성"""
        builder = OmniGraphBuilder(self.graph_path)
        builder.remove_existing()

        ns = self.namespace

        # 노드 추가
        builder.add_nodes([
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", ROS2NodeFactory.get_node_type("context")),
            ("SubscribeJointState", ROS2NodeFactory.get_node_type("subscribe_joint_state")),
            ("ArticulationController", ROS2NodeFactory.get_node_type("articulation_controller")),
            ("PublishJointState", ROS2NodeFactory.get_node_type("publish_joint_state")),
        ])

        # 값 설정
        builder.set_values([
            ("ROS2Context.inputs:useDomainIDEnvVar", True),
            ("SubscribeJointState.inputs:topicName", f"{ns}/joint_command"),
            ("ArticulationController.inputs:robotPath", self.robot_prim_path),
            ("ArticulationController.inputs:usePath", True),
            ("PublishJointState.inputs:topicName", f"{ns}/joint_states"),
            ("PublishJointState.inputs:targetPrim", self.robot_prim_path),
        ])

        # 연결
        builder.connect_many([
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ])

        self._graph_handle = builder.build()
        return self._graph_handle
