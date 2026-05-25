"""
omnigraph/teleoperation_graph.py
Meta Quest 3S 텔레오퍼레이션 그래프

Quest2ROS를 통해 VR 컨트롤러 데이터를 수신하고
로봇을 제어합니다.

컨트롤러 매핑:
    오른쪽 컨트롤러:
        - Position → IK Target XYZ
        - Rotation → IK Target Rotation (4축 로봇 제한)
        - Index Trigger → Gripper (0=Open, 1=Close)
        - Middle Trigger → Clutch (누르는 동안만 제어)
        - Button A → Home Position
        - Button B → Ready Position

    왼쪽 컨트롤러:
        - Thumbstick → Mobile Base cmd_vel
        - Index Trigger → Speed Scale
        - Button X → Emergency Stop
        - Button Y → Mode Switch

좌표계 변환:
    Quest (Unity): Y-up, Left-handed
    Isaac (USD): Z-up, Right-handed
    변환: Isaac(X, Y, Z) = Quest(X, Z, Y)

Usage:
    graph = TeleoperationGraph()
    graph.create()

Author: UST Robotics Project
Date: 2024
"""

from typing import Optional, Dict, Any, List
from .graph_builder import OmniGraphBuilder, ROS2NodeFactory, ScriptNodeGenerator


class TeleoperationGraph:
    """
    Meta Quest 3S VR 컨트롤러 텔레오퍼레이션 그래프

    Quest2ROS 토픽을 구독하여:
    - 오른쪽 컨트롤러 → 매니퓰레이터 IK 타겟
    - 왼쪽 컨트롤러 → 모바일 베이스 이동
    - 트리거 입력 → 그리퍼 제어

    그래프 구조:
        OnPlaybackTick → SubscribeRightPose → CoordinateTransform → WriteIKTarget
                       → SubscribeLeftTwist → DifferentialController → MobileCtrl
                       → GripperScript → GripperCtrl

    Quest2ROS 구독 토픽:
        - /q2r_right_hand_pose (geometry_msgs/PoseStamped)
        - /q2r_left_hand_twist (geometry_msgs/Twist)

    Attributes:
        graph_path: OmniGraph 경로
        manipulator_path: 매니퓰레이터 Articulation 경로
        mobile_base_path: 모바일 베이스 Articulation 경로
        ik_target_path: IK Target Xform 경로
    """

    # 기본 설정
    DEFAULT_GRAPH_PATH = "/World/Teleoperation_Graph"
    DEFAULT_MANIPULATOR_PATH = "/World/Robot/open_manipulator_x"
    DEFAULT_MOBILE_BASE_PATH = "/World/Robot/MobileBase"
    DEFAULT_IK_TARGET_PATH = "/World/IK_Target"

    # 기본 바퀴 파라미터
    DEFAULT_WHEEL_RADIUS = 0.05
    DEFAULT_WHEEL_DISTANCE = 0.3

    # Quest2ROS 토픽
    QUEST_TOPICS = {
        "right_hand_pose": "/q2r_right_hand_pose",
        "left_hand_pose": "/q2r_left_hand_pose",
        "right_hand_twist": "/q2r_right_hand_twist",
        "left_hand_twist": "/q2r_left_hand_twist",
        "right_hand_inputs": "/q2r_right_hand_inputs",
        "left_hand_inputs": "/q2r_left_hand_inputs",
        "right_haptic": "/q2r_right_hand_haptic_feedback",
        "left_haptic": "/q2r_left_hand_haptic_feedback",
    }

    # 좌표 변환 설정
    COORDINATE_TRANSFORM = {
        "scale": 1.0,
        "offset": [0.0, 0.0, 0.0],
    }

    # 그리퍼 설정
    GRIPPER_CONFIG = {
        "joint_name": "gripper_left_joint",
        "min_position": -0.01,  # 닫힘
        "max_position": 0.02,   # 열림
    }

    def __init__(
        self,
        graph_path: str = None,
        manipulator_path: str = None,
        mobile_base_path: str = None,
        ik_target_path: str = None,
        wheel_radius: float = None,
        wheel_distance: float = None,
        coordinate_scale: float = None,
        coordinate_offset: List[float] = None,
        gripper_joint_name: str = None,
        enable_mobile_control: bool = True,
        enable_gripper_control: bool = True,
        enable_haptic_feedback: bool = True
    ):
        """
        TeleoperationGraph 초기화

        Args:
            graph_path: OmniGraph USD 경로
            manipulator_path: 매니퓰레이터 Prim 경로
            mobile_base_path: 모바일 베이스 Prim 경로
            ik_target_path: IK Target Xform 경로
            wheel_radius: 바퀴 반경 (m)
            wheel_distance: 바퀴 간 거리 (m)
            coordinate_scale: 좌표 스케일 팩터
            coordinate_offset: 좌표 오프셋 [x, y, z]
            gripper_joint_name: 그리퍼 조인트 이름
            enable_mobile_control: 모바일 베이스 제어 활성화
            enable_gripper_control: 그리퍼 제어 활성화
            enable_haptic_feedback: 햅틱 피드백 활성화
        """
        self.graph_path = graph_path or self.DEFAULT_GRAPH_PATH
        self.manipulator_path = manipulator_path or self.DEFAULT_MANIPULATOR_PATH
        self.mobile_base_path = mobile_base_path or self.DEFAULT_MOBILE_BASE_PATH
        self.ik_target_path = ik_target_path or self.DEFAULT_IK_TARGET_PATH
        self.wheel_radius = wheel_radius or self.DEFAULT_WHEEL_RADIUS
        self.wheel_distance = wheel_distance or self.DEFAULT_WHEEL_DISTANCE

        # 좌표 변환 설정
        self.coordinate_scale = coordinate_scale or self.COORDINATE_TRANSFORM["scale"]
        self.coordinate_offset = coordinate_offset or self.COORDINATE_TRANSFORM["offset"]

        # 그리퍼 설정
        self.gripper_joint_name = gripper_joint_name or self.GRIPPER_CONFIG["joint_name"]

        # 기능 활성화 플래그
        self.enable_mobile_control = enable_mobile_control
        self.enable_gripper_control = enable_gripper_control
        self.enable_haptic_feedback = enable_haptic_feedback

        self._graph_handle = None
        self._builder: Optional[OmniGraphBuilder] = None

    def create(self) -> Optional[Any]:
        """
        텔레오퍼레이션 그래프 생성

        Returns:
            그래프 핸들 (실패 시 None)
        """
        print(f"[TeleoperationGraph] Creating graph at {self.graph_path}")

        self._builder = OmniGraphBuilder(self.graph_path)
        self._builder.remove_existing()

        # 노드 추가
        self._add_nodes()

        # 값 설정
        self._set_values()

        # 연결
        self._create_connections()

        # 빌드
        self._graph_handle = self._builder.build()

        # Script Node 설정
        if self._graph_handle:
            self._setup_script_nodes()
            print(f"[TeleoperationGraph] Graph created successfully")
            self._print_topic_info()
        else:
            print(f"[TeleoperationGraph] Graph creation failed")

        return self._graph_handle

    def _add_nodes(self):
        """그래프 노드 추가 (Isaac Sim 버전 자동 감지)"""
        # Note: Isaac Sim 4.5.0+에서 ROS2SubscribePoseStamped가 없을 수 있음
        # Script Node를 사용하여 ROS2 토픽을 직접 구독하는 방식으로 처리
        nodes = [
            # 실행 트리거
            ("OnPlaybackTick", ROS2NodeFactory.get_node_type("on_playback_tick")),

            # ROS2 컨텍스트 (버전에 따라 자동 선택)
            ("ROS2Context", ROS2NodeFactory.get_node_type("context")),

            # === 매니퓰레이터 제어 (오른쪽 컨트롤러) ===
            # Script Node에서 rclpy로 직접 Pose 구독 및 좌표 변환
            ("TeleoperationScript", ROS2NodeFactory.get_node_type("script_node")),
        ]

        # 모바일 베이스 제어
        # Note: Isaac Sim 4.5.0+에서 DifferentialController는 데이터 노드로
        # execOut이 없고, linearVelocity/angularVelocity가 double (스칼라)
        # SubscribeTwist.outputs는 double3 (벡터)이므로 Script Node로 대체
        if self.enable_mobile_control:
            nodes.extend([
                # Script Node로 Twist 구독 및 바퀴 속도 계산 통합
                ("MobileBaseScript", ROS2NodeFactory.get_node_type("script_node")),
            ])

        # 그리퍼 제어 (Script Node에 통합)
        # if self.enable_gripper_control:
        #     nodes.extend([
        #         ("GripperScript", ROS2NodeFactory.get_node_type("script_node")),
        #         ("GripperArticulationCtrl", ROS2NodeFactory.get_node_type("articulation_controller")),
        #     ])

        self._builder.add_nodes(nodes)

    def _set_values(self):
        """노드 속성 값 설정"""
        values = [
            # ROS2 Context
            ("ROS2Context.inputs:useDomainIDEnvVar", True),
            # TeleoperationScript는 별도 설정 불필요 (스크립트 내부에서 처리)
        ]

        # 모바일 베이스 설정
        # MobileBaseScript는 스크립트 내부에서 파라미터 처리
        # (DifferentialController 대체로 인해 별도 값 설정 불필요)

        # 그리퍼 설정 (Script Node에 통합됨)
        # if self.enable_gripper_control:
        #     values.extend([
        #         ("GripperArticulationCtrl.inputs:targetPrim", self.manipulator_path),
        #         ("GripperArticulationCtrl.inputs:jointNames", [self.gripper_joint_name]),
        #     ])

        self._builder.set_values(values)

    def _create_connections(self):
        """노드 연결"""
        # TeleoperationScript는 독립적으로 실행 (내부에서 rclpy로 토픽 구독)
        connections = [
            # 실행 흐름: TeleoperationScript 실행
            ("OnPlaybackTick.outputs:tick", "TeleoperationScript.inputs:execIn"),
        ]

        # 모바일 베이스 연결
        # MobileBaseScript는 내부에서 rclpy로 Twist 구독하고 직접 로봇 제어
        if self.enable_mobile_control:
            connections.extend([
                ("OnPlaybackTick.outputs:tick", "MobileBaseScript.inputs:execIn"),
            ])

        # 그리퍼 연결 (Script Node에 통합됨)
        # if self.enable_gripper_control:
        #     connections.extend([
        #         ("OnPlaybackTick.outputs:tick", "GripperScript.inputs:execIn"),
        #         ("GripperScript.outputs:execOut", "GripperArticulationCtrl.inputs:execIn"),
        #     ])

        self._builder.connect_many(connections)

    def _setup_script_nodes(self):
        """Script Node 설정"""
        try:
            import omni.graph.core as og

            # 텔레오퍼레이션 스크립트 (Pose 구독 + 좌표 변환 + IK Target 업데이트 통합)
            teleop_script = self._generate_teleoperation_script()
            try:
                og.Controller.attribute(
                    f"{self.graph_path}/TeleoperationScript.inputs:script"
                ).set(teleop_script)
                print("[TeleoperationGraph] TeleoperationScript configured")
            except Exception as e:
                print(f"[TeleoperationGraph] Could not set TeleoperationScript: {e}")

            # 모바일 베이스 스크립트 (Twist 구독 + 바퀴 속도 계산)
            if self.enable_mobile_control:
                mobile_script = self._generate_mobile_base_script()
                try:
                    og.Controller.attribute(
                        f"{self.graph_path}/MobileBaseScript.inputs:script"
                    ).set(mobile_script)
                    print("[TeleoperationGraph] MobileBaseScript configured")
                except Exception as e:
                    print(f"[TeleoperationGraph] Could not set MobileBaseScript: {e}")

        except Exception as e:
            print(f"[TeleoperationGraph] Script setup warning: {e}")

    def _generate_coordinate_transform_script(self) -> str:
        """좌표 변환 스크립트 생성"""
        return f'''"""
Quest to Isaac Coordinate Transform Script
Quest (Unity): Y-up, Left-handed
Isaac (USD): Z-up, Right-handed

Transform: Isaac(X, Y, Z) = Quest(X, Z, Y)
"""
import numpy as np
import omni.usd
from pxr import UsdGeom, Gf

SCALE = {self.coordinate_scale}
OFFSET = np.array({self.coordinate_offset})
IK_TARGET_PATH = "{self.ik_target_path}"

# State
_last_valid_position = None

def compute(db):
    """Transform Quest coordinates and write to IK target"""
    global _last_valid_position

    try:
        # Get Quest position from PoseStamped
        quest_pos = db.inputs.position
        if quest_pos is None or len(quest_pos) < 3:
            return

        # Coordinate transformation
        # Quest Y-up -> Isaac Z-up
        isaac_pos = np.array([
            quest_pos[0],    # X stays same
            quest_pos[2],    # Quest Z -> Isaac Y
            quest_pos[1]     # Quest Y -> Isaac Z
        ])

        # Apply scale and offset
        isaac_pos = isaac_pos * SCALE + OFFSET

        # Write to IK target
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(IK_TARGET_PATH)
        if prim.IsValid():
            xformable = UsdGeom.Xformable(prim)

            # Get or create translate op
            ops = xformable.GetOrderedXformOps()
            translate_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break

            if translate_op is None:
                translate_op = xformable.AddTranslateOp()

            translate_op.Set(Gf.Vec3d(*isaac_pos))
            _last_valid_position = isaac_pos

        db.outputs.transformedPosition = isaac_pos.tolist()

    except Exception as e:
        pass
'''

    def _generate_gripper_script(self) -> str:
        """그리퍼 제어 스크립트 생성"""
        min_pos = self.GRIPPER_CONFIG["min_position"]
        max_pos = self.GRIPPER_CONFIG["max_position"]

        return f'''"""
Gripper Control Script for Quest2ROS

Maps Index Trigger (0.0 ~ 1.0) to gripper position
Trigger 0.0 (released) -> Gripper open ({max_pos})
Trigger 1.0 (pressed)  -> Gripper closed ({min_pos})
"""
import numpy as np

MIN_POS = {min_pos}
MAX_POS = {max_pos}

# ROS2 subscriber for trigger input
_trigger_value = 0.0

def compute(db):
    """Map trigger to gripper position"""
    global _trigger_value

    try:
        # Get trigger value
        # Note: In actual implementation, this would come from
        # /q2r_right_hand_inputs topic
        trigger = getattr(db.inputs, 'triggerValue', _trigger_value)
        if trigger is None:
            trigger = 0.0

        trigger = max(0.0, min(1.0, float(trigger)))

        # Map trigger to gripper position
        gripper_pos = MAX_POS - (trigger * (MAX_POS - MIN_POS))

        db.outputs.positionCommand = np.array([gripper_pos])

    except Exception:
        db.outputs.positionCommand = np.array([MAX_POS])
'''

    def _generate_teleoperation_script(self) -> str:
        """
        통합 텔레오퍼레이션 스크립트 생성

        ROS2 토픽을 직접 구독하고 IK Target을 업데이트합니다.
        Isaac Sim 4.5.0+에서 ROS2SubscribePoseStamped가 없을 수 있으므로
        Script Node 내에서 rclpy를 사용하여 직접 구독합니다.
        """
        return f'''"""
Teleoperation Script for Quest2ROS
Subscribes to PoseStamped topic and updates IK Target

Topic: {self.QUEST_TOPICS["right_hand_pose"]}
Target: {self.ik_target_path}
"""
import numpy as np
import omni.usd
from pxr import UsdGeom, Gf

# Configuration
SCALE = {self.coordinate_scale}
OFFSET = np.array({self.coordinate_offset})
IK_TARGET_PATH = "{self.ik_target_path}"
POSE_TOPIC = "{self.QUEST_TOPICS["right_hand_pose"]}"

# Global state
_initialized = False
_ros_node = None
_last_position = None
_subscription = None

def setup(db):
    """Initialize ROS2 subscriber"""
    global _initialized, _ros_node, _subscription

    if _initialized:
        return True

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped

        # Initialize rclpy if not already
        if not rclpy.ok():
            rclpy.init()

        class PoseSubscriber(Node):
            def __init__(self):
                super().__init__('isaac_teleop_subscriber')
                self.position = None
                self.subscription = self.create_subscription(
                    PoseStamped,
                    POSE_TOPIC,
                    self.pose_callback,
                    10
                )

            def pose_callback(self, msg):
                # Quest (Y-up) to Isaac (Z-up) coordinate transform
                quest_x = msg.pose.position.x
                quest_y = msg.pose.position.y
                quest_z = msg.pose.position.z

                # Transform: Isaac(X, Y, Z) = Quest(X, Z, Y)
                isaac_pos = np.array([quest_x, quest_z, quest_y])
                self.position = isaac_pos * SCALE + OFFSET

        _ros_node = PoseSubscriber()
        _initialized = True
        print("[TeleoperationScript] ROS2 subscriber initialized")
        return True

    except ImportError:
        print("[TeleoperationScript] rclpy not available - using fallback mode")
        _initialized = True
        return True
    except Exception as e:
        print(f"[TeleoperationScript] Setup error: {{e}}")
        return False


def update_ik_target(position):
    """Update IK Target Xform position"""
    try:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False

        prim = stage.GetPrimAtPath(IK_TARGET_PATH)
        if not prim.IsValid():
            return False

        xformable = UsdGeom.Xformable(prim)

        # Find or create translate op
        xform_ops = xformable.GetOrderedXformOps()
        translate_op = None
        for op in xform_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                break

        if translate_op is None:
            translate_op = xformable.AddTranslateOp()

        translate_op.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        return True

    except Exception as e:
        return False


def compute(db):
    """Main compute function - called every tick"""
    global _initialized, _ros_node, _last_position

    if not _initialized:
        if not setup(db):
            return

    try:
        # Spin ROS2 node to process callbacks
        if _ros_node is not None:
            import rclpy
            rclpy.spin_once(_ros_node, timeout_sec=0.001)

            if _ros_node.position is not None:
                _last_position = _ros_node.position.copy()

        # Update IK target if we have a position
        if _last_position is not None:
            update_ik_target(_last_position)

    except Exception as e:
        pass
'''

    def _generate_mobile_base_script(self) -> str:
        """
        모바일 베이스 제어 스크립트 생성

        Quest2ROS 왼손 Twist를 구독하여 바퀴 속도로 변환
        """
        twist_topic = self.QUEST_TOPICS["left_hand_twist"]

        return f'''"""
Mobile Base Control Script for Quest2ROS Teleoperation
Subscribes to Twist topic and controls differential drive robot

Topic: {twist_topic}
Robot: {self.mobile_base_path}
Wheel Radius: {self.wheel_radius} m
Wheel Distance: {self.wheel_distance} m
"""
import numpy as np

# Robot parameters
WHEEL_RADIUS = {self.wheel_radius}
WHEEL_DISTANCE = {self.wheel_distance}
MAX_LINEAR = 1.0  # m/s
MAX_ANGULAR = 2.0  # rad/s
TWIST_TOPIC = "{twist_topic}"
ROBOT_PRIM_PATH = "{self.mobile_base_path}"

# Global state
_initialized = False
_robot = None
_twist_node = None


def setup(db):
    """Initialize robot articulation"""
    global _initialized, _robot

    if _initialized:
        return True

    try:
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation

        world = World.instance()
        if world is None:
            return False

        _robot = world.scene.get_object("teleop_mobile_robot")
        if _robot is None:
            _robot = world.scene.add(
                Articulation(prim_path=ROBOT_PRIM_PATH, name="teleop_mobile_robot")
            )
            world.reset()

        _initialized = True
        print("[MobileBaseScript] Initialized")
        return True

    except Exception as e:
        print(f"[MobileBaseScript] Setup error: {{e}}")
        return False


def compute(db):
    """
    Subscribe to Twist and apply wheel velocities

    Differential drive kinematics:
        ωR = (v + ω * L/2) / r
        ωL = (v - ω * L/2) / r
    """
    global _initialized, _robot, _twist_node

    if not _initialized:
        if not setup(db):
            return

    if _robot is None:
        return

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist

        if not rclpy.ok():
            return

        # Create subscriber if not exists
        if _twist_node is None:
            class TwistSubscriber(Node):
                def __init__(self):
                    super().__init__('teleop_mobile_twist_sub')
                    self.linear_x = 0.0
                    self.angular_z = 0.0
                    self.subscription = self.create_subscription(
                        Twist,
                        TWIST_TOPIC,
                        self.twist_callback,
                        10
                    )

                def twist_callback(self, msg):
                    self.linear_x = msg.linear.x
                    self.angular_z = msg.angular.z

            _twist_node = TwistSubscriber()

        # Spin once
        rclpy.spin_once(_twist_node, timeout_sec=0.001)

        # Get velocities
        linear_vel = max(-MAX_LINEAR, min(MAX_LINEAR, _twist_node.linear_x))
        angular_vel = max(-MAX_ANGULAR, min(MAX_ANGULAR, _twist_node.angular_z))

        # Differential drive kinematics
        half_dist = WHEEL_DISTANCE / 2.0
        right_wheel_vel = (linear_vel + angular_vel * half_dist) / WHEEL_RADIUS
        left_wheel_vel = (linear_vel - angular_vel * half_dist) / WHEEL_RADIUS

        # Apply velocity
        from omni.isaac.core.utils.types import ArticulationAction
        action = ArticulationAction(
            joint_velocities=np.array([left_wheel_vel, right_wheel_vel])
        )
        _robot.apply_action(action)

    except ImportError:
        pass
    except Exception as e:
        pass
'''

    def _print_topic_info(self):
        """토픽 정보 출력"""
        print(f"[TeleoperationGraph] Quest2ROS Topics:")
        print(f"  Subscribe: {self.QUEST_TOPICS['right_hand_pose']} (Manipulator)")
        if self.enable_mobile_control:
            print(f"  Subscribe: {self.QUEST_TOPICS['left_hand_twist']} (Mobile Base)")

    def get_graph_handle(self):
        """그래프 핸들 반환"""
        return self._graph_handle

    def get_config(self) -> Dict[str, Any]:
        """현재 설정 반환"""
        return {
            "graph_path": self.graph_path,
            "manipulator_path": self.manipulator_path,
            "mobile_base_path": self.mobile_base_path,
            "ik_target_path": self.ik_target_path,
            "wheel_radius": self.wheel_radius,
            "wheel_distance": self.wheel_distance,
            "coordinate_scale": self.coordinate_scale,
            "coordinate_offset": self.coordinate_offset,
            "gripper_joint_name": self.gripper_joint_name,
            "enable_mobile_control": self.enable_mobile_control,
            "enable_gripper_control": self.enable_gripper_control,
            "enable_haptic_feedback": self.enable_haptic_feedback,
            "quest_topics": self.QUEST_TOPICS,
        }

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> 'TeleoperationGraph':
        """설정 딕셔너리에서 인스턴스 생성"""
        return cls(
            graph_path=config.get("graph_path"),
            manipulator_path=config.get("manipulator_path"),
            mobile_base_path=config.get("mobile_base_path"),
            ik_target_path=config.get("ik_target_path"),
            wheel_radius=config.get("wheel_radius"),
            wheel_distance=config.get("wheel_distance"),
            coordinate_scale=config.get("coordinate_scale"),
            coordinate_offset=config.get("coordinate_offset"),
            gripper_joint_name=config.get("gripper_joint_name"),
            enable_mobile_control=config.get("enable_mobile_control", True),
            enable_gripper_control=config.get("enable_gripper_control", True),
            enable_haptic_feedback=config.get("enable_haptic_feedback", True),
        )

    def __repr__(self) -> str:
        return (
            f"TeleoperationGraph("
            f"path='{self.graph_path}', "
            f"manipulator='{self.manipulator_path}', "
            f"mobile='{self.mobile_base_path}')"
        )


# =============================================================================
# Standalone Manipulator Teleop Graph
# =============================================================================

class ManipulatorTeleoperationGraph:
    """
    매니퓰레이터 전용 텔레오퍼레이션 그래프

    모바일 베이스 없이 매니퓰레이터만 제어합니다.
    """

    def __init__(
        self,
        graph_path: str = "/World/Manipulator_Teleop_Graph",
        manipulator_path: str = "/World/Robot/open_manipulator_x",
        ik_target_path: str = "/World/IK_Target",
        enable_gripper: bool = True
    ):
        self.graph_path = graph_path
        self.manipulator_path = manipulator_path
        self.ik_target_path = ik_target_path
        self.enable_gripper = enable_gripper
        self._graph_handle = None

    def create(self) -> Optional[Any]:
        """그래프 생성 (Isaac Sim 버전 자동 감지)"""
        builder = OmniGraphBuilder(self.graph_path)
        builder.remove_existing()

        # 노드 추가 (버전에 따라 자동 선택)
        nodes = [
            ("OnPlaybackTick", ROS2NodeFactory.get_node_type("on_playback_tick")),
            ("ROS2Context", ROS2NodeFactory.get_node_type("context")),
            ("SubscribePose", ROS2NodeFactory.get_node_type("subscribe_pose_stamped")),
            ("TransformScript", ROS2NodeFactory.get_node_type("script_node")),
            ("WriteTarget", ROS2NodeFactory.get_node_type("write_prim_attribute")),
        ]
        builder.add_nodes(nodes)

        # 값 설정
        builder.set_values([
            ("ROS2Context.inputs:useDomainIDEnvVar", True),
            ("SubscribePose.inputs:topicName", "/q2r_right_hand_pose"),
            ("WriteTarget.inputs:primPath", self.ik_target_path),
            ("WriteTarget.inputs:name", "xformOp:translate"),
        ])

        # 연결
        builder.connect_many([
            ("OnPlaybackTick.outputs:tick", "SubscribePose.inputs:execIn"),
            ("SubscribePose.outputs:execOut", "TransformScript.inputs:execIn"),
            ("TransformScript.outputs:execOut", "WriteTarget.inputs:execIn"),
        ])

        self._graph_handle = builder.build()
        return self._graph_handle
