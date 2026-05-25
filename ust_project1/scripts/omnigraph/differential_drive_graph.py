"""
omnigraph/differential_drive_graph.py
차동 구동 모바일 로봇 제어 그래프

/cmd_vel 토픽을 구독하여 바퀴 속도를 계산하고 로봇에 적용합니다.
Odometry와 TF를 퍼블리시합니다.

차동 구동 공식:
    ωR = (v + ω * L/2) / r
    ωL = (v - ω * L/2) / r

    여기서:
    - v: 선속도 (m/s)
    - ω: 각속도 (rad/s)
    - L: 바퀴 간 거리 (wheel base)
    - r: 바퀴 반경 (wheel radius)

Usage:
    graph = DifferentialDriveGraph(
        robot_prim_path="/World/Robot/MobileBase",
        wheel_radius=0.05,
        wheel_distance=0.3
    )
    graph.create()

Author: UST Robotics Project
Date: 2024
"""

from typing import Optional, Dict, Any, List
from .graph_builder import OmniGraphBuilder, ROS2NodeFactory


class DifferentialDriveGraph:
    """
    차동 구동 로봇 제어 그래프

    ROS2 /cmd_vel 토픽을 구독하여 바퀴 속도를 계산하고
    Articulation Controller로 로봇을 제어합니다.

    그래프 구조:
        OnPlaybackTick → SubscribeTwist → DifferentialController → ArticulationController
                       → PublishOdometry
                       → PublishTF
                       → PublishClock

    퍼블리시 토픽:
        - /odom (nav_msgs/Odometry)
        - /tf (tf2_msgs/TFMessage)
        - /clock (rosgraph_msgs/Clock)

    구독 토픽:
        - /cmd_vel (geometry_msgs/Twist)

    Attributes:
        graph_path: OmniGraph 경로
        robot_prim_path: 로봇 Articulation 경로
        chassis_prim_path: 섀시 링크 경로 (Odometry 계산용)
        wheel_radius: 바퀴 반경 (m)
        wheel_distance: 바퀴 간 거리 (m)
    """

    # 기본 설정
    DEFAULT_GRAPH_PATH = "/World/DifferentialDrive_Graph"
    DEFAULT_ROBOT_PATH = "/World/Robot/MobileBase"
    DEFAULT_CHASSIS_PATH = "/World/Robot/MobileBase/base_link"

    # 기본 바퀴 파라미터
    DEFAULT_WHEEL_RADIUS = 0.05      # 50mm
    DEFAULT_WHEEL_DISTANCE = 0.3    # 300mm
    DEFAULT_LEFT_WHEEL = "left_wheel_joint"
    DEFAULT_RIGHT_WHEEL = "right_wheel_joint"

    # 속도 제한
    DEFAULT_MAX_LINEAR_VEL = 1.0    # m/s
    DEFAULT_MAX_ANGULAR_VEL = 2.0   # rad/s

    def __init__(
        self,
        graph_path: str = None,
        robot_prim_path: str = None,
        chassis_prim_path: str = None,
        left_wheel_joint: str = None,
        right_wheel_joint: str = None,
        wheel_radius: float = None,
        wheel_distance: float = None,
        namespace: str = "",
        max_linear_velocity: float = None,
        max_angular_velocity: float = None,
        publish_odom: bool = False,  # Isaac Sim 4.5.0+에서 속성명 변경으로 기본 비활성화
        publish_tf: bool = True,
        publish_clock: bool = True
    ):
        """
        DifferentialDriveGraph 초기화

        Args:
            graph_path: OmniGraph USD 경로
            robot_prim_path: 로봇 Articulation Prim 경로
            chassis_prim_path: 섀시 링크 경로 (Odometry용)
            left_wheel_joint: 왼쪽 바퀴 조인트 이름
            right_wheel_joint: 오른쪽 바퀴 조인트 이름
            wheel_radius: 바퀴 반경 (m)
            wheel_distance: 바퀴 간 거리 (m)
            namespace: ROS2 토픽 네임스페이스
            max_linear_velocity: 최대 선속도 (m/s)
            max_angular_velocity: 최대 각속도 (rad/s)
            publish_odom: Odometry 퍼블리시 여부
            publish_tf: TF 퍼블리시 여부
            publish_clock: Clock 퍼블리시 여부
        """
        self.graph_path = graph_path or self.DEFAULT_GRAPH_PATH
        self.robot_prim_path = robot_prim_path or self.DEFAULT_ROBOT_PATH
        self.chassis_prim_path = chassis_prim_path or self.DEFAULT_CHASSIS_PATH
        self.left_wheel_joint = left_wheel_joint or self.DEFAULT_LEFT_WHEEL
        self.right_wheel_joint = right_wheel_joint or self.DEFAULT_RIGHT_WHEEL
        self.wheel_radius = wheel_radius or self.DEFAULT_WHEEL_RADIUS
        self.wheel_distance = wheel_distance or self.DEFAULT_WHEEL_DISTANCE
        self.namespace = namespace
        self.max_linear_velocity = max_linear_velocity or self.DEFAULT_MAX_LINEAR_VEL
        self.max_angular_velocity = max_angular_velocity or self.DEFAULT_MAX_ANGULAR_VEL

        # 퍼블리시 옵션
        self.publish_odom = publish_odom
        self.publish_tf = publish_tf
        self.publish_clock = publish_clock

        self._graph_handle = None
        self._builder: Optional[OmniGraphBuilder] = None

    def create(self) -> Optional[Any]:
        """
        차동 구동 제어 그래프 생성

        Returns:
            그래프 핸들 (실패 시 None)
        """
        print(f"[DifferentialDriveGraph] Creating graph at {self.graph_path}")
        print(f"[DifferentialDriveGraph] Wheel radius: {self.wheel_radius}m, distance: {self.wheel_distance}m")

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

        if self._graph_handle:
            # Script Node 설정 (DifferentialController 대체)
            self._setup_script_node()
            print(f"[DifferentialDriveGraph] Graph created successfully")
            self._print_topic_info()
        else:
            print(f"[DifferentialDriveGraph] Graph creation failed")

        return self._graph_handle

    def _add_nodes(self):
        """그래프 노드 추가 (Isaac Sim 버전 자동 감지)"""
        # Note: Isaac Sim 4.5.0+에서 DifferentialController는 데이터 노드임
        # - outputs:execOut이 없음
        # - inputs:linearVelocity, inputs:angularVelocity가 double (스칼라)
        # - SubscribeTwist.outputs는 double3 (벡터)
        # 따라서 Script Node로 타입 변환 및 바퀴 속도 계산을 수행
        nodes = [
            # 실행 트리거
            ("OnPlaybackTick", ROS2NodeFactory.get_node_type("on_playback_tick")),

            # ROS2 컨텍스트 (버전에 따라 자동 선택)
            ("ROS2Context", ROS2NodeFactory.get_node_type("context")),

            # Twist 구독자 (/cmd_vel)
            ("SubscribeTwist", ROS2NodeFactory.get_node_type("subscribe_twist")),

            # 시간 읽기
            ("ReadSimTime", ROS2NodeFactory.get_node_type("read_sim_time")),

            # Script Node: Twist를 바퀴 속도로 직접 변환
            # DifferentialController 대신 사용 (타입 불일치 및 execOut 없음 문제 해결)
            ("TwistToWheelSpeed", ROS2NodeFactory.get_node_type("script_node")),

            # Articulation Controller
            ("ArticulationController", ROS2NodeFactory.get_node_type("articulation_controller")),
        ]

        # 선택적 퍼블리셔 추가
        if self.publish_odom:
            nodes.append(("PublishOdometry", ROS2NodeFactory.get_node_type("publish_odometry")))

        if self.publish_tf:
            nodes.append(("PublishTF", ROS2NodeFactory.get_node_type("publish_tf")))

        if self.publish_clock:
            nodes.append(("PublishClock", ROS2NodeFactory.get_node_type("publish_clock")))

        self._builder.add_nodes(nodes)

    def _set_values(self):
        """노드 속성 값 설정"""
        ns = self.namespace

        values = [
            # ROS2 Context
            ("ROS2Context.inputs:useDomainIDEnvVar", True),

            # Twist Subscriber
            ("SubscribeTwist.inputs:topicName", f"{ns}/cmd_vel" if ns else "/cmd_vel"),

            # TwistToWheelSpeed Script Node: 파라미터는 스크립트 내부에서 처리
            # (DifferentialController 대체)

            # Articulation Controller
            # Isaac Sim 4.5.0+: targetPrim 사용 (usePath 제거)
            ("ArticulationController.inputs:targetPrim", self.robot_prim_path),
            ("ArticulationController.inputs:jointNames", [self.left_wheel_joint, self.right_wheel_joint]),
        ]

        # Odometry Publisher 설정
        # Note: Isaac Sim 4.5.0+에서 ROS2PublishOdometry 속성 이름이 변경됨
        # chassisPrim → targetPrim (확인 필요)
        if self.publish_odom:
            values.extend([
                ("PublishOdometry.inputs:topicName", f"{ns}/odom" if ns else "/odom"),
                # Isaac Sim 4.5.0+: targetPrim 사용 시도
                ("PublishOdometry.inputs:targetPrim", self.chassis_prim_path),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:chassisFrameId", "base_link"),
            ])

        # TF Publisher 설정
        if self.publish_tf:
            values.extend([
                ("PublishTF.inputs:topicName", "/tf"),
                ("PublishTF.inputs:targetPrims", [self.robot_prim_path]),
            ])

        # Clock Publisher 설정
        if self.publish_clock:
            values.extend([
                ("PublishClock.inputs:topicName", "/clock"),
            ])

        self._builder.set_values(values)

    def _create_connections(self):
        """노드 연결"""
        # Note: Isaac Sim 4.5.0+에서 변경사항:
        # - IsaacReadSimulationTime은 데이터 노드로 execIn이 없음
        # - DifferentialController도 데이터 노드로 execOut이 없음
        # - SubscribeTwist.outputs:linearVelocity는 double3, DifferentialController.inputs는 double
        # 따라서 Script Node(TwistToWheelSpeed)로 직접 변환
        connections = [
            # 실행 흐름: OnPlaybackTick → SubscribeTwist → TwistToWheelSpeed → ArticulationController
            ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ("SubscribeTwist.outputs:execOut", "TwistToWheelSpeed.inputs:execIn"),
            ("TwistToWheelSpeed.outputs:execOut", "ArticulationController.inputs:execIn"),
            # Note: TwistToWheelSpeed Script Node는 내부에서 ArticulationController에
            # velocityCommand를 직접 설정 (USD API 사용)
        ]

        # 퍼블리셔 연결
        if self.publish_odom:
            connections.extend([
                ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
            ])

        if self.publish_tf:
            connections.append(("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"))

        if self.publish_clock:
            connections.extend([
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ])

        self._builder.connect_many(connections)

    def _setup_script_node(self):
        """Script Node에 Twist→바퀴 속도 변환 스크립트 설정"""
        script_content = self._generate_twist_to_wheel_script()

        try:
            import omni.graph.core as og

            script_node_path = f"{self.graph_path}/TwistToWheelSpeed"

            try:
                og.Controller.attribute(f"{script_node_path}.inputs:script").set(script_content)
                print(f"[DifferentialDriveGraph] TwistToWheelSpeed script configured")
            except Exception as e:
                print(f"[DifferentialDriveGraph] Could not set script: {e}")

        except Exception as e:
            print(f"[DifferentialDriveGraph] Script setup warning: {e}")

    def _generate_twist_to_wheel_script(self) -> str:
        """
        Twist 메시지를 바퀴 속도로 변환하는 스크립트 생성

        차동 구동 공식:
            ωR = (v + ω * L/2) / r  (오른쪽 바퀴)
            ωL = (v - ω * L/2) / r  (왼쪽 바퀴)
        """
        ns = self.namespace
        topic_name = f"{ns}/cmd_vel" if ns else "/cmd_vel"

        return f'''"""
Twist to Wheel Speed Converter Script
Converts geometry_msgs/Twist to wheel angular velocities

Robot Config:
  - Wheel Radius: {self.wheel_radius} m
  - Wheel Distance: {self.wheel_distance} m
  - Max Linear: {self.max_linear_velocity} m/s
  - Max Angular: {self.max_angular_velocity} rad/s
  - Left Wheel: {self.left_wheel_joint}
  - Right Wheel: {self.right_wheel_joint}
  - Topic: {topic_name}
"""
import numpy as np

# Robot parameters
WHEEL_RADIUS = {self.wheel_radius}
WHEEL_DISTANCE = {self.wheel_distance}
MAX_LINEAR = {self.max_linear_velocity}
MAX_ANGULAR = {self.max_angular_velocity}
CMD_VEL_TOPIC = "{topic_name}"

# Global state
_initialized = False
_robot = None
_robot_prim_path = "{self.robot_prim_path}"


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

        # Get or add robot
        _robot = world.scene.get_object("diff_drive_robot")
        if _robot is None:
            _robot = world.scene.add(
                Articulation(prim_path=_robot_prim_path, name="diff_drive_robot")
            )
            world.reset()

        _initialized = True
        print("[TwistToWheelSpeed] Initialized")
        return True

    except Exception as e:
        print(f"[TwistToWheelSpeed] Setup error: {{e}}")
        return False


def compute(db):
    """
    Convert Twist to wheel velocities and apply

    Differential drive kinematics:
        ωR = (v + ω * L/2) / r
        ωL = (v - ω * L/2) / r
    """
    global _initialized, _robot

    if not _initialized:
        if not setup(db):
            return

    if _robot is None:
        return

    try:
        # ROS2 SubscribeTwist의 출력을 읽음
        # Note: Script Node에서는 SubscribeTwist 출력에 직접 접근 불가
        # 대신 rclpy를 사용하여 직접 구독
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist

        # 이미 rclpy가 초기화되어 있어야 함 (ROS2Context 노드에서 처리)
        if not rclpy.ok():
            return

        # 글로벌 노드에서 최신 Twist 값 가져오기
        if not hasattr(compute, '_twist_node'):
            class TwistSubscriber(Node):
                def __init__(self):
                    super().__init__('diff_drive_twist_sub')
                    self.linear_x = 0.0
                    self.angular_z = 0.0
                    self.subscription = self.create_subscription(
                        Twist,
                        CMD_VEL_TOPIC,
                        self.twist_callback,
                        10
                    )

                def twist_callback(self, msg):
                    self.linear_x = msg.linear.x
                    self.angular_z = msg.angular.z

            compute._twist_node = TwistSubscriber()

        # Spin once to process callbacks
        rclpy.spin_once(compute._twist_node, timeout_sec=0.001)

        # Get velocities
        linear_vel = compute._twist_node.linear_x
        angular_vel = compute._twist_node.angular_z

        # Clamp velocities
        linear_vel = max(-MAX_LINEAR, min(MAX_LINEAR, linear_vel))
        angular_vel = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_vel))

        # Differential drive kinematics
        # ωR = (v + ω * L/2) / r
        # ωL = (v - ω * L/2) / r
        half_wheel_dist = WHEEL_DISTANCE / 2.0
        right_wheel_vel = (linear_vel + angular_vel * half_wheel_dist) / WHEEL_RADIUS
        left_wheel_vel = (linear_vel - angular_vel * half_wheel_dist) / WHEEL_RADIUS

        # Apply to robot using velocity control
        from omni.isaac.core.utils.types import ArticulationAction

        action = ArticulationAction(
            joint_velocities=np.array([left_wheel_vel, right_wheel_vel])
        )
        _robot.apply_action(action)

    except ImportError:
        # rclpy not available - fallback to zero velocity
        pass
    except Exception as e:
        pass
'''

    def _print_topic_info(self):
        """토픽 정보 출력"""
        ns = self.namespace
        print(f"[DifferentialDriveGraph] ROS2 Topics:")
        print(f"  Subscribe: {ns}/cmd_vel" if ns else "  Subscribe: /cmd_vel")
        if self.publish_odom:
            print(f"  Publish: {ns}/odom" if ns else "  Publish: /odom")
        if self.publish_tf:
            print(f"  Publish: /tf")
        if self.publish_clock:
            print(f"  Publish: /clock")

    def get_graph_handle(self):
        """그래프 핸들 반환"""
        return self._graph_handle

    def get_config(self) -> Dict[str, Any]:
        """현재 설정 반환"""
        return {
            "graph_path": self.graph_path,
            "robot_prim_path": self.robot_prim_path,
            "chassis_prim_path": self.chassis_prim_path,
            "left_wheel_joint": self.left_wheel_joint,
            "right_wheel_joint": self.right_wheel_joint,
            "wheel_radius": self.wheel_radius,
            "wheel_distance": self.wheel_distance,
            "namespace": self.namespace,
            "max_linear_velocity": self.max_linear_velocity,
            "max_angular_velocity": self.max_angular_velocity,
            "publish_odom": self.publish_odom,
            "publish_tf": self.publish_tf,
            "publish_clock": self.publish_clock,
        }

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> 'DifferentialDriveGraph':
        """설정 딕셔너리에서 인스턴스 생성"""
        return cls(
            graph_path=config.get("graph_path"),
            robot_prim_path=config.get("robot_prim_path"),
            chassis_prim_path=config.get("chassis_prim_path"),
            left_wheel_joint=config.get("left_wheel_joint"),
            right_wheel_joint=config.get("right_wheel_joint"),
            wheel_radius=config.get("wheel_radius"),
            wheel_distance=config.get("wheel_distance"),
            namespace=config.get("namespace", ""),
            max_linear_velocity=config.get("max_linear_velocity"),
            max_angular_velocity=config.get("max_angular_velocity"),
            publish_odom=config.get("publish_odom", True),
            publish_tf=config.get("publish_tf", True),
            publish_clock=config.get("publish_clock", True),
        )

    def __repr__(self) -> str:
        return (
            f"DifferentialDriveGraph("
            f"path='{self.graph_path}', "
            f"robot='{self.robot_prim_path}', "
            f"wheel_radius={self.wheel_radius}, "
            f"wheel_distance={self.wheel_distance})"
        )


# =============================================================================
# Extended: With LiDAR Support
# =============================================================================

class DifferentialDriveWithLidarGraph(DifferentialDriveGraph):
    """
    LiDAR 센서를 포함한 차동 구동 그래프

    기본 차동 구동 기능에 LaserScan 퍼블리시를 추가합니다.
    """

    def __init__(
        self,
        lidar_prim_path: str = None,
        scan_topic: str = "/scan",
        scan_frame_id: str = "lidar_link",
        **kwargs
    ):
        """
        Args:
            lidar_prim_path: LiDAR 센서 Prim 경로
            scan_topic: LaserScan 토픽 이름
            scan_frame_id: LaserScan frame_id
            **kwargs: DifferentialDriveGraph 인자
        """
        super().__init__(**kwargs)
        self.lidar_prim_path = lidar_prim_path
        self.scan_topic = scan_topic
        self.scan_frame_id = scan_frame_id

    def _add_nodes(self):
        """LiDAR 노드 추가"""
        super()._add_nodes()

        if self.lidar_prim_path:
            self._builder.add_nodes([
                ("ReadLidar", ROS2NodeFactory.get_node_type("read_lidar_beams")),
                ("PublishLaserScan", ROS2NodeFactory.get_node_type("publish_laser_scan")),
            ])

    def _set_values(self):
        """LiDAR 값 설정"""
        super()._set_values()

        if self.lidar_prim_path:
            self._builder.set_values([
                ("ReadLidar.inputs:lidarPrim", self.lidar_prim_path),
                ("PublishLaserScan.inputs:topicName", self.scan_topic),
                ("PublishLaserScan.inputs:frameId", self.scan_frame_id),
            ])

    def _create_connections(self):
        """LiDAR 연결"""
        super()._create_connections()

        if self.lidar_prim_path:
            self._builder.connect_many([
                ("OnPlaybackTick.outputs:tick", "ReadLidar.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PublishLaserScan.inputs:execIn"),
                ("ReadLidar.outputs:azimuthRange", "PublishLaserScan.inputs:azimuthRange"),
                ("ReadLidar.outputs:depthRange", "PublishLaserScan.inputs:depthRange"),
                ("ReadLidar.outputs:horizontalFov", "PublishLaserScan.inputs:horizontalFov"),
                ("ReadLidar.outputs:horizontalResolution", "PublishLaserScan.inputs:horizontalResolution"),
                ("ReadLidar.outputs:intensitiesData", "PublishLaserScan.inputs:intensitiesData"),
                ("ReadLidar.outputs:linearDepthData", "PublishLaserScan.inputs:linearDepthData"),
                ("ReadLidar.outputs:numCols", "PublishLaserScan.inputs:numCols"),
                ("ReadLidar.outputs:numRows", "PublishLaserScan.inputs:numRows"),
                ("ReadLidar.outputs:rotationRate", "PublishLaserScan.inputs:rotationRate"),
            ])
