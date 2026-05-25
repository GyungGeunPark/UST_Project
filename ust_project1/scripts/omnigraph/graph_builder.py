"""
omnigraph/graph_builder.py
OmniGraph 생성을 위한 공통 유틸리티 클래스

이 모듈은 Isaac Sim OmniGraph를 프로그래매틱하게 생성하기 위한
빌더 패턴과 팩토리 패턴을 제공합니다.

Usage:
    builder = OmniGraphBuilder("/World/MyGraph")
    builder.add_node("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")
    builder.add_node("Controller", "omni.isaac.core_nodes.IsaacArticulationController")
    builder.set_value("Controller.inputs:robotPath", "/World/Robot")
    builder.connect("OnPlaybackTick.outputs:tick", "Controller.inputs:execIn")
    graph = builder.build()

Author: UST Robotics Project
Date: 2024
"""

from typing import Dict, List, Any, Optional, Tuple, Union
from enum import Enum
import os


class GraphEvaluator(Enum):
    """OmniGraph 평가자 타입"""
    EXECUTION = "execution"      # Action Graph (실행 기반)
    PUSH = "push"               # Push Graph (데이터 기반)
    DIRTY_PUSH = "dirty_push"   # Dirty Push Graph


class OmniGraphBuilder:
    """
    OmniGraph 생성 및 관리를 위한 빌더 클래스

    플루언트 인터페이스를 제공하여 체이닝 방식으로 그래프를 구성할 수 있습니다.

    Attributes:
        graph_path: USD 스테이지 내 그래프 경로
        evaluator_name: 그래프 평가자 타입

    Example:
        >>> builder = OmniGraphBuilder("/World/MyGraph")
        >>> builder.add_node("Tick", "omni.graph.action.OnPlaybackTick")
        >>> builder.add_node("Ctrl", "omni.isaac.core_nodes.IsaacArticulationController")
        >>> builder.set_value("Ctrl.inputs:robotPath", "/World/Robot")
        >>> builder.connect("Tick.outputs:tick", "Ctrl.inputs:execIn")
        >>> graph = builder.build()
    """

    def __init__(
        self,
        graph_path: str,
        evaluator_name: Union[str, GraphEvaluator] = GraphEvaluator.EXECUTION
    ):
        """
        OmniGraphBuilder 초기화

        Args:
            graph_path: USD 스테이지 내 그래프 경로 (예: "/World/MyGraph")
            evaluator_name: 그래프 평가자 타입 ("execution" for Action Graph)
        """
        self.graph_path = graph_path

        if isinstance(evaluator_name, GraphEvaluator):
            self.evaluator_name = evaluator_name.value
        else:
            self.evaluator_name = evaluator_name

        # 빌드 데이터 저장
        self._nodes: List[Tuple[str, str]] = []
        self._values: List[Tuple[str, Any]] = []
        self._connections: List[Tuple[str, str]] = []

        # 결과 저장
        self._graph_handle = None
        self._node_map: Dict[str, Any] = {}
        self._is_built = False

    def add_node(self, node_name: str, node_type: str) -> 'OmniGraphBuilder':
        """
        그래프에 노드 추가

        Args:
            node_name: 노드 이름 (그래프 내 고유 식별자)
            node_type: OmniGraph 노드 타입 (전체 경로)

        Returns:
            self (체이닝 지원)

        Example:
            >>> builder.add_node("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")
        """
        self._nodes.append((node_name, node_type))
        return self

    def add_nodes(self, nodes: List[Tuple[str, str]]) -> 'OmniGraphBuilder':
        """
        여러 노드를 한 번에 추가

        Args:
            nodes: (노드이름, 노드타입) 튜플 리스트

        Returns:
            self (체이닝 지원)

        Example:
            >>> builder.add_nodes([
            ...     ("Tick", "omni.graph.action.OnPlaybackTick"),
            ...     ("Ctrl", "omni.isaac.core_nodes.IsaacArticulationController"),
            ... ])
        """
        self._nodes.extend(nodes)
        return self

    def set_value(self, attribute_path: str, value: Any) -> 'OmniGraphBuilder':
        """
        노드 속성 값 설정

        Args:
            attribute_path: "노드이름.inputs:속성이름" 형식
            value: 설정할 값

        Returns:
            self (체이닝 지원)

        Example:
            >>> builder.set_value("Controller.inputs:robotPath", "/World/Robot")
            >>> builder.set_value("Controller.inputs:usePath", True)
        """
        self._values.append((attribute_path, value))
        return self

    def set_values(self, values: List[Tuple[str, Any]]) -> 'OmniGraphBuilder':
        """
        여러 속성 값을 한 번에 설정

        Args:
            values: (속성경로, 값) 튜플 리스트

        Returns:
            self (체이닝 지원)
        """
        self._values.extend(values)
        return self

    def connect(self, output_path: str, input_path: str) -> 'OmniGraphBuilder':
        """
        노드 간 연결 생성

        Args:
            output_path: "노드이름.outputs:출력이름" 형식
            input_path: "노드이름.inputs:입력이름" 형식

        Returns:
            self (체이닝 지원)

        Example:
            >>> builder.connect("OnPlaybackTick.outputs:tick", "Controller.inputs:execIn")
        """
        self._connections.append((output_path, input_path))
        return self

    def connect_many(self, connections: List[Tuple[str, str]]) -> 'OmniGraphBuilder':
        """
        여러 연결을 한 번에 생성

        Args:
            connections: (출력경로, 입력경로) 튜플 리스트

        Returns:
            self (체이닝 지원)
        """
        self._connections.extend(connections)
        return self

    def remove_existing(self) -> 'OmniGraphBuilder':
        """
        기존 그래프가 있으면 제거

        Returns:
            self (체이닝 지원)
        """
        try:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            if stage and stage.GetPrimAtPath(self.graph_path):
                stage.RemovePrim(self.graph_path)
                print(f"[OmniGraphBuilder] Removed existing graph: {self.graph_path}")
        except Exception as e:
            print(f"[OmniGraphBuilder] Warning during removal: {e}")
        return self

    def build(self) -> Optional[Any]:
        """
        그래프 빌드 실행

        모든 노드, 값 설정, 연결을 적용하여 OmniGraph를 생성합니다.

        Returns:
            그래프 핸들 (실패 시 None)
        """
        try:
            import omni.graph.core as og

            keys = og.Controller.Keys

            self._graph_handle, nodes, _, _ = og.Controller.edit(
                {
                    "graph_path": self.graph_path,
                    "evaluator_name": self.evaluator_name
                },
                {
                    keys.CREATE_NODES: self._nodes,
                    keys.SET_VALUES: self._values,
                    keys.CONNECT: self._connections,
                }
            )

            # 노드 맵 생성
            for i, (name, _) in enumerate(self._nodes):
                self._node_map[name] = nodes[i]

            self._is_built = True
            print(f"[OmniGraphBuilder] Graph created at {self.graph_path}")
            print(f"[OmniGraphBuilder] Nodes: {len(self._nodes)}, Connections: {len(self._connections)}")

            return self._graph_handle

        except Exception as e:
            print(f"[OmniGraphBuilder] Build failed: {e}")
            import traceback
            traceback.print_exc()
            return None

    def get_node(self, node_name: str) -> Optional[Any]:
        """
        빌드된 노드 가져오기

        Args:
            node_name: 노드 이름

        Returns:
            노드 객체 (빌드 전이거나 없으면 None)
        """
        return self._node_map.get(node_name)

    def get_graph_handle(self) -> Optional[Any]:
        """그래프 핸들 반환"""
        return self._graph_handle

    @property
    def is_built(self) -> bool:
        """빌드 완료 여부"""
        return self._is_built

    @property
    def node_count(self) -> int:
        """추가된 노드 수"""
        return len(self._nodes)

    @property
    def connection_count(self) -> int:
        """추가된 연결 수"""
        return len(self._connections)

    def __repr__(self) -> str:
        status = "built" if self._is_built else "pending"
        return f"OmniGraphBuilder(path={self.graph_path}, nodes={len(self._nodes)}, status={status})"

    # =========================================================================
    # 정적 유틸리티 메서드
    # =========================================================================

    @staticmethod
    def create_xform_target(
        target_path: str,
        position: List[float] = None,
        rotation: List[float] = None,
        scale: List[float] = None
    ) -> bool:
        """
        IK Target용 Xform Prim 생성

        Args:
            target_path: USD 경로
            position: 초기 위치 [x, y, z] (기본: [0.2, 0.0, 0.15])
            rotation: 초기 회전 [x, y, z] 오일러 각도 (기본: [0, 0, 0])
            scale: 초기 스케일 [x, y, z] (기본: [1, 1, 1])

        Returns:
            생성 성공 여부
        """
        if position is None:
            position = [0.2, 0.0, 0.15]

        try:
            import omni.usd
            from pxr import UsdGeom, Gf

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                print("[OmniGraphBuilder] No stage available")
                return False

            if not stage.GetPrimAtPath(target_path):
                xform = UsdGeom.Xform.Define(stage, target_path)
                xform.AddTranslateOp().Set(Gf.Vec3d(*position))

                if rotation:
                    xform.AddRotateXYZOp().Set(Gf.Vec3d(*rotation))

                if scale:
                    xform.AddScaleOp().Set(Gf.Vec3d(*scale))

                print(f"[OmniGraphBuilder] Created Xform target at {target_path}")
                return True
            else:
                print(f"[OmniGraphBuilder] Xform already exists at {target_path}")
                return True

        except Exception as e:
            print(f"[OmniGraphBuilder] Failed to create Xform: {e}")
            return False

    @staticmethod
    def get_stage():
        """현재 USD 스테이지 가져오기"""
        try:
            import omni.usd
            return omni.usd.get_context().get_stage()
        except:
            return None

    @staticmethod
    def prim_exists(prim_path: str) -> bool:
        """Prim 존재 여부 확인"""
        try:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                return prim.IsValid()
            return False
        except:
            return False


class ROS2NodeFactory:
    """
    ROS2 관련 OmniGraph 노드 생성 팩토리

    자주 사용되는 ROS2 노드의 전체 타입 경로를 관리하고
    짧은 이름으로 접근할 수 있게 합니다.

    Isaac Sim 4.5.0+ 버전부터 확장 이름이 변경되었습니다:
    - omni.isaac.ros2_bridge → isaacsim.ros2.bridge
    - omni.isaac.core_nodes → isaacsim.core.nodes
    - omni.isaac.wheeled_robots → isaacsim.wheeled_robots

    Example:
        >>> node_type = ROS2NodeFactory.get_node_type("subscribe_twist")
        >>> # Returns: "isaacsim.ros2.bridge.ROS2SubscribeTwist"
    """

    # =========================================================================
    # Isaac Sim 4.5.0+ 새 노드 타입 (isaacsim.*)
    # =========================================================================

    # ROS2 노드 타입 매핑 (Isaac Sim 4.5.0+)
    # 참고: 일부 노드 타입은 Isaac Sim 버전에 따라 다를 수 있음
    # ROS2SubscribePoseStamped는 일부 버전에서 없을 수 있음 -> ROS2SubscribeTransform 사용
    NODE_TYPES_NEW = {
        # 컨텍스트
        "context": "isaacsim.ros2.bridge.ROS2Context",

        # 구독자 (Subscribers)
        "subscribe_twist": "isaacsim.ros2.bridge.ROS2SubscribeTwist",
        "subscribe_joint_state": "isaacsim.ros2.bridge.ROS2SubscribeJointState",
        # PoseStamped: 여러 대체 노드 시도 (일부 버전에서 없을 수 있음)
        "subscribe_pose_stamped": "isaacsim.ros2.bridge.ROS2SubscribeTransformTree",
        "subscribe_pose": "isaacsim.ros2.bridge.ROS2SubscribeTransformTree",
        "subscribe_transform_stamped": "isaacsim.ros2.bridge.ROS2SubscribeTransformTree",
        "subscribe_transform_tree": "isaacsim.ros2.bridge.ROS2SubscribeTransformTree",
        "subscribe_bool": "isaacsim.ros2.bridge.ROS2SubscribeBool",
        "subscribe_float32": "isaacsim.ros2.bridge.ROS2SubscribeFloat32",
        "subscribe_int32": "isaacsim.ros2.bridge.ROS2SubscribeInt32",

        # 퍼블리셔 (Publishers)
        "publish_joint_state": "isaacsim.ros2.bridge.ROS2PublishJointState",
        "publish_odometry": "isaacsim.ros2.bridge.ROS2PublishOdometry",
        "publish_tf": "isaacsim.ros2.bridge.ROS2PublishTransformTree",
        "publish_clock": "isaacsim.ros2.bridge.ROS2PublishClock",
        "publish_laser_scan": "isaacsim.ros2.bridge.ROS2PublishLaserScan",
        "publish_point_cloud": "isaacsim.ros2.bridge.ROS2PublishPointCloud",
        "publish_image": "isaacsim.ros2.bridge.ROS2PublishImage",
        "publish_camera_info": "isaacsim.ros2.bridge.ROS2PublishCameraInfo",
        "publish_imu": "isaacsim.ros2.bridge.ROS2PublishImu",
        "publish_raw": "isaacsim.ros2.bridge.ROS2PublishRaw",

        # 서비스
        "service_teleport": "isaacsim.ros2.bridge.ROS2ServiceTeleport",
    }

    # Isaac Core 노드 타입 매핑 (Isaac Sim 4.5.0+)
    CORE_NODE_TYPES_NEW = {
        "articulation_controller": "isaacsim.core.nodes.IsaacArticulationController",
        "articulation_state": "isaacsim.core.nodes.IsaacArticulationState",
        "read_sim_time": "isaacsim.core.nodes.IsaacReadSimulationTime",
        "read_system_time": "isaacsim.core.nodes.IsaacReadSystemTime",
        "create_viewport": "isaacsim.core.nodes.IsaacCreateViewport",
        "get_viewport_render_product": "isaacsim.core.nodes.IsaacGetViewportRenderProduct",
        "set_camera_on_render_product": "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct",
    }

    # Wheeled Robot 노드 타입 (Isaac Sim 4.5.0+)
    # 참고: 확장 이름이 버전에 따라 다를 수 있음
    #   - isaacsim.wheeled_robots
    #   - isaacsim.robot.wheeled_robots
    #   - omni.isaac.wheeled_robots (레거시)
    WHEELED_ROBOT_TYPES_NEW = {
        "differential_controller": "isaacsim.robot.wheeled_robots.DifferentialController",
        "holonomic_controller": "isaacsim.robot.wheeled_robots.HolonomicController",
    }

    # 대체 Wheeled Robot 노드 타입 (일부 Isaac Sim 4.5.x 버전)
    WHEELED_ROBOT_TYPES_ALT = {
        "differential_controller": "isaacsim.wheeled_robots.DifferentialController",
        "holonomic_controller": "isaacsim.wheeled_robots.HolonomicController",
    }

    # 센서 노드 타입 (Isaac Sim 4.5.0+)
    SENSOR_NODE_TYPES_NEW = {
        "read_lidar_beams": "isaacsim.range_sensor.IsaacReadLidarBeams",
        "read_lidar_point_cloud": "isaacsim.range_sensor.IsaacReadLidarPointCloud",
    }

    # =========================================================================
    # 레거시 노드 타입 (omni.isaac.* - Isaac Sim 4.2 이하)
    # =========================================================================

    # ROS2 노드 타입 매핑 (레거시)
    NODE_TYPES = {
        # 컨텍스트
        "context": "omni.isaac.ros2_bridge.ROS2Context",

        # 구독자 (Subscribers)
        "subscribe_twist": "omni.isaac.ros2_bridge.ROS2SubscribeTwist",
        "subscribe_joint_state": "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
        "subscribe_pose_stamped": "omni.isaac.ros2_bridge.ROS2SubscribePoseStamped",
        "subscribe_pose": "omni.isaac.ros2_bridge.ROS2SubscribePose",
        "subscribe_transform_stamped": "omni.isaac.ros2_bridge.ROS2SubscribeTransformStamped",
        "subscribe_bool": "omni.isaac.ros2_bridge.ROS2SubscribeBool",
        "subscribe_float32": "omni.isaac.ros2_bridge.ROS2SubscribeFloat32",
        "subscribe_int32": "omni.isaac.ros2_bridge.ROS2SubscribeInt32",

        # 퍼블리셔 (Publishers)
        "publish_joint_state": "omni.isaac.ros2_bridge.ROS2PublishJointState",
        "publish_odometry": "omni.isaac.ros2_bridge.ROS2PublishOdometry",
        "publish_tf": "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
        "publish_clock": "omni.isaac.ros2_bridge.ROS2PublishClock",
        "publish_laser_scan": "omni.isaac.ros2_bridge.ROS2PublishLaserScan",
        "publish_point_cloud": "omni.isaac.ros2_bridge.ROS2PublishPointCloud",
        "publish_image": "omni.isaac.ros2_bridge.ROS2PublishImage",
        "publish_camera_info": "omni.isaac.ros2_bridge.ROS2PublishCameraInfo",
        "publish_imu": "omni.isaac.ros2_bridge.ROS2PublishImu",
        "publish_raw": "omni.isaac.ros2_bridge.ROS2PublishRaw",

        # 서비스
        "service_teleport": "omni.isaac.ros2_bridge.ROS2ServiceTeleport",
    }

    # Isaac Core 노드 타입 매핑 (레거시)
    CORE_NODE_TYPES = {
        "articulation_controller": "omni.isaac.core_nodes.IsaacArticulationController",
        "articulation_state": "omni.isaac.core_nodes.IsaacArticulationState",
        "read_sim_time": "omni.isaac.core_nodes.IsaacReadSimulationTime",
        "read_system_time": "omni.isaac.core_nodes.IsaacReadSystemTime",
        "create_viewport": "omni.isaac.core_nodes.IsaacCreateViewport",
        "get_viewport_render_product": "omni.isaac.core_nodes.IsaacGetViewportRenderProduct",
        "set_camera_on_render_product": "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct",
    }

    # Wheeled Robot 노드 타입 (레거시)
    WHEELED_ROBOT_TYPES = {
        "differential_controller": "omni.isaac.wheeled_robots.DifferentialController",
        "holonomic_controller": "omni.isaac.wheeled_robots.HolonomicController",
    }

    # 센서 노드 타입 (레거시)
    SENSOR_NODE_TYPES = {
        "read_lidar_beams": "omni.isaac.range_sensor.IsaacReadLidarBeams",
        "read_lidar_point_cloud": "omni.isaac.range_sensor.IsaacReadLidarPointCloud",
    }

    # 일반 OmniGraph 노드 타입 (버전 무관)
    GRAPH_NODE_TYPES = {
        "on_playback_tick": "omni.graph.action.OnPlaybackTick",
        "on_stage_event": "omni.graph.action.OnStageEvent",
        "on_tick": "omni.graph.action.OnTick",
        "on_keyboard_input": "omni.graph.action.OnKeyboardInput",
        "script_node": "omni.graph.scriptnode.ScriptNode",
        "read_prim_attribute": "omni.graph.nodes.ReadPrimAttribute",
        "write_prim_attribute": "omni.graph.nodes.WritePrimAttribute",
        "read_prims": "omni.graph.nodes.ReadPrims",
    }

    # 캐시된 버전 정보
    _use_new_api = None

    @classmethod
    def _detect_api_version(cls) -> bool:
        """
        Isaac Sim API 버전 감지

        Returns:
            True: 새 API (isaacsim.*) 사용
            False: 레거시 API (omni.isaac.*) 사용
        """
        if cls._use_new_api is not None:
            return cls._use_new_api

        # 방법 1: 확장 관리자를 통해 확인 (가장 신뢰할 수 있음)
        try:
            import omni.kit.app
            ext_manager = omni.kit.app.get_app().get_extension_manager()

            new_ext_enabled = ext_manager.is_extension_enabled("isaacsim.ros2.bridge")
            old_ext_enabled = ext_manager.is_extension_enabled("omni.isaac.ros2_bridge")

            if new_ext_enabled:
                cls._use_new_api = True
                print("[ROS2NodeFactory] Detected Isaac Sim 4.5.0+ via extension manager (isaacsim.* API)")
                return cls._use_new_api
            elif old_ext_enabled:
                cls._use_new_api = False
                print("[ROS2NodeFactory] Detected Isaac Sim 4.2 or earlier via extension manager (omni.isaac.* API)")
                return cls._use_new_api
        except Exception:
            pass

        # 방법 2: OmniGraph 노드 타입 확인
        try:
            all_types = cls._get_available_node_types()

            if all_types:
                # 새 API 노드 타입 존재 확인
                new_api_exists = any('isaacsim.ros2.bridge' in t for t in all_types)
                old_api_exists = any('omni.isaac.ros2_bridge' in t for t in all_types)

                if new_api_exists:
                    cls._use_new_api = True
                    print("[ROS2NodeFactory] Detected Isaac Sim 4.5.0+ via node types (isaacsim.* API)")
                elif old_api_exists:
                    cls._use_new_api = False
                    print("[ROS2NodeFactory] Detected Isaac Sim 4.2 or earlier via node types (omni.isaac.* API)")
                else:
                    # 기본값: 확장 관리자 결과에 따름
                    cls._use_new_api = True
                    print("[ROS2NodeFactory] Could not detect API version via node types, defaulting to new API")
            else:
                # 노드 타입 목록을 가져올 수 없으면 확장 관리자 결과 사용
                cls._use_new_api = True
                print("[ROS2NodeFactory] No node types available, defaulting to new API")

        except Exception as e:
            # OmniGraph 사용 불가 시 새 API로 기본 설정
            cls._use_new_api = True
            print(f"[ROS2NodeFactory] API detection failed: {e}, defaulting to new API")

        return cls._use_new_api

    # 캐시된 사용 가능한 노드 타입
    _available_node_types = None

    @classmethod
    def _get_available_node_types(cls) -> set:
        """사용 가능한 노드 타입 목록 캐시하여 반환"""
        if cls._available_node_types is None:
            cls._available_node_types = set()
            try:
                import omni.graph.core as og
                # 여러 API 시도
                if hasattr(og, 'get_all_node_types'):
                    cls._available_node_types = set(og.get_all_node_types())
                elif hasattr(og, 'get_registered_nodes'):
                    cls._available_node_types = set(og.get_registered_nodes())
                elif hasattr(og, 'get_node_type_registry'):
                    registry = og.get_node_type_registry()
                    if registry and hasattr(registry, 'get_all_node_type_names'):
                        cls._available_node_types = set(registry.get_all_node_type_names())
            except Exception as e:
                print(f"[ROS2NodeFactory] Could not get available node types: {e}")
        return cls._available_node_types

    @classmethod
    def _find_matching_node_type(cls, base_name: str, prefixes: list) -> str:
        """
        주어진 접두사들로 실제 존재하는 노드 타입 찾기

        Args:
            base_name: 노드 기본 이름 (예: "ROS2SubscribeTwist")
            prefixes: 시도할 접두사 목록 (예: ["isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"])

        Returns:
            찾은 노드 타입 또는 첫 번째 후보
        """
        available = cls._get_available_node_types()

        for prefix in prefixes:
            full_type = f"{prefix}.{base_name}"
            if full_type in available:
                return full_type

        # 부분 매칭 시도 (대소문자 무시)
        base_lower = base_name.lower()
        for node_type in available:
            if base_lower in node_type.lower():
                return node_type

        # 찾지 못하면 첫 번째 후보 반환
        return f"{prefixes[0]}.{base_name}" if prefixes else base_name

    @classmethod
    def get_node_type(cls, short_name: str) -> str:
        """
        단축 이름으로 전체 노드 타입 반환

        Isaac Sim 버전을 자동 감지하여 적절한 노드 타입을 반환합니다.
        실제로 존재하는 노드 타입을 확인하고 폴백합니다.

        Args:
            short_name: 노드 단축 이름 (예: "subscribe_twist")

        Returns:
            전체 노드 타입 경로

        Example:
            >>> ROS2NodeFactory.get_node_type("subscribe_twist")
            'isaacsim.ros2.bridge.ROS2SubscribeTwist'  # Isaac Sim 4.5.0+
        """
        use_new = cls._detect_api_version()

        # 일반 그래프 노드 확인 (버전 무관)
        graph_type = cls.GRAPH_NODE_TYPES.get(short_name)
        if graph_type:
            return graph_type

        # 실제 존재하는 노드 타입 확인
        available = cls._get_available_node_types()

        # 노드 타입 후보 목록 생성 (우선순위 순)
        candidates = []

        if use_new:
            # 새 API 우선
            new_type = cls.NODE_TYPES_NEW.get(short_name) or \
                       cls.CORE_NODE_TYPES_NEW.get(short_name) or \
                       cls.WHEELED_ROBOT_TYPES_NEW.get(short_name) or \
                       cls.SENSOR_NODE_TYPES_NEW.get(short_name)

            # 대체 타입 (Wheeled Robot 등 일부 확장은 다른 이름일 수 있음)
            alt_type = cls.WHEELED_ROBOT_TYPES_ALT.get(short_name)

            old_type = cls.NODE_TYPES.get(short_name) or \
                       cls.CORE_NODE_TYPES.get(short_name) or \
                       cls.WHEELED_ROBOT_TYPES.get(short_name) or \
                       cls.SENSOR_NODE_TYPES.get(short_name)

            if new_type:
                candidates.append(new_type)
            if alt_type:
                candidates.append(alt_type)
            if old_type:
                candidates.append(old_type)
        else:
            # 레거시 API 우선
            old_type = cls.NODE_TYPES.get(short_name) or \
                       cls.CORE_NODE_TYPES.get(short_name) or \
                       cls.WHEELED_ROBOT_TYPES.get(short_name) or \
                       cls.SENSOR_NODE_TYPES.get(short_name)

            new_type = cls.NODE_TYPES_NEW.get(short_name) or \
                       cls.CORE_NODE_TYPES_NEW.get(short_name) or \
                       cls.WHEELED_ROBOT_TYPES_NEW.get(short_name) or \
                       cls.SENSOR_NODE_TYPES_NEW.get(short_name)

            alt_type = cls.WHEELED_ROBOT_TYPES_ALT.get(short_name)

            if old_type:
                candidates.append(old_type)
            if new_type:
                candidates.append(new_type)
            if alt_type:
                candidates.append(alt_type)

        # 후보 중 실제 존재하는 노드 타입 찾기
        if available:
            for candidate in candidates:
                if candidate in available:
                    return candidate

            # 부분 매칭 시도 (대소문자 무시)
            # 예: "DifferentialController"가 어떤 네임스페이스에 있는지 찾기
            if short_name in ["differential_controller", "holonomic_controller"]:
                base_name = short_name.replace("_controller", "").title() + "Controller"
                for node_type in available:
                    if base_name.lower() in node_type.lower():
                        print(f"[ROS2NodeFactory] Found '{short_name}' as '{node_type}' via partial match")
                        return node_type

        # 후보 중 첫 번째 반환 (실제 존재 여부 무관)
        if candidates:
            return candidates[0]

        # 매핑에 없으면 원본 반환
        return short_name

    @classmethod
    def reset_cache(cls):
        """
        캐시 리셋 - 버전 재감지를 위해 호출

        Isaac Sim Script Editor에서 모듈이 캐시될 수 있으므로,
        확장 로드 후 이 메서드를 호출하여 버전을 재감지합니다.
        """
        cls._use_new_api = None
        cls._available_node_types = None
        print("[ROS2NodeFactory] Cache reset - will re-detect API version and available node types")

    @classmethod
    def force_new_api(cls, use_new: bool = True):
        """
        강제로 API 버전 설정

        Args:
            use_new: True면 새 API (isaacsim.*), False면 레거시 API (omni.isaac.*)
        """
        cls._use_new_api = use_new
        api_name = "isaacsim.*" if use_new else "omni.isaac.*"
        print(f"[ROS2NodeFactory] Forced to use {api_name} API")

    @classmethod
    def get_available_types(cls) -> Dict[str, str]:
        """사용 가능한 모든 노드 타입 반환"""
        return {
            **cls.NODE_TYPES,
            **cls.CORE_NODE_TYPES,
            **cls.WHEELED_ROBOT_TYPES,
            **cls.SENSOR_NODE_TYPES,
            **cls.GRAPH_NODE_TYPES,
        }

    @classmethod
    def list_ros2_nodes(cls) -> List[str]:
        """ROS2 관련 노드 목록 반환"""
        return list(cls.NODE_TYPES.keys())


class ScriptNodeGenerator:
    """
    OmniGraph Script Node용 Python 코드 생성기

    Script Node에 삽입할 Python 코드를 템플릿 기반으로 생성합니다.
    """

    @staticmethod
    def generate_ik_solver_script(
        robot_prim_path: str,
        config_dir: str,
        robot_desc_file: str = "open_x1_des.yaml",
        urdf_file: str = "open_manipulator_x.urdf",
        end_effector_frame: str = "end_effector_link"
    ) -> str:
        """
        IK Solver Script Node 코드 생성

        Args:
            robot_prim_path: 로봇 Prim 경로
            config_dir: 설정 파일 디렉토리
            robot_desc_file: Lula robot description 파일명
            urdf_file: URDF 파일명
            end_effector_frame: 엔드이펙터 프레임 이름

        Returns:
            Script Node에 삽입할 Python 코드
        """
        return f'''"""
IK Solver Script Node - Auto Generated
Robot: {robot_prim_path}
End Effector: {end_effector_frame}
"""
import numpy as np

# Global state (initialized once)
_initialized = False
_ik_solver = None
_art_solver = None
_robot = None

def setup(db):
    """Initialize IK solver on first run"""
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

        config_dir = "{config_dir}"
        robot_path = "{robot_prim_path}"
        ee_frame = "{end_effector_frame}"

        world = World.instance()
        if world is None:
            return False

        # Get or create robot articulation
        _robot = world.scene.get_object("ik_target_robot")
        if _robot is None:
            _robot = world.scene.add(
                Articulation(prim_path=robot_path, name="ik_target_robot")
            )
            world.reset()

        # Initialize Lula IK Solver
        _ik_solver = LulaKinematicsSolver(
            robot_description_path=f"{{config_dir}}/{robot_desc_file}",
            urdf_path=f"{{config_dir}}/{urdf_file}"
        )

        # Initialize Articulation Kinematics Solver
        _art_solver = ArticulationKinematicsSolver(
            _robot, _ik_solver, ee_frame
        )

        _initialized = True
        print("[IK Script] Initialized successfully")
        return True

    except Exception as e:
        print(f"[IK Script] Setup error: {{e}}")
        return False

def compute(db):
    """Compute IK and apply to robot"""
    global _initialized, _art_solver, _robot, _ik_solver

    if not _initialized:
        if not setup(db):
            return

    if _art_solver is None or _robot is None:
        return

    try:
        # Get target position from input
        target_pos = db.inputs.targetPosition
        if target_pos is None or len(target_pos) < 3:
            return

        target = np.array([target_pos[0], target_pos[1], target_pos[2]])

        # Update robot base pose
        robot_base_pos, robot_base_ori = _robot.get_world_pose()
        _ik_solver.set_robot_base_pose(robot_base_pos, robot_base_ori)

        # Compute IK (position only for 4-DOF robot)
        action, success = _art_solver.compute_inverse_kinematics(
            target, target_orientation=None
        )

        if success and action is not None:
            _robot.apply_action(action)
            db.outputs.success = True
            db.outputs.jointPositions = action.joint_positions
        else:
            db.outputs.success = False

    except Exception as e:
        db.outputs.success = False
'''

    @staticmethod
    def generate_coordinate_transform_script(
        scale: float = 1.0,
        offset: List[float] = None
    ) -> str:
        """
        Quest → Isaac 좌표 변환 Script Node 코드 생성

        Args:
            scale: 스케일 팩터
            offset: 오프셋 [x, y, z]

        Returns:
            Script Node에 삽입할 Python 코드
        """
        offset_str = str(offset) if offset else "[0.0, 0.0, 0.0]"

        return f'''"""
Quest to Isaac Coordinate Transform Script
Quest (Unity): Y-up, Left-handed
Isaac (USD): Z-up, Right-handed

Transform: Isaac(X, Y, Z) = Quest(X, Z, Y)
"""
import numpy as np

SCALE = {scale}
OFFSET = np.array({offset_str})

def compute(db):
    """Transform Quest coordinates to Isaac coordinates"""
    try:
        # Get Quest position
        quest_pos = db.inputs.questPosition
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

        # Output
        db.outputs.isaacPosition = isaac_pos.tolist()

        # Transform quaternion if available
        if hasattr(db.inputs, 'questOrientation') and db.inputs.questOrientation is not None:
            quest_quat = db.inputs.questOrientation
            if len(quest_quat) >= 4:
                # Quaternion axis swap: [x, y, z, w] -> [x, z, y, w]
                isaac_quat = [quest_quat[0], quest_quat[2], quest_quat[1], quest_quat[3]]
                db.outputs.isaacOrientation = isaac_quat

    except Exception as e:
        pass  # Silently handle errors during runtime
'''

    @staticmethod
    def generate_gripper_control_script(
        min_position: float = -0.01,
        max_position: float = 0.02
    ) -> str:
        """
        그리퍼 제어 Script Node 코드 생성

        Args:
            min_position: 그리퍼 최소 위치 (닫힘)
            max_position: 그리퍼 최대 위치 (열림)

        Returns:
            Script Node에 삽입할 Python 코드
        """
        return f'''"""
Gripper Control Script
Maps trigger value (0.0 ~ 1.0) to gripper position

Trigger 0.0 (released) -> Gripper {max_position} (open)
Trigger 1.0 (pressed) -> Gripper {min_position} (closed)
"""
import numpy as np

MIN_POS = {min_position}
MAX_POS = {max_position}

def compute(db):
    """Map trigger input to gripper position"""
    try:
        # Get trigger value (0.0 = released, 1.0 = fully pressed)
        trigger = getattr(db.inputs, 'triggerValue', 0.0)
        if trigger is None:
            trigger = 0.0

        # Clamp trigger value
        trigger = max(0.0, min(1.0, float(trigger)))

        # Map trigger to gripper position
        # 0.0 -> MAX_POS (open)
        # 1.0 -> MIN_POS (closed)
        gripper_pos = MAX_POS - (trigger * (MAX_POS - MIN_POS))

        # Output
        db.outputs.gripperPosition = [gripper_pos]

    except Exception as e:
        db.outputs.gripperPosition = [MAX_POS]  # Default to open
'''


# =============================================================================
# Utility Functions
# =============================================================================

def create_simple_action_graph(
    graph_path: str,
    robot_prim_path: str,
    topic_namespace: str = ""
) -> Optional[Any]:
    """
    간단한 Action Graph 생성 (ROS2 기본 연결)

    Args:
        graph_path: 그래프 경로
        robot_prim_path: 로봇 Prim 경로
        topic_namespace: ROS2 토픽 네임스페이스

    Returns:
        그래프 핸들
    """
    builder = OmniGraphBuilder(graph_path)
    builder.remove_existing()

    # 기본 노드 추가
    builder.add_nodes([
        ("OnPlaybackTick", ROS2NodeFactory.get_node_type("on_playback_tick")),
        ("ROS2Context", ROS2NodeFactory.get_node_type("context")),
        ("PublishJointState", ROS2NodeFactory.get_node_type("publish_joint_state")),
        ("PublishClock", ROS2NodeFactory.get_node_type("publish_clock")),
        ("PublishTF", ROS2NodeFactory.get_node_type("publish_tf")),
        ("ReadSimTime", ROS2NodeFactory.get_node_type("read_sim_time")),
    ])

    ns = topic_namespace

    # 값 설정
    builder.set_values([
        ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ("PublishJointState.inputs:topicName", f"{ns}/joint_states"),
        ("PublishJointState.inputs:targetPrim", robot_prim_path),
        ("PublishClock.inputs:topicName", "/clock"),
        ("PublishTF.inputs:topicName", "/tf"),
        ("PublishTF.inputs:targetPrims", [robot_prim_path]),
    ])

    # 연결
    # Note: IsaacReadSimulationTime은 데이터 노드로 execIn이 없음
    builder.connect_many([
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
        # ReadSimTime은 execIn이 없음 - 매 틱마다 자동으로 시간 출력
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
    ])

    return builder.build()
