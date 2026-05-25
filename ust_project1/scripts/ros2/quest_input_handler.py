"""
ros2/quest_input_handler.py
Quest2ROS 입력 처리 핸들러

Quest2ROS 토픽을 구독하고 입력 데이터를 처리합니다.

Quest2ROS 토픽 구조:
    - /q2r_right_hand_pose (geometry_msgs/PoseStamped)
    - /q2r_left_hand_pose (geometry_msgs/PoseStamped)
    - /q2r_right_hand_twist (geometry_msgs/Twist)
    - /q2r_left_hand_twist (geometry_msgs/Twist)
    - /q2r_right_hand_inputs (Custom)
    - /q2r_left_hand_inputs (Custom)

Custom Input 구조:
    bool button_upper      # A/X 버튼
    bool button_lower      # B/Y 버튼
    float32 thumbstick_h   # 썸스틱 수평 (-1.0 ~ 1.0)
    float32 thumbstick_v   # 썸스틱 수직 (-1.0 ~ 1.0)
    float32 index_trigger  # 검지 트리거 (0.0 ~ 1.0)
    float32 middle_trigger # 중지 트리거 (0.0 ~ 1.0)

Author: UST Robotics Project
Date: 2024
"""

from dataclasses import dataclass, field
from typing import Optional, Callable, List
import numpy as np


@dataclass
class ControllerInput:
    """
    VR 컨트롤러 입력 데이터 클래스

    Quest2ROS에서 수신한 컨트롤러 입력을 저장합니다.

    Attributes:
        position: 컨트롤러 위치 [x, y, z]
        orientation: 컨트롤러 방향 [x, y, z, w]
        linear_velocity: 선속도 [x, y, z]
        angular_velocity: 각속도 [x, y, z]
        button_upper: A/X 버튼 상태
        button_lower: B/Y 버튼 상태
        thumbstick_h: 썸스틱 수평 (-1 ~ 1)
        thumbstick_v: 썸스틱 수직 (-1 ~ 1)
        index_trigger: 검지 트리거 (0 ~ 1)
        middle_trigger: 중지 트리거 (0 ~ 1)
        timestamp: 타임스탬프
    """
    # Pose
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 1]))

    # Twist
    linear_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Buttons
    button_upper: bool = False  # A/X
    button_lower: bool = False  # B/Y

    # Thumbstick
    thumbstick_h: float = 0.0
    thumbstick_v: float = 0.0

    # Triggers
    index_trigger: float = 0.0
    middle_trigger: float = 0.0

    # Timestamp
    timestamp: float = 0.0

    def to_dict(self) -> dict:
        """딕셔너리로 변환"""
        return {
            "position": self.position.tolist(),
            "orientation": self.orientation.tolist(),
            "linear_velocity": self.linear_velocity.tolist(),
            "angular_velocity": self.angular_velocity.tolist(),
            "button_upper": self.button_upper,
            "button_lower": self.button_lower,
            "thumbstick_h": self.thumbstick_h,
            "thumbstick_v": self.thumbstick_v,
            "index_trigger": self.index_trigger,
            "middle_trigger": self.middle_trigger,
            "timestamp": self.timestamp,
        }


class Quest2ROSInputHandler:
    """
    Quest2ROS 입력 핸들러

    Quest2ROS 토픽을 구독하고 입력 데이터를 관리합니다.

    Note:
        이 클래스는 Isaac Sim OmniGraph 외부에서 ROS2 노드로
        사용될 때의 참조 구현입니다. OmniGraph 내에서는
        ROS2Subscriber 노드를 직접 사용합니다.

    Attributes:
        right_controller: 오른쪽 컨트롤러 입력
        left_controller: 왼쪽 컨트롤러 입력

    Example:
        handler = Quest2ROSInputHandler()
        handler.start()

        # 메인 루프에서
        right = handler.right_controller
        if right.index_trigger > 0.5:
            gripper.close()
    """

    # 토픽 이름
    TOPICS = {
        "right_pose": "/q2r_right_hand_pose",
        "left_pose": "/q2r_left_hand_pose",
        "right_twist": "/q2r_right_hand_twist",
        "left_twist": "/q2r_left_hand_twist",
        "right_inputs": "/q2r_right_hand_inputs",
        "left_inputs": "/q2r_left_hand_inputs",
    }

    def __init__(self):
        """Quest2ROSInputHandler 초기화"""
        self.right_controller = ControllerInput()
        self.left_controller = ControllerInput()

        # 콜백 함수 저장
        self._callbacks: dict = {
            "right_pose": [],
            "left_pose": [],
            "right_trigger": [],
            "left_trigger": [],
            "right_button": [],
            "left_button": [],
        }

        # ROS2 노드 (실제 사용 시 초기화)
        self._node = None
        self._subscriptions = []

    def register_callback(
        self,
        event_type: str,
        callback: Callable
    ):
        """
        이벤트 콜백 등록

        Args:
            event_type: 이벤트 타입 ("right_pose", "right_trigger", etc.)
            callback: 콜백 함수
        """
        if event_type in self._callbacks:
            self._callbacks[event_type].append(callback)

    def _trigger_callbacks(self, event_type: str, data):
        """콜백 실행"""
        for callback in self._callbacks.get(event_type, []):
            try:
                callback(data)
            except Exception as e:
                print(f"[Quest2ROSInputHandler] Callback error: {e}")

    def update_right_pose(self, position: List[float], orientation: List[float], timestamp: float = 0.0):
        """
        오른쪽 컨트롤러 포즈 업데이트

        Args:
            position: [x, y, z]
            orientation: [x, y, z, w]
            timestamp: 타임스탬프
        """
        self.right_controller.position = np.array(position)
        self.right_controller.orientation = np.array(orientation)
        self.right_controller.timestamp = timestamp
        self._trigger_callbacks("right_pose", self.right_controller)

    def update_left_pose(self, position: List[float], orientation: List[float], timestamp: float = 0.0):
        """왼쪽 컨트롤러 포즈 업데이트"""
        self.left_controller.position = np.array(position)
        self.left_controller.orientation = np.array(orientation)
        self.left_controller.timestamp = timestamp
        self._trigger_callbacks("left_pose", self.left_controller)

    def update_right_twist(self, linear: List[float], angular: List[float]):
        """오른쪽 컨트롤러 속도 업데이트"""
        self.right_controller.linear_velocity = np.array(linear)
        self.right_controller.angular_velocity = np.array(angular)

    def update_left_twist(self, linear: List[float], angular: List[float]):
        """왼쪽 컨트롤러 속도 업데이트"""
        self.left_controller.linear_velocity = np.array(linear)
        self.left_controller.angular_velocity = np.array(angular)

    def update_right_inputs(
        self,
        button_upper: bool,
        button_lower: bool,
        thumbstick_h: float,
        thumbstick_v: float,
        index_trigger: float,
        middle_trigger: float
    ):
        """
        오른쪽 컨트롤러 버튼/트리거 업데이트

        Args:
            button_upper: A 버튼
            button_lower: B 버튼
            thumbstick_h: 썸스틱 수평
            thumbstick_v: 썸스틱 수직
            index_trigger: 검지 트리거
            middle_trigger: 중지 트리거
        """
        old_trigger = self.right_controller.index_trigger

        self.right_controller.button_upper = button_upper
        self.right_controller.button_lower = button_lower
        self.right_controller.thumbstick_h = thumbstick_h
        self.right_controller.thumbstick_v = thumbstick_v
        self.right_controller.index_trigger = index_trigger
        self.right_controller.middle_trigger = middle_trigger

        # 트리거 변화 감지
        if abs(index_trigger - old_trigger) > 0.1:
            self._trigger_callbacks("right_trigger", index_trigger)

    def update_left_inputs(
        self,
        button_upper: bool,
        button_lower: bool,
        thumbstick_h: float,
        thumbstick_v: float,
        index_trigger: float,
        middle_trigger: float
    ):
        """왼쪽 컨트롤러 버튼/트리거 업데이트"""
        self.left_controller.button_upper = button_upper
        self.left_controller.button_lower = button_lower
        self.left_controller.thumbstick_h = thumbstick_h
        self.left_controller.thumbstick_v = thumbstick_v
        self.left_controller.index_trigger = index_trigger
        self.left_controller.middle_trigger = middle_trigger

    def is_clutch_active(self) -> bool:
        """
        클러치 활성화 확인 (중지 트리거)

        Returns:
            클러치 활성화 여부
        """
        return self.right_controller.middle_trigger > 0.5

    def get_mobile_command(self, speed_scale: float = 1.0) -> tuple:
        """
        모바일 베이스 명령 계산

        왼쪽 컨트롤러 thumbstick을 cmd_vel로 변환합니다.

        Args:
            speed_scale: 속도 스케일 (0~1)

        Returns:
            (linear_x, angular_z) 튜플
        """
        # 검지 트리거로 속도 조절
        trigger_scale = 0.1 + (self.left_controller.index_trigger * 0.9)
        total_scale = speed_scale * trigger_scale

        linear_x = self.left_controller.thumbstick_v * 0.5 * total_scale
        angular_z = -self.left_controller.thumbstick_h * 1.0 * total_scale

        # 데드존 적용
        if abs(linear_x) < 0.01:
            linear_x = 0.0
        if abs(angular_z) < 0.01:
            angular_z = 0.0

        return linear_x, angular_z

    def get_gripper_command(self) -> float:
        """
        그리퍼 명령 계산

        오른쪽 검지 트리거를 그리퍼 위치로 변환합니다.

        Returns:
            그리퍼 트리거 값 (0=열림, 1=닫힘)
        """
        return self.right_controller.index_trigger

    def is_emergency_stop_pressed(self) -> bool:
        """긴급 정지 버튼 확인 (왼쪽 X 버튼)"""
        return self.left_controller.button_upper

    def is_home_position_requested(self) -> bool:
        """홈 포지션 요청 확인 (오른쪽 A 버튼)"""
        return self.right_controller.button_upper

    def is_ready_position_requested(self) -> bool:
        """Ready 포지션 요청 확인 (오른쪽 B 버튼)"""
        return self.right_controller.button_lower

    def get_status(self) -> dict:
        """현재 상태 요약 반환"""
        return {
            "right": self.right_controller.to_dict(),
            "left": self.left_controller.to_dict(),
            "clutch_active": self.is_clutch_active(),
            "emergency_stop": self.is_emergency_stop_pressed(),
        }
