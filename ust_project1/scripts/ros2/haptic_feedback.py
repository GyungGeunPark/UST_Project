"""
ros2/haptic_feedback.py
Quest2ROS 햅틱 피드백 퍼블리셔

Quest VR 컨트롤러에 햅틱(진동) 피드백을 전송합니다.

햅틱 피드백 토픽:
    - /q2r_right_hand_haptic_feedback
    - /q2r_left_hand_haptic_feedback

메시지 구조:
    float32 frequency   # 진동 주파수 (Hz)
    float32 amplitude   # 진동 강도 (0.0 ~ 1.0)
    float32 duration    # 지속 시간 (seconds)

Author: UST Robotics Project
Date: 2024
"""

from dataclasses import dataclass
from typing import Optional
from enum import Enum


class HapticHand(Enum):
    """컨트롤러 손 구분"""
    RIGHT = "right"
    LEFT = "left"


@dataclass
class HapticFeedback:
    """
    햅틱 피드백 데이터

    Attributes:
        frequency: 진동 주파수 (Hz, 일반적으로 100-500)
        amplitude: 진동 강도 (0.0 ~ 1.0)
        duration: 지속 시간 (초)
    """
    frequency: float = 200.0
    amplitude: float = 0.5
    duration: float = 0.1

    def __post_init__(self):
        # 값 클램핑
        self.frequency = max(0.0, min(1000.0, self.frequency))
        self.amplitude = max(0.0, min(1.0, self.amplitude))
        self.duration = max(0.0, min(10.0, self.duration))


class HapticPattern:
    """
    사전 정의된 햅틱 패턴

    자주 사용되는 햅틱 피드백 패턴을 제공합니다.
    """

    @staticmethod
    def light_tap() -> HapticFeedback:
        """가벼운 탭"""
        return HapticFeedback(frequency=200, amplitude=0.3, duration=0.05)

    @staticmethod
    def medium_tap() -> HapticFeedback:
        """중간 탭"""
        return HapticFeedback(frequency=250, amplitude=0.5, duration=0.1)

    @staticmethod
    def strong_tap() -> HapticFeedback:
        """강한 탭"""
        return HapticFeedback(frequency=300, amplitude=0.8, duration=0.15)

    @staticmethod
    def success() -> HapticFeedback:
        """성공 피드백"""
        return HapticFeedback(frequency=200, amplitude=0.6, duration=0.2)

    @staticmethod
    def error() -> HapticFeedback:
        """에러 피드백"""
        return HapticFeedback(frequency=400, amplitude=0.9, duration=0.3)

    @staticmethod
    def warning() -> HapticFeedback:
        """경고 피드백"""
        return HapticFeedback(frequency=350, amplitude=0.7, duration=0.15)

    @staticmethod
    def collision() -> HapticFeedback:
        """충돌 피드백"""
        return HapticFeedback(frequency=500, amplitude=1.0, duration=0.1)

    @staticmethod
    def grip_contact() -> HapticFeedback:
        """그립 접촉 피드백"""
        return HapticFeedback(frequency=150, amplitude=0.4, duration=0.05)

    @staticmethod
    def workspace_limit() -> HapticFeedback:
        """작업 공간 한계 피드백"""
        return HapticFeedback(frequency=300, amplitude=0.6, duration=0.2)


class HapticFeedbackPublisher:
    """
    햅틱 피드백 퍼블리셔

    Quest2ROS를 통해 VR 컨트롤러에 햅틱 피드백을 전송합니다.

    Note:
        이 클래스는 ROS2 노드 없이도 피드백 데이터를 준비할 수 있습니다.
        실제 퍼블리시는 OmniGraph ROS2Publisher 노드 또는
        외부 ROS2 노드를 통해 수행됩니다.

    Example:
        publisher = HapticFeedbackPublisher()
        publisher.vibrate_right(HapticPattern.success())
        publisher.vibrate_on_collision(HapticHand.LEFT)
    """

    # 토픽 이름
    TOPICS = {
        HapticHand.RIGHT: "/q2r_right_hand_haptic_feedback",
        HapticHand.LEFT: "/q2r_left_hand_haptic_feedback",
    }

    def __init__(self):
        """HapticFeedbackPublisher 초기화"""
        # 퍼블리시 큐 (실제 퍼블리시 전 저장)
        self._pending_right: Optional[HapticFeedback] = None
        self._pending_left: Optional[HapticFeedback] = None

        # 마지막 피드백 시간 (중복 방지용)
        self._last_right_time: float = 0.0
        self._last_left_time: float = 0.0

        # 최소 피드백 간격 (초)
        self._min_interval: float = 0.05

    def vibrate(self, hand: HapticHand, feedback: HapticFeedback):
        """
        햅틱 피드백 전송

        Args:
            hand: 컨트롤러 손 (RIGHT/LEFT)
            feedback: 햅틱 피드백 데이터
        """
        if hand == HapticHand.RIGHT:
            self._pending_right = feedback
        else:
            self._pending_left = feedback

    def vibrate_right(self, feedback: HapticFeedback):
        """오른쪽 컨트롤러 진동"""
        self._pending_right = feedback

    def vibrate_left(self, feedback: HapticFeedback):
        """왼쪽 컨트롤러 진동"""
        self._pending_left = feedback

    def vibrate_both(self, feedback: HapticFeedback):
        """양쪽 컨트롤러 동시 진동"""
        self._pending_right = feedback
        self._pending_left = feedback

    def vibrate_on_collision(self, hand: HapticHand = HapticHand.RIGHT):
        """충돌 피드백"""
        self.vibrate(hand, HapticPattern.collision())

    def vibrate_on_grip(self, hand: HapticHand = HapticHand.RIGHT):
        """그립 접촉 피드백"""
        self.vibrate(hand, HapticPattern.grip_contact())

    def vibrate_on_workspace_limit(self, hand: HapticHand = HapticHand.RIGHT):
        """작업 공간 한계 피드백"""
        self.vibrate(hand, HapticPattern.workspace_limit())

    def vibrate_success(self, hand: HapticHand = HapticHand.RIGHT):
        """성공 피드백"""
        self.vibrate(hand, HapticPattern.success())

    def vibrate_error(self, hand: HapticHand = HapticHand.RIGHT):
        """에러 피드백"""
        self.vibrate(hand, HapticPattern.error())

    def get_pending_right(self) -> Optional[HapticFeedback]:
        """대기 중인 오른쪽 피드백 가져오기 및 초기화"""
        feedback = self._pending_right
        self._pending_right = None
        return feedback

    def get_pending_left(self) -> Optional[HapticFeedback]:
        """대기 중인 왼쪽 피드백 가져오기 및 초기화"""
        feedback = self._pending_left
        self._pending_left = None
        return feedback

    def has_pending(self) -> bool:
        """대기 중인 피드백이 있는지 확인"""
        return self._pending_right is not None or self._pending_left is not None

    def clear_pending(self):
        """대기 중인 모든 피드백 초기화"""
        self._pending_right = None
        self._pending_left = None

    @staticmethod
    def get_topic(hand: HapticHand) -> str:
        """토픽 이름 반환"""
        return HapticFeedbackPublisher.TOPICS[hand]


# =============================================================================
# Proportional Haptic Feedback
# =============================================================================

class ProportionalHapticController:
    """
    비례 햅틱 피드백 컨트롤러

    거리, 힘 등의 연속적인 값에 비례하여 햅틱 피드백을 생성합니다.
    """

    def __init__(
        self,
        min_amplitude: float = 0.1,
        max_amplitude: float = 1.0,
        frequency: float = 200.0
    ):
        """
        Args:
            min_amplitude: 최소 진폭
            max_amplitude: 최대 진폭
            frequency: 기본 주파수
        """
        self.min_amplitude = min_amplitude
        self.max_amplitude = max_amplitude
        self.frequency = frequency

    def from_distance(
        self,
        current_distance: float,
        threshold_distance: float,
        duration: float = 0.05
    ) -> Optional[HapticFeedback]:
        """
        거리 기반 햅틱 피드백

        threshold 이하일 때 거리에 반비례하여 강도 증가

        Args:
            current_distance: 현재 거리
            threshold_distance: 임계 거리
            duration: 지속 시간

        Returns:
            HapticFeedback 또는 None (threshold 초과 시)
        """
        if current_distance >= threshold_distance:
            return None

        # 거리 비율 (0: threshold, 1: 0거리)
        ratio = 1.0 - (current_distance / threshold_distance)
        ratio = max(0.0, min(1.0, ratio))

        amplitude = self.min_amplitude + (ratio * (self.max_amplitude - self.min_amplitude))

        return HapticFeedback(
            frequency=self.frequency,
            amplitude=amplitude,
            duration=duration
        )

    def from_force(
        self,
        force: float,
        max_force: float,
        duration: float = 0.05
    ) -> HapticFeedback:
        """
        힘 기반 햅틱 피드백

        Args:
            force: 현재 힘
            max_force: 최대 힘 (이 값에서 max_amplitude)
            duration: 지속 시간

        Returns:
            HapticFeedback
        """
        ratio = min(abs(force) / max_force, 1.0)
        amplitude = self.min_amplitude + (ratio * (self.max_amplitude - self.min_amplitude))

        return HapticFeedback(
            frequency=self.frequency,
            amplitude=amplitude,
            duration=duration
        )
