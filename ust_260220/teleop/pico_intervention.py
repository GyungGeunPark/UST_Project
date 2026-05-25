"""PICO 4 Ultra 컨트롤러 기반 HG-DAgger 개입 인터페이스.

Quest 3S의 OpenXR 제스처 대신 XRoboToolkit JSON의 컨트롤러 버튼을 사용.

개입 트리거:
  양손 그립 동시 누름 (grip > threshold) → 개입 시작
  양손 트리거 동시 누름 (trigger > threshold) → 자율 복귀
  양손 메뉴 동시 누름 → 에피소드 리셋 (선택)
"""

from __future__ import annotations

from typing import Optional

from .pico_fullbody_device import PICOFullBodyTeleopDevice


class PICOInterventionInterface:
    """PICO 컨트롤러 버튼 기반 개입 인터페이스.

    InterventionManager와 함께 사용:
        interface = PICOInterventionInterface(pico_device)
        manager = InterventionManager(policy, idle_action)

        while not done:
            teleop_action = pico_device.advance()
            intervention_trigger = interface.check_intervention_trigger()
            resume_trigger = interface.check_resume_trigger()
            action, is_intervention, iid = manager.step(
                obs, teleop_action, intervention_trigger, resume_trigger
            )
    """

    def __init__(
        self,
        device: PICOFullBodyTeleopDevice,
        grip_threshold: float = 0.8,
        trigger_threshold: float = 0.8,
        debounce_frames: int = 3,
    ):
        """
        Args:
            device: PICO 텔레오퍼레이션 디바이스
            grip_threshold: 그립 트리거 임계값 (개입 시작)
            trigger_threshold: 트리거 임계값 (자율 복귀)
            debounce_frames: 디바운스 프레임 수 (연속 N프레임 유지 시 트리거)
        """
        self.device = device
        self.grip_threshold = grip_threshold
        self.trigger_threshold = trigger_threshold
        self.debounce_frames = debounce_frames

        self._grip_count = 0
        self._trigger_count = 0
        self._reset_count = 0

    def check_intervention_trigger(self) -> bool:
        """개입 시작: 양손 그립 동시 누름 (디바운스 적용)."""
        ctrl = self.device.get_controller_data()
        if ctrl is None:
            self._grip_count = 0
            return False

        left = ctrl["left"]
        right = ctrl["right"]

        if left.grip > self.grip_threshold and right.grip > self.grip_threshold:
            self._grip_count += 1
        else:
            self._grip_count = 0

        return self._grip_count >= self.debounce_frames

    def check_resume_trigger(self) -> bool:
        """자율 복귀: 양손 트리거 동시 누름 (디바운스 적용)."""
        ctrl = self.device.get_controller_data()
        if ctrl is None:
            self._trigger_count = 0
            return False

        left = ctrl["left"]
        right = ctrl["right"]

        if left.trigger > self.trigger_threshold and right.trigger > self.trigger_threshold:
            self._trigger_count += 1
        else:
            self._trigger_count = 0

        return self._trigger_count >= self.debounce_frames

    def check_reset_trigger(self) -> bool:
        """에피소드 리셋: 양손 메뉴 동시 누름."""
        ctrl = self.device.get_controller_data()
        if ctrl is None:
            self._reset_count = 0
            return False

        left = ctrl["left"]
        right = ctrl["right"]

        if left.menu_button and right.menu_button:
            self._reset_count += 1
        else:
            self._reset_count = 0

        return self._reset_count >= self.debounce_frames

    def reset(self):
        """디바운스 카운터 리셋."""
        self._grip_count = 0
        self._trigger_count = 0
        self._reset_count = 0
