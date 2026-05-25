"""HG-DAgger 개입 관리자.

정책 자율 실행 중 사람의 VR 텔레오퍼레이션 개입을 관리합니다.

상태 머신:
  AUTONOMOUS → INTERVENING (개입 트리거)
  INTERVENING → AUTONOMOUS (복귀 트리거)
"""

from __future__ import annotations

import time
import torch
from enum import Enum
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


class InterventionState(Enum):
    """교정 개입 상태."""

    AUTONOMOUS = "autonomous"
    MONITORING = "monitoring"
    INTERVENING = "intervening"
    RESUMING = "resuming"


@dataclass
class InterventionStats:
    """개입 통계."""

    total_steps: int = 0
    autonomous_steps: int = 0
    intervention_steps: int = 0
    num_interventions: int = 0
    intervention_triggers: List[Tuple[int, str]] = field(default_factory=list)
    episode_count: int = 0


class InterventionManager:
    """HG-DAgger 개입 관리자.

    자율 정책 실행 중 사람이 VR 텔레오퍼레이션으로 교정할 수 있도록
    제어권 인계/복귀를 관리합니다.

    개입 트리거:
    - VR 컨트롤러 양손 그립 동시 누름 → 개입 시작
    - VR 컨트롤러 양손 트리거 동시 누름 → 자율 복귀

    사용법:
        manager = InterventionManager(policy, idle_action)
        while not done:
            action, is_intervention = manager.step(obs, teleop_action)
            obs, reward, done, info = env.step(action)
    """

    def __init__(
        self,
        policy,
        idle_action: torch.Tensor,
        grace_period_steps: int = 10,
    ):
        """
        Args:
            policy: .predict(obs) -> action 메서드를 가진 정책 객체
            idle_action: 대기 액션 (38D)
            grace_period_steps: 개입 후 자율 복귀 유예 스텝
        """
        self.policy = policy
        self.idle_action = idle_action
        self.grace_period = grace_period_steps

        self.state = InterventionState.AUTONOMOUS
        self.stats = InterventionStats()

        self._current_intervention_id = -1
        self._grace_counter = 0
        self._intervention_start_step = 0

    def step(
        self,
        obs: torch.Tensor,
        teleop_action: Optional[torch.Tensor] = None,
        intervention_trigger: bool = False,
        resume_trigger: bool = False,
    ) -> Tuple[torch.Tensor, bool, int]:
        """한 스텝 실행.

        Args:
            obs: 현재 관찰
            teleop_action: VR 텔레오퍼레이션 액션 (개입 중일 때 사용)
            intervention_trigger: 개입 시작 신호
            resume_trigger: 자율 복귀 신호

        Returns:
            action: 실행할 액션 (38D)
            is_intervention: 현재 개입 중 여부
            intervention_id: 현재 개입 세션 ID (-1 = 자율)
        """
        self.stats.total_steps += 1

        if self.state == InterventionState.AUTONOMOUS:
            # 정책에서 액션 생성
            action = self.policy.predict(obs)
            if action is None:
                action = self.idle_action

            # 개입 트리거 확인
            if intervention_trigger:
                self._start_intervention("human_triggered")
                if teleop_action is not None:
                    action = teleop_action
                    self.stats.intervention_steps += 1
                    return action, True, self._current_intervention_id

            self.stats.autonomous_steps += 1
            return action, False, -1

        elif self.state == InterventionState.INTERVENING:
            # 사람의 VR 텔레오퍼레이션 액션 사용
            if teleop_action is not None:
                action = teleop_action
            else:
                # 텔레옵 액션 없으면 대기
                action = self.idle_action

            # 복귀 트리거 확인
            if resume_trigger:
                self._end_intervention()
                self.state = InterventionState.RESUMING
                self._grace_counter = self.grace_period

            self.stats.intervention_steps += 1
            return action, True, self._current_intervention_id

        elif self.state == InterventionState.RESUMING:
            # 유예 기간 (부드러운 전환)
            self._grace_counter -= 1

            if teleop_action is not None and self._grace_counter > 0:
                # 유예 기간 중에도 사람 액션 우선
                action = teleop_action
                is_intervention = True
            else:
                action = self.policy.predict(obs)
                if action is None:
                    action = self.idle_action
                is_intervention = False

            if self._grace_counter <= 0:
                self.state = InterventionState.AUTONOMOUS

            if is_intervention:
                self.stats.intervention_steps += 1
            else:
                self.stats.autonomous_steps += 1

            return action, is_intervention, (
                self._current_intervention_id if is_intervention else -1
            )

        # fallback
        return self.idle_action, False, -1

    def _start_intervention(self, reason: str):
        """개입 시작."""
        self.state = InterventionState.INTERVENING
        self._current_intervention_id += 1
        self.stats.num_interventions += 1
        self._intervention_start_step = self.stats.total_steps
        self.stats.intervention_triggers.append(
            (self.stats.total_steps, reason)
        )
        print(
            f"[INTERVENTION #{self.stats.num_interventions}] "
            f"시작 (step {self.stats.total_steps}): {reason}"
        )

    def _end_intervention(self):
        """개입 종료."""
        duration = self.stats.total_steps - self._intervention_start_step
        print(
            f"[INTERVENTION #{self.stats.num_interventions}] "
            f"종료. 지속: {duration} 스텝"
        )

    def force_start_intervention(self, reason: str = "forced"):
        """강제 개입 시작 (외부 호출)."""
        if self.state != InterventionState.INTERVENING:
            self._start_intervention(reason)

    def force_end_intervention(self):
        """강제 개입 종료 (외부 호출)."""
        if self.state == InterventionState.INTERVENING:
            self._end_intervention()
            self.state = InterventionState.AUTONOMOUS

    def reset_episode(self):
        """에피소드 리셋."""
        self.state = InterventionState.AUTONOMOUS
        self._grace_counter = 0
        self.stats.episode_count += 1
        if hasattr(self.policy, "reset"):
            self.policy.reset()

    @property
    def is_intervening(self) -> bool:
        return self.state == InterventionState.INTERVENING

    @property
    def intervention_ratio(self) -> float:
        if self.stats.total_steps == 0:
            return 0.0
        return self.stats.intervention_steps / self.stats.total_steps

    def get_summary(self) -> str:
        """통계 요약."""
        return (
            f"=== 개입 통계 ===\n"
            f"  총 스텝: {self.stats.total_steps}\n"
            f"  자율 스텝: {self.stats.autonomous_steps}\n"
            f"  개입 스텝: {self.stats.intervention_steps}\n"
            f"  개입 횟수: {self.stats.num_interventions}\n"
            f"  개입 비율: {self.intervention_ratio:.1%}\n"
            f"  에피소드: {self.stats.episode_count}\n"
            f"================"
        )
