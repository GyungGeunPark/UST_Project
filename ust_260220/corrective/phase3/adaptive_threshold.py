"""적응적 불확실성 임계값 (ThriftyDAgger 영감).

실행 중 불확실성 분포와 목표 개입률에 따라
임계값을 자동으로 조정합니다.
"""

from __future__ import annotations

import numpy as np
from typing import List


class AdaptiveThreshold:
    """적응적 불확실성 임계값.

    현재 도움 요청 비율이 목표보다 높으면 임계값을 올리고 (덜 요청),
    낮으면 내려서 (더 요청) 목표 비율에 수렴합니다.

    ThriftyDAgger (Hoque et al., CoRL 2022) 접근법에 영감을 받았습니다.

    사용법:
        threshold = AdaptiveThreshold(initial=0.05, target=0.15)
        for step in steps:
            if threshold.should_request_help(uncertainty):
                # 도움 요청
            # 주기적 조정
            threshold.adapt()
    """

    def __init__(
        self,
        initial_threshold: float = 0.05,
        target_intervention_ratio: float = 0.15,
        adaptation_rate: float = 0.01,
        min_threshold: float = 0.001,
        max_threshold: float = 1.0,
        adaptation_interval: int = 100,
    ):
        """
        Args:
            initial_threshold: 초기 임계값
            target_intervention_ratio: 목표 개입률 (기본 15%)
            adaptation_rate: 임계값 조정 속도
            min_threshold: 최소 임계값
            max_threshold: 최대 임계값
            adaptation_interval: 적응 간격 (스텝 수)
        """
        self.threshold = initial_threshold
        self.target_ratio = target_intervention_ratio
        self.alpha = adaptation_rate
        self.min_threshold = min_threshold
        self.max_threshold = max_threshold
        self.adaptation_interval = adaptation_interval

        # 통계
        self.total_steps = 0
        self.help_requests = 0
        self.uncertainty_values: List[float] = []
        self.threshold_history: List[float] = [initial_threshold]
        self._steps_since_adapt = 0

    def should_request_help(self, uncertainty: float) -> bool:
        """도움 요청 여부 판단.

        Args:
            uncertainty: 현재 불확실성 값

        Returns:
            도움 요청 여부
        """
        self.total_steps += 1
        self._steps_since_adapt += 1
        self.uncertainty_values.append(uncertainty)

        if uncertainty > self.threshold:
            self.help_requests += 1

            # 주기적 자동 적응
            if self._steps_since_adapt >= self.adaptation_interval:
                self.adapt()

            return True

        # 주기적 자동 적응
        if self._steps_since_adapt >= self.adaptation_interval:
            self.adapt()

        return False

    def adapt(self):
        """현재 개입률에 따라 임계값 조정."""
        if self.total_steps == 0:
            return

        current_ratio = self.help_requests / self.total_steps

        old_threshold = self.threshold

        if current_ratio > self.target_ratio:
            # 너무 많이 요청 → 임계값 올림
            self.threshold *= (1.0 + self.alpha)
        elif current_ratio < self.target_ratio:
            # 너무 적게 요청 → 임계값 내림
            self.threshold *= (1.0 - self.alpha)

        # 범위 제한
        self.threshold = np.clip(
            self.threshold, self.min_threshold, self.max_threshold
        )

        self.threshold_history.append(self.threshold)
        self._steps_since_adapt = 0

        if abs(self.threshold - old_threshold) > 1e-6:
            print(
                f"[Threshold] {old_threshold:.4f} → {self.threshold:.4f} "
                f"(현재 비율: {current_ratio:.2%}, 목표: {self.target_ratio:.2%})"
            )

    def reset_stats(self):
        """에피소드 통계 리셋 (임계값은 유지)."""
        self.total_steps = 0
        self.help_requests = 0
        self.uncertainty_values.clear()
        self._steps_since_adapt = 0

    def get_stats(self) -> dict:
        """통계."""
        unc_arr = np.array(self.uncertainty_values) if self.uncertainty_values else np.array([0.0])
        return {
            "threshold": self.threshold,
            "target_ratio": self.target_ratio,
            "current_ratio": (
                self.help_requests / self.total_steps
                if self.total_steps > 0
                else 0.0
            ),
            "total_steps": self.total_steps,
            "help_requests": self.help_requests,
            "uncertainty_mean": float(unc_arr.mean()),
            "uncertainty_std": float(unc_arr.std()),
            "threshold_changes": len(self.threshold_history) - 1,
        }
