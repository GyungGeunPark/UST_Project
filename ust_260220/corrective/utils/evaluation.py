"""정책 평가 유틸리티.

Phase 1~3 공용 평가 지표 및 평가 루프.
"""

from __future__ import annotations

import numpy as np
import torch
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class EvaluationMetrics:
    """평가 결과 메트릭."""
    num_episodes: int = 0
    success_rate: float = 0.0
    avg_sorted_objects: float = 0.0
    avg_episode_length: float = 0.0
    avg_reward: float = 0.0
    # Phase 2
    avg_intervention_ratio: float = 0.0
    # Phase 3
    avg_uncertainty: float = 0.0
    help_request_rate: float = 0.0
    # 상세
    per_object_success: Dict[str, float] = field(default_factory=dict)

    def summary(self) -> str:
        lines = [
            f"=== 평가 결과 ({self.num_episodes}개 에피소드) ===",
            f"  성공률: {self.success_rate:.1%}",
            f"  평균 분류 물체: {self.avg_sorted_objects:.1f}/8",
            f"  평균 에피소드 길이: {self.avg_episode_length:.0f} 스텝",
            f"  평균 보상: {self.avg_reward:.2f}",
        ]
        if self.avg_intervention_ratio > 0:
            lines.append(f"  평균 개입 비율: {self.avg_intervention_ratio:.1%}")
        if self.help_request_rate > 0:
            lines.append(f"  도움 요청률: {self.help_request_rate:.1%}")
        if self.per_object_success:
            lines.append("  물체별 성공률:")
            for obj, rate in self.per_object_success.items():
                lines.append(f"    {obj}: {rate:.1%}")
        lines.append("=" * 40)
        return "\n".join(lines)


class PolicyEvaluator:
    """정책 평가기.

    Isaac Lab 환경에서 정책을 실행하고 성능 지표를 수집합니다.
    """

    OBJECT_NAMES = ["mug", "plate", "bowl", "can", "bottle", "apple", "sponge", "teddy"]

    def __init__(self, env, idle_action: torch.Tensor):
        self.env = env
        self.idle_action = idle_action

    def evaluate(
        self,
        policy,
        num_episodes: int = 50,
        max_steps_per_episode: int = 2400,
    ) -> EvaluationMetrics:
        """정책 평가 실행.

        Args:
            policy: .predict(obs) -> action 메서드를 가진 정책 객체
            num_episodes: 평가 에피소드 수
            max_steps_per_episode: 에피소드당 최대 스텝

        Returns:
            EvaluationMetrics 결과
        """
        all_successes = []
        all_sorted_counts = []
        all_lengths = []
        all_rewards = []
        per_object_success = {obj: [] for obj in self.OBJECT_NAMES}

        for ep in range(num_episodes):
            obs, info = self.env.reset()
            episode_reward = 0.0
            step = 0

            for step in range(max_steps_per_episode):
                with torch.inference_mode():
                    action = policy.predict(obs)
                    if action is None:
                        action = self.idle_action
                    if action.dim() == 1:
                        action = action.unsqueeze(0)
                    action = action.to(self.env.device)

                    obs, rewards, terminated, truncated, info = self.env.step(action)
                    episode_reward += rewards.mean().item()

                    if terminated.any() or truncated.any():
                        break

            # 분류 결과 확인
            from ust_ws.ust_260220.mdp.terminations import (
                all_objects_sorted, count_sorted_objects, object_in_correct_bin,
            )
            success = all_objects_sorted(self.env).any().item()
            sorted_count = count_sorted_objects(self.env).mean().item()

            all_successes.append(success)
            all_sorted_counts.append(sorted_count)
            all_lengths.append(step + 1)
            all_rewards.append(episode_reward)

            # 물체별 분류 확인
            for obj_name in self.OBJECT_NAMES:
                try:
                    in_bin = object_in_correct_bin(self.env, obj_name)
                    per_object_success[obj_name].append(in_bin.any().item())
                except (KeyError, AttributeError):
                    pass

            if (ep + 1) % 10 == 0:
                print(f"  평가 진행: {ep+1}/{num_episodes}, "
                      f"현재 성공률: {np.mean(all_successes):.1%}")

        metrics = EvaluationMetrics(
            num_episodes=num_episodes,
            success_rate=float(np.mean(all_successes)),
            avg_sorted_objects=float(np.mean(all_sorted_counts)),
            avg_episode_length=float(np.mean(all_lengths)),
            avg_reward=float(np.mean(all_rewards)),
            per_object_success={
                obj: float(np.mean(vals)) if vals else 0.0
                for obj, vals in per_object_success.items()
            },
        )

        return metrics
