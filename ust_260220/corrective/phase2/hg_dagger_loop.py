"""HG-DAgger 교정 루프.

정책 실행 → 개입 수집 → IWR 재학습 → 평가의 반복 루프입니다.

프로토콜:
  Iteration 0: 기준선 평가 (Phase 1 BC-RNN)
  Iteration 1+: 정책실행 → 개입수집(20ep) → IWR재학습 → 평가(50ep)
"""

from __future__ import annotations

import time
import torch
import numpy as np
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .intervention_manager import InterventionManager
from .iwr_trainer import IWRTrainer, IWRDataset, IWRConfig
from ..utils.corrective_hdf5 import (
    CorrectiveHDF5Recorder,
    LABEL_AUTONOMOUS,
    LABEL_INTERVENTION,
)
from ..utils.evaluation import PolicyEvaluator, EvaluationMetrics


@dataclass
class HGDAggerConfig:
    """HG-DAgger 루프 설정."""

    # 반복 설정
    max_iterations: int = 5
    target_success_rate: float = 0.8

    # 데이터 수집
    episodes_per_iteration: int = 20
    max_steps_per_episode: int = 1200  # 60초 × 20Hz

    # 평가
    eval_episodes: int = 50

    # IWR 학습
    iwr_config: IWRConfig = None

    # 데이터 저장
    data_dir: str = "./data/corrective/"
    model_dir: str = "./models/hg_dagger/"

    # 수렴 조건
    min_improvement: float = 0.01  # 최소 성공률 개선

    def __post_init__(self):
        if self.iwr_config is None:
            self.iwr_config = IWRConfig()


class HGDAggerLoop:
    """HG-DAgger 교정 루프 실행기.

    Phase 1 BC-RNN 정책을 시작으로, 사람의 교정 개입을 통해
    정책 성능을 점진적으로 향상시킵니다.

    사용법:
        loop = HGDAggerLoop(env, policy, teleop_device, config)
        final_policy, metrics = loop.run()
    """

    def __init__(
        self,
        env,
        policy,
        idle_action: torch.Tensor,
        config: Optional[HGDAggerConfig] = None,
    ):
        """
        Args:
            env: Isaac Lab 환경
            policy: 초기 BC-RNN 정책
            idle_action: 대기 액션 (38D)
            config: HG-DAgger 설정
        """
        self.env = env
        self.policy = policy
        self.idle_action = idle_action
        self.config = config or HGDAggerConfig()

        # 모듈 초기화
        self.intervention_manager = InterventionManager(
            policy=policy,
            idle_action=idle_action,
        )
        self.iwr_trainer = IWRTrainer(self.config.iwr_config)
        self.evaluator = PolicyEvaluator(env, idle_action)

        # 결과 추적
        self.iteration_results = []
        self.all_data_paths = []

        Path(self.config.data_dir).mkdir(parents=True, exist_ok=True)
        Path(self.config.model_dir).mkdir(parents=True, exist_ok=True)

    def run(self) -> tuple:
        """HG-DAgger 루프 실행.

        Returns:
            policy: 최종 학습된 정책
            results: 반복별 결과 리스트
        """
        print("\n" + "=" * 60)
        print("  HG-DAgger 교정 루프 시작")
        print("=" * 60)

        # Iteration 0: 기준선 평가
        print("\n--- Iteration 0: 기준선 평가 ---")
        baseline_metrics = self.evaluator.evaluate(
            self.policy,
            num_episodes=self.config.eval_episodes,
        )
        print(baseline_metrics.summary())
        self.iteration_results.append({
            "iteration": 0,
            "type": "baseline",
            "metrics": baseline_metrics,
            "data_path": None,
        })

        # 교정 반복
        for iteration in range(1, self.config.max_iterations + 1):
            print(f"\n--- Iteration {iteration}/{self.config.max_iterations} ---")

            # 1. 데이터 수집 (정책 실행 + 개입)
            data_path = self._collect_corrective_data(iteration)
            self.all_data_paths.append(data_path)

            # 2. IWR 재학습
            self._retrain_with_iwr(iteration)

            # 3. 평가
            eval_metrics = self.evaluator.evaluate(
                self.policy,
                num_episodes=self.config.eval_episodes,
            )
            print(eval_metrics.summary())

            self.iteration_results.append({
                "iteration": iteration,
                "type": "corrective",
                "metrics": eval_metrics,
                "data_path": data_path,
                "intervention_ratio": self.intervention_manager.intervention_ratio,
            })

            # 4. 수렴 확인
            if self._check_convergence(eval_metrics):
                print(f"\n[수렴] 목표 성공률 달성: {eval_metrics.success_rate:.1%}")
                break

            prev_rate = self.iteration_results[-2]["metrics"].success_rate
            improvement = eval_metrics.success_rate - prev_rate
            if improvement < self.config.min_improvement and iteration >= 2:
                print(f"\n[수렴] 개선 정체 (Δ={improvement:.3f})")
                break

        # 최종 결과
        self._print_final_summary()

        return self.policy, self.iteration_results

    def _collect_corrective_data(self, iteration: int) -> str:
        """교정 데이터 수집 (정책 실행 + 인간 개입).

        Args:
            iteration: 현재 반복 번호

        Returns:
            저장된 HDF5 파일 경로
        """
        data_path = (
            Path(self.config.data_dir)
            / f"corrective_iter{iteration}.hdf5"
        )
        recorder = CorrectiveHDF5Recorder(
            file_path=str(data_path),
            action_dim=38,
            phase=2,
        )

        self.intervention_manager.reset_episode()

        print(f"\n  데이터 수집: {self.config.episodes_per_iteration}개 에피소드")

        for ep in range(self.config.episodes_per_iteration):
            obs, info = self.env.reset()
            recorder.start_episode()
            self.intervention_manager.reset_episode()

            if hasattr(self.policy, "reset"):
                self.policy.reset()

            episode_reward = 0.0

            for step in range(self.config.max_steps_per_episode):
                # 텔레오퍼레이션 액션은 VR 디바이스에서 가져옴
                # 실제 실행 시 teleop_device.advance()로 교체
                teleop_action = None
                intervention_trigger = False
                resume_trigger = False

                action, is_intervention, iid = self.intervention_manager.step(
                    obs,
                    teleop_action=teleop_action,
                    intervention_trigger=intervention_trigger,
                    resume_trigger=resume_trigger,
                )

                if action.dim() == 1:
                    action = action.unsqueeze(0)
                action = action.to(self.env.device)

                obs_next, rewards, terminated, truncated, info = self.env.step(action)
                reward = rewards.mean().item()
                episode_reward += reward

                # 기록
                recorder.add_step(
                    obs=obs.cpu().numpy() if isinstance(obs, torch.Tensor) else obs,
                    action=action.squeeze(0).cpu().numpy(),
                    done=terminated.any().item() or truncated.any().item(),
                    reward=reward,
                    is_intervention=is_intervention,
                    intervention_id=iid,
                )

                obs = obs_next

                if terminated.any() or truncated.any():
                    break

            # 에피소드 종료
            success = False  # 실제로는 termination 조건으로 판단
            recorder.end_episode(success=success)

            if (ep + 1) % 5 == 0:
                print(
                    f"    에피소드 {ep+1}/{self.config.episodes_per_iteration}, "
                    f"개입비율: {self.intervention_manager.intervention_ratio:.1%}"
                )

        recorder.save()
        print(self.intervention_manager.get_summary())

        return str(data_path)

    def _retrain_with_iwr(self, iteration: int):
        """IWR 가중치로 정책 재학습.

        Args:
            iteration: 현재 반복 번호
        """
        print(f"\n  IWR 재학습 (iteration {iteration})")

        # 모든 수집 데이터 결합
        for data_path in self.all_data_paths:
            if not Path(data_path).exists():
                continue

            dataset = IWRDataset(
                hdf5_path=data_path,
                w_autonomous=self.config.iwr_config.autonomous_weight,
                w_intervention=self.config.iwr_config.intervention_weight,
            )

            if len(dataset) == 0:
                print(f"    [SKIP] 빈 데이터셋: {data_path}")
                continue

            # 파인튜닝
            self.config.iwr_config.save_dir = str(
                Path(self.config.model_dir) / f"iter_{iteration}"
            )

            if hasattr(self.policy, "model") and self.policy.model is not None:
                model = self.policy.model
                if isinstance(model, torch.nn.Module):
                    self.iwr_trainer.finetune(model, dataset)
            else:
                print("    [WARNING] 정책 모델에 직접 접근 불가. 파인튜닝 건너뜀.")

    def _check_convergence(self, metrics: EvaluationMetrics) -> bool:
        """수렴 확인."""
        return metrics.success_rate >= self.config.target_success_rate

    def _print_final_summary(self):
        """최종 결과 요약."""
        print("\n" + "=" * 60)
        print("  HG-DAgger 최종 결과")
        print("=" * 60)
        print(f"\n  {'Iter':<6}{'성공률':<12}{'분류물체':<12}{'개입비율':<12}")
        print(f"  {'-'*42}")

        for result in self.iteration_results:
            m = result["metrics"]
            ir = result.get("intervention_ratio", 0.0)
            print(
                f"  {result['iteration']:<6}"
                f"{m.success_rate:<12.1%}"
                f"{m.avg_sorted_objects:<12.1f}"
                f"{ir:<12.1%}"
            )

        if len(self.iteration_results) >= 2:
            baseline = self.iteration_results[0]["metrics"].success_rate
            final = self.iteration_results[-1]["metrics"].success_rate
            improvement = final - baseline
            print(f"\n  기준선: {baseline:.1%} → 최종: {final:.1%}")
            print(f"  개선: {improvement:+.1%}")

        print("=" * 60)
