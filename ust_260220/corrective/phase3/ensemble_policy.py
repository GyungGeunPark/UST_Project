"""앙상블 정책으로 행동 불확실성 추정.

K개의 독립 학습된 BC-RNN 정책의 행동 분산을 불확실성으로 사용합니다.
다양성 확보: 다른 랜덤 시드 + 부트스트랩 샘플링 + 다른 초기화
"""

from __future__ import annotations

import torch
import torch.nn as nn
import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple

from ..phase1.bc_rnn_policy import BCRNNPolicy, BCRNNPolicyCfg


class EnsemblePolicy:
    """앙상블 정책으로 행동 불확실성 추정.

    K개의 독립 학습된 정책 네트워크의 예측 분산을 통해
    행동 불확실성을 추정합니다.

    사용법:
        models = [load_model(f"ensemble_{k}.pth") for k in range(5)]
        ensemble = EnsemblePolicy(models)
        action, uncertainty, should_ask = ensemble.predict_with_uncertainty(obs)
    """

    def __init__(
        self,
        models: List[nn.Module],
        uncertainty_threshold: float = 0.05,
        use_multimodal_aware: bool = True,
        n_clusters: int = 3,
    ):
        """
        Args:
            models: K개의 독립 학습된 정책 네트워크
            uncertainty_threshold: 도움 요청 임계값
            use_multimodal_aware: 다중 모달 인식 불확실성 사용
            n_clusters: 클러스터 수 (다중 모달 인식 시)
        """
        self.models = models
        self.K = len(models)
        self.threshold = uncertainty_threshold
        self.use_multimodal_aware = use_multimodal_aware
        self.n_clusters = n_clusters

        # 불확실성 이력
        self.uncertainty_history: List[float] = []
        self._step_count = 0

        # 모든 모델을 eval 모드로
        for model in self.models:
            if isinstance(model, nn.Module):
                model.eval()

    @classmethod
    def from_checkpoints(
        cls,
        checkpoint_paths: List[str],
        policy_cfg: Optional[BCRNNPolicyCfg] = None,
        **kwargs,
    ) -> "EnsemblePolicy":
        """체크포인트 파일에서 앙상블 생성.

        Args:
            checkpoint_paths: K개의 체크포인트 경로
            policy_cfg: BC-RNN 정책 설정

        Returns:
            EnsemblePolicy 인스턴스
        """
        cfg = policy_cfg or BCRNNPolicyCfg()
        models = []

        for path in checkpoint_paths:
            policy = BCRNNPolicy(cfg, checkpoint_path=path)
            if policy.model is not None:
                models.append(policy.model)
            else:
                print(f"[WARNING] 모델 로드 실패: {path}")

        if not models:
            raise ValueError("로드된 모델이 없습니다.")

        print(f"[EnsemblePolicy] {len(models)}개 모델 로드 완료")
        return cls(models, **kwargs)

    def predict_with_uncertainty(
        self, obs: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, bool]:
        """앙상블 예측 + 불확실성 추정.

        Args:
            obs: 관찰 텐서 (batch, obs_dim)

        Returns:
            mean_action: 평균 행동 (batch, 38)
            uncertainty: 행동 불확실성 (batch,) 스칼라
            should_ask_help: 도움 요청 여부
        """
        actions = []
        with torch.no_grad():
            for model in self.models:
                if hasattr(model, "reset_hidden"):
                    pass  # RNN 상태는 유지
                action = model(obs)
                actions.append(action)

        actions_stack = torch.stack(actions, dim=0)  # (K, batch, 38)

        # 평균 행동
        mean_action = actions_stack.mean(dim=0)  # (batch, 38)

        # 불확실성 계산
        if self.use_multimodal_aware and self.K >= self.n_clusters:
            uncertainty = self._multimodal_uncertainty(actions_stack)
        else:
            uncertainty = self._simple_uncertainty(actions_stack)

        # 도움 요청 판단
        should_ask_help = (uncertainty > self.threshold).any().item()

        # 이력 기록
        self.uncertainty_history.append(uncertainty.mean().item())
        self._step_count += 1

        return mean_action, uncertainty, should_ask_help

    def _simple_uncertainty(self, actions_stack: torch.Tensor) -> torch.Tensor:
        """단순 분산 기반 불확실성.

        Args:
            actions_stack: (K, batch, 38)

        Returns:
            uncertainty: (batch,) 각 차원의 분산 평균
        """
        variance = actions_stack.var(dim=0)  # (batch, 38)
        uncertainty = variance.mean(dim=-1)  # (batch,)
        return uncertainty

    def _multimodal_uncertainty(self, actions_stack: torch.Tensor) -> torch.Tensor:
        """다중 모달 인식 불확실성 (클러스터링 기반).

        여러 가지 올바른 행동이 있을 때 (예: 왼쪽/오른쪽에서 잡기)
        가장 큰 클러스터 내의 분산만 측정하여 오탐을 줄입니다.

        Args:
            actions_stack: (K, batch, 38)

        Returns:
            uncertainty: (batch,)
        """
        K, batch_size, action_dim = actions_stack.shape
        uncertainties = []

        for b in range(batch_size):
            actions_b = actions_stack[:, b, :].cpu().numpy()  # (K, 38)

            try:
                from sklearn.cluster import KMeans

                n_clusters = min(self.n_clusters, K)
                kmeans = KMeans(n_clusters=n_clusters, n_init=3, random_state=0)
                labels = kmeans.fit_predict(actions_b)

                # 가장 큰 클러스터의 분산
                largest_cluster = np.bincount(labels).argmax()
                cluster_actions = actions_b[labels == largest_cluster]
                unc = float(cluster_actions.var(axis=0).mean())
            except (ImportError, ValueError):
                # sklearn 없거나 데이터 부족
                unc = float(actions_b.var(axis=0).mean())

            uncertainties.append(unc)

        return torch.tensor(uncertainties, device=actions_stack.device)

    def get_action(self, obs: torch.Tensor) -> torch.Tensor:
        """자율 실행용 액션 (불확실성 무시)."""
        mean_action, _, _ = self.predict_with_uncertainty(obs)
        return mean_action

    def predict(self, obs: torch.Tensor) -> torch.Tensor:
        """PolicyEvaluator 호환 인터페이스."""
        return self.get_action(obs)

    def reset(self):
        """에피소드 시작 시 RNN 상태 초기화."""
        for model in self.models:
            if hasattr(model, "start_episode"):
                model.start_episode()
            elif hasattr(model, "reset_hidden"):
                model.reset_hidden()

    def get_uncertainty_stats(self) -> dict:
        """불확실성 통계."""
        if not self.uncertainty_history:
            return {"mean": 0.0, "std": 0.0, "max": 0.0, "min": 0.0}

        history = np.array(self.uncertainty_history)
        return {
            "mean": float(history.mean()),
            "std": float(history.std()),
            "max": float(history.max()),
            "min": float(history.min()),
            "total_steps": self._step_count,
            "above_threshold": int(np.sum(history > self.threshold)),
            "threshold_rate": float(np.mean(history > self.threshold)),
        }
