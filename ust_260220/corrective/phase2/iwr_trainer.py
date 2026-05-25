"""IWR (Intervention Weighted Regression) 학습기.

개입 데이터에 높은 가중치를 부여하여 재학습합니다.
- 자율 데이터: weight = 1.0
- 개입 데이터: weight = w_intervention (기본 5.0)

RTX PRO 6000 최적화: BF16 혼합 정밀도, cosine LR scheduler
"""

from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
from dataclasses import dataclass
from pathlib import Path
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
from typing import Dict, List, Optional, Tuple

from ..utils.corrective_hdf5 import (
    CorrectiveHDF5Reader,
    LABEL_AUTONOMOUS,
    LABEL_INTERVENTION,
)


@dataclass
class IWRConfig:
    """IWR 학습 설정 (RTX PRO 6000 최적화)."""

    # 가중치
    autonomous_weight: float = 1.0
    intervention_weight: float = 5.0

    # 학습 하이퍼파라미터
    batch_size: int = 64
    num_epochs: int = 200
    learning_rate: float = 5e-5
    weight_decay: float = 1e-5
    grad_clip_norm: float = 1.0

    # 온라인 학습 (시뮬레이션 중 재학습)
    online_batch_size: int = 16
    gradient_accumulation_steps: int = 4  # 유효 배치 = 64

    # BF16 혼합 정밀도 (Blackwell 최적화)
    use_bf16: bool = True

    # 스케줄링
    lr_scheduler_type: str = "cosine"  # "cosine" 또는 "plateau"
    lr_decay_factor: float = 0.5
    lr_decay_patience: int = 20
    warmup_epochs: int = 10

    # 저장
    save_dir: str = "./models/hg_dagger/"
    save_every_n_epochs: int = 20

    # 데이터
    sequence_length: int = 15  # RNN 시퀀스 길이 (10→15 증가)


class IWRDataset(Dataset):
    """IWR 가중치가 적용된 교정 데이터셋."""

    def __init__(
        self,
        hdf5_path: str,
        w_autonomous: float = 1.0,
        w_intervention: float = 5.0,
        sequence_length: int = 10,
    ):
        """
        Args:
            hdf5_path: 교정 HDF5 데이터셋 경로
            w_autonomous: 자율 데이터 가중치
            w_intervention: 개입 데이터 가중치
            sequence_length: RNN 시퀀스 길이
        """
        self.w_autonomous = w_autonomous
        self.w_intervention = w_intervention
        self.sequence_length = sequence_length

        self.samples = []
        self.sample_weights = []

        self._load_data(hdf5_path)

    def _load_data(self, path: str):
        """HDF5에서 시퀀스 단위 데이터 로드."""
        reader = CorrectiveHDF5Reader(path)

        try:
            train_indices = reader.get_train_episodes()
            if not train_indices:
                train_indices = list(range(reader.num_episodes))

            for ep_idx in train_indices:
                ep = reader.get_episode(ep_idx)
                obs = ep["obs"]
                actions = ep["actions"]
                labels = ep["intervention_labels"]
                length = ep["length"]

                if labels is None:
                    labels = np.zeros(length, dtype=np.int32)

                # 시퀀스 단위로 분할
                for start in range(0, length - self.sequence_length + 1):
                    end = start + self.sequence_length
                    seq_obs = obs[start:end]
                    seq_actions = actions[start:end]
                    seq_labels = labels[start:end]

                    # 시퀀스 내 개입 비율로 가중치 결정
                    intervention_ratio = np.mean(
                        seq_labels == LABEL_INTERVENTION
                    )
                    weight = (
                        self.w_autonomous * (1 - intervention_ratio)
                        + self.w_intervention * intervention_ratio
                    )

                    self.samples.append({
                        "obs": seq_obs.astype(np.float32),
                        "actions": seq_actions.astype(np.float32),
                        "labels": seq_labels.astype(np.int32),
                        "weight": float(weight),
                    })
                    self.sample_weights.append(float(weight))

        finally:
            reader.close()

        print(
            f"[IWRDataset] {len(self.samples)}개 시퀀스 로드 "
            f"(seq_len={self.sequence_length})"
        )

    def get_weighted_sampler(self) -> WeightedRandomSampler:
        """IWR 가중치 기반 샘플러."""
        return WeightedRandomSampler(
            weights=self.sample_weights,
            num_samples=len(self.samples),
            replacement=True,
        )

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        sample = self.samples[idx]
        return {
            "obs": torch.tensor(sample["obs"], dtype=torch.float32),
            "actions": torch.tensor(sample["actions"], dtype=torch.float32),
            "labels": torch.tensor(sample["labels"], dtype=torch.int64),
            "weight": torch.tensor(sample["weight"], dtype=torch.float32),
        }


class IWRTrainer:
    """IWR 가중치 학습 루프.

    BC-RNN 정책을 IWR 가중치로 재학습합니다.
    개입 데이터의 가중치가 높아 정책이 실패하는 상태에서의
    올바른 행동을 더 강하게 학습합니다.
    """

    def __init__(self, config: IWRConfig):
        self.config = config
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        Path(config.save_dir).mkdir(parents=True, exist_ok=True)

    def train(
        self,
        policy: nn.Module,
        dataset: IWRDataset,
        validation_dataset: Optional[IWRDataset] = None,
    ) -> nn.Module:
        """IWR 가중치 학습 실행.

        Args:
            policy: BC-RNN 정책 네트워크
            dataset: IWR 학습 데이터셋
            validation_dataset: 검증 데이터셋 (선택)

        Returns:
            학습된 정책
        """
        policy = policy.to(self.device)
        policy.train()

        optimizer = torch.optim.Adam(
            policy.parameters(),
            lr=self.config.learning_rate,
            weight_decay=self.config.weight_decay,
        )

        # LR 스케줄러 선택
        if self.config.lr_scheduler_type == "cosine":
            scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                optimizer,
                T_max=self.config.num_epochs,
                eta_min=self.config.learning_rate * 0.01,
            )
        else:
            scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                optimizer,
                mode="min",
                factor=self.config.lr_decay_factor,
                patience=self.config.lr_decay_patience,
            )

        sampler = dataset.get_weighted_sampler()
        dataloader = DataLoader(
            dataset,
            batch_size=self.config.batch_size,
            sampler=sampler,
            num_workers=4,
            pin_memory=True,
        )

        # BF16 설정 (Blackwell 최적화 - GradScaler 불필요)
        use_amp = self.config.use_bf16 and torch.cuda.is_available()
        amp_dtype = torch.bfloat16 if use_amp else torch.float32

        best_loss = float("inf")
        best_model_state = None

        print(f"\n[IWR Trainer] 학습 시작")
        print(f"  데이터: {len(dataset)}개 시퀀스")
        print(f"  자율 가중치: {self.config.autonomous_weight}")
        print(f"  개입 가중치: {self.config.intervention_weight}")
        print(f"  에폭: {self.config.num_epochs}")
        print(f"  BF16: {use_amp}")
        print(f"  LR 스케줄러: {self.config.lr_scheduler_type}")

        for epoch in range(self.config.num_epochs):
            epoch_loss = 0.0
            num_batches = 0

            for batch in dataloader:
                obs = batch["obs"].to(self.device)        # (B, T, obs_dim)
                target = batch["actions"].to(self.device)  # (B, T, 38)
                weights = batch["weight"].to(self.device)  # (B,)

                # RNN 상태 리셋
                if hasattr(policy, "reset_hidden"):
                    policy.reset_hidden()

                # BF16 혼합 정밀도 순전파
                with torch.amp.autocast("cuda", dtype=amp_dtype, enabled=use_amp):
                    predicted = policy(obs)  # (B, 38) 또는 (B, T, 38)

                    # 마지막 타임스텝의 액션만 비교
                    if predicted.dim() == 2:
                        target_last = target[:, -1, :]
                    else:
                        target_last = target
                        predicted = predicted[:, -1, :] if predicted.dim() == 3 else predicted

                    # 가중 MSE 손실
                    mse_per_sample = ((predicted - target_last) ** 2).mean(dim=-1)
                    weighted_loss = (mse_per_sample * weights).mean()

                optimizer.zero_grad()
                weighted_loss.backward()

                if self.config.grad_clip_norm > 0:
                    nn.utils.clip_grad_norm_(
                        policy.parameters(), self.config.grad_clip_norm
                    )

                optimizer.step()

                epoch_loss += weighted_loss.item()
                num_batches += 1

            avg_loss = epoch_loss / max(num_batches, 1)

            if self.config.lr_scheduler_type == "cosine":
                scheduler.step()
            else:
                scheduler.step(avg_loss)

            # 최적 모델 저장
            if avg_loss < best_loss:
                best_loss = avg_loss
                best_model_state = {
                    k: v.clone() for k, v in policy.state_dict().items()
                }

            # 주기적 로그
            if (epoch + 1) % 20 == 0:
                lr = optimizer.param_groups[0]["lr"]
                print(
                    f"  Epoch {epoch+1}/{self.config.num_epochs}, "
                    f"Loss: {avg_loss:.6f}, LR: {lr:.2e}"
                )

            # 주기적 저장
            if (
                self.config.save_every_n_epochs > 0
                and (epoch + 1) % self.config.save_every_n_epochs == 0
            ):
                self._save_checkpoint(policy, epoch + 1, avg_loss)

        # 최적 모델 복원
        if best_model_state is not None:
            policy.load_state_dict(best_model_state)

        # 최종 저장
        self._save_checkpoint(policy, self.config.num_epochs, best_loss, is_best=True)

        print(f"\n[IWR Trainer] 학습 완료. Best loss: {best_loss:.6f}")

        policy.eval()
        return policy

    def finetune(
        self,
        policy: nn.Module,
        new_dataset: IWRDataset,
        num_epochs: Optional[int] = None,
        lr: Optional[float] = None,
    ) -> nn.Module:
        """기존 정책을 새 교정 데이터로 파인튜닝.

        Args:
            policy: 기존 학습된 정책
            new_dataset: 새 교정 데이터셋
            num_epochs: 에폭 수 (기본: config의 절반)
            lr: 학습률 (기본: config의 1/10)

        Returns:
            파인튜닝된 정책
        """
        original_epochs = self.config.num_epochs
        original_lr = self.config.learning_rate

        self.config.num_epochs = num_epochs or original_epochs // 2
        self.config.learning_rate = lr or original_lr * 0.1

        try:
            result = self.train(policy, new_dataset)
        finally:
            self.config.num_epochs = original_epochs
            self.config.learning_rate = original_lr

        return result

    def _save_checkpoint(
        self,
        policy: nn.Module,
        epoch: int,
        loss: float,
        is_best: bool = False,
    ):
        """체크포인트 저장."""
        save_dir = Path(self.config.save_dir)
        checkpoint = {
            "epoch": epoch,
            "model_state_dict": policy.state_dict(),
            "loss": loss,
            "config": {
                "autonomous_weight": self.config.autonomous_weight,
                "intervention_weight": self.config.intervention_weight,
                "batch_size": self.config.batch_size,
                "learning_rate": self.config.learning_rate,
            },
        }

        if is_best:
            path = save_dir / "model_best.pth"
        else:
            path = save_dir / f"model_epoch_{epoch}.pth"

        torch.save(checkpoint, path)
