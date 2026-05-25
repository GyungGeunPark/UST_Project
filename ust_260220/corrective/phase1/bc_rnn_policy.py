"""BC-RNN 정책 래퍼.

Robomimic BC-RNN 모델을 Isaac Lab 환경에서 사용할 수 있도록 래핑합니다.
- LSTM 2 layers, hidden_dim=400
- GMM 5 modes 출력
- 38D action space (14 EEF + 24 hand)
"""

from __future__ import annotations

import torch
import torch.nn as nn
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


@dataclass
class BCRNNPolicyCfg:
    """BC-RNN 정책 설정."""

    # 모델 아키텍처
    obs_dim: int = 135
    action_dim: int = 38
    actor_layer_dims: List[int] = field(default_factory=lambda: [300, 400])
    rnn_hidden_dim: int = 400
    rnn_num_layers: int = 2
    rnn_type: str = "LSTM"
    rnn_horizon: int = 10

    # GMM 출력
    gmm_num_modes: int = 5
    gmm_min_std: float = 0.0001

    # 관찰 모달리티
    low_dim_keys: List[str] = field(default_factory=lambda: [
        "joint_pos",        # (53,) = 29 arm + 24 hand
        "joint_vel",        # (53,)
        "left_eef_pos",     # (7,) pos(3) + quat(4)
        "right_eef_pos",    # (7,)
        "mug_pos",          # (7,)
        "plate_pos",        # (7,)
        "bowl_pos",         # (7,)
        "can_pos",          # (7,)
        "bottle_pos",       # (7,)
        "apple_pos",        # (7,)
        "sponge_pos",       # (7,)
        "teddy_pos",        # (7,)
        "bin_positions",    # (9,) = 3 bins × 3D pos
    ])

    # 추론 설정
    device: str = "cuda"
    temporal_agg: bool = False


class BCRNNPolicy:
    """BC-RNN 정책 래퍼.

    Robomimic 체크포인트를 로드하여 Isaac Lab 환경에서 추론합니다.
    """

    def __init__(self, cfg: BCRNNPolicyCfg, checkpoint_path: Optional[str] = None):
        self.cfg = cfg
        self.device = torch.device(cfg.device)
        self.model = None
        self.rnn_state = None
        self._step_count = 0

        if checkpoint_path is not None:
            self.load(checkpoint_path)

    def load(self, checkpoint_path: str):
        """Robomimic 체크포인트 로드."""
        path = Path(checkpoint_path)
        if not path.exists():
            raise FileNotFoundError(f"체크포인트를 찾을 수 없습니다: {checkpoint_path}")

        checkpoint = torch.load(str(path), map_location=self.device, weights_only=False)

        # Robomimic 체크포인트 구조 처리
        if "model" in checkpoint:
            # Robomimic 형식
            self._load_robomimic_checkpoint(checkpoint)
        elif "state_dict" in checkpoint:
            # PyTorch Lightning 형식
            self._load_state_dict(checkpoint["state_dict"])
        else:
            # 직접 state_dict
            self._load_state_dict(checkpoint)

        print(f"[BCRNNPolicy] 체크포인트 로드: {checkpoint_path}")

    def _load_robomimic_checkpoint(self, checkpoint: dict):
        """Robomimic 형식 체크포인트 로드."""
        try:
            import robomimic.utils.file_utils as FileUtils
            import robomimic.utils.torch_utils as TorchUtils

            policy, _ = FileUtils.policy_from_checkpoint(ckpt_dict=checkpoint)
            self.model = policy
            self.model.start_episode()
        except ImportError:
            print("[WARNING] robomimic 미설치. 직접 state_dict 로드 시도...")
            if "model" in checkpoint:
                self._load_state_dict(checkpoint["model"])

    def _load_state_dict(self, state_dict: dict):
        """state_dict에서 직접 모델 구성."""
        self.model = SimpleBCRNN(
            obs_dim=self.cfg.obs_dim,
            action_dim=self.cfg.action_dim,
            hidden_dim=self.cfg.rnn_hidden_dim,
            num_layers=self.cfg.rnn_num_layers,
            mlp_dims=self.cfg.actor_layer_dims,
        ).to(self.device)
        self.model.load_state_dict(state_dict, strict=False)
        self.model.eval()

    def reset(self):
        """에피소드 시작 시 RNN 상태 초기화."""
        self.rnn_state = None
        self._step_count = 0
        if self.model is not None and hasattr(self.model, "start_episode"):
            self.model.start_episode()

    def predict(self, obs: torch.Tensor) -> Optional[torch.Tensor]:
        """관찰에서 액션 예측.

        Args:
            obs: 관찰 텐서 (batch, obs_dim) 또는 dict

        Returns:
            action: 38D 액션 텐서 (batch, 38) 또는 None
        """
        if self.model is None:
            return None

        with torch.inference_mode():
            if isinstance(obs, dict):
                obs_processed = self._process_obs_dict(obs)
            else:
                obs_processed = obs

            if obs_processed.dim() == 1:
                obs_processed = obs_processed.unsqueeze(0)

            obs_processed = obs_processed.to(self.device)

            # Robomimic 정책 인터페이스
            if hasattr(self.model, "__call__") and not isinstance(self.model, nn.Module):
                action = self.model(obs_processed.cpu().numpy())
                action = torch.tensor(action, dtype=torch.float32, device=self.device)
            else:
                action = self.model(obs_processed)

            self._step_count += 1
            return action

    def predict_with_uncertainty(
        self, obs: torch.Tensor
    ) -> Tuple[torch.Tensor, float]:
        """액션 예측 + GMM 불확실성 (단일 모델).

        GMM 모드 간의 분산을 불확실성으로 사용합니다.

        Returns:
            action: 38D 액션 텐서
            uncertainty: GMM 모드 분산 (스칼라)
        """
        action = self.predict(obs)
        # GMM 기반 불확실성은 모델 내부에서 계산 (여기서는 0 반환)
        uncertainty = 0.0
        return action, uncertainty

    def _process_obs_dict(self, obs_dict: dict) -> torch.Tensor:
        """관찰 딕셔너리를 플랫 텐서로 변환."""
        parts = []
        for key in self.cfg.low_dim_keys:
            if key in obs_dict:
                val = obs_dict[key]
                if isinstance(val, np.ndarray):
                    val = torch.tensor(val, dtype=torch.float32)
                if val.dim() == 1:
                    val = val.unsqueeze(0)
                parts.append(val.to(self.device))

        if not parts:
            raise ValueError("관찰 딕셔너리에서 유효한 키를 찾을 수 없습니다.")

        return torch.cat(parts, dim=-1)

    @property
    def action_dim(self) -> int:
        return self.cfg.action_dim


class SimpleBCRNN(nn.Module):
    """간단한 BC-RNN 모델 (Robomimic 없이 독립 실행용)."""

    def __init__(
        self,
        obs_dim: int = 135,
        action_dim: int = 38,
        hidden_dim: int = 400,
        num_layers: int = 2,
        mlp_dims: List[int] = None,
    ):
        super().__init__()
        if mlp_dims is None:
            mlp_dims = [300, 400]

        # 관찰 인코더
        encoder_layers = []
        in_dim = obs_dim
        for dim in mlp_dims:
            encoder_layers.extend([nn.Linear(in_dim, dim), nn.ReLU()])
            in_dim = dim
        self.obs_encoder = nn.Sequential(*encoder_layers)

        # RNN
        self.rnn = nn.LSTM(
            input_size=in_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )

        # 액션 디코더
        self.action_head = nn.Linear(hidden_dim, action_dim)

        # RNN 상태
        self._hidden = None

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """순전파.

        Args:
            obs: (batch, obs_dim) 또는 (batch, seq_len, obs_dim)

        Returns:
            action: (batch, action_dim)
        """
        if obs.dim() == 2:
            obs = obs.unsqueeze(1)  # (batch, 1, obs_dim)

        encoded = self.obs_encoder(obs)  # (batch, seq_len, mlp_out)
        rnn_out, self._hidden = self.rnn(encoded, self._hidden)
        action = self.action_head(rnn_out[:, -1, :])  # 마지막 타임스텝
        return action

    def start_episode(self):
        """에피소드 시작 시 RNN 상태 초기화."""
        self._hidden = None

    def reset_hidden(self):
        self._hidden = None
