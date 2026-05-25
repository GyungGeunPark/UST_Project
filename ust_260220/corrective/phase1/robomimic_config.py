"""Robomimic BC-RNN 학습 설정 생성기.

G1 Kitchen Sorting 환경에 맞는 Robomimic 학습 config를 생성합니다.
RTX PRO 6000 (96GB GDDR7) 최적화: batch=128, BF16, num_workers=8
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Optional


def create_robomimic_config(
    dataset_path: str = "./data/demos/kitchen_sorting_augmented.hdf5",
    output_dir: str = "./models/bc_rnn/",
    experiment_name: str = "kitchen_sorting_bc_rnn",
    num_epochs: int = 2000,
    batch_size: int = 128,
    lr: float = 1e-4,
    rnn_horizon: int = 10,
    rnn_hidden_dim: int = 400,
    rnn_num_layers: int = 2,
    gmm_num_modes: int = 5,
    seed: int = 42,
    rollout_enabled: bool = True,
    rollout_n: int = 10,
    rollout_horizon: int = 1200,
    include_rgb: bool = False,
) -> dict:
    """Robomimic BC-RNN 학습 config 생성.

    Args:
        dataset_path: HDF5 데이터셋 경로
        output_dir: 모델 저장 경로
        experiment_name: 실험 이름
        num_epochs: 학습 에폭 수
        batch_size: 배치 크기
        lr: 학습률
        rnn_horizon: RNN 시퀀스 길이
        rnn_hidden_dim: RNN 은닉 차원
        rnn_num_layers: RNN 레이어 수
        gmm_num_modes: GMM 모드 수
        seed: 랜덤 시드
        rollout_enabled: 롤아웃 평가 여부
        rollout_n: 롤아웃 에피소드 수
        rollout_horizon: 롤아웃 최대 스텝
        include_rgb: RGB 이미지 포함 여부

    Returns:
        Robomimic 학습 config dict
    """
    # 관찰 모달리티
    low_dim_keys = [
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
    ]

    obs_modalities = {
        "obs": {
            "low_dim": low_dim_keys,
        },
    }

    encoder_config = {}

    if include_rgb:
        obs_modalities["obs"]["rgb"] = ["rgb_image"]
        encoder_config["rgb"] = {
            "core_class": "VisualCore",
            "core_kwargs": {
                "feature_dimension": 64,
                "backbone_class": "ResNet18Conv",
                "backbone_kwargs": {
                    "pretrained": False,
                    "input_coord_conv": False,
                },
                "pool_class": "SpatialSoftmax",
                "pool_kwargs": {
                    "num_kp": 32,
                    "learnable_temperature": False,
                    "temperature": 1.0,
                    "noise_std": 0.0,
                },
            },
        }

    config = {
        "algo_name": "bc",
        "experiment": {
            "name": experiment_name,
            "validate": True,
            "logging": {
                "terminal_output_to_txt": True,
                "log_tb": True,
            },
            "epoch_every_n_steps": 100,
            "save": {
                "enabled": True,
                "every_n_epochs": 20,
            },
            "rollout": {
                "enabled": rollout_enabled,
                "n": rollout_n,
                "horizon": rollout_horizon,
                "rate": 50,
                "terminate_on_success": True,
            },
        },
        "train": {
            "data": dataset_path,
            "output_dir": output_dir,
            "num_data_workers": 8,
            "hdf5_cache_mode": "low_dim",
            "hdf5_use_swmr": True,
            "batch_size": batch_size,
            "num_epochs": num_epochs,
            "seed": seed,
        },
        "algo": {
            "optim_params": {
                "policy": {
                    "optimizer_type": "adam",
                    "learning_rate": {
                        "initial": lr,
                        "decay_factor": 0.1,
                        "epoch_schedule": [],
                    },
                },
            },
            "actor_layer_dims": [300, 400],
            "rnn": {
                "enabled": True,
                "horizon": rnn_horizon,
                "hidden_dim": rnn_hidden_dim,
                "rnn_type": "LSTM",
                "num_layers": rnn_num_layers,
                "kwargs": {
                    "bidirectional": False,
                },
            },
            "gmm": {
                "enabled": True,
                "num_modes": gmm_num_modes,
                "min_std": 0.0001,
                "std_activation": "softplus",
                "low_noise_eval": True,
            },
        },
        "observation": {
            "modalities": obs_modalities,
            "encoder": encoder_config,
        },
    }

    return config


def save_robomimic_config(
    config: dict,
    output_path: str,
) -> str:
    """Robomimic config를 JSON 파일로 저장.

    Args:
        config: Robomimic 학습 config
        output_path: 저장 경로

    Returns:
        저장된 파일 경로
    """
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    print(f"[RobomimicConfig] 설정 저장: {path}")
    return str(path)


def create_ensemble_configs(
    base_config: dict,
    num_models: int = 5,
    subsample_ratio: float = 0.8,
) -> list:
    """앙상블 학습용 config 목록 생성 (Phase 3).

    각 모델에 다른 시드와 서브샘플링을 적용합니다.

    Args:
        base_config: 기본 config
        num_models: 앙상블 모델 수
        subsample_ratio: 데이터 서브샘플 비율

    Returns:
        config 리스트
    """
    configs = []
    for k in range(num_models):
        config = json.loads(json.dumps(base_config))  # deep copy
        config["train"]["seed"] = 42 + k
        config["experiment"]["name"] = f"{base_config['experiment']['name']}_ensemble_{k}"
        config["train"]["output_dir"] = (
            f"{base_config['train']['output_dir']}/ensemble_{k}/"
        )
        # 서브샘플링 (Robomimic 커스텀 키)
        config["train"]["subsample_ratio"] = subsample_ratio
        configs.append(config)

    return configs
