#!/usr/bin/env python3
# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""UST 모방학습 정책 학습 스크립트.

all_dev.md 스펙에 따른 Robomimic BC/BC-RNN 학습 래퍼.

사용법:
    # Robomimic BC 학습
    ./isaaclab.sh -p scripts/train_policy.py \
        --algo bc \
        --dataset ./datasets/ust_manipulation_*.hdf5

    # BC-RNN 학습
    ./isaaclab.sh -p scripts/train_policy.py \
        --algo bc_rnn \
        --dataset ./datasets/ust_manipulation_*.hdf5 \
        --epochs 2000

    # 커스텀 설정 파일 사용
    ./isaaclab.sh -p scripts/train_policy.py \
        --algo bc_rnn \
        --dataset ./datasets/augmented_demos.hdf5 \
        --config ./config/robomimic_bc_rnn.json
"""

from __future__ import annotations

import argparse
import sys
import os
import json
from pathlib import Path

# 프로젝트 경로
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def create_robomimic_config(args) -> dict:
    """Robomimic 학습 설정 생성.

    IMITATION_LEARNING_GUIDE.md의 BC-RNN 하이퍼파라미터 기반.
    """
    config = {
        "algo_name": args.algo,
        "experiment": {
            "name": f"ust_{args.algo}_{args.task}",
            "validate": True,
            "epoch_every_n_steps": 100,
            "save": {
                "enabled": True,
                "every_n_epochs": 50,
            },
            "rollout": {
                "enabled": False,
            },
        },
        "train": {
            "data": str(args.dataset),
            "output_dir": str(args.output_dir),
            "num_epochs": args.epochs,
            "batch_size": args.batch_size,
            "seed": args.seed,
        },
        "observation": {
            "modalities": {
                "obs": {
                    "low_dim": [
                        "right_arm_joint_pos",
                        "right_gripper_pos",
                        "right_ee_pose",
                        "left_arm_joint_pos",
                        "left_gripper_pos",
                        "left_ee_pose",
                        "object_pos",
                    ],
                },
            },
        },
        "algo": {},
    }

    # 알고리즘별 설정
    if args.algo == "bc":
        config["algo"] = {
            "optim_params": {
                "policy": {
                    "learning_rate": {"initial": 1e-4},
                },
            },
            "actor_layer_dims": [300, 400],
        }
    elif args.algo == "bc_rnn":
        config["algo"] = {
            "optim_params": {
                "policy": {
                    "learning_rate": {"initial": 1e-4},
                },
            },
            "actor_layer_dims": [300, 400],
            "rnn": {
                "enabled": True,
                "horizon": args.seq_length,
                "hidden_dim": 400,
                "rnn_type": "LSTM",
                "num_layers": 2,
            },
            "gmm": {
                "enabled": True,
                "num_modes": 5,
                "min_std": 0.0001,
            },
        }

    return config


def main():
    parser = argparse.ArgumentParser(description="UST Policy Training")
    parser.add_argument("--algo", type=str, default="bc_rnn",
                       choices=["bc", "bc_rnn"],
                       help="학습 알고리즘")
    parser.add_argument("--dataset", type=str, required=True,
                       help="HDF5 데이터셋 경로")
    parser.add_argument("--task", type=str, default="manipulation",
                       help="태스크 이름")
    parser.add_argument("--output_dir", type=str, default="./trained_models",
                       help="학습 결과 저장 디렉토리")
    parser.add_argument("--epochs", type=int, default=2000,
                       help="학습 에포크 수")
    parser.add_argument("--batch_size", type=int, default=100,
                       help="배치 크기")
    parser.add_argument("--seq_length", type=int, default=10,
                       help="시퀀스 길이 (BC-RNN)")
    parser.add_argument("--seed", type=int, default=42,
                       help="랜덤 시드")
    parser.add_argument("--config", type=str, default=None,
                       help="커스텀 Robomimic JSON 설정 파일")
    args = parser.parse_args()

    # 데이터셋 확인
    dataset_path = Path(args.dataset)
    if not dataset_path.exists():
        print(f"[ERROR] Dataset not found: {args.dataset}")
        sys.exit(1)

    # 출력 디렉토리 생성
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 설정 생성 또는 로드
    if args.config:
        with open(args.config) as f:
            config = json.load(f)
        print(f"[INFO] Custom config loaded: {args.config}")
    else:
        config = create_robomimic_config(args)
        # 설정 파일 저장
        config_path = output_dir / f"config_{args.algo}_{args.task}.json"
        with open(config_path, "w") as f:
            json.dump(config, f, indent=2)
        print(f"[INFO] Config saved: {config_path}")

    # Robomimic 학습 실행
    print(f"\n{'=' * 60}")
    print(f"       UST Policy Training ({args.algo.upper()})")
    print(f"{'=' * 60}")
    print(f"\n[Configuration]")
    print(f"  Algorithm: {args.algo}")
    print(f"  Dataset: {args.dataset}")
    print(f"  Epochs: {args.epochs}")
    print(f"  Batch size: {args.batch_size}")
    if args.algo == "bc_rnn":
        print(f"  Sequence length: {args.seq_length}")
    print(f"  Output: {args.output_dir}")
    print(f"  Seed: {args.seed}")
    print(f"\n{'=' * 60}\n")

    try:
        # 방법 1: Robomimic 직접 학습
        try:
            import robomimic.utils.train_utils as TrainUtils
            import robomimic.config as config_module

            ext_cfg = config_module.config_factory(args.algo)
            with ext_cfg.values_unlocked():
                ext_cfg.merge(config)

            print("[INFO] Starting Robomimic training...")
            TrainUtils.run_training(ext_cfg)
            print("[SUCCESS] Training completed!")

        except ImportError:
            print("[WARNING] Robomimic not installed.")
            print("\n[Alternative Options]")
            print("-" * 40)
            print("\n1. Install Robomimic:")
            print("   pip install robomimic")
            print(f"\n2. Use Isaac Lab's built-in imitation learning:")
            print(f"   ./isaaclab.sh -p source/isaaclab.robomimic/scripts/train.py \\")
            print(f"       --task UST-MobileManipulator-v0 \\")
            print(f"       --algo {args.algo} \\")
            print(f"       --dataset {args.dataset}")
            print(f"\n3. Manual training with config file:")
            print(f"   python -m robomimic.scripts.train \\")
            print(f"       --config {output_dir}/config_{args.algo}_{args.task}.json")
            print(f"\n[INFO] Config JSON saved at: {output_dir}/config_{args.algo}_{args.task}.json")

    except Exception as e:
        print(f"[ERROR] Training failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
