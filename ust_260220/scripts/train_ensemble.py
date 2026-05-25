"""Phase 3: 앙상블 정책 학습.

K개의 독립 BC-RNN 모델을 학습하여 앙상블을 구성합니다.
다양성 확보: 다른 시드 + 부트스트랩 80% 서브샘플링

사용법:
    python scripts/train_ensemble.py \
        --dataset ./data/corrective/all_data.hdf5 \
        --num_models 5

    # 기존 BC-RNN 기반
    python scripts/train_ensemble.py \
        --dataset ./data/demos/kitchen_sorting_augmented.hdf5 \
        --num_models 5 \
        --num_epochs 400
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="앙상블 정책 학습 (Phase 3)")

    parser.add_argument(
        "--dataset",
        type=str,
        required=True,
        help="HDF5 데이터셋 경로",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="./models/ensemble/",
        help="앙상블 모델 저장 디렉토리",
    )
    parser.add_argument("--num_models", type=int, default=5)
    parser.add_argument("--num_epochs", type=int, default=2000)
    parser.add_argument("--batch_size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--subsample_ratio", type=float, default=0.8)
    parser.add_argument("--base_seed", type=int, default=42)

    args = parser.parse_args()

    if not Path(args.dataset).exists():
        print(f"[ERROR] 데이터셋 없음: {args.dataset}")
        sys.exit(1)

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from corrective.phase1.robomimic_config import (
        create_robomimic_config,
        create_ensemble_configs,
    )

    print("\n" + "=" * 50)
    print("  앙상블 정책 학습 (Phase 3)")
    print("=" * 50)
    print(f"  데이터셋: {args.dataset}")
    print(f"  모델 수: {args.num_models}")
    print(f"  에폭: {args.num_epochs}")
    print(f"  서브샘플: {args.subsample_ratio:.0%}")
    print("=" * 50)

    # 기본 config 생성
    base_config = create_robomimic_config(
        dataset_path=args.dataset,
        output_dir=args.output_dir,
        num_epochs=args.num_epochs,
        batch_size=args.batch_size,
        lr=args.lr,
    )

    # 앙상블 configs 생성
    ensemble_configs = create_ensemble_configs(
        base_config,
        num_models=args.num_models,
        subsample_ratio=args.subsample_ratio,
    )

    # 각 모델 학습
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    for k, config in enumerate(ensemble_configs):
        print(f"\n--- 앙상블 멤버 {k+1}/{args.num_models} ---")
        print(f"  시드: {config['train']['seed']}")
        print(f"  출력: {config['train']['output_dir']}")

        # Config 저장
        config_path = output_dir / f"ensemble_{k}_config.json"
        with open(config_path, "w") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)

        # 학습 (Robomimic 또는 독립 모드)
        try:
            import robomimic.utils.train_utils as TrainUtils
            from robomimic.config import config_factory

            ext_cfg = config_factory(config["algo_name"])
            with ext_cfg.values_unlocked():
                ext_cfg.update(config)

            TrainUtils.train(ext_cfg, auto_remove_exp=False)

        except ImportError:
            print("  [WARNING] robomimic 미설치. 독립 학습 모드...")
            _train_standalone_member(config, k)

    # 완료 요약
    print("\n" + "=" * 50)
    print("  앙상블 학습 완료")
    print("=" * 50)
    for k in range(args.num_models):
        model_path = output_dir / f"ensemble_{k}" / "model_best.pth"
        status = "OK" if model_path.exists() else "없음"
        print(f"  모델 {k}: {model_path} [{status}]")
    print("=" * 50)


def _train_standalone_member(config: dict, member_idx: int):
    """독립 학습 (robomimic 없이, RTX PRO 6000 BF16 최적화)."""
    import torch
    import h5py
    import numpy as np
    from torch.utils.data import DataLoader, TensorDataset

    from corrective.phase1.bc_rnn_policy import SimpleBCRNN

    dataset_path = config["train"]["data"]
    output_dir = Path(config["train"]["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)
    seed = config["train"]["seed"]

    torch.manual_seed(seed)
    np.random.seed(seed)

    # 데이터 로드 + 서브샘플링
    with h5py.File(dataset_path, "r") as f:
        demo_keys = sorted(f["data"].keys())
        subsample_ratio = config["train"].get("subsample_ratio", 0.8)
        n_select = max(1, int(len(demo_keys) * subsample_ratio))
        selected = np.random.choice(demo_keys, n_select, replace=False)

        all_obs = []
        all_actions = []
        for key in selected:
            all_obs.append(f["data"][key]["obs"][:])
            all_actions.append(f["data"][key]["actions"][:])

    obs = np.concatenate(all_obs, axis=0)
    actions = np.concatenate(all_actions, axis=0)

    obs_t = torch.tensor(obs, dtype=torch.float32)
    act_t = torch.tensor(actions, dtype=torch.float32)
    dataloader = DataLoader(
        TensorDataset(obs_t, act_t),
        batch_size=config["train"]["batch_size"],
        shuffle=True,
        num_workers=8,
        pin_memory=True,
    )

    model = SimpleBCRNN(
        obs_dim=obs.shape[1],
        action_dim=actions.shape[1],
        hidden_dim=config["algo"]["rnn"]["hidden_dim"],
        num_layers=config["algo"]["rnn"]["num_layers"],
    ).cuda()

    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=config["algo"]["optim_params"]["policy"]["learning_rate"]["initial"],
    )

    num_epochs = config["train"]["num_epochs"]
    scheduler = torch.optim.lr_scheduler.MultiStepLR(
        optimizer, milestones=[num_epochs // 2, int(num_epochs * 0.75)], gamma=0.1
    )

    # BF16 혼합 정밀도
    use_bf16 = torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8
    amp_dtype = torch.bfloat16 if use_bf16 else torch.float32

    best_loss = float("inf")

    for epoch in range(num_epochs):
        model.train()
        total_loss = 0.0
        n = 0

        for obs_b, act_b in dataloader:
            model.reset_hidden()

            with torch.amp.autocast("cuda", dtype=amp_dtype, enabled=use_bf16):
                pred = model(obs_b.cuda())
                loss = ((pred - act_b.cuda()) ** 2).mean()

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()

            total_loss += loss.item()
            n += 1

        avg_loss = total_loss / max(n, 1)
        scheduler.step()

        if avg_loss < best_loss:
            best_loss = avg_loss
            torch.save(model.state_dict(), output_dir / "model_best.pth")

        if (epoch + 1) % 100 == 0:
            lr = optimizer.param_groups[0]["lr"]
            print(f"    Epoch {epoch+1}/{num_epochs}, Loss: {avg_loss:.6f}, LR: {lr:.2e}")

    print(f"  멤버 {member_idx}: Best loss = {best_loss:.6f}")


if __name__ == "__main__":
    main()
