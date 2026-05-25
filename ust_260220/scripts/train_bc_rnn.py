"""Phase 1: BC-RNN 기본 정책 학습.

MimicGen으로 증강된 데이터 또는 원본 VR 시연으로 BC-RNN 정책을 학습합니다.
RTX PRO 6000 (96GB) 최적화: batch=128, 2000 epochs, BF16 혼합 정밀도

사용법:
    # 기본 학습 (RTX PRO 6000 최적 설정)
    python scripts/train_bc_rnn.py \
        --dataset ./data/demos/kitchen_sorting_augmented.hdf5

    # 커스텀 설정
    python scripts/train_bc_rnn.py \
        --dataset ./data/demos/kitchen_sorting_demos.hdf5 \
        --output_dir ./models/bc_rnn_custom/ \
        --num_epochs 1000 --batch_size 256
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="BC-RNN 정책 학습")

    parser.add_argument(
        "--dataset",
        type=str,
        default="./data/demos/kitchen_sorting_augmented.hdf5",
        help="HDF5 데이터셋 경로",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default="./models/bc_rnn/",
        help="모델 저장 디렉토리",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Robomimic config JSON 경로 (지정 시 다른 인자 무시)",
    )
    parser.add_argument("--num_epochs", type=int, default=2000)
    parser.add_argument("--batch_size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--rnn_hidden_dim", type=int, default=400)
    parser.add_argument("--rnn_num_layers", type=int, default=2)
    parser.add_argument("--gmm_num_modes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--include_rgb", action="store_true")
    parser.add_argument(
        "--save_config",
        type=str,
        default=None,
        help="생성된 config를 JSON으로 저장 (학습 없이)",
    )

    args = parser.parse_args()

    # 데이터셋 확인
    if not Path(args.dataset).exists():
        print(f"[ERROR] 데이터셋 없음: {args.dataset}")
        print("  먼저 시연 수집을 완료해주세요:")
        print("  python scripts/record_demos.py --num_demos 50")
        sys.exit(1)

    # Config 생성/로드
    if args.config:
        with open(args.config) as f:
            config = json.load(f)
        print(f"[Train] config 로드: {args.config}")
    else:
        # corrective 패키지에서 config 생성
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from corrective.phase1.robomimic_config import (
            create_robomimic_config,
            save_robomimic_config,
        )

        config = create_robomimic_config(
            dataset_path=args.dataset,
            output_dir=args.output_dir,
            num_epochs=args.num_epochs,
            batch_size=args.batch_size,
            lr=args.lr,
            rnn_hidden_dim=args.rnn_hidden_dim,
            rnn_num_layers=args.rnn_num_layers,
            gmm_num_modes=args.gmm_num_modes,
            seed=args.seed,
            include_rgb=args.include_rgb,
        )

    # Config 저장만
    if args.save_config:
        from corrective.phase1.robomimic_config import save_robomimic_config

        save_robomimic_config(config, args.save_config)
        print(f"Config 저장 완료: {args.save_config}")
        return

    # 학습 실행
    print("\n" + "=" * 50)
    print("  BC-RNN 정책 학습 (Phase 1)")
    print("=" * 50)
    print(f"  데이터셋: {args.dataset}")
    print(f"  출력: {args.output_dir}")
    print(f"  에폭: {config['train']['num_epochs']}")
    print(f"  배치: {config['train']['batch_size']}")
    print(f"  RNN: {config['algo']['rnn']['rnn_type']} "
          f"({config['algo']['rnn']['num_layers']}L, "
          f"h={config['algo']['rnn']['hidden_dim']})")
    print(f"  GMM: {config['algo']['gmm']['num_modes']} modes")
    print("=" * 50)

    try:
        import robomimic.utils.train_utils as TrainUtils
        from robomimic.config import config_factory

        # Robomimic config 객체 생성
        ext_cfg = config_factory(config["algo_name"])
        with ext_cfg.values_unlocked():
            ext_cfg.update(config)

        # 학습 실행
        TrainUtils.train(ext_cfg, auto_remove_exp=False)

        print("\n[Train] 학습 완료!")
        print(f"  모델: {args.output_dir}/model_best.pth")

    except ImportError:
        print("\n[WARNING] robomimic 미설치. 독립 학습 모드로 실행합니다.")
        _train_standalone(config)


def _train_standalone(config: dict):
    """Robomimic 없이 독립 학습 (RTX PRO 6000 BF16 최적화)."""
    import torch
    import h5py
    import numpy as np
    from torch.utils.data import DataLoader, TensorDataset

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from corrective.phase1.bc_rnn_policy import SimpleBCRNN

    dataset_path = config["train"]["data"]
    output_dir = Path(config["train"]["output_dir"])
    output_dir.mkdir(parents=True, exist_ok=True)

    # 데이터 로드
    print("[Standalone] 데이터 로드 중...")
    with h5py.File(dataset_path, "r") as f:
        all_obs = []
        all_actions = []
        for key in sorted(f["data"].keys()):
            all_obs.append(f["data"][key]["obs"][:])
            all_actions.append(f["data"][key]["actions"][:])

    obs = np.concatenate(all_obs, axis=0)
    actions = np.concatenate(all_actions, axis=0)
    print(f"  데이터: {obs.shape[0]} 스텝, obs={obs.shape[1]}D, action={actions.shape[1]}D")

    obs_t = torch.tensor(obs, dtype=torch.float32)
    act_t = torch.tensor(actions, dtype=torch.float32)
    dataset = TensorDataset(obs_t, act_t)
    dataloader = DataLoader(
        dataset,
        batch_size=config["train"]["batch_size"],
        shuffle=True,
        num_workers=8,
        pin_memory=True,
    )

    # 모델 생성
    model = SimpleBCRNN(
        obs_dim=obs.shape[1],
        action_dim=actions.shape[1],
        hidden_dim=config["algo"]["rnn"]["hidden_dim"],
        num_layers=config["algo"]["rnn"]["num_layers"],
        mlp_dims=config["algo"]["actor_layer_dims"],
    ).cuda()

    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=config["algo"]["optim_params"]["policy"]["learning_rate"]["initial"],
    )

    # LR 스케줄러: MultiStep [1000, 1500] × 0.1
    num_epochs = config["train"]["num_epochs"]
    scheduler = torch.optim.lr_scheduler.MultiStepLR(
        optimizer, milestones=[num_epochs // 2, int(num_epochs * 0.75)], gamma=0.1
    )

    # BF16 혼합 정밀도 (Blackwell 최적화)
    use_bf16 = torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8
    amp_dtype = torch.bfloat16 if use_bf16 else torch.float32
    print(f"  BF16 혼합 정밀도: {use_bf16}")
    print(f"  배치 크기: {config['train']['batch_size']}")

    best_loss = float("inf")

    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0.0
        n = 0

        for obs_batch, act_batch in dataloader:
            obs_batch = obs_batch.cuda()
            act_batch = act_batch.cuda()

            model.reset_hidden()

            with torch.amp.autocast("cuda", dtype=amp_dtype, enabled=use_bf16):
                pred = model(obs_batch)
                loss = ((pred - act_batch) ** 2).mean()

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()

            epoch_loss += loss.item()
            n += 1

        avg_loss = epoch_loss / max(n, 1)
        scheduler.step()

        if avg_loss < best_loss:
            best_loss = avg_loss
            torch.save(model.state_dict(), output_dir / "model_best.pth")

        if (epoch + 1) % 100 == 0:
            lr = optimizer.param_groups[0]["lr"]
            print(f"  Epoch {epoch+1}/{num_epochs}, Loss: {avg_loss:.6f}, LR: {lr:.2e}")

    print(f"\n[Standalone] 학습 완료. Best loss: {best_loss:.6f}")
    print(f"  모델: {output_dir / 'model_best.pth'}")


if __name__ == "__main__":
    main()
