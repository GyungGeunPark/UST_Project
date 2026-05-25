"""Phase 2: HG-DAgger 교정 루프 실행.

BC-RNN 기본 정책 위에서 HG-DAgger 교정 루프를 실행합니다.
VR 텔레오퍼레이션으로 사람이 실시간 교정하고, IWR 가중치로 재학습합니다.

사용법:
    # 기본 실행
    python scripts/run_hg_dagger.py \
        --checkpoint ./models/bc_rnn/model_best.pth

    # 커스텀 설정
    python scripts/run_hg_dagger.py \
        --checkpoint ./models/bc_rnn/model_best.pth \
        --max_iterations 5 \
        --episodes_per_iteration 20 \
        --intervention_weight 5.0

    # VR 텔레오퍼레이션 (실제 실행 시)
    python scripts/run_hg_dagger.py \
        --checkpoint ./models/bc_rnn/model_best.pth \
        --enable_vr
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch


def main():
    parser = argparse.ArgumentParser(description="HG-DAgger 교정 루프")

    # 정책
    parser.add_argument(
        "--checkpoint",
        type=str,
        required=True,
        help="BC-RNN 체크포인트 경로",
    )

    # HG-DAgger 설정
    parser.add_argument("--max_iterations", type=int, default=5)
    parser.add_argument("--episodes_per_iteration", type=int, default=20)
    parser.add_argument("--eval_episodes", type=int, default=50)
    parser.add_argument("--target_success_rate", type=float, default=0.8)
    parser.add_argument("--max_steps_per_episode", type=int, default=1200)

    # IWR 설정
    parser.add_argument("--intervention_weight", type=float, default=5.0)
    parser.add_argument("--iwr_epochs", type=int, default=200)
    parser.add_argument("--iwr_lr", type=float, default=5e-5)

    # 데이터/모델 경로
    parser.add_argument("--data_dir", type=str, default="./data/corrective/")
    parser.add_argument("--model_dir", type=str, default="./models/hg_dagger/")

    # VR / 디바이스
    parser.add_argument("--enable_vr", action="store_true")
    parser.add_argument("--teleop_device", type=str, default="handtracking",
                        choices=["handtracking", "manusvive", "pico", "pico_no_udcap"],
                        help="텔레오퍼레이션 디바이스 (PICO + UDCAP 지원)")
    parser.add_argument("--num_envs", type=int, default=1)

    args = parser.parse_args()

    # 체크포인트 확인
    if not Path(args.checkpoint).exists():
        print(f"[ERROR] 체크포인트 없음: {args.checkpoint}")
        print("  먼저 BC-RNN 학습을 완료해주세요:")
        print("  python scripts/train_bc_rnn.py --dataset ...")
        sys.exit(1)

    # 환경 설정 (Isaac Lab AppLauncher 필요)
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

    print("\n" + "=" * 50)
    print("  HG-DAgger 교정 루프 (Phase 2)")
    print("=" * 50)
    print(f"  체크포인트: {args.checkpoint}")
    print(f"  최대 반복: {args.max_iterations}")
    print(f"  반복당 에피소드: {args.episodes_per_iteration}")
    print(f"  개입 가중치: {args.intervention_weight}")
    print(f"  VR 활성화: {args.enable_vr}")
    print(f"  텔레옵 디바이스: {args.teleop_device}")
    print("=" * 50)

    # 정책 로드
    from corrective.phase1.bc_rnn_policy import BCRNNPolicy, BCRNNPolicyCfg

    policy_cfg = BCRNNPolicyCfg()
    policy = BCRNNPolicy(policy_cfg, checkpoint_path=args.checkpoint)

    # HG-DAgger 루프 설정
    from corrective.phase2.hg_dagger_loop import HGDAggerLoop, HGDAggerConfig
    from corrective.phase2.iwr_trainer import IWRConfig

    iwr_config = IWRConfig(
        intervention_weight=args.intervention_weight,
        num_epochs=args.iwr_epochs,
        learning_rate=args.iwr_lr,
        save_dir=args.model_dir,
    )

    hg_config = HGDAggerConfig(
        max_iterations=args.max_iterations,
        target_success_rate=args.target_success_rate,
        episodes_per_iteration=args.episodes_per_iteration,
        max_steps_per_episode=args.max_steps_per_episode,
        eval_episodes=args.eval_episodes,
        iwr_config=iwr_config,
        data_dir=args.data_dir,
        model_dir=args.model_dir,
    )

    # 환경은 실행 시 Isaac Lab에서 생성
    # 여기서는 설정만 출력
    print("\n[INFO] HG-DAgger 설정 완료.")
    print("  실제 실행을 위해서는 Isaac Lab 환경이 필요합니다.")
    print("  Isaac Lab AppLauncher로 실행해주세요:")
    print()

    if args.enable_vr:
        device_flag = f"--teleop_device {args.teleop_device}"
        if args.teleop_device in ("pico", "pico_no_udcap"):
            print("  # PICO 모드 (실제 교정)")
            print("  # 사전: 호스트에서 source setup_pico_env.sh 실행 필요")
            print(f"  isaaclab -p {__file__} \\")
            print(f"    --checkpoint {args.checkpoint} \\")
            print(f"    --enable_vr {device_flag} \\")
            print(f"    --task Isaac-KitchenSorting-G1-InspireFTP-VR-v0")
            print()
            print("  # PICO 개입 트리거: 양손 그립 동시 / 복귀: 양손 트리거 동시")
        else:
            print("  # Quest/VisionPro VR 모드 (실제 교정)")
            print(f"  isaaclab -p {__file__} \\")
            print(f"    --checkpoint {args.checkpoint} \\")
            print(f"    --enable_vr \\")
            print(f"    --task Isaac-KitchenSorting-G1-InspireFTP-VR-v0")
    else:
        print("  # 시뮬레이션 모드 (자동 교정 테스트)")
        print(f"  isaaclab -p {__file__} \\")
        print(f"    --checkpoint {args.checkpoint} \\")
        print(f"    --task Isaac-KitchenSorting-G1-InspireFTP-v0")


if __name__ == "__main__":
    main()
