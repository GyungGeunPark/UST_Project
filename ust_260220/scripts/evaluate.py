"""전체 평가 스크립트.

Phase 1/2/3 정책을 일관된 방식으로 평가합니다.

사용법:
    # Phase 1 BC-RNN 평가
    python scripts/evaluate.py \
        --checkpoint ./models/bc_rnn/model_best.pth \
        --phase 1

    # Phase 2 HG-DAgger 평가
    python scripts/evaluate.py \
        --checkpoint ./models/hg_dagger/iter_3/model_best.pth \
        --phase 2

    # Phase 3 앙상블 평가
    python scripts/evaluate.py \
        --ensemble_dir ./models/ensemble/ \
        --phase 3 --num_models 5

    # 전체 비교
    python scripts/evaluate.py --compare \
        --phase1_checkpoint ./models/bc_rnn/model_best.pth \
        --phase2_checkpoint ./models/hg_dagger/model_best.pth \
        --ensemble_dir ./models/ensemble/
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="교정 학습 시스템 평가")

    # 평가 대상
    parser.add_argument("--checkpoint", type=str, default=None)
    parser.add_argument("--ensemble_dir", type=str, default=None)
    parser.add_argument("--num_models", type=int, default=5)
    parser.add_argument("--phase", type=int, choices=[1, 2, 3], default=1)

    # 비교 모드
    parser.add_argument("--compare", action="store_true")
    parser.add_argument("--phase1_checkpoint", type=str, default=None)
    parser.add_argument("--phase2_checkpoint", type=str, default=None)

    # 평가 설정
    parser.add_argument("--num_episodes", type=int, default=50)
    parser.add_argument("--max_steps", type=int, default=1200)
    parser.add_argument(
        "--models_dir", type=str,
        default="/workspace/isaaclab/ust_ws/models",
        help="사전학습 모델 기본 디렉토리",
    )

    # 출력
    parser.add_argument("--output", type=str, default=None,
                        help="결과 JSON 저장 경로")

    args = parser.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

    print("\n" + "=" * 50)
    print("  교정 학습 시스템 평가")
    print("=" * 50)

    if args.compare:
        _compare_phases(args)
    else:
        _evaluate_single(args)


def _evaluate_single(args):
    """단일 정책 평가."""
    if args.phase == 3 and args.ensemble_dir:
        _evaluate_ensemble(args)
    elif args.checkpoint:
        _evaluate_checkpoint(args)
    else:
        print("[ERROR] --checkpoint 또는 --ensemble_dir을 지정해주세요.")
        sys.exit(1)


def _evaluate_checkpoint(args):
    """체크포인트 기반 평가."""
    from corrective.phase1.bc_rnn_policy import BCRNNPolicy, BCRNNPolicyCfg

    if not Path(args.checkpoint).exists():
        print(f"[ERROR] 체크포인트 없음: {args.checkpoint}")
        sys.exit(1)

    policy = BCRNNPolicy(BCRNNPolicyCfg(), checkpoint_path=args.checkpoint)
    print(f"  정책: {args.checkpoint}")
    print(f"  Phase: {args.phase}")
    print(f"  에피소드: {args.num_episodes}")
    print(f"\n  [INFO] 실제 평가를 위해서는 Isaac Lab 환경이 필요합니다.")
    print(f"  isaaclab -p {__file__} --checkpoint {args.checkpoint}")


def _evaluate_ensemble(args):
    """앙상블 정책 평가."""
    from corrective.phase3.ensemble_policy import EnsemblePolicy

    ensemble_dir = Path(args.ensemble_dir)
    model_paths = []
    for k in range(args.num_models):
        path = ensemble_dir / f"ensemble_{k}" / "model_best.pth"
        if path.exists():
            model_paths.append(str(path))

    if not model_paths:
        print(f"[ERROR] 앙상블 모델 없음: {ensemble_dir}")
        sys.exit(1)

    print(f"  앙상블: {len(model_paths)}개 모델")
    print(f"  에피소드: {args.num_episodes}")
    print(f"\n  [INFO] 실제 평가를 위해서는 Isaac Lab 환경이 필요합니다.")


def _compare_phases(args):
    """Phase 1/2/3 비교 평가."""
    print("  비교 모드")
    print()

    results = {}

    if args.phase1_checkpoint:
        print(f"  Phase 1: {args.phase1_checkpoint}")
        results["phase1"] = {"checkpoint": args.phase1_checkpoint}

    if args.phase2_checkpoint:
        print(f"  Phase 2: {args.phase2_checkpoint}")
        results["phase2"] = {"checkpoint": args.phase2_checkpoint}

    if args.ensemble_dir:
        print(f"  Phase 3: {args.ensemble_dir}")
        results["phase3"] = {"ensemble_dir": args.ensemble_dir}

    print(f"\n  [INFO] 실제 비교 평가를 위해서는 Isaac Lab 환경이 필요합니다.")
    print(f"  각 정책을 {args.num_episodes}개 에피소드로 평가합니다.")

    if args.output:
        import json
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"\n  설정 저장: {args.output}")


if __name__ == "__main__":
    main()
