"""Phase 1: MimicGen 데이터 증강.

50개 VR 시연 → 1,000+ 증강 궤적을 생성합니다.

사용법:
    python scripts/augment_demos.py \
        --source ./data/demos/kitchen_sorting_demos.hdf5 \
        --output ./data/demos/kitchen_sorting_augmented.hdf5 \
        --target_trajectories 1000
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="MimicGen 데이터 증강")

    parser.add_argument(
        "--source",
        type=str,
        default="./data/demos/kitchen_sorting_demos.hdf5",
        help="원본 시연 HDF5 경로",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="./data/demos/kitchen_sorting_augmented.hdf5",
        help="증강 결과 HDF5 경로",
    )
    parser.add_argument(
        "--target_trajectories",
        type=int,
        default=1000,
        help="목표 궤적 수",
    )
    parser.add_argument("--num_trials", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--stats_only", action="store_true",
                        help="통계만 출력")

    args = parser.parse_args()

    # 원본 데이터 확인
    if not Path(args.source).exists():
        print(f"[ERROR] 원본 데이터 없음: {args.source}")
        print("  먼저 시연 수집을 완료해주세요:")
        print("  python scripts/record_demos.py --num_demos 50")
        sys.exit(1)

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from corrective.phase1.mimicgen_augmentor import (
        MimicGenAugmentor,
        DataGenConfig,
    )

    # 설정
    datagen_config = DataGenConfig(
        num_trials=args.num_trials,
        target_num_trajectories=args.target_trajectories,
        seed=args.seed,
    )

    augmentor = MimicGenAugmentor(
        source_hdf5_path=args.source,
        output_hdf5_path=args.output,
        datagen_config=datagen_config,
    )

    if args.stats_only:
        # 기존 증강 데이터 통계
        if Path(args.output).exists():
            stats = augmentor.get_statistics()
            print("\n=== 증강 데이터 통계 ===")
            for k, v in stats.items():
                print(f"  {k}: {v}")
        else:
            print(f"증강 데이터 없음: {args.output}")
        return

    # 증강 실행
    print("\n" + "=" * 50)
    print("  MimicGen 데이터 증강 (Phase 1)")
    print("=" * 50)
    print(f"  원본: {args.source}")
    print(f"  출력: {args.output}")
    print(f"  목표 궤적: {args.target_trajectories}")
    print("=" * 50)

    output_path = augmentor.generate()

    # 결과 통계
    stats = augmentor.get_statistics()
    print("\n=== 증강 결과 ===")
    for k, v in stats.items():
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
