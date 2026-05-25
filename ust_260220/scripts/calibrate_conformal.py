"""Phase 3: Conformal Prediction 교정.

VLM의 카테고리 예측에 대해 conformal prediction을 교정하여
형식적 보장을 추가합니다.

사용법:
    # 교정 데이터 생성 (시뮬레이션 이미지 + 정답 레이블)
    python scripts/calibrate_conformal.py \
        --dataset ./data/demos/kitchen_sorting_demos.hdf5 \
        --output ./models/conformal/calibration.npz

    # 기존 교정 데이터로 임계값 설정
    python scripts/calibrate_conformal.py \
        --calibration_data ./models/conformal/calibration.npz \
        --alpha 0.05
"""

from __future__ import annotations

import argparse
import json
import sys
import numpy as np
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Conformal Prediction 교정")

    parser.add_argument(
        "--calibration_data",
        type=str,
        default=None,
        help="기존 교정 데이터 (.npz) 경로",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="./models/conformal/calibration.npz",
        help="교정 결과 저장 경로",
    )
    parser.add_argument("--alpha", type=float, default=0.05,
                        help="오류율 (0.05 = 95% 커버리지)")
    parser.add_argument("--num_samples", type=int, default=100,
                        help="교정 데이터 샘플 수 (시뮬레이션 생성 시)")
    parser.add_argument("--vlm_model", type=str, default="Qwen/Qwen3-VL-8B-Instruct")
    parser.add_argument("--vlm_provider", type=str, default="openai")
    parser.add_argument(
        "--vlm_base_url", type=str, default="http://localhost:8000/v1",
        help="로컬 VLM 서버 URL (SGLang/vLLM)",
    )
    parser.add_argument("--use_mock", action="store_true",
                        help="VLM API 대신 모의 데이터 사용")

    args = parser.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from corrective.phase3.conformal_predictor import ConformalCategoryPredictor

    print("\n" + "=" * 50)
    print("  Conformal Prediction 교정 (Phase 3)")
    print("=" * 50)
    print(f"  α (오류율): {args.alpha}")
    print(f"  VLM 모델: {args.vlm_model}")
    print("=" * 50)

    predictor = ConformalCategoryPredictor(
        categories=["kitchen", "food", "misc"],
        alpha=args.alpha,
    )

    if args.calibration_data and Path(args.calibration_data).exists():
        # 기존 교정 데이터 로드
        data = np.load(args.calibration_data)
        probs = data["probs"]
        labels = data["labels"]
        calibration_data = list(zip(probs, labels))
        print(f"\n  교정 데이터 로드: {len(calibration_data)}개 샘플")
    else:
        # 모의 교정 데이터 생성
        print(f"\n  모의 교정 데이터 생성: {args.num_samples}개 샘플")
        calibration_data = _generate_mock_calibration(args.num_samples)

        # 저장
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        probs = np.array([p for p, _ in calibration_data])
        labels = np.array([l for _, l in calibration_data])
        np.savez(output_path, probs=probs, labels=labels)
        print(f"  교정 데이터 저장: {output_path}")

    # 교정 실행
    predictor.calibrate(calibration_data)

    # 결과
    stats = predictor.get_stats()
    print(f"\n=== 교정 결과 ===")
    for k, v in stats.items():
        print(f"  {k}: {v}")

    # 테스트 예측
    print(f"\n=== 테스트 예측 ===")
    test_cases = [
        ("mug", np.array([0.9, 0.05, 0.05])),      # 확신
        ("unknown_item", np.array([0.33, 0.33, 0.34])),  # 불확실
        ("can", np.array([0.15, 0.7, 0.15])),       # 확신 (food)
        ("weird_object", np.array([0.4, 0.35, 0.25])),   # 약간 불확실
    ]

    for name, probs in test_cases:
        pred_set, should_ask = predictor.predict(name, probs)
        status = "질문 필요" if should_ask else "자율 결정"
        print(f"  {name}: {pred_set} [{status}]")


def _generate_mock_calibration(num_samples: int):
    """모의 교정 데이터 생성."""
    rng = np.random.RandomState(42)
    calibration_data = []

    categories = ["kitchen", "food", "misc"]

    for _ in range(num_samples):
        true_label = rng.randint(0, 3)

        # VLM이 대체로 정확하지만 가끔 틀리는 확률 분포 생성
        if rng.rand() < 0.8:  # 80% 정확
            probs = rng.dirichlet([10, 1, 1] if true_label == 0 else
                                  [1, 10, 1] if true_label == 1 else
                                  [1, 1, 10])
        else:  # 20% 부정확
            probs = rng.dirichlet([1, 1, 1])

        calibration_data.append((probs.astype(np.float64), int(true_label)))

    return calibration_data


if __name__ == "__main__":
    main()
