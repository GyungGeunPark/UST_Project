"""Phase 3: 불확실성 기반 도움 요청 루프.

3-Tier 추론 + 앙상블 정책 + Conformal Prediction을 통합하여
불확실할 때 자동으로 사람에게 도움을 요청합니다.

RTX PRO 6000 최적화:
  - Tier 1: SigLIP2 빠른 분류 (~2GB, 매 스텝 <10ms)
  - Tier 2: Florence-2 물체 탐지 (~4GB, 30스텝 <50ms)
  - Tier 3: Qwen3-VL 상세 분석 (로컬 SGLang, 불확실 시만)

사용법:
    # 기본 실행 (VLM 모의 모드, SigLIP2 + Florence-2)
    python scripts/run_uncertainty_loop.py \
        --ensemble_dir ./models/ensemble/ \
        --num_models 5

    # 로컬 VLM (SGLang 서버 필요)
    python scripts/run_uncertainty_loop.py \
        --ensemble_dir ./models/ensemble/ \
        --num_models 5 \
        --vlm_model Qwen/Qwen3-VL-8B-Instruct \
        --vlm_base_url http://localhost:8000/v1

    # 전체 3-Tier + VR 연동
    python scripts/run_uncertainty_loop.py \
        --ensemble_dir ./models/ensemble/ \
        --num_models 5 \
        --enable_siglip --enable_florence \
        --vlm_model Qwen/Qwen3-VL-32B-Instruct \
        --enable_vr --with_sim_camera
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import torch
import numpy as np

# 기본 모델 경로
DEFAULT_MODELS_DIR = os.environ.get(
    "UST_MODELS_DIR",
    "/workspace/isaaclab/ust_ws/models",
)


def main():
    parser = argparse.ArgumentParser(
        description="불확실성 기반 도움 요청 루프 (Phase 3)"
    )

    # 앙상블 설정
    parser.add_argument(
        "--ensemble_dir",
        type=str,
        default=os.path.join(DEFAULT_MODELS_DIR, "ensemble"),
        help="앙상블 모델 디렉토리",
    )
    parser.add_argument("--num_models", type=int, default=5)

    # VLM 설정 (Tier 3: 로컬 SGLang/vLLM)
    parser.add_argument(
        "--vlm_model", type=str, default="Qwen/Qwen3-VL-8B-Instruct",
        help="VLM 모델명 (로컬 서버에 로드된 모델)",
    )
    parser.add_argument(
        "--vlm_base_url", type=str, default="http://localhost:8000/v1",
        help="로컬 VLM 서버 URL (SGLang/vLLM)",
    )
    parser.add_argument("--vlm_provider", type=str, default="openai")
    parser.add_argument("--vlm_interval", type=int, default=30,
                        help="VLM 호출 간격 (스텝)")

    # SigLIP2 설정 (Tier 1: 빠른 분류)
    parser.add_argument("--enable_siglip", action="store_true",
                        help="SigLIP2 빠른 분류기 활성화")
    parser.add_argument(
        "--siglip_model_dir", type=str,
        default=os.path.join(DEFAULT_MODELS_DIR, "siglip2-so400m"),
        help="SigLIP2 모델 디렉토리",
    )

    # Florence-2 설정 (Tier 2: 물체 탐지)
    parser.add_argument("--enable_florence", action="store_true",
                        help="Florence-2 물체 탐지기 활성화")
    parser.add_argument(
        "--florence_model_dir", type=str,
        default=os.path.join(DEFAULT_MODELS_DIR, "florence2-large"),
        help="Florence-2 모델 디렉토리",
    )
    parser.add_argument("--tier2_interval", type=int, default=30,
                        help="Florence-2 호출 간격 (스텝)")

    # Conformal 설정
    parser.add_argument("--conformal_data", type=str, default=None)
    parser.add_argument("--conformal_alpha", type=float, default=0.05)

    # 임계값 설정
    parser.add_argument("--initial_threshold", type=float, default=0.05)
    parser.add_argument("--target_intervention_ratio", type=float, default=0.15)

    # 실행 설정
    parser.add_argument("--num_episodes", type=int, default=20)
    parser.add_argument("--max_steps", type=int, default=1200)
    parser.add_argument("--enable_vr", action="store_true")
    parser.add_argument("--with_sim_camera", action="store_true",
                        help="시뮬레이션 카메라 이미지 사용 (VLM 분석용)")

    # 데이터 저장
    parser.add_argument("--data_dir", type=str, default="./data/uncertainty/")
    parser.add_argument("--model_dir", type=str, default="./models/phase3/")

    args = parser.parse_args()

    # 앙상블 모델 확인
    ensemble_dir = Path(args.ensemble_dir)
    if not ensemble_dir.exists():
        print(f"[ERROR] 앙상블 디렉토리 없음: {ensemble_dir}")
        sys.exit(1)

    model_paths = []
    for k in range(args.num_models):
        path = ensemble_dir / f"ensemble_{k}" / "model_best.pth"
        if path.exists():
            model_paths.append(str(path))
        else:
            print(f"[WARNING] 모델 없음: {path}")

    if not model_paths:
        print("[ERROR] 로드 가능한 앙상블 모델 없음")
        sys.exit(1)

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

    print("\n" + "=" * 60)
    print("  불확실성 기반 도움 요청 루프 (Phase 3)")
    print("  RTX PRO 6000 (96GB) 3-Tier 아키텍처")
    print("=" * 60)
    print(f"  앙상블 모델: {len(model_paths)}개")
    print(f"  Tier 1 (SigLIP2): {'활성' if args.enable_siglip else '비활성'}")
    print(f"  Tier 2 (Florence-2): {'활성' if args.enable_florence else '비활성'}")
    print(f"  Tier 3 (VLM): {args.vlm_model}")
    print(f"  VLM 서버: {args.vlm_base_url}")
    print(f"  VLM 호출 간격: {args.vlm_interval} 스텝")
    print(f"  임계값: {args.initial_threshold}")
    print(f"  목표 개입률: {args.target_intervention_ratio:.0%}")
    print(f"  에피소드: {args.num_episodes}")
    print(f"  VR: {args.enable_vr}")
    print(f"  시뮬레이션 카메라: {args.with_sim_camera}")
    print("=" * 60)

    # 모듈 초기화
    from corrective.phase3.ensemble_policy import EnsemblePolicy
    from corrective.phase3.vlm_analyzer import (
        VLMObjectAnalyzer, CachedVLMAnalyzer,
        SigLIP2Classifier, Florence2Detector,
    )
    from corrective.phase3.conformal_predictor import ConformalCategoryPredictor
    from corrective.phase3.adaptive_threshold import AdaptiveThreshold
    from corrective.phase3.help_request_decider import HelpRequestDecider

    # 1. 앙상블 정책
    ensemble = EnsemblePolicy.from_checkpoints(
        model_paths,
        uncertainty_threshold=args.initial_threshold,
    )

    # 2. Tier 1: SigLIP2 빠른 분류기
    siglip = None
    if args.enable_siglip:
        siglip = SigLIP2Classifier(model_path=args.siglip_model_dir)
        siglip.load()

    # 3. Tier 2: Florence-2 물체 탐지기
    florence = None
    if args.enable_florence:
        florence = Florence2Detector(model_path=args.florence_model_dir)
        florence.load()

    # 4. Tier 3: VLM 분석기 (로컬 SGLang/vLLM)
    vlm_base = VLMObjectAnalyzer(
        model=args.vlm_model,
        provider=args.vlm_provider,
        base_url=args.vlm_base_url,
    )
    cached_vlm = CachedVLMAnalyzer(
        vlm_base,
        call_interval=args.vlm_interval,
        siglip_classifier=siglip,
        florence_detector=florence,
        tier2_interval=args.tier2_interval,
    )

    # 5. Conformal predictor
    conformal = ConformalCategoryPredictor(alpha=args.conformal_alpha)
    if args.conformal_data and Path(args.conformal_data).exists():
        data = np.load(args.conformal_data)
        calibration_data = list(zip(data["probs"], data["labels"]))
        conformal.calibrate(calibration_data)
    else:
        print("[INFO] Conformal 교정 데이터 없음. 보수적 모드 사용.")

    # 6. 적응적 임계값
    adaptive_threshold = AdaptiveThreshold(
        initial_threshold=args.initial_threshold,
        target_intervention_ratio=args.target_intervention_ratio,
    )

    # 7. 도움 요청 결정기 (SigLIP2 통합)
    decider = HelpRequestDecider(
        ensemble_policy=ensemble,
        vlm_analyzer=cached_vlm if args.with_sim_camera else None,
        conformal_predictor=conformal,
        adaptive_threshold=adaptive_threshold,
        siglip_classifier=siglip,
    )

    print("\n[INFO] 모듈 초기화 완료.")

    # VRAM 사용량 추정
    vram_estimate = 7.0  # Isaac Sim 기본
    if siglip is not None and siglip.is_loaded:
        vram_estimate += 2.0
    if florence is not None and florence.is_loaded:
        vram_estimate += 4.0
    print(f"  예상 VRAM (Isaac Sim + 분류기): ~{vram_estimate:.0f} GB")
    print(f"  VLM 서버 별도 VRAM 필요: ~19-75 GB (모델 크기에 따라)")
    print(f"  앙상블 추론: ~1 GB")

    print("\n  실제 실행을 위해서는 Isaac Lab 환경이 필요합니다.")
    print("  Isaac Lab AppLauncher로 실행해주세요:")
    print()

    task_id = (
        "Isaac-KitchenSorting-G1-InspireFTP-DataCollect-v0"
        if args.with_sim_camera
        else "Isaac-KitchenSorting-G1-InspireFTP-v0"
    )

    print(f"  isaaclab -p {__file__} \\")
    print(f"    --ensemble_dir {args.ensemble_dir} \\")
    print(f"    --num_models {args.num_models} \\")
    if args.enable_siglip:
        print(f"    --enable_siglip \\")
    if args.enable_florence:
        print(f"    --enable_florence \\")
    print(f"    --vlm_model {args.vlm_model} \\")
    print(f"    --vlm_base_url {args.vlm_base_url} \\")
    if args.with_sim_camera:
        print(f"    --with_sim_camera \\")
    if args.enable_vr:
        print(f"    --enable_vr \\")
    print(f"    --task {task_id}")

    # 통계 출력
    print("\n=== 시스템 구성 요약 ===")
    print(f"  앙상블: {ensemble.K}개 모델, threshold={ensemble.threshold}")
    print(f"  Tier 1 (SigLIP2): {'로드됨' if siglip and siglip.is_loaded else '비활성'}")
    print(f"  Tier 2 (Florence-2): {'로드됨' if florence and florence.is_loaded else '비활성'}")
    print(f"  Tier 3 (VLM): {vlm_base.model}, base_url={vlm_base.base_url}")
    print(f"  Conformal: {conformal.get_stats()}")
    print(f"  Threshold: {adaptive_threshold.get_stats()}")


if __name__ == "__main__":
    main()
