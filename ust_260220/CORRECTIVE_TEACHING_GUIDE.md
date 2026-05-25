# 교정 티칭 AI 시스템 실행 가이드

> 작성일: 2026-02-20 (RTX PRO 6000 최적화 업데이트)
> 대상 환경: G1 Kitchen Sorting (`Isaac-KitchenSorting-G1-InspireFTP-v0`)
> 로봇: Unitree G1 + INSPIRE 5지 핸드 (Pink IK 38D)
> GPU: NVIDIA RTX PRO 6000 (96GB GDDR7, Blackwell)

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [사전 요구사항](#2-사전-요구사항)
3. [사전학습 모델 설정](#3-사전학습-모델-설정)
4. [Phase 1: 기초 모방 학습](#4-phase-1-기초-모방-학습)
5. [Phase 2: HG-DAgger 교정 루프](#5-phase-2-hg-dagger-교정-루프)
6. [Phase 3: 불확실성 기반 도움 요청](#6-phase-3-불확실성-기반-도움-요청)
7. [전체 평가](#7-전체-평가)
8. [디렉토리 구조](#8-디렉토리-구조)
9. [문제 해결](#9-문제-해결)

---

## 1. 시스템 개요

### 1.1 3-Phase 로드맵

| Phase | 목표 | 핵심 기술 | 기간 |
|-------|------|----------|------|
| **Phase 1** | 기초 인프라 구축 | BC-RNN + MimicGen | 2-3주 |
| **Phase 2** | 교정 루프 | HG-DAgger + IWR | 2-3주 |
| **Phase 3** | 불확실성 도움 요청 | Ensemble + 3-Tier VLM + Conformal | 3-4주 |

### 1.2 3-Tier 추론 아키텍처 (RTX PRO 6000 최적화)

```
Tier 1: SigLIP2 빠른 분류 (~2GB, 매 스텝 <10ms)
  └── 이미지 → 카테고리 확률 [kitchen, food, misc]

Tier 2: Florence-2 물체 탐지 (~4GB, 30스텝마다 <50ms)
  └── 이미지 → 바운딩박스 + 물체 레이블

Tier 3: Qwen3-VL 상세 분석 (19-75GB, 불확실 시만 ~1-2초)
  └── 이미지 → 상세 물체 분석 + 카테고리 판단
```

### 1.3 VRAM 사용 프로파일

```
Phase 1:   ~8 GB / 96 GB (8%)   ← 대형 배치로 빠른 학습
Phase 2:  ~14 GB / 96 GB (15%)  ← SigLIP2 + Florence-2 + IWR 학습
Phase 3:  ~33 GB / 96 GB (34%)  ← Qwen3-VL-8B + 학습 동시
최종 운용: ~85 GB / 96 GB (89%)  ← Qwen3-VL-32B 최대 활용
```

### 1.4 핵심 구성 요소

```
corrective/
├── phase1/          # BC-RNN 정책, MimicGen 증강, Robomimic 설정
├── phase2/          # InterventionManager, IWR Trainer, HG-DAgger Loop
├── phase3/          # Ensemble, VLM(3-Tier), Conformal, HelpDecider, Threshold
└── utils/           # HDF5 레코더, 평가기
```

---

## 2. 사전 요구사항

### 2.1 환경 설정

```bash
# Isaac Lab 2.3.0 컨테이너 내부
cd /workspace/isaaclab

# 필수 패키지
pip install scikit-learn        # 다중 모달 불확실성 (Phase 3)
pip install transformers        # SigLIP2, Florence-2 (Phase 3)
pip install openai              # VLM API 클라이언트 (로컬/클라우드 공용)

# 선택 패키지
pip install robomimic           # BC-RNN 학습 (Phase 1, 고급)
pip install "sglang[all]>=0.4"  # 로컬 VLM 서빙 (Phase 3)
# pip install vllm>=0.7         # SGLang 대안
```

### 2.2 데이터 디렉토리 생성

```bash
mkdir -p ust_ws/ust_260220/data/demos
mkdir -p ust_ws/ust_260220/data/corrective
mkdir -p ust_ws/ust_260220/data/uncertainty
```

### 2.3 VR 텔레오퍼레이션 (Phase 1-2 필수)

- CloudXR Runtime 6.0.1-webrtc Docker 실행
- Quest 3S 또는 Apple Vision Pro 연결
- OpenXR 환경변수 설정 (`XR_RUNTIME_JSON`, `IPC_IGNORE_VERSION=1`)
- 자세한 설정은 `EXECUTION_GUIDE.md` 참조

---

## 3. 사전학습 모델 설정

### 3.1 모델 경로

모든 사전학습 모델은 `/workspace/isaaclab/ust_ws/models/` 에 저장합니다.

```bash
# 환경변수 설정 (선택)
export UST_MODELS_DIR=/workspace/isaaclab/ust_ws/models
```

### 3.2 모델 다운로드

```bash
# HuggingFace CLI 설치 + 로그인
pip install -U huggingface_hub
huggingface-cli login  # 토큰: https://huggingface.co/settings/tokens

# Tier 1: SigLIP2 빠른 분류기 (~1.5GB)
huggingface-cli download google/siglip2-so400m-patch14-384 \
    --local-dir /workspace/isaaclab/ust_ws/models/siglip2-so400m \
    --local-dir-use-symlinks False

# Tier 2: Florence-2 물체 탐지기 (~3GB)
huggingface-cli download microsoft/Florence-2-large-ft \
    --local-dir /workspace/isaaclab/ust_ws/models/florence2-large \
    --local-dir-use-symlinks False

# Tier 3: VLM - 개발용 8B (~17GB)
huggingface-cli download Qwen/Qwen3-VL-8B-Instruct \
    --local-dir /workspace/isaaclab/ust_ws/models/qwen3-vl-8b \
    --local-dir-use-symlinks False

# Tier 3: VLM - 운용용 32B (~65GB, 선택)
huggingface-cli download Qwen/Qwen3-VL-32B-Instruct \
    --local-dir /workspace/isaaclab/ust_ws/models/qwen3-vl-32b \
    --local-dir-use-symlinks False
```

### 3.3 모델 디렉토리 구조

```
/workspace/isaaclab/ust_ws/models/
├── siglip2-so400m/        # ~1.5 GB (Tier 1: 빠른 분류)
├── florence2-large/       # ~3 GB (Tier 2: 물체 탐지)
├── qwen3-vl-8b/           # ~17 GB (Tier 3: 개발용 VLM)
├── qwen3-vl-32b/          # ~65 GB (Tier 3: 운용용 VLM, 선택)
├── bc_rnn/                # Phase 1 학습 모델 (자동 생성)
├── hg_dagger/             # Phase 2 학습 모델 (자동 생성)
├── ensemble/              # Phase 3 앙상블 모델 (자동 생성)
└── conformal/             # Conformal 교정 데이터 (자동 생성)
```

### 3.4 로컬 VLM 서버 실행

Phase 3에서 VLM 분석을 사용하려면 SGLang 서버를 실행해야 합니다.

```bash
# 개발 모드: Qwen3-VL-8B (19GB VRAM)
python -m sglang.launch_server \
    --model /workspace/isaaclab/ust_ws/models/qwen3-vl-8b \
    --port 8000 \
    --dtype float16 \
    --mem-fraction-static 0.25 \
    --max-running-requests 8 \
    --trust-remote-code

# 운용 모드: Qwen3-VL-32B (75GB VRAM)
python -m sglang.launch_server \
    --model /workspace/isaaclab/ust_ws/models/qwen3-vl-32b \
    --port 8000 \
    --dtype float16 \
    --mem-fraction-static 0.78 \
    --max-running-requests 4 \
    --trust-remote-code
```

> **참고**: SGLang 서버는 별도 터미널에서 실행합니다. OpenAI-compatible API를 제공하므로 기존 코드 수정 없이 `base_url`만 변경하면 됩니다.

---

## 4. Phase 1: 기초 모방 학습

### 4.1 단계 요약

```
VR 시연 수집 (50개) → MimicGen 증강 (1,000+) → BC-RNN 학습 → 기준선 평가
```

### 4.2 시연 수집

```bash
# VR 텔레오퍼레이션으로 시연 수집 (50개)
isaaclab -p ust_ws/ust_260220/scripts/record_demos.py \
    --num_demos 50 \
    --output_dir ./ust_ws/ust_260220/data/demos/

# 시뮬레이션 카메라 이미지 포함 수집 (Phase 3용)
isaaclab -p ust_ws/ust_260220/scripts/record_demos.py \
    --num_demos 50 \
    --output_dir ./ust_ws/ust_260220/data/demos/ \
    --with_sim_camera
```

**시연 수집 프로토콜:**
1. 환경 초기화 (물체 랜덤 배치)
2. VR HMD으로 양팔 제어 시작
3. 각 물체를 올바른 정리함으로 이동 (접근 → 파지 → 이동 → 해제)
4. 에피소드당 2~4개 물체 분류
5. 성공 에피소드만 저장

### 4.3 MimicGen 데이터 증강

```bash
cd ust_ws/ust_260220

# 50개 원본 → 1,000+ 증강 궤적
python scripts/augment_demos.py \
    --source ./data/demos/kitchen_sorting_demos.hdf5 \
    --output ./data/demos/kitchen_sorting_augmented.hdf5 \
    --target_trajectories 1000
```

### 4.4 BC-RNN 정책 학습 (RTX PRO 6000 최적 설정)

```bash
cd ust_ws/ust_260220

# RTX PRO 6000 최적 설정 (batch=128, 2000 epochs, BF16)
python scripts/train_bc_rnn.py \
    --dataset ./data/demos/kitchen_sorting_augmented.hdf5 \
    --output_dir /workspace/isaaclab/ust_ws/models/bc_rnn/ \
    --num_epochs 2000 \
    --batch_size 128

# Config만 생성 (학습 없이)
python scripts/train_bc_rnn.py \
    --dataset ./data/demos/kitchen_sorting_augmented.hdf5 \
    --save_config /workspace/isaaclab/ust_ws/models/bc_rnn/config.json
```

**학습 설정 (RTX PRO 6000):**
| 항목 | 값 | 비고 |
|------|-----|------|
| 배치 크기 | 128 | 96GB에서 충분 |
| 에폭 | 2,000 | ~25-35분 |
| 혼합 정밀도 | BF16 | Blackwell 최적화 |
| LR 스케줄러 | MultiStep [1000, 1500] | 0.1배 감쇠 |
| VRAM 사용 | ~1 GB | State-based |

### 4.5 학습 결과

| 항목 | 기대 값 |
|------|---------|
| 학습 데이터 | 1,000+ 궤적 (증강 후) |
| 모델 | `/workspace/isaaclab/ust_ws/models/bc_rnn/model_best.pth` |
| 학습 시간 | ~25-35분 (RTX PRO 6000) |

---

## 5. Phase 2: HG-DAgger 교정 루프

### 5.1 단계 요약

```
BC-RNN 정책 배포 → 사람이 VR로 교정 → IWR 가중치 재학습 → 평가 → 반복
```

### 5.2 개입 인터페이스

| 동작 | VR 입력 |
|------|---------|
| 개입 시작 | 양손 그립 동시 누름 |
| 자율 복귀 | 양손 트리거 동시 누름 |
| 교정 시연 | VR 텔레오퍼레이션 (통상) |

### 5.3 HG-DAgger 루프 실행

```bash
cd ust_ws/ust_260220

# VR 교정 모드 (실제 실행)
isaaclab -p scripts/run_hg_dagger.py \
    --checkpoint /workspace/isaaclab/ust_ws/models/bc_rnn/model_best.pth \
    --enable_vr \
    --max_iterations 5 \
    --episodes_per_iteration 20 \
    --intervention_weight 5.0 \
    --model_dir /workspace/isaaclab/ust_ws/models/hg_dagger/

# 시뮬레이션 모드 (테스트용)
python scripts/run_hg_dagger.py \
    --checkpoint /workspace/isaaclab/ust_ws/models/bc_rnn/model_best.pth \
    --max_iterations 3 \
    --episodes_per_iteration 10
```

### 5.4 IWR 설정 (RTX PRO 6000)

| 항목 | 값 | 비고 |
|------|-----|------|
| 자율 가중치 | 1.0 | 기본 |
| 개입 가중치 | 5.0 | 병목 상태 강조 |
| 배치 크기 | 64 | 오프라인 재학습 |
| 온라인 배치 | 16 × 4 accumulation | 유효 배치 = 64 |
| LR | 5e-5 | 기본 LR의 1/2 |
| 혼합 정밀도 | BF16 | Blackwell 최적화 |
| LR 스케줄러 | Cosine | 안정적 수렴 |
| 시퀀스 길이 | 15 | LSTM 메모리 활용 |

### 5.5 교정 루프 프로토콜

```
Iteration 0: 기준선 평가 (50 에피소드)
  → 성공률 기록

Iteration 1~N:
  1. 정책 배포 → 자율 실행 (사람이 VR HMD로 관찰)
  2. 실패 감지 시 → 양손 그립 → 제어권 인계 → VR 교정 시연
  3. 양손 트리거 → 자율 복귀
  4. 20 에피소드 수집 (자율 + 개입 라벨)
  5. IWR 재학습 (BF16, 개입 가중치 5.0)
  6. 평가 (50 에피소드)
  7. 수렴 확인 (성공률 ≥ 80% 또는 개선 정체)
```

### 5.6 기대 시간

| 단계 | 예상 시간 |
|------|-----------|
| 데이터 수집 (20 에피소드) | ~20분 |
| IWR 재학습 (200 에폭) | ~3-5분 |
| 평가 (50 에피소드) | ~25분 |
| **이터레이션당 합계** | **~50분** |
| **전체 5회 반복** | **~4-5시간** |

---

## 6. Phase 3: 불확실성 기반 도움 요청

### 6.1 단계 요약

```
앙상블 학습 → SigLIP2/Florence-2 로드 → VLM 서버 시작 → Conformal 교정 → 통합 루프 실행
```

### 6.2 앙상블 정책 학습

```bash
cd ust_ws/ust_260220

# 5개 독립 모델 순차 학습 (RTX PRO 6000)
python scripts/train_ensemble.py \
    --dataset ./data/corrective/all_data.hdf5 \
    --output_dir /workspace/isaaclab/ust_ws/models/ensemble/ \
    --num_models 5 \
    --num_epochs 2000 \
    --batch_size 128 \
    --subsample_ratio 0.8
```

**앙상블 설정:**
| 항목 | 값 |
|------|-----|
| 모델 수 (K) | 5 |
| 서브샘플 | 80% 부트스트랩 |
| 학습 전략 | **순차** (안정성 우선) |
| 개별 모델 학습 시간 | ~30분 |
| **총 학습 시간** | **~2.5시간** |

### 6.3 Conformal Prediction 교정

```bash
cd ust_ws/ust_260220

# 교정 데이터 생성 + 임계값 설정
python scripts/calibrate_conformal.py \
    --output /workspace/isaaclab/ust_ws/models/conformal/calibration.npz \
    --alpha 0.05 \
    --num_samples 100 \
    --vlm_model Qwen/Qwen3-VL-8B-Instruct \
    --vlm_base_url http://localhost:8000/v1
```

### 6.4 불확실성 루프 실행

```bash
cd ust_ws/ust_260220

# 1) 모의 모드 (모델 없이 테스트)
python scripts/run_uncertainty_loop.py \
    --ensemble_dir /workspace/isaaclab/ust_ws/models/ensemble/ \
    --num_models 5

# 2) SigLIP2 + Florence-2만 (VLM 없이)
python scripts/run_uncertainty_loop.py \
    --ensemble_dir /workspace/isaaclab/ust_ws/models/ensemble/ \
    --num_models 5 \
    --enable_siglip --enable_florence

# 3) 로컬 VLM-8B (개발 모드, ~33GB)
isaaclab -p scripts/run_uncertainty_loop.py \
    --ensemble_dir /workspace/isaaclab/ust_ws/models/ensemble/ \
    --num_models 5 \
    --enable_siglip --enable_florence \
    --vlm_model Qwen/Qwen3-VL-8B-Instruct \
    --vlm_base_url http://localhost:8000/v1 \
    --with_sim_camera \
    --conformal_data /workspace/isaaclab/ust_ws/models/conformal/calibration.npz

# 4) 로컬 VLM-32B + VR (최종 운용, ~85GB)
isaaclab -p scripts/run_uncertainty_loop.py \
    --ensemble_dir /workspace/isaaclab/ust_ws/models/ensemble/ \
    --num_models 5 \
    --enable_siglip \
    --vlm_model Qwen/Qwen3-VL-32B-Instruct \
    --vlm_base_url http://localhost:8000/v1 \
    --enable_vr --with_sim_camera \
    --target_intervention_ratio 0.15
```

### 6.5 도움 요청 유형

| 유형 | 트리거 | VR 표시 | 사람 응답 |
|------|--------|---------|----------|
| **행동 교정** | 앙상블 분산 > τ | "동작 불확실. VR 교정 필요" | VR 텔레옵 시연 |
| **물체 식별** | VLM 신뢰도 < 0.7 | "이 물체가 뭔가요?" | 카테고리 선택 |
| **카테고리 선택** | Conformal 집합 > 1 | "어느 카테고리?" | 옵션 선택 |
| **복합** | 행동+물체 동시 불확실 | "행동도, 물체도 모름" | VR 교정 또는 텍스트 |

### 6.6 적응적 임계값

- 초기 임계값: 0.05
- 목표 개입률: 15%
- 실행 중 자동 조정:
  - 도움 요청 > 15% → 임계값 올림 (덜 요청)
  - 도움 요청 < 15% → 임계값 내림 (더 요청)

---

## 7. 전체 평가

### 7.1 평가 실행

```bash
cd ust_ws/ust_260220

# Phase 1 평가
isaaclab -p scripts/evaluate.py \
    --checkpoint /workspace/isaaclab/ust_ws/models/bc_rnn/model_best.pth \
    --phase 1 --num_episodes 50

# Phase 2 평가
isaaclab -p scripts/evaluate.py \
    --checkpoint /workspace/isaaclab/ust_ws/models/hg_dagger/model_best.pth \
    --phase 2 --num_episodes 50

# Phase 3 앙상블 평가
isaaclab -p scripts/evaluate.py \
    --ensemble_dir /workspace/isaaclab/ust_ws/models/ensemble/ \
    --phase 3 --num_models 5 --num_episodes 50

# 전체 비교
isaaclab -p scripts/evaluate.py --compare \
    --phase1_checkpoint /workspace/isaaclab/ust_ws/models/bc_rnn/model_best.pth \
    --phase2_checkpoint /workspace/isaaclab/ust_ws/models/hg_dagger/model_best.pth \
    --ensemble_dir /workspace/isaaclab/ust_ws/models/ensemble/ \
    --output ./results/comparison.json
```

### 7.2 평가 메트릭

| 메트릭 | Phase 1 | Phase 2 | Phase 3 |
|--------|---------|---------|---------|
| 성공률 | O | O | O |
| 평균 분류 물체 | O | O | O |
| 에피소드 길이 | O | O | O |
| 평균 보상 | O | O | O |
| 개입 비율 | - | O | O |
| 도움 요청률 | - | - | O |
| 불확실성 AUROC | - | - | O |
| VLM 인식 정확도 | - | - | O |
| Conformal 커버리지 | - | - | O |

---

## 8. 디렉토리 구조

```
ust_ws/ust_260220/
├── corrective/                          # 교정 학습 시스템
│   ├── __init__.py
│   ├── phase1/                          # Phase 1: 기초
│   │   ├── __init__.py
│   │   ├── bc_rnn_policy.py             # BC-RNN 정책 래퍼
│   │   ├── robomimic_config.py          # Robomimic 설정 (batch=128, 2000ep)
│   │   └── mimicgen_augmentor.py        # MimicGen 증강
│   ├── phase2/                          # Phase 2: HG-DAgger
│   │   ├── __init__.py
│   │   ├── intervention_manager.py      # 개입 상태 머신
│   │   ├── iwr_trainer.py              # IWR 가중치 학습 (BF16)
│   │   └── hg_dagger_loop.py           # 교정 루프
│   ├── phase3/                          # Phase 3: 불확실성 (3-Tier)
│   │   ├── __init__.py
│   │   ├── ensemble_policy.py           # 앙상블 불확실성
│   │   ├── vlm_analyzer.py             # 3-Tier: SigLIP2+Florence-2+VLM
│   │   ├── conformal_predictor.py      # Conformal prediction
│   │   ├── help_request_decider.py     # 도움 요청 결정 (SigLIP2 통합)
│   │   └── adaptive_threshold.py       # 적응적 임계값
│   └── utils/                           # 공용 유틸리티
│       ├── __init__.py
│       ├── corrective_hdf5.py           # 확장 HDF5 레코더
│       └── evaluation.py               # 정책 평가기
├── scripts/                             # 실행 스크립트
│   ├── run_teleop.py                    # VR 텔레오퍼레이션
│   ├── record_demos.py                  # 시연 수집
│   ├── train_bc_rnn.py                  # BC-RNN 학습 (BF16, batch=128)
│   ├── augment_demos.py                 # MimicGen 증강
│   ├── run_hg_dagger.py                 # HG-DAgger 루프
│   ├── train_ensemble.py                # 앙상블 학습 (BF16, batch=128)
│   ├── calibrate_conformal.py           # Conformal 교정
│   ├── run_uncertainty_loop.py          # 3-Tier 불확실성 루프
│   └── evaluate.py                      # 전체 평가
├── data/                                # 데이터
│   ├── demos/                           # Phase 1 시연
│   ├── corrective/                      # Phase 2 교정 데이터
│   └── uncertainty/                     # Phase 3 불확실성 데이터
├── EXECUTION_GUIDE.md                   # 환경 실행 가이드
└── CORRECTIVE_TEACHING_GUIDE.md         # 이 문서

ust_ws/models/                           # 사전학습 + 학습 모델 (공유)
├── siglip2-so400m/                      # Tier 1: 빠른 분류 (~1.5GB)
├── florence2-large/                     # Tier 2: 물체 탐지 (~3GB)
├── qwen3-vl-8b/                         # Tier 3: VLM 개발용 (~17GB)
├── qwen3-vl-32b/                        # Tier 3: VLM 운용용 (~65GB)
├── bc_rnn/                              # Phase 1 모델
├── hg_dagger/                           # Phase 2 모델
├── ensemble/                            # Phase 3 앙상블 (K=5)
└── conformal/                           # Conformal 교정 데이터
```

---

## 9. 문제 해결

### 9.1 Phase 1 문제

**Q: Robomimic이 설치되지 않으면?**
- `train_bc_rnn.py`가 자동으로 독립 학습 모드(SimpleBCRNN)로 전환합니다.
- BF16 혼합 정밀도는 독립 모드에서도 자동 적용됩니다.

**Q: MimicGen이 설치되지 않으면?**
- `augment_demos.py`가 자동으로 노이즈 주입 기반 단순 증강을 수행합니다.

### 9.2 Phase 2 문제

**Q: VR 연결이 안 되면?**
- `EXECUTION_GUIDE.md`의 VR 연결 가이드를 참조하세요.
- CloudXR Runtime 6.0.1, `IPC_IGNORE_VERSION=1`, `XR_RUNTIME_JSON` 설정 확인

**Q: 개입 트리거가 작동하지 않으면?**
- VR 컨트롤러 매핑을 확인하세요.
- `InterventionManager.force_start_intervention()` 으로 강제 개입 테스트 가능

### 9.3 Phase 3 문제

**Q: SigLIP2/Florence-2 로드 실패?**
- 모델이 `/workspace/isaaclab/ust_ws/models/` 에 다운로드되었는지 확인
- `transformers` 패키지 버전 확인 (`pip install -U transformers`)
- 모델 로드 실패 시 자동으로 모의 응답으로 대체됩니다.

**Q: VLM 서버(SGLang)에 연결 안 되면?**
- SGLang 서버가 별도 터미널에서 실행 중인지 확인
- `curl http://localhost:8000/v1/models` 로 서버 상태 확인
- VLM 서버 없이도 SigLIP2 + Conformal만으로 기본 불확실성 판단 가능
- `--vlm_provider mock` 옵션으로 모의 모드 사용 가능

**Q: 앙상블 분산이 너무 높으면?**
- 다중 모달 문제 (여러 올바른 행동 존재) 가능성
- `EnsemblePolicy(use_multimodal_aware=True)`로 클러스터링 기반 분산 사용

**Q: VRAM이 부족하면?**
- Phase 3 개발: Qwen3-VL-8B (19GB) 사용 → 총 ~33GB
- Phase 3 운용: Qwen3-VL-32B (75GB) 사용 → 총 ~85GB
- 동시 학습 필요 시 VLM 서버의 `--mem-fraction-static` 값 조절
- `nvidia-smi` 로 실시간 VRAM 모니터링

**Q: Conformal 예측 집합이 항상 전체면?**
- 교정 데이터 부족 가능. α를 0.1로 올리거나 더 많은 교정 데이터 수집
- SigLIP2 분류 확률이 너무 균등한 경우 카테고리 텍스트 프롬프트 개선

---

## 참고 논문

| Phase | 핵심 논문 | 용도 |
|-------|----------|------|
| Phase 1 | MimicGen (Mandlekar et al., 2023) | 데이터 증강 |
| Phase 1 | Robomimic (Mandlekar et al., 2021) | BC-RNN 학습 |
| Phase 2 | HG-DAgger (Kelly et al., 2019) | 인간-게이트 교정 |
| Phase 2 | IWR (Spencer et al.) | 개입 가중 회귀 |
| Phase 3 | EnsembleDAgger (Menda et al., 2019) | 앙상블 불확실성 |
| Phase 3 | KnowNo (Ren et al., CoRL 2023) | Conformal 도움 요청 |
| Phase 3 | ThriftyDAgger (Hoque et al., CoRL 2022) | 적응적 임계값 |

## RTX PRO 6000 최적화 참고

| 설정 | 값 | 근거 |
|------|-----|------|
| 배치 크기 | 128 (오프라인) | 96GB VRAM 충분 |
| 혼합 정밀도 | BF16 | Blackwell 5세대 Tensor 코어 |
| LR 스케줄러 | MultiStep (BC-RNN), Cosine (IWR) | 안정적 수렴 |
| 앙상블 학습 | 순차 (K=5) | 메모리 안정성 |
| VLM 서빙 | SGLang (FP16) | vLLM 대비 2.3배 빠름 |
| DataLoader workers | 8 | CPU 활용 극대화 |
