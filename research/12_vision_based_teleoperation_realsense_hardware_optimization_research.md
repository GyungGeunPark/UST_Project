# 12. 비전 기반 텔레오퍼레이션 & RealSense 하드웨어 최적 활용 연구

> **작성일**: 2026-02-26
> **프로젝트**: UST 비전 기반 텔레오퍼레이션 시스템 설계 (기존 카메라 활용 최적화)
> **보유 카메라**: Intel RealSense D455 ×1, Intel RealSense L515 ×2, Livox Mid-360 ×1
> **목표 로봇**: Unitree G1 29DOF + INSPIRE 5-Finger Hand / TurtleBot3 + Dual OpenMANIPULATOR-X
> **GPU**: NVIDIA RTX PRO 6000 (96GB VRAM)
> **관련 문서**: [연구 10] VR Hand Tracking, [연구 11] Full-Body MoCap Hardware

---

## 목차

1. [요약 (Executive Summary)](#1-요약-executive-summary)
2. [보유 장비 상세 스펙 분석](#2-보유-장비-상세-스펙-분석)
3. [비전 기반 텔레오퍼레이션 기술 종합 분석](#3-비전-기반-텔레오퍼레이션-기술-종합-분석)
4. [기존 카메라 활용 가능한 접근법](#4-기존-카메라-활용-가능한-접근법)
5. [포즈 추정 파이프라인 상세](#5-포즈-추정-파이프라인-상세)
6. [리타겟팅 기술 분석](#6-리타겟팅-기술-분석)
7. [추가 장비 투자 시나리오 분석](#7-추가-장비-투자-시나리오-분석)
8. [하이브리드 접근법 (비전 + IMU/VR 융합)](#8-하이브리드-접근법-비전--imuvr-융합)
9. [종합 비교표](#9-종합-비교표)
10. [최종 권장 전략 (단계별 로드맵)](#10-최종-권장-전략-단계별-로드맵)
11. [Isaac Lab 구현 가이드](#11-isaac-lab-구현-가이드)
12. [참고문헌](#12-참고문헌)

---

## 1. 요약 (Executive Summary)

### 1.1 문제 정의

연구 11에서 전신 텔레오퍼레이션을 위한 MoCap 하드웨어를 분석했으나, **현재 보유 중인 비전 센서(D455, L515 ×2, Mid-360)를 텔레오퍼레이션에 직접 활용하는 방법**은 다루지 않았다. 본 연구는 다음 질문에 답한다:

> **"RealSense D455 1대 + L515 2대 + Mid-360 1대만으로 휴머노이드 텔레오퍼레이션이 가능한가? 가능하다면, 최소한의 추가 투자로 최대 효율을 내는 방법은?"**

### 1.2 핵심 발견

| 발견 | 상세 |
|------|------|
| **D455 단독으로 전신 텔레오퍼레이션 가능** | HumanPlus 방식: D455 RGB → WHAM(전신) + HaMeR(손) → 리타겟팅 → G1 (추가 비용 $0) |
| **D455 Depth로 정확도 향상** | 모노큘러 포즈 추정의 scale ambiguity 문제를 D455 stereo depth로 해결 |
| **L515 ×2로 멀티뷰 삼각측량** | 3시점 커버리지로 self-occlusion 문제 대폭 감소 |
| **Mid-360은 보행 오도메트리에 직접 활용** | HumanoidExo 논문에서 정확히 Mid-360을 사용 |
| **RTX PRO 6000으로 모든 파이프라인 실시간 가능** | WHAM + HaMeR + VLM 동시 실행 시에도 96GB 중 ~20GB만 사용 |

### 1.3 3단계 권장 전략

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        UST 비전 텔레오퍼레이션 전략                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ★ Tier 0 ($0) - 즉시 실행 가능                                             │
│  ├─ D455 RGB → HumanPlus (WHAM+HaMeR) → GMR Retargeting → G1              │
│  ├─ L515 ×2 추가 시 멀티뷰 3D 포즈 추정 (옥클루전 해결)                      │
│  ├─ 용도: 초기 프로토타이핑, 데이터 수집 (20-50 demos)                       │
│  └─ 정확도: ★★★☆☆ (단일 카메라) / ★★★★☆ (멀티 카메라)                     │
│                                                                             │
│  ★ Tier 0.5 (+$100-500) - 최고 가성비 ⭐ 추천                               │
│  ├─ Option A: SlimeVR DIY IMU 5-7개 ($100-200) → 비전+IMU 센서 융합         │
│  ├─ Option B: HOMIE 외골격 3D 프린팅 ($500) → 직접 관절 매핑 + 비전 보조     │
│  ├─ 기존 카메라: 환경 인식 + depth 보정 + VLM 입력                           │
│  └─ 정확도: ★★★★☆                                                         │
│                                                                             │
│  ★ Tier 2 (+$900) - 연구 11 결론과 일치                                     │
│  ├─ PICO 4 Ultra + Motion Tracker ×2 → SONIC/GR00T WBC                     │
│  ├─ 기존 카메라: 보조 관찰 + VLM + 데이터 기록                               │
│  └─ 정확도: ★★★★★                                                         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.4 기존 장비의 장기적 가치

텔레오퍼레이션 방식이 VR/외골격으로 전환되더라도, 기존 카메라는 다음 용도로 **영구적으로 활용**된다:

- **D455**: VLM(Qwen3-VL) 입력, 물체 인식, 데이터 기록용 관찰 카메라
- **L515 ×2**: 정밀 물체 6DOF 포즈 추정, 근거리 grasping 보조
- **Mid-360**: 실내 SLAM, 네비게이션, 장애물 회피 (Phase 3 노인 지원에 필수)

---

## 2. 보유 장비 상세 스펙 분석

### 2.1 Intel RealSense D455 (×1)

| 항목 | 스펙 |
|------|------|
| **타입** | Active IR Stereo Depth Camera |
| **Baseline** | 95mm (D400 시리즈 최장 → 원거리 depth 정확도 최고) |
| **Depth 해상도** | 1280×720 @ 90fps / 848×480 @ 90fps |
| **RGB 해상도** | 1280×800 @ 30fps (IMX556) |
| **Depth 범위** | 0.4m ~ 6m (추천 사용 범위) |
| **Depth 오차** | <2% @ 4m 거리 |
| **FOV (Depth)** | 87° × 58° |
| **FOV (RGB)** | 90° × 65° |
| **IMU** | BMI055 6축 (가속도계 + 자이로) |
| **인터페이스** | USB 3.1 Type-C |
| **SDK** | Intel RealSense SDK 2.0 (`pyrealsense2`) |

**텔레오퍼레이션 적합도 분석**:
- RGB 해상도(1280×800)는 MediaPipe/WHAM 포즈 추정에 충분
- 95mm baseline으로 2-4m 거리에서 전신 depth 측정 가능
- IMU 내장으로 카메라 자세 추정 가능 (시간 동기화)
- **Isaac Lab 기존 통합**: `ust_config/ust_scene_cfg.py`의 `USTSceneWithSensorsCfg.camera_d455`에 이미 설정됨

### 2.2 Intel RealSense L515 (×2)

| 항목 | 스펙 |
|------|------|
| **타입** | Solid-State LiDAR Depth Camera |
| **Depth 기술** | MEMS LiDAR (ToF, 860nm) |
| **Depth 해상도** | 1024×768 @ 30fps |
| **RGB 해상도** | 1920×1080 @ 30fps |
| **Depth 범위** | 0.25m ~ 9m |
| **Depth 정확도** | ±5mm @ 1m (스테레오 대비 매우 정밀) |
| **Depth 포인트** | 23M points/sec |
| **FOV (Depth)** | 70° × 55° |
| **FOV (RGB)** | 70° × 43° |
| **IMU** | BMI085 6축 |
| **전력** | <3.5W |

**텔레오퍼레이션 적합도 분석**:
- **근거리(0.25-2m) 최고 정밀도**: LiDAR 기반이므로 IR 패턴 간섭 없음
- 2대 보유 → **멀티뷰 3D 재구성** 또는 **양손 각각 정밀 추적** 가능
- RGB 1080p → 고해상도 포즈 추정 가능
- **주의**: Intel RealSense 사업부 축소로 단종됨. 드라이버 지원은 계속되나 추가 구매 불가.

### 2.3 Livox Mid-360 (×1)

| 항목 | 스펙 |
|------|------|
| **타입** | 360° 3D LiDAR (Non-repetitive scanning) |
| **FOV** | 360° × (-7° ~ +52°) |
| **포인트 레이트** | 200,000 points/sec |
| **범위** | 40m (10% 반사율) |
| **정확도** | ±2cm |
| **무게** | 265g |
| **인터페이스** | 100BASE-TX Ethernet |
| **소비전력** | 9W (typical) |

**텔레오퍼레이션 적합도 분석**:
- **HumanoidExo 논문에서 정확히 Mid-360을 사용** → 보행 오도메트리 + SLAM
- FAST-LIO2 알고리즘과 호환 → 실시간 6DOF 포즈 추정
- 실외/실내 모두 동작 (비전 카메라의 조명 의존성 보완)
- **Isaac Lab 기존 통합**: `ust_config/ust_scene_cfg.py`의 `USTSceneWithSensorsCfg.lidar_mid360`에 RayCasterCfg로 설정됨

### 2.4 센서 융합 가능성 매트릭스

| 조합 | 용도 | 장점 | 한계 |
|------|------|------|------|
| **D455 단독** | 전신 포즈 추정 (RGB+Depth) | 추가 비용 $0, 즉시 가능 | 단일 시점 옥클루전 |
| **D455 + L515 ×2** | 멀티뷰 3D 포즈 (3대 카메라) | 옥클루전 대폭 감소, depth 교차 검증 | 캘리브레이션 필요 |
| **D455 + Mid-360** | 조작(비전) + 이동(LiDAR) | 실내외 겸용, HumanoidExo 패턴 | 별도 파이프라인 |
| **L515 ×2 (양손)** | 정밀 양손 추적 (탁상 조작) | LiDAR depth → mm급 정확도 | 전신 추적 불가 |
| **전체 융합** | 360° 커버리지 + 전신 + 이동 | 최대 데이터, 최대 정확도 | 복잡한 동기화, 높은 대역폭 |

---

## 3. 비전 기반 텔레오퍼레이션 기술 종합 분석

### 3.1 주요 프레임워크 종합 비교표

| 프레임워크 | 기관 | 연도 | 입력 | 로봇 | G1 지원 | D455 활용 | L515 활용 | Mid-360 활용 | 추가 비용 |
|-----------|------|------|------|------|---------|----------|----------|------------|---------|
| **HumanPlus** | Stanford | 2024 | RGB 카메라 1대 | H1 | 가능(리타겟팅) | ✅ RGB | ✅ RGB | ❌ | $0 |
| **HumanoidExo** | NUDT/Midea | 2025 | 외골격+LiDAR+카메라 | G1 | ✅ 검증됨 | ✅ 카메라 | ❌ | ✅ 오도메트리 | $500+ |
| **AnyTeleop** | NVIDIA | 2023 | RGB/RGB-D | 다수 | 간접 | ✅ 최적 | ✅ 가능 | ❌ | $0 |
| **ACE** | 2024 | 외골격+카메라 | 다수 | ✅ 가능 | ✅ 보조 | ✅ 보조 | ❌ | $300-500 |
| **GELLO** | 2024 | 물리 컨트롤러 | Franka/UR/xArm | 부분적 | 보조만 | 보조만 | ❌ | $300 |
| **UMI** | Stanford | 2024 | GoPro+T265 | 다수 | 간접 | △ 대체 | ❌ | ❌ | $200+ |
| **CLONE** | 2024 | Vision Pro+LiDAR | H1 | 가능 | 보조 | 보조 | ✅ 오도메트리 | $3,500 |
| **Open-TeleVision** | MIT/UCSD | 2024 | Vision Pro+ZED | H1/GR1 | 가능 | △ | ❌ | ❌ | $3,500+ |
| **Bunny-VisionPro** | 2024 | Vision Pro | xArm7 | 간접 | 보조 | ❌ | ❌ | $3,500 |
| **SONIC/GR00T WBC** | NVIDIA | 2025 | PICO+Tracker | G1 | ✅ 100% | 보조 | 보조 | 보조 | $900-5,900 |

### 3.2 HumanPlus (Stanford, CoRL 2024) — D455 직접 활용 가능 ⭐

**개요**: 단일 RGB 카메라 하나로 인간의 전신 동작을 실시간 추정하여 휴머노이드 로봇에 전달하는 시스템.

**아키텍처**:
```
┌──────────┐    ┌─────────────────┐    ┌──────────────────┐    ┌───────────┐
│ D455 RGB │───▶│ WHAM (전신 3D)  │───▶│ Retargeting      │───▶│ G1 Robot  │
│ 카메라   │    │ 25Hz, SMPL-X    │    │ SMPL → G1 joints │    │ 29 DOF    │
└──────────┘    └─────────────────┘    └──────────────────┘    └───────────┘
                ┌─────────────────┐    ┌──────────────────┐
                │ HaMeR (손 3D)   │───▶│ MANO → INSPIRE   │
                │ 30Hz, MANO      │    │ hand retargeting  │
                └─────────────────┘    └──────────────────┘
```

**핵심 구성요소**:

| 구성요소 | 역할 | 성능 |
|---------|------|------|
| **WHAM** (World-grounded Humans with Accurate Motion) | 단일 RGB → SMPL-X 전신 3D 포즈 + 글로벌 좌표계 | 25Hz @ RTX 4090 |
| **HaMeR** (Hand Mesh Recovery) | 단일 RGB → MANO 손 메시 (21 joints/hand) | 30Hz @ RTX 4090 |
| **Retargeting** | SMPL-X → 로봇 joint angles (Euler angle copy) | 실시간 |
| **Behavior Cloning** | 수집된 데모 → BC policy → 자율 동작 | 40-100개 데모로 60-100% 성공률 |

**D455 활용 시 보너스**:
- WHAM은 RGB만으로 3D 포즈를 추정하지만 **scale ambiguity** 문제가 있음
- D455의 **stereo depth**로 절대 거리를 보정 가능 → 정확도 향상
- D455 내장 IMU로 카메라 자세 추정 → WHAM의 글로벌 좌표계 안정화

**검증된 태스크** (H1 로봇):
- 신발 신기, 창고 물건 정리, 옷 접기, 걷기
- 데모 40-100개 → 행동 복제 성공률 60-100%

**GitHub**: [MarkFzp/humanplus](https://github.com/MarkFzp/humanplus) (MIT License)
**논문**: "HumanPlus: Humanoid Shadowing and Imitation from Humans" (CoRL 2024)

### 3.3 HumanoidExo (NUDT/Midea, 2025) — Mid-360 직접 활용 가능 ⭐

**개요**: 인간과 동형(isomorphic)인 착용형 외골격으로 동작 데이터를 수집하여 휴머노이드를 학습시키는 시스템. **Mid-360 LiDAR를 외골격 등에 장착하여 오도메트리 제공**.

**아키텍처**:
```
┌──────────────────┐
│ Isomorphic       │    ┌──────────────────┐    ┌───────────┐
│ Exoskeleton      │───▶│ Joint Mapping    │───▶│ G1 Robot  │
│ (7 arm joints)   │    │ (isomorphic 1:1) │    │ 29 DOF    │
└──────────────────┘    └──────────────────┘    └───────────┘
┌──────────────────┐    ┌──────────────────┐
│ Mid-360 LiDAR    │───▶│ FAST-LIO SLAM    │───▶ 보행 오도메트리
│ (등에 장착)       │    │ 6DOF odometry    │
└──────────────────┘    └──────────────────┘
```

**핵심 특징**:
- 외골격 관절축이 인간 관절축과 정확히 정렬 (isomorphic design)
- **Mid-360 LiDAR**로 조명/텍스처 무관한 오도메트리 (비전 대비 우월)
- 5개 텔레오퍼레이션 데모 + 195개 외골격 세션 → **80% pick-and-place 성공**
- 로봇 보행 데이터 없이 외골격 보행 데이터만으로 보행 학습 성공

**기존 장비 활용**:
- **Mid-360**: 논문과 동일 용도 (외골격/로봇 등에 장착 → SLAM 오도메트리)
- **D455**: 논문의 wrist camera 대체 (조작 시 물체 인식)

**추가 필요 장비**: 외골격 ($500 HOMIE로 대체 가능, 아래 3.5 참조)

**GitHub**: [humanoid-exo.github.io](https://humanoid-exo.github.io/)
**논문**: "HumanoidExo: Wearable Isomorphic Exoskeleton for Scalable Humanoid Data Collection" (2025)

### 3.4 AnyTeleop (NVIDIA, 2023) — D455 최적 매칭 ⭐

**개요**: 단일 통합 시스템으로 다양한 로봇(Franka, xArm, dexterous hands)을 텔레조작. RGB/RGB-D 카메라 유연하게 지원.

**핵심 특징**:
- **D455가 공식 지원 카메라**: RGB-D stereo로 wrist pose 정확도 향상
- 손 키포인트 → 로봇 손가락 리타겟팅, 손목 위치 → 로봇 팔 제어
- CUDA 가속 motion planner → 실시간 궤적 생성
- IsaacGym/SAPIEN 시뮬레이터 지원
- **NVIDIA 공식이므로 Isaac Lab 통합 경로 명확**

**한계**: 전신 텔레오퍼레이션이 아닌 **팔+손 (tabletop manipulation)** 전용

**GitHub**: [yzqin/anyteleop](https://yzqin.github.io/anyteleop/)

### 3.5 ACE (2024) — 3D 프린트 외골격 + 카메라 ⭐

**개요**: 3D 프린트 양팔 외골격에 hand-facing 카메라를 부착하여, 팔 위치는 외골격 FK(forward kinematics)로, 손 포즈는 카메라로 추적.

**핵심 특징**:
- 외골격 관절 위치 → FK → wrist 6DOF pose
- Hand-facing 웹캠 2대 → 손가락 포즈 추적 (~27Hz → 100Hz with fast camera)
- **크로스 플랫폼**: Ability Hand, Inspire Hand, H1, GR-1, B1+Z1, Franka

**기존 장비 활용**:
- D455 또는 L515를 hand-facing camera로 사용 가능 (depth bonus)
- 외골격은 3D 프린팅 필요 ($300-500)

**GitHub**: [ACETeleop/ACETeleop](https://github.com/ACETeleop/ACETeleop)

### 3.6 GELLO (2024) — 초저가 물리 컨트롤러

**개요**: $300 미만의 3D 프린트 운동학적(kinematic) 컨트롤러. 목표 로봇의 관절 구조를 복제한 물리 장치.

**핵심 특징**:
- 목표 로봇과 동일한 기구학 구조 → 직관적 1:1 매핑
- VR이나 스페이스마우스 대비 빠른 완료 시간 + 높은 성공률
- **비전 불필요** (물리적 관절 인코더 사용)
- 지원 로봇: Franka, UR5, xArm (G1 팔에 맞는 커스텀 필요)

**한계**: 팔 전용, 전신 텔레오퍼레이션 불가

**GitHub**: [wuphilipp/gello_site](https://wuphilipp.github.io/gello_site/)

### 3.7 UMI (Universal Manipulation Interface, Stanford 2024)

**개요**: 핸드헬드 그리퍼에 GoPro + Intel T265를 장착하여 "야생(in-the-wild)" 데이터 수집.

**핵심 특징**:
- 로봇 없이도 데모 데이터 수집 가능 (하드웨어 무관)
- GoPro 155° 피시아이 → 넓은 시야각
- T265 → 엔드이펙터 6DOF 추적

**기존 장비 활용**: D455를 T265 대체 가능하나, D455가 더 크고 무거워 핸드헬드에 부적합

### 3.8 CLONE (2024) — Vision Pro + LiDAR

**개요**: Apple Vision Pro의 hand tracking + 로봇 탑재 LiDAR 오도메트리로 전신 제어.

**핵심 특징**:
- 최소 입력(양 손목 6DOF + 머리 3DOF)만으로 전신 제어
- LiDAR closed-loop → 5.1cm 평균 위치 오차 (8.9m 거리)
- MoE(Mixture of Experts) 기반 장기 태스크 실행

**기존 장비 활용**: Mid-360을 로봇 탑재 LiDAR로 활용 가능

### 3.9 SONIC/GR00T WBC (NVIDIA, 2025) — 비디오 모드 가능

연구 11에서 상세 분석됨. 핵심 추가 사항:

- SONIC은 **비디오 모드**를 지원 → VR 없이 RGB 카메라만으로 포즈 추정
- D455 RGB를 비디오 입력으로 사용 가능 (추가 비용 $0)
- 정확도는 VR 모드(PICO + Tracker) 대비 낮지만, 초기 프로토타이핑에 충분
- **G1에서 100% 성공률** (50개 다양한 동작, VR 모드 기준)

---

## 4. 기존 카메라 활용 가능한 접근법

### 4.1 접근법 A: D455 단독 → HumanPlus 방식 (추가 비용 $0) ⭐ 최저비용

**아키텍처**:
```
┌─────────────────────────────────────────────────────────────┐
│                     접근법 A 파이프라인                       │
│                                                              │
│  D455 USB ──▶ pyrealsense2 ──▶ ┬─ RGB Frame ──────────┐     │
│                                │                       ▼     │
│                                │              WHAM (25Hz)    │
│                                │              ├─ 전신 SMPL-X │
│                                │              └─ 글로벌 좌표  │
│                                │                       │     │
│                                │              HaMeR (30Hz)   │
│                                │              └─ 양손 MANO   │
│                                │                       │     │
│                                └─ Depth Frame ──▶ Scale 보정 │
│                                                        │     │
│                                                   GMR Retarg │
│                                                   (CPU, G1)  │
│                                                        │     │
│                                                   G1 Joints  │
│                                                   29 DOF     │
└─────────────────────────────────────────────────────────────┘
```

**구현 단계**:

1. **D455 스트림 설정**:
```python
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)

# Depth-RGB alignment
align = rs.align(rs.stream.color)
```

2. **WHAM으로 전신 3D 포즈 추정**:
```python
# WHAM: World-grounded Humans with Accurate Motion
# Input: RGB video → Output: SMPL-X body params + global trajectory
from wham import WHAM
model = WHAM(checkpoint='wham_vit_w_3dpw.pth')
smpl_output = model.predict(rgb_frame)  # 25Hz
```

3. **HaMeR로 손 포즈 추정**:
```python
# HaMeR: Hand Mesh Recovery
from hamer import HaMeR
hand_model = HaMeR(checkpoint='hamer_demo.pth')
mano_params = hand_model.predict(rgb_frame)  # per-frame, 21 joints/hand
```

4. **D455 Depth로 scale 보정**:
```python
# WHAM의 scale ambiguity를 depth로 보정
aligned_depth = align.process(frames).get_depth_frame()
body_center_pixel = smpl_output.body_center_2d  # 골반 중심 좌표
real_depth = aligned_depth.get_distance(int(body_center_pixel[0]), int(body_center_pixel[1]))
scale_factor = real_depth / smpl_output.predicted_depth
smpl_output.global_translation *= scale_factor
```

5. **GMR로 G1 리타겟팅**:
```python
# GMR: General Motion Retargeting (ICRA 2026)
from gmr import Retargeter
retargeter = Retargeter(source='smpl', target='unitree_g1')
g1_joint_angles = retargeter.retarget(smpl_output)  # CPU 실시간
```

**예상 성능**:
| 항목 | 값 |
|------|-----|
| 전신 추적 주파수 | 20-25 Hz |
| 손 추적 주파수 | 25-30 Hz |
| 총 지연 | 40-60 ms |
| 전신 정확도 | ±5-10° (어깨/팔꿈치), ±10-15° (엉덩이/무릎) |
| 손 정확도 | ±10-20° (손가락 관절) |
| GPU 사용량 | ~6 GB (WHAM 4GB + HaMeR 2GB) |
| **정확도 등급** | ★★★☆☆ |

**장점**: 추가 비용 $0, 즉시 구현 가능, GPU 부하 낮음
**한계**: 단일 시점 → self-occlusion, 빠른 동작 시 모션 블러, depth 보정에도 한계

### 4.2 접근법 B: D455 + L515 ×2 멀티뷰 3D 포즈 추정 (추가 비용 $0) ⭐⭐ 최고 정확도(기존 장비)

**카메라 배치**:
```
                    ┌──────────┐
                    │ L515 #1  │ (정면 좌측 45°, 1.5m 거리, 1.2m 높이)
                    └────┬─────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
   ┌────┴─────┐    ┌────┴─────┐    ┌────┴─────┐
   │ L515 #1  │    │  오퍼레  │    │ L515 #2  │
   │ 좌측 45° │    │  이터    │    │ 우측 45° │
   └──────────┘    └────┬─────┘    └──────────┘
                        │
                   ┌────┴─────┐
                   │  D455    │ (정면, 2m 거리, 1.5m 높이)
                   │  정면    │
                   └──────────┘
```

**3시점 삼각측량 파이프라인**:
```
┌─────────────────────────────────────────────────────────────────────┐
│                    접근법 B 멀티뷰 파이프라인                         │
│                                                                      │
│  D455 RGB ──▶ MediaPipe 2D (33 keypoints) ──┐                       │
│  L515#1 RGB ─▶ MediaPipe 2D (33 keypoints) ──┼──▶ DLT 삼각측량     │
│  L515#2 RGB ─▶ MediaPipe 2D (33 keypoints) ──┘    ──▶ 3D Skeleton  │
│                                                          │          │
│  D455 Depth ──▶ per-keypoint depth ──┐                   │          │
│  L515#1 Depth ▶ per-keypoint depth ──┼──▶ Depth 교차검증 │          │
│  L515#2 Depth ▶ per-keypoint depth ──┘        │          │          │
│                                               ▼          ▼          │
│                                         Fused 3D Skeleton           │
│                                               │                     │
│                                        GMR Retargeting → G1        │
└─────────────────────────────────────────────────────────────────────┘
```

**캘리브레이션 절차**:

1. **Intrinsic 캘리브레이션**: 각 카메라의 RGB/Depth intrinsic (공장 캘리브레이션 사용 가능)
2. **Extrinsic 캘리브레이션**: Caliscope 또는 NCams 사용
   - CharuCo 보드 (체커보드 + ArUco 마커) 권장
   - 3대 카메라가 동시에 보드를 볼 수 있는 위치에서 촬영
   - Bundle adjustment로 6DOF 외부 파라미터 최적화
3. **시간 동기화**: RealSense HW sync cable (D455 ↔ L515) 또는 소프트웨어 타임스탬프 매칭

```python
# Caliscope 멀티카메라 캘리브레이션 예시
from caliscope import CameraArray, Calibrator

cameras = CameraArray([
    Camera("D455", intrinsic_d455, stream_d455),
    Camera("L515_L", intrinsic_l515_l, stream_l515_l),
    Camera("L515_R", intrinsic_l515_r, stream_l515_r),
])
calibrator = Calibrator(cameras, board_type='charuco')
extrinsics = calibrator.calibrate()  # 6DOF transforms
```

**예상 성능**:
| 항목 | 값 |
|------|-----|
| 전신 추적 주파수 | 15-20 Hz (3× MediaPipe) |
| 3D 재구성 정확도 | ±2-5° (삼각측량 기반) |
| 옥클루전 내성 | 높음 (3시점 → 최소 2시점 커버) |
| GPU 사용량 | ~3 GB (MediaPipe ×3, 경량) |
| **정확도 등급** | ★★★★☆ |

**장점**: 기존 장비만으로 최고 정확도, 옥클루전 강건성, depth 교차 검증
**한계**: 캘리브레이션 필요, 설치 공간 필요, 처리 속도 약간 하락

### 4.3 접근법 C: D455 + Mid-360 → HumanoidExo 변형 (추가 비용 $0~$500)

**아키텍처**:
```
┌────────────────────────────────────────────────────────────────┐
│                    접근법 C 하이브리드 파이프라인                 │
│                                                                 │
│  [오퍼레이터 측]                      [로봇 측]                  │
│                                                                 │
│  D455 RGB ──▶ WHAM/MediaPipe         Mid-360 ──▶ FAST-LIO      │
│  ──▶ 상체 포즈 (팔+손+상체)          ──▶ 6DOF 오도메트리        │
│           │                                  │                  │
│           ▼                                  ▼                  │
│   상체 리타겟팅 ─────────────────▶ 전신 제어 명령               │
│   (팔+손+상체)                     (상체 포즈 + 보행 속도)       │
│                                         │                       │
│  (선택) HOMIE 외골격                     ▼                      │
│  ──▶ 직접 팔 매핑 (더 정확)        G1 Robot                     │
│                                    29 DOF                       │
└────────────────────────────────────────────────────────────────┘
```

**두 가지 변형**:

| 변형 | 추가 비용 | 상체 입력 | 이동 입력 | 정확도 |
|------|---------|---------|---------|-------|
| **C-1**: D455 비전 + Mid-360 SLAM | $0 | D455 RGB 포즈 추정 | Mid-360 SLAM | ★★★☆☆ |
| **C-2**: HOMIE 외골격 + Mid-360 SLAM | $500 | 외골격 직접 매핑 | Mid-360 SLAM | ★★★★☆ |

**Mid-360 SLAM 설정** (FAST-LIO2):
```bash
# FAST-LIO2 with Mid-360
# 로봇 등에 Mid-360 장착 → 실시간 6DOF pose estimation
roslaunch fast_lio mapping_mid360.launch
# Output: /Odometry (nav_msgs/Odometry) → 로봇 기저 속도 피드백
```

### 4.4 접근법 D: L515 ×2 → 양손 정밀 추적 (탁상 조작 전용)

**아키텍처**:
```
┌─────────────────────────────────────────────────────────────┐
│                 접근법 D 양손 정밀 추적                       │
│                                                              │
│  L515 #1 (오른손 facing)    L515 #2 (왼손 facing)            │
│  ┌──────────────┐           ┌──────────────┐                 │
│  │ 0.3m 거리    │           │ 0.3m 거리    │                 │
│  │ RGB+LiDAR    │           │ RGB+LiDAR    │                 │
│  └──────┬───────┘           └──────┬───────┘                 │
│         │                          │                         │
│         ▼                          ▼                         │
│  Hand Keypoint (21pts)      Hand Keypoint (21pts)            │
│  + LiDAR Depth              + LiDAR Depth                    │
│  = 3D Hand Mesh             = 3D Hand Mesh                   │
│         │                          │                         │
│         ▼                          ▼                         │
│  Right INSPIRE              Left INSPIRE                     │
│  Hand Retarg                Hand Retarg                      │
│  (12 DOF)                   (12 DOF)                         │
└─────────────────────────────────────────────────────────────┘
```

**용도**: G1 INSPIRE 5-finger hand 양손 정밀 조작 (부엌 정리, 물체 분류)
**장점**: L515 LiDAR depth → **mm급 정확도** (0.25-0.5m 근거리에서)
**한계**: 전신 추적 불가, 탁상 시나리오 전용, 팔 위치 추적 별도 필요

### 4.5 접근법 비교 요약

| 접근법 | 카메라 | 추가 비용 | 전신 | 손 | 보행 | 정확도 | 지연 | 구현 난이도 |
|--------|-------|---------|------|-----|------|-------|------|-----------|
| **A**: D455 단독 HumanPlus | D455 | $0 | ✅ | ✅ | △ | ★★★☆☆ | 40-60ms | 중 |
| **B**: 멀티뷰 삼각측량 | D455+L515×2 | $0 | ✅ | ✅ | ❌ | ★★★★☆ | 50-70ms | 상 |
| **C-1**: 비전+SLAM | D455+Mid360 | $0 | ✅ | ✅ | ✅ | ★★★☆☆ | 40-60ms | 중 |
| **C-2**: 외골격+SLAM | Mid360+외골격 | $500 | ✅ | △ | ✅ | ★★★★☆ | 20-30ms | 중상 |
| **D**: 양손 정밀 | L515×2 | $0 | ❌ | ✅✅ | ❌ | ★★★★★ | 30-40ms | 중하 |

---

## 5. 포즈 추정 파이프라인 상세

### 5.1 모노큘러 RGB 기반 전신 포즈 추정

#### 5.1.1 WHAM (World-grounded Humans with Accurate Motion)

| 항목 | 상세 |
|------|------|
| **입력** | 단일 RGB 비디오 |
| **출력** | SMPL-X body params + 글로벌 궤적 (6DOF) |
| **특징** | SLAM 통합 → 절대 좌표계, 시간적 일관성 |
| **속도** | 25 Hz @ RTX 4090 → **30+ Hz @ RTX PRO 6000** |
| **VRAM** | ~4 GB |
| **GitHub** | [yohanshin/WHAM](https://github.com/yohanshin/WHAM) |
| **논문** | CVPR 2024 |

**WHAM이 텔레오퍼레이션에 적합한 이유**:
- 대부분의 HMR(Human Mesh Recovery) 모델은 **카메라 상대 좌표**만 출력 → 로봇에 매핑 어려움
- WHAM은 **글로벌 좌표계**에서의 인간 궤적 출력 → 로봇 보행 제어에 직접 활용
- SLAM 기반이므로 카메라가 움직여도 안정적

#### 5.1.2 HaMeR (Hand Mesh Recovery)

| 항목 | 상세 |
|------|------|
| **입력** | 단일 RGB 이미지 (hand crop) |
| **출력** | MANO hand mesh (21 joints/hand) |
| **속도** | 30 Hz @ RTX 4090 → **40+ Hz @ RTX PRO 6000** |
| **VRAM** | ~2 GB |
| **GitHub** | [geopavlakos/hamer](https://github.com/geopavlakos/hamer) |
| **논문** | CVPR 2024 |

**HaMeR의 한계**:
- Per-frame 추정 → 시간적 떨림(jitter) 발생 가능
- 해결: temporal smoothing (1-Euro filter 또는 Kalman filter)

#### 5.1.3 4DHumans / HMR2.0

| 항목 | 상세 |
|------|------|
| **입력** | 단일 RGB 비디오 |
| **출력** | SMPL body params (시간적 일관성 향상) |
| **GitHub** | [shubham-goel/4D-Humans](https://github.com/shubham-goel/4D-Humans) |
| **비고** | WHAM의 기반 모델, WHAM이 이를 발전시킴 |

### 5.2 MediaPipe + Depth 융합 파이프라인

**Google MediaPipe**는 가장 빠르고 접근성 높은 포즈 추정 솔루션:

| 모듈 | 키포인트 수 | 용도 |
|------|-----------|------|
| **Pose** | 33 body landmarks | 전신 골격 |
| **Hands** | 21 joints/hand | 손가락 추적 |
| **Holistic** | 33+21×2+468 | 전신+양손+얼굴 (통합) |

**D455 통합 파이프라인** (검증된 방법):

```python
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np

# 1. RealSense D455 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)

# Depth → RGB 정렬
align = rs.align(rs.stream.color)

# D455 intrinsics 가져오기
depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

# 2. MediaPipe 초기화
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    model_complexity=2,       # 최고 정확도
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

# 3. 메인 루프
while True:
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)

    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()

    color_image = np.asanyarray(color_frame.get_data())

    # MediaPipe 2D 키포인트 추출
    results = pose.process(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))

    if results.pose_landmarks:
        landmarks_3d = []
        for lm in results.pose_landmarks.landmark:
            # 2D pixel 좌표
            px = int(lm.x * 1280)
            py = int(lm.y * 800)

            # D455 depth에서 실제 거리 획득
            depth = depth_frame.get_distance(px, py)

            # 2D + depth → 3D 카메라 좌표 변환
            point_3d = rs.rs2_deproject_pixel_to_point(
                depth_intrin, [px, py], depth
            )
            landmarks_3d.append(point_3d)

        # landmarks_3d: 33개 3D 좌표 → 리타겟팅 입력
```

**MediaPipe의 한계 (텔레오퍼레이션 관점)**:
- MediaPipe의 **z값(깊이 예측)은 사용하면 안 됨** → RGB로만 학습된 모델
- **반드시 D455 depth frame으로 3D 좌표 계산**해야 정확
- 빠른 동작 시 추적 손실 (복구에 200-500ms)
- Self-occlusion 시 관절 위치 추측값 (부정확)

**MediaPipe vs WHAM 비교**:

| 항목 | MediaPipe + Depth | WHAM |
|------|------------------|------|
| 속도 | 30+ Hz (경량) | 25 Hz (중량) |
| VRAM | <1 GB | ~4 GB |
| 3D 방식 | 2D keypoint + depth lookup | 직접 3D mesh 추정 |
| 글로벌 좌표 | 카메라 상대 | ✅ SLAM 기반 글로벌 |
| 시간 일관성 | 낮음 (프레임 독립) | 높음 (시퀀스 모델) |
| 옥클루전 내성 | 낮음 | 중간 |
| 전신 정확도 | ★★☆☆☆ | ★★★★☆ |
| **추천 용도** | 빠른 프로토타이핑 | 실제 텔레오퍼레이션 |

### 5.3 OpenPose + 멀티카메라 3D 삼각측량

**OpenPose**는 CMU에서 개발한 multi-person 포즈 추정 모델:

| 항목 | 상세 |
|------|------|
| **Body keypoints** | 25 (COCO format) |
| **Hand keypoints** | 21/hand |
| **Face keypoints** | 70 |
| **멀티카메라 3D** | DLT(Direct Linear Transform) 삼각측량 |

**RealSense 통합 프로젝트**:
- **RSPOP** (RealSense Plus OpenPose): D455 + OpenPose → 3D 포즈 → C3D 파일 출력
  - GitHub: [JuanMiguelGV/rspop](https://github.com/JuanMiguelGV/rspop)
- **RealTime3DPoseTracker**: D435i + OpenPose → 실시간 3D 추적 + 제스처 인식
  - GitHub: [bagridag/RealTime3DPoseTracker-OpenPose](https://github.com/bagridag/RealTime3DPoseTracker-OpenPose)

**멀티카메라 삼각측량 절차** (D455 + L515 ×2):

1. 각 카메라에서 독립적으로 2D keypoint 추출
2. Extrinsic calibration으로 카메라 간 변환 행렬 획득
3. DLT(Direct Linear Transform)로 최소 2시점 이상의 2D keypoint → 3D 좌표 삼각측량
4. RANSAC으로 outlier 제거
5. Temporal smoothing (Kalman filter)

### 5.4 Nuitrack SDK (상용 바디 트래킹)

| 항목 | 상세 |
|------|------|
| **지원 카메라** | D455 ✅, L515 ✅, D435/D415 ✅ |
| **바디 트래킹** | 19 관절, 최대 6명 동시 |
| **부가 기능** | 제스처 인식, 얼굴 인식 |
| **플랫폼** | Windows, Linux (Raspberry Pi 4도 가능) |
| **라이선스** | 커뮤니티 무료 (제한) / 상용 유료 |
| **GitHub** | [3DiVi/nuitrack-sdk](https://github.com/3DiVi/nuitrack-sdk) |
| **장점** | 빠른 프로토타이핑, 설치 용이 |
| **한계** | 손가락 추적 미지원, 19 관절만 (33보다 적음) |

**권장 용도**: 빠른 PoC(Proof of Concept) 구현 → 이후 WHAM/MediaPipe로 전환

### 5.5 Depth 보정 핵심 기법

RealSense depth 데이터를 포즈 추정에 활용할 때 알아야 할 핵심 사항:

**Depth 에지 노이즈 문제**:
- 인체 실루엣 경계에서 depth 값이 불안정 (stereo matching 한계)
- **해결**: keypoint 주변 5×5 window의 median depth 사용
```python
def get_robust_depth(depth_frame, px, py, window=5):
    """keypoint 주변 median depth (에지 노이즈 제거)"""
    half = window // 2
    depths = []
    for dx in range(-half, half+1):
        for dy in range(-half, half+1):
            d = depth_frame.get_distance(px+dx, py+dy)
            if d > 0:  # valid depth만
                depths.append(d)
    return np.median(depths) if depths else 0.0
```

**Temporal Smoothing** (1-Euro Filter):
```python
class OneEuroFilter:
    """적응형 저주파 필터: 느린 동작은 강하게, 빠른 동작은 약하게 필터링"""
    def __init__(self, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    def __call__(self, x, t):
        if self.t_prev is None:
            self.x_prev = x
            self.t_prev = t
            return x

        dt = t - self.t_prev
        dx = (x - self.x_prev) / dt

        # Derivative smoothing
        alpha_d = self._alpha(self.d_cutoff, dt)
        dx_hat = alpha_d * dx + (1 - alpha_d) * self.dx_prev

        # Adaptive cutoff
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        alpha = self._alpha(cutoff, dt)

        x_hat = alpha * x + (1 - alpha) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat

    def _alpha(self, cutoff, dt):
        tau = 1.0 / (2 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)
```

---

## 6. 리타겟팅 기술 분석

### 6.1 GMR: General Motion Retargeting (ICRA 2026) ⭐ 추천

| 항목 | 상세 |
|------|------|
| **기능** | 임의의 인간 동작 → 임의의 휴머노이드 로봇 joint angles |
| **입력** | SMPL/SMPL-X body params 또는 BVH 동작 데이터 |
| **출력** | 로봇별 joint angles |
| **G1 지원** | ✅ 공식 지원 (`unitree_g1` target) |
| **속도** | **CPU 실시간** (GPU 불필요) |
| **GitHub** | [YanjieZe/GMR](https://github.com/YanjieZe/GMR) |
| **논문** | ICRA 2026 |

**사용법**:
```bash
# GVHMR(비디오) → GMR(리타겟팅) → G1 joint angles
python scripts/gvhmr_to_robot.py \
    --gvhmr_pred_file <hmr4d_results.pt> \
    --robot unitree_g1 \
    --record_video
```

**핵심 장점**:
- CPU에서 실행 → GPU는 포즈 추정 전용으로 활용 가능
- Unitree G1을 공식 타겟으로 지원 → 별도 설정 불필요
- 범용적 (로봇 morphology가 달라도 동작)

### 6.2 Pose2Sim Pipeline

| 항목 | 상세 |
|------|------|
| **기능** | 2D keypoints → 3D 삼각측량 → OpenSim 생체역학 모델 |
| **입력** | MediaPipe, OpenPose, RTMPose 2D keypoints |
| **출력** | 3D joint angles (BVH/TRC format) |
| **GitHub** | [perfanalytics/pose2sim](https://github.com/perfanalytics/pose2sim) |
| **장점** | 멀티카메라 지원, 연구 수준 정확도, 오픈소스 |
| **한계** | OpenSim 기반 → 로봇 리타겟팅 추가 변환 필요 |

**D455 + L515 멀티카메라 연동 가능**: Pose2Sim이 멀티카메라 캘리브레이션 + 삼각측량을 통합 제공

### 6.3 기존 Isaac Lab 리타겟터 활용

UST 프로젝트의 Isaac Lab에는 이미 G1 리타겟팅 코드가 포함되어 있음:

| 파일 | 용도 | 입력 |
|------|------|------|
| `source/isaaclab/isaaclab/devices/openxr/retargeters/humanoid/unitree/inspire/g1_upper_body_retargeter.py` | G1 상체 (팔+허리) 리타겟팅 | SMPL → G1 joints |
| `source/isaaclab/isaaclab/devices/openxr/retargeters/humanoid/unitree/inspire/g1_dex_retargeting_utils.py` | INSPIRE hand 리타겟팅 | MANO → 12 DOF/hand |
| `source/isaaclab/isaaclab/devices/openxr/retargeters/humanoid/unitree/inspire/g1_lower_body_standing.py` | G1 하체 고정 포즈 | - |

**핵심 통찰**: 이 리타겟터들은 **OpenXR 입력을 기대**하지만, 내부적으로는 SMPL/MANO 포맷을 사용. 따라서 **카메라 기반 포즈 추정 출력(SMPL/MANO)을 동일한 포맷으로 변환**하면 그대로 사용 가능.

```python
# 카메라 기반 포즈 → 기존 Isaac Lab 리타겟터 활용 구조
camera_smpl_output = wham.predict(rgb_frame)   # SMPL-X body params
camera_mano_output = hamer.predict(rgb_frame)  # MANO hand params

# OpenXR 대신 카메라 출력을 리타겟터에 전달
g1_joints = g1_upper_body_retargeter.retarget(camera_smpl_output)
inspire_joints = g1_dex_retargeting.retarget(camera_mano_output)
```

### 6.4 Unitree xr_teleoperate

| 항목 | 상세 |
|------|------|
| **기능** | XR 디바이스 → G1 텔레오퍼레이션 |
| **입력** | Apple Vision Pro / PICO / 카메라 |
| **GitHub** | [unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) |
| **장점** | Unitree 공식 → G1 최적화 |
| **한계** | XR 디바이스 중심 설계, 순수 카메라 모드 제한적 |

### 6.5 리타겟팅 파이프라인 비교

| 항목 | GMR | Pose2Sim | Isaac Lab 내장 | Unitree 공식 |
|------|-----|---------|---------------|-------------|
| **실시간성** | ✅ CPU 실시간 | △ 오프라인 | ✅ 실시간 | ✅ 실시간 |
| **G1 지원** | ✅ 공식 | △ 변환 필요 | ✅ 기존 코드 | ✅ 공식 |
| **전신** | ✅ | ✅ | 상체만 | ✅ |
| **손가락** | ❌ | ❌ | ✅ INSPIRE | △ |
| **입력 형식** | SMPL/BVH | 2D keypoints | SMPL/MANO | XR/카메라 |
| **오픈소스** | ✅ MIT | ✅ | ✅ BSD-3 | ✅ |

**권장 조합**:
- **전신**: GMR (SMPL → G1 전신 joints)
- **손가락**: Isaac Lab 내장 `g1_dex_retargeting_utils.py` (MANO → INSPIRE)
- **보행**: WHAM 글로벌 궤적 → G1 보행 명령

---

## 7. 추가 장비 투자 시나리오 분석

### 7.1 Tier 0: 추가 투자 없음 ($0)

| 항목 | 상세 |
|------|------|
| **구성** | D455 RGB(+Depth) → WHAM/HaMeR → GMR → G1 |
| **또는** | D455 + L515 ×2 멀티뷰 삼각측량 |
| **정확도** | ★★★☆☆ (단일) / ★★★★☆ (멀티뷰) |
| **적합 용도** | 초기 프로토타이핑, 데이터 수집 보조, 기술 검증 |
| **한계** | 옥클루전, 빠른 동작 시 정확도 저하, 보행 제어 제한 |
| **구현 기간** | 1-2주 (WHAM+GMR 설치 및 통합) |

### 7.2 Tier 0.5: SlimeVR DIY IMU ($100-200) ⭐ 최고 가성비

| 항목 | 상세 |
|------|------|
| **구성** | D455 + SlimeVR IMU 5-7개 (가슴, 허리, 양 무릎, 양 발) |
| **원리** | 비전으로 글로벌 위치, IMU로 관절 각도 보정 |
| **하드웨어** | ESP32 + ICM-45686 IMU + QMC6309 자기계 (DIY) |
| **비용 상세** | ESP32 $5 × 7 + IMU $3 × 7 + 배터리/PCB = $100-200 |
| **정확도** | ★★★★☆ |
| **장점** | 옥클루전 완전 해결, 비전 드리프트 보정, DIY 가능 |
| **한계** | IMU 드리프트 (15-30분마다 재캘리브), 납땜 필요 |
| **GitHub** | [SlimeVR/SlimeVR-Tracker-ESP](https://github.com/SlimeVR/) |

**센서 융합 아키텍처**:
```
┌──────────────────────────────────────────────────────────┐
│              비전 + IMU 센서 융합                          │
│                                                           │
│  D455 RGB → WHAM → 글로벌 포즈 (위치 + 자세) ──┐         │
│                                                  ├─ EKF   │
│  SlimeVR IMU ×7 → 관절 각도 (로컬 회전) ────────┘  융합   │
│                                                     │     │
│  상보적(Complementary):                             ▼     │
│  - 비전: 글로벌 위치 정확 + 드리프트 없음         Fused    │
│  - IMU: 로컬 회전 정확 + 옥클루전 무관           3D Pose  │
│  - 비전이 IMU 드리프트 보정                         │     │
│  - IMU가 비전 옥클루전 보정                         ▼     │
│                                              GMR → G1     │
└──────────────────────────────────────────────────────────┘
```

### 7.3 Tier 1: GELLO 3D 프린트 컨트롤러 ($300)

| 항목 | 상세 |
|------|------|
| **구성** | GELLO 양팔 컨트롤러 + D455 (환경 인식) |
| **원리** | 물리적 관절 인코더 → 1:1 직접 매핑 |
| **정확도** | ★★★★☆ (팔만) |
| **적합 용도** | 탁상 양팔 조작 데이터 수집 |
| **한계** | 팔만 제어, 전신/보행 불가, 로봇별 커스텀 필요 |

### 7.4 Tier 1.5: HOMIE 외골격 ($500)

| 항목 | 상세 |
|------|------|
| **구성** | HOMIE 3D 프린트 외골격 + D455 + Mid-360 |
| **원리** | 외골격 → 상체 직접 매핑, Mid-360 → SLAM 보행, D455 → 보조 |
| **정확도** | ★★★★☆ |
| **적합 용도** | 전신 텔레오퍼레이션 (보행 + 양팔) |
| **장점** | HumanoidExo와 유사한 파이프라인, Mid-360 직접 활용 |
| **한계** | 3D 프린팅 시간, 외골격 착용 불편, 손가락 추적 별도 필요 |

### 7.5 Tier 2: PICO 4 Ultra + Motion Tracker ($900)

연구 11에서 **최소 권장** 구성:

| 항목 | 상세 |
|------|------|
| **구성** | PICO 4 Ultra ($500) + Motion Tracker ×2 ($200×2) |
| **원리** | SONIC/GR00T WBC native 지원 |
| **정확도** | ★★★★★ |
| **G1 검증** | ✅ NVIDIA 공식 100% 성공률 (50개 동작) |
| **기존 카메라 역할** | D455 → VLM 입력/관찰, L515 → 물체 추적, Mid-360 → 내비 |
| **구현 기간** | 2-4주 (SONIC 설치 + Isaac Lab 통합) |

### 7.6 Tier 3: PICO + MANUS ($3,900-5,900)

연구 11에서 **최적 권장** 구성:

| 항목 | 상세 |
|------|------|
| **구성** | Tier 2 + MANUS Prime 3 Haptic ($2,999) 또는 Metagloves Pro ($4,999) |
| **정확도** | ★★★★★+ |
| **추가 가치** | 손가락 mm급 정밀도, 햅틱 피드백 |
| **기존 카메라 역할** | Tier 2와 동일 |

### 7.7 가격-성능 분석

```
정확도
  5 ★ │                                          ● Tier 3 ($3,900+)
       │                                    ● Tier 2 ($900)
  4 ★ │              ● Tier 0.5     ● Tier 1.5
       │              ($100-200)     ($500)
  3 ★ │    ● Tier 0 (단일)   ● Tier 1
       │    ($0)              ($300)
  2 ★ │
       │
  1 ★ │
       └────────────┬────────┬────────┬────────┬────────┬──── 비용
                   $0      $200     $500    $1,000   $4,000

  ━━━ 가성비 최적 구간: Tier 0.5 ($100-200) ━━━
  ━━━ 품질 최적 구간: Tier 2 ($900) ━━━
```

**가격 대비 성능 향상률**:

| 업그레이드 경로 | 비용 증가 | 정확도 향상 | 가성비 |
|---------------|---------|-----------|-------|
| Tier 0 → 0.5 | +$100-200 | ★★★ → ★★★★ (+33%) | ⭐⭐⭐⭐⭐ 최고 |
| Tier 0 → 1.5 | +$500 | ★★★ → ★★★★ (+33%) | ⭐⭐⭐⭐ |
| Tier 0 → 2 | +$900 | ★★★ → ★★★★★ (+67%) | ⭐⭐⭐⭐ |
| Tier 2 → 3 | +$3,000 | ★★★★★ → ★★★★★+ (+10%) | ⭐⭐ |

---

## 8. 하이브리드 접근법 (비전 + IMU/VR 융합)

### 8.1 D455 + SlimeVR IMU 융합 (추천)

**원리**: Extended Kalman Filter(EKF)로 두 센서의 장단점을 상보적으로 융합

| 센서 | 장점 | 단점 |
|------|------|------|
| **D455 (비전)** | 절대 위치 정확, 드리프트 없음 | 옥클루전에 약함, 빠른 동작 블러 |
| **SlimeVR (IMU)** | 옥클루전 무관, 고주파(100Hz), 저지연 | 드리프트 누적, 글로벌 위치 불가 |

**융합 결과**: 비전의 글로벌 위치 + IMU의 로컬 회전 = 정확하고 안정적인 전신 포즈

**EKF 센서 융합 구조**:
```python
class VisionIMUFusion:
    """D455 비전 + SlimeVR IMU Extended Kalman Filter 융합"""

    def __init__(self, n_joints=33):
        # State: [position(3), orientation(3), joint_angles(n_joints*3)]
        self.state_dim = 6 + n_joints * 3
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 0.1  # covariance

        # Process noise (IMU 기반 예측)
        self.Q = np.eye(self.state_dim) * 0.01

        # Vision measurement noise
        self.R_vision = np.eye(n_joints * 3) * 0.05  # 비전은 노이즈 있지만 드리프트 없음

        # IMU measurement noise
        self.R_imu = np.eye(n_joints * 3) * 0.001  # IMU는 순간 정확하지만 드리프트 있음

    def predict(self, imu_data, dt):
        """IMU 데이터로 상태 예측 (100Hz)"""
        # IMU 가속도 → 위치 적분
        # IMU 각속도 → 자세 적분
        self.x = self._motion_model(self.x, imu_data, dt)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_vision(self, vision_3d_keypoints):
        """비전 데이터로 보정 (25-30Hz)"""
        # 비전 관측값으로 드리프트 보정
        innovation = vision_3d_keypoints - self._observation_model(self.x)
        S = self.H @ self.P @ self.H.T + self.R_vision
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(self.state_dim) - K @ self.H) @ self.P

    def get_fused_pose(self):
        """융합된 3D 포즈 반환"""
        return self.x
```

### 8.2 D455 + Quest 3S (기존 VR) 병행

현재 Quest 3S를 버리지 않고 기존 카메라와 함께 활용하는 방법:

```
┌──────────────────────────────────────────────────────┐
│           Quest 3S + D455 하이브리드                   │
│                                                       │
│  Quest 3S ──▶ 양손 추적 (26 joints/hand) ──┐         │
│              ──▶ 머리 6DOF pose           ├─ 상체    │
│                                            │         │
│  D455 RGB ──▶ 하체 포즈 추정 ──────────────┤         │
│              (WHAM lower body)             ├─ 하체   │
│                                            │         │
│  Mid-360 ──▶ SLAM 오도메트리 ──────────────┘ 이동    │
│                                                       │
│  결합: Quest(상체+손) + D455(하체) + Mid-360(이동)    │
└──────────────────────────────────────────────────────┘
```

**장점**: Quest 3S의 정밀 hand tracking(기존 인프라) + D455의 하체 추적(새 기능) 조합
**한계**: Quest 3S는 하체 추적 불가, D455가 하체만 별도 추적해야 함

### 8.3 멀티카메라 비전 + Mid-360 SLAM

```
┌──────────────────────────────────────────────────────┐
│        전 센서 융합 (최대 활용)                        │
│                                                       │
│  [Manipulation Channel]                               │
│  D455 (정면) ──────┐                                  │
│  L515 #1 (좌측) ───┼──▶ 3D 전신 포즈 → 상체 제어     │
│  L515 #2 (우측) ───┘    (삼각측량)     + 손 제어      │
│                                                       │
│  [Navigation Channel]                                 │
│  Mid-360 (로봇 탑재) ──▶ SLAM → 보행 속도 명령        │
│                                                       │
│  [두 채널 독립 → 동시 제어]                           │
│  상체 포즈 + 보행 속도 → G1 전신 명령 (29 DOF)       │
└──────────────────────────────────────────────────────┘
```

### 8.4 단계적 업그레이드 전략

기존 장비에서 시작하여 점진적으로 업그레이드:

```
Month 1: Tier 0                     Month 2: Tier 0.5
┌──────────────────────┐           ┌──────────────────────┐
│ D455 → WHAM → G1     │    +$100  │ D455 + SlimeVR IMU   │
│ (기술 검증)           │ ────────▶ │ (정확도 향상 검증)    │
│ 정확도: ★★★☆☆       │           │ 정확도: ★★★★☆       │
└──────────────────────┘           └──────────────────────┘
                                              │
                                              │ 비교 평가
                                              ▼
Month 3-4: Tier 2 (선택적)
┌──────────────────────┐
│ PICO + Tracker        │
│ + 기존 카메라 보조    │
│ 정확도: ★★★★★       │
│                       │
│ 기존 카메라 →         │
│ VLM 입력, 데이터 기록 │
└──────────────────────┘
```

---

## 9. 종합 비교표

### 9.1 전체 접근법 대형 비교표

| 접근법 | 비용 | 전신 정확도 | 손 정확도 | 지연(ms) | 구현 난이도 | Isaac Lab 통합 | 보행 | 옥클루전 내성 | RTX PRO 활용 |
|--------|------|-----------|----------|---------|-----------|--------------|------|------------|-------------|
| **A**: D455 HumanPlus | $0 | ★★★☆☆ | ★★★☆☆ | 40-60 | 중 | 높음 | △ | 낮음 | ~6GB |
| **B**: D455+L515 멀티뷰 | $0 | ★★★★☆ | ★★★☆☆ | 50-70 | 상 | 중 | ❌ | 높음 | ~3GB |
| **C-1**: D455+Mid360 | $0 | ★★★☆☆ | ★★★☆☆ | 40-60 | 중 | 높음 | ✅ | 낮음 | ~6GB |
| **C-2**: 외골격+Mid360 | $500 | ★★★★☆ | ★★☆☆☆ | 20-30 | 중상 | 중 | ✅ | 높음 | ~1GB |
| **D**: L515×2 양손 | $0 | ❌ | ★★★★★ | 30-40 | 중하 | 중 | ❌ | 중 | ~2GB |
| **Tier 0.5**: +SlimeVR | $100-200 | ★★★★☆ | ★★★☆☆ | 30-50 | 중 | 높음 | ✅ | 높음 | ~6GB |
| **Tier 1**: +GELLO | $300 | ❌ | ★★★★☆ | 10-20 | 중하 | 중 | ❌ | 해당없음 | ~0GB |
| **Tier 2**: +PICO | $900 | ★★★★★ | ★★★★☆ | 20-30 | 중 | 높음 | ✅ | 해당없음 | ~4GB |
| **Tier 3**: +PICO+MANUS | $3,900+ | ★★★★★ | ★★★★★ | 20-30 | 중 | 높음 | ✅ | 해당없음 | ~4GB |

### 9.2 UST Phase별 최적 매칭

| Phase | 목표 | 최적 접근법 | 비용 | 기존 카메라 역할 |
|-------|------|-----------|------|---------------|
| **1**: 테이블 양팔 조작 | Kitchen sorting (G1) | Quest 3S (기존) or 접근법 A | $0 | D455/L515: 관찰+VLM |
| **2**: 전신 이동+조작 | 로코매니퓰레이션 | Tier 0.5 (SlimeVR) or Tier 2 (PICO) | $100-900 | Mid-360: SLAM, D455: 보조 |
| **3**: 노인 지원 | 복합 전신 동작 | Tier 2 (PICO) + 기존 전체 | $900 | D455/L515: VLM, Mid-360: 내비 |

### 9.3 GPU 부하 분석 (RTX PRO 6000, 96GB VRAM)

| 동시 실행 조합 | VRAM 사용량 | 잔여 VRAM | 실행 가능 여부 |
|---------------|-----------|---------|-------------|
| WHAM + HaMeR | ~6 GB | 90 GB | ✅ 여유로움 |
| WHAM + HaMeR + Isaac Sim | ~14 GB | 82 GB | ✅ 여유로움 |
| WHAM + HaMeR + Isaac Sim + Qwen3-VL-8B | ~24 GB | 72 GB | ✅ 여유로움 |
| WHAM + HaMeR + Isaac Sim + Qwen3-VL-32B | ~79 GB | 17 GB | ✅ 가능 |
| 전체 (포즈추정 + 시뮬 + VLM-32B + BC-RNN 학습) | ~85 GB | 11 GB | ⚠️ 타이트 |
| PICO/SONIC + Isaac Sim + Qwen3-VL-32B | ~73 GB | 23 GB | ✅ 가능 |

**결론**: RTX PRO 6000은 비전 기반 텔레오퍼레이션 파이프라인의 모든 구성요소를 동시에 실행할 수 있을 만큼 VRAM이 충분. Qwen3-VL-32B를 8B로 대체하면 학습까지 동시 가능.

---

## 10. 최종 권장 전략 (단계별 로드맵)

### 10.1 Phase 0: 즉시 실행 (Month 1, $0)

**목표**: D455 기반 비전 텔레오퍼레이션 프로토타입 구축 및 기술 검증

| 주차 | 작업 | 산출물 |
|------|------|--------|
| **Week 1** | WHAM + HaMeR 설치, D455 RGB 연동 테스트 | 실시간 3D 포즈 스트림 |
| **Week 2** | GMR 설치, G1 리타겟팅 연동 | D455 → WHAM → GMR → G1 (Isaac Sim) |
| **Week 3** | D455 Depth 보정 구현, temporal filtering 추가 | 정확도 향상된 파이프라인 |
| **Week 4** | L515 ×2 멀티뷰 캘리브레이션 + 삼각측량 테스트 | 멀티뷰 정확도 비교 데이터 |

**핵심 산출물**:
- 비전 기반 텔레오퍼레이션 데모 (20-50개)
- 단일 카메라 vs 멀티 카메라 정확도 비교 보고서
- 기존 Quest 3S 대비 정확도 비교 데이터

### 10.2 Phase 0.5: 최소 투자 검증 (Month 2, +$100-200)

**목표**: SlimeVR IMU 추가로 비전+IMU 융합 효과 검증

| 주차 | 작업 | 산출물 |
|------|------|--------|
| **Week 5** | SlimeVR 5-7개 제작 (ESP32 + IMU 납땜) | DIY IMU 트래커 |
| **Week 6** | SlimeVR → OSC 프로토콜 → Python 수신 | IMU 데이터 스트림 |
| **Week 7** | EKF 센서 융합 (D455 비전 + SlimeVR IMU) | 융합 파이프라인 |
| **Week 8** | 정확도 비교 (비전 단독 vs 비전+IMU) | 정량적 비교 보고서 |

**의사결정 포인트**: Phase 0.5 결과에 따라
- 비전+IMU 충분 → **Tier 0.5에서 데이터 수집 지속** (추가 투자 불필요)
- 정확도 부족 → **Phase 1로 진행** (PICO 구매)

### 10.3 Phase 1: 최적 텔레오퍼레이션 (Month 3-4, +$900)

**목표**: PICO 4 Ultra + SONIC/GR00T WBC로 최고 품질 텔레오퍼레이션

| 주차 | 작업 | 산출물 |
|------|------|--------|
| **Week 9-10** | PICO 4 Ultra + Motion Tracker 구매 및 설정 | SONIC 하드웨어 |
| **Week 11-12** | SONIC/GR00T WBC → Isaac Lab 통합 | 전신 텔레오퍼레이션 시스템 |
| **Week 13-14** | 기존 카메라 보조 연동 (VLM, 관찰, 데이터 기록) | 통합 시스템 |
| **Week 15-16** | 대규모 데모 수집 (100+ demos) | 학습용 데이터셋 |

### 10.4 기존 카메라의 장기 활용 전략

텔레오퍼레이션이 VR/외골격으로 전환되어도, 기존 카메라는 **영구적으로 가치**를 가짐:

```
┌──────────────────────────────────────────────────────────────┐
│              기존 카메라 장기 활용 로드맵                      │
│                                                               │
│  ┌─────────────┐                                              │
│  │ D455        │──▶ VLM 입력 (Qwen3-VL RGB-D)                │
│  │             │──▶ 물체 인식/추적 (grasping 관점)             │
│  │             │──▶ 데이터 기록용 관찰 카메라                   │
│  │             │──▶ Phase 3 VLM Analyzer에 RGB-D 공급         │
│  └─────────────┘                                              │
│                                                               │
│  ┌─────────────┐                                              │
│  │ L515 ×2     │──▶ 정밀 물체 6DOF 포즈 추정                  │
│  │             │──▶ 근거리 grasping depth 보조                 │
│  │             │──▶ 테이블 위 물체 3D 재구성                    │
│  └─────────────┘                                              │
│                                                               │
│  ┌─────────────┐                                              │
│  │ Mid-360     │──▶ 실내 SLAM (FAST-LIO2)                    │
│  │             │──▶ 내비게이션 + 장애물 회피                    │
│  │             │──▶ Phase 3 노인 지원 환경 인식                 │
│  │             │──▶ HumanoidExo 방식 보행 오도메트리            │
│  └─────────────┘                                              │
└──────────────────────────────────────────────────────────────┘
```

### 10.5 총 비용 시나리오 요약

| 시점 | 누적 투자 | 보유 능력 | 정확도 |
|------|---------|---------|-------|
| **즉시** | $0 | D455 비전 텔레오퍼레이션 (HumanPlus) | ★★★☆☆ |
| **+2주** | $0 | + L515 멀티뷰 삼각측량 | ★★★★☆ |
| **+1개월** | $100-200 | + SlimeVR IMU 센서 융합 | ★★★★☆ |
| **+2개월** | $500 | + HOMIE 외골격 (선택적) | ★★★★☆ |
| **+3개월** | $1,100 | + PICO/SONIC (최고 품질) | ★★★★★ |
| **+4개월** | $4,100 | + MANUS 글러브 (프리미엄) | ★★★★★+ |

---

## 11. Isaac Lab 구현 가이드

### 11.1 기존 코드 활용 범위

| 기존 파일 | 재사용 가능성 | 용도 |
|----------|-------------|------|
| `ust_config/ust_scene_cfg.py` → `USTSceneWithSensorsCfg` | ✅ 직접 사용 | D455 CameraCfg + Mid-360 RayCasterCfg 이미 설정됨 |
| `devices/openxr/retargeters/.../g1_upper_body_retargeter.py` | ✅ 입력 변환 | SMPL → G1 상체 리타겟팅 (OpenXR 대신 카메라 입력) |
| `devices/openxr/retargeters/.../g1_dex_retargeting_utils.py` | ✅ 입력 변환 | MANO → INSPIRE hand (카메라 HaMeR 출력 호환) |
| `corrective/phase3/vlm_analyzer.py` | ✅ 패턴 참고 | 카메라 RGB → VLM 파이프라인 패턴 |
| `ust_utils/hdf5_recorder.py` | ✅ 직접 사용 | 비전 텔레오퍼레이션 데모 기록 |
| `ust_config/ust_teleop_device_cfg.py` | 참고용 | 텔레오퍼레이션 디바이스 설정 패턴 |
| `devices/device_base.py` | ✅ 상속 | 새 VisionTeleopDevice 기반 클래스 |

### 11.2 새로운 Device 클래스: VisionTeleopDevice

기존 `DeviceBase`를 상속하여 카메라 기반 텔레오퍼레이션 디바이스 구현:

```python
from isaaclab.devices.device_base import DeviceBase
from dataclasses import dataclass

@configclass
class VisionTeleopDeviceCfg:
    """비전 기반 텔레오퍼레이션 디바이스 설정"""
    # 카메라 설정
    camera_type: str = "d455"  # "d455", "l515", "multi"
    camera_serial: str = ""    # RealSense serial number

    # 포즈 추정 설정
    pose_estimator: str = "wham"  # "wham", "mediapipe", "openpose"
    hand_estimator: str = "hamer"  # "hamer", "mediapipe_hands"

    # 리타겟팅 설정
    retargeter: str = "gmr"  # "gmr", "isaac_lab_builtin"
    target_robot: str = "unitree_g1"

    # 필터링 설정
    use_temporal_filter: bool = True
    filter_min_cutoff: float = 1.0
    filter_beta: float = 0.007

    # Depth 보정
    use_depth_correction: bool = True
    depth_median_window: int = 5


class VisionTeleopDevice(DeviceBase):
    """D455/L515 카메라 기반 텔레오퍼레이션 디바이스

    HumanPlus 방식: RGB → WHAM(전신) + HaMeR(손) → GMR 리타겟팅 → 로봇 관절
    """

    def __init__(self, cfg: VisionTeleopDeviceCfg):
        super().__init__()
        self.cfg = cfg

        # RealSense 카메라 초기화
        self._init_camera()

        # 포즈 추정 모델 로드
        self._init_pose_estimator()

        # 리타겟터 초기화
        self._init_retargeter()

        # Temporal filter
        if cfg.use_temporal_filter:
            self._filters = {
                joint: OneEuroFilter(cfg.filter_min_cutoff, cfg.filter_beta)
                for joint in range(29)  # G1 29 DOF
            }

    def advance(self) -> torch.Tensor:
        """한 프레임 처리: 카메라 → 포즈 추정 → 리타겟팅 → joint angles"""
        # 1. 카메라 프레임 획득
        rgb, depth = self._get_camera_frame()

        # 2. 전신 포즈 추정
        smpl_params = self.pose_estimator.predict(rgb)

        # 3. Depth 보정 (옵션)
        if self.cfg.use_depth_correction and depth is not None:
            smpl_params = self._correct_with_depth(smpl_params, depth)

        # 4. 손 포즈 추정
        mano_params = self.hand_estimator.predict(rgb)

        # 5. 리타겟팅
        joint_angles = self.retargeter.retarget(smpl_params, mano_params)

        # 6. Temporal filtering
        if self.cfg.use_temporal_filter:
            joint_angles = self._apply_filters(joint_angles)

        return torch.tensor(joint_angles, dtype=torch.float32)

    def reset(self):
        """필터 상태 초기화"""
        if self.cfg.use_temporal_filter:
            for f in self._filters.values():
                f.reset()
```

### 11.3 코드 구조 제안

```
ust_ws/ust_260207/
├── ust_config/
│   ├── ust_vision_teleop_cfg.py      # VisionTeleopDeviceCfg + 프리셋
│   └── ust_teleop_device_cfg.py      # 기존 (DEVICE_MAP에 "vision" 추가)
├── ust_controllers/
│   ├── vision_pose_estimator.py      # WHAM/HaMeR/MediaPipe 래퍼
│   ├── multi_camera_tracker.py       # 멀티뷰 삼각측량
│   └── sensor_fusion.py             # EKF 비전+IMU 융합
├── ust_utils/
│   ├── camera_calibration.py         # 멀티카메라 캘리브레이션
│   └── temporal_filter.py           # 1-Euro, Kalman 필터
└── scripts/
    ├── run_vision_teleop.py          # 비전 텔레오퍼레이션 메인
    ├── calibrate_cameras.py          # 멀티카메라 캘리브레이션 스크립트
    └── test_pose_estimation.py       # 포즈 추정 정확도 테스트
```

### 11.4 실제 카메라 → 시뮬레이션 연동

핵심 원칙: **실제 카메라(오퍼레이터 추적)** 와 **시뮬레이션 카메라(로봇 관찰)** 는 별개

```
[실제 카메라 - 오퍼레이터 측]
D455 USB → pyrealsense2 → WHAM/HaMeR → 포즈 추정 → action tensor
                                                        │
                                                        ▼
[시뮬레이션 환경]                                  env.step(action)
Isaac Sim CameraCfg → 로봇 주변 시각 관찰 → observation tensor
                                                        │
                                                        ▼
[데이터 수집]
hdf5_recorder → { actions: 실제 카메라 → 포즈,
                   observations: 시뮬레이션 카메라 → 이미지 }
```

### 11.5 데이터 수집 파이프라인

기존 `hdf5_recorder.py`와 호환되는 형식으로 비전 텔레오퍼레이션 데모 기록:

```python
# 비전 텔레오퍼레이션 데모 기록 구조
demo_data = {
    "actions": action_tensor,           # (T, 29) G1 joint angles
    "obs/joint_pos": joint_positions,    # (T, 29) 현재 관절 위치
    "obs/joint_vel": joint_velocities,   # (T, 29) 현재 관절 속도
    "obs/ee_pose": ee_poses,             # (T, 14) 양손 EE 6DOF
    "obs/camera_rgb": camera_images,     # (T, H, W, 3) 시뮬 카메라 RGB
    "obs/camera_depth": camera_depths,   # (T, H, W) 시뮬 카메라 depth

    # 비전 텔레오퍼레이션 추가 데이터 (디버깅/분석용)
    "teleop/raw_smpl": smpl_params,      # (T, 72) SMPL body params
    "teleop/raw_mano": mano_params,      # (T, 90) MANO hand params
    "teleop/confidence": confidence,      # (T, 33) keypoint 신뢰도
}
```

**Robomimic 호환**: 동일한 HDF5 구조이므로 BC/BC-RNN 학습에 바로 사용 가능

---

## 12. 참고문헌

### 12.1 핵심 논문 & 프레임워크

| 프레임워크 | 기관 | 연도 | 프로젝트 | GitHub |
|-----------|------|------|---------|--------|
| **HumanPlus** | Stanford | CoRL 2024 | [humanoid-ai.github.io](https://humanoid-ai.github.io/) | [MarkFzp/humanplus](https://github.com/MarkFzp/humanplus) |
| **HumanoidExo** | NUDT/Midea | 2025 | [humanoid-exo.github.io](https://humanoid-exo.github.io/) | - |
| **AnyTeleop** | NVIDIA | RSS 2023 | [yzqin.github.io/anyteleop](https://yzqin.github.io/anyteleop/) | - |
| **ACE** | 2024 | [ace-teleop.github.io](https://ace-teleop.github.io/) | [ACETeleop/ACETeleop](https://github.com/ACETeleop/ACETeleop) |
| **GELLO** | 2024 | [wuphilipp.github.io/gello_site](https://wuphilipp.github.io/gello_site/) | - |
| **UMI** | Stanford | 2024 | - | [real-stanford/universal_manipulation_interface](https://github.com/real-stanford/universal_manipulation_interface) |
| **CLONE** | 2024 | [humanoid-clone.github.io](https://humanoid-clone.github.io/) | - |
| **Open-TeleVision** | MIT/UCSD | CoRL 2024 | [robot-tv.github.io](https://robot-tv.github.io/) | [OpenTeleVision/TeleVision](https://github.com/OpenTeleVision/TeleVision) |
| **Bunny-VisionPro** | 2024 | [dingry.github.io](https://dingry.github.io/projects/bunny_visionpro.html) | [Dingry/BunnyVisionPro](https://github.com/Dingry/BunnyVisionPro) |
| **SONIC/GR00T WBC** | NVIDIA | 2025 | - | - |

### 12.2 포즈 추정 & 리타겟팅

| 도구 | 용도 | GitHub |
|------|------|--------|
| **WHAM** | 모노큘러 → SMPL-X 전신 3D (CVPR 2024) | [yohanshin/WHAM](https://github.com/yohanshin/WHAM) |
| **HaMeR** | 모노큘러 → MANO 손 메시 (CVPR 2024) | [geopavlakos/hamer](https://github.com/geopavlakos/hamer) |
| **4DHumans/HMR2.0** | 모노큘러 → SMPL 전신 | [shubham-goel/4D-Humans](https://github.com/shubham-goel/4D-Humans) |
| **GMR** | 범용 모션 리타겟팅 (ICRA 2026) | [YanjieZe/GMR](https://github.com/YanjieZe/GMR) |
| **Pose2Sim** | 2D → 3D → OpenSim | [perfanalytics/pose2sim](https://github.com/perfanalytics/pose2sim) |
| **MediaPipe** | 경량 포즈/손/얼굴 추정 | [google-ai-edge/mediapipe](https://github.com/google-ai-edge/mediapipe) |
| **OpenPose** | 멀티파슨 2D 포즈 (CMU) | [CMU-Perceptual-Computing-Lab/openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) |
| **Nuitrack** | 상용 바디 트래킹 SDK | [3DiVi/nuitrack-sdk](https://github.com/3DiVi/nuitrack-sdk) |

### 12.3 카메라 캘리브레이션 & 3D 재구성

| 도구 | 용도 | GitHub |
|------|------|--------|
| **Caliscope** | 멀티카메라 캘리브레이션 (GUI) | [mprib/caliscope](https://github.com/mprib/caliscope) |
| **NCams** | 멀티카메라 모션 캡처 | [CMGreenspon/NCams](https://github.com/CMGreenspon/NCams) |
| **VoxelPose** | 멀티카메라 3D HPE (Microsoft) | [microsoft/voxelpose-pytorch](https://github.com/microsoft/voxelpose-pytorch) |
| **RealSense ROS2** | RealSense 카메라 ROS2 드라이버 | [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) |

### 12.4 IMU & 저비용 모션 캡처

| 제품 | 타입 | 가격 | 링크 |
|------|------|------|------|
| **SlimeVR** | DIY IMU 트래커 | $50-200 | [slimevr.dev](https://slimevr.dev/), [GitHub](https://github.com/SlimeVR/) |
| **Perception Neuron 3** | 상용 IMU 수트 | $750-1,599 | [noitom.com](https://noitom.com/) |
| **Rokoko Smartsuit Pro II** | 상용 IMU 수트 | $2,745 | [rokoko.com](https://www.rokoko.com/products/smartsuit-pro) |
| **HOMIE** | 3D 프린트 외골격 | ~$500 | - |

### 12.5 RealSense + 포즈 추정 통합 프로젝트

| 프로젝트 | 내용 | GitHub |
|---------|------|--------|
| **MediaPipe + D455** | MediaPipe 2D → D455 depth → 3D | [SiaMahmoudi/...](https://github.com/SiaMahmoudi/MediaPipe-pose-estimation-using-intel-realsense-debth-camera) |
| **RSPOP** | OpenPose + RealSense → C3D | [JuanMiguelGV/rspop](https://github.com/JuanMiguelGV/rspop) |
| **RealTime3DPoseTracker** | OpenPose + D435i → 3D 실시간 | [bagridag/RealTime3DPoseTracker-OpenPose](https://github.com/bagridag/RealTime3DPoseTracker-OpenPose) |
| **RealSense Body Tracker** | RealSense + 포즈 검출 | [cansik/realsense-pose-detector](https://github.com/cansik/realsense-pose-detector) |

### 12.6 관련 UST 연구 문서

| 문서 번호 | 제목 | 관련 내용 |
|---------|------|---------|
| **연구 5** | Humanoid VR Teleop & Imitation Learning | G1 VR 텔레오퍼레이션 + IL 파이프라인 |
| **연구 6** | Elderly Fall Support | Phase 3 노인 지원 시스템 |
| **연구 8** | Corrective Teaching System | HG-DAgger + Ensemble 시스템 |
| **연구 10** | VR Hand Tracking & Dexterous Teleop | Quest 3S 정밀도 + MANUS 해결책 |
| **연구 11** | Full-Body Teleop & MoCap Hardware | SONIC/GR00T WBC + PICO 추천 (본 연구의 전편) |

---

> **문서 끝** | 작성: Claude Code | UST 프로젝트 비전 기반 텔레오퍼레이션 & RealSense 하드웨어 최적 활용 연구
> **핵심 메시지**: 기존 D455 한 대만으로도 HumanPlus 방식 전신 텔레오퍼레이션이 가능하며, $100-200 SlimeVR IMU 추가로 정확도를 크게 향상시킬 수 있다. 궁극적으로는 PICO + SONIC($900)이 최적이지만, 기존 카메라는 VLM 입력/환경 인식/데이터 기록으로 영구적 가치를 가진다.
