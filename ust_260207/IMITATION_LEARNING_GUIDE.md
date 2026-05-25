# 모방 학습 (Imitation Learning) 완전 가이드

**UST Mobile Manipulator - VR 텔레오퍼레이션 없이 모방 학습 실행하기**

**작성일**: 2026-02-08 (구현 완료 반영 업데이트)
**환경**: Isaac Lab 2.3.0, Isaac Sim 5.1.0, RTX PRO 6000 Blackwell (96GB VRAM)
**참고**: `IMPLEMENTATION_AND_EXECUTION_GUIDE.md`, `4. all_dev_gap_analysis_and_implementation_guide.md`의 구현 완료 내용 반영

---

## 목차

1. [모방 학습이란?](#1-모방-학습이란)
2. [전체 파이프라인 개요](#2-전체-파이프라인-개요)
3. [STEP 1: 데이터 수집 (키보드 텔레옵)](#3-step-1-데이터-수집-키보드-텔레옵)
4. [STEP 2: 데이터셋 구조 이해](#4-step-2-데이터셋-구조-이해)
5. [STEP 3: 데이터 증강 (Isaac Lab Mimic)](#5-step-3-데이터-증강-isaac-lab-mimic)
6. [STEP 4: 정책 학습 (Robomimic BC-RNN)](#6-step-4-정책-학습-robomimic-bc-rnn)
7. [STEP 5: 학습된 정책 평가](#7-step-5-학습된-정책-평가)
8. [대안 경로: LeRobot (ACT/Diffusion Policy)](#8-대안-경로-lerobot-actdiffusion-policy)
9. [Isaac Lab 내장 예제로 먼저 시도하기 (권장)](#9-isaac-lab-내장-예제로-먼저-시도하기-권장)
10. [UST 프로젝트 맞춤 실행](#10-ust-프로젝트-맞춤-실행)
11. [문제 해결](#11-문제-해결)
12. [핵심 개념 정리](#12-핵심-개념-정리)
13. [참고 자료](#13-참고-자료)

---

## 1. 모방 학습이란?

### 1.1 기본 개념

모방 학습(Imitation Learning, IL)은 **사람의 시연(demonstration)을 보고 로봇이 동일한 행동을 따라하도록 학습하는 방법**입니다.

강화 학습(RL)과의 핵심 차이:

| 구분 | 강화 학습 (RL) | 모방 학습 (IL) |
|------|---------------|---------------|
| **학습 신호** | 보상 함수 (reward) | 사람의 시연 데이터 |
| **데이터** | 시뮬레이션에서 탐험 | 사람이 직접 수행한 궤적 |
| **장점** | 보상만 있으면 됨 | 복잡한 보상 설계 불필요 |
| **단점** | 보상 설계 어려움, 학습 느림 | 좋은 시연 데이터 필요 |
| **적합한 태스크** | 단순 목표 (도달, 이동) | 복잡한 조작 (집기, 쌓기) |

### 1.2 행동 복제 (Behavior Cloning, BC)

가장 기본적인 모방 학습 방법입니다:

```
입력: 관측(Observation) → 신경망(Policy Network) → 출력: 액션(Action)
```

**학습 과정:**
1. 사람이 로봇을 조작하며 (관측, 액션) 쌍을 수집
2. 수집된 데이터로 지도 학습 (Supervised Learning) 수행
3. 신경망이 "이 관측을 보면 이 액션을 해야 한다"를 학습

**수식으로 표현하면:**
```
π*(a|s) = argmin_π Σ L(π(s_i), a_i)
         = "관측 s_i일 때, 사람의 액션 a_i를 최대한 따라하는 정책 π를 찾기"
```

### 1.3 BC-RNN (BC with Recurrent Neural Network)

기본 BC에 **시간적 맥락(temporal context)**을 추가한 버전:

```
[s_{t-9}, s_{t-8}, ..., s_{t-1}, s_t] → LSTM → a_t
```

- 과거 10 스텝의 관측을 LSTM에 입력
- 시퀀스의 흐름을 파악해 더 부드러운 행동 생성
- **Isaac Lab에서 기본 지원하는 가장 검증된 알고리즘**

### 1.4 전체 흐름 요약

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  데이터   │    │  데이터   │    │  정책    │    │  정책    │
│  수집     │───→│  증강     │───→│  학습    │───→│  평가    │
│ (10 데모) │    │(→1000)   │    │ (BC-RNN) │    │(시뮬/실제)│
└──────────┘    └──────────┘    └──────────┘    └──────────┘
   키보드/VR      Isaac Lab       Robomimic      Isaac Lab
   텔레옵         Mimic           / LeRobot      환경에서
```

---

## 2. 전체 파이프라인 개요

### 2.1 VR 없이 모방 학습을 하려면?

현재 VR 텔레오퍼레이션이 동작하지 않으므로, **두 가지 방법**이 있습니다:

#### 방법 A: 키보드 텔레옵으로 직접 데이터 수집 (UST 프로젝트)
```
키보드로 로봇 조작 → HDF5 데이터 저장 → Robomimic 학습 → 평가
```
- 장점: UST 프로젝트 코드 직접 사용
- 단점: 키보드로 정밀 조작 어려움, 데이터 품질 낮을 수 있음

#### 방법 B: Isaac Lab 내장 예제 활용 (권장 - 먼저 시도!)
```
Isaac Lab 제공 Franka 큐브 쌓기 예제 → 이미 검증된 파이프라인
```
- 장점: 검증된 환경, 스크립트, 설정 파일 모두 제공
- 단점: UST 로봇이 아닌 Franka 로봇 사용

**→ 방법 B로 먼저 파이프라인을 익히고, 방법 A로 UST 프로젝트에 적용하는 것을 권장합니다.**

### 2.2 파이프라인 단계별 소요 시간 (예상)

| 단계 | 소요 시간 | 비고 |
|------|----------|------|
| 데이터 수집 (10 데모) | 10~30분 | 키보드 조작 숙련도에 따라 |
| 데이터 증강 (1000 데모) | ~30분 | GPU 사용, 병렬 환경 |
| 정책 학습 (BC-RNN 2000 에폭) | ~30분 | 상태 기반 학습 |
| 정책 평가 (50 에피소드) | ~5분 | |

---

## 3. STEP 1: 데이터 수집 (키보드 텔레옵)

### 3.1 Isaac Lab 내장 데이터 수집 스크립트 (Franka 예제)

Isaac Lab은 검증된 데이터 수집 스크립트를 제공합니다:

```bash
cd /workspace/isaaclab

# Franka 큐브 쌓기 태스크로 10개 데모 수집 (키보드)
./isaaclab.sh -p scripts/tools/record_demos.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --device cpu \
    --teleop_device keyboard \
    --dataset_file ./datasets/my_first_demos.hdf5 \
    --num_demos 10
```

**키보드 조작법:**
| 키 | 기능 |
|----|------|
| W/S | End-effector 전진/후진 |
| A/D | End-effector 좌/우 이동 |
| Q/E | End-effector 상/하 이동 |
| ↑/↓/←/→ | End-effector 회전 |
| G | 그리퍼 열기/닫기 토글 |
| R | 에피소드 리셋 (성공 시) |

**데이터 수집 팁:**
- 천천히 정확하게 조작 (품질 > 속도)
- 실패한 에피소드는 자동 폐기됨
- 최소 10개의 성공 데모가 필요
- 성공 조건이 충족되면 자동으로 에피소드 완료 처리됨

### 3.2 UST 프로젝트 데이터 수집 스크립트

UST 프로젝트의 자체 수집 스크립트도 있습니다:

```bash
cd /workspace/isaaclab

# 키보드로 UST 로봇 데이터 수집 (기본) - 센서 씬 사용 시 --enable_cameras 필수
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py \
    --num_demos 10 \
    --task pick_and_place \
    --teleop_device keyboard \
    --dataset_path ./ust_ws/ust_260207/datasets \
    --headless --enable_cameras

# VR 컨트롤러로 수집 (CloudXR 연결 필요)
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py \
    --num_demos 20 \
    --teleop_device handtracking \
    --include_images \
    --enable_cameras

# 이미지 포함 수집 (비주모터 학습용)
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py \
    --num_demos 10 \
    --teleop_device keyboard \
    --include_images \
    --max_episode_length 500 \
    --min_episode_length 10 \
    --headless --enable_cameras
```

> **중요**: `USTMobileManipulatorDataCollectEnvCfg`는 카메라 센서가 포함된 씬(`USTSceneWithSensorsCfg`)을 사용합니다. 따라서 **`--enable_cameras` 플래그가 필수**입니다. 이 플래그가 없으면 `RuntimeError: A camera was spawned without the --enable_cameras flag` 에러가 발생합니다.

**UST 데이터 수집 스크립트의 작동 방식:**

1. `USTMobileManipulatorDataCollectEnvCfg` 환경 생성 (씬, 로봇, 객체, 카메라, LiDAR 포함)
2. 입력 → **11차원 액션**: 바퀴 속도 (4D) + 암 IK delta pose (6D) + 그리퍼 (1D)
   - 키보드 모드: WASD/QE → DifferentialDriveController → 4바퀴 속도
   - VR 모드: 오른쪽/왼쪽 조이스틱 전/후 → 직접 4바퀴 매핑
3. 매 스텝마다 `HDF5DatasetRecorder`가 관측/액션 쌍을 기록
4. R키 또는 손바닥 아래로 제스처(VR)로 에피소드 종료 → 최소 길이 확인 → 성공 저장
5. 모든 데모 완료 후 HDF5 파일로 저장

> **VR 컨트롤러 매핑 (Meta Quest 3S)**:
> - 오른쪽 컨트롤러 → 매니퓰레이터 SE3 트래킹
> - 오른쪽 앞쪽 트리거 → 그리퍼 on/off
> - 오른쪽 조이스틱 전/후 → 오른쪽 바퀴 2개 (RF, RR)
> - 왼쪽 조이스틱 전/후 → 왼쪽 바퀴 2개 (LF, LR)
> - (향후) 왼쪽 컨트롤러 → 2번째 매니퓰레이터 (USD에 추가 시)
>
> CloudXR 연결 설정은 `VR_CONNECTION_TROUBLESHOOTING_GUIDE.md`를 참조하세요.

> **참고 (구현 완료)**: 현재 텔레오퍼레이션 장치 설정에는 **2종 프리셋**이 제공됩니다:
> - `CURRENT_TELEOP_PRESET`: Se3RelRetargeter (상대 좌표, delta 기반) - 텔레오퍼레이션에 직관적
> - `ALL_DEV_TELEOP_PRESET`: Se3AbsRetargeter (절대 좌표) - 정밀 조작에 유리
>
> `create_ust_teleop_device(cfg, retargeter_type="rel")` 또는 `retargeter_type="abs"`로 선택 가능합니다.

**기록되는 관측 데이터 (policy 그룹: 19D):**
- `arm_joint_pos`: 암 관절 위치 (4D)
- `arm_joint_vel`: 암 관절 속도 (4D)
- `gripper_pos`: 그리퍼 위치 (1D)
- `ee_pose`: End-effector 포즈 (7D: pos 3D + quat 4D)
- `object_pos`: 대상 객체 위치 (3D)

**기록되는 액션 데이터 (11D):**
- [0:4]: 바퀴 속도 (LF, RF, LR, RR) - 4바퀴 독립 제어
- [4:10]: 암 delta pose (위치 3D + 회전 3D)
- [10]: 그리퍼 명령 (0=열림, 1=닫힘)

---

## 4. STEP 2: 데이터셋 구조 이해

### 4.1 HDF5 파일 구조

모방 학습의 데이터는 **HDF5 형식**으로 저장됩니다. HDF5는 대용량 과학 데이터를 위한 파일 형식으로, 계층적 구조를 가집니다.

```
my_demos.hdf5
│
├── [속성] num_episodes: 10          # 전체 에피소드 수
├── [속성] total_steps: 2500         # 전체 스텝 수
├── [속성] env_args: {...}           # 환경 정보 JSON
├── [속성] action_dim: 11            # 액션 차원
│
├── data/                            # 데이터 그룹
│   ├── demo_0/                      # 첫 번째 데모
│   │   ├── [속성] success: True     # 성공 여부
│   │   ├── [속성] length: 250      # 에피소드 길이
│   │   ├── [속성] duration: 12.5   # 소요 시간 (초)
│   │   │
│   │   ├── obs          (250, 19)   # 관측 데이터 [T × obs_dim]
│   │   ├── actions      (250, 11)   # 액션 데이터 [T × action_dim]
│   │   ├── dones        (250,)      # 종료 플래그 [T]
│   │   ├── timestamps   (250,)      # 타임스탬프 [T]
│   │   ├── rewards      (250,)      # 보상 (옵션) [T]
│   │   ├── images_rgb   (250, 720, 1280, 3)   # RGB 이미지 [옵션]
│   │   └── images_depth (250, 720, 1280)       # 깊이 이미지 [옵션]
│   │
│   ├── demo_1/
│   │   └── ... (같은 구조)
│   └── demo_9/
│       └── ...
│
└── mask/                            # 학습/검증 분할 마스크
    ├── train    (10,) bool          # 학습용 에피소드 마스크
    └── valid    (10,) bool          # 검증용 에피소드 마스크
```

### 4.2 Robomimic 호환 HDF5 구조

Isaac Lab의 내장 스크립트 (`scripts/tools/record_demos.py`)는 **Robomimic과 완전 호환되는 구조**로 저장합니다:

```
robomimic_compatible.hdf5
│
├── data/
│   ├── [속성] total: 2500              # 전체 샘플 수
│   ├── [속성] env_args: {"env_name": "...", "type": 2}
│   │
│   ├── demo_0/
│   │   ├── [속성] num_samples: 250
│   │   ├── [속성] success: True
│   │   │
│   │   ├── actions         (250, 7)    # 액션
│   │   ├── initial_state/              # 초기 상태 (리셋용)
│   │   │   ├── robot_joint_pos  (8,)
│   │   │   └── object_pos      (3,)
│   │   └── obs/                        # 관측 (키별 분리)
│   │       ├── eef_pos     (250, 3)    # EE 위치
│   │       ├── eef_quat    (250, 4)    # EE 방향
│   │       ├── gripper_pos (250, 1)    # 그리퍼
│   │       └── object      (250, 14)   # 객체 상태
```

**핵심 차이:**
- UST 레코더: `obs`가 단일 연결 벡터 `(T, obs_dim)`
- Robomimic 호환: `obs/` 하위에 키별 개별 데이터셋

### 4.3 데이터셋 확인 코드

```python
import h5py
import numpy as np

# HDF5 파일 열기
f = h5py.File("./datasets/my_demos.hdf5", "r")

# 기본 정보
print(f"에피소드 수: {f.attrs.get('num_episodes', 'N/A')}")
print(f"총 스텝 수: {f.attrs.get('total_steps', 'N/A')}")

# 데모 목록
demos = list(f["data"].keys())
print(f"데모 목록: {demos}")

# 첫 번째 데모 확인
demo = f["data/demo_0"]
print(f"관측 형태: {demo['obs'].shape}")      # (T, obs_dim)
print(f"액션 형태: {demo['actions'].shape}")   # (T, action_dim)
print(f"성공 여부: {demo.attrs.get('success')}")
print(f"에피소드 길이: {demo.attrs.get('length')}")

# 데이터 분포 확인
actions = demo["actions"][:]
print(f"액션 범위: [{actions.min():.3f}, {actions.max():.3f}]")
print(f"액션 평균: {actions.mean(axis=0)}")

f.close()
```

### 4.4 UST HDF5DatasetReader 사용

```python
# UST 프로젝트의 리더 클래스 사용
import sys
sys.path.insert(0, "/workspace/isaaclab/ust_ws/ust_260207")
from ust_utils.hdf5_recorder import HDF5DatasetReader

reader = HDF5DatasetReader("./datasets/my_demos.hdf5")
reader.print_info()

# 특정 에피소드 데이터 접근
ep = reader.get_episode(0)
print(f"관측: {ep['obs'].shape}")
print(f"액션: {ep['actions'].shape}")
print(f"성공: {ep['success']}")

# 학습/검증 에피소드 확인
train_eps = reader.get_train_episodes()
valid_eps = reader.get_valid_episodes()
print(f"학습용: {train_eps}")
print(f"검증용: {valid_eps}")

reader.close()
```

---

## 5. STEP 3: 데이터 증강 (Isaac Lab Mimic)

### 5.1 Isaac Lab Mimic이란?

10개의 사람 시연 데이터로부터 **1,000개 이상의 새로운 시연**을 자동 생성하는 시스템입니다.

**핵심 아이디어:**
```
원본 시연에서의 "로봇-객체 상대 관계"를 보존하면서,
새로운 객체 위치에 맞게 궤적을 변환
```

**예시:**
- 원본: 큐브가 (0.4, 0.0, 0.3)에 있을 때의 집기 동작
- 변환: 큐브가 (0.35, 0.1, 0.32)에 있을 때로 궤적 자동 변환

### 5.2 작동 원리 (3단계)

#### 단계 1: 서브태스크 분할

사람의 시연을 의미 있는 구간으로 나눕니다:

```
시연: [접근 → 잡기 → 이동 → 놓기]
       ↓       ↓       ↓      ↓
서브:  ST1     ST2     ST3    ST4
```

각 서브태스크의 경계는 **종료 신호(termination signal)**로 표시됩니다:
- `grasp_1`: 큐브를 잡았을 때 True
- `stack_1`: 큐브를 올렸을 때 True

#### 단계 2: 좌표 변환

새로운 환경에서의 객체 위치에 맞게 EE 궤적을 변환:

```python
# 핵심 변환 알고리즘 (의사코드)
def transform_trajectory(source_eef_poses, source_obj_pose, new_obj_pose):
    # 1. 원본 EE 궤적을 객체 좌표계로 변환 (상대 좌표)
    relative_poses = source_eef_poses - source_obj_pose

    # 2. 새로운 객체 위치에 맞게 절대 좌표로 복원
    new_eef_poses = relative_poses + new_obj_pose

    return new_eef_poses
```

#### 단계 3: 시뮬레이션 실행 + 검증

변환된 궤적을 실제 시뮬레이션에서 실행하여 성공 여부를 확인:
- 성공 → 데이터셋에 추가
- 실패 → 폐기하고 다른 시도

### 5.3 실행 방법

**단계 1: 서브태스크 주석 추가**

```bash
cd /workspace/isaaclab

# 자동 주석 모드 (환경에서 서브태스크 완료 신호 자동 감지)
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
    --device cpu \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --auto \
    --input_file ./datasets/my_first_demos.hdf5 \
    --output_file ./datasets/annotated_demos.hdf5
```

**단계 2: 데이터 생성**

```bash
# 주석된 데이터에서 1000개 데모 생성
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --device cpu \
    --headless \
    --num_envs 10 \
    --generation_num_trials 1000 \
    --input_file ./datasets/annotated_demos.hdf5 \
    --output_file ./datasets/generated_1k.hdf5
```

**파라미터 설명:**
| 파라미터 | 설명 |
|---------|------|
| `--device cpu` | CPU 사용 (GPU도 가능) |
| `--headless` | GUI 없이 실행 (빠름) |
| `--num_envs 10` | 10개 병렬 환경으로 동시 생성 |
| `--generation_num_trials 1000` | 1000개 성공 데모 목표 |
| `--input_file` | 주석된 소스 데이터셋 |
| `--output_file` | 생성될 데이터셋 경로 |

**기대 결과:**
- 성공률: ~50% (1000개 생성 시 ~2000번 시도)
- 소요 시간: ~30분 (10개 병렬 환경 기준)

### 5.4 통합 데모 스크립트 (수집 + 생성 동시)

Isaac Lab은 데이터 수집과 생성을 동시에 수행하는 스크립트도 제공합니다:

```bash
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/consolidated_demo.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --num_envs 5 \
    --teleop_env_index 0 \
    --teleop_device keyboard \
    --output_file ./datasets/recorded_demos.hdf5 \
    --generated_output_file ./datasets/generated_demos.hdf5
```

이 스크립트는:
- 환경 0: 사람이 키보드로 조작 (텔레옵)
- 환경 1~4: 수집된 데이터로 자동 증강 생성
- 실시간으로 시연 + 증강이 동시에 진행됨

---

## 6. STEP 4: 정책 학습 (Robomimic BC-RNN)

### 6.1 Robomimic 설치

```bash
cd /workspace/isaaclab

# Robomimic 설치 (Isaac Lab 통합 버전)
./isaaclab.sh -i robomimic
```

### 6.2 UST 프로젝트 학습 스크립트 (구현 완료)

`scripts/train_policy.py`가 구현되어, UST 프로젝트에서 직접 Robomimic BC/BC-RNN 학습을 실행할 수 있습니다:

```bash
cd /workspace/isaaclab

# BC-RNN 학습 (권장)
./isaaclab.sh -p ust_ws/ust_260207/scripts/train_policy.py \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_20260208_120000.hdf5 \
    --epochs 2000

# BC 학습 (시퀀스 없이)
./isaaclab.sh -p ust_ws/ust_260207/scripts/train_policy.py \
    --algo bc \
    --dataset ./datasets/ust_manipulation_20260208_120000.hdf5 \
    --epochs 1000

# 고급 옵션
./isaaclab.sh -p ust_ws/ust_260207/scripts/train_policy.py \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_20260208_120000.hdf5 \
    --epochs 2000 \
    --batch_size 100 \
    --seq_length 10 \
    --output_dir ./trained_models \
    --seed 42
```

**UST train_policy.py 파라미터:**
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `--algo` | `bc_rnn` | 알고리즘 (`bc` 또는 `bc_rnn`) |
| `--dataset` | (필수) | HDF5 데이터셋 경로 |
| `--task` | `manipulation` | 태스크 이름 |
| `--output_dir` | `./trained_models` | 학습 결과 저장 디렉토리 |
| `--epochs` | `2000` | 학습 에포크 수 |
| `--batch_size` | `100` | 배치 크기 |
| `--seq_length` | `10` | 시퀀스 길이 (BC-RNN) |
| `--seed` | `42` | 랜덤 시드 |
| `--config` | `None` | 커스텀 Robomimic JSON 설정 파일 |

**학습 출력:**
- 설정 JSON: `./trained_models/config_bc_rnn_manipulation.json`
- 체크포인트: `./trained_models/` (Robomimic 설치 시)

> **Robomimic 미설치 시**: `train_policy.py`는 설치 안내 메시지와 Isaac Lab 내장 학습 대안 명령어를 출력합니다.

### 6.3 Isaac Lab 내장 학습 실행 (Franka 예제)

```bash
# 상태 기반 BC-RNN 학습 (Isaac Lab 내장)
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --algo bc \
    --dataset ./datasets/generated_1k.hdf5
```

**핵심 파라미터:**
| 파라미터 | 설명 |
|---------|------|
| `--task` | 학습할 환경 이름 (Gym 등록 ID) |
| `--algo bc` | 알고리즘 (`bc` = BC-RNN) |
| `--dataset` | 학습 데이터 HDF5 경로 |
| `--epochs` | 에폭 수 (기본: 2000) |
| `--normalize_training_actions` | 액션을 [-1,1]로 정규화 |

### 6.4 학습 설정 (JSON Config) 이해

학습은 JSON 설정 파일에 의해 제어됩니다.

#### UST train_policy.py가 생성하는 BC-RNN 설정

`train_policy.py --algo bc_rnn` 실행 시 자동 생성되는 설정:

```json
{
    "algo_name": "bc_rnn",
    "experiment": {
        "name": "ust_bc_rnn_manipulation",
        "validate": true,
        "epoch_every_n_steps": 100,
        "save": { "enabled": true, "every_n_epochs": 50 }
    },
    "train": {
        "data": "./datasets/ust_manipulation_*.hdf5",
        "output_dir": "./trained_models",
        "num_epochs": 2000,
        "batch_size": 100,
        "seed": 42
    },
    "observation": {
        "modalities": {
            "obs": {
                "low_dim": [
                    "arm_joint_pos",
                    "gripper_pos",
                    "ee_pos",
                    "ee_quat",
                    "object_pos"
                ]
            }
        }
    },
    "algo": {
        "optim_params": {
            "policy": { "learning_rate": { "initial": 1e-4 } }
        },
        "actor_layer_dims": [300, 400],
        "rnn": {
            "enabled": true,
            "horizon": 10,
            "hidden_dim": 400,
            "rnn_type": "LSTM",
            "num_layers": 2
        },
        "gmm": {
            "enabled": true,
            "num_modes": 5,
            "min_std": 0.0001
        }
    }
}
```

> **참고**: UST 관측 키(`arm_joint_pos`, `gripper_pos`, `ee_pos`, `ee_quat`, `object_pos`)는 `ust_observations_cfg.py`의 `PolicyCfg`와 일치합니다.

#### Isaac Lab Franka 큐브 쌓기 기본 설정 (참고용)

**파일**: `source/isaaclab_tasks/.../stack/config/franka/agents/robomimic/bc_rnn_low_dim.json`

```json
{
    "algo_name": "bc",
    "train": {
        "seq_length": 10,           // LSTM 시퀀스 길이
        "batch_size": 100,          // 배치 크기
        "num_epochs": 2000,         // 전체 에폭 수
        "hdf5_cache_mode": "all"    // 데이터 전부 메모리 로드
    },
    "algo": {
        "rnn": {
            "enabled": true,        // RNN 활성화 (BC-RNN)
            "horizon": 10,          // 시퀀스 길이
            "hidden_dim": 400,      // LSTM 히든 차원
            "rnn_type": "LSTM",     // RNN 타입
            "num_layers": 2         // LSTM 레이어 수
        },
        "gmm": {
            "enabled": true,        // Gaussian Mixture Model
            "num_modes": 5          // GMM 모드 수
        }
    },
    "observation": {
        "modalities": {
            "obs": {
                "low_dim": ["eef_pos", "eef_quat", "gripper_pos", "object"],
                "rgb": []            // 이미지 없음 (상태 기반)
            }
        }
    }
}
```

**주요 학습 하이퍼파라미터 해설:**

| 파라미터 | 값 | 의미 |
|---------|---|------|
| `seq_length` | 10 | 과거 10 스텝의 관측을 입력으로 사용 |
| `batch_size` | 100 | 한 번에 100개 시퀀스를 학습 |
| `num_epochs` | 2000 | 전체 데이터를 2000번 반복 학습 |
| `hidden_dim` | 400 | LSTM 내부 상태 크기 |
| `num_modes` | 5 | GMM의 가우시안 분포 수 (다중 모달 액션) |

### 6.4 학습 과정 모니터링

```bash
# TensorBoard 실행
tensorboard --logdir logs/robomimic --port 6006
```

**주요 모니터링 지표:**
- `train/loss`: 학습 손실 (감소해야 함)
- `val/loss`: 검증 손실 (과적합 감지)
- `rollout/success_rate`: 롤아웃 성공률 (증가해야 함)

### 6.5 학습 결과 파일 구조

```
logs/robomimic/
└── bc_rnn_YYYYMMDDHHMMSS/
    ├── config.json               # 학습 설정
    ├── logs/
    │   └── log.txt              # 학습 로그
    ├── models/
    │   ├── model_epoch_100.pth  # 100 에폭 체크포인트
    │   ├── model_epoch_200.pth  # 200 에폭 체크포인트
    │   ├── ...
    │   └── model_epoch_2000.pth # 최종 체크포인트
    └── videos/                   # 롤아웃 영상 (있는 경우)
```

**중요 팁:** 최종 체크포인트가 항상 최고 성능은 아닙니다. **여러 체크포인트를 테스트**하세요.

### 6.6 이미지 기반 학습 (비주모터 정책)

카메라 이미지를 관측으로 사용하는 학습도 가능합니다:

```bash
# 이미지 포함 데이터로 학습
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-v0 \
    --algo bc \
    --dataset ./datasets/visuomotor_demos.hdf5
```

이 경우:
- 입력: 카메라 이미지 (84×84 또는 원본 해상도) + 저차원 상태
- 인코더: ResNet18 + SpatialSoftmax
- 배치 크기 감소: 100 → 16 (GPU 메모리)
- 에폭 감소: 2000 → 600

---

## 7. STEP 5: 학습된 정책 평가

### 7.1 기본 평가

```bash
cd /workspace/isaaclab

# 학습된 정책 평가 (50 에피소드)
./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
    --device cpu \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --num_rollouts 50 \
    --checkpoint logs/robomimic/bc_rnn_YYYYMMDDHHMMSS/models/model_epoch_2000.pth
```

**출력 예시:**
```
[INFO] Rollout 1/50: SUCCESS (steps: 145)
[INFO] Rollout 2/50: SUCCESS (steps: 167)
[INFO] Rollout 3/50: FAILURE (steps: 300)
...
[RESULT] Success Rate: 42/50 = 84.0%
[RESULT] Average Steps to Success: 158.3
```

### 7.2 다양한 체크포인트 비교

```bash
# 여러 체크포인트를 순차적으로 평가
for epoch in 500 1000 1500 2000; do
    echo "=== Epoch $epoch ==="
    ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
        --device cpu \
        --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
        --num_rollouts 20 \
        --checkpoint logs/robomimic/bc_rnn_*/models/model_epoch_${epoch}.pth
done
```

### 7.3 강건성 평가 (Robust Evaluation)

다양한 환경 설정에서 정책의 일반화 능력을 테스트합니다:

```bash
./isaaclab.sh -p scripts/imitation_learning/robomimic/robust_eval.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-v0 \
    --input_dir logs/robomimic/bc_rnn_*/models \
    --log_dir robust_results/ \
    --seeds 0 \
    --num_rollouts 15
```

7가지 설정에서 평가:
- `vanilla`: 기본 설정
- `light_intensity`: 조명 강도 변경
- `light_color`: 조명 색상 변경
- `light_texture`: 조명 텍스처 변경
- `table_texture`: 테이블 텍스처 변경
- `robot_texture`: 로봇 텍스처 변경
- `combined`: 모든 변경 동시 적용

### 7.4 성능 기대치

**Franka 큐브 쌓기 (Isaac Lab 제공 예제):**

| 데이터셋 | 데모 수 | BC-RNN 성공률 |
|---------|---------|--------------|
| 직접 수집 | 10 | 20~40% |
| Mimic 증강 | 1,000 | 40~60% |
| Mimic 증강 | 2,000 | 60~96% |
| Mimic + Cosmos | 2,000 | 60~87% (도메인 랜덤) |

---

## 8. 대안 경로: LeRobot (ACT/Diffusion Policy)

### 8.1 LeRobot이란?

Hugging Face에서 만든 로봇 학습 프레임워크로, 최신 모방 학습 알고리즘을 제공합니다:

| 알고리즘 | 특징 |
|---------|------|
| **ACT** (Action Chunking with Transformers) | 트랜스포머 기반, 청크 단위 액션 예측 |
| **Diffusion Policy** | 확산 모델로 액션 분포 학습 |
| **SmolVLA** | 경량 비전-언어-액션 모델 (450M) |
| **PI0.5** | 멀티 로봇 지원 정책 |

### 8.2 HDF5 → LeRobot 데이터 변환

Isaac Lab에는 직접적인 LeRobot 변환기가 내장되어 있지 않지만, **Isaac Lab Arena**를 통해 가능합니다:

```bash
# Isaac Lab Arena의 변환 스크립트 사용
python isaaclab_arena_gr00t/lerobot/convert_hdf5_to_lerobot.py \
    --yaml_file isaaclab_arena_gr00t/lerobot/config/my_config.yaml
```

**YAML 설정 예시:**
```yaml
data_root: /workspace/isaaclab/datasets
hdf5_name: "generated_1k.hdf5"
language_instruction: "Pick up the green cube and place it on the table."
task_index: 0
state_name_sim: "robot_joint_pos"
action_name_sim: "processed_actions"
fps: 50
chunks_size: 1000
```

### 8.3 GR00T N1.6 파인튜닝

NVIDIA의 로봇 파운데이션 모델을 사용한 학습:

```bash
# 단일 GPU (96GB VRAM에서 가능)
CUDA_VISIBLE_DEVICES=0 python \
    submodules/Isaac-GR00T/gr00t/experiment/launch_finetune.py \
    --dataset_path ./datasets/lerobot_format \
    --output_dir ./models/groot_finetuned \
    --global_batch_size 16 \
    --max_steps 30000 \
    --num_gpus 1 \
    --base_model_path nvidia/GR00T-N1.6-3B \
    --no_tune_llm \
    --tune_visual \
    --tune_projector \
    --tune_diffusion_model
```

**참고:** LeRobot/GR00T 경로는 추가 설치와 설정이 필요하므로, **먼저 Robomimic BC-RNN으로 기본 파이프라인을 완성**하는 것을 권장합니다.

---

## 9. Isaac Lab 내장 예제로 먼저 시도하기 (권장)

### 9.1 왜 내장 예제부터?

UST 프로젝트의 커스텀 환경은:
- `Isaac-UST-MobileManip-v0` 태스크가 Gym에 등록되어 있지 않음
- Robomimic 학습 JSON 설정 파일이 없음
- Mimic용 서브태스크 신호가 구현되어 있지 않음

반면 Isaac Lab 내장 예제는:
- 모든 것이 검증되어 있음
- JSON 설정, 환경 등록, 서브태스크 신호 모두 제공
- 문서와 튜토리얼이 완비됨

### 9.2 단계별 실행 (Franka 큐브 쌓기)

#### Step 1: 데이터 수집

```bash
cd /workspace/isaaclab

# 키보드로 10개 데모 수집
./isaaclab.sh -p scripts/tools/record_demos.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --device cpu \
    --teleop_device keyboard \
    --dataset_file ./datasets/stack_cube_10demos.hdf5 \
    --num_demos 10
```

> **태스크 설명**: Franka 로봇 암이 테이블 위의 큐브를 집어서 다른 큐브 위에 쌓는 작업

#### Step 2: 서브태스크 주석

```bash
# 자동 주석
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
    --device cpu \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --auto \
    --input_file ./datasets/stack_cube_10demos.hdf5 \
    --output_file ./datasets/stack_cube_annotated.hdf5
```

#### Step 3: 데이터 증강 (Mimic)

```bash
# 1000개 데모 생성
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --device cpu \
    --headless \
    --num_envs 10 \
    --generation_num_trials 1000 \
    --input_file ./datasets/stack_cube_annotated.hdf5 \
    --output_file ./datasets/stack_cube_mimic_1k.hdf5
```

#### Step 4: Robomimic 설치 + 학습

```bash
# Robomimic 설치
./isaaclab.sh -i robomimic

# BC-RNN 학습
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --algo bc \
    --dataset ./datasets/stack_cube_mimic_1k.hdf5
```

#### Step 5: 평가

```bash
# 50 에피소드 평가
./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
    --device cpu \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --num_rollouts 50 \
    --checkpoint logs/robomimic/bc_rnn_*/models/model_epoch_2000.pth
```

### 9.3 더 간단한 태스크로 시작하기

큐브 쌓기가 어렵다면, **큐브 들기(Lift)**부터 시작:

```bash
# 큐브 들기 태스크로 데이터 수집
./isaaclab.sh -p scripts/tools/record_demos.py \
    --task Isaac-Lift-Cube-Franka-IK-Rel-v0 \
    --device cpu \
    --teleop_device keyboard \
    --dataset_file ./datasets/lift_cube_demos.hdf5 \
    --num_demos 10
```

---

## 10. UST 프로젝트 맞춤 실행

### 10.1 구현 완료된 UST 학습 파이프라인

`all_dev_gap_analysis_and_implementation_guide.md` Phase 4 구현 완료에 따라, UST 프로젝트에서 직접 모방학습을 실행할 수 있는 파이프라인이 갖춰졌습니다:

#### 즉시 사용 가능한 파이프라인

```bash
cd /workspace/isaaclab

# 1. 데이터 수집 (키보드) - --enable_cameras 필수
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py \
    --num_demos 20 \
    --teleop_device keyboard \
    --headless --enable_cameras

# 1b. 데이터 수집 (VR - CloudXR 연결 필요)
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py \
    --num_demos 20 \
    --teleop_device handtracking \
    --enable_cameras

# 2. 정책 학습 (BC-RNN)
./isaaclab.sh -p ust_ws/ust_260207/scripts/train_policy.py \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_*.hdf5 \
    --epochs 2000

# 3. (선택) Isaac Lab 내장 학습 대안
./isaaclab.sh -p source/isaaclab.robomimic/scripts/train.py \
    --task UST-MobileManipulator-v0 \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_*.hdf5
```

> **참고 (IK 솔버 선택)**: 데이터 수집 시 IK 솔버를 선택할 수 있습니다:
> - `IK_METHOD = "dls"` (기본): DLS Differential IK - 실시간 텔레오퍼레이션에 적합
> - `IK_METHOD = "lula"`: Lula IK (all_dev.md 스펙) - RMP 기반 모션 플래닝에 적합
>
> `ust_config/ust_actions_cfg.py`에서 `IK_METHOD` 변수로 전환 가능합니다.
> Lula IK 래퍼는 `ust_config/lula_ik_cfg.py`의 `LulaIKWrapper` 클래스로 구현되어 있습니다.

> **참고 (암 파라미터)**: 데이터 수집/학습 시 로봇 암 파라미터 프리셋을 선택할 수 있습니다:
> - `TURTLEBOT3_ARM_PARAMS` (기본): stiffness=100, damping=10 - TurtleBot3 실기 연동에 최적화
> - `ALL_DEV_ARM_PARAMS`: stiffness=80, damping=4 - all_dev.md 스펙의 보수적 설정
>
> `ust_config/ust_mobile_manipulator_cfg.py` 상단의 `ACTIVE_ARM_PARAMS`로 전환합니다.

### 10.2 추가 통합 작업 (아직 필요한 항목)

UST 환경을 Isaac Lab의 **표준 모방 학습 파이프라인** (Mimic 증강 등)과 완전히 연결하려면 아래 추가 작업이 필요합니다:

#### 필요 작업 1: Gymnasium 환경 등록

```python
# 예시: __init__.py에 추가
import gymnasium as gym

gym.register(
    id="Isaac-UST-MobileManip-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": "config.ust_teleop_env_cfg:USTMobileManipulatorTrainEnvCfg",
        "robomimic_bc_cfg_entry_point": "path/to/bc_rnn_config.json",
    },
)
```

#### 필요 작업 2: HDF5 형식 맞추기

UST의 `HDF5DatasetRecorder`가 저장하는 형식을 Robomimic 호환으로 변환하는 스크립트가 필요합니다:

```python
# 변환 의사코드
# UST 형식: obs (T, 15) 단일 벡터
# Robomimic 형식: obs/eef_pos (T, 3), obs/eef_quat (T, 4), ...

def convert_ust_to_robomimic(ust_hdf5_path, output_path):
    with h5py.File(ust_hdf5_path, 'r') as src:
        with h5py.File(output_path, 'w') as dst:
            for demo_key in src['data'].keys():
                obs = src[f'data/{demo_key}/obs'][:]

                # 관측 분리
                arm_joint_pos = obs[:, 0:4]   # 4D
                gripper_pos = obs[:, 4:5]     # 1D
                ee_pos = obs[:, 5:8]          # 3D
                ee_quat = obs[:, 8:12]        # 4D
                object_pos = obs[:, 12:15]    # 3D

                # Robomimic 형식으로 저장
                grp = dst.create_group(f'data/{demo_key}/obs')
                grp.create_dataset('eef_pos', data=ee_pos)
                grp.create_dataset('eef_quat', data=ee_quat)
                grp.create_dataset('gripper_pos', data=gripper_pos)
                grp.create_dataset('object', data=object_pos)
                # ... 액션, 메타데이터 등
```

#### 필요 작업 3: Mimic 환경 구현

Isaac Lab Mimic 데이터 증강을 사용하려면 `ManagerBasedRLMimicEnv`를 상속한 환경이 필요합니다:

```python
class USTMimicEnv(ManagerBasedRLMimicEnv):
    def get_robot_eef_pose(self, eef_name):
        """End-effector 4x4 포즈 반환"""
        ...

    def target_eef_pose_to_action(self, target_eef_pose_dict, ...):
        """타겟 포즈를 액션으로 변환"""
        ...

    def get_object_poses(self):
        """모든 객체의 4x4 포즈 딕셔너리 반환"""
        ...

    def get_subtask_term_signals(self):
        """서브태스크 종료 신호 반환"""
        ...
```

### 10.3 현실적인 UST 프로젝트 모방 학습 경로

**이미 완료된 단계 (✅)와 남은 단계 (⬜):**

```
✅ 1. 데이터 수집 스크립트 (record_demos.py) - 키보드/VR 지원
✅ 2. 정책 학습 스크립트 (train_policy.py) - BC/BC-RNN 지원
✅ 3. 로봇 파라미터 프리셋 (ARM_PARAMS) - TurtleBot3/all_dev 전환
✅ 4. IK 솔버 옵션 (DLS/Lula) - IK_METHOD로 선택
✅ 5. 텔레옵 프리셋 (Abs/Rel Retargeter) - retargeter_type으로 선택
✅ 6. 물리 속성 유틸리티 (physics_setup.py) - 런타임 물리 튜닝
   ↓
⬜ 7. UST 환경을 Gymnasium에 등록
⬜ 8. HDF5 형식 변환 스크립트 작성 (UST → Robomimic 호환)
⬜ 9. Isaac Lab Mimic 환경 통합 (서브태스크 신호 구현)
⬜ 10. 실제 데이터 수집 + 학습 + 평가 실행
```

> **권장**: 먼저 Isaac Lab 내장 Franka 예제 (§9)로 전체 파이프라인을 익힌 뒤, UST 프로젝트 스크립트 (`train_policy.py`)로 직접 학습을 시작하세요.

---

## 11. 문제 해결

### 11.1 "Task not found" 에러

```
gymnasium.error.NameNotFound: Environment `Isaac-UST-MobileManip-v0` doesn't exist.
```

**원인**: UST 환경이 Gymnasium에 등록되지 않음
**해결**: 먼저 Isaac Lab 내장 태스크 (`Isaac-Stack-Cube-Franka-IK-Rel-v0`) 사용

### 11.2 "robomimic not installed" 에러

```
ModuleNotFoundError: No module named 'robomimic'
```

**해결**:
```bash
./isaaclab.sh -i robomimic
```

### 11.3 "No robomimic config found" 에러

```
KeyError: 'robomimic_bc_cfg_entry_point'
```

**원인**: 해당 태스크에 Robomimic 설정이 등록되지 않음
**해결**: Robomimic 설정이 있는 태스크 사용 (예: `Isaac-Stack-Cube-Franka-IK-Rel-v0`)

### 11.4 학습 손실이 줄지 않음

**가능한 원인:**
- 데이터 품질 문제 (키보드 조작이 너무 불규칙)
- 관측/액션 차원 불일치
- 학습률이 너무 높거나 낮음

**해결:**
- 데이터셋 시각화하여 품질 확인
- 배치 크기 조정 (100 → 50)
- 에폭 수 증가 (2000 → 3000)

### 11.5 평가 시 성공률이 0%

**가능한 원인:**
- 학습 환경과 평가 환경이 다름
- 관측 정규화 불일치
- 체크포인트 경로 오류

**해결:**
- `--task`가 학습 시와 동일한지 확인
- 여러 체크포인트 시도 (초기 에폭부터)
- 학습 로그에서 검증 손실 확인

### 11.6 CUDA Out of Memory

```bash
# GPU 메모리 확인
nvidia-smi

# 배치 크기 줄이기
# bc_rnn_low_dim.json에서 batch_size: 100 → 50

# 이미지 기반 학습 시 추가 절약
# batch_size: 16 → 8
```

---

## 12. 핵심 개념 정리

### 12.1 용어 사전

| 용어 | 설명 |
|------|------|
| **Behavior Cloning (BC)** | 시연 데이터로 (관측→액션) 매핑을 지도학습 |
| **BC-RNN** | BC + LSTM으로 시간적 맥락을 고려한 행동 복제 |
| **Demonstration** | 사람이 수행한 한 번의 태스크 수행 기록 (에피소드) |
| **Episode** | 환경 리셋부터 종료까지의 한 번의 시행 |
| **HDF5** | 계층적 데이터 형식 (Hierarchical Data Format v5) |
| **Observation** | 로봇이 현재 환경에서 관찰하는 정보 (관절 각도, EE 위치 등) |
| **Action** | 로봇에게 내리는 제어 명령 (관절 속도, IK 타겟 등) |
| **Policy** | 관측 → 액션 매핑 함수 (신경망) |
| **Rollout** | 학습된 정책을 환경에서 실행하는 것 |
| **GMM** | Gaussian Mixture Model - 다중 모달 액션 분포 표현 |
| **LSTM** | Long Short-Term Memory - 시계열 학습 RNN |
| **Isaac Lab Mimic** | 소량 시연에서 대량 시연을 자동 생성하는 시스템 |
| **Subtask** | 전체 태스크의 하위 단계 (접근→잡기→이동→놓기) |
| **Retargeting** | VR 손 동작을 로봇 팔 동작으로 변환 |
| **Differential IK** | 미분 역기구학 - EE 속도를 관절 속도로 변환 |
| **End-Effector (EE)** | 로봇 팔의 끝단 (그리퍼) |
| **Teleop** | Teleoperation - 원격 조작 |

### 12.2 파일 경로 요약

**Isaac Lab 내장 스크립트 (검증됨):**
```
/workspace/isaaclab/
├── scripts/
│   ├── tools/
│   │   ├── record_demos.py              # 데이터 수집
│   │   ├── replay_demos.py              # 데이터 재생
│   │   ├── merge_hdf5_datasets.py       # 데이터셋 병합
│   │   ├── hdf5_to_mp4.py              # HDF5→영상 변환
│   │   └── mp4_to_hdf5.py              # 영상→HDF5 변환
│   └── imitation_learning/
│       ├── robomimic/
│       │   ├── train.py                 # BC-RNN 학습
│       │   ├── play.py                  # 정책 평가
│       │   └── robust_eval.py           # 강건성 평가
│       └── isaaclab_mimic/
│           ├── annotate_demos.py        # 서브태스크 주석
│           ├── generate_dataset.py      # 데이터 증강
│           └── consolidated_demo.py     # 통합 수집+증강
```

**UST 프로젝트 스크립트 (구현 완료 반영):**
```
/workspace/isaaclab/ust_ws/ust_260207/
├── scripts/
│   ├── run_teleop.py                      # VR 텔레오퍼레이션 메인
│   ├── record_demos.py                    # 데모 데이터 녹화
│   ├── run_ros2_bridge.py                 # ROS2 브릿지
│   └── train_policy.py                    # ★ 정책 학습 (BC/BC-RNN)
├── ust_config/
│   ├── __init__.py                        # 패키지 export (업데이트됨)
│   ├── ust_mobile_manipulator_cfg.py      # 로봇 설정 (ARM_PARAMS 프리셋 추가)
│   ├── ust_scene_cfg.py                   # 씬 설정
│   ├── ust_actions_cfg.py                 # 액션 (IK_METHOD 선택 추가)
│   ├── ust_observations_cfg.py            # 관측 (관절, EE, 객체)
│   ├── ust_teleop_env_cfg.py              # 환경 설정 4종
│   ├── ust_teleop_device_cfg.py           # VR 디바이스 (프리셋 + retargeter 옵션)
│   ├── lula_ik_cfg.py                     # ★ Lula IK 솔버 래퍼
│   ├── open_x1_des.yaml                   # ★ Lula robot descriptor
│   └── open_manipulator_x.urdf            # ★ OpenMANIPULATOR-X URDF
├── ust_controllers/
│   └── differential_drive_controller.py   # 차동 구동 컨트롤러
├── ust_utils/
│   ├── __init__.py                        # 패키지 export (업데이트됨)
│   ├── hdf5_recorder.py                   # HDF5 레코더/리더
│   └── physics_setup.py                   # ★ 물리 속성 유틸리티
├── datasets/                              # 수집 데이터 저장소
├── checkpoints/                           # 학습 모델 저장소
├── logs/                                  # TensorBoard 로그
│
├── IMPLEMENTATION_AND_EXECUTION_GUIDE.md  # 구현 완료 실행 가이드
├── IMITATION_LEARNING_GUIDE.md            # 이 문서
└── setup_cloudxr_env.sh                   # CloudXR 환경 설정
```

> **★ 표시**: `all_dev_gap_analysis_and_implementation_guide.md` Phase 1~5에서 새로 생성되거나 수정된 파일

### 12.3 데이터 흐름 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                    데이터 수집 단계                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  키보드/VR 입력 → Teleop Controller → Action (11D)          │
│       ↓                                    ↓               │
│  Base Wheels (4D)  +  Arm IK (6D)  +  Gripper (1D)        │
│       ↓                                    ↓               │
│  Env.step(action) → Observation (19D)                      │
│       ↓                                    ↓               │
│  HDF5DatasetRecorder.add_step(obs, action)                 │
│       ↓                                                    │
│  demo_0.hdf5: {obs: (T,19), actions: (T,11), ...}         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│                    데이터 증강 단계 (선택)                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  10개 소스 데모 → 서브태스크 분할 → 좌표 변환               │
│       ↓                                                    │
│  새 객체 위치에 맞는 궤적 생성 → 시뮬레이션 검증             │
│       ↓                                                    │
│  1000개 증강 데모 (generated_1k.hdf5)                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│                    정책 학습 단계                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  HDF5 데이터 로드 → 시퀀스 샘플링 (10 스텝)                 │
│       ↓                                                    │
│  [obs_t-9, ..., obs_t] → LSTM (400D, 2층)                 │
│       ↓                                                    │
│  → GMM (5 모드) → action_t 예측                            │
│       ↓                                                    │
│  L2 Loss(예측_action, 실제_action) → 역전파 → 파라미터 갱신 │
│       ↓                                                    │
│  2000 에폭 반복 → 체크포인트 저장 (.pth)                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│                    정책 평가 단계                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  체크포인트 로드 → 환경 리셋                                 │
│       ↓                                                    │
│  obs → 정책(obs) → action → env.step(action) → 반복        │
│       ↓                                                    │
│  성공/실패 판정 → 50 에피소드 통계                           │
│       ↓                                                    │
│  성공률: XX%, 평균 스텝: YYY                                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 13. 참고 자료

### 공식 문서
- [Isaac Lab 모방 학습 개요](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/index.html)
- [Isaac Lab 텔레옵 + 모방 학습 튜토리얼](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html)
- [Isaac Lab 증강 모방 학습](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/augmented_imitation.html)
- [Isaac Lab SkillGen](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/skillgen.html)
- [Robomimic 공식 문서](https://robomimic.github.io/)
- [Robomimic 데이터셋 튜토리얼](https://robomimic.github.io/docs/tutorials/dataset_contents.html)

### 논문
- [Behavior Cloning (Pomerleau, 1988)](https://papers.nips.cc/paper/1988/hash/812b4ba287f5ee0bc9d43bbf5bb87f4-Abstract.html): 최초의 BC 연구
- [MimicGen (2023)](https://mimicgen.github.io/): Isaac Lab Mimic의 기반 논문
- [ACT (2023)](https://tonyzhaozh.github.io/aloha/): Action Chunking with Transformers
- [Diffusion Policy (2023)](https://diffusion-policy.cs.columbia.edu/): 확산 모델 기반 정책
- [GR00T N1 (2024)](https://developer.nvidia.com/isaac/gr00t): NVIDIA 로봇 파운데이션 모델

### 커뮤니티
- [Isaac Lab GitHub Issues](https://github.com/isaac-sim/IsaacLab/issues)
- [Isaac Lab Discussions](https://github.com/isaac-sim/IsaacLab/discussions)
- [LeRobot GitHub](https://github.com/huggingface/lerobot)

---

## 빠른 시작 요약

### UST 프로젝트 파이프라인 (구현 완료)

```bash
cd /workspace/isaaclab

# 1. Robomimic 설치
./isaaclab.sh -i robomimic

# 2. 키보드로 20개 데모 수집 (UST 모바일 매니퓰레이터)
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py \
    --num_demos 20 \
    --teleop_device keyboard \
    --enable_cameras

# 3. BC-RNN 정책 학습
./isaaclab.sh -p ust_ws/ust_260207/scripts/train_policy.py \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_*.hdf5 \
    --epochs 2000
```

### Isaac Lab 내장 예제 파이프라인 (Franka 큐브 쌓기)

```bash
cd /workspace/isaaclab

# 1. 키보드로 10개 데모 수집
./isaaclab.sh -p scripts/tools/record_demos.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --device cpu \
    --teleop_device keyboard \
    --dataset_file ./datasets/stack_10.hdf5 \
    --num_demos 10

# 2. 서브태스크 주석 (자동)
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
    --device cpu \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --auto \
    --input_file ./datasets/stack_10.hdf5 \
    --output_file ./datasets/stack_annotated.hdf5

# 3. 데이터 증강 (10→1000)
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --device cpu --headless --num_envs 10 \
    --generation_num_trials 1000 \
    --input_file ./datasets/stack_annotated.hdf5 \
    --output_file ./datasets/stack_1k.hdf5

# 4. BC-RNN 학습
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --algo bc \
    --dataset ./datasets/stack_1k.hdf5

# 5. 정책 평가
./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
    --device cpu \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0 \
    --num_rollouts 50 \
    --checkpoint logs/robomimic/bc_rnn_*/models/model_epoch_2000.pth
```

---

*UST Project - 모방 학습 완전 가이드*
*작성일: 2026-02-08 (구현 완료 반영 업데이트)*
*Isaac Lab 2.3.0 / Isaac Sim 5.1.0 기준*
*참고: IMPLEMENTATION_AND_EXECUTION_GUIDE.md, 4. all_dev_gap_analysis_and_implementation_guide.md*
