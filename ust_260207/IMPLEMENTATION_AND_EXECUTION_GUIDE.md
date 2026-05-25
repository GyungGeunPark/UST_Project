# UST 모바일 매니퓰레이터 - 구현 완료 및 실행 가이드

> **작성일**: 2026-02-08
> **기반**: `all_dev_gap_analysis_and_implementation_guide.md` Phase 1~5 구현 완료

---

## 목차

1. [구현 완료 요약](#1-구현-완료-요약)
2. [변경된 파일 목록](#2-변경된-파일-목록)
3. [사전 요구사항](#3-사전-요구사항)
4. [실행 가이드](#4-실행-가이드)
   - 4.1 [환경 설정](#41-환경-설정)
   - 4.2 [VR 텔레오퍼레이션 실행](#42-vr-텔레오퍼레이션-실행)
   - 4.3 [키보드 텔레오퍼레이션 실행](#43-키보드-텔레오퍼레이션-실행)
   - 4.4 [데모 데이터 녹화](#44-데모-데이터-녹화)
   - 4.5 [정책 학습 (모방학습)](#45-정책-학습-모방학습)
   - 4.6 [ROS2 브릿지 실행](#46-ros2-브릿지-실행)
5. [설정 변경 가이드](#5-설정-변경-가이드)
   - 5.1 [암 파라미터 프리셋 변경](#51-암-파라미터-프리셋-변경)
   - 5.2 [IK 솔버 변경 (DLS ↔ Lula)](#52-ik-솔버-변경)
   - 5.3 [텔레오퍼레이션 프리셋 변경](#53-텔레오퍼레이션-프리셋-변경)
   - 5.4 [물리 속성 튜닝](#54-물리-속성-튜닝)
6. [CloudXR VR 연결 설정](#6-cloudxr-vr-연결-설정)
7. [트러블슈팅](#7-트러블슈팅)

---

## 1. 구현 완료 요약

`all_dev_gap_analysis_and_implementation_guide.md`에 따라 Phase 1~5를 모두 구현했습니다.

| Phase | 내용 | 상태 |
|-------|------|------|
| **Phase 1** | 로봇 구성 파라미터 정합 | ✅ 완료 |
| **Phase 2** | Lula IK 통합 옵션 추가 | ✅ 완료 |
| **Phase 3** | 텔레오퍼레이션 설정 정합 | ✅ 완료 |
| **Phase 4** | 학습 파이프라인 스크립트 | ✅ 완료 |
| **Phase 5** | 물리 속성 튜닝 유틸리티 | ✅ 완료 |

### 핵심 변경사항

- **ARM_PARAMS 프리셋 시스템**: `TURTLEBOT3_ARM_PARAMS` (기본) / `ALL_DEV_ARM_PARAMS` 전환 가능
- **Lula IK 래퍼**: `config/lula_ik_cfg.py` - Lula/DLS 이중 IK 지원
- **텔레오퍼레이션 프리셋**: `ALL_DEV_TELEOP_PRESET` / `CURRENT_TELEOP_PRESET`
- **Se3Abs/Rel 리타게터 선택**: `retargeter_type="abs"` 또는 `"rel"` 옵션
- **정책 학습 스크립트**: `scripts/train_policy.py` - Robomimic BC/BC-RNN
- **물리 속성 유틸리티**: `utils/physics_setup.py` - 런타임 물리 속성 적용

---

## 2. 변경된 파일 목록

### 수정된 파일 (기존)

| 파일 | 변경 내용 |
|------|----------|
| `config/ust_mobile_manipulator_cfg.py` | ARM_PARAMS 프리셋 추가, 액추에이터에 프리셋 적용 |
| `config/ust_actions_cfg.py` | `IK_METHOD` 선택 변수 추가 |
| `config/ust_teleop_device_cfg.py` | 텔레옵 프리셋 2종 추가, `retargeter_type` 파라미터 추가 |
| `config/__init__.py` | 새 모듈 export 추가 |
| `utils/__init__.py` | `physics_setup` export 추가 |

### 새로 생성된 파일

| 파일 | 역할 |
|------|------|
| `config/lula_ik_cfg.py` | Lula IK 솔버 래퍼 (all_dev.md 스펙) |
| `config/open_x1_des.yaml` | Lula robot descriptor (ust_project1에서 복사) |
| `config/open_manipulator_x.urdf` | OpenMANIPULATOR-X URDF (ust_project1에서 복사) |
| `scripts/train_policy.py` | Robomimic BC/BC-RNN 학습 스크립트 |
| `utils/physics_setup.py` | 물리 속성 설정 유틸리티 |
| `IMPLEMENTATION_AND_EXECUTION_GUIDE.md` | 이 문서 |

---

## 3. 사전 요구사항

### 필수

- **Isaac Lab 2.3.0** + **Isaac Sim 5.1.0** Docker 컨테이너
- **GPU**: NVIDIA RTX 계열 (CUDA 지원)
- **UST 프로젝트 USD 파일**: `/workspace/isaaclab/ust_ws/isaac_file/ust_project1.usd`

### VR 텔레오퍼레이션 추가 요구사항

- **CloudXR Runtime 5.0.1** Docker 이미지
- **Meta Quest 3S** (또는 호환 XR 디바이스)
- 서버와 Quest가 **같은 5GHz WiFi 네트워크**에 연결

### 모방학습 추가 요구사항 (선택)

- **Robomimic** (`pip install robomimic`)
- 수집된 HDF5 데모 데이터셋

---

## 4. 실행 가이드

### 4.1 환경 설정

#### 방법 A: Docker Compose (권장 - VR 사용 시)

```bash
# 호스트 터미널에서 실행 (Docker 컨테이너 밖)
cd /workspace/isaaclab

# openxr 공유 폴더 권한 설정
chmod -R 777 ./ust_ws/openxr/

# CloudXR Runtime + Isaac Lab 동시 시작
./docker/container.py start \
    --files ust_ws/260207_ust/docker-compose.cloudxr-ust.patch.yaml \
    --env-file docker/.env.cloudxr-runtime

# Isaac Lab 컨테이너 진입
./docker/container.py enter base
```

#### 방법 B: 수동 설정 (이미 실행 중인 컨테이너)

```bash
# Isaac Lab 컨테이너 내부에서
source /workspace/isaaclab/ust_ws/260207_ust/setup_cloudxr_env.sh

# 확인
echo "XR_RUNTIME_JSON = $XR_RUNTIME_JSON"
echo "XDG_RUNTIME_DIR = $XDG_RUNTIME_DIR"
```

#### 방법 C: VR 없이 (키보드/SpaceMouse만)

```bash
# 별도의 CloudXR 설정 불필요
cd /workspace/isaaclab
```

### 4.2 VR 텔레오퍼레이션 실행

```bash
# Isaac Lab 컨테이너 내부에서 실행
cd /workspace/isaaclab

# VR 핸드트래킹 (기본 - Se3RelRetargeter)
./isaaclab.sh -p ust_ws/260207_ust/scripts/run_teleop.py \
    --teleop_device handtracking

# VR 핸드트래킹 (카메라 활성화)
./isaaclab.sh -p ust_ws/260207_ust/scripts/run_teleop.py \
    --teleop_device handtracking \
    --enable_cameras
```

**VR 컨트롤**:
| 동작 | 기능 |
|------|------|
| 오른손 이동 | 암 End-Effector 제어 |
| 오른손 핀치 | 그리퍼 열림/닫힘 |
| 왼손 썸스틱 | 모바일 베이스 이동 |
| 손바닥 아래로 | 환경 리셋 |

### 4.3 키보드 텔레오퍼레이션 실행

```bash
# 키보드 제어 (VR 없이)
./isaaclab.sh -p ust_ws/260207_ust/scripts/run_teleop.py \
    --teleop_device keyboard
```

**키보드 컨트롤**:
| 키 | 기능 |
|----|------|
| W/A/S/D | 수평 이동 |
| Q/E | 상하 이동 |
| 방향키 | 회전 |
| G | 그리퍼 토글 |
| R | 환경 리셋 |
| ESC | 종료 |

### 4.4 데모 데이터 녹화

```bash
# 기본: 20개 데모, VR 핸드트래킹
./isaaclab.sh -p ust_ws/260207_ust/scripts/record_demos.py \
    --num_demos 20

# 이미지 포함 녹화
./isaaclab.sh -p ust_ws/260207_ust/scripts/record_demos.py \
    --num_demos 20 \
    --include_images

# 키보드로 녹화 (VR 없이)
./isaaclab.sh -p ust_ws/260207_ust/scripts/record_demos.py \
    --num_demos 10 \
    --teleop_device keyboard

# 고급 옵션
./isaaclab.sh -p ust_ws/260207_ust/scripts/record_demos.py \
    --num_demos 50 \
    --task pick_and_place \
    --include_images \
    --max_episode_length 500 \
    --min_episode_length 10 \
    --dataset_path ./datasets
```

**녹화 중 컨트롤**:
| 동작 | 기능 |
|------|------|
| 태스크 수행 | 데모 데이터 녹화 |
| 손바닥 아래로 | 에피소드 종료 (성공) |
| 주먹 제스처 | 에피소드 폐기 (실패) |
| R 키 | 현재 에피소드 재시작 |

**출력**: `./datasets/ust_manipulation_YYYYMMDD_HHMMSS.hdf5`

### 4.5 정책 학습 (모방학습)

#### Robomimic BC-RNN 학습 (권장)

```bash
# BC-RNN 학습 (기본 설정)
./isaaclab.sh -p ust_ws/260207_ust/scripts/train_policy.py \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_20260208_120000.hdf5 \
    --epochs 2000

# BC 학습 (시퀀스 없이)
./isaaclab.sh -p ust_ws/260207_ust/scripts/train_policy.py \
    --algo bc \
    --dataset ./datasets/ust_manipulation_20260208_120000.hdf5 \
    --epochs 1000

# 고급 옵션
./isaaclab.sh -p ust_ws/260207_ust/scripts/train_policy.py \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_20260208_120000.hdf5 \
    --epochs 2000 \
    --batch_size 100 \
    --seq_length 10 \
    --output_dir ./trained_models \
    --seed 42
```

**학습 출력**:
- 설정 JSON: `./trained_models/config_bc_rnn_manipulation.json`
- 체크포인트: `./trained_models/` (Robomimic 설치 시)

#### Robomimic 미설치 시 대안

```bash
# 1. Robomimic 설치
pip install robomimic

# 2. Isaac Lab 내장 학습 (대안)
./isaaclab.sh -p source/isaaclab.robomimic/scripts/train.py \
    --task UST-MobileManipulator-v0 \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_20260208_120000.hdf5
```

### 4.6 ROS2 브릿지 실행

```bash
# 시뮬레이션 → 실제 로봇 (상태 퍼블리시)
./isaaclab.sh -p ust_ws/260207_ust/scripts/run_ros2_bridge.py \
    --mode sim2real

# 실제 로봇 → 시뮬레이션 (명령 수신)
./isaaclab.sh -p ust_ws/260207_ust/scripts/run_ros2_bridge.py \
    --mode real2sim

# 양방향 미러링
./isaaclab.sh -p ust_ws/260207_ust/scripts/run_ros2_bridge.py \
    --mode bidirectional
```

**ROS2 토픽**:
| 토픽 | 타입 | 방향 |
|------|------|------|
| `/joint_states` | JointState | 퍼블리시 (sim→real) |
| `/ee_pose` | PoseStamped | 퍼블리시 (sim→real) |
| `/odom` | Odometry | 퍼블리시 (sim→real) |
| `/cmd_vel` | Twist | 구독 (real→sim) |
| `/arm_controller/command` | Float64MultiArray | 구독 (real→sim) |
| `/gripper_controller/command` | Float64MultiArray | 구독 (real→sim) |

---

## 5. 설정 변경 가이드

### 5.1 암 파라미터 프리셋 변경

`config/ust_mobile_manipulator_cfg.py` 파일의 상단:

```python
# 현재: TurtleBot3 튜닝 파라미터 (stiffness=100, damping=10)
ACTIVE_ARM_PARAMS = TURTLEBOT3_ARM_PARAMS

# all_dev.md 스펙으로 전환 시 (stiffness=80, damping=4):
ACTIVE_ARM_PARAMS = ALL_DEV_ARM_PARAMS
```

| 프리셋 | Stiffness | Damping | Velocity Limit | 적합한 상황 |
|--------|-----------|---------|---------------|------------|
| `TURTLEBOT3_ARM_PARAMS` | 100.0 | 10.0 | 4.8 rad/s | TurtleBot3 실기 연동 |
| `ALL_DEV_ARM_PARAMS` | 80.0 | 4.0 | 2.0 rad/s | 보수적 시뮬레이션 |

### 5.2 IK 솔버 변경

#### DLS (기본) - 실시간 텔레오퍼레이션

`config/ust_actions_cfg.py`:
```python
IK_METHOD = "dls"  # 현재 기본값
```

모든 액션 설정 (`USTActionsCfg`, `USTTeleopActionsCfg`)이 DLS Differential IK를 사용합니다.

#### Lula IK - RMP 기반 모션 플래닝

스탠드얼론 스크립트에서 Lula IK를 사용하는 예제:

```python
from config.lula_ik_cfg import LulaIKWrapper, LulaIKConfig
import numpy as np

# 로봇 아티큘레이션 획득 후
ik = LulaIKWrapper(robot_articulation)

# IK 계산
target_pos = np.array([0.2, 0.0, 0.3])
target_rot = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz quaternion
action, success = ik.compute_ik(target_pos, target_rot)

# 작업 공간 확인
if ik.is_in_workspace(target_pos):
    print("Target is reachable")
```

### 5.3 텔레오퍼레이션 프리셋 변경

`scripts/run_teleop.py`에서 디바이스 생성 시:

```python
# 현재 (Se3RelRetargeter, 상대 좌표)
from config.ust_teleop_device_cfg import CURRENT_TELEOP_PRESET
cfg = CURRENT_TELEOP_PRESET
device = create_ust_teleop_device(cfg, retargeter_type="rel")

# all_dev.md 스펙 (Se3AbsRetargeter, 절대 좌표)
from config.ust_teleop_device_cfg import ALL_DEV_TELEOP_PRESET
cfg = ALL_DEV_TELEOP_PRESET
device = create_ust_teleop_device(cfg, retargeter_type="abs")
```

| 프리셋 | 리타게터 | 손목 위치 | 스케일 | 적합한 상황 |
|--------|---------|----------|-------|------------|
| `CURRENT_TELEOP_PRESET` | Se3Rel | 손목 | 10.0 | 텔레오퍼레이션 (직관적) |
| `ALL_DEV_TELEOP_PRESET` | Se3Abs | 핀치 | 1.0 | 정밀 조작 |

### 5.4 물리 속성 튜닝

Isaac Sim Script Editor에서 런타임 적용:

```python
from utils.physics_setup import apply_physics_properties

# USD Stage 획득
import omni.usd
stage = omni.usd.get_context().get_stage()

# 물리 속성 적용
apply_physics_properties(stage, robot_path="/World/envs/env_0/Robot")
```

개별 함수 사용:

```python
from utils.physics_setup import create_wheel_material, set_mass

# 바퀴 마찰력 설정
create_wheel_material(stage, friction=0.9)

# 특정 링크 질량 변경
set_mass(stage, "/World/envs/env_0/Robot/base_link", mass=2.0)
```

---

## 6. CloudXR VR 연결 설정

### Step 1: CloudXR Runtime 시작

```bash
# 호스트 터미널에서
cd /workspace/isaaclab

chmod -R 777 ./ust_ws/openxr/

./docker/container.py start \
    --files ust_ws/260207_ust/docker-compose.cloudxr-ust.patch.yaml \
    --env-file docker/.env.cloudxr-runtime
```

### Step 2: 상태 확인

```bash
# CloudXR Runtime 상태
docker ps | grep cloudxr
# 기대: ust-cloudxr-runtime ... Up (healthy)

# 포트 리스닝
ss -tlnp | grep 48010    # TCP 시그널링
ss -ulnp | grep 47998    # UDP 미디어

# 서버 IP 확인 (Quest에 입력할 주소)
hostname -I | awk '{print $1}'
```

### Step 3: Isaac Lab 환경 확인

```bash
# Isaac Lab 컨테이너 진입
./docker/container.py enter base

# 환경변수 확인
echo $XR_RUNTIME_JSON     # /openxr/share/openxr/1/openxr_cloudxr.json
echo $XDG_RUNTIME_DIR     # /openxr/run

# /openxr 마운트 확인
ls /openxr/               # lib/ run/ share/ 보여야 함
```

### Step 4: Quest 3S 연결

1. Quest 3S를 **5GHz WiFi**에 연결 (서버와 같은 네트워크)
2. CloudXR 클라이언트 앱 실행
3. 서버 IP 입력 (Step 2에서 확인한 IP)
4. Connect

> **주의**: CloudXR SDK 5.0.1+ 부터는 네이티브 APK 대신 **CloudXR.js 웹 클라이언트** 사용 권장
> 자세한 내용: `6. CLOUDXR_JS_SETUP_GUIDE.md` 참조

---

## 7. 트러블슈팅

### VR 연결 안됨

| 증상 | 원인 | 해결 |
|------|------|------|
| `XR_ERROR_RUNTIME_UNAVAILABLE` | CloudXR Runtime 미실행 또는 `/openxr` 미마운트 | Docker Compose로 재시작 |
| Quest 앱이 바로 종료 | 서버 미실행 또는 IP 오류 | `docker ps`로 확인, IP 재확인 |
| 연결 후 화면 끊김 | 네트워크 대역폭 부족 | 5GHz WiFi 6 사용 |
| SDK 버전 불일치 | CloudXR 4.x APK로 5.x 서버 연결 | CloudXR.js 웹 클라이언트 사용 |

### 시뮬레이션 관련

| 증상 | 원인 | 해결 |
|------|------|------|
| USD 로딩 실패 | 경로 오류 | `UST_USD_PATH` 확인 |
| 로봇 떨림 | 액추에이터 파라미터 불일치 | `ALL_DEV_ARM_PARAMS`로 전환 테스트 |
| IK 실패 | 타겟이 작업 공간 밖 | `is_in_workspace()` 확인 |
| GPU 메모리 부족 | 환경 수 과다 | `num_envs` 줄이기 |

### 학습 관련

| 증상 | 원인 | 해결 |
|------|------|------|
| `ImportError: robomimic` | Robomimic 미설치 | `pip install robomimic` |
| 데이터셋 미발견 | 경로 오류 | `--dataset` 경로 확인 |
| 학습 수렴 안됨 | 데모 수 부족 | 최소 20개 이상 데모 수집 |
| 관측 차원 불일치 | config 변경 후 미재수집 | 데이터 재수집 또는 config 복원 |

---

## 프로젝트 디렉토리 구조 (최종)

```
ust_ws/260207_ust/
├── config/
│   ├── __init__.py                      # 패키지 export (업데이트됨)
│   ├── ust_mobile_manipulator_cfg.py     # 로봇 구성 (ARM_PARAMS 프리셋 추가)
│   ├── ust_scene_cfg.py                  # 씬 구성
│   ├── ust_actions_cfg.py                # 액션 공간 (IK_METHOD 추가)
│   ├── ust_observations_cfg.py           # 관측 공간
│   ├── ust_teleop_env_cfg.py             # 환경 설정
│   ├── ust_teleop_device_cfg.py          # VR 디바이스 (프리셋 + retargeter 옵션 추가)
│   ├── lula_ik_cfg.py                    # ★ 새 파일: Lula IK 래퍼
│   ├── open_x1_des.yaml                  # ★ 복사: Lula robot descriptor
│   └── open_manipulator_x.urdf           # ★ 복사: OpenMANIPULATOR-X URDF
│
├── controllers/
│   ├── __init__.py
│   └── differential_drive_controller.py   # 차동 구동 컨트롤러
│
├── utils/
│   ├── __init__.py                        # 패키지 export (업데이트됨)
│   ├── hdf5_recorder.py                   # HDF5 데이터 녹화
│   └── physics_setup.py                   # ★ 새 파일: 물리 속성 유틸리티
│
├── scripts/
│   ├── run_teleop.py                      # VR 텔레오퍼레이션 메인
│   ├── record_demos.py                    # 데모 데이터 녹화
│   ├── run_ros2_bridge.py                 # ROS2 브릿지
│   └── train_policy.py                    # ★ 새 파일: 정책 학습
│
├── assets/                                # USD 모델 파일 (비어 있음)
├── checkpoints/                           # 학습된 모델 (비어 있음)
├── datasets/                              # 수집된 데이터셋 (비어 있음)
├── logs/                                  # 시뮬레이션 로그 (비어 있음)
│
├── setup_cloudxr_env.sh                   # CloudXR 환경 설정
├── start_cloudxr_runtime.sh               # CloudXR Docker 실행
├── docker-compose.cloudxr-ust.patch.yaml  # Docker Compose 설정
│
├── IMPLEMENTATION_AND_EXECUTION_GUIDE.md  # ★ 이 문서
├── 1. EXECUTION_GUIDE.md                  # 기존 실행 가이드
├── 2. VR_TELEOP_SETUP_GUIDE.md            # VR 설정 가이드
├── 3. VR_CONNECTION_TROUBLESHOOTING_GUIDE.md
├── 4. VR_ISSUE_RESOLUTION_GUIDE.md
├── 5. QUEST3S_CLOUDXR_CRASH_ANALYSIS.md
├── 6. CLOUDXR_JS_SETUP_GUIDE.md
└── IMITATION_LEARNING_GUIDE.md
```

---

## 빠른 시작 (Quick Start)

### 최소한의 테스트 (VR 없이)

```bash
cd /workspace/isaaclab

# 키보드 텔레오퍼레이션으로 빠른 테스트
./isaaclab.sh -p ust_ws/260207_ust/scripts/run_teleop.py \
    --teleop_device keyboard
```

### 전체 파이프라인 (VR → 데모 수집 → 학습)

```bash
# 1. CloudXR + Isaac Lab 시작
./docker/container.py start \
    --files ust_ws/260207_ust/docker-compose.cloudxr-ust.patch.yaml \
    --env-file docker/.env.cloudxr-runtime

./docker/container.py enter base

# 2. VR 데모 수집 (20개)
./isaaclab.sh -p ust_ws/260207_ust/scripts/record_demos.py \
    --num_demos 20 \
    --include_images

# 3. 정책 학습
./isaaclab.sh -p ust_ws/260207_ust/scripts/train_policy.py \
    --algo bc_rnn \
    --dataset ./datasets/ust_manipulation_*.hdf5 \
    --epochs 2000
```

---

*이 문서는 `all_dev_gap_analysis_and_implementation_guide.md`의 Phase 1~5 구현 완료 후 작성된 실행 가이드입니다.*
