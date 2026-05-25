# Isaac Lab 실행 가이드

**환경**: nvcr.io/nvidia/isaac-lab:2.3.0 Docker 컨테이너
**GPU**: NVIDIA RTX PRO 6000 Blackwell Workstation Edition (96GB VRAM)
**Isaac Lab 경로**: `/workspace/isaaclab/`
**작성일**: 2026-02-07

---

## 목차

1. [기본 실행 방법](#1-기본-실행-방법)
2. [데모 스크립트 실행](#2-데모-스크립트-실행)
3. [환경 테스트](#3-환경-테스트)
4. [VR 텔레오퍼레이션](#4-vr-텔레오퍼레이션-cloudxr--meta-quest-3s)
5. [강화 학습](#5-강화-학습-훈련)
6. [모방 학습](#6-모방-학습)
7. [ROS2 통합](#7-ros2-통합)
8. [유용한 명령어 옵션](#8-유용한-명령어-옵션)
9. [튜토리얼](#9-튜토리얼)
10. [트러블슈팅](#10-트러블슈팅)

---

## 1. 기본 실행 방법

### 1.1 isaaclab.sh 래퍼 스크립트

모든 Isaac Lab 스크립트는 `isaaclab.sh` 래퍼를 통해 실행합니다. 이 스크립트가 필요한 환경 설정과 Python 경로를 자동으로 구성합니다.

```bash
cd /workspace/isaaclab

# 기본 실행 형식
./isaaclab.sh -p <스크립트_경로> [옵션들]
```

### 1.2 주요 플래그

| 플래그 | 설명 |
|--------|------|
| `-p <script>` | Python 스크립트 실행 |
| `-i <package>` | 추가 패키지 설치 (예: robomimic, rsl_rl) |
| `-h` | 도움말 표시 |
| `-v` | 버전 정보 표시 |

### 1.3 GPU 및 환경 확인

```bash
# GPU 상태 확인
nvidia-smi

# 예상 출력:
# NVIDIA RTX PRO 6000 Blackwell Workstation Edition, 97887 MiB

# Isaac Lab 버전 확인
./isaaclab.sh -p -c "import isaaclab; print(isaaclab.__version__)"
```

---

## 2. 데모 스크립트 실행

### 2.1 사용 가능한 데모 목록

```
scripts/demos/
├── arms.py              # 로봇 암 데모
├── bipeds.py            # 이족 로봇 (휴머노이드) 데모
├── deformables.py       # 변형 가능 객체 데모
├── h1_locomotion.py     # H1 휴머노이드 보행
├── hands.py             # 로봇 핸드 데모
├── markers.py           # 마커 시각화 데모
├── multi_asset.py       # 다중 에셋 데모
├── pick_and_place.py    # 픽앤플레이스 데모
├── procedural_terrain.py # 절차적 지형 생성
├── quadcopter.py        # 쿼드콥터 드론 데모
├── quadrupeds.py        # 4족 로봇 데모
└── sensors/             # 센서 관련 데모
```

### 2.2 데모 실행 예시

```bash
# 로봇 암 (Franka, UR10 등) 시각화
./isaaclab.sh -p scripts/demos/arms.py

# 4족 로봇 (ANYmal, Spot 등) 시각화
./isaaclab.sh -p scripts/demos/quadrupeds.py

# 이족 보행 로봇 (휴머노이드) 시각화
./isaaclab.sh -p scripts/demos/bipeds.py

# H1 휴머노이드 보행 데모
./isaaclab.sh -p scripts/demos/h1_locomotion.py

# Pick and Place 작업 데모
./isaaclab.sh -p scripts/demos/pick_and_place.py

# 쿼드콥터 비행 데모
./isaaclab.sh -p scripts/demos/quadcopter.py

# 변형 가능 객체 (천, 로프 등) 데모
./isaaclab.sh -p scripts/demos/deformables.py

# 절차적 지형 생성 데모
./isaaclab.sh -p scripts/demos/procedural_terrain.py
```

---

## 3. 환경 테스트

### 3.1 사용 가능한 환경 목록 확인

```bash
./isaaclab.sh -p scripts/environments/list_envs.py
```

이 명령은 등록된 모든 Isaac Lab 환경을 출력합니다.

### 3.2 환경 스크립트 구조

```
scripts/environments/
├── list_envs.py           # 환경 목록 출력
├── random_agent.py        # 랜덤 액션 에이전트
├── zero_agent.py          # 제로 액션 에이전트
├── export_IODescriptors.py # I/O 디스크립터 내보내기
├── state_machine/         # 상태 머신 기반 환경
└── teleoperation/         # 텔레오퍼레이션 환경
```

### 3.3 랜덤 에이전트로 환경 테스트

랜덤 액션을 수행하며 환경이 정상 동작하는지 확인합니다:

```bash
# Franka 로봇 큐브 들기 환경
./isaaclab.sh -p scripts/environments/random_agent.py \
    --task Isaac-Lift-Cube-Franka-v0

# Franka 로봇 큐브 쌓기 환경 (IK 상대 좌표)
./isaaclab.sh -p scripts/environments/random_agent.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-v0

# ANYmal C 보행 환경
./isaaclab.sh -p scripts/environments/random_agent.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0

# 휴머노이드 보행 환경
./isaaclab.sh -p scripts/environments/random_agent.py \
    --task Isaac-Velocity-Flat-H1-v0
```

### 3.4 제로 에이전트로 환경 테스트

액션 없이 환경 시뮬레이션만 실행합니다:

```bash
./isaaclab.sh -p scripts/environments/zero_agent.py \
    --task Isaac-Lift-Cube-Franka-v0
```

### 3.5 병렬 환경 수 조정

96GB VRAM을 활용하여 대규모 병렬 환경 실행:

```bash
# 4096개 병렬 환경
./isaaclab.sh -p scripts/environments/random_agent.py \
    --task Isaac-Lift-Cube-Franka-v0 \
    --num_envs 4096

# 8192개 병렬 환경 (96GB VRAM 활용)
./isaaclab.sh -p scripts/environments/random_agent.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 8192
```

---

## 4. VR 텔레오퍼레이션 (CloudXR + Meta Quest 3S)

### 4.1 사전 조건

- CloudXR Runtime 5.0.1 설치됨
- Meta Quest 3S에 CloudXR Client 앱 설치됨
- 동일 네트워크에 연결 (5GHz WiFi 권장)

### 4.2 환경 변수 설정

```bash
# CloudXR OpenXR 런타임 설정
export XDG_RUNTIME_DIR=$(pwd)/openxr/run
export XR_RUNTIME_JSON=$(pwd)/openxr/share/openxr/1/openxr_cloudxr.json

# GPU 인덱스 설정
export NV_GPU_INDEX=0

# 고정 타임스텝 설정 (VR 안정성)
export NV_PACER_FIXED_TIME_STEP_MS=11
```

### 4.3 텔레오퍼레이션 스크립트 실행

```bash
# 핸드 트래킹 텔레오퍼레이션 (Franka 로봇)
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-Stack-Cube-Franka-IK-Abs-v0 \
    --teleop_device handtracking

# 스페이스마우스 텔레오퍼레이션
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-Stack-Cube-Franka-IK-Abs-v0 \
    --teleop_device spacemouse

# 키보드 텔레오퍼레이션
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-Stack-Cube-Franka-IK-Abs-v0 \
    --teleop_device keyboard
```

### 4.4 Meta Quest 3S 연결 방법

1. Quest 3S에서 **Settings** → **Unknown Sources** 활성화
2. **CloudXR Client** 앱 실행
3. 서버 IP 자동 연결 확인 (또는 수동 입력)
4. 시뮬레이션 화면이 VR 헤드셋에 스트리밍됨
5. 핸드 트래킹으로 로봇 제어 테스트

### 4.5 네트워크 포트 확인

```bash
# 방화벽 포트 개방 (필요시)
sudo ufw allow 47998:48000,48005,48008,48012/udp
sudo ufw allow 48010/tcp

# 연결 테스트
nc -vz <quest-ip> 48010
```

---

## 5. 강화 학습 훈련

### 5.1 지원 라이브러리

Isaac Lab은 여러 강화 학습 라이브러리를 지원합니다:

- **RSL-RL**: ETH Zurich의 로봇 학습 라이브러리
- **rl_games**: NVIDIA의 고속 강화 학습 라이브러리
- **Stable Baselines3**: 널리 사용되는 RL 라이브러리
- **SKRL**: 모듈식 강화 학습 라이브러리

### 5.2 RSL-RL로 훈련

```bash
# PPO 알고리즘으로 Franka 로봇 훈련
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Lift-Cube-Franka-v0 \
    --num_envs 4096

# ANYmal C 보행 훈련
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 4096

# H1 휴머노이드 보행 훈련
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-H1-v0 \
    --num_envs 4096
```

### 5.3 rl_games로 훈련

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py \
    --task Isaac-Ant-v0 \
    --num_envs 4096
```

### 5.4 훈련된 정책 평가

```bash
# 체크포인트에서 정책 로드 및 평가
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Lift-Cube-Franka-v0 \
    --checkpoint <체크포인트_경로>
```

### 5.5 96GB VRAM 최적화 설정

```bash
# 대규모 배치로 훈련 (96GB VRAM 활용)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Anymal-C-v0 \
    --num_envs 8192 \
    --max_iterations 10000
```

---

## 6. 모방 학습

### 6.1 Robomimic 설치

```bash
# 의존성 설치
sudo apt install cmake build-essential

# Robomimic 설치
./isaaclab.sh -i robomimic
```

### 6.2 데이터 수집

VR 텔레오퍼레이션으로 데이터를 수집합니다:

```bash
./isaaclab.sh -p scripts/imitation_learning/robomimic/record_demos.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --teleop_device handtracking \
    --dataset_file ./datasets/my_demos.hdf5 \
    --num_demos 20
```

### 6.3 Isaac Lab Mimic으로 데이터 증강

수집된 데이터를 증강하여 더 많은 훈련 데이터를 생성합니다:

```bash
# 서브태스크 어노테이션
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
    --input_file ./datasets/my_demos.hdf5 \
    --output_file ./datasets/annotated_demos.hdf5 \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --auto

# 데이터 증강 (10개 → 1000개)
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --input_file ./datasets/annotated_demos.hdf5 \
    --output_file ./datasets/mimic_1k.hdf5 \
    --num_envs 10 \
    --generation_num_trials 100
```

### 6.4 SkillGen + cuRobo 데이터 증강 (GPU 가속)

```bash
./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --input_file ./datasets/annotated_demos.hdf5 \
    --output_file ./datasets/skillgen_1k.hdf5 \
    --num_envs 10 \
    --generation_num_trials 100 \
    --use_skillgen
```

### 6.5 Robomimic 정책 학습

```bash
# BC (Behavioral Cloning) 학습
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --algo bc \
    --dataset ./datasets/mimic_1k.hdf5

# BC-RNN 학습
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --algo bc_rnn \
    --dataset ./datasets/mimic_1k.hdf5
```

### 6.6 학습된 정책 평가

```bash
./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 \
    --checkpoint ./logs/robomimic/best_model.pth
```

---

## 7. ROS2 통합

### 7.1 ROS2 Bridge 활성화

```bash
./isaaclab.sh -p <스크립트> --enable isaacsim.ros2.bridge
```

### 7.2 ROS2 환경 설정

```bash
# ROS2 환경 소싱
source /opt/ros/humble/setup.bash

# Isaac Lab 스크립트 실행 (ROS2 Bridge 활성화)
./isaaclab.sh -p scripts/environments/random_agent.py \
    --task Isaac-Lift-Cube-Franka-v0 \
    --enable isaacsim.ros2.bridge
```

### 7.3 ROS2 토픽 확인

별도 터미널에서:

```bash
source /opt/ros/humble/setup.bash

# 발행중인 토픽 확인
ros2 topic list

# 조인트 상태 확인
ros2 topic echo /joint_states

# TF 트리 확인
ros2 run tf2_tools view_frames
```

### 7.4 MoveIt 2 연동

```bash
# MoveIt 2와 함께 시뮬레이션 실행
ros2 launch mobile_manipulator_moveit move_group.launch.py
```

---

## 8. 유용한 명령어 옵션

### 8.1 공통 옵션

| 옵션 | 설명 | 예시 |
|------|------|------|
| `--task <name>` | 실행할 태스크/환경 이름 | `--task Isaac-Lift-Cube-Franka-v0` |
| `--num_envs <N>` | 병렬 환경 수 | `--num_envs 4096` |
| `--headless` | GUI 없이 실행 (서버용) | `--headless` |
| `--enable <ext>` | 확장 기능 활성화 | `--enable isaacsim.ros2.bridge` |
| `--device <dev>` | 실행 디바이스 | `--device cpu` 또는 `--device cuda:0` |

### 8.2 시뮬레이션 옵션

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--physics_dt` | 물리 시뮬레이션 타임스텝 | 0.005 (200Hz) |
| `--rendering_dt` | 렌더링 타임스텝 | 0.02 (50Hz) |
| `--decimation` | 제어 주기 배수 | 4 |

### 8.3 훈련 옵션

| 옵션 | 설명 |
|------|------|
| `--max_iterations` | 최대 훈련 반복 수 |
| `--checkpoint` | 체크포인트 파일 경로 |
| `--resume` | 이전 훈련 재개 |
| `--experiment_name` | 실험 이름 |

### 8.4 텔레오퍼레이션 옵션

| 옵션 | 설명 |
|------|------|
| `--teleop_device` | 텔레오퍼레이션 디바이스 (handtracking, spacemouse, keyboard) |
| `--dataset_file` | 데이터셋 저장 경로 |
| `--num_demos` | 수집할 데모 수 |

---

## 9. 튜토리얼

### 9.1 튜토리얼 스크립트 구조

```
scripts/tutorials/
├── 00_sim/           # 시뮬레이션 기본
├── 01_assets/        # 에셋 로딩
├── 02_scene/         # 씬 구성
├── 03_envs/          # 환경 설정
├── 04_sensors/       # 센서 설정
├── 05_controllers/   # 컨트롤러 설정
└── ...
```

### 9.2 기본 시뮬레이션 튜토리얼

```bash
# 프리미티브 생성
./isaaclab.sh -p scripts/tutorials/00_sim/spawn_prims.py

# 시뮬레이션 루프
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

### 9.3 에셋 로딩 튜토리얼

```bash
# USD 에셋 로딩
./isaaclab.sh -p scripts/tutorials/01_assets/run_articulation.py

# 강체 에셋
./isaaclab.sh -p scripts/tutorials/01_assets/run_rigid_object.py
```

### 9.4 센서 튜토리얼

```bash
# 카메라 센서
./isaaclab.sh -p scripts/tutorials/04_sensors/run_camera.py

# 라이다 센서
./isaaclab.sh -p scripts/tutorials/04_sensors/run_ray_caster.py
```

---

## 10. 트러블슈팅

### 10.1 일반적인 문제

| 증상 | 원인 | 해결 방법 |
|------|------|----------|
| `ModuleNotFoundError: isaaclab` | 환경 미설정 | `./isaaclab.sh -p` 사용 |
| GPU 메모리 부족 | num_envs 과다 | `--num_envs` 줄이기 |
| 시뮬레이션 느림 | headless 미사용 | `--headless` 옵션 추가 |
| 물리 불안정 | 타임스텝 과대 | `physics_dt` 줄이기 |

### 10.2 CloudXR 연결 문제

| 증상 | 원인 | 해결 방법 |
|------|------|----------|
| 검은 화면 | GPU 인덱스 불일치 | `export NV_GPU_INDEX=0` |
| 연결 실패 | 포트 차단 | 방화벽 포트 개방 |
| 높은 지연 | 무선 네트워크 | 5GHz WiFi 사용 |
| 핸드 트래킹 불안정 | 조명 부족 | 조명 개선 |

### 10.3 ROS2 통합 문제

| 증상 | 원인 | 해결 방법 |
|------|------|----------|
| 토픽 안보임 | Bridge 미활성화 | `--enable isaacsim.ros2.bridge` |
| TF 오류 | 시간 동기화 | Clock 토픽 확인 |
| 메시지 손실 | QoS 불일치 | QoS 설정 조정 |

### 10.4 로그 및 디버깅

```bash
# 상세 로그 출력
./isaaclab.sh -p <script> --verbose

# Python 디버거 사용
./isaaclab.sh -p -m pdb <script>

# 프로파일링
./isaaclab.sh -p <script> --profile
```

---

## 빠른 참조 카드

```bash
# 환경 목록 확인
./isaaclab.sh -p scripts/environments/list_envs.py

# 간단한 데모 실행
./isaaclab.sh -p scripts/demos/arms.py

# 환경 테스트
./isaaclab.sh -p scripts/environments/random_agent.py --task Isaac-Lift-Cube-Franka-v0

# VR 텔레오퍼레이션
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-Stack-Cube-Franka-IK-Abs-v0 --teleop_device handtracking

# 강화 학습 훈련
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Lift-Cube-Franka-v0 --num_envs 4096

# 모방 학습
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --dataset ./datasets/demo.hdf5

# ROS2 통합
./isaaclab.sh -p <script> --enable isaacsim.ros2.bridge
```

---

*본 가이드는 Isaac Lab 2.3.0, RTX PRO 6000 Blackwell (96GB VRAM) 환경 기준입니다.*
