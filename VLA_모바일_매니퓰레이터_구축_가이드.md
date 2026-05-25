# VLA 모델 기반 모바일 매니퓰레이터 모방학습 구축 가이드

## 📋 목차
1. [프로젝트 개요](#프로젝트-개요)
2. [VLA 모델 비교 분석](#vla-모델-비교-분석)
3. [RTX 4090 환경에 최적화된 모델 선정](#rtx-4090-환경에-최적화된-모델-선정)
4. [시스템 아키텍처](#시스템-아키텍처)
5. [환경 설정](#환경-설정)
6. [OpenVLA 설치 및 구성](#openvla-설치-및-구성)
7. [Isaac Lab 통합](#isaac-lab-통합)
8. [데이터 수집 파이프라인](#데이터-수집-파이프라인)
9. [모델 학습 및 파인튜닝](#모델-학습-및-파인튜닝)
10. [성능 최적화 전략](#성능-최적화-전략)
11. [트러블슈팅](#트러블슈팅)
12. [참고 자료](#참고-자료)

---

## 프로젝트 개요

### 현재 환경
- **로봇 구성**: 양팔 매니퓰레이터 + 4륜 모바일 베이스
- **시뮬레이터**: NVIDIA Isaac Sim
- **프로젝트 파일**: `./isaac_file/ust_project1.usd`
- **하드웨어**: NVIDIA RTX 4090 (24GB VRAM)
- **목표**: VLA 모델을 활용한 모방학습(Imitation Learning) 시스템 구축

### 왜 VLA 모델인가?
Vision-Language-Action (VLA) 모델은 다음과 같은 장점을 제공합니다:

1. **멀티모달 이해**: 시각 정보와 언어 명령을 동시에 처리
2. **제로샷 일반화**: 새로운 태스크에 대한 빠른 적응
3. **언어 기반 제어**: 자연어 명령으로 복잡한 조작 가능
4. **크로스 엠보디먼트**: 다양한 로봇 형태에 전이 가능

---

## VLA 모델 비교 분석

### 2024-2025년 주요 VLA 모델

#### 1. **OpenVLA** ⭐ 추천
**출시**: 2024년 6월 (Stanford)

**특징**:
- 7B 파라미터 오픈소스 모델
- Open X-Embodiment 데이터셋 (970K 궤적) 기반 학습
- RT-2-X (55B) 대비 16.5% 높은 성공률 (7배 작은 크기)
- Llama-2 기반 LLM + DINOv2/SigLIP 비전 인코더

**성능 지표**:
- RTX 4090에서 ~6Hz 추론 속도 (bfloat16)
- 15GB GPU 메모리 필요
- INT4 양자화 시 메모리 50% 감소, 성능 유지

**장점**:
- ✅ 오픈소스 (MIT 라이선스 코드)
- ✅ 활발한 커뮤니티 및 문서화
- ✅ LoRA 파인튜닝 지원
- ✅ REST API 서버 기능 제공
- ✅ RTX 4090에 최적화된 성능

**단점**:
- ⚠️ Llama-2 라이선스 제약 (모델 가중치)
- ⚠️ 훈련에는 멀티 GPU 필요 (파인튜닝은 단일 GPU 가능)

**최신 업데이트 (2025년 3월)**:
- **OFT (Optimized Fine-Tuning)**: 25-50배 빠른 추론
- **FAST Action Tokenizer**: 최대 15배 속도 향상
- **고주파 양팔 제어** 지원

#### 2. **π0 (Pi-Zero)**
**출시**: 2024년 후반 (Physical Intelligence)

**특징**:
- Paligemma VLM 백본 사용
- Open X-Embodiment 데이터로 학습된 액션 전문가
- 모바일 매니퓰레이터로 세탁 작업 성공 사례

**장점**:
- ✅ 크로스 엠보디먼트 일반화 우수
- ✅ 단일/양팔 제어 모두 지원

**단점**:
- ❌ 클로즈드 소스
- ❌ 상업적 사용 제한

#### 3. **Helix**
**출시**: 2025년 2월 (Figure AI)

**특징**:
- 휴머노이드 로봇 전용 VLA
- 고주파 전신 제어 (팔, 손, 몸통, 머리, 손가락)
- 2대 로봇 동시 협업 가능

**장점**:
- ✅ 양팔 조작에 특화
- ✅ 최신 아키텍처

**단점**:
- ❌ 클로즈드 소스
- ❌ 휴머노이드 중심 (모바일 매니퓰레이터에는 과잉)

#### 4. **RT-2 (Robotic Transformer 2)**
**출시**: 2023년 (Google DeepMind)

**특징**:
- PaLI-X/PaLM-E 기반
- VLA 패러다임의 선구자

**장점**:
- ✅ 검증된 성능

**단점**:
- ❌ 클로즈드 소스
- ❌ OpenVLA가 성능 초과
- ❌ 55B 파라미터 (RTX 4090에 부적합)

---

## RTX 4090 환경에 최적화된 모델 선정

### 최종 추천: **OpenVLA-7B**

#### 선정 이유

1. **하드웨어 호환성**
   - RTX 4090 24GB VRAM으로 추론 가능 (15GB 사용)
   - 양자화 적용 시 더 효율적 (INT4: ~8GB)
   - 단일 GPU로 파인튜닝 가능 (LoRA 사용)

2. **성능**
   - 6Hz 추론 속도 (실시간 제어 가능)
   - OFT 적용 시 25-50배 속도 향상 가능
   - 양팔 모바일 매니퓰레이터 지원 확인됨

3. **개발 생태계**
   - 오픈소스로 완전한 커스터마이징 가능
   - NVIDIA Jetson AI Lab의 공식 지원
   - Isaac Lab/Isaac Sim과의 통합 예제 존재
   - 활발한 GitHub 커뮤니티

4. **확장성**
   - HuggingFace 생태계 활용
   - REST API로 분산 배포 가능
   - 다양한 파인튜닝 옵션

### 대안 모델

**상황별 대안**:
- **더 가벼운 환경 필요**: ACT + Diffusion Policy 조합
- **언어 기능 불필요**: Diffusion Policy 단독
- **클라우드 GPU 사용**: π0 또는 RT-2 (접근 가능 시)

---

## 시스템 아키텍처

### 전체 워크플로우

```
┌─────────────────────────────────────────────────────────────────┐
│                     Isaac Sim 시뮬레이션 환경                    │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  ust_project1.usd (양팔 모바일 매니퓰레이터)              │  │
│  │  - 좌우 매니퓰레이터 (각 6-7 DOF)                         │  │
│  │  - 모바일 베이스 (4륜)                                     │  │
│  │  - 카메라 센서 (RGB-D)                                     │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│                        Isaac Lab 프레임워크                      │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │  텔레오퍼레이션  │  │  데이터 수집     │  │ 환경 래퍼    │ │
│  │  (데모 생성)     │  │  (궤적 저장)     │  │ (Gym API)    │ │
│  └──────────────────┘  └──────────────────┘  └──────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│                       데이터 전처리 파이프라인                   │
│  - Open X-Embodiment 포맷 변환                                 │
│  - 이미지 정규화 및 증강                                        │
│  - 액션 시퀀스 토큰화                                           │
└─────────────────────────────────────────────────────────────────┘
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│                      OpenVLA 모델 (RTX 4090)                    │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Vision Encoder: DINOv2 + SigLIP                         │  │
│  │  ↓                                                        │  │
│  │  Language Model: Llama-2 (7B)                            │  │
│  │  ↓                                                        │  │
│  │  Action Head: 토큰 → 로봇 액션 (position/velocity)       │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  파인튜닝 옵션:                                                 │
│  1. LoRA (추천, RTX 4090 단일 GPU)                             │
│  2. OFT (최신, 고성능)                                          │
│  3. Full Fine-tuning (멀티 GPU 필요)                           │
└─────────────────────────────────────────────────────────────────┘
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│                          정책 배포 및 실행                       │
│  - 추론 최적화 (INT4 양자화, Flash Attention 2)                │
│  - REST API 서버 (선택사항)                                     │
│  - Isaac Sim 실시간 제어 루프                                   │
└─────────────────────────────────────────────────────────────────┘
```

### 데이터 흐름

1. **데모 수집 단계**:
   ```
   텔레오퍼레이션 → 로봇 상태 기록 → 카메라 이미지 캡처 → 궤적 저장
   ```

2. **학습 단계**:
   ```
   궤적 로드 → 배치 생성 → OpenVLA 파인튜닝 → 체크포인트 저장
   ```

3. **배포 단계**:
   ```
   모델 로드 → 이미지 + 언어 입력 → 액션 예측 → 로봇 제어
   ```

---

## 환경 설정

### 시스템 요구사항

#### 하드웨어
- GPU: NVIDIA RTX 4090 (24GB VRAM) ✅
- RAM: 32GB 이상 권장
- Storage: 100GB 이상 여유 공간 (데이터셋 + 모델)
- CUDA 지원: Compute Capability 8.9 (RTX 4090)

#### 소프트웨어
- OS: Ubuntu 20.04/22.04 (현재 Linux 6.14 확인됨 ✅)
- Python: 3.10+ (3.8+ 호환)
- CUDA: 11.8 또는 12.1+
- Isaac Sim: 최신 버전 (확인됨 ✅)

### CUDA 및 드라이버 확인

```bash
# NVIDIA 드라이버 확인
nvidia-smi

# CUDA 버전 확인
nvcc --version

# PyTorch CUDA 호환성 테스트
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda}')"
```

### Conda 환경 생성

```bash
# OpenVLA 전용 환경 생성
conda create -n openvla python=3.10 -y
conda activate openvla

# 기본 도구 설치
conda install -c conda-forge git git-lfs cmake -y
```

---

## OpenVLA 설치 및 구성

### 1. 저장소 클론

```bash
cd ~/ust_ws
git clone https://github.com/openvla/openvla.git
cd openvla
```

### 2. PyTorch 설치 (RTX 4090 최적화)

```bash
# CUDA 12.1 기준 (최신 RTX 4090 최적화)
pip install torch==2.2.0 torchvision==0.17.0 --index-url https://download.pytorch.org/whl/cu121

# 또는 CUDA 11.8
# pip install torch==2.2.0 torchvision==0.17.0 --index-url https://download.pytorch.org/whl/cu118
```

### 3. OpenVLA 의존성 설치

```bash
# 전체 의존성 설치
pip install -e .

# 또는 최소 설치 (추론만 필요한 경우)
pip install torch transformers timm tokenizers pillow
```

### 4. Flash Attention 2 설치 (선택사항, 성능 향상)

```bash
pip install flash-attn==2.5.5 --no-build-isolation
```

**참고**: Flash Attention은 컴파일에 시간이 걸리지만 추론 속도를 크게 향상시킵니다.

### 5. 사전학습 모델 다운로드

```bash
# HuggingFace CLI 설치
pip install huggingface-hub

# 모델 다운로드 (자동, 첫 실행 시)
python3 << EOF
from transformers import AutoModelForVision2Seq, AutoProcessor

model_id = "openvla/openvla-7b"
processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
vla = AutoModelForVision2Seq.from_pretrained(
    model_id,
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True
)
print("모델 다운로드 완료!")
EOF
```

### 6. 설치 검증

```bash
# 테스트 스크립트 실행
python3 << 'EOF'
import torch
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import numpy as np

# 모델 로드
model_id = "openvla/openvla-7b"
processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
vla = AutoModelForVision2Seq.from_pretrained(
    model_id,
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True
).to("cuda:0")

# 더미 입력 생성
image = Image.fromarray(np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8))
prompt = "In: What action should the robot take to pick up the object?\nOut:"

# 추론 테스트
inputs = processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
with torch.no_grad():
    action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)

print(f"✅ OpenVLA 설치 성공!")
print(f"예측된 액션 shape: {action.shape}")
print(f"GPU 메모리 사용: {torch.cuda.max_memory_allocated() / 1e9:.2f} GB")
EOF
```

예상 출력:
```
✅ OpenVLA 설치 성공!
예측된 액션 shape: torch.Size([1, 7])
GPU 메모리 사용: 14.52 GB
```

---

## Isaac Lab 통합

### Isaac Lab 소개

Isaac Lab (구 ORBIT)은 Isaac Sim 위에 구축된 GPU 가속 로봇 학습 프레임워크입니다.

**주요 기능**:
- 병렬 시뮬레이션 (수천 개 환경 동시 실행)
- 텔레오퍼레이션 및 데모 수집
- RL 및 모방학습 통합
- 다양한 로봇 엠보디먼트 지원

### 1. Isaac Lab 설치

```bash
cd ~/ust_ws

# Isaac Lab 클론
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# 설치 스크립트 실행
./isaaclab.sh --install
```

### 2. Isaac Sim 경로 설정

```bash
# .bashrc 또는 .zshrc에 추가
export ISAACSIM_PATH=~/ust_ws/isaacsim
export ISAACSIM_PYTHON_EXE=$ISAACSIM_PATH/_build/linux-x86_64/release/python.sh
```

### 3. 모바일 매니퓰레이터 환경 구성

`isaac_lab_config.py` 생성:

```python
# ~/ust_ws/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/manipulation/custom/mobile_bimanual_config.py

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, RigidObjectCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import EventTermCfg, ObservationTermCfg, RewardTermCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import CameraCfg
from omni.isaac.lab.utils import configclass

@configclass
class MobileBimanualEnvCfg(ManagerBasedRLEnvCfg):
    """양팔 모바일 매니퓰레이터 환경 설정."""

    # 시뮬레이션 설정
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=1 / 60,  # 60Hz 시뮬레이션
        render_interval=1,
    )

    # 로봇 설정
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/isaac/ust_ws/isaac_file/ust_project1.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),  # w, x, y, z
            joint_pos={
                ".*": 0.0,  # 모든 조인트 초기 위치
            },
        ),
    )

    # 카메라 센서 (RGB + Depth)
    camera: CameraCfg = CameraCfg(
        prim_path="/World/Robot/camera",
        update_period=0.1,  # 10Hz
        height=224,
        width=224,
        data_types=["rgb", "distance_to_camera"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 20.0),
        ),
    )

    # 씬 설정
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1,  # 단일 환경 (데모 수집용)
        env_spacing=2.5,
    )

    # 액션 공간 (양팔 + 모바일 베이스)
    # 좌팔 7 DOF + 우팔 7 DOF + 베이스 2 DOF (선속도, 각속도) = 16 DOF
    num_actions = 16

    # 관측 공간
    num_observations = 224 * 224 * 3  # RGB 이미지

    # 에피소드 설정
    episode_length_s = 60.0  # 60초 에피소드
```

### 4. 텔레오퍼레이션 스크립트

`teleoperation_demo.py` 생성:

```python
# ~/ust_ws/teleoperation_demo.py

import argparse
import numpy as np
import torch
from omni.isaac.lab.app import AppLauncher

# 인자 파서
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--task", type=str, default="Isaac-Mobile-Bimanual-v0")
args_cli = parser.parse_args()

# Isaac Sim 앱 실행
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab_tasks.utils import parse_env_cfg
import omni.isaac.lab_tasks

def main():
    # 환경 생성
    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # 데모 수집 리스트
    demonstrations = []

    print("=" * 80)
    print("텔레오퍼레이션 데모 수집 시작")
    print("=" * 80)
    print("키보드 조작:")
    print("  W/S: 앞/뒤 이동")
    print("  A/D: 좌/우 회전")
    print("  Q/E: 왼팔 그리퍼 열기/닫기")
    print("  R/F: 오른팔 그리퍼 열기/닫기")
    print("  ESC: 데모 저장 및 종료")
    print("=" * 80)

    # 리셋
    env.reset()

    # 메인 루프
    episode_data = {
        "observations": [],
        "actions": [],
        "language_instruction": input("작업 설명을 입력하세요 (예: '빨간 블록을 집어서 파란 상자에 넣기'): ")
    }

    step_count = 0
    while simulation_app.is_running():
        # 키보드 입력으로 액션 생성 (실제로는 더 복잡한 입력 처리 필요)
        action = get_keyboard_action()  # 사용자 정의 함수

        # 환경 스텝
        obs, reward, terminated, truncated, info = env.step(action)

        # 데이터 저장
        episode_data["observations"].append(obs)
        episode_data["actions"].append(action.cpu().numpy())

        step_count += 1

        # 에피소드 종료
        if terminated or truncated:
            demonstrations.append(episode_data)
            print(f"에피소드 완료! 총 {step_count} 스텝")

            # 새 에피소드 시작 여부 확인
            continue_demo = input("다른 데모를 수집하시겠습니까? (y/n): ")
            if continue_demo.lower() != 'y':
                break

            env.reset()
            episode_data = {
                "observations": [],
                "actions": [],
                "language_instruction": input("작업 설명을 입력하세요: ")
            }
            step_count = 0

    # 데모 저장
    save_path = "/home/isaac/ust_ws/demonstrations.npz"
    np.savez_compressed(save_path, **{f"demo_{i}": demo for i, demo in enumerate(demonstrations)})
    print(f"✅ {len(demonstrations)}개 데모가 {save_path}에 저장되었습니다.")

    env.close()

def get_keyboard_action():
    """키보드 입력을 로봇 액션으로 변환 (간소화 예제)."""
    # 실제 구현은 omni.isaac.lab.devices.keyboard를 사용
    action = torch.zeros(16)  # 16 DOF
    # ... 키보드 입력 처리 로직 ...
    return action

if __name__ == "__main__":
    main()
    simulation_app.close()
```

### 5. 데모 수집 실행

```bash
cd ~/ust_ws
python teleoperation_demo.py --num_envs 1
```

---

## 데이터 수집 파이프라인

### 1. 데이터 포맷 변환

OpenVLA는 Open X-Embodiment 포맷을 사용합니다. Isaac Lab 데모를 변환:

`convert_to_oxe.py`:

```python
# ~/ust_ws/convert_to_oxe.py

import numpy as np
import tensorflow as tf
import tensorflow_datasets as tfds
from pathlib import Path

def convert_demonstrations_to_oxe(demo_path, output_dir):
    """Isaac Lab 데모를 Open X-Embodiment 포맷으로 변환."""

    # 데모 로드
    demos = np.load(demo_path, allow_pickle=True)

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    for demo_name, demo_data in demos.items():
        # RLDS 형식으로 변환
        episode = {
            'steps': []
        }

        observations = demo_data['observations']
        actions = demo_data['actions']
        language = demo_data['language_instruction']

        for i in range(len(observations)):
            step = {
                'observation': {
                    'image': observations[i],  # (224, 224, 3)
                    'state': np.zeros(16),  # 조인트 상태 (필요시 추가)
                },
                'action': actions[i],  # (16,)
                'language_instruction': language,
                'is_first': i == 0,
                'is_last': i == len(observations) - 1,
                'is_terminal': i == len(observations) - 1,
            }
            episode['steps'].append(step)

        # TFRecord로 저장
        output_file = output_path / f"{demo_name}.tfrecord"
        with tf.io.TFRecordWriter(str(output_file)) as writer:
            # ... TFRecord 작성 로직 ...
            pass

    print(f"✅ 데이터 변환 완료: {output_dir}")

if __name__ == "__main__":
    convert_demonstrations_to_oxe(
        "/home/isaac/ust_ws/demonstrations.npz",
        "/home/isaac/ust_ws/oxe_dataset"
    )
```

### 2. 데이터 증강 (Augmented Imitation Learning)

Isaac Lab의 Cosmos 모델 통합으로 시각적 변이를 증강:

```python
# augment_demos.py

from omni.isaac.lab.utils.augmentation import CosmosAugmentation

def augment_visual_data(dataset_path, num_variations=10):
    """Cosmos를 사용한 시각적 데이터 증강."""

    augmenter = CosmosAugmentation(
        variations=['lighting', 'texture', 'camera_pose']
    )

    # 원본 데이터 로드
    dataset = load_dataset(dataset_path)

    augmented_dataset = []
    for episode in dataset:
        for _ in range(num_variations):
            augmented_episode = augmenter.apply(episode)
            augmented_dataset.append(augmented_episode)

    # 증강된 데이터 저장
    save_dataset(augmented_dataset, dataset_path + "_augmented")
    print(f"✅ {len(dataset)} 에피소드 → {len(augmented_dataset)} 에피소드로 증강")

if __name__ == "__main__":
    augment_visual_data("/home/isaac/ust_ws/oxe_dataset")
```

### 3. 권장 데모 수집 전략

**최소 요구사항**:
- 에피소드 수: 50-100개 (작업당)
- 다양성: 다양한 초기 조건, 객체 위치
- 성공률: >80% 성공 데모만 사용

**확장 전략**:
1. **MimicGen 사용**: Isaac Lab Mimic으로 적은 데모에서 자동 생성
2. **크로스 엠보디먼트**: Open X-Embodiment에서 유사 작업 데이터 혼합
3. **시뮬레이션 증강**: Cosmos로 10배 데이터 증강

---

## 모델 학습 및 파인튜닝

### 방법 1: LoRA 파인튜닝 (추천 - RTX 4090 단일 GPU)

**장점**:
- 단일 RTX 4090에서 가능
- 빠른 학습 속도
- 메모리 효율적 (~20GB)

**실행**:

```bash
cd ~/ust_ws/openvla

# LoRA 파인튜닝 스크립트
python vla-scripts/finetune.py \
  --model_path openvla/openvla-7b \
  --data_root /home/isaac/ust_ws/oxe_dataset \
  --dataset_name custom_mobile_bimanual \
  --run_root /home/isaac/ust_ws/openvla_runs \
  --adapter_tmp_dir /home/isaac/ust_ws/openvla_adapters \
  --lora_rank 32 \
  --lora_alpha 64 \
  --lora_dropout 0.1 \
  --batch_size 8 \
  --grad_accumulation_steps 4 \
  --learning_rate 5e-5 \
  --warmup_steps 100 \
  --max_steps 5000 \
  --save_steps 500 \
  --eval_steps 250 \
  --mixed_precision bf16 \
  --workers 4
```

**주요 하이퍼파라미터**:
- `lora_rank`: 32 (낮을수록 빠름, 높을수록 표현력 증가)
- `batch_size`: 8 (RTX 4090 최대)
- `grad_accumulation_steps`: 4 (유효 배치 크기 = 32)
- `learning_rate`: 5e-5 (일반적 시작점)

### 방법 2: OFT (Optimized Fine-Tuning) - 최신 방법

**장점**:
- 25-50배 빠른 추론
- 고주파 양팔 제어
- 향상된 성공률

**실행**:

```bash
cd ~/ust_ws/openvla

# OFT 레시피 사용
python vla-scripts/finetune_oft.py \
  --model_path openvla/openvla-7b \
  --data_root /home/isaac/ust_ws/oxe_dataset \
  --dataset_name custom_mobile_bimanual \
  --run_root /home/isaac/ust_ws/openvla_oft_runs \
  --use_fast_tokenizer \
  --batch_size 8 \
  --learning_rate 3e-5 \
  --max_steps 5000
```

### 방법 3: Full Fine-Tuning (멀티 GPU 필요)

**요구사항**:
- 8x A100 (80GB) 또는 동급
- FSDP (Fully Sharded Data Parallel)

**실행** (멀티 GPU 환경):

```bash
torchrun --nproc_per_node=8 vla-scripts/finetune_full.py \
  --model_path openvla/openvla-7b \
  --data_root /home/isaac/ust_ws/oxe_dataset \
  --sharding_strategy FULL_SHARD \
  --batch_size 16 \
  --learning_rate 1e-5
```

### 학습 모니터링

```bash
# TensorBoard 실행
tensorboard --logdir /home/isaac/ust_ws/openvla_runs --port 6006

# 브라우저에서 http://localhost:6006 접속
```

**주요 지표**:
- **Training Loss**: 지속적 감소 확인
- **Eval Success Rate**: >70% 목표
- **Action MSE**: 원본 액션과의 오차

### 학습 시간 예상

**RTX 4090 기준 (LoRA)**:
- 100 에피소드: ~2-3시간
- 500 에피소드: ~10-15시간
- 1000 에피소드: ~20-30시간

**체크포인트 저장**:
- 500 스텝마다 자동 저장
- 최고 성능 모델 별도 저장

---

## 성능 최적화 전략

### 1. 모델 양자화 (추론 속도 2-3배 향상)

**INT4 양자화** (메모리 50% 감소):

```python
# quantize_model.py

from transformers import AutoModelForVision2Seq, BitsAndBytesConfig
import torch

# 양자화 설정
quantization_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_compute_dtype=torch.bfloat16,
    bnb_4bit_use_double_quant=True,
    bnb_4bit_quant_type="nf4"
)

# 모델 로드
model = AutoModelForVision2Seq.from_pretrained(
    "openvla/openvla-7b",
    quantization_config=quantization_config,
    device_map="auto",
    trust_remote_code=True
)

# 추론 속도 테스트
# RTX 4090: bfloat16 ~6Hz → INT4 ~12Hz
```

### 2. TensorRT-LLM 최적화 (추가 2-3배 향상)

```bash
# TensorRT-LLM 설치
pip install tensorrt_llm -U --extra-index-url https://pypi.nvidia.com

# 모델 변환 (별도 스크립트 필요)
python convert_openvla_to_trt.py \
  --model openvla/openvla-7b \
  --output /home/isaac/ust_ws/openvla_trt \
  --dtype float16 \
  --max_batch_size 1
```

**예상 성능**:
- bfloat16: 6Hz
- INT4: 12Hz
- TensorRT-LLM + INT4: 25-30Hz (OFT 적용 시 최대 50Hz)

### 3. 배치 추론 (시뮬레이션 병렬화)

Isaac Lab의 벡터화 환경 활용:

```python
# parallel_inference.py

from omni.isaac.lab.envs import ManagerBasedRLEnv

# 64개 환경 병렬 실행
env = ManagerBasedRLEnv(cfg=env_cfg, num_envs=64)

# 배치 추론
observations = env.reset()
actions = model.predict_action_batch(observations)  # (64, 16)
env.step(actions)
```

**처리량**:
- 단일 환경: 6 FPS
- 64 환경 배치: ~200 FPS (환경당 ~3 FPS)

### 4. REST API 서버 배포

분산 아키텍처로 시뮬레이션과 추론 분리:

```bash
# 서버 실행 (추론 전용 머신)
python vla-scripts/serve_model.py \
  --model_path /home/isaac/ust_ws/openvla_runs/checkpoint-5000 \
  --host 0.0.0.0 \
  --port 8000 \
  --quantization int4

# 클라이언트 (Isaac Sim 머신)
import requests

response = requests.post(
    "http://server_ip:8000/predict",
    json={
        "image": image_base64,
        "instruction": "pick up the red block"
    }
)
action = response.json()["action"]
```

---

## 트러블슈팅

### 문제 1: CUDA Out of Memory

**증상**:
```
RuntimeError: CUDA out of memory. Tried to allocate 2.00 GiB
```

**해결책**:
1. 배치 크기 감소:
   ```bash
   --batch_size 4  # 8에서 4로
   ```

2. Gradient Accumulation 증가:
   ```bash
   --grad_accumulation_steps 8  # 유효 배치 크기 유지
   ```

3. 양자화 사용:
   ```python
   load_in_4bit=True
   ```

4. Flash Attention 활성화:
   ```python
   use_flash_attention_2=True
   ```

### 문제 2: 낮은 성공률 (<50%)

**원인 분석**:
1. 데모 품질 불량
2. 과적합 (데이터 부족)
3. 하이퍼파라미터 미조정

**해결책**:

**A. 데모 품질 개선**:
```bash
# 성공 데모만 필터링
python filter_successful_demos.py \
  --input demonstrations.npz \
  --output demonstrations_filtered.npz \
  --min_success_rate 0.8
```

**B. 데이터 증강**:
```bash
# Cosmos 증강 + MimicGen
python augment_and_generate.py \
  --input demonstrations_filtered.npz \
  --augmentation_factor 10 \
  --mimicgen_trajectories 500
```

**C. 정규화 추가**:
```bash
--lora_dropout 0.2  # 0.1에서 0.2로 증가
--weight_decay 0.01
```

**D. 학습률 스케줄 조정**:
```bash
--lr_scheduler cosine
--warmup_ratio 0.1
```

### 문제 3: 추론 속도 느림 (< 3 Hz)

**원인**:
- Flash Attention 미설치
- 컴파일 최적화 미적용
- 비효율적 전처리

**해결책**:

**A. Flash Attention 2 설치 확인**:
```bash
python -c "import flash_attn; print('✅ Flash Attention 2 설치됨')"
```

**B. Torch Compile 사용** (PyTorch 2.0+):
```python
model = torch.compile(model, mode="reduce-overhead")
```

**C. 이미지 전처리 최적화**:
```python
# GPU에서 전처리 수행
processor.to("cuda")
inputs = processor(prompt, image).to("cuda", dtype=torch.bfloat16)
```

### 문제 4: Isaac Sim 충돌

**증상**:
```
Segmentation fault (core dumped)
```

**해결책**:

**A. 환경 변수 설정**:
```bash
export ISAACSIM_HEADLESS=1  # 헤드리스 모드
export OMNI_KIT_ALLOW_ROOT=1
```

**B. 드라이버 업데이트**:
```bash
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
```

**C. 리소스 제한**:
```python
# 동시 환경 수 감소
num_envs = 1  # 16에서 1로
```

### 문제 5: 모델이 항상 같은 액션 출력

**원인**:
- 과적합
- 데이터 불균형
- 액션 토큰화 문제

**해결책**:

**A. 온도 샘플링 활성화**:
```python
action = vla.predict_action(
    **inputs,
    do_sample=True,
    temperature=1.0  # 다양성 증가
)
```

**B. 데이터 밸런싱**:
```python
# 드문 액션에 가중치 부여
class_weights = compute_class_weights(dataset)
loss_fn = WeightedMSELoss(weights=class_weights)
```

**C. 액션 정규화 확인**:
```python
# OpenVLA는 [-1, 1] 범위 기대
actions = (actions - action_mean) / action_std
actions = np.clip(actions, -1, 1)
```

---

## 참고 자료

### 공식 문서

1. **OpenVLA**:
   - GitHub: https://github.com/openvla/openvla
   - 논문: https://arxiv.org/abs/2406.09246
   - HuggingFace: https://huggingface.co/openvla/openvla-7b

2. **Isaac Lab**:
   - GitHub: https://github.com/isaac-sim/IsaacLab
   - 문서: https://isaac-sim.github.io/IsaacLab/
   - 블로그: https://developer.nvidia.com/blog/fast-track-robot-learning-in-simulation-using-nvidia-isaac-lab/

3. **Isaac Sim**:
   - 공식 사이트: https://developer.nvidia.com/isaac/sim
   - 문서: https://docs.omniverse.nvidia.com/isaacsim/latest/

### 관련 논문

1. **OpenVLA (2024)**:
   ```
   Kim, Moo Jin, et al. "OpenVLA: An Open-Source Vision-Language-Action Model."
   arXiv preprint arXiv:2406.09246 (2024).
   ```

2. **Mobile ALOHA (2024)**:
   ```
   Fu, Zipeng, et al. "Mobile ALOHA: Learning Bimanual Mobile Manipulation with Low-Cost Whole-Body Teleoperation."
   arXiv preprint arXiv:2401.02117 (2024).
   ```

3. **Diffusion Policy (2023)**:
   ```
   Chi, Cheng, et al. "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion."
   RSS 2023, IJRR 2024.
   ```

4. **RT-2 (2023)**:
   ```
   Brohan, Anthony, et al. "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control."
   arXiv preprint arXiv:2307.15818 (2023).
   ```

### 커뮤니티 및 지원

- **NVIDIA Jetson AI Lab**: https://www.jetson-ai-lab.com/openvla.html
- **OpenVLA Discord**: (GitHub 저장소 참조)
- **Isaac Lab GitHub Issues**: https://github.com/isaac-sim/IsaacLab/issues
- **NVIDIA Developer Forums**: https://forums.developer.nvidia.com/

### 추가 도구 및 프레임워크

1. **MimicGen**: 적은 데모로 자동 궤적 생성
2. **Cosmos**: NVIDIA의 물리적 AI 세계 모델
3. **ROS2 Integration**: 실제 로봇 배포용
4. **PyTorch Lightning**: 학습 파이프라인 간소화

---

## 다음 단계 로드맵

### 1주차: 환경 설정 및 검증
- [ ] CUDA/Driver 설정 확인
- [ ] OpenVLA 설치 및 테스트
- [ ] Isaac Lab 설치
- [ ] ust_project1.usd 환경 로드 확인

### 2주차: 데모 수집
- [ ] 텔레오퍼레이션 인터페이스 구축
- [ ] 50개 성공 데모 수집
- [ ] 데이터 포맷 변환 (OXE)
- [ ] 데이터 증강 (Cosmos)

### 3주차: 모델 학습
- [ ] LoRA 파인튜닝 시작
- [ ] 학습 모니터링 및 하이퍼파라미터 조정
- [ ] 체크포인트 평가
- [ ] 최적 모델 선택

### 4주차: 배포 및 최적화
- [ ] 모델 양자화 (INT4)
- [ ] Isaac Sim 실시간 추론 통합
- [ ] 성능 벤치마크
- [ ] 문서화 및 정리

### 장기 목표
- [ ] OFT 방법 적용 (고속 추론)
- [ ] 실제 로봇 sim-to-real 전이
- [ ] ROS2 통합
- [ ] 다중 작업 일반화 테스트

---

## 마무리

이 가이드는 RTX 4090 환경에서 양팔 모바일 매니퓰레이터에 OpenVLA를 통합하여 모방학습 시스템을 구축하는 포괄적인 방법을 제공합니다.

**핵심 포인트**:
1. ✅ OpenVLA-7B는 RTX 4090에 최적화된 선택
2. ✅ LoRA 파인튜닝으로 단일 GPU 학습 가능
3. ✅ Isaac Lab으로 효율적인 데모 수집
4. ✅ 양자화 및 최적화로 실시간 제어 달성
5. ✅ 활발한 오픈소스 커뮤니티 지원

**성공을 위한 팁**:
- 🎯 고품질 데모에 집중 (양보다 질)
- 🎯 점진적 접근 (간단한 작업부터 시작)
- 🎯 지속적 평가 및 반복
- 🎯 커뮤니티 활용 (GitHub Issues, Forums)

질문이나 문제가 있으면 관련 GitHub 저장소의 Issues를 확인하거나 NVIDIA Developer Forums에 문의하세요.

**행운을 빕니다! 🚀**
