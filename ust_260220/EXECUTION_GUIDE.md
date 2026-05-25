# G1 주방 물체 분류 환경 - 실행 가이드

> GPU: NVIDIA RTX PRO 6000 (96GB VRAM, Blackwell)
> CloudXR: 6.0.1-webrtc (공유 폴더: `./ust_ws/openxr`)
> Isaac Lab: 2.3.0 + Isaac Sim 5.1.0

---

## 개요

Unitree G1 휴머노이드 로봇에 INSPIRE 5지 핸드(G1_INSPIRE_FTP_CFG)를 장착하여, 주방 테이블(높이 0.75m) 위 8개 물체를 3개 분류 빈에 정리하는 환경입니다.

### 카메라 관련 주의사항

> **물리적 카메라는 사용하지 않습니다.**
>
> 본 환경에서 "카메라"는 모두 **Isaac Sim 시뮬레이션 내부 가상 카메라**를 의미합니다.
> Isaac Sim의 RTX 렌더러가 시뮬레이션 세계 내에서 가상 카메라 이미지(RGB, 깊이, 시맨틱 분할)를
> 생성합니다. VR 텔레오퍼레이션 시 오퍼레이터는 CloudXR을 통해 시뮬레이션 화면을 봅니다.
>
> 시뮬레이션 카메라 이미지는 비전 기반 정책 학습(HG-DAgger Phase 2+)에 필요할 때만
> `--with_sim_camera --enable_cameras` 옵션으로 녹화합니다.

### 물체-빈 매핑

| 카테고리 | 물체 | 목표 빈 | 빈 색상 |
|----------|------|---------|---------|
| 주방용품 | mug, plate, bowl | BinKitchen (왼쪽) | 파란색 |
| 식품 | can, bottle, apple | BinFood (중앙) | 파란색 |
| 기타 | sponge, teddy | BinMisc (오른쪽) | 검정색 |

### 환경 사양

| 항목 | 값 |
|------|-----|
| 로봇 | Unitree G1 + INSPIRE 5지 핸드 (고정 베이스, 중력 비활성화) |
| 액션 공간 | 38D (14 EEF: 좌/우 손목 각 7D, 24 핸드 관절) |
| IK 제어기 | Pink IK (손목별 FrameTask + NullSpacePostureTask) |
| 테이블 | Cuboid 0.9x0.6m, 표면 높이 0.75m |
| 물리 | 120 Hz (dt=1/120), decimation=6 (20 Hz 제어) |
| 씬 쿼리 | `enable_scene_query_support = True` (VR 필수) |
| 렌더링 | 매 프레임 (render_interval=1), VR 모드: 매 2프레임 |
| 에피소드 길이 | 60초 (기본), 3600초 (VR/데이터수집), 120초 (학습) |
| 성공 조건 | 8개 물체 전부 올바른 빈에 배치 (XY 거리 < 0.12m) |

### 등록된 Gym 환경 ID

| ID | 설명 |
|----|------|
| `Isaac-KitchenSorting-G1-InspireFTP-v0` | 기본 (1개 환경, 60초 에피소드) |
| `Isaac-KitchenSorting-G1-InspireFTP-Vision-v0` | 시뮬레이션 카메라 포함 (720p, 60fps) |
| `Isaac-KitchenSorting-G1-InspireFTP-Train-v0` | 다중 환경 학습 (16개 환경, 120초) |
| `Isaac-KitchenSorting-G1-InspireFTP-VR-v0` | VR 텔레오퍼레이션 (3600초, CloudXR 최적화) |
| `Isaac-KitchenSorting-G1-InspireFTP-DataCollect-v0` | 데모 녹화 + 시뮬레이션 카메라 (3600초) |

---

## 사전 준비

### 1. Isaac Lab 환경

```bash
# Docker 컨테이너 또는 로컬 설치
# Isaac Lab 2.3.0 + Isaac Sim 5.1.0
```

### 2. CloudXR 런타임 설정 (VR 텔레오퍼레이션)

```bash
# 1단계: CloudXR 런타임 Docker 시작
cd /workspace/isaaclab
./docker/container.py start \
    --files ust_ws/ust_260220/docker-compose.cloudxr-kitchen-sorting.patch.yaml \
    --env-file docker/.env.cloudxr-runtime

# 2단계: 환경 변수 설정 (Isaac Lab 컨테이너 내부에서)
source ust_ws/ust_260220/setup_cloudxr_env.sh

# 3단계: CloudXR 연결 확인
ls -la ust_ws/openxr/run/ipc_cloudxr  # 파일이 존재해야 함
```

### 3. CloudXR.js 클라이언트 (Quest 3S / Vision Pro)

```bash
# HTTPS 개발 서버 시작
cd ust_ws/cloudxr_js/isaac
npm run dev-server:https  # 포트 8080

# HAProxy SSL 프록시 시작
cd ust_ws/cloudxr_js/proxy
docker build -t cloudxr-proxy . && docker run -d -p 48322:48322 cloudxr-proxy

# Quest 브라우저에서: https://<호스트IP>:8080
# 개발 서버 인증서(8080)와 HAProxy 인증서(48322) 모두 수락 필요
```

### 4. 주요 환경 변수

| 변수 | 값 | 용도 |
|------|-----|------|
| `XDG_RUNTIME_DIR` | `/workspace/isaaclab/ust_ws/openxr/run` | IPC 소켓 디렉토리 |
| `XR_RUNTIME_JSON` | `.../openxr/share/openxr/1/openxr_cloudxr.json` | OpenXR 매니페스트 |
| `IPC_IGNORE_VERSION` | `1` | Kit 5.0.1 <-> Runtime 6.0.1 호환성 |
| `NV_GPU_INDEX` | `0` | RTX PRO 6000 GPU 인덱스 |
| `NV_PACER_FIXED_TIME_STEP_MS` | `11` | 약 90Hz VR 렌더링 |

---

## 실행 명령어

### 1. 기본 환경 테스트 (VR 없이)

```bash
# GUI 모드 - 씬 레이아웃 확인
python ust_ws/ust_260220/scripts/run_env.py --num_envs 1

# 헤드리스 (서버)
python ust_ws/ust_260220/scripts/run_env.py --num_envs 1 --headless

# 시뮬레이션 카메라 포함 (RTX PRO 6000: 720p, 60fps)
python ust_ws/ust_260220/scripts/run_env.py --num_envs 1 --enable_cameras

# 다중 환경 학습 (RTX PRO 6000: 최대 16개 환경)
python ust_ws/ust_260220/scripts/run_env.py --num_envs 16 --headless
```

**확인 항목:**
- [ ] G1 로봇이 (0, 0, 1.0) 위치에서 +Y 방향을 향하고 있는지
- [ ] Cuboid 테이블 (나무 색상, 0.9x0.6x0.75m)이 Y=0.55에 있는지
- [ ] 3개 빈이 테이블 뒤쪽(Y=0.82)에 배치: 파란-파란-검정
- [ ] 8개 물체가 테이블 표면(Z ~ 0.77-0.84)에 있는지
- [ ] 물체가 물리적으로 안정적인지 (테이블을 관통하지 않는지)
- [ ] 따뜻한 주방 조명 (돔 + 스팟)

### 2. VR 텔레오퍼레이션

```bash
# 중요: 먼저 CloudXR 설정 실행
source ust_ws/ust_260220/setup_cloudxr_env.sh

# OpenXR 핸드 트래킹 (Quest 3S / Vision Pro)
# VR 환경 변형 사용: 1시간 에피소드, render_interval=2, scene_query=True
python ust_ws/ust_260220/scripts/run_teleop.py --teleop_device handtracking

# XR Kit 파일 사용 (전체 CloudXR 렌더링 파이프라인)
python ust_ws/ust_260220/scripts/run_teleop.py \
    --teleop_device handtracking \
    --kit_app apps/isaaclab.python.xr.openxr.kit

# Manus VR 글러브 + HTC Vive 트래커
python ust_ws/ust_260220/scripts/run_teleop.py --teleop_device manusvive
```

**참고:** 키보드 텔레옵은 G1 Pink IK 액션 공간(38D)에서 지원되지 않습니다.
`handtracking`과 `manusvive` VR 디바이스만 사용 가능합니다.

**VR 환경 설정값:**
| 설정 | 값 | 이유 |
|------|-----|------|
| `episode_length_s` | 3600초 (1시간) | 중단 없는 VR 데모 세션 |
| `render_interval` | 2 | 안정적인 CloudXR를 위한 약 60Hz 렌더링 |
| `enable_scene_query_support` | True | VR 렌더링 필수 |

### 3. 데모 녹화

물리적 카메라가 없으므로, 기본적으로 관측/액션만 녹화합니다.
비전 정책 학습이 필요한 경우에만 시뮬레이션 카메라를 활성화합니다.

```bash
# [기본] VR 텔레옵 데모 녹화 (카메라 없음, 1시간 에피소드)
# → 관측 벡터 + 액션만 HDF5에 저장
python ust_ws/ust_260220/scripts/record_demos.py \
    --teleop_device handtracking \
    --num_demos 50 \
    --output_dir ust_ws/ust_260220/data/demos/

# [시뮬레이션 카메라] 가상 카메라 이미지 함께 녹화 (비전 학습용)
# → 관측 + 액션 + RGB/깊이 이미지를 HDF5에 저장
# 주의: --enable_cameras 플래그 필수
python ust_ws/ust_260220/scripts/record_demos.py \
    --teleop_device handtracking \
    --num_demos 50 \
    --with_sim_camera \
    --enable_cameras

# 빠른 테스트 (10개 데모)
python ust_ws/ust_260220/scripts/record_demos.py --num_demos 10

# 학습/검증 분할 비율 변경
python ust_ws/ust_260220/scripts/record_demos.py \
    --num_demos 50 \
    --train_split 0.9
```

**`--with_sim_camera` 옵션 상세:**
| 항목 | `--with_sim_camera` 없음 (기본) | `--with_sim_camera` 사용 |
|------|-------------------------------|--------------------------|
| 환경 | VR-v0 (카메라 없음) | DataCollect-v0 (시뮬레이션 카메라) |
| 에피소드 | 3600초 (1시간) | 3600초 (1시간) |
| 저장 데이터 | obs + actions | obs + actions + RGB + 깊이 |
| 이미지 출처 | 없음 | Isaac Sim RTX 렌더러 (가상) |
| 카메라 | 없음 | robot_head_cam (720p) |
| 플래그 | (없음) | `--enable_cameras` 필수 |
| 용도 | Phase 1 데모 수집 | Phase 2+ 비전 학습 데이터 |

**HDF5 출력 구조:**
```
kitchen_sorting_demos_YYYYMMDD_HHMMSS.hdf5
├── data/
│   ├── demo_0/
│   │   ├── obs          (T, obs_dim)     # 평탄화된 관측
│   │   ├── actions      (T, 38)          # 14 EEF + 24 핸드 관절
│   │   ├── dones        (T,)             # 종료 플래그
│   │   ├── timestamps   (T,)             # 실시간 타임스탬프
│   │   ├── rewards      (T,)             # 스텝별 보상
│   │   ├── images_rgb   (T, 720, 1280, 3)  # [--with_sim_camera] 시뮬레이션 RGB
│   │   └── images_depth (T, 720, 1280)      # [--with_sim_camera] 시뮬레이션 깊이
│   └── ...
├── mask/
│   ├── train            (N,) bool        # 학습 분할 마스크
│   └── valid            (N,) bool        # 검증 분할 마스크
├── env_args: {env_name, robot, ik_controller, action_dim}
├── num_episodes: N
└── total_steps: M
```

---

## 프로젝트 구조

```
ust_ws/ust_260220/
├── __init__.py                                 # Gym 환경 등록 (5개 변형)
├── kitchen_sorting_env_cfg.py                  # 메인 환경 설정
│   ├── KitchenSortingSceneCfg                  #   기본 씬 (카메라 없음)
│   ├── KitchenSortingVisionSceneCfg            #   시뮬레이션 가상 카메라 씬
│   ├── ActionsCfg                              #   Pink IK 38D
│   ├── ObservationsCfg                         #   관측 그룹
│   ├── RewardsCfg / TerminationsCfg / EventCfg #   MDP
│   ├── KitchenSortingG1EnvCfg                  #   기본 (1개 환경, 60초)
│   ├── KitchenSortingG1VisionEnvCfg            #   시뮬레이션 카메라 (1개 환경)
│   ├── KitchenSortingG1TrainEnvCfg             #   학습 (16개 환경, 120초)
│   ├── KitchenSortingG1VREnvCfg                #   VR 텔레옵 (1개 환경, 3600초)
│   └── KitchenSortingG1DataCollectEnvCfg       #   데모녹화+시뮬카메라 (3600초)
├── mdp/
│   ├── __init__.py
│   ├── observations.py                         # EEF, 물체, 빈 위치
│   ├── terminations.py                         # 전체 분류 완료, 분류 개수
│   └── events.py                               # 물체 위치 랜덤화
├── utils/
│   ├── __init__.py
│   └── hdf5_recorder.py                        # HDF5 데이터셋 레코더/리더
├── scripts/
│   ├── run_env.py                              # 환경 테스트
│   ├── run_teleop.py                           # VR 텔레오퍼레이션 (VR-v0)
│   └── record_demos.py                         # 데모 녹화 (VR-v0/DataCollect-v0)
├── setup_cloudxr_env.sh                        # CloudXR 환경변수 설정
├── docker-compose.cloudxr-kitchen-sorting.patch.yaml  # Docker Compose 패치
├── EXECUTION_GUIDE.md                          # 이 파일
└── data/
    └── demos/                                  # 녹화된 데모 데이터
```

---

## 씬 레이아웃

```
     X=-0.4                  X=0                  X=+0.4

     BinKitchen           BinFood              BinMisc
     (-0.28,0.82,0.80)    (0,0.82,0.80)       (0.28,0.82,0.80)      Y=0.82
          |                   |                    |
     ┌─────────────────────────────────────────────────┐
     │                                                 │
     │   [plate]   [bowl]    [mug]     [bottle]        │  Y=0.55
     │      [can]    [apple]   [sponge]   [teddy]      │  (중앙)
     │                                                 │
     └─────────────────────────────────────────────────┘
     테이블: 0.9m x 0.6m x 0.75m (Cuboid, 나무 색상)      Y=0.25
                                                          (앞쪽 가장자리)

                    로봇 G1 (0, 0, 1.0)
                    +Y 방향 향함                            Y=0.0
```

**물체 높이 (테이블 표면 0.75m 기준):**
| 물체 | 스폰 Z | 계산식 |
|------|--------|--------|
| plate (접시) | 0.77 | 0.75 + 0.02/2 |
| sponge (스펀지) | 0.77 | 0.75 + 0.03/2 |
| bowl (그릇) | 0.78 | 0.75 + ~0.03 |
| apple (사과) | 0.79 | 0.75 + 0.04 |
| mug (머그컵) | 0.80 | 0.75 + ~0.05 |
| can (캔) | 0.81 | 0.75 + 0.10/2 + 여유분 |
| teddy (곰인형) | 0.82 | 0.75 + ~0.07 |
| bottle (병) | 0.84 | 0.75 + 0.18/2 |

---

## 환경 변형 비교

| 설정 | 기본 (v0) | 시뮬카메라 | 학습 | VR | 데모녹화+시뮬카메라 |
|------|-----------|-----------|------|-----|-------------------|
| `num_envs` | 1 | 1 | 16 | 1 | 1 |
| `episode_length_s` | 60 | 60 | 120 | 3600 | 3600 |
| `render_interval` | 1 | 1 | 2 | 2 | 2 |
| `scene_query_support` | True | True | False | True | True |
| 시뮬레이션 카메라 | 없음 | 720p@60fps | 없음 | 없음 | 720p@60fps |
| 물리 카메라 | 없음 | 없음 | 없음 | 없음 | 없음 |
| 주요 용도 | 테스트 | 비전 학습 | RL 학습 | VR 텔레옵 | VR 데모+이미지 녹화 |

---

## RTX PRO 6000 성능 참고

| 설정 | 값 | 근거 |
|------|-----|------|
| `render_interval` | 1 (기본), 2 (VR/학습) | RTX PRO 6000은 전체 프레임 렌더링 가능 |
| 시뮬레이션 카메라 해상도 | 1280x720 @ 60fps | 96GB VRAM에 적합 |
| 최대 병렬 환경 수 | 16 (학습), 1 (텔레옵) | 96GB VRAM으로 다수 환경 지원 |
| `solver_position_iteration_count` | 16 | 높은 정밀도 그래스핑 물리 |
| `gpu_found_lost_aggregate_pairs_capacity` | 1M | 대규모 접촉 쌍 버퍼 |
| 물리 재질 | static=0.7, dynamic=0.5 | 안정적 그래스핑을 위한 전역 마찰력 |
| `enable_scene_query_support` | True (학습 제외 전부) | VR/시뮬레이션 카메라 렌더링 필수 |

---

## 문제 해결

### Nucleus 에셋 로드 실패
```
Error: Could not load USD file from ISAACLAB_NUCLEUS_DIR
```
Nucleus 서버 접속을 확인하세요. 오프라인 환경에서는 로컬 USD 경로로 변경이 필요합니다.

### Pink IK URDF 변환 오류
```
Error: Failed to convert USD to URDF
```
`/tmp` 디렉토리의 쓰기 권한 및 디스크 용량을 확인하세요.

### VR 연결 실패
```
[WARN] Device 'handtracking' not found.
```
1. `source ust_ws/ust_260220/setup_cloudxr_env.sh` 실행 여부 확인
2. `docker ps | grep cloudxr` - 런타임 실행 확인
3. `ls -la ust_ws/openxr/run/ipc_cloudxr` - IPC 소켓 존재 확인
4. Quest/Vision Pro에서 CloudXR.js 클라이언트 연결 확인

### 물체가 테이블을 관통하여 떨어지는 경우
- `solver_position_iteration_count` 값 확인 (기본값 16)
- 테이블의 `kinematic_enabled=True` 설정 확인
- 물체 스폰 Z 좌표가 테이블 표면(0.75m) 위인지 확인

### 시뮬레이션 카메라가 작동하지 않는 경우
```
RuntimeError: Camera sensor requires --enable_cameras
```
시뮬레이션 카메라가 포함된 환경(Vision, DataCollect) 사용 시 반드시 `--enable_cameras` 플래그를 추가하세요.
기본 환경(`v0`, `VR-v0`)에는 시뮬레이션 카메라가 없으므로 이 플래그가 필요 없습니다.

**참고:** 본 환경에는 물리적 카메라가 없습니다. 모든 카메라 이미지는 Isaac Sim RTX 렌더러가 시뮬레이션 내부에서 생성하는 가상 이미지입니다.

### 씬 쿼리 오류
```
RuntimeError: enable_scene_query_support must be True for VR
```
모든 VR 관련 환경에서는 `enable_scene_query_support = True`가 자동 설정됩니다.
학습(Train) 환경만 `False`입니다 (헤드리스 학습 전용).

---

## 데모 수집 워크플로우

### Phase 1: 기본 데모 수집 (시뮬레이션 카메라 없이)

```bash
# 1. CloudXR 설정
source ust_ws/ust_260220/setup_cloudxr_env.sh

# 2. Quest 3S에서 CloudXR.js 연결

# 3. 데모 50개 녹화 (관측+액션만, 카메라 없음)
python ust_ws/ust_260220/scripts/record_demos.py \
    --teleop_device handtracking \
    --num_demos 50
```

### Phase 2+: 비전 학습용 데모 수집 (시뮬레이션 카메라 포함)

```bash
# 1. CloudXR 설정 (동일)
source ust_ws/ust_260220/setup_cloudxr_env.sh

# 2. Quest 3S에서 CloudXR.js 연결

# 3. 데모 녹화 (관측+액션+시뮬레이션 카메라 RGB/깊이)
python ust_ws/ust_260220/scripts/record_demos.py \
    --teleop_device handtracking \
    --num_demos 50 \
    --with_sim_camera \
    --enable_cameras
```

---

## 다음 단계

1. **Phase 1**: VR 텔레오퍼레이션으로 데모 50개 수집 (`record_demos.py`)
2. **Phase 2**: HG-DAgger 교정 학습 시스템 구축
3. **Phase 3**: Ensemble 불확실성 기반 도움 요청 시스템
4. **Level 4**: VLM 미지 물체 인식 + KnowNo 카테고리 예측
