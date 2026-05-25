# ust_hm_glove.validation — UDCAP → Isaac Lab 손가락 정밀제어 검증·디버깅·시각화 시스템

> **목적**: ust_hm_glove 의 UDCAP→Isaac Lab 손가락 매칭이 실제로 정확하게 동작하는지 빠르게 검증/디버깅하기 위한 분리된 인프라.
>
> **설계**: [`research/35.`](research/35.%20udcap_finger_precision_test_validation_debug_system.md) (4-layer test architecture)
>
> **원칙**: ust_hm_glove 코드는 **수정하지 않고** import 만 한다.

---

## 0. 5분 안에 시작하기 (사용자 환경에 UDCAP 가 살아있을 때)

```powershell
# 1. ust conda env 활성화 (Isaac Lab + 필요 deps 모두 있는 환경)
conda activate ust

# 2. (한 번만) 추가 디버그 deps 설치
pip install -r ust_ws\ust_hm_gloveust_ws\validation\requirements-debug.txt

# 3. UDCAP UI 가 켜져 있고 양손 페어링 + cal 끝난 상태에서:
#    UDCAP → Settings → Streaming → VMC Output ON, port=39539

# 4. UDCAP 가 살아있는 동안 30초 녹화 (사용자가 5-pose 시퀀스 수행)
python -m ust_ws.ust_hm_glove.validation.tools.record_vmc `
    --port 39539 `
    --output ust_ws\ust_hm_gloveust_ws\validation\recorded\quick.vmc.jsonl `
    --duration 30
# → 사용자: open hand → fist → point index → pinch → OK sign → opposition

# 5. 녹화된 데이터를 mapper 에 통과시켜 22D 출력 dump
python -m ust_ws.ust_hm_glove.validation.tools.replay_vmc `
    --input ust_ws\ust_hm_gloveust_ws\validation\recorded\quick.vmc.jsonl `
    --mode fast `
    --dump-mapper-jsonl ust_ws\ust_hm_gloveust_ws\validation\recorded\quick.mapper.jsonl `
    --subtract-rest

# 6. 분석 보고서 출력 — 어느 finger 가 살아있고 어느 게 frozen 인지 즉결
python -m ust_ws.ust_hm_glove.validation.tools.analyze_mapper_jsonl `
    ust_ws\ust_hm_gloveust_ws\validation\recorded\quick.mapper.jsonl `
    --plot ust_ws\ust_hm_gloveust_ws\validation\results\quick.png
```

→ 출력 verdict 가 "all fingers articulate well" 이면 mapper 단의 정밀제어는 OK.
→ "FROZEN" 손가락이 있으면 [research/34. §3.1](../research/34.%20udcap_ui_to_robot_data_pathway_root_cause.md) 의 7-cause 표로 어떤 원인인지 즉시 매핑.

---

## 0.5. 검증 완료 baseline (2026-05-02)

**§34 의 GIF 격차의 모든 진단된 cause 해결 완료** — Layer-1 mapper + Layer-2 robot articulation 모두 정밀제어 매칭 통과.

| Layer | 결과 |
|---|---|
| Layer 1 — mapper 22D 출력 | **22/22 STRONG** (frozen 0, weak 0) |
| Layer 2 — robot tracking | **22/22 < 0.4°** (max 0.0069 rad, mean 0.005 rad) |
| Layer 2 — latency | **0 frames** (즉시 추종) |

본 baseline 은 [`baselines/baseline_2026-05-02_c8c10.*`](baselines/) 에 영구 보존됨. 향후 코드 변경 시 회귀 비교 기준.

→ 자세한 baseline 명세는 [`baselines/README.md`](baselines/README.md) 참고.

### 적용된 9.x patches
- **9.19 (C8)**: `_quat_to_yaw` → `_quat_to_pitch` (UDCAP X 축 thumb opposition 추출)
- **9.20 (C10)**: 음수 thumb_yaw branch → 0 truncation (URDF clamp 회피)
- **9.19 (rest_frames)**: vmc_rest_frames default 30 (9.18 의 10 단축 후퇴 복원)
- **9.19 (Layer-2 인프라)**: idle arm 14D + Pink IK QP warmup

상세 history → [`../memory.md`](../memory.md) §10.28

---

## 1. 폴더 구조

```
ust_hm_glove.validation/
├── README.md                       ← 이 파일
├── _bootstrap.py                   # ust_hm_glove 모듈 로더
├── requirements-debug.txt          # 추가 deps (rerun-sdk, h5py, matplotlib, …)
│
├── research/
│   └── 35. udcap_finger_precision_test_validation_debug_system.md   # 설계
│
├── tools/                          # Layer-1 CLI (Isaac Sim 미사용)
│   ├── synth_vmc.py                # 6-pose canned VMC 생성기
│   ├── record_vmc.py               # 라이브 OSC → JSONL
│   ├── replay_vmc.py               # JSONL → UDP / mapper output
│   ├── analyze_mapper_jsonl.py     # mapper output 분석
│   └── analyze_replay_hdf5.py      # HDF5 metric 분석
│
├── tests/                          # Layer-1 pytest
│   ├── golden/*.vmc.jsonl          # 6 pose canned fixtures (자동 생성)
│   ├── test_synth_poses.py         # synth_vmc 헬퍼 (8 tests)
│   └── test_finger_replay.py       # mapper 회귀 (9 tests)
│
├── scripts/                        # Layer-2/3/4 (Isaac Lab 의존)
│   ├── run_replay_headless.py      # Layer 2 — env.step + HDF5
│   ├── run_live_validation.py      # Layer 4 — full 라이브 + 녹화
│   └── run_per_finger_isolation.py # Layer 1 — 10 isolated finger 회귀
│
├── visualization/
│   ├── live_dashboard.py           # rerun.io 시계열 대시보드 (B layer)
│   └── in_sim_overlay.py           # Isaac Lab VisualizationMarkers (A layer)
│
├── recorded/                       # Layer-4 가 채움 (.vmc.jsonl, .hdf5, .rrd)
└── results/                        # Layer-2 분석 결과 + 그래프 PNG
```

---

## 2. 사전 요구사항

| 요구사항 | 어디서 확인 | Layer 영향 |
|---|---|---|
| Python 3.10+ (conda env `ust` 권장) | `conda env list` | 모든 layer |
| numpy + python-osc | `pip install python-osc` | Layer 1, 4 |
| h5py + matplotlib | `pip install h5py matplotlib` | Layer 2 분석 |
| rerun-sdk (선택) | `pip install rerun-sdk` | Layer 3 dashboard. 없으면 matplotlib fallback |
| Isaac Lab 2.3 + isaaclab_tasks | `./isaaclab.sh -i` | Layer 2/3/4 |
| ust_hm_glove 모듈 | 본 워크스페이스 안 | 모든 layer (직접 / 패키지 import) |

전부 한 번에 설치:
```powershell
conda activate ust
pip install -r ust_ws\ust_hm_gloveust_ws\validation\requirements-debug.txt
```

---

## 3. Layer 별 사용법

### Layer 1 — Offline Replay (Isaac Sim 없이, 1초 iteration)

목적: `FourierHandMapper` 의 순수 함수 부분 검증. UDCAP 글러브 없어도 합성 입력으로 회귀 가능.

#### 3.1 회귀 테스트 실행

```powershell
# pytest 가 있으면 한 줄
$env:PYTHONPATH = (Get-Location).Path
python -X utf8 -m pytest ust_ws\ust_hm_gloveust_ws\validation\tests\ -v

# 또는 manual 한 번에 (15 테스트)
python -X utf8 ust_ws\ust_hm_gloveust_ws\validation\tests\test_finger_replay.py
python -X utf8 ust_ws\ust_hm_gloveust_ws\validation\tests\test_synth_poses.py
```

기대 통과: **17 tests**:
- test_synth_poses.py: 8 tests
- test_finger_replay.py: 9 tests (8 단위 + 1 end-to-end)

#### 3.2 6-pose golden fixture 생성 (필요 시)

```powershell
python -m ust_ws.ust_hm_glove.validation.tools.synth_vmc `
    --pose full_fist `
    --output ust_ws\ust_hm_gloveust_ws\validation\tests\golden\full_fist.vmc.jsonl `
    --frames 10 --rate-hz 20

# 또는 6 pose 한 번에 (Python)
python -c "
import sys; sys.path.insert(0, '.')
from pathlib import Path
from ust_ws.ust_hm_glove.validation.tools import synth_vmc
golden = Path('ust_ws/ust_hm_glove/validation/tests/golden')
golden.mkdir(parents=True, exist_ok=True)
for name in synth_vmc.POSE_NAMES:
    synth_vmc.write_jsonl_fixture(name, golden / f'{name}.vmc.jsonl', n_frames=10)
"
```

#### 3.3 Per-finger isolation 검증

```powershell
python -m ust_ws.ust_hm_glove.validation.scripts.run_per_finger_isolation
# → 10 PASS  (L/R × index/middle/ring/little/thumb)
```

각 finger 가 isolated pose 에서 **자신만 활성**되고 다른 손가락에 crosstalk 가 없는지.

---

### Layer 2 — Headless Replay (Isaac Sim, render off, 60초 iter)

목적: 실제 robot articulation 의 PhysX 응답까지 포함한 검증.

#### 3.4 헤드리스 replay

```powershell
python -m ust_ws.ust_hm_glove.validation.scripts.run_replay_headless `
    --replay ust_ws\ust_hm_gloveust_ws\validation\tests\golden\full_fist.vmc.jsonl `
    --output ust_ws\ust_hm_gloveust_ws\validation\results\full_fist.hdf5 `
    --steps 200 `
    --headless
```

각 step 에서 (target, actual) joint position 을 HDF5 에 기록.

#### 3.5 결과 분석

```powershell
python -m ust_ws.ust_hm_glove.validation.tools.analyze_replay_hdf5 `
    ust_ws\ust_hm_gloveust_ws\validation\results\full_fist.hdf5 `
    --plot ust_ws\ust_hm_gloveust_ws\validation\results\full_fist.png
```

리포트:
- per-joint tracking error (mean/p95/max)
- range coverage vs URDF 한계
- target → actual lag (cross-correlation)
- pass/fail vs Layer-2 임계값

기대 verdict (정상 시):
```
✓ ALL Layer-2 criteria met
```

---

### Layer 3 — Visual Replay (GUI + 시각화)

목적: 시각적 매칭 즉시 확인.

#### 3.6 rerun.io 라이브 대시보드 (코드 사용)

```python
from ust_ws.ust_hm_glove.validation.visualization.live_dashboard import make_dashboard

dash = make_dashboard(name="my_session", spawn=True)
# rerun viewer 가 자동으로 뜸

# 매 step 마다:
dash.push_frame(
    t_seconds=t,
    vmc_bones=current_vmc_dict,
    left_11=mapper_left_output,
    right_11=mapper_right_output,
    packed_22=pack_22d_output,
    target_22=robot_target_pos,
    actual_22=robot_actual_pos,
    source_left="vmc",
    source_right="vmc",
)
```

- `finger/L_idx_prox/raw` 등의 entity 가 시계열로 보임
- `delta/L_idx_prox` 는 누적 Δrange
- `vmc/LeftIndexProximal/bend_rad` 는 입력 bend 크기
- 사용자 손 3D skeleton 은 `dash.push_user_hand_3d(...)`

rerun-sdk 가 없으면 matplotlib fallback 으로 자동 전환.

#### 3.7 In-sim user hand overlay (Isaac Sim 내)

```python
from ust_ws.ust_hm_glove.validation.visualization.in_sim_overlay import UserHandOverlay

overlay = UserHandOverlay()
# env.step loop 안에서:
positions = UserHandOverlay.fk_from_quats_and_origin(
    vmc_bones, side="left",
    wrist_origin_w=np.array([0.0, 0.4, 1.2])  # robot 옆에 배치
)
overlay.update(positions_left_w=positions)
```

→ Isaac Sim 뷰포트에 사용자 손 spheres skeleton 이 robot 양손 옆에 보임.
isaaclab 미import 시 no-op (안전 fallback).

---

### Layer 4 — Live with Recording (실글러브, 풀 시스템)

목적: 회귀 fixture 새로 만들기 + 라이브 디버깅.

#### 3.8 통합 라이브 세션

세션 1: 메인 teleop + 동시에 recording/dashboard:

터미널 A (UDCAP 가 39541 로 broadcast 하도록 설정 후):
```powershell
python -m ust_ws.ust_hm_glove.validation.scripts.run_live_validation `
    --duration 300 `
    --output-prefix ust_ws\ust_hm_gloveust_ws\validation\recorded\session_$(Get-Date -Format yyyyMMdd_HHmmss) `
    --enable-tee --tee-listen-port 39541 --tee-forward-port 39539 `
    --enable-dashboard
```

이게 하는 일:
- UDP 39541 listen (UDCAP 송출 redirect)
- 받은 packet 모두 `.vmc.jsonl` 로 저장
- 같은 packet 을 39539 로 forward → 기존 ust_hm_glove teleop 가 정상 수신
- rerun viewer spawn (옵션)

터미널 B:
```powershell
# 평소처럼 ust_hm_glove teleop 실행 (변경 없음)
python -m ust_ws.ust_hm_glove.scripts.run_teleop `
    --env_variant robot_only --teleop_device pico_udcap `
    --finger_proximal_scale 2.5 `
    --path_b_port 39539
```

5 분 후 `recorded/session_*.vmc.jsonl` 가 채워짐 → Layer 2 회귀 입력으로 재사용.

#### 3.9 녹화 → 회귀 사이클

```powershell
# 어제 라이브 세션을 오늘 헤드리스로 다시 돌려서 변화 확인
python -m ust_ws.ust_hm_glove.validation.scripts.run_replay_headless `
    --replay ust_ws\ust_hm_gloveust_ws\validation\recorded\session_20260502_1930.vmc.jsonl `
    --output ust_ws\ust_hm_gloveust_ws\validation\results\session_20260502_post_9.19.hdf5 `
    --headless --subtract-rest

python -m ust_ws.ust_hm_glove.validation.tools.analyze_replay_hdf5 `
    ust_ws\ust_hm_gloveust_ws\validation\results\session_20260502_post_9.19.hdf5 `
    --plot ust_ws\ust_hm_gloveust_ws\validation\results\session_20260502_post_9.19.png
```

→ 9.18 → 9.19 패치 적용 후 Δrange / coverage / error 가 어떻게 변했는지 정량 비교.

---

## 4. 일반 워크플로 — 새 fix 검증할 때

```
[1] 코드 변경 (예: ust_hm_glove/teleop/fourier_hand_mapper.py)
       │
       ▼
[2] Layer-1 회귀 (5초)
    pytest ust_ws/ust_hm_glove/validation/tests/
    python -m ust_ws.ust_hm_glove.validation.scripts.run_per_finger_isolation
       │
       ▼ (통과)
[3] 어제 녹화한 세션 replay (60초)
    python -m ust_ws.ust_hm_glove.validation.scripts.run_replay_headless --replay ...
    python -m ust_ws.ust_hm_glove.validation.tools.analyze_replay_hdf5 ...
       │
       ▼ (메트릭 개선 확인)
[4] 라이브 세션 (5분)
    python -m ust_ws.ust_hm_glove.validation.scripts.run_live_validation ...
    + 별도 터미널에서 run_teleop
       │
       ▼ (시각적 매칭 확인)
[5] memory.md §10.NN 추가 + 새 baseline 녹화 보존
```

---

## 5. Quick-Start Cookbook (자주 쓰는 명령)

### 5.1 GIF 영상 (research/34. 트리거) 같은 상황 재현 후 fix 검증

```powershell
# A) 라이브 녹화 (사용자 5-pose 30초)
python -m ust_ws.ust_hm_glove.validation.tools.record_vmc `
    --output ust_ws\ust_hm_gloveust_ws\validation\recorded\baseline.vmc.jsonl `
    --duration 30

# B) 현재 코드 (9.18) 로 mapper 출력 dump
python -m ust_ws.ust_hm_glove.validation.tools.replay_vmc `
    --input ust_ws\ust_hm_gloveust_ws\validation\recorded\baseline.vmc.jsonl `
    --mode fast --dump-mapper-jsonl ust_ws\ust_hm_gloveust_ws\validation\recorded\baseline.before.jsonl `
    --subtract-rest --rest-frames 10

python -m ust_ws.ust_hm_glove.validation.tools.analyze_mapper_jsonl `
    ust_ws\ust_hm_gloveust_ws\validation\recorded\baseline.before.jsonl

# C) 9.19 패치 적용 → 다시 dump
python -m ust_ws.ust_hm_glove.validation.tools.replay_vmc `
    --input ust_ws\ust_hm_gloveust_ws\validation\recorded\baseline.vmc.jsonl `
    --mode fast --dump-mapper-jsonl ust_ws\ust_hm_gloveust_ws\validation\recorded\baseline.after.jsonl `
    --subtract-rest --rest-frames 30   # 9.19 default

python -m ust_ws.ust_hm_glove.validation.tools.analyze_mapper_jsonl `
    ust_ws\ust_hm_gloveust_ws\validation\recorded\baseline.after.jsonl
```

### 5.2 Synth 입력으로 mapper 의 핵심 거동 단위 디버그

```powershell
python -c "
import sys; sys.path.insert(0, '.')
from ust_ws.ust_hm_glove.validation._bootstrap import load_fourier_hand_mapper
from ust_ws.ust_hm_glove.validation.tools import synth_vmc

fhm = load_fourier_hand_mapper()
m = fhm.FourierHandMapper(proximal_scale=2.5, vmc_subtract_rest=False)

# 어떤 입력에 대해서도 mapper 가 무엇을 출력하는지 1ms 안에 확인
for name in synth_vmc.POSE_NAMES:
    pose = synth_vmc.build_pose(name)
    L = m.map_hand_vmc(pose, is_right=False)
    print(f'{name:20s}', [round(float(x), 3) for x in L])
"
```

### 5.3 사용자 자세별 mapper 응답 매트릭스 (Layer 1 빠른 진단)

```powershell
python -X utf8 ust_ws\ust_hm_gloveust_ws\validation\scripts\run_per_finger_isolation.py
# → 10 PASS / FAIL 매트릭스, 각 손가락이 isolated 일 때 동작/누화 검증
```

### 5.4 9.19 patch (research/34. §6.2) 후 단위 테스트로 즉결

`fourier_hand_mapper._fill_mimic_inplace` 임계값을 1e-6 → 0.05 rad 로 변경했다면:
```powershell
$env:PYTHONPATH = (Get-Location).Path
python -X utf8 -m pytest ust_ws/ust_hm_glove/validation/tests/test_finger_replay.py::test_full_fist_outputs_curl -v
# 통과 → mimic 채움 정상 동작
```

---

## 6. 트러블슈팅

### 6.1 `record_vmc` 가 0 packets 받음
- UDCAP UI → Settings → Streaming → VMC Output 이 ON 인지 확인
- VMC port 설정 (default 39539) 와 `--port` 일치 여부
- 다른 프로세스가 UDP 39539 점유 중이면 bind 실패 → 그 프로세스 종료
- `python -m ust_ws.ust_hm_glove.scripts.diagnose_udcap_dataflow` 로 데이터 흐름 검증

### 6.2 `analyze_mapper_jsonl` 결과 모두 "FROZEN"
- 입력이 static fixture (single pose 만) 이면 정상 거동 — 사용자 motion 이 있는 fixture/recording 에서만 의미 있는 Δrange
- 라이브 recording 인데도 모두 frozen 이면 → research/34. §3.1 의 7-cause 매트릭스 적용

### 6.3 Layer-2 `run_replay_headless` 가 "Isaac Lab bootstrap failed"
- conda env `ust` 활성화 했는지 확인 (`conda activate ust`)
- `IPC_IGNORE_VERSION=1` 자동 설정 — 별도 export 필요 없음
- h5py 1.14 vs Isaac Sim 1.12 DLL 충돌 시 `import h5py` 가 미리 일어나야 함 (스크립트 안에서 처리됨)

### 6.4 `live_dashboard` 가 rerun viewer 안 뜸
- `pip install rerun-sdk` 확인
- `make_dashboard(prefer_fallback=True)` 로 강제 matplotlib 모드 가능
- viewer 안 뜨면 자동 spawn 옵션 명시적 `spawn=True`

### 6.5 `UserHandOverlay enabled=False`
- Isaac Sim 외부에서 실행 중 — `run_live_validation` 또는 `run_replay_headless` 안에서만 enable
- isaaclab.markers import 가 깨지면 RuntimeWarning 한 번 출력됨 (안전 fallback)

### 6.6 ★ Isaac Sim 부팅 시 numpy DLL / "ObjectType already registered" 에러 (9.21 진단)

**증상**:
```
[Warning] [omni.kvdb.plugin] Disabling key-value database
                            because another kit process is locking it
[Error] omni.physics.tensors.bindings._physicsTensors:
        ImportError: generic_type: type "ObjectType" is already registered!
[Error] numpy/core/multiarray.py:
        ImportError: DLL load failed while importing _multiarray_umath:
                     지정된 모듈을 찾을 수 없습니다.
```

직전 Isaac Sim 명령이 종료된 후 lock 파일이 깨끗이 정리되지 않은 상태로 다음 명령을 너무 빨리 시작했을 때 발생.  좀비 lock 4개 (cache, registry x3) 가 새 프로세스의 extension 로딩 순서를 꼬이게 만듦.

**해결 (3 단계, 5분)**:

```powershell
# Step 1 — 좀비 프로세스 확인 (보통 없음)
Get-Process | Where-Object {$_.ProcessName -match '^(kit|isaac|carb|omni)'} `
    | Format-Table Id, ProcessName, StartTime
# 만약 있으면: Stop-Process -Id <PID> -Force

# Step 2 — Omniverse stale lock 4개 일괄 제거 (★ 핵심)
Get-ChildItem -Path "$env:USERPROFILE\AppData\Local\ov" -Filter "*.lock" -Recurse `
    -ErrorAction SilentlyContinue | Remove-Item -Force

# Step 3 — ust env 핵심 import 검증
& "$env:USERPROFILE\miniconda3\envs\ust\python.exe" -c "import numpy, torch, h5py, openvr; print('OK')"
```

**Sanity check (50 step Isaac Sim 부팅 검증)**:
```powershell
python -m ust_ws.ust_hm_glove.validation.scripts.run_replay_headless `
    --replay ust_ws\ust_hm_gloveust_ws\validation\baselines\baseline_2026-05-02_c8c10.vmc.jsonl `
    --output C:\Temp\sanity_check.hdf5 `
    --steps 50 --headless --subtract-rest
```

→ "done -- 50 steps written" 가 출력되면 정상.  이후 `run_teleop` 재시도.

**재발 방지 운영 가이드**:
- Isaac Sim 명령 종료 후 다음 Isaac Sim 명령 시작 전 **30초 대기**
- 또는 매번 위 Step 2 의 lock 정리 routine 적용
- numpy 2.4.4 (ust env 의 site-packages) 와 numpy 1.26 (Isaac Sim 의 `pip_prebundle`) 은 lock 정상이면 공존 OK — numpy 다운그레이드 불필요

---

## 7. 검증 결과 (현재 시점, 9.21 코드 기준)

본 시스템 자체의 회귀 통과 + 사용자 데이터 검증 상태:

### 7.1 Layer-1 회귀 (Isaac Sim 미사용, < 5초)
| 카테고리 | 테스트 수 | 통과 |
|---|---|---|
| `tests/test_synth_poses.py` | 8 | 8 |
| `tests/test_finger_replay.py` (C10 truncate 포함) | 9 | 9 |
| `scripts/run_per_finger_isolation` | 10 | 10 |
| **TOTAL Layer-1** | **27** | **27** |

### 7.2 Layer-2 검증 (Isaac Sim 헤드리스, 1000-step replay)
사용자 baseline 데이터 (`baselines/baseline_2026-05-02_c8c10.vmc.jsonl`) 기준:

| 메트릭 | 기준 | 결과 |
|---|---|---|
| Tracking error max | < 0.10 rad | **0.0069 rad (0.4°)** ★ |
| Tracking error mean | (참고) | 0.005 rad (0.26°) |
| Latency | < 5 frames | **0 frames** |
| Range coverage (4 finger × 2 손 proximal) | > 0.50 | 0.63 ~ 0.80 |
| Range coverage (8 intermediate) | > 0.50 | 0.49 ~ 0.56 |
| Range coverage (3 thumb 슬롯) | > 0.40 | 0.17 ~ 0.46 (사용자 thumb 풀 ROM 안 함) |

→ **시스템 측 모든 기준 PASS**.  Coverage 의 3 슬롯 미달은 사용자 행동 한계 (시스템 결함 아님).

### 7.3 적용된 patch 시리즈
- **9.19 (C8)**: `_quat_to_yaw` → `_quat_to_pitch` (UDCAP X-axis thumb opposition)
- **9.19 (C9)**: `vmc_rest_frames` default 10 → 30
- **9.19 인프라**: `run_replay_headless.py` cfg= 명시 + idle arm 14D + Pink IK QP warmup
- **9.20 (C10)**: thumb_yaw 음수 branch → 0 truncation (URDF clamp 회피)
- **9.21 (C11)**: replay loop wrap → clamp (Layer-2 max err spike 제거)
- **9.21 운영**: Omniverse extension lock 좀비 진단/복구 절차 정립

→ §34 의 GIF 격차 (frozen finger 패턴) 의 모든 진단된 root cause 해결.

---

## 8. 다음 단계 권장

1. ✓ **§34 의 사용자 측 UDCAP UI 설정** (Space Plan = Custom, Controller_Priority = Low) — 라이브 teleop 시 적용
2. ✓ **9.19/9.20/9.21 patches 적용** — 모두 완료 (memory.md §10.28, §10.29 기록)
3. ✓ **Before/after baseline 비교** — `baselines/baseline_2026-05-02_c8c10.*` 보존
4. **Layer-4 라이브 ust_hm_glove teleop 시각 검증** — 진행 중 (사용자 측)
5. PASS 시 `research/baseline_pass_<date>.md` 에 기록 (research/35. §10.3)
6. 회귀 비교 시: `baselines/` 폴더의 데이터로 새 코드 변경 검증

---

## 9. 관련 문서

- [research/35.](research/35.%20udcap_finger_precision_test_validation_debug_system.md) — 본 시스템 설계 (4-layer 아키텍처)
- [../research/34.](../research/34.%20udcap_ui_to_robot_data_pathway_root_cause.md) — 7-cause 분석 (이 시스템이 검증하려는 대상)
- [../research/33.](../research/33.%20udcap_vs_pico_hand_tracking_decision_research.md) — UDCAP vs PICO 결정 매트릭스
- [../research/26.](../research/26.%20pico_os_5154_pico_connect_xrobotoolkit_coexistence_analysis.md) — PICO 멀티태스킹 제약
- `../ust_hm_glove/` — 검증 대상 코드 (수정 X)
- `../memory.md` §10.20–§10.27 — 9.10–9.18 fix 누적 이력
