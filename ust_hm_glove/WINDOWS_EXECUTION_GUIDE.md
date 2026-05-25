# Fourier GR1T2 + 6-DoF Fourier Hand — Windows/SteamVR 실행 가이드

> 대상: `ust_ws/ust_hm_glove/` (본 폴더)
> 기준 OS: Windows 11 Pro 24H2 / RTX PRO 6000 Blackwell 96GB / Miniconda `ust` env
> SteamVR 스택: Virtual Desktop Streamer + Pico 4 Ultra + UDCAP VR Glove (기존 [ust_hm_glove](../ust_hm_glove/) 구성 그대로)
> 설계 근거: [research/31. ust_ws_g1_to_gr1t2_fourier_migration_guide_ko.md](../research/31.%20ust_ws_g1_to_gr1t2_fourier_migration_guide_ko.md)

---

## 0. TL;DR (30초 개요)

```powershell
# 0) 단위 테스트 (VR 없이 7/7 + 36/36 PASS)
conda activate ust
cd C:\develop\IsaacLab
python -m ust_ws.ust_hm_glove.scripts.smoke_test
python -m pytest ust_ws\ust_hm_glove\tests -q

# 1) idle 포즈 캘리브레이션 (처음 한 번만, ~1분)
python -m ust_ws.ust_hm_glove.scripts.calibrate_gr1t2_idle_pose --headless
#   → 결과를 teleop/gr1t2_retargeter.py 의 DEFAULT_*_POS/QUAT 로 치환

# 2) Pink IK 건강성 — 빈 테이블 + 합성 사인파 모션
python -m ust_ws.ust_hm_glove.scripts.run_teleop `
    --env_variant monitor --render_mode monitor --diag oscillate --debug_ik

# 3) 실 VR 텔레오퍼레이션 — Pico 4 Ultra + Virtual Desktop + UDCAP
python -m ust_ws.ust_hm_glove.scripts.run_teleop `
    --env_variant waist_enabled --render_mode steamvr_desktop --teleop_device pico_udcap
```

전체 과정은 **단일 PC, 단일 Isaac Sim 프로세스, Docker/ROS 2 미사용**. G1 환경 (`ust_hm_glove`) 과 **같은 Python 환경, 같은 SteamVR/VD 설치, 같은 트래커 바인딩** 을 공유한다.

---

## 1. 폴더 구성

```
ust_hm_glove/
├── WINDOWS_EXECUTION_GUIDE.md          ← 본 문서
├── __init__.py                          ← Gym 6종 등록 (Isaac Lab 로드 시에만)
├── kitchen_sorting_gr1t2_env_cfg.py     ← GR1T2 env 클래스 6종
│
├── teleop/
│   ├── __init__.py
│   ├── gr1t2_retargeter.py              ← SteamVRSnapshot → 36D action
│   ├── fourier_hand_mapper.py           ← UDCAP/VMC → 11 Fourier joints (폴백)
│   ├── waist_estimator.py               ← hips 트래커 → yaw/pitch/roll
│   └── gr1t2_udcap_device.py            ← Isaac Lab DeviceBase (36D)
│
├── scripts/
│   ├── __init__.py
│   ├── smoke_test.py                    ← VR 없이 7건 self-check
│   ├── run_teleop.py                    ← Isaac Lab 진입점
│   ├── calibrate_gr1t2_idle_pose.py     ← base_link 프레임 팔 T-pose 측정
│   └── validate_fourier_dex.py          ← DexPilot YAML/URDF 로딩 + 3 케이스 솔브
│
├── config/
│   ├── tracker_binding.json             ← 5 트래커 + 10-세그먼트 자동 매핑
│   ├── openvr_actions/actions.json      ← OpenVR Skeletal 매니페스트
│   └── dex_retargeting/
│       ├── fourier_left_dexpilot.yml    ← Isaac Lab 내장 사본
│       └── fourier_right_dexpilot.yml
│
└── tests/   (pytest — 36 케이스 전부 Isaac Sim 없이 실행)
    ├── test_fourier_hand_mapper.py
    ├── test_waist_estimator.py
    └── test_gr1t2_retargeter.py
```

재사용 모듈 (`vr_sampler`, `coord_transforms`, `fingertip_extractor`, `vmc_receiver`, `udcap_finger_mapper`, `enumerate_trackers`, `calibrate_forearm_offset`, `PICOInterventionInterface`) 은 **직접 import 로 [ust_hm_glove](../ust_hm_glove/)** 을 공유한다 — 동일 하드웨어에서 동작 검증 완료된 코드를 중복 작성하지 않기 위함.

---

## 2. 사전 요구 사항 (Prereq)

### 2.1 소프트웨어
- Windows 11 Pro (24H2 이상)
- Miniconda 3 `ust` 환경 — `C:\Users\<user>\miniconda3\envs\ust\`
  (memory.md §4.1 의 패키지 버전으로 구성된 환경)
- Isaac Lab `main` + Isaac Sim 5.1 editable install
- SteamVR + Virtual Desktop Streamer + UDCAP Driver (기존 [ust_hm_glove](../ust_hm_glove/) §2 그대로)
- Pico OS 5.14+ (Enhanced Forearm 모드 지원)

### 2.2 하드웨어
- NVIDIA RTX Blackwell (또는 동급)
- Pico 4 Ultra + 5× Pico Motion Tracker + UDCAP VR Glove × 2
- Virtual Desktop 라이선스

### 2.3 필수 패키지 확인

```powershell
conda activate ust
python -c "import torch, numpy, h5py, pinocchio; import pink.tasks; import openvr; import dex_retargeting; print('deps OK')"
```

없는 패키지가 있으면 [ust_hm_glove\setup_steamvr_env.ps1](../ust_hm_glove/setup_steamvr_env.ps1) 을 재실행해도 된다 — 4-phase 설치는 GR1T2 에도 동일하게 충분하다.

### 2.4 SteamVR / Virtual Desktop 설정
`ust_hm_glove` 의 설정을 그대로 사용. 핵심만:
- SteamVR → **Manage Add-Ons**: `Virtual Desktop Streamer` ON, `udcap` ON, `prism` OFF
- VD Streamer OPTIONS: `Forward Trackers to SteamVR` ON, `Full Body Tracking` ON
- SteamVR → **Settings → Developer**: Set SteamVR as OpenXR runtime

### 2.5 Isaac Nucleus 경로 확인
GR1T2 USD 는 Isaac Nucleus 에 의존한다.
```powershell
python -c "from isaaclab_assets.robots.fourier import GR1T2_HIGH_PD_CFG; print(GR1T2_HIGH_PD_CFG.spawn.usd_path)"
# 예상 출력: omniverse://.../Robots/FourierIntelligence/GR-1/GR1T2_fourier_hand_6dof/GR1T2_fourier_hand_6dof.usd
```

---

## 3. 최초 1회 설정

### 3.1 트래커 역할 바인딩 (재사용)
G1 경로의 [tracker_binding.json](./config/tracker_binding.json) 과 동일한 매핑을 사용 — 이미 복제되어 있음.
재생성이 필요하면 G1 스크립트를 그대로 호출:
```powershell
python -m ust_ws.ust_hm_glove.scripts.enumerate_trackers `
    --output ust_ws\ust_hm_glove\config\tracker_binding.json
```

### 3.2 팔뚝 → 손목 오프셋 캘리브
(사용자 실체치) GR1T2 과 G1 의 forearm-to-wrist 오프셋은 **사용자 본인의 팔 길이에 의존** — 로봇과 무관.
```powershell
python -m ust_ws.ust_hm_glove.scripts.calibrate_forearm_offset
```
결과 JSON 은 `ust_hm_glove/config/forearm_offset.json` 과 동일 형식. 값 (보통 0.24 ~ 0.30 m) 을 기억해 두고 나중에 `--forearm_wrist_offset` 로 넘기면 된다.

### 3.3 GR1T2 idle pose 캘리브 (**필수**, GR1T2 전용)
기본값 `DEFAULT_LEFT_POS=(-0.20, 0.00, 1.05)` 등은 Isaac Lab 2.3 GR1T2 의 근사치다. 당신의 USD / URDF 한계에 정확히 맞추려면:

```powershell
python -m ust_ws.ust_hm_glove.scripts.calibrate_gr1t2_idle_pose `
    --headless --settle_steps 300 `
    --output ust_ws\ust_hm_glove\config\gr1t2_idle_pose.json
```

스크립트가 스폰 후 300 PhysX step 을 대기한 뒤 `left_hand_pitch_link` / `right_hand_pitch_link` 의 base_link-local pose 를 측정해 JSON + 터미널에 Python 리터럴을 출력한다. 이 값을 [teleop/gr1t2_retargeter.py](./teleop/gr1t2_retargeter.py) 상단의 `DEFAULT_*_POS/QUAT` 에 붙여넣거나, `GR1T2FourierRetargeterCfg(idle_left_pos=...)` 로 전달하라.

### 3.4 Fourier DexPilot YAML / URDF 검증 (선택)
DexPilot 이 URDF 를 로딩해 합성 손가락 포즈 3종(T-pose / 주먹 / pinch) 에 대해 수렴하는지 빠르게 확인:

```powershell
# URDF 는 Pink IK 가 최초 기동 시 Isaac Lab temp dir 에 생성.
# 별도 위치에 옮겨두었다면 해당 경로를 --urdf_dir 로 지정.
python -m ust_ws.ust_hm_glove.scripts.validate_fourier_dex `
    --urdf_dir C:\Users\<user>\AppData\Local\Temp
```

실패하면 (a) URDF 파일 누락, (b) `dex_retargeting` 미설치, (c) mesh 경로 깨짐 중 하나. GR1T2 run_teleop 은 이 때 자동으로 `FourierHandMapper` 폴백으로 내려간다 — 손가락 품질만 저하되고 파이프라인은 살아있다.

---

## 4. 실행 시나리오

### 4.1 시나리오 A — Pink IK 건강성 체크 (VR 없이)
빈 씬에서 사인파 모션을 바로 로봇에 넘겨 Pink IK + env.step 파이프라인이 로봇을 실제로 움직이는지 확인.

```powershell
python -m ust_ws.ust_hm_glove.scripts.run_teleop `
    --env_variant monitor --render_mode monitor `
    --diag oscillate --debug_ik
```
- **통과 기준**: Isaac Sim 창에서 로봇 양 손목이 0.5 Hz 사인파로 ±10 cm 진동.
- **실패하면**: `--debug_ik` 로그의 "FrameTask residual" 메시지 확인. 대개 URDF 변환 실패 또는 프레임 이름 오타.

### 4.2 시나리오 B — 실 VR + UDCAP (기본)
```powershell
python -m ust_ws.ust_hm_glove.scripts.run_teleop `
    --env_variant base --render_mode steamvr_desktop `
    --teleop_device pico_udcap
```
- 동작 흐름:
  1. SteamVR + VD 가 이미 기동 중이어야 함 (`enumerate_trackers` 에서 5 트래커 보임).
  2. Isaac Sim 창이 VD "Desktop Theater" 로 VR 내부에 2D 로 투영됨.
  3. 사용자가 팔 / 손가락을 움직이면 GR1T2 가 따라감.

### 4.3 시나리오 C — WaistEnabled (허리까지 IK 제어)
```powershell
python -m ust_ws.ust_hm_glove.scripts.run_teleop `
    --env_variant waist_enabled --render_mode steamvr_desktop `
    --teleop_device pico_udcap
```
- `pink_controlled_joint_names` 에 `waist_yaw/pitch/roll` 3개가 추가되어 손목 타겟이 팔만으로 도달 불가할 때 솔버가 자동으로 허리를 사용.
- `GR1T2FourierUDCAPDevice` 가 `WaistEstimator` 를 켜서 VD hips 트래커 기반 torso 쿼터니언을 posture target 으로 추적 (v0.1: null-space posture 로만 사용, 액션 채널에는 없음).

### 4.4 시나리오 D — 데이터 수집 (HG-DAGGER 용 데모)
`ust_260220/scripts/record_demos.py` 가 GR1T2 env 를 인식하도록 teleop_device import 한 줄을 수정하면 기존 스크립트를 그대로 사용할 수 있다.

```python
# ust_260220/scripts/record_demos.py 의 device import 줄을 아래처럼 교체
from ust_ws.ust_hm_glove.teleop.gr1t2_udcap_device import (
    GR1T2FourierUDCAPDevice as DeviceCls,
    GR1T2FourierUDCAPDeviceCfg as DeviceCfgCls,
)

# 그리고 env_id 를 아래로 변경
env_id = "Isaac-KitchenSorting-GR1T2-Fourier-DataCollect-v0"
```

### 4.5 시나리오 E — HG-DAGGER 개입 루프
`ust_260220/scripts/run_hg_dagger.py` 도 동일하게 device / env_id 2개 줄만 교체. `GR1T2InterventionInterface` 는 `PICOInterventionInterface` 를 상속하므로 양 grip/양 trigger/양 menu 버튼 debounce 로직이 완전히 동일.

### 4.6 시나리오 F — BC-RNN / Ensemble / Conformal
`scripts/train_bc_rnn.py`, `train_ensemble.py`, `calibrate_conformal.py`, `run_uncertainty_loop.py` 는 **완전히 로봇 무관** — HDF5 metadata 의 `action_dim` 을 읽어 알아서 36D 로 네트워크를 구성한다. 데이터셋 파일만 분리 (예: `gr1t2_kitchen_v1.hdf5`) 하면 된다.

---

## 5. 진단 플래그 (G1 과 동일 의미, GR1T2 에도 유효)

| 플래그 | 용도 | 통과 조건 → 확정되는 사실 |
|--------|------|---------------------------|
| `--diag oscillate --debug_ik` | 합성 사인파만 사용, VR 완전 무시 | 로봇이 사인파로 흔들림 → **Pink IK + env.step 정상, 문제는 VR 매핑** |
| `--debug_ik` | `show_ik_warnings=True` 강제 | 콘솔에 잔차 값 출력 → 어떤 타겟이 워크스페이스 밖인지 확인 |
| `--freeze_orientation --debug_ik` | forearm 쿼터니언 무시, idle quat 전송 | 위치만 추종되면 → 방향 매핑 문제 (quat 회전 변환) |

권장 순서:
1. `--diag oscillate --debug_ik` → 파이프라인 OK 확인.
2. 그 다음 실 VR 로 이동, `--debug_ik` 켜고 첫 세션 진행.
3. 방향 이슈가 의심되면 `--freeze_orientation` 추가.

---

## 6. 데이터 플로우 요약 다이어그램

```
Pico 4 Ultra ──Wi-Fi 6E─▶ Virtual Desktop Streamer
   ▲                         │
   └─5 Pico Motion Tracker    │ (Forward Trackers, Full Body)
                              ▼
                         SteamVR (OpenVR + OpenXR)
                              ▲
UDCAP VR Glove L/R ──USB──▶ UDCAP Driver (SteamVR Add-on)

                              ▼
  ┌────────── Python 3.11 (Isaac Lab) ──────────┐
  │  SteamVRSampler (pyopenvr 120 Hz)            │
  │    └→ snapshot: HMD + 5 trackers +           │
  │       31-bone skeletal + controllers         │
  │                                              │
  │  GR1T2FourierSteamVRRetargeter  (본 폴더)    │
  │    ├── forearm→wrist, SVR→IL 변환            │
  │    ├── right wrist Z180 correction           │
  │    ├── DexPilot (Fourier YAML) [주]          │
  │    ├── FourierHandMapper (skeletal/VMC) [폴백] │
  │    └── WaistEstimator → null-space posture   │
  │                                              │
  │  36D action [L_pos3+L_quat4, R_pos3+R_quat4, │
  │              22 hand joints in §2.4 순서]    │
  │                                              │
  │  GR1T2FourierUDCAPDevice (DeviceBase)        │
  │    └→ env.step()                             │
  │                                              │
  │  ManagerBasedRLEnv                           │
  │    ├── Pink IK: palm-frame FrameTask × 2     │
  │    │     + NullSpacePostureTask              │
  │    │     + waist in WaistEnabled             │
  │    └── GR1T2_HIGH_PD_CFG articulation        │
  └──────────────────────────────────────────────┘
```

---

## 7. FAQ / 알려진 증상

| # | 증상 | 원인 | 조치 |
|---|------|------|------|
| 1 | `ModuleNotFoundError: carb` (smoke_test 실행 시) | 평범한 Python 은 Isaac Sim 없음 | 정상 — `env_cfg_import` 는 `SKIP` 으로 표시되지만 테스트는 PASS 처리됨. 실 VR 실행 시에는 `conda activate ust` + Isaac Sim 경유 |
| 2 | `FATAL: dex-retargeting solver unavailable` | URDF 변환 실패 / mesh 누락 | 자동으로 `FourierHandMapper` 폴백. `validate_fourier_dex.py` 로 사전 확인 권장 |
| 3 | 로봇이 움직이지 않음 | Pink IK 수렴 실패 또는 VR 매핑 오류 | §5 3-stage 격리법 순서대로 |
| 4 | 부팅 ~150 s | robocasa kitchen 에셋 우세 | 로봇 교체로 해결 안 됨 — 빈 씬 (`--env_variant monitor`) 에서 먼저 검증 |
| 5 | `convert_usd_to_urdf` tmp 경로가 재부팅마다 초기화 | `tempfile.gettempdir()` 가 Windows 휘발성 | env_cfg 의 `temp_urdf_dir` 을 영구 경로(예: `./data/gr1t2_urdf_cache/`) 로 덮어쓰기 |
| 6 | 손가락이 과도하게 curl 되거나 반대로 움직임 | DexPilot YAML 의 `scaling_factor`(기본 1.2) 가 사용자 손에 비해 큼/작음 | `fourier_right_dexpilot.yml` 을 복사 후 `scaling_factor: 1.0~1.5` 로 튜닝 |
| 7 | 오른쪽 손목이 거울 반대 방향으로 회전 | `right_wrist_z180` 미적용 | `GR1T2FourierUDCAPDeviceCfg(right_wrist_z180=True)` 기본값 유지 |
| 8 | 허리가 전혀 움직이지 않음 | 기본 env 는 `enable_waist_dof=False` | `--env_variant waist_enabled` 사용 |
| 9 | G1 회귀 실패 | 잘못된 import 경로 (ust_hm_glove 변경) | `ust_hm_glove` 은 건드리지 않는 게 원칙. `smoke_test` 를 CI 로 추가 |
| 10 | Isaac Lab gym 등록 2회 중복 경고 | Python 재import | `_register()` 가 중복 체크 → 무해, 무시 |
| 11 | `gymnasium.error.NameNotFound: Environment 'Isaac-KitchenSorting-GR1T2-Fourier-WaistEnabled' doesn't exist` | `kitchen_sorting_gr1t2_env_cfg.py` import 가 실패해 Gym 등록이 안 됨. 예전에는 `__init__.py::_register()` 내부에서 silent skip 되어 traceback 이 숨겨졌음 | **run_teleop.py v2** (2026-04-22 2nd fix) 는 이제 gym.register 사이드이펙트에 의존하지 않고 `kitchen_sorting_gr1t2_env_cfg` 모듈을 직접 import → 실패 시 traceback 이 main() 에 바로 도달. 추가로 `__init__.py` 의 `_register()` 에서 실패를 stdout/stderr + `config/last_import_error.log` 3중 채널로 기록. 여전히 원인을 못 찾으면: `python -m ust_ws.ust_hm_glove.scripts.diagnose_env_cfg --headless` 로 의존성 체인을 한 줄씩 탐침. **과거 실제 사례**: `DampingTask` 를 `isaaclab.controllers.pink_ik` 에서 import → 실제로는 `pink.tasks` 에 존재 (2026-04-22 1차 fix 에서 해결). |
| 12 | `0xc0000139` Windows fatal exception + `isaacsim.sensors.rtx DLL load failed while importing _generic_model_output` + `omni.sensors.nv.{lidar,radar} plugin preload failed` | Isaac Sim 5.1 의 일부 센서 확장이 `generic_mo_io.dll` 심볼 충돌로 로딩 실패 (memory.md §3.8 h5py DLL 사례의 동형 문제; 현 케이스는 센서 RTX 경로) | **비치명적** — Isaac Sim 은 `Simulation App Startup Complete` 까지 계속 부팅한다. 본 마이그레이션은 LiDAR/Radar 센서를 사용하지 않으므로 실행에 지장 없음. 완전 제거를 원하면 `apps/isaaclab.python.xr.openxr.kit` 에서 `omni.sensors.nv.lidar`, `omni.sensors.nv.radar`, `isaacsim.sensors.rtx` 확장을 비활성화. |
| 13 | `AttributeError: 'DampingTask' object has no attribute 'set_target_from_configuration'` at `pink_ik.py:102` during `ManagerBasedRLEnv(cfg=env_cfg)` | Isaac Lab 내장 `pickplace_gr1t2_env_cfg.py` 는 `variable_input_tasks` 에 `pink.tasks.DampingTask(cost=0.5)` 를 포함. 그러나 Isaac Lab 0.48.0 의 `PinkIKController.__init__` 은 `NullSpacePostureTask` 외 모든 task 에 `set_target_from_configuration(...)` 을 **무조건 호출**. `DampingTask` 는 이 메서드를 구현하지 않아 AttributeError 발생 | **2026-04-22 3차 fix**: `kitchen_sorting_gr1t2_env_cfg.py::FourierActionsCfg` 의 `variable_input_tasks` 에서 `DampingTask` 를 제거 (`NullSpacePostureTask` 만으로도 충분한 regularization 제공). G1 (`ust_260220/kitchen_sorting_env_cfg.py`) 및 내장 G1 Inspire cfg 도 애초에 `DampingTask` 를 쓰지 않음 — 이들이 작동하는 증거. Isaac Lab upstream 이 `pink_ik.py` 에 `DampingTask` 특수 케이스를 추가하는 날이 오면 재도입 가능. |
| 14 | Gym registry 가 비어 있음 (`Registered GR1T2 env IDs: []`) 인데 env_cfg import 성공 | `_register()` 가 순환 import 중에 호출되어 for-loop 의 `gym.register()` 가 silent 실패 | **2026-04-22 3차 fix**: `_register()` 를 **class-object 직접 매핑** (문자열 키 indirection 제거) + **3채널 loud logging** (stdout + stderr + config/last_import_error.log) 으로 재작성. 등록 결과를 `registered=N skipped=N failed=N` 요약 출력. `run_teleop.py` 는 Isaac Sim 부팅 후 `register_envs_now()` 로 재시도 + `gym.registry` 요약. 어쨌든 run_teleop 은 gym.spec 에 의존하지 않으므로 primary 경로는 영향 없음. |
| 15 | `ValueError: 'left_wrist_yaw_link' is not in list` at `ObservationManager._prepare_terms` | G1 의 `ObservationsCfg` 는 EEF 관측을 `left_wrist_yaw_link` / `right_wrist_yaw_link` 로 조회. GR1T2 는 해당 body 를 노출하지 않음 (palm 구성이 `left_hand_pitch_link` → `left_hand_roll_link`) | **2026-04-22 4차 fix**: 신규 `FourierObservationsCfg` 클래스 (`kitchen_sorting_gr1t2_env_cfg.py`) 가 G1 ObservationsCfg 의 구조는 유지하되 모든 EEF link 참조를 `GR1T2_EEF_LINK_NAMES = {"left": "left_hand_roll_link", "right": "right_hand_roll_link"}` 로 치환 (Isaac Lab 내장 `pickplace_gr1t2_env_cfg.py` 와 동일 선택). 관측 차원/순서는 불변이라 데이터셋 구조 호환성 유지. |
| 16 | 로봇이 전혀 움직이지 않고 매 프레임 `Warning: IK quadratic solver could not find a solution!` + `Error: 'osqp' does not seem to be installed (found solvers: [])` 스팸 | **진짜 원인**: `qpsolvers 4.11.0` 의 osqp 어댑터는 `from osqp import OSQP, SolverStatus` 를 요구 — 그러나 `SolverStatus` 는 osqp **1.0+** 에만 존재. `ust` 환경은 `osqp==0.6.7.post3` 로 고정 (isaacsim-core 의 hard pin). 결과적으로 qpsolvers 가 osqp 어댑터 import 에서 `ImportError` 로 실패 → `available_solvers: []` → Isaac Lab `pink_ik.py:224` 의 하드코딩 `solver="osqp"` 가 매 프레임 에러 → 현재 관절 위치 반환 → 로봇 정지. (memory.md §3.7 의 "qpsolvers 경고는 무해" 기록은 G1 에서 우연히 solve 가 거의 안 필요했을 뿐이라 **오류 평가**였음.) | **2026-04-22 5차 fix** — **compat shim 방식 (pip 의존 충돌 없음)**: `ust_ws/ust_hm_glove/teleop/_osqp_compat.py` 가 osqp 0.6 의 C enum 정수 코드로 `SolverStatus` `IntEnum` 을 구성해 `osqp.SolverStatus` 네임스페이스에 주입. `qpsolvers.solvers.osqp_` 가 require 하는 `SolverStatus.OSQP_SOLVED == 1` 비교가 정상화 → `available_solvers: ['osqp']` 복원. shim 은 `kitchen_sorting_gr1t2_env_cfg.py`, `scripts/run_teleop.py`, `scripts/diagnose_env_cfg.py` 세 진입점의 top level 에서 `apply()` 호출. isaacsim-core 의 `osqp==0.6.7.post3` pin 은 그대로 유지. |
| 17 | 로봇이 초기 스폰 시 **오른쪽으로 90° 틀어져** 보이고, 사용자가 앞으로 손을 뻗으면 로봇이 옆으로 움직임 | G1 에서 상속받은 스폰 rot `(0.7071, 0, 0, 0.7071)` (yaw+90°) 이 GR1T2 base_link 의 정면(+X) 을 world +Y 로 돌려 버림. `svr_to_isaaclab` 은 world-aligned 좌표를 주는데 Pink IK 는 이를 base_link-local 로 해석 → 90° azimuth offset | **2026-04-22 6차 fix**: `_fourier_robot_articulation()` 의 `rot` 을 `(1.0, 0.0, 0.0, 0.0)` (identity) 로 변경. base_link axes 가 world axes 와 일치하므로 retargeter 의 world-frame target 이 base_link-local 로 정확히 해석됨. 부작용: G1 에서 상속받은 테이블(y=+0.55) / 빈 이 로봇 왼쪽에 배치됨 — 정밀 씬 정렬은 별도 sub-class 에서 수행 권장 (`KitchenSortingGR1T2EnvCfg` 서브클래싱 후 scene 에서 테이블/빈 pos 를 `+X` 로 교체) |
| 18 | `sources={..., 'left_finger': 'idle', 'right_finger': 'idle'}` — UDCAP 글러브 착용해도 손가락이 로봇으로 전달 안 됨 | **`ust_ws/ust_hm_glove/config/openvr_actions/actions.json` 의 `default_bindings` 가 `[]`**. SteamVR 은 binding file 없이는 `/actions/teleop/in/skeleton_left`/`right` 를 어떤 드라이버에도 연결하지 않음. 그 결과 `getSkeletalBoneData()` 가 내부 에러 → `vr_sampler.py:378` 의 `except Exception: pass` 가 silent 로 삼켜 `snapshot['hands']['left']` 가 `None` 으로 고정. | **2026-04-22 6차 fix**: 신규 `config/openvr_actions/bindings_index.json` 에 Valve Index 프로필(UDCAP 이 에뮬레이션하는 컨트롤러 계열) 용 binding 작성 — 양손 skeleton action 을 `/user/hand/{left,right}/input/skeleton/*` 에 라우팅 + trigger/grip vector1 action 도 함께 등록. `actions.json` 의 `default_bindings` 에 `controller_type: knuckles` → `binding_url: bindings_index.json` 추가. `GR1T2FourierUDCAPDevice.advance()` 의 첫-프레임 diagnostic 에 `hands.{left,right}` 의 bones/fingerCurls/fingerSplays shape 까지 dump 해서 여전히 None 이면 어느 경로에서 막히는지 보이도록. |
| 19 | `dex-retargeting solver unavailable (URDF path C:\tmp\GR1_T2_*_hand.urdf does not exist)` → `FourierHandMapper` 폴백 | Isaac Lab 내장 `GR1TR2DexRetargeting` 은 AWS 에서 손만의 URDF 를 다운받고 YAML 의 `urdf_path` 를 동적 주입. 우리 retargeter 는 이 로직을 포팅하지 않음. | **2026-04-22 현재 상태로 유지** (follow-up) — Issue #18 이 해결되어 skeletal 데이터가 실제로 흐르기 시작하면, `FourierHandMapper` 의 skeletal 브랜치(`map_hand_skeletal`) 가 동작해 손가락이 움직임. DexPilot 는 더 정확하지만 not blocking. 향후 `teleop/gr1t2_retargeter.py::_build_dex_solver` 에서 Isaac Lab 의 `gr1_t2_dex_retargeting_utils.GR1TR2DexRetargeting` 클래스를 직접 차용하면 해결 가능 — AWS URDF 다운로드 + YAML 경로 주입 + 솔버 빌드 전부 처리됨. |
| 20 | 로봇 팔/허리는 움직이는데 **손가락이 전혀 안 움직임** 상태 지속. 진단 블록에서 `hands.left : None (sampler returned no skeletal data)` + `ctrls.left : pose=... trigger=0.00 grip=0.00 menu=False` (**컨트롤러는 populate, skeletal 만 None**) | `default_bindings` + `bindings_index.json` 을 추가했어도 여전히 skeletal 실패. 세 가지 중 하나: (a) UDCAP 의 `controller_type` 이 `"knuckles"` 가 아님 → binding 매칭 실패. (b) UDCAP 드라이버가 Skeletal Input 2.0 프로토콜 자체를 구현 안 함 (pose/trigger/grip 만 emit). (c) SteamVR 이 manifest cache 재로드 안 함. | **2026-04-22 7차 fix — 진단 + button-grip 폴백 2중 대응**: <br>(1) `gr1t2_udcap_device.py::_probe_openvr_inventory()` + `_probe_openvr_skeletal()` 를 `start()` 말미에 호출 — (a) 모든 컨트롤러의 실제 `controller_type`, serial, render model 덤프, (b) skeletal action handle 에 대해 `getSkeletalActionData(bActive, activeOrigin)` + `getSkeletalTrackingLevel` + `getBoneCount` 를 개별 호출해 **어느 단계에서 실패하는지 구체 에러와 함께 출력**. 이 정보로 세 가지 원인 중 무엇인지 판별 가능. <br>(2) `FourierHandMapper.map_from_controller_buttons(trigger, grip, thumb_touch)` 신규 — trigger → 검지/중지 curl, grip → 약지/새끼 curl, 둘 중 큰 값이 threshold 초과 시 엄지 pinch. Retargeter 의 **finger source priority chain 을 5단계로 확장**: DexPilot → Skeletal → VMC → **Button-grip (신규)** → idle. UDCAP 드라이버가 skeletal 미지원이어도 **trigger 를 잡으면 손가락이 오므라지는 binary grip 제스처** 는 즉시 작동. 기본 ON (`enable_button_grip_fallback=True`). |
| 21 | 7차 fix 후에도 손가락 여전히 안 움직임. 진단 결과 **3가지 사실 확정**: (A) UDCAP 은 `controller_type='knuckles'` 로 올바르게 에뮬레이션 됨, (B) Skeletal action 은 `getBoneCount=31` 이지만 `bActive=False` (= 드라이버가 Skeletal Input 2.0 을 emit 안 함 — LucidVR-family 드라이버 공통), (C) legacy `ctrls.*.trigger = 0.00` (= `vr_sampler` 의 legacy `getControllerState()` API 가 knuckles 에뮬레이터에 대해 0 반환 → button-grip 폴백 조차 발화 안 함). | **2026-04-23 8차 fix — per-finger curl action + action-API trigger/grip**: <br>LucidVR/UDCAP 계열 knuckles 에뮬레이터가 실제로 emit 하는 것은 **Valve Index per-finger curl 입력**(`/user/hand/{l,r}/input/finger/{thumb,index,middle,ring,pinky}`, 각 vector1 0~1) 이라는 사실을 이용. <br>(1) **`actions.json`** 에 10개 finger-curl vector1 action 추가 (+기존 trigger/grip 4개). <br>(2) **`bindings_index.json`** 에 `/user/hand/*/input/finger/*` → finger_curl_* 으로 `force_sensor` mode 바인딩. <br>(3) **`FourierHandMapper.map_from_finger_curls(curls_5)`** 신설 — thumb/index/middle/ring/pinky 각 독립 제어 + thumb curl 로 opposition yaw 연동 (pinch 제스처). <br>(4) **`GR1T2FourierUDCAPDevice`** 에 action handle 캐싱 + 매 프레임 `getAnalogActionData` 로 14개 action(4 trigger/grip + 10 finger) 읽기. **`vr_sampler.py` 의 legacy `getControllerState()` API 우회** — sampler 는 건드리지 않음. <br>(5) **Retargeter priority chain 6단계로 확장**: DexPilot → Skeletal → **Finger-curl action (신규, UDCAP 의 primary 소스)** → VMC → Button-grip (action-API trigger/grip 을 legacy 보다 우선) → idle. <br>(6) 신규 진단 블록 `--- OpenVR action values probe ---` 와 `--- action-API input diagnostic ---` 추가 — trigger 를 누르면 curl 값이 실시간으로 찍히는지 바로 확인 가능. **UDCAP 은 이제 skeletal 없이도 5-DoF/손 finger 제어가 동작**. |

---

## 8. 검증 상태 (2026-04-21)

| 항목 | 상태 | 비고 |
|------|------|------|
| `py_compile` 14/14 파일 | ✅ PASS | Python 3.11.15 |
| `scripts/smoke_test.py` 7건 | ✅ 7/7 PASS | Isaac Sim 없이 실행 |
| `pytest tests/` 36건 | ✅ 36/36 PASS | 단위 테스트 |
| `ust_hm_glove` G1 smoke 6건 (regression) | ✅ 6/6 PASS | 기존 G1 경로 영향 없음 |
| Isaac Sim 런타임 (`run_teleop.py --diag oscillate`) | ⏳ 실 하드웨어 검증 대기 | memory.md 에 결과 기록할 것 |
| 실 VR + UDCAP 텔레오퍼레이션 | ⏳ 대기 | Pico 4 Ultra + VD + 5 tracker 페어링 필요 |
| 30 데모 수집 + BC-RNN 학습 | ⏳ 대기 | §4.4 참고 |

---

## 9. 마이그레이션 체크리스트 (research/31 기반)

### 9.1 완료
- [x] `ust_hm_glove/` 폴더 생성 + `__init__.py` 6 env 등록
- [x] `teleop/fourier_hand_mapper.py` — 11-joint Fourier 폴백
- [x] `teleop/waist_estimator.py` — hips 트래커 기반 torso 쿼터니언
- [x] `teleop/gr1t2_retargeter.py` — 36D action 조립 (palm frame, right Z180, DexPilot 주)
- [x] `teleop/gr1t2_udcap_device.py` — DeviceBase, 36D advance()
- [x] `kitchen_sorting_gr1t2_env_cfg.py` — 6 env 클래스 (base / WaistEnabled / Vision / Monitor / VR / DataCollect)
- [x] `config/` 복제: `tracker_binding.json`, `openvr_actions/actions.json`, `dex_retargeting/fourier_*_dexpilot.yml`
- [x] `scripts/run_teleop.py` — GR1T2 env 선택 + 3 진단 플래그
- [x] `scripts/calibrate_gr1t2_idle_pose.py` — idle 포즈 자동 측정
- [x] `scripts/validate_fourier_dex.py` — DexPilot 솔버 합성 테스트
- [x] `tests/` pytest 36건 — 100% PASS
- [x] `scripts/smoke_test.py` 7건 — 100% PASS (VR 없이)
- [x] G1 회귀 테스트 — 6/6 PASS 유지

### 9.2 사용자 작업 대기
- [ ] `calibrate_gr1t2_idle_pose.py` 실행 후 idle pose 상수 교체
- [ ] VD/SteamVR 재기동 후 실 VR 세션 첫 시동
- [ ] §6.4 씬 치수 재튜닝 (테이블 높이 등)
- [ ] `record_demos.py` / `run_hg_dagger.py` 한 줄 패치 (device import + env_id)
- [ ] GR1T2 전용 HDF5 데이터셋 30 에피소드 수집
- [ ] BC-RNN 재학습 + Conformal 재보정

### 9.3 선택 (장기)
- [ ] G1 경로 `_legacy_g1/` 로 아카이브 (GR1T2 가 안정화되면)
- [ ] 액션 tensor 39D 확장 (waist 명시 제어, research/31 §11.2)
- [ ] GR00T N1.5 post-training (research/31 §11.4)

---

## 10. 관련 문서

- [research/31. ust_ws_g1_to_gr1t2_fourier_migration_guide_ko.md](../research/31.%20ust_ws_g1_to_gr1t2_fourier_migration_guide_ko.md) — 본 구현의 설계 원전
- [research/30. humanoid_robot_teleop_alternatives_for_isaac_lab_ko.md](../research/30.%20humanoid_robot_teleop_alternatives_for_isaac_lab_ko.md) — GR1T2 선정 근거
- [research/29. ust_ws_ubuntu_to_windows_steamvr_migration_analysis.md](../research/29.%20ust_ws_ubuntu_to_windows_steamvr_migration_analysis.md) — Ubuntu → Windows 마이그레이션 선행 분석
- [ust_hm_glove/WINDOWS_EXECUTION_GUIDE.md](../ust_hm_glove/WINDOWS_EXECUTION_GUIDE.md) — G1 경로 실행 가이드 (Pico/VD/SteamVR 설정, UDCAP 드라이버 등 공통 사항은 여전히 유효)
- [../memory.md](../memory.md) — ust_ws 마이그레이션 작업 메모리

---

작성: 2026-04-21
검증 상태: 단위 테스트 + smoke 테스트 100% PASS, 실 VR 하드웨어 검증 대기.
