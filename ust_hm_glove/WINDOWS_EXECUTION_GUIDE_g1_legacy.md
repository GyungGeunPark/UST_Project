# ust_hm_glove — Windows 11 + SteamVR 실행 가이드

> **대상 구성**: Windows 11 단일 PC + Pico 4 Ultra + Virtual Desktop + 5× Pico Motion Tracker (Enhanced Forearm) + UDCAP VR Glove (L/R) + Isaac Lab 2.x(Windows pip) + SteamVR + `miniconda/ust` 가상환경.
>
> **기반 문서**: `research/28. SteamVR 휴머노이드 텔레오퍼레이션 구현 가이드 (Windows 단일 PC + Virtual Desktop + UDCAP VR Glove).md`, `research/29. ust_ws_ubuntu_to_windows_steamvr_migration_analysis.md`.
>
> **본 폴더** `ust_ws/ust_hm_glove/`: 상기 두 문서의 이행을 위해 신규/수정된 파일만 담고 있으며, Gym 환경·보상·학습 파이프라인 등 플랫폼 독립 코드는 `ust_ws/ust_260220/`에서 **그대로 재사용**합니다.

---

## 1. 폴더 구성

```
ust_ws/ust_hm_glove/
├── __init__.py
├── WINDOWS_EXECUTION_GUIDE.md          (이 파일)
├── requirements-windows.txt
├── setup_steamvr_env.ps1
│
├── teleop/
│   ├── __init__.py
│   ├── coord_transforms.py             # SVR(Y-up) <-> Isaac Lab(Z-up), forearm→wrist
│   ├── vr_sampler.py                   # pyopenvr 120Hz 스레드
│   ├── fingertip_extractor.py          # 31-bone → 5 fingertip
│   ├── udcap_finger_mapper.py          # VMC → Inspire 12D (폴백)
│   ├── vmc_receiver.py                 # UDP:39539 OSC 수신기 (Path B)
│   ├── g1_retargeter.py                # SteamVR snapshot → 38D Pink IK action
│   └── pico_udcap_device.py            # DeviceBase + 개입 인터페이스
│
├── scripts/
│   ├── smoke_test.py                   # VR 하드웨어 없이 실행되는 단위 검증
│   ├── enumerate_trackers.py           # LHR-* 시리얼 덤프 → tracker_binding.json
│   ├── calibrate_forearm_offset.py     # T-pose 오프셋 측정
│   └── run_teleop.py                   # Isaac Lab 텔레오퍼레이션 진입점
│
├── config/
│   ├── openvr_actions/actions.json     # OpenVR 액션 매니페스트
│   ├── tracker_binding.json            # 시리얼→역할 매핑 (편집 필수)
│   └── dex_retargeting/
│       ├── inspire_left_dexpilot.yml
│       └── inspire_right_dexpilot.yml
│
└── tests/                              (예비 — pytest 도입 시 사용)
```

각 파일은 `research/29 …` §4.4 / §3.1 매핑표에 1:1 대응되며, 폐기 대상 (`unified_bridge.py`, `xrt_data_parser.py`, 모든 `.sh`, Docker compose 등)은 **이 폴더에 옮기지 않습니다**. 폐기는 `ust_260220/` 원본을 유지한 채 호출 경로에서만 제거합니다.

---

## 2. 하드웨어/OS 사전 조건

| 항목 | 요구값 / 확인 방법 |
|------|-------------------|
| Windows 11 22H2 이상 | `winver` |
| NVIDIA 드라이버 555+ (CUDA 12.8) | `nvidia-smi` |
| Pico OS 5.14 이상 (Enhanced Forearm) | Pico 설정 → 시스템 업데이트 |
| SteamVR 최신 (Skeletal Input 2.0) | Steam 라이브러리 자동 갱신 |
| Virtual Desktop Streamer + Client | `Streaming` 탭 → **Forward Trackers to SteamVR: ON** |
| UDCAP Driver (SteamVR Add-on) | UDCAP 앱 → “Enable SteamVR driver: ON” |
| Long Path 지원 | `setup_steamvr_env.ps1`가 자동 처리 |
| Python 3.11 / `miniconda/ust` | `conda activate ust` 후 `python --version` |

> ⚠️ Pico 컨트롤러가 SteamVR에 들어와 있으면 UDCAP이 에뮬레이션하는 Index 컨트롤러와 슬롯이 충돌합니다 — **Pico 컨트롤러는 페어링 해제** 또는 절전 상태로 두세요 (doc 28 §8.2).

---

## 3. 라이브러리 설치

### 3.1 자동 스크립트

```powershell
conda activate ust
cd C:\develop\IsaacLab\ust_ws\ust_hm_glove
# 최초 1회만 관리자 PowerShell (Long Path 레지스트리 반영)
.\setup_steamvr_env.ps1 -SetSteamVROpenXR
```

스크립트가 수행하는 내용:

1. `HKLM\SYSTEM\CurrentControlSet\Control\FileSystem\LongPathsEnabled = 1`
2. `conda install -n ust -c conda-forge pinocchio` (Pinocchio Windows wheel은 conda-forge에만 존재)
3. `pip install --constraint constraints-windows.txt -r requirements-windows.txt`
   - `openvr>=1.26.7`, `python-osc>=1.8`, `pyzmq>=25.0`, `dex-retargeting>=0.5.0`
   - `constraints-windows.txt`가 `numpy==1.26.0`을 강제해 `dex-retargeting`의 `numpy>=2.0.0` 선언이 numpy를 업그레이드하지 못하도록 막음 (Isaac Sim 5.x는 numpy 1.26 필요).
4. 설치 후 numpy 버전을 검증하여 필요 시에만 `numpy==1.26.0`으로 재핀.
5. `-SetSteamVROpenXR`가 주어지면 OpenXR ActiveRuntime 레지스트리를 작성:
   - 관리자 PowerShell → `HKLM\Software\Khronos\OpenXR\1\ActiveRuntime`
   - 일반 PowerShell → `HKCU\Software\Khronos\OpenXR\1\ActiveRuntime`에 per-user로 기록 (HKLM 권한 없을 때 자동 폴백). 일부 OpenXR 로더는 HKLM만 읽으므로, Isaac Sim이 SteamVR을 인식하지 못하면 **SteamVR → 설정 → 개발자 → "Set SteamVR as OpenXR Runtime"** GUI 또는 관리자 PowerShell로 재실행.
6. `smoke_test.py` 실행

### 3.2 수동 설치 (동일 결과)

```powershell
conda activate ust
conda install -c conda-forge -y pinocchio
pip install openvr python-osc pyzmq dex-retargeting
pip install numpy==1.26.0   # Isaac Sim 호환
```

### 3.3 dex-retargeting 자산 (URDF)

`config/dex_retargeting/*.yml`은 Inspire Hand URDF를 참조합니다. 다음 중 하나를 선택:

1. **권장**: `dexsuite/dex-retargeting` 저장소 `assets/robots/` 디렉토리를 다운로드하여 예컨대 `C:\develop\IsaacLab\ust_ws\ust_hm_glove\assets\dex_robots\inspire_hand\` 로 배치 후 `PicoUDCAPDeviceCfg.dex_urdf_dir`에 해당 부모 경로 지정.
2. **건너뛰기**: YAML 경로를 `None`으로 두면 `UDCAPFingerMapper`가 SteamVR skeletal/VMC 데이터를 직접 12D로 변환합니다. 초기 부트스트랩에는 이 폴백으로도 충분합니다.

### 3.4 실행 검증 (이미 완료됨)

```
[smoke] PASS  coord_transforms
[smoke] PASS  fingertip_extractor
[smoke] PASS  udcap_finger_mapper
[smoke] PASS  g1_retargeter
[smoke] PASS  device_import
[smoke] 5/5 passed
```

위 출력이 나오면 Windows 포트는 정상입니다. VR 하드웨어가 없어도 단위 검증은 100% 통과합니다.

---

## 4. 최초 설정 절차 (SteamVR/VD 구성이 준비된 이후)

### 4.1 트래커 시리얼 바인딩

```powershell
conda activate ust
# SteamVR + Pico 4 Ultra (Virtual Desktop 스트리밍) 작동 상태에서
python -m ust_ws.ust_hm_glove.scripts.enumerate_trackers `
    --out ust_ws\ust_hm_glove\config\tracker_binding.json
```

Virtual Desktop의 **Full Body Tracking**이 활성화돼 있으면 **10개 세그먼트**가 감지됩니다 (물리 Pico 5개 + Pico 헤드셋 AI가 합성한 5개). enumerate 스크립트는 VD 규약에 맞춰 **자동으로 역할을 매핑**합니다:

| VD 시리얼 | 자동 매핑 | 출처 |
|-----------|-----------|------|
| `hips` | `waist` | 물리 Pico 허리 |
| `left_arm_lower` / `right_arm_lower` | `left_forearm` / `right_forearm` | 물리 Pico 팔뚝 |
| `left_lower_leg` / `right_lower_leg` | `left_ankle` / `right_ankle` | 물리 Pico 발목 |
| `chest`, `left_arm_upper`, `right_arm_upper`, `left_foot_transverse`, `right_foot_transverse` | `""` (비활성) | AI 추론 — 샘플러가 스킵 |

물리 `LHR-xxxxxxxx` Vive Tracker 퍽을 따로 달고 있다면 (Pico VD 경로가 아닌 경우) 자동 매핑 대상에 없으므로 `role: "TODO"` 로 남고, 사용자가 직접 다섯 역할 이름 중 하나로 수정해야 합니다. SteamVR `Manage Vive Trackers` UI → **Identify** 버튼으로 어느 트래커가 어느 부위인지 시각 확인 가능.

### 4.2 팔뚝→손목 오프셋 측정

```powershell
python -m ust_ws.ust_hm_glove.scripts.calibrate_forearm_offset `
    --lower_arm_m 0.27 `
    --out ust_ws\ust_hm_glove\config\forearm_offset.json
```

* `--lower_arm_m`은 트래커 스트랩 중심에서 손목 중심까지의 실측 거리 (성인 남성 기준 0.24–0.30 m).
* 결과 JSON의 `left_forearm_offset` / `right_forearm_offset` (튜플 `[x, y, z]`) 값을 `PicoUDCAPDeviceCfg.forearm_wrist_offset`으로 전달.

### 4.3 SteamVR OpenXR 런타임 지정

`run_teleop.py --render_mode steamvr_desktop` 이상을 쓰려면 Windows의 OpenXR 활성 런타임이 SteamVR이어야 합니다.

* GUI: **SteamVR → 설정 → 개발자 → “Set SteamVR as OpenXR runtime”**
* CLI: `setup_steamvr_env.ps1 -SetSteamVROpenXR`
* 레지스트리 확인: `HKLM:\Software\Khronos\OpenXR\1\ActiveRuntime` = `…\SteamVR\steamxr_win64.json`

---

## 5. 실행

### 5.1 단위 스모크 (VR 없이, 언제든 실행 가능)

```powershell
python -m ust_ws.ust_hm_glove.scripts.smoke_test
```

### 5.2 텔레오퍼레이션 — 모니터 모드 (1단계 권장 기본)

```powershell
# isaaclab.bat가 Isaac Sim 환경변수를 설정해 주므로 이를 경유하는 것이 가장 안전
C:\develop\IsaacLab\isaaclab.bat -p `
    -m ust_ws.ust_hm_glove.scripts.run_teleop `
        --teleop_device pico_udcap `
        --render_mode monitor `
        --use_usd_scene
```

* `render_mode monitor`: PC 창에 Isaac Sim 씬을 표시. VR 헤드셋으로는 스트리밍 안 함. 마이그레이션 1단계 검증용.
* `--teleop_device pico_udcap`: Windows 포트의 주 경로. 기존 `--teleop_device pico`/`pico_no_udcap`은 폐기.
* `--use_usd_scene`: `Isaac-KitchenSorting-G1-InspireFTP-USD-v0` 환경 사용 (기본값).
* 추가 옵션: `--path_b_port 39539`를 지정하면 VMC OSC 폴백 수신기가 동시에 기동됩니다 (UDCAP Driver의 VMC Broadcast를 39539로 설정했을 때).

### 5.3 텔레오퍼레이션 — VR 헤드셋 피드백 (Virtual Desktop Desktop Theater)

```powershell
C:\develop\IsaacLab\isaaclab.bat -p `
    -m ust_ws.ust_hm_glove.scripts.run_teleop `
        --teleop_device pico_udcap `
        --render_mode steamvr_desktop
```

* Virtual Desktop 클라이언트에서 **Desktop / Desktop Theater** 모드로 전환 → Isaac Sim PC 창이 VR 내부에 2D 가상 모니터로 표시 (doc 28 Path-4). 스테레오 렌더링은 아직 하지 않지만, 헤드셋을 벗지 않아도 씬을 볼 수 있어 교시 루프가 가능합니다.
* `render_mode steamvr_native`는 Isaac Sim 4.5 + `omni.kit.xr.system.steamvr` 경로로 예약되어 있으며, 5.x에서는 실험 상태입니다 (doc 28 §A.3 Path-2).

### 5.3.1 로봇이 안 움직일 때 진단 모드 (3단계 격리법)

`run_teleop.py`에 세 개의 진단 플래그를 추가했습니다. 이 순서대로 실행해 문제 위치를 좁힐 수 있습니다.

**단계 1 — 파이프라인 자체 검증 (VR 무시)**:
```powershell
C:\develop\IsaacLab\isaaclab.bat -p -m ust_ws.ust_hm_glove.scripts.run_teleop `
    --teleop_device pico_udcap --render_mode monitor `
    --diag oscillate --debug_ik
```
- `--diag oscillate`는 VR 샘플러를 완전히 무시하고 **시간에 따른 사인파 wrist 타겟**을 보냅니다. idle ±10cm X 방향으로 0.5 Hz 스윙.
- **로봇이 움직이면**: env.step() → Pink IK → 관절 제어 파이프라인이 정상. 이후 VR 경로의 매핑 문제로 좁혀짐.
- **로봇이 여전히 안 움직이면**: 파이프라인(PinkIK/액션 매니저/액츄에이터)에 더 깊은 문제. `--debug_ik`가 찍는 warning을 읽어 정확한 원인 파악.

**단계 2 — IK 경고 활성화**:
```powershell
... --teleop_device pico_udcap --debug_ik
```
- `env_cfg.actions.pink_ik_cfg.controller.show_ik_warnings`를 런타임에 `True`로 강제.
- Pink IK solver가 수렴 실패할 때마다 `Warning: IK quadratic solver could not find a solution!` 로그가 찍힘.
- 매 프레임 찍히면 타겟이 영구 unreachable. 간헐적이면 사용자가 특정 제스처(예: 손목 극단 회전)를 했을 때만 실패.

**단계 3 — 방향 고정으로 위치만 테스트**:
```powershell
... --teleop_device pico_udcap --freeze_orientation --debug_ik
```
- 팔뚝 트래커의 회전은 무시하고 **G1 idle quaternion (0.707, 0, 0, 0.707)** 을 그대로 wrist orientation으로 전달.
- Pink IK는 위치만 추종하면 되므로 운동 범위가 크게 완화됨.
- **로봇이 위치 추종하면**: 원인은 **orientation 매핑**(forearm 트래커 local X축 방향 가정이 틀림, 또는 G1 wrist URDF 규약과 SteamVR 프레임 규약 차이). 이 경우 `coord_transforms.svr_to_isaaclab`의 quaternion 변환 회전축이 G1 손목 URDF와 맞도록 재조정 필요.
- **여전히 안 움직이면**: 원인은 **position 매핑** (좌표 원점 / XR anchor 오프셋 / scale 문제). `--diag oscillate`가 작동했다면 retargeter의 위치 산출 로직에 문제 있음.

> ⚠️ **research/30.md §1의 경고**: 본 env_cfg는 Pink IK target을 `left_wrist_yaw_link` (손목 yaw 이후, roll/pitch 이전)에 설정. 즉 마지막 2개 손목 조인트가 IK에 들어가지 않아 **실질적으로 5-DoF 팔 + 2개 자유 조인트**. 이 때문에 자연스러운 손 orientation 범위가 좁은 것. 본질적인 해결은 target frame을 `palm_link` 또는 `hand_pitch_link`로 변경하는 것(= GR1T2 접근).

### 5.4 데이터 수집 / HG-DAgger

`ust_260220/scripts/record_demos.py`, `run_hg_dagger.py` 등은 그대로 사용 가능합니다. 디바이스만 새 import로 교체하세요:

```python
from ust_ws.ust_hm_glove.teleop.pico_udcap_device import (
    PicoUDCAPDevice,
    PicoUDCAPDeviceCfg,
    PICOInterventionInterface,
)

cfg = PicoUDCAPDeviceCfg(path_b_port=39539)   # 필요 시
device = PicoUDCAPDevice(cfg)
device.start()

intervention = PICOInterventionInterface(device)
...
```

* `PICOInterventionInterface`는 기존과 동일한 API(`check_intervention_trigger`, `check_resume_trigger`, `check_reset_trigger`)를 제공합니다. 단, 버튼 데이터는 UDCAP가 에뮬레이션하는 Valve Index 컨트롤러에서 읽습니다.

---

## 6. 데이터 플로우 요약

```
Pico 4 Ultra  ─Wi-Fi 6E─▶ Virtual Desktop Streamer ──▶ SteamVR (OpenVR + OpenXR)
   ▲                                                       ▲
   └─ 5× Pico Tracker (2.4 GHz, Enhanced Forearm)          │
                                                           │
UDCAP VR Glove L/R ─USB─▶ UDCAP Driver ──SteamVR Addon────┤
                           │                               │
                           └─(선택) VMC UDP:39539 ─────────┼──▶ VMCHandReceiver
                                                           │            │
                                                           ▼            ▼
                             ┌────────── Isaac Lab 프로세스 (ust env) ──────────┐
                             │ SteamVRSampler (120 Hz)                          │
                             │   └─ HMD + 5 Tracker + 양손 Skeletal(31+요약)    │
                             │                  │                               │
                             │                  ▼                               │
                             │ PicoUDCAPDevice (DeviceBase)                     │
                             │   ├─ forearm_to_wrist(+X offset)                 │
                             │   ├─ svr_to_isaaclab (Y-up → Z-up)               │
                             │   ├─ G1SteamVRRetargeter                          │
                             │   │    ├─ dex-retargeting(주) → 12D              │
                             │   │    └─ UDCAPFingerMapper(폴백) → 12D          │
                             │   └─ 38D Pink IK action 조립                      │
                             │                  │                               │
                             │                  ▼                               │
                             │ ManagerBasedRLEnv (Isaac-KitchenSorting-G1-USD)  │
                             └──────────────────────────────────────────────────┘
```

---

## 7. 자주 발생하는 문제

| 증상 | 원인 / 해결 |
|------|------------|
| `ModuleNotFoundError: isaaclab` | `ust` 환경의 편집 가능 설치(`__editable__.isaaclab*_finder.py`)가 다른 경로를 가리킴. `C:\Users\pjwpy\miniconda3\envs\ust\Lib\site-packages\__editable___isaaclab*_finder.py` 각각의 `MAPPING`을 현재 `C:\develop\IsaacLab\source\isaaclab\isaaclab`로 수정. |
| `openvr.OpenVRError: Not Initialized` | SteamVR이 켜져 있지 않거나 Virtual Desktop이 연결되지 않음. SteamVR 알림창에 “헤드셋 감지” 문구가 떠야 함. |
| `dex-retargeting 'cp949' codec' 오류 | YAML에 ASCII 외 문자가 들어 있음. 본 레포 YAML은 ASCII 전용이지만, 수정 시 주의하거나 `set PYTHONUTF8=1`로 회피 가능. |
| `URDF path ... does not exist` | Inspire Hand URDF 자산 미배치. §3.3 안내대로 배치하거나 YAML을 `None`으로 두고 UDCAP 폴백 사용. |
| `numpy 2.x 설치됨` 경고 | `dex-retargeting`이 numpy>=2를 요구하지만 Isaac Sim은 1.26을 요구. `pip install numpy==1.26.0`으로 고정. 런타임은 numpy 1.26에서 동작함이 확인됨. |
| 트래커가 인식은 되지만 `pose_valid=False` | Pico의 Play Area가 정의되지 않음. SteamVR **Room Setup → Standing Only**를 한 번 실행. |
| UDCAP의 손 스켈레톤이 비어 있음 | UDCAP Driver가 SteamVR보다 먼저 기동되지 않음. UDCAP Driver 설정에서 “Auto-start with SteamVR”을 켜거나, UDCAP를 수동 먼저 실행. |
| 손가락 제스처가 반대 방향 | `G1SteamVRRetargeter`의 `bend_scale`/`spread_scale` 부호 조정. Inspire FTP URDF가 좌우 대칭이 아닌 경우 해당 손만 `-1` 스케일. |
| enumerate에서 HMD가 **"Oculus Meta Quest 3"** 로 표시되는데 실제로는 Pico 4 Ultra를 착용 중 | **정상 동작**입니다. Virtual Desktop은 Pico도 `oculus_virtualdesktop` SteamVR 드라이버(Quest OVR 호환 레이어)로 스트리밍하므로 SteamVR에서는 HMD가 항상 "Oculus Meta Quest 3"로 보입니다. 증거: `C:\Program Files (x86)\Steam\config\steamvr.vrsettings` → `LastKnown.ActualHMDDriver = "oculus_virtualdesktop"`. HMD 이름이 아니라 **GenericTracker 5개가 보이는지**로 판단하세요. |
| `Found 0 generic trackers` — Pico에선 5개가 연결되어 있는데 SteamVR에는 안 올라옴 | 다음 3개 조건이 모두 충족돼야 트래커가 SteamVR에 나타납니다: ① Pico OS ≥ 5.14, ② Pico 설정 → Motion Tracker → **Enhanced Forearm (5 tracker)** 모드 선택, ③ VD Streamer → OPTIONS → **"Forward tracking to SteamVR"** + **"Full body tracking"** ON. 토글 후 VD Streamer 재시작 + Pico에서 VD 클라이언트 재접속. |
| SteamVR Add-Ons 목록에 `prism`이 나타남 | Pico Connect의 내장 SteamVR 드라이버 (`…\SteamVR\drivers\prism`). Virtual Desktop 경로와 동시에 켜 놓으면 HMD 식별이 충돌할 수 있음 → **OFF** 권장. `Virtual Desktop Streamer (Quest)`와 `udcap`은 ON 유지. |
| Isaac Sim 부팅 중 `ImportError: cannot import name '_promote' from 'scipy.spatial.transform._rotation' (...pyd)` (또는 `_spropack`, `_rotation_xp` 등 유사 심볼) | scipy 설치 오염. `pip --force-reinstall`은 `.py`만 덮어쓰고 이전 버전의 `.pyd` 컴파일 바이너리를 완전히 제거하지 못함. **디렉토리 물리 삭제 후 재설치**: `pip uninstall -y scipy` → `Remove-Item -Recurse -Force "$env:USERPROFILE\miniconda3\envs\ust\Lib\site-packages\scipy*"` → `pip install --no-cache-dir scipy==1.15.3` → `python -c "from scipy.spatial.transform import Rotation"`로 검증. 동반 증상: 수십 개 isaacsim 확장(`isaacsim.core.prims`, `isaacsim.robot.manipulators` 등)이 모두 동일한 scipy 에러로 실패. |
| `ModuleNotFoundError: No module named 'pink'` | `ust_260220/kitchen_sorting_env_cfg.py`가 Pink IK를 사용하지만 모듈 미설치. **PyPI 패키지명은 `pin-pink`** (하이픈 위치 주의 — `pink-ik` 아님): `pip install pin-pink`. 설치 후 `import pink`로 확인. `qpsolvers`의 `no QP solver found on your system` 경고는 실행에 치명적이지 않으며, 정확도를 높이려면 `pip install qpsolvers[open_source_solvers]`. |
| 로그는 `[PicoUDCAPDevice] first action: L_EEF=[…]`를 찍고 트래커 5개가 계속 갱신되는데 Isaac Sim 로봇이 전혀 움직이지 않음 | **Pink IK가 조용히 수렴 실패**. `pink_ik.py:229-241` 소스 확인 결과: solve_ik가 예외를 던지면 **현재 관절 위치를 그대로 반환**(`show_ik_warnings=False`가 기본이라 에러 숨김). 원인 후보: ① 손목 6DoF 목표 방향이 G1의 `wrist_yaw_link` (5-DoF 손목 루트) 운동 범위 밖 — research/30.md §1 참고, ② 좌표 프레임 어긋남, ③ 90° Z-회전된 G1 spawn과 사용자 정면 방향 불일치. **진단 방법**: `--debug_ik` 플래그로 IK warning을 켜서 실제 실패 사유 확인, `--freeze_orientation`로 방향은 idle 고정해 위치만 추종 테스트, `--diag oscillate`로 VR 데이터 무시하고 합성 사인파를 보내 파이프라인 자체 검증. `research/30. humanoid_robot_teleop_alternatives_for_isaac_lab.md`에 GR1T2 등 대안 로봇 비교 있음. |
| `ImportError: DLL load failed while importing _errors: 지정된 프로시저를 찾을 수 없습니다.` (h5py/`_errors.pyd`), Isaac Sim 부팅 뒤 isaaclab_tasks 로드 중에 발생 | **HDF5 DLL 선점 충돌**. Isaac Sim은 자체 `hdf5.dll`(HDF5 1.10/1.12 ABI, 3.2 MB)을 프로세스에 먼저 로드하는데, h5py 3.16+는 HDF5 1.14 ABI(번들된 3.9 MB `site-packages/h5py/hdf5.dll`)를 기대. Windows는 같은 이름의 DLL을 프로세스당 하나만 로드하므로 누가 먼저 로드되느냐가 전체 프로세스의 HDF5 버전을 결정. **해결**: AppLauncher 호출 전에 `import h5py`를 수행해 h5py 번들 DLL이 먼저 프로세스에 박히게. `run_teleop.py`는 이미 이 선로딩을 포함. 사용자가 새 스크립트를 쓸 때도 `from isaaclab.app import AppLauncher` 위에 `import h5py`를 넣을 것. |

---

## 8. 마이그레이션 체크리스트 (research/29 §7 → 이행 상태)

| # | 항목 | 상태 |
|---|------|------|
| Week 1-1 | `ust` env Python 3.11 + PyTorch 2.7.0+cu128 | ✅ (검증됨) |
| Week 1-2 | 필수 패키지 설치 (openvr, dex-retargeting, python-osc, pyzmq, pinocchio) | ✅ |
| Week 1-3 | NVIDIA 555+ / Long Path | ⏳ 사용자 환경 확인 필요 |
| Week 1-4 | Virtual Desktop + 5 tracker + UDCAP 페어링 | ⏳ 사용자 하드웨어 |
| Week 1-5 | `enumerate_trackers.py` → `tracker_binding.json` | ⏳ 하드웨어 필요 |
| Week 1-6 | `SteamVRSampler` 120 Hz 구동 확인 | ✅ 구현 완료, 하드웨어 부하 실증은 현장 |
| Week 2-1 | Monitor 모드 부팅 (`run_teleop.py --render_mode monitor`) | ✅ 코드, 런타임은 Isaac Sim 실행 환경 |
| Week 2-2 | `PicoUDCAPDevice` 스켈레톤 | ✅ |
| Week 2-3 | Snapshot → 38D 루프백 | ✅ (smoke_test) |
| Week 2-4 | SVR→Isaac 좌표 단위 테스트 | ✅ |
| Week 3-1 | Forearm 캘리브레이션 스크립트 | ✅ |
| Week 3-2 | PinkIK 연결 (`self.actions.pink_ik_cfg` 재사용) | ✅ (환경 cfg 재사용) |
| Week 3-3 | dex-retargeting Inspire FTP YAML | ✅ (URDF 자산 미배치 시 UDCAP 폴백) |
| Week 3-4 | 38D 액션 → G1 실제 움직임 | ⏳ 하드웨어+Isaac Sim 구동 |
| Week 3-5 | VMC 폴백 (`VMCHandReceiver`) | ✅ |
| Week 4 | 데모 기록, HG-DAgger, 지연 프로파일링 | ⏳ (`ust_260220/scripts/*` 그대로 사용; device 교체만) |

---

## 9. 남은 하드웨어 의존 검증

본 구현은 VR 장비가 없는 환경에서 단위 테스트까지만 검증되었습니다. 하드웨어 결합 시 확인해야 할 항목:

1. `enumerate_trackers.py` 출력에 **5개의 `GenericTracker`** 가 전부 등장하는지.
2. `calibrate_forearm_offset.py` 세션 동안 `pose_valid=True` 비율이 99% 이상인지.
3. `PicoUDCAPDevice.start()` 직후 `snapshot()["hands"]["left"]` / `right`가 `None`이 아닌 `(31, 7)` 배열을 돌려주는지.
4. `run_teleop.py --render_mode monitor`가 Isaac Sim 창을 열고, 양손이 T-pose → 팔꿈치 90도 → 양손 교차 제스처를 따라가는지 (로봇이 동일 방향으로 움직이는지).
5. `Virtual Desktop` → `Desktop Theater`로 전환 시 Isaac Sim 창이 VR 내부에 투영되는지 (visual feedback).

위 5단계가 모두 통과하면 마이그레이션이 물리적으로도 닫힙니다. 이후 Week 4 튜닝(지연, 스케일 게인, 데모 수집, HG-DAgger)은 `ust_260220/scripts/`의 기존 파이프라인을 **device import만 바꿔** 재사용하면 됩니다.

---

## 10. 참고 파일

* 마이그레이션 분석: [research/29. ust_ws_ubuntu_to_windows_steamvr_migration_analysis.md](../research/29.%20ust_ws_ubuntu_to_windows_steamvr_migration_analysis.md)
* 설계 원전: [research/28. SteamVR 휴머노이드 텔레오퍼레이션 구현 가이드 (Windows 단일 PC + Virtual Desktop + UDCAP VR Glove).md](../research/28.%20SteamVR%20휴머노이드%20텔레오퍼레이션%20구현%20가이드%20%28Windows%20단일%20PC%20%2B%20Virtual%20Desktop%20%2B%20UDCAP%20VR%20Glove%29.md)
* Ubuntu 원본 실행 가이드: [ust_260220/EXECUTION_GUIDE.md](../ust_260220/EXECUTION_GUIDE.md)
* 본 포트 코드: `ust_ws/ust_hm_glove/`
