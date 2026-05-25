# XRoboToolkit Backend — 실행 가이드

> **날짜**: 2026-05-15 (11th session)
> **선행 문서**:
> - [`../research/47. xrobotoolkit_implementation_guide.md`](../research/47.%20xrobotoolkit_implementation_guide.md) — 상세 구현 사양
> - [`claude.md`](claude.md) §3.13 — backend 분기 컨벤션
> - [`memory.md`](memory.md) §10 — PCVR Input 차단 (이 가이드가 해결하는 문제)
>
> **목적**: 11번째 세션에서 구현된 XRoboToolkit 백엔드를 **즉시 실행**할 수 있도록, 사용자 PC 의 실제 경로 + 빌드 / 실행 / 진단 순서를 정리.
>
> **범위**: Phase A (입력 분리 + 모니터 시각화). PICO Connect 와의 비디오 스트리밍 공존 (Phase B), 5-tracker body mocap (Phase C) 은 본 가이드 §10 / §11 의 비고 참고.

---

## 0. 한 줄 요약

기존 SteamVR Personal Binding 경로 (10th session 에서 PCVR 환경 차단 확정) 대신 PICO 공식 **XRoboToolkit gRPC** 경로로 PICO 컨트롤러 입력을 우회. `--input_backend xrobotoolkit` 한 flag 로 백엔드 전환, snapshot dict 동일 → retargeter / env_cfg / Pink IK / Robotiq drive 무수정.

> ## ⚠️ **PICO HMD 단일 APK 제약** (필수)
>
> PICO 4 Ultra 헤드셋은 **한 번에 하나의 stream APK 만 실행 가능**.
> - `XRoboToolkit Unity Client` (XR-Robotics 의 PICO APK) — controller / pose / body 데이터 streaming via gRPC
> - `PICO Connect` 의 in-headset companion — PCVR video streaming via SteamVR
>
> **둘은 mutually exclusive**. PICO Connect 가 SteamVR session 을 시작하면 Unity Client APK 가 즉시 종료됨.
>
> 따라서 본 가이드의 모든 텔레오퍼레이션 명령은 **PC 모니터 렌더링** (`--render_mode monitor`) 만 사용한다. HMD stereo 렌더링이 필요하면 [§10 (Phase B / Phase C 옵션)](#10-hmd-시각화-options-phase-b--c) 참조 — XRoboToolkit 대신 CloudXR / 키보드 fallback / ALVR 같은 별도 경로 필요.

---

## 1. 11th 세션에서 완료된 작업 (이미 머지됨)

### 신규 파일 (6개)

| 파일 | 줄수 | 책임 |
|---|---|---|
| [teleop/xrobo_sampler.py](teleop/xrobo_sampler.py) | ~290 | XRoboSampler — SteamVRSampler 와 동일한 snapshot dict 인터페이스로 xrt SDK 폴링 |
| [scripts/minimal_pico_check.py](scripts/minimal_pico_check.py) | ~135 | Isaac Sim 의존성 없이 xrobotoolkit_sdk 만 검증 |
| [scripts/diagnose_xrobotoolkit.py](scripts/diagnose_xrobotoolkit.py) | ~170 | L1–L4 layered probe (process / port / SDK init / live data) |
| [tests/test_coord_transforms_xr.py](tests/test_coord_transforms_xr.py) | ~110 | 좌표계 변환 11 케이스 |
| [tests/test_xrobo_sampler.py](tests/test_xrobo_sampler.py) | ~200 | XRoboSampler 8 케이스 (fake xrt 모듈로 hardware-free) |
| [config/xrobotoolkit_settings.json](config/xrobotoolkit_settings.json) | ~32 | 사용자 수정 가능 환경 설정 (참고용; 코드에는 default 값 내장) |

### 수정 파일 (3개)

| 파일 | 변경 | 책임 |
|---|---|---|
| [teleop/coord_transforms.py](teleop/coord_transforms.py) | +110 line | `xr_to_isaaclab`, `xyzw_to_wxyz`, `R_XR2IL` 추가. **버그 수정**: 가이드 #47 의 R_XR2IL 은 det = -1 (improper rotation) 이라 quaternion conjugation 이 깨짐 → OpenXR LOCAL 컨벤션 (+Z = back) 에 맞춰 `[[0,0,-1],[-1,0,0],[0,1,0]]` 로 보정. |
| [teleop/gr1t2_gripper_device.py](teleop/gr1t2_gripper_device.py) | +90 line | `input_backend` / `xrt_enable_body` / `xrt_enable_hand` cfg 필드 + `start()` / `_read_action_inputs()` backend 분기 + `_probe_action_values_xrt()` |
| [scripts/run_teleop.py](scripts/run_teleop.py) | +70 line | `--input_backend` / `--xrt_enable_body` / `--xrt_enable_hand` CLI + RoboticsServiceProcess / Pico Connect pre-flight 진단 + device cfg 전달 |

### 사용자 PC 에 이미 준비된 자원

- **XRoboToolkit-PC-Service.win**: `C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service.win\` — pre-built v1.0.0 Windows 배포본 (`RoboticsServiceProcess.exe` + `SDK/x64/PXREARobotSDK.{dll,lib}` 포함). **별도 source 빌드 불필요**.
- **XRoboToolkit-PC-Service (source)**: `C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service\` — Pybind 빌드 시 `PXREARobotSDK.h` 와 `nlohmann/json.hpp` 추출용으로만 사용. Qt source 빌드는 안 함.
- **XRoboToolkit-PC-Service-Pybind**: `C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service-Pybind\` — 11th 세션이 clone 했고, `include/` + `lib/` 미리 준비된 상태 (MSVC 설치 후 `python setup.py install` 한 단계만 남음).
- **Qt 6.11.1**: `C:\Qt\6.11.1\` — Pybind 빌드 자체에는 불필요 (SDK 만 link). PC-Service 를 source 빌드할 때만 사용 (지금은 안 함).
- **Inno Setup 6.7.1**: 설치만 됨 — `.exe` installer 만들 때만 사용 (지금은 안 함).
- **Unity Client APK**: 헤드셋에 sideload 완료 (구 우분투 환경의 `adb install` 잔재). 헤드셋 메뉴 → Apps → "XRoboToolkit" 으로 실행.

---

## 2. **검증 결과** (2026-05-15)

| 검증 항목 | 결과 |
|---|---|
| `coord_transforms.py` import + identity / axis / combined / quat 테스트 | **11 / 11 PASS** |
| `XRoboSampler` lifecycle / snapshot shape / IL frame / clamping / monotonic / threadsafe / 미설치 fallback | **8 / 8 PASS** |
| 기존 `test_gripper_retargeter.py` (22) + `test_action_manifest.py` (21) | **43 / 43 PASS** (회귀 없음) |
| 전체 `ust_hm_grip/tests/` | **62 / 62 PASS** |
| `run_teleop --input_backend openvr` 회귀 (5 step monitor) | `reached --steps=5` + FATAL/Traceback 없음 |
| **MSVC Build Tools 2019 설치 검증 (11th 세션 후속)** | `cl.exe` @ `MSVC\14.29.30133`, `vcvars64.bat`, Windows SDK 10.0.19041 / 10.0.22621 ✓ |
| **xrobotoolkit_sdk-1.0.2 wheel 빌드 + pip install** | `Successfully built xrobotoolkit_sdk` + `pip install .` 성공 |
| **`PXREARobotSDK.dll` site-packages 에 배치** | `C:\Users\pjwpy\miniconda3\envs\ust\Lib\site-packages\PXREARobotSDK.dll` |
| **`import xrobotoolkit_sdk` 실제 동작** | 38 API 노출 (`init`, `close`, `get_left_*`, `get_motion_tracker_*`, `get_left_menu_button` 등) |
| **`diagnose_xrobotoolkit --skip_live` (service ON)** | `ALL LAYERS PASS` (L1 process / L2 port 127.0.0.1:60061 / L3 `xrt.init()`) |
| **`minimal_pico_check --seconds 2` (service ON, no APK)** | 30Hz polling 60 samples 수집, all-zero (헤드셋 미스트리밍 → 의도된 FAIL verdict) |
| `run_teleop --input_backend xrobotoolkit` (no service) | 의도된 WARN 출력 + actionable RuntimeError |
| `diagnose_xrobotoolkit --skip_live` (no service) | L1 FAIL: `RoboticsServiceProcess.exe is NOT running.` + 시작 명령 안내 |
| `minimal_pico_check` (no SDK) | `[FATAL] xrobotoolkit_sdk import failed` + 빌드 명령 안내 |

> **남은 단계**: §4 헤드셋 측 Unity Client APK 켜고 Direction=Send 활성 → 실제 trigger/grip 데이터 흐름 확인 (사용자가 헤드셋 착용해야 함). 그 이후 Isaac Lab 텔레오퍼레이션 (§8).

---

## 3. MSVC + xrobotoolkit_sdk 설치 (✅ **11th session 에서 완료**)

> **상태 (2026-05-15 갱신)**: 사용자가 MSVC Build Tools 2019 를 `C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\` 에 설치 + 11th 세션이 `xrobotoolkit_sdk-1.0.2` 빌드 + 설치 + DLL 복사 완료. `python -c "import xrobotoolkit_sdk"` PASS, 38개 API 노출. `RoboticsServiceProcess` 띄운 상태로 `diagnose_xrobotoolkit --skip_live` 가 **ALL LAYERS PASS** (L1/L2/L3).
>
> 본 §는 처음부터 재설치 / 다른 환경 복제 시 참고용. 이미 동작 중이라면 §4 로 건너뛰어도 됨.

xrobotoolkit_sdk 의 pybind11 모듈은 Python 의 ABI 와 동일한 MSVC 로 빌드돼야 함.

### 3.1 MSVC Build Tools 2019 설치 (~10 min)

```powershell
# 1. Visual Studio Installer 다운로드 (administrator PowerShell)
$installerUrl = "https://aka.ms/vs/16/release/vs_buildtools.exe"
$installer = "$env:TEMP\vs_buildtools.exe"
Invoke-WebRequest -Uri $installerUrl -OutFile $installer

# 2. C++ Build Tools 워크로드 + Windows 10 SDK 설치
& $installer --quiet --wait --norestart --nocache `
    --add Microsoft.VisualStudio.Workload.VCTools `
    --add Microsoft.VisualStudio.Component.Windows10SDK.19041 `
    --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64
# 약 5-10 분 소요.

# 3. 검증
Get-ChildItem "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Tools\MSVC" -ErrorAction SilentlyContinue | Select-Object Name
# 11th 세션 사용자 PC: 14.29.30133 (v142)
```

> **사용자 PC 실제 경로 (11th 세션 검증)**:
> - MSVC 툴체인: `C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Tools\MSVC\14.29.30133\bin\Hostx64\x64\cl.exe`
> - vcvars64 진입점: `C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat`
> - Windows SDK: `C:\Program Files (x86)\Windows Kits\10\Include\10.0.19041.0` (+ 10.0.22621.0)
>
> **대안**: VS2022 Community / Professional 이 이미 설치돼 있으면 별도 BuildTools 불필요 — `vcvars64.bat` 경로만 그쪽으로 바꿔서 §3.2 진행.
>
> **참고**: VS2019 (v142) 이면 PC-Service v1.0.0 의 dll 과 ABI 일치 (Pybind 빌드 시 link 에러 회피).  VS2022 (v143) 도 호환되지만 일부 케이스에서 _ITERATOR_DEBUG_LEVEL 경고 가능.

### 3.2 xrobotoolkit_sdk 빌드 + 설치 (~5-10 min)

11th 세션이 `XRoboToolkit-PC-Service-Pybind/` 에 이미 준비:
- `include/PXREARobotSDK.h` ← `XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/` 에서 복사
- `include/nlohmann/json.hpp` + `json_fwd.hpp`
- `lib/PXREARobotSDK.dll` + `PXREARobotSDK.lib` ← `XRoboToolkit-PC-Service.win/SDK/x64/` 에서 복사

conda `ust` env 의 `pybind11 3.0.4` 도 설치됨. 즉, 다음만 실행 (11th 세션이 검증한 정확한 순서):

```powershell
cd C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service-Pybind

# 1. pybind11 cmake config 노출 (없으면 cmake 가 못 찾음)
$env:CMAKE_PREFIX_PATH = (& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -m pybind11 --cmakedir)

# 2. MSVC 환경 source + wheel build + pip install (한 cmd 세션에서)
& cmd /c '"C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat" && set PATH=C:\Qt\Tools\CMake_64\bin;C:\Qt\Tools\Ninja;%PATH% && "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -m pip install .'

# 출력 마지막 줄 기대: "Successfully installed xrobotoolkit_sdk-1.0.2"
# 빌드는 5~10분.  cl.exe C4244 warning 2건은 무해 (double->int).

# 3. PXREARobotSDK.dll 을 site-packages 에 같이 배치 (xrobotoolkit_sdk.pyd 옆에)
Copy-Item "C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service.win\SDK\x64\PXREARobotSDK.dll" `
          "C:\Users\pjwpy\miniconda3\envs\ust\Lib\site-packages\" -Force

# 4. 검증
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -c "import xrobotoolkit_sdk as xrt; print('OK', len([a for a in dir(xrt) if not a.startswith('_')]), 'APIs')"
# 기대: OK 38 APIs
```

> **setup_windows.bat 안 쓰는 이유**: 11th 세션 중 발견 — `setup_windows.bat` 가 `tmp/` 디렉토리에서 PC-Service 를 git clone 하면서 이미 준비된 `include/` `lib/` 를 일부 케이스에서 덮어쓸 수 있음. 위 수동 절차가 더 안전.

### 3.3 검증

```powershell
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -c "
import xrobotoolkit_sdk as xrt
for name in sorted([a for a in dir(xrt) if not a.startswith('_')]):
    print(' -', name)
"
```

11th 세션 검증 결과 (38 개 API):
```
 - close
 - device_control_json
 - get_A_button / get_B_button / get_X_button / get_Y_button
 - get_body_joints_acceleration / get_body_joints_pose
 - get_body_joints_timestamp / get_body_joints_velocity
 - get_body_timestamp_ns
 - get_headset_pose
 - get_left_axis / get_left_axis_click
 - get_left_controller_pose
 - get_left_grip
 - get_left_hand_is_active / get_left_hand_tracking_state
 - get_left_menu_button  ← 11th 세션에서 xrobo_sampler 가 활용
 - get_left_trigger
 - get_motion_timestamp_ns
 - get_motion_tracker_{acceleration,pose,serial_numbers,velocity}  ← Phase C
 - get_right_axis / get_right_axis_click
 - get_right_controller_pose
 - get_right_grip
 - get_right_hand_is_active / get_right_hand_tracking_state
 - get_right_menu_button
 - get_right_trigger
 - get_time_stamp_ns
 - init
 - is_body_data_available
 - num_motion_data_available
 - send_bytes_to_device
```

핵심 API (`init`, `close`, `get_left_controller_pose`, `get_left_trigger`, `get_left_grip`, `get_left_menu_button` 등) 가 모두 노출되면 §3 완료.

### 3.4 실패 패턴

| 증상 | 원인 | 해결 |
|---|---|---|
| `cmake error: pybind11 not found` | cmake 가 pybind11 cmake config 미발견 | `$env:CMAKE_PREFIX_PATH = (& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -m pybind11 --cmakedir)` 후 재시도 |
| `LINK error LNK1181: PXREARobotSDK.lib not found` | setup_windows.bat 의 git clone 단계가 lib 디렉토리 덮어씀 (이미 준비됐는데) | `Copy-Item C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service.win\SDK\x64\PXREARobotSDK.{dll,lib} C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service-Pybind\lib\ -Force` 후 재빌드 |
| `ImportError: DLL load failed while importing xrobotoolkit_sdk` | `PXREARobotSDK.dll` 이 egg 디렉토리에 없음 | §3.2 step 4 의 Copy-Item 재실행, 또는 PATH 에 lib 디렉토리 추가 |
| `Cannot open include file 'pybind11/pybind11.h'` | MSVC 가 pybind11 헤더를 못 찾음 | conda env 의 site-packages PATH 확인; `$env:CMAKE_PREFIX_PATH` 명시 |
| `error C2039: 'find': is not a member of 'std::string'` | C++14 미만 표준으로 컴파일됨 | CMakeLists.txt 의 `CMAKE_CXX_STANDARD 17` 적용 확인 (기본값) |

---

## 4. PC-Service 시작

```powershell
# A. 서비스 시작 (별도 PowerShell 창)
& "C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service.win\runService.bat"
# → "RoboticsServiceProcess.exe" 가 별도 콘솔에서 실행.
# → 출력: "PXREAServerConnect ..." 등.

# B. 검증
Get-Process RoboticsServiceProcess
# Handles  NPM(K)    PM(K)    WS(K)  CPU(s)     Id  ProcessName
# -------  ------    -----    -----  ------     --  -----------
#     ...     ...      ...      ...     ...    ...  RoboticsServiceProcess

# C. listen port 확인
Get-NetTCPConnection -State Listen |
    Where-Object OwningProcess -EQ (Get-Process RoboticsServiceProcess).Id |
    Select-Object LocalAddress, LocalPort
# 기대: 127.0.0.1:60061 (setting.ini 의 listenPort)
```

### setting.ini 주요 항목 (변경 시 §5 candidate_ports 도 같이 수정)

```ini
[Service]
listenAddr=127.0.0.1
listenPort=60061    # ← diagnose_xrobotoolkit.py 의 L2 probe candidate_ports
```

---

## 5. 헤드셋 측 Unity Client APK 시작

> **사용자 PC**: 이미 sideload 완료 — 추가 `adb install` 불필요.

1. PICO 헤드셋 착용
2. Apps → All → **XRoboToolkit** 아이콘 클릭
3. 메인 패널에 PC IPv4 가 detected hosts 에 표시되는지 확인 (5GHz 동일 WiFi 필수)
4. PC IP 선택 + 컨트롤러 **trigger 클릭** → "Connected" 변경 확인
5. 토글 설정:
   - **Head**: ON
   - **Controller**: ON ★ (필수 — 우리가 트리거/그립/포즈를 받는 채널)
   - **Hand**: OFF (현재 unused)
   - **Body**: OFF (Phase C 까지)
   - **PICO Motion Tracker (Independent)**: OFF
6. **Direction**: **Send** (헤드셋 → PC) ★
7. **Start** / **Begin Streaming** 클릭

`RoboticsServiceProcess` 콘솔에서 다음 출력 확인:
```
PXREADeviceFind: <PICO 4U serial>
PXREADeviceConnect
PXREADeviceStateJson: {"functionName":"Tracking","value":"{...controller...}"}
```

JSON 안 `"trigger": 0.x` 가 컨트롤러 grip/trigger 에 따라 변하면 페어링 OK.

---

## 6. 단독 SDK 검증 (Isaac Sim 의존성 없이)

```powershell
cd C:\develop\IsaacLab
$env:PYTHONPATH = "."

& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.minimal_pico_check --seconds 15
```

기대:
```
[1/3] xrobotoolkit_sdk imported.
[2/3] xrt.init() OK -- connected to PC service.
[3/3] Polling for 15s at 10 Hz.
      Squeeze grip/trigger on either controller -- values should respond.

  t=...  L: trig=0.00 grip=0.00 pose=[+0.123,+1.234,-0.567]  R: trig=0.00 grip=0.00
  ... (grip / trigger 당기면 0.x ~ 1.0 변화 출력) ...

============================================================
Summary  (150 samples)
============================================================
  max L_trigger = 0.85    L_grip = 0.92
  max R_trigger = 0.91    R_grip = 0.78

PASS -- xrobotoolkit_sdk receives controller analog input.
   Continue to Step 5 (coord_transforms unit test).
```

`PASS` 면 SDK 채널 healthy. `FAIL` 면 §7 layered diagnostic 으로.

---

## 7. Layered diagnostic (문제 발생 시)

```powershell
cd C:\develop\IsaacLab
$env:PYTHONPATH = "."

# 전체 4-layer 점검 (live data 포함, ~8 초)
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.diagnose_xrobotoolkit

# plumbing 만 확인 (live data skip)
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.diagnose_xrobotoolkit --skip_live
```

### Layer 매핑

| Layer | 검증 대상 | FAIL 시 조치 |
|---|---|---|
| **L1** | RoboticsServiceProcess 실행 + PICO Connect 충돌 검출 | §4 의 `runService.bat` 시작; PICO Connect 가 같이 떠 있으면 `taskkill /IM "Pico Connect.exe" /F` |
| **L2** | TCP listen port (60061 / 50051 / 12345 / 23306 sweep) | `setting.ini` 의 listenPort 확인 + Windows Firewall 의 RoboticsServiceProcess 인바운드 허용 |
| **L3** | `xrobotoolkit_sdk` import + `xrt.init()` 핸드셰이크 | §3 의 빌드 단계; DLL load 실패면 PATH / egg 디렉토리 확인 |
| **L4** | 8초 폴링 동안 trigger/grip ≥ 0.3 변화 검출 | Unity Client APK 의 Direction=Send 토글 / Controller 채널 ON 확인 |

---

## 8. Isaac Lab 텔레오퍼레이션 (XRoboToolkit 백엔드)

### 8.1 Monitor 모드 (XR 헤드셋 시각화 없이, 디버깅용)

> **⚠️ Isaac Sim 시작 단계 noise 안내**: 시작 직후 약 5-10초 동안 `Windows fatal exception: code 0xc0000139` + `isaacsim.sensors.rtx` / `_generic_model_output` DLL load 실패 같은 stack trace 가 출력될 수 있음.  **모두 비치명적** — Isaac Sim 의 선택적 RTX sensor 모듈이 누락된 상태(시스템마다 다름)에서 자동 fallback 한다는 의미.  `✅ READY` banner 가 뜨면 텔레오퍼레이션 루프가 정상 동작 중.  종료 시 `✅ NORMAL EXIT` (정상) 또는 `⚠️ ANOMALOUS EXIT` (이상) banner 로 구분 명시.

#### A. 5초 자동 smoke (env load + ~280 step + 정상 exit 확인)

```powershell
cd C:\develop\IsaacLab
$env:PYTHONPATH = "."

& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only `
    --render_mode monitor --headless `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit `
    --gripper_signal_source grip `
    --max_seconds 5 `
    --process_priority normal
```

> `--max_seconds 5` = 약 5초 후 정상 종료 (실제로 약 280 step ≈ 60 Hz 평균 처리율).
> `--headless` = Isaac Sim GUI 창 안 열어 디버깅 빠름.

기대 출력 (핵심):
```
[run_teleop] ⏳ Starting Isaac Sim (~10s).  You may see 'Windows fatal exception...' — they are NON-FATAL.
...
[XRoboSampler] xrt.init() OK; spawning poll thread at 120.0 Hz.
[GR1T2GripperDevice] --- XRoboToolkit channel probe ---
  left: trigger=0.000  grip=0.000  menu=False  pose=(+0.402,-0.408,+0.283)
  right: trigger=0.000  grip=0.000  menu=False  pose=(+0.436,-0.300,-0.229)
========================================================================
[run_teleop] ✅ READY — Isaac Sim + env_cfg loaded, teleop pipeline alive.
           render_mode=monitor, xr=False, input_backend=xrobotoolkit
           budget: max_seconds=5
========================================================================
[GR1T2Gripper #1 first-call] 16D action vector (L=default, R=default)
[GR1T2Gripper #20] L=controller R=controller | L_pos=(+0.402,-0.408,+0.283) ... L_cmd=+1 R_cmd=+1 | max_grip L=0.00 R=0.00
... (env.step 280회) ...
[run_teleop] reached --max_seconds=5s (actual=5.0s, steps=288); exiting.
========================================================================
[run_teleop] ✅ NORMAL EXIT — reason: reached --max_seconds=5s (actual=5.0s, steps=288)
           steps_completed=288, elapsed=5.0s, avg_rate=57.6 Hz
           simulation_app.is_running()=True
========================================================================
```

성공 기준:
- `✅ READY` banner 출력
- `✅ NORMAL EXIT` banner 출력 (`⚠️ ANOMALOUS` 가 아님)
- `Action Manager: 3 active terms (pink_ik_cfg=14, left_gripper_action=1, right_gripper_action=1)`
- `FATAL` / `Traceback` / `negative mass` 없음
- `controller` pose 가 (0,0,0) 아닌 실제 좌표 (헤드셋 / Unity APK 페어링 OK)

#### B. 라이브 인터랙티브 데모 (5분, GUI 창 + 사용자가 grip 토글 시각 확인)

```powershell
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only `
    --render_mode monitor `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit `
    --gripper_signal_source grip `
    --max_seconds 300 `
    --process_priority normal
```

> `--max_seconds 300` = 5분 자동 종료 (Ctrl-C 로 조기 종료도 가능 → `✅ NORMAL EXIT — reason: KeyboardInterrupt`).
> `--headless` 빼면 Isaac Sim GUI 창이 뜸 (env 로딩 ~10초 후).

이 모드에서 사용자가 확인:
1. Isaac Sim GUI 가 PC 모니터에 표시 (DESKTOP rendering)
2. 좌/우 controller 의 grip 당김 ≥ 0.6 → gripper close (~0.3s)
3. Controller 위치를 움직이면 robot wrist EEF 가 따라옴 (Pink IK)
4. `[GR1T2Gripper #N]` 로그가 약 5초마다 출력 — `max_grip` 값이 0.0 → 1.0 변하면 grip 입력 도달, `L_cmd / R_cmd` 가 -1 (close) 로 flip 하면 그리퍼 닫히는 중
5. `[run_teleop][XR-HEALTH] sampler snapshot is X.Xs old` warn 이 뜨면 → 헤드셋 측 Unity APK 의 Direction=Send 가 꺼졌거나 헤드셋 sleep — APK 측 토글 다시 확인

### 8.2 정량적 close 검증

```powershell
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.test_robotiq_close --headless
# 기대: VERDICT: PASS — lead 가 +0.785 도달, followers gear × +0.785
```

이 테스트는 backend 와 무관 (synthetic action 사용). XRoboToolkit 전환이 robot 거동에 영향 없음을 검증.

### 8.3 Pose drift 회귀

```powershell
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.test_robotiq_pose --headless
# 기대: VERDICT: PASS — 12 robotiq joint 전부 ±10° 이내 (best ~2.6°)
```

### 8.3.3 Phase D 풀바디 텔레오퍼레이션 (13th-bis session) — `--full_body True`

> **Status**: 부분 구현 완료. HMD → 로봇 head_yaw/pitch/roll, waist tracker → 로봇 waist_yaw/pitch/roll 가 매 frame side-channel 으로 driving.  Forearm tracker / elbow position / ankle 은 차후 Phase D++ 작업.

#### 동작

`run_teleop.py` 의 main loop 가 매 `env.step` 직전에 `_phase_d_apply(snapshot)` 호출:

1. **HMD → head joints**: `snap["hmd"]["quat"]` (IL frame wxyz) → ZYX Euler → `head_yaw_joint`, `head_pitch_joint`, `head_roll_joint` 의 position target.  Clamp 범위: yaw ±1.5 rad (86°), pitch ±1.0 rad (57°), roll ±0.7 rad (40°).  사용자가 머리를 돌리면 로봇 머리도 같이 돌아감.
2. **waist tracker → waist joints**: `snap["trackers"]["waist"]["quat"]` (IL frame wxyz) → ZYX Euler → `waist_yaw_joint`, `waist_pitch_joint`, `waist_roll_joint` position target.  Clamp 범위: yaw ±1.2 rad (69°), pitch ±0.6 rad (34°), roll ±0.5 rad (29°).  사용자가 허리를 틀면 로봇 torso 도 따라 돌아감.

Action manager 무관 — 16-D Pink IK + gripper action layout 그대로 유지.  Side-channel 로 articulation API 의 `set_joint_position_target(target, joint_ids=[...])` 호출.

#### CLI

```powershell
& "..." -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only --render_mode monitor `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit --xrt_enable_body True `
    --full_body True ` # ← 13th-bis session, default True
    --gripper_signal_source grip `
    --max_seconds 300 --process_priority high
```

`--full_body False` 로 비활성 가능 (wrist + gripper 만).

#### 출력 확인

```
[run_teleop][phase_d] joint resolution: head_ids=[21, 16, 11] waist_ids=[2, 5, 8]
[run_teleop][phase_d] first head target applied: yaw=+0.123 pitch=-0.087 roll=+0.045
```

3초마다 [TRACK step=N] 진단 에 HMD/waist 의 LIVE 값이 노출 — Phase D 가 driving 하고 있음을 의미.

#### 13th-bis session — 정자세 캘리브레이션 (auto, startup)

이전 (no calibration) 의 문제: 사용자가 의자에 앉아 약간 앞으로 굽어있으면 waist tracker 의 raw quat 이 그대로 robot 의 waist target 으로 매핑되어 robot 이 시작부터 허리 굽혀짐 → 어깨가 앞으로 → wrist target reach 안 됨 → 사용자가 컨트롤러를 머리 위로 들어야 함.

**자동 캘리브레이션 (13th-bis)**: `_phase_d_apply` 가 시작 시 **첫 valid (non-zero) HMD quat / waist quat / controller pos** 를 "zero" 로 캡쳐 → 이후 모든 target 은 delta-from-zero:
- `head_target = euler(raw_hmd_quat * inv(zero_hmd_quat))`
- `waist_target = euler(raw_waist_quat * inv(zero_waist_quat))`
- `wrist_target = idle_wrist_pos + (raw_controller_pos - zero_controller_pos)` (1:1 tracking)

기대 출력 (헤드셋 + body 토글 ON + 5개 PMT 페어링 상태):
```
[run_teleop][phase_d] HMD calibrated to zero quat = (-0.993, +0.005, +0.003, +0.116) — head deltas are now relative to this orientation.
[run_teleop][phase_d] waist calibrated to zero quat = (+0.776, +0.150, -0.044, +0.611) — torso deltas are now relative to this orientation.
[run_teleop][phase_d] wrist calibrated — L_zero=(+0.445, -0.037, -0.398) R_zero=(+0.423, -0.093, -0.405); robot now stays in idle T-pose at startup, wrist tracks user controller deltas 1:1.
[run_teleop][phase_d] first head target applied (delta from zero): yaw=+0.000 pitch=+0.000 roll=+0.000
```

`first head target = 0,0,0` 는 사용자가 캘리브레이션 시점 자세로 있을 때 robot 이 **idle T-pose 유지** 한다는 의미.

**캘리브레이션 시점에서의 사용자 자세 권장**:
- 의자에 등 편하게 앉거나 똑바로 서서
- 헤드셋 정면 향하기 (책상 화면 보는 자연스러운 자세)
- 양 컨트롤러 무릎 / 허리 옆 / 가슴 앞 - 어디든 편한 위치
- 이 자세 = robot 의 idle T-pose (양 팔 가슴 앞으로 살짝 든 자세) 로 매핑

이후 움직이는 동작이 robot 에 1:1 반영.

**Wrist 자세 (orientation) 도 calibration** (13th-bis session, part 7):
이전 part 5 에서는 controller **position** 만 calibration 했고 quaternion 은 raw 값을 retargeter 가 그대로 적용 → 사용자 controller 의 startup 방향이 robot wrist 의 영구 비틀림으로 매핑됨 (왼손이 뒤로 꺾인 채로 보임).  fix 후: `delta_q = raw_q * inverse(zero_q)`, `wrist_target_quat = delta_q * idle_q` (오른쪽은 `right_wrist_z180` Z180 보정 추가).  결과:
- 캘리브레이션 시 robot wrist quat = idle_q (양손 표준 idle 자세)
- 사용자가 컨트롤러를 회전 → robot wrist 가 동일 delta 만큼 회전

**Re-calibrate** (런타임 재 캘리브레이션) — **PICO 우측 컨트롤러 `A` 버튼**:

13th-bis session 추가: 우측 컨트롤러의 `A` 버튼 (lower face button) 을 누르면 즉시 모든 zero (HMD / waist / 양 wrist) 리셋 → 다음 frame 에서 **현재 자세** 를 새 zero 로 재 캡쳐.  의자에 자세를 바꿔 앉았거나 책상에서 일어났을 때 즉시 재 캘리브레이션 가능.

```
[run_teleop][phase_d] 🔄 A button pressed — re-calibration triggered.  
Hold your current posture for one frame; HMD / waist / wrist zeros 
will be re-captured to your CURRENT pose.
[run_teleop][phase_d] HMD calibrated to zero quat = (...) — head deltas are now relative to this orientation.
[run_teleop][phase_d] waist calibrated to zero quat = (...) — torso deltas are now relative to this orientation.
[run_teleop][phase_d] wrist calibrated — L_zero=(...) R_zero=(...) 
[run_teleop][phase_d] first head target applied (delta from zero): yaw=+0.000 pitch=+0.000 roll=+0.000
```

Rising-edge detection: A 를 한 번 누르면 정확히 1회 trigger.  잡고 있어도 추가 trigger 없음.  연속 트리거 방지 cooldown 0.5초 (button bounce 방어).

**즉시 default pose 로 snap** (13th-bis session, part 7): A 누름 시 `robot.write_joint_state_to_sim(default_joint_pos)` 호출 → 모든 joint 가 한 sim step 에 default 값으로 jump.  이전엔 PD control 의 transient (100~200ms) 동안 robot 이 현재 자세에서 idle 로 천천히 이동하면서 어색한 중간 자세 거쳤음.  지금은 `🔄 A button pressed` 출력과 동시에 robot 이 idle T-pose 로 즉시 복귀.

> PICO Touch 컨트롤러 face button convention: 우측 = **A** (lower) / B (upper), 좌측 = X (lower) / Y (upper).  `A` 가 가장 자연스럽고 안전한 위치 — Y/B 같은 upper button 은 grip / trigger 와 동시 조작이 가능해 충돌 위험.

#### 한계 + 차후 (Phase D++)

| 채널 | 현재 | Phase D++ |
|---|---|---|
| HMD | ✓ head 3 joints driving (yaw/pitch/roll) | 추가 X |
| Waist tracker | ✓ waist 3 joints driving | 추가 X |
| Forearm tracker | fallback only (controller absent 시) | Pink IK 에 elbow PositionTask 추가 → forearm 트래커가 elbow 위치 driving |
| Ankle tracker | excluded from role map ("발목 픽스") | (의도적 — leg actuation 없음) |
| Controller | PRIMARY wrist driver | 유지 |

Forearm tracker → elbow position driving 은 Pink IK 의 `variable_input_tasks` 에 `PositionTask("left_elbow_pitch_link", 3D pos)` × 2 추가 + retargeter ACTION_DIM 16 → 22 확장 + env_cfg idle_action 업데이트 + test_robotiq_pose/close 업데이트.  큰 작업이라 별도 세션.

### 8.3.4 트래커 → 로봇 매핑 (13th session 추가) — 어떤 트래커가 무엇을 구동하는지

**핵심**: 현재 16-D action layout (`[7 L wrist, 7 R wrist, 2 grippers]`) 은 wrist + gripper 만 actuated.  Head / torso / legs 액션 없음.  Pink IK 가 wrist target → 어깨/팔꿈치/손목 joint 를 solver 로 채움.

| XR 입력 | snapshot key | retargeter 사용 | 로봇 visible 효과 | 비고 |
|---|---|---|---|---|
| **Controller pose (양손)** | `controllers.left.pose`, `controllers.right.pose` | **PRIMARY** wrist target | 로봇 양 wrist 가 사용자 손 따라감 | `prefer_controller_for_eef=True` (default) |
| **Controller trigger/grip** | `controllers.{side}.buttons.{trigger,grip}` | gripper close/open | 로봇 grip 닫힘/열림 | `gripper_signal_source=grip` (default) |
| **Waist tracker** (body idx 0 = Pelvis) | `trackers.waist` | base_link origin XY+Z 빼기 | 미세함 — user 자세 변화 보정 | `use_waist_origin=True`, `subtract_waist_z=True` (13th-bis default) |
| **Forearm trackers** (body idx 18, 19 = L_Elbow, R_Elbow) | `trackers.{side}_forearm` | **fallback only** | controller 없을 때만 wrist 따라감 | `prefer_controller_for_eef=False` 로 active 가능 |
| **HMD pose** | `hmd` | **사용 안 함** | **없음** — robot 머리 미actuated | TRACK diag 에는 표시 |
| **Ankle trackers** (body idx 7, 8 = L_Ankle, R_Ankle) | (제외됨) | — | **없음** — 13th session 에 `_DEFAULT_BODY_ROLE_MAP` 에서 제거 | "발목 픽스" |

> ⚠️ 사용자가 "HMD 트래킹 / 모션트래커 트래킹 안 됨" 으로 인지하는 이유는 데이터 흐름 문제가 아니라 **현재 architecture 가 wrist+gripper 만 actuated 라서 head/torso/legs 트래커가 visible 효과가 없음**.  `[run_teleop][TRACK step=N]` 진단 (3초마다 자동) 으로 각 채널이 LIVE 인지 확인 가능.

**`--prefer_controller False` 모드** — forearm tracker 가 wrist 를 driving 하길 원할 때:

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only --render_mode monitor `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit --xrt_enable_body True `
    --prefer_controller False ` # ← forearm tracker 가 wrist driver
    --gripper_signal_source grip `
    --max_seconds 300 --process_priority high
```

이 모드에서: forearm tracker 가 wrist target 을 직접 driving.  controller 는 grip/trigger 만 (loose held → buttons 만 active).  user 가 양 elbow 트래커를 움직이면 로봇 wrist 가 명확히 따라감.

> **풀바디 텔레오퍼레이션** (head + torso + legs 도 driving) 은 향후 Phase D 작업.  16-D action → 30+-D action 확장 + Pink IK 에 head/pelvis FrameTask 추가 + env_cfg 에 head/torso/leg actuator 추가 필요.

### 8.3.5 채널 진단 (13th session 추가) — 어떤 트래킹이 살아있는지 즉시 확인

`run_teleop --input_backend xrobotoolkit` 시작 시 sampler 가 자동으로 channel probe 출력:

```
[XRoboSampler] channel probe:
  HMD pose:                   LIVE
  Body skeleton (24-joint):   AVAILABLE
  Independent PMT (Motion Tracker): n=5, serials=['PMT-...','...']
  Body role map (enable_body=True): {0: 'waist', 18: 'left_forearm', 19: 'right_forearm'}
```

해석:
- `HMD pose: ZERO` → 헤드셋이 책상 위거나 APK 의 Head 토글이 OFF.  헤드셋 착용 후 APK Head=ON 활성.
- `Body skeleton (24-joint): OFF` → APK Body 토글이 OFF.  활성하면 24-joint SMPL 추정이 도착 (waist + 양 forearm 자동 driver).
- `Independent PMT: n=0` → APK 의 "PICO Motion Tracker (Independent)" 토글 OFF.  HMD-based body 추정과 별개 채널 (5개 PMT 가 개별 streaming 되길 원할 때 사용).  현재는 body 24-joint 으로 5개 트래커가 묶여 들어오니 PMT 토글은 보통 OFF 로 둠.

**중요**: 13th session 이후 retargeter 가 XR 백엔드 시 `pose_in_il_frame=True` 자동 적용 — sampler 의 IL-frame 변환 (`xr_to_isaaclab`) 위에 legacy `svr_to_isaaclab` 가 한 번 더 적용되는 double-transform 버그 fix.  이 전엔 waist 트래커가 활성이어도 retargeter 가 좌표 mangling 으로 잘못된 wrist 타깃을 계산.

### 8.4 실연 (PC 모니터 화면 + XRoboToolkit input — **표준 경로**)

> **이게 사용자가 실제 텔레오퍼레이션 진행 시 쓰는 표준 명령**. PICO 단일 APK 제약 (§0 의 ⚠️ 박스) 때문에 PICO Connect 는 끄고 XRoboToolkit Unity Client APK 만 실행.  Isaac Sim GUI 는 PC 모니터에 표시되고, 사용자는 모니터 보면서 헤드셋 컨트롤러로 조작.  HMD stereo 렌더링은 §10 (Phase B) 의 CloudXR / ALVR 같은 별도 경로 필요.

#### 사전 조건 (한 번만 점검)

```powershell
# 1. PICO Connect 가 떠 있으면 종료 (단일 APK 제약)
Get-Process "Pico Connect" -ErrorAction SilentlyContinue | Stop-Process -Force
# 또는 PICO Connect 콘솔에서 "Disconnect" 직접 클릭.

# 2. SteamVR 도 떠 있으면 종료 (XRoboToolkit 는 SteamVR 불요)
Get-Process vrserver, vrmonitor, vrcompositor -ErrorAction SilentlyContinue | Stop-Process -Force

# 3. RoboticsServiceProcess (PC-Service) 실행 확인
Get-Process RoboticsServiceProcess -ErrorAction SilentlyContinue
# 없으면 §4 의 runService.bat 실행

# 4. 헤드셋 측에서 Apps → "XRoboToolkit" → PC IP 페어링 → Controller=ON, Direction=Send → Start
```

#### 라이브 데모 (5분 자동 종료, 컨트롤러로 grip / 위치 조작)

```powershell
cd C:\develop\IsaacLab
$env:PYTHONPATH = "."

& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only `
    --render_mode monitor `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit `
    --xrt_enable_body True `
    --gripper_signal_source grip `
    --max_seconds 300 `
    --process_priority high
```

> `--xrt_enable_body True` (13th session, **default**) — body 24-joint 트래킹 활성 → waist + 양 forearm 트래커가 retargeter 에 채워짐.  발목 (LEFT_ANKLE / RIGHT_ANKLE) 는 default role map 에서 제외돼 무시 (사용자 요청 "발목 픽스").  헤드셋의 Unity Client APK Body 토글도 같이 ON 이어야 실제 데이터 도착.

> Ctrl-C 로 조기 종료 시 `✅ NORMAL EXIT — reason: KeyboardInterrupt` 가 마지막 banner 로 출력.  더 길게 돌리려면 `--max_seconds 1800` (30분) 등으로 키워라.  완전 무제한 (Ctrl-C 만으로 끝내려면) 은 `--max_seconds` 인자 빼면 됨.

사용자가 확인 (PC 모니터):
1. Isaac Sim GUI 창이 PC 모니터에 표시 (DESKTOP rendering)
2. 좌/우 controller 의 grip 당김 ≥ 0.6 → gripper close (~0.3 s)
3. 좌/우 controller 위치 → robot wrist EEF target (Pink IK)
4. `[GR1T2Gripper #N]` 로그가 약 5초마다 출력 — `max_grip` 값이 0.0 → 1.0 변하면 grip 입력 도달, `L_cmd / R_cmd` 가 -1 (close) 로 flip 하면 그리퍼 닫히는 중
5. `[run_teleop][XR-HEALTH] sampler snapshot is X.Xs old` 가 뜨면 → Unity APK 의 Direction=Send 꺼졌거나 헤드셋 sleep — 헤드셋 측 APK 다시 ON

> **`--process_priority high`**: 120Hz 빡빡 루프가 input thread starve 시키지 않도록 16-frame 마다 추가 `simulation_app.update()`.  GUI 클릭 / 메뉴 동작 보장.  ([claude.md §3.6](claude.md))

### 8.5 ignore_arms 모드 (그리퍼 close 단독 검증)

```powershell
# 팔은 idle T-pose 로 lock, 그리퍼만 동작
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit `
    --gripper_signal_source grip `
    --ignore_arms True `
    --render_mode monitor
```

10th session 의 차단을 처음으로 우회한 시나리오. 그립 당김이 그리퍼 close 로 즉시 전달되는지 시각 확인.

---

## 9. 백엔드 비교 (openvr ↔ xrobotoolkit)

| 측면 | `--input_backend openvr` | `--input_backend xrobotoolkit` |
|---|---|---|
| 데이터 경로 | PICO controller → PICO Connect 의 prism driver → SteamVR Action API → openvr Python binding | PICO controller → Unity Client APK → XRoboToolkit-PC-Service (gRPC) → xrobotoolkit_sdk pybind |
| Personal Binding 필요? | **YES** (10th session 차단 원인) | NO |
| SteamVR 실행 필요? | YES | **NO** (XRoboToolkit 는 자체 gRPC).  `--render_mode steamvr_native` 와 조합 시 PICO Connect 가 필요하지만, PICO Connect 는 **XRoboToolkit Unity APK 와 충돌** → 본 가이드 권장 X. |
| 좌표계 변환 | `R_SVR2IL`, `svr_to_isaaclab` | **`R_XR2IL`** (proper rotation, det=+1), `xr_to_isaaclab` |
| Quaternion 순서 | wxyz (openvr 가 이미 wxyz) | xyzw (OpenXR XrQuaternionf) → `xyzw_to_wxyz` 으로 reorder |
| Snapshot dict | 동일 형식 (5 키: timestamp/hmd/trackers/hands/controllers/frame_count) | 동일 — retargeter / env_cfg 무수정 |
| Body mocap (Phase C) | Virtual Desktop body segments or PICO Connect PMT | XRoboToolkit body 24-joint (`--xrt_enable_body True`) |
| 차선책 (현재 환경) | 키보드 fallback or ALVR 교체 (memory.md §10 의 6-path 차단) | **본 가이드의 표준 경로** |

---

## 10. HMD 시각화 Options (Phase B / C)

> **결정적 제약**: PICO 4 Ultra 의 OS 는 **한 번에 하나의 stream APK 만** 활성 — XRoboToolkit Unity Client 와 PICO Connect 의 in-headset companion 은 **mutually exclusive**.  실험적으로 PICO Connect 의 SteamVR session 이 시작하는 순간 XRoboToolkit APK 가 즉시 종료된다 (12th session, 사용자 보고).

### 10.1 Phase A (현재) — PC 모니터 화면만

이 가이드의 모든 표준 명령 (§8) 의 default.  Isaac Sim GUI 가 PC 모니터에 표시되고 사용자는 모니터를 보며 컨트롤러로 조작.

| 장점 | 단점 |
|---|---|
| Setup 가장 간단 | HMD immersion 없음 |
| 컨트롤러 입력 100% 신뢰 (XRoboToolkit gRPC) | 사용자 시야가 PC 모니터에 묶임 |
| PICO Connect / SteamVR 의존성 없음 | 책상에 앉은 채 헤드셋 컨트롤러 들고 모니터 보기 어색 |
| 디버깅 / 로그 보기 쉬움 | — |

### 10.2 Phase B — HMD stereo 렌더링 (XRoboToolkit 이외 input 경로 필요)

PICO Connect 를 켜면 Unity Client APK 종료되므로, HMD video 가 필요하면 **input 경로를 XRoboToolkit 가 아닌 것** 으로 교체해야 함:

| 옵션 | Input 경로 | Video 경로 | Setup 난이도 | 비고 |
|---|---|---|---|---|
| **B-1: CloudXR** | XRoboToolkit (Unity APK 가 CloudXR + XRoboToolkit 동시 처리?) | CloudXR Runtime 6.0.1 → 헤드셋 | 높음 | research/24 의 통합 가이드.  현시점 미검증, 향후 도전 |
| **B-2: ALVR** | ALVR 의 OpenXR 컨트롤러 입력 (SteamVR 우회) | ALVR 의 자체 streaming | 중간 | XRoboToolkit 안 씀.  research/45 §B |
| **B-3: 키보드 fallback** | PC 키보드 (`C` / `V` 키 = 좌/우 close) | Isaac Sim GUI 가 모니터 | 낮음 | 1-2h 구현, 컨트롤러 pose 없음.  단순 데모 / data collection 용 |
| **B-4: Virtual Desktop** | VD 의 OpenXR + SteamVR Action API | Virtual Desktop streaming | 중간 | $20-30 라이선스.  10th session 의 PCVR Input 차단 재발 위험 |

> Phase B 모든 경로가 **XRoboToolkit 의 PICO Connect 충돌 문제** 를 우회하지만, 본 가이드 §8 의 표준 경로 (PC 모니터 + XRoboToolkit) 가 가장 안정적.  HMD 시각화가 필요한 시점이 명확해지면 그때 Phase B 선택 + 별도 가이드 작성.

### 10.3 Phase C — 5-Tracker Body Mocap

§11 참고.  Phase A 위에 `--xrt_enable_body True` 만 추가 — HMD 시각화 무관.

---

## 11. 5-Tracker Body Mocap (Phase C 비고)

PICO Motion Tracker 5개를 페어링 후 다음 옵션 추가:

```powershell
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --input_backend xrobotoolkit `
    --xrt_enable_body True `
    ...
```

`XRoboSampler._read_body_joints` 가 24-joint body mocap 의 다음 인덱스를 ust_hm_grip 의 tracker role 로 매핑:

| Body joint idx | Role |
|---|---|
| 0  (Pelvis) | `waist` |
| 7  (LEFT_ANKLE) | `left_ankle` |
| 8  (RIGHT_ANKLE) | `right_ankle` |
| 18 (LEFT_ELBOW) | `left_forearm` (forearm-mounted tracker semantically = elbow) |
| 19 (RIGHT_ELBOW) | `right_forearm` |

사전 작업:
1. PICO Motion Tracker 5개 페어링 (Unity Client APK → PICO Motion Tracker 토글 ON)
2. 헤드셋 calibration 진행
3. 정상 인덱스 검증:
   ```powershell
   & "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -c "
   import xrobotoolkit_sdk as xrt
   import numpy as np
   xrt.init()
   print('Body data available:', xrt.is_body_data_available())
   print('Body 24-joint shape:', np.asarray(xrt.get_body_joints_pose()).shape)
   xrt.close()
   "
   ```

`config/xrobotoolkit_settings.json` 의 `body_role_map` 으로 다른 매핑 override 가능 (현재는 default `_DEFAULT_BODY_ROLE_MAP` 만 코드에 내장; settings.json 로딩 hook 은 미구현 — Phase C 시 추가).

---

## 12. 에러 → 복구 매트릭스 (운영 중)

| 에러 | 발생 시점 | 복구 |
|---|---|---|
| `xrt.init() failed: connection refused` | `device.start()` | `runService.bat` 재실행 |
| `xrt.init() failed: timeout` | `device.start()` | Windows Firewall 확인 + 5GHz WiFi |
| `RuntimeError: xrobotoolkit_sdk is not installed` | `XRoboSampler.start()` | §3 의 빌드 단계 (특히 §3.2) |
| `frame_count == 0` 5초 이후 | 런타임 | Unity Client APK Direction=Send 확인 |
| `analog values 영원히 0.0` | 런타임 | 컨트롤러 페어링 재시도; 헤드셋 재시작 |
| `Snapshot 의 pose 가 NaN` | 런타임 | `XRoboSampler._is_zero_pose` 가 자동 reject — 별도 조치 불필요 |
| `RoboticsServiceProcess.exe crash` | 런타임 | Task Manager kill + `runService.bat` 재시작 |
| Isaac Sim 창 클릭 안 됨 | `--process_priority high` | `run_teleop.py` 가 자동 `time.sleep(0)` + 16-frame `simulation_app.update()` |

---

## 13. 회귀 안전망

### 13.1 OpenVR backend 로 즉시 복귀

```powershell
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -X utf8 `
    -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only `
    --render_mode monitor --headless `
    --diag idle --steps 5 `
    --process_priority normal
# --input_backend openvr 가 default — 명시 안 해도 됨
```

11th 세션 검증 시 5-step monitor 회귀 PASS 확인됨.

### 13.2 전체 unit test

```powershell
$env:PYTHONPATH = "."
& "C:\Users\pjwpy\miniconda3\envs\ust\python.exe" -m pytest ust_ws/ust_hm_grip/tests/ -v
# 기대: 62 / 62 PASS (43 기존 + 11 coord_transforms_xr + 8 xrobo_sampler)
```

### 13.3 git rollback (필요 시)

```powershell
# 11th 세션 전체 되돌리기
git log --oneline -10
git checkout <pre-11th-commit-sha>

# 또는 device 만 되돌리기 (samples 보존)
git checkout HEAD~ -- ust_ws/ust_hm_grip/teleop/gr1t2_gripper_device.py
git checkout HEAD~ -- ust_ws/ust_hm_grip/scripts/run_teleop.py
```

---

## 14. 11th 세션 결정 — 가이드 #47 §7 의 R_XR2IL 보정

> **버그**: 가이드 #47 §7 의 `R_XR2IL = [[0, 0, 1], [-1, 0, 0], [0, 1, 0]]` 은 **det = -1 (improper rotation / reflection)**. 두 right-handed 좌표계 사이의 변환이라면 det 가 +1 이어야 함. 또한 `matrix_to_quat_wxyz(R_XR2IL)` 는 proper rotation 만 표현 가능한 quaternion 으로 변환할 때 R_XR2IL 과 **다른 행렬** 을 반환 → quaternion conjugation 결과가 geometry 와 불일치.
>
> **수정**: OpenXR LOCAL 의 +Z 가 *user 쪽으로 향함 (back)* 이라는 Khronos 사양을 채택해 `R_XR2IL = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]` 로 변경. 이 행렬은 proper rotation 이고 (det=+1), `matrix_to_quat_wxyz(R_XR2IL) → quat_wxyz_to_matrix → R_XR2IL` 가 round-trip 성립.
>
> **테스트 영향**:
> - XR (0,0,1) → IL (1,0,0) **이었던 것** → XR (0,0,1) → IL (-1,0,0) **로**
> - XR (1,2,3) → IL (3,-1,2) **이었던 것** → XR (1,2,3) → IL (-3,-1,2) **로**
> - XR (0.1,1.2,0.5) → IL (0.5,-0.1,1.2) **이었던 것** → XR (0.1,1.2,0.5) → IL (-0.5,-0.1,1.2) **로**
> - quat 테스트: **이제 PASS** (90° about XR +X → 90° about IL -Y, 예상대로)
>
> 사용자 PC 의 실제 PICO controller 위치에서:
>   - 컨트롤러가 user 앞 (forward) → XR (0, ~1.2, -0.5) → IL (+0.5, 0, ~1.2) ← robot 의 forward 와 일치
>   - 컨트롤러가 user 오른쪽 (right) → XR (+0.5, ~1.2, 0) → IL (0, -0.5, ~1.2) ← robot 의 right (-Y) 와 일치
>   - 컨트롤러 위 (up) → XR (0, ~1.5, 0) → IL (0, 0, ~1.5) ✓
>
> 이 매핑이 retargeter 의 `prefer_controller_for_eef=True` 분기와 직접 호환.

---

## 15. 참고 링크

- 11th 세션 구현 사양: [`../research/47. xrobotoolkit_implementation_guide.md`](../research/47.%20xrobotoolkit_implementation_guide.md)
- 10th 세션 차단 결론: [`memory.md`](memory.md) §10 (PCVR Input 차단 6-path)
- 옵션 비교 design guide: [`../research/45. pcvr_input_unblock_options_research.md`](../research/45.%20pcvr_input_unblock_options_research.md)
- 마이그레이션 설계 (개요): [`../research/46. xrobotoolkit_migration_design_guide.md`](../research/46.%20xrobotoolkit_migration_design_guide.md)
- XRoboToolkit 공식 GitHub: https://github.com/XR-Robotics
  - [PC-Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)
  - [Unity-Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client)
  - [Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind)
- OpenXR LOCAL space spec: https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#XR_REFERENCE_SPACE_TYPE_LOCAL
- 사용자 PC 의 SDK 사본: [`../XRoboToolkit-PC-Service.win/SDK/include/PXREARobotSDK.h`](../XRoboToolkit-PC-Service.win/SDK/include/PXREARobotSDK.h)

---

## 16. 최종 체크리스트 (이 가이드를 따른 후)

- [ ] §3 MSVC + xrobotoolkit_sdk 빌드 — `python -c "import xrobotoolkit_sdk"` 성공
- [ ] §4 `RoboticsServiceProcess` 실행 — port 60061 listen
- [ ] §5 Unity Client APK Direction=Send + Connected
- [ ] **PICO Connect 종료** (§0 의 ⚠️ 단일 APK 제약): `Get-Process "Pico Connect" -ErrorAction SilentlyContinue | Stop-Process -Force`
- [ ] §6 `minimal_pico_check.py` — `PASS -- xrobotoolkit_sdk receives controller analog input.`
- [ ] §7 `diagnose_xrobotoolkit.py` — `ALL LAYERS PASS`
- [ ] §8.1.A 5초 smoke (`--max_seconds 5 --headless`) — `✅ NORMAL EXIT` banner
- [ ] §8.2 `test_robotiq_close.py` — `VERDICT: PASS`
- [ ] §8.3 `test_robotiq_pose.py` — `VERDICT: PASS`
- [ ] §8.4 실연 — `--render_mode monitor --max_seconds 300 --process_priority high` 으로 PICO 컨트롤러로 라이브 조작
- [ ] §8.5 `--ignore_arms True` — 그리퍼 close 시각 확인
- [ ] §13.2 전체 unit test — 62 / 62 PASS
- [ ] §13.1 회귀 (`--input_backend openvr` default) — 5-step monitor smoke PASS

이상이 완료되면 10th 세션의 PCVR Input 차단이 **확실히 해소**됐다고 판단 — PICO grip 당김이 Isaac Lab GR1T2 그리퍼 close 로 실시간 도달.
