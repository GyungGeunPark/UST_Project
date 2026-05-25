# ust_hm_grip 실행 가이드 (option B 정식 그리퍼 마이그레이션)

> **작성일**: 2026-05-04
> **기반 리서치**: [`research/36. gripper_elbow_tracker_pico_controller_migration_design_guide.md`](../research/36.%20gripper_elbow_tracker_pico_controller_migration_design_guide.md)
> **마이그레이션 베이스**: `ust_fourier_260421` (9.13~9.27 fix 누적)
> **변경 핵심**: 6-DoF Fourier 손 22D → 2-finger 그리퍼 16D, UDCAP 글러브 제거, PICO Touch 컨트롤러 트리거로 그리퍼 제어, 모션 트래커 손목→팔꿈치 이동.

---

## 0. 한 페이지 요약

| 항목 | 값 |
|---|---|
| 로봇 | Fourier GR1T2 (54 DoF) + 2-finger 그리퍼 (양손) |
| 손 액션 | 16-D = 7 (L wrist) + 7 (R wrist) + 2 (binary gripper) |
| 손목 EEF 1차 소스 | **PICO Touch 컨트롤러 pose** (raw) |
| 손목 EEF fallback | 팔꿈치 트래커 (`*_arm_lower`) + 0.28 m forearm offset |
| 그리퍼 제어 | **트리거 analog → hysteresis [0.4, 0.6] → ±1** |
| UDCAP / Windows 미니 PC | **사용 안 함** (모두 비활성화 권장) |
| 새 Gym ID | `Isaac-KitchenSorting-GR1T2-Gripper-{v0, WaistEnabled, Monitor, VR, Vision, DataCollect, RobotOnly}-v0` |
| 새 SteamVR app_key | `ust.teleop.gr1t2_gripper` |

---

## 1. 사전 준비 (1회)

### 1.1 USD 빌드 — 그리퍼 부착 GR1T2

```powershell
# Isaac Sim Python 환경에서 1회 실행
./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_gripper_usd.py
```

기대 결과:
```
[build_gripper_usd] source = '<...>/GR1T2_fourier_hand_6dof.usd'
[build_gripper_usd] output = '<...>/ust_hm_grip/isaac_file/GR1T2_with_gripper.usd'
[build_gripper_usd] removed N Fourier-hand prims.
[build_gripper_usd] DONE — gripper-equipped GR1T2 USD written to ...
```

이 USD가 없으면 env_cfg는 stock GR1T2 USD로 폴백합니다 (경고 메시지 출력). 그러면 손은 여전히 5-finger 휴머노이드 손 모양으로 보이지만 액션 매니저 측에서는 그리퍼 인덱스가 일치하지 않아 작동이 어색합니다 — 반드시 USD 빌드 먼저 수행하세요.

### 1.2 UDCAP / VMC 비활성화 (사용자 측 작업)

| 단계 | 작업 |
|---|---|
| 1 | Windows 미니 PC 의 `UdcapDriver.exe` 자동 시작 비활성 (서비스 또는 시작 프로그램에서 제거) |
| 2 | SteamVR 종료 → Add-on 폴더 (`<SteamVR>/drivers/`) 에서 `udcap` 폴더 이름을 `udcap_disabled` 로 임시 rename |
| 3 | UDCAP system tray 위젯 종료 |
| 4 | (필요시) 미니 PC ↔ 메인 PC VMC OSC UDP 39539 라우팅 차단 |

이후 SteamVR 재시작 시 `Prop_ControllerType_String` 가 `pico_neo3_controller` 또는 `oculus_touch` 로 보고되어야 합니다 (`knuckles` 아님).

### 1.3 Streaming Layer — `pico` (PICO Connect) primary, **다른 모든 add-on OFF**

> **9.34+ memory.md §10.42 / §10.43**: SteamVR Add-Ons 패널의 add-on 라벨링이
> 시간에 따라 변했습니다.  현재 (2026-05+) 기준 정확한 매핑:
>
> | Add-On 라벨 | 진짜 정체 | 위치 |
> |---|---|---|
> | **`pico`** | **PICO Connect** 의 SteamVR 드라이버 (PICO Inc. 외부 driver) | `C:/Program Files/PICO Connect/openvr_driver/` |
> | **`prism`** | **Steam Link** 의 SteamVR 드라이버 (Valve 가 SteamVR 에 번들링) | `C:/Program Files (x86)/Steam/.../SteamVR/drivers/prism/` |
> | `Virtual Desktop Streamer (Quest)` | Virtual Desktop (paid) | 별도 |
> | `udcap` | UDCAP 글러브 driver (decommissioned) | 별도 |
>
> **반드시 하나만 ON**.  두 HMD-redirecting driver 가 동시 ON 이면 driver 충돌.

#### 전신 제어 (Body Tracking) 사용 시 — **PICO Connect (`pico`) 권장**

전신 제어에 다음이 필요하면:
- ✅ Forearm Tracking Enhanced (팔뚝 트래커 augmentation)
- ✅ AI Body Tracking (hips / chest / legs 추정)
- ✅ Pico Motion Tracker 펑크 (벨트/팔뚝 등 물리 트래커)

→ **`pico` 만 ON**, 나머지 모두 OFF.

| Add-On | 상태 |
|---|---|
| **pico (PICO Connect)** | **ON** ★ primary |
| prism (Steam Link) | OFF |
| Virtual Desktop Streamer (Quest) | OFF |
| udcap | OFF (글러브 안 쓰면) |

##### PICO Connect (Windows app) 설정

| 항목 | 값 |
|---|---|
| Settings → General → Controller Type | **Default** (Index Knuckles 가 아님) |
| Settings → General → Controller Priority | **High** |
| Settings → General → Streaming Assistant Compatibility Mode | `Default` (controller_type=`pico_controller`).  `Quest/OpenXR Compatibility` 도 OK (controller_type=`oculus_touch`) — 9.34 binding 은 둘 다 매칭 |
| Settings → Streaming → **Body Tracking** | **Forearm Tracking Enhanced** ON (팔뚝 트래커 augmentation) |
| Settings → Streaming → **AI Body Tracking** | 본인 신체 비율에 맞춰 ON (hips/chest/legs 가상 트래커 emit) |
| Settings → Streaming → Hand Tracking | 컨트롤러만 쓰면 OFF (`PICO_HAND_*` 가짜 controller 제거 → 진단 깔끔) |

##### PICO HMD 측 클라이언트

- **PICO Connect Streaming Assistant** 실행 (PICO 기본 탑재 앱)
- Steam Link 앱은 **사용하지 않음** (그쪽은 prism 경로)

##### Pico Motion Tracker (물리 트래커) — 선택, 정확도 향상

- PICO HMD: Settings → Motion Tracker → 트래커 페어링
- Forearm Tracking Enhanced 캘리브레이션 수행 (PICO Connect Windows 앱이 가이드 제공)
- 우리 코드의 `tracker_binding.json` 키 (`hips`, `chest`, `left_arm_lower`, …) 는 PICO Connect 의 AI body tracking 가상 트래커 시리얼과 직접 매칭됨 — 코드 변경 불필요

##### Controller_type 매핑 (검증됨)

| PICO Connect 모드 | controller_type | 9.34 default_bindings 매칭 |
|---|---|---|
| Default (Pico Connect 6.x+, PICO 4 Ultra) | `pico_controller` | ✓ |
| Default (Pico Connect 5.x, PICO Neo 3) | `pico_neo3_controller` | ✓ |
| Quest/OpenXR Compatibility | `oculus_touch` | ✓ |

#### 대안 1: 전신 제어 불필요 시 — Steam Link (`prism`)

컨트롤러만으로 충분하고 PICO body tracking 기능을 안 쓰면 **`prism` (Steam Link)**
가 더 단순/안정적 (Valve 의 official driver, Pico Connect Windows 앱 의존성 없음):

| Add-On | 상태 |
|---|---|
| pico (PICO Connect) | OFF |
| **prism (Steam Link)** | **ON** |
| Virtual Desktop Streamer (Quest) | OFF |
| udcap | OFF |

→ PICO HMD 의 Steam Link 앱 → PC 의 Steam 자동 연결.  PICO Connect Windows 앱 불필요.

#### 대안 2: Virtual Desktop (paid)

VD 를 이미 구입했고 익숙하면:

| Add-On | 상태 |
|---|---|
| **Virtual Desktop Streamer (Quest)** | **ON** |
| pico | OFF |
| prism | OFF |
| udcap | OFF |

VD Streamer (Windows tray) 설정: `Disable VR pointer in apps = ON`, `Forward tracking to SteamVR = ON`, `Forward controller input to SteamVR = ON`.  controller_type=`oculus_touch`.

### 1.4 SteamVR 바인딩 활성 (1회 — 9.32 이후 변경 시 필수 재실행)

1. SteamVR 시작 + 컨트롤러 + HMD 페어링
2. `python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper` 한 번 실행 — 첫 실행 시 manifest 가 SteamVR 에 등록됨
3. SteamVR → Settings → Controllers → **Manage Controller Bindings**
4. 드롭다운 → "UST Teleop GR1T2 Gripper" 선택
5. **9.32 fix 적용 직후 또는 streaming layer 변경 직후**: 만약 Personal Binding 이 활성화 되어 있다면 우측 점-3개 메뉴 → **"Replace with Default"** 클릭 (이전 `force_sensor` 또는 다른 streaming layer 의 캐시 제거)
6. **Edit This Binding** → 좌/우 컨트롤러 모두에서:
   - `Trigger` 입력 → mode = **Trigger**, output = **`Pull` → trigger_left/right**
   - `Grip` 입력 → mode = **Trigger** (NOT 'Force Sensor'), output = **`Pull` → grip_left/right**
7. 화면 하단 **"Save Personal Binding"** 클릭 (이름 그대로 두면 됨)
8. "Manage Controller Bindings" 로 돌아가 방금 저장한 binding 이 **Active** 인지 확인

### 1.5 모션 트래커 부착

```
좌측 트래커 → 좌측 팔뚝 (팔꿈치 위 ~5cm 지점)
우측 트래커 → 우측 팔뚝 (팔꿈치 위 ~5cm 지점)
hips 트래커 (선택) → 골반
```

PICO Connect "Forearm Tracking Enhanced" 모드에서 양 팔뚝 트래커가 `TrackerRole_LeftElbow` / `TrackerRole_RightElbow` 로 SteamVR 에 보고됩니다.

---

## 2. 검증 (실행 전 확인)

### 2.1 Smoke test (Isaac Sim 없이 — 코드 정합성 검증)

```bash
PYTHONPATH=. python -X utf8 ust_ws/ust_hm_grip/scripts/smoke_test.py
```

기대 결과:
```
ust_hm_grip smoke test
============================================================
  [PASS] 1. GR1T2GripperRetargeterCfg defaults
  [PASS] 2. retarget output 16D float32 — shape=(16,) dtype=torch.float32
  [PASS] 3. Idle action = idle pose + gripper open — ...
  [PASS] 4. Trigger > 0.6 closes (left only) — L_grip=-1.0 R_grip=1.0
  [PASS] 5. Hysteresis: 0.8→0.5 holds, 0.5→0.3 opens — mid_L=-1.0 open_L=1.0
  [PASS] 6. GR1T2GripperDeviceCfg standalone instantiation
  [PASS] 7. forearm_to_wrist + svr_to_isaaclab chain — pos_il=[...]
============================================================
OK -- 7/7 passed
```

### 2.2 Pytest regression suite (Isaac Sim 없이)

```bash
PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_grip/tests/ -v
```

기대 결과: 30+ tests passed

### 2.3 SteamVR + PICO 컨트롤러 연결 확인

```bash
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper
```

### 2.4 PICO Connect → SteamVR → Isaac Lab 파이프라인 진단 (9.37)

PICO Connect (`prism` 드라이버) 를 통해 PICO Motion Tracker 까지 라우팅하는
경우 ─ 즉 `--vr_runtime pico_connect` 사용 시 ─ 6-layer 진단 스크립트로
파이프라인 무결성을 확인하세요. 트래커 인벤토리, 드라이버 등록, 바인딩
파일 placeholder 잔존 여부까지 한 번에 체크합니다.

```bash
# 1) PICO Motion Tracker 시리얼을 자동으로 감지해 템플릿에 채워 넣기
python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers `
    --out ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json

# 2) 6-layer 진단 (PICO Connect 프로세스 + prism 드라이버 + OpenVR 인벤토리 + 바인딩)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pico_connect
```

기대 출력 (모든 레이어 PASS 시):
```
======================================================================
 PICO Connect -> SteamVR -> PC -> Isaac Lab pipeline diagnosis
 (ust_hm_grip / 16-D gripper track)
======================================================================

[ OK ] 1. PICO Connect Streaming Service
[ OK ] 2. SteamVR drivers (prism / vd / udcap)
[ OK ] 3-5. OpenVR devices (HMD + PICO trackers + controllers)
[ OK ] 6. tracker_binding_pico_connect.json
...
======================================================================
 Overall: PIPELINE READY
======================================================================
```

`[FAIL]` 이 뜨면 그 줄 밑의 reason 텍스트가 그대로 다음 액션을 알려줍니다.
가장 흔한 실패 원인:
* Layer 1: PICO Connect 미실행 → Start Menu 에서 PICO Connect 실행 + 헤드셋 페어링
* Layer 2: `prism` 드라이버 미등록 → SteamVR > Manage Add-Ons 에서 prism ON
* Layer 5: PMT 트래커 0개 → 트래커 충전 + 페어링 + Forearm Tracking Enhanced 활성
* Layer 6: 바인딩 파일에 `PMT_REPLACE_ME_*` placeholder 잔존 → §2.4 step 1 의
  `enumerate_trackers --out` 로 실제 시리얼로 덮어쓰기

10초 동안 트리거를 당기면:
```
  t= 1.0s  L_trig=0.85 R_trig=0.00  L_grip_cmd=-1 R_grip_cmd=+1
  t= 1.5s  L_trig=0.40 R_trig=0.00  L_grip_cmd=-1 R_grip_cmd=+1   ← hysteresis: 여전히 close
  t= 2.0s  L_trig=0.10 R_trig=0.00  L_grip_cmd=+1 R_grip_cmd=+1   ← open
============================================================
OK — trigger input observed.  Gripper hysteresis is working.
```

`L_trig` 가 0 으로만 보인다면 §1.5 SteamVR 바인딩 단계를 다시 수행하세요.

---

## 3. 라이브 텔레오퍼레이션

### 3.1 Monitor 모드 (PC 화면, 권장 첫 실행)

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only `
    --teleop_device pico_gripper `
    --vr_runtime pico_connect `
    --render_mode monitor `
    --process_priority high
```

> **9.37 — `--vr_runtime pico_connect`** 를 추가하면:
> 1. `tracker_binding_json` 이 자동으로 `config/tracker_binding_pico_connect.json` 으로 스왑됨
> 2. 시작 시 권장 SteamVR Add-On 레이아웃 (prism ON / VD OFF / udcap OFF) 출력
> 3. `prism` (PICO Connect) 가 트래커/컨트롤러의 단일 source 가 됨
>
> Virtual Desktop 으로 돌아가려면 `--vr_runtime virtual_desktop` (default
> tracker_binding.json 의 VD body segment 시리얼이 그대로 매칭되도록 calibrated).
> Add-On 선택을 사용자에게 맡기려면 `--vr_runtime auto` (기본값).

기대 콘솔:
```
[run_teleop] Windows process priority -> HIGH
[run_teleop] Loading env_cfg for variant='robot_only'...
[run_teleop] env_cfg = KitchenSortingGR1T2GripperRobotOnlyEnvCfg  →  Isaac-KitchenSorting-GR1T2-Gripper-RobotOnly-v0
...
[GR1T2GripperDevice] started — actions='./ust_ws/...' binding=N trackers ...
[GR1T2GripperDevice] --- OpenVR device inventory ---
  idx= 0 cls=HMD         serial='LHR-...'
  idx= 4 cls=Controller  serial='LHR-...'  controller_type='pico_neo3_controller' role=Left
  idx= 5 cls=Controller  serial='LHR-...'  controller_type='pico_neo3_controller' role=Right
[GR1T2GripperDevice] --- OpenVR action values probe ---
  left:  trigger=0.000 grip=0.000 menu=False
  right: trigger=0.000 grip=0.000 menu=False
  → all action values 0.0 / False.  Either user is at rest OR ...

[GR1T2Gripper #1 first-call] 16D action vector (L=controller, R=controller)
  L_wrist: pos=(-0.200,+0.000,+1.050) gripper=+1
  R_wrist: pos=(+0.200,+0.000,+1.050) gripper=+1
[GR1T2Gripper #20] L=controller R=controller | L_pos=(...,...,...) R_pos=(...,...,...) | L_grip=+1 R_grip=+1 | max_trig L=0.00 R=0.00
```

### 3.2 키친 소팅 (Vision + 데모 수집용)

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant data_collect `
    --teleop_device pico_gripper `
    --render_mode monitor `
    --process_priority high
```

### 3.3 VR 모드 (PICO HMD 헤드셋 안에서)

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant vr `
    --teleop_device pico_gripper `
    --render_mode steamvr_desktop `
    --render_interval 2 `
    --process_priority high
```

---

## 4. 주요 CLI 옵션

| flag | 기본값 | 설명 |
|---|---|---|
| `--env_variant` | `auto` | `base / waist_enabled / monitor / vision / vr / data_collect / robot_only` |
| `--render_mode` | `monitor` | `monitor / steamvr_desktop / steamvr_native / cloudxr` |
| `--prefer_controller` | `true` | 손목 EEF 를 컨트롤러 pose 에서 가져옴 (false 면 forearm 트래커 우선) |
| `--ignore_arms` | `false` | 양 팔을 idle T-pose 로 강제 (그리퍼만 디버그) |
| `--forearm_offset` | (env_cfg) | 팔꿈치→손목 추정 거리 (m). 기본 0.28 m |
| `--gripper_close_threshold` | `0.6` | hysteresis 닫힘 임계값 |
| `--gripper_open_threshold` | `0.4` | hysteresis 열림 임계값 (반드시 < close) |
| `--use_grip_as_close` | `true` | grip 버튼도 닫힘 신호로 사용 |
| `--render_interval` | `1` | sim.render_interval 오버라이드 (VR 시 2 권장) |
| `--process_priority` | `high` | Windows 프로세스 우선순위 (`normal/high/realtime`) |
| `--diag` | `off` | `idle/oscillate` — 텔레오퍼레이션 디바이스 우회, 진단용 |
| `--steps` | `0` | > 0 일 때 그 step 수만 실행 후 종료 (smoke) |

---

## 5. 트러블슈팅

### 5.1 `[GR1T2GripperDevice] *** UDCAP STILL RUNNING ***` 경고

증상: console 에 위 메시지 출력 → controller_type 가 `knuckles` 로 보고됨.

해결: §1.2 단계 1~4 를 다시 수행. 특히 `<SteamVR>/drivers/udcap` 폴더 rename + SteamVR 재시작.

### 5.2 트리거 당겨도 그리퍼 안 닫힘

진단:
```
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper
```

예상 출력 분석:
- `L_trig=0.00` 만 나옴 → SteamVR 바인딩 미적용 (§1.4 다시)
- `L_trig` 가 정상이고 `L_grip_cmd=+1` 만 나옴 → `--gripper_close_threshold` 너무 높음 (0.4 로 낮춰보기)

### 5.3 손목 위치가 어색함 (팔꿈치 트래커 모드)

증상: 로봇 손목이 어깨 옆에 붙거나 너무 멀리 떨어짐.

원인: 사용자 팔뚝 길이가 default 0.28 m 와 차이남.

해결:
```
--forearm_offset 0.32   # 더 긴 팔뚝
--forearm_offset 0.24   # 짧은 팔뚝
```

또는 `--prefer_controller true` (default) 로 컨트롤러 pose 직접 사용 → forearm offset 무관.

### 5.4 트리거 당기면 VR 화면이 드래그됨

원인 (우선순위):
1. SteamVR 바인딩 미적용 (§1.4 — Save Personal Binding 누락)
2. (VD 경로) Virtual Desktop "Disable VR pointer in apps" OFF (§1.3 Option B)
3. (prism 경로) PICO Connect Compatibility Mode 또는 Controller Priority 설정 누락 (§1.3 Option A)

검증:
```
SteamVR > Settings > Controllers > Manage Controller Bindings
  → "UST Teleop GR1T2 Gripper" 가 보이는가? 보이지 않으면 manifest 미등록
```

### 5.5 Pink IK 가 수렴 안 함

증상: 콘솔에 "OSQP problem seems to be non-convex" 또는 "Workspace allocation error".

원인: action[0:14] 의 wrist pos/quat 가 invalid (e.g. quaternion norm 0).

확인: `[GR1T2Gripper #N]` 로그에서 `L=default R=default` 가 보이면 idle 폴백 — 정상. 만약 컨트롤러 / 트래커 모두 잃으면 idle 로 떨어지므로 invalid action 은 발생하지 않음.

### 5.6 그리퍼 USD 빌드 실패

증상:
```
[build_gripper_usd] FATAL — pxr / Isaac Sim USD libraries not available.
```

원인: 일반 Python 으로 실행됨. Isaac Sim Python 환경 필요.

해결:
```powershell
./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_gripper_usd.py
```

### 5.7 트리거/그립이 0.000 으로만 인식됨 — Personal Binding 미적용 (9.39)

증상: enumerate_trackers / diagnose_pico_connect / SteamVR Test Controller
모두 정상 (PICO 헤드셋 녹색, L/R 컨트롤러 100%, controller_type=pico_controller,
identifyApplication OK), 그런데 `diagnose_gripper` / `diagnose_controller_raw`
가 트리거/그립 모두 `0.00` 만 반환하면서 `bActive=False` (`(a0)` flag) 표기.

원인: SteamVR 의 **Per-Application Personal Binding** 이 우리 앱
(`ust.teleop.gr1t2_gripper`) 에 적용되지 않은 상태. SteamVR 의 *"Test Controller"*
패널과 *PICO Connect* 의 컨트롤러 테스트 화면은 컨트롤러 드라이버의 default
binding 을 사용하기 때문에 항상 동작하지만, 우리 앱처럼 별도 manifest
(`ust.teleop.gr1t2_gripper`) 를 등록한 경우는 SteamVR 가 **명시적으로**
"Manage Controller Bindings" 에서 Active Binding 을 지정/저장해야 합니다.
9.32 binding rewrite 후 stale 한 빈 Personal Binding 이 남아있어도 같은 증상.

해결 — **가장 확실한 PRIMARY FIX (9.40)**: stale Personal Binding 파일을
디스크에서 직접 삭제 후 SteamVR 재시작:

```powershell
$env:PYTHONPATH = "."
# 1) 현재 어떤 Personal Binding 파일이 있는지 확인 (--list 가 default)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding

# 2) 모두 삭제 (자동으로 *.bak 백업)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding --clear

# 3) SteamVR 완전 종료 후 재시작
#    - 시스템 트레이의 SteamVR 아이콘 우클릭 → "Quit SteamVR"
#    - 5초 대기 (vrserver.exe / vrcompositor.exe 종료)
#    - Steam → Library → Tools → SteamVR → Launch
#    - 헤드셋 아이콘이 녹색이 될 때까지 대기

# 4) 검증 — 이제 (a1) 플래그가 표시되어야 함
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw
```

`repair_binding --clear` 가 `%LOCALAPPDATA%\openvr\input\binding_ust.teleop.gr1t2_gripper_*.json` +
`<Steam>\config\steamvr_input\binding_ust.teleop.gr1t2_gripper_*.json` 를
모두 `*.bak` 으로 옮깁니다. SteamVR 재시작 후 우리 앱 manifest 의
`default_bindings` (PICO Connect = `pico_controller`) 가 자동 적용되어
`bindings_pico.json` 의 trigger/grip Pull 매핑이 유효해집니다.

**If SteamVR 재시작이 어려운 경우** — `--write-default` 로 fresh Personal
Binding 을 강제 디스크 작성:

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.repair_binding --clear --write-default
# 그래도 SteamVR 재시작이 가장 확실한 방법
```

#### Secondary fix — UI 경유 (PRIMARY 가 안 될 때)

```powershell
$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.open_binding_ui
```

`IVRInput::OpenBindingUI` 를 직접 호출해서 SteamVR 바인딩 에디터를 우리 앱에
포커스해 띄웁니다 (사용자가 30+ 앱 중에서 우리 앱을 찾을 필요 없음). 다이얼로그에서:

1. **Active Controller Binding** 섹션 → `UST Teleop GR1T2 Gripper Default` 선택
2. (만약 stale 한 Personal Binding 이 보이면) `Reset to Default` 먼저 클릭
3. **`Save Personal Binding`** 클릭 (다이얼로그 *맨 아래* — 가장 자주 누락되는 단계)
4. 다이얼로그 닫기

검증:

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw
```

각 채널이 `(a1)` 플래그로 표시되어야 하고, 컨트롤러 그립을 누르면
`A_grip=0.85(a1)` 같은 nonzero 값이 출력되어야 합니다.

#### 추가 — forearm 역할 매핑 (PICO Connect 4-Ultra full-body 트래커)

`enumerate_trackers --out` 으로 자동 채워진 `tracker_binding_pico_connect.json`
은 **모든 트래커의 role 이 `TODO_pico`** 입니다. 그리퍼 트랙은 손목 EEF
fallback 으로 forearm 트래커 2개를 소비하므로, JSON 을 직접 편집해서:

```json
"LeftWrist":  { "role": "left_forearm",  "steamvr_role": "TrackerRole_LeftElbow" },
"RightWrist": { "role": "right_forearm", "steamvr_role": "TrackerRole_RightElbow" },
"Waist":      { "role": "waist",         "steamvr_role": "TrackerRole_Waist" },
"LeftFoot":   { "role": "",              "steamvr_role": "TrackerRole_LeftFoot" },
"RightFoot":  { "role": "",              "steamvr_role": "TrackerRole_RightFoot" }
```

PICO 4 Ultra 가 트래커를 손목에 부착했을 때 시리얼이 `LeftWrist` / `RightWrist`
인 점을 활용해 `left_forearm` / `right_forearm` 으로 매핑합니다. PICO Connect
가 *"Forearm Tracking Enhanced"* 모드일 때 이 트래커가 실제로 팔꿈치 위치에서
forearm 길이를 추정하므로, retargeter 의 `forearm_wrist_offset=0.28 m` 가
손목 EEF fallback 으로 정확하게 작동합니다 (CLI `--forearm_offset` 으로 미세조정).

### 5.8 `enumerate_trackers` / `diagnose_pico_connect` 가 무반응 (9.38)

증상: `python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers --out ...`
명령어를 실행해도 화면에 아무것도 안 나오고 프롬프트가 멈춘 듯한 상태로 머무름.

원인: `openvr.init(VRApplication_Other)` 가 SteamVR 미실행 상태에서 OpenVR
런타임이 SteamVR 자동 부팅을 silently 트리거하면서 30초~수분 (또는 영구) 동안
블로킹.  9.37 까지의 스크립트는 init 호출 직전에 진행 메시지가 한 줄도 없어서
사용자 입장에서 "hang" 으로 인식됨.

해결 (9.38 패치 이후):

* 스크립트 시작 즉시 `[enumerate_trackers] starting...` 출력 → 무반응 아님 확인.
* SteamVR 미실행 시 `vrserver.exe` 사전 체크에서 fast-fail 하고 액션 가이드 출력.
* `openvr.init` 가 `--init-timeout` 초 안에 끝나지 않으면 watchdog 가 강제 abort
  하면서 "두 HMD-redirecting 드라이버 충돌 (gotcha #29)" 등 우선순위 원인 안내.

복구 절차:

```powershell
# 1) Steam 실행 → Library → Tools → SteamVR → Launch
# 2) SteamVR 윈도우의 헤드셋 아이콘이 녹색/파란색이 될 때까지 대기 (헤드셋 페어링 완료)
# 3) PICO Connect Streaming Service 실행 + 헤드셋 페어링
# 4) SteamVR > Manage Add-Ons:
#       prism                            ON   (PICO Connect)
#       Virtual Desktop Streamer (Quest) OFF
#       udcap                            OFF
# 5) 다시 enumerate_trackers 실행
$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers `
    --out ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json
```

여전히 init 가 느리다면 `--init-timeout 120` 으로 늘려서 SteamVR cold start 대기:

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers `
    --out ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json `
    --init-timeout 120
```

vrserver.exe 사전 체크가 잘못 misreport 한다면 (드물지만):

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers `
    --skip-steamvr-check --init-timeout 90
```

### 5.8 stock GR1T2 USD 폴백 경고

증상:
```
[ust_hm_grip] WARNING — gripper USD not found at .../GR1T2_with_gripper.usd
  Falling back to stock GR1T2 USD ...
```

원인: §1.1 USD 빌드 미수행. 일반적으로 monitor 모드 디버깅 시는 폴백으로도 동작하나 (그리퍼는 작동 안 함, 손은 5-finger 휴머노이드), 실제 텔레오퍼레이션은 빌드 필수.

---

## 6. 폴더 / 파일 구조

```
ust_ws/ust_hm_grip/
├── __init__.py                                 # gym registration
├── EXECUTION_GUIDE.md                          # 이 파일
├── kitchen_sorting_gr1t2_gripper_env_cfg.py    # 16D action env cfg (7 variants)
├── teleop/
│   ├── __init__.py
│   ├── gr1t2_gripper_retargeter.py             # 16D retargeter + hysteresis
│   ├── gr1t2_gripper_device.py                 # PICO controller device
│   └── _osqp_compat.py                         # qpsolvers 4.x ↔ osqp 0.6 shim
├── config/
│   ├── tracker_binding.json                    # 팔꿈치 트래커 매핑
│   └── openvr_actions/
│       ├── actions.json                        # PICO trigger/grip/menu only
│       ├── bindings_pico.json                  # PICO Touch / oculus_touch / knuckles
│       └── manifest.vrmanifest                 # app_key=ust.teleop.gr1t2_gripper
├── isaac_file/
│   └── build_gripper_usd.py                    # 그리퍼 부착 USD 빌드 (Isaac Sim)
├── scripts/
│   ├── run_teleop.py                           # 메인 실행 진입점
│   ├── smoke_test.py                           # standalone sanity check (7 tests)
│   └── diagnose_gripper.py                     # SteamVR + 트리거 진단
└── tests/
    ├── __init__.py
    ├── test_gripper_retargeter.py              # 16+ pytest tests
    └── test_action_manifest.py                 # JSON config 정합성 12+ tests
```

---

## 7. 모듈 출처 / ust_hm_glove 와의 관계

ust_hm_grip 은 9.36 분리 시점에 ust_260504_win 에서 분리된 컨트롤러-그립 전용
서브프로젝트입니다.  과거 ust_260418_win 에서 import 하던 하드웨어-제너릭
모듈은 9.36 에서 이 디렉터리 안으로 **복사 (자기-완결)** 했습니다.

| 모듈 (현재 경로) | 9.36 이전 출처 | 사용처 |
|---|---|---|
| `ust_hm_grip/teleop/vr_sampler.py` (`SteamVRSampler`) | `ust_260418_win/teleop/vr_sampler.py` | `gr1t2_gripper_device._sampler` |
| `ust_hm_grip/teleop/coord_transforms.py` (`svr_to_isaaclab`, `forearm_to_wrist`) | `ust_260418_win/teleop/coord_transforms.py` | `gr1t2_gripper_retargeter` 내부 |
| `ust_hm_grip/teleop/_osqp_compat.py` | (복제) | `kitchen_sorting_gr1t2_gripper_env_cfg.py` |
| `KitchenSortingSceneCfg` | `ust_260220/kitchen_sorting_env_cfg` | 키친 소팅 씬 (그리퍼 robot 으로 swap) |

**ust_hm_glove (UDCAP 글러브 트랙) 와 분리된 의존성**:
- `VMCHandReceiver` (UDCAP VMC OSC 39539 — glove 전용)
- `udcap_finger_mapper` / `fourier_hand_mapper` (22D finger — glove 전용)
- `fingertip_extractor` (skeletal 26-bone — glove 전용)
- `head_estimator`, `waist_estimator` (이번 마이그레이션에서는 단순화 위해 제외)
- DexPilot YAML / `dex_retargeting` (그리퍼는 IK target 1개만)

상호 무관: ust_hm_grip 와 ust_hm_glove 는 SteamVR app_key (`ust.teleop.gr1t2_gripper`
vs `ust.teleop.fourier_gr1t2`) 가 다르므로 SteamVR 는 두 앱을 별개로 관리합니다.

기존 ust_fourier_260421 텔레오퍼레이션 워크플로우를 동시 운영하려면 두 패키지가 별도 `app_key` 와 `vrmanifest` 를 쓰므로 SteamVR 측에서는 두 앱이 각각 별개로 등록됩니다 — Manage Controller Bindings 에서 사용자가 하나를 선택하면 됩니다.

---

## 8. 다음 단계 (마이그레이션 후속)

| 작업 | 우선순위 | 분량 | 비고 |
|---|---|---|---|
| 16D action HDF5 demo 수집 | 1 | 1주 | 기존 22D demo 와 호환 안 됨 — 새로 수집 |
| BC-RNN 학습 cfg 16D 로 갱신 | 1 | 0.5일 | `policy.gmm.modalities.actions.shape = [16]` |
| MimicGen augmentation | 2 | 1주 | 그리퍼 binary 는 augmentation friendly |
| 컨트롤러 햅틱 피드백 | 3 | 1-2일 | `IVRSystem::triggerHapticPulse()` + ContactSensor |
| Visual gripper progress overlay | 3 | 1일 | `VisualizationMarkers` 활용 |
| Pink IK 팔꿈치 posture target 추가 | 3 | 0.5일 | 팔꿈치 트래커를 second `FrameTask` 로 |

---

## 9. 검증 결과 (이 빌드)

### 9.1 코드 통계

| 파일 | 라인 수 (대략) |
|---|---|
| `gr1t2_gripper_retargeter.py` | ~360 |
| `gr1t2_gripper_device.py` | ~440 |
| `kitchen_sorting_gr1t2_gripper_env_cfg.py` | ~470 |
| `build_gripper_usd.py` | ~190 |
| `run_teleop.py` | ~290 |
| `smoke_test.py` | ~180 |
| `diagnose_gripper.py` | ~95 |
| `test_gripper_retargeter.py` | ~205 |
| `test_action_manifest.py` | ~135 |
| `__init__.py` | ~95 |
| **총 production + test** | **~2470** |

### 9.2 검증 명령

```bash
# Step 1: smoke (Isaac Sim 불필요)
PYTHONPATH=. python -X utf8 ust_ws/ust_hm_grip/scripts/smoke_test.py

# Step 2: pytest regression
PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_grip/tests/ -v

# Step 3: SteamVR + 컨트롤러 진단 (실 하드웨어 필요)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper

# Step 4: USD 빌드 (Isaac Sim Python)
./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_gripper_usd.py

# Step 5: 라이브 텔레오퍼레이션 (monitor)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop --env_variant robot_only --teleop_device pico_gripper --render_mode monitor
```

---

마지막 업데이트: 2026-05-04 (option B 정식 그리퍼 마이그레이션 — research/36 §6.2 우선순위 2 적용)
