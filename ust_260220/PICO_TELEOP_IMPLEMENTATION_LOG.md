# PICO 텔레오퍼레이션 구현 히스토리 로그

> 작성일: 2026-04-08
> 프로젝트: ust_260220 (G1 Kitchen Sorting + PICO 전신 텔레오퍼레이션)

---

## 1. 초기 문제: `[XRT] Connection lost, reconnecting in 2s...`

### 증상
- `unified_bridge.py`가 `localhost:7777`에 TCP 연결을 반복 시도하지만 실패
- RoboticsServiceProcess 실행 중, PICO XRoboToolkit 앱에서 연결(Working) 상태 확인됨

### 근본 원인
**ConsoleDemo(`main.cpp`)에 TCP 서버 코드가 없음.** ConsoleDemo는 XRoboToolkit SDK 콜백을 받아 `stdout`에만 출력:
```cpp
case PXREADeviceStateJson:
    std::cout <<"device data"<< dsj.stateJson << std::endl;  // stdout만!
```
`unified_bridge.py`는 `localhost:7777` TCP 서버에 연결을 시도했지만, 그런 서버는 존재하지 않았음.

### 해결
`unified_bridge.py`에 **subprocess 모드** 추가:
- ConsoleDemo를 자식 프로세스로 직접 실행
- stdout 파이프에서 `device data{json}` 라인을 읽고 파싱
- TCP 모드는 `--xrt_mode tcp`로 레거시 지원 유지

**수정 파일**: `teleop/unified_bridge.py`

---

## 2. ConsoleDemo 크래시: `libPXREARobotSDK.so: cannot open shared object file`

### 증상
- ConsoleDemo가 시작 즉시 종료, 반복 재시작
- `[XRT] ConsoleDemo 종료됨, 3초 후 재시작...` 무한 반복

### 근본 원인
`libPXREARobotSDK.so`가 `Redistributable/linux/` 디렉토리에 없음. SDK 라이브러리가 다른 위치에만 존재:
```
SDKDemo/UnityBin/RobotLinuxDemo/RobotLinuxDemo_Data/Plugins/libPXREARobotSDK.so
bin/SDK/x64/libPXREARobotSDK.so
SDK/linux/64/libPXREARobotSDK.so
```

### 해결
```bash
cp ~/ust_ws/XRoboToolkit-PC-Service/RoboticsService/bin/SDK/x64/libPXREARobotSDK.so \
   ~/ust_ws/XRoboToolkit-PC-Service/RoboticsService/Redistributable/linux/
```

### 추가 수정
- `unified_bridge.py`에 stderr 캡처 + 종료 코드 출력 추가 (디버그 강화)
- `bufsize=1` 경고 제거 (Python 3.12 binary mode 호환)

---

## 3. 렌더링 모드 구현: 방안3 (모니터 뷰) + 방안1 (PICO Connect)

### 배경
PICO에서 XRoboToolkit(트래킹)과 CloudXR(VR 렌더링)을 동시에 실행할 수 없음 (PICO OS 5 제약).

### 방안3: 모니터 뷰 (즉시 사용 가능)
- XR 없이 PC 모니터에서 Isaac Lab 렌더링
- PICO XRoboToolkit으로 전신 트래킹만 사용
- `--render_mode monitor` 플래그 추가

### 방안1: PICO Connect (조사 결과 Ubuntu에서 불가)
- PICO Connect PC 소프트웨어: **Windows 전용**
- SteamVR on Linux: PICO 미지원
- N100 미니PC로 중계: GPU 부족 (GTX 970 이상 필요)
- **결론**: Ubuntu 환경에서 PICO Connect PCVR 경로는 불가

### 수정 파일
| 파일 | 변경 내용 |
|------|-----------|
| `scripts/run_teleop.py` | `--render_mode` 플래그 추가 (monitor/pico_connect/cloudxr) |
| `scripts/record_demos.py` | `--render_mode` 플래그 추가 |
| `kitchen_sorting_env_cfg.py` | `KitchenSortingG1MonitorEnvCfg` 추가 |
| `__init__.py` | `Isaac-KitchenSorting-G1-InspireFTP-Monitor-v0` Gym 등록 |
| `setup_pico_env.sh` | ConsoleDemo 별도 실행 제거, 안내문 업데이트 |
| `setup_pico_connect_env.sh` | 신규 (SteamVR + PICO Connect, Windows에서만 유효) |

---

## 4. 포트 변경: 8888 → 8889

사용자 요청으로 통합 브릿지 출력 포트를 8889로 변경.

**수정 파일** (전체 6개):
- `teleop/unified_bridge.py` - 기본 output_port
- `teleop/pico_fullbody_device.py` - 기본 bridge_port
- `kitchen_sorting_env_cfg.py` - pico_device_cfg (2곳)
- `setup_pico_env.sh` - 브릿지 실행 명령
- `setup_pico_connect_env.sh` - 브릿지 실행 명령

---

## 5. Windows Isaac Lab 가능성 조사

### 조사 결과
| 시나리오 | 가능? | 이유 |
|----------|-------|------|
| Isaac Sim Docker on Windows WSL2 | **불가** | Vulkan 미지원 (NVIDIA 공식 확인) |
| Isaac Sim 네이티브 Windows | **가능** | pip install isaacsim==5.1.0 |
| CloudXR Runtime Windows | **가능** | 네이티브 지원, Docker 불필요 |
| XRoboToolkit PC Service Windows | **가능** | 크로스 플랫폼 |

### 결론
Windows로 전환하면 Isaac Lab + CloudXR + XRoboToolkit 모두 네이티브 실행 가능하지만, 먼저 PICO OS 6 멀티태스킹으로 Ubuntu에서 해결 시도 권장.

**리서치 문서**: `ust_ws/research/25. windows_isaac_lab_pico_connect_architecture_analysis.md`

---

## 6. PICO OS 6 Early Access 조사

### 현황
- PICO OS 6: 2026-03-02 발표, **개발자 Early Access (비공개 베타)**
- 정식 출시: 2026년 하반기 (Project Swan 헤드셋과 함께)
- PICO 4 Ultra 지원 확인됨

### OS 5 vs OS 6 멀티태스킹
| 기능 | OS 5 (현재) | OS 6 (미출시) |
|------|-------------|---------------|
| 몰입형 XR 앱 + 2D 앱 동시 | X | O (Shared Space) |
| XR 앱 + XR 앱 동시 | X | O (Spatial Engine) |

### Early Access 신청
- 개발자 콘솔: `developer.picoxr.com` → Platform Access → OS 6 Early Access
- 조직(Organization) 계정 필요 (개인 계정에서는 메뉴 안 보일 수 있음)
- 심사 기간: 5~10 영업일
- 직접 이메일 가능: developer@picoxr.com

**이메일 템플릿**: `ust_ws/ust_260220/pico_os6_early_access_email.md`

---

## 7. OSQP "non-convex" 에러: Pink IK 액션 포맷 불일치

### 증상
Isaac Lab에서 매 스텝마다 반복:
```
ERROR in LDL_factor: Error in KKT matrix LDL factorization when computing the nonzero elements.
The problem seems to be non-convex
ERROR in osqp_setup: KKT matrix factorization.
```
로봇이 전혀 움직이지 않음.

### 근본 원인: 액션 포맷 완전 불일치

**Pink IK가 기대하는 38D**:
```
[left_pos(3), left_quat(4), right_pos(3), right_quat(4), hand_joints(24)]
 = 절대 EEF 타깃 좌표 (예: [-0.15, 0.20, 1.10, 0.707, 0, 0, 0.707, ...])
```

**G1PICORetargeter가 출력하는 38D**:
```
[right_pos_delta(3), right_euler_delta(3), right_grip(1),
 left_pos_delta(3), left_euler_delta(3), left_grip(1),
 right_inspire_12(12), left_inspire_12(12)]
 = 델타 값 + 오일러 각 + 그리퍼 (예: [0.001, 0.002, -0.001, ...])
```

5가지 불일치:
1. **순서**: env은 [LEFT, RIGHT], retargeter는 [RIGHT, LEFT]
2. **좌표 형식**: env은 절대좌표, retargeter는 델타
3. **회전 표현**: env은 쿼터니언(4D), retargeter는 오일러(3D) + 그리퍼(1D)
4. **핸드 DOF**: env은 24관절, retargeter는 2×12 Inspire DOF
5. **그리퍼**: env에는 없음 (핸드 관절로 제어), retargeter에는 있음

→ 작은 델타값(~0.001)이 절대 좌표로 해석 → 원점 근처 도달불가 타깃 → OSQP 실패

### 해결
`PICOFullBodyTeleopDevice.advance()`에 **델타→절대 좌표 변환 어댑터** 추가:

1. idle 포즈에서 시작 (기본 팔 위치):
   - Left: pos=(-0.1487, 0.2038, 1.0952), quat=(0.707, 0, 0, 0.707)
   - Right: pos=(0.1487, 0.2038, 1.0952), quat=(0.707, 0, 0, 0.707)
2. 매 프레임 위치 델타를 누적하여 절대 위치 업데이트
3. 오일러 델타 → 쿼터니언 곱셈으로 회전 업데이트
4. `_inspire12_to_24joints()`: Inspire 12 DOF → 24 hand joint 매핑
5. 최종 출력: `[L_pos(3), L_quat(4), R_pos(3), R_quat(4), hand(24)]`

**수정 파일**: `teleop/pico_fullbody_device.py`

---

## 8. PICO "start bodytracking 0" 문제

### 상태
- PICO XRoboToolkit 앱에서 status: "start bodytracking 0"
- Motion Tracker에서 Full Body + High-Acc 체크했음에도 0 반환

### 분석
- Body tracking 활성화 실패 (Motion Tracker 연결은 되었으나 트래킹 미시작)
- 현재 코드는 **컨트롤러 트래킹** 기반으로 동작하므로 body tracking 없이도 양팔 제어 가능
- Body tracking은 PICO 앱 쪽 문제 (XRoboToolkit 앱 재시작, Motion Tracker 재페어링 필요)

### 대응
- 현재: 컨트롤러 기반 양팔 제어로 진행 (body tracking 불필요)
- 향후: body tracking 활성화 후 전신 제어 확장

---

## 현재 수정된 전체 파일 목록

| 파일 | 수정 내용 |
|------|-----------|
| `teleop/unified_bridge.py` | subprocess 모드, stderr 캡처, 디버그 로그 |
| `teleop/pico_fullbody_device.py` | 델타→절대 어댑터, Inspire→24joint 매핑, 디버그 |
| `scripts/run_teleop.py` | `--render_mode` 플래그, monitor 환경 지원 |
| `scripts/record_demos.py` | `--render_mode` 플래그, monitor 환경 지원 |
| `kitchen_sorting_env_cfg.py` | MonitorEnvCfg 추가, bridge_port 8889 통일 |
| `__init__.py` | Monitor-v0 Gym 등록 |
| `setup_pico_env.sh` | ConsoleDemo subprocess화, 안내문 업데이트 |
| `setup_pico_connect_env.sh` | 신규 (Windows 전용 가이드) |
| `pico_os6_early_access_email.md` | 신규 (PICO OS 6 EA 신청 이메일 템플릿) |

---

## 실행 가이드 (현재 상태)

### 전체 아키텍처
```
PICO 4 Ultra (XRoboToolkit)
  └─ gRPC → RoboticsServiceProcess (호스트)
               └─ ConsoleDemo (subprocess, stdout)
                    └─ unified_bridge.py (TCP:8889)
                         └─ Isaac Lab Docker (PICOFullBodyTeleopDevice)
                              └─ Pink IK → G1 로봇 제어

Windows 미니PC (UDCAP 글러브)
  └─ VMC UDP:39539 → unified_bridge.py
```

### 실행 순서

**Step 1: 호스트에서 환경 시작**
```bash
cd ~/ust_ws/ust_260220
source setup_pico_env.sh
```

**Step 2: PICO에서 XRoboToolkit 연결**
- XRoboToolkit App → PC Service IP: 호스트 IP
- `[XRT] 첫 번째 트래킹 데이터 수신!` 확인

**Step 3: Isaac Lab 실행 (Docker)**
```bash
# 텔레오퍼레이션
./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py \
    --teleop_device pico --render_mode monitor

# 데모 녹화
./isaaclab.sh -p ust_ws/ust_260220/scripts/record_demos.py \
    --teleop_device pico --render_mode monitor \
    --use_usd_scene --num_demos 50
```

### 종료
```bash
pkill -f 'RoboticsServiceProcess|unified_bridge'
```

---

## 미해결 사항

1. **PICO body tracking "0"**: Motion Tracker 트래킹 미활성화 → PICO 앱 재시작/재페어링 필요
2. **PICO OS 6 Early Access**: 신청 후 승인 대기 (5~10 영업일)
3. **CloudXR.js + XRoboToolkit 동시 실행**: PICO OS 6에서만 가능, OS 5에서는 불가
4. **Pink IK 어댑터 실제 테스트**: 델타→절대 변환 후 실제 로봇 움직임 검증 필요
5. **Inspire 12→24 관절 매핑 검증**: 실제 손가락 움직임이 올바르게 매핑되는지 확인 필요

---

## 9. "start motion tracker 0" 에도 로봇이 움직이지 않는 이슈 (2026-04-10)

### 증상
- PICO 5.15.4 + XRoboToolkit v1.1.1 (최신) + Motion Tracker 5개 페어링 + Body+HighAccuracy 모드
- PICO Motion Tracker 앱에서 사전 캘리브레이션 완료
- 헤드셋 status: `"start motion tracker 0"` (= **PICO SDK 성공 코드**, 에러 아님)
- 그러나 Isaac Lab의 G1 로봇이 전혀 움직이지 않음

### 핵심 원인 (Isaac Lab 측, 코드 4건)

#### 9-1. **`g1_retargeter.py`가 Body 트래킹 데이터를 무시**
이전 구현은 `xrt_frame.right_controller.pose` / `left_controller.pose`만 읽음.
Body+HighAccuracy 모드에서는 실제 데이터가 `frame.body_joints` (24관절 = LEFT_WRIST=20, RIGHT_WRIST=21)에 들어옴. 컨트롤러를 들고 있지 않으면 모든 델타가 0 → 로봇이 idle 포즈 유지.

#### 9-2. **델타 누적 → 절대 좌표 변환의 정보 손실**
이전 흐름: `retargeter` → `delta` → `pico_fullbody_device.advance()` → `idle + Σdelta`
컨트롤러 절대 포즈가 있어도 매 프레임 차이만 계산해서 idle pose 위에 더하는 구조 → 노이즈/jitter가 그대로 누적, 캘리브레이션 손실, body 트래킹 절대 좌표를 활용할 수 없음.

#### 9-3. **ConsoleDemo subprocess stdout block buffering**
`subprocess.Popen([ConsoleDemo], stdout=PIPE)` → stdout이 PIPE이면 C++ `std::cout`은 자동으로 **block-buffered (~4KB)**. 트래킹 JSON 데이터가 버퍼에 쌓여 큰 지연. `main loop alive` 한 줄로는 1KB도 안 차서 1초 이상 출력 지연 가능.
**FIX**: `stdbuf -oL -eL` 래퍼로 강제 line-buffered 변환 (`unified_bridge.py`).

#### 9-4. **XRT JSON 키 가시성 부재**
`xrt_raw.keys()` 는 outer keys (`functionName`, `value`)만 출력 → 내부의 `Head/Controller/Hand/Body/Motion` 중 어떤 트래킹이 활성인지 알 수 없음.
`value`는 문자열로 인코딩된 JSON이라 한 단계 더 풀어야 함. 진단 어려움 → 디버깅 불가능.

### 적용된 수정

#### A. `teleop/g1_retargeter.py` 전면 재작성
- **출력 포맷 변경**: 38D 델타 → 38D **절대 Pink IK** `[L_pos(3), L_quat(4), R_pos(3), R_quat(4), hand(24)]`
- **데이터 소스 우선순위** (좌/우 독립):
  1. `body_joints[20/21]` (LEFT/RIGHT_WRIST) — Body+HighAcc 활성 시
  2. `controller.pose` — 컨트롤러 들고 있을 때
  3. 디폴트 idle 포즈
- 좌표계 변환: XRT (Z-in) → IsaacLab (Z-out), `pos.z *= -1`, `quat.qz *= -1`
- 사용자 키 보정: `position_scale`, 평행이동: `body_pos_offset`, `controller_pos_offset`
- `_pico_body_to_g1_eef()`, `_pico_controller_to_g1_eef()` 분리
- `_inspire12_to_24joints()` 핸드 매핑은 retargeter 내부로 이동
- `get_source_info()` 추가 → 현재 활성 데이터 소스 추적
- 60프레임마다 디버그 로그 (소스 + 좌표 출력)

#### B. `teleop/pico_fullbody_device.py` 단순화
- **델타 누적 코드 제거** (`_current_left_pos += delta` 등 일체 삭제)
- `_DEFAULT_LEFT_POS`, `_quat_multiply`, `_euler_to_quat_delta`, `_inspire12_to_24joints` 전부 제거 (retargeter로 이동)
- `advance()` → 그냥 `self._retargeter.retarget()` 호출 후 38D 텐서 그대로 반환
- `_extract_inner_keys()` 추가 → XRT JSON `value` 안의 `Head/Controller/Hand/Body/Motion` 키 추출
- `_recv_loop()` 디버그 로그 강화: outer keys, **inner keys**, body/tracker 개수, 누적 키 통계, 첫 액션 시 데이터 소스 출력
- 새 인자: `body_pos_offset`, `controller_pos_offset`, `debug`

#### C. `teleop/unified_bridge.py` 버퍼링 수정
- `stdbuf -oL -eL ConsoleDemo` (line-buffered) 로 실행 — `shutil.which("stdbuf")` 로 fallback 처리
- `_extract_inner_keys()` 추가 → ConsoleDemo 출력에서 트래킹 키 검사
- 첫 데이터 수신 시: inner keys + raw JSON byte size (16352 SDK 버퍼와 비교)
- `_recv_count == 0` 일 때는 모든 stdout 라인 echo (디버그용)

#### D. `kitchen_sorting_env_cfg.py` & 스크립트 인자 전달
- `pico_device_cfg`에 `body_pos_offset`, `controller_pos_offset`, `debug` 필드 추가 (Monitor/USD/G1 EnvCfg 모두)
- `run_teleop.py`, `record_demos.py` → `cfg.get(...)`로 새 인자 전달

### 진단 절차 (수정 후)

수정된 코드 실행 시 stdout에 다음 로그가 단계별로 나타남:

```
[XRT raw] server connect
[XRT raw] device find...
[XRT raw] device connect...
[XRT] ✓ 첫 번째 트래킹 데이터 수신!
[XRT]   inner keys: ['Head', 'Controller', 'Hand', 'Body']  ← 여기서 'Body' 있는지 확인!
[XRT]   raw size: 6234 bytes (SDK buffer 16352)             ← 16352 근접하면 truncation 위험

[Bridge] Isaac Lab 클라이언트 연결: ('127.0.0.1', ...)

[PICODevice] ✓ 첫 데이터 수신!
  outer keys: ['functionName', 'value']
  inner keys: ['Head', 'Controller', 'Hand', 'Body']        ← 'Body' 있어야 함
  Body joints: 24 (24 expected for Body+HighAcc)            ← 24 이어야 함
  Motion trackers: 0
  UDCAP bones: 30

[PICODevice] ✓ 첫 Pink IK 액션:
  소스: L=body R=body                                       ← body 면 OK!
  L-EEF: pos=[..., ..., ...]
  R-EEF: pos=[..., ..., ...]

[Retarget #60] L=body R=body | body_joints=24 trackers=0 | L_pos=(...) R_pos=(...)
```

### 진단 분기

| 증상 | 원인 | 해결 |
|------|------|------|
| `[XRT raw]` 로그 자체가 안 나옴 | ConsoleDemo 미실행 / SDK lib 누락 | `libPXREARobotSDK.so` 위치 확인, RoboticsServiceProcess 실행 확인 |
| `inner keys` 에 `Body` 가 없음 | PICO 앱에서 Body 모드 활성 안 됨 | XRoboToolkit 헤드셋 앱 → Body 드롭다운 + HighAccuracy ON |
| `Body joints: 0` | JSON에 Body 키는 있는데 joints 배열 비어있음 | PICO Motion Tracker 앱 캘리브레이션 다시 실행 |
| `소스: L=default R=default` | body/controller 모두 무효 | 헤드셋 상에서 직접 손/팔 움직여보고 `R-ctrl pose` 좌표 확인 |
| `소스: L=body R=body` 인데 로봇 안 움직임 | Pink IK 좌표 범위 밖 | `body_pos_offset` 조정, `position_scale` 사용자 키 비율로 조정 (1.7m / 1.32m ≈ 1.3) |

### 키 튜닝 포인트

`pico_device_cfg` 에서 사용자 환경에 맞춰 조정:
- `position_scale`: 사용자 키가 G1(1.32m)과 다르면 비율로 보정 (1.7m 사용자 → 약 0.78)
- `body_pos_offset`: PICO Body 좌표 원점은 캘리브레이션 시 사용자 pelvis. G1 pelvis는 (0,0,0). 보통 (0,0,0)에서 시작
- `controller_pos_offset`: 컨트롤러 모드 사용 시만 의미

### 수정 파일 요약

| 파일 | 변경 |
|------|------|
| `teleop/g1_retargeter.py` | 전면 재작성: Body 트래킹 통합, 절대 좌표 출력, Inspire 12→24 매핑 통합 |
| `teleop/pico_fullbody_device.py` | 델타 누적 제거, inner keys 추출, retargeter 직결, 새 인자 추가 |
| `teleop/unified_bridge.py` | stdbuf line-buffering, inner keys 추출, raw 디버그 로그 |
| `kitchen_sorting_env_cfg.py` | `pico_device_cfg`에 body/controller offset, debug 추가 (2곳) |
| `scripts/run_teleop.py` | 새 인자 전달 |
| `scripts/record_demos.py` | 새 인자 전달 |

### 핵심 교훈
- **PICO SDK 성공 코드 = 0** (실패가 아님). 헤드셋 status `"start motion tracker 0"` / `"Start BodyTracking 0"` 모두 정상 시작.
- **Body+HighAcc는 `Body` 키, Motion mode는 `Motion` 키** — 둘은 완전히 다른 데이터 소스. 5개 트래커 풀바디는 Body 모드 사용.
- **subprocess + std::cout = block-buffered**. 항상 `stdbuf -oL` 또는 PTY 래핑.
- **JSON inner key 가시성** 없이는 데이터 흐름 디버깅 불가능. 항상 `value` 안쪽까지 풀어서 로깅.
- **델타 누적 vs 절대 좌표**: 트래킹 시스템이 절대 좌표를 주면 그대로 사용 (정보 손실 X, 노이즈 누적 X).
