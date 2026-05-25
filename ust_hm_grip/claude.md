# ust_hm_grip — Claude context guide

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

> 이 파일은 본 폴더에서 작업을 이어갈 미래 Claude 세션을 위한 운영 가이드입니다.
> 새로운 세션은 `/init` 또는 작업 시작 전 이 파일을 먼저 읽어 컨텍스트를 회복하세요.
> 작업 이력 / 디버깅 기록은 동일 폴더의 [memory.md](memory.md) 를 참고하세요.

---

## 0. 프로젝트 한 줄 요약

PICO 4 Ultra 헤드셋 + 컨트롤러 2개로 **Fourier GR1T2 휴머노이드 + Robotiq 2F-85 그리퍼**를 Isaac Lab 2.3 / Isaac Sim 5.1 에서 텔레오퍼레이션. 좌/우 컨트롤러의 trigger·grip 으로 그리퍼 binary open/close, controller pose 로 wrist EEF target (Pink IK) 을 구동하는 16-D action 환경.

상위 컨텍스트는 [`../CLAUDE.md`](../CLAUDE.md) 및 [`../memory.md`](../memory.md) — 본 폴더는 그 sub-project.

---

## 1. 폴더 지도 (필수 파일)

```
ust_hm_grip/
├── claude.md                            ← 이 파일
├── memory.md                            ← 작업 이력 / 디버깅 기록
├── EXECUTION_GUIDE.md                   ← 사용자용 실행 가이드 (기존)
├── kitchen_sorting_gr1t2_gripper_env_cfg.py
│       Isaac Lab env_cfg.  옵션 A (Robotiq 2F-85) 활성.
│       관건 상수:
│         GR1T2_PINK_CONTROLLED_ARM_JOINTS  — 14 arm joints
│         GR1T2_WAIST_JOINT_NAMES           — 3 waist joints (Waist 변형에서 추가)
│         ROBOTIQ_LEAD_JOINT_LEFT/RIGHT     — 그리퍼 lead joint 이름 (관찰용)
│         _ROBOTIQ_LEFT_JOINTS_WITH_GEAR    — 6 joint × gearing tuple list
│         _ROBOTIQ_RIGHT_JOINTS_WITH_GEAR
│         ROBOTIQ_ALL_JOINTS_LEFT/RIGHT     — 6 joint name list (actuator + binary action)
│         ROBOTIQ_CLOSE_RAD = 0.785         — binary close target (lead 기준)
│         GR1T2_EEF_LINK_NAMES              — TCP 링크 (관찰값 + Pink IK target)
│
├── isaac_file/
│   ├── build_robotiq_usd.py             ← 옵션 A USD 빌드 (Robotiq 2F-85 attach)
│   │     주요 helper:
│   │       _remove_fourier_hand           — L_*/R_* finger prim 제거
│   │       _strip_hand_pitch_link_geometry — wrist 의 visuals/collisions 제거 (8th session)
│   │       _wrist_to_gripper_rotation     — per-side / per-strategy rotation (현재 Ry(180°))
│   │       _attach_robotiq                — 메인 entry, 7+ 단계
│   ├── build_gripper_usd.py             ← 옵션 B (deprecated, 박스 자작 그리퍼)
│   ├── GR1T2_with_robotiq.usd           ← 빌드 산출물 (env_cfg 가 참조)
│   ├── GR1T2_with_gripper.usd           ← 옵션 B 산출물 (deprecated)
│   └── robotiq/                         ← Robotiq stock USD 캐시
│       ├── Robotiq_2F_85_edit.usd       ← default prim /Robotiq_2F_85
│       ├── configuration/Robotiq_2F_85_robot.usd
│       ├── payloads/{Robotiq_2F_85_base.usda, Robotiq_2F_85_phyisics_mimic.usda}
│       └── parts/×6 mesh USDs           ← NVIDIA S3 에서 다운로드
│
├── teleop/
│   ├── _osqp_compat.py                  ← osqp 0.6 ↔ qpsolvers 4.x shim (env_cfg 가 즉시 적용)
│   ├── _pink_hand_dim_zero_patch.py     ← Pink IK 의 actions[:, -0:] 버그 monkey-patch
│   ├── coord_transforms.py              ← SteamVR ↔ Isaac Lab 좌표 변환
│   ├── gr1t2_gripper_device.py          ← GR1T2GripperDevice (PICO 인터페이스)
│   ├── gr1t2_gripper_retargeter.py      ← PICO pose → 16-D action retargeter
│   └── vr_sampler.py                    ← SteamVR action / pose 샘플러 + IPC 진단
│
├── scripts/
│   ├── run_teleop.py                    ← 메인 엔트리 (UI yield + diagnostic instrumentation 포함)
│   │
│   ├── # USD 검증 / 진단
│   ├── inspect_usd.py                   ← 옵션 B USD 검증
│   ├── inspect_robotiq_usd.py           ← 옵션 A USD 검증 (prim tree + mimic + ArticulationRoot)
│   ├── inspect_stock_meshes.py          ← stock Robotiq USD 의 prototype 구조 + instancing 확인 (4th session)
│   ├── inspect_visuals.py               ← built/stock USD 의 visuals subtree dump (4th session)
│   ├── inspect_wrist_frame.py           ← GR1T2 wrist 의 world 방향 + Fourier 손가락 delta (3rd session)
│   ├── inspect_hand_pitch_link.py       ← hand_pitch_link 의 APIs/children/참조 joint dump (8th session)
│   ├── diagnose_robotiq_attach.py       ← body/joint rel 유효성 + drive 값 + limits + mimic gearing (2nd session)
│   │
│   ├── # 빌드 후 검증
│   ├── verify_all_visuals.py            ← 9 body × 2 side 의 visuals children 가 Robotiq 메시인지 확인 (4th session)
│   ├── verify_gripper_world_pos.py      ← fingertip direction + TCP world position (3rd session)
│   ├── verify_wrist_joints.py           ← strip 후 wrist_pitch + attach 의 body refs 보존 검사 (8th session)
│   ├── test_robotiq_pose.py             ← idle 액션 60 step 후 12 robotiq joint drift 확인 (2nd session)
│   │
│   ├── # PICO / SteamVR
│   ├── cleanup_vr_env.py                ← SteamVR / Oculus runtime 정리 (관리자 PowerShell)
│   ├── diagnose_pico_connect.py         ← PICO Connect → SteamVR 파이프라인 진단
│   ├── diagnose_controller_*.py / diagnose_gripper.py / smoke_test.py
│   ├── open_binding_ui.py               ← SteamVR Controller Binding UI 열기
│   ├── repair_binding.py / force_reregister.py / restore_pico_driver.py
│   └── enumerate_trackers.py            ← tracker serial 기반 binding 자동 생성
│
├── config/
│   ├── tracker_binding.json             ← default
│   ├── tracker_binding_pico_connect.json← PICO Connect 파이프라인용 (PMT_*)
│   └── openvr_actions/{actions.json, manifest.vrmanifest, default_*.json}
│
└── tests/                                ← unit / smoke tests
```

---

## 2. 캐논 명령어

### 2.1 USD 빌드 (옵션 A — Robotiq 2F-85)

```powershell
./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_robotiq_usd.py
```

산출물: `ust_hm_grip/isaac_file/GR1T2_with_robotiq.usd`.

**URDF 캐시 무효화 — 매우 중요**: env_cfg 의 `__post_init__` 이 `mtime(URDF) >= mtime(USD)` 인 경우 cache hit. 빌드 직후 즉시 실행하면 URDF 가 *직전 빌드의 USD* 로 만들어진 상태가 그대로 hit 됨 (시각 변경이 안 보이는 원인 #1). **권장: 빌드 전에 강제 삭제**:
```powershell
Remove-Item -Force "$env:LOCALAPPDATA\Temp\urdf\GR1T2_with_robotiq.urdf" -ErrorAction SilentlyContinue
```

### 2.2 USD 빌드 후 검증 (한 줄씩)

```powershell
# 1. 모든 9 body × 2 side 의 visuals 가 Robotiq 메시인지 (instancing leak 검출)
./isaaclab.bat -p -m ust_ws.ust_hm_grip.scripts.verify_all_visuals --headless

# 2. fingertip 방향 + TCP world position 확인 (rotation 검증)
./isaaclab.bat -p -m ust_ws.ust_hm_grip.scripts.verify_gripper_world_pos --headless

# 3. wrist_pitch + attach joint 가 hand_pitch_link 를 body 로 참조하는지 확인
./isaaclab.bat -p -m ust_ws.ust_hm_grip.scripts.verify_wrist_joints --headless

# 4. body0/body1 rel + drive 값 + limits + mimic gearing 전체 dump (deep)
./isaaclab.bat -p -m ust_ws.ust_hm_grip.scripts.diagnose_robotiq_attach --headless
```

### 2.3 monitor 모드 검증 (PICO/SteamVR 불필요)

```powershell
$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only --render_mode monitor --headless `
    --diag idle --steps 5 --process_priority normal
```
기대: `Action Manager: 3 active terms (pink_ik_cfg=14, left_gripper_action=1, right_gripper_action=1)` + `reached --steps=5` + FATAL/Traceback 없음 + `negative mass` 경고 없음.

### 2.4 정량적 pose drift 검증 (idle 60 step settle)

```powershell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.test_robotiq_pose --headless
```
기대: `VERDICT: PASS` — 12 robotiq joint 전부 ±10° tol 이내. 현재 best max drift 약 ±2.6° (8th session).

### 2.5 실제 텔레오퍼레이션 (PICO + SteamVR 켜진 상태)

```powershell
$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only --teleop_device pico_gripper `
    --vr_runtime pico_connect --gripper_signal_source grip `
    --render_mode steamvr_native --process_priority high
```
필수 사전 작업: SteamVR Settings → Controllers → Manage Controller Bindings → **"UST Teleop GR1T2 Gripper"** → Default → **Save Personal Binding** (또는 `python -m ust_ws.ust_hm_grip.scripts.open_binding_ui`). 그렇지 않으면 모든 action `bActive=False` 로 진단 로그가 안내.

### 2.6 SteamVR IPC 정리 (Oculus 충돌 / stale vrserver 시)

```powershell
# 관리자 PowerShell
python -X utf8 -m ust_ws.ust_hm_grip.scripts.cleanup_vr_env --restart-steamvr
```
→ Oculus service 비활성화 + SteamVR 모든 프로세스 종료 → PICO Connect 에서 PCVR 스트리밍 다시 시작 → fresh IPC namespace.

### 2.8 XRoboToolkit 백엔드 텔레오퍼레이션 (11th session, PCVR Input 차단 우회) — research/47 + [EXECUTION_GUIDE](XROBOTOOLKIT_EXECUTION_GUIDE.md)

> ⚠️ **PICO 단일 APK 제약 (12th session)**: PICO 4 Ultra 는 한 번에 stream APK 하나만 — XRoboToolkit Unity Client 와 PICO Connect 는 mutually exclusive.  PICO Connect 가 SteamVR session 시작 시 Unity Client APK 즉시 종료.  따라서 표준 실행은 **PC 모니터 렌더링** (`--render_mode monitor`) 만 가능, HMD stereo 는 Phase B 별도 경로 필요 (EXECUTION_GUIDE §10).

```powershell
# 사전:
#   1. PICO Connect 와 SteamVR 종료 (있으면) — 단일 APK 제약 우회
#       Get-Process "Pico Connect", vrserver -ErrorAction SilentlyContinue | Stop-Process -Force
#   2. C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service.win\runService.bat 실행
#   3. PICO 헤드셋: Apps -> XRoboToolkit -> PC IP 페어링 -> Controller=ON, Direction=Send -> Start
#   4. xrobotoolkit_sdk 빌드됨 (MSVC + setup.py install — EXECUTION_GUIDE §3)

$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit `
    --xrt_enable_body True `       # body 24-joint stream (waist+forearm)
    --full_body True `             # Phase D: HMD→head, waist→waist joint
    --gripper_signal_source grip `
    --render_mode monitor `
    --max_seconds 300 `
    --process_priority high
# --max_seconds 300 = 5분 자동 종료.  Ctrl-C 로도 정상 종료 가능 (NORMAL EXIT 라벨).
# --max_seconds 빼면 무제한 (Ctrl-C 만으로 끝남).
#
# 13th session flags (디폴트 True, 명시 안 해도 됨):
#  --xrt_enable_body True  : body skeleton 24-joint stream 활성 → waist, L/R forearm tracker
#  --full_body True        : Phase D — HMD→head_yaw/pitch/roll, waist→waist_yaw/pitch/roll
#                            매 frame side-channel articulation API 로 robot 에 target 주입
#
# 런타임 재 캘리브레이션: PICO 우측 컨트롤러 **A 버튼** 한 번 누름 →
#   (1) robot 이 default joint pose 로 즉시 snap
#   (2) 다음 frame 에 현재 user pose 를 새 zero 로 캡쳐 (HMD + waist + L/R wrist 모두)
#   (3) 이후 모든 target = delta-from-new-zero
#
# 표준 출력 (TRACK step=N 3 초마다):
#   [run_teleop][tcp_diag] base_link world pos=(...) + L/R TCP base_link pos=(0.003,±0.229,-0.235)
#   [run_teleop][phase_d] HMD/waist/wrist calibrated  (각 채널)
#   [run_teleop][TRACK step=N] HMD/waist delta euler + L/R wrist action pos+quat
```

진단: `python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_xrobotoolkit` (L1–L4 layered probe).
단독 SDK 검증: `python -X utf8 -m ust_ws.ust_hm_grip.scripts.minimal_pico_check --seconds 15`.

### 2.7 PCVR Input 진단 toolkit (10th session, 그리퍼 close 작동 안 할 때 차례로)

PICO grip 입력이 우리 앱까지 도달 안 하는 경우 — `bActive=False` 또는 `is_active=False` 영구 — 다음을 순서대로 실행해서 어느 layer 에서 차단되는지 진단:

```powershell
# 1. OpenVR Property API 채널 (PICO 가 noVal 미사용 확인)
$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_properties --seconds 12

# 2. SteamVR Action API + Personal Binding 상태
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw --seconds 10
# bActive=False 면 binding pipeline 차단 — open_binding_ui 시도

# 3. SteamVR OpenXR runtime 의 hand_tracking + headless 지원 확인
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_probe
# GREEN 이면 4~6 진행, RED 이면 OpenXR 경로 차단

# 4. pyopenxr headless session + hand_tracker (hand-tracking 이 활성 상태일 때)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_session --seconds 15

# 5. pyopenxr Action API + controller binding (FOCUSED 미도달이 normal)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_controller --seconds 18

# 6. Isaac Sim XR 부팅 + 같은 process 의 pyopenxr piggyback (FOCUSED 도달 OK, 그러나 secondary 에 input 안 옴)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_piggyback --render_mode steamvr_native --seconds 60
```

**6 모두 FAIL = PICO Connect 10.6.6 + PUI 5.15.4 + SteamVR 2.15.6 현재 환경의 fundamental block**. 차선책 (키보드 fallback 또는 ALVR) 외 방법 없음. 향후 PICO Connect 11.x / SteamVR 2.16+ / ALVR 등 환경 변경 시 같은 toolkit 으로 즉시 재검증 가능.

---

## 3. 의무적 코드 컨벤션

### 3.1 GR1T2 link/joint naming

- **Link**: `{side}_hand_pitch_link` (URDF 관례: link 은 segment 이름). `{side}_wrist_pitch_link` 는 **존재하지 않음**.
- **Joint**: `{side}_wrist_pitch_joint` (URDF 관례: joint 는 DoF 이름).
- 빌드 스크립트의 9.45 fix 가 이 사실을 코멘트로 명시. 새 코드에서도 동일 컨벤션 따를 것.

### 3.2 Pink IK frame naming dual rule

- `target_eef_link_names` 의 value → **Isaac Lab body name** (no prefix). 예: `"left_gripper_tcp_link"`.
- `FrameTask(first arg)` / `NullSpacePostureTask.controlled_frames` → **Pinocchio frame name**. USD prim path 의 prefix 가 붙음.
  - GR1T2 stock body: `GR1T2_fourier_hand_6dof_*` prefix.
  - 빌드 스크립트가 root 직하에 추가한 prim (예: `*_gripper_tcp_link`): prefix 없음.

### 3.3 env_cfg 의 `__post_init__` — list 누적 방지

`@configclass` 의 mutable default (예: `pink_controlled_joint_names`) 는 instance 간 공유된다.
`__post_init__` 에서 `append` / `update` 하면 누적 → shape mismatch 유발.
반드시 **새 list 로 재할당**:

```python
def __post_init__(self):
    super().__post_init__()
    self.actions.pink_ik_cfg.pink_controlled_joint_names = (
        list(GR1T2_PINK_CONTROLLED_ARM_JOINTS) + list(GR1T2_WAIST_JOINT_NAMES)
    )
```

### 3.4 Pink IK 의 `hand_joint_dim=0` 슬라이싱 버그

isaaclab core 의 `pink_task_space_actions.py:200` 이 `actions[:, -hand_joint_dim:]` 로 슬라이싱. Python 룰상 `-0:` 은 전체 텐서.
ust_hm_grip 처럼 `num_hand_joints=0` 인 cfg 는 이 버그에 직접 노출.
해결: `teleop/_pink_hand_dim_zero_patch.py` 의 `apply()` 가 `process_actions` 를 monkey-patch.
**env_cfg 와 run_teleop 두 곳에서 `_pink_hand_dim_zero_patch.apply()` 를 호출** (idempotent). 임의 삭제 금지.

### 3.5 stdout buffering / URDF 캐시

- `run_teleop.py` 진입 직후 `sys.stdout.reconfigure(line_buffering=True)` 강제 — PowerShell `python -X utf8` 의 stdout fake-hang 방지.
- env_cfg 의 USD→URDF 변환은 mtime 비교로 자동 캐싱. `force_conversion=True` 로 되돌리지 말 것 — 매 launch 30-90초 hang 보임.
- **빌드 후 변경 사항이 visual 에 안 나타나면 URDF cache 의심**: USD 가 갓 빌드돼도 직전 빌드의 URDF mtime 이 그 사이 미세하게 더 클 수 있음 → `Remove-Item -Force "$env:LOCALAPPDATA\Temp\urdf\GR1T2_with_robotiq.urdf"` 로 강제 삭제 (6th session 의 학습).

### 3.6 main loop 진단 + UI yield

`run_teleop.py:main` 의 main loop:
- `except BaseException` 으로 confine — `KeyboardInterrupt` 외 모든 예외 traceback 출력.
- 매 frame `time.sleep(0)` (OS scheduler yield) + 16 frame 마다 `simulation_app.update()` — `--process_priority high` 에서 Isaac Sim 창 동결 방지.
- 종료 시 `exit_reason` 명시 print — 다시는 silent shutdown 안 됨.

### 3.7 SteamVR IPC 진단

`vr_sampler.py` 의 `openvr.init()` 실패 시 `_format_openvr_init_failure` 가 psutil 로 vrserver / OVRServer_x64 / OVRServiceLauncher / PICO Connect 실시간 상태를 찍고 정확한 시나리오를 분기. 새 진단 추가 시 해당 함수에 추가.

### 3.8 Robotiq 옵션 A USD 빌드 패턴 (`build_robotiq_usd.py`)

현재 빌드 흐름 (`_attach_robotiq(side)` 가 side 별로 호출됨):

1. **GR1T2 stock USD flatten + Fourier 손 제거** (`_remove_fourier_hand`, 9.45 와 동일).
2. **`hand_pitch_link/visuals` + `hand_pitch_link/collisions` 제거** (`_strip_hand_pitch_link_geometry`, 8th session). body + end_effector_link 자식은 보존 — articulation 노드는 유지.
3. **Robotiq stock stage open + 모든 instanceable 끄기** (`SetInstanceable(False)`, 4th session). `Stage.Flatten()` 시 `/Flattened_Prototype_N` prim 이 만들어지면 CopySpec 후 reference 가 GR1T2 의 prototype 으로 cross-resolve 됨 → visual 이 thigh mesh 로 보임.
4. **Robotiq subtree 평탄화 import** (`Sdf.CopySpec`). reference 가 아니라 평탄화 — joint rename 가능.
5. **container Xform 의 transform op = rotation + wrist_translation** (URDF 변환기의 joint-transform-consistency 검사 통과). 현재 rotation = `Ry(180°)` (7th session).
6. **`{side}_` prefix 로 모든 joint rename** + **PhysxMimicJointAPI:rot{X,Y,Z}:referenceJoint rel 재작성**.
7. **outer_knuckle joint limit 대칭화** (`[0°, 47°]` → `[-47°, 47°]`, 2nd session). gearing=-1 follower 가 -47° 까지 갈 수 있도록.
8. **6 joint 전부에 PhysicsDriveAPI:angular + PhysxMimicJointAPI 재적용** (2nd session). stock USD 가 follower drive 를 삭제하므로 명시적으로 재추가.
9. **base_link + outer_knuckle 에 PhysicsMassAPI 부여** (2nd session). datasheet 0.925 kg 합산 위해 base 0.6 kg + outer_knuckle 0.05 kg.
10. **ArticulationRootAPI 제거** (humanoid root 만 유지, gripper 의 nested root 삭제).
11. **wrist FixedJoint 추가** (`*_hand_pitch_link` → `*_robotiq_arg2f_85/base_link`). `localRot0 = R quat`, `localRot1 = identity` (URDF transform consistency 통과).
12. **TCP frame 추가** (`*_gripper_tcp_link`) + fixed joint (base_link → TCP, +0.150 m local Z). World 위치는 `R(0,0,+0.15) + wrist_translation`.

build script 수정 시 위 순서 변경 금지 — 한 단계라도 빠지면 URDF 변환, articulation 초기화, 또는 visual rendering 에서 죽는다.

### 3.9 USD instancing 처리 — Sdf.CopySpec 사용 시 필수

외부 USD (예: Robotiq stock) 가 `instanceable=True` 인 prim 을 가지면 `Stage.Flatten()` 이 `/Flattened_Prototype_N` 을 *루트* 에 생성한다. `Sdf.CopySpec` 으로 sub-tree 만 복사하면 prototype 은 *원본* 에 남고 dst stage 에선 broken reference 가 됨.

**무조건 flatten 전에 un-instance**:
```python
for prim in src_stage.Traverse():
    if prim.IsInstanceable():
        prim.SetInstanceable(False)
flat = src_stage.Flatten()  # 이제 prototype 안 만들어짐
```

검증: `scripts/verify_all_visuals.py` 가 dst stage 의 각 body 의 `visuals` 자식 prim 이 실제 source 의 메시 이름 (`Defeatured_2F_85_*`) 인지 자동 확인.

### 3.10 hand_pitch_link 의 visual/collision 제거 (Robotiq 부착용)

GR1T2 stock USD 는 `{side}_hand_pitch_link` 아래에 hand 형태의 visual + collision mesh 를 갖고 있는데, Robotiq 를 그 위에 부착하면 두 mesh 가 공간적으로 겹쳐 보임. body 자체 (RigidBody + Mass) 와 `end_effector_link` 자식은 유지하고 `visuals` + `collisions` 자식만 제거:

```python
def _strip_hand_pitch_link_geometry(stage, side):
    wrist_path = _find_wrist_path(stage, side)
    for child_name in ("visuals", "collisions"):
        child_path = wrist_path.AppendChild(child_name)
        if stage.GetPrimAtPath(child_path):
            stage.RemovePrim(child_path)
```

이 작업은 **articulation chain / joint ref 에 영향 없음** — wrist_pitch_joint 가 hand_pitch_link 를 body 로, attach joint 도 hand_pitch_link 를 body 로 그대로 참조함 (8th session 의 `scripts/verify_wrist_joints.py` 검증).

### 3.11 lead-only actuator + binary action (9th session 재설계)

Isaac Sim 5.1 의 mimic constraint 가 일부 follower joint 에 적용 안 되는 known issue 가 있어 2nd~8th 세션은 `6 joint 전부 driven` fallback (design-guide #43 §6.6) 을 채택했었음. 그러나 9th 세션에서 신규 `test_robotiq_close.py` 가 발견:

- **OPEN 상태 (target=0)**: 6 joint drive 안정 ✓
- **CLOSE 상태 (gear × +0.785)**: 어떤 K/D 조합 (10/80 / 50/5 / 200/20 / 200/80) 도 follower 가 ±100~1000° 발산. closed 4-bar linkage 의 매-step kinematic 해와 PD 타깃의 누적 오차가 발산을 유발.

해법: **lead 1 개만 driving**, 5 followers 는 K=0 + small damping 으로 mimic + linkage 가 자동 coordinate:

```python
actuators["robotiq-{side}-lead"] = ImplicitActuatorCfg(
    joint_names_expr=[ROBOTIQ_LEAD_JOINT_{SIDE}],  # *_finger_joint
    effort_limit_sim=50.0, velocity_limit_sim=5.0,
    stiffness=200.0, damping=20.0,                 # τ = K/D = 0.1s
)
actuators["robotiq-{side}-followers"] = ImplicitActuatorCfg(
    joint_names_expr=[j for j in ROBOTIQ_ALL_JOINTS_{SIDE} if j != ROBOTIQ_LEAD_JOINT_{SIDE}],
    effort_limit_sim=10.0, velocity_limit_sim=5.0,
    stiffness=0.0, damping=2.0,                     # 위치 enforce 안 함
)

# BinaryJointPositionAction 도 lead 1 개로 (Action Manager 의 외부 형상 유지: 1 binary DoF / side)
left_gripper_action = BinaryJointPositionActionCfg(
    asset_name="robot",
    joint_names=[ROBOTIQ_LEAD_JOINT_LEFT],
    open_command_expr ={ROBOTIQ_LEAD_JOINT_LEFT: 0.0},
    close_command_expr={ROBOTIQ_LEAD_JOINT_LEFT: ROBOTIQ_CLOSE_RAD},
)
```

`ROBOTIQ_ALL_JOINTS_*` / `_ROBOTIQ_*_JOINTS_WITH_GEAR` 상수는 보존 — diagnose / pose-test 가 12 joint state 를 읽을 때 사용. actuator + action 에서만 lead-only 로 좁힘.

---

### 3.11-legacy 6-joint actuator + binary action (deprecated, 2nd~8th session)

> ⚠️ **이 fallback 은 OPEN 상태에서만 안정** — CLOSE 에서 발산. 9th 세션의 §3.11 lead-only 가 대체. 참고용으로 보존.

Isaac Sim 5.1 의 mimic constraint 가 일부 follower joint 에 적용 안 되는 known issue 가 있어, 이전 env_cfg 는 lead 1 개 대신 **6 joint 전부 driven** (design-guide #43 §6.6 fallback path):

```python
_ROBOTIQ_LEFT_JOINTS_WITH_GEAR = [
    ("left_finger_joint",                     +1.0),  # lead
    ("left_right_outer_knuckle_joint",        -1.0),  # rotZ mimic
    ("left_right_inner_finger_joint",         -1.0),  # rotX mimic
    ("left_right_inner_finger_knuckle_joint", +1.0),  # rotX mimic
    ("left_left_inner_finger_knuckle_joint",  +1.0),  # rotX mimic
    ("left_left_inner_finger_joint",          +1.0),  # rotX mimic
]
ROBOTIQ_ALL_JOINTS_LEFT = [j for j, _ in _ROBOTIQ_LEFT_JOINTS_WITH_GEAR]

actuators["robotiq-left"] = ImplicitActuatorCfg(
    joint_names_expr=ROBOTIQ_ALL_JOINTS_LEFT,
    effort_limit_sim=25.0,
    velocity_limit_sim=2.0,
    stiffness=10.0,
    damping=80.0,
)

left_gripper_action = BinaryJointPositionActionCfg(
    asset_name="robot",
    joint_names=ROBOTIQ_ALL_JOINTS_LEFT,
    open_command_expr ={name: 0.0 for name, _ in _ROBOTIQ_LEFT_JOINTS_WITH_GEAR},
    close_command_expr={name: gear * ROBOTIQ_CLOSE_RAD
                        for name, gear in _ROBOTIQ_LEFT_JOINTS_WITH_GEAR},
)
```

gearing 부호: `right_outer_knuckle_joint` 와 `right_inner_finger_joint` 는 -1 (lead 와 반대 방향 회전), 나머지 3 개 inner-side joint 는 +1. `BinaryJointPositionAction` 은 6 → 1 collapse 하므로 action manager 에는 여전히 `left_gripper_action=1` 로 보인다.

### 3.13 Backend 분기 (XRoboToolkit ↔ OpenVR) — 11th session

`gr1t2_gripper_device.py` 의 `start()` + `_read_action_inputs()` 가 `cfg.input_backend` 분기:

- **openvr** (default): 기존 SteamVR Action API + Personal Binding 경로.  10th session 차단 확인됨.
- **xrobotoolkit** (research/47 + [`XROBOTOOLKIT_EXECUTION_GUIDE.md`](XROBOTOOLKIT_EXECUTION_GUIDE.md)):
  - PICO 공식 XRoboToolkit gRPC + Unity Client APK 경로. Personal Binding 불요.
  - `teleop/xrobo_sampler.py` 의 `XRoboSampler` 가 `SteamVRSampler` 와 같은 snapshot dict 인터페이스 → retargeter / env_cfg / Pink IK / Robotiq drive 무수정.
  - 좌표계: `teleop/coord_transforms.py` 의 `xr_to_isaaclab` (XR `+X right / +Y up / +Z back per OpenXR LOCAL` → IL `+X forward / +Y left / +Z up`).  R_XR2IL 은 proper rotation (det=+1) — research/47 §7 의 가이드는 det=-1 (improper) 이라 그대로 쓰면 quat conjugation 깨짐.  보정 사유는 EXECUTION_GUIDE §14 참고.
  - quat 은 xyzw → wxyz reorder (`xyzw_to_wxyz` helper).
  - 사전 조건: `XRoboToolkit-PC-Service.win\runService.bat` 실행 + Unity Client APK 의 Direction=Send.
  - **PICO 단일 APK 제약 (12th session)**: PICO 4 Ultra OS 는 한 번에 streaming APK 하나만 — XRoboToolkit Unity Client 와 PICO Connect 의 in-headset companion 은 mutually exclusive.  PICO Connect 가 SteamVR session 을 시작하는 순간 Unity Client APK 종료.  따라서 `--render_mode steamvr_native` (HMD stereo) 와 조합 불가; PC 모니터 렌더링 (`--render_mode monitor`) 만 표준.  HMD 시각화 필요 시 EXECUTION_GUIDE §10 의 Phase B 경로 (CloudXR / ALVR / 키보드 fallback) 참고.

### 3.14 Retargeter double-transform 가드 + 수직 매핑 (13th session, part 2-3)

- **`pose_in_il_frame` cfg flag** (`gr1t2_gripper_retargeter.py`): XRoboToolkit 백엔드 사용 시 `XRoboSampler._pose_to_pq` 가 이미 IL frame 으로 변환된 pose 를 snapshot 에 넣으므로 retargeter 가 `ct.svr_to_isaaclab` 을 또 적용하면 안 됨 (R_XR2IL == R_SVR2IL 이지만 R²≠R → 좌표 mangling).  3 callsite (`_user_pelvis_origin_il`, `_from_forearm`, `_from_controller`) 모두 `if not pose_in_il_frame:` 가드.  `gr1t2_gripper_device.py` 가 backend 별 `_pose_in_il = (input_backend=="xrobotoolkit")` 으로 자동 전달.
- **`subtract_waist_z=True`** (`pico_device_cfg` 디폴트): XR LOCAL 좌표계 원점은 보통 헤드셋 부근.  controller 의 IL Z (= XR +Y) 는 head-relative 이므로 robot wrist Z (pelvis-relative) 로 직매핑하면 wrist target 이 pelvis 아래 40cm (knee 근처).  `subtract_waist_z=True` 로 두면 user pelvis Z 도 빼서 controller_Z - waist_Z = "user 의 wrist-above-pelvis" → robot 의 chest 높이로 정상 매핑.

### 3.15 Phase D 풀바디 텔레오퍼레이션 (13th session, part 4) — direct articulation API side-channel

`run_teleop.py` 의 main loop 가 `env.step` 직전에 `_phase_d_apply(snapshot)` 호출 → robot head_yaw/pitch/roll + waist_yaw/pitch/roll joint 에 직접 target 주입.  **action layout (16-D Pink IK + gripper) 무변경** — action manager 와 무관한 side-channel.

`--full_body` CLI 디폴트 True with `--input_backend=xrobotoolkit`.  `_phase_d_resolve_joints()` 가 한 번만 호출, joint_ids 해소 후 매 frame 적용.  Missing joint (env 변형) 시 per-channel disable + 단일 warn.

Limits (clamp): head yaw ±1.5 rad / pitch ±1.0 / roll ±0.7, waist yaw ±1.2 / pitch ±0.6 / roll ±0.5.

Forearm tracker / ankle 은 의도적 미사용 (Phase D++ 차후 작업).

### 3.16 정자세 캘리브레이션 — 자동 + A 버튼 수동 (13th session, part 5-8)

**핵심**: HMD/waist tracker/controller 의 **raw** 값을 robot 에 직접 매핑하면 사용자의 startup 자세 (앉음, 약간 굽음, 컨트롤러 옆에 들음) 가 그대로 robot 의 영구 자세 (허리 굽힘, 손목 비틀림) 로 반영.  **delta-from-zero** 매핑 필수.

#### 자동 캘리브레이션 (startup)
`_phase_d_apply` 가 시작 시 **첫 valid (non-zero) sample** 을 zero 로 캡쳐:
- `_fb["zero_hmd_quat"]` — HMD quat
- `_fb["zero_waist_quat"]` — waist tracker quat
- `device._retargeter.cfg.controller_pose_zero` — L/R controller pose (pos+quat)

이후 매 frame 의 target:
- `head_target = euler_zyx(raw_hmd_quat * inv(zero_hmd_quat))` (delta from zero)
- `waist_target = euler_zyx(raw_waist_quat * inv(zero_waist_quat))`
- `wrist_pos_target = idle_pos + (raw_controller_pos - zero_pos) * scale + ctrl_offset`
- `wrist_quat_target = (raw_quat * inv(zero_quat)) * idle_q` ← part 7 fix.  Part 5-6 시점엔 wrist quat 는 raw 그대로 → LEFT 손목이 비틀림.  Part 7 에서 quat 까지 calibration.

#### 런타임 재 캘리브레이션 (A 버튼)
`_phase_d_check_recalibration(snap)` 이 매 frame 호출, **PICO 우측 컨트롤러 A 버튼** rising-edge 감지 (cooldown 0.5초 debounce):
1. `robot.write_joint_state_to_sim(default_joint_pos, zeros)` → 모든 joint 한 sim step 에 default 로 jump (part 7, **transient 없음**)
2. 모든 zero (HMD/waist/L/R wrist) reset → `None`
3. 다음 frame 에 현재 user pose 를 새 zero 로 capture

`xrobo_sampler.py` 의 `_read_face_button(name)` 헬퍼 + `_read_controller` 가 right→A/B, left→X/Y surface.  `getattr(xrt, "get_*_button", None)` graceful fallback (구버전 SDK 호환).

#### Quaternion helper (`coord_transforms.py`)
- `quat_conjugate(q)` = `(w, -x, -y, -z)` — unit quat 의 inverse, calibration delta 계산용
- `quat_wxyz_to_euler_zyx(q)` → `(yaw, pitch, roll)` — ZYX intrinsic, gimbal-lock-safe (asin clamp)

### 3.17 Idle pose anchoring at measured T-pose (13th session, part 8)

**`DEFAULT_LEFT_POS / DEFAULT_RIGHT_POS / DEFAULT_LEFT_QUAT / DEFAULT_RIGHT_QUAT` 는 실측값**:
- `DEFAULT_LEFT_POS = (0.003, +0.229, -0.235)` (base_link frame)
- `DEFAULT_RIGHT_POS = (0.003, -0.229, -0.235)` (L 의 Y mirror)
- `DEFAULT_LEFT_QUAT = DEFAULT_RIGHT_QUAT = (0, 0, 1, 0)` (180° about Y = palm-down, `build_robotiq_usd.py` 의 Ry(180°) 와 일치)

측정 방법: `scripts/inspect_tcp_world_pose.py` 또는 `run_teleop.py` 시작 시 `[tcp_diag]` 출력 (Isaac Sim 부트 + env.reset + `robot.data.body_pos_w[idx]` / `body_quat_w[idx]` 로 left/right_gripper_tcp_link 의 world 좌표 → base_link frame 으로 변환).

**기존값 (-/+0.20, 0, +1.05) + quat (0.707, 0, 0.707, 0) 는 잘못된 legacy**:
- Z=+1.05 (펠비스 위 1m, robot 머리 높이) ≠ 실측 -0.235m
- L X=-0.20 (뒤쪽), R X=+0.20 (앞쪽) — 좌우 비대칭 → LEFT 만 뒤쪽으로 reach → IK 가 LEFT arm 을 backward 로 twist

수정 후 calibration moment 의 wrist target = idle_q (그대로) → Pink IK 가 zero error → robot 이 정확히 default T-pose 유지.

`right_wrist_z180` 디폴트 **False** — idle quat 이 이미 palm-down 이라 Z180 hack 불요.

### 3.25 PICO Motion Tracker 앱 동작 매칭 — pos=SMPL, quat=controller (13th session, part 15, research/48)

**연구 결론** ([research/48](../research/48.%20xrobotoolkit_body_fusion_research.md)):
- `xrt.get_body_joints_pose()` = PICO PUI 의 AI 융합 24-joint SMPL 그대로 (XR_BD_body_tracking extension).
- SMPL wrist (idx 20/21) 는 fine controller 회전 미보존 → NVIDIA SONIC/GR00T 동일 패턴: **SMPL pos + controller quat overlay**.
- 캘리브 앱 화면 = 동일 SMPL 데이터를 렌더 layer 에서 controller quat overlay 한 결과.

**디폴트 split** (part 15):
- `wrist_pos_source = "wrist_tracker"` (SMPL idx 20/21)
- `wrist_quat_source = "controller"` (delta-from-zero)

**APK 모드 자동 진단** ([xrobo_sampler.py:154](ust_ws/ust_hm_grip/teleop/xrobo_sampler.py:154)): startup probe 가 `body_avail` + `num_pmt` 조합으로 Full body / Object / None / BOTH 자동 판정 + 사용자 가이드 출력.

**사전 조건**:
- PICO PUI ≥ 5.11.0
- PICO Motion Tracker 앱에서 PMT 캘리브 완료 (≥ 2개)
- XRoboToolkit Unity APK → Tracking → Mode = `Full body tracking` + Direction = Send
- `runService.bat` 실행

### 3.24 `_resolve_eef_target` 구조적 dispatch (13th session, part 14)

**문제**: part 13 cal branch 는 정확했지만 router 가 controller 존재 시 항상 `_from_controller()` 진입. 그 안의 non-cal `else` 가 `controller_pose_zero` 캡쳐 전 raw controller pose 를 wrist target 으로 dump → part 13 source split 우회.

**수정**: `_resolve_eef_target` 가 source cfg 로 dispatch:
```python
pos_src  = (cfg.wrist_pos_source  or "controller").lower()
quat_src = (cfg.wrist_quat_source or "controller").lower()
if pos_src == "controller" and quat_src == "controller":
    # legacy router (forearm fallback 포함)
else:
    return self._compose_eef_target(snapshot, side, pos_src, quat_src, ...)
```

**`_compose_eef_target`** ([retargeter.py:597](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:597)) — 각 axis 독립 source 에서만 데이터. Source missing → idle (delta=0). Controller 는 `pos_src`/`quat_src` 가 명시적 `"controller"` 일 때만 호출 — silent 진입 불가. 진단 `_pos_source_last`/`_quat_source_last` stash.

**결과**: pre-cal startup window 부터 controller pose 가 wrist target 에 영향 못 미침. Body wrist 트래커 첫 valid sample 후부터 비로소 wrist 따라감.

### 3.23 Controller 완전 decoupling — wrist pos+quat 모두 body tracker (13th session, part 13)

**의도**: controller 는 buttons (trigger/grip) 만 사용. wrist EEF 의 pos+quat 둘 다 body skeleton SMPL wrist 가 단독 driving. Body data 없으면 idle fallback (controller 절대 재진입 불가).

**Cfg** ([retargeter.py:206-226](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:206)):
```python
wrist_pos_source:  str = "controller"   # device cfg default "wrist_tracker"
wrist_quat_source: str = "controller"   # device cfg default "wrist_tracker"
wrist_pose_zero: Optional[Dict] = None  # {"left":{"pos","quat"}, ...}
```

**Cal branch idle fallback** ([retargeter.py:457-540](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:457)):
```python
# POS
if wrist_pos_source == "wrist_tracker":
    if tracker_live + zero_captured:
        delta_pos = wt_pos - wt_zero
    else:
        delta_pos = 0      # → wrist target = idle_pos (NOT controller)
else:
    delta_pos = ctrl_pos - ctrl_zero   # legacy

# QUAT
if wrist_quat_source == "wrist_tracker":
    if tracker_live + zero_captured:
        quat = (wt_quat * inv(wt_zero_q)) * idle_q
    else:
        quat = idle_q      # → fallback idle (NOT controller)
else:
    quat = (ctrl_quat * inv(ctrl_zero_q)) * idle_q   # legacy
```

**run_teleop wrist quat zero 캡쳐**: body wrist 가 처음 valid 할 때 `wrist_pose_zero[side] = {"pos", "quat"}` 둘 다 저장. A 버튼 recal 도 둘 다 리셋.

**디폴트**: device cfg + pico_device_cfg 둘 다 `"wrist_tracker"` for pos + quat. 결과: PICO controller 만 들고 body 트래커 없으면 robot wrist 정적 (idle). Body 트래커 paired + Body APK 토글 ON 시 body wrist 만 driving.

### 3.22 Wrist 위치 ↔ 회전 source 분리 (13th session, part 12)

**아키텍처**: wrist EEF target = `pos(body_wrist_tracker) + quat(controller)`. 사용자 의도 매핑.

**Cfg 추가** ([retargeter.py:206-225](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:206)):
```python
wrist_pos_source: str = "controller"  # or "wrist_tracker"
wrist_pose_zero: Optional[Dict] = None  # {"left":{"pos":(3,)}, "right":{"pos":(3,)}}
```

**Cal branch 분기** ([retargeter.py:434-487](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:434)):
```python
if wrist_pos_source == "wrist_tracker" and trackers[f"{side}_wrist"] and wrist_pose_zero[side]:
    delta_il = wt_pos - wt_zero          # body skeleton delta
else:
    delta_il = raw_ctrl_pos - controller_zero  # fallback controller pos
# Quat 항상 controller: quat_il = (raw_q * inv(zero_q)) * idle_q
pos_il = idle_pos + delta_il * axis_scale * position_scale + ctrl_offset
```

**run_teleop zero 캡쳐**: 시작 시 body wrist tracker pos 가 처음 valid 하게 도착할 때 캡쳐. A 버튼 recal 시 `wrist_pose_zero = None` 도 리셋.

**디폴트**: `GR1T2GripperDeviceCfg.wrist_pos_source = "wrist_tracker"` + `pico_device_cfg["wrist_pos_source"] = "wrist_tracker"`. SMPL body data 없을 때 controller pos 로 자동 fallback (graceful degradation).

### 3.21 Wrist tracker 노출 + IK orientation_cost 강화 (13th session, part 11)

**1. Wrist tracker (SMPL idx 20/21) 노출** — `_DEFAULT_BODY_ROLE_MAP` 에 `20: "left_wrist"`, `21: "right_wrist"` 추가. snapshot.trackers 에 body-skeleton 의 wrist 위치 surface (현재 정보 표시용; controller 가 여전히 PRIMARY driver).

**2. Pink IK FrameTask orientation_cost 1.0 → 6.0** (양 L/R). 이전 `position_cost=8.0` vs `orientation_cost=1.0` 8:1 weight 비율 때문에 IK 가 position 만 우선하고 wrist orientation 거의 free → 컨트롤러 회전해도 robot wrist 안 돌아감. 6.0 으로 bump 하면 position 여전히 우세하면서 orientation 실제 효력.

**3. TRACK diagnostic 강화**: wrist tracker pose + controller `rotation_delta=yaw/pitch/roll` (raw_q * inv(zero_q) euler) + robot 실제 wrist joint state (yaw/roll/pitch for 3-DoF wrist chain).

검증: smoke 로그 `Body role map (enable_body=True): {0: 'waist', 18: 'left_forearm', 19: 'right_forearm', 20: 'left_wrist', 21: 'right_wrist'}` + NORMAL EXIT.

### 3.20 Head actuator 복원 + Phase D / Wrist 축별 amplification (13th session, part 10)

**핵심 함정**: `isaaclab_assets/robots/fourier.py:129` 의 `GR1T2_HIGH_PD_CFG = GR1T2_CFG.replace(actuators={...})` 가 base actuators dict 를 **전체 덮어씀** → base 의 `"head"` ImplicitActuatorCfg 가 silently drop. `_gripper_robot_articulation()` 가 이 cfg 를 그대로 쓰면 head 에 PD 없음 → `set_joint_position_target` 호출은 되지만 implicit solver 가 joint 안 움직임.

**복원**: `env_cfg._gripper_robot_articulation()` 의 actuators dict 에 명시적 head actuator 추가:
```python
actuators["head"] = ImplicitActuatorCfg(
    joint_names_expr=["head_.*"],
    effort_limit_sim=50.0, velocity_limit_sim=20.0,
    stiffness=300.0, damping=20.0, armature=0.005,
)
```
검증: smoke 로그의 `Not all actuators are configured! 32 != 44` (이전 `29 != 44`) — 3 head joint actuated 확인.

**Amplification (사용자가 시각적으로 "미약" 으로 인지)**:
1. **Waist pitch/roll** — SMPL Pelvis quat 는 user pelvis 만 잡고 척추 굽힘 안 잡음. `run_teleop._WAIST_SCALE = {"yaw": 1.0, "pitch": 2.0, "roll": 1.5}` 로 pelvis delta amplify. 한계 `_WAIST_LIMITS_RAD`: pitch ±0.6 → ±1.0, roll ±0.5 → ±0.7.
2. **Wrist Z** — `GR1T2GripperRetargeterCfg.wrist_pos_scale_per_axis = (1.0, 1.0, 1.5)`. cal branch 에서 delta_il × axis_scale 적용.

**Cal branch -5cm Z bias bug 동시 fix**: 기존엔 `delta = pos_il - zero_pos` 에서 `pos_il` 가 post-offset (`forearm_to_wrist` -5cm Z) 이고 `zero_pos` 가 raw → 시작 시 delta_z = -0.05 → wrist target 이 idle_pos 보다 5cm 아래. fix: cal branch 에서 raw pos 사용:
```python
raw_pos = pose["pos"]
raw_pos_il = raw_pos if pose_in_il_frame else ct.svr_to_isaaclab(raw_pos, pose["quat"])[0]
delta_il = raw_pos_il - zero_pos
```
`controller_to_wrist_offset` 는 idle_pos 에 이미 흡수돼 있어 cal branch 에서 중복 적용 불요.

### 3.19 Phase D ↔ Pink IK 소유권 분리 — waist 조인트 (13th session, part 9)

**핵심 함정**: `KitchenSortingGR1T2GripperRobotOnlyEnvCfg` 가 `KitchenSortingGR1T2GripperWaistEnvCfg` 를 상속하므로 `pink_controlled_joint_names` 에 waist 3 조인트가 들어가 있음.  Pink IK 의 `apply_actions()` 가 `set_joint_position_target` 으로 arm + waist 전부를 매 frame 덮어씀 → Phase D 의 `set_joint_position_target(target, joint_ids=waist_ids)` 호출이 즉시 덮어써짐.

이 상태에선 사용자가 허리 트래커를 어떻게 움직이건 robot waist 는 Pink IK 의 redundancy 솔루션 (보통 약한 forward bend) 으로 따라감.  사용자 인지: "허리 트래커가 안 들어와요" / "정자세 캘리브해도 허리가 굽혀져요" — 실제로는 Pink IK 가 덮어쓴 것.

**해결**: `--full_body=True` + `--input_backend=xrobotoolkit` 조합에서 `run_teleop.py` 가 env_cfg 생성 직후 / env construction 직전에 waist 3 조인트를 Pink IK 의 controlled joint 셋에서 제거.

```python
_WAIST_JOINTS = {"waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"}
_pink_cfg = env_cfg.actions.pink_ik_cfg
_pink_cfg.pink_controlled_joint_names = [
    j for j in _pink_cfg.pink_controlled_joint_names if j not in _WAIST_JOINTS
]
# Mirror change on every NullSpacePostureTask
from isaaclab.controllers.pink_ik.null_space_posture_task import NullSpacePostureTask
for _task in _pink_cfg.controller.variable_input_tasks:
    if isinstance(_task, NullSpacePostureTask):
        _task.controlled_joints = [
            j for j in _task.controlled_joints if j not in _WAIST_JOINTS
        ]
```

`NullSpacePostureTask.controlled_joints` 도 같이 정리해야 null-space 마스크 일관.  Pink IK 는 14 arm joint 만 솔브 → waist 안 건드림 → Phase D 가 waist 의 유일한 owner.

**Head 조인트는 본래 `pink_controlled_joint_names` 에 없으므로 Phase D 가 이미 단독 소유자** — 그래서 head tracking 은 9 session 이전부터 동작했었음.  Waist 만 충돌이었음.

**TRACK 진단 출력 (3초 주기, part 9)**:
```
[run_teleop][TRACK step=N] LIVE channels:
  HMD pose:        (+0.58,-0.05,-0.40)    → head target euler yaw=-0.00 pitch=+0.00 roll=-0.00 (drives robot head_yaw/pitch/roll)
  waist tracker:   (+0.72,-0.15,-0.91)    → waist target euler yaw=-0.00 pitch=-0.01 roll=+0.00 (drives robot waist_yaw/pitch/roll)
  ...
  → action       L_wrist pos=(...) quat=(...)
  → robot actual head  yaw=+0.00 pitch=+0.00 roll=+0.00
                 waist yaw=+0.00 pitch=+0.00 roll=+0.00  (should match Phase D target euler above — divergence means Pink IK is overwriting)
```

target euler 와 robot actual 값이 일치하면 Phase D 가 소유 중.  Divergence 발생 시 즉시 Pink IK overwriting 의 회귀.

### 3.18 그리퍼 USD constraints fix (13th session, part 1)

Stock NVIDIA Robotiq USD 가 lead `finger_joint` 에만 baked 한 두 attribute 가 implicit-solver 와 충돌해 close target +0.785 rad 명령 시 lead 가 ~3° 에서 hard stall:
- `physxJoint:maxJointVelocity = 146.46` deg/s (followers 는 10000)
- `physxJoint:armature = 9.9999e-5` (1e-4 kg·m²)
- `drive:angular:physics:maxForce = 50` N·m (effort_limit 의 ceiling)

`isaac_file/build_robotiq_usd.py` §4d 의 6개 joint 전부에 author:
```python
mvel_attr.Set(10000.0)   # was 146.46 on lead
arm_attr.Set(0.01)       # was 1e-4, 100x larger
maxf_attr.Set(500.0)     # was 50
```

또한 `_ROBOTIQ_*_JOINTS_WITH_GEAR` 의 follower 부호를 실측 4-bar linkage 동작 기준으로 정정 (memory.md 13th part 1 참조).  Stock USD 의 PhysxMimicJointAPI gearing 은 PhysX 5.1 에서 강하게 enforce 안 되므로 mechanical linkage 가 실제 follower 방향 결정.

### 3.12 그리퍼 mount 회전 (`_wrist_to_gripper_rotation(side)`)

현재 적용: **`Gf.Rotation(Gf.Vec3d(0, 1, 0), 180.0)`** — world Y 축 180° 회전 (7th session, 사용자 요청).

GR1T2 wrist 가 bind pose 에서 world rotation = identity 이므로 wrist local 에 적용 = world 에 적용. 결과: gripper +Z (fingertip) → world -Z (down), gripper +X → world -X, gripper +Y 유지.

#### Rotation 변경 시 손대야 할 3 곳 (전부 같은 R 로)
- `container.world.transform` (rotation + wrist translation)
- `FixedJoint.localRot0` (wrist 쪽), `localRot1 = identity`
- TCP world transform (R 을 적용한 (0,0,+0.15) offset)

이 3 곳이 *URDF transform consistency* 검사를 통과하려면 동일해야 한다 (3rd session 의 학습).

#### 이전에 시도한 rotation 들 (memory.md 참조)
| Session | Rotation | Effect |
|---|---|---|
| 3rd | `Rx(∓90°)` per-side | gripper +Z → wrist ±Y (arm continuation), grasp axis = wrist ±Z |
| 5th | `Rx(∓90°) ∘ Rz(±90°)` per-side | 위 + grasp axis 가 wrist X (forward) 로 회전 |
| 6th | identity | gripper native 방향 (stock 그대로) |
| 7th | `Ry(180°)` (both sides) | fingertip down (palm-down 자세) ← **현재** |

수정 시 6th session 처럼 identity 로 reset 한 다음 한 step 씩 추가하는 것이 디버깅에 안전.

---

## 4. 자주 부딪치는 함정 (이미 모두 fix 됐지만 향후 회귀 주의)

| 증상 | 원인 | 코드 위치 |
|---|---|---|
| `pink.exceptions.FrameNotFound: "left_wrist_pitch_link"` | GR1T2 는 link 이름이 `*_hand_pitch_link` (joint 만 `*_wrist_pitch_joint`) | env_cfg 의 `target_eef_link_names`, `FrameTask`, `controlled_frames` 모두 `*_hand_pitch_link` 또는 `GR1T2_fourier_hand_6dof_*_hand_pitch_link` |
| `InitError_IPC_NamespaceUnavailable` | stale vrserver / Oculus runtime 점유 | `cleanup_vr_env.py --restart-steamvr` → PICO Connect 재시작 |
| `Simulation App Shutting Down` 3 초 후 (silent) | except KeyboardInterrupt 만 잡혔던 시절의 hand_joint_dim=0 슬라이싱 버그 | `_pink_hand_dim_zero_patch.apply()` 적용 + `except BaseException` 으로 확장됨 |
| `shape mismatch [31] vs [1, 17]` | hand_joint_dim=0 슬라이싱 + actions[:, -0:] = 전체 | 위와 동일 |
| `env_cfg = ...` 후 6.7 분 무응답 | PowerShell stdout buffering + 매번 강제 URDF 재변환 | `run_teleop.py` 의 line_buffering 강제 + env_cfg 의 mtime 캐시 |
| `joint 'Joints_finger_joint' is not unique` | 두 그리퍼 (좌/우) 가 같은 stock USD 를 reference 해서 joint leaf name 충돌 | `build_robotiq_usd.py` 가 평탄화 + `{side}_` prefix rename + PhysxMimicJointAPI rel 재작성 |
| `ValueError: 'left_robotiq_attach_fixed_joint' transforms are not consistent` | container Xform 의 rotation 과 FixedJoint.localRot 이 불일치 | container.xform 의 rotation = FixedJoint.localRot0 으로 통일 |
| Isaac Sim 창 클릭 안 됨 (`--process_priority high`) | HIGH-priority + 120 Hz 빡빡 루프가 input thread starve | `run_teleop.py` 의 `time.sleep(0)` + 16-frame extra `simulation_app.update()` |
| `bActive=False` for all controller actions | SteamVR Personal Binding 미저장 | `python -m ust_ws.ust_hm_grip.scripts.open_binding_ui` 또는 SteamVR Settings 에서 "Save Personal Binding" |
| 그리퍼가 **사슬처럼 늘어진 모양** | follower joint 의 PhysicsDriveAPI 가 stock 에서 삭제됨 + base_link 무질량 → 5.1 mimic known issue 발현 시 free fall | `_attach_robotiq` 의 step 8 (drive 재적용) + step 9 (mass 부여) — 9th 세션 이후 actuator 는 lead-only |
| **CLOSE 명령 시 follower joint 가 ±100~1000° 발산** | 6 joint 모두에 독립 PD 타깃 (gear×close) 을 부여 → 4-bar linkage 의 매-step kinematic 해와 어긋나 누적 오차 발산 | env_cfg §3.11 lead-only actuator (K=200 lead + K=0 followers) + BinaryJointPositionAction 도 lead 1 개로 |
| **CLOSE 명령 후 8초 동안 그리퍼 안 닫힘** | 이전 K=10 / D=80 으로 슬로우 폴 rate K/D=0.125 → 시상수 8초 | §3.11 의 K=200 / D=20 (τ=0.1s, ~0.3s 만에 close) |
| `[Warning] possibly invalid inertia tensor of {1.0, 1.0, 1.0} and a negative mass` | base_link / outer_knuckle 에 `PhysicsMassAPI` 미설정 | `_attach_robotiq` step 9 가 명시적 mass + diagonal inertia 부여 |
| 그리퍼 visual 이 thigh / human 손 mesh 로 보임 | stock USD 의 instanceable visuals 가 `Stage.Flatten()` 시 prototype 으로 collapse, `Sdf.CopySpec` 후 dst stage 의 동일 path prototype (GR1T2 의 것) 으로 cross-resolve | `_attach_robotiq` step 3 의 `SetInstanceable(False)` 전수 적용 → flatten 결과에 prototype 없음 |
| 그리퍼 fingertip 사이에 hand 모양 mesh 가 끼어 보임 | wrist link `hand_pitch_link/visuals` 가 그대로 남아 Robotiq base 와 공간적으로 overlap | `_attach_robotiq` step 2 의 `_strip_hand_pitch_link_geometry` 가 visuals + collisions 제거 |
| USD 빌드했는데 visual 변화 안 나타남 | URDF cache hit — 직전 URDF mtime 이 새 USD mtime 보다 크거나 같음 | 빌드 전 `Remove-Item -Force "$env:LOCALAPPDATA\Temp\urdf\GR1T2_with_robotiq.urdf"` |
| 그리퍼 회전이 의도와 다름 | `_wrist_to_gripper_rotation` 의 R 만 바꾸고 container.xform / FixedJoint.localRot / TCP world 중 하나가 빠짐 | 셋 다 같은 R 사용 — §3.12 의 "Rotation 변경 시 손대야 할 3 곳" |
| **PCVR 실연에서 PICO grip 입력이 우리 앱까지 도달 안 함** (`bActive=False` 영구) | SteamVR Action System 의 Personal Binding commit 이 우리 앱에 대해 silently 실패. SteamVR UI 의 "Replace Default Binding" 클릭 → vrsettings 의 `CurrentURL_steamvrinput` 키 reset → commit 안 됨. SteamVR 의 알려진 UI bug 또는 PICO Connect 10.6.6 + PUI 5.15.4 회귀. | 10th session 의 6 가지 우회 시도 모두 차단 (Action API binding / OpenVR Property API / Isaac Lab OpenXRDevice hand_tracking / pyopenxr headless hand_tracker / pyopenxr headless Action / pyopenxr piggyback same-process). **현재 차단 — 차선책 키보드 fallback 또는 ALVR 로 streaming 솔루션 교체 권장**. |
| **OpenXR `XR_MND_headless` session 이 FOCUSED 도달 못 함** (SteamVR 측 정책) | OpenXR Action API 의 `is_active=True` 는 session FOCUSED 가 필수 조건. SteamVR/OpenXR 2.15.6 가 headless session 을 VISIBLE → FOCUSED 로 transition 안 시킴 (graphics binding 없이 visible 불가). | 진단 toolkit 으로만 사용 — `scripts/diagnose_pyopenxr_session.py`, `scripts/diagnose_pyopenxr_controller.py` 가 검증된 reference. Production input pipeline 으로는 부적합. |
| **OpenXR multi-instance per process 시 secondary 가 input 못 받음** (SteamVR 라우팅) | 같은 process 의 두 OpenXR instance 모두 FOCUSED 도달해도 SteamVR 가 primary instance (Isaac Sim 의 omni.kit.xr.core) 에만 input 라우팅. | `scripts/diagnose_pyopenxr_piggyback.py` 가 이 패턴을 검증. Production 에서 우회 불가능. |
| **그리퍼 close 명령 시 lead joint 가 ~3° 에서 hard stall** (13th part 1) | Stock Robotiq USD 의 `maxJointVelocity=146 deg/s` + `armature=1e-4` + `drive.maxForce=50` 이 implicit-solver + mimic + 4-bar closed-loop 과 충돌 | `build_robotiq_usd.py` §4d 에서 10000 / 0.01 / maxForce=500 author + env_cfg actuator K=400 D=40 effort=500 (§3.18) |
| **컨트롤러 X 방향 움직임이 robot wrist 의 Y 방향으로 매핑** (xrobotoolkit 백엔드) | retargeter 가 IL-frame 으로 변환된 XRoboSampler snapshot 에 `svr_to_isaaclab` 을 또 적용 (double-transform); R_SVR2IL == R_XR2IL 이지만 R²≠R | `pose_in_il_frame=True` cfg flag (§3.14); `gr1t2_gripper_device` 가 backend 자동 감지 |
| **컨트롤러를 챙기 높이로 들었는데 robot wrist 는 hip 높이** | `subtract_waist_z=False` 시 controller XR Z (head-relative) → robot wrist Z (pelvis-relative) 로 직매핑 → 40cm 차이 | `pico_device_cfg["subtract_waist_z"] = True` (§3.14) |
| **시작 시 사용자 자세 (앉음, 약간 굽음) 가 robot 의 영구 자세 (허리 굽힘, 손목 비틀림) 로 매핑** | raw HMD/waist/controller quat/pos 를 그대로 robot target 으로 사용 → 사용자 startup 자세가 영구 매핑 | `_phase_d_apply` 가 첫 valid sample 을 zero 로 capture, `delta = raw * inv(zero)` 매핑 (§3.16) |
| **시작 시 왼손이 뒤로 꺾여 보임 + 컨트롤러를 머리 위로 들어야 reach** (13th part 8) | `DEFAULT_LEFT_POS=(-0.20,0,+1.05)` 가 잘못된 legacy 값 (Z=+1.05 머리 위, X=-0.20 뒤쪽; 좌우 X 부호 반대) | 실측값 anchor: `(0.003, +0.229, -0.235), quat=(0,0,1,0)` palm-down (§3.17); `right_wrist_z180=False` |
| **A 버튼 캘리브레이션 후 robot 이 PD transient 동안 어색한 중간 자세** (200ms 사이) | zero reset 만 하면 PD control 이 robot 을 현재 자세에서 idle 으로 천천히 이동 | `robot.write_joint_state_to_sim(default_joint_pos, zeros)` 으로 한 sim step 에 jump (§3.16) |
| **정자세 캘리브해도 robot waist 가 앞으로 굽힌 채로 유지 + 허리 트래커 데이터가 robot 에 반영 안 됨** (영상 13th part 9) | `robot_only` env 가 `WaistEnvCfg` 상속 → `pink_controlled_joint_names` 에 waist 3 조인트 포함 → Pink IK `apply_actions()` 가 매 frame `set_joint_position_target` 으로 Phase D 의 waist target 덮어씀 (head 는 본래 Pink IK 에 없어 정상 동작) | `--full_body=True` + `--input_backend=xrobotoolkit` 시 `run_teleop.py` 가 env_cfg 생성 직후 waist 3 조인트를 `pink_controlled_joint_names` + `NullSpacePostureTask.controlled_joints` 에서 제거 (§3.19) |
| **HMD 회전 0** (head joint target 작성되지만 robot head 안 움직임, 13th part 10) | `GR1T2_HIGH_PD_CFG = GR1T2_CFG.replace(actuators={...})` 가 base 의 `"head"` ImplicitActuatorCfg 를 silently drop → head joints 에 PD 없음 → implicit solver가 target 무시 | `env_cfg._gripper_robot_articulation` actuators dict 에 명시적 `"head"` ImplicitActuatorCfg 추가 (K=300, D=20) (§3.20) |
| **Waist pitch / roll 매핑 미약** (yaw 만 또렷, 13th part 10) | SMPL Pelvis quat 는 user 의 pelvis-frame 회전만 잡고 척추 (lumbar/thoracic) 굽힘 무시 → pelvis delta 작음 → 1:1 매핑 시 visible 효과 미약 | `_WAIST_SCALE = {"yaw":1.0, "pitch":2.0, "roll":1.5}` + `_WAIST_LIMITS_RAD` pitch ±0.6→±1.0 / roll ±0.5→±0.7 확장 (§3.20) |
| **Wrist Z 매핑 미약 + 시작 시 wrist target 이 idle 보다 5cm 아래** (13th part 10) | controller delta 1:1 + cal branch 에서 `delta = pos_il(post-offset) - zero_pos(raw)` → 시작 시 delta_z = -0.05 bias | `wrist_pos_scale_per_axis = (1, 1, 1.5)` + cal branch 에서 raw_pos 사용해 delta 계산 (§3.20) |
| **컨트롤러 회전이 robot wrist 회전으로 안 반영 + body wrist tracker 노출 안 됨** (13th part 11) | Pink IK FrameTask `orientation_cost=1.0` vs `position_cost=8.0` 8:1 weight → IK 가 orientation 거의 무시. SMPL idx 20/21 (L/R Wrist) 가 role map 에 없어 snapshot 노출 0 | `orientation_cost` 1.0 → 6.0 (양 L/R FrameTask). `_DEFAULT_BODY_ROLE_MAP` 에 `20: "left_wrist"`, `21: "right_wrist"` 추가 (§3.21) |
| **컨트롤러를 평행이동하면 robot 팔/손목 위치도 따라가버림 (사용자는 컨트롤러로 손 회전만 원함)** (13th part 12) | retargeter cal branch 가 controller pos 를 wrist EEF pos 의 단일 source 로 사용 → controller 이동이 robot arm 까지 영향 | `wrist_pos_source = "wrist_tracker"` cfg (디폴트). body skeleton SMPL L/R Wrist (idx 20/21) 가 pos delta source; controller 는 quat 만. Body data 없으면 controller fallback (§3.22) |
| **여전히 컨트롤러가 robot 팔 위치에 silent re-couple — part 12 fallback 이 controller 로 폴백** (13th part 13) | part 12 의 cal branch fallback (body tracker 부재 시 controller pos 사용) 이 사용자 인지의 "여전히 연동됨" 원인. 또한 wrist 회전 source 도 controller 였음 | `wrist_quat_source = "wrist_tracker"` cfg 추가, pos+quat fallback 모두 idle (delta=0) — controller 절대 재진입 못 함. `wrist_pose_zero` 에 quat key 도 저장 (§3.23) |
| **여전히 컨트롤러 활성화되면 robot 팔 위치/회전 연동됨 — startup window 에서** (13th part 14) | `_resolve_eef_target` router 가 controller 존재 시 무조건 `_from_controller()` 호출. 내부 non-cal else (cal 캡쳐 전) 가 raw controller pose 를 wrist target 으로 dump → source split 우회 | `_resolve_eef_target` 가 `wrist_pos_source`/`wrist_quat_source` 로 구조적 dispatch. 최소 한 source 가 wrist_tracker 면 `_compose_eef_target` 호출 — controller path silent 진입 불가 (§3.24) |

---

## 5. 옵션 A vs 옵션 B (현재 옵션 A 활성)

| 측면 | 옵션 A (현재) | 옵션 B (deprecated) |
|---|---|---|
| 그리퍼 | Robotiq 2F-85 stock USD | 자작 box-finger |
| Lead joint | `{side}_finger_joint` (1 lead + 5 mimics per side) | `{side}_gripper_finger_left/right_joint` (2 prismatic per side) |
| Mimic | PhysX MimicJointAPI 활성 + **6 joint 전부 drive** (5.1 known issue fallback, design-guide #43 §6.6) | 없음 |
| Close target | 0.785 rad (× gearing 부호 per follower) | 0.0 m (prismatic) |
| Open target | 0.0 rad | 0.04 m |
| Pink IK target | `*_gripper_tcp_link` (+0.150 m local Z from base) | `*_hand_pitch_link` (wrist 직접) |
| 빌드 스크립트 | `build_robotiq_usd.py` | `build_gripper_usd.py` |
| 산출 USD | `GR1T2_with_robotiq.usd` | `GR1T2_with_gripper.usd` |
| Actuator PD | **lead K=200 / D=20 / effort=50** + follower K=0 / D=2 / effort=10 (9th session lead-only, §3.11) | stiffness=2e3, damping=1e2, effort=200 |
| Mount rotation | `Ry(180°)` world Y axis (palm-down) | identity |

옵션 B 의 잔재 (`build_gripper_usd.py`, `GR1T2_with_gripper.usd`) 는 일단 보존 — 추후 cleanup PR 에서 제거 가능.

---

## 6. 외부 컨벤션 참고

- 36 번 설계 가이드: [`../research/36. gripper_elbow_tracker_pico_controller_migration_design_guide.md`](../research/36.%20gripper_elbow_tracker_pico_controller_migration_design_guide.md)
- 43 번 옵션 A 설계 가이드: [`../research/43. robotiq_2f85_optionA_migration_design_guide.md`](../research/43.%20robotiq_2f85_optionA_migration_design_guide.md)
- 사용자용 실행 가이드: [`EXECUTION_GUIDE.md`](EXECUTION_GUIDE.md)
- Robotiq 2F-85 stock USD: NVIDIA Isaac Sim 5.1 S3, `Assets/Isaac/5.1/Isaac/Robots/Robotiq/2F-85/`
- 알려진 issue: [Isaac Sim 5.1 known issues](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/overview/known_issues.html) — 2F-85 의 일부 follower link 가 lead 와 안 움직임 (현재 우리 빌드는 6 joint 전부 drive 로 우회)

---

## 7. 사용자 환경 — 2026-05-16 시점 (13th session 완료)

### 하드웨어 + 페어링
- **PICO 4 Ultra** 헤드셋 + 좌/우 Touch 컨트롤러 + **PICO Motion Tracker 5 개** (waist + L/R forearm + L/R ankle 위치 calibrated)
- Unity Client APK ON: Head, Controller, Body 토글 (PICO Motion Tracker Independent 토글은 OFF — 5개 트래커 데이터가 body 24-joint SMPL stream 으로 합쳐져 도착)
- `XRoboToolkit-PC-Service.win\runService.bat` 실행 중

### 렌더 / 입력 백엔드
- `--render_mode monitor` (PC 모니터 렌더링).  HMD stereo 는 PICO 단일 APK 제약으로 XRoboToolkit + PICO Connect 동시 불가 (12th session).
- `--input_backend xrobotoolkit` (PICO 공식 gRPC).  retargeter 가 `pose_in_il_frame=True` 자동 적용 (§3.14).
- `--xrt_enable_body True` (디폴트, §2.8).  body 24-joint stream 활성 → waist + L/R forearm tracker 도착.
- `--full_body True` (디폴트, §3.15).  Phase D 풀바디 — HMD→head, waist→waist joint 매 frame side-channel.
- `--process_priority high` 사용 (jitter ↓).  UI 동결 fix 적용됨 (§3.6).

### 캘리브레이션
- **자동** (startup): 첫 valid sample 캡쳐 → 모든 target = delta-from-zero (§3.16)
- **수동** (런타임): **PICO 우측 컨트롤러 A 버튼** 한 번 누름 → robot 즉시 default pose snap + 현재 자세 = 새 zero (§3.16)
- **Idle anchor**: `DEFAULT_*_POS/QUAT` 가 실측 T-pose 값 (§3.17) → calibration moment 에 robot 이 정확히 default T-pose 유지

### 시스템 환경
- 사용자 OS: Windows 11 Pro, NVIDIA RTX PRO 6000 Blackwell
- conda env: `ust` (`C:\Users\pjwpy\miniconda3\envs\ust\python.exe`) — `isaacsim-rl 5.1.0.0`, `xrobotoolkit_sdk 1.0.2` (Pybind 빌드 완료)
- MSVC v14.29.30133 (VS2019 BuildTools)

이 환경이 바뀌면 본 파일의 명령어 / 권장사항도 다시 점검 필요.  특히 `inspect_tcp_world_pose.py` (또는 `run_teleop.py` 시작 시 `[tcp_diag]` 출력) 의 측정값이 `DEFAULT_*_POS/QUAT` 와 일치하지 않으면 robot 거동 비정상.

---

## 8. 디버깅 워크플로우 (그리퍼 시각 / 거동 문제 시)

새 세션에서 "그리퍼가 이상하다" 류의 보고를 받으면, 아래 순서로 점검:

1. **URDF cache 무효화**: `Remove-Item -Force "$env:LOCALAPPDATA\Temp\urdf\GR1T2_with_robotiq.urdf"`. 이전 빌드의 cache 가 hit 되어 변경이 안 보일 수 있음.
2. **USD rebuild**: `./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_robotiq_usd.py`
3. **빠른 검증** (visual 먼저):
   - `scripts/verify_all_visuals.py` → 18/18 OK 면 visual mesh 는 정상 (instancing leak 아님).
   - `scripts/verify_gripper_world_pos.py` → fingertip direction 확인 (rotation 의도와 일치?).
   - `scripts/verify_wrist_joints.py` → wrist_pitch / attach joint 가 hand_pitch_link 를 body 로 참조하는지.
4. **물리 거동**:
   - `scripts/diagnose_robotiq_attach.py` → joint table 의 drive 값, limits, mimic gearing 확인.
   - `scripts/test_robotiq_pose.py` → 60 step settle 후 12 joint drift. ±10° 이하면 PASS.
5. **monitor 모드** smoke: `--steps 5` 로 `reached --steps=5` 확인 + FATAL/Traceback/negative mass 경고 없는지.
6. **GUI 시각 확인** (사용자에게 요청) — 위 4 단계가 자동 PASS 라도 visual 의도 확인은 인간 눈만 가능.

각 단계의 실패 패턴은 §4 함정 표에 매핑돼 있다.
