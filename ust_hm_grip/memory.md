# ust_hm_grip — Working memory / debugging log

> 본 폴더의 디버깅 / 마이그레이션 이력을 시간순으로 보존. 새 작업 시 [claude.md](claude.md) 의
> "자주 부딪치는 함정" 표와 함께 본 파일을 훑어 회귀 위험을 확인하세요.

---

## 일자별 작업 로그

### 2026-05-17 (13th session, part 15) — XRoboToolkit body fusion 연구 + PICO 캘리브 앱 동작 매칭 (pos=wrist_tracker, quat=controller)

사용자 질문: XRoboToolkit 이 PICO Motion Tracker 앱의 AI 트래킹 결과 그대로 데이터 수신 중인가? 캘리브 앱: 손목 트래커 → 팔+손목 위치, 컨트롤러 → 손목 회전.

#### 연구 결과 → [research/48](../research/48.%20xrobotoolkit_body_fusion_research.md)

**핵심 결론**:
1. **YES** — `get_body_joints_pose()` 24-joint SMPL stream 이 PICO Motion Tracker 앱의 AI 융합 결과 그대로. XR_BD_body_tracking extension 동일 채널.
2. **단 SMPL wrist (idx 20/21) 는 fine controller 회전 보존 X**. NVIDIA SONIC/GR00T 도 동일 이유로 SMPL pos + controller quat overlay 패턴 사용.
3. 캘리브 앱이 보여주는 "자연스러운 wrist 회전" = 앱 렌더링 layer 가 controller pose 를 wrist 에 overlay 한 결과. 별도 채널 없음.

#### 우리 구현 수정

**1. Cfg 디폴트** ([device.py](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_device.py)):
- `wrist_pos_source = "wrist_tracker"` (SMPL idx 20/21)
- `wrist_quat_source = "controller"` (delta-from-zero)

**2. env_cfg pico_device_cfg** 동일 디폴트.

**3. APK 모드 진단 추가** ([xrobo_sampler.py:154-175](ust_ws/ust_hm_grip/teleop/xrobo_sampler.py:154)):
시작 시 `body_avail` + `num_pmt` 조합으로 APK 모드 자동 판정:
- Full body (SMPL fusion ON)
- Object (raw PMT)
- None
- BOTH (비표준)

→ 사용자가 Unity APK 모드 잘못 설정한 경우 startup probe 가 명시적으로 알려줌.

#### 검증

| 검증 | 결과 |
|---|---|
| Unit tests | **142/142 PASS** (1 갱신: part 13 default 테스트가 part 15 split 검증으로) |
| Smoke | (running) |
| 연구 문서 | [research/48. xrobotoolkit_body_fusion_research.md](../research/48.%20xrobotoolkit_body_fusion_research.md) — 9 섹션 완전 분석 |

#### 사용자 다시 실행 시 차이점

1. **POS**: body wrist tracker (SMPL idx 20/21) 가 robot arm/wrist 위치 driving. PICO 캘리브 앱 동작과 동일.
2. **QUAT**: controller delta-from-zero 가 robot wrist 회전 driving. PICO 캘리브 앱 동작과 동일.
3. **시작 로그**: `APK tracking mode: Full body tracking (SMPL 24-joint AI fusion ON)` — 모드 명확.
4. **Controller pos** 는 영향 0 (part 14 dispatch + `wrist_pos_source=wrist_tracker`).
5. **Body data 없으면 idle** — controller silent re-couple 불가 (part 14 dispatch).

#### 산출 파일 변경

수정
- `teleop/gr1t2_gripper_device.py` — `wrist_quat_source` default "wrist_tracker" → "controller"
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — `pico_device_cfg["wrist_quat_source"] = "controller"`
- `teleop/xrobo_sampler.py` — APK mode auto-detect probe
- `tests/test_part13_full_wrist_tracker_decoupling.py` — default assertion 갱신

신규
- `research/48. xrobotoolkit_body_fusion_research.md` — 9-section 연구 문서

---

### 2026-05-17 (13th session, part 14) — `_resolve_eef_target` 구조적 dispatch: controller path 가 더 이상 wrist EEF 에 silent 진입 못 함

사용자 보고 (part 13 후): "여전히 컨트롤러 활성화되면 로봇 팔 위치/회전 연동됨". 손목 트래커가 항상 wrist 회전+위치 driving 원함.

#### 진단

Part 13 의 cal branch idle fallback 은 정확. 하지만 `_resolve_eef_target` 의 라우터:
```python
if self.cfg.prefer_controller_for_eef:
    if ctrl is not None and not _is_zero_pose(ctrl.get("pose")):
        return _from_controller()   # <-- controller present → 무조건 진입
```

`_from_controller()` 안에서 `controller_pose_zero is None` 이면 (run_teleop 캡쳐 전) cal branch 우회하고 line 545 `else:` 로 가서 **raw controller pose 를 그대로 pos_il/quat_il 로 dump** → robot wrist follows raw controller. Part 13 의 source split 무력화.

→ Controller present 가 첫 frame 부터 활성 (sampler 가 즉시 controller pose 받음). `controller_pose_zero` 캡쳐는 한 frame 늦음. 그 사이 사용자에게 visible.

#### 수정

**1. `_resolve_eef_target` 구조적 라우터** ([gr1t2_gripper_retargeter.py:552-595](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:552)):
```python
pos_src  = (cfg.wrist_pos_source  or "controller").lower()
quat_src = (cfg.wrist_quat_source or "controller").lower()
if pos_src == "controller" and quat_src == "controller":
    # legacy router (forearm fallback 등)
    ...
else:
    # 최소 한 source 가 wrist_tracker → _compose_eef_target
    return self._compose_eef_target(snapshot, side, pos_src, quat_src, ...)
```

**2. `_compose_eef_target` 신규** ([gr1t2_gripper_retargeter.py:597-707](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:597)):
- 각 axis (pos / quat) 가 자기 source 에서만 데이터 수집
- Source missing → idle (delta=0)
- **Controller path 는 `pos_src`/`quat_src` 가 명시적으로 `"controller"` 일 때만 호출** — silent 진입 불가
- 진단 stash: `_pos_source_last`, `_quat_source_last`

#### 검증

| 검증 | 결과 |
|---|---|
| Unit tests | **142/142 PASS** (135 → 142, +7 part 14) |
| 신규 test 파일 | [test_part14_dispatch_structural.py](ust_ws/ust_hm_grip/tests/test_part14_dispatch_structural.py) — pre-cal startup idle (1) + partial-cal idle (1) + tracker pos drive (1) + tracker quat drive (1) + cross-source mix (1) + legacy controller backcompat (1) + source diag (1) |
| Smoke | (running) |

#### 사용자 다시 실행 시 차이점

1. **시작 직후부터 robot wrist = idle** (controller_pose_zero 캡쳐 전에도). 컨트롤러 흔들기 효과 0.
2. **Body wrist 트래커 첫 valid sample 후부터** robot wrist 가 트래커 따라감 (pos + quat 모두).
3. **컨트롤러 buttons (trigger/grip)** 만 gripper 제어. 컨트롤러 pose 는 완전 무시.

#### 산출 파일 변경

수정
- `teleop/gr1t2_gripper_retargeter.py` — `_resolve_eef_target` 구조적 라우터, `_compose_eef_target` 신규

신규
- `tests/test_part14_dispatch_structural.py` — 7 cases

---

### 2026-05-17 (13th session, part 13) — Controller 완전 분리: wrist EEF pos+quat 모두 body wrist tracker 단독 소유, fallback 도 idle (no controller silent re-coupling)

사용자 보고 (part 12 후): "여전히 컨트롤러에 로봇 팔 위치 연동됨". 추가 요청: 손목 모션 트래커가 wrist 회전까지 driving.

#### 진단

Part 12 의 `wrist_pos_source="wrist_tracker"` 가 fallback 로직에서 body tracker 부재 시 silently controller pos 로 폴백 → user 의 신체 트래커 아직 안 잡힐 때 컨트롤러가 다시 pos 소유권 가져감 → 사용자 인지 "여전히 연동됨".

추가로 quat 은 항상 controller (part 11 → orientation_cost=6.0 → 회전 visible). 사용자가 이제 **wrist tracker 가 wrist 회전도** driving 원함. Controller 는 buttons (trigger/grip) 만 사용.

#### 수정

**1. `wrist_quat_source` cfg 추가** ([gr1t2_gripper_retargeter.py:206-226](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:206)):
```python
wrist_pos_source:  str = "controller"   # or "wrist_tracker"
wrist_quat_source: str = "controller"   # or "wrist_tracker"
wrist_pose_zero: Optional[Dict] = None  # {"left":{"pos","quat"}, "right":{...}}
```

**2. Cal branch 리팩터** ([gr1t2_gripper_retargeter.py:435-540](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:435)):
- POS source = "wrist_tracker" + tracker 부재 → `delta=0` (snap to idle). **Controller fallback 없음**.
- QUAT source = "wrist_tracker" + tracker 부재 → `quat_il = idle_q`. **Controller fallback 없음**.
- Idle fallback 이 핵심 — controller 가 silent 재진입 못 함.

**3. `wrist_pose_zero` quat 추가** ([run_teleop.py:1010-1037](ust_ws/ust_hm_grip/scripts/run_teleop.py:1010)):
```python
zero_dict[side] = {
    "pos":  np.asarray(wt_pose["pos"], ...),
    "quat": np.asarray(wt_pose["quat"], ...),  # NEW
}
```
시작 시 + A 버튼 recal 시 body wrist quat 도 zero 캡쳐.

**4. 디폴트**: `GR1T2GripperDeviceCfg.wrist_quat_source = "wrist_tracker"`, `pico_device_cfg["wrist_quat_source"] = "wrist_tracker"`.

#### 검증

| 검증 | 결과 |
|---|---|
| Unit tests | **135/135 PASS** (128 → 135, +6 part 13 + 1 part 12 test 갱신) |
| 신규 test 파일 | [test_part13_full_wrist_tracker_decoupling.py](ust_ws/ust_hm_grip/tests/test_part13_full_wrist_tracker_decoupling.py) — cfg defaults (2) + tracker-driven pos+quat (1) + idle fallback pos (1) + idle fallback quat (1) + huge controller no effect (1) + zero schema with quat (1) |
| Smoke | (running) |

#### 사용자 다시 실행 시 차이점

1. **컨트롤러 평행이동/회전 → robot wrist target 변화 0** (이전엔 fallback으로 영향 있었음)
2. **사용자 신체 wrist 이동 → robot 팔/wrist 위치 매칭** (Part 12 기능 유지)
3. **사용자 신체 wrist 회전 → robot wrist 회전 매칭** (NEW)
4. **Body data 없을 때 robot wrist = idle** (정적, controller 무관). 캘리브레이션이 시작되면 비로소 추적 시작.
5. **Controller buttons (trigger/grip) → gripper open/close**만 유지

#### 산출 파일 변경

수정
- `teleop/gr1t2_gripper_retargeter.py` — `wrist_quat_source` cfg, cal branch idle fallback
- `teleop/gr1t2_gripper_device.py` — cfg field + passthrough; default "wrist_tracker"
- `scripts/run_teleop.py` — wrist quat zero 캡쳐, device_cfg `wrist_quat_source` 전달
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — `pico_device_cfg["wrist_quat_source"] = "wrist_tracker"`
- `tests/test_part12_split_pos_quat_source.py` — fallback-to-controller test 갱신해서 idle-fallback 확인

신규
- `tests/test_part13_full_wrist_tracker_decoupling.py` — 6 cases

---

### 2026-05-17 (13th session, part 12) — Wrist 위치 ↔ 회전 소스 분리: body skeleton wrist tracker → pos, controller → quat only

사용자 요청 (part 11 후): 손목 트래커 (body skeleton SMPL L_Wrist/R_Wrist) → robot 팔·손목 위치 매칭. Controller → 손 회전만 (현재는 컨트롤러 pos 도 robot 팔/손목 위치에 반영됨).

#### 진단

기존 retargeter (`_from_controller` cal branch) 가 controller pose 전체 (pos + quat) 를 단일 source 로 사용 → controller 이동 시 robot arm/wrist 도 따라 이동. SMPL wrist tracker (idx 20/21) 는 part 11 에서 snapshot 에 surface 했지만 retargeter consumer 없음.

User intent: position 과 orientation 을 **독립 source** 로 분리.
- Body skeleton wrist tracker → 사용자의 실제 손 위치 (PMT 기반 IK 추정) → 더 anatomically accurate
- Controller → 사용자가 손에 쥔 컨트롤러 회전 → gripper 자체의 회전 의도

#### 수정

**1. Retargeter cfg 분리** ([gr1t2_gripper_retargeter.py:206-218](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:206)):
```python
wrist_pos_source: str = "controller"   # "wrist_tracker" or "controller"
wrist_pose_zero: Optional[Dict[str, Dict[str, Any]]] = None
    # {"left": {"pos": (3,)}, "right": {"pos": (3,)}}
```

**2. `_from_controller` cal branch 분기** ([gr1t2_gripper_retargeter.py:434-487](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:434)):
- `wrist_pos_source == "wrist_tracker"` + body tracker 존재 + zero 캡쳐됨 → `delta_il = wt_pos - wt_zero` (body skeleton)
- 그 외 → 기존대로 `delta_il = raw_ctrl_pos - controller_zero` (controller fallback)
- Quat 부분 **변경 없음** — controller delta_q * idle_q 그대로 (orientation 항상 controller 출신)

**3. run_teleop 가 wrist tracker zero 캡쳐** ([run_teleop.py:993-1029](ust_ws/ust_hm_grip/scripts/run_teleop.py:993)):
- 시작 시 + A 버튼 recal 시 첫 valid body wrist pose 를 zero 로 저장
- `wrist_pose_zero[side] = {"pos": <SMPL wrist pos>}`
- A 버튼 recal 시 `cfg.wrist_pose_zero = None` 으로 리셋

**4. Device cfg pass-through + env_cfg default `"wrist_tracker"`**:
- `GR1T2GripperDeviceCfg.wrist_pos_source: str = "wrist_tracker"` (새 디폴트)
- `pico_device_cfg["wrist_pos_source"] = "wrist_tracker"`
- `run_teleop` 가 device_cfg 로 전달

#### 검증

| 검증 | 결과 |
|---|---|
| Unit tests | **128/128 PASS** (119 → 128, +9 part 12) |
| 신규 test 파일 | [test_part12_split_pos_quat_source.py](ust_ws/ust_hm_grip/tests/test_part12_split_pos_quat_source.py) — cfg defaults (3) + controller mode unchanged (1) + wrist_tracker mode (3) + quat still from controller (1) + fallback (1) |
| Smoke | (running) |

#### 사용자 다시 실행 시 차이점

1. **컨트롤러 평행이동 → robot arm 위치 변화 없음** (이전엔 따라 이동했음)
2. **사용자 실제 손이 움직임 → robot arm/wrist 위치 따라감** (body skeleton wrist 매핑)
3. **컨트롤러 회전 → robot wrist 회전** (변경 없음; part 11 orientation_cost=6.0 유지)
4. **자동 캘리브레이션**: 시작 시 또는 A 버튼 누르면 body wrist 와 controller 둘 다 새 zero 캡쳐
5. **Body data 없을 때 fallback**: SMPL 데이터 오기 전엔 controller pos 로 fallback → 그래도 동작

#### 산출 파일 변경

수정
- `teleop/gr1t2_gripper_retargeter.py` — cfg `wrist_pos_source`/`wrist_pose_zero`, cal branch 분기
- `teleop/gr1t2_gripper_device.py` — cfg field + retargeter passthrough; default "wrist_tracker"
- `scripts/run_teleop.py` — wrist tracker zero 캡쳐 + A 버튼 recal reset + device_cfg 전달
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — `pico_device_cfg["wrist_pos_source"] = "wrist_tracker"`

신규
- `tests/test_part12_split_pos_quat_source.py` — 9 cases

---

### 2026-05-17 (13th session, part 11) — Wrist tracker 노출 + 컨트롤러 회전 IK orientation_cost 강화

사용자 보고 (part 10 fix 후): waist + head 정상. 남은 이슈:
- "손목 트래커 데이터 - 제어 X" — wrist tracker 가 어디에도 노출되지 않음
- "컨트롤러로 손 회전 반영" 안 됨 — controller rotate 해도 robot wrist 안 돌아감

#### 진단

**1. Wrist tracker 미노출**: `_DEFAULT_BODY_ROLE_MAP` 가 SMPL idx 0/18/19 (Pelvis + 2 forearms) 만 매핑. idx 20 (L_Wrist) / 21 (R_Wrist) **빠짐**. 따라서 snapshot.trackers 에 `left_wrist`/`right_wrist` 키 없음 → 모든 consumer (retargeter, TRACK diag) 가 wrist tracker 데이터 못 봄.

**2. Controller 회전 미반영 — IK weight imbalance**: 코드 추적 결과 retargeter math 는 정확:
```python
zero_inv = ct.quat_conjugate(zero_quat)
delta_q = ct.quat_multiply(quat_il, zero_inv)
quat_il = ct.quat_multiply(delta_q, idle_q)
```
시작 시 delta_q = identity → quat = idle_q (T-pose). User 회전 시 delta_q 변화 → quat_il 회전. 16-D action 의 `[3:7]` / `[10:14]` 에 정확히 들어감.

**문제는 Pink IK FrameTask 의 weight**:
```python
FrameTask("..._gripper_tcp_link", position_cost=8.0, orientation_cost=1.0, lm_damping=12, gain=0.5)
```
`position_cost=8.0` vs `orientation_cost=1.0` = **8:1 비율**. IK 가 position error 를 우선 minimize 하고 orientation 은 거의 free → wrist 3-DoF chain (yaw/roll/pitch) 이 rotation target 무시. 결과: 컨트롤러 회전해도 robot wrist 안 돌아감.

#### 수정

**1. Wrist trackers 노출** ([xrobo_sampler.py:54](ust_ws/ust_hm_grip/teleop/xrobo_sampler.py:54)):
```python
_DEFAULT_BODY_ROLE_MAP = {
    0:  "waist",
    18: "left_forearm",
    19: "right_forearm",
    20: "left_wrist",     # NEW — SMPL L_Wrist
    21: "right_wrist",    # NEW — SMPL R_Wrist
}
```

**2. Pink IK orientation_cost 1.0 → 6.0** ([env_cfg.py:546](ust_ws/ust_hm_grip/kitchen_sorting_gr1t2_gripper_env_cfg.py:546)) — both L/R FrameTask. Ratio 8:6 → position 여전히 우세 but orientation 효력 충분.

**3. TRACK diagnostic 강화** ([run_teleop.py:1217-1254](ust_ws/ust_hm_grip/scripts/run_teleop.py:1217), [1349-1365](ust_ws/ust_hm_grip/scripts/run_teleop.py:1349)):
- `left_wrist` / `right_wrist` body-skeleton pose 출력
- Controller `rotation_delta=yaw/pitch/roll` (raw_q * inv(zero_q) → euler)
- Robot 실제 wrist joint state: `L_wrist yaw=X roll=Y pitch=Z` (3-DoF chain)
→ 사용자가 컨트롤러 회전이 robot wrist 로 흘러가는지 실시간 검증 가능

#### 검증

| 검증 | 결과 |
|---|---|
| Unit tests | **119/119 PASS** (112 → 119, +7 part 11) |
| 신규 test 파일 | [test_part11_wrist_tracker_and_orientation.py](ust_ws/ust_hm_grip/tests/test_part11_wrist_tracker_and_orientation.py) — role map (4) + sampler emits wrist (1) + orientation_cost ≥6 (2) |
| Smoke | (in progress) |

#### 사용자가 다시 실행 시 차이점

1. **TRACK 출력에 wrist tracker pose 표시** — body skeleton 의 L/R wrist 위치 visible
2. **TRACK 출력에 controller rotation_delta euler** — 컨트롤러 회전이 zero 대비 얼마나 돌아갔는지 실시간
3. **TRACK 출력에 robot wrist actual yaw/roll/pitch** — IK 가 controller rotation target 을 실제로 wrist 조인트로 변환했는지 검증
4. **컨트롤러 회전 → robot wrist 회전** — orientation_cost 6× 강화로 IK 가 wrist target quat 따라감

#### 산출 파일 변경

수정
- `teleop/xrobo_sampler.py` — `_DEFAULT_BODY_ROLE_MAP` 에 idx 20/21 추가
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — FrameTask orientation_cost 1.0 → 6.0
- `scripts/run_teleop.py` — TRACK 출력에 wrist trackers + controller rotation_delta + robot actual wrist joint state

신규
- `tests/test_part11_wrist_tracker_and_orientation.py` — 7 cases

---

### 2026-05-17 (13th session, part 10) — Head actuator missing + waist pitch/wrist Z 약함 amplification

사용자 보고 (영상 `screanshot/bandicam 2026-05-17 00-39-49-276.mp4`): part 9 fix 후 허리 yaw OK, 그러나
- waist **pitch (앞뒤)** 미약
- wrist **Z (위아래)** 미약
- head 회전 **전혀 안 됨**
- translation 없음 (translation 은 별도 feature)

#### 진단

**1. Head 회전 0**: `isaaclab_assets/robots/fourier.py:129` 의 `GR1T2_HIGH_PD_CFG = GR1T2_CFG.replace(actuators={...})` — `replace(actuators={...})` 가 base `GR1T2_CFG` 의 actuators dict 를 **전체 덮어씀** → base 의 `"head"` ImplicitActuatorCfg (`head_.*` joints) 가 **silently drop**. 따라서 `head_yaw/pitch/roll_joint` 에 **PD 없음** → `set_joint_position_target` 호출은 되지만 implicit solver 가 PD 없으니 joint 가 안 움직임. Phase D 가 정확히 target 을 보내고 있는데 actuator 부재로 결과 0.

**2. Waist pitch 약함**: `_DEFAULT_BODY_ROLE_MAP[0] = "waist"` = SMPL Pelvis. SMPL Pelvis quat 는 user 의 PELVIS frame 회전만 잡음. 사용자가 "forward bend" 라고 인지하는 동작 대부분은 **척추 (lumbar/thoracic)** 굽힘이고 pelvis 자체는 거의 안 움직임 (특히 의자 앉음). Pelvis pitch delta 작음 → robot 의 waist_pitch_joint 가 1:1 매핑이라 시각적으로 미약.

**3. Wrist Z 약함**: controller delta Z 가 `position_scale=1.0` 으로 1:1. 사용자가 손을 위로 10cm 들면 robot wrist target 도 10cm 위. 작아 보임. 게다가 기존 cal branch 에 **숨겨진 -5cm Z bias** 버그: `controller_pose_zero` 가 raw ctrl pos 저장하지만 `delta = pos_il - zero_pos` 에서 `pos_il` 는 `forearm_to_wrist(pose, (0,0,-0.05))` 적용된 post-offset → 시작 시 delta_z = -0.05 → idle_pos_z 보다 5cm 아래에서 시작.

#### 수정

**1. Head actuator 복원** ([env_cfg.py:281-298](ust_ws/ust_hm_grip/kitchen_sorting_gr1t2_gripper_env_cfg.py:281)):
```python
actuators["head"] = ImplicitActuatorCfg(
    joint_names_expr=["head_.*"],
    effort_limit_sim=50.0,
    velocity_limit_sim=20.0,
    stiffness=300.0,
    damping=20.0,
    armature=0.005,
)
```

**2. Phase D scale + limit 확장** ([run_teleop.py:817-845](ust_ws/ust_hm_grip/scripts/run_teleop.py:817)):
- `_WAIST_LIMITS_RAD["pitch"]`: ±0.6 → ±1.0 (~34° → ~57°)
- `_WAIST_LIMITS_RAD["roll"]`:  ±0.5 → ±0.7 (~29° → ~40°)
- `_WAIST_SCALE = {"yaw": 1.0, "pitch": 2.0, "roll": 1.5}` — pelvis delta amplify
- `_HEAD_SCALE = {"yaw": 1.0, "pitch": 1.0, "roll": 1.0}` — 그대로 (HMD 는 1:1 OK)
- `_phase_d_apply` head/waist branch 에서 scale 적용 후 clamp

**3. Wrist 축별 scale + cal branch -5cm bias 제거** ([gr1t2_gripper_retargeter.py:412-449](ust_ws/ust_hm_grip/teleop/gr1t2_gripper_retargeter.py:412)):
- `wrist_pos_scale_per_axis: Tuple[float, float, float] = (1.0, 1.0, 1.5)` — Z만 1.5×
- Cal branch 에서 delta 계산 시 **raw pos** (pre-offset) 사용:
  ```python
  raw_pos = np.asarray(pose["pos"], dtype=np.float64)
  raw_pos_il = raw_pos if pose_in_il_frame else ct.svr_to_isaaclab(raw_pos, pose["quat"])[0]
  delta_il = raw_pos_il - zero_pos   # zero_pos 도 raw 임 (run_teleop 캡처 일치)
  pos_il = idle_pos + delta_il * axis_scale * position_scale + ctrl_offset
  ```
  → `controller_to_wrist_offset` (-5cm) bias 사라짐. 5cm Z dip 더 없음. 시작 시 delta=0 → wrist target = idle_pos 정확.

**4. Device cfg pass-through**: `GR1T2GripperDeviceCfg.wrist_pos_scale_per_axis` 추가 + retargeter 에 forward.

#### 검증

| 검증 | 결과 |
|---|---|
| Unit test | **112/112 PASS** (102 → 112, +10 part 10 tests) |
| 신규 test 파일 | [test_part10_scales_and_head_actuator.py](ust_ws/ust_hm_grip/tests/test_part10_scales_and_head_actuator.py) — wrist Z scale (4) + waist scale math (5) + head actuator presence (1) |
| Smoke (`--full_body --input_backend xrobotoolkit --steps 5`) | (실행 중) |

#### 사용자가 다시 실행 시 차이점

1. **Robot head 가 사용자 HMD 회전 따라감** — yaw / pitch / roll 모두 (이전엔 actuator 부재로 안 움직였음)
2. **Robot waist pitch 가 더 또렷하게 따라감** — pelvis delta 의 2× amplify + 한계 ±57°
3. **Robot waist roll 도 1.5× amplify + ±40°** 한계
4. **Wrist Z 1.5× amplify** + 시작 시 -5cm Z dip 사라짐 (cal branch idle alignment 정확)

#### 산출 파일 변경

수정
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — `actuators["head"]` 추가
- `scripts/run_teleop.py` — `_WAIST_LIMITS_RAD` 확장, `_WAIST_SCALE`/`_HEAD_SCALE` 추가, head/waist apply 에 scale
- `teleop/gr1t2_gripper_retargeter.py` — `wrist_pos_scale_per_axis` cfg, cal branch raw_pos delta + axis scale
- `teleop/gr1t2_gripper_device.py` — `wrist_pos_scale_per_axis` cfg pass-through

신규
- `tests/test_part10_scales_and_head_actuator.py` — 10 cases
- `screanshot/_convert_bandicam_2026_05_17_00_39.py` + `bandicam_2026_05_17_00_39_analysis/`

---

### 2026-05-16 (13th session, part 9) — Phase D ↔ Pink IK 소유권 충돌 fix: 허리/손목 트래커가 로봇에 반영 안 되던 결정적 원인

사용자 보고 (영상 `screanshot/bandicam 2026-05-16 11-44-07-938.mp4`): 정자세 캘리브레이션 후에도 로봇 허리가 계속 앞으로 굽힌다.  허리/손목 모션트래커 데이터를 못 받아 트래커 이동이 로봇에 전혀 반영 안 됨.

#### 진단 — 영상 8 keyframe 분석 (`bandicam_2026_05_16_11_44_analysis/`)

73초 영상:
- Frame 0~1140: 로봇이 앞으로 강하게 굽힘 (waist 거의 90°), 양 팔 앞으로 늘어뜨림 → 사용자 입력과 무관한 자세
- Frame 1425: A 버튼 reset 직후 → default T-pose 로 한 sim step 에 jump ✓ (`write_joint_state_to_sim` 동작 확인)
- Frame 1710~1995: 즉시 다시 굽힘 → calibration 으로 anchor 했어도 다음 frame 부터 무언가 다시 waist 를 굽힘 target 으로 설정 중

→ **A 버튼은 정상 동작**.  Reset 후 robot 이 default 자세로 한 번 가는 것 까지는 OK.  하지만 그 다음 frame 부터 즉시 robot 의 waist 가 다시 굽힘.  Phase D `_phase_d_apply` 가 매 frame target 을 쓰고 있는데도.

#### 결정적 원인 — Pink IK 가 waist 조인트 target 을 매 frame 덮어쓰고 있었음

코드 추적:

1. `KitchenSortingGR1T2GripperRobotOnlyEnvCfg` (사용자 `--env_variant robot_only`) 가 `KitchenSortingGR1T2GripperWaistEnvCfg` 상속
2. `WaistEnvCfg.__post_init__` 가 `self.actions.pink_ik_cfg.pink_controlled_joint_names = list(ARM_14) + list(WAIST_3)` (17 조인트 — Pink IK 가 waist 도 redundancy DOF 로 솔브)
3. Pink IK 의 `apply_actions()` 가 `self._asset.set_joint_position_target(processed_actions, controlled_joint_ids)` → **arm + waist 전부에 target 작성** ([pink_task_space_actions.py:321](source/isaaclab/isaaclab/envs/mdp/actions/pink_task_space_actions.py:321))
4. Phase D 의 `_phase_d_apply` 는 `env.step` 직전에 `robot.set_joint_position_target(target, joint_ids=waist_ids)` 를 호출 — 하지만 그 직후 같은 `env.step` 안에서 Pink IK `apply_actions` 가 즉시 덮어씀
5. Head joint 은 `pink_controlled_joint_names` 에 없으므로 Pink IK 가 안 건드림 → Phase D head 만 visible 하게 동작 ✓

→ User 가 waist 트래커를 어떻게 움직이건 robot waist 는 **Pink IK 가 16-D wrist target 을 달성하기 위해 계산한 redundancy 솔루션** (보통 약한 forward bend 가 effort 적게 듦) 으로 따라감.  Phase D 의 noisy delta=0 target 은 매 frame 덮어써짐.

#### 사용자 인지 vs 실제

| 사용자 인지 | 실제 |
|---|---|
| "허리 트래커 데이터가 안 들어옴" | trackers["waist"] 정상 수신, Phase D 의 zero_waist_quat 정상 capture, delta euler 정상 계산, `set_joint_position_target` 정상 호출.  **하지만 Pink IK 가 즉시 덮어씀** |
| "손목 트래커 데이터가 안 들어옴" | 손목 트래커는 본래 사용 안 함 — controller pose 가 PRIMARY wrist driver (`prefer_controller_for_eef=True`).  forearm tracker (PMT) 는 fallback only.  Wrist 가 사용자 의도와 다르게 움직이는 건 waist 가 굽혀서 어깨 위치가 앞으로 가서 IK 솔루션이 비틀어진 것 |
| "정자세 캘리브레이션이 안 됨" | A 버튼 캘리브레이션은 정상 (영상 frame 1425 default jump 확인).  하지만 다음 frame 부터 Pink IK 가 다시 waist 굽힘 |

#### 수정

`scripts/run_teleop.py`: env_cfg 생성 직후 + env construction 직전에 다음 블록 삽입 (`args.full_body and args.input_backend == "xrobotoolkit"` 일 때만 활성).

```python
_WAIST_JOINTS = {"waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"}
_pink_cfg = env_cfg.actions.pink_ik_cfg
# 1. Remove from pink_controlled_joint_names
_pink_cfg.pink_controlled_joint_names = [
    j for j in _pink_cfg.pink_controlled_joint_names
    if j not in _WAIST_JOINTS
]
# 2. Mirror on every NullSpacePostureTask so null-space mask stays consistent
from isaaclab.controllers.pink_ik.null_space_posture_task import NullSpacePostureTask
for _task in _pink_cfg.controller.variable_input_tasks:
    if isinstance(_task, NullSpacePostureTask):
        _task.controlled_joints = [
            j for j in _task.controlled_joints if j not in _WAIST_JOINTS
        ]
```

결과: Pink IK 는 14 arm joint 만 솔브 → waist 안 건드림 → Phase D 가 waist 의 유일한 소유자.

#### 진단 강화 (TRACK 출력)

`run_teleop.py` 의 3초 주기 TRACK 출력에 robot 의 **실제** joint state 추가:
```
[run_teleop][TRACK step=N] LIVE channels:
  ...
  → action       L_wrist pos=(...) quat=(...)
                 R_wrist pos=(...) quat=(...)  L_grip=±1 R_grip=±1
  → robot actual head  yaw=+0.00 pitch=+0.00 roll=+0.00
                 waist yaw=+0.00 pitch=+0.00 roll=+0.00  (should match Phase D target euler above — divergence means Pink IK is overwriting)
```

→ 사용자가 Phase D target euler 와 robot actual 값을 직접 비교 가능.  분기 발생 시 (target≈0 인데 actual=+0.5 pitch) Pink IK 가 덮어쓰고 있다는 직접적 증거.

#### 검증

| 검증 | 결과 |
|---|---|
| 전체 unit test | **102/102 PASS** (94 → 102, +8 ownership filter tests) |
| AST syntax | OK — `python -c "ast.parse(...)"` 통과 |
| 신규 test 파일 | `tests/test_phase_d_pink_ik_ownership.py` — filter 로직 + 디폴트 CLI contract 8 케이스 |

#### 사용자가 다시 실행 시 보게 될 변화

1. **시작 시 추가 로그**:
   ```
   [run_teleop][phase_d] Phase D owns the waist joints — removed from Pink IK control to stop Pink IK from overwriting the direct articulation targets.
     pink_controlled_joint_names: 17 → 14 (removed ['waist_yaw_joint', 'waist_pitch_joint', 'waist_roll_joint'])
     NullSpacePostureTask.controlled_joints: removed ['waist_yaw_joint', 'waist_pitch_joint', 'waist_roll_joint']
   ```
2. **Robot waist 가 사용자 waist 트래커를 그대로 따라감** — 캘리브레이션 후 정자세 유지, 사용자가 허리를 굽히면 robot 도 굽힘.
3. **A 버튼 reset 후에도 robot 이 default 자세 유지** (이전엔 즉시 다시 굽혀짐).
4. **TRACK 출력의 robot actual waist 값이 Phase D target euler 와 일치** → 사용자가 실시간으로 매핑 확인 가능.

#### 산출 파일 변경

수정
- `scripts/run_teleop.py` — env_cfg 생성 후 Pink IK waist 제거 + TRACK 출력에 robot actual head/waist 추가

신규
- `tests/test_phase_d_pink_ik_ownership.py` — 8 ownership filter 케이스
- `screanshot/_convert_bandicam_2026_05_16_11_44.py` — 영상 → GIF + 8 keyframe 변환
- `screanshot/bandicam_2026_05_16_11_44_analysis/` — GIF + 8 PNG

---

### 2026-05-16 (13th session, part 8) — Idle pose re-anchored to measured T-pose + L/R asymmetry fix + diagnostic 강화

사용자 보고 (스크린샷 + 영상 분석): 캘리브레이션을 계속해도 LEFT 손이 자꾸 뒤로 꺾이는 현상.  RIGHT 보다 LEFT 만 유독 제어 매칭 안 됨.  모션트래커 데이터 + waist 움직임 반영 의심.

#### 진단 — `bandicam_2026_05_16_left_wrist_analysis/` (GIF + 6 keyframes)

23.57초 영상, 6 keyframes 분석:
- Frame 0: robot 시작 자세 (default T-pose)
- Frame 126/252/378: **LEFT arm 이 뒤쪽으로 꺾이고**, RIGHT arm 은 자연스럽게 가슴 앞
- Frame 630: LEFT arm 뒤쪽으로 extended

#### 결정적 증거 — `inspect_tcp_world_pose.py` 실측 (`[tcp_diag]`)

Default 관절 상태 (T-pose) 의 실제 gripper TCP 위치:

| TCP | base_link 좌표 (측정) | quaternion |
|---|---|---|
| L_gripper_tcp_link | **(+0.003, +0.229, -0.235)** | **(0, 0, 1, 0)** (palm-down, Ry 180°) |
| R_gripper_tcp_link | **(+0.003, -0.229, -0.235)** | **(0, 0, 1, 0)** |

vs 기존 `DEFAULT_*_POS/QUAT`:
- L: (-0.20, 0, **+1.05**), quat=(0.707, 0, 0.707, 0) ← Z=+1.05 머리 높이, X=-0.20 **뒤쪽** ❌
- R: (+0.20, 0, +1.05), quat=(0.707, 0, 0.707, 0) ← Z=+1.05, X=+0.20 앞쪽 ❌

**완전히 잘못된 idle anchor**:
- Z=+1.05m (실측 -0.235m 와 1.3m 차이)
- L/R 의 X 부호가 반대 → LEFT 만 뒤쪽으로 reach 시도 → 어색한 IK twist (영상의 LEFT arm 뒤쪽 꺾임)

#### 수정

1. `teleop/gr1t2_gripper_retargeter.py`:
   - `DEFAULT_LEFT_POS = (0.003, +0.229, -0.235)` (측정값)
   - `DEFAULT_RIGHT_POS = (0.003, -0.229, -0.235)` (측정값, L 의 mirror)
   - `DEFAULT_LEFT_QUAT = DEFAULT_RIGHT_QUAT = (0, 0, 1, 0)` (palm-down, Ry 180°)
   - `right_wrist_z180: bool = False` (디폴트) — idle quat 이 이미 palm-down 이라 Z180 hack 불요
2. `kitchen_sorting_gr1t2_gripper_env_cfg.py`:
   - `_gripper_idle_action()` 의 16-D idle action 도 측정값으로 갱신
   - `pico_device_cfg["right_wrist_z180"] = False`
3. `teleop/gr1t2_gripper_device.py`:
   - `right_wrist_z180: bool = False` (디폴트)
4. `scripts/run_teleop.py`:
   - 시작 시 `[tcp_diag]` 진단 — `base_link` + 양 `gripper_tcp_link` 의 world+base_link 좌표 출력 (검증용)
   - TRACK 진단 강화: HMD/waist quat → euler 변환값 명시 (`yaw/pitch/roll`), retargeter 가 출력한 16-D action 의 wrist pos/quat 표시 (`L_wrist pos=(...) quat=(...)`, `R_wrist pos=(...) quat=(...)`, `L_grip / R_grip`)
   - `_last_action_cached` 으로 double-advance 방지 (retargeter hysteresis state 보호)

#### 검증 결과

| 검증 | 결과 |
|---|---|
| 전체 unit test | **94/94 PASS** (변화 없음, DEFAULT_*_POS 값만 바뀜) |
| `[tcp_diag]` 출력 | base_link=(0,0,+0.93) + L_TCP base_link pos=(+0.003,+0.229,-0.235) + R_TCP=(+0.003,-0.229,-0.235) — 측정값과 cfg 일치 |
| Wrist target 좌우 대칭 | `L_wrist pos=(+0.00,+0.27,-0.27) quat=(0,0,1,0)` + `R_wrist pos=(-0.01,-0.27,-0.27) quat=(0,0,1,0)` ← 양 wrist 가 palm-down 으로 동일 quat |
| Phase D head/waist driving 검증 | TRACK step=157: HMD live → `head delta euler yaw=-0.00 pitch=+0.00 roll=-0.00 (drives robot head_yaw/pitch/roll)` + Waist live → `waist delta euler yaw=-0.00 pitch=-0.01 roll=+0.00 (drives robot waist_yaw/pitch/roll)` ← Phase D 가 head/waist joint 에 target 보내고 있음 |
| Live smoke | 500 step / 10s = 50 Hz, ✅ NORMAL EXIT, FATAL/NameError 없음 |

#### 사용자가 다시 실행 시 보게 될 변화

1. **양 wrist 가 좌우 대칭으로 자연스러운 idle 자세** — L/R 모두 (X≈0, Y=±0.23, Z=-0.235) palm-down.  LEFT 뒤쪽 꺾임 해소.
2. `[tcp_diag]` 출력으로 실제 TCP 위치 vs DEFAULT 값 일치 확인 가능
3. TRACK 진단에 HMD/waist delta euler + wrist action 모두 표시 → 사용자가 무엇이 robot 으로 흘러가는지 즉시 검증
4. Robot waist 가 사용자 허리 움직임 따라감 (`drives robot waist_yaw/pitch/roll` 라벨이 보임 = Phase D 실제 동작 중)

#### 산출 파일 변경

수정
- `teleop/gr1t2_gripper_retargeter.py` — DEFAULT_*_POS/QUAT 측정값 + right_wrist_z180=False
- `teleop/gr1t2_gripper_device.py` — right_wrist_z180=False
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — _gripper_idle_action + pico_device_cfg.right_wrist_z180=False
- `scripts/run_teleop.py` — [tcp_diag] startup + 강화된 TRACK 출력 + _last_action_cached

신규
- `scripts/inspect_tcp_world_pose.py` — TCP world+base_link 측정 진단 (booting Isaac Sim)
- `screanshot/_convert_bandicam_2026_05_16_left_wrist.py` — 사용자 영상 → GIF + 6 keyframes

---

### 2026-05-16 (13th session, part 7) — Wrist quat calibration + 즉시 default pose snap

사용자 보고: 캘리브레이션 후에도 왼손이 뒤로 꺾인 채로 움직임 (스크린샷).  + A 버튼 캘리브레이션 시 즉시 정자세 복귀 원함.

#### 진단

1. **Wrist orientation calibration 누락**: 13th part 5/6 의 `controller_pose_zero` 가 **position** 만 calibration 하고 **quaternion** 은 raw 그대로 retargeter 의 `_apply_orientation` 으로 전달.  사용자가 controller 를 잡은 startup 방향이 robot wrist 의 영구 비틀림 (왼손이 뒤로 꺾인 자세) 으로 매핑.
2. **PD transient 동안 어색한 자세**: A 버튼으로 zero reset → 다음 frame 에 새 zero capture → wrist target = idle_q + delta(0) = idle_q.  하지만 PD control 이 robot 을 현재 자세에서 idle 로 100~200ms 동안 천천히 이동시키는 transient 가 보임.

#### 수정

1. `teleop/gr1t2_gripper_retargeter.py`:
   - `_from_controller` 의 calibration 분기에 wrist quat 처리 추가:
     ```python
     zero_inv = ct.quat_conjugate(zero_quat)
     delta_q = ct.quat_multiply(quat_il, zero_inv)
     quat_il = ct.quat_multiply(delta_q, idle_q)
     if side == "right" and right_wrist_z180:
         quat_il = ct.quat_multiply(quat_il, _Z180_WXYZ)
     ```
   - `freeze_orientation=True` 옵션도 calibration 분기에서 동작
2. `scripts/run_teleop.py`:
   - `_phase_d_check_recalibration` 의 A 버튼 trigger 시:
     ```python
     robot = env.scene["robot"]
     default_pos = robot.data.default_joint_pos.clone()
     default_vel = torch.zeros_like(default_pos)
     robot.write_joint_state_to_sim(default_pos, default_vel)
     ```
   - 모든 joint 가 한 sim step 안에 default 값으로 jump → "snapped to default idle pose" 메시지
3. `tests/test_gripper_retargeter.py` +2:
   - calibration 직후 wrist quat == idle_quat (사용자 controller 방향 무관)
   - 사용자 90° yaw rotation → wrist target = delta * idle_q (1:1 tracking)

#### 검증

| 검증 | 결과 |
|---|---|
| 전체 unit test | **94/94 PASS** (92 → 94, +2 wrist quat tests) |
| Live smoke (헤드셋 착용, 자동 calibration) | `wrist calibrated — L_zero=(+0.388,+0.184,-0.333) R_zero=(+0.413,-0.191,-0.400)` + `first head target = +0.000 +0.000 +0.000` + NORMAL EXIT |

#### 사용자가 다음 실행 시 보게 될 변화

1. **시작 시 robot 양 wrist 가 idle quaternion 자세** (idle T-pose 와 자연스러운 손바닥 방향) — 사용자 controller 방향과 무관
2. **A 버튼 누름 → 모든 joint 가 즉시 default pose 로 jump** + 새 zero 캡쳐.  중간 transient 없음.
3. 캘리브레이션 후 user 가 controller 회전 → robot wrist 가 동일 delta 만큼 회전 (1:1 tracking)

#### 산출 파일 변경

수정
- `teleop/gr1t2_gripper_retargeter.py` — `_from_controller` 의 wrist quat calibration
- `scripts/run_teleop.py` — A 버튼 trigger 시 `write_joint_state_to_sim` 호출
- `XROBOTOOLKIT_EXECUTION_GUIDE.md` — wrist quat + instant snap 설명 추가

신규 tests
- `tests/test_gripper_retargeter.py` +2: wrist quat at calibration / delta tracking

---

### 2026-05-16 (13th session, part 6) — 컨트롤러 A 버튼 런타임 재캘리브레이션

사용자 요청: PICO 우측 컨트롤러의 A 버튼 (lower face button) 으로 런타임에 정자세 재캘리브레이션 가능하게.

#### 구현

1. `teleop/xrobo_sampler.py`:
   - `_read_face_button(name)` 추가 — `get_{A,B,X,Y}_button` SDK API 호출, graceful fallback (getattr 로 미존재 SDK 대비)
   - `_read_controller("right")`: A (lower) / B (upper) 추가
   - `_read_controller("left")`: X (lower) / Y (upper) 추가
   - PICO Touch convention 따름
2. `scripts/run_teleop.py`:
   - `_fb["prev_a_button"]` / `_fb["last_recal_time"]` state
   - `_RECAL_COOLDOWN_SEC = 0.5` (debounce)
   - `_phase_d_check_recalibration(snapshot)` 신규 helper — rising-edge detect + cooldown.  Trigger 시:
     - `zero_hmd_quat = None`, `zero_waist_quat = None` (next frame 에 재 capture)
     - `device._retargeter.cfg.controller_pose_zero = None`
     - `first_target = False` (재 capture 후 first head target 로그 다시)
   - `_phase_d_apply` 시작 시 (zero capture 전) `_phase_d_check_recalibration` 호출
3. `tests/test_recalibration_trigger.py` 신규 — rising-edge + cooldown 로직 5 테스트:
   - 단일 trigger (held button → 1회만)
   - 풀고 다시 누르면 cooldown 후 fire
   - 안 누른 동안 안 fire
   - 첫 press 는 cooldown 무관 fire (sentinel -1.0)
   - 독립 state 인스턴스 간섭 없음
4. `tests/test_xrobo_sampler.py` +2:
   - face button default zero (SDK 미노출 → False)
   - `get_A_button=True` monkey-patch 시 snapshot.right.buttons.a = True

#### 검증

| 검증 | 결과 |
|---|---|
| 전체 unit test | **92/92 PASS** (85 → 92, 7 신규: 2 sampler + 5 trigger) |
| Live smoke (헤드셋 착용, body 토글 ON, A 미터치) | 정상 calibration 캡쳐 + NORMAL EXIT, no false trigger |

#### 사용자가 다음 실행 시 사용법

1. 자세 잡고 시작 → 자동 캘리브레이션
2. 자세를 바꿔서 재 캘리브레이션이 필요해질 때 → **A 버튼 한 번 누르기**
3. 즉시 `🔄 A button pressed — re-calibration triggered` 메시지 출력
4. 다음 frame 에 현재 자세를 새 zero 로 캡쳐 → robot 다시 idle T-pose 로 정렬

#### 산출 파일 변경

수정
- `teleop/xrobo_sampler.py` — `_read_face_button` 추가, A/B/X/Y 버튼 surface
- `scripts/run_teleop.py` — recalibration trigger + 0.5s cooldown
- `XROBOTOOLKIT_EXECUTION_GUIDE.md` — A 버튼 사용법 명시

신규 tests
- `tests/test_recalibration_trigger.py` — rising-edge + cooldown 5 cases
- `tests/test_xrobo_sampler.py` +2 face button cases

---

### 2026-05-16 (13th session, part 5) — Phase D 정자세 캘리브레이션 (HMD/waist/wrist auto-zero on startup)

사용자 보고: Phase D 활성 후 시작하자마자 로봇 허리가 forward-bent 되어있음 + 양 팔이 너무 낮아 컨트롤러를 매우 높게 들어야 wrist 가 올라감.

#### 진단

`_phase_d_apply` 와 retargeter `_from_controller` 가 raw quat/pos 를 그대로 사용 → 사용자의 startup 자세 (의자에 앉음, 약간 앞으로 굽음, 컨트롤러는 허리 옆) 가 그대로 robot 의 자세로 매핑.  Result:
- waist tracker quat → robot waist forward-bend
- Robot 어깨 앞으로 → arm reach 제한
- Controller pos (waist 옆) → wrist target IL_Z ≈ hip 높이 → 자연스럽지 못한 IK pose
- 사용자가 컨트롤러를 매우 높게 들어야 robot wrist 가 chest 까지 옴

User 요청: "정자세로 캘리브레이션이 되고 움직일 수 있게".

#### 수정 — Auto-zero 캘리브레이션

1. `teleop/coord_transforms.py`: `quat_conjugate(q) → (w, -x, -y, -z)` 헬퍼 추가 (unit quat 의 inverse).
2. `teleop/gr1t2_gripper_retargeter.py`:
   - `GR1T2GripperRetargeterCfg.controller_pose_zero: Optional[Dict]` 추가 — `{"left": {"pos", "quat"}, "right": {...}}`
   - `_from_controller`: 캘리브레이션 set 됐을 때 `wrist_target = idle_pos + (raw - zero) * scale + ctrl_offset` (delta-from-zero + idle anchor).
3. `scripts/run_teleop.py`:
   - `_fb["zero_hmd_quat"]`, `_fb["zero_waist_quat"]` startup 캡쳐
   - `_phase_d_apply` 가 매 frame 첫 valid (non-zero) HMD/waist quat 캡쳐 → 이후 `delta = raw * inv(zero)` 으로 head/waist target 계산
   - Wrist 캘리브레이션: 매 frame 첫 valid controller pose 양쪽 캡쳐 → `device._retargeter.cfg.controller_pose_zero` 에 주입 (mutable dataclass)
   - `np` import 를 main() scope 에 추가 (이전 누락으로 NameError)
4. `tests/test_coord_transforms_xr.py`: `quat_conjugate` 5 신규 테스트 (identity / flip / inverse / delta zero / delta extra-pitch).
5. `tests/test_gripper_retargeter.py`: `controller_pose_zero` 3 신규 테스트 (idle anchored / delta tracking / partial fallback).

#### 검증 결과

| 검증 | 결과 |
|---|---|
| 전체 unit test | **85/85 PASS** (77 → 85, 8 신규: 5 conjugate + 3 retargeter) |
| Live smoke (헤드셋 착용, body+head 토글 ON) | 모든 캘리브레이션 캡쳐 + `first head target = (+0.000, +0.000, +0.000)` |
| Live smoke 의 NORMAL EXIT + 510 step / 10s = 51 Hz | 정상 |

#### 사용자가 다음 실행 시 보게 될 변화

1. 시작 시 `[phase_d] HMD calibrated / waist calibrated / wrist calibrated` 메시지 + `first head target applied: yaw=+0.000 pitch=+0.000 roll=+0.000`
2. **Robot 이 시작부터 idle T-pose 유지** — 사용자가 의자에 앉거나 약간 굽어있어도 robot 은 똑바로
3. **Wrist target 가 idle wrist pos** — 사용자가 컨트롤러를 어디에 들고 있든 robot wrist 는 chest 앞 (idle position)
4. 이후 움직임은 1:1 tracking — 사용자가 머리 30° 돌리면 robot 도 30° 돌림, 컨트롤러 +0.1m 앞으로 내밀면 wrist 도 +0.1m 앞으로

#### 산출 파일 변경

수정
- `teleop/coord_transforms.py` — `quat_conjugate` 추가
- `teleop/gr1t2_gripper_retargeter.py` — `controller_pose_zero` cfg + delta-from-zero mode
- `scripts/run_teleop.py` — 캘리브레이션 캡쳐 + np import fix

신규 unit tests
- `tests/test_coord_transforms_xr.py` +5: quat_conjugate (identity/flip/inverse), delta-from-zero quat math (2)
- `tests/test_gripper_retargeter.py` +3: controller_pose_zero idle anchor / delta tracking / partial fallback

---

### 2026-05-16 (13th session, part 4) — Phase D 풀바디 텔레오퍼레이션 (HMD→head, waist→waist) 직접 articulation API path

사용자 요청: "풀바디 텔레오퍼레이션을 원하니 Phase D 작업 진행".

#### 설계 결정 — direct articulation API side-channel

대안 (action layout 확장: Pink IK 에 head/pelvis/elbow FrameTask 추가 + ACTION_DIM 16→30+) 은 invasive:
- env_cfg 의 Pink IK cfg 큰 변경
- retargeter ACTION_DIM + idle_action 변경
- test_robotiq_close/pose 모두 업데이트
- 다른 dependent 코드 회귀 위험

대신 ust_hm_glove 의 head-follow 패턴 차용 (`scripts/run_teleop.py:986 _update_head_follow`).  매 frame 마다 `robot.set_joint_position_target(target, joint_ids=head_ids)` 호출 — action manager 무관.

#### 구현

1. `teleop/coord_transforms.py`: `quat_wxyz_to_euler_zyx(q) → (yaw, pitch, roll)` 헬퍼 추가.  ZYX intrinsic, gimbal-lock-safe (asin clamp).
2. `scripts/run_teleop.py`:
   - `--full_body` CLI flag (default True with `--input_backend=xrobotoolkit`)
   - `_phase_d_resolve_joints()`: 한 번만 호출, `head_yaw_joint`/`head_pitch_joint`/`head_roll_joint` + `waist_yaw_joint`/`waist_pitch_joint`/`waist_roll_joint` 의 joint_ids 해소.  Missing joint 는 warn + per-channel disable.
   - `_phase_d_apply(snapshot)`: 매 frame 호출.  HMD quat → Euler → head target.  Waist tracker quat → Euler → waist target.  Clamp 범위: head yaw±1.5 / pitch±1.0 / roll±0.7, waist yaw±1.2 / pitch±0.6 / roll±0.5 rad.
   - Main loop 의 `env.step` 직전에 호출 — PhysX 가 같은 frame 에 target 반영.
3. `tests/test_coord_transforms_xr.py`: identity / yaw 90° / pitch 45° / roll 30° / gimbal lock 5 신규 테스트.
4. `tests/test_phase_d_quat_to_targets.py`: HMD neutral → 0 target, 90° yaw → π/2 target, clamping → ±limit, snapshot consumption path (hmd / trackers.waist) 검증 — 8 신규 테스트.

#### 검증 결과

| 검증 | 결과 |
|---|---|
| 전체 unit test | **77/77 PASS** (62 기존 + 11 Phase D 13th 시리즈 신규 + 5 euler + 8 phase_d_apply path) |
| live smoke (`--full_body True`, 10초) | `joint resolution: head_ids=[21, 16, 11] waist_ids=[2, 5, 8]` 자동 해소, 554 step / 10s = 55.4 Hz, ✅ NORMAL EXIT |
| Phase D math (HMD identity → 0 target, ±π/2 yaw → ±π/2, clamping → ±limit) | 모두 검증 |

#### 한계 (차후 Phase D++)

- Forearm tracker → elbow position: Pink IK 에 `PositionTask("left_elbow_pitch_link")` × 2 추가 + retargeter ACTION_DIM 16 → 22 확장 필요.  현 단계 미구현.
- Ankle tracker: 의도적으로 미사용 ("발목 픽스").
- 사용자 visible 효과:
  - 머리 돌리기 → 로봇 head 회전 ✓
  - 허리 틀기 → 로봇 torso 회전 ✓
  - Elbow 위치 → null-space optimization 으로 간접 영향만 (직접 driving 안 함, Phase D++ 작업)

#### 산출 파일 변경

수정
- `teleop/coord_transforms.py` — `quat_wxyz_to_euler_zyx` 추가
- `scripts/run_teleop.py` — `--full_body` CLI + `_phase_d_resolve_joints` / `_phase_d_apply` + main loop wire-up

신규
- `tests/test_phase_d_quat_to_targets.py` — Phase D math 검증 8 tests
- 5 Euler tests 추가됨 in `tests/test_coord_transforms_xr.py`

문서
- `XROBOTOOLKIT_EXECUTION_GUIDE.md` §8.3.3 Phase D 섹션 추가
- `memory.md` 13th-bis part 4

---

### 2026-05-16 (13th session, part 3) — 사용자 "HMD/모션트래커 트래킹 안됨" 의 진짜 원인 + 수직 매핑 fix + architecture 매핑 가이드

사용자 보고 (13th session part 2 fix 후 재 테스트): 여전히 HMD/모션트래커 트래킹이 visible 하게 안 됨.

#### 진단 — `_probe_body_indices.py`

25초 polling 결과:
- **HMD pose: LIVE** (peak |pos|=[0.189, 0.088, 0.117])
- **Body skeleton: LIVE** — **모든 24 SMPL joint populated** (Pelvis, L/R_Hip, L/R_Knee, L/R_Ankle, ..., L/R_Wrist, L/R_Hand 전부)

→ **데이터 흐름은 perfect**.  데이터가 retargeter 에 도달함.  하지만 retargeter 가 그 데이터를 robot motion 으로 visible 하게 매핑 안 함.

#### 실제 원인 — 16-D action architecture limit

현재 action = `[7 L wrist, 7 R wrist, 2 grippers]`.  **Head / torso / legs 액션 없음**.

| 트래커 | retargeter 가 사용? | 로봇 visible 효과 |
|---|---|---|
| Controller pose (양손) | PRIMARY wrist | ✓ wrist 따라감 |
| Controller grip/trigger | gripper close/open | ✓ 그리퍼 |
| Waist tracker | base_link origin (`use_waist_origin`) | △ 미세 (수직 매핑) |
| Forearm tracker | fallback only (`prefer_controller_for_eef=True`) | ✗ controller 우선 |
| HMD pose | snapshot only, retargeter 미참조 | ✗ head joint 없음 |
| Ankle tracker | role map 에서 제외 | ✗ "발목 픽스" 의도 |

사용자가 "안 됨" 으로 인지하는 이유: HMD 와 forearm 트래커 데이터가 도착하지만 **visible robot motion 으로 매핑 안 됨**.

#### 추가 수정

1. **수직 매핑 fix**: `pico_device_cfg["subtract_waist_z"]: False → True`
   - 이전: controller 의 XR Z (head-relative) 가 robot wrist Z (pelvis-relative) 로 직매핑 → wrist target ~40cm BELOW robot pelvis (knee 근처)
   - 이후: user pelvis Z 도 빼서 "controller height above user pelvis" 가 "robot wrist height above robot pelvis" 로 매핑.  smoke 검증: `L_pos Z: -0.441 → +0.472` (knee 근처 → chest 근처, 90cm 보정)

2. **Mid-run TRACK 진단** (`run_teleop.py`): 3초마다 wall-clock 기반으로 HMD/waist/forearm/controller 각 채널의 LIVE 값 + 어떤 source 가 PRIMARY 인지 / 어떤 게 fallback / 어떤 게 미사용인지 명시 출력.
   - step 124 예: `HMD=(+0.58,-0.05,-0.40) (not actuated — robot has no head joint targets in this 16-D cfg)`, `waist=(+0.72,-0.15,-0.91) (drives origin subtraction)`, `left_forearm=(+0.77,+0.07,-0.78) (fallback for L wrist when controller absent; ignored otherwise)`, `left_controller=(+0.39,-0.13,-0.42) (PRIMARY L wrist driver)`

3. **EXECUTION_GUIDE 매핑 표** (§8.3.4): 어떤 XR 입력이 어떤 snapshot key 로 들어가고 어떤 retargeter 분기를 통해 어느 로봇 효과를 생성하는지 명시.  `--prefer_controller False` 모드 안내 (forearm tracker 가 wrist driver).

#### 검증 결과

| 검증 | 결과 |
|---|---|
| `subtract_waist_z=True` smoke (`L_pos Z`) | **+0.472m** (이전 -0.441m, 정상 chest 높이) |
| TRACK diag 출력 (3초마다 wall-clock) | step 124/247/370 에서 정상 출력, 모든 채널 LIVE 표시 |
| 전체 unit test | **64/64 PASS** (변화 없음) |

#### 결론

데이터 흐름은 12th/13th session 의 fix 후 완벽.  사용자 인지의 "안 됨" 은 architecture limit (16-D action 이 wrist+gripper 만 actuated) 이 원인.  TRACK 진단으로 사용자가 무엇이 LIVE / 무엇이 사용 안 되는지 즉시 확인 가능.  forearm tracker visible driving 원할 시 `--prefer_controller False` 옵션 (CLI 이미 존재).

#### 향후 Phase D (풀바디 매핑)

사용자가 HMD / forearm / waist 의 visible robot effect 를 원하면:
- 16-D → 30+-D action 확장 (head, torso, leg joint actions 추가)
- env_cfg 의 Pink IK 에 head_pose + pelvis_pose FrameTask 추가
- GR1T2 의 head/spine/leg joint 들에 ImplicitActuator 추가
- retargeter 가 HMD pose / waist tracker 를 직접 robot joint 로 매핑

별도 큰 작업.  현 단계는 controller-based wrist teleop + body tracker visibility 진단 강화로 충분.

---

### 2026-05-16 (13th session, part 2) — 그리퍼 speed-up + HMD/body 트래킹 retargeter double-transform bug fix

사용자 보고: 1) 그리퍼 close/open 속도가 너무 느림.  2) PICO 모션트래커 5개 페어링 + XRoboToolkit 으로 데이터 송신 중인데 헤드트래킹 + 모션트래커 트래킹이 전혀 안되고 있는 듯.  발목은 픽스해야 함.

#### 진단 — channel probe (`screanshot/_probe_all_channels.py`)

`xrt.init()` 후 20초 polling 결과:
- **Controllers: LIVE** (`max_rt=1.00`, L/R pose 변화함)
- **HMD pose: LIVE** (`HMD=[-0.054, -0.027, -0.097]` 등 변화)
- **Body skeleton (24-joint): LIVE** (`is_body_data_available()=True`)
- **Independent PMT: EMPTY** (`num_motion_data_available()=0`, `serial_numbers=[]`)

→ 사용자의 "5개 트래커" 는 SDK 의 *independent PMT* API 가 아닌 **body skeleton 24-joint stream** 으로 도착.  XRoboToolkit-PC-Service 가 5개 PMT 데이터를 합쳐 SMPL 추정 → 24-joint body 로 노출.  Unity Client APK 의 "PICO Motion Tracker (Independent)" 토글은 OFF 인 게 정상 (그래야 body stream 으로 합쳐짐).

→ 그런데 retargeter 는 body 데이터를 못 받고 있음.  원인 2개 발견.

#### 원인 1 — `--xrt_enable_body` default=False

`run_teleop.py` 가 default 로 false 라 sampler 의 `_read_body_joints` 가 호출 안 됨 → `snap["trackers"]` 항상 빈 dict.  retargeter 가 waist 못 찾아 `use_waist_origin` 비활성, controller 가 base_link-relative 가 아닌 SteamVR-world 좌표로 들어감.

#### 원인 2 — Retargeter double-transform bug

XRoboSampler 의 `_pose_to_pq` 가 이미 `ct.xr_to_isaaclab` 적용해 **IL-frame** 포즈를 snapshot 에 넣음.  그런데 `gr1t2_gripper_retargeter.py` 의 `_user_pelvis_origin_il`, `_from_forearm`, `_from_controller` 가 다시 `ct.svr_to_isaaclab(pose)` 호출.  11th session fix 후 `R_SVR2IL == R_XR2IL` (det=+1, 같은 matrix) 이지만 `R² ≠ R` 이므로 결과 mangled — wrist target 이 엉뚱한 방향으로.

→ 컨트롤러로 그리퍼 close 는 작동했지만 (binary cmd 만 사용), waist/forearm 추적이 활성화돼도 좌표가 잘못 매핑.

#### 수정

1. `gr1t2_gripper_retargeter.py`:
   - `GR1T2GripperRetargeterCfg` 에 `pose_in_il_frame: bool = False` 추가
   - 3 callsite (`_user_pelvis_origin_il`, `_from_forearm`, `_from_controller`) 의 `ct.svr_to_isaaclab(...)` 호출을 `if not self.cfg.pose_in_il_frame` 로 가드
2. `gr1t2_gripper_device.py`:
   - `start()` 직전 `_pose_in_il = backend == "xrobotoolkit"` 계산 → retargeter cfg 에 전달
3. `xrobo_sampler.py`:
   - `_DEFAULT_BODY_ROLE_MAP` 에서 ankle (idx 7, 8) 제거 (사용자 "발목 픽스" 요청 반영)
   - `start()` 시 channel probe — HMD / body / PMT 활성 여부 즉시 출력
4. `run_teleop.py`:
   - `--xrt_enable_body` default `False → True`
5. `tests/test_gripper_retargeter.py`:
   - `test_pose_in_il_frame_skips_double_transform` — pose_in_il_frame=True 시 변환 skip 검증
   - `test_pose_in_il_frame_waist_origin_passthrough` — waist 원점 빼기가 IL-frame 에서 정확

#### Gripper speed-up

`kitchen_sorting_gr1t2_gripper_env_cfg.py`:
- Lead actuator `stiffness=200 → 400`, D=40 유지, effort=500 유지
- K*err_max = 400 * 0.785 = 314 N·m < effort=500 → 여전히 linear PD regime (no clamp)
- τ = D/K = 0.1s (이전 0.2s 의 절반) → 5τ = 0.5s 만에 close target 도달

#### 검증

| 검증 | 결과 |
|---|---|
| `test_robotiq_close` VERDICT | **PASS** |
| Lead close 진행률 | step 5: +7° (이전 +4°, **70% 빠름**) · step 30: **+22.94°** (이전 +4.36° 정체, **5배 빠름**) · final: **+42.65°** (target +44.98°, err -2.3° vs 이전 -4.9°) |
| 전체 unit test | **64/64 PASS** (62 + 2 신규 retargeter 가드 테스트) |
| `run_teleop --input_backend xrobotoolkit --xrt_enable_body True` 10초 smoke | 453 step / 10s = 45.3 Hz, channel probe 출력, ✅ NORMAL EXIT |

#### 사용자가 다시 실행 시 보게 될 변화

1. **그리퍼 close/open 가 ~2배 빠름**.  Grip 당기는 순간 거의 즉시 닫힘 (0.5s vs 이전 1s).
2. **Channel probe 출력**으로 무엇이 살아있고 무엇이 OFF 인지 즉시 확인 가능 — 헤드셋 미착용 / Unity APK 토글 OFF 의 경우 명확한 fix 메시지.
3. **Waist + forearm 트래커가 실제로 retargeter 에 도달** — 이전엔 default 가 disable + double-transform 둘 다 차단.  이제 user 가 헤드셋 착용 + Body 토글 ON 하면 waist origin 자동 보정 + controller pose 가 정확한 base_link-relative 좌표로 변환.
4. **Ankle 채널은 default role map 에서 제외** — wobble / drift 가 retargeter 에 새지 않음.

#### 산출 파일 변경

수정
- `teleop/gr1t2_gripper_retargeter.py` — `pose_in_il_frame` cfg + 3 callsite 가드
- `teleop/gr1t2_gripper_device.py` — backend 별 자동 전달
- `teleop/xrobo_sampler.py` — default role map (ankle 제거) + start() probe
- `scripts/run_teleop.py` — `--xrt_enable_body` default True
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — lead K 200→400
- `tests/test_gripper_retargeter.py` — 2 신규 테스트

신규 (진단)
- `screanshot/_probe_all_channels.py` — 20s polling 으로 HMD / body / PMT / controller 채널 상태 측정

---

### 2026-05-16 (13th session) — 그리퍼 close 실제로 못 닫히는 USD 제약 해결

사용자 보고 (12th session 텔레오퍼레이션 영상 `screanshot/bandicam 2026-05-15 22-38-21-451.mp4`): grip 당기면 반응은 하지만 그리퍼가 100% 닫히지 않고 찔끔만 움직임.

#### 정량 진단 (`test_robotiq_close.py`)

- Lead `left_finger_joint` 명령: +44.98° (= 0.785 rad close target)
- 실제 도달: **+3.36°** (7% only)
- Step 0~30 사이 거의 변화 없음 (+1.74° → +2.71° → +2.71°) ← **하드 스톨**
- 9th session 의 K=200 / D=20 / effort=50 lead-only 아키텍처는 NEVER ACTUALLY VERIFIED — close 측정 안 했었음

#### 근본 원인

USD `inspect_lead_joint.py` 검사 결과 lead `finger_joint` 가 stock Robotiq USD 로부터 baked 된 두 가지 제약을 갖고 있었음:

1. **`physxJoint:maxJointVelocity = 146.46`** (degrees/s = 2.56 rad/s) — followers 는 10000 deg/s 인데 lead 만 캡됨.  명목적으론 0.3s 면 close 가능해야 하지만 implicit solver 와의 상호작용에서 stall.
2. **`physxJoint:armature = 9.9999e-5`** (1e-4) — outer_knuckle 의 inertia 5e-5 + armature 1e-4 = total ~1.5e-4 kg·m².  너무 작아 LCP 솔버가 mimic + 4-bar linkage closed-loop 와 동시 풀기 어려움.

추가로 USD-level `drive:angular:physics:maxForce = 50` 이 actuator effort_limit 의 ceiling 이라 effort_limit_sim=500 이라도 50 으로 clamp.

#### 수정 — USD build_robotiq_usd.py §4d 의 6개 joint 모두

```python
mvel_attr.Set(10000.0)   # was 146.46 on lead (stock)
arm_attr.Set(0.01)       # was 9.999e-5 (1e-4), now 100x larger
maxf_attr.Set(500.0)     # was 50.0
```

env_cfg actuator (lead, both sides):
```python
effort_limit_sim=500.0   # was 50.0
velocity_limit_sim=1000  # was 5.0 (effectively unlimited)
stiffness=200.0          # unchanged
damping=40.0             # was 20.0 (slightly more damping)
```

#### `_ROBOTIQ_*_JOINTS_WITH_GEAR` 부호 정정

v4 test 결과 + USD inspection 으로 4-bar linkage 의 실제 mechanical 동작 측정:

| Joint | 9th session gear | 13th session corrected |
|---|---|---|
| `left_finger_joint` (lead) | +1 | +1 |
| `left_right_outer_knuckle_joint` | -1 | **+1** |
| `left_right_inner_finger_joint` | -1 | **+1** |
| `left_right_inner_finger_knuckle_joint` | +1 | **-1** |
| `left_left_inner_finger_knuckle_joint` | +1 | **-1** |
| `left_left_inner_finger_joint` | +1 | **-1** |

Stock USD 의 `PhysxMimicJointAPI.gearing` 값 (-1 for outer_knuckle, +1 for inner_finger_*) 는 다른 축 컨벤션 기준으로 author 됐고, PhysX 5.1 이 강하게 enforce 안 함 — 실제로는 4-bar linkage 의 mechanical 동작이 followers 의 부호를 결정.  env_cfg 의 gear 값은 test_robotiq_close.py + diagnose_robotiq_attach.py 만 사용 (lead-only BinaryJointPositionAction 은 안 씀).

#### 검증 결과 (v5)

- **`test_robotiq_close.py` VERDICT: PASS** ← Phase 1 (open) + Phase 2 (both close) + Phase 3 (asymmetric) 전부 OK
  - Lead: +40.04° (target +44.98°, err -4.93°, tol ±10° ✓)
  - 모든 5 followers: ±40° 도달 (err ±5° within ±25° tol ✓)
  - Phase 3 RIGHT 재 open: +7-8° (within ±10° tol ✓)
- **`test_robotiq_pose.py` VERDICT: PASS** (open settle 회귀)
- 전체 unit test 62/62 PASS
- `run_teleop --input_backend xrobotoolkit --max_seconds 8`: 345 step / 8.0s, NORMAL EXIT, controller pose live, no negative mass warnings

#### 산출 파일 변경

수정
- `isaac_file/build_robotiq_usd.py` — §4d 의 drive 재적용 block 에 maxJointVelocity / armature 추가 author
- `kitchen_sorting_gr1t2_gripper_env_cfg.py` — `_ROBOTIQ_*_JOINTS_WITH_GEAR` 부호 정정, lead actuator effort/damping/velocity 튜닝
- `isaac_file/GR1T2_with_robotiq.usd` — 재빌드 산출물

신규 (진단)
- `scripts/inspect_lead_joint.py` — USD joint 의 limits / armature / maxJointVelocity / body0 / body1 / mimic refJoint dump
- `screanshot/_convert_bandicam_2026_05_15_gripper.py` — 사용자 영상 → GIF + keyframe PNG 분석

#### 차후 (Phase B HMD 시각화)

12th session 의 PICO 단일 APK 제약 (아래 12th session 항목) 는 그대로 — XRoboToolkit ↔ PICO Connect mutually exclusive.

---

### 2026-05-15 (12th session) — PICO 단일 APK 제약 발견 + 텔레오퍼레이션 명령 PC 모니터 표준화

사용자 보고 (12th session 시작): "실연 시 PICO VR 에서는 하나의 APP 만 실행이 가능하다. PICO Connect — steamvr 을 사용하면 xrobotoolkit 은 종료가 된다."

#### 발견 사항

PICO 4 Ultra OS 는 한 번에 **streaming APK 하나만** 활성:
- `XRoboToolkit Unity Client` (XR-Robotics 공식, gRPC) — 우리 표준 input 경로
- `PICO Connect` 의 in-headset companion app — PCVR video → SteamVR session

두 APK 는 **mutually exclusive**: PICO Connect 가 SteamVR session 을 시작하는 순간 Unity Client APK 가 OS-level 로 종료됨.  USB 경합 / OpenXR runtime 충돌이 아니라 OS-level APK lifecycle 제약.

11th session 의 EXECUTION_GUIDE §8.4 가 가정했던 "PICO Connect (video) + XRoboToolkit (input) 동시 실행" 시나리오는 **불가능**.  §10 의 "공존 가능, USB 경합 가능성" 도 잘못된 표현 — 실제론 OS 가 다른 APK 를 강제 종료.

#### 변경 사항

`XROBOTOOLKIT_EXECUTION_GUIDE.md`:
- §0 한 줄 요약 끝에 **⚠️ PICO HMD 단일 APK 제약** 박스 추가
- §8.4 (구 HMD demo with `steamvr_native`) → **§8.4 실연 (PC 모니터 + XRoboToolkit) — 표준 경로** 로 재작성.  `--render_mode monitor --max_seconds 300 --process_priority high` 권장.  PICO Connect / SteamVR / vrserver 사전 종료 명령 포함.
- §9 백엔드 비교 표의 "SteamVR 실행 필요?" 행에 "PICO Connect 가 XRoboToolkit Unity APK 와 충돌 — 본 가이드 권장 X" 추가
- §10 (구 PICO Connect 공존) → **HMD 시각화 Options (Phase B / C)** 로 재작성.  Phase A (현재) / Phase B-1 CloudXR / B-2 ALVR / B-3 키보드 fallback / B-4 Virtual Desktop 비교 표.

`claude.md`:
- §2.8 캐논 명령어: 사전 단계에 `Get-Process "Pico Connect", vrserver | Stop-Process -Force` 추가, `--max_seconds 300` 채택
- §3.13 backend 분기 컨벤션 끝에 "PICO 단일 APK 제약 (12th session)" 문단 추가

#### 사용자가 실제 실행할 표준 명령

```powershell
# 사전
Get-Process "Pico Connect", vrserver -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\develop\IsaacLab\ust_ws\XRoboToolkit-PC-Service.win\runService.bat"
# 헤드셋: Apps → XRoboToolkit → PC IP 페어링 → Controller=ON, Direction=Send → Start

# 실연
$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only --render_mode monitor `
    --teleop_device pico_gripper `
    --input_backend xrobotoolkit --gripper_signal_source grip `
    --max_seconds 300 --process_priority high
```

#### 차후 (Phase B HMD 시각화)

사용자가 HMD stereo 가 꼭 필요해지면 — XRoboToolkit 가 아닌 다른 input 경로 (CloudXR / ALVR / 키보드 fallback / Virtual Desktop) 와 video 통합 진행.  현 단계는 PC 모니터로 충분.

---

### 2026-05-15 (11th session) — XRoboToolkit 백엔드 통합으로 PCVR Input 차단 해소

10th 세션이 6-path 우회 모두 차단 확정 후, research/47 의 PICO 공식 XRoboToolkit gRPC 경로로 backend 분기 추가. SteamVR Personal Binding 의존성 제거.

#### 추가 컴포넌트

- [`teleop/xrobo_sampler.py`](teleop/xrobo_sampler.py) — `XRoboSampler` (SteamVRSampler 와 동일 snapshot dict 인터페이스, 120Hz 폴 스레드)
- [`teleop/coord_transforms.py`](teleop/coord_transforms.py) — `R_XR2IL`, `xr_to_isaaclab`, `xyzw_to_wxyz`, `wxyz_to_xyzw` 추가
- [`scripts/minimal_pico_check.py`](scripts/minimal_pico_check.py) — Isaac Sim 의존성 없이 xrobotoolkit_sdk 만 단독 검증
- [`scripts/diagnose_xrobotoolkit.py`](scripts/diagnose_xrobotoolkit.py) — L1 (process) / L2 (port) / L3 (SDK init) / L4 (live data) layered probe
- [`tests/test_coord_transforms_xr.py`](tests/test_coord_transforms_xr.py) — 11 케이스 (axis/quat/array/round-trip/error)
- [`tests/test_xrobo_sampler.py`](tests/test_xrobo_sampler.py) — 8 케이스 (fake xrt module 로 hardware-free)
- [`config/xrobotoolkit_settings.json`](config/xrobotoolkit_settings.json) — 참고용 환경 설정
- [`XROBOTOOLKIT_EXECUTION_GUIDE.md`](XROBOTOOLKIT_EXECUTION_GUIDE.md) — 사용자용 실행 가이드 (MSVC 설치부터 §16 체크리스트까지)

#### 수정 컴포넌트

- [`teleop/gr1t2_gripper_device.py`](teleop/gr1t2_gripper_device.py) — `input_backend` / `xrt_enable_body` / `xrt_enable_hand` cfg, `start()` / `_read_action_inputs()` backend 분기, `_probe_action_values_xrt()`
- [`scripts/run_teleop.py`](scripts/run_teleop.py) — `--input_backend` / `--xrt_enable_body` / `--xrt_enable_hand` CLI, RoboticsServiceProcess / Pico Connect pre-flight 진단, device cfg 전달
- [`claude.md`](claude.md) §2.8 / §3.13 추가

#### 좌표계 설계 결정

research/47 §7 의 `R_XR2IL = [[0,0,1],[-1,0,0],[0,1,0]]` 은 **det = -1 (improper rotation)** 이라 두 right-handed 좌표계 사이의 변환으로 부적합:
- 위치 transform 만 보면 동작하지만 (XR (0,0,1) → IL (1,0,0)),
- quaternion conjugation `q_frame * q_dev * q_frame^-1` 는 proper rotation 만 표현 가능한 quat 로 변환할 때 R_XR2IL 과 **다른 행렬** 을 반환 → quat 결과가 geometry 와 불일치 (test fail 로 노출).

수정: OpenXR LOCAL Khronos 사양 (`+Z 가 user-facing back`) 채택 → `R_XR2IL = [[0,0,-1],[-1,0,0],[0,1,0]]` (det=+1). 이때:
- XR `+X` (right) → IL `-Y` (right)
- XR `+Y` (up) → IL `+Z` (up)
- XR `+Z` (back) → IL `-X` (backward)
- quat conjugation 결과가 geometry 와 일치 (90° about XR +X → 90° about IL -Y, 예상대로)

#### 검증 결과

- `test_coord_transforms_xr` 11/11 + `test_xrobo_sampler` 8/8 + 기존 43 = **62/62 PASS**
- `run_teleop --input_backend openvr` (default) 회귀 5-step monitor: `reached --steps=5` + FATAL 없음
- `run_teleop --input_backend xrobotoolkit` (no service): pre-flight WARN + actionable RuntimeError
- `diagnose_xrobotoolkit --skip_live` (no service): L1 FAIL with start command
- `minimal_pico_check` (no SDK): import FATAL with build instructions

#### 사용자 PC 환경 (2026-05-15 시점)

- 11th 세션 시작 전:
  - `XRoboToolkit-PC-Service.win/` (pre-built v1.0.0) — `runService.bat` + `SDK/x64/` 준비됨
  - `XRoboToolkit-PC-Service/` (source) — Pybind 빌드 시 header / nlohmann 추출용
  - Qt 6.11.1 (`C:\Qt\6.11.1\`), Inno Setup 6.7.1 설치됨
  - Unity Client APK 헤드셋에 sideload 완료 (구 우분투 환경에서 adb install 잔재)
  - **MSVC Build Tools 미설치** (`where cl` 결과 없음)
- 11th 세션 완료 후:
  - `XRoboToolkit-PC-Service-Pybind/` clone + `include/` `lib/` 준비됨
  - conda `ust` env 에 `pybind11 3.0.4` 추가
  - **남은 사용자 액션**: `setup_windows.bat` 한 줄 (MSVC 설치 + activate 후)

#### 차후 확장 후보

- Phase B (video streaming 공존) — XROBOTOOLKIT_EXECUTION_GUIDE §10
- Phase C (5-tracker body mocap) — XROBOTOOLKIT_EXECUTION_GUIDE §11
- `config/xrobotoolkit_settings.json` 의 runtime 로딩 hook 추가 (현재 default 만 코드 내장)

---

### 2026-05-13 (10th session) — PCVR 실연 시 PICO grip → 그리퍼 close 작동 안 함, 6 가지 우회 경로 차단 확정

9th 세션의 lead-only PD 재설계 후 사용자가 PICO Connect + SteamVR 로 PCVR 연결해 실제 텔레오퍼레이션을 시도. 그리퍼가 **전혀 닫히지 않음**. SteamVR Test Controller 패널에선 grip 입력 보임. 6 단계의 우회 경로를 차례로 검증해 모든 자동화된 controller input 채널이 차단됨을 확정.

#### 진단 toolkit (6 신규 스크립트, 재사용 가능)

| 스크립트 | 검증 대상 | 결과 |
|---|---|---|
| [scripts/diagnose_controller_properties.py](scripts/diagnose_controller_properties.py) | OpenVR Property API (Float/Bool/Int32 sweep 1000-21000) | PICO 는 Property API 미사용 — 12 s × 변화 0 |
| [scripts/diagnose_openxr_hand.py](scripts/diagnose_openxr_hand.py) | Isaac Lab `OpenXRDevice` + `GripperRetargeter` (hand-tracking) | hand_tracking 데이터 0 도달 (사용자 hand-tracking OFF 이므로 정상) |
| [scripts/diagnose_pyopenxr_probe.py](scripts/diagnose_pyopenxr_probe.py) | pyopenxr `enumerate_instance_extension_properties` | SteamVR/OpenXR 2.15.6 이 `XR_EXT_hand_tracking` + `XR_MND_headless` 모두 지원 (GREEN) |
| [scripts/diagnose_pyopenxr_session.py](scripts/diagnose_pyopenxr_session.py) | pyopenxr headless session + `xrLocateHandJointsEXT` | hand_tracker_ext 까지 생성 OK, locate 시 `is_active=False` (PICO hand-tracking OFF 일치) |
| [scripts/diagnose_pyopenxr_controller.py](scripts/diagnose_pyopenxr_controller.py) | pyopenxr Action API + controller suggested binding | `interaction_profile=/interaction_profiles/oculus/touch_controller` 가 PICO 4 컨트롤러에 매핑, binding accepted, 그러나 headless session 이 FOCUSED 도달 못 함 → `is_active=False` |
| [scripts/diagnose_pyopenxr_piggyback.py](scripts/diagnose_pyopenxr_piggyback.py) | Isaac Sim XR boot + same-process pyopenxr secondary instance | 우리 instance 가 SYNCHRONIZED → VISIBLE → **FOCUSED 도달 확인**, interaction profile 적용됨, 그러나 60s 내내 `L_trig=0.00(0) … focused=True` — **SteamVR 가 secondary instance 에 input 라우팅 안 함** |

#### 6 path 차단 사슬

```
PICO controller (hardware)
  ├─ Path 1: SteamVR Action API (Personal Binding)
  │   └─> "Replace Default Binding" 클릭이 silently CurrentURL_steamvrinput 키 reset
  │       → bActive=False 영구
  │
  ├─ Path 2: OpenVR Property API
  │   └─> PICO driver 가 Property API 미사용 (12 s prop sweep 변화 0)
  │
  ├─ Path 3: Isaac Lab OpenXRDevice (omni.kit.xr.core, hand_tracking)
  │   └─> 사용자 hand_tracking OFF + omni.kit.xr.core 가 extension enable 안 함
  │
  ├─ Path 4: pyopenxr headless session (hand_tracking)
  │   └─> hand_tracker_ext OK, 그러나 사용자 hand_tracking OFF 이므로 데이터 없음
  │
  ├─ Path 5: pyopenxr headless session (controller Action API)
  │   └─> session 이 FOCUSED 도달 못 함 → is_active=False
  │
  └─ Path 6: pyopenxr piggyback (Isaac Sim same-process secondary instance)
      └─> session FOCUSED 도달 OK, interaction profile 적용됨,
          그러나 SteamVR multi-instance routing 정책: input 은 primary instance 만
```

#### 근본 원인 (확정)

**PICO Connect 10.6.6 + PUI 5.15.4 + SteamVR/OpenXR 2.15.6 환경에서 third-party 앱이 PICO 컨트롤러의 trigger/grip analog value 를 자동화된 방법으로 받을 수 있는 경로가 없음**:

1. PICO 의 `pico` driver 는 controller state 를 **SteamVR Action System 내부 채널** (`IVRDriverInput::UpdateScalarComponent`) 로만 push.
2. 이 채널은 SteamVR Personal Binding 으로 commit 된 앱만 read 가능.
3. 우리 앱 `ust.teleop.gr1t2_gripper` 의 Personal Binding commit 이 SteamVR UI bug 로 silently 실패 (CurrentURL key reset).
4. OpenXR layer 우회 시도: SteamVR runtime 의 OpenXR backend 도 같은 Action System 사용 + multi-instance input routing 미지원.
5. PICO 측 PicoStreamingXR runtime 도 omni.kit.xr.core 와 호환 안 됨.

#### Workshop bindings 의 미스터리

vrserver.txt 의 binding load 로그는 *우리 disk binding 을 성공적으로 load* 했다고 보고:
```
[Input] ust.teleop.gr1t2_gripper (pico_controller) attempting to load default config from
        file:///C:/develop/IsaacLab/ust_ws/ust_hm_grip/config/openvr_actions/bindings_pico_controller.json
[Workshop] Successfully loaded binding file '...bindings_pico_controller.json' for app 'ust.teleop.gr1t2_gripper'.
```

그러나 binding *parse* 와 binding *commit* 은 별개. SteamVR 는 Personal Binding commit 단계에서 silently fail, 그래서 모든 action handle `bActive=False`. SteamVR 의 알려진 UI bug 또는 PICO 4 Ultra + PICO Connect 10.6.6 의 회귀.

#### 차선책 (구현 안 함, 사용자 결정 대기)

| 옵션 | 비용 | 작동 보장 |
|---|---|---|
| 🥇 키보드 fallback (`C`/`V` 키 = 좌/우 close) | 1-2 h | 100% |
| 🥈 ALVR 로 PCVR streaming 솔루션 교체 | 4-6 h setup | 높음 (다른 binding pipeline) |
| 🥉 PICO 공식 지원 + Virtual Desktop 시도 | 사용자 시간 | 미보장 |

#### 환경 변경 사항 (정리됨)

- pyopenxr 1.1.5301 conda `ust` env 설치 (향후 OpenXR 작업 재활용 가능)
- HKLM\SOFTWARE\Khronos\OpenXR\1\ActiveRuntime: PICO 로 switch 후 SteamVR 로 복원 완료
- AvailableRuntimes 에 PicoStreamingXR 등록 (registry 잔재)
- `steamvr.vrsettings.bak.*` 다수 (각 단계 백업)
- `swift.ini` + PICO Connect `settings.json` 원본 보존

#### 재검증 trigger (향후 추적)

이 toolkit 으로 즉시 재검증 가능한 경우:
- PICO Connect 11.x 출시
- SteamVR 2.16+ 출시 (Personal Binding commit UI fix)
- PUI 5.16+ 또는 6.x 출시
- ALVR 로 streaming 솔루션 교체
- 다른 헤드셋 (Quest, Vive, Index) 로 변경

#### 산출 파일 변경 (2026-05-13 10th session)

신규
- `scripts/diagnose_controller_properties.py`
- `scripts/diagnose_openxr_hand.py`
- `scripts/diagnose_pyopenxr_probe.py`
- `scripts/diagnose_pyopenxr_session.py`
- `scripts/diagnose_pyopenxr_controller.py`
- `scripts/diagnose_pyopenxr_piggyback.py`

수정
- (없음 — 모든 변경은 환경 측. 코드 변경은 차선책 결정 후 진행)

---

### 2026-05-13 (9th session) — PICO grip → 그리퍼 닫힘 E2E 검증 + 4-bar linkage PD 불안정성 발견 + lead-only 드라이브로 재설계

사용자 요청: "PICO 컨트롤러의 그립을 당기는 입력을 아이작 랩의 로봇이 받아서 그리퍼가 닫히는 기능". 이전 8 세션은 USD 빌드 / 시각 / 자세까지만 검증했고, 실제 close 동작 (lead joint 가 +0.785 rad 에 도달하는지) 은 한 번도 측정한 적 없음. 신규 E2E 테스트 `scripts/test_robotiq_close.py` 작성 → 1~2초 단위 settle 후에 lead 가 close target 에 도달했는지 + followers 가 mimic gearing 으로 따라가는지 검증.

#### 정적 검증 (사전 PASS)

- `tests/test_gripper_retargeter.py` 22/22 + `tests/test_action_manifest.py` 21/21 = **43/43 PASS** (PICO grip ≥ 0.6 → action[14]=-1 hysteresis, 좌/우 독립, deadband, 6 binding 파일 controller_type 매핑).
- `scripts/verify_all_visuals.py` 18/18 OK (Robotiq 메시), `scripts/verify_wrist_joints.py` body0/body1 보존, `scripts/diagnose_robotiq_attach.py` 6 joint drive + mass + 대칭 limit + mimic refJoint 모두 정상.
- Monitor mode (`run_teleop --diag idle --steps 5`): Action Manager 3 active terms (`pink_ik_cfg=14, left_gripper_action=1, right_gripper_action=1`), FATAL/Traceback/negative mass 경고 없음, `reached --steps=5`.

이 시점에서 wiring 은 완벽: `processed_actions` 까지 close target 이 정확히 도달함.

#### 신규 E2E 테스트 `test_robotiq_close.py` — close 가 실제로 일어나는지 측정

3-phase test:
1. **Phase 1** idle (action[14,15]=+1, OPEN), 60 step settle. 기대: 12 joint 전부 ±10°.
2. **Phase 2** both close (action[14,15]=-1, -1), 120 step settle. 기대: lead 가 +45°, followers 가 gear × +45° ± tol.
3. **Phase 3** asymmetric (action[14]=-1 close, action[15]=+1 open). 좌/우 독립성 검증.

#### Issue C-1 — 원본 PD (K=10 D=80) 로는 close target 에 못 도달 (τ=8초)

진단 diag log:
```
[diag step 0] raw L=[-1.0] processed_L=[0.785, -0.785, 0.785, -0.785, 0.785, 0.785]
              L_finger_joint pos=+0.27deg target=+44.98deg  ← target은 정확
[diag step 30] L_finger_joint pos=-0.15deg target=+44.98deg ← 30 step (0.25s) 후에도 안 움직임
```

원인: `ImplicitActuatorCfg(stiffness=10, damping=80)` → 슬로우 폴 rate `K/D = 0.125` rad/s → 시상수 `τ = 8초`. 120 step (1초) 으로는 close target 의 12% 만 진행. 실제 텔레오퍼레이션에서도 사용자가 grip 을 8초 동안 잡고 있어야 그리퍼가 닫히는 셈이라 실용 불가.

8 세션에서 측정한 적이 없었던 이유: `test_robotiq_pose.py` 는 **idle (open) 명령 만** 60 step 검증해서, 이미 0 위치인 joint 가 0 근처에 머무는 것만 확인했음. close 동작은 미검증.

#### Issue C-2 — 6-joint 동시 driving 이 closed-loop 4-bar linkage 에서 PhysX 솔버 발산

K=200 D=20 (τ=0.1s 목표):
```
left_finger_joint = +48° ✓ (lead reached target)
left_left_inner_finger_joint = +124° ✗ (follower overshoot)
right_finger_joint = -27° ✗ (wrong direction!)
right_right_inner_finger_joint = -60° ✗ (follower overshoot)
```

K=200 D=80 (overdamped 추가):
```
left_left_inner_finger_joint = -923° (~2.5 revolutions!)
right_finger_joint = -763°
```

K=50 D=5 (Robotiq 원본 USD 드라이브 값):
```
left_left_inner_finger_joint = +628°
right_finger_joint = -763°
```

**원인 분석**: Robotiq 2F-85 는 closed 4-bar linkage. 6 joint 모두에 독립 PD 타깃 (gear × +0.785) 을 부여하면 — 각 joint 의 PD 타깃이 **linkage 의 매 step kinematic 해 (closed-loop solution)** 와 정확히 일치하지 않는 한 — 솔버가 transient 단계에서 누적 오차를 발생시킴. 누적 오차가 mimic constraint + linkage 의 hard constraint 와 부딪쳐 발산.

설계 가이드 #43 §6.6 의 "5.1 mimic known issue 우회 = 6 joint 전부 drive" fallback 은 **OPEN 상태 (target=0) 에서만 안정적**. CLOSE (gear×non-zero target) 에서는 발산.

#### Fix — lead-only 드라이브 아키텍처 (Robotiq stock USD 의 원래 의도)

[`kitchen_sorting_gr1t2_gripper_env_cfg.py`](kitchen_sorting_gr1t2_gripper_env_cfg.py) 의 actuator + binary action 재설계:

```python
# 강한 lead 드라이브 — close 동작의 활성 소스
actuators["robotiq-{side}-lead"] = ImplicitActuatorCfg(
    joint_names_expr=[ROBOTIQ_LEAD_JOINT_{SIDE}],   # left_finger_joint or right_finger_joint
    effort_limit_sim=50.0, velocity_limit_sim=5.0,
    stiffness=200.0, damping=20.0,                  # τ = K/D = 10 rad/s → 0.1s
)

# 패시브 follower 드라이브 — K=0 이면 PhysX 는 위치 타깃을 enforce 안 함
actuators["robotiq-{side}-followers"] = ImplicitActuatorCfg(
    joint_names_expr=[j for j in ROBOTIQ_ALL_JOINTS_{SIDE} if j != ROBOTIQ_LEAD_JOINT_{SIDE}],
    effort_limit_sim=10.0, velocity_limit_sim=5.0,
    stiffness=0.0, damping=2.0,                      # 진동 흡수만, 타깃 없음
)

# BinaryJointPositionAction 도 lead 1 개로 축소
left_gripper_action = BinaryJointPositionActionCfg(
    joint_names=[ROBOTIQ_LEAD_JOINT_LEFT],
    open_command_expr ={ROBOTIQ_LEAD_JOINT_LEFT: 0.0},
    close_command_expr={ROBOTIQ_LEAD_JOINT_LEFT: ROBOTIQ_CLOSE_RAD},
)
```

핵심 아이디어:
- **Lead** 만 강하게 driving (K=200) → 0.1s 만에 +0.785 rad 도달.
- **Followers** 는 K=0 이라 PhysX 가 위치 enforce 안 함 → 4-bar linkage 의 기계적 구속 + PhysxMimicJointAPI 가 자동 coordinate. 작은 damping (D=2) 은 솔버 안정성용.
- Action Manager 의 외부 형상은 그대로 (`pink_ik=14, L_grip=1, R_grip=1`, 16-D action). 따라서 retargeter 및 PICO device 코드 무수정.

이는 **Robotiq stock USD 의 원래 설계 의도** 와 일치 — stock 은 lead 에만 드라이브를 두고 mimic 으로 followers 를 끌었음 (5.1 known issue 때문에 우리가 6 joint drive 로 fallback 했으나, CLOSE 동작에서 발산하는 부작용).

#### 향후 검증 필요 (다음 세션)

- [ ] `test_robotiq_close.py` 가 새 아키텍처에서 PASS 하는지 — Phase 1 (open 60 step), Phase 2 (close 240 step = 5τ), Phase 3 (asym 240 step). 9th 세션은 사용자 요청으로 Isaac Sim 부팅 시간 (30-60s) 인 백그라운드 검증을 중단했음.
- [ ] `test_robotiq_pose.py` regression — 새 PD 가 OPEN 상태에서도 ±10° tol 유지하는지.
- [ ] Monitor mode smoke — Action Manager 가 여전히 3 active terms 인지.
- [ ] GUI 시각 확인 — 사용자 환경에서 close 명령 시 그리퍼가 실제로 닫히는지.

#### 산출 파일 변경 (2026-05-13 9th session)

수정
- [kitchen_sorting_gr1t2_gripper_env_cfg.py](kitchen_sorting_gr1t2_gripper_env_cfg.py) — `_gripper_robot_articulation()` 의 actuators dict 에 4-key 아키텍처 (lead/followers × L/R), `GripperActionsCfg.left_gripper_action` + `.right_gripper_action` 의 `joint_names` / `open_command_expr` / `close_command_expr` 를 lead 1 개로 축소.

신규
- [scripts/test_robotiq_close.py](scripts/test_robotiq_close.py) — 3-phase E2E close 검증 + diag 출력 (raw_actions, processed_actions, joint_pos_target).

---

### 2026-05-10 — Pink IK FrameNotFound + Pink IK 차원 + IPC + UI freeze + URDF 캐시

이 세션 시작 시점에 GR1T2 + 자작 2-finger 그리퍼 (옵션 B) USD 가 빌드돼 있었으나 첫 텔레오퍼레이션 실행이 **`pink.exceptions.FrameNotFound: "left_wrist_pitch_link"`** 로 죽었다. 그 후 연쇄적인 5 개의 issue 를 한 세션에서 진단/수정.

#### Issue 1 — Pink IK FrameNotFound (env_cfg 의 잘못된 link 이름)

- **증상**: env 생성 시 `ActionManager.__init__` → `PinkIKController.__init__` 에서 `FrameNotFound`. GR1T2 의 실제 link 이름은 `*_hand_pitch_link` (joint 만 `*_wrist_pitch_joint`).
- **Fix**: [kitchen_sorting_gr1t2_gripper_env_cfg.py:336-422](kitchen_sorting_gr1t2_gripper_env_cfg.py#L336)
  - `target_eef_link_names`: `*_wrist_pitch_link` → `*_hand_pitch_link`
  - `FrameTask` 첫 인자: `*_wrist_pitch_link` → `GR1T2_fourier_hand_6dof_*_hand_pitch_link` (Pinocchio prefix)
  - `NullSpacePostureTask.controlled_frames` 동일
- **회귀 방지**: claude.md §3.1 + §3.2 dual naming rule 명시.

#### Issue 2 — SteamVR IPC namespace 충돌

- **증상**: `openvr.init()` 실패, `openvr.error_code.InitError_IPC_NamespaceUnavailable`.
- **원인 진단**: Oculus runtime (OVRServer_x64) 동시 실행 + stale vrserver IPC handle (이전 client crash 후 회수 안 됨).
- **Fix**:
  - [vr_sampler.py:191-249](teleop/vr_sampler.py#L191) — `openvr.init()` try/except + psutil 동적 진단 + 정확한 시나리오별 안내 메시지.
  - [scripts/cleanup_vr_env.py](scripts/cleanup_vr_env.py) 신규 — 관리자 PowerShell 에서 Oculus service disable + SteamVR fresh restart.
- **회귀 방지**: claude.md §3.7.

#### Issue 3 — URDF 변환 6.7 분 fake-hang

- **증상**: `[run_teleop] env_cfg = ...` 출력 후 사용자가 6.7 분간 무응답으로 인식 → Ctrl+C → 그제야 진행. 실제로는 USD→URDF 변환이 30-90 초 걸리는데 PowerShell stdout buffering 으로 보이지 않았음.
- **Fix**:
  - [kitchen_sorting_gr1t2_gripper_env_cfg.py:`__post_init__`](kitchen_sorting_gr1t2_gripper_env_cfg.py) — USD mtime > URDF mtime 일 때만 재변환. `force_conversion=True` 폐기.
  - [run_teleop.py 진입부](scripts/run_teleop.py#L29) — `sys.stdout.reconfigure(line_buffering=True)` 강제.
  - 변환 전후로 명시적 `print(..., flush=True)` 진행 메시지.
- **회귀 방지**: claude.md §3.5.

#### Issue 4 — Pink IK `hand_joint_dim=0` 슬라이싱 버그 (silent shutdown)

- **증상**: 첫 `env.step()` 후 `Simulation App Shutting Down` (~3 초). traceback 없음. main loop 의 `except KeyboardInterrupt` 만 잡아서 silent.
- **Root cause**: isaaclab core 의 [`pink_task_space_actions.py:200`](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab/isaaclab/envs/mdp/actions/pink_task_space_actions.py#L200) 가 `actions[:, -self.hand_joint_dim:]` 로 슬라이싱. `hand_joint_dim=0` 이면 Python 룰상 `-0:` 이 전체 텐서 → cat 결과가 (1, 14 + 17) = (1, 31) → set_joint_position_target([1,17]) 에 broadcast 실패 → `RuntimeError: shape mismatch [31] vs [1, 17]`.
- **Fix**:
  - [run_teleop.py main loop](scripts/run_teleop.py#L470-555) — `except BaseException` 확장 + finally `exit_reason` print. silent shutdown 영구 차단.
  - [teleop/_pink_hand_dim_zero_patch.py](teleop/_pink_hand_dim_zero_patch.py) 신규 — idempotent monkey-patch. `hand_joint_dim == 0` 일 때 `actions[:, 0:0]` 빈 슬라이스.
  - env_cfg 와 run_teleop 두 곳에서 `_pink_hand_dim_zero_patch.apply()` 호출.
  - 동반: env_cfg `__post_init__` 에서 `pink_controlled_joint_names` 를 매번 재할당해 누적 차단 (configclass mutable default 패턴).
- **회귀 방지**: claude.md §3.3 + §3.4 + §3.6.

#### Issue 5 — Isaac Sim window UI 동결 (HIGH-priority + 120 Hz)

- **증상**: IK 는 4500+ step 안정 동작 중인데 Isaac Sim 창 클릭/이동 불가능. 메뉴 선택 안 됨.
- **원인**: `--process_priority high` + `decimation=1` + `render_interval=1` + `sim.dt=1/120` 조합으로 main thread 가 한 코어 거의 100% 점유 → Windows input thread starve. `sim.render()` 내부의 `_app.update()` 는 호출되지만 OS 가 mouse event 를 process 로 전달 못 함.
- **Fix**: [run_teleop.py main loop](scripts/run_teleop.py) 끝에 두 줄 추가
  - `_time.sleep(0)` — OS scheduler yield, jitter 영향 거의 없음.
  - 16 frame 마다 `simulation_app.update()` — 추가 UI message pump.
- **회귀 방지**: claude.md §3.6.

---

### 2026-05-13 (8th session) — hand_pitch_link 의 GR1T2 손 visual+collision 제거

7th session 의 회전 fix 후 사용자가 GUI 스크린샷에서 그리퍼 fingerpad 사이에 metal 손 모양 mesh 가 끼어 있는 것을 발견. 그리퍼 회전에 영향이 없으면 제거, 있으면 간섭만 회피 옵션 요청.

#### 진단 (scripts/inspect_hand_pitch_link.py 신규)

`{side}_hand_pitch_link` 의 구조:
- 자체 APIs: `[PhysicsRigidBodyAPI, PhysicsMassAPI]` ← 강체, 질량 보유 (articulation 노드)
- 참조하는 joint:
  - `{side}_wrist_pitch_joint` (body1) — wrist_pitch 가 여기서 끝남
  - `{side}_robotiq_attach_fixed_joint` (body0) — Robotiq 가 여기서 시작
- 자식 prim: `end_effector_link` (frame Xform), `visuals` (메시), `collisions` (메시)

→ **결론**: body 자체는 articulation 의 필수 노드 (wrist_pitch ↔ Robotiq 의 anchor). 그러나 `visuals` 와 `collisions` 자식 prim 은 *순수 geometry* — kinematic chain 에 영향 없음. 손 회전은 wrist_yaw/roll/pitch joint 값으로 결정되며, 이 joint 들은 link 의 *visual* 이나 *collision* 메시와 무관.

#### Fix — visuals + collisions 만 제거, body + end_effector_link 유지

[isaac_file/build_robotiq_usd.py:_strip_hand_pitch_link_geometry](isaac_file/build_robotiq_usd.py) 신규 함수:
```python
for child_name in ("visuals", "collisions"):
    child_path = wrist_path.AppendChild(child_name)
    if stage.GetPrimAtPath(child_path):
        stage.RemovePrim(child_path)
```

`_attach_robotiq(side)` 의 step 0 으로 호출 — Robotiq 부착 전에 wrist 의 손 mesh 비움. Robotiq 의 base_link mesh (~9 cm × 8 cm) 가 같은 위치에 부착되므로 visual + collision 모두 Robotiq 이 cover.

지키는 부분:
- `hand_pitch_link` 본체의 RigidBodyAPI + MassAPI (articulation 노드 자격)
- `end_effector_link` 자식 (frame reference Xform)
- 모든 joint relationship (wrist_pitch / attach 가 hand_pitch_link 를 body 로 그대로 참조)

#### 검증

[scripts/inspect_hand_pitch_link.py](scripts/inspect_hand_pitch_link.py) (rebuild 후):
```
left_hand_pitch_link:
  applied APIs : [PhysicsRigidBodyAPI, PhysicsMassAPI]   ← 변함없음
  children     : [end_effector_link]                      ← visuals + collisions 제거됨
right_hand_pitch_link: 동일
```

[scripts/verify_wrist_joints.py](scripts/verify_wrist_joints.py) (신규, joint 보존 확인):
```
/.../joints/left_wrist_pitch_joint
  body0: left_hand_roll_link, body1: left_hand_pitch_link        ← OK
/.../left_robotiq_attach_fixed_joint
  body0: left_hand_pitch_link, body1: left_robotiq_arg2f_85/base_link  ← OK
```

pose test 60 step: 12 joint 전부 ±10° 이내, max drift +2.60° on right_left_inner_finger_knuckle_joint. 7th session 결과와 거의 동일 — body 와 joint 가 그대로 유지되므로 물리 거동 변화 없음.

smoke test (5 step): `reached --steps=5`, FATAL/Traceback/negative mass 경고 없음.

#### 산출 파일 변경 (2026-05-13 8th session)

수정
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py) — `_strip_hand_pitch_link_geometry(stage, side)` 신규, `_attach_robotiq` step 0 에서 호출

신규
- [scripts/inspect_hand_pitch_link.py](scripts/inspect_hand_pitch_link.py) — hand_pitch_link 의 APIs + children + 참조 joint dump
- [scripts/verify_wrist_joints.py](scripts/verify_wrist_joints.py) — strip 후 wrist_pitch + attach 의 body refs 보존 검사

---

### 2026-05-13 (7th session) — 월드 Y축 180° 회전 적용

6th session 의 identity baseline 으로 그리퍼의 native 방향을 확인한 후 사용자가 "월드 Y축 180° 회전" 을 요청. GR1T2 wrist 의 world rotation 이 bind pose 에서 identity 이므로 wrist 의 local frame 에 `Ry(180°)` 를 적용하면 world Y 회전과 동일.

#### 변경

[isaac_file/build_robotiq_usd.py:_wrist_to_gripper_rotation](isaac_file/build_robotiq_usd.py):
```python
return Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), 180.0)
```

Axis mapping (gripper local → wrist local):
- gripper +X → wrist -X (forward → backward)
- gripper +Y → wrist +Y (unchanged, rotation axis)
- gripper +Z → wrist -Z (up → **down**)

#### 검증

[scripts/verify_gripper_world_pos.py](scripts/verify_gripper_world_pos.py):
```
[LEFT]  TCP world: (+0.003, +0.229, -0.235)  fingertip dir: (0, 0, -0.15)
[RIGHT] TCP world: (+0.003, -0.229, -0.235)  fingertip dir: (0, 0, -0.15)
container rot row0=(-1,0,0) row1=(0,+1,0) row2=(0,0,-1)  ← Ry(180°) matrix ✓
```

양쪽 그리퍼 모두 fingertip-out 이 world -Z (down) — 손바닥 아래로 향하는 자세. 4th session 의 inspect_wrist_frame 결과와 일치 (원본 Fourier 손도 fingers 가 -Z 방향).

URDF cache 강제 삭제 후 rebuild → cache miss → 새 URDF 생성 (Pink IK 가 최신 USD 의 회전 반영).

pose test 60 step: 12 joint 전부 ±10° 이내, max drift +2.50° on right_left_inner_finger_knuckle_joint. 6th session (±6.66°) 대비 개선 — fingertips-down orientation 에서 gravity 가 outer_knuckle 의 회전축에 평행하지 않아 PD steady-state 가 더 깨끗.

smoke test (5 step): 통과.

#### 산출 파일 변경 (2026-05-13 7th session)

수정
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py) — `_wrist_to_gripper_rotation` 가 `Gf.Rotation(Gf.Vec3d(0,1,0), 180)` 반환

---

### 2026-05-13 (6th session) — 회전값 전부 초기화 (identity baseline)

5th session 의 yaw 추가 후에도 사용자가 visual 변화를 인지하지 못함. URDF cache hit 으로 인해 변경된 USD 가 Pink IK 에 반영 안 됐을 가능성. 사용자 요청에 따라 그리퍼 회전을 *전부 identity 로 리셋* — 모든 추측을 제거하고 clean baseline 으로 돌아감.

#### 변경

[isaac_file/build_robotiq_usd.py:_wrist_to_gripper_rotation](isaac_file/build_robotiq_usd.py) 가 이제 무조건 `Gf.Rotation(Gf.Vec3d(1,0,0), 0.0)` (= identity) 반환. 호출 측 (`_attach_robotiq`) 은 그대로 — container.world 의 rotation 부분이 identity matrix, FixedJoint.localRot0 = identity quat, TCP world position = wrist + (0, 0, +0.15) (Robotiq 의 local +Z 방향).

URDF cache (`%TEMP%\urdf\GR1T2_with_robotiq.urdf`) 를 명시적으로 삭제하고 빌드 → run_teleop 이 cache miss 로 새 URDF 생성. **이전 세션에서 visual 이 같아 보였던 이유 = URDF cache 가 stale 이었을 가능성**. design-guide #43 의 env_cfg mtime check 가 정상 작동하지만 6 분 전 빌드 후 즉시 재실행하면 URDF mtime 이 USD mtime 직후에 작성되어 cache hit. 변경 후 강제 invalidation 안전.

#### 검증

[scripts/verify_gripper_world_pos.py](scripts/verify_gripper_world_pos.py):
```
[LEFT]  container rot = identity, fingertip dir = (0, 0, +0.15)  ← gripper +Z = world +Z (up)
[RIGHT] container rot = identity, fingertip dir = (0, 0, +0.15)  ← 동일
TCP world LEFT  = (0.003, +0.229, +0.065)  = wrist + (0, 0, +0.15)
TCP world RIGHT = (0.003, -0.229, +0.065)
```

pose test 60 step: 12 joint 전부 ±10° 이내, max drift +6.66° on right_left_inner_finger_knuckle_joint. 5th session (±1°) 대비 큼 — identity orientation 에선 gravity 가 base_link 의 local frame 에 직접 작용하므로 PD 가 더 큰 torque 와 싸워야 함. 그래도 chain-like deformation (30-90° drift) 와는 거리가 멀어 visually OK.

smoke test (5 step): `reached --steps=5`, FATAL/Traceback/negative mass 경고 없음.

#### 산출 파일 변경 (2026-05-13 6th session)

수정
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py) — `_wrist_to_gripper_rotation` 가 무조건 identity 반환. 호출 인프라 (container.SetRotate, FixedJoint.localRot0, TCP world transform) 그대로 유지 — 추후 다른 rotation 필요 시 함수만 교체.

---

### 2026-05-13 (5th session) — 그리퍼 yaw 추가 (LEFT +90°, RIGHT -90°)

4th session 의 visual mesh fix 후에도 사용자 스크린샷에서 그리퍼가 "90° 옆으로 꺽인" 모양 — Robotiq 메시는 올바르지만 회전이 한 축 더 필요. 4th session 의 `Rx(±90°)` per-side rotation 은 fingertip 방향 (gripper +Z) 을 arm 연장 방향으로 align 시키지만, grasp axis (gripper +Y, 두 outer_knuckle 가 spread 하는 방향) 가 wrist 의 ±Z (= world ±Z = up-down) 으로 떨어져 그리퍼가 "옆으로 누운" 형태.

#### Fix — `Rz(±90°)` yaw 를 gripper local frame 에서 추가

[isaac_file/build_robotiq_usd.py:_wrist_to_gripper_rotation](isaac_file/build_robotiq_usd.py) 가 이제 두 단계 회전을 compose:
```python
if side == "left":
    r_position = Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)  # fingertip-out
    r_yaw      = Gf.Rotation(Gf.Vec3d(0, 0, 1), +90)  # gripper local Z 기준
elif side == "right":
    r_position = Gf.Rotation(Gf.Vec3d(1, 0, 0), +90)
    r_yaw      = Gf.Rotation(Gf.Vec3d(0, 0, 1), -90)
composed = (Matrix4d.SetRotate(r_yaw)) * (Matrix4d.SetRotate(r_position))  # row-vector: yaw first (in local), then position
```

USD row-vector convention 이므로 `M1 * M2 = M1 먼저 적용`. 따라서 `yaw * position` 은 *yaw 를 gripper local 에서 적용한 뒤* position 으로 wrist frame 까지 변환 → fingertip 방향 (gripper +Z) 은 변하지 않고 yaw 만큼 grasp axis 가 돌아감.

결과 axis mapping (gripper local → wrist local):
- LEFT  (Rx(-90°) ∘ Rz(+90°)): +X → -Z, +Y → -X, +Z → +Y
- RIGHT (Rx(+90°) ∘ Rz(-90°)): +X → -Z, +Y → +X, +Z → -Y

- Fingertip-out (gripper +Z) 은 여전히 arm 연장 방향 (LEFT +Y, RIGHT -Y) — 4th session 의 alignment 보존.
- Grasp axis (gripper +Y) 가 wrist 의 ±X 방향 (forward/backward) 으로 옴 — 두 finger 가 수평으로 마주봄.

#### 검증

[scripts/verify_gripper_world_pos.py](scripts/verify_gripper_world_pos.py):
```
[LEFT]  TCP world: (0.003, +0.379, -0.085)  fingertip dir: (0, +0.15, 0)  ← +Y 그대로
[RIGHT] TCP world: (0.003, -0.379, -0.085)  fingertip dir: (0, -0.15, 0)  ← -Y 그대로
container rot LEFT  row2=(0, +1, 0)  ← gripper +Z = world +Y ✓
container rot RIGHT row2=(0, -1, 0)  ← gripper +Z = world -Y ✓
```

pose test 60 step settle: 12 joint 전부 ±1° 이내 (max +0.94° on right_left_inner_finger_knuckle_joint). 이전 (3rd session) ±2.8° 대비 개선 — 새 orientation 에서 gravity torque 가 joint 축에 거의 align 되지 않아 PD steady-state 가 더 깨끗.

smoke test (5 step): `reached --steps=5`, FATAL/Traceback/negative mass 경고 없음.

#### 산출 파일 변경 (2026-05-13 5th session)

수정
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py) — `_wrist_to_gripper_rotation` 에 `Rz` yaw step 추가 + Gf.Matrix4d 행렬 합성

---

### 2026-05-13 (4th session) — Robotiq visual mesh 가 GR1T2 thigh 로 보였던 본질적 버그 fix (instancing prototypes leak)

3rd session 후에도 사용자 GUI 스크린샷에서 그리퍼가 **Robotiq 2F-85 모양이 아닌** 다른 메시 (보라색 호스 + 다리 일부 같은 외형) 로 보였다. 사용자가 직접 Stage 패널에서 `right_robotiq_arg2f_85/base_link/visuals/left_thigh_r/mesh` 를 확인 — GR1T2 의 thigh roll link 메시가 그리퍼 자리에 grafted 되어 있었음. 1~3rd session 의 fix 모두 articulation / 물리적 부분이었지, **visual mesh** 자체가 잘못 잡혀 있던 것을 못 봤다.

#### 근본 원인 (scripts/inspect_visuals.py + inspect_stock_meshes.py 신규로 확인)

Robotiq stock USD `Robotiq_2F_85_edit.usd` 의 모든 `*/visuals` prim 이 `instanceable=True` + `references=</Meshes/...>` 로 정의돼 있음. `Stage.Open(stock).Flatten()` 호출 시 USD 가 자동으로 **5 개의 prototype** 을 layer 루트에 생성:
```
/Flattened_Prototype_1  → Defeatured_2F_85_PAD_OPEN_basestep_01 (base_link 메시)
/Flattened_Prototype_2  → ...finger3step (inner_knuckle 메시)
/Flattened_Prototype_3  → ...finger2step (outer_finger 메시)
/Flattened_Prototype_4  → ...Finger1step (outer_knuckle 메시)
/Flattened_Prototype_5  → ...finger4step + fingertipsstep (inner_finger 메시)
```
모든 visuals 가 `references=/Flattened_Prototype_N` 로 변환됨.

빌드 스크립트의 `Sdf.CopySpec(flat, "/Robotiq_2F_85/Robotiq_2F_85", dst_layer, "/{side}_robotiq_arg2f_85")` 는 sub-tree 만 복사 — **루트의 prototype prim 들은 *소스 레이어* 에 남는다**. dst stage 의 visuals 는 `/Flattened_Prototype_N` 를 참조하는데, dst stage 의 `/Flattened_Prototype_N` 는 GR1T2 의 자체 instancing prototype 으로 이미 채워져 있음 (`left_thigh_roll_link/visuals → /Flattened_Prototype_2` 등). 결과: 그리퍼 visual 이 GR1T2 thigh mesh 로 cross-resolve.

#### Fix — flatten 전 un-instance

[isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py):
```python
robotiq_stage = Usd.Stage.Open(robotiq_layer_path)
for p in robotiq_stage.Traverse():
    if p.IsInstanceable():
        p.SetInstanceable(False)
robotiq_flat_layer = robotiq_stage.Flatten()  # prototype 생성 안 됨, mesh 가 visuals 에 inline
```

이렇게 하면 flatten 결과가 prototype 없이 visuals 각각에 mesh 데이터 inline 됨. CopySpec 이 자기 자식 prim 까지 함께 복사한다.

#### 검증 (scripts/verify_all_visuals.py 신규)

```
[OK] base_link/visuals children: ['Defeatured_2F_85_PAD_OPEN_basestep_01']
[OK] left_outer_knuckle/visuals children: ['Defeatured_2F_85_PAD_OPEN_Finger1step_01']
[OK] right_outer_knuckle/visuals children: ['Defeatured_2F_85_PAD_OPEN_Finger1step_01']
[OK] left_outer_finger/visuals children: ['Defeatured_2F_85_PAD_OPEN_finger2step_01']
[OK] right_outer_finger/visuals children: ['Defeatured_2F_85_PAD_OPEN_finger2step_01']
[OK] left_inner_finger/visuals children: ['Defeatured_2F_85_PAD_OPEN_finger4step_01', 'Defeatured_2F_85_PAD_OPEN_fingertipsstep_01']
[OK] right_inner_finger/visuals children: ['...finger4step_01', '...fingertipsstep_01']
[OK] left_inner_knuckle/visuals children: ['Defeatured_2F_85_PAD_OPEN_finger3step_01']
[OK] right_inner_knuckle/visuals children: ['Defeatured_2F_85_PAD_OPEN_finger3step_01']
```
양쪽 gripper 모두 9 개 body × 2 = 18/18 visual 이 실제 Robotiq 메시.

pose test (60 step, idle 액션) 12 joint 전부 ±3° 이내, smoke test (5 step) 통과 — physics 거동은 그대로.

#### 산출 파일 변경 (2026-05-13 4th session)

수정
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py) — Flatten() 전 모든 instanceable prim 을 SetInstanceable(False)

신규
- [scripts/inspect_visuals.py](scripts/inspect_visuals.py) — built/stock USD 의 visuals subtree dump
- [scripts/inspect_stock_meshes.py](scripts/inspect_stock_meshes.py) — Flatten() 후 prototype 구조 + instancing 확인
- [scripts/verify_all_visuals.py](scripts/verify_all_visuals.py) — 9 body × 2 side 의 visuals children 자동 검증
- [~/.claude/.../memory/feedback_usd_flatten_instancing_leak.md](file:///C:/Users/pjwpy/.claude/projects/C--develop-IsaacLab/memory/feedback_usd_flatten_instancing_leak.md)

---

### 2026-05-13 (3rd session) — 그리퍼 mount 방향 fix (per-side rotation)

2nd session 후 사용자 스크린샷에서 그리퍼가 손목에서 **위로 (+Z world)** 향해 튀어나와 있는 모습 — design-guide #43 §7.2 가 경고했던 "GR1T2 wrist 와 Robotiq mount face 의 축 misalign". 2nd session 이 mimic / drive / mass 의 *물리적* 결함은 풀었으나 *기하* 결함이 남아 있었다.

#### 진단 (scripts/inspect_wrist_frame.py 신규)

GR1T2 stock USD 의 wrist link 의 world 방향 + 원본 Fourier 손가락 (`L_thumb_proximal_link` 등) 의 위치를 직접 inspect.

확인:
- 두 wrist 모두 bind pose 에서 world rotation = identity. 즉 wrist local +X/+Y/+Z = world +X/+Y/+Z.
- Fourier 손가락의 wrist 로부터 delta: 주성분이 **-Z** (대략 -0.12 m). 즉 원래 hand 의 fingertip 방향 = wrist 의 local **-Z**.
- 단, "natural industrial mount" 관점에선 그리퍼가 *팔의 연장 방향* 으로 뻗어야 보기 좋다 — LEFT 팔의 wrist 는 world +Y 에 있고 팔이 +Y 방향으로 뻗어 있으니 LEFT gripper fingertip → world +Y. RIGHT 는 -Y.

#### Fix — per-side FixedJoint localRot + container xform

```python
def _wrist_to_gripper_rotation(side: str):
    if side == "left":
        return Gf.Rotation(Gf.Vec3d(1, 0, 0), -90)  # +Z gripper → +Y wrist
    if side == "right":
        return Gf.Rotation(Gf.Vec3d(1, 0, 0), +90)  # +Z gripper → -Y wrist
```

build_robotiq_usd.py 변경:
- `_attach_robotiq(side)` 에서 `gripper_rotation = _wrist_to_gripper_rotation(side)` per-side 결정
- container.world.transform = R + wrist_translation (rotation set, translation overwrite)
- FixedJoint `localRot0 = R quat`, `localRot1 = identity` (URDF transform consistency 통과)
- TCP world position = R(0,0,GRIPPER_TCP_OFFSET_Z) + wrist_translation. LEFT 의 경우 TCP 가 wrist 의 +Y 방향 0.15 m → world (0.003, **+0.379**, -0.085). RIGHT 는 (0.003, **-0.379**, -0.085).

#### 검증 (scripts/verify_gripper_world_pos.py 신규)

```
[LEFT]
  fingertip dir (TCP - base_link) : (+0.0000, +0.1500, -0.0000)  ← +Y direction
[RIGHT]
  fingertip dir                    : (+0.0000, -0.1500, -0.0000)  ← -Y direction
```

✓ 두 그리퍼 모두 팔의 연장 방향으로 fingertip 이 뻗음.

[scripts/test_robotiq_pose.py](scripts/test_robotiq_pose.py) 60-step settle: 12 joint 전부 ±10° 이내, max drift **2.6°** (이전 180° (1,1,0) 방향 대비 더 작음 — gravity torque 가 새 orientation 에선 joint 축에 덜 align 됨).

monitor mode steps=5: `reached --steps=5`, FATAL/Traceback/negative mass 경고 없음, Action Manager 그대로.

#### IDLE pose 의 일관성 확인

env_cfg 의 idle 액션은 wrist target quat = `(0.707, 0, 0.707, 0)` = 90° around Y. 새 rotation 과 합성:
- LEFT: gripper.rotation at idle = Ry(90°) * Rx(-90°). gripper +Z direction in world = +Y (변함없음)
- RIGHT: gripper +Z = -Y (변함없음)

즉 bind pose 와 idle pose 모두에서 그리퍼가 동일하게 *바깥 방향* 으로 뻗는다. wrist 의 90°Y idle 회전이 Y 축을 보존하므로.

#### 산출 파일 변경 (2026-05-13 3rd session)

수정
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py) — `_wrist_to_gripper_rotation(side)` per-side 정의, container.xform 에 rotation 적용, FixedJoint.localRot0 = R, TCP world 위치 계산에 R 반영

신규
- [scripts/inspect_wrist_frame.py](scripts/inspect_wrist_frame.py) — GR1T2 wrist 의 world 방향 + Fourier 손가락 delta 진단
- [scripts/verify_gripper_world_pos.py](scripts/verify_gripper_world_pos.py) — 빌드 후 fingertip direction 확인

---

### 2026-05-13 (2nd session) — 옵션 A 그리퍼 시각 변형 fix (mimic + drive + mass)

빌드/실행 후 사용자가 캡쳐한 Isaac Sim 스크린샷에서 그리퍼가 **사슬처럼 늘어진 모양**으로 잘못 표시됨. 5.1 known issue ("일부 follower link 가 lead 와 동기 안 됨") 가 정확히 우리 케이스에서 발현된 것. design-guide #43 §6.6 fallback 경로로 우회.

#### 진단 (scripts/diagnose_robotiq_attach.py 신규)

`inspect_robotiq_usd.py` 보다 더 자세히 — 모든 body/joint 의 `rel target` 유효성 + `drive` 값 + `limits` + `mimic gearing` 을 dump.

확인된 사실:
- joint body0/body1, mimic referenceJoint 의 rel target 은 전부 OK
- mimic API 스키마는 전부 적용돼 있음
- **그러나** follower 5개 중 4 개는 `drive: stiff=0 damp=0 maxF=0`, `left_left_inner_finger_knuckle_joint` 와 `left_right_outer_knuckle_joint` 는 drive API 자체 미적용
- `base_link` / `outer_knuckle` 에 `PhysicsMassAPI` 없음 → PhysX `negative mass` 경고 + small-sphere fallback
- `{side}_right_outer_knuckle_joint` (gearing=-1 follower) 의 limit 이 `[0°, 47°]` 인데 lead=+47° 시 -47° 로 가야 함 → clamp 됨

#### Issue B-1 — Follower joint drive 미적용

- **증상**: 시뮬레이션 시작 직후 follower link 들이 중력으로 표류 → 그리퍼가 변형
- **Fix**: [build_robotiq_usd.py:4d](isaac_file/build_robotiq_usd.py) — 6 joint 모두에 `UsdPhysics.DriveAPI.Apply(j, "angular")` + `stiff=50, damp=5, maxF=50` 명시. Isaac Lab actuator cfg 가 stiff/damp 를 runtime 에 override 하지만, drive API 자체가 있어야 binding 됨. mimic API 도 defensive 하게 remove + re-apply.

#### Issue B-2 — `base_link` / `outer_knuckle` 질량 미설정

- **증상**: `[Warning] possibly invalid inertia tensor of {1.0, 1.0, 1.0} and a negative mass`. 그리퍼 자체 거동은 OK 처럼 보이지만 articulation 의 반력 균형이 깨져서 wrist rotation artifact 유발.
- **Fix**: [build_robotiq_usd.py:4e](isaac_file/build_robotiq_usd.py) — `base_link` 0.6 kg, 각 `outer_knuckle` 0.05 kg 명시. datasheet 0.925 kg 의 잔여분이 정확히 이 두 prim 에 들어감 (다른 4개 visible body 는 stock USD 가 mass 0.039 / 0.027 로 이미 설정).

#### Issue B-3 — outer_knuckle 비대칭 limit

- **증상**: gearing=-1 follower 가 -47° 로 못 감 → parallel-grasp linkage 파괴
- **Fix**: [build_robotiq_usd.py:4c](isaac_file/build_robotiq_usd.py) — `{side}_right_outer_knuckle_joint` 의 `physics:lowerLimit` 을 `-47.0` 으로 명시. upper 는 stock 의 47.0 그대로.

#### env_cfg 6-joint 전환 (design-guide #43 §6.6 fallback path)

- `ROBOTIQ_ALL_JOINTS_LEFT/RIGHT` 와 gearing 테이블 추가
- `ImplicitActuatorCfg.joint_names_expr` 를 lead → 6 joints 전체로
- `BinaryJointPositionAction.joint_names` + `open/close_command_expr` 도 6 joints, 각 follower 는 `gear * ROBOTIQ_CLOSE_RAD` 부호 적용
- `joint_pos` 초기값에 12 joint (6×2) 모두 0.0 명시
- 관찰 (`gripper_joint_state`) 는 lead 2 joint 만 유지 — gripper open/close 상태만 알면 됨

검증:
- [scripts/diagnose_robotiq_attach.py](scripts/diagnose_robotiq_attach.py): joint table 에 6개 모두 `drive: stiff=50.0 damp=5.0`, outer_knuckle limits `[-47, 47]`, mimic gearing 정상
- monitor mode steps=5 통과, `negative mass` 경고 사라짐, Action Manager 그대로 (`pink_ik_cfg=14, left_gripper_action=1, right_gripper_action=1`)
- [scripts/test_robotiq_pose.py](scripts/test_robotiq_pose.py) 신규: idle 액션 60 step 후 12개 robotiq joint 가 ±2.6° 이내 → 그리퍼가 open 자세 안정적 유지. step 29 에서 일시적 over-shoot (~5-7°) 후 step 59 에 settle — overdamped 거동.

#### PD 튜닝 노트

- env_cfg actuator stiffness=10, damping=80 (design-guide #43 의 safe zone 1.0 보다 10× 강한 값. Issue #3347 의 ≥2000 한계로부터 200× 여유).
- stiffness=1.0 일 때 follower steady-state drift 가 ~5°. 10× 강화로 거의 0°.
- 60 step (0.5s @ 120Hz) 이 PD settle 시간. 그 이하로 측정하면 overshoot 구간에서 잘못된 값.

#### 산출 파일 변경 (2026-05-13 2nd session)

수정
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py) — 4c (대칭 limit), 4d (drive + mimic 재적용), 4e (mass + inertia)
- [kitchen_sorting_gr1t2_gripper_env_cfg.py](kitchen_sorting_gr1t2_gripper_env_cfg.py) — 6-joint actuator + binary action

신규
- [scripts/diagnose_robotiq_attach.py](scripts/diagnose_robotiq_attach.py)
- [scripts/test_robotiq_pose.py](scripts/test_robotiq_pose.py)
- [~/.claude/.../memory/feedback_robotiq_attach_mimic_drives.md](file:///C:/Users/pjwpy/.claude/projects/C--develop-IsaacLab/memory/feedback_robotiq_attach_mimic_drives.md)

---

### 2026-05-13 — 옵션 A (Robotiq 2F-85) 마이그레이션

옵션 B (자작 박스 그리퍼) 의 비주얼/물리적 한계가 명확해 옵션 A 로 마이그레이션. 설계 가이드는 [`../research/43. robotiq_2f85_optionA_migration_design_guide.md`](../research/43.%20robotiq_2f85_optionA_migration_design_guide.md). 한 세션에서 빌드 → 검증 → 3 개 issue fix → 통과 완료.

#### 단계 0 — Robotiq stock USD 다운로드 + inspect

- NVIDIA Isaac Sim 5.1 S3 (`Assets/Isaac/5.1/Isaac/Robots/Robotiq/2F-85/`) 에서 9 개 파일 (`Robotiq_2F_85_edit.usd` + `configuration/` + `payloads/` + `parts/×6`) 을 `ust_hm_grip/isaac_file/robotiq/` 로 캐시.
- [scripts/inspect_robotiq_usd.py](scripts/inspect_robotiq_usd.py) 신규 — prim tree, joint 표, PhysxMimicJointAPI 적용 여부, ArticulationRootAPI 위치 자동 검증.
- 확인된 stock 구조:
  - default prim: `/Robotiq_2F_85`
  - articulation root: `/Robotiq_2F_85/Robotiq_2F_85` (한 단계 nested)
  - 8 revolute joints: 1 lead (`finger_joint`) + 5 mimics (`right_outer_knuckle_joint`, `right_inner_finger_joint`, `right_inner_finger_knuckle_joint`, `left_inner_finger_knuckle_joint`, `left_inner_finger_joint`) + 2 fixed (outer_finger 들).
  - VariantSet: `Physics = 'Physx_Mimic'` 기본 선택.
  - PhysxMimicJointAPI 5 개 모두 보존 (`rotZ` 또는 `rotX`).

#### 단계 1 — `build_robotiq_usd.py` 작성

- 패턴: build_gripper_usd 의 9.45 / 9.48 fix 를 계승 + Robotiq 통합.
- 7 단계 빌드 흐름 (claude.md §3.8 참조):
  1. GR1T2 stock USD flatten + 22-DoF Fourier hand 제거
  2. Robotiq stock USD 평탄화 (`Sdf.CopySpec`) + container Xform 으로 import
  3. container 의 transform op = `wrist_world` (URDF consistency check 통과)
  4. ArticulationRootAPI 제거 (humanoid root 만 유지)
  5. 모든 joint 에 `{side}_` prefix rename + PhysxMimicJointAPI `referenceJoint` rel 재작성
  6. TCP frame 추가 (`*_gripper_tcp_link`) + fixed joint (base_link → TCP, +0.150 m Z)
  7. wrist fixed joint (`*_hand_pitch_link` → `*_robotiq_arg2f_85/base_link`)

#### Issue A-1 — FixedJoint transform consistency 실패

- **증상**: `ValueError: The 'left_robotiq_attach_fixed_joint''s joint transforms are not consistent.` URDF 변환 시.
- **원인**: container prim 에 xformOp 없어서 `base_link_world ≠ wrist_world`. URDF 변환기의 `parent_world * localPos0 == child_world * localPos1` 검사 실패.
- **Fix**: `_attach_robotiq` 에서 container Xform 에 `wrist_world` transform op 명시 설정. base_link 의 reference-internal local transform 이 identity 이므로 container.world = wrist_world 면 base_link.world = wrist_world.

#### Issue A-2 — `joint 'Joints_finger_joint' is not unique` (URDF export 실패)

- **증상**: URDF 변환기가 두 그리퍼의 동일 leaf joint 이름 (`finger_joint`) 을 export 하려다 충돌.
- **원인**: Reference 방식 import 가 source prim 이름을 보존 → 좌/우 그리퍼 모두 같은 leaf name. Isaac Lab `find_joints` 는 path 가 아닌 leaf name regex.
- **Fix**: build_robotiq_usd.py 를 reference → **`Sdf.CopySpec` 평탄화 import** 로 변경 + 모든 `Joints/*` prim 을 `{side}_` prefix 로 rename + PhysxMimicJointAPI 의 `rot{X,Y,Z}:referenceJoint` rel-target 도 새 path 로 재작성.
- 결과: joint 이름이 `left_finger_joint`, `right_finger_joint`, `left_right_outer_knuckle_joint`, ... 식으로 unique.

#### Issue A-3 — Observation `joint_names` 의 옵션 B 잔재

- **증상**: `ValueError: Not all regular expressions are matched! .*_gripper_finger_.*_joint: []`
- **원인**: env_cfg 의 ObservationTermCfg 에 옵션 B 의 `.*_gripper_finger_.*_joint` regex 가 남아 있어 새 robotiq joint 이름과 매치 안 됨.
- **Fix**: `[ROBOTIQ_LEAD_JOINT_LEFT, ROBOTIQ_LEAD_JOINT_RIGHT]` explicit list 로 교체 (양쪽 lead joint 만 관찰).

#### 최종 검증 (monitor mode, steps=5)

```
[ust_hm_grip] _pink_hand_dim_zero_patch applied
[ust_hm_grip] URDF cache hit -> ...\Temp\urdf\GR1T2_with_robotiq.urdf
[ust_hm_grip][waist] pink_controlled_joint_names (17 joints): [...]
[INFO] Action Manager:  <ActionManager> contains 3 active terms.
   pink_ik_cfg          = 14
   left_gripper_action  =  1
   right_gripper_action =  1
[run_teleop] main loop exit -- reason='reached --steps=5', steps_completed=5
```
FATAL / RuntimeError / ValueError / FrameNotFound / shape mismatch 전부 없음.

#### 산출 파일 변경 요약 (2026-05-13)

신규
- [isaac_file/build_robotiq_usd.py](isaac_file/build_robotiq_usd.py)
- [isaac_file/GR1T2_with_robotiq.usd](isaac_file/GR1T2_with_robotiq.usd)
- [isaac_file/robotiq/](isaac_file/robotiq/) (stock USD 9 개 캐시)
- [scripts/inspect_robotiq_usd.py](scripts/inspect_robotiq_usd.py)
- [../research/43. robotiq_2f85_optionA_migration_design_guide.md](../research/43.%20robotiq_2f85_optionA_migration_design_guide.md) (설계 가이드)

수정
- [kitchen_sorting_gr1t2_gripper_env_cfg.py](kitchen_sorting_gr1t2_gripper_env_cfg.py) — USD path / Robotiq actuator / TCP IK target / binary action / obs joint names

Deprecated (보존)
- [isaac_file/build_gripper_usd.py](isaac_file/build_gripper_usd.py)
- [isaac_file/GR1T2_with_gripper.usd](isaac_file/GR1T2_with_gripper.usd)
- env_cfg 의 `GRIPPER_JOINT_NAMES_LEFT/RIGHT` 상수 (unused dead code)

---

## 핵심 학습 (재사용 가능)

### 2026-05-10 의 5 가지
1. **configclass mutable default 누적** — class-level `pink_controlled_joint_names` list 가 instance 간 공유 → `append` 가 누적. `__post_init__` 에서 매번 재할당.
2. **Pink IK `actions[:, -hand_joint_dim:]` 슬라이싱 버그** — `hand_joint_dim=0` 이면 전체 텐서 반환 (Python `-0:` 룰). monkey-patch 로 우회.
3. **PowerShell stdout buffering** — 긴 작업이 hang 으로 보임. `sys.stdout.reconfigure(line_buffering=True)` + ETA print + 결과 캐시.
4. **HIGH priority + tight loop UI starve** — `time.sleep(0)` yield + 주기적 explicit `simulation_app.update()`.
5. **Sdf.CopySpec 평탄화 + joint rename + Mimic rel 재작성** — 외부 articulation USD 를 humanoid 에 attach 시 joint 이름 충돌 회피하는 정공법.

### 2026-05-13 (10th session) 의 3 가지

14. **SteamVR Action System binding 의 Personal Binding commit 차단** — `ust.teleop.gr1t2_gripper` 앱의 binding URL 을 vrsettings 의 `CurrentURL_steamvrinput` 키에 set 해도 SteamVR UI 의 "Replace Default Binding" 클릭이 silently key 를 reset (Personal Binding 으로 commit 안 됨). 결과: 모든 Action API handle `bActive=False` 영구. SteamVR 의 알려진 UI bug 또는 PICO 4 Ultra + PICO Connect 10.6.6 의 회귀. *fix: 차후 SteamVR 2.16+ 또는 ALVR/Virtual Desktop 로 streaming 솔루션 교체*.

15. **SteamVR/OpenXR runtime 의 multi-instance input routing 정책** — 같은 process 안에 multiple OpenXR instance 가능하고 secondary instance 도 SYNCHRONIZED → VISIBLE → **FOCUSED** 상태 도달 가능. 그러나 SteamVR 가 *primary* instance (Isaac Sim 의 omni.kit.xr.core) 에만 input data 라우팅, secondary instance 는 FOCUSED 여도 `is_active=False`. piggyback 으로 우회 시도해도 차단.

16. **OpenXR `XR_MND_headless` 의 한계** — headless session 은 graphics binding 없이 instance + session + hand_tracker + action 생성 모두 가능 (SteamVR/OpenXR 2.15.6 에서 검증). 그러나 SteamVR 의 focus 정책상 headless session 은 VISIBLE 도 FOCUSED 도 못 도달 → Action API 결과 `is_active=False`. *headless 는 진단 toolkit 으로만 유용, production input pipeline 으로는 부적합*.

### 2026-05-13 (9th session) 의 2 가지
12. **Closed-loop linkage + 다중 PD 타깃 = 솔버 발산** — Robotiq 2F-85 의 4-bar linkage 6 joint 에 독립 PD 타깃을 부여하면 (gear×lead 식 의도해도), transient 단계에서 각 joint 의 타깃이 linkage 의 매-step kinematic 해와 미세하게 어긋나며 누적 오차가 mimic + linkage 의 hard constraint 와 충돌 → joint 가 ±100~1000° 로 발산. 어떤 K/D 조합 (10/80, 50/5, 200/20, 200/80) 도 해결 안 됨. **해법: lead 1 개만 driving (K=200, D=20), followers 는 K=0 + small D 로 mimic + linkage 가 자동 coordinate 하게 둠** (Robotiq stock USD 의 원래 의도).
13. **`test_robotiq_pose.py` 의 맹점** — idle (target=0) 만 60 step 검증해서 OPEN 상태가 stable 한 것만 확인. CLOSE 동작 (joint 가 traversal 해야 하는) 은 한 번도 측정 안 했음. **신규 `test_robotiq_close.py` 가 3-phase E2E** (open settle → close both → asym L=close/R=open) 로 wiring + 물리 모두 검증.

### 2026-05-16 (13th session, part 1~8) 의 8 가지

17. **Robotiq lead joint hard stall** (part 1) — stock NVIDIA Robotiq USD 가 lead `finger_joint` 에 baked `maxJointVelocity=146.46 deg/s` + `armature=1e-4 kg·m²` + `drive.maxForce=50 N·m` 가 implicit-solver + mimic + 4-bar closed-loop 와 충돌해 close 명령 시 ~3° 에서 stall.  build script 의 §4d 6 joint 전부에 `10000 / 0.01 / maxForce=500` author + env_cfg lead actuator K=400/D=40/effort=500 으로 해소.  `test_robotiq_close.py` lead 가 +40° 도달 확인.

18. **Retargeter double-transform bug** (part 2) — XR backend 의 XRoboSampler 가 `xr_to_isaaclab` 으로 이미 IL frame 으로 변환한 pose 를 snapshot 에 넣음.  retargeter 가 `svr_to_isaaclab` 을 또 적용하면 좌표 mangling (R_SVR2IL == R_XR2IL 이지만 R²≠R).  `pose_in_il_frame` cfg flag 로 backend 별 가드, `gr1t2_gripper_device.start` 가 자동 전달.

19. **수직 매핑 mismatch** (part 3) — XR LOCAL 좌표계 원점이 head 부근.  controller 의 raw IL Z = head-relative 값을 robot wrist Z (pelvis-relative) 로 직매핑하면 wrist target 이 pelvis 아래 40cm (knee 근처).  `pico_device_cfg["subtract_waist_z"] = True` 로 user pelvis Z 도 빼서 wrist height-above-pelvis 매핑 정상화.

20. **Phase D direct articulation API side-channel** (part 4) — Pink IK FrameTask 확장 (ACTION_DIM 16→30+) 의 invasive 한 대안으로 ust_hm_glove 패턴 차용.  매 frame `env.step` 직전에 `robot.set_joint_position_target(target, joint_ids=...)` 로 head_yaw/pitch/roll + waist_yaw/pitch/roll 직접 주입.  action manager 무수정, 기존 16-D Pink IK + gripper 그대로.  단 forearm/ankle 은 미구현 (Phase D++).

21. **정자세 캘리브레이션 — delta-from-zero** (part 5-7) — sensor raw 를 robot 에 직접 매핑하면 사용자의 startup 자세 (앉음, 굽음) 가 robot 의 영구 자세로 매핑.  fix: 첫 valid sample 캡쳐 → `delta = raw * inv(zero)`.  HMD/waist 의 quat, controller pos+quat 모두 calibration.  Part 7 에서 quat 추가 (이전엔 position 만, wrist orientation 은 raw 채택해 왼손 비틀림 발생).

22. **컨트롤러 A 버튼 런타임 재캘리브레이션** (part 6) — `xrobo_sampler` 가 `get_A_button` (PICO 우측 lower face) 노출, `_phase_d_check_recalibration` 가 rising-edge + 0.5s cooldown 으로 trigger.  Trigger 시 (a) `robot.write_joint_state_to_sim(default_joint_pos, zeros)` 으로 모든 joint 한 sim step 에 default 로 jump (transient 없음, part 7), (b) zero state reset → 다음 frame 의 user pose 가 새 zero.

23. **Idle pose anchoring at measured T-pose** (part 8) — `DEFAULT_LEFT_POS = (-0.20, 0, +1.05) + quat (0.707, 0, 0.707, 0)` 같은 legacy 값이 실측 T-pose 와 1.3m 차이 + L/R X 부호 반대 (LEFT 만 뒤쪽).  Pink IK 가 LEFT arm 만 backward twist.  `scripts/inspect_tcp_world_pose.py` 또는 `run_teleop.py` 의 `[tcp_diag]` 시작 출력으로 실측: `L_TCP=(0.003,+0.229,-0.235)`, `R_TCP=(0.003,-0.229,-0.235)`, quat=(0,0,1,0) palm-down.  실측값으로 anchoring → robot 이 calibration moment 에 정확히 default T-pose 유지.  `right_wrist_z180=False` 로 변경 (idle quat 이 이미 palm-down).

24. **TRACK 진단 강화 — 데이터 흐름 검증** — 사용자가 "트래킹 안 됨" 으로 인지하는 일이 반복.  실제론 데이터가 흐르지만 architecture 가 visible robot motion 으로 매핑 안 함.  `run_teleop.py` 의 3 초마다 wall-clock 기반 `[TRACK step=N]` 출력 — HMD/waist 의 raw + delta euler, L/R forearm + controller LIVE 값, retargeter 가 출력한 16-D action 의 wrist pos+quat 모두 표시.  "drives robot waist_yaw/pitch/roll" 같은 label 로 어디로 흘러가는지 명시.  `_last_action_cached` 으로 double-`device.advance()` 방지 (gripper hysteresis 보호).

### 2026-05-13 (2nd–8th session) 의 6 가지
6. **5.1 mimic known issue 우회 = 6 joint 전부 drive** — stock Robotiq USD 가 follower joint 의 PhysicsDriveAPI 를 삭제했음. mimic constraint 만으로는 5.1 의 known issue 발현 시 follower 가 중력으로 표류 (chain-like 변형). drive 명시 + gearing 부호 별 close target 으로 우회. **단, 이 fallback 은 OPEN (target=0) 에서만 안정** — CLOSE 에서는 #12 발생 → 9th session 에서 lead-only 로 재설계.
7. **PhysicsRigidBodyAPI 만 있고 MassAPI 없으면 PhysX 가 negative mass fallback** — `[Warning] possibly invalid inertia tensor and a negative mass`. articulation 의 반력 균형이 깨져 wrist rotation artifact. base_link 등 cabin 비어있는 body 에 mass + diagonal inertia 명시 필수.
8. **USD instancing 의 cross-stage leak** — `instanceable=True` prim 을 `Stage.Flatten()` 후 `Sdf.CopySpec` 으로 sub-tree 만 복사하면 prototype prim 은 *원본* layer 에 남는다. dst stage 의 `/Flattened_Prototype_N` 는 다른 articulation 의 prototype 으로 cross-resolve. **flatten 전에 모든 instanceable 끄기.**
9. **wrist link 의 visuals/collisions 제거 가능 (kinematic chain 무관)** — articulation 의 노드 (rigid body) 자체는 유지 필요, 그러나 자식 prim `visuals`, `collisions` 는 rendering / collision shape proxy 일 뿐이며 joint chain 과 무관. Robotiq attach 시 wrist 의 hand mesh 가 visual clutter 만 만들면 안전하게 제거 가능.
10. **URDF cache 의 mtime 비교 함정** — env_cfg 의 `__post_init__` 이 `mtime(URDF) >= mtime(USD)` 면 cache hit. USD rebuild 직후 즉시 launch 하면 직전 URDF 가 cache hit 되어 변경 안 반영됨. 빌드 전 강제 삭제 권장.
11. **USD rotation 변경의 3-점 일치** — `_attach_robotiq` 의 (1) container.world.transform rotation, (2) FixedJoint.localRot0 (`localRot1=identity`), (3) TCP world transform rotation — 셋이 같은 R 이 아니면 URDF 변환기의 transform consistency 검사가 실패하거나 시각이 어긋남.

각 항목은 claude.md 의 컨벤션 / 함정 표에 더 자세히 명문화돼 있다.

---

## 다음에 할 일 (TODO)

### 완료된 (9th session 기준)
- [x] **그리퍼가 chain-like 변형** (5.1 mimic known issue) — 6 joint drive + base_link mass + 대칭 limit (2nd session). pose test ±10° tol PASS.
- [x] **그리퍼 visual 이 GR1T2 thigh mesh 로 보임** — instancing prototype leak fix (4th session). `verify_all_visuals.py` 18/18 OK.
- [x] **그리퍼 mount orientation** — `Ry(180°)` (palm-down, 7th session). 추후 사용자가 다른 자세 원하면 `_wrist_to_gripper_rotation` 만 수정.
- [x] **그리퍼 fingertip 사이 hand mesh 끼임** — `hand_pitch_link/visuals` + `/collisions` 제거 (8th session). articulation 무관.
- [x] **PICO grip → 16-D action wiring** (9th session) — `tests/test_gripper_retargeter.py` 22/22 PASS, monitor mode 3-term action manager 확인, diag log 로 `raw=-1.0 → processed=+0.785 → joint_pos_target=+44.98°` 까지 정상 전달 확인.
- [x] **6-joint CLOSE 발산 → lead-only 재설계** (9th session) — actuator: lead K=200/D=20 + followers K=0/D=2; BinaryJointPositionAction joint_names 도 lead 1 개로 축소. 코드/syntax 검증 OK.

### 사용자 환경에서 추가 확인 필요
- [ ] **GUI 시각 최종 확인** (사용자 환경): Isaac Sim 창에서 Robotiq 2F-85 가 wrist 에 palm-down 자세로 부착되고 hand mesh 잔재 없는지.
- [ ] **XR 모드 텔레오퍼레이션 end-to-end**: SteamVR Personal Binding 저장 후 PICO 컨트롤러 grip → 해당 측 그리퍼만 closing, controller pose → TCP target. 좌/우 독립 동작 확인.
- [ ] **TCP offset 캘리브레이션**: `GRIPPER_TCP_OFFSET_Z = 0.150` 이 실제 fingertip midpoint 와 일치하는지 사용자 컨트롤러 cursor 와 비교 → 필요 시 build_robotiq_usd.py 의 상수 조정.
- [ ] **실제 grasping 검증**: 객체를 잡아보고 ① 두 그리퍼 finger 가 대칭으로 close 하는지 (gearing 부호), ② contact force 비대칭으로 인한 wrist rotation artifact 가 보이는지. 후자 발생 시 stiffness 를 10 → 5 또는 1 로 후퇴.
- [ ] **PD gain 튜닝**: 현재 stiffness=10, damping=80, effort=25 (8th session). Issue #3347 한계 2000 까지 여유 200×. grasping 시 object drop 발생하면 effort 점진 증가.

### Future cleanup
- [ ] **옵션 B 산출물 정리**: cleanup PR 에서 `build_gripper_usd.py` / `GR1T2_with_gripper.usd` / env_cfg 의 `GRIPPER_JOINT_NAMES_*` 제거.
- [ ] **`materials/materials.usd` sublayer 누락** 경고: visual 영향만 — 추후 NVIDIA S3 에 해당 파일 있다면 다운로드해서 cache 에 추가.
- [ ] **5.1 known issue 재발현 모니터링**: 6 joint drive 로 우회 중이지만, 추후 Isaac Sim 5.2+ 에서 mimic constraint 가 reliable 해지면 actuator 를 lead 1개로 축소 가능 (현재는 mimic-broken-safe 모드).

---

## 참고

- 작업 이전 단계의 사용자/feedback 메모리: [`~/.claude/projects/C--develop-IsaacLab/memory/MEMORY.md`](file:///C:/Users/pjwpy/.claude/projects/C--develop-IsaacLab/memory/MEMORY.md)
- 상위 폴더의 ust_ws-wide 컨텍스트: [`../CLAUDE.md`](../CLAUDE.md), [`../memory.md`](../memory.md)
- 설계 가이드: 36 (옵션 A/B/C 비교), 43 (옵션 A 구현)
