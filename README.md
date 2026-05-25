# UST Robotics Research Workspace (`ust_ws`)

NVIDIA Isaac Sim / Isaac Lab 2.3.0 위에 구축한 로보틱스 연구 워크스페이스. VR 텔레오퍼레이션, 모방학습(imitation learning), 교정 교육(corrective teaching), VLM/LLM 로봇 제어를 다룬다.

- **기반**: Isaac Lab 2.3.0, Isaac Sim 4.5/5.0/5.1, Python 3.10+
- **타깃 GPU**: NVIDIA RTX PRO 6000 (96GB VRAM) / RTX 4090
- **플랫폼**: 시뮬레이션은 Linux, VR 텔레오퍼레이션은 Windows/SteamVR
- **언어 관례**: 코드 주석·문서는 한국어, 클래스/함수명은 영어

> 상세 아키텍처·gotcha 레퍼런스는 [`CLAUDE.md`](CLAUDE.md), 시간순 수정 이력(v9.x)은 [`memory.md`](memory.md) 참조.

---

## 프로젝트 한눈에 보기

| 프로젝트 | 로봇 | 핵심 내용 | 상태 |
|---------|------|----------|------|
| [`ust_260207/`](#ust_260207--듀얼암-모바일-매니퓰레이터) | TurtleBot3 + 듀얼 OpenMANIPULATOR-X | 모바일 듀얼암 텔레오퍼레이션 + 모방학습 | 1차 프로젝트 |
| [`ust_260220/`](#ust_260220--g1-주방-분류-교정-교육) | Unitree G1 + INSPIRE 5지 핸드 | 주방 물체 분류 + 교정 교육(HG-DAgger/앙상블/VLM) | 고급 |
| [`ust_hm_glove/`](#ust_hm_glove--gr1t2--udcap-글러브) | Fourier GR1T2 + Fourier 6-DoF 핸드 | UDCAP VR 글러브 전신 텔레오퍼레이션 (22지 손가락) | 통합 트랙 |
| [`ust_hm_grip/`](#ust_hm_grip--gr1t2--2지-그리퍼) | Fourier GR1T2 + 2지 평행 그리퍼 | PICO 컨트롤러 그립 텔레오퍼레이션 (16D) | **활성** |
| [`ust_project1/`](#ust_project1--omnigraph--llm-제어-레거시) | TurtleBot3 + 듀얼 암 | OmniGraph 기반 텔레오퍼레이션 + LLM 제어 | 레거시 |
| [`LLM/`](#llm--자연어-로봇-제어) | (제어 백엔드) | 자연어 로봇 제어 (GPT-4/Claude + 5계층 안전) | 지원 |
| [`cloudxr_js/`](#cloudxr_js--quest-3s-웹-클라이언트) | (VR 클라이언트) | Quest 3S용 CloudXR.js TypeScript 웹 클라이언트 | 지원 |

---

## ust_260207 — 듀얼암 모바일 매니퓰레이터

TurtleBot3 Waffle Pi + 2×OpenMANIPULATOR-X (총 16관절) 모바일 매니퓰레이터의 텔레오퍼레이션 및 모방학습.

**로봇 시스템**
- 16관절: 휠 4 + 우암 4R + 우그리퍼 2R + 좌암 4R + 좌그리퍼 2R
- 휠 반경 0.033m, 휠베이스 0.287m, 암 리치 0.38m
- USD 체인: `isaac_file/ust_project1_robot.usd` → `ust_project1_fixed.usd`

**액션 공간 (18D)**: 휠 4D(속도) + 우암 6D(IK Δpose) + 우그리퍼 1D + 좌암 6D + 좌그리퍼 1D
- IK: DifferentialIK (DLS, λ=0.05) 또는 Lula RMP (`IK_METHOD` 전환)

**환경 변형 4종**: Teleop(키보드, 1env) / VR(D455+LiDAR CloudXR, 1env, 1h) / Train(4096env RL/IL) / DataCollect(Robomimic HDF5)

**디렉터리 구조**
- `ust_config/` — `ArticulationCfg`, 액션/관측/씬/디바이스/환경 cfg, Lula IK
- `ust_controllers/differential_drive_controller.py` — cmd_vel → 휠 속도
- `ust_utils/` — HDF5 데모 레코더(Robomimic 호환), 물리 설정
- `scripts/` — `run_teleop.py`, `record_demos.py`, `train_policy.py`(BC/BC-RNN), `run_ros2_bridge.py`

```bash
./isaaclab.sh -p ust_ws/ust_260207/scripts/run_teleop.py --teleop_device keyboard
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py --num_demos 20 --enable_cameras
./isaaclab.sh -p ust_ws/ust_260207/scripts/train_policy.py --algo bc_rnn --dataset ./datasets/ust_*.hdf5
```

---

## ust_260220 — G1 주방 분류 / 교정 교육

Unitree G1 + INSPIRE 5지 핸드(고정 베이스, 듀얼암 Pink IK)로 주방 물체를 분류 통에 정렬. 인간 교정 교육 파이프라인 3단계가 핵심.

**Gym IDs**: `Isaac-KitchenSorting-G1-InspireFTP-{v0,Vision-v0,Train-v0,VR-v0,DataCollect-v0}`

**교정 교육 파이프라인 (`corrective/`)**

| Phase | 모듈 | 핵심 클래스 | 목적 |
|-------|------|------------|------|
| 1 | `phase1/` | `BCRNNPolicy`, `MimicGenAugmentor`, `RobomimicConfig` | BC-RNN + MimicGen 데이터 증강 |
| 2 | `phase2/` | `HGDAggerLoop`, `InterventionManager`, `IWRTrainer` | Human-gated DAgger + 중요도 가중 회귀 |
| 3 | `phase3/` | `EnsemblePolicy`, `ConformalPredictor`, `VLMAnalyzer`, `HelpRequestDecider`, `AdaptiveThreshold` | 앙상블 불확실성 + 3-tier VLM + conformal prediction |

`corrective/utils/` — corrective HDF5 I/O, 평가 유틸.

```bash
./isaaclab.sh -p ust_ws/ust_260220/scripts/train_bc_rnn.py --dataset ./data/demos/kitchen_sorting_augmented.hdf5
./isaaclab.sh -p ust_ws/ust_260220/scripts/run_hg_dagger.py --checkpoint ./models/bc_rnn/model_best.pth
./isaaclab.sh -p ust_ws/ust_260220/scripts/run_uncertainty_loop.py --ensemble_dir ./models/ensemble/
```

---

## ust_hm_glove — GR1T2 + UDCAP 글러브

Windows/SteamVR 환경에서 **Fourier GR1T2 휴머노이드 + Fourier 6-DoF 핸드(22 손가락 관절)** 를 **UDCAP VR 글러브** + PICO 4 Ultra HMD로 전신 제어. Pink IK 36D 액션(14 EEF + 22 핸드 관절).

> 9.36 통합: 구 `ust_260418_win`(SteamVR 모듈 라이브러리) + `ust_fourier_260421`(GR1T2 프로젝트) + `ust_260502_win`(검증 하니스 → `validation/`)을 단일 self-contained 패키지로 합침.

**데이터 흐름**: UDCAP 글러브 → (SteamVR Skeletal Input 2.0 / VMC OSC) → `SteamVRSampler` → 손가락 매퍼 → Pink IK → Articulation

**teleop/ 모듈**
- 하드웨어 공통: `vr_sampler.py`(pyopenvr 스레드), `vmc_receiver.py`(UDP OSC), `coord_transforms.py`(SteamVR↔IsaacLab 좌표), `fingertip_extractor.py`
- GR1T2 전용: `gr1t2_udcap_device.py`, `gr1t2_retargeter.py`(36D, 손가락 소스 우선순위 체인), `fourier_hand_mapper.py`(VMC/skeletal→11관절/측, tanh 증폭, per-bone REST 캘리브레이션), `waist_estimator.py`, `head_estimator.py`

**Gym IDs**: `Isaac-KitchenSorting-GR1T2-Fourier-{v0,WaistEnabled-v0,Monitor-v0,VR-v0,Vision-v0,DataCollect-v0,RobotOnly-v0}`

**진단 도구**: `diagnose_pico_connect.py`(6계층 프로브), `diagnose_udcap_dataflow.py`, `diagnose_finger_actuator_limits.py`, `sniff_vmc_finger_motion.py`

```bash
python -X utf8 -m ust_ws.ust_hm_glove.scripts.run_teleop \
    --env_variant robot_only --teleop_device pico_udcap \
    --skeleton2 true --vr_runtime pico_connect \
    --finger_proximal_scale 2.5 --ignore_trackers true \
    --finger_lp_alpha 0.4 --render_interval 2 --process_priority high
```

주요 gotcha: 핸드 액추에이터 stiffness 오버라이드(USD 기본 0), `PACK_22D_SIGNS` 부호 관례, VMC bone REST POSE 차감, 팬텀 트래커 왜곡(`--ignore_trackers`), `velocity_limit_sim`/`effort_limit_sim` 사용 — 상세는 CLAUDE.md.

---

## ust_hm_grip — GR1T2 + 2지 그리퍼

22-DoF Fourier 핸드를 측당 **2-DoF 평행 그리퍼**로 교체. 사용자가 **PICO Touch 컨트롤러를 직접 쥐고** 그립 Pull로 그리퍼 개폐. 16D 액션(14 EEF + 2 그리퍼). **현재 활성 프로젝트** (memory §10.34+).

> 9.36 통합: 구 `ust_260504_win` (그리퍼 마이그레이션). 9.37: PICO Connect → SteamVR → PC → Isaac Lab 파이프라인 추가.

**핵심 파일**
- `isaac_file/build_gripper_usd.py` — 스톡 GR1T2 USD에서 Fourier 핸드 prim 제거 후 손목에 2-prismatic 그리퍼 부착 (`GR1T2_with_gripper.usd` 생성). `build_robotiq_usd.py`도 동봉.
- `teleop/gr1t2_gripper_retargeter.py` — 16D, hysteresis(close 0.6 / open 0.4), `gripper_signal_source` ∈ {grip(기본), trigger, both}
- `teleop/gr1t2_gripper_device.py` — DeviceBase 래퍼
- `config/openvr_actions/` — 컨트롤러 6종 바인딩(`pico_controller`/`pico4`/`pico_phoenix`/`pico_neo3`/`oculus_touch`/`knuckles`)

**계층별 진단** (그리퍼 무응답 시 순서대로)
```bash
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pico_connect       # L0: PICO Connect 파이프라인
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw     # L1: 액션 매니페스트 우회 raw probe
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper            # L2: 액션 API 경로
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop \
    --env_variant robot_only --teleop_device pico_gripper \
    --vr_runtime pico_connect --gripper_signal_source grip \
    --render_mode monitor --render_interval 2 --process_priority high     # L3: 전체 루프
```

`enumerate_trackers.py` — PICO Motion Tracker 자동 감지 후 `tracker_binding_pico_connect.json` 채움. XRoboToolkit 연동 가이드(`XROBOTOOLKIT_EXECUTION_GUIDE.md`)도 포함.

---

## ust_project1 — OmniGraph + LLM 제어 (레거시)

ust_260207의 전신. OmniGraph 비주얼 노드그래프로 텔레오퍼레이션/IK/차동구동을 구성하고, Quest 입력을 직접 처리하는 초기 접근.

- `scripts/omnigraph/` — `master_graph.py`, `differential_drive_graph.py`, `ik_graph.py`, `teleoperation_graph.py`, `graph_builder.py`
- `scripts/ros2/` — `quest_input_handler.py`(Quest 입력), `haptic_feedback.py`
- `scripts/controllers/` — IK 컨트롤러, 좌표 변환, 그리퍼 컨트롤러
- `LLMRobotControl/` — LLM 로봇 제어 모듈 + Bio IK 연동 + 웹 UI 문서(KR/EN 이중 문서)
- `scripts/setup/` — OmniGraph 적용/검증 유틸

---

## LLM — 자연어 로봇 제어

GPT-4/Claude API 기반 자연어 로봇 명령. FastAPI 서버 + 5계층 안전 검증.

- `core/` — `llm_client.py`, `llm_tools.py`, `prompts.py`, `response_parser.py`, `command_validator.py`, `control_manager.py`, `robot_command.py`
- `isaac_interface/` — `robot_controller.py`, `manipulator.py`, `mobile_base.py`, `gripper.py`, `ik_solver.py`
- `safety/` — `collision_checker.py`, `emergency_stop.py` (+ 추가 검증 계층)
- `config/` — llm/robot/server/workspace YAML
- 문서: `docs/{QUICK_START,INSTALLATION_GUIDE,DEVELOPMENT_GUIDE}.md`

---

## cloudxr_js — Quest 3S 웹 클라이언트

Quest 3S 브라우저에서 CloudXR 스트림을 받는 TypeScript 웹 클라이언트 (npm, webpack, HTTPS dev 서버).

- `isaac/` — Isaac Sim용 클라이언트 (WebGL 상태 관리, 브라우저 capability, 성능 프로파일, 메트릭)
- `react/` — React 변형
- `proxy/` — HAProxy SSL 프록시 (WSS:48322 → WS:49100)

**Quest 3S 흐름**: Quest 브라우저 → HTTPS:8080 → CloudXR.js → WSS:48322 (HAProxy) → WS:49100 (CloudXR Runtime 6.0.1)

```bash
cd ust_ws/cloudxr_js/isaac && npm run dev-server:https
```

---

## 지원 자산 / 문서

| 디렉터리 | 내용 |
|---------|------|
| `isaac_file/` | USD 로봇 자산, URDF 임포트 스크립트, 물리/계층 수정 스크립트, Lula 설정 가이드 |
| `packages/` | MID-360 LiDAR USD/STEP 자산 (Livox 시뮬레이터, RealSense ROS2 드라이버는 외부 클론) |
| `openxr/` | CloudXR Runtime Docker 마운트용 공유 OpenXR 런타임 디렉터리 |
| `claudedocs/` | HRI 논문 요약 8종 (음성/제스처 생성, 소셜 로봇, 노인 케어 평가) |

> 외부 클론 레포(`openvla/`, `robotis_mujoco_menagerie/`, `XRoboToolkit-*`, `packages/realsense-ros/` 등), 모델 가중치(`models/`), 빌드/런타임 덤프는 `.gitignore`로 제외됨. 설계·연구 문서(`research/`, `cloudxr_research/`, `documents/`)는 로컬 전용.

---

## 공통 명령 (`isaaclab.sh`)

```bash
./isaaclab.sh -i              # 전체 확장 + RL 프레임워크 설치
./isaaclab.sh -f              # pre-commit (black/flake8/isort/pyupgrade/codespell)
./isaaclab.sh -t              # 전체 pytest
./isaaclab.sh -p script.py    # Isaac Sim 인터프리터로 스크립트 실행
./isaaclab.sh -s              # 확장 포함 Isaac Sim 실행
```

**포매팅**: Black(120, `--unstable`), isort(profile=black + 커스텀 섹션), Flake8(simplify/return), pyupgrade(`--py310-plus`). 라이선스 헤더 BSD-3-Clause (isaaclab_mimic은 Apache 2.0).

**리그레션** (Isaac Sim 불필요):
```bash
PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_glove/tests/   # 84 tests
PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_grip/tests/    # 22 tests
```

> Korean Windows에서는 cp949 인코딩 이슈 회피를 위해 모든 Windows 텔레오퍼레이션 엔트리포인트를 `python -X utf8 -m ...` 로 실행할 것 (CLAUDE.md gotcha #20).
