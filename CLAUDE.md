# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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

## Project Overview

Isaac Lab is a GPU-accelerated robotics research framework built on NVIDIA Isaac Sim. It supports reinforcement learning, imitation learning, and motion planning with fast physics/sensor simulation. Version 2.3.0, targeting Isaac Sim 4.5/5.0/5.1, Python 3.10+. The active research workspace is `ust_ws/` containing custom robot environments, VR teleoperation, corrective teaching, and VLM integrations documented below.

## Common Commands

All development commands go through `isaaclab.sh`:

```bash
./isaaclab.sh -i              # Install all extensions + RL frameworks
./isaaclab.sh -i none         # Install extensions only (no RL frameworks)
./isaaclab.sh -f              # Run pre-commit (black, flake8, isort, pyupgrade, codespell)
./isaaclab.sh -t              # Run all pytest tests
./isaaclab.sh -p script.py    # Run Python script via Isaac Sim's interpreter
./isaaclab.sh -s              # Launch Isaac Sim with extensions
./isaaclab.sh -d              # Build Sphinx docs
./isaaclab.sh -v              # Generate VSCode settings
```

### Running a Single Test

Tests run independently (one file per process). Use pytest directly:
```bash
./isaaclab.sh -p -m pytest source/isaaclab/test/test_something.py -v
```

Or use the test runner with extension filter:
```bash
./isaaclab.sh -p tools/run_all_tests.py --extension isaaclab
```

### UST Project Commands

```bash
# ust_260207: Dual-arm mobile manipulator teleop & imitation learning
./isaaclab.sh -p ust_ws/ust_260207/scripts/run_teleop.py --teleop_device keyboard
./isaaclab.sh -p ust_ws/ust_260207/scripts/run_teleop.py --teleop_device handtracking
./isaaclab.sh -p ust_ws/ust_260207/scripts/record_demos.py --num_demos 20 --enable_cameras
./isaaclab.sh -p ust_ws/ust_260207/scripts/train_policy.py --algo bc_rnn --dataset ./datasets/ust_*.hdf5 --epochs 2000
./isaaclab.sh -p ust_ws/ust_260207/scripts/test_dual_arm_env.py

# ust_260220: G1 kitchen sorting with corrective teaching
./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py --teleop_device handtracking
./isaaclab.sh -p ust_ws/ust_260220/scripts/train_bc_rnn.py --dataset ./data/demos/kitchen_sorting_augmented.hdf5
./isaaclab.sh -p ust_ws/ust_260220/scripts/run_hg_dagger.py --checkpoint ./models/bc_rnn/model_best.pth
./isaaclab.sh -p ust_ws/ust_260220/scripts/run_uncertainty_loop.py --ensemble_dir ./models/ensemble/

# ust_hm_glove: Windows/SteamVR Fourier GR1T2 + UDCAP glove teleop (전신 제어)
# 9.36 통합 — was ust_fourier_260421 + ust_260418_win + ust_260502_win.
# 9.37 — Skeleton 2.0 (SteamVR Skeletal Input)이 UDCAP 손가락의 PRIMARY 소스가 되었음.
#        VMC (Path B)는 default OFF (--path_b_port 0). UDCAP의 Skeletal Input 2.0
#        지원이 없을 때만 --path_b_port 39539 로 fallback 활성화.
# 9.37 — PICO Connect → SteamVR → PC → Isaac Lab 파이프라인 추가.
#        SteamVR Add-Ons에서 prism=ON (PICO Connect), VD=OFF, udcap=ON.
python -X utf8 -m ust_ws.ust_hm_glove.scripts.run_teleop \
    --env_variant robot_only --teleop_device pico_udcap \
    --skeleton2 true \
    --vr_runtime pico_connect \
    --finger_proximal_scale 2.5 \
    --ignore_trackers true \
    --finger_lp_alpha 0.4 \
    --render_mode monitor \
    --render_interval 2 \
    --process_priority high
# Legacy VMC 모드 (UDCAP가 Skeletal Input 2.0 미지원일 때):
python -X utf8 -m ust_ws.ust_hm_glove.scripts.run_teleop \
    --env_variant robot_only --teleop_device pico_udcap \
    --skeleton2 false --path_b_port 39539 --vmc_rest_frames 60 \
    --vr_runtime virtual_desktop \
    --finger_proximal_scale 2.5 --finger_lp_alpha 0.4 \
    --render_mode monitor --render_interval 2 --process_priority high
# 주요 9.x CLI flags (research/32, 33 참조):
#   --skeleton2 true|false   : 9.37 신규 — SteamVR Skeletal Input 2.0 (31-bone)
#                              primary 손가락 소스. default true.
#   --vr_runtime ARG         : 9.37 신규 — pico_connect | virtual_desktop |
#                              steamvr_native | auto. tracker_binding 자동 선택.
#   --path_b_port 0|39539    : 9.37 default 0 (VMC 비활성). UDCAP의 Skeleton 2.0
#                              지원이 없을 때만 39539 로 명시적 활성화.
#   --ignore_trackers true   : 트래커 없는 환경, 팔 idle T-pose 고정 (9.26)
#   --disable_arm_tracking   : 팔만 idle 고정 (waist/head는 추적 가능, 9.26)
#   --finger_proximal_scale  : 손가락 굽힘 amplification (default 2.5, 권장 2.0~3.5)
#   --finger_lp_alpha        : 22D 출력 EMA alpha (default 0.4, jitter↑0.2 / lag↑0.6)
#   --vmc_rest_frames        : rest pose 캘리브레이션 윈도우 (default 10, 권장 60)
#                              VMC fallback 사용 시에만 의미 있음.
#   --render_interval N      : 9.27 신규 — sim.render_interval 오버라이드 (default 1, 권장 2)
#   --process_priority       : 9.27 신규 — Windows process priority (default high, normal/high/realtime)
# 9.37 진단 (PICO Connect 파이프라인 6-layer probe):
python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_pico_connect
# UDCAP / 전체 데이터 흐름 진단:
python -X utf8 -m ust_ws.ust_hm_glove.scripts.diagnose_udcap_dataflow
# 9.27 진단 (USD 가 진짜 velocity 캡 하는지 확인):
./isaaclab.bat -p ust_ws/ust_hm_glove/scripts/diagnose_finger_actuator_limits.py
# Regression suite (no Isaac Sim required):
PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_glove/tests/
PYTHONPATH=. python -X utf8 ust_ws/ust_hm_glove/scripts/smoke_test.py
# Legacy G1 teleop (less common — kept for backwards compat):
python -X utf8 -m ust_ws.ust_hm_glove.scripts.run_teleop_g1_legacy --teleop_device pico_udcap

# ust_hm_grip: Windows/SteamVR Fourier GR1T2 + 2-finger gripper teleop (active project, 9.34+)
# 9.36 통합 — was ust_260504_win.
# 9.37 — PICO Connect -> SteamVR -> PC -> Isaac Lab 파이프라인 추가 (--vr_runtime pico_connect).
#        SteamVR Add-Ons: prism=ON (PICO Connect), VD=OFF, udcap=OFF.
#        tracker_binding template 자동 스왑 -> config/tracker_binding_pico_connect.json
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop \
    --env_variant robot_only --teleop_device pico_gripper \
    --vr_runtime pico_connect \
    --gripper_signal_source grip \
    --render_mode monitor --render_interval 2 \
    --process_priority high
# Legacy VD 모드 (PICO Connect 없이 Virtual Desktop 으로 트래커 라우팅):
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop \
    --env_variant robot_only --teleop_device pico_gripper \
    --vr_runtime virtual_desktop --gripper_signal_source grip \
    --render_mode monitor --render_interval 2 --process_priority high
# 9.37 진단 (PICO Connect 파이프라인 6-layer probe -- process/driver/openvr/binding):
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pico_connect
# PICO Motion Tracker 시리얼을 자동 감지해 tracker_binding template 채우기:
python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers \
    --out ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json
# Controller raw / gripper 진단:
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper
PYTHONPATH=. python -X utf8 -m pytest ust_ws/ust_hm_grip/tests/

# CloudXR environment setup (must source before running VR teleop)
source ust_ws/ust_260207/setup_cloudxr_env.sh
```

### Formatting

- **Black**: line length 120, `--unstable` flag
- **isort**: profile=black, custom section ordering (see `pyproject.toml`)
- **Flake8**: with flake8-simplify and flake8-return plugins
- **pyupgrade**: `--py310-plus`
- **Codespell**: ignore words in `pyproject.toml [tool.codespell]`
- **License headers**: BSD-3-Clause (Apache 2.0 for isaaclab_mimic)

## Source Code Layout

Five extension packages under `source/`:

| Package | Purpose |
|---------|---------|
| `isaaclab` | Core framework: sim, scene, envs, managers, assets, sensors, controllers, actuators, devices, terrains |
| `isaaclab_tasks` | Task/environment implementations (30+ envs: locomotion, manipulation, navigation, classic control) |
| `isaaclab_assets` | Pre-built robot configs (Franka, Unitree, Anymal, Humanoid, UR, etc.) |
| `isaaclab_rl` | RL framework wrappers (RSL-RL, SKRL, RL Games, Stable Baselines3) |
| `isaaclab_mimic` | Behavior cloning / imitation learning |

Each package is an Omniverse extension with `config/extension.toml` metadata and installed via `pip install -e`.

## Architecture

### Two Environment Workflows

**Manager-Based** (most common): Configure environments declaratively via `ManagerBasedRLEnvCfg`. MDP components (actions, observations, rewards, terminations, commands, events, curriculum) are assembled from reusable term functions in `isaaclab/envs/mdp/`.

```
ManagerBasedRLEnvCfg
├── scene: InteractiveSceneCfg (assets, sensors, terrain)
├── actions: ActionsCfg (JointPositionAction, DifferentialIKAction, etc.)
├── observations: ObservationsCfg (groups of observation terms)
├── rewards: RewardsCfg (weighted reward terms)
├── terminations: TerminationsCfg
├── commands: CommandsCfg
├── events: EventsCfg (domain randomization)
└── curriculum: CurriculumCfg
```

**Direct**: Subclass `DirectRLEnv`/`DirectMARLEnv` and implement `_setup_scene()`, `_compute_observations()`, `_compute_rewards()`, `_compute_dones()`, `_reset_idx()` manually.

### Configuration System

The `@configclass` decorator (wrapper around dataclasses) is used everywhere. Configs compose hierarchically. Use `MISSING` sentinel for required fields. Configs support `.to_dict()`, `.from_dict()`, `.replace()`, `.copy()`.

**Important**: Kit persistent settings in `user.config.json` override `.kit` file `persistent.*` settings.

### Scene & Assets

`InteractiveScene` parses `InteractiveSceneCfg`, spawns entities, and clones them across vectorized environments via `GridCloner`. Access assets by name: `scene["robot"]`.

Asset types: `Articulation`, `RigidObject`, `RigidObjectCollection`, `DeformableObject`. All inherit `AssetBase` with USD spawning, physics handles, and state buffers.

### App Startup Flow

1. `AppLauncher` parses CLI args (`--headless`, `--enable_cameras`, etc.) and launches Isaac Sim
2. `SimulationContext` singleton manages physics stepping and render modes
3. Environment creates `InteractiveScene` → spawns all entities
4. Managers instantiate → `env.reset()` → `env.step(action)` loop

### Task Registration

Tasks register via `gymnasium.register()` in their `__init__.py`:
```python
gym.register(id="Isaac-Humanoid-v0", entry_point="isaaclab.envs:ManagerBasedRLEnv",
             kwargs={"env_cfg_entry_point": "...humanoid_env_cfg:HumanoidEnvCfg"})
```

Task directories follow the pattern: `__init__.py` (registration), `*_env_cfg.py` (config), `agents/` (RL algo configs), `mdp/` (custom terms).

### Key Patterns

- **Spawners** (`isaaclab/sim/spawners/`): Convert configs to USD prims (shapes, files, lights, materials)
- **Converters** (`isaaclab/sim/converters/`): URDF/MJCF/mesh → USD
- **Actuators** (`isaaclab/actuators/`): `ImplicitActuatorCfg` for manager-based envs, with stiffness/damping/effort_limit
- **Sensors** (`isaaclab/sensors/`): Camera, RayCaster, ContactSensor, IMU, FrameTransformer
- **Controllers** (`isaaclab/controllers/`): DifferentialIK, OperationalSpace, JointImpedance, RmpFlow
- **Devices** (`isaaclab/devices/`): Keyboard, Gamepad, SpaceMouse, OpenXR teleoperation

### Import Ordering

isort sections (configured in `pyproject.toml`):
```
FUTURE → STDLIB → THIRDPARTY (numpy, torch, gym, pxr, omni, warp) →
ASSETS_FIRSTPARTY (isaaclab_assets) → FIRSTPARTY (isaaclab) →
EXTRA_FIRSTPARTY (isaaclab_rl, isaaclab_mimic) → TASK_FIRSTPARTY (isaaclab_tasks) → LOCALFOLDER
```

## USD Conventions

- PhysxMimicJointAPI: `apiSchemas` instance name MUST match property namespace (`transY` vs `rotY`)
- Visual meshes must be children of `RigidBody` prims to follow physics simulation
- `SdfTokenListOp()` on apiSchemas means "override with empty" (blocks inherited schemas); use `ClearInfo("apiSchemas")` to remove
- Sensor `prim_path` must point to a `RigidBody` prim, not a parent Xform
- Use `Sdf.CopySpec` + delete for reliable USD prim reparenting

## UST Workspace (`ust_ws/`)

User's active research workspace. Code comments and documentation are primarily in Korean (한국어).

### Directory Map

| Directory | Purpose |
|-----------|---------|
| `ust_260207/` | Primary project: TurtleBot3 + dual OpenMANIPULATOR-X teleoperation & imitation learning |
| `ust_260220/` | Advanced project: Unitree G1 kitchen sorting with corrective teaching (HG-DAgger, ensemble, VLM) |
| `ust_hm_glove/` | **9.36 unified UDCAP-glove track** (was `ust_260418_win` + `ust_fourier_260421` + `ust_260502_win` pre-9.36).  Fourier GR1T2 + UDCAP VR Glove teleop (Pink IK 36D action, RobotOnly debug env, HMD-driven head joint follow, hips/forearm trackers).  Self-contained — `teleop/` merges hardware-generic SteamVRSampler/coord_transforms with Fourier-specific GR1T2FourierUDCAPDevice/FourierHandMapper/WaistEstimator.  `validation/` (was `ust_260502_win`) holds 4-layer test harness (offline/headless/visual replay + live recording, rerun.io dashboard, baselines).  Legacy G1 entry-points renamed `*_g1_legacy.*` |
| `ust_hm_grip/` | **9.36 unified controller-grip track** (was `ust_260504_win` pre-9.36).  Fourier GR1T2 + 2-finger parallel gripper teleop with PICO Touch controllers (no UDCAP gloves).  16D action (14 EEF + 2 gripper).  `isaac_file/build_gripper_usd.py` materialises `GR1T2_with_gripper.usd`; `teleop/gr1t2_gripper_{device,retargeter}.py` translate SteamVR action-API trigger/grip to gripper open/close; `scripts/diagnose_gripper.py` + `scripts/diagnose_controller_raw.py` (legacy + Action API dual probe with tracker inventory + unknown-controller-type warning) for layered binding diagnostics.  Self-contained — copies `vr_sampler.py` + `coord_transforms.py` from the legacy shared layer.  Active project as of memory.md §10.34+ |
| `isaac_file/` | USD robot assets, URDF sources, physics fix scripts, arm sub-USDs |
| `cloudxr_js/` | CloudXR.js TypeScript web client for Quest 3S (npm, HTTPS dev server, HAProxy SSL proxy) |
| `openxr/` | Shared OpenXR runtime directory (CloudXR Runtime Docker mounts here) |
| `LLM/` | Natural language robot control (OpenAI/Claude API, FastAPI server, 5-layer safety) |
| `openvla/` | OpenVLA vision-language-action model integration |
| `models/` | Pre-trained VLMs: Qwen3-VL-8B/32B, SigLIP2-so400m, Florence2-large |
| `screanshot/` | Per-session video captures + ffmpeg-converted GIFs from teleop debugging runs |
| `research/` | 11 architecture/design research guides (numbered 1-9) |
| `cloudxr_research/` | 7 CloudXR + VLA research docs (`all_dev.md` is master spec) |
| `documents/` | 8 LLM robot control system design docs |
| `claudedocs/` | 8 HRI paper summaries |
| `memory.md` | **Chronological fix history** (running log of every numbered fix v9.x with root cause / change / verification). New entries append to the bottom |
| `CLAUDE.md` | **This file** — static workspace architecture reference |

### ust_260207: Dual-Arm Mobile Manipulator

#### Robot System

- **Platform**: TurtleBot3 Waffle Pi + 2x OpenMANIPULATOR-X
- **Total joints**: 16 (4 wheels + 4R arm + 2R gripper + 4L arm + 2L gripper)
- **USD chain**: `isaac_file/ust_project1_robot.usd` (wrapper, defaultPrim=Robot) -> `ust_project1_fixed.usd` (physics)
- **Arm sub-USDs**: `isaac_file/open_manipulator_x_right/` and `open_manipulator_x_left/` (prefixed joints/links)
- **Wheel radius**: 0.033m, **Wheel base**: 0.287m, **Arm reach**: 0.38m

#### File Map

| File (relative to `ust_ws/ust_260207/`) | Purpose |
|------|---------|
| `ust_config/ust_mobile_manipulator_cfg.py` | `ArticulationCfg`: 16 joints, actuator groups, arm param presets |
| `ust_config/ust_actions_cfg.py` | `USTActionsCfg` (18D IK), `USTSimpleActionsCfg` (14D direct) |
| `ust_config/ust_observations_cfg.py` | Observation groups: joint states, EE poses, object pose |
| `ust_config/ust_scene_cfg.py` | Scene variants: basic, with sensors (D455+MID-360), simple |
| `ust_config/ust_teleop_device_cfg.py` | `USTTeleopController`, VR/keyboard device factory, presets |
| `ust_config/ust_teleop_env_cfg.py` | 4 env configs: Teleop, VR, Train (4096), DataCollect |
| `ust_config/lula_ik_cfg.py` | `LulaIKConfig`, `LulaIKWrapper` for RMP-based IK |
| `ust_controllers/differential_drive_controller.py` | `DifferentialDriveController`: cmd_vel -> wheel velocities |
| `ust_utils/hdf5_recorder.py` | HDF5 demo recording (Robomimic-compatible) |
| `ust_utils/physics_setup.py` | Physics material and articulation setup helpers |
| `scripts/run_teleop.py` | Main teleoperation entry (sets `IPC_IGNORE_VERSION=1` before AppLauncher) |
| `scripts/record_demos.py` | Demo recording to HDF5 |
| `scripts/train_policy.py` | BC / BC-RNN training wrapper (Robomimic) |
| `scripts/run_ros2_bridge.py` | ROS2 bridge for real robot (Twist, JointState topics) |

#### Environment Variants

| Config Class | num_envs | Sensors | Episode | Use Case |
|-------------|----------|---------|---------|----------|
| `USTMobileManipulatorTeleopEnvCfg` | 1 | None | 60s | Keyboard teleop |
| `USTMobileManipulatorVREnvCfg` | 1 | D455+LiDAR | 1hr | VR CloudXR teleop |
| `USTMobileManipulatorTrainEnvCfg` | 4096 | None | 10s | RL/IL training |
| `USTMobileManipulatorDataCollectEnvCfg` | 1 | D455+LiDAR | 2min | Demo recording |

#### Action Space (18D)

| Index | Dim | Action Group | Control |
|-------|-----|--------------|---------|
| 0-3 | 4D | `base_action` (4 wheels) | Velocity (rad/s) |
| 4-9 | 6D | `right_arm_action` (IK delta pose) | DifferentialIK (DLS, lambda=0.05) |
| 10 | 1D | `right_gripper_action` | Binary (+0.019 open, -0.01 close) |
| 11-16 | 6D | `left_arm_action` (IK delta pose) | DifferentialIK |
| 17 | 1D | `left_gripper_action` | Binary |

#### Joint Index Map

```
0-3:   wheel_left_front, wheel_right_front, wheel_left_rear, wheel_right_rear
4-7:   right_joint1, right_joint2, right_joint3, right_joint4
8-9:   right_gripper_left_joint, right_gripper_right_joint
10-13: left_joint1, left_joint2, left_joint3, left_joint4
14-15: left_gripper_left_joint, left_gripper_right_joint
```

#### Key Exports (`ust_config/__init__.py`)

- `UST_MOBILE_MANIPULATOR_CFG` -- ArticulationCfg instance
- `ALL_DEV_ARM_PARAMS`, `TURTLEBOT3_ARM_PARAMS`, `ACTIVE_ARM_PARAMS` -- actuator presets
- `USTActionsCfg`, `IK_METHOD` -- action configs (`IK_METHOD = "dls"` or `"lula"`)
- `USTSceneCfg`, `USTObservationsCfg`, `USTMobileManipulatorTeleopEnvCfg`
- `USTTeleopDeviceCfg`, `create_ust_teleop_device`, `ALL_DEV_TELEOP_PRESET`, `CURRENT_TELEOP_PRESET`
- `LulaIKConfig`, `LulaIKWrapper`

### ust_260220: G1 Kitchen Sorting / Corrective Teaching

- **Robot**: Unitree G1 + INSPIRE 5-finger hand (fixed base, dual-arm Pink IK)
- **Task**: Sort kitchen objects into categorized bins
- **GPU target**: NVIDIA RTX PRO 6000 (96GB VRAM)
- **Env config**: `kitchen_sorting_env_cfg.py`
- **Gym IDs**: `Isaac-KitchenSorting-G1-InspireFTP-{v0,Vision-v0,Train-v0,VR-v0,DataCollect-v0}`

#### Corrective Teaching Pipeline (3 Phases)

| Phase | Module Path | Key Classes | Purpose |
|-------|-------------|-------------|---------|
| 1 | `corrective/phase1/` | `BCRNNPolicy`, `MimicGenAugmentor`, `RobomimicConfig` | BC-RNN + MimicGen data augmentation |
| 2 | `corrective/phase2/` | `HGDAggerLoop`, `InterventionManager`, `IWRTrainer` | Human-gated DAgger + importance-weighted regression |
| 3 | `corrective/phase3/` | `EnsemblePolicy`, `ConformalPredictor`, `VLMAnalyzer`, `HelpRequestDecider` | Ensemble uncertainty + 3-tier VLM + conformal prediction |

### ust_hm_glove: Fourier GR1T2 + UDCAP VR Glove Teleop (9.36 unified)

Windows/SteamVR teleop for **Fourier GR1T2 humanoid + 6-DoF Fourier hand** (22 finger joints) using **UDCAP VR gloves** + Pico 4 Ultra HMD.  Pink IK 36D action (14 EEF + 22 hand joints).

**Pre-9.36 lineage** (memory.md §10.44): `ust_hm_glove/` consolidates three formerly-separate packages:
- `ust_260418_win/` — Windows/SteamVR module library (originally for Unitree G1, was reused as base for the Fourier port)
- `ust_fourier_260421/` — active GR1T2 + Fourier-hand project that imported `ust_260418_win.teleop.*`
- `ust_260502_win/` — test/validation harness (now `ust_hm_glove/validation/` subpackage)

After 9.36 the entire UDCAP-glove track is **self-contained** under one package — cross-package imports were removed and modules were co-located in `ust_hm_glove/teleop/`.

#### Module Overview (relative to `ust_ws/ust_hm_glove/teleop/`)

Hardware-generic (was `ust_260418_win/teleop/`):

| File | Purpose |
|------|---------|
| `vr_sampler.py` | `SteamVRSampler` — pyopenvr background thread; snapshot dict with HMD / trackers / hands / controllers + skeletal action handles |
| `vmc_receiver.py` | `VMCHandReceiver` — UDP OSC listener for UDCAP's VMC bone broadcast (default port 39539) |
| `coord_transforms.py` | `svr_to_isaaclab(pos,quat)`, `forearm_to_wrist(pose,offset)`, `quat_multiply` — SteamVR ↔ IsaacLab coord swap helpers |
| `fingertip_extractor.py` | SteamVR Skeletal 31-bone → 5-fingertip wrist-frame positions (for DexPilot input) |
| `pico_udcap_device.py` | `PICOUDCAPDevice` for G1 + INSPIRE hand (legacy); `PICOInterventionInterface` (button debouncing) reused by GR1T2 device |
| `g1_retargeter.py` | G1-specific 38D Pink IK retargeter (legacy reference for the 36D GR1T2 variant) |
| `udcap_finger_mapper.py` | G1 INSPIRE-hand finger mapper (analogue of `fourier_hand_mapper`) |

Fourier GR1T2-specific (was `ust_fourier_260421/teleop/`):

| File | Purpose |
|------|---------|
| `gr1t2_udcap_device.py` | `GR1T2FourierUDCAPDevice` + `GR1T2FourierUDCAPDeviceCfg`; UDCAP setup-check + frozen-Z runtime watchdog; HeadEstimator wiring |
| `gr1t2_retargeter.py` | `GR1T2FourierSteamVRRetargeter` — 36D action; finger source priority chain (DexPilot → skeletal → action curls → VMC → button → idle) |
| `fourier_hand_mapper.py` | `FourierHandMapper` — VMC/skeletal → 11-joint per side; tanh amplification; per-bone REST POSE calibration |
| `waist_estimator.py` | `WaistEstimator` — hips tracker quat → (yaw, pitch, roll) with averaged zero-cal + per-axis deadband |
| `head_estimator.py` | `HeadEstimator` — HMD quat → robot head joint targets (averaged zero-cal + clamp) |
| `_osqp_compat.py` | qpsolvers 4.x ↔ osqp 0.6 shim |

`teleop/__init__.py` re-exports both module families so a single `from ust_ws.ust_hm_glove.teleop import ...` covers all of them (was split across two `__init__.py` pre-9.36).

#### Entry-point disambiguation (post-9.36)

- `scripts/run_teleop.py` — **active GR1T2 + Fourier hand** teleop (was `ust_fourier_260421/scripts/run_teleop.py`)
- `scripts/run_teleop_g1_legacy.py` — legacy G1 + INSPIRE teleop (was `ust_260418_win/scripts/run_teleop.py`)
- `scripts/smoke_test.py` / `scripts/smoke_test_g1_legacy.py` — corresponding standalone (no Isaac Sim) sanity checks
- `WINDOWS_EXECUTION_GUIDE.md` — primary guide (Fourier track) + `WINDOWS_EXECUTION_GUIDE_g1_legacy.md` for G1
- `validation/` — 4-layer test harness (was `ust_260502_win`): `scripts/run_replay_headless.py`, `tools/synth_vmc.py`, `visualization/in_sim_overlay.py`, `tests/test_finger_replay.py`, etc.

#### Robot System

- **Platform**: Fourier GR1T2 (humanoid, fixed base) + 6-DoF Fourier hand (5-finger × 11 joints per side: 4 proximal drivers + 1 thumb yaw + 4 PIP mimics + thumb pitch + thumb distal)
- **Joints**: 54 total (39 actuated). Hand: 22 finger joints. Head: `head_yaw_joint`, `head_pitch_joint`, `head_roll_joint`. Waist: `waist_yaw/pitch/roll_joint`
- **Control**: Pink IK with `PinkInverseKinematicsActionCfg`. Action 36D = 7 (L wrist pos+quat) + 7 (R wrist pos+quat) + 22 hand joints
- **GPU target**: NVIDIA RTX PRO 6000 (96GB VRAM)

#### Gym Environment IDs

| ID | Purpose |
|----|---------|
| `Isaac-KitchenSorting-GR1T2-Fourier-v0` | Base scene |
| `Isaac-KitchenSorting-GR1T2-Fourier-WaistEnabled-v0` | + 3 waist joints in Pink IK null-space |
| `Isaac-KitchenSorting-GR1T2-Fourier-Monitor-v0` | PC-only viewport |
| `Isaac-KitchenSorting-GR1T2-Fourier-VR-v0` | 1 h VR episode |
| `Isaac-KitchenSorting-GR1T2-Fourier-Vision-v0` | + wrist/shoulder cameras |
| `Isaac-KitchenSorting-GR1T2-Fourier-DataCollect-v0` | Vision + 90 s episode |
| `Isaac-KitchenSorting-GR1T2-Fourier-RobotOnly-v0` | **Empty scene** (robot+ground+light only) for finger/arm teleop debugging — no table/box obstructions |

#### File Map (relative to `ust_ws/ust_hm_glove/`)

| File | Purpose |
|------|---------|
| `__init__.py` | Lazy gym registration with explicit error capture (writes `config/last_import_error.log`) |
| `kitchen_sorting_gr1t2_env_cfg.py` | All 7 env cfg classes; hand-actuator override (stiffness=10000, damping=100) |
| `teleop/gr1t2_udcap_device.py` | `GR1T2FourierUDCAPDevice` + `GR1T2FourierUDCAPDeviceCfg`; UDCAP setup-check + frozen-Z runtime watchdog; HeadEstimator wiring |
| `teleop/gr1t2_retargeter.py` | `GR1T2FourierSteamVRRetargeter` — 36D action; finger source priority chain (DexPilot → skeletal → action curls → VMC → button → idle); rich periodic 5-finger log |
| `teleop/fourier_hand_mapper.py` | `FourierHandMapper` — VMC/skeletal → 11-joint per side; tanh amplification; per-bone REST POSE calibration |
| `teleop/waist_estimator.py` | `WaistEstimator` — hips tracker quat → (yaw, pitch, roll) with averaged zero-cal + per-axis deadband |
| `teleop/head_estimator.py` | `HeadEstimator` — HMD quat → robot head joint targets (averaged zero-cal + clamp) |
| `scripts/run_teleop.py` | Main teleop entry; CLI flags for finger scale/tanh/cal frames, prefer_controller, follow_hmd, head_follow_hmd, waist_pitch_deadband, etc. |
| `scripts/smoke_test.py` | Standalone (no Isaac Sim) sanity check; 7 tests |
| `scripts/diagnose_udcap_dataflow.py` | Layer-by-layer UDCAP probe (process, vrserver.txt, UDP listener) |
| `scripts/sniff_vmc_finger_motion.py` | Live VMC bone variation tracker |
| `tests/` | 84 pytest tests (mapper / retargeter / waist / head estimator) |
| `config/openvr_actions/` | `actions.json` + Index-profile binding + `manifest.vrmanifest` (auto-rewritten at runtime to point at active interpreter) |
| `config/tracker_binding.json` | Maps tracker serials (hips, *_arm_lower, *_lower_leg) to retargeter roles (waist, *_forearm, *_ankle) |
| `config/dex_retargeting/fourier_*_dexpilot.yml` | Optional DexPilot solver YAMLs (require URDF; falls back to FourierHandMapper when missing) |

#### Action Layout (36D)

| Index | Dim | Content |
|-------|-----|---------|
| 0:3 | 3 | Left wrist position (base_link frame, metres) |
| 3:7 | 4 | Left wrist quaternion (wxyz) |
| 7:10 | 3 | Right wrist position |
| 10:14 | 4 | Right wrist quaternion (wxyz, includes 180° Z correction) |
| 14:36 | 22 | Hand joints (`pack_22d` order: 5 L proximals, 5 R proximals, 4 L intermediates + L thumb pitch, 4 R intermediates + R thumb pitch, L+R thumb distals) |

#### Hand Joint Sign Convention

GR1T2 USD has finger flexion in the **negative** direction for proximal/intermediate/thumb_yaw joints (limits `[-x, 0]`) but **positive** for thumb pitch/distal.  `PACK_22D_SIGNS` in `fourier_hand_mapper.py` applies per-slot sign at packing time.

#### Critical Gotchas (GR1T2-specific)

1. **Hand actuator stiffness**: USD ships with `stiffness=None` (= 0) on hand joints → fingers don't move regardless of target. Override to `stiffness=10000, damping=100, effort_limit=100` in `kitchen_sorting_gr1t2_env_cfg.py::__post_init__`.
2. **Sign convention**: Sending +0.5 to a `[-1.57, 0]` joint → PhysX clamps to 0 → joint never moves. `PACK_22D_SIGNS` in `fourier_hand_mapper.py` MUST be applied.
3. **VMC bone REST POSE**: UDCAP broadcasts non-identity bone quats at "open hand" rest (~16° offset on pinky/ring). Without `vmc_subtract_rest=True` the mapper outputs a static curl baseline.
4. **Thumb yaw midpoint bug** (fixed in 9.15): naive `out = yaw*range/2 + (lo+hi)/2` maps yaw=0 to URDF range midpoint (+0.25 for `[-0.5, 1.0]`) → robot thumb stuck at -0.25 after `PACK_22D_SIGNS`. Use piecewise linear with yaw=0 → 0.
5. **UDCAP "Vive Tracker 3.0" Space Plan**: Without an actual Vive Tracker, UDCAP synthesizes a static fake knuckles pose → controller Z frozen across all frames. Either change UDCAP UI Space Plan to match the real hardware OR use `--prefer_controller false` and rely on wrist-mounted physical Vive trackers.
6. **Virtual Desktop AI body tracker noise**: VD-inferred hips quat can vary ~110° in pitch even when user stands still → robot auto-bends. WaistEstimator applies an averaged zero-cal + 17° pitch deadband by default.
7. **VMC port**: UDCAP's default OSC broadcast is **39539**. Always-on (`path_b_port` default 39539 in 9.13) so finger fallback works even when SteamVR Input curls stay 0.
8. **HMD camera vs head joint follow**: `--follow_hmd` (viewport camera follows HMD) was deprecated in 9.18 in favour of `--head_follow_hmd` (robot's head_yaw/pitch/roll joints follow HMD) which is the natural first-person teleop primitive.
9. **Thumb yaw axis (UDCAP X-axis vs Z-axis)** (fixed in 9.19, C8): UDCAP encodes thumb opposition on the X-axis of `LeftThumbProximal/RightThumbProximal` (qx Δ ≈ 0.38 dominant over qy/qz at 0.15-0.19 in raw VMC quaternion analysis).  The original `_quat_to_yaw` extraction was Z-axis Euler — discarded ~70% of the signal.  Switch to `_quat_to_pitch` (X-axis Euler) in both `map_hand_vmc` and `map_hand_skeletal`.
10. **Thumb yaw URDF clamp** (fixed in 9.20, C10): GR1T2 USD only allows opposition direction (URDF `[-1.74, 0]`).  When mapper output is negative thumb_yaw (extension direction), `PACK_22D_SIGNS[4]=-1` produces a positive packed value exceeding URDF max → PhysX clamps to 0 → spurious 0.08+ rad tracking errors observed on R thumb.  Truncate negative `yaw_norm` branch to 0 in both `map_hand_vmc` and `map_hand_skeletal`.
11. **VMC rest cal frames** (set in 9.19, C9): default `vmc_rest_frames=30` (was 10 in 9.18, but 10-frame window absorbed user fidget producing -2~3% Δrange degradation across all slots).  User holds open hand still for ~1.5s after start.
12. **Pink IK QP requires valid wrist quaternions** (fixed in 9.19): zeroing the first 14D of the 36D action gives Pink IK an invalid quaternion (norm 0) → OSQP builds non-PSD KKT matrix → "The problem seems to be non-convex" + "Workspace allocation error!" spam every step.  Always supply idle T-pose `(DEFAULT_LEFT/RIGHT_POS/QUAT)` for the arm 14D portion when only animating fingers (e.g. `ust_hm_glove/validation/scripts/run_replay_headless.py::build_idle_arm_14`).
13. **Replay loop wrap discontinuity** (fixed in 9.21, C11): `step % n_frames` wraps from last fully-flexed frame back to frame 0 (rest pose) in one tick → PhysX takes 30+ steps to catch up → tracking error spike up to 1 rad polluting `analyze_replay_hdf5` max statistics.  Clamp instead of wrap (hold last frame).  Layer-2 `--steps > recorded frame count` should NOT loop the recording.
14. **Omniverse extension lock files** (fixed in 9.21 operations guide): When a previous Isaac Sim run shuts down quickly (e.g. `analyze_replay_hdf5` matplotlib plot fires immediately after), stale lock files at `$env:USERPROFILE\AppData\Local\ov\cache\_cache.lock` and `$env:USERPROFILE\AppData\Local\ov\data\exts\v2\index\*\registry.lock` can persist.  Next Isaac Sim startup logs `[omni.kvdb.plugin] Disabling key-value database because another kit process is locking it`, then crashes during extension loading with `ImportError: DLL load failed while importing _multiarray_umath` (numpy bundled in `pip_prebundle`) and `ImportError: generic_type: type "ObjectType" is already registered!` (omni.physics.tensors).  Resolution: `Get-ChildItem -Path "$env:USERPROFILE\AppData\Local\ov" -Filter "*.lock" -Recurse | Remove-Item -Force`.  Operational rule: 30s pause between Isaac Sim runs.  **9.22 에서 lock-leak 무한루프 패턴 추가 발견** — see #18.
15. **Finger-output low-pass missing** (fixed in 9.23, see memory.md §10.31): `WaistEstimator` (9.13) and `HeadEstimator` (9.18) both ship `low_pass_alpha` to absorb Virtual Desktop AI body-tracker noise, but the finger path on `FourierHandMapper` / `GR1T2FourierSteamVRRetargeter` had no analogous filtering.  UDCAP broadcasts at ~140 Hz, SteamVRSampler at 120 Hz, env.step at 20 Hz (decimation 6 / sim.dt 1/120) — only 1 of every ~7 source frames reaches the action manager.  The non-linear `_quat_to_bend → tanh(scale*raw)` pipeline then amplifies any single-frame outlier into visible 0↔limit jumps in `[GR1T2Retarget #N]` (e.g. `l_idx` swinging -1.26 → -0.00 → -1.26 across 40 frames).  Fix is a single-pole EMA on the retargeter's 22D finger output (`finger_low_pass_alpha`, default 0.4).  Wrist EEF (action[0:14]) is NOT filtered to keep arm latency unchanged.  CLI: `--finger_lp_alpha {1.0=off, 0.4=default, 0.2=strong}`.  First frame passes through verbatim (no cold-start lag) thanks to `_prev_finger_22 is None` guard.

16. **env.step rate 20 Hz → 120 Hz** (changed in 9.24, see memory.md §10.32): The default `decimation = 6 / sim.dt = 1/120` produces env.step rate 20 Hz, sub-sampling UDCAP's 140 Hz finger stream at 7:1 — even with the 9.23 EMA fix the natural finger motion still showed visible jagged steps in the user video.  9.24 sets `decimation = 1` so env.step runs at 120 Hz, matching UDCAP at 1.17:1 (effectively 1:1).  Pre-requisites validated for the user's box: RTX PRO 6000 Blackwell + Ryzen 9 7950X3D + AMD 3D V-Cache Performance Optimizer Service running (CCD scheduling correct), Pink IK QP P99 ≈ 5 ms / 8.33 ms budget = 40% margin.  All 6 env variants (Base / WaistEnabled / Vision / Monitor / VR / DataCollect / RobotOnly) inherit the change without override.  9.23 EMA's `--finger_lp_alpha 0.4` time constant changes from 98 ms to 17 ms at 120 Hz — still feels smooth and now nearly lag-free; tune to 0.2 for stronger smoothing or 0.6 for more responsive.  Do NOT increase to 240 Hz: UDCAP 140 Hz is hard ceiling, Pink IK budget would drop to 4.17 ms (P99 violation), VR streaming locks at 90 Hz anyway.  See research/29~31 for the framerate ceiling derivation, hardware-specific analysis, and CPU-upgrade ROI rationale.

17. **`--ignore_trackers` semantic redefinition** (fixed in 9.26, see memory.md §10.34): The 9.25 umbrella flag forced `prefer_controller=True` so when the user removed all wrist trackers, the controllers (Touch/knuckles) drove the wrist instead — the OPPOSITE of user intent ("팔은 트래커가 있을 때만 움직여야 한다, 컨트롤러가 아닌 트래커에만").  9.26 introduces a dedicated `disable_arm_tracking` cfg + CLI flag that bypasses BOTH forearm AND controller paths in `_resolve_eef_target()` and returns idle T-pose directly.  `--ignore_trackers true` now forces `prefer_controller=False` + `disable_arm_tracking=True` (plus `enable_waist_dof=False`, `head_follow_hmd=False`).  Diagnostic signal: `[GR1T2Retarget #N]` shows `L=default(disabled)/vmc R=default(disabled)/vmc` confirming arms are locked.  Independent flag `--disable_arm_tracking true` available for users with hips tracker but no wrist trackers (waist follows tracker, arms stay at idle).  Logs verified the UDCAP finger transmission is intact: `bones_received=30` (full VMC payload) and `act_tgt = jpt = pos` (action manager / Pink IK / articulation healthy) — perceived "lag" is actually scale + rest-cal accuracy, not transmission loss.

18. **Phantom-tracker pose distortion** (fixed in 9.25, see memory.md §10.33): When a no-tracker rig (only PICO HMD + UDCAP gloves + 2 controllers) is used, Virtual Desktop AI body tracking and/or UDCAP's tracker-emulation can synthesise 5 phantom trackers (hips/chest/*_arm_lower) that report a static incorrect pose.  SteamVR sees them as real, and the retargeter's default forearm→controller priority picks the bad forearm pose → wrist target lands behind the back → Pink IK twists arms 180° to reach it.  WaistEstimator (re-enabled in 9.14) locks the bad hips quat as "rest" → torso bends forward indefinitely.  Three-part fix: (1) `KitchenSortingGR1T2RobotOnlyEnvCfg.enable_waist_dof` default True → **False** (revert to 9.13 default — only enable when a real hips tracker is mounted); (2) new `--ignore_trackers true` umbrella flag that auto-overrides `--prefer_controller=true`, `--enable_waist_dof=false`, `--head_follow_hmd=false`; (3) user must also disable Virtual Desktop AI body tracking (Settings → Streaming → Body Tracking).  Diagnostic signal: `[GR1T2Retarget #N]` shows `trackers=5` despite no physical tracker, plus `raw_SVR_arm_Z range: L=R=0.000m waist_pitch range=+0.0deg` for many seconds.  Also: VMC rest cal can lock a bad hand pose if the user starts with fingers slightly curled; bump `--vmc_rest_frames` to 60 and hold open hand still for ~1 s after launch.

19. **Lock-leak 무한루프** (diagnosed in 9.22, see memory.md §10.30): When a boot fails mid-startup (numpy/ObjectType cascade), the abnormal `SystemExit` skips lock cleanup → `_cache.lock` persists → next retry boots into stale KVDB → same cascade → another leaked lock → infinite loop.  A single `Remove-Item *.lock` (per #14) is **not sufficient** if the next boot itself fails — every failed retry leaves a fresh lock.  Diagnostic signal: `_cache.lock` `LastWriteTime` matches each failed retry timestamp.  Verification tool: 50-step sanity check `python -X utf8 -m ust_ws.ust_hm_glove.validation.scripts.run_replay_headless --replay <baseline> --output C:\Temp\sanity.hdf5 --steps 50 --headless --subtract-rest` — proves Isaac Sim can boot cleanly + Pink IK + articulation work, with `_cache.lock` released on success.  Standard recovery routine: lock cleanup → sanity check → second lock cleanup → `-X utf8` retry of teleop (see #20).  numpy 1.x↔2.x ABI mismatch (ust env 2.4.4 stub vs Isaac Sim bundled 1.26 PYD) appears in cascade fingerprint but is NOT the blocking factor — sys.path matching consistently picks isaacsim bundled when locks are clean (memory.md §10.30 §4.5 user-validated).
20. **`-X utf8` standardization** (Korean Windows, fixed in 9.22 operations guide): Default cp949 codec on Korean Windows fails to encode em-dash (`—`, U+2014) and other non-ASCII output in `print()` statements → `UnicodeEncodeError` raised after the actual work completes → abnormal `SystemExit` → cleanup skipped → `_cache.lock` leak (feeds into #19).  Confirmed offender: `ust_hm_glove/validation/scripts/run_replay_headless.py:296` `print(f"... done — {len(timestamps)} steps written.")`.  Mitigation: invoke all `ust_*_win` / `ust_fourier_*` Python entry points with `python -X utf8 -m ...` (matches CLAUDE.md UST Project Commands regression-suite pattern: `PYTHONPATH=. python -X utf8 -m pytest ...`).  Long-term fix: replace non-ASCII chars in user-facing prints with ASCII equivalents (`—` → `--`) or call `sys.stdout.reconfigure(encoding="utf-8")` at script entry.

21. **`ImplicitActuatorCfg.velocity_limit` / `effort_limit` silent-ignored** (fixed in 9.27, see memory.md §10.35 + research/33 §2.2): Isaac Lab `actuator_pd.py:79-100` nullifies the legacy `velocity_limit` (and warns once via `omni.log.warn` that's easy to miss in Isaac Sim's stdout flood) for all `ImplicitActuator` instances.  PhysX then uses whatever is baked into the USD's `physics:maxJointVelocity` — for the GR1T2 6-DoF hand this can be lower than the user's natural finger-flex speed (12-20 rad/s).  Fix is to use the `_sim` suffix variants (`velocity_limit_sim=50.0`, `effort_limit_sim=200.0`) which propagate to the ArticulationView at startup.  Symptom: target-vs-pos lag where `[FingerCmp #N]` shows `act_tgt = jpt = pos` matching at steady state but lagging by 0.1-0.3 rad during fast finger flex (PhysX rate-limited).  Diagnostic: `./isaaclab.bat -p ust_ws/ust_hm_glove/scripts/diagnose_finger_actuator_limits.py` reads the USD directly and prints per-joint `maxJointVelocity` / `maxForce`.

22. **9.27 finger-tracking lag suite** (fixed in 9.27, memory.md §10.35 + research/33): Even after 9.23 EMA + 9.24 decimation=1 + 9.25/9.26 phantom-tracker fixes, the user reports "real-time tracking 못 미친다" in monitor mode (so VR streaming-back is NOT the dominant culprit).  Four causes accumulate: (#2) `velocity_limit` silent-ignore (gotcha #21 above), (#3) `effort_limit=100 N·m` rate-limits fast transients, (#4) `sim.render_interval=1` at 120 Hz starves the encoder/streaming thread (~5-10 ms wall-time per env.step on RTX PRO 6000), (#5) Windows process priority NORMAL lets Discord/browser/AV preempt Isaac Sim (P99 jitter ~30 ms).  9.27 applies all four: cfg `effort_limit_sim=200, velocity_limit_sim=50` + new `--render_interval N` CLI flag (recommend 2 for VR / 4 for headless data-collection) + new `--process_priority {normal,high,realtime}` CLI flag (default `high`, calls `psutil.Process.nice(HIGH_PRIORITY_CLASS)` at startup).

23. **Standalone USD-editing scripts must boot Isaac Sim before `from pxr import ...`** (fixed in 9.28 for `build_gripper_usd.py`, see memory.md §10.37): `isaaclab.bat -p` runs the conda env's Python (`%CONDA_PREFIX%\python.exe`), which does NOT register the `pxr` USD libraries until `isaacsim` is imported AND `SimulationApp` is launched.  Helper scripts that call `from pxr import Usd, UsdGeom, UsdPhysics` at module / function level without first instantiating `AppLauncher` fail with `ImportError: No module named 'pxr'` even when launched correctly.  Diagnostic scripts that go through `from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR` get away with it because `isaaclab.*` import chain triggers the necessary path injection transitively; bare `from pxr` does not.  Standard fix pattern (used in `ust_hm_grip/isaac_file/build_gripper_usd.py`):
    ```python
    def _boot_isaac_sim():
        from isaaclab.app import AppLauncher
        boot_parser = argparse.ArgumentParser(add_help=False)
        AppLauncher.add_app_launcher_args(boot_parser)
        boot_args, remaining = boot_parser.parse_known_args()
        boot_args.headless = True              # USD-only: no GUI
        app_launcher = AppLauncher(boot_args)
        sys.argv = [sys.argv[0]] + remaining   # restore script's own flags
        return app_launcher.app
    def main():
        sim_app = _boot_isaac_sim()           # FIRST -- registers pxr schemas
        try:
            ...                                # script-specific argparse + USD work
        finally:
            sim_app.close()                    # gotcha #14/#19 lock-leak prevention
    ```
    The split parser is required so `add_app_launcher_args` doesn't hijack the script's own flags (`--output`, `--source`, etc.).

24. **Virtual Desktop synthesizes pose-only "controller" stubs when no physical PICO controllers are paired** (diagnosed in 9.28 ust_260504_win, see memory.md §10.37): When the PICO 4 Ultra is streamed via Virtual Desktop and the physical PICO controllers are powered off / unpaired / out of tracking volume, VD STILL emits two `oculus_touch` controllers with serials of the form `<HMD_serial>_Controller_Left` and `<HMD_serial>_Controller_Right` (e.g. `1PASH5D1P17365_Controller_Left`).  Their poses are fabricated from HMD orientation, but **all button axes return 0.0 forever** — `IVRSystem::getControllerState()` returns `success=true` with `rAxis[0..2].x = 0`, `ulButtonPressed = 0`.  Symptom: SteamVR Settings → Test Controller (Left/Right Hand) shows pose moving but no button activity; our `diagnose_gripper.py` shows `trigger=0.000 grip=0.000` even when the user is squeezing.  Diagnostic differentiator: the new `scripts/diagnose_controller_raw.py` calls `getControllerState()` directly (Action Manifest bypass) — if its output is also all-zero, the issue is at the controller→SteamVR layer (NOT bindings, NOT Save Personal Binding, NOT our action manifest).  Recovery checklist (in order, see memory.md §10.37 §3): (1) Power on PICO controllers, hold them in HMD camera view, (2) verify pairing in PICO OS Settings → Controllers, (3) Virtual Desktop Streamer Options → "Forward controller input to SteamVR" = ON, (4) SteamVR → Manage Add-Ons → `prism` (Pico Connect) OFF + `udcap` OFF (when not using gloves), (5) `Stop-Process -Name UdcapDriver -Force` if still running.  Only AFTER raw probe shows non-zero values does the binding's "Save Personal Binding" + active-binding selection (gotcha-adjacent ust_260504_win-specific) become relevant.

25. **SteamVR auto-remaps `bindings_index.json` (knuckles) to oculus_touch and SILENTLY SKIPs `/input/skeleton/*` + `/input/finger/*` paths when `default_bindings` lacks an explicit `oculus_touch` entry** (fixed in 9.29, see memory.md §10.38 + research/40): `vrserver.txt` shows `[Remapping] Beginning remapping from knuckles to oculus_touch` followed by `[Remapping] Skipped remapping of path::mode: /user/hand/left/input/skeleton/left::` and the same for all 10 finger curl paths.  The skipped binding entries silently leave the action handles unbound (`bActive=False`, `activeOrigin=0x0`) — neither our app nor SteamVR Test Controller's legacy API notices.  Fix is to add an explicit `{"controller_type": "oculus_touch", "binding_url": "bindings_oculus_touch.json"}` entry to `actions.json::default_bindings` and ship a minimal `bindings_oculus_touch.json` with only the trigger + grip sources (skeleton + finger paths simply don't exist on oculus_touch profile and SHOULD be omitted, not auto-remapped).  Companion change: `bindings_index.json` grip `mode: force_sensor → trigger` (auto-remap was already converting it; explicit form makes the binding plain).  vrserver.txt should now show `Successfully loaded binding file 'bindings_oculus_touch.json'` instead of the SKIP cascade.

26. **UDCAP UI "Working" status + `Gloves are working` ≠ finger pose pipeline working** (diagnosed in 9.30, see memory.md §10.38 + research/41): The UDCAP UI's top-half hand capture preview (the photographic gloves with overlaid hand model) is the FIRST consumer of the driver's finger pose computation — VMC OSC broadcast, SteamVR knuckles virtual controller, and our app's action handles all consume the SAME upstream output.  When the UI's preview does NOT animate as the user flexes fingers, the driver's `sensor raw → angle → hand pose` pipeline is broken, and ALL downstream consumers will report 0/idle regardless of any binding/calibration/SteamVR fix on our side.  Diagnostic separator: RF/battery/USB/dongle "Working" status in the UDCAP UI lower panel (FPS 90+, RSSI -55 to -60 dBm, battery >50%) only validates the heartbeat — payload validation requires the upper preview animating.  When the upper preview is frozen, candidate causes (priority order): (1) glove firmware ↔ Driver v0.1.8.x version mismatch (most likely after the 1-year jump from v0.1.3 to v0.1.8), (2) Driver v0.1.8.x regression in the finger pose pipeline, (3) physical sensor failure (less likely on both gloves simultaneously), (4) calibration data corruption, (5) USB driver / RF dongle packet drop.  This problem is OUT OF SCOPE for our codebase fixes — escalate to UDexREAL support, try Driver downgrade (v0.1.8.x → v0.1.7.x), or pivot to `ust_hm_grip` (gripper) which doesn't depend on glove finger sensors.

27. **`controller_type`-mismatch trap — `default_bindings` must enumerate every controller_type a driver may report, otherwise SteamVR silently loads NO binding and Action API returns 0 for every action** (recurrence #3, fixed across multiple iterations: §10.14 udcap, §10.40 oculus_touch grip-mode, §10.42 pico_controller; gotcha consolidated in 9.34, see memory.md §10.42): When SteamVR sees a controller whose `Prop_ControllerType_String` doesn't match any `default_bindings[i].controller_type` entry in our `actions.json`, **it silently uses no binding at all** — `getAnalogActionData` / `getDigitalActionData` return 0 / False forever.  No log warning, no fallback.  Each new streaming-layer driver tends to use a fresh string: UDCAP gloves report `udcap` (§10.14), Virtual Desktop's Oculus-Touch emulation reports `oculus_touch` (§10.40), prism (Pico Connect) on PICO 4 Ultra reports `pico_controller` (§10.42, distinct from `pico_neo3_controller` for older PICO Neo 3 hardware).  Mitigation: maintain a generous `default_bindings` list covering all known PICO/Touch/Knuckles variants (current set: `pico_controller`, `pico_phoenix_controller`, `pico4_controller`, `pico_neo3_controller`, `oculus_touch`, `knuckles`).  9.34 added an unknown-controller-type warning to `ust_hm_grip/scripts/diagnose_controller_raw.py` that prints the exact `default_bindings` line to add when an unrecognised type is detected — should catch any 4th-time recurrence within seconds rather than days.

28. **OpenVR binding `mode: force_sensor` requires the underlying controller profile to expose `/input/grip/force` — NOT all PICO/Touch profiles do** (fixed in 9.32, see memory.md §10.40): `mode: force_sensor` with input `force` reads the controller's `/input/grip/force` sub-component.  Knuckles + `pico_neo3_controller` expose this natively, but **VD's Oculus-Touch emulation only exposes `/input/grip/value`** (analog 0..1) and the documented "auto-fallback to grip click → force ≈ 1.0" is unreliable when the click sub-component isn't emitted either.  Symptom: grip works under Steam Link / Pico Connect (PICO Neo 3 native force) but is silent under Virtual Desktop streaming (force channel absent).  This causes streaming-layer-asymmetric grip behaviour that's invisible until the user switches streaming method.  Universal fix: bind grip with `mode: trigger` + `pull` (reads `/input/grip/value`, present on all standard controller profiles).  Identical hysteresis behaviour, vector1 [0, 1] output, no profile dependency.  Applied uniformly in `ust_hm_grip/config/openvr_actions/bindings_pico.json` so SteamVR loads the same binding for any of the 6 controller_type entries (gotcha #27).

29. **SteamVR Add-On labels `pico` and `prism` are NOT what they sound like — `pico` = PICO Connect (PICO Inc.), `prism` = Steam Link (Valve)** (clarified in 9.35 after years of mislabel propagation through memory.md §3.13 / §10.14 / §10.41, see memory.md §10.43): Both add-ons expose a PICO HMD to SteamVR but via different streaming clients.  `pico` is the **external** driver registered by Pico Connect (`C:/Program Files/PICO Connect/openvr_driver/` per `openvrpaths.vrpath external_drivers`).  `prism` is **bundled with SteamVR itself** at `C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/prism/` — Valve's officially-shipped driver that powers the Steam Link app on the headset.  Earlier memory entries assumed `prism` was Pico Connect because Pico Connect's executable used to be named `prism.exe` — that was a name collision, NOT a shared identity.  Both drivers have `redirectsDisplay: true` + HMD provider — **simultaneous activation causes HMD double-emit conflict**, undefined which driver "wins" controller registration.  Operational rule: enable EXACTLY ONE of `pico` / `prism` / `Virtual Desktop Streamer (Quest)` at any time.  Selection criterion: `pico` when PICO body tracking (Forearm Tracking Enhanced / AI Body Tracking / Pico Motion Tracker pucks) is needed, else `prism` (Valve-stable, fewer dependencies).  controller_type maps consistently to `pico_controller` / `oculus_touch` (both covered by 9.34 default_bindings).

### ust_hm_grip: GR1T2 + 2-Finger Parallel Gripper, PICO Touch Controllers (9.36 unified)

Replaces the 22-DoF Fourier hand with a 2-DoF parallel gripper per side.  The user holds **PICO Touch controllers directly** (NOT UDCAP gloves) and the controller's grip Pull (default 9.28+) drives gripper open/close.

**Pre-9.36 lineage** (memory.md §10.44): `ust_hm_grip/` was `ust_260504_win/` (Option B gripper migration sub-project).  9.36 renamed and copied `vr_sampler.py` + `coord_transforms.py` from the legacy shared layer (`ust_260418_win/teleop/`) into the package so the controller-grip track is **fully self-contained** — no cross-package imports.

#### File Map (relative to `ust_ws/ust_hm_grip/`)

| File | Purpose |
|------|---------|
| `isaac_file/build_gripper_usd.py` | Materialise `GR1T2_with_gripper.usd` from the stock GR1T2 USD: strips L_*/R_* Fourier hand prims, attaches a base+2-prismatic-finger chain to each `*_wrist_pitch_link`.  Boots Isaac Sim headless via `_boot_isaac_sim()` so `pxr` schemas register (gotcha #23).  `try/finally: sim_app.close()` to prevent lock leaks (#14/#19) |
| `teleop/gr1t2_gripper_retargeter.py` | `GR1T2GripperSteamVRRetargeter` — 16D Pink IK action (14 EEF + 2 gripper).  Hysteresis on `gripper_close_threshold=0.6` / `gripper_open_threshold=0.4`. **`gripper_signal_source: str = "grip"`** (default 9.28) selects which controller input drives close: `"grip"` (default), `"trigger"`, or `"both"` (logical OR).  Diagnostic log tag adapts to source (`max_grip` / `max_trig` / `max_either`) |
| `teleop/gr1t2_gripper_device.py` | `GR1T2GripperDevice` + `GR1T2GripperDeviceCfg` — DeviceBase wrapper.  `_probe_action_values()` and `_log_first_advance()` print "Squeeze the {grip\|trigger}" depending on `gripper_signal_source`.  WARN block on zero-input lists "Save Personal Binding" missed step as #1 suspect (binding editor change is easy to forget to commit) |
| `scripts/run_teleop.py` | Main teleop entry.  CLI `--gripper_signal_source {grip,trigger,both}` (default `grip`), `--use_grip_as_close` (deprecated, kept for compat), `--gripper_close_threshold` / `--gripper_open_threshold`, `--render_interval`, `--process_priority`. **9.37**: `--vr_runtime {auto,pico_connect,virtual_desktop,steamvr_native}` (default `auto`).  When `pico_connect`, swaps `tracker_binding_json` to `config/tracker_binding_pico_connect.json` and prints the recommended SteamVR Add-On layout (prism ON / VD OFF / udcap OFF) at startup |
| `scripts/diagnose_pico_connect.py` | **9.37 신규** — 6-layer probe for the `PICO Connect → SteamVR → PC → Isaac Lab` pipeline.  Layers: (1) PICO Connect Streaming Service process, (2) SteamVR drivers via vrpathreg (prism/VD/udcap), (3-5) OpenVR HMD + tracker + controller inventory + PICO classification, (6) `tracker_binding_pico_connect.json` placeholder/role validation.  Fails with actionable next-step text per layer.  `--json` for machine-readable output |
| `scripts/enumerate_trackers.py` | **9.37 신규** — auto-detects PICO Motion Trackers (PMT_/PICOBT_ serials, manufacturer/model heuristics) and emits a populated `tracker_binding_pico_connect.json`.  Tags PICO trackers `TODO_pico` (user must inspect physical mounting); auto-maps VD body segments to grip-track roles (waist / *_forearm; legs role="" since unused).  Detects mixed PICO+VD setup and warns to pick one pipeline.  Pass `--out PATH` to overwrite the template in place |
| `config/tracker_binding_pico_connect.json` | **9.37 신규** — PICO Connect tracker template with `PMT_REPLACE_ME_*` placeholders for waist + 2 forearm + 2 ankle slots.  Forearm slots are STRICTLY required (wrist-EEF fallback when controller pose is briefly unavailable); waist OPTIONAL (only consumed by `WaistEnabled` env variant); ankle slots present for symmetry with `ust_hm_glove` but role="" by default since the grip retargeter ignores lower-body segments |
| `scripts/diagnose_gripper.py` | Layer-by-layer diagnostic — uses the SAME action-API path as `run_teleop` so a 0 here matches what teleop would see.  CLI `--signal-source {grip,trigger,both}` (default `grip`), `--seconds N`.  WARN block on zero input enumerates 6 ordered causes ending with "Save Personal Binding never clicked" |
| `scripts/diagnose_controller_raw.py` | **Binding-bypass probe** — calls `IVRSystem::getControllerState()` directly, bypassing Action Manifest entirely.  If THIS is zero too, the issue is upstream of SteamVR (PICO controllers off / VD not forwarding buttons / etc.) — see gotcha #24 + the recovery checklist printed at the bottom of the script.  Distinguishes "controller hardware/driver problem" from "action manifest binding not applied" definitively |
| `tests/test_gripper_retargeter.py` | 22 tests (5 legacy trigger-driven tests opt into `gripper_signal_source="trigger"` explicitly; 6 new tests cover the source-selection field across all 3 modes + default + unknown-fallback) |
| `config/openvr_actions/{actions,bindings_pico,manifest}.{json,vrmanifest}` | OpenVR action manifest with `trigger_left/right`, `grip_left/right`, `menu_left/right` analog/digital actions.  9.34 `default_bindings` covers 6 controller_type values: `pico_controller` / `pico_phoenix_controller` / `pico4_controller` / `pico_neo3_controller` / `oculus_touch` / `knuckles` (memory.md §10.42).  9.32 grip mode = `trigger` + `pull` (NOT `force_sensor`) for streaming-layer-universal `/input/grip/value` channel (§10.40).  At runtime device generates `manifest.runtime.vrmanifest` with `binary_path_windows` = `sys.executable` (same pattern reused by `ust_hm_glove`) |

#### CLI Quickstart (PowerShell)

```powershell
# 0) Build / refresh the gripper USD (requires Isaac Sim boot for pxr schemas)
./isaaclab.bat -p ust_ws/ust_hm_grip/isaac_file/build_gripper_usd.py

# 0a) [9.37 PICO Connect users] populate tracker_binding template once per rig
$env:PYTHONPATH = "."
python -X utf8 -m ust_ws.ust_hm_grip.scripts.enumerate_trackers `
    --out ust_ws/ust_hm_grip/config/tracker_binding_pico_connect.json

# 1) Layered diagnosis when gripper close/open isn't responding
# Layer 0 -- PICO Connect pipeline (only when --vr_runtime pico_connect)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_pico_connect
# Layer 1 -- raw OpenVR (bypasses our action manifest entirely)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_controller_raw
# Layer 2 -- action-API path (same code path as run_teleop)
python -X utf8 -m ust_ws.ust_hm_grip.scripts.diagnose_gripper
# Layer 3 -- full teleop loop in monitor mode
python -X utf8 -m ust_ws.ust_hm_grip.scripts.run_teleop `
    --env_variant robot_only --render_mode monitor `
    --vr_runtime pico_connect `
    --gripper_signal_source grip --process_priority high
```

#### Critical Gotchas (gripper-specific)

* `gripper_signal_source` default is **`"grip"`** in 9.28 (was an implicit OR via `use_grip_as_close=True` in 9.27).  Set explicitly when behaviour matters.
* The OpenVR action handle path uses `/actions/teleop/in/grip_{left,right}` and `/actions/teleop/in/trigger_{left,right}` — both must be bound to controller "Pull" (NOT "Use as Trigger" / "Use as Force Sensor") in the SteamVR binding editor for the action manifest to receive analog values.
* SteamVR Binding Editor's **"Save Personal Binding"** button at the bottom is mandatory and easy to miss; editing without saving silently discards changes on next launch.
* If `diagnose_controller_raw.py` shows all-zero AND inventory serials match `<HMD>_Controller_*`, no amount of binding work helps — see gotcha #24.

### CloudXR VR Integration

- **Runtime**: CloudXR Early Access 6.0.1-webrtc Docker image
- **Signaling port**: 49100 (Runtime 6.0.1), NOT 48010 (legacy 5.0.1)
- **Quest 3S flow**: Quest Browser -> HTTPS:8080 -> CloudXR.js -> WSS:48322 (HAProxy) -> WS:49100 (Runtime)
- **Web client**: `cloudxr_js/isaac/` (TypeScript, webpack, `npm run dev-server:https`)
- **SSL proxy**: `cloudxr_js/proxy/Dockerfile` (HAProxy, WSS:48322 -> WS:49100)
- **Docker Compose**: `ust_260207/docker-compose.cloudxr-ust.patch.yaml`

**Required env vars** (set via `setup_cloudxr_env.sh` or Docker Compose):
```
XR_RUNTIME_JSON=/workspace/isaaclab/ust_ws/openxr/share/openxr/1/openxr_cloudxr.json
XDG_RUNTIME_DIR=/workspace/isaaclab/ust_ws/openxr/run
IPC_IGNORE_VERSION=1
```

**Kit XR settings** (in `.kit` file):
- `app.asyncRendering=true`, `app.xr.enabled=true`, `xr.profile.ar.enabled=true`
- `persistent.xr.system.openxr.runtime="system"` (reads `XR_RUNTIME_JSON`)

### Supporting Projects

| Project | Path | Description |
|---------|------|-------------|
| LLM Control | `ust_ws/LLM/` | Natural language robot control (GPT-4/Claude API, FastAPI, 5-layer safety validation) |
| OpenVLA | `ust_ws/openvla/` | Vision-Language-Action model (Prismatic framework, finetune/deploy scripts) |
| VLM Models | `ust_ws/models/` | Qwen3-VL-8B/32B, SigLIP2-so400m, Florence2-large |
| ROS2 Sensors | `ust_ws/packages/` | Livox LiDAR simulator, RealSense ROS2 driver |
| MuJoCo Assets | `ust_ws/robotis_mujoco_menagerie/` | OpenMANIPULATOR-X and TurtleBot3 MJCF models |

### Critical Gotchas

1. **`IPC_IGNORE_VERSION=1`** must be set BEFORE `AppLauncher` import (run_teleop.py does this automatically). Required for CloudXR 5.0.1/6.0.1 mismatch.
2. **Empty `SdfTokenListOp()`** on apiSchemas blocks inherited physics APIs across ALL referenced sub-assemblies (gripper joints, livox bodies, realsense bodies). Fix: `prim.ClearInfo("apiSchemas")`.
3. **Gripper sign convention**: `localRot0=(0,1,0,0)` (180 deg X rotation) already handles URDF `axis=(0,-1,0)` inversion. Both fingers use SAME sign: `+value=open, -value=close`. Do NOT double-invert right finger values.
4. **`PhysicsDriveAPI:linear`** required on gripper joints (not `PhysxMimicJointAPI:transY` which doesn't work at runtime with Pixar USD lib).
5. **Visual meshes** must be children of `RigidBody` prims (not siblings) to follow physics simulation. Use `Sdf.CopySpec` + delete for reparenting.
6. **RayCaster/sensor `prim_path`** must point to a `RigidBody` prim, not a parent Xform.
7. **Se3Keyboard** requires GUI window (`omni.appwindow`), does NOT work in `--headless` mode.
8. **Kit `user.config.json`** persistent settings override `.kit` file `persistent.*` settings.
9. **Wheel rotation**: positive Y-axis wheel rotation = FORWARD (tested empirically). Previous `scale=-1.0` was wrong.

### Code Style Notes

- Code comments in Korean (한국어), class/function names in English
- All config classes use `@configclass` decorator (Isaac Lab convention)
- Import ordering follows Isaac Lab's isort profile (see pyproject.toml)
- Project path added to `sys.path` at script entry points

### Research Documentation Index

| Location | Contents |
|----------|----------|
| `ust_ws/research/` | 35+ architecture/design research guides — Isaac Lab stack setup, UST architecture, humanoid VR teleop, G1 scene design, **+ 9.23~9.30 fix supplements**: `28. cross_simulator_teleop_data_pipeline_research.md` (cross-sim 비교), `29. isaaclab_framerate_vs_newton_backend_comparison.md` (Newton vs PhysX), `30. hardware_specific_framerate_analysis.md` (RTX PRO 6000 + 7950X3D 정량 분석), `31. cpu_upgrade_roi_7950x3d_to_9950x3d_or_x3d2_analysis.md` (CPU 업그레이드 ROI), `32. fourier_hand_mapper_parameter_tuning_guide.md` (scale/rest cal/EMA 파라미터 튜닝 가이드), `33. realtime_finger_tracking_latency_root_cause_and_optimization.md` (9.27 lag 5-cause 분석 + Phase A/B/C/D ROI matrix), `34. udcap_udexreal_official_docs_deep_dive` (UDCAP official docs + Protobuf path), `35. udcap_udexreal_complete_documentation_reference`, `36. gripper_elbow_tracker_pico_controller_migration_design_guide` (9.28 gripper subproject 설계), `37. steamvr_direct_input_vs_vmc_path_recommendation` (Phase X1/X2/X3 결정 트리), `38. udcap_oculus_driver_conflict_pico_controller_recovery` (Manage Add-Ons 4단계 모델), `39. udcap_vd_quest_coexistence_controller_override_recovery` (Plan A/B 정정), `40. udcap_skeletal_binding_remap_root_cause_and_fix` (9.29 oculus_touch binding 추가), `41. udcap_glove_sensor_pipeline_failure_and_gripper_pivot_decision` (9.30 UDCAP UI hand capture 미동작 진단 + gripper pivot) |
| `ust_ws/cloudxr_research/` | CloudXR architecture, VLA research, development specs. **`all_dev.md`** is the master specification referenced throughout |
| `ust_ws/documents/` | LLM robot control system: architecture, core modules, Isaac Sim interface, LLM integration, web UI, safety system, API spec |
| `ust_ws/ust_260207/*.md` | 8 execution/troubleshooting guides: VR setup, CloudXR.js, Quest 3S crash analysis, imitation learning pipeline |
| `ust_ws/claudedocs/` | 8 HRI paper summaries (speech/gesture generation, social robots, elderly care evaluation) |

## Working History Convention

Two files together capture the workspace state.  **Update both** whenever a non-trivial change lands:

| File | Role | When to update |
|------|------|----------------|
| `CLAUDE.md` (this file) | **Static architecture reference** — directory map, file map, gotchas, conventions, gym IDs, cfg classes | When a *new module / file / cfg class / gotcha / convention* is introduced.  Keep stable / structural facts here.  Aim for "everything a fresh agent needs to orient itself." |
| `memory.md` | **Chronological fix history** — numbered fix log (currently at **v9.36 in §10.44**) with root cause / change / verification per entry | After *every numbered fix* (v9.x or v10.x).  Append a new `### 10.NN YYYY-MM-DD 9.MM차 — <title>` section. Includes test counts (pytest N/N + smoke 7/7), recommended next command, expected results, user-facing checklist. |

### Update workflow when shipping a fix

1. Apply code changes
2. Run tests — `pytest ust_ws/ust_hm_glove/tests/` + `python ust_ws/ust_hm_glove/scripts/smoke_test.py`
3. Append `### 10.NN ...` section to `memory.md` (at the bottom, before the "마지막 업데이트" line)
4. If the fix introduced a new module, file, gotcha, cfg field, gym ID, or CLI flag → update the relevant section of `CLAUDE.md`
5. Update the "마지막 업데이트" line in `memory.md` with the version + one-line summary

### Fix-history numbering scheme

- **9.x** — currently active series.  9.13~9.27 cover the legacy `ust_fourier_260421` (now `ust_hm_glove`) debugging (Pico+UDCAP+VD finger setup).  **9.28** opens the `ust_260504_win` (now `ust_hm_grip`) gripper sub-project series (PICO Touch controller + 2-finger gripper, UDCAP optional / typically off).  **9.37 (grip)**: ports the PICO Connect → SteamVR → PC → Isaac Lab pipeline from `ust_hm_glove` into `ust_hm_grip` — adds `--vr_runtime {auto,pico_connect,virtual_desktop,steamvr_native}` CLI flag in `run_teleop.py`, ships `config/tracker_binding_pico_connect.json` template (PMT_* placeholders for waist + 2 forearm + 2 ankle slots), adds `scripts/enumerate_trackers.py` (auto-detect PICO trackers, tag TODO_pico) + `scripts/diagnose_pico_connect.py` (6-layer process/driver/openvr/binding probe).  `--vr_runtime pico_connect` swaps the tracker_binding template automatically and prints the recommended Add-On layout at startup (prism ON / VD OFF / udcap OFF).  Default remains `auto` for backward compatibility.  **9.29** fixes the SteamVR knuckles→oculus_touch auto-remap that silently dropped skeleton + 10 finger-curl bindings (gotcha #25 / research/40).  **9.30** diagnoses the UDCAP v0.1.8.2 sensor → finger pose pipeline failure (UI hand capture preview frozen — gotcha #26 / research/41) and decides to PIVOT to gripper for manipulation tracks while finger control is shelved pending UDCAP firmware/driver/hardware investigation.  **9.31~9.36 series** (memory.md §10.39-§10.44) hardens the controller-grip diagnostic + binding stack: **9.31** fixes the `diagnose_controller_raw.py` legacy-API-only false negative (PICO/VD-Touch only populates Action API path, gotcha #27); **9.32** changes grip binding from `force_sensor`/`force` to `trigger`/`pull` so the universal `/input/grip/value` channel works under VD's Oculus-Touch emulation (gotcha #28); **9.33** decommissions VD as required dependency; **9.34** adds `pico_controller` (and PICO 4 variants) to `default_bindings` after prism on PICO 4 Ultra reported a controller_type missing from our list (controller_type-mismatch trap recurrence #3, gotcha #27); **9.35** corrects the `prism` ↔ `pico` SteamVR Add-On labeling (`prism`=Steam Link/Valve, `pico`=PICO Connect/PICO Inc.) and decides PICO Connect single-active for body-tracking access (gotcha #29); **9.36** unifies the four legacy directories (`ust_260418_win` + `ust_fourier_260421` + `ust_260502_win` + `ust_260504_win`) into the two paradigm-based packages `ust_hm_glove` + `ust_hm_grip`, self-contained with cross-package import dependency 0.  Latest: **9.36** (directory unification, code 0 lines changed, 346 mechanical edits across 56 files).  Recent fourier-series milestones: 9.27 (finger lag 5-cause fix), 9.26 (`disable_arm_tracking=True` semantic correction), 9.25 (phantom-tracker fix + `--ignore_trackers` v1), 9.24 (env step 20→120 Hz), 9.23 (retargeter 22D EMA low-pass).  이전 milestones: 9.21 (replay loop wrap-clamp + Omniverse lock 진단), 9.22 (lock-leak 무한루프 + `-X utf8` 표준화), 9.19 (C8 thumb yaw axis), 9.20 (C10 URDF clamp), 9.18 (HeadEstimator), 9.15 (thumb yaw midpoint), 9.14 (per-bone REST POSE cal), 9.13 (VMC always-on default).
- **10.x** — section number inside `memory.md` (e.g. `§10.44` for 9.36 directory unification, `§10.43` for 9.35 PICO Connect single-active, `§10.42` for 9.34 pico_controller default_bindings fix, `§10.41` for 9.33 streaming-layer flexibility, `§10.40` for 9.32 grip binding mode, `§10.39` for 9.31 diagnose probe rewrite, `§10.38` for 9.29 + 9.30 joint, `§10.37` for 9.28, `§10.34` for 9.26, `§10.33` for 9.25, `§10.32` for 9.24, `§10.31` for 9.23)
- New fix → bump 9.x AND add §10.x+1 at the same time
