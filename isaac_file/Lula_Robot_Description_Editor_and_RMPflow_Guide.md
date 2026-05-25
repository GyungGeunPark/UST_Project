
# Lula Robot Description Editor 및 RMPflow 설정 완벽 가이드

이 문서는 Isaac Sim에서 Lula Robot Description Editor를 사용하여 생성한 robot_description.yaml 파일을 로봇에 적용하고, RMPflow를 설정하는 방법을 상세히 안내합니다.

---

## 목차

1. [개요](#1-개요)
2. [사전 준비](#2-사전-준비)
3. [RMPflow에 필요한 파일](#3-rmpflow에-필요한-파일)
4. [RMPflow 설정 파일 작성](#4-rmpflow-설정-파일-작성)
5. [Python으로 RMPflow 적용하기](#5-python으로-rmpflow-적용하기)
6. [지원 로봇 빠른 설정](#6-지원-로봇-빠른-설정)
7. [커스텀 로봇 RMPflow 적용](#7-커스텀-로봇-rmpflow-적용)
8. [장애물 회피 설정](#8-장애물-회피-설정)
9. [디버깅 및 시각화](#9-디버깅-및-시각화)
10. [RMPflow 튜닝 가이드](#10-rmpflow-튜닝-가이드)
11. [전체 코드 예제](#11-전체-코드-예제)
12. [문제 해결](#12-문제-해결)
13. [참고 자료](#13-참고-자료)

---

## 1. 개요

### 1.1 RMPflow란?

RMPflow(Riemannian Motion Policy flow)는 **기하학적으로 일관된 리만 운동 정책 변환에 기반한 정책 합성 알고리즘**입니다. 다음과 같은 기능을 제공합니다:

- **목표 추적**: End-effector를 목표 위치/방향으로 이동
- **장애물 회피**: 실시간으로 동적 장애물 회피
- **관절 한계 준수**: 관절 위치, 속도 한계 내에서 동작
- **자체 충돌 회피**: 로봇 자체 링크 간 충돌 방지

### 1.2 워크플로우 요약

```
┌─────────────────────────────────────────────────────────────┐
│                      RMPflow 워크플로우                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. URDF 파일 준비                                          │
│         ↓                                                   │
│  2. Lula Robot Description Editor로 robot_descriptor.yaml   │
│         ↓                                                   │
│  3. rmpflow_config.yaml 작성 (또는 템플릿 사용)              │
│         ↓                                                   │
│  4. Python에서 RMPflow 초기화                               │
│         ↓                                                   │
│  5. ArticulationMotionPolicy로 로봇과 연결                  │
│         ↓                                                   │
│  6. 시뮬레이션 루프에서 동작 생성                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 사전 준비

### 2.1 필요한 파일 확인

RMPflow를 적용하기 전에 다음 파일들이 준비되어야 합니다:

| 파일 | 설명 | 생성 방법 |
|------|------|----------|
| `robot.urdf` | 로봇 운동학 정의 | USD to URDF Exporter 또는 직접 작성 |
| `robot_descriptor.yaml` | 로봇 설명 파일 | Lula Robot Description Editor |
| `rmpflow_config.yaml` | RMPflow 파라미터 | 템플릿 복사 후 수정 |

### 2.2 Extension 활성화

```
Window > Extensions에서 다음 Extension 활성화:
- Isaac Sim Motion Generation
- Isaac Sim Lula Extension
```

---

## 3. RMPflow에 필요한 파일

### 3.1 URDF 파일

로봇의 운동학 정보를 담고 있는 파일입니다.

**필수 요소:**
- 관절 이름 (Joint names)
- 링크 이름 (Link names)
- 관절 위치 한계 (Position limits)

**선택 요소 (무시됨):**
- 질량, 관성 모멘트
- Visual/Collision 메시

### 3.2 Robot Descriptor YAML 파일

Lula Robot Description Editor에서 생성한 파일입니다.

**파일 구조 예시:**

```yaml
# robot_descriptor.yaml

# URDF 파일 경로 (선택사항 - RMPflow 초기화 시 별도 지정 가능)
urdf_path: "my_robot.urdf"

# Configuration Space (제어 공간) 정의
cspace:
  - joint1    # Active Joint 1
  - joint2    # Active Joint 2
  - joint3    # Active Joint 3
  - joint4    # Active Joint 4
  - joint5    # Active Joint 5
  - joint6    # Active Joint 6

# 기본 관절 설정 (라디안)
default_q:
  - 0.0       # joint1 기본값
  - -1.0      # joint2 기본값
  - 0.0       # joint3 기본값
  - -2.2      # joint4 기본값
  - 0.0       # joint5 기본값
  - 2.4       # joint6 기본값

# 충돌 구체 정의
collision_spheres:
  - link: base_link
    center: [0.0, 0.0, 0.05]
    radius: 0.06
  - link: link1
    center: [0.0, 0.0, 0.0]
    radius: 0.05
  - link: link2
    center: [0.0, 0.0, 0.1]
    radius: 0.04
  # ... 모든 링크에 대해 반복

# 고정 관절 (그리퍼 등)
fixed_joints:
  - name: gripper_joint
    value: 0.04  # 고정 위치
```

### 3.3 RMPflow Config YAML 파일

RMPflow 알고리즘의 동작을 제어하는 파라미터 파일입니다.

---

## 4. RMPflow 설정 파일 작성

### 4.1 rmpflow_config.yaml 구조

```yaml
# rmpflow_config.yaml

# ============================================
# 관절 한계 버퍼
# ============================================
joint_limit_buffers: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
# 각 관절의 한계에서 얼마나 떨어져야 하는지 (라디안)

# ============================================
# RMP 파라미터 (선택사항 - 기본값 사용 가능)
# ============================================
rmp_params:
  # 목표 추적 RMP
  cspace_target_rmp:
    metric_scalar: 50.0
    position_gain: 100.0
    damping_gain: 50.0
    robust_position_term_thresh: 0.5
    inertia: 1.0

  # 관절 한계 회피 RMP
  joint_limit_rmp:
    metric_scalar: 1000.0
    metric_length_scale: 0.01
    metric_exploder_eps: 1e-3
    metric_velocity_gate_length_scale: 0.01
    accel_damper_gain: 200.0
    accel_potential_gain: 1.0
    accel_potential_exploder_length_scale: 0.1
    accel_potential_exploder_eps: 1e-2

  # 충돌 회피 RMP
  collision_rmp:
    damping_gain: 50.0
    damping_std_dev: 0.04
    damping_robustness_eps: 1e-2
    damping_velocity_gate_length_scale: 0.01
    repulsion_gain: 800.0
    repulsion_std_dev: 0.01
    metric_modulation_radius: 0.5
    metric_scalar: 10000.0
    metric_exploder_std_dev: 0.02
    metric_exploder_eps: 0.001

  # 관절 속도 제한 RMP
  joint_velocity_cap_rmp:
    max_velocity: 1.0  # rad/s
    velocity_damping_region: 0.3
    damping_gain: 1000.0
    metric_weight: 100.0

# ============================================
# 자체 충돌 회피용 바디 실린더
# ============================================
body_cylinders:
  - name: base_cylinder
    pt1: [0.0, 0.0, 0.0]
    pt2: [0.0, 0.0, 0.3]
    radius: 0.15

# ============================================
# 바디 충돌 컨트롤러 (End-effector 보호)
# ============================================
body_collision_controllers:
  - name: gripper_collision
    frame_name: tool0
    radius: 0.05
    collision_controllers:
      - base_cylinder
```

### 4.2 템플릿 파일 위치

Isaac Sim에는 사전 설정된 템플릿 파일이 포함되어 있습니다:

```python
from omni.isaac.core.utils.extensions import get_extension_path_from_name
import os

mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

# 예시 경로:
# - Franka: {rmp_config_dir}/franka/rmpflow/franka_rmpflow_common.yaml
# - UR10:   {rmp_config_dir}/ur10/rmpflow/ur10_rmpflow_common.yaml
```

### 4.3 커스텀 로봇용 템플릿 생성

**권장 방법:** 비슷한 구조의 로봇 템플릿을 복사하여 수정

```bash
# 6-DOF 로봇: UR10 템플릿 기반
cp ur10_rmpflow_common.yaml my_robot_rmpflow.yaml

# 7-DOF 로봇: Franka 템플릿 기반
cp franka_rmpflow_common.yaml my_robot_rmpflow.yaml
```

---

## 5. Python으로 RMPflow 적용하기

### 5.1 기본 구조

```python
from isaacsim.robot_motion.motion_generation import RmpFlow
from isaacsim.robot_motion.motion_generation import ArticulationMotionPolicy
from isaacsim.core.prims import SingleArticulation as Articulation
```

### 5.2 RMPflow 초기화

```python
# RMPflow 인스턴스 생성
rmpflow = RmpFlow(
    robot_description_path="/path/to/robot_descriptor.yaml",
    urdf_path="/path/to/robot.urdf",
    rmpflow_config_path="/path/to/rmpflow_config.yaml",
    end_effector_frame_name="tool0",  # URDF에 정의된 End-effector 프레임 이름
    maximum_substep_size=0.00334       # 시뮬레이션 서브스텝 크기
)
```

**파라미터 설명:**

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `robot_description_path` | robot_descriptor.yaml 파일 경로 | 필수 |
| `urdf_path` | URDF 파일 경로 | 필수 |
| `rmpflow_config_path` | RMPflow 설정 파일 경로 | 필수 |
| `end_effector_frame_name` | End-effector 프레임 이름 | 필수 |
| `maximum_substep_size` | 최대 서브스텝 크기 (초) | 0.00334 |

### 5.3 ArticulationMotionPolicy 연결

```python
# Articulation 가져오기
articulation = Articulation("/World/my_robot")

# RMPflow와 Articulation 연결
articulation_rmpflow = ArticulationMotionPolicy(
    articulation,
    rmpflow,
    default_physics_dt=1/60.0  # 시뮬레이션 물리 timestep
)
```

### 5.4 시뮬레이션 루프에서 사용

```python
import numpy as np

def physics_step(step_size):
    # 1. 목표 위치/방향 설정
    target_position = np.array([0.5, 0.0, 0.5])  # 미터
    target_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # 쿼터니언 (w, x, y, z)

    rmpflow.set_end_effector_target(
        target_position=target_position,
        target_orientation=target_orientation
    )

    # 2. 월드 상태 업데이트 (장애물 위치 등)
    rmpflow.update_world()

    # 3. 로봇 베이스 위치 업데이트 (이동 베이스인 경우)
    base_translation, base_orientation = articulation.get_world_pose()
    rmpflow.set_robot_base_pose(
        translation=base_translation,
        orientation=base_orientation
    )

    # 4. 다음 동작 계산
    action = articulation_rmpflow.get_next_articulation_action(step_size)

    # 5. 로봇에 동작 적용
    articulation.apply_action(action)
```

---

## 6. 지원 로봇 빠른 설정

### 6.1 지원 로봇 목록

Isaac Sim에서 기본 지원하는 로봇들은 설정 파일 없이 바로 사용 가능합니다:

| 제조사 | 로봇 모델 |
|--------|----------|
| **Franka** | Panda |
| **Universal Robots** | UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e |
| **Flexiv** | Rizon4 |
| **Denso** | Cobotta Pro 900, Cobotta Pro 1300 |
| **Techman** | TM12 |
| **Kinova** | Gen3 |
| **Fanuc** | CRX10IAL |
| **Kawasaki** | RS007L, RS007N, RS013N, RS080N |
| **Kuka** | KR210 |

### 6.2 빠른 로딩 코드

```python
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    load_supported_motion_policy_config
)
from isaacsim.robot_motion.motion_generation import RmpFlow

# 지원 로봇의 설정을 자동으로 로드
rmp_config = load_supported_motion_policy_config(
    robot_name="Franka",      # 로봇 이름
    policy_name="RMPflow"     # 정책 이름
)

# RMPflow 인스턴스 생성
rmpflow = RmpFlow(**rmp_config)
```

### 6.3 Franka 로봇 전용 컨트롤러

```python
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import RMPFlowController

# Franka 로봇 생성
franka = Franka(
    prim_path="/World/Franka",
    name="franka_robot"
)

# RMPFlow 컨트롤러 생성
controller = RMPFlowController(
    name="rmpflow_controller",
    robot_articulation=franka
)

# 목표 위치로 이동
target_position = np.array([0.5, 0.0, 0.5])
target_orientation = np.array([1.0, 0.0, 0.0, 0.0])

actions = controller.forward(
    target_end_effector_position=target_position,
    target_end_effector_orientation=target_orientation
)

franka.apply_action(actions)
```

---

## 7. 커스텀 로봇 RMPflow 적용

### 7.1 전체 과정

```
1. URDF 파일 준비
       ↓
2. Lula Robot Description Editor에서 robot_descriptor.yaml 생성
       ↓
3. rmpflow_config.yaml 작성 (템플릿 복사 후 수정)
       ↓
4. 설정 파일 경로 지정하여 RMPflow 초기화
       ↓
5. 테스트 및 튜닝
```

### 7.2 단계별 가이드

#### Step 1: 설정 파일 디렉토리 구성

```
my_robot_config/
├── urdf/
│   └── my_robot.urdf
├── rmpflow/
│   ├── robot_descriptor.yaml
│   └── rmpflow_config.yaml
```

#### Step 2: RMPflow 초기화 코드

```python
import os
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy

# 설정 파일 경로
config_dir = "/path/to/my_robot_config"

rmpflow = RmpFlow(
    robot_description_path=os.path.join(config_dir, "rmpflow/robot_descriptor.yaml"),
    urdf_path=os.path.join(config_dir, "urdf/my_robot.urdf"),
    rmpflow_config_path=os.path.join(config_dir, "rmpflow/rmpflow_config.yaml"),
    end_effector_frame_name="tool0",
    maximum_substep_size=0.00334
)
```

#### Step 3: 전체 시뮬레이션 스크립트

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
import numpy as np
import os

# 월드 생성
world = World(stage_units_in_meters=1.0)

# 로봇 로드
from isaacsim.core.utils.stage import add_reference_to_stage
robot_prim_path = "/World/my_robot"
add_reference_to_stage(
    usd_path="/path/to/my_robot.usd",
    prim_path=robot_prim_path
)

# Articulation 생성
articulation = Articulation(robot_prim_path)
world.scene.add(articulation)

# 타겟 마커 생성
target = VisualCuboid(
    prim_path="/World/target",
    name="target",
    position=np.array([0.5, 0.0, 0.5]),
    size=0.05,
    color=np.array([1.0, 0.0, 0.0])
)

# RMPflow 초기화
config_dir = "/path/to/my_robot_config"
rmpflow = RmpFlow(
    robot_description_path=os.path.join(config_dir, "rmpflow/robot_descriptor.yaml"),
    urdf_path=os.path.join(config_dir, "urdf/my_robot.urdf"),
    rmpflow_config_path=os.path.join(config_dir, "rmpflow/rmpflow_config.yaml"),
    end_effector_frame_name="tool0",
    maximum_substep_size=0.00334
)

# 월드 리셋 및 Articulation 초기화
world.reset()
articulation.initialize()

# ArticulationMotionPolicy 생성
articulation_rmpflow = ArticulationMotionPolicy(
    articulation,
    rmpflow,
    default_physics_dt=1/60.0
)

# 시뮬레이션 루프
while simulation_app.is_running():
    world.step(render=True)

    if world.is_playing():
        # 타겟 위치 가져오기
        target_position, target_orientation = target.get_world_pose()

        # End-effector 타겟 설정
        rmpflow.set_end_effector_target(
            target_position=target_position,
            target_orientation=target_orientation
        )

        # 월드 상태 업데이트
        rmpflow.update_world()

        # 로봇 베이스 위치 업데이트
        base_trans, base_rot = articulation.get_world_pose()
        rmpflow.set_robot_base_pose(base_trans, base_rot)

        # 동작 계산 및 적용
        action = articulation_rmpflow.get_next_articulation_action(1/60.0)
        articulation.apply_action(action)

simulation_app.close()
```

---

## 8. 장애물 회피 설정

### 8.1 장애물 추가

```python
from isaacsim.core.api.objects.cuboid import FixedCuboid
from isaacsim.core.api.objects.sphere import FixedSphere

# 큐브 장애물
cube_obstacle = FixedCuboid(
    prim_path="/World/obstacle_cube",
    name="obstacle_cube",
    position=np.array([0.4, 0.0, 0.3]),
    size=0.1,
    color=np.array([1.0, 0.0, 0.0])
)

# 구 장애물
sphere_obstacle = FixedSphere(
    prim_path="/World/obstacle_sphere",
    name="obstacle_sphere",
    position=np.array([0.3, 0.2, 0.4]),
    radius=0.05,
    color=np.array([0.0, 1.0, 0.0])
)

# RMPflow에 장애물 등록
rmpflow.add_obstacle(cube_obstacle)
rmpflow.add_obstacle(sphere_obstacle)
```

### 8.2 동적 장애물 처리

```python
def physics_step(step_size):
    # 장애물 위치가 변경된 경우 update_world() 호출
    rmpflow.update_world()

    # ... 나머지 동작 계산
```

### 8.3 장애물 제거

```python
# 특정 장애물 제거
rmpflow.remove_obstacle(cube_obstacle)

# 모든 장애물 제거
rmpflow.reset()
```

### 8.4 Ground Plane 설정

```python
# Ground를 장애물로 추가
from isaacsim.core.api.objects import GroundPlane

ground = GroundPlane(
    prim_path="/World/ground",
    z_position=0.0
)

rmpflow.add_obstacle(ground)
```

---

## 9. 디버깅 및 시각화

### 9.1 충돌 구체 시각화

```python
# 로봇의 충돌 구체를 시각적으로 표시
rmpflow.visualize_collision_spheres()

# 시각화 해제 (리셋 시 자동 해제됨)
rmpflow.reset()
```

### 9.2 상태 업데이트 무시 모드

시뮬레이터 지연 문제인지 RMPflow 알고리즘 문제인지 구분할 때 유용합니다.

```python
# 시뮬레이터 피드백 무시 (순수 RMPflow 성능 테스트)
rmpflow.set_ignore_state_updates(True)

# 정상 모드로 복귀
rmpflow.set_ignore_state_updates(False)
```

### 9.3 End-effector 위치 확인

```python
# 현재 End-effector 위치 가져오기
ee_position = rmpflow.get_end_effector_position()
ee_orientation = rmpflow.get_end_effector_orientation()

print(f"End-effector Position: {ee_position}")
print(f"End-effector Orientation: {ee_orientation}")
```

---

## 10. RMPflow 튜닝 가이드

### 10.1 핵심 파라미터

| 파라미터 | 위치 | 효과 |
|---------|------|------|
| `joint_limit_buffers` | 최상위 | 관절 한계에서의 안전 마진 |
| `position_gain` | cspace_target_rmp | 목표 추적 속도 (높을수록 빠름) |
| `damping_gain` | cspace_target_rmp | 목표 도달 시 진동 감소 |
| `repulsion_gain` | collision_rmp | 장애물 회피 강도 |
| `max_velocity` | joint_velocity_cap_rmp | 최대 관절 속도 |

### 10.2 일반적인 튜닝 시나리오

#### 로봇이 너무 느리게 움직임

```yaml
rmp_params:
  cspace_target_rmp:
    position_gain: 150.0  # 기본값 100.0에서 증가
```

#### 목표 도달 시 진동 발생

```yaml
rmp_params:
  cspace_target_rmp:
    damping_gain: 80.0  # 기본값 50.0에서 증가
```

#### 장애물 회피가 너무 공격적

```yaml
rmp_params:
  collision_rmp:
    repulsion_gain: 400.0  # 기본값 800.0에서 감소
```

#### 관절 속도가 너무 빠름

```yaml
rmp_params:
  joint_velocity_cap_rmp:
    max_velocity: 0.5  # 기본값 1.0에서 감소
```

### 10.3 로봇 스케일에 따른 조정

**작은 로봇 (< 0.5m 도달 거리):**
- `metric_length_scale` 값들을 작게 조정
- `repulsion_std_dev` 값 감소

**큰 로봇 (> 1.5m 도달 거리):**
- `metric_length_scale` 값들을 크게 조정
- `repulsion_std_dev` 값 증가

---

## 11. 전체 코드 예제

### 11.1 기본 RMPflow 예제 (커스텀 로봇)

```python
"""
커스텀 로봇에 RMPflow 적용 예제
"""
from isaacsim import SimulationApp

# SimulationApp 초기화
simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.api.objects.cuboid import VisualCuboid, FixedCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy


class RMPflowExample:
    def __init__(self):
        self.world = None
        self.robot = None
        self.target = None
        self.rmpflow = None
        self.articulation_policy = None

    def setup_scene(self):
        """씬 설정"""
        # 월드 생성
        self.world = World(stage_units_in_meters=1.0)

        # Ground Plane 추가
        self.world.scene.add_default_ground_plane()

        # 로봇 로드
        robot_prim_path = "/World/my_robot"
        add_reference_to_stage(
            usd_path="/path/to/my_robot.usd",
            prim_path=robot_prim_path
        )

        self.robot = Articulation(
            prim_path=robot_prim_path,
            name="my_robot"
        )
        self.world.scene.add(self.robot)

        # 타겟 마커 추가
        self.target = VisualCuboid(
            prim_path="/World/target",
            name="target",
            position=np.array([0.5, 0.0, 0.5]),
            size=0.05,
            color=np.array([1.0, 0.0, 0.0])
        )

        # 장애물 추가
        self.obstacle = FixedCuboid(
            prim_path="/World/obstacle",
            name="obstacle",
            position=np.array([0.4, 0.1, 0.3]),
            size=0.08,
            color=np.array([0.0, 0.0, 1.0])
        )

        return

    def setup_rmpflow(self):
        """RMPflow 설정"""
        config_dir = "/path/to/my_robot_config"

        # RMPflow 초기화
        self.rmpflow = RmpFlow(
            robot_description_path=os.path.join(
                config_dir, "rmpflow/robot_descriptor.yaml"
            ),
            urdf_path=os.path.join(
                config_dir, "urdf/my_robot.urdf"
            ),
            rmpflow_config_path=os.path.join(
                config_dir, "rmpflow/rmpflow_config.yaml"
            ),
            end_effector_frame_name="tool0",
            maximum_substep_size=0.00334
        )

        # 장애물 등록
        self.rmpflow.add_obstacle(self.obstacle)

        # ArticulationMotionPolicy 생성
        self.articulation_policy = ArticulationMotionPolicy(
            self.robot,
            self.rmpflow,
            default_physics_dt=1/60.0
        )

        # 충돌 구체 시각화 (디버깅용)
        # self.rmpflow.visualize_collision_spheres()

        return

    def physics_step(self, step_size):
        """물리 스텝 콜백"""
        # 타겟 위치 가져오기
        target_position, target_orientation = self.target.get_world_pose()

        # End-effector 타겟 설정
        self.rmpflow.set_end_effector_target(
            target_position=target_position,
            target_orientation=target_orientation
        )

        # 월드 상태 업데이트 (장애물 위치)
        self.rmpflow.update_world()

        # 로봇 베이스 위치 업데이트
        base_translation, base_orientation = self.robot.get_world_pose()
        self.rmpflow.set_robot_base_pose(
            translation=base_translation,
            orientation=base_orientation
        )

        # 다음 동작 계산
        action = self.articulation_policy.get_next_articulation_action(step_size)

        # 로봇에 동작 적용
        self.robot.apply_action(action)

    def run(self):
        """시뮬레이션 실행"""
        # 씬 설정
        self.setup_scene()

        # 월드 리셋
        self.world.reset()

        # RMPflow 설정 (월드 리셋 후에 해야 함)
        self.setup_rmpflow()

        # 시뮬레이션 루프
        while simulation_app.is_running():
            self.world.step(render=True)

            if self.world.is_playing():
                self.physics_step(1/60.0)

        simulation_app.close()


if __name__ == "__main__":
    example = RMPflowExample()
    example.run()
```

### 11.2 지원 로봇 예제 (Franka)

```python
"""
Franka 로봇 RMPflow 예제 (간단 버전)
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    load_supported_motion_policy_config
)

# 월드 생성
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Franka 로봇 로드
robot_prim_path = "/World/Franka"
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    prim_path=robot_prim_path
)

robot = Articulation(robot_prim_path, name="franka")
world.scene.add(robot)

# 타겟 추가
target = VisualCuboid(
    "/World/target",
    name="target",
    position=np.array([0.5, 0.0, 0.5]),
    size=0.05,
    color=np.array([1.0, 0.0, 0.0])
)

# 월드 리셋
world.reset()

# RMPflow 설정 (지원 로봇용 간단 로드)
rmp_config = load_supported_motion_policy_config("Franka", "RMPflow")
rmpflow = RmpFlow(**rmp_config)

articulation_rmpflow = ArticulationMotionPolicy(
    robot,
    rmpflow,
    default_physics_dt=1/60.0
)

# 시뮬레이션 루프
while simulation_app.is_running():
    world.step(render=True)

    if world.is_playing():
        target_pos, target_rot = target.get_world_pose()

        rmpflow.set_end_effector_target(target_pos, target_rot)
        rmpflow.update_world()

        base_trans, base_rot = robot.get_world_pose()
        rmpflow.set_robot_base_pose(base_trans, base_rot)

        action = articulation_rmpflow.get_next_articulation_action(1/60.0)
        robot.apply_action(action)

simulation_app.close()
```

---

## 12. 문제 해결

### 12.1 일반적인 오류

#### "Failed to load robot description file"

**원인:** robot_descriptor.yaml 파일 경로가 잘못되었거나 파일 형식 오류

**해결:**
1. 파일 경로가 절대 경로인지 확인
2. YAML 문법 검증 (들여쓰기, 콜론 등)
3. cspace에 정의된 조인트 이름이 URDF와 일치하는지 확인

#### "Joint not found in URDF"

**원인:** robot_descriptor.yaml의 cspace에 정의된 조인트가 URDF에 없음

**해결:**
1. URDF 파일에서 조인트 이름 확인
2. robot_descriptor.yaml의 cspace 섹션 수정

#### "End effector frame not found"

**원인:** end_effector_frame_name이 URDF에 정의되지 않음

**해결:**
1. URDF에서 프레임/링크 이름 확인
2. RMPflow 초기화 시 올바른 프레임 이름 사용

#### 로봇이 움직이지 않음

**확인 사항:**
1. 시뮬레이션이 Play 상태인지 확인
2. `world.step(render=True)` 호출 확인
3. `apply_action()` 호출 확인
4. Active Joint가 올바르게 설정되었는지 확인

#### 로봇이 이상하게 움직임

**확인 사항:**
1. URDF의 관절 한계가 올바른지 확인
2. rmpflow_config.yaml의 `joint_limit_buffers` 확인
3. 충돌 구체 시각화로 올바른 형태인지 확인

### 12.2 성능 최적화

#### 시뮬레이션이 느림

- `maximum_substep_size` 값 증가 (정확도 감소)
- 충돌 구체 개수 줄이기
- 장애물 개수 줄이기

#### 목표 도달 정확도가 낮음

- `maximum_substep_size` 값 감소
- `position_gain` 값 조정
- End-effector 프레임 위치 확인

---

## 13. 참고 자료

### 13.1 공식 문서

- [Lula RMPflow - Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_rmpflow.html)
- [RMPflow Concepts](https://docs.robotsfan.com/isaacsim/4.5.0/manipulators/concepts/rmpflow.html)
- [Configuring RMPflow for a New Manipulator](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/advanced_tutorials/tutorial_configure_rmpflow_denso.html)
- [Motion Generation Extension](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_motion_generation.html)

### 13.2 관련 가이드

- [Lula Robot Description Editor 완벽 가이드](./Lula_Robot_Description_Editor_Complete_Guide.md)
- [RMPflow Tuning Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/concepts/motion_generation/rmpflow_tuning_guide.html)

### 13.3 예제 코드

- Isaac Sim 내장 예제: `Standalone Examples > Motion Generation > RMPflow`
- [Simulately - Using RMPFlow to Control Manipulators](https://simulately.wiki/docs/snippets/isaac-sim/rmpflow/)

---

## 부록: API 레퍼런스

### RmpFlow 클래스

```python
class RmpFlow:
    def __init__(
        self,
        robot_description_path: str,
        urdf_path: str,
        rmpflow_config_path: str,
        end_effector_frame_name: str,
        maximum_substep_size: float = 0.00334
    ) -> None:
        """RMPflow 인스턴스 생성"""

    def set_end_effector_target(
        self,
        target_position: np.ndarray,
        target_orientation: np.ndarray = None
    ) -> None:
        """End-effector 목표 설정

        Args:
            target_position: [x, y, z] 미터
            target_orientation: [w, x, y, z] 쿼터니언 (선택사항)
        """

    def update_world(self) -> None:
        """장애물 위치 업데이트"""

    def set_robot_base_pose(
        self,
        translation: np.ndarray,
        orientation: np.ndarray
    ) -> None:
        """로봇 베이스 위치 설정

        Args:
            translation: [x, y, z] 미터
            orientation: [w, x, y, z] 쿼터니언
        """

    def add_obstacle(self, obstacle) -> None:
        """장애물 추가"""

    def remove_obstacle(self, obstacle) -> None:
        """장애물 제거"""

    def visualize_collision_spheres(self) -> None:
        """충돌 구체 시각화"""

    def set_ignore_state_updates(self, ignore: bool) -> None:
        """상태 업데이트 무시 모드 설정"""

    def reset(self) -> None:
        """RMPflow 상태 리셋"""

    def get_end_effector_position(self) -> np.ndarray:
        """현재 End-effector 위치 반환"""

    def get_end_effector_orientation(self) -> np.ndarray:
        """현재 End-effector 방향 반환"""
```

### ArticulationMotionPolicy 클래스

```python
class ArticulationMotionPolicy:
    def __init__(
        self,
        articulation: Articulation,
        motion_policy: RmpFlow,
        default_physics_dt: float = 1/60.0
    ) -> None:
        """ArticulationMotionPolicy 생성"""

    def get_next_articulation_action(
        self,
        step_size: float
    ) -> ArticulationAction:
        """다음 Articulation 액션 계산

        Args:
            step_size: 물리 스텝 크기 (초)

        Returns:
            ArticulationAction: 로봇에 적용할 액션
        """
```

---

*이 문서는 Isaac Sim 4.x 버전을 기준으로 작성되었습니다.*
*최신 정보는 NVIDIA 공식 문서를 참조하세요.*
