# OpenMANIPULATOR-X Lula IK 적용 및 구현 가이드

이 문서는 Lula Robot Description Editor로 생성한 `.yaml` 설정 파일을 사용하여 Isaac Sim 5.1.0에서 OpenMANIPULATOR-X 매니퓰레이터에 IK를 적용하고, IK Target을 따라 로봇이 움직이도록 구현하는 방법을 상세히 설명합니다.

---

## 목차

1. [개요](#1-개요)
2. [사전 준비](#2-사전-준비)
3. [핵심 클래스 이해](#3-핵심-클래스-이해)
4. [구현 방법 1: Standalone Python Script](#4-구현-방법-1-standalone-python-script)
5. [구현 방법 2: Extension Scenario 방식](#5-구현-방법-2-extension-scenario-방식)
6. [구현 방법 3: GUI에서 직접 테스트](#6-구현-방법-3-gui에서-직접-테스트)
7. [4축 로봇 특수 설정](#7-4축-로봇-특수-설정)
8. [IK Target 생성 및 연동](#8-ik-target-생성-및-연동)
9. [문제 해결](#9-문제-해결)
10. [전체 예제 코드](#10-전체-예제-코드)

---

## 1. 개요

### 1.1 Lula IK 적용 흐름

```
[Lula Robot Description Editor]
         ↓
   robot_description.yaml  ←  이미 생성됨 ✓
         ↓
[LulaKinematicsSolver]  ←  IK 솔버 초기화
         ↓
[ArticulationKinematicsSolver]  ←  Articulation과 연결
         ↓
[IK Target (XFormPrim)]  ←  목표 위치/자세 설정
         ↓
[compute_inverse_kinematics()]  ←  IK 계산
         ↓
[ArticulationAction]  ←  조인트 위치 명령
         ↓
[Articulation.apply_action()]  ←  로봇 움직임
```

### 1.2 필요한 파일

| 파일 | 설명 | 상태 |
|------|------|------|
| `robot_description.yaml` | Lula 설정 파일 (cspace, collision_spheres 등) | ✅ 생성됨 |
| `open_manipulator.urdf` | 로봇 URDF 파일 | 필요 |
| `로봇.usd` | Isaac Sim용 USD 에셋 | 필요 |

---

## 2. 사전 준비

### 2.1 파일 구조 준비

```
my_robot_project/
├── config/
│   ├── robot_description.yaml    # Lula Editor에서 생성한 파일
│   └── open_manipulator.urdf     # ROBOTIS URDF
├── usd/
│   └── open_manipulator.usd      # Isaac Sim 로봇 에셋
└── scripts/
    └── follow_target.py          # IK 제어 스크립트
```

### 2.2 robot_description.yaml 확인

Lula Editor에서 생성한 파일이 다음 형식인지 확인:

```yaml
api_version: 1.0

cspace:
  - joint1
  - joint2
  - joint3
  - joint4

root_link: world  # 또는 base_link
default_q: [0.0, -1.0, 1.3, 0.0]

cspace_to_urdf_rules: []
composite_task_spaces: []

# 충돌 구체 (RMPflow용, IK만 사용할 경우 없어도 됨)
collision_spheres:
  - link1:
    - center: [0.0, 0.0, 0.04]
      radius: 0.035
  # ... 나머지 링크
```

### 2.3 End-Effector 프레임 이름 확인

URDF에서 엔드이펙터 링크 이름을 확인해야 합니다:

```python
# 사용 가능한 프레임 확인 방법
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

solver = LulaKinematicsSolver(
    robot_description_path="config/robot_description.yaml",
    urdf_path="config/open_manipulator.urdf"
)
print("사용 가능한 프레임:", solver.get_all_frame_names())
```

OpenMANIPULATOR-X의 일반적인 엔드이펙터 프레임:
- `end_effector_link`
- `gripper_link`
- `link5`

---

## 3. 핵심 클래스 이해

### 3.1 LulaKinematicsSolver

**역할**: 순운동학(FK) 및 역운동학(IK) 계산

```python
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# 초기화
solver = LulaKinematicsSolver(
    robot_description_path="/path/to/robot_description.yaml",
    urdf_path="/path/to/open_manipulator.urdf"
)

# 주요 메서드
solver.get_all_frame_names()      # 모든 프레임 이름 조회
solver.get_joint_names()          # 조인트 이름 조회
solver.set_robot_base_pose()      # 로봇 베이스 위치 설정
solver.set_max_iterations(100)    # IK 최대 반복 횟수 설정
```

### 3.2 ArticulationKinematicsSolver

**역할**: LulaKinematicsSolver를 Articulation과 연결하여 직접 적용 가능한 형태로 변환

```python
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver

# 초기화
art_solver = ArticulationKinematicsSolver(
    robot_articulation,           # Articulation 객체
    lula_kinematics_solver,       # LulaKinematicsSolver 인스턴스
    end_effector_frame_name       # 엔드이펙터 프레임 이름
)

# 주요 메서드
# IK 계산 - 결과는 ArticulationAction으로 반환
action, success = art_solver.compute_inverse_kinematics(
    target_position,              # np.array([x, y, z])
    target_orientation            # np.array([qw, qx, qy, qz]) 또는 None
)

# FK 계산 - 현재 엔드이펙터 위치/방향 반환
ee_position, ee_rotation = art_solver.compute_end_effector_pose()
```

### 3.3 ArticulationAction

**역할**: 로봇에 적용할 조인트 명령 데이터 구조

```python
from isaacsim.core.utils.types import ArticulationAction

# ArticulationAction 구조
action = ArticulationAction(
    joint_positions=np.array([...]),    # 조인트 위치
    joint_velocities=np.array([...]),   # 조인트 속도 (선택)
    joint_efforts=np.array([...])       # 조인트 토크 (선택)
)

# 로봇에 적용
robot_articulation.apply_action(action)
```

---

## 4. 구현 방법 1: Standalone Python Script

가장 직접적인 방법으로, `./python.sh`로 실행합니다.

### 4.1 기본 구조

```python
from isaacsim import SimulationApp

# 1. SimulationApp 초기화 (가장 먼저!)
simulation_app = SimulationApp({"headless": False})

# 2. 이후에 다른 import 수행
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver
)
import carb

# 3. 경로 설정
ROBOT_USD_PATH = "/path/to/open_manipulator.usd"
ROBOT_DESCRIPTION_PATH = "/path/to/robot_description.yaml"
URDF_PATH = "/path/to/open_manipulator.urdf"
END_EFFECTOR_FRAME = "end_effector_link"  # URDF 확인 필요

# 4. World 생성
my_world = World(stage_units_in_meters=1.0)

# 5. 로봇 추가
robot_prim_path = "/World/open_manipulator"
add_reference_to_stage(ROBOT_USD_PATH, robot_prim_path)
robot = Articulation(robot_prim_path)
my_world.scene.add(robot)

# 6. IK Target 추가 (시각적 표시용)
target_prim_path = "/World/ik_target"
target = XFormPrim(target_prim_path, scale=[0.02, 0.02, 0.02])
target.set_world_pose(
    position=np.array([0.2, 0.0, 0.15]),
    orientation=euler_angles_to_quats([0, np.pi, 0])
)
my_world.scene.add(target)

# 7. World 초기화
my_world.reset()

# 8. Lula IK Solver 초기화
kinematics_solver = LulaKinematicsSolver(
    robot_description_path=ROBOT_DESCRIPTION_PATH,
    urdf_path=URDF_PATH
)

# 사용 가능한 프레임 확인 (디버깅용)
print("Available frames:", kinematics_solver.get_all_frame_names())

# 9. ArticulationKinematicsSolver 초기화
art_kinematics_solver = ArticulationKinematicsSolver(
    robot,
    kinematics_solver,
    END_EFFECTOR_FRAME
)

# 10. 메인 루프
while simulation_app.is_running():
    my_world.step(render=True)
    
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        
        # Target 위치 가져오기
        target_position, target_orientation = target.get_world_pose()
        
        # 로봇 베이스 위치 업데이트 (이동 베이스인 경우)
        robot_base_pos, robot_base_ori = robot.get_world_pose()
        kinematics_solver.set_robot_base_pose(robot_base_pos, robot_base_ori)
        
        # IK 계산
        # 4축 로봇: orientation을 None으로 설정 (위치만 제어)
        action, success = art_kinematics_solver.compute_inverse_kinematics(
            target_position,
            target_orientation=None  # ⚠️ 4축 로봇 중요!
        )
        
        if success:
            robot.apply_action(action)
        else:
            carb.log_warn("IK 수렴 실패 - 타겟이 작업 공간 외부일 수 있음")

# 11. 종료
simulation_app.close()
```

### 4.2 실행 방법

```bash
cd <isaac_sim_root>
./python.sh /path/to/your_script.py
```

---

## 5. 구현 방법 2: Extension Scenario 방식

Isaac Sim의 Extension 구조를 사용하는 방법입니다.

### 5.1 시나리오 클래스 구조

```python
# scenario.py
import numpy as np
import os
import carb

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver
)


class OpenManipulatorIKExample:
    """OpenMANIPULATOR-X IK 제어 시나리오"""
    
    def __init__(self):
        self._kinematics_solver = None
        self._art_kinematics_solver = None
        self._articulation = None
        self._target = None
        
        # 경로 설정 (실제 경로로 수정 필요)
        self._config_dir = os.path.dirname(__file__)
        self._robot_description_path = os.path.join(
            self._config_dir, "config", "robot_description.yaml"
        )
        self._urdf_path = os.path.join(
            self._config_dir, "config", "open_manipulator.urdf"
        )
    
    def load_example_assets(self):
        """스테이지에 에셋 로드"""
        
        # 로봇 추가
        robot_prim_path = "/World/open_manipulator"
        robot_usd_path = os.path.join(self._config_dir, "usd", "open_manipulator.usd")
        add_reference_to_stage(robot_usd_path, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)
        
        # IK Target 추가
        target_prim_path = "/World/ik_target"
        self._target = XFormPrim(target_prim_path, scale=[0.02, 0.02, 0.02])
        self._target.set_default_state(
            position=np.array([0.2, 0.0, 0.15]),
            orientation=euler_angles_to_quats([0, np.pi, 0])
        )
        
        return self._articulation, self._target
    
    def setup(self):
        """시뮬레이션 시작 시 호출"""
        
        # LulaKinematicsSolver 초기화
        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=self._robot_description_path,
            urdf_path=self._urdf_path
        )
        
        # 사용 가능한 프레임 출력
        frames = self._kinematics_solver.get_all_frame_names()
        print(f"사용 가능한 프레임: {frames}")
        
        # 엔드이펙터 프레임 설정
        # ⚠️ 실제 URDF의 프레임 이름으로 수정 필요!
        end_effector_frame = "end_effector_link"
        
        if end_effector_frame not in frames:
            carb.log_warn(f"프레임 '{end_effector_frame}'을 찾을 수 없음. 사용 가능: {frames}")
            # 마지막 링크를 사용하는 fallback
            end_effector_frame = frames[-1]
            carb.log_info(f"'{end_effector_frame}'을 엔드이펙터로 사용")
        
        # ArticulationKinematicsSolver 초기화
        self._art_kinematics_solver = ArticulationKinematicsSolver(
            self._articulation,
            self._kinematics_solver,
            end_effector_frame
        )
    
    def update(self, step: float):
        """매 프레임 호출 - IK 계산 및 적용"""
        
        # Target 위치 가져오기
        target_position, target_orientation = self._target.get_world_pose()
        
        # 로봇 베이스 위치 업데이트
        robot_base_pos, robot_base_ori = self._articulation.get_world_pose()
        self._kinematics_solver.set_robot_base_pose(robot_base_pos, robot_base_ori)
        
        # IK 계산 (4축 로봇: orientation=None)
        action, success = self._art_kinematics_solver.compute_inverse_kinematics(
            target_position,
            target_orientation=None  # 위치만 제어
        )
        
        if success:
            self._articulation.apply_action(action)
        else:
            carb.log_warn("IK 수렴 실패")
    
    def reset(self):
        """리셋 시 호출"""
        # Kinematics는 상태가 없으므로 특별한 처리 불필요
        pass
```

---

## 6. 구현 방법 3: GUI에서 직접 테스트

Script Editor를 사용하여 빠르게 테스트하는 방법입니다.

### 6.1 Script Editor 열기

```
Window > Script Editor
```

### 6.2 테스트 스크립트

```python
# Script Editor에서 실행
# 먼저 로봇이 스테이지에 로드되어 있고, Play 상태여야 함

import numpy as np
from pxr import Usd, UsdGeom
import omni.usd

from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver
)

# 경로 설정 (자신의 경로로 수정!)
ROBOT_PRIM_PATH = "/World/open_manipulator"  # 스테이지의 로봇 경로
ROBOT_DESCRIPTION_PATH = "/home/user/config/robot_description.yaml"
URDF_PATH = "/home/user/config/open_manipulator.urdf"
END_EFFECTOR_FRAME = "end_effector_link"

# Articulation 가져오기
robot = Articulation(ROBOT_PRIM_PATH)
robot.initialize()

# Lula Solver 초기화
kinematics_solver = LulaKinematicsSolver(
    robot_description_path=ROBOT_DESCRIPTION_PATH,
    urdf_path=URDF_PATH
)

# 사용 가능한 프레임 확인
print("사용 가능한 프레임:", kinematics_solver.get_all_frame_names())

# ArticulationKinematicsSolver 초기화
art_solver = ArticulationKinematicsSolver(
    robot,
    kinematics_solver,
    END_EFFECTOR_FRAME
)

# 테스트: 특정 위치로 IK 계산
target_position = np.array([0.15, 0.05, 0.2])  # x, y, z

action, success = art_solver.compute_inverse_kinematics(
    target_position,
    target_orientation=None  # 4축 로봇
)

if success:
    print(f"IK 성공! 조인트 위치: {action.joint_positions}")
    robot.apply_action(action)
else:
    print("IK 실패 - 타겟이 작업 공간 외부")

# 현재 엔드이펙터 위치 확인
ee_pos, ee_rot = art_solver.compute_end_effector_pose()
print(f"현재 EE 위치: {ee_pos}")
```

---

## 7. 4축 로봇 특수 설정

OpenMANIPULATOR-X는 4축 로봇이므로 특별한 고려가 필요합니다.

### 7.1 핵심: Orientation을 None으로 설정

```python
# ❌ 잘못된 방법 (6축 로봇용)
action, success = art_solver.compute_inverse_kinematics(
    target_position,
    target_orientation=np.array([1, 0, 0, 0])  # quaternion
)

# ✅ 올바른 방법 (4축 로봇용)
action, success = art_solver.compute_inverse_kinematics(
    target_position,
    target_orientation=None  # 위치만 제어!
)
```

### 7.2 IK 성공률 높이기

```python
# IK 솔버 설정 조정
kinematics_solver.set_max_iterations(200)  # 기본값보다 높게

# 작업 공간 내 타겟인지 확인
WORKSPACE_RADIUS = 0.38  # OpenMANIPULATOR-X의 최대 도달 거리 (약 380mm)

def is_target_reachable(target_pos, robot_base_pos):
    """타겟이 작업 공간 내에 있는지 확인"""
    distance = np.linalg.norm(target_pos - robot_base_pos)
    return distance < WORKSPACE_RADIUS and distance > 0.05  # 최소 거리도 확인
```

### 7.3 robot_description.yaml 설정

```yaml
api_version: 1.0

# 4개의 active joint만 포함
cspace:
  - joint1
  - joint2
  - joint3
  - joint4

root_link: link1  # 또는 base_link (URDF에 따라)

# 특이점을 피하는 기본 자세
default_q: [0.0, -1.0, 1.3, 0.0]

cspace_to_urdf_rules: []
composite_task_spaces: []
```

---

## 8. IK Target 생성 및 연동

### 8.1 시각적 IK Target 생성

```python
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

# 방법 1: 간단한 XFormPrim (보이지 않음)
target = XFormPrim("/World/ik_target")
target.set_world_pose(position=np.array([0.2, 0, 0.15]))

# 방법 2: 시각적 마커 사용 (권장)
assets_root = get_assets_root_path()
frame_prim_path = assets_root + "/Isaac/Props/UIElements/frame_prim.usd"
add_reference_to_stage(frame_prim_path, "/World/ik_target")
target = XFormPrim("/World/ik_target", scale=[0.03, 0.03, 0.03])
target.set_world_pose(
    position=np.array([0.2, 0, 0.15]),
    orientation=euler_angles_to_quats([0, np.pi, 0])
)
```

### 8.2 인터랙티브 Target 이동

GUI에서 Target을 드래그하여 이동하면 로봇이 따라갑니다:

```python
# 메인 루프에서
while simulation_app.is_running():
    my_world.step(render=True)
    
    if my_world.is_playing():
        # GUI에서 이동된 Target 위치 가져오기
        target_pos, target_ori = target.get_world_pose()
        
        # IK 계산 및 적용
        action, success = art_solver.compute_inverse_kinematics(
            target_pos,
            target_orientation=None
        )
        
        if success:
            robot.apply_action(action)
```

### 8.3 프로그래밍 방식으로 Target 이동

```python
import time

# 여러 위치를 순회
waypoints = [
    np.array([0.2, 0.0, 0.15]),
    np.array([0.15, 0.1, 0.2]),
    np.array([0.2, -0.1, 0.1]),
    np.array([0.25, 0.0, 0.15]),
]

current_waypoint = 0

def update(step):
    global current_waypoint
    
    # 현재 waypoint로 target 이동
    target.set_world_pose(position=waypoints[current_waypoint])
    target_pos, _ = target.get_world_pose()
    
    # IK 적용
    action, success = art_solver.compute_inverse_kinematics(
        target_pos,
        target_orientation=None
    )
    
    if success:
        robot.apply_action(action)
        
        # 도달 확인
        ee_pos, _ = art_solver.compute_end_effector_pose()
        if np.linalg.norm(ee_pos - target_pos) < 0.01:  # 1cm 이내
            current_waypoint = (current_waypoint + 1) % len(waypoints)
```

---

## 9. 문제 해결

### 9.1 "프레임을 찾을 수 없음" 오류

**증상**: `Frame 'xxx' not found in robot description`

**해결**:
```python
# 사용 가능한 프레임 확인
frames = kinematics_solver.get_all_frame_names()
print(f"사용 가능한 프레임: {frames}")

# 마지막 링크 사용 (일반적으로 엔드이펙터)
end_effector_frame = frames[-1]
```

### 9.2 IK가 항상 실패

**원인 1**: 타겟이 작업 공간 외부
```python
# 작업 공간 확인
distance = np.linalg.norm(target_pos - robot_base_pos)
if distance > 0.38:  # 380mm 초과
    print(f"타겟이 너무 멀음: {distance}m")
```

**원인 2**: 4축 로봇에 orientation 지정
```python
# ✅ 올바른 방법
action, success = art_solver.compute_inverse_kinematics(
    target_pos,
    target_orientation=None  # 반드시 None!
)
```

**원인 3**: robot_description.yaml의 cspace 오류
```yaml
# URDF의 조인트 이름과 정확히 일치해야 함
cspace:
  - joint1  # URDF에서 확인
  - joint2
  - joint3
  - joint4
```

### 9.3 로봇이 움직이지 않음

**원인 1**: Articulation이 초기화되지 않음
```python
# World.reset() 후에 Articulation이 초기화됨
my_world.reset()

# 또는 수동 초기화
robot.initialize()
```

**원인 2**: apply_action 호출 누락
```python
if success:
    robot.apply_action(action)  # 이 줄 확인!
```

**원인 3**: 시뮬레이션이 Play 상태가 아님
```python
if my_world.is_playing():  # Play 상태 확인
    # IK 로직
```

### 9.4 로봇이 진동/불안정

**해결**: Joint Drive 게인 조정
```python
# Gain Tuner 사용 (GUI)
# Tools > Robotics > Asset Editors > Gain Tuner

# 또는 프로그래밍 방식
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
for joint_name in ["joint1", "joint2", "joint3", "joint4"]:
    joint_prim = stage.GetPrimAtPath(f"/World/open_manipulator/{joint_name}")
    drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
    drive.GetStiffnessAttr().Set(1000.0)  # 강성 증가
    drive.GetDampingAttr().Set(100.0)     # 감쇠 증가
```

---

## 10. 전체 예제 코드

### 10.1 완전한 Standalone 스크립트

```python
#!/usr/bin/env python3
"""
OpenMANIPULATOR-X Lula IK Follow Target Example
Isaac Sim 5.1.0+

실행 방법:
    cd <isaac_sim_root>
    ./python.sh /path/to/this_script.py
"""

from isaacsim import SimulationApp

# SimulationApp 초기화 (가장 먼저!)
CONFIG = {
    "headless": False,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(CONFIG)

# === 이후 import ===
import numpy as np
import os
import carb

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver
)


# ============================================
# 설정 (자신의 환경에 맞게 수정!)
# ============================================
CONFIG_DIR = "/home/user/open_manipulator_config"  # 수정 필요!

ROBOT_USD_PATH = os.path.join(CONFIG_DIR, "open_manipulator.usd")
ROBOT_DESCRIPTION_PATH = os.path.join(CONFIG_DIR, "robot_description.yaml")
URDF_PATH = os.path.join(CONFIG_DIR, "open_manipulator.urdf")

# 엔드이펙터 프레임 (URDF 확인 후 수정)
END_EFFECTOR_FRAME = "end_effector_link"

# ============================================
# World 및 Scene 설정
# ============================================
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# 로봇 추가
robot_prim_path = "/World/open_manipulator"
add_reference_to_stage(ROBOT_USD_PATH, robot_prim_path)
robot = Articulation(robot_prim_path)
my_world.scene.add(robot)

# IK Target 추가 (시각적 마커)
assets_root = get_assets_root_path()
if assets_root:
    frame_usd = assets_root + "/Isaac/Props/UIElements/frame_prim.usd"
    add_reference_to_stage(frame_usd, "/World/ik_target")
    
target = XFormPrim("/World/ik_target", scale=[0.03, 0.03, 0.03])
target.set_default_state(
    position=np.array([0.15, 0.0, 0.15]),
    orientation=euler_angles_to_quats([0, np.pi, 0])
)
my_world.scene.add(target)

# World 초기화
my_world.reset()

# ============================================
# Lula IK Solver 초기화
# ============================================
kinematics_solver = LulaKinematicsSolver(
    robot_description_path=ROBOT_DESCRIPTION_PATH,
    urdf_path=URDF_PATH
)

# 사용 가능한 프레임 확인
available_frames = kinematics_solver.get_all_frame_names()
print(f"\n=== 사용 가능한 프레임 ===")
for i, frame in enumerate(available_frames):
    print(f"  [{i}] {frame}")

# 엔드이펙터 프레임 검증
if END_EFFECTOR_FRAME not in available_frames:
    carb.log_warn(f"프레임 '{END_EFFECTOR_FRAME}'을 찾을 수 없음!")
    END_EFFECTOR_FRAME = available_frames[-1]
    carb.log_info(f"'{END_EFFECTOR_FRAME}'을 엔드이펙터로 사용")

print(f"\n엔드이펙터 프레임: {END_EFFECTOR_FRAME}")

# ArticulationKinematicsSolver 초기화
art_kinematics_solver = ArticulationKinematicsSolver(
    robot,
    kinematics_solver,
    END_EFFECTOR_FRAME
)

# IK 설정 최적화
kinematics_solver.set_max_iterations(200)

# ============================================
# 메인 시뮬레이션 루프
# ============================================
print("\n=== 시뮬레이션 시작 ===")
print("GUI에서 IK Target (frame_prim)을 드래그하여 이동하세요.")
print("로봇이 Target을 따라 움직입니다.")
print("종료하려면 창을 닫으세요.\n")

ik_fail_count = 0

while simulation_app.is_running():
    my_world.step(render=True)
    
    if my_world.is_playing():
        # 첫 프레임에서 리셋
        if my_world.current_time_step_index == 0:
            my_world.reset()
            ik_fail_count = 0
        
        # Target 위치 가져오기
        target_position, target_orientation = target.get_world_pose()
        
        # 로봇 베이스 위치 업데이트 (이동 베이스인 경우 필요)
        robot_base_pos, robot_base_ori = robot.get_world_pose()
        kinematics_solver.set_robot_base_pose(robot_base_pos, robot_base_ori)
        
        # === IK 계산 ===
        # 4축 로봇이므로 orientation=None (위치만 제어)
        action, success = art_kinematics_solver.compute_inverse_kinematics(
            target_position,
            target_orientation=None  # ⚠️ 4축 로봇 핵심 설정!
        )
        
        if success:
            robot.apply_action(action)
            ik_fail_count = 0
        else:
            ik_fail_count += 1
            if ik_fail_count == 1:  # 첫 실패만 로그
                carb.log_warn(
                    f"IK 수렴 실패 - Target 위치: {target_position}"
                )
        
        # (선택) 현재 엔드이펙터 위치 출력 (디버깅용)
        # ee_pos, ee_rot = art_kinematics_solver.compute_end_effector_pose()
        # print(f"EE 위치: {ee_pos}, Target: {target_position}")

# ============================================
# 정리 및 종료
# ============================================
simulation_app.close()
print("시뮬레이션 종료")
```

### 10.2 사용자 정의 KinematicsSolver 클래스

재사용 가능한 클래스로 만들기:

```python
# ik_solver.py
import os
from typing import Optional
import numpy as np

from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver
)


class OpenManipulatorIKSolver(ArticulationKinematicsSolver):
    """OpenMANIPULATOR-X용 IK Solver 래퍼 클래스"""
    
    def __init__(
        self,
        robot_articulation: Articulation,
        config_dir: str,
        end_effector_frame_name: Optional[str] = None
    ) -> None:
        """
        Args:
            robot_articulation: Isaac Sim Articulation 객체
            config_dir: robot_description.yaml과 URDF가 있는 디렉토리
            end_effector_frame_name: 엔드이펙터 프레임 이름 (None이면 자동 감지)
        """
        # 파일 경로 설정
        robot_description_path = os.path.join(config_dir, "robot_description.yaml")
        urdf_path = os.path.join(config_dir, "open_manipulator.urdf")
        
        # LulaKinematicsSolver 생성
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=robot_description_path,
            urdf_path=urdf_path
        )
        
        # 엔드이펙터 프레임 자동 감지
        if end_effector_frame_name is None:
            frames = self._kinematics.get_all_frame_names()
            # 일반적인 엔드이펙터 이름 패턴 검색
            for name in ["end_effector_link", "gripper_link", "tool0", "link5"]:
                if name in frames:
                    end_effector_frame_name = name
                    break
            if end_effector_frame_name is None:
                end_effector_frame_name = frames[-1]  # 마지막 프레임 사용
        
        # 부모 클래스 초기화
        ArticulationKinematicsSolver.__init__(
            self,
            robot_articulation,
            self._kinematics,
            end_effector_frame_name
        )
        
        # IK 설정 최적화 (4축 로봇용)
        self._kinematics.set_max_iterations(200)
    
    def solve_ik_position_only(
        self,
        target_position: np.ndarray
    ) -> tuple:
        """
        위치만 지정하여 IK 계산 (4축 로봇용)
        
        Args:
            target_position: 목표 위치 [x, y, z]
            
        Returns:
            (ArticulationAction, success: bool)
        """
        return self.compute_inverse_kinematics(
            target_position,
            target_orientation=None
        )
    
    def get_available_frames(self) -> list:
        """사용 가능한 프레임 목록 반환"""
        return self._kinematics.get_all_frame_names()
```

### 10.3 사용 예시

```python
from ik_solver import OpenManipulatorIKSolver

# 초기화
ik_solver = OpenManipulatorIKSolver(
    robot_articulation=robot,
    config_dir="/path/to/config"
)

# IK 계산 (위치만)
target_pos = np.array([0.2, 0.0, 0.15])
action, success = ik_solver.solve_ik_position_only(target_pos)

if success:
    robot.apply_action(action)
```

---

## 요약

### 핵심 포인트

1. **필수 파일 2개**: `robot_description.yaml` + `open_manipulator.urdf`

2. **핵심 클래스 2개**:
   - `LulaKinematicsSolver`: IK 계산 엔진
   - `ArticulationKinematicsSolver`: Articulation과 연결

3. **4축 로봇 핵심**: `target_orientation=None`으로 위치만 제어

4. **적용 흐름**:
   ```
   LulaKinematicsSolver 생성
   → ArticulationKinematicsSolver 생성
   → compute_inverse_kinematics(position, orientation=None)
   → robot.apply_action(action)
   ```

5. **문제 발생 시**:
   - `get_all_frame_names()`로 프레임 확인
   - 작업 공간(380mm) 내 타겟인지 확인
   - Joint Drive 게인 조정 고려

---

*이 문서는 Isaac Sim 5.1.0을 기준으로 작성되었습니다.*
