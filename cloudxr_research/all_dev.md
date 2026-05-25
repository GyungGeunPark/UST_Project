# OpenMANIPULATOR-X + 4륜 모바일 베이스 Isaac Sim 5.1.0 CloudXR 텔레오퍼레이션 구현 가이드

Meta Quest 3S를 통한 로봇 텔레오퍼레이션과 모방학습 데이터 수집을 위한 **Isaac Sim 5.1.0** 완전 구현 가이드입니다. 이 문서에서는 Lula IK 설정, 4륜 모바일 베이스 구성, CloudXR VR 연동, 그리고 전체 통합 코드를 단계별로 다룹니다.

---

## 1. Lula IK와 OpenMANIPULATOR-X 연결

### Isaac Sim 5.1.0 API 임포트 경로

Isaac Sim **4.5 이후** `omni.isaac.*`에서 `isaacsim.*`로 패키지 경로가 변경되었습니다. 5.1.0에서는 다음 경로를 사용합니다:

```python
# Isaac Sim 5.1.0 새로운 API 경로
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
    interface_config_loader
)
from isaacsim.core.prims import Articulation
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import get_extension_path_from_name
```

| 이전 버전 (4.2 이하) | 현재 버전 (5.0.0+) |
|---------------------|-------------------|
| `omni.isaac.motion_generation` | `isaacsim.robot_motion.motion_generation` |
| `omni.isaac.core` | `isaacsim.core.api` |
| `omni.isaac.core.prims` | `isaacsim.core.prims` |

### robot_descriptor.yaml 커스텀 설정

OpenMANIPULATOR-X (4축 암 + 1축 그리퍼)용 Lula 설정 파일입니다. **그리퍼 조인트는 Fixed로 설정**하여 IK 계산에서 제외합니다:

```yaml
# openmanipulator_descriptor.yaml
api_version: 1.0

# Configuration Space - 4축 암 관절만 포함
cspace:
  - joint1
  - joint2
  - joint3
  - joint4

# 루트 링크 (URDF와 일치)
root_link: link1

# 기본 관절 위치 (라디안)
default_q: [0.0, 0.0, 0.0, 0.0]

# 그리퍼 관절을 Fixed로 설정 (Lula에서 제외)
cspace_to_urdf_rules:
  - {name: gripper, rule: fixed, default: 0.01}
  # 미믹 조인트가 있는 경우
  - {name: gripper_sub, rule: fixed, default: 0.01}

# 관절 가속도/저크 제한
acceleration_limits: [10.0, 10.0, 10.0, 10.0]
jerk_limits: [500.0, 500.0, 500.0, 500.0]

# 충돌 구체 (선택사항)
collision_spheres: []
```

### LulaKinematicsSolver 초기화 및 IK 계산 코드

```python
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver
)

class OpenManipulatorIK:
    """OpenMANIPULATOR-X Lula IK 래퍼 클래스"""
    
    def __init__(self, robot_articulation: Articulation):
        # 커스텀 config 파일 경로
        self.kinematics_solver = LulaKinematicsSolver(
            robot_description_path="./config/openmanipulator_descriptor.yaml",
            urdf_path="./assets/open_manipulator_x.urdf"
        )
        
        # 사용 가능한 프레임 확인
        print(f"사용 가능한 프레임: {self.kinematics_solver.get_all_frame_names()}")
        
        # ArticulationKinematicsSolver 래핑
        self.articulation_kinematics = ArticulationKinematicsSolver(
            robot_articulation,
            self.kinematics_solver,
            "end_effector_link"  # 엔드이펙터 프레임명
        )
    
    def compute_ik(self, target_position: np.ndarray, target_orientation: np.ndarray, 
                   articulation: Articulation) -> tuple:
        """
        IK 계산 및 적용
        
        Args:
            target_position: [x, y, z] 미터 단위
            target_orientation: [w, x, y, z] 쿼터니언
        Returns:
            (action, success) 튜플
        """
        # 로봇 베이스 포즈 업데이트 (모바일 베이스인 경우)
        base_pos, base_rot = articulation.get_world_pose()
        self.kinematics_solver.set_robot_base_pose(base_pos, base_rot)
        
        # IK 계산
        action, success = self.articulation_kinematics.compute_inverse_kinematics(
            target_position=target_position,
            target_orientation=target_orientation
        )
        
        if success:
            articulation.apply_action(action)
        return action, success
    
    def compute_fk(self, joint_positions: np.ndarray) -> tuple:
        """순기구학 계산"""
        position, rotation_matrix = self.kinematics_solver.compute_forward_kinematics(
            frame_name="end_effector_link",
            joint_positions=joint_positions
        )
        return position, rotation_matrix

# 사용 예시
my_world = World(stage_units_in_meters=1.0)
add_reference_to_stage(usd_path="./assets/open_manipulator_x.usd", prim_path="/World/Robot")
robot = Articulation("/World/Robot")
my_world.scene.add(robot)
my_world.reset()

ik_solver = OpenManipulatorIK(robot)

# IK 계산 및 적용
target_pos = np.array([0.2, 0.0, 0.15])
target_rot = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz
action, success = ik_solver.compute_ik(target_pos, target_rot, robot)
```

### 그리퍼 별도 제어

그리퍼는 Lula에서 제외되므로 **별도 컨트롤러**로 제어합니다:

```python
# 그리퍼 열기/닫기
def control_gripper(articulation: Articulation, open: bool):
    """그리퍼 제어 (Lula와 별도)"""
    gripper_pos = 0.019 if open else 0.0  # 열림/닫힘 위치
    gripper_indices = [4]  # 그리퍼 조인트 인덱스
    
    articulation.set_joint_positions(
        positions=np.array([gripper_pos]),
        joint_indices=gripper_indices
    )
```

---

## 2. 4륜 모바일 베이스 Articulation Joint 설정

### 구동 방식 비교 및 권장

| 구동 방식 | 바퀴 수 | 장점 | 단점 | 구현 난이도 |
|----------|---------|------|------|-------------|
| **Differential Drive** | 2륜+캐스터 | 단순, 제자리 회전 | 직진 안정성 낮음 | ⭐ 쉬움 |
| **Skid-Steer** | 4륜 | 안정성 높음, 험지 | 마찰 문제 | ⭐⭐ **권장** |
| **Mecanum** | 4륜 | 전방향 이동 | 복잡, 효율 낮음 | ⭐⭐⭐ |

**권장: Skid-Steer (4륜)** — DifferentialController로 동일하게 제어 가능하며, 좌측 2바퀴와 우측 2바퀴를 그룹으로 같은 속도로 구동합니다.

### Revolute Joint 생성 (Python API)

```python
from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd

def create_wheel_joint(stage, joint_path: str, body_path: str, wheel_path: str, 
                       local_pos: tuple, axis: str = "Y"):
    """바퀴 Revolute Joint 생성"""
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    
    # 부모(본체) / 자식(바퀴) 관계
    joint.CreateBody0Rel().SetTargets([body_path])
    joint.CreateBody1Rel().SetTargets([wheel_path])
    
    # 로컬 위치/회전 설정
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(0.707, 0.707, 0, 0))  # 90도 회전
    
    # 회전축
    joint.CreateAxisAttr().Set(axis)
    return joint

# 4개 바퀴 조인트 생성
stage = omni.usd.get_context().get_stage()
wheel_positions = {
    "wheel_fl": (0.15, 0.12, 0),   # Front Left
    "wheel_fr": (0.15, -0.12, 0),  # Front Right
    "wheel_rl": (-0.15, 0.12, 0),  # Rear Left
    "wheel_rr": (-0.15, -0.12, 0), # Rear Right
}

for name, pos in wheel_positions.items():
    create_wheel_joint(
        stage,
        joint_path=f"/World/Robot/joints/{name}_joint",
        body_path="/World/Robot/chassis",
        wheel_path=f"/World/Robot/{name}",
        local_pos=pos
    )
```

### Angular Drive 설정 (속도 제어)

```python
def setup_velocity_drive(stage, joint_path: str, damping: float = 1e4, 
                         max_force: float = 1e6):
    """속도 제어용 Angular Drive 설정"""
    joint_prim = stage.GetPrimAtPath(joint_path)
    
    # Angular Drive API 적용
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateDampingAttr().Set(damping)      # 속도 제어: 높은 Damping
    drive.CreateStiffnessAttr().Set(0)          # 속도 제어: Stiffness = 0
    drive.CreateMaxForceAttr().Set(max_force)
    drive.CreateTargetVelocityAttr().Set(0)     # 초기 속도
    return drive

# 4개 바퀴에 Drive 적용
for name in ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]:
    setup_velocity_drive(stage, f"/World/Robot/joints/{name}_joint", damping=1e4)
```

**권장 파라미터:**
| 로봇 크기 | Stiffness | Damping | Max Force |
|----------|-----------|---------|-----------|
| 소형 (< 5kg) | 0 | 1e4 | 1e3 |
| 중형 (5-20kg) | 0 | 1e5 | 1e4 |
| 대형 (> 20kg) | 0 | 1e6 | 1e5 |

### Articulation Root 설정 (Floating Base)

```python
def setup_floating_base(stage, robot_path: str):
    """Floating base articulation 설정"""
    robot_prim = stage.GetPrimAtPath(robot_path)
    
    # ArticulationRootAPI 적용
    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    
    # PhysX 확장 설정
    physx_art = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
    physx_art.CreateEnabledSelfCollisionsAttr().Set(False)
    physx_art.CreateSolverPositionIterationCountAttr().Set(32)
    physx_art.CreateSolverVelocityIterationCountAttr().Set(1)

setup_floating_base(stage, "/World/Robot")
```

### DifferentialController 연동

```python
import numpy as np
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.core.utils.types import ArticulationAction

class SkidSteerController:
    """4륜 Skid-Steer 제어 클래스"""
    
    def __init__(self, wheel_radius: float = 0.05, wheel_base: float = 0.24):
        self.controller = DifferentialController(
            name="skid_steer",
            wheel_radius=wheel_radius,
            wheel_base=wheel_base
        )
        # 조인트 순서: FL, FR, RL, RR
        self.wheel_indices = [0, 1, 2, 3]
    
    def compute_wheel_velocities(self, linear_vel: float, angular_vel: float) -> np.ndarray:
        """
        선속도/각속도 → 바퀴 속도 변환
        
        Args:
            linear_vel: 전진 속도 (m/s)
            angular_vel: 회전 속도 (rad/s)
        Returns:
            4개 바퀴 속도 [FL, FR, RL, RR] (rad/s)
        """
        action = self.controller.forward([linear_vel, angular_vel])
        left_vel = action.joint_velocities[0]
        right_vel = action.joint_velocities[1]
        
        # Skid-steer: 좌측 바퀴 동일, 우측 바퀴 동일
        return np.array([left_vel, right_vel, left_vel, right_vel])
    
    def apply_to_robot(self, robot, linear_vel: float, angular_vel: float):
        """로봇에 속도 명령 적용"""
        wheel_vels = self.compute_wheel_velocities(linear_vel, angular_vel)
        action = ArticulationAction(
            joint_velocities=wheel_vels,
            joint_indices=self.wheel_indices
        )
        robot.apply_wheel_actions(action)

# 사용 예시
base_controller = SkidSteerController(wheel_radius=0.05, wheel_base=0.24)
base_controller.apply_to_robot(robot, linear_vel=0.5, angular_vel=0.2)
```

### 물리 속성 튜닝 (마찰, 질량)

```python
from pxr import UsdShade

def create_wheel_material(stage, friction: float = 0.8):
    """바퀴용 물리 재질 생성"""
    material_path = "/World/Materials/WheelMaterial"
    UsdShade.Material.Define(stage, material_path)
    material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))
    material.CreateStaticFrictionAttr().Set(friction)
    material.CreateDynamicFrictionAttr().Set(friction)
    material.CreateRestitutionAttr().Set(0.1)
    return material_path

def set_mass(stage, prim_path: str, mass: float, com: tuple = None):
    """질량 및 무게중심 설정"""
    mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(prim_path))
    mass_api.CreateMassAttr().Set(mass)
    if com:
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*com))

# 적용
set_mass(stage, "/World/Robot/chassis", mass=8.0, com=(0, 0, -0.05))  # 무게중심 낮게
for wheel in ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]:
    set_mass(stage, f"/World/Robot/{wheel}", mass=0.3)
```

---

## 3. CloudXR + Meta Quest 3S 텔레오퍼레이션

### CloudXR Early Access 가입

**Meta Quest 3S**는 현재 **CloudXR Early Access Program**을 통해서만 지원됩니다:

1. [NVIDIA CloudXR Early Access](https://developer.nvidia.com/cloudxr-sdk-early-access-program/join) 가입
2. Use case에 "Isaac Sim/Lab teleoperation" 명시
3. 승인 후 NGC에서 **CloudXR.js with Isaac Teleop samples** 다운로드

### CloudXR Runtime Docker 설정

**시스템 요구사항:**
- Ubuntu 22.04/24.04, GPU RTX 4090 이상
- Docker 26.0.0+, NVIDIA Container Toolkit

```bash
# 방화벽 포트 오픈
sudo ufw allow 47998:48000,48005,48008,48012/udp
sudo ufw allow 48010/tcp

# Isaac Lab + CloudXR 컨테이너 시작
./docker/container.py start \
    --files docker-compose.cloudxr-runtime.patch.yaml \
    --env-file .env.cloudxr-runtime

# 컨테이너 진입
./docker/container.py enter base

# 환경변수 설정
export XDG_RUNTIME_DIR=$(pwd)/openxr/run
export XR_RUNTIME_JSON=$(pwd)/openxr/share/openxr/1/openxr_cloudxr.json

# 텔레오퍼레이션 실행
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-PickPlace-Custom-v0 \
    --teleop_device handtracking \
    --enable_pinocchio
```

### 대안 솔루션 비교

| 솔루션 | 가격 | Linux | Isaac Lab 통합 | 핸드트래킹 |
|--------|------|-------|---------------|-----------|
| **CloudXR** | 무료(SDK) | ✅ | ⭐⭐⭐ 공식 | ✅ |
| ALVR | 무료 | ✅ | ⭐⭐ | ✅ |
| Quest Link | 무료 | ❌ | ⭐⭐ | ✅ |

### OpenXRDevice 및 Retargeter 설정

```python
from isaaclab.devices import OpenXRDevice, OpenXRDeviceCfg, DeviceBase
from isaaclab.devices.openxr import XrCfg
from isaaclab.devices.openxr.retargeters import (
    Se3AbsRetargeter, Se3AbsRetargeterCfg,
    Se3RelRetargeter, Se3RelRetargeterCfg,
    GripperRetargeter, GripperRetargeterCfg
)

# XR 환경 설정
xr_cfg = XrCfg(
    anchor_pos=[0.0, 0.0, 0.0],      # 로봇 위치
    anchor_rot=[1.0, 0.0, 0.0, 0.0]  # 쿼터니언 (wxyz)
)

# Se3AbsRetargeter: 절대 좌표 기반 (손 위치 → EE 위치)
position_retargeter = Se3AbsRetargeter(
    Se3AbsRetargeterCfg(
        bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
        zero_out_xy_rotation=True,   # Z축 회전만 허용
        use_wrist_position=False,    # 핀치 포지션 사용 (정밀 조작)
        use_wrist_rotation=True,
        enable_visualization=True
    )
)

# GripperRetargeter: 핀치 제스처 → 그리퍼 제어
# 엄지-검지 거리 6cm↑: 열림, 4cm↓: 닫힘
gripper_retargeter = GripperRetargeter(
    GripperRetargeterCfg(
        bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT
    )
)

# OpenXR 디바이스 생성
device = OpenXRDevice(
    OpenXRDeviceCfg(xr_cfg=xr_cfg),
    retargeters=[position_retargeter, gripper_retargeter]
)
```

### VR → IK 통합 제어 루프

```python
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
import torch

class VRTeleopController:
    """VR 텔레오퍼레이션 + IK 통합 컨트롤러"""
    
    def __init__(self, env, num_envs: int = 1, device: str = "cuda"):
        self.env = env
        self.device = device
        
        # OpenXR 디바이스
        self.xr_device = OpenXRDevice(
            OpenXRDeviceCfg(xr_cfg=XrCfg()),
            retargeters=[
                Se3AbsRetargeter(Se3AbsRetargeterCfg(
                    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT
                )),
                GripperRetargeter(GripperRetargeterCfg(
                    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT
                ))
            ]
        )
        
        # Differential IK 컨트롤러
        self.ik_controller = DifferentialIKController(
            DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=False,  # 절대 포즈
                ik_method="dls",
                ik_params={"lambda_val": 0.05}
            ),
            num_envs=num_envs,
            device=device
        )
    
    def step(self, robot):
        """VR 입력 → IK → 로봇 명령"""
        # 1. VR에서 리타겟팅된 명령 획득
        commands = self.xr_device.advance()
        if commands is None:
            return None
        
        # 2. 포즈/그리퍼 분리
        target_pose = torch.tensor(commands[:7], device=self.device).unsqueeze(0)
        gripper_cmd = commands[7] if len(commands) > 7 else 0.0
        
        # 3. 현재 로봇 상태 획득
        ee_pos = robot.data.body_pos_w[:, self.ee_body_id]
        ee_quat = robot.data.body_quat_w[:, self.ee_body_id]
        jacobian = robot.root_physx_view.get_jacobians()[:, self.jacobian_idx]
        joint_pos = robot.data.joint_pos[:, self.joint_ids]
        
        # 4. IK 계산
        self.ik_controller.set_command(target_pose)
        joint_pos_des = self.ik_controller.compute(
            ee_pos, ee_quat, jacobian, joint_pos
        )
        
        return joint_pos_des, gripper_cmd
```

### 모바일 베이스 VR 제어 (조이스틱)

```python
# VR 컨트롤러 썸스틱으로 모바일 베이스 제어
def get_base_velocity_from_thumbstick(xr_device) -> tuple:
    """왼손 썸스틱 → 모바일 베이스 속도"""
    data = xr_device.get_controller_data()
    thumbstick_x = data["left_thumbstick_x"]
    thumbstick_y = data["left_thumbstick_y"]
    
    linear_vel = thumbstick_y * 0.5   # 최대 0.5 m/s
    angular_vel = -thumbstick_x * 1.0  # 최대 1.0 rad/s
    return linear_vel, angular_vel
```

---

## 4. 전체 통합 코드

### ArticulationCfg 정의 (매니퓰레이터 + 모바일 베이스)

```python
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.sim as sim_utils

MOBILE_MANIPULATOR_CFG = ArticulationCfg(
    # USD 파일 로딩
    spawn=sim_utils.UsdFileCfg(
        usd_path="./assets/mobile_openmanipulator.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=64,
            solver_velocity_iteration_count=4,
        ),
    ),
    
    # 초기 상태
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        joint_pos={
            # 모바일 베이스 (Skid-steer)
            "wheel_fl_joint": 0.0,
            "wheel_fr_joint": 0.0,
            "wheel_rl_joint": 0.0,
            "wheel_rr_joint": 0.0,
            # OpenMANIPULATOR-X
            "joint1": 0.0,
            "joint2": -0.5,
            "joint3": 0.3,
            "joint4": 0.2,
            # 그리퍼
            "gripper": 0.019,
        },
    ),
    
    # 액추에이터 그룹
    actuators={
        # 바퀴 (속도 제어)
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=["wheel_.*"],
            effort_limit_sim=20.0,
            velocity_limit_sim=10.0,
            stiffness=0.0,
            damping=1e4,
        ),
        # 암 (위치 제어)
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-4]"],
            effort_limit_sim=10.0,
            velocity_limit_sim=2.0,
            stiffness=80.0,
            damping=4.0,
        ),
        # 그리퍼 (위치 제어)
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["gripper.*"],
            effort_limit_sim=5.0,
            velocity_limit_sim=0.5,
            stiffness=200.0,
            damping=10.0,
        ),
    },
)
```

### 텔레오퍼레이션 환경 설정

```python
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnvCfg, ManagerBasedEnv
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.utils import configclass

@configclass
class MobileManipulatorSceneCfg(InteractiveSceneCfg):
    """씬 설정"""
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0))
    robot: ArticulationCfg = MOBILE_MANIPULATOR_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

@configclass 
class ActionsCfg:
    """액션 설정 - IK + 그리퍼 + 베이스"""
    # 암 IK 제어
    arm_action = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["joint[1-4]"],
        body_name="end_effector_link",
        controller=DifferentialIKControllerCfg(command_type="pose", ik_method="dls"),
    )
    # 그리퍼 바이너리 제어
    gripper_action = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["gripper"],
        open_command_expr={"gripper": 0.019},
        close_command_expr={"gripper": 0.0},
    )
    # 모바일 베이스 속도 제어
    base_action = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["wheel_.*"],
    )

@configclass
class ObservationsCfg:
    """관측 설정"""
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        ee_pos = ObsTerm(func=mdp.body_pos_w, params={"asset_cfg": SceneEntityCfg("robot", body_names=["end_effector_link"])})
        ee_quat = ObsTerm(func=mdp.body_quat_w, params={"asset_cfg": SceneEntityCfg("robot", body_names=["end_effector_link"])})
        gripper_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": SceneEntityCfg("robot", joint_names=["gripper"])})
        def __post_init__(self):
            self.concatenate_terms = True
    policy: PolicyCfg = PolicyCfg()

@configclass
class MobileManipulatorTeleopEnvCfg(ManagerBasedEnvCfg):
    """전체 환경 설정"""
    scene = MobileManipulatorSceneCfg(num_envs=1, env_spacing=4.0)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    
    def __post_init__(self):
        self.decimation = 4
        self.sim.dt = 0.005  # 200Hz 물리
        self.sim.render_interval = self.decimation
```

### 메인 텔레오퍼레이션 스크립트

```python
# run_teleop.py
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--teleop_device", type=str, default="handtracking")
parser.add_argument("--record", action="store_true")
parser.add_argument("--dataset_file", type=str, default="./datasets/demo.hdf5")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
from config.env_cfg import MobileManipulatorTeleopEnvCfg
from isaaclab.envs import ManagerBasedEnv

def main():
    # 환경 생성
    env_cfg = MobileManipulatorTeleopEnvCfg()
    env = ManagerBasedEnv(cfg=env_cfg)
    
    # 텔레옵 디바이스 초기화
    if args.teleop_device == "handtracking":
        from isaaclab.devices import OpenXRDevice, OpenXRDeviceCfg
        device = OpenXRDevice(OpenXRDeviceCfg(...))
    else:
        from isaaclab.devices import Se3SpaceMouse
        device = Se3SpaceMouse(pos_sensitivity=0.1)
    
    # 녹화 설정
    recorder = None
    if args.record:
        from isaaclab.utils.datasets import HDF5DatasetFileHandler
        recorder = HDF5DatasetFileHandler(args.dataset_file)
    
    # 메인 루프
    obs, info = env.reset()
    episode_data = {"obs": [], "actions": []}
    
    while simulation_app.is_running():
        with torch.inference_mode():
            # VR 명령 획득
            commands = device.advance()
            if commands is None:
                env.step(torch.zeros(env.num_envs, env.action_manager.total_action_dim))
                continue
            
            # 액션 구성: [arm_pose(6), gripper(1), base_vel(4)]
            action = torch.zeros(env.num_envs, 11, device=env.device)
            action[:, :6] = torch.tensor(commands[:6])  # 암 SE3
            action[:, 6] = commands[6]                   # 그리퍼
            # 모바일 베이스 (썸스틱)
            base_linear, base_angular = get_base_from_thumbstick(device)
            wheel_vels = compute_skid_steer_vels(base_linear, base_angular)
            action[:, 7:11] = torch.tensor(wheel_vels)
            
            # 녹화
            if recorder:
                episode_data["obs"].append(obs["policy"].cpu().numpy())
                episode_data["actions"].append(action.cpu().numpy())
            
            # 환경 스텝
            obs, info = env.step(action)
            
            # 리셋 콜백
            if device.reset_triggered:
                if recorder:
                    recorder.write_episode(episode_data)
                    episode_data = {"obs": [], "actions": []}
                obs, info = env.reset()
    
    if recorder:
        recorder.close()
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
```

### 데모 녹화 및 모방학습 데이터 수집

```bash
# 데모 수집 (내장 스크립트)
./isaaclab.sh -p scripts/tools/record_demos.py \
    --task Isaac-MobileManipulator-Teleop-v0 \
    --teleop_device handtracking \
    --dataset_file ./datasets/teleop_demos.hdf5 \
    --num_demos 20 \
    --device cpu

# 데모 재생 검증
./isaaclab.sh -p scripts/tools/replay_demos.py \
    --task Isaac-MobileManipulator-Teleop-v0 \
    --dataset_file ./datasets/teleop_demos.hdf5

# Robomimic으로 BC 학습
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-MobileManipulator-v0 \
    --algo bc \
    --dataset ./datasets/teleop_demos.hdf5
```

**HDF5 데이터 구조:**
```
teleop_demos.hdf5
├── data/
│   ├── demo_0/
│   │   ├── obs/joint_pos, joint_vel, ee_pos, ee_quat, gripper_pos
│   │   ├── actions/
│   │   └── dones/
│   └── demo_N/
└── env_args/
```

---

## 권장 프로젝트 구조

```
mobile_manipulator_teleop/
├── config/
│   ├── robot_cfg.py           # ArticulationCfg
│   ├── env_cfg.py             # ManagerBasedEnvCfg
│   └── openmanipulator_descriptor.yaml
├── assets/
│   ├── mobile_openmanipulator.usd
│   └── open_manipulator_x.urdf
├── scripts/
│   ├── run_teleop.py
│   ├── record_demos.py
│   └── train_policy.py
├── datasets/
└── docker/
    └── docker-compose.cloudxr.yaml
```

---

## 핵심 체크리스트

| 단계 | 항목 | 확인 |
|------|------|------|
| **Lula IK** | robot_descriptor.yaml cspace에 joint1-4만 포함 | ☐ |
| | 그리퍼 조인트 Fixed 또는 제외 | ☐ |
| | end_effector_link 프레임명 URDF 일치 | ☐ |
| **모바일 베이스** | 4개 Revolute Joint 생성 | ☐ |
| | Angular Drive Velocity 모드 (Stiffness=0) | ☐ |
| | ArticulationRoot Floating base 설정 | ☐ |
| **CloudXR** | Early Access 가입 및 승인 | ☐ |
| | Docker 환경 및 포트 설정 | ☐ |
| | XR_RUNTIME_JSON 환경변수 | ☐ |
| **통합** | ArticulationCfg 액추에이터 그룹 분리 | ☐ |
| | HDF5 데모 녹화 테스트 | ☐ |

이 가이드를 따라 구현하면 **OpenMANIPULATOR-X + 4륜 모바일 베이스**를 Meta Quest 3S로 텔레오퍼레이션하고, 모방학습용 데이터를 수집할 수 있는 완전한 시스템을 구축할 수 있습니다.