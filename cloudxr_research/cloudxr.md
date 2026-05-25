# NVIDIA CloudXR과 Isaac Lab으로 구현하는 Meta Quest 3S 듀얼 암 텔레오퍼레이션 완전 가이드

VR 헤드셋에서 실시간 로봇 양팔을 제어하는 것이 이제 **Isaac Lab 2.3과 CloudXR 4.0/5.0**의 결합으로 가능해졌습니다. 이 가이드는 Meta Quest 3S를 사용한 듀얼 암 텔레오퍼레이션 시스템의 전체 구축 과정을 다룹니다. 핵심은 CloudXR이 VR 스트리밍과 핸드 트래킹 데이터 전송을 담당하고, Isaac Lab의 **OpenXRDevice**와 **Retargeter** 시스템이 손 동작을 로봇 명령으로 변환한다는 점입니다. 권장 구성은 RTX 4090/5090 GPU, WiFi 6 네트워크, Ubuntu 22.04 Docker 환경입니다.

---

## CloudXR SDK 설치와 서버 구성

CloudXR SDK **4.0.1**은 NVIDIA의 Early Access 프로그램을 통해서만 제공됩니다. https://developer.nvidia.com/cloudxr-sdk-early-access-program에서 회사 이메일로 등록한 후 승인을 받아야 합니다.

### 시스템 요구사항

| 구성요소 | 최소 사양 | 권장 사양 |
|---------|----------|----------|
| GPU | RTX 3090 (Ampere) | **RTX 4090** / RTX 5090 |
| 드라이버 | 552.74+ | 최신 Game Ready |
| OS | Ubuntu 22.04 / Windows 11 | Ubuntu 22.04 |
| 메모리 | 32GB | **64GB** |
| CPU | 8-Core | 16-Core Threadripper Pro |
| 네트워크 | WiFi 5 | **WiFi 6/6E (5GHz)** |

RTX 5090은 공식 테스트 목록에 없지만, NVENC 인코더를 포함한 Ada/Blackwell 아키텍처이므로 완전 호환됩니다. A100/H100 같은 **컴퓨트 전용 GPU는 지원되지 않습니다** (그래픽 드라이버 필요).

### 방화벽 포트 개방

```bash
# UDP 포트 (비디오/오디오 스트리밍)
sudo ufw allow 47998:48000,48005,48008,48012/udp

# TCP 포트 (시그널링)
sudo ufw allow 48010/tcp
```

---

## CloudXR Runtime Docker 설정

Isaac Lab 텔레오퍼레이션에서는 **Docker 기반 CloudXR Runtime 5.0.1**을 사용하는 것이 표준입니다.

### 사전 설치 요구사항

```bash
# Docker Engine 26.0.0+ 설치
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# NVIDIA Container Toolkit 설치
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# Docker 런타임 구성 및 재시작
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# GPU 접근 확인
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi
```

### CloudXR Runtime 컨테이너 실행 (독립 실행)

```bash
# 공유 디렉토리 생성
mkdir -p $(pwd)/openxr

# CloudXR Runtime 시작
docker run -it --rm --name cloudxr-runtime \
    --user $(id -u):$(id -g) \
    --gpus=all \
    -e "ACCEPT_EULA=Y" \
    --mount type=bind,src=$(pwd)/openxr,dst=/openxr \
    -p 48010:48010 \
    -p 47998:47998/udp \
    -p 47999:47999/udp \
    -p 48000:48000/udp \
    -p 48005:48005/udp \
    -p 48008:48008/udp \
    -p 48012:48012/udp \
    nvcr.io/nvidia/cloudxr-runtime:5.0.1
```

---

## Meta Quest 3/3S 클라이언트 앱 설치

CloudXR SDK 4.0+는 **사전 빌드된 APK를 제공하지 않으므로** 직접 빌드해야 합니다.

### 빌드 환경 준비

1. **Android Studio** 설치 (JDK 13 포함)
2. **OVR Mobile SDK 1.46.0** 다운로드 (developer.oculus.com)
3. **Google Oboe SDK 1.5.0** 다운로드

### APK 빌드 과정

```bash
# SDK 디렉토리에 필요 파일 복사
cp ovr_mobile_sdk.zip {sdk-root}/Sample/Android/OculusVR/app/libs/
cp oboe-1.5.0.aar {sdk-root}/Sample/Android/OculusVR/app/libs/
cp {sdk-root}/Client/Lib/Android/CloudXR.aar {sdk-root}/Sample/Android/OculusVR/app/libs/

# Gradle 빌드
cd {sdk-root}/Sample/Android/OculusVR
./gradlew build
# 출력: app/build/outputs/apk/debug/CloudXRClient.apk
```

### Quest 3S에 설치

```bash
# 1. Meta Quest 개발자 모드 활성화 (Meta Quest 모바일 앱에서)

# 2. USB 연결 후 ADB 디버깅 허용

# 3. APK 사이드로딩
adb install -r CloudXRClient.apk

# 4. 서버 IP 설정 파일 푸시
echo "cmd -s 192.168.1.100" > CloudXRLaunchOptions.txt
adb push CloudXRLaunchOptions.txt /sdcard/Android/data/com.nvidia.cloudxr.ovr/files/
```

Quest에서 **라이브러리 → Unknown Sources → CloudXR Client**를 실행하면 서버에 자동 연결됩니다.

---

## Isaac Sim 5.1.0에서 CloudXR Extension 활성화

### Docker Compose 방식 (권장)

```bash
# Isaac Lab 디렉토리에서 CloudXR Runtime과 함께 컨테이너 시작
./docker/container.py start \
    --files docker-compose.cloudxr-runtime.patch.yaml \
    --env-file .env.cloudxr-runtime

# 컨테이너 진입
./docker/container.py enter base

# 텔레오퍼레이션 스크립트 실행
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-PickPlace-GR1T2-Abs-v0 \
    --teleop_device handtracking \
    --enable_pinocchio
```

### Isaac Sim UI에서 수동 활성화

1. **AR Panel** 열기
2. **Selected Output Plugin**: `OpenXR` 선택
3. **OpenXR Runtime**: `System OpenXR Runtime` (Docker CloudXR 사용 시)
4. **Start AR** 클릭 → 스테레오 렌더링 확인

### 환경 변수 설정 (로컬 프로세스 방식)

```bash
export XDG_RUNTIME_DIR=$(pwd)/openxr/run
export XR_RUNTIME_JSON=$(pwd)/openxr/share/openxr/1/openxr_cloudxr.json
```

---

## OpenXRDevice 클래스와 핸드 트래킹 데이터

**OpenXRDevice**는 CloudXR로부터 XR 핸드 트래킹 데이터를 수신하고 Retargeter를 통해 로봇 명령으로 변환하는 핵심 클래스입니다.

### 핸드 트래킹 데이터 구조

각 손에 대해 **26개 관절**의 7D 포즈(위치 3D + 쿼터니언 4D)가 수신됩니다:

```python
# 관절 이름 (손당 26개)
HAND_JOINTS = [
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    # middle, ring, little 손가락도 동일 구조
]

# TrackingTarget 열거형
class TrackingTarget(Enum):
    HAND_LEFT = 0   # 왼손
    HAND_RIGHT = 1  # 오른손
    HEAD = 2        # 헤드셋
```

### OpenXRDevice 초기화 예제

```python
from isaaclab.devices import OpenXRDevice, OpenXRDeviceCfg
from isaaclab.devices.openxr.retargeters import Se3AbsRetargeter, GripperRetargeter
from isaaclab.devices import DeviceBase

# Retargeter 설정
position_retargeter = Se3AbsRetargeter(
    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
    zero_out_xy_rotation=True,
    use_wrist_position=False  # 핀치 위치(엄지-검지 중간점) 사용
)
gripper_retargeter = GripperRetargeter(
    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT
)

# OpenXR 디바이스 생성
device = OpenXRDevice(
    OpenXRDeviceCfg(xr_cfg=env_cfg.xr),
    retargeters=[position_retargeter, gripper_retargeter],
)
```

---

## Retargeter 시스템으로 손 동작을 로봇 명령으로 변환

### Se3AbsRetargeter (절대 위치 매핑)

손 위치를 로봇 엔드이펙터의 **절대 좌표**로 직접 매핑합니다. 1:1 공간 제어에 적합합니다.

```python
from isaaclab.devices.openxr.retargeters import Se3AbsRetargeterCfg

left_arm_cfg = Se3AbsRetargeterCfg(
    bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
    zero_out_xy_rotation=False,  # 모든 회전 허용
    use_wrist_rotation=True,
    use_wrist_position=True,
    enable_visualization=True,
    device="cuda"
)
# 출력: 7D 텐서 [x, y, z, qw, qx, qy, qz]
```

### Se3RelRetargeter (상대 위치 매핑)

연속 프레임 간의 **델타(변화량)**를 계산하여 증분 이동을 생성합니다.

```python
from isaaclab.devices.openxr.retargeters import Se3RelRetargeterCfg

rel_cfg = Se3RelRetargeterCfg(
    bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
    delta_pos_scale_factor=1.5,  # 위치 변화 증폭
    delta_rot_scale_factor=1.0,
    alpha_pos=0.8,  # 위치 스무딩 (0-1)
    alpha_rot=0.7,  # 회전 스무딩
)
# 출력: 6D 텐서 [dx, dy, dz, rx, ry, rz]
```

### GripperRetargeter (그리퍼 제어)

**엄지-검지 거리**를 측정하여 그리퍼 열림/닫힘을 결정합니다. Hysteresis가 적용되어 빠른 토글링을 방지합니다.

```python
# 내부 상수
GRIPPER_CLOSE_METERS = 0.03  # 3cm 이하면 닫기
GRIPPER_OPEN_METERS = 0.05   # 5cm 이상이면 열기

# GripperRetargeter 설정
gripper_cfg = GripperRetargeterCfg(
    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT
)
# 출력: [-1.0] (열기) 또는 [1.0] (닫기)
```

---

## 듀얼 암 로봇 환경 설정

### 듀얼 암 ArticulationCfg 정의

```python
from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils

DUAL_FRANKA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="{NUCLEUS_DIR}/Robots/dual_franka.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={"left_.*": 0.0, "right_.*": 0.0},
    ),
    actuators={
        "left_arm": ImplicitActuatorCfg(joint_names_expr=["left_panda_joint.*"]),
        "right_arm": ImplicitActuatorCfg(joint_names_expr=["right_panda_joint.*"]),
    },
)
```

### 양손 바인딩 전체 설정

```python
# 왼팔 리타게터
left_arm_retargeter = Se3AbsRetargeter(Se3AbsRetargeterCfg(
    bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
    use_wrist_rotation=True,
    use_wrist_position=True,
))
left_gripper_retargeter = GripperRetargeter(GripperRetargeterCfg(
    bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
))

# 오른팔 리타게터
right_arm_retargeter = Se3AbsRetargeter(Se3AbsRetargeterCfg(
    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
    use_wrist_rotation=True,
    use_wrist_position=True,
))
right_gripper_retargeter = GripperRetargeter(GripperRetargeterCfg(
    bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
))

# OpenXR 디바이스에 모든 리타게터 등록
device = OpenXRDevice(
    cfg=OpenXRDeviceCfg(),
    retargeters=[
        left_arm_retargeter, left_gripper_retargeter,
        right_arm_retargeter, right_gripper_retargeter,
    ]
)
```

---

## IK 솔버 통합

### DifferentialIKController (Isaac Lab 내장)

PhysX Jacobian을 활용한 빠른 미분 IK 솔버입니다.

```python
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg

diff_ik_cfg = DifferentialIKControllerCfg(
    command_type="pose",
    use_relative_mode=True,  # 텔레오퍼레이션에 적합
    ik_method="dls",  # Damped Least Squares
    ik_params={"lambda_val": 0.1}
)

ik_controller = DifferentialIKController(
    cfg=diff_ik_cfg,
    num_envs=num_envs,
    device=sim.device
)

# IK 계산
joint_target = ik_controller.compute(
    ee_pos=current_ee_pos,
    ee_quat=current_ee_quat,
    jacobian=robot_jacobian,
    joint_pos=current_joint_pos
)
```

### cuRobo (GPU 가속 모션 플래닝)

충돌 회피가 내장된 고성능 IK/모션 플래닝 라이브러리입니다. 듀얼 암 설정 파일(`dual_ur10e.yml`)을 지원합니다.

```python
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

ik_config = IKSolverConfig.load_from_robot_config(
    robot_cfg="franka.yml",
    world_cfg=world_config,
    tensor_args=tensor_args,
)
ik_solver = IKSolver(ik_config)

result = ik_solver.solve_batch(
    goal_pose=target_poses,
    seed_config=current_joint_config,
)
```

---

## 실시간 제어 루프 구현

```python
def run_dual_arm_teleop(sim, scene, teleop_device):
    """듀얼 암 텔레오퍼레이션 메인 루프"""
    
    robot_left = scene["robot_left"]
    robot_right = scene["robot_right"]
    
    # IK 컨트롤러 초기화
    ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls")
    ik_left = DifferentialIKController(ik_cfg, num_envs, sim.device)
    ik_right = DifferentialIKController(ik_cfg, num_envs, sim.device)
    
    while simulation_app.is_running():
        # 1. VR 디바이스에서 명령 획득
        commands = teleop_device.advance()
        if commands is None:
            continue
        
        # 2. 명령 분리 (좌/우 팔 + 그리퍼)
        # [left_pose(7), left_gripper(1), right_pose(7), right_gripper(1)]
        left_pose = commands[:7]
        left_grip = commands[7]
        right_pose = commands[8:15]
        right_grip = commands[15]
        
        # 3. IK 계산
        ik_left.set_command(left_pose)
        left_joint_target = ik_left.compute(
            ee_pos=robot_left.data.body_pos_w[:, ee_idx],
            ee_quat=robot_left.data.body_quat_w[:, ee_idx],
            jacobian=robot_left.data.jacobian[:, ee_frame_idx],
            joint_pos=robot_left.data.joint_pos[:, arm_joints]
        )
        
        ik_right.set_command(right_pose)
        right_joint_target = ik_right.compute(...)
        
        # 4. 관절 목표 적용
        robot_left.set_joint_position_target(left_joint_target, joint_ids=arm_joints)
        robot_right.set_joint_position_target(right_joint_target, joint_ids=arm_joints)
        
        # 5. 그리퍼 제어
        gripper_left = 0.04 if left_grip > 0 else 0.0
        gripper_right = 0.04 if right_grip > 0 else 0.0
        robot_left.set_joint_position_target(torch.full((num_envs, 2), gripper_left), joint_ids=gripper_joints)
        robot_right.set_joint_position_target(torch.full((num_envs, 2), gripper_right), joint_ids=gripper_joints)
        
        # 6. 시뮬레이션 스텝
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
```

---

## 전체 Launch 명령어 모음

```bash
# 기본 핸드 트래킹 텔레오퍼레이션 (Franka)
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-Stack-Cube-Franka-IK-Abs-v0 \
    --teleop_device handtracking \
    --device cpu

# GR1T2 휴머노이드 bimanual 텔레오퍼레이션
./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py \
    --task Isaac-PickPlace-GR1T2-Abs-v0 \
    --teleop_device handtracking \
    --enable_pinocchio

# 데모 녹화 (imitation learning용)
./isaaclab.sh -p scripts/tools/record_demos.py \
    --device cpu \
    --task Isaac-PickPlace-GR1T2-Abs-v0 \
    --teleop_device handtracking \
    --dataset_file ./datasets/dual_arm_demos.hdf5 \
    --num_demos 10 \
    --enable_pinocchio
```

---

## 트러블슈팅 가이드

### 연결 문제

| 증상 | 원인 | 해결 방법 |
|------|------|----------|
| 검은 화면 | GPU 인덱스 불일치 | `export NV_GPU_INDEX=0` 설정 |
| XR_ERROR_INSTANCE_LOST | CloudXR 런타임 먼저 종료 | 런타임 재시작 |
| 48010 연결 실패 | 포트 차단 | 방화벽 규칙 확인 |
| 높은 지연 | 무선 네트워크 | 5GHz WiFi 사용, 유선 서버 연결 |

### 연결 테스트

```bash
# TCP 시그널링 포트 확인
nc -vz <server-ip> 48010
# 예상: Connection to <ip> port 48010 [tcp/*] succeeded!

# Docker 컨테이너 상태 확인
docker ps | grep cloudxr
```

### 무시해도 되는 경고

- `XR_ERROR_VALIDATION_FAILURE: xrWaitFrame` - AR 모드 중지 시 경쟁 상태
- `TF_PYTHON_EXCEPTION` - AR 진입/종료 시 발생
- `Invalid version string` - 구버전 USD 셰이더 호환성

---

## 성능 최적화 설정

### VR 스트리밍 품질

```python
@configclass
class XrTeleopEnvCfg(ManagerBasedRLEnvCfg):
    def __post_init__(self):
        self.sim.dt = 1.0 / 90  # 90Hz 시뮬레이션
        self.sim.render_interval = 2  # 45Hz 렌더링
```

### 환경 변수 최적화

```bash
# 고정 타임스텝 (가변 렌더 시간 문제 해결)
export NV_PACER_FIXED_TIME_STEP_MS=11

# 네트워크 QoS 힌트 (클라이언트 측)
-nic wifi5ghz -nt lan
```

### 권장 네트워크 구성

- 서버: **유선 이더넷** (1Gbps+)
- 클라이언트: **WiFi 6 (5GHz)**, 160MHz 채널 폭
- 대역폭: **100Mbps+** 지속
- 지연: **60ms 미만** (목표 30ms)

---

## 실제 로봇 연동 (ROS2 Bridge)

```bash
# ROS2 Humble 환경 소싱
source /opt/ros/humble/setup.bash

# Isaac Sim에서 ROS2 Bridge 확장 활성화
./isaaclab.sh -p <script> --enable isaacsim.ros2.bridge

# Sim2Real 정책 내보내기
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task=Velocity-G1-Student-Finetune-v1 \
    --num_envs=32
# 출력: export/ 디렉토리에 .onnx 파일 생성
```

---

## 결론

NVIDIA CloudXR과 Isaac Lab의 결합은 **VR 헤드셋 기반 로봇 텔레오퍼레이션**의 진입 장벽을 크게 낮췄습니다. Quest 3S 클라이언트 APK 빌드가 필요하다는 점과 WiFi 6 네트워크 구성이 중요하다는 점을 제외하면, Docker 컨테이너 기반 설정으로 **30분 내에 동작하는 시스템을 구축**할 수 있습니다. 핵심 통찰은 **Retargeter 선택**입니다: 정밀한 작업에는 Se3AbsRetargeter, 넓은 작업 공간에서의 자연스러운 조작에는 Se3RelRetargeter가 적합합니다. 듀얼 암 시스템에서는 양손 각각에 독립적인 Retargeter를 바인딩하고, DifferentialIKController나 cuRobo를 통해 실시간 IK를 계산하는 것이 검증된 아키텍처입니다.