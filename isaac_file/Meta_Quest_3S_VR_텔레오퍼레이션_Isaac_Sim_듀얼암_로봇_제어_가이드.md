# Meta Quest 3S VR 텔레오퍼레이션 - Isaac Sim 듀얼 암 로봇 제어 구현 가이드

Isaac Sim에서 듀얼 암 로봇의 VR 텔레오퍼레이션을 구현하려면 **컨트롤러 트래킹을 위한 OpenXR/SteamVR 통합**, **매니퓰레이터 조작을 위한 Lula IK 솔버**, **모바일 베이스 제어를 위한 차동 구동 기구학**을 통합해야 합니다. 이 가이드는 지정된 하드웨어 구성(USB Link로 연결된 Meta Quest 3S, 4륜 TurtleBot 베이스 위의 듀얼 OpenMANIPULATOR-X 암)에 대한 코드 예제와 함께 완전한 구현 경로를 제공합니다.

권장 아키텍처는 컨트롤러 입력 캡처를 위해 **pyopenvr**(또는 pyopenxr), 실시간 데이터 스트리밍을 위해 **ZMQ**, IK 기반 매니퓰레이터 제어를 위해 **Isaac Sim의 ArticulationKinematicsSolver**를 사용합니다. 유선 USB 연결로 **300ms 미만**의 엔드투엔드 지연 시간을 달성할 수 있으며, 이는 효과적인 텔레오퍼레이션을 위한 임계값 내에 충분히 들어옵니다.

---

## 목차

1. [시스템 개요 및 아키텍처](#1-시스템-개요-및-아키텍처)
2. [Meta Quest 3S PC 통합 설정](#2-meta-quest-3s-pc-통합-설정)
3. [VR 컨트롤러 입력 캡처](#3-vr-컨트롤러-입력-캡처)
4. [ZMQ 기반 저지연 통신](#4-zmq-기반-저지연-통신)
5. [듀얼 암 IK 제어 구현](#5-듀얼-암-ik-제어-구현)
6. [TurtleBot 4륜 스키드 스티어 제어](#6-turtlebot-4륜-스키드-스티어-제어)
7. [버튼 상태 머신 및 모드 관리](#7-버튼-상태-머신-및-모드-관리)
8. [그리퍼 제어](#8-그리퍼-제어)
9. [전체 시스템 통합](#9-전체-시스템-통합)
10. [의존성 및 설치 체크리스트](#10-의존성-및-설치-체크리스트)
11. [결론 및 향후 개선 사항](#11-결론-및-향후-개선-사항)

---

## 1. 시스템 개요 및 아키텍처

### 1.1 하드웨어 구성

| 구성 요소 | 사양 |
|----------|------|
| VR 헤드셋 | Meta Quest 3S |
| 연결 방식 | USB 3.0+ 유선 (Quest Link) |
| 매니퓰레이터 | OpenMANIPULATOR-X × 2 (듀얼 암) |
| 모바일 베이스 | TurtleBot (4륜 구성) |
| 시뮬레이터 | Isaac Sim 5.1.0 |

### 1.2 컨트롤 매핑

| 입력 | 기능 |
|------|------|
| 버튼 1 (A/X) | 컨트롤러 ↔ 시뮬레이터 연결 On/Off |
| 양측 트리거 1 (인덱스 트리거) | 시뮬레이터 ↔ 로봇 제어 연결 On/Off |
| 양측 트리거 2 (그립 트리거) | 그리퍼 Open/Close |
| 양측 조이스틱 | 모바일 로봇 바퀴 (왼쪽/오른쪽 각각) 전진/후진 |
| 컨트롤러 위치/회전 | 각 매니퓰레이터 IK 타겟 제어 |

### 1.3 시스템 아키텍처 다이어그램

```
┌─────────────────────────────────────────────────────────────────────┐
│                      프로세스 1: VR 입력                             │
│  ┌──────────────┐    ┌─────────────────┐    ┌─────────────────┐   │
│  │ Meta Quest   │───▶│  pyopenvr       │───▶│ ZMQ Publisher   │   │
│  │ 3S + SteamVR │    │  컨트롤러       │    │ (TCP:5555)      │   │
│  └──────────────┘    │  리더           │    └────────┬────────┘   │
│                      └─────────────────┘             │            │
└──────────────────────────────────────────────────────│────────────┘
                                                       │ 90 Hz
                                                       ▼
┌──────────────────────────────────────────────────────│────────────┐
│                     프로세스 2: Isaac Sim                          │
│  ┌─────────────────┐                                              │
│  │ ZMQ Subscriber  │◀────────────────────────────────┘            │
│  └────────┬────────┘                                              │
│           │                                                       │
│           ▼                                                       │
│  ┌─────────────────┐    ┌─────────────────┐    ┌──────────────┐  │
│  │ 버튼 상태       │───▶│ 모드 컨트롤러   │───▶│ 듀얼 암 IK   │  │
│  │ 머신            │    │                 │    │ (Lula)       │  │
│  └─────────────────┘    └────────┬────────┘    └──────┬───────┘  │
│                                  │                     │          │
│                                  ▼                     ▼          │
│                         ┌─────────────────┐    ┌──────────────┐  │
│                         │ 모바일 베이스   │    │ Articulation │  │
│                         │ 컨트롤러        │    │ Actions      │  │
│                         └─────────────────┘    └──────────────┘  │
└───────────────────────────────────────────────────────────────────┘
```

### 1.4 지연 시간 분석

| 구간 | 예상 지연 시간 |
|------|---------------|
| USB Link 전송 | 7-10 ms |
| ZMQ pub-sub | ≤35 ms |
| IK 계산 (Lula) | <1 ms |
| 물리 시뮬레이션 | ~16 ms (60Hz) |
| **총 엔드투엔드** | **<100 ms** |

효과적인 텔레오퍼레이션을 위한 목표 지연 시간은 **300ms 미만**이며, 유선 연결로 이를 충분히 달성할 수 있습니다.

---

## 2. Meta Quest 3S PC 통합 설정

### 2.1 Quest Link 개요

Meta Quest 3S는 Quest Link를 통해 USB 3.0+로 PC에 연결됩니다. 유선 연결은 무선 Air Link보다 **7-10ms 더 낮은 지연 시간**을 제공하며, 장시간 사용 시 헤드셋 충전도 동시에 가능합니다.

### 2.2 하드웨어 요구 사항

| 항목 | 최소 사양 | 권장 사양 |
|------|----------|----------|
| USB 케이블 | USB 3.0 (5 Gbps), 3m | USB 3.2 Gen 2, 5m (공식 Link 케이블) |
| GPU | NVIDIA GTX 1060 | NVIDIA RTX 2060+ |
| RAM | 8GB | 16GB |
| OS | Windows 10/11 | Windows 11 |

### 2.3 설정 절차

#### 단계 1: Meta Quest Link PC 앱 설치

```bash
# Meta 공식 사이트에서 다운로드
# https://www.meta.com/quest/setup/
```

#### 단계 2: USB 연결 및 테스트

1. USB-C 케이블을 PC의 USB 3.0 포트에 연결
2. Meta Quest Link 앱에서: **Devices → Add Headset → Link (Cable) 선택**
3. 연결 테스트: **Devices → USB Test → Test Connection**

#### 단계 3: 헤드셋 설정

1. 헤드셋에서: **Settings → System → Quest Link → Toggle ON**
2. "Allow" 버튼을 눌러 PC와의 연결 허용

#### 단계 4: OpenXR 런타임 설정

```
Meta Quest Link 앱에서:
Settings → General → "Set Meta Quest Link as active OpenXR Runtime"
```

### 2.4 SteamVR 설정 (pyopenvr 사용 시 필요)

```bash
# Steam에서 SteamVR 설치
# SteamVR → Settings → Developer → "Enable SteamVR Developer Mode"
```

---

## 3. VR 컨트롤러 입력 캡처

### 3.1 라이브러리 선택

Python 기반 VR 입력 캡처를 위한 두 가지 주요 라이브러리:

| 라이브러리 | 장점 | 단점 |
|-----------|------|------|
| **pyopenvr** | 성숙한 API, SteamVR 호환성 우수 | SteamVR 필요 |
| **pyopenxr** | OpenXR 표준, 미래 지향적 | API가 덜 직관적 |

Meta Quest + SteamVR 호환성을 위해 **pyopenvr**이 더 안정적인 선택입니다.

### 3.2 설치

```bash
pip install openvr pyopenxr pyzmq numpy scipy
```

### 3.3 컨트롤러 ID 탐지 함수

```python
import openvr
import numpy as np

def get_controller_ids(vrsystem):
    """왼쪽과 오른쪽 컨트롤러 디바이스 인덱스를 찾습니다."""
    left, right = None, None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsystem.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            role = vrsystem.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                right = i
            if role == openvr.TrackedControllerRole_LeftHand:
                left = i
    return left, right
```

### 3.4 포즈 변환 함수

```python
def matrix_to_pose(matrix):
    """HmdMatrix34_t에서 위치와 회전을 추출합니다."""
    # 위치 추출 (4열의 처음 3개 요소)
    position = np.array([matrix[0][3], matrix[1][3], matrix[2][3]])
    
    # 회전 행렬 추출 (3x3 상단 왼쪽)
    rot_matrix = np.array([
        [matrix[0][0], matrix[0][1], matrix[0][2]],
        [matrix[1][0], matrix[1][1], matrix[1][2]],
        [matrix[2][0], matrix[2][1], matrix[2][2]]
    ])
    return position, rot_matrix
```

### 3.5 컨트롤러 상태 추출 함수

```python
def get_controller_state(vrsystem, device_id):
    """컨트롤러에서 버튼 및 축 상태를 추출합니다."""
    result, state = vrsystem.getControllerState(device_id)
    
    return {
        # 트리거 (인덱스 트리거) - 0.0 ~ 1.0
        'trigger': state.rAxis[1].x,
        
        # 그립 트리거 - 0.0 ~ 1.0
        'grip': state.rAxis[2].x if len(state.rAxis) > 2 else 0,
        
        # 조이스틱 - -1.0 ~ 1.0
        'joystick_x': state.rAxis[0].x,
        'joystick_y': state.rAxis[0].y,
        
        # 버튼 상태 (비트 마스크에서 추출)
        'button_a': bool(state.ulButtonPressed >> 7 & 1),   # A/X 버튼
        'button_b': bool(state.ulButtonPressed >> 1 & 1),   # B/Y 버튼
        'grip_button': bool(state.ulButtonPressed >> 2 & 1),
        'trigger_pressed': state.rAxis[1].x > 0.9
    }
```

### 3.6 VR 컨트롤러 리더 클래스

```python
class VRControllerReader:
    """VR 컨트롤러 포즈와 상태를 읽는 클래스"""
    
    def __init__(self):
        # OpenVR 초기화
        openvr.init(openvr.VRApplication_Scene)
        self.vrsystem = openvr.VRSystem()
        self.compositor = openvr.VRCompositor()
        
        # 컨트롤러 ID 탐지
        self.left_id, self.right_id = get_controller_ids(self.vrsystem)
        self.poses = []
        
        print(f"왼쪽 컨트롤러 ID: {self.left_id}")
        print(f"오른쪽 컨트롤러 ID: {self.right_id}")
        
    def update(self):
        """최신 컨트롤러 포즈와 상태를 가져옵니다."""
        # 포즈 대기 및 업데이트
        self.poses, _ = self.compositor.waitGetPoses(self.poses, None)
        
        data = {'left': None, 'right': None}
        
        for side, device_id in [('left', self.left_id), ('right', self.right_id)]:
            if device_id is not None and self.poses[device_id].bPoseIsValid:
                matrix = self.poses[device_id].mDeviceToAbsoluteTracking
                position, rotation = matrix_to_pose(matrix)
                buttons = get_controller_state(self.vrsystem, device_id)
                
                data[side] = {
                    'position': position,
                    'rotation': rotation,
                    **buttons
                }
        
        return data
    
    def shutdown(self):
        """OpenVR 종료"""
        openvr.shutdown()
```

---

## 4. ZMQ 기반 저지연 통신

### 4.1 통신 아키텍처

BEAVR 텔레오퍼레이션 프레임워크의 연구 결과에 따르면 ZMQ pub-sub 패턴으로 **≤35ms 지연 시간**을 달성할 수 있습니다. 이 아키텍처는 VR 입력 캡처와 시뮬레이션 처리를 분리하여 장애 격리와 유연한 배포를 가능하게 합니다.

### 4.2 VR 데이터 퍼블리셔

```python
import zmq
import json
import time

class VRDataPublisher:
    """VR 컨트롤러 데이터를 ZMQ로 퍼블리시하는 클래스
    
    SteamVR과 함께 별도 프로세스로 실행됩니다.
    """
    
    def __init__(self, port=5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        self.vr_reader = VRControllerReader()
        
        print(f"VR 데이터 퍼블리셔 시작 - 포트: {port}")
        
    def run(self, rate_hz=90):
        """지정된 주파수로 VR 데이터를 스트리밍합니다."""
        period = 1.0 / rate_hz
        
        while True:
            start = time.time()
            
            # VR 데이터 업데이트
            data = self.vr_reader.update()
            
            # JSON 직렬화를 위해 numpy 배열 변환
            message = {
                'timestamp': time.time(),
                'left': self._serialize_controller(data['left']),
                'right': self._serialize_controller(data['right'])
            }
            
            # ZMQ로 전송
            self.socket.send_json(message)
            
            # 주파수 유지
            elapsed = time.time() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    
    def _serialize_controller(self, controller_data):
        """컨트롤러 데이터를 JSON 직렬화 가능한 형태로 변환"""
        if controller_data is None:
            return None
        
        return {
            'position': controller_data['position'].tolist(),
            'rotation': controller_data['rotation'].tolist(),
            'trigger': controller_data['trigger'],
            'grip': controller_data.get('grip', 0),
            'joystick_x': controller_data['joystick_x'],
            'joystick_y': controller_data['joystick_y'],
            'button_a': controller_data['button_a'],
            'button_b': controller_data['button_b'],
            'grip_button': controller_data.get('grip_button', False),
            'trigger_pressed': controller_data['trigger_pressed']
        }
    
    def shutdown(self):
        """리소스 정리"""
        self.vr_reader.shutdown()
        self.socket.close()
        self.context.term()


# 실행 스크립트
if __name__ == "__main__":
    publisher = VRDataPublisher(port=5555)
    try:
        publisher.run(rate_hz=90)
    except KeyboardInterrupt:
        print("\n종료 중...")
        publisher.shutdown()
```

### 4.3 Isaac Sim 데이터 서브스크라이버

```python
import zmq
import threading
import numpy as np

class VRDataSubscriber:
    """ZMQ를 통해 VR 데이터를 수신하는 클래스
    
    Isaac Sim 프로세스 내에서 실행됩니다.
    """
    
    def __init__(self, host="localhost", port=5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{host}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 모든 메시지 구독
        self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms 타임아웃
        
        self.latest_data = None
        self._lock = threading.Lock()
        self._running = True
        
        # 백그라운드 수신 스레드
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        
        print(f"VR 데이터 서브스크라이버 연결 - {host}:{port}")
    
    def _receive_loop(self):
        """백그라운드에서 데이터를 지속적으로 수신합니다."""
        while self._running:
            try:
                data = self.socket.recv_json()
                with self._lock:
                    self.latest_data = data
            except zmq.Again:
                pass  # 타임아웃, 계속
    
    def get_latest(self):
        """최신 VR 데이터를 반환합니다."""
        with self._lock:
            return self.latest_data
    
    def get_latest_numpy(self):
        """numpy 배열로 변환된 최신 데이터를 반환합니다."""
        data = self.get_latest()
        if data is None:
            return None
        
        result = {'left': None, 'right': None, 'timestamp': data['timestamp']}
        
        for side in ['left', 'right']:
            if data[side] is not None:
                result[side] = {
                    'position': np.array(data[side]['position']),
                    'rotation': np.array(data[side]['rotation']),
                    'trigger': data[side]['trigger'],
                    'grip': data[side].get('grip', 0),
                    'joystick_x': data[side]['joystick_x'],
                    'joystick_y': data[side]['joystick_y'],
                    'button_a': data[side]['button_a'],
                    'button_b': data[side]['button_b'],
                    'grip_button': data[side].get('grip_button', False),
                    'trigger_pressed': data[side]['trigger_pressed']
                }
        
        return result
    
    def stop(self):
        """서브스크라이버 종료"""
        self._running = False
        self._thread.join(timeout=1.0)
        self.socket.close()
        self.context.term()
```

---

## 5. 듀얼 암 IK 제어 구현

### 5.1 Lula IK 솔버 개요

Isaac Sim의 **LulaKinematicsSolver**는 각 매니퓰레이터에 대해 독립적으로 해석적 IK를 제공합니다. 듀얼 암 설정의 경우, 별도의 솔버 인스턴스를 생성하고 각 암의 베이스 포즈를 올바른 월드 프레임 변환을 위해 추적합니다.

### 5.2 좌표계 변환

VR 공간과 로봇 작업 공간 간의 좌표계 변환이 필요합니다:

```
VR 좌표계 (SteamVR):     로봇 좌표계 (Isaac Sim):
- Y-up                    - Z-up
- 미터 단위               - 미터 단위
- 오른손 좌표계           - 오른손 좌표계
```

### 5.3 회전 행렬 → 쿼터니언 변환 함수

```python
def rotation_matrix_to_quat(R):
    """3x3 회전 행렬을 wxyz 쿼터니언으로 변환합니다.
    
    Isaac Sim은 wxyz 형식의 쿼터니언을 사용합니다.
    """
    trace = np.trace(R)
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    
    return np.array([w, x, y, z])
```

### 5.4 듀얼 암 IK 컨트롤러 클래스

```python
import numpy as np
from isaacsim.core.prims import Articulation, XFormPrim
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver, 
    ArticulationKinematicsSolver
)
from isaacsim.core.utils.types import ArticulationAction


class DualArmIKController:
    """듀얼 암 매니퓰레이터를 위한 IK 컨트롤러
    
    각 VR 컨트롤러가 하나의 매니퓰레이터를 제어합니다.
    """
    
    def __init__(self, left_arm_path, right_arm_path, 
                 robot_descriptor_path, urdf_path):
        """
        Args:
            left_arm_path: 왼쪽 암 USD 경로
            right_arm_path: 오른쪽 암 USD 경로
            robot_descriptor_path: Lula 설명자 YAML 경로
            urdf_path: URDF 파일 경로
        """
        # Articulation 초기화
        self.left_arm = Articulation(prim_path=left_arm_path)
        self.right_arm = Articulation(prim_path=right_arm_path)
        
        # 독립적인 IK 솔버 생성
        self.left_ik = LulaKinematicsSolver(
            robot_description_path=robot_descriptor_path,
            urdf_path=urdf_path
        )
        self.right_ik = LulaKinematicsSolver(
            robot_description_path=robot_descriptor_path,
            urdf_path=urdf_path
        )
        
        # Articulation-aware 솔버로 래핑
        self.left_art_ik = ArticulationKinematicsSolver(
            self.left_arm, self.left_ik, "end_effector_link"
        )
        self.right_art_ik = ArticulationKinematicsSolver(
            self.right_arm, self.right_ik, "end_effector_link"
        )
        
        # VR → 로봇 작업 공간 매핑 파라미터
        self.position_scale = 0.5  # VR 움직임 스케일
        self.workspace_bounds = np.array([
            [-0.4, 0.4],   # X 범위 (m)
            [-0.4, 0.4],   # Y 범위 (m)
            [0.0, 0.6]     # Z 범위 (m)
        ])
        
        # 델타 제어를 위한 초기 위치 저장
        self.left_home = None
        self.right_home = None
        self.vr_left_origin = None
        self.vr_right_origin = None
        
        # 연결 상태
        self.is_connected = False
        
    def initialize(self):
        """world reset 후 초기 위치를 가져옵니다."""
        self.left_arm.initialize()
        self.right_arm.initialize()
        
        # 홈 엔드이펙터 위치 저장
        self.left_home, _ = self.left_art_ik.compute_end_effector_pose()
        self.right_home, _ = self.right_art_ik.compute_end_effector_pose()
        
        print(f"왼쪽 암 홈 위치: {self.left_home}")
        print(f"오른쪽 암 홈 위치: {self.right_home}")
    
    def set_connected(self, connected):
        """VR 연결 상태를 설정합니다."""
        self.is_connected = connected
        if not connected:
            # 연결 해제 시 원점 초기화
            self.vr_left_origin = None
            self.vr_right_origin = None
    
    def set_vr_origin(self, left_vr_pos, right_vr_pos):
        """델타 매핑을 위한 VR 컨트롤러 원점을 설정합니다."""
        self.vr_left_origin = np.array(left_vr_pos)
        self.vr_right_origin = np.array(right_vr_pos)
        print(f"VR 원점 설정 - 왼쪽: {self.vr_left_origin}, 오른쪽: {self.vr_right_origin}")
    
    def vr_to_robot_target(self, vr_position, vr_rotation, 
                           vr_origin, robot_home):
        """VR 포즈를 로봇 작업 공간 타겟으로 변환합니다.
        
        Args:
            vr_position: VR 컨트롤러 위치 (3,)
            vr_rotation: VR 컨트롤러 회전 행렬 (3, 3)
            vr_origin: VR 원점 위치 (3,)
            robot_home: 로봇 홈 위치 (3,)
            
        Returns:
            target_pos: 타겟 위치 (3,)
            target_rot: 타겟 회전 쿼터니언 (4,) - wxyz
        """
        # VR 원점으로부터의 델타 계산
        vr_delta = (np.array(vr_position) - vr_origin) * self.position_scale
        
        # VR 좌표계 → 로봇 좌표계 변환 (Y-up → Z-up)
        robot_delta = np.array([vr_delta[2], -vr_delta[0], vr_delta[1]])
        
        # 로봇 홈 위치에 델타 적용
        target_pos = robot_home + robot_delta
        
        # 작업 공간 경계로 클램핑
        target_pos = np.clip(
            target_pos, 
            self.workspace_bounds[:, 0], 
            self.workspace_bounds[:, 1]
        )
        
        # 회전 행렬을 쿼터니언으로 변환
        # OpenMANIPULATOR-X는 4축이므로 위치 전용 모드 권장
        target_rot = None  # 솔버가 방향 결정
        
        return target_pos, target_rot
    
    def update(self, left_vr_data, right_vr_data):
        """양쪽 암의 IK를 계산하고 적용합니다.
        
        Args:
            left_vr_data: 왼쪽 컨트롤러 데이터 딕셔너리
            right_vr_data: 오른쪽 컨트롤러 데이터 딕셔너리
            
        Returns:
            dict: {'left': bool, 'right': bool} - IK 성공 여부
        """
        if not self.is_connected:
            return {'left': False, 'right': False}
        
        results = {'left': False, 'right': False}
        
        # IK 솔버용 베이스 포즈 업데이트
        left_base_pos, left_base_rot = self.left_arm.get_world_pose()
        right_base_pos, right_base_rot = self.right_arm.get_world_pose()
        
        self.left_ik.set_robot_base_pose(left_base_pos, left_base_rot)
        self.right_ik.set_robot_base_pose(right_base_pos, right_base_rot)
        
        # 왼쪽 암 처리
        if left_vr_data and self.vr_left_origin is not None:
            target_pos, target_rot = self.vr_to_robot_target(
                left_vr_data['position'],
                left_vr_data['rotation'],
                self.vr_left_origin,
                self.left_home
            )
            
            action, success = self.left_art_ik.compute_inverse_kinematics(
                target_pos, target_rot
            )
            
            if success:
                self.left_arm.apply_action(action)
                results['left'] = True
        
        # 오른쪽 암 처리
        if right_vr_data and self.vr_right_origin is not None:
            target_pos, target_rot = self.vr_to_robot_target(
                right_vr_data['position'],
                right_vr_data['rotation'],
                self.vr_right_origin,
                self.right_home
            )
            
            action, success = self.right_art_ik.compute_inverse_kinematics(
                target_pos, target_rot
            )
            
            if success:
                self.right_arm.apply_action(action)
                results['right'] = True
        
        return results
```

### 5.5 OpenMANIPULATOR-X Lula 설명자 파일

`config/open_manipulator_descriptor.yaml` 파일 생성:

```yaml
# OpenMANIPULATOR-X Lula 로봇 설명자
# 4축 매니퓰레이터 + 그리퍼

api_version: 1.0

# 로봇 설정
robot:
  urdf_path: "urdf/open_manipulator_x.urdf"

# 구성 공간 조인트 (4 DOF)
cspace:
  - joint1
  - joint2
  - joint3
  - joint4

# 루트 링크
root_link: world

# 기본 자세 (준비 자세)
default_q: [0.0, -1.0, 1.3, 0.0]

# 엔드이펙터 설정
end_effector:
  name: "end_effector_link"

# URDF 매핑 규칙
cspace_to_urdf_rules: []

# 복합 태스크 공간
composite_task_spaces: []

# 충돌 구체 정의
collision_spheres:
  - link: "link1"
    spheres:
      - center: [0.0, 0.0, 0.02]
        radius: 0.03
      - center: [0.0, 0.0, 0.04]
        radius: 0.035
        
  - link: "link2"
    spheres:
      - center: [0.024, 0.0, 0.0]
        radius: 0.025
      - center: [0.048, 0.0, 0.0]
        radius: 0.022
        
  - link: "link3"
    spheres:
      - center: [0.03, 0.0, 0.0]
        radius: 0.02
      - center: [0.06, 0.0, 0.0]
        radius: 0.02
        
  - link: "link4"
    spheres:
      - center: [0.02, 0.0, 0.0]
        radius: 0.018
      - center: [0.04, 0.0, 0.0]
        radius: 0.015

  - link: "end_effector_link"
    spheres:
      - center: [0.0, 0.0, 0.0]
        radius: 0.015
```

---

## 6. TurtleBot 4륜 스키드 스티어 제어

### 6.1 차동 구동 기구학

4륜 TurtleBot 구성에서 같은 쪽의 바퀴들은 동일한 속도 명령을 받습니다. **DifferentialController**는 유니사이클 기구학을 사용하여 선형/각속도 명령을 바퀴 각속도로 변환합니다.

```
바퀴 속도 계산:
- v_left = (2*v_linear - ω*L) / (2*r)
- v_right = (2*v_linear + ω*L) / (2*r)

여기서:
- v_linear: 선형 속도 (m/s)
- ω: 각속도 (rad/s)
- L: 트랙 폭 (m)
- r: 바퀴 반경 (m)
```

### 6.2 모바일 베이스 컨트롤러 클래스

```python
from isaacsim.core.utils.types import ArticulationAction
import numpy as np


class SkidSteerMobileBaseController:
    """4륜 스키드 스티어 모바일 베이스 컨트롤러
    
    조이스틱 입력을 바퀴 속도 명령으로 변환합니다.
    """
    
    def __init__(self, robot_articulation, wheel_joint_names,
                 wheel_radius=0.033, track_width=0.287):
        """
        Args:
            robot_articulation: 로봇 Articulation
            wheel_joint_names: [front_left, rear_left, front_right, rear_right]
            wheel_radius: 바퀴 반경 (m)
            track_width: 트랙 폭 (m)
        """
        self.robot = robot_articulation
        self.wheel_radius = wheel_radius
        self.track_width = track_width
        self.wheel_joint_names = wheel_joint_names
        
        # 조인트 인덱스 가져오기
        all_joints = self.robot.dof_names
        self.wheel_indices = [
            all_joints.index(name) for name in wheel_joint_names
        ]
        
        # 속도 제한
        self.max_linear_vel = 0.5    # m/s
        self.max_angular_vel = 1.0   # rad/s
        self.max_wheel_vel = 10.0    # rad/s
        
        print(f"모바일 베이스 초기화 - 바퀴 인덱스: {self.wheel_indices}")
        
    def joystick_to_velocity_arcade(self, left_stick_y, right_stick_x):
        """아케이드 스타일 제어로 조이스틱 입력을 속도로 변환합니다.
        
        - 왼쪽 스틱 Y: 전진/후진 선형 속도
        - 오른쪽 스틱 X: 좌/우 각속도
        
        Args:
            left_stick_y: 왼쪽 스틱 Y축 (-1 ~ 1)
            right_stick_x: 오른쪽 스틱 X축 (-1 ~ 1)
            
        Returns:
            tuple: (linear_vel, angular_vel)
        """
        linear_vel = left_stick_y * self.max_linear_vel
        angular_vel = right_stick_x * self.max_angular_vel
        return linear_vel, angular_vel
    
    def joystick_to_velocity_tank(self, left_stick_y, right_stick_y):
        """탱크 스타일 제어로 조이스틱 입력을 속도로 변환합니다.
        
        각 스틱이 한쪽 바퀴를 제어합니다.
        
        Args:
            left_stick_y: 왼쪽 스틱 Y축 (-1 ~ 1) - 왼쪽 바퀴
            right_stick_y: 오른쪽 스틱 Y축 (-1 ~ 1) - 오른쪽 바퀴
            
        Returns:
            tuple: (linear_vel, angular_vel)
        """
        # 탱크 입력을 차동 속도로 변환
        left_vel = left_stick_y * self.max_wheel_vel * self.wheel_radius
        right_vel = right_stick_y * self.max_wheel_vel * self.wheel_radius
        
        # 선형/각속도로 변환
        linear_vel = (left_vel + right_vel) / 2.0
        angular_vel = (right_vel - left_vel) / self.track_width
        
        return linear_vel, angular_vel
    
    def velocity_to_wheel_commands(self, linear_vel, angular_vel):
        """차동 구동 기구학으로 바퀴 속도를 계산합니다.
        
        Args:
            linear_vel: 선형 속도 (m/s)
            angular_vel: 각속도 (rad/s)
            
        Returns:
            np.array: 바퀴 속도 [FL, RL, FR, RR] (rad/s)
        """
        # 차동 구동 방정식
        left_wheel_vel = (
            2 * linear_vel - angular_vel * self.track_width
        ) / (2 * self.wheel_radius)
        
        right_wheel_vel = (
            2 * linear_vel + angular_vel * self.track_width
        ) / (2 * self.wheel_radius)
        
        # 제한으로 클램핑
        left_wheel_vel = np.clip(
            left_wheel_vel, -self.max_wheel_vel, self.max_wheel_vel
        )
        right_wheel_vel = np.clip(
            right_wheel_vel, -self.max_wheel_vel, self.max_wheel_vel
        )
        
        # 같은 쪽의 앞/뒤 바퀴에 동일한 속도 적용
        return np.array([
            left_wheel_vel, left_wheel_vel,    # FL, RL
            right_wheel_vel, right_wheel_vel   # FR, RR
        ])
    
    def apply_velocity_command(self, linear_vel, angular_vel):
        """속도 명령을 로봇 바퀴에 적용합니다."""
        wheel_vels = self.velocity_to_wheel_commands(linear_vel, angular_vel)
        
        action = ArticulationAction(
            joint_velocities=wheel_vels,
            joint_indices=np.array(self.wheel_indices)
        )
        self.robot.apply_action(action)
    
    def update_from_joysticks_arcade(self, left_joy_y, right_joy_x):
        """아케이드 스타일 제어를 위한 편의 메서드"""
        linear_vel, angular_vel = self.joystick_to_velocity_arcade(
            left_joy_y, right_joy_x
        )
        self.apply_velocity_command(linear_vel, angular_vel)
    
    def update_from_joysticks_tank(self, left_joy_y, right_joy_y):
        """탱크 스타일 제어를 위한 편의 메서드"""
        linear_vel, angular_vel = self.joystick_to_velocity_tank(
            left_joy_y, right_joy_y
        )
        self.apply_velocity_command(linear_vel, angular_vel)
    
    def stop(self):
        """모든 바퀴를 정지합니다."""
        self.apply_velocity_command(0.0, 0.0)
```

---

## 7. 버튼 상태 머신 및 모드 관리

### 7.1 텔레오퍼레이션 상태

```python
from enum import Enum, auto


class TeleoperationState(Enum):
    """텔레오퍼레이션 연결 상태"""
    DISCONNECTED = auto()       # VR이 시뮬레이터에 연결되지 않음
    CONNECTED_SIM_ONLY = auto() # VR이 시뮬레이터에 연결됨, 로봇 연결 안됨
    FULLY_CONNECTED = auto()    # 양쪽 연결 모두 활성화
```

### 7.2 버튼 상태 머신 클래스

```python
import time


class ButtonStateMachine:
    """버튼 입력을 처리하고 모드 전환을 관리하는 상태 머신
    
    컨트롤 매핑:
    - 버튼 1 (A/X): VR ↔ 시뮬레이터 연결 토글
    - 양측 트리거 1: 시뮬레이터 ↔ 로봇 연결 토글
    - 양측 트리거 2: 그리퍼 Open/Close
    - 양측 조이스틱: 모바일 베이스 제어
    """
    
    def __init__(self):
        self.state = TeleoperationState.DISCONNECTED
        
        # 엣지 감지를 위한 이전 버튼 상태
        self._prev_button1_left = False
        self._prev_button1_right = False
        self._prev_trigger1_left = False
        self._prev_trigger1_right = False
        self._prev_grip_left = False
        self._prev_grip_right = False
        
        # 그리퍼 상태
        self.left_gripper_open = True
        self.right_gripper_open = True
        
        # 디바운스
        self._debounce_time = 0.2  # 초
        self._last_toggle_time = 0
        
    def _rising_edge(self, current, previous):
        """버튼 누름 감지 (False → True 전환)"""
        return current and not previous
    
    def update(self, vr_data, current_time):
        """버튼 입력을 처리하고 액션을 반환합니다.
        
        Args:
            vr_data: VR 컨트롤러 데이터 딕셔너리
            current_time: 현재 시간 (초)
            
        Returns:
            dict: 액션 딕셔너리
                - toggle_vr_sim: VR-Sim 연결 토글 여부
                - toggle_sim_robot: Sim-Robot 연결 토글 여부
                - toggle_left_gripper: 왼쪽 그리퍼 토글 여부
                - toggle_right_gripper: 오른쪽 그리퍼 토글 여부
                - left_joy: (x, y) 왼쪽 조이스틱
                - right_joy: (x, y) 오른쪽 조이스틱
        """
        left = vr_data.get('left', {}) or {}
        right = vr_data.get('right', {}) or {}
        
        actions = {
            'toggle_vr_sim': False,
            'toggle_sim_robot': False,
            'toggle_left_gripper': False,
            'toggle_right_gripper': False,
            'left_joy': (0, 0),
            'right_joy': (0, 0),
            'state': self.state
        }
        
        # === 버튼 1 (A/X): VR-Sim 연결 토글 ===
        button1_left = left.get('button_a', False)
        button1_right = right.get('button_a', False)
        
        if (self._rising_edge(button1_left, self._prev_button1_left) or 
            self._rising_edge(button1_right, self._prev_button1_right)):
            if current_time - self._last_toggle_time > self._debounce_time:
                actions['toggle_vr_sim'] = True
                self._last_toggle_time = current_time
                self._toggle_vr_sim_state()
                print(f"VR-Sim 연결 토글: {self.state.name}")
        
        # === 양측 트리거 1 (인덱스 트리거): Sim-Robot 연결 토글 ===
        trigger1_left = left.get('trigger', 0) > 0.9
        trigger1_right = right.get('trigger', 0) > 0.9
        
        # 양측 트리거가 동시에 눌렸을 때만
        if (self._rising_edge(trigger1_left, self._prev_trigger1_left) and
            self._rising_edge(trigger1_right, self._prev_trigger1_right)):
            if current_time - self._last_toggle_time > self._debounce_time:
                actions['toggle_sim_robot'] = True
                self._last_toggle_time = current_time
                self._toggle_sim_robot_state()
                print(f"Sim-Robot 연결 토글: {self.state.name}")
        
        # === 트리거 2 (그립 트리거): 그리퍼 토글 ===
        grip_left = left.get('grip_button', False) or left.get('grip', 0) > 0.9
        grip_right = right.get('grip_button', False) or right.get('grip', 0) > 0.9
        
        if self._rising_edge(grip_left, self._prev_grip_left):
            actions['toggle_left_gripper'] = True
            self.left_gripper_open = not self.left_gripper_open
            print(f"왼쪽 그리퍼: {'열림' if self.left_gripper_open else '닫힘'}")
            
        if self._rising_edge(grip_right, self._prev_grip_right):
            actions['toggle_right_gripper'] = True
            self.right_gripper_open = not self.right_gripper_open
            print(f"오른쪽 그리퍼: {'열림' if self.right_gripper_open else '닫힘'}")
        
        # === 조이스틱: 연결된 상태에서만 모바일 베이스 제어 ===
        if self.state != TeleoperationState.DISCONNECTED:
            actions['left_joy'] = (
                left.get('joystick_x', 0),
                left.get('joystick_y', 0)
            )
            actions['right_joy'] = (
                right.get('joystick_x', 0),
                right.get('joystick_y', 0)
            )
        
        # 이전 상태 업데이트
        self._prev_button1_left = button1_left
        self._prev_button1_right = button1_right
        self._prev_trigger1_left = trigger1_left
        self._prev_trigger1_right = trigger1_right
        self._prev_grip_left = grip_left
        self._prev_grip_right = grip_right
        
        return actions
    
    def _toggle_vr_sim_state(self):
        """VR-Sim 연결 상태를 토글합니다."""
        if self.state == TeleoperationState.DISCONNECTED:
            self.state = TeleoperationState.CONNECTED_SIM_ONLY
        elif self.state == TeleoperationState.CONNECTED_SIM_ONLY:
            self.state = TeleoperationState.DISCONNECTED
        elif self.state == TeleoperationState.FULLY_CONNECTED:
            self.state = TeleoperationState.CONNECTED_SIM_ONLY
    
    def _toggle_sim_robot_state(self):
        """Sim-Robot 연결 상태를 토글합니다."""
        if self.state == TeleoperationState.CONNECTED_SIM_ONLY:
            self.state = TeleoperationState.FULLY_CONNECTED
        elif self.state == TeleoperationState.FULLY_CONNECTED:
            self.state = TeleoperationState.CONNECTED_SIM_ONLY
    
    def is_arm_control_enabled(self):
        """암 제어가 활성화되어 있는지 확인합니다."""
        return self.state != TeleoperationState.DISCONNECTED
    
    def is_robot_control_enabled(self):
        """실제 로봇 제어가 활성화되어 있는지 확인합니다."""
        return self.state == TeleoperationState.FULLY_CONNECTED
```

---

## 8. 그리퍼 제어

### 8.1 OpenMANIPULATOR-X 그리퍼 사양

OpenMANIPULATOR-X는 위치 명령으로 제어할 수 있는 프리즈매틱 그리퍼 조인트를 가지고 있습니다:

| 상태 | 위치 |
|------|------|
| 열림 | 0.019m |
| 닫힘 | 0.0m |

### 8.2 그리퍼 컨트롤러 클래스

```python
from isaacsim.core.utils.types import ArticulationAction
import numpy as np


class GripperController:
    """OpenMANIPULATOR-X 그리퍼 컨트롤러
    
    위치 명령으로 프리즈매틱 그리퍼를 제어합니다.
    """
    
    def __init__(self, arm_articulation, gripper_joint_names):
        """
        Args:
            arm_articulation: 암 Articulation
            gripper_joint_names: ["gripper_left_joint", "gripper_right_joint"]
        """
        self.arm = arm_articulation
        self.gripper_joint_names = gripper_joint_names
        
        # OpenMANIPULATOR-X 그리퍼 위치
        self.open_positions = np.array([0.019, -0.019])
        self.closed_positions = np.array([0.0, 0.0])
        
        # 조인트 인덱스 가져오기
        all_joints = self.arm.dof_names
        self.gripper_indices = [
            all_joints.index(name) for name in gripper_joint_names
        ]
        
        self.is_open = True
        
    def open(self):
        """그리퍼를 엽니다."""
        action = ArticulationAction(
            joint_positions=self.open_positions,
            joint_indices=np.array(self.gripper_indices)
        )
        self.arm.apply_action(action)
        self.is_open = True
        
    def close(self):
        """그리퍼를 닫습니다."""
        action = ArticulationAction(
            joint_positions=self.closed_positions,
            joint_indices=np.array(self.gripper_indices)
        )
        self.arm.apply_action(action)
        self.is_open = False
        
    def toggle(self):
        """그리퍼 상태를 토글합니다."""
        if self.is_open:
            self.close()
        else:
            self.open()
    
    def set_position(self, position):
        """그리퍼를 특정 위치로 설정합니다.
        
        Args:
            position: 0.0 (닫힘) ~ 0.019 (열림)
        """
        positions = np.array([position, -position])
        action = ArticulationAction(
            joint_positions=positions,
            joint_indices=np.array(self.gripper_indices)
        )
        self.arm.apply_action(action)
        self.is_open = position > 0.01
```

---

## 9. 전체 시스템 통합

### 9.1 메인 애플리케이션 클래스

```python
from isaacsim import SimulationApp

# Isaac Sim 앱 시작 (headless=False로 GUI 사용)
simulation_app = SimulationApp({"headless": False})

import numpy as np
import time
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage


class VRTeleoperationSystem:
    """VR 텔레오퍼레이션 통합 시스템
    
    VR 입력, 듀얼 암 IK, 모바일 베이스 제어, 버튼 처리를
    통합하여 완전한 텔레오퍼레이션 시스템을 구성합니다.
    """
    
    def __init__(self):
        # World 생성
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        
        # VR 데이터 서브스크라이버 (VR 퍼블리셔 프로세스에 연결)
        self.vr_subscriber = VRDataSubscriber(host="localhost", port=5555)
        
        # 상태 머신
        self.state_machine = ButtonStateMachine()
        
        # 컨트롤러들 (world 설정 후 초기화)
        self.dual_arm_controller = None
        self.mobile_base_controller = None
        self.left_gripper = None
        self.right_gripper = None
        
        # 상태 표시용
        self._last_status_time = 0
        
    def setup_scene(self):
        """로봇을 로드하고 컨트롤러를 구성합니다."""
        
        # === 로봇 USD 또는 URDF 임포트 ===
        # 경로는 실제 설정에 맞게 수정하세요
        add_reference_to_stage(
            usd_path="/path/to/turtlebot_dual_arm.usd",
            prim_path="/World/Robot"
        )
        
        # === Articulation 참조 가져오기 ===
        mobile_base = Articulation(prim_path="/World/Robot/turtlebot_base")
        left_arm = Articulation(prim_path="/World/Robot/left_arm")
        right_arm = Articulation(prim_path="/World/Robot/right_arm")
        
        # === 듀얼 암 IK 컨트롤러 초기화 ===
        self.dual_arm_controller = DualArmIKController(
            left_arm_path="/World/Robot/left_arm",
            right_arm_path="/World/Robot/right_arm",
            robot_descriptor_path="config/open_manipulator_descriptor.yaml",
            urdf_path="urdf/open_manipulator_x.urdf"
        )
        
        # === 모바일 베이스 컨트롤러 초기화 ===
        self.mobile_base_controller = SkidSteerMobileBaseController(
            robot_articulation=mobile_base,
            wheel_joint_names=[
                "front_left_wheel_joint",
                "rear_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_right_wheel_joint"
            ],
            wheel_radius=0.033,
            track_width=0.287
        )
        
        # === 그리퍼 컨트롤러 초기화 ===
        self.left_gripper = GripperController(
            left_arm, 
            ["left_gripper_left_joint", "left_gripper_right_joint"]
        )
        self.right_gripper = GripperController(
            right_arm,
            ["right_gripper_left_joint", "right_gripper_right_joint"]
        )
        
        print("씬 설정 완료")
        
    def physics_step(self, step_size):
        """각 물리 스텝에서 호출됩니다."""
        current_time = time.time()
        
        # 최신 VR 데이터 가져오기
        vr_data = self.vr_subscriber.get_latest_numpy()
        if vr_data is None:
            return
        
        # 버튼 입력 처리
        actions = self.state_machine.update(vr_data, current_time)
        
        # === 그리퍼 토글 처리 ===
        if actions['toggle_left_gripper']:
            self.left_gripper.toggle()
        if actions['toggle_right_gripper']:
            self.right_gripper.toggle()
        
        # === 연결된 상태에서만 암 및 베이스 제어 ===
        if self.state_machine.is_arm_control_enabled():
            # 첫 연결 시 VR 원점 설정
            if self.dual_arm_controller.vr_left_origin is None:
                left_data = vr_data.get('left')
                right_data = vr_data.get('right')
                if left_data is not None and right_data is not None:
                    self.dual_arm_controller.set_vr_origin(
                        left_data['position'],
                        right_data['position']
                    )
                    self.dual_arm_controller.set_connected(True)
            
            # 암 IK 업데이트
            ik_results = self.dual_arm_controller.update(
                vr_data.get('left'),
                vr_data.get('right')
            )
            
            # 조이스틱으로 모바일 베이스 업데이트
            # 탱크 스타일: 각 조이스틱 Y축이 해당 쪽 바퀴 제어
            left_joy = actions['left_joy']
            right_joy = actions['right_joy']
            self.mobile_base_controller.update_from_joysticks_tank(
                left_joy[1],   # 왼쪽 스틱 Y → 왼쪽 바퀴
                right_joy[1]   # 오른쪽 스틱 Y → 오른쪽 바퀴
            )
        else:
            # 연결 해제 시 정지
            if self.dual_arm_controller.is_connected:
                self.dual_arm_controller.set_connected(False)
                self.mobile_base_controller.stop()
        
        # === 상태 표시 (1초마다) ===
        if current_time - self._last_status_time > 1.0:
            self._print_status(vr_data, actions)
            self._last_status_time = current_time
    
    def _print_status(self, vr_data, actions):
        """현재 상태를 출력합니다."""
        state_name = self.state_machine.state.name
        left_pos = vr_data['left']['position'] if vr_data.get('left') else "N/A"
        right_pos = vr_data['right']['position'] if vr_data.get('right') else "N/A"
        
        print(f"\r[{state_name}] "
              f"L: {left_pos} | "
              f"R: {right_pos} | "
              f"그리퍼: L={'O' if self.state_machine.left_gripper_open else 'X'} "
              f"R={'O' if self.state_machine.right_gripper_open else 'X'}", 
              end="")
    
    def run(self):
        """메인 시뮬레이션 루프"""
        print("씬 설정 중...")
        self.setup_scene()
        
        print("World 리셋 중...")
        self.world.reset()
        
        print("컨트롤러 초기화 중...")
        self.dual_arm_controller.initialize()
        
        # 물리 콜백 등록
        self.world.add_physics_callback("teleop_control", self.physics_step)
        
        print("\n=== VR 텔레오퍼레이션 시작 ===")
        print("컨트롤:")
        print("  - A/X 버튼: VR-시뮬레이터 연결 토글")
        print("  - 양측 인덱스 트리거 동시: 시뮬레이터-로봇 연결 토글")
        print("  - 그립 트리거: 그리퍼 토글")
        print("  - 조이스틱: 모바일 베이스 이동")
        print("=====================================\n")
        
        # 시뮬레이션 루프
        while simulation_app.is_running():
            self.world.step(render=True)
        
        # 정리
        self.vr_subscriber.stop()
        simulation_app.close()


# === 메인 실행 ===
if __name__ == "__main__":
    system = VRTeleoperationSystem()
    system.run()
```

### 9.2 VR 퍼블리셔 실행 스크립트

`scripts/vr_publisher.py`:

```python
#!/usr/bin/env python3
"""VR 컨트롤러 데이터 퍼블리셔

SteamVR이 실행된 상태에서 이 스크립트를 실행하세요.
Quest Link가 활성화되어 있어야 합니다.
"""

import sys
import signal

# 위의 VRDataPublisher, VRControllerReader 클래스들 import
# from vr_input import VRDataPublisher

def signal_handler(sig, frame):
    print("\n종료 신호 수신...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 50)
    print("VR 컨트롤러 데이터 퍼블리셔")
    print("=" * 50)
    print("필수 조건:")
    print("  1. SteamVR 실행 중")
    print("  2. Quest Link 활성화됨")
    print("  3. 컨트롤러 전원 켜짐")
    print("=" * 50)
    
    try:
        publisher = VRDataPublisher(port=5555)
        print("\n퍼블리셔 시작됨. Ctrl+C로 종료하세요.\n")
        publisher.run(rate_hz=90)
    except Exception as e:
        print(f"오류 발생: {e}")
        print("\nSteamVR이 실행 중인지 확인하세요.")
        sys.exit(1)

if __name__ == "__main__":
    main()
```

---

## 10. 의존성 및 설치 체크리스트

### 10.1 시스템 요구 사항

| 항목 | 사양 |
|------|------|
| OS | Windows 10/11 또는 Ubuntu 22.04 |
| GPU | NVIDIA RTX 2060+ (권장) |
| RAM | 16GB+ |
| Isaac Sim | 4.5+ 또는 5.1.0 |

### 10.2 소프트웨어 설치

```bash
# === Python 패키지 ===
pip install openvr pyopenxr pyzmq numpy scipy

# === VR 소프트웨어 ===
# 1. Meta Quest Link PC App (Meta 공식 사이트)
# 2. SteamVR (Steam에서 설치)

# === 선택: NVIDIA collab-sim 참조 ===
git clone https://github.com/NVlabs/collab-sim
```

### 10.3 프로젝트 파일 구조

```
vr_teleop_project/
├── config/
│   └── open_manipulator_descriptor.yaml
├── urdf/
│   └── open_manipulator_x.urdf
├── scripts/
│   ├── vr_publisher.py          # VR 입력 캡처 프로세스
│   ├── teleop_main.py           # Isaac Sim 메인 애플리케이션
│   ├── vr_input.py              # VR 입력 클래스들
│   ├── dual_arm_controller.py   # 듀얼 암 IK 컨트롤러
│   ├── mobile_base_controller.py # 모바일 베이스 컨트롤러
│   ├── gripper_controller.py    # 그리퍼 컨트롤러
│   └── state_machine.py         # 버튼 상태 머신
├── models/
│   └── turtlebot_dual_arm.usd   # 로봇 USD 모델
└── README.md
```

### 10.4 실행 순서

```bash
# === 1단계: SteamVR 시작 ===
# Steam에서 SteamVR 실행

# === 2단계: Quest Link 활성화 ===
# Quest 헤드셋에서 Quest Link 연결

# === 3단계: 터미널 1 - VR 퍼블리셔 시작 ===
python scripts/vr_publisher.py

# === 4단계: 터미널 2 - Isaac Sim 텔레오퍼레이션 시작 ===
# Isaac Sim Python 환경 사용
./isaaclab.sh -p scripts/teleop_main.py
# 또는
~/.local/share/ov/pkg/isaac-sim-4.5.0/python.sh scripts/teleop_main.py
```

---

## 11. 결론 및 향후 개선 사항

### 11.1 구현 요약

이 가이드는 VR 컨트롤러 입력부터 듀얼 암 IK 및 모바일 베이스 제어까지의 완전한 경로를 제공합니다. 핵심 기술 결정 사항:

| 구성 요소 | 선택 | 이유 |
|----------|------|------|
| VR 입력 | pyopenvr | 성숙한 API, SteamVR 호환성 |
| 통신 | ZMQ pub-sub | 저지연, 장애 격리 |
| IK 솔버 | Lula | Isaac Sim 네이티브 통합 |
| 모바일 베이스 | 차동 구동 | 스키드 스티어에 적합 |

### 11.2 예상 성능

| 지표 | 목표값 | 참고 |
|------|--------|------|
| 엔드투엔드 지연 | <100 ms | 유선 USB 연결 |
| IK 계산 | <1 ms | Lula CPU 기반 |
| 제어 주파수 | 60-90 Hz | 부드러운 조작 |

### 11.3 향후 개선 사항

1. **작업 공간 충돌 체크**: Isaac Sim의 충돌 API를 사용한 안전 경계
2. **속도 제한**: 작업 공간 경계 근접 시 속도 감소
3. **시각적 피드백**: 시뮬레이션에서 IK 타겟 렌더링
4. **VR 헤드셋 뷰**: `semu.xr.openxr` 확장을 통한 스테레오 렌더링
5. **실제 로봇 연동**: ROS2 브릿지를 통한 물리 로봇 제어

### 11.4 참고 자료

- [BEAVR: Bimanual VR Teleoperation System](https://arxiv.org/html/2508.09606v1)
- [Isaac Sim Lula Kinematics Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_lula_kinematics.html)
- [Isaac Sim Mobile Robot Controllers](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_simulation/mobile_robot_controllers.html)
- [OpenVR Python Bindings (pyopenvr)](https://github.com/cmbruns/pyopenvr)
- [triad_openvr - Enhanced OpenVR Wrapper](https://github.com/TriadSemi/triad_openvr)
- [NVLabs collab-sim](https://github.com/NVlabs/collab-sim)

---

*이 문서는 Isaac Sim 5.1.0, Meta Quest 3S, OpenMANIPULATOR-X, TurtleBot 4륜 구성을 기준으로 작성되었습니다.*
