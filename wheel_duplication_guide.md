# Turtlebot3 Waffle Pi에 Wheel 추가하기

레퍼런스된 USD 파일의 하위 오브젝트(wheel 링크)를 복사하는 방법은 일반적인 복사/붙여넣기가 작동하지 않습니다. 다음 3가지 방법 중 하나를 선택하세요.

## 방법 1: Reference를 Payload로 변환 후 복사 (권장)

### 단계:
1. **Isaac Sim에서 ust_project1.usd 열기**

2. **Stage 패널에서 turtlebot3_waffle_pi 찾기**
   - 경로: `/World/turtlebot3_waffle_pi`
   - 주황색 화살표 아이콘이 표시됩니다 (Reference)

3. **Reference를 Payload로 변환**
   - `turtlebot3_waffle_pi` 프림을 우클릭
   - **"Payloads" → "Add Internal Payload"** 또는 **"Convert to Payload"** 선택
   - 아이콘이 파란색 화살표로 변경됩니다

4. **하위 wheel 링크 복사**
   - Stage 패널에서 확장: `/World/turtlebot3_waffle_pi/base/wheel_left_link`
   - `wheel_left_link`를 선택하고 **Ctrl+C** (복사)
   - `base`를 선택하고 **Ctrl+V** (붙여넣기)
   - 자동으로 `wheel_left_link1`이 생성됩니다

5. **이름과 위치 수정**
   - `wheel_left_link1`을 선택
   - Property 패널에서:
     - **이름 변경**: `wheel_front_left_link`
     - **Transform → Translate** 값 수정 (예: X를 +0.15 증가)

6. **같은 방법으로 오른쪽 wheel도 복사**
   - `wheel_right_link` 복사 → 붙여넣기
   - 이름: `wheel_front_right_link`
   - 위치 조정

### 장점:
- 가장 직관적인 방법
- GUI에서 즉시 확인 가능
- 모든 하위 오브젝트와 속성이 자동으로 복사됨

### 단점:
- Payload는 메모리에 완전히 로드되므로 파일 크기가 커질 수 있음

---

## 방법 2: Python 스크립트 사용 (자동화)

### 준비:
제공된 `add_wheels_to_robot.py` 스크립트를 사용합니다.

### 실행 방법:

#### Option A: Isaac Sim Script Editor에서 실행
1. Isaac Sim에서 `ust_project1.usd` 열기
2. **Window → Script Editor** 열기
3. `add_wheels_to_robot.py` 파일 내용을 복사해서 붙여넣기
4. 스크립트 상단의 경로 확인 및 수정:
   ```python
   robot_base_path = "/World/turtlebot3_waffle_pi/base"
   ```
5. **Run** 버튼 클릭

#### Option B: Standalone Python 스크립트 실행
```bash
cd /home/isaac/ust_ws
# Isaac Sim Python 환경 사용
~/.local/share/ov/pkg/isaac_sim-*/python.sh add_wheels_to_robot.py
```

### 스크립트 커스터마이징:
스크립트에서 다음 값들을 수정할 수 있습니다:

```python
# 새로운 wheel 위치 (x, y, z)
new_left_pos = (0.15, 0.144, 0.033)   # 앞쪽 왼쪽
new_right_pos = (0.15, -0.144, 0.033) # 앞쪽 오른쪽

# 또는 뒤쪽에 추가하려면:
new_left_pos = (-0.15, 0.144, 0.033)   # 뒤쪽 왼쪽
new_right_pos = (-0.15, -0.144, 0.033) # 뒤쪽 오른쪽

# Wheel 이름
new_wheel_name = "wheel_front_left_link"  # 원하는 이름으로 변경
```

### 장점:
- 자동화 가능
- 정확한 위치 지정
- 여러 wheel을 한 번에 추가 가능

### 단점:
- Python 코드 이해 필요
- Isaac Sim Python 환경 필요

---

## 방법 3: Override를 사용한 인스턴스 추가

Reference를 유지하면서 새로운 wheel 인스턴스를 추가하는 방법입니다.

### 단계:
1. **Stage 패널에서 base 프림 선택**
   - `/World/turtlebot3_waffle_pi/base`

2. **Create → Xform 추가**
   - `base` 를 우클릭 → **Create → Xform**
   - 이름을 `wheel_front_left_link`로 변경

3. **Mesh 레퍼런스 추가**
   - 새로 만든 `wheel_front_left_link` 선택
   - Property 패널에서 **Add → References**
   - **Asset Path**: `../robotis_mujoco_menagerie/robotis_tb3/assets/left_tire/left_tire.usd`
   - **Prim Path**: `/left_tire` (또는 해당 USD의 루트 프림)

4. **Transform 설정**
   - Property 패널에서 **Transform** 섹션
   - **Translate**: `(0.15, 0.144, 0.033)` (예시)
   - **Rotate**: 기존 wheel과 동일하게 설정
   - **Scale**: `(1, 1, 1)` 또는 기존 wheel과 동일

5. **같은 방법으로 오른쪽 wheel 추가**

### 장점:
- Reference를 유지하여 원본 파일 보호
- 가벼운 방법

### 단점:
- 수동으로 속성을 복사해야 함
- 물리 속성(inertia, joint 등)을 별도로 설정해야 함

---

## 주의사항

### 1. 위치 좌표 시스템
Turtlebot3 Waffle Pi의 기본 wheel 위치 (MuJoCo XML 기준):
- **wheel_left_link**: `(0, 0.144, 0.033)`
- **wheel_right_link**: `(0, -0.144, 0.033)`

좌표계:
- **X축**: 전방(+) / 후방(-)
- **Y축**: 좌측(+) / 우측(-)
- **Z축**: 상방(+) / 하방(-)

### 2. 물리 시뮬레이션
Wheel을 복사한 후 물리 시뮬레이션을 위해 추가로 설정해야 할 사항:

#### a. Joint 추가/수정
- 각 wheel에는 revolute joint가 필요합니다
- **Physics → Add Joint → Revolute Joint**
- Axis: `(0, 0, 1)` (Z축 회전)

#### b. PhysX 속성
- **Mass**: `0.0285 kg` (기존 wheel과 동일)
- **Inertia**: 대각행렬 `[2.07e-05, 1.12e-05, 1.12e-05]`

#### c. Collision Shape
- 각 wheel에 collision geometry 추가 필요
- Cylinder 또는 Mesh Collider

### 3. 파일 저장
- 변경사항을 저장하기 전에 백업 생성 권장:
  ```bash
  cp ./isaac_file/ust_project1.usd ./isaac_file/ust_project1_backup.usd
  ```
- **File → Save** 또는 **Ctrl+S**

---

## 검증

### 추가된 wheel 확인:
1. **Stage 패널 확인**
   - `/World/turtlebot3_waffle_pi/base/` 아래에 새로운 wheel 링크가 있는지 확인

2. **Viewport에서 확인**
   - 3D 뷰에서 wheel이 올바른 위치에 표시되는지 확인
   - 다른 part와 충돌하지 않는지 확인

3. **물리 시뮬레이션 테스트**
   - Play 버튼을 눌러 시뮬레이션 시작
   - Wheel이 바닥과 충돌하는지 확인
   - Joint가 제대로 작동하는지 확인

---

## 문제 해결

### 문제 1: "Cannot copy prims from referenced layer"
**해결책**: 방법 1을 사용하여 Reference를 Payload로 변환하세요.

### 문제 2: 복사한 wheel이 보이지 않음
**원인**: Visibility 또는 Transform 문제
**해결책**:
- Property 패널에서 **Visibility**가 "inherited" 또는 "visible"인지 확인
- **Transform → Translate** 값이 올바른지 확인

### 문제 3: Wheel이 바닥을 통과함
**원인**: Collision shape 미설정
**해결책**:
- Wheel 프림에 **Physics → Collision Shape** 추가
- Collision API 활성화

### 문제 4: 스크립트 실행 오류
**오류**: `ModuleNotFoundError: No module named 'omni'`
**해결책**:
- Isaac Sim 내부에서 스크립트 실행 (Script Editor 사용)
- 또는 Isaac Sim Python 환경 사용:
  ```bash
  ~/.local/share/ov/pkg/isaac_sim-*/python.sh your_script.py
  ```

---

## 추가 리소스

### Isaac Sim 문서:
- [Working with USD](https://docs.isaacsim.omniverse.nvidia.com/latest/omniverse_usd/intro_to_usd.html)
- [Python Scripting](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/core_api_overview.html)

### USD 문서:
- [USD API Reference](https://openusd.org/release/api/index.html)
- [USD Glossary](https://openusd.org/release/glossary.html)

---

## 요약

가장 간단한 방법은 **방법 1 (Payload 변환 후 복사)**입니다:
1. Reference를 Payload로 변환
2. Wheel을 복사/붙여넣기
3. 이름과 위치 수정
4. 저장

프로그래밍에 익숙하다면 **방법 2 (Python 스크립트)**가 더 정확하고 재사용 가능합니다.
