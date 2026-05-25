# Open Manipulator X - Isaac Sim URDF 임포트 가이드

## ✅ URDF 파일 준비 완료!

XACRO 파일을 URDF로 변환 완료했습니다:
- **위치**: `/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf`
- **메쉬 파일**: `/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/meshes/open_manipulator_x/`

## Isaac Sim 임포트 단계별 가이드 (30분)

### 1단계: Isaac Sim 준비 (2분)

1. **Isaac Sim 실행**
2. **새 Stage 생성**:
   - `File` > `New` (또는 Ctrl+N)
   - 빈 씬에서 시작

### 2단계: URDF Importer 설정 (3분)

1. **URDF Importer 열기**:
   - 상단 메뉴: `Isaac Utils` > `URDF Importer`
   - 또는 `Window` > `Isaac Utils` > `URDF Importer`

2. **임포트 설정 확인**:
   - URDF Importer 창이 열립니다

### 3단계: URDF 파일 선택 및 설정 (5분)

#### URDF 파일 경로 입력:
```
Input File: /home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf
```

#### 중요 설정:

**Import Config 섹션:**
- ✅ **Fix Base Link**: 체크 (로봇을 고정된 베이스로 설정)
- ✅ **Self Collision**: 체크 (자기 충돌 감지)
- ✅ **Import Inertia Tensor**: 체크 (관성 텐서 임포트)
- ✅ **Create Physics Scene**: 체크
- ✅ **Make Default Prim**: 체크

**Advanced Settings (필요시):**
- **Distance Scale**: `1.0` (미터 단위 유지)
- **Drive Type**: `velocity` 또는 `position` (기본값 사용)
- **Drive Strength**: `1e7` (기본값)

**Mesh 경로 설정:**
- **Package Paths**:
  ```
  open_manipulator_description: /home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description
  ```

### 4단계: 임포트 실행 (5분)

1. **Import 버튼 클릭**
2. **진행 상황 확인**:
   - Console 창에서 임포트 로그 확인
   - 경고나 에러 메시지 확인

3. **성공 확인**:
   ```
   Import successful!
   ```

### 5단계: 로봇 확인 (5분)

#### Stage 창에서 계층 구조 확인:
```
/World
  └── open_manipulator_x
      └── link1 (ArticulationRoot)
          └── link2
              └── link3
                  └── link4
                      └── link5
                          ├── gripper_left_link
                          ├── gripper_right_link
                          └── end_effector_link
```

#### Viewport에서 시각적 확인:
- 로봇이 올바르게 로드되었는지 확인
- 메쉬가 제대로 표시되는지 확인

#### Physics 확인:
1. **Play 버튼** 클릭 (시뮬레이션 시작)
2. 로봇이 떨어지지 않고 고정되어 있는지 확인
3. **Stop 버튼** 클릭

### 6단계: Articulation 확인 (3분)

1. **Stage에서 `link1` 선택**
2. **Property 패널 확인**:
   - `Physics` > `ArticulationRootAPI`가 적용되어 있어야 함
   - `Rigid Body`도 설정되어 있어야 함

3. **Joint 확인**:
   - 각 joint (joint1, joint2, joint3, joint4)가 올바르게 설정되었는지 확인
   - Joint limits가 URDF 값과 일치하는지 확인

### 7단계: 저장 (2분)

**USD 파일로 저장**:
```
File > Save As...
이름: open_manipulator_x_isaac.usd
위치: /home/isaac/ust_ws/isaac_file/
```

---

## Lula Robot Description Editor 설정 (10분)

### 1. Lula 에디터 열기

1. **Extensions 확인**:
   - `Window` > `Extensions`
   - "Lula" 검색
   - `Lula Robot Description Editor` 활성화

2. **에디터 열기**:
   - `Window` > `Lula Robot Description Editor`

### 2. Robot Prim 선택

1. **Robot Prim 필드**에서 `Select` 버튼 클릭
2. **Stage에서 `link1` 선택**
   - 경로: `/World/open_manipulator_x/link1`
3. 자동으로 모든 관절이 감지됨

### 3. Joints 설정

**Joints 탭**에서:
- ✅ joint1 (revolute, Z축)
- ✅ joint2 (revolute, Y축)
- ✅ joint3 (revolute, Y축)
- ✅ joint4 (revolute, Y축)

**Active Joints로 추가**:
- 위의 4개 joint를 모두 Active Joints에 추가
- gripper_left_joint, gripper_right_joint는 선택사항

### 4. Collision Spheres 추가

**Spheres 탭**에서 각 링크에 충돌 구 추가:

1. **link1**:
   - Add Sphere
   - Radius: 0.05
   - Position: (0, 0, 0)

2. **link2**:
   - Add Sphere
   - Radius: 0.04
   - Position: (0, 0, 0.03)

3. **link3**:
   - Add 2-3 Spheres (팔 길이를 커버)
   - Radius: 0.035

4. **link4**:
   - Add 2-3 Spheres
   - Radius: 0.03

5. **link5 + 그리퍼**:
   - Add 2 Spheres
   - Radius: 0.025

### 5. YAML 내보내기

1. **Export 탭**으로 이동
2. **Export Robot Description** 클릭
3. **저장 위치**:
   ```
   /home/isaac/ust_ws/isaac_file/open_manipulator_x_robot_description.yaml
   ```

---

## 예상 결과

### 성공 시 얻게 되는 것:

✅ **올바른 계층 구조**:
- link1이 Articulation Root
- 모든 링크가 올바른 부모-자식 관계

✅ **완전한 Joint 정보**:
- 4개의 revolute joints
- 올바른 joint limits
- 정확한 joint axes

✅ **물리 속성**:
- 정확한 Mass와 Inertia
- Collision 설정
- Articulation 설정

✅ **Lula 호환**:
- RMPflow에서 즉시 사용 가능
- 모든 관절 인식됨
- 충돌 회피 작동

---

## 문제 해결

### 메쉬가 표시되지 않음
**원인**: Package path가 잘못 설정됨

**해결**:
1. URDF Importer에서 `Package Paths` 재확인
2. 경로가 정확한지 확인:
   ```
   open_manipulator_description: /home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description
   ```

### 로봇이 떨어짐
**원인**: Fix Base Link가 체크되지 않음

**해결**:
1. 재임포트 시 `Fix Base Link` 체크
2. 또는 `world_fixed` joint를 수동으로 `fixed` 타입으로 설정

### Joint가 움직이지 않음
**원인**: Articulation이 올바르게 설정되지 않음

**해결**:
1. `link1`에 ArticulationRootAPI가 있는지 확인
2. 각 joint에 `ArticulationJointAPI`가 있는지 확인
3. Drive type과 strength 확인

### Lula에서 관절이 보이지 않음
**원인**: 잘못된 Robot Prim 선택

**해결**:
1. Robot Prim으로 반드시 `link1` 선택
2. `world` 또는 `open_manipulator_x` 선택하지 말 것

---

## 다음 단계

### RMPflow 설정

1. **RMPflow Controller 추가**:
   - `Isaac Utils` > `Motion Generation` > `RMPflow`

2. **Robot Description 로드**:
   - 방금 export한 YAML 파일 사용

3. **Target 설정**:
   - End effector target 설정
   - 이동 목표 지점 지정

4. **테스트**:
   - Play 버튼으로 시뮬레이션 시작
   - Target 이동 시 로봇이 따라가는지 확인

---

## 시간 비교

| 작업 | URDF 임포트 | 기존 USD 수정 |
|------|-------------|---------------|
| **총 소요 시간** | ✅ **30-40분** | ❌ 불가능/수일 |
| **성공 확률** | ✅ **~100%** | ❌ <10% |
| **Lula 호환성** | ✅ **완벽** | ❌ 불확실 |
| **유지보수** | ✅ **쉬움** | ❌ 어려움 |

**결론**: URDF 임포트가 압도적으로 우수합니다!
