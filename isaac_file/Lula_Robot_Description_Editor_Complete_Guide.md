# Isaac Sim Lula Robot Description Editor 완벽 가이드

이 문서는 Isaac Sim의 Lula Robot Description Editor의 모든 설정 옵션과 사용 방법을 상세히 설명합니다.

---

## 목차

1. [개요](#1-개요)
2. [사전 준비](#2-사전-준비)
3. [에디터 접근 방법](#3-에디터-접근-방법)
4. [Selection Panel (선택 패널)](#4-selection-panel-선택-패널)
5. [Set Joint Properties (조인트 속성 설정)](#5-set-joint-properties-조인트-속성-설정)
6. [Link Sphere Editor (링크 구체 편집기)](#6-link-sphere-editor-링크-구체-편집기)
7. [Editor Tools (편집 도구)](#7-editor-tools-편집-도구)
8. [Import/Export (가져오기/내보내기)](#8-importexport-가져오기내보내기)
9. [단계별 사용 가이드](#9-단계별-사용-가이드)
10. [문제 해결](#10-문제-해결)
11. [참고 자료](#11-참고-자료)

---

## 1. 개요

### 1.1 Lula Robot Description Editor란?

Lula Robot Description Editor는 Isaac Sim에서 제공하는 UI 도구로, 로봇의 운동학, 역학 및 충돌 정보를 포함한 설정 파일을 생성합니다. 이 설정 파일은 다음 알고리즘에서 사용됩니다:

| 알고리즘 | 용도 | 필요 파일 |
|---------|------|----------|
| **RMPflow** | 실시간 모션 플래닝 | Lula Robot Description (.yaml) |
| **Lula Kinematics Solver** | 순/역운동학 계산 | Lula Robot Description (.yaml) |
| **Lula RRT** | 경로 계획 | Lula Robot Description (.yaml) |
| **cuMotion** | GPU 가속 모션 플래닝 | XRDF (.xrdf) |

### 1.2 왜 별도의 설정 파일이 필요한가?

URDF 파일만으로는 다음 정보가 부족합니다:
- 충돌 회피를 위한 구체(Sphere) 표현
- Active/Fixed 조인트 구분
- 가속도/저크 제한값
- 기본 로봇 자세(Default Configuration)

---

## 2. 사전 준비

### 2.1 Extension 활성화

Lula Robot Description Editor를 사용하기 전에 필요한 Extension을 활성화해야 합니다:

1. **Window > Extensions** 메뉴 열기
2. 검색창에 "Lula" 입력
3. **Isaac Sim Lula Extension** 찾아서 활성화
4. **AUTOLOAD** 체크박스 선택 (다음 실행 시 자동 로드)

### 2.2 로봇 자산 준비

**중요**: Lula Robot Description Editor는 **Instanceable Assets을 지원하지 않습니다**.

Instanceable 설정 해제 방법:
1. Stage 창에서 로봇의 `visuals`와 `collisions` 프리미티브 선택
2. Property 패널에서 **Instantiable** 필드 체크 해제

### 2.3 Articulation Root 설정

**매우 중요**: Articulation Root는 로봇의 **베이스 링크에 단 하나만** 설정해야 합니다.

잘못된 설정 예:
- ❌ 모든 링크에 Articulation Root API 적용
- ❌ 중간 링크에만 Articulation Root API 적용

올바른 설정:
- ✅ 베이스 링크(예: `base_link`, `link1`)에만 Articulation Root API 적용

Articulation Root 제거 방법:
1. 해당 링크 우클릭
2. **Remove API > Physics > ArticulationRootAPI** 선택

---

## 3. 에디터 접근 방법

### 3.1 메뉴에서 열기

```
Tools > Robotics > Lula Robot Description Editor
```

또는

```
Window > Lula Robot Description Editor
```

### 3.2 필수 조건

- 스테이지에 로봇(Articulation)이 로드되어 있어야 함
- **시뮬레이션이 재생(Play) 상태여야 함** ⚠️

---

## 4. Selection Panel (선택 패널)

Selection Panel은 에디터 상단에 위치하며, 작업할 로봇과 링크를 선택합니다.

### 4.1 Select Articulation

| 항목 | 설명 |
|------|------|
| **드롭다운 메뉴** | 스테이지에 있는 모든 Articulation 목록 표시 |
| **활성화 조건** | 시뮬레이션 재생 중일 때만 목록 표시 |
| **선택 대상** | 로봇의 루트 프림 경로 (예: `/World/ur10e`) |

### 4.2 Select Link

| 항목 | 설명 |
|------|------|
| **드롭다운 메뉴** | 선택된 로봇의 모든 링크 목록 |
| **용도** | 충돌 구체 편집 시 대상 링크 지정 |
| **업데이트** | Articulation 선택 시 자동으로 목록 갱신 |

---

## 5. Set Joint Properties (조인트 속성 설정)

> **참고**: Isaac Sim 4.0.0부터 기존 "Command Panel"이 "Set Joint Properties"로 이름 변경되었습니다.

### 5.1 개요

이 패널에서는 로봇의 각 조인트에 대한 속성을 설정합니다. 모든 Lula 알고리즘에서 **Active/Fixed 조인트 구분은 필수**입니다.

### 5.2 조인트 목록

| 컬럼 | 설명 |
|------|------|
| **Joint Name** | 조인트 이름 (URDF에서 정의된 이름) |
| **Joint Position** | 현재/기본 조인트 위치 (라디안 또는 미터) |
| **Joint Status** | Active 또는 Fixed 상태 |

### 5.3 Joint Status 옵션

#### Active Joint
- Lula 알고리즘이 **직접 제어**하는 조인트
- RMPflow, Lula RRT 등에서 모션 계획에 포함
- 일반적으로 매니퓰레이터의 모든 팔 조인트

**Active Joint로 설정해야 하는 경우:**
- 역운동학 계산에 포함되어야 하는 조인트
- 경로 계획에서 움직여야 하는 조인트
- End-effector 위치/방향에 영향을 주는 조인트

#### Fixed Joint
- Lula 알고리즘이 **제어하지 않는** 조인트
- 설정된 위치에 고정된 것으로 간주
- 그리퍼 조인트 등 별도 제어기로 관리되는 조인트

**Fixed Joint로 설정해야 하는 경우:**
- 그리퍼/핑거 조인트
- 별도의 컨트롤러로 제어되는 조인트
- 운동학 체인에서 제외할 조인트

### 5.4 Joint Position (기본 설정)

| 항목 | 설명 |
|------|------|
| **입력 방식** | 슬라이더 또는 직접 입력 |
| **단위** | Revolute: 라디안, Prismatic: 미터 |
| **권장 자세** | 로봇 앞쪽(+X축), 조인트 한계에서 떨어진 위치 |

**기본 설정 선택 가이드:**

```
✅ 좋은 기본 설정:
- 팔이 앞으로 뻗은 자세
- 모든 조인트가 중간 위치
- 특이점(Singularity)에서 벗어난 자세

❌ 나쁜 기본 설정:
- 조인트 한계(limit)에 가까운 위치
- 특이점 근처 (팔이 완전히 펴지거나 접힌 상태)
- 자기 충돌이 발생할 수 있는 자세
```

### 5.5 동역학 파라미터 (Isaac Sim 4.0.0+)

Isaac Sim 4.0.0부터 각 조인트에 대해 추가 파라미터를 설정할 수 있습니다:

| 파라미터 | 설명 | 단위 |
|---------|------|------|
| **Acceleration Limit** | 최대 가속도 제한 | rad/s² 또는 m/s² |
| **Jerk Limit** | 최대 저크(가속도 변화율) 제한 | rad/s³ 또는 m/s³ |

---

## 6. Link Sphere Editor (링크 구체 편집기)

### 6.1 개요

충돌 구체(Collision Spheres)는 로봇의 충돌 감지를 위한 간소화된 기하학적 표현입니다.
RMPflow와 cuMotion에서 장애물 회피에 사용됩니다.

**충돌 구체가 필요한 알고리즘:**
- RMPflow (장애물 회피용)
- cuMotion (GPU 기반 충돌 감지)
- Lula RRT (경로 계획 시 충돌 체크)

**충돌 구체가 필요 없는 알고리즘:**
- Lula Kinematics Solver (순수 운동학 계산)

### 6.2 구체 추가 방법

#### 방법 1: Add Sphere (수동 추가)

| 옵션 | 설명 |
|------|------|
| **Add Sphere 버튼** | 선택된 링크에 단일 구체 추가 |
| **위치 조정** | 구체를 드래그하여 위치 변경 |
| **반경 조정** | 구체 핸들을 드래그하여 크기 변경 |

**사용 시기:**
- 간단한 형태의 링크
- 세밀한 수동 조정이 필요한 경우

#### 방법 2: Connect Spheres (구체 연결)

| 옵션 | 설명 |
|------|------|
| **첫 번째 구체** | 시작점 구체 위치/크기 지정 |
| **두 번째 구체** | 끝점 구체 위치/크기 지정 |
| **보간 개수** | 두 구체 사이에 생성할 구체 수 |

**사용 시기:**
- 원통형 링크 (로봇 팔의 일반적인 형태)
- 규칙적인 형태의 링크

#### 방법 3: Generate Spheres (자동 생성)

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| **Select Mesh** | 구체 생성 기준이 될 메시 선택 | - |
| **Number of Spheres** | 생성할 구체 개수 | 8 |
| **Radius Offset** | 메시 표면에서의 오프셋 | 0.03m |
| **Generate Spheres** | 미리보기 후 구체 생성 확정 | - |

**사용 시기:**
- 복잡한 형태의 링크
- 빠른 초기 설정이 필요한 경우

**주의사항:**
- 폐쇄된(Watertight) 메시만 지원
- Instantiable 메시는 작동하지 않음

### 6.3 구체 편집

| 기능 | 설명 |
|------|------|
| **선택** | 구체 클릭하여 선택 |
| **이동** | 선택된 구체 드래그 |
| **크기 조절** | 핸들을 드래그하여 반경 변경 |
| **삭제** | 선택 후 Delete 키 또는 Clear 버튼 |

### 6.4 충돌 구체 튜닝 가이드

```
좋은 충돌 구체 설정:
├── 링크 형태를 충분히 감싸야 함
├── 실제 링크보다 약간 큰 정도가 적당
├── 너무 크면 움직임이 제한됨
└── 너무 작으면 충돌 감지 실패

구체 개수 가이드:
├── 적은 구체 (3-5개): 빠른 계산, 부정확한 형태
├── 중간 구체 (6-10개): 균형잡힌 설정
└── 많은 구체 (10개+): 정확한 형태, 느린 계산
```

### 6.5 Scale Spheres

선택된 링크의 모든 구체를 한 번에 크기 조절:

| 옵션 | 설명 |
|------|------|
| **Scale Factor** | 배율 (1.0 = 원본 크기) |
| **Apply** | 스케일 적용 |

### 6.6 Clear Spheres

| 옵션 | 설명 |
|------|------|
| **Clear Link Spheres** | 현재 선택된 링크의 구체만 삭제 |
| **Clear All Spheres** | 로봇 전체의 모든 구체 삭제 |

---

## 7. Editor Tools (편집 도구)

### 7.1 Undo/Redo

| 버튼 | 단축키 | 설명 |
|------|--------|------|
| **Undo** | Ctrl+Z | 이전 작업 취소 |
| **Redo** | Ctrl+Y | 취소한 작업 다시 실행 |

### 7.2 Sphere Color (구체 색상)

충돌 구체의 시각적 색상을 변경합니다. 기능에는 영향 없음.

| 옵션 | 설명 |
|------|------|
| **Color Picker** | 구체 표시 색상 선택 |
| **Transparency** | 구체 투명도 조절 |

### 7.3 Robot Visibility (로봇 가시성)

| 옵션 | 설명 |
|------|------|
| **Toggle Visibility** | 로봇 메시 표시/숨김 전환 |
| **용도** | 구체만 보면서 작업할 때 유용 |

### 7.4 Show Sphere Labels

| 옵션 | 설명 |
|------|------|
| **Toggle Labels** | 구체 인덱스 라벨 표시/숨김 |
| **용도** | 특정 구체 식별에 유용 |

---

## 8. Import/Export (가져오기/내보내기)

### 8.1 Export to Lula Robot Description File

Lula 알고리즘용 YAML 형식 파일 생성

| 항목 | 설명 |
|------|------|
| **파일 형식** | `.yaml` |
| **사용 대상** | RMPflow, Lula RRT, Lula Kinematics |
| **경로 선택** | 로컬 파일 시스템 경로 지정 |

**내보내기 절차:**
1. `Export To File` 섹션 펼치기
2. `Export to Lula Robot Description File` 버튼 클릭
3. 파일 경로와 이름 입력 (예: `my_robot.yaml`)
4. `Save` 버튼 클릭

### 8.2 Export to cuMotion XRDF

cuMotion용 XRDF 형식 파일 생성

| 항목 | 설명 |
|------|------|
| **파일 형식** | `.xrdf` 또는 `.yaml` |
| **사용 대상** | cuMotion |
| **특징** | 자동으로 단일 충돌 그룹 생성 |

**추가 옵션:**

| 옵션 | 설명 |
|------|------|
| **Merge with existing XRDF** | 기존 XRDF 파일과 병합 |
| **Overwrite** | 기존 파일 덮어쓰기 |

### 8.3 Import Lula Robot Description File

기존 YAML 파일에서 설정 가져오기

| 항목 | 설명 |
|------|------|
| **동작** | 현재 설정을 완전히 덮어씀 |
| **주의** | 기존 작업 내용이 사라짐 |

**가져오기 절차:**
1. `Import From File` 섹션 펼치기
2. `Import Lula Robot Description File` 버튼 클릭
3. 파일 선택
4. `Open` 버튼 클릭

### 8.4 Import XRDF File

기존 XRDF 파일에서 충돌 구체 가져오기

| 항목 | 설명 |
|------|------|
| **가져오는 데이터** | 충돌 구체 정보만 |
| **무시되는 데이터** | Tool Frames, Modifiers 등 |

---

## 9. 단계별 사용 가이드

### 9.1 전체 워크플로우

```
1. 로봇 준비
   └── USD 파일 로드 (드래그 & 드롭)
   └── Instanceable 설정 해제
   └── Articulation Root 확인

2. 에디터 실행
   └── Play 버튼으로 시뮬레이션 시작
   └── Tools > Robotics > Lula Robot Description Editor

3. 로봇 선택
   └── Select Articulation에서 로봇 선택

4. 조인트 설정
   └── 각 조인트의 Active/Fixed 설정
   └── 기본 자세 설정
   └── 동역학 파라미터 입력 (선택사항)

5. 충돌 구체 생성
   └── 각 링크 선택
   └── 구체 추가 (자동 또는 수동)
   └── 위치/크기 조정

6. 파일 내보내기
   └── Export to Lula Robot Description File
   └── (선택) Export to cuMotion XRDF

7. 완료
   └── 시뮬레이션 중지
```

### 9.2 UR10e 로봇 예제

#### Step 1: 조인트 설정

```yaml
Active Joints:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint

Fixed Joints:
  - robotiq_85_left_knuckle_joint
  - robotiq_85_right_knuckle_joint
  - (기타 그리퍼 조인트)
```

#### Step 2: 충돌 구체 생성

각 링크에 대해:

| 링크 | 메시 선택 | 구체 개수 | Radius Offset |
|------|----------|----------|---------------|
| base_link | /collisions/base/mesh | 5 | 0.03 |
| shoulder_link | /collisions/shoulder/mesh | 6 | 0.03 |
| upper_arm_link | /collisions/upperarm/mesh | 8 | 0.03 |
| forearm_link | /collisions/forearm/mesh | 8 | 0.03 |
| wrist_1_link | /collisions/wrist1/mesh | 4 | 0.03 |
| wrist_2_link | /collisions/wrist2/mesh | 4 | 0.03 |
| wrist_3_link | /collisions/wrist3/mesh | 4 | 0.03 |

#### Step 3: 파일 내보내기

```
파일명: ur10e_robot_description.yaml
경로: /home/user/robot_configs/
```

---

## 10. 문제 해결

### 10.1 조인트가 목록에 표시되지 않음

**증상:** Select Link 드롭다운에서 특정 링크/조인트가 보이지 않음

**해결책:**
1. Articulation Root가 베이스 링크에만 있는지 확인
2. 시뮬레이션이 Play 상태인지 확인
3. 로봇의 URDF/USD 계층 구조 확인
4. 로봇을 다시 선택 (Select 버튼)

### 10.2 Generate Spheres가 작동하지 않음

**증상:** 구체가 생성되지 않거나 이상한 위치에 생성됨

**해결책:**
1. 메시가 폐쇄된(Watertight) 형태인지 확인
2. Instantiable 설정이 해제되었는지 확인
3. 다른 메시(visual 대신 collision 메시 등) 선택 시도

### 10.3 Export 버튼이 비활성화됨

**증상:** Export 버튼을 클릭할 수 없음

**해결책:**
1. 최소 하나의 Active Joint가 설정되었는지 확인
2. 파일 경로가 올바르게 입력되었는지 확인
3. 파일명이 `.yaml` 확장자를 포함하는지 확인

### 10.4 RMPflow에서 장애물 회피가 작동하지 않음

**증상:** 로봇이 장애물을 무시하고 충돌함

**해결책:**
1. 충돌 구체가 모든 링크에 생성되었는지 확인
2. 구체 크기가 너무 작지 않은지 확인
3. Robot Description 파일을 다시 내보내기

### 10.5 조인트 한계에서 에러 발생

**증상:** 특정 자세에서 IK가 실패하거나 이상한 동작

**해결책:**
1. 기본 설정(Default Configuration)이 조인트 한계에서 떨어져 있는지 확인
2. 특이점(Singularity) 근처가 아닌지 확인
3. Active Joint 설정이 올바른지 확인

---

## 11. 참고 자료

### 11.1 공식 문서

- [Lula Robot Description and XRDF Editor](https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_robot_description_editor.html)
- [Tutorial: Generate Robot Configuration File](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/tutorial_generate_robot_config.html)
- [Lula Kinematics Solver](https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_lula_kinematics.html)

### 11.2 관련 알고리즘 문서

- RMPflow Motion Generation
- cuMotion for Isaac Sim
- Lula RRT Planner

### 11.3 포럼 및 커뮤니티

- [NVIDIA Developer Forums - Isaac Sim](https://forums.developer.nvidia.com/c/omniverse/simulation/isaac-sim/)

---

## 부록: Robot Description YAML 파일 구조

생성된 YAML 파일의 기본 구조:

```yaml
# Robot Description File for Lula

# URDF 파일 경로 (선택사항)
urdf_path: "path/to/robot.urdf"

# Cspace (Configuration Space) 설정
cspace:
  - name: joint1
    type: revolute
    default_position: 0.0
    acceleration_limit: 10.0
    jerk_limit: 100.0
  - name: joint2
    type: revolute
    default_position: 0.0
    ...

# 충돌 구체 정의
collision_spheres:
  - link: base_link
    spheres:
      - center: [0.0, 0.0, 0.05]
        radius: 0.05
      - center: [0.0, 0.0, 0.1]
        radius: 0.04
  - link: link1
    spheres:
      ...

# End-effector 설정 (선택사항)
end_effector:
  link: tool0
  position: [0.0, 0.0, 0.0]
  orientation: [1.0, 0.0, 0.0, 0.0]  # quaternion (w, x, y, z)
```

---

*이 문서는 Isaac Sim 4.x 버전을 기준으로 작성되었습니다.*
*최신 정보는 NVIDIA 공식 문서를 참조하세요.*
