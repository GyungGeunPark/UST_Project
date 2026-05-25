
# Isaac Sim 로봇 계층 구조 문제 해결 및 Lula 설정 고급 안내서

이 문서는 `open_manipulator_x1` 로봇과 같이 베이스 링크(`link1`)가 나머지 로봇(`link2` 이하)과 분리되어 있는 복잡한 계층 구조 문제를 해결하고, Lula Robot Description Editor에서 올바른 관절 순서(`link1`부터 `link5`까지)로 설정하는 방법을 심층적으로 안내합니다.

## 1. 문제 분석: 왜 Lula가 로봇을 올바르게 인식하지 못하는가?

현재 로봇의 구성은 다음과 같습니다.

*   `worldBody`
    *   `link1` (베이스 링크, 모터 포함)
    *   `link2` (하위 링크 `link3,4,5` 및 그리퍼 포함)

이 구조의 핵심적인 문제는 **로봇이 사실상 두 개의 분리된 개체로 존재한다**는 것입니다. Lula와 Isaac Sim의 물리 엔진은 로봇을 단일 계층, 즉 **하나의 Articulation(관절체)**으로 인식해야 합니다. `link1`과 `link2`가 동일한 `worldBody` 아래에 형제 관계로 있으면, Isaac Sim은 이들을 별개의 로봇으로 취급하므로 `link1`에서 `link2`로 이어지는 관절을 찾을 수 없습니다.

결과적으로 Lula Robot Description Editor는 `link1`을 선택하면 그 하위 관절이 없는 것으로 판단하고, `link2`를 선택하면 `link1`과의 연결을 인식하지 못하여 전체 관절 체인을 구성할 수 없게 됩니다.

## 2. 해결 방법: 로봇 계층 구조 재구성

이 문제를 해결하기 위해 `Stage`에서 직접 로봇의 계층 구조를 수정하여 `link1`을 명확한 베이스 링크로 만들어야 합니다.

### 2.1. 계층 구조 수정 (Reparenting)

1.  **시뮬레이션 중지:** 먼저 Isaac Sim 시뮬레이션을 `Stop` 상태로 전환합니다.
2.  **`link2`를 `link1`의 자식으로 이동:**
    *   `Stage` 창에서 `link2` 프림을 찾습니다.
    *   `link2` 프림을 마우스로 드래그하여 `link1` 프림 위로 가져다 놓습니다. 이렇게 하면 `link2`가 `link1`의 자식이 되어 계층 구조가 형성됩니다.
3.  **수정된 계층 구조 확인:**
    *   `Stage` 창에서 `link1`을 확장했을 때, `link2`가 그 아래에 표시되는지 확인합니다. 최종적으로 다음과 같은 구조가 되어야 합니다.
        ```
        /World
        └── /open_manipulator_x1  (또는 worldBody)
            └── /link1  (<- Articulation Root는 여기에만 설정)
                ├── /link2
                │   ├── /link3
                │   │   ├── /link4
                │   │   │   └── /link5
                │   │   │       ├── /gripper_left_link
                │   │   │       └── /gripper_right_link
                │   │   └── /end_effector
                ... (기타 컴포넌트)
        ```

### 2.2. Articulation Root 재설정

1.  **기존 Articulation Root 제거:** 이전 가이드에서 언급했듯이, `link1`을 제외한 모든 다른 링크(`link2`, `link3` 등)에 적용된 `ArticulationRootAPI`를 모두 제거합니다.
2.  **`link1`에 Articulation Root 적용:** **새로운 최상위 베이스 링크인 `link1`에만 `ArticulationRootAPI`를 적용합니다.**

## 3. 수정된 구조로 Lula Robot Description Editor 설정하기

로봇 계층 구조를 올바르게 재구성한 후, Lula 설정을 진행합니다.

1.  **시뮬레이션 재시작:** 시뮬레이션을 다시 `Play` 상태로 만듭니다.
2.  **Lula Robot Description Editor 열기:** `Window` > `Lula Robot Description Editor`를 엽니다.
3.  **Robot Prim 선택:**
    *   에디터의 `Robot Prim` 필드 옆에 있는 `Select` 버튼을 클릭합니다.
    *   **반드시 `link1`을 선택합니다.** 이것이 이제 로봇의 유일한 베이스 링크입니다.
4.  **관절 목록 확인 및 설정:**
    *   `Joints` 탭으로 이동합니다.
    *   이제 `Select link` 드롭다운 메뉴에 `link1`부터 모든 하위 링크가 올바르게 표시되어야 합니다.
    *   **Active Joints 설정:** `open_manipulator_x1`의 모든 구동 모터에 해당하는 관절을 `Active Joints` 목록에 추가합니다.
        *   `link1`
        *   `link2`
        *   `link3`
        *   `link4`
        *   `link5` (그리퍼 회전 관절)
5.  **충돌 구(Collision Spheres) 설정:**
    *   `Spheres` 탭으로 이동하여 각 링크(`link1`부터 그리퍼까지)에 충돌 구를 추가하고 크기와 위치를 조절합니다.
6.  **파일 내보내기:**
    *   `Export` 탭에서 `Export Robot Description`을 클릭하여 최종 YAML 파일을 저장합니다.

이제 분리된 로봇 계층 구조 문제를 해결하고, `link1`부터 시작하는 올바른 관절 체인을 구성하여 RMPflow를 위한 설정을 성공적으로 마칠 수 있습니다. 이 수정된 YAML 파일을 사용하면 RMPflow가 로봇의 모든 관절을 올바르게 제어할 수 있게 됩니다.
