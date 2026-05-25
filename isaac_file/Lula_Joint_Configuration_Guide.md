
# Lula Robot Description Editor 고급 관절 설정 안내서

이 문서는 Isaac Sim의 Lula Robot Description Editor에서 특정 관절(이 경우 `link5`)이 보이지 않는 문제를 해결하고, `open_manipulator_x1` 로봇의 그리퍼와 같이 회전으로 잡는 메커니즘을 올바르게 설정하는 방법을 안내합니다.

## 1. 문제 상황: `link5` 관절이 목록에 없음

Lula Robot Description Editor의 `Joints` 탭에서 `link2`를 선택했을 때, `link5`가 선택 가능한 관절 목록에 나타나지 않는 문제가 있습니다. 이는 `open_manipulator_x1`의 그리퍼가 `link5`의 회전으로 작동하기 때문에 RMPflow 설정을 위해 반드시 해결해야 하는 문제입니다.

## 2. 해결 단계

### 2.1. Articulation Root 수정

가장 가능성이 높은 원인은 Articulation Root 설정 오류입니다. **Articulation Root는 로봇의 최상위 부모 링크(보통 베이스 링크)에 *단 하나만* 설정해야 합니다.** 모든 링크에 Articulation Root를 설정하면 로봇의 계층 구조가 깨져 Lula가 관절을 올바르게 인식하지 못합니다.

**해결 방법:**

1.  Isaac Sim의 `Stage` 창에서 `open_manipulator_x1` 로봇의 모든 링크를 확인합니다.
2.  베이스 링크(예: `open_manipulator_x1` 또는 `link1`)를 제외한 모든 링크에서 `Articulation Root API`를 제거합니다.
    *   링크를 마우스 오른쪽 버튼으로 클릭합니다.
    *   `Remove API` > `Physics` > `ArticulationRootAPI`를 선택합니다.
3.  **베이스 링크에만 `Articulation Root API`가 적용되어 있는지 다시 확인합니다.**

### 2.2. 로봇 관절 계층 구조 확인

Articulation Root를 수정한 후, 로봇의 관절 계층 구조가 올바른지 확인해야 합니다. `link5`는 `link4`에 연결되어 있어야 합니다.

1.  `Stage` 창에서 `open_manipulator_x1`의 계층 구조를 확장하여 각 링크의 부모-자식 관계를 확인합니다.
2.  `link5`가 `link4`의 자식으로 올바르게 연결되어 있는지 확인합니다. 만약 그렇지 않다면, 로봇의 USD 또는 URDF 파일 자체에 문제가 있을 수 있습니다.

### 2.3. Lula Robot Description Editor 재설정 및 관절 구성

위의 단계를 완료한 후, Lula Robot Description Editor에서 설정을 다시 시도합니다.

1.  **시뮬레이션 재시작:** Isaac Sim 시뮬레이션을 중지(`Stop`)했다가 다시 시작(`Play`)합니다.
2.  **Lula Robot Description Editor 열기:** `Window` > `Lula Robot Description Editor`를 엽니다.
3.  **Robot Prim 재선택:** `Robot Prim` 필드 옆의 `Select` 버튼을 클릭하여 로봇의 베이스 링크를 다시 선택합니다.
4.  **관절 목록 확인:**
    *   `Joints` 탭으로 이동합니다.
    *   이제 `Select link` 드롭다운에서 상위 링크(예: `link2`, `link3`, `link4`)를 선택했을 때, 연결된 모든 하위 관절(`link2`, `link3`, `link4`, `link5`, `gripper_left_link`, `gripper_right_link`)이 목록에 나타나야 합니다.

5.  **Active Joints 설정:**
    *   RMPflow가 직접 제어해야 하는 구동 관절들을 `Active Joints` 목록에 추가합니다. `open_manipulator_x1`의 경우, 다음과 같은 관절들이 해당될 수 있습니다.
        *   `link2`
        *   `link3`
        *   `link4`
        *   `link5` (그리퍼를 회전시키는 관절)
    *   `gripper_left_link`와 `gripper_right_link`는 `link5`의 회전에 따라 수동적으로 움직이므로, `Active Joints`에 추가하지 않을 수 있습니다. 대신, 이들은 `link5`에 의해 구동되는 것으로 간주됩니다.

6.  **파일 내보내기:**
    *   `Export` 탭으로 이동하여 `Export Robot Description` 버튼을 클릭해 YAML 파일을 저장합니다.

## 3. 추가 문제 해결

만약 위 단계를 모두 수행했음에도 `link5`가 여전히 목록에 나타나지 않는다면, 다음을 확인해 보십시오.

*   **USD/URDF 파일 확인:** 로봇의 원본 USD 또는 URDF 파일을 텍스트 에디터로 열어 `link5` 관절이 올바르게 정의되어 있는지, 그리고 `link4`와 `link5` 사이의 `joint` 태그가 정확한지 확인합니다. `type`이 `revolute` 또는 `continuous`로 설정되어 있어야 합니다.
*   **시뮬레이션 상태:** 시뮬레이션이 반드시 `Play` 상태여야 합니다.

이제 `link5`를 포함한 모든 구동 관절을 올바르게 설정하여 RMPflow를 위한 로봇 설정을 성공적으로 마칠 수 있습니다.
