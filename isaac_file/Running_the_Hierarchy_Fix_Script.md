
# 로봇 계층 구조 수정을 위한 스크립트 실행 안내서

이 문서는 `ust_project1.usd` 파일의 분리된 로봇 계층 구조 문제를 해결하기 위해 제공된 `fix_robot_hierarchy.py` 스크립트를 Isaac Sim에서 실행하는 방법을 안내합니다.

## 배경

현재 `ust_project1.usd` 파일 내의 `open_manipulator_x1` 로봇은 `link1`과 `link2`가 분리되어 있어 Lula와 같은 로보틱스 툴에서 단일 로봇으로 인식되지 않는 문제가 있습니다.

제공된 `fix_robot_hierarchy.py` 스크립트는 다음 작업을 자동으로 수행합니다.

1.  `link2`를 `link1`의 자식으로 이동시켜 올바른 계층 구조를 만듭니다.
2.  기존에 잘못 적용된 `ArticulationRootAPI`를 모두 제거합니다.
3.  새로운 베이스 링크인 `link1`에 `ArticulationRootAPI`를 올바르게 적용합니다.
4.  수정된 결과를 **`ust_project1_fixed.usd`** 라는 새로운 파일로 저장합니다.

## 스크립트 실행 방법

1.  **원본 파일 열기:** Isaac Sim에서 기존의 `ust_project1.usd` 파일을 엽니다.

2.  **스크립트 에디터 열기:** 상단 메뉴에서 `Window` > `Script Editor`를 선택하여 스크립트 에디터 창을 엽니다.

3.  **스크립트 내용 복사 및 붙여넣기:**
    *   `fix_robot_hierarchy.py` 파일의 **전체 내용**을 복사합니다.
    *   스크립트 에디터 창에 그대로 붙여넣습니다.

4.  **스크립트 실행:**
    *   스크립트 에디터 상단의 `Run` 버튼을 클릭하여 스크립트를 실행합니다.
    *   스크립트가 실행되는 동안 콘솔 창(`Window` > `Console`)을 통해 진행 상황(프림 이동, API 적용, 저장 등)을 확인할 수 있습니다.

5.  **결과 확인:**
    *   스크립트 실행이 완료되면, `isaac_file` 폴더 안에 **`ust_project1_fixed.usd`** 라는 새로운 파일이 생성됩니다.
    *   이 파일이 바로 계층 구조가 수정된 새로운 프로젝트 파일입니다.

## 후속 작업

1.  **새 파일 사용:** 앞으로 Lula Robot Description Editor 설정 및 기타 로보틱스 작업을 할 때는 원본 `ust_project1.usd` 파일 대신 **`ust_project1_fixed.usd` 파일을 열어서 사용하십시오.**
2.  **Lula 설정 진행:** `ust_project1_fixed.usd` 파일을 연 상태에서, 이전에 제공된 `Fixing_Robot_Hierarchy_for_Lula.md` 안내서에 따라 Lula Robot Description Editor 설정을 진행합니다. 이제 `link1`부터 `link5`까지 모든 관절이 올바르게 표시될 것입니다.

이 과정을 통해 로봇의 구조적인 문제를 해결하고 RMPflow를 위한 설정을 원활하게 진행할 수 있습니다.
