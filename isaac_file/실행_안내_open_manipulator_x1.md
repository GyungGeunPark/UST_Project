# open_manipulator_x1 로봇 계층 구조 자동 수정 안내서

이 문서는 `ust_project1.usd` 파일 내의 `open_manipulator_x1` 로봇의 계층 구조를 자동으로 수정하는 방법을 안내합니다.

## 문제 상황

현재 `ust_project1.usd` 파일에서 `open_manipulator_x1` 로봇은 다음과 같은 문제가 있습니다:

### 현재 잘못된 구조:
```
/World/Robot/open_manipulator_x1/
  ├── link2 (link2,3,4,5 및 그리퍼들이 모두 여기 있음)
  └── worldBody (link1 메쉬만 있음)
```

**문제점:**
1. **link1이 누락**: 실제 link1이 `worldBody` 안에 메쉬로만 존재하고, 독립된 링크가 아닙니다
2. **분리된 계층**: `worldBody`와 `link2`가 형제 관계로 분리되어 있어, Lula가 하나의 완전한 로봇으로 인식하지 못합니다
3. **잘못된 Articulation Root**: ArticulationRootAPI가 잘못된 위치에 설정되어 있을 수 있습니다
4. **관절 인식 문제**: Lula Robot Description Editor에서 모든 관절이 올바르게 표시되지 않습니다

## 해결 방법

제공된 `fix_open_manipulator_x1.py` 스크립트는 다음 작업을 자동으로 수행합니다:

### 스크립트가 수행하는 작업

1. **자동 경로 탐색**:
   - Stage 전체를 탐색하여 `open_manipulator_x1`, `worldBody`, `link2`의 정확한 경로를 자동으로 찾습니다
   - 수동으로 경로를 입력할 필요가 없습니다

2. **페이로드 처리**:
   - 로봇에 페이로드(Payload)가 있는지 확인합니다
   - 페이로드가 있으면 자동으로 로드하여 편집 가능한 상태로 만듭니다

3. **worldBody에서 link1 생성**:
   - `worldBody`의 내용을 복사하여 새로운 `link1`을 생성합니다
   - 페이로드로 로드된 경우에도 작동하도록 복사 방식을 사용합니다
   - 원본 `worldBody`는 삭제됩니다
   - 기존 link1 메쉬와 모든 속성이 그대로 유지됩니다

4. **계층 구조 재구성**:
   - `link2`(및 모든 자식들)를 새로운 `link1`의 자식으로 이동시킵니다
   - 올바른 상위-하위 관계를 만들어 하나의 통합된 로봇 계층을 구성합니다

5. **Articulation Root 정리**:
   - 기존에 잘못 적용된 모든 ArticulationRootAPI를 제거합니다
   - 새로운 `link1`(베이스 링크)에만 ArticulationRootAPI를 올바르게 적용합니다

6. **계층 검증**:
   - 새로운 구조가 올바르게 생성되었는지 확인합니다

7. **새 파일 저장**:
   - 수정된 결과를 **`ust_project1_fixed.usd`**라는 새로운 파일로 저장합니다
   - 원본 파일은 그대로 보존됩니다

### 수정 후 올바른 구조:
```
/World/Robot/open_manipulator_x1/
  └── link1 (ArticulationRoot, worldBody에서 이름 변경)
      └── link2 (이동됨, 모든 자식들과 함께)
          ├── link2
          ├── link3
          ├── link4
          ├── link5
          ├── gripper_left_link
          ├── gripper_right_link
          └── end_effector_target
```

## 스크립트 실행 방법

### 1단계: Isaac Sim에서 원본 파일 열기
- Isaac Sim을 실행합니다.
- `File` > `Open`을 선택하여 `ust_project1.usd` 파일을 엽니다.

### 2단계: 스크립트 에디터 열기
- 상단 메뉴에서 `Window` > `Script Editor`를 선택합니다.
- 스크립트 에디터 창이 열립니다.

### 3단계: 스크립트 복사 및 붙여넣기
1. `fix_open_manipulator_x1.py` 파일을 텍스트 에디터로 엽니다.
2. **파일의 전체 내용**을 복사합니다 (Ctrl+A, Ctrl+C).
3. Isaac Sim의 스크립트 에디터 창에 붙여넣습니다 (Ctrl+V).

### 4단계: 스크립트 실행
1. 스크립트 에디터 상단의 **`Run`** 버튼을 클릭합니다.
2. 콘솔 창(`Window` > `Console`)에서 진행 상황을 확인할 수 있습니다:
   ```
   === Starting Robot Hierarchy Fix ===

   Current structure:
     open_manipulator_x1/
       ├── link2 (has link2,3,4,5, grippers)
       └── worldBody (has link1 mesh)

   Target structure:
     open_manipulator_x1/
       └── link1 (renamed from worldBody)
           └── link2 (moved here with all children)

   --- Step 1: Searching for robot components ---
   Found robot root at: /World/Robot/open_manipulator_x1
   Found worldBody at: /World/Robot/open_manipulator_x1/worldBody
   Found link2 at: /World/Robot/open_manipulator_x1/link2
   ✓ Successfully located all components

   --- Step 2: Checking for payloads ---
   ✓ No payloads found (this is normal)

   --- Step 3: Creating new link1 from worldBody ---
     Creating link1 at: /World/Robot/open_manipulator_x1/link1
   ✓ Successfully created link1 from worldBody
     New path: /World/Robot/open_manipulator_x1/link1
     Removing original worldBody...
   ✓ Original worldBody removed

   --- Step 4: Moving link2 (and all children) under link1 ---
     Moving '/World/Robot/open_manipulator_x1/link2' to '/World/Robot/open_manipulator_x1/link1/link2'...
   ✓ Successfully moved link2 under link1
     All children (link2,3,4,5, grippers) moved together

   --- Step 5: Cleaning up Articulation Root APIs ---
   ✓ Removed 2 ArticulationRootAPI instances

   --- Step 6: Applying ArticulationRootAPI to link1 ---
   ✓ Applied ArticulationRootAPI to '/World/Robot/open_manipulator_x1/link1'

   --- Step 7: Verifying new hierarchy ---
   ✓ Hierarchy verified:
     ├── link1: /World/Robot/open_manipulator_x1/link1
     └── link2: /World/Robot/open_manipulator_x1/link1/link2
     link2 has 7 children (link2,3,4,5, grippers, etc.)

   --- Step 8: Saving fixed USD file ---
   Saving to: /home/isaac/ust_ws/isaac_file/ust_project1_fixed.usd
   ✓ Save complete!

   ======================================================================
   SUCCESS! Robot hierarchy has been fixed!
   ======================================================================
   ```

### 5단계: 수정된 파일 확인
- `isaac_file` 폴더에 **`ust_project1_fixed.usd`** 파일이 생성되었는지 확인합니다.

## 다음 단계: Lula 설정

수정된 파일로 Lula Robot Description Editor 설정을 진행합니다.

### 1. 새 파일 열기
- Isaac Sim에서 현재 파일을 닫습니다.
- `File` > `Open`으로 **`ust_project1_fixed.usd`** 파일을 엽니다.

### 2. Lula Robot Description Editor 실행
1. 상단 메뉴에서 `Window` > `Extensions`를 엽니다.
2. "Lula"를 검색하여 Lula Robot Description Editor 확장을 활성화합니다.
3. `Window` > `Lula Robot Description Editor`를 엽니다.

### 3. Robot Prim 선택
1. Lula 에디터의 **`Robot Prim`** 필드 옆 **`Select`** 버튼을 클릭합니다.
2. Stage 창에서 **`link1`**을 선택합니다.
   - 경로는 `/World/Robot/open_manipulator_x1/link1`입니다.

### 4. 관절 확인
1. `Joints` 탭으로 이동합니다.
2. 이제 드롭다운 메뉴에 다음 관절들이 모두 표시되어야 합니다:
   - `link1`
   - `link2`
   - `link3`
   - `link4`
   - `link5`
   - 그리퍼 관련 링크들

### 5. Active Joints 설정
- 로봇의 구동 관절(`link1` ~ `link5`)을 **Active Joints** 목록에 추가합니다.

### 6. 충돌 구(Collision Spheres) 설정
- `Spheres` 탭에서 각 링크에 충돌 구를 추가하고 위치와 크기를 조정합니다.

### 7. YAML 파일 내보내기
- `Export` 탭에서 **`Export Robot Description`**을 클릭하여 RMPflow용 설정 파일을 저장합니다.

## 문제 해결

### 스크립트 실행 오류
- **"Could not find required components" 오류**:
  - Stage 창에서 `open_manipulator_x1`, `worldBody`, `link2`가 모두 존재하는지 확인하세요
  - 스크린샷과 같은 구조인지 확인하세요
- **"Failed to create link1" 오류**:
  - worldBody의 복사에 실패했습니다
  - 스크립트가 자동으로 대체 방법(DefinePrim)을 시도합니다
  - 시뮬레이션을 중지한 상태에서 스크립트를 실행하세요
- **"Failed to move link2" 오류**:
  - link2가 이미 다른 곳으로 이동했거나 삭제되었을 수 있습니다
  - 원본 파일을 다시 열고 처음부터 시도하세요

### Lula에서 여전히 관절이 보이지 않음
1. Stage 창에서 `link1`을 확장하여 `link2`가 그 안에 있는지 확인합니다.
2. `link1`에 ArticulationRootAPI가 적용되어 있는지 확인합니다 (Property 패널에서).
3. 다른 링크에는 ArticulationRootAPI가 없는지 확인합니다.

## 중요 참고사항

- **원본 파일 보존**: 원본 `ust_project1.usd` 파일은 수정되지 않습니다. 모든 변경사항은 `ust_project1_fixed.usd`에 저장됩니다.
- **향후 작업**: 앞으로는 항상 **`ust_project1_fixed.usd`** 파일을 사용하세요.
- **재실행**: 문제가 발생하면 언제든지 원본 파일로 돌아가서 스크립트를 다시 실행할 수 있습니다.

## 스크립트의 장점

현재 로봇 구조에 맞춰 특별히 제작된 스크립트의 특징:

1. ✅ **현재 구조 정확히 파악**: worldBody와 link2가 분리된 특수한 구조를 정확히 인식합니다
2. ✅ **자동 경로 탐색**: 로봇의 정확한 경로를 자동으로 찾습니다
3. ✅ **페이로드 호환 복사**: 페이로드로 로드된 경우에도 작동하도록 worldBody를 복사하여 link1 생성합니다
4. ✅ **계층 재구성**: link2와 모든 자식들을 새로운 link1 아래로 이동시킵니다
5. ✅ **페이로드 자동 처리**: 페이로드 유무를 확인하고 자동으로 로드합니다
6. ✅ **상세한 진행 상황**: 각 단계별로 명확한 피드백을 제공합니다
7. ✅ **계층 검증**: 수정 후 구조가 올바른지 자동으로 확인합니다
8. ✅ **안전성**: 원본 파일을 보존하고 새 파일로 저장합니다

## 요약

이 스크립트는 **스크린샷에 나온 바로 그 구조**(worldBody + link2 분리)를 정확히 인식하고 수정합니다:

- **Before**: `open_manipulator_x1/` 아래에 `worldBody`와 `link2`가 형제로 분리
- **After**: `open_manipulator_x1/link1/` 아래에 `link2`가 자식으로 통합

수동으로 계층 구조를 수정하는 번거로움 없이, 스크립트 실행 한 번으로 로봇을 Lula와 RMPflow 사용에 적합하게 준비할 수 있습니다!
