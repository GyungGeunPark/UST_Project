# Isaac Sim URDF Importer 활성화 가이드

## URDF Importer는 기본 내장되어 있습니다!

Isaac Sim에는 `omni.isaac.urdf` 확장이 **기본으로 포함**되어 있습니다. 별도 설치가 필요 없습니다.

## 방법 1: Extensions Manager에서 활성화 (GUI)

### 1단계: Extensions Manager 열기
```
Window > Extensions
```

### 2단계: URDF 검색
Extensions 창 상단의 검색창에:
```
urdf
```
입력

### 3단계: 확장 활성화
다음 확장들을 찾아서 활성화:

#### 필수 확장:
- ✅ **`omni.isaac.urdf`** - URDF Importer 핵심
- ✅ **`omni.importer.urdf`** - URDF 파서

#### 선택 확장 (자동으로 활성화됨):
- `omni.isaac.core`
- `omni.isaac.dynamic_control`

### 4단계: 확인
각 확장 옆에 **토글 스위치가 켜져 있는지** 확인 (파란색)

---

## 방법 2: Script Editor로 활성화 (Python)

### Isaac Sim Script Editor에서 실행:

```python
import omni.kit.app

# Extension Manager 가져오기
ext_manager = omni.kit.app.get_app().get_extension_manager()

# URDF 관련 확장 활성화
extensions_to_enable = [
    "omni.isaac.urdf",
    "omni.importer.urdf",
    "omni.isaac.core"
]

for ext in extensions_to_enable:
    if not ext_manager.is_extension_enabled(ext):
        print(f"Enabling {ext}...")
        ext_manager.set_extension_enabled_immediate(ext, True)
        print(f"✓ {ext} enabled")
    else:
        print(f"✓ {ext} already enabled")

print("\nAll URDF extensions are ready!")
```

---

## 방법 3: URDF Importer 메뉴 확인

활성화되었는지 확인하는 가장 쉬운 방법:

### 메뉴 확인:
```
Isaac Utils > URDF Importer
```

이 메뉴가 보이면 **이미 활성화되어 있는 것**입니다!

---

## URDF Import 사용 방법

### GUI 방법:

#### 1. URDF Importer 열기
```
Isaac Utils > URDF Importer
또는
Window > Isaac Utils > URDF Importer
```

#### 2. 파일 경로 입력
```
Input File: [URDF 파일 경로]
```

#### 3. Import 버튼 클릭

---

### Python Script 방법 (권장):

Isaac Sim의 **Script Editor**에서 실행:

```python
from omni.isaac.urdf import _urdf
import omni.kit.commands

# URDF 파일 경로
urdf_path = "/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf"

# Package 경로
package_paths = {
    "open_manipulator_description": "/home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description"
}

# Import 설정
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = True
import_config.make_default_prim = True
import_config.self_collision = True
import_config.create_physics_scene = True
import_config.distance_scale = 1.0

print("Starting URDF import...")

# URDF 임포트 실행
try:
    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        package_paths=package_paths
    )

    print(f"✓ Import successful!")
    print(f"✓ Robot imported at: {prim_path}")

except Exception as e:
    print(f"✗ Error: {e}")
```

---

## 문제 해결

### "omni.isaac.urdf not found" 에러

#### 원인:
Extension이 비활성화되어 있음

#### 해결:
1. `Window > Extensions` 열기
2. `urdf` 검색
3. `omni.isaac.urdf` 활성화

---

### "URDFParseAndImportFile command not found" 에러

#### 원인:
Isaac Sim이 제대로 초기화되지 않음

#### 해결:
```python
# Script Editor에서 먼저 실행
import omni.isaac.core.utils.extensions as extensions_utils

# 필요한 확장 로드
extensions_utils.enable_extension("omni.isaac.urdf")
extensions_utils.enable_extension("omni.importer.urdf")

print("Extensions loaded. Please try importing again.")
```

---

### URDF Importer 메뉴가 안 보임

#### 해결 1: Extensions 확인
```
Window > Extensions > 검색: "urdf" > 활성화
```

#### 해결 2: Isaac Sim 재시작
```
File > Exit
그 다음 Isaac Sim 다시 실행
```

#### 해결 3: Script로 직접 임포트
위의 Python Script 방법 사용

---

## 확장 의존성

URDF Importer가 작동하려면 다음 확장들이 필요합니다:

### 자동으로 활성화되는 확장:
- `omni.isaac.core` - Isaac Sim 핵심
- `omni.isaac.dynamic_control` - 물리 제어
- `omni.physx` - PhysX 물리 엔진
- `omni.usd` - USD 처리

### 수동 확인 필요:
```python
# Script Editor에서 확인
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()

required = [
    "omni.isaac.urdf",
    "omni.importer.urdf",
    "omni.isaac.core"
]

for ext in required:
    enabled = ext_manager.is_extension_enabled(ext)
    print(f"{ext}: {'✓ Enabled' if enabled else '✗ Disabled'}")
```

---

## 추가 팁

### 1. 확장 자동 활성화 설정

**Extensions Manager에서**:
- 확장 찾기
- 우클릭 > `Autoload` 체크
- 다음부터 자동으로 로드됨

### 2. 확장 상태 저장

Isaac Sim은 마지막 세션의 확장 상태를 기억합니다:
- 한 번 활성화하면 계속 활성화 상태 유지
- 재시작 후에도 유지됨

### 3. Python에서 확장 확인

```python
import carb.settings

settings = carb.settings.get_settings()

# URDF 관련 설정 확인
urdf_settings = settings.get("/exts/omni.isaac.urdf")
print(f"URDF Extension settings: {urdf_settings}")
```

---

## 빠른 확인 체크리스트

✅ **Extensions Manager에서 `omni.isaac.urdf` 활성화됨**
✅ **Extensions Manager에서 `omni.importer.urdf` 활성화됨**
✅ **메뉴에 `Isaac Utils > URDF Importer` 표시됨**
✅ **Script Editor에서 `from omni.isaac.urdf import _urdf` 에러 없이 실행됨**

모두 체크되면 URDF Import 준비 완료!

---

## 추천 워크플로우

### 초보자:
1. `Window > Extensions` 열기
2. `urdf` 검색
3. `omni.isaac.urdf` 활성화
4. `Isaac Utils > URDF Importer` 사용

### 고급 사용자:
1. Script Editor 열기
2. 위의 Python 코드 사용
3. 더 많은 제어 및 자동화 가능

---

**대부분의 경우 Isaac Sim에 URDF Importer가 이미 활성화되어 있습니다!**
`Isaac Utils` 메뉴를 확인해보세요.
