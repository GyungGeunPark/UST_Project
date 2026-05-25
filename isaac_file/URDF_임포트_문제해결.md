# URDF 임포트 Overwrite 문제 해결 가이드

## 문제 상황

"Overwrite?" 다이얼로그가 반복적으로 나타나고, `Yes`를 눌러도 임포트가 진행되지 않습니다.

## 원인

1. **대상 폴더에 기존 USD 파일이 이미 존재**
2. **파일 권한 문제**로 덮어쓰기 실패
3. **Isaac Sim이 임포트 결과를 저장할 수 없음**

## 해결 방법

### 방법 1: 기존 파일 삭제 후 재시도 (가장 빠름)

#### 1단계: 기존 임포트 파일 삭제
터미널에서 실행:
```bash
# 기존에 임포트된 open_manipulator 관련 USD 파일 삭제
rm -f /home/isaac/ust_ws/isaac_file/open_manipulator*.usd
rm -rf /home/isaac/ust_ws/isaac_file/open_manipulator*

# 또는 전체 isaac_file 폴더 정리 (선택사항)
cd /home/isaac/ust_ws/isaac_file
ls -la
# 불필요한 파일만 삭제
```

#### 2단계: Isaac Sim에서 재임포트
1. **Isaac Sim 재시작** (깨끗한 상태에서 시작)
2. **URDF Importer 다시 열기**
3. **임포트 시도**

---

### 방법 2: 다른 출력 경로 사용

#### URDF Importer 설정 변경:

1. **Import Directory 변경**:
   ```
   기존: /home/isaac/ust_ws/isaac_file/
   새로: /home/isaac/ust_ws/isaac_file/import_output/
   ```

2. **새 폴더 생성**:
   ```bash
   mkdir -p /home/isaac/ust_ws/isaac_file/import_output
   chmod 777 /home/isaac/ust_ws/isaac_file/import_output
   ```

3. **URDF Importer에서**:
   - `Output Directory` 또는 `Import To` 필드를 찾음
   - 위의 새 경로로 변경
   - Import 버튼 클릭

---

### 방법 3: Stage에 직접 임포트 (권장)

이 방법은 파일로 저장하지 않고 현재 Stage에 직접 임포트합니다.

#### 1단계: 빈 Stage 준비
```
File > New (Ctrl+N)
```

#### 2단계: URDF Import 설정
1. **Isaac Utils > URDF Importer**
2. **Import Configuration**:
   - ✅ `Merge Fixed Joints`: 체크 해제 (모든 joint 유지)
   - ✅ `Fix Base Link`: 체크
   - ✅ `Self Collision`: 체크
   - ✅ `Import Inertia Tensor`: 체크
   - ✅ `Make Default Prim`: 체크

#### 3단계: URDF 파일 경로
```
Input File: /home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf
```

#### 4단계: Package Directory 설정
```
open_manipulator_description: /home/isaac/ust_ws/robotis_mujoco_menagerie/open_manipulator/open_manipulator_description
```

#### 5단계: Import To Current Stage
- **`Import To`** 옵션을 찾아서 `Current Stage` 선택
- 또는 출력 경로를 지정하지 않음

#### 6단계: Import 실행
- `Import` 버튼 클릭
- Stage에 직접 로드됨

#### 7단계: 수동 저장
```
File > Save As...
이름: open_manipulator_x_from_urdf.usd
위치: /home/isaac/ust_ws/isaac_file/
```

---

### 방법 4: Python 스크립트로 임포트

Isaac Sim의 Script Editor를 사용하여 직접 임포트:

#### Script Editor에서 실행할 코드:

```python
from omni.isaac.urdf import _urdf
import omni.usd

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

# Stage 가져오기
stage = omni.usd.get_context().get_stage()

# URDF 임포트 실행
result, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    package_paths=package_paths
)

print(f"Import result: {result}")
print(f"Robot imported at: {prim_path}")
```

**실행 방법**:
1. `Window` > `Script Editor`
2. 위 코드 복사 & 붙여넣기
3. `Run` 버튼 클릭

---

## 추가 해결 방법

### 터미널에서 기존 파일 확인 및 삭제

```bash
# 1. 기존 파일 확인
ls -la /home/isaac/ust_ws/isaac_file/*manipulator*

# 2. USD 캐시 삭제
rm -rf ~/.local/share/ov/data/Kit/Isaac-Sim/*/cache/*

# 3. 임시 파일 삭제
rm -rf /tmp/*manipulator*

# 4. 권한 수정
chmod -R 755 /home/isaac/ust_ws/isaac_file/
chown -R isaac:isaac /home/isaac/ust_ws/isaac_file/
```

---

## 권장 순서

### 🥇 가장 빠른 해결 (5분):

1. **터미널에서 기존 파일 삭제**:
   ```bash
   rm -f /home/isaac/ust_ws/isaac_file/open_manipulator*.usd
   ```

2. **Isaac Sim 재시작**

3. **방법 3 (Stage에 직접 임포트) 사용**

4. **성공 후 수동으로 저장**

---

## 예방 방법

앞으로 이 문제를 피하려면:

1. ✅ **항상 빈 Stage에서 시작**
2. ✅ **Import To Current Stage 옵션 사용**
3. ✅ **임포트 완료 후 수동으로 저장**
4. ✅ **출력 폴더에 쓰기 권한 확인**

---

## Console 에러 확인

만약 여전히 실패하면:

1. **Console 창 열기**: `Window` > `Console`
2. **에러 메시지 확인**
3. **에러 내용**:
   - Permission denied → 권한 문제
   - File already exists → 기존 파일 삭제 필요
   - Invalid path → 경로 문제

---

## 최종 확인 사항

✅ URDF 파일 경로가 정확한가?
✅ Package path가 올바르게 설정되었는가?
✅ 메쉬 파일들이 모두 존재하는가?
✅ 대상 폴더에 쓰기 권한이 있는가?

---

**바로 시도해보세요**: 방법 1 (기존 파일 삭제) + 방법 3 (Stage에 직접 임포트)를 조합하면 가장 빠르게 해결됩니다!
