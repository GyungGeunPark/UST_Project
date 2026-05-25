
# Isaac Sim Articulation Root 연결 안내서

이 문서는 Isaac Sim에서 로봇의 Articulation Root를 설정하는 방법을 안내합니다.

## 1. Isaac Sim 실행

먼저, Isaac Sim을 실행합니다. 터미널에서 다음 명령어를 입력하여 Isaac Sim을 실행할 수 있습니다.

```bash
./isaac-sim.sh
```

## 2. 스크립트 에디터 열기

Isaac Sim이 실행되면, 상단 메뉴에서 `Window` > `Script Editor`를 선택하여 스크립트 에디터를 엽니다.

## 3. 스크립트 작성

스크립트 에디터에 다음 코드를 입력합니다. 이 코드는 `/World/Robot/open_manipulator_x1` 프림에 Articulation Root API를 적용하는 예제입니다.

```python
import omni.usd
from pxr import Usd, UsdPhysics

def set_articulation_root():
    # USD 스테이지 가져오기
    stage = omni.usd.get_context().get_stage()

    # 프림 가져오기
    prim_path = "/World/Robot/open_manipulator_x1"
    prim = stage.GetPrimAtPath(prim_path)

    # Articulation Root API 적용
    if prim:
        print(f"'{prim_path}' 프림에 Articulation Root API를 적용합니다.")
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        print("Articulation Root API가 적용되었습니다.")

        # (선택 사항) 수정된 USD 파일 저장
        # output_path = "/path/to/your/project/ust_project1_articulation.usd"
        # omni.usd.get_context().save_as_stage(output_path)
        # print(f"스테이지가 '{output_path}'에 저장되었습니다.")
    else:
        print(f"'{prim_path}' 프림을 찾을 수 없습니다.")

# 함수 실행
set_articulation_root()
```

## 4. 스크립트 실행

스크립트 에디터에서 `Run` 버튼을 클릭하여 스크립트를 실행합니다. 스크립트가 실행되면, 콘솔 창에 Articulation Root API가 적용되었다는 메시지가 출력됩니다.

## 5. (선택 사항) 수정된 USD 파일 저장

만약 수정된 USD 파일을 저장하고 싶다면, 스크립트의 다음 라인의 주석을 해제하고 `output_path`를 원하는 경로로 수정한 후 스크립트를 다시 실행하십시오.

```python
# output_path = "/path/to/your/project/ust_project1_articulation.usd"
# omni.usd.get_context().save_as_stage(output_path)
# print(f"스테이지가 '{output_path}'에 저장되었습니다.")
```

이제 Isaac Sim에서 로봇의 Articulation Root가 성공적으로 설정되었습니다.
