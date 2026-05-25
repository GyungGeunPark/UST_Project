# LLM 로봇 제어 시스템
## OpenAI API 통합 Unity 구현

자연어 명령을 사용하여 Unity에서 로봇을 제어하는 포괄적인 시스템입니다. 대화형 입력을 정확한 역운동학(IK) 타겟 움직임으로 변환합니다.

---

## 📋 목차

- [기능](#기능)
- [시스템 아키텍처](#시스템-아키텍처)
- [전제 조건](#전제-조건)
- [설치](#설치)
- [빠른 시작](#빠른-시작)
- [컴포넌트 참조](#컴포넌트-참조)
- [설정](#설정)
- [사용 예제](#사용-예제)
- [문제 해결](#문제-해결)
- [API 참조](#api-참조)

---

## ✨ 기능

- **자연어 제어**: 일상적인 언어로 로봇 명령
- **OpenAI 통합**: GPT-4/GPT-3.5를 활용한 지능적 명령 파싱
- **IK 타겟 제어**: Animation Rigging을 통한 정밀한 엔드 이펙터 위치 지정
- **다층 안전**: 작업 공간 경계, 충돌 감지, 사용자 확인
- **웹 UI 인터페이스**: HTML 기반 제어 패널(버튼 및 텍스트 입력)
- **성능 모니터링**: 실시간 메트릭 및 통계
- **비상 정지**: Space 키로 즉시 정지
- **명령 캐싱**: 반복 명령에 대한 응답 시간 개선
- **좌표 변환**: 자동 cm-Unity 단위 변환

### 좌표 시스템

- **앞/뒤**: Z축 (+앞, -뒤)
- **좌/우**: X축 (-좌, +우)
- **위/아래**: Y축 (+위, -아래)

예제: "앞으로 30cm 이동" → IK 타겟이 Z축에서 +0.3 단위 이동

---

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────┐
│              사용자 인터페이스 계층                    │
│  ┌───────────────┐      ┌──────────────────────┐  │
│  │ HTML UI       │      │ Unity UI (TextMeshPro)│  │
│  │ - text_input  │      │ - 입력 필드            │  │
│  │ - button_input│      │ - 피드백 디스플레이     │  │
│  └───────┬───────┘      └──────────┬───────────┘  │
└──────────┼──────────────────────────┼──────────────┘
           │                          │
┌──────────▼──────────────────────────▼──────────────┐
│            LLMRobotControlManager                   │
│  - 명령 오케스트레이션                                │
│  - 사용자 확인 워크플로우                              │
│  - 명령 캐싱                                         │
└──────────┬──────────────────────────┬──────────────┘
           │                          │
     ┌─────▼─────┐             ┌─────▼──────┐
     │ OpenAI    │             │ Command    │
     │ Client    │             │ Validator  │
     └─────┬─────┘             └─────┬──────┘
           │                         │
           ▼                         ▼
     ┌──────────────────────────────────┐
     │     IKRobotController            │
     │  - 부드러운 움직임 보간             │
     │  - Animation Rigging 통합         │
     └──────────────────────────────────┘
```

---

## 📦 전제 조건

### 필수
- Unity 2021.3 LTS 이상
- .NET 4.x 또는 그 이상
- OpenAI API 키 ([여기서 발급](https://platform.openai.com/api-keys))

### 권장 Unity 패키지
- **TextMeshPro** (내장) - UI 텍스트용
- **Animation Rigging** (com.unity.animation.rigging) - IK 제약 조건용
- **Bio IK** - 고급 IK 솔빙 (선택 사항)
- **Newtonsoft Json** (com.unity.nuget.newtonsoft-json) - JSON 처리 (선택 사항)

### 선택 사항
- Unity WebGL 내보내기 (웹 기반 배포용)
- 적절한 리깅이 된 로봇 모델

---

## 🚀 설치

### 1단계: Unity 패키지 설치

```
Window → Package Manager
```

설치:
1. **TextMeshPro** (처음 사용 시 프롬프트 표시)
2. **Animation Rigging** (선택 사항)
   - 검색: "Animation Rigging"
   - 클릭: Install
3. **Bio IK** (권장)
   - Asset Store에서 설치

### 2단계: 스크립트 가져오기

모든 스크립트는 다음 위치에 있습니다:
```
Assets/Scripts/LLMRobotControl/
```

포함된 스크립트:
- `RobotCommand.cs` - 데이터 구조
- `RobotControlConfig.cs` - 구성 ScriptableObject
- `OpenAIClient.cs` - API 통신
- `OpenAIResponseParser.cs` - 응답 파싱
- `CommandValidator.cs` - 안전 검증
- `IKRobotController.cs` - IK 제어
- `LLMRobotControlManager.cs` - 메인 오케스트레이터
- `WebUIBridge.cs` - HTML UI 통합
- `PerformanceMonitor.cs` - 메트릭 추적
- `EmergencyStopSystem.cs` - 비상 정지
- `RobotControlSetupHelper.cs` - 에디터 설정 도우미

### 3단계: 구성 자산 생성

**방법 1: 메뉴 사용 (권장)**
```
Tools → Robot Control → Create Config Asset
```

**방법 2: 수동 생성**
```
Assets → Create → Robot Control → Config
```

이름 지정: `RobotControlConfig`

### 4단계: API 키 구성

**⚠️ 중요: API 키를 버전 관리에 커밋하지 마세요!**

1. `RobotControlConfig` 자산 선택
2. Inspector에서 OpenAI API 키 입력
3. 설정 구성:
   - Model: `gpt-4-turbo` (권장) 또는 `gpt-3.5-turbo`
   - 작업 공간 경계
   - 안전 제약 조건

---

## ⚡ 빠른 시작

### 방법 1: 자동 설정 (권장)

#### Unity 메뉴를 통한 설정

```
Tools → Robot Control → Setup Scene
```

이 명령은 자동으로:
1. RobotControlSystem GameObject 생성
2. 모든 필요한 컴포넌트 추가
3. IK_Target GameObject 생성
4. Bio IK 자동 감지 및 구성
5. 모든 참조 연결

#### 설정 검증

```
Tools → Robot Control → Validate Setup
```

예상 결과:
```
✓ LLMRobotControlManager 존재
✓ OpenAIClient 존재
✓ CommandValidator 존재
✓ IKRobotController 존재
✓ 구성 할당됨
✓ 구성 유효함
✓ IK Target 할당됨
✓ Bio IK 컴포넌트 할당됨
✓ Bio IK Position objective 할당됨
```

### 방법 2: 수동 설정

#### 1. 로봇 제어 시스템 생성

```
1. 빈 GameObject 생성: "RobotControlSystem"
2. 순서대로 컴포넌트 추가:
   - OpenAIClient
   - CommandValidator
   - IKRobotController
   - LLMRobotControlManager
   - WebUIBridge (선택 사항)
   - PerformanceMonitor (선택 사항)
   - EmergencyStopSystem (선택 사항)
```

#### 2. IK 타겟 설정

```
1. 빈 GameObject 생성: "IK_Target"
2. 원하는 엔드 이펙터 위치에 배치
3. 디버깅용 시각적 표시기(작은 구체) 추가
4. IKRobotController → IK Target 필드에 할당
```

#### 3. Bio IK 설정 (Bio IK 사용 시)

```
로봇 GameObject에:
1. "BioIK" 컴포넌트 추가
2. BioSegment 설정
3. Position objective 추가
4. Position objective의 Target을 IK_Target으로 설정
5. Weight를 1.0으로 설정
```

#### 4. 참조 연결

`LLMRobotControlManager`에서:
```
- Config → RobotControlConfig 자산
- OpenAI Client → OpenAIClient 컴포넌트
- Command Validator → CommandValidator 컴포넌트
- IK Controller → IKRobotController 컴포넌트
```

`IKRobotController`에서:
```
- Config → RobotControlConfig 자산
- IK Target → IK_Target GameObject
- Bio IK → BioIK.BioIK 컴포넌트 (자동 감지됨)
- Bio IK Position Objective → Position objective (자동 감지됨)
```

### 방법 3: 웹 UI 인터페이스

#### 1. 웹 서버 활성화

```
WebUIBridge 컴포넌트:
- Enable Web Server: ✓
- Web Server Port: 8080
```

#### 2. HTML UI 액세스

씬을 실행한 후 브라우저에서 열기:
- 텍스트 입력: `http://localhost:8080/text_input_unity.html` (Unity 호환 버전, 권장!)
- 텍스트 입력 (구버전): `http://localhost:8080/text_input.html` (Webots 전용, Unity에서 작동 안함)
- 버튼 입력: `http://localhost:8080/button_input.html`

`Assets/Report/`의 HTML 파일이 자동으로 제공됩니다.

---

## 🎮 사용 예제

### 자연어 명령

```
// Unity UI 또는 웹 UI에서:
"앞으로 30cm 이동"
"천천히 뒤로 20cm 이동"
"왼쪽으로 10cm 이동"
"손을 1.5미터까지 올려"
"x=0.5, y=1.0, z=0.3 위치로 이동"
"정지"
```

### 프로그래밍 방식 제어

```csharp
using LLMRobotControl;

public class MyController : MonoBehaviour
{
    [SerializeField] private LLMRobotControlManager controlManager;

    void Start()
    {
        // 프로그래밍 방식으로 명령 처리
        controlManager.ProcessCommand("앞으로 30cm 이동");

        // 이벤트 등록
        controlManager.OnCommandReceived += OnCommand;
        controlManager.OnCommandValidated += OnValidated;
        controlManager.OnCommandFailed += OnFailed;
    }

    void OnCommand(string command)
    {
        Debug.Log($"명령 수신: {command}");
    }

    void OnValidated(RobotCommand cmd)
    {
        Debug.Log($"명령 검증됨: {cmd}");
    }

    void OnFailed(string error)
    {
        Debug.LogError($"명령 실패: {error}");
    }
}
```

### 비상 정지

```csharp
// 비상 정지 트리거
if (Input.GetKeyDown(KeyCode.Space))
{
    controlManager.EmergencyStop();
}

// 또는 EmergencyStopSystem 컴포넌트 사용
var emergencySystem = GetComponent<EmergencyStopSystem>();
emergencySystem.ExecuteEmergencyStop();
```

---

## ⚙️ 설정

### RobotControlConfig 설정

#### OpenAI API
```
API Key: [OpenAI API 키]
Model: gpt-4-turbo
API Timeout: 30초
```

#### 작업 공간 경계
```
Min: (-1, 0, -1)
Max: (1, 2, 1)
```

#### 안전 제약 조건
```
최대 단일 이동: 1.0m (100cm)
장애물까지 최소 거리: 0.2m (20cm)
장애물 레이어: Obstacles
```

#### 이동 설정
```
Movement Curve: EaseInOut (기본값)
Position Threshold: 0.01m
```

#### 사용자 상호 작용
```
사용자 확인 활성화: 예
확인 시간 초과: 10초
```

#### 성능
```
최소 API 호출 간격: 1.0초
최대 캐시 크기: 50
```

#### 좌표 시스템
```
센티미터를 Unity 단위로: 0.01 (1cm = 0.01 Unity 단위)
```

---

## 🔧 컴포넌트 참조

### 핵심 컴포넌트

#### LLMRobotControlManager
모든 컴포넌트를 조정하는 메인 오케스트레이터입니다.

**공개 메서드:**
- `ProcessCommand(string command)` - 자연어 명령 처리
- `ConfirmCommand()` - 대기 중인 이동 확인
- `CancelCommand()` - 대기 중인 이동 취소
- `EmergencyStop()` - 즉시 정지
- `GetStatistics()` - 성능 통계 가져오기

**이벤트:**
- `OnCommandReceived` - 명령 수신 시
- `OnCommandValidated` - 명령 검증 통과 시
- `OnCommandFailed` - 명령 실패 시

#### OpenAIClient
OpenAI API 통신을 처리합니다.

**공개 메서드:**
- `SendChatRequest(string message, Action<string> onSuccess, Action<string> onError)` - API 요청 전송
- `CanMakeCall()` - 속도 제한 확인
- `GetStats()` - API 통계 가져오기

#### CommandValidator
안전을 위해 명령을 검증합니다.

**공개 메서드:**
- `ValidateCommand(RobotCommand command, Vector3 currentPosition)` - 명령 검증

**반환값:** 다음을 포함하는 `ValidationResult`:
- `isValid` - 검증 통과
- `errorMessage` - 오류 설명
- `safePosition` - 안전한 대체 위치
- `validatedCommand` - 명령

#### IKRobotController
IK 타겟 이동을 제어합니다.

**공개 메서드:**
- `MoveToPosition(Vector3 targetPos, float duration, float speed)` - 이동 실행
- `StopCurrentMovement()` - 현재 이동 중지
- `IsMoving()` - 이동 중인지 확인
- `GetCurrentPosition()` - IK 타겟 위치 가져오기
- `GetDistanceToTarget()` - 남은 거리
- `GetMovementProgress()` - 진행률 (0-1)
- `SetBioIK(BioIK.BioIK bioIKComponent)` - Bio IK 컴포넌트 설정
- `SetBioIKPositionObjective(Position objective)` - Bio IK Position objective 설정

**이벤트:**
- `OnMovementStarted` - 이동 시작
- `OnMovementCompleted` - 이동 종료
- `OnMovementFailed` - 이동 실패

#### WebUIBridge
Unity와 HTML UI 간의 브리지입니다.

**공개 메서드:**
- `ReceiveMessage(string message)` - 웹에서 수신
- `SendMessage(string message)` - 웹으로 전송
- `SendCommandFromExternal(string command)` - 외부 API

---

## 🐛 문제 해결

### 일반적인 문제

#### 1. API 키 오류

**증상:** "Configuration error: API Key is not set"

**해결책:**
1. `RobotControlConfig` ScriptableObject 열기
2. OpenAI API 키 붙여넣기
3. 이 파일을 버전 관리에 커밋하지 마세요!

#### 2. IK 타겟이 움직이지 않음

**증상:** 명령이 수락되었지만 로봇이 움직이지 않음

**진단:**
1. Bio IK 컴포넌트 활성화 및 weight > 0 확인
2. Position objective가 IK_Target에 할당되었는지 확인
3. Bio IK 솔버 설정 확인 (반복 횟수 > 10)
4. IK Target이 IKRobotController에 할당되었는지 확인

**해결책:**
```csharp
// Bio IK 강제 업데이트
bioIK.enabled = false;
yield return null;
bioIK.enabled = true;
```

#### 3. API 시간 초과

**증상:** "Request timeout" 오류

**해결책:**
- 시간 초과 증가: Config → API Timeout (60초 시도)
- 인터넷 연결 확인
- OpenAI 서비스 상태 확인
- 재시도 로직 추가

#### 4. 떨리는 움직임

**증상:** 로봇 움직임이 부드럽지 않음

**해결책:**
1. Config에서 movement curve 조정
2. 물리 계산에 FixedUpdate 사용
3. IK 솔버 반복 횟수 증가
4. Bio IK 솔버 설정 조정 (damping 활성화)

#### 5. 명령이 파싱되지 않음

**증상:** LLM이 예상치 못한 응답 반환

**해결책:**
1. Config에서 시스템 프롬프트 확인
2. 더 구체적인 명령 사용
3. 다른 모델 시도 (gpt-4 vs gpt-3.5-turbo)
4. 시스템 프롬프트에 예제 추가

#### 6. 웹 UI가 로드되지 않음

**증상:** `http://localhost:8080`에 액세스할 수 없음

**해결책:**
1. WebUIBridge가 활성화되었는지 확인
2. 포트가 사용 중이지 않은지 확인
3. 방화벽 설정 확인
4. 다른 포트 시도
5. 씬이 재생 중인지 확인

### 디버그 모드

자세한 로깅 활성화:
```csharp
// 각 컴포넌트에서
Debug.Log("[ComponentName] 상세 메시지");
```

콘솔에서 확인:
- `[OpenAIClient]` - API 통신
- `[Parser]` - 응답 파싱
- `[CommandValidator]` - 검증 결과
- `[IKRobotController]` - 이동 실행
- `[ControlManager]` - 오케스트레이션 흐름
- `[Setup]` - 설정 과정

---

## 📚 API 참조

### RobotCommand 구조

```csharp
public class RobotCommand
{
    public string movementType;      // "relative" 또는 "absolute"
    public string direction;         // "forward", "backward" 등
    public float distance;           // 센티미터 단위
    public Vector3? absolutePosition; // 절대 이동용
    public float speed;              // 0.1 - 2.0
    public float duration;           // 0.1 - 10.0초
}
```

### ValidationResult 구조

```csharp
public class ValidationResult
{
    public bool isValid;
    public string errorMessage;
    public Vector3 safePosition;
    public RobotCommand validatedCommand;
}
```

### OpenAI 함수 스키마

시스템이 사용하는 함수 정의:

```json
{
  "name": "move_robot_ik",
  "description": "로봇의 IK 타겟 이동",
  "parameters": {
    "type": "object",
    "properties": {
      "movement_type": {
        "type": "string",
        "enum": ["relative", "absolute"]
      },
      "direction": {
        "type": "string",
        "enum": ["forward", "backward", "left", "right", "up", "down"]
      },
      "distance": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 100.0
      },
      "position": {
        "type": "object",
        "properties": {
          "x": { "type": "number" },
          "y": { "type": "number" },
          "z": { "type": "number" }
        }
      },
      "speed": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 2.0,
        "default": 1.0
      },
      "duration": {
        "type": "number",
        "minimum": 0.1,
        "maximum": 10.0,
        "default": 2.0
      }
    },
    "required": ["movement_type"]
  }
}
```

---

## 🎯 모범 사례

### 1. 안전 우선
- 항상 적절한 작업 공간 경계 설정
- 중요한 이동에 대해 사용자 확인 활성화
- 충돌 감지 철저히 테스트
- 개발 중 비상 정지 사용

### 2. 성능 최적화
- 명령 캐싱 활성화
- 적절한 API 호출 간격 설정
- 더 빠른 응답을 위해 낮은 토큰 제한 사용
- 속도를 위해 gpt-3.5-turbo 사용 고려

### 3. 오류 처리
- 항상 검증 결과 확인
- API 시간 초과를 우아하게 처리
- 명확한 사용자 피드백 제공
- 디버깅을 위한 오류 로그

### 4. 테스트
- 작은 움직임부터 시작
- 엣지 케이스 테스트 (경계, 충돌)
- 비상 정지 작동 확인
- 성능 메트릭 모니터링

---

## 📝 예제 씬 설정 체크리스트

- [ ] 모든 스크립트를 `Assets/Scripts/LLMRobotControl/`에 가져오기
- [ ] Animation Rigging 패키지 설치 (선택 사항)
- [ ] Bio IK 패키지 설치 (권장)
- [ ] TextMeshPro 설치
- [ ] RobotControlConfig ScriptableObject 생성
- [ ] Config에 OpenAI API 키 추가
- [ ] 작업 공간 경계 구성
- [ ] Tools → Robot Control → Setup Scene 실행
- [ ] Tools → Robot Control → Validate Setup으로 검증
- [ ] Bio IK Position objective를 IK_Target에 수동 할당 (필요 시)
- [ ] 간단한 명령으로 테스트
- [ ] 비상 정지 작동 확인
- [ ] 안전 제약 조건 구성
- [ ] 웹 UI 활성화 (선택 사항)

---

## 📄 라이선스

연구 문서 기반: `LLM_Robot_Control_System_Design.md`

---

## 🤝 기여

문제, 개선 사항 또는 질문이 있는 경우:
1. 문제 해결 섹션 확인
2. 콘솔 로그 검토
3. 컴포넌트 참조 확인
4. 최소 씬으로 테스트

---

## 📞 지원

도움이 필요한 경우:
- **OpenAI API**: https://platform.openai.com/docs
- **Unity Animation Rigging**: https://docs.unity3d.com/Packages/com.unity.animation.rigging
- **Bio IK**: Asset Store 문서 참조
- **Unity 스크립팅**: https://docs.unity3d.com/ScriptReference/

---

**마지막 업데이트:** 2025-11-03
**버전:** 1.1 (Bio IK 통합 포함)
**Unity 버전:** 2021.3 LTS+
