# LLM 로봇 제어 - 빠른 참조 카드

## 🚀 5분 설정

```
1. Tools → Robot Control → Create Config Asset
2. config에 OpenAI API 키 추가
3. Tools → Robot Control → Setup Scene
4. Tools → Robot Control → Validate Setup
5. Play 누르기 → 명령 입력 → 완료!
```

---

## 📍 파일 위치

```
스크립트:       Assets/Scripts/LLMRobotControl/
구성:          Assets/[사용자 위치]/RobotControlConfig.asset
HTML UI:       Assets/Report/text_input_unity.html (권장!), button_input.html
문서:          Assets/Scripts/LLMRobotControl/README_KR.md
```

---

## 🎯 좌표 시스템

```
앞       →  +Z축  →  "앞으로 30cm 이동"  →  (0, 0, +0.3)
뒤       →  -Z축  →  "뒤로 20cm 이동"    →  (0, 0, -0.2)
왼쪽     →  -X축  →  "왼쪽으로 10cm"     →  (-0.1, 0, 0)
오른쪽   →  +X축  →  "오른쪽으로 15cm"   →  (+0.15, 0, 0)
위       →  +Y축  →  "위로 5cm 이동"     →  (0, +0.05, 0)
아래     →  -Y축  →  "아래로 8cm 이동"   →  (0, -0.08, 0)

변환: 1 cm = 0.01 Unity 단위 (기본값)
```

---

## 💬 명령 예제

### 기본 이동
```
"앞으로 30cm 이동"
"뒤로 20cm 가"
"천천히 왼쪽으로 10cm 이동"
"빠르게 오른쪽으로 15cm 이동"
"위로 5cm 이동"
"정지"
```

### 절대 위치 지정
```
"x=0.5, y=1.0, z=0.3 위치로 이동"
"좌표 0.5, 1.0, 0.3으로 이동"
"x=0, y=1.5, z=0 위치에 도달"
```

### 복잡한 명령
```
"앞으로 30cm 이동한 다음 왼쪽으로 10cm 이동"
"손을 1.5미터까지 올려"
"5초 동안 천천히 앞으로 이동"
```

---

## ⚙️ 구성 빠른 설정

### 필수 설정
```
API Key:           [OpenAI API 키] ⚠️ 절대 커밋하지 마세요!
Model:             gpt-4-turbo (최고) 또는 gpt-3.5-turbo (빠름)
API Timeout:       30초
```

### 안전 (기본값)
```
Workspace Min:     (-1, 0, -1)
Workspace Max:     (1, 2, 1)
Max Movement:      명령당 1.0m
Min Obstacle Dist: 0.2m
```

### 성능
```
API Call Interval: 최소 1.0초
Cache Size:        50개 명령
User Confirmation: 활성화 (선택 사항)
Confirmation Time: 10초
```

---

## 🎮 단축키

```
Space          비상 정지 (즉시 정지)
R              비상 정지 재설정
```

---

## 🔧 Unity 메뉴 명령

```
Tools → Robot Control →
  ├─ Setup Scene              (완전한 시스템 생성)
  ├─ Create Config Asset      (구성 만들기)
  ├─ Validate Setup           (모든 것 확인)
  ├─ Open Documentation       (README.md 보기)
  └─ About                    (버전 정보)
```

---

## 📦 필수 컴포넌트 설정

```
RobotControlSystem (GameObject)
  ├─ RobotControlConfig (할당됨)
  ├─ OpenAIClient
  ├─ CommandValidator
  ├─ IKRobotController
  │   ├─ Bio IK (선택 사항, 자동 감지)
  │   └─ Bio IK Position Objective (선택 사항, 자동 감지)
  ├─ LLMRobotControlManager
  ├─ WebUIBridge (선택 사항)
  ├─ PerformanceMonitor (선택 사항)
  └─ EmergencyStopSystem (선택 사항)

IK_Target (GameObject)
  └─ 시각적 표시기 (구체)
```

---

## 🌐 웹 UI 액세스

```
씬 시작:
  - 텍스트 UI:  http://localhost:8080/text_input_unity.html (Unity 호환)
  - 버튼 UI:    http://localhost:8080/button_input.html

포트: WebUIBridge에서 구성 가능 (기본: 8080)
```

---

## 🐛 빠른 문제 해결

| 문제 | 해결책 |
|------|--------|
| "API Key not set" | RobotControlConfig에 키 추가 |
| 로봇이 움직이지 않음 | Bio IK weight = 1.0 확인 또는 Position objective 확인 |
| 웹 UI 명령이 안 들어감 | text_input_unity.html 사용 (text_input.html은 Webots 전용) |
| 시간 초과 오류 | 시간 초과 증가, 인터넷 확인 |
| 웹 UI 404 | Report 폴더의 HTML 파일 확인 |
| 떨리는 움직임 | config에서 movement curve 조정 |
| 파싱 오류 | 더 간단한 명령 시도, 모델 확인 |
| Bio IK 감지 안됨 | 씬에 BioIK.BioIK 컴포넌트가 있는지 확인 |

---

## 📊 컴포넌트 참조

### 핵심 흐름
```
사용자 명령
  ↓
LLMRobotControlManager (오케스트레이터)
  ↓
OpenAIClient (API 호출)
  ↓
OpenAIResponseParser (JSON 파싱)
  ↓
CommandValidator (안전 확인)
  ↓
IKRobotController (이동 실행)
  ↓
Bio IK (IK 솔빙) 또는 직접 타겟 제어
```

### 주요 메서드

#### LLMRobotControlManager
```csharp
ProcessCommand(string cmd)      // 메인 진입점
ConfirmCommand()                // 이동 승인
CancelCommand()                 // 이동 취소
EmergencyStop()                 // 즉시 정지
GetStatistics()                 // 메트릭 가져오기
```

#### IKRobotController
```csharp
MoveToPosition(Vector3, float, float)  // 이동 실행
StopCurrentMovement()                  // 정지
IsMoving()                             // 상태 확인
GetCurrentPosition()                   // 위치 가져오기
GetMovementProgress()                  // 진행률 0-1
SetBioIK(BioIK.BioIK)                 // Bio IK 설정
SetBioIKPositionObjective(Position)    // Position objective 설정
```

---

## 📈 성능 메트릭

### 예상 성능
```
API 호출:          1-3초 (첫 번째)
캐시된 명령:       < 0.1초
이동:             0.1-10초 (구성 가능)
성공률:           > 95% (유효한 명령)
메모리 오버헤드:   < 100 MB
```

### 모니터링
```csharp
// 자동 추적:
- API 응답 시간
- 이동 지속 시간
- 성공/실패 횟수
- 캐시 히트율
- FPS 및 메모리
```

---

## 🔒 안전 계층

```
1. LLM 지침        ← 지침이 포함된 시스템 프롬프트
2. JSON 스키마      ← 유형 검증
3. Command Validator ← 경계 + 충돌
4. 사용자 확인      ← 인간 승인
5. 비상 정지       ← 즉시 정지
```

---

## 🎨 시각화 (Scene View)

```
녹색 큐브:        작업 공간 경계
녹색 구체:        IK 타겟 현재 위치
빨간 구체:        목표 목적지 (이동 중)
노란 선:          이동 경로
```

---

## 📝 로깅 레벨

```csharp
// 상세 로깅 활성화:
[OpenAIClient]        // API 통신
[Parser]              // 응답 파싱
[CommandValidator]    // 검증 결과
[IKRobotController]   // 이동 실행
[ControlManager]      // 오케스트레이션 흐름
[WebUIBridge]         // 웹 통신
[Performance]         // 메트릭 업데이트
[EmergencyStop]       // 정지 이벤트
[Setup]               // 설정 과정
```

---

## 🔗 중요 링크

### 문서
- README_KR.md - 전체 문서
- IMPLEMENTATION_SUMMARY_KR.md - 기술 세부 정보
- BIO_IK_INTEGRATION.md - Bio IK 통합 가이드
- LLM_Robot_Control_System_Design.md - 설계 사양

### 외부 리소스
- OpenAI API: https://platform.openai.com/docs
- Bio IK: Asset Store 문서
- Unity Animation Rigging: https://docs.unity3d.com/Packages/com.unity.animation.rigging
- Unity 스크립팅: https://docs.unity3d.com/ScriptReference/

---

## 💡 전문가 팁

### 1. 간단하게 시작
```
시작: "앞으로 10cm 이동"
아님:  "장애물 회피 포함 복잡한 궤적 실행"
```

### 2. 캐싱 사용
```
일반 명령 반복하여 즉시 실행
캐시가 자동으로 마지막 50개 명령 저장
```

### 3. 안전 테스트
```
실제 배포 전에 항상 비상 정지 테스트
처음에는 보수적인 작업 공간 경계 설정
```

### 4. 성능 모니터링
```
PerformanceMonitor 디스플레이 확인
정기적으로 통계 검토
메트릭을 기반으로 최적화
```

### 5. 구성 반복
```
작은 작업 공간으로 시작
점진적으로 경계 확장
속도 및 지속 시간 조정
안전 여유 조정
```

### 6. Bio IK 사용
```
복잡한 로봇에 Bio IK 권장
Position objective를 IK_Target에 수동 할당
Bio IK 솔버 반복 횟수 조정 (20-50)
여러 제약 조건 결합 가능
```

---

## 🎯 성공 체크리스트

설정:
- [ ] Config 자산 생성됨
- [ ] API 키 추가됨 (git에 커밋 안됨!)
- [ ] 씬 설정 완료
- [ ] 검증 통과
- [ ] IK target 할당됨
- [ ] Bio IK 감지됨 (선택 사항)

테스트:
- [ ] 간단한 명령 작동
- [ ] 비상 정지 작동
- [ ] 사용자 확인 작동
- [ ] 웹 UI 액세스 가능
- [ ] 성능 허용 가능

안전:
- [ ] 작업 공간 경계 올바르게 설정됨
- [ ] 충돌 감지 작동
- [ ] 비상 정지 테스트됨
- [ ] 시간 초과 처리 작동
- [ ] 오류 메시지 명확함

---

## 🚨 비상 절차

### 로봇이 오작동하는 경우
```
1. 즉시 SPACE 누르기 (비상 정지)
2. 오류에 대한 콘솔 확인
3. Tools → Robot Control → Validate Setup
4. 피드백 디스플레이에서 마지막 명령 검토
5. 필요 시 안전 제약 조정
```

### 시스템이 멈춘 경우
```
1. ESC 눌러 Play 모드 종료
2. 스택 추적에 대한 콘솔 확인
3. API 연결 확인
4. 시간 초과 설정 검토
5. 필요 시 Unity 재시작
```

---

## 📞 도움 받기

### 디버그 단계
```
1. 오류 메시지에 대한 콘솔 확인
2. 모든 컴포넌트 참조 확인
3. 검증 도구 실행
4. 구성 설정 검토
5. 최소 씬으로 테스트
```

### 제공할 정보
```
- Unity 버전
- 콘솔의 오류 메시지
- 구성 설정
- 실패한 명령
- 검증 보고서
- Bio IK 사용 여부
```

---

## 🆕 Bio IK 통합 빠른 가이드

### 자동 설정
```
1. 씬에 BioIK.BioIK 컴포넌트 확인
2. Tools → Robot Control → Setup Scene 실행
3. Bio IK 자동 감지 및 할당됨
4. Position objective를 IK_Target에 수동 할당 (권장)
```

### 수동 설정
```
1. IKRobotController 선택
2. Bio IK 필드에 BioIK.BioIK 컴포넌트 할당
3. Bio IK Position Objective 필드에 Position 할당
4. Position objective의 Target을 IK_Target으로 설정
5. Weight를 1.0으로 설정
```

### 검증
```
Tools → Robot Control → Validate Setup

예상 출력:
✓ Bio IK 컴포넌트 할당됨
✓ Bio IK Position objective 할당됨
```

---

**이 카드를 편리하게 보관하세요! 📌**

자세한 정보는 [README_KR.md](README_KR.md)를 참조하세요.
