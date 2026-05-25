# 문제 해결 가이드

## 🔍 "IK Target이 움직이지 않음" 문제 해결

### 진단 체크리스트

#### 1단계: 기본 설정 확인

**Unity Console 확인:**
```
Window → General → Console
```

Play 모드에서 다음 로그를 확인하세요:

✅ **기대되는 로그:**
```
[WebUIBridge] Web UI Bridge initialized
[ControlManager] System initialized. Ready for commands.
[IKRobotController] Bio IK detected and enabled
```

❌ **에러 로그 예시:**
```
[ControlManager] Configuration Error: API Key is not set
[WebUIBridge] Control Manager not available
[IKRobotController] IK Target not assigned!
```

---

#### 2단계: 컴포넌트 확인

**RobotControlSystem GameObject 확인:**

1. **Hierarchy**에서 `RobotControlSystem` 선택
2. **Inspector**에서 다음 컴포넌트 확인:

```
RobotControlSystem
├─ LLMRobotControlManager
│   ├─ Config → RobotControlConfig 할당됨 ✓
│   ├─ OpenAI Client → OpenAIClient 컴포넌트 ✓
│   ├─ Command Validator → CommandValidator 컴포넌트 ✓
│   └─ IK Controller → IKRobotController 컴포넌트 ✓
│
├─ OpenAIClient
│   └─ Config → RobotControlConfig 할당됨 ✓
│
├─ CommandValidator
│   └─ Config → RobotControlConfig 할당됨 ✓
│
├─ IKRobotController
│   ├─ Config → RobotControlConfig 할당됨 ✓
│   ├─ IK Target → IK_Target Transform 할당됨 ✓
│   ├─ Bio IK → BioIK.BioIK 할당됨 (선택 사항) ✓
│   └─ Bio IK Position Objective → Position 할당됨 (선택 사항) ✓
│
└─ WebUIBridge
    └─ Control Manager → LLMRobotControlManager 할당됨 ✓
```

---

#### 3단계: RobotControlConfig 확인

**Project 창에서 RobotControlConfig 자산 선택:**

1. **OpenAI Settings:**
   - ✅ API Key가 입력되어 있어야 함 (절대 비어있으면 안됨!)
   - ✅ Model: `gpt-4-turbo` 또는 `gpt-3.5-turbo`
   - ✅ API Timeout: 30초 이상

2. **Workspace Bounds:**
   - ✅ Min: (-1, 0, -1)
   - ✅ Max: (1, 2, 1)

3. **Safety Constraints:**
   - ✅ Max Single Movement: 1.0m
   - ✅ Min Distance To Obstacles: 0.2m

---

#### 4단계: IK_Target 확인

**Hierarchy에서 IK_Target 찾기:**

1. IK_Target GameObject가 존재하는지 확인
2. Transform 위치 확인
3. Scene View에서 시각적으로 보이는지 확인

**문제:** IK_Target이 없는 경우
**해결:**
```
1. GameObject → Create Empty
2. 이름을 "IK_Target"으로 변경
3. Position 설정 (예: 0, 1, 0.5)
4. IKRobotController의 IK Target 필드에 할당
```

---

#### 5단계: Bio IK 확인 (Bio IK 사용 시)

**Bio IK 컴포넌트 확인:**

1. 로봇 GameObject에 `BioIK.BioIK` 컴포넌트가 있는지 확인
2. BioSegment와 Position objective가 설정되어 있는지 확인
3. Position objective의 Target이 IK_Target으로 설정되어 있는지 확인
4. Weight가 1.0인지 확인

**Console 로그 확인:**
```
[IKRobotController] Bio IK detected and enabled
[IKRobotController] Found 1 Position objective(s) in Bio IK. Using the first one.
```

**경고 메시지:**
```
[IKRobotController] Please manually verify that the Position objective's target is set to IK_Target.
```

**해결:**
1. Bio IK component 선택
2. Segments 펼치기
3. Position objective 찾기
4. Target을 IK_Target Transform으로 설정
5. Weight = 1.0 확인

---

## 🧪 테스트 방법

### 방법 1: 테스터 스크립트 사용 (권장)

1. **LLMRobotControlTester.cs 추가:**
   ```
   RobotControlSystem에 LLMRobotControlTester 컴포넌트 추가
   ```

2. **Inspector에서 설정:**
   ```
   - Control Manager: LLMRobotControlManager 할당
   - Test Command: "앞으로 30cm 이동" 입력
   - Enable Verbose Logging: ✓
   ```

3. **테스트 실행:**
   ```
   Play 모드 진입
   → Inspector에서 Execute Test Command 체크박스 클릭
   또는
   → 키보드 T 키 누르기
   ```

4. **Console 로그 확인:**
   ```
   [Tester] ===== 테스트 명령 실행 =====
   [Tester] 명령: 앞으로 30cm 이동
   [ControlManager] Processing command: 앞으로 30cm 이동
   [OpenAIClient] Sending request to OpenAI API...
   [Parser] Function arguments: {...}
   [CommandValidator] Validating command...
   [IKRobotController] Moving from (0, 0, 0) to (0, 0, 0.3)
   ```

### 방법 2: Unity UI 사용

HTML UI가 작동하지 않는 경우, Unity UI를 직접 사용하세요:

1. **Canvas 생성:**
   ```
   GameObject → UI → Canvas
   ```

2. **Input Field 추가:**
   ```
   Canvas → UI → Input Field - TextMeshPro
   ```

3. **Button 추가:**
   ```
   Canvas → UI → Button - TextMeshPro
   버튼 텍스트: "실행"
   ```

4. **연결:**
   ```
   LLMRobotControlManager 선택
   → Command Input Field: Input Field 할당

   Button 선택
   → On Click() 이벤트 추가
   → LLMRobotControlManager.ProcessCommand 선택
   ```

5. **테스트:**
   ```
   Play 모드 진입
   → Input Field에 "앞으로 30cm 이동" 입력
   → 실행 버튼 클릭
   ```

---

## 🐛 일반적인 문제와 해결책

### 문제 1: "API Key is not set"

**증상:**
```
[ControlManager] Configuration Error: API Key is not set
```

**해결:**
1. Project 창에서 `RobotControlConfig` 선택
2. Inspector에서 API Key 필드에 OpenAI API 키 입력
3. Apply 또는 저장

**확인:**
- API 키가 "sk-"로 시작하는지 확인
- 공백이나 줄바꿈이 없는지 확인

---

### 문제 2: "IK Target not assigned"

**증상:**
```
[IKRobotController] IK Target not assigned!
```

**해결:**
1. Hierarchy에서 IK_Target GameObject 찾기 (없으면 생성)
2. RobotControlSystem → IKRobotController 선택
3. IK Target 필드에 IK_Target의 Transform 드래그

---

### 문제 3: "Control Manager not available"

**증상:**
```
[WebUIBridge] Control Manager not available
```

**해결:**
1. RobotControlSystem → WebUIBridge 선택
2. Control Manager 필드에 LLMRobotControlManager 할당

---

### 문제 4: API 호출 후 응답 없음

**증상:**
- 명령을 입력했지만 로봇이 움직이지 않음
- Console에 "Processing command..." 후 아무 메시지 없음

**가능한 원인:**

**A. API 타임아웃**
```
Console 확인:
[OpenAIClient] Request timeout

해결:
- RobotControlConfig → API Timeout을 60초로 증가
- 인터넷 연결 확인
```

**B. API 키 오류**
```
Console 확인:
[OpenAIClient] API Error: Invalid API key

해결:
- OpenAI API 키가 유효한지 확인
- https://platform.openai.com/api-keys 에서 새 키 생성
```

**C. 파싱 오류**
```
Console 확인:
[Parser] Failed to parse response

해결:
- 더 간단한 명령 시도: "앞으로 10cm"
- 다른 모델 시도: gpt-3.5-turbo → gpt-4-turbo
```

---

### 문제 5: IK Target은 움직이지만 로봇이 안움직임

**증상:**
- Scene View에서 IK_Target (녹색 구체)이 움직임
- 로봇 관절이 움직이지 않음

**Bio IK 사용 시 해결:**

1. **Bio IK 컴포넌트 확인:**
   ```
   로봇 GameObject 선택
   → BioIK.BioIK 컴포넌트 확인
   → Enabled 체크
   ```

2. **Position Objective 확인:**
   ```
   Bio IK → Segments 펼치기
   → Objectives 펼치기
   → Position objective 찾기
   → Target: IK_Target 설정
   → Weight: 1.0 설정
   → Enabled 체크
   ```

3. **Solver 설정:**
   ```
   Bio IK → Solver Settings
   → Iterations: 20-50
   → Enable Solver: ✓
   ```

**Animation Rigging 사용 시 해결:**

1. **Rig Builder 확인:**
   ```
   로봇 GameObject 선택
   → Rig Builder 컴포넌트
   → Rig Layers 확인
   → Build Rig 버튼 클릭
   ```

2. **Rig Weight 확인:**
   ```
   Rig Layer → Rig 컴포넌트
   → Weight: 1.0
   ```

3. **Constraint 확인:**
   ```
   IK Constraint 선택
   → Target: IK_Target
   → Target Position Weight: 1.0
   ```

---

### 문제 6: 웹 UI가 작동하지 않음 (SOLVED!)

**증상:**
- `text_input.html`에서 명령 입력해도 반응 없음
- `http://localhost:8080` 연결 가능하지만 명령이 Unity에 도달하지 않음

**원인:**
- 기존 `text_input.html`은 Webots RobotWindow API를 사용
- Unity SimpleHTTPServer와 호환되지 않음

**해결책 (3가지 방법):**

**방법 A: Unity 호환 HTML 사용 (신규, 권장!)**
```
1. 브라우저에서 http://localhost:8080/text_input_unity.html 열기
2. 명령 입력: "앞으로 30cm 이동"
3. 실행 버튼 클릭 또는 Enter 키
4. ✅ 연결됨 상태 확인
5. IK Target이 이동하는 것 확인
```

**방법 B: Unity UI 사용**
```
위의 "방법 2: Unity UI 사용" 참조
```

**방법 C: LLMRobotControlTester 사용**
```
위의 "방법 1: 테스터 스크립트 사용" 참조
```

**참고:** 새로운 `text_input_unity.html`은 Unity SimpleHTTPServer와 완벽하게 호환되며, 연결 상태 표시 및 예제 버튼 기능이 포함되어 있습니다!

---

## 📋 디버그 체크리스트

명령이 작동하지 않을 때 순서대로 확인:

- [ ] Play 모드인가?
- [ ] Console에 에러 메시지가 있는가?
- [ ] RobotControlConfig에 API 키가 입력되어 있는가?
- [ ] IK_Target GameObject가 존재하는가?
- [ ] IKRobotController에 IK Target이 할당되어 있는가?
- [ ] LLMRobotControlManager의 모든 참조가 할당되어 있는가?
- [ ] Bio IK를 사용한다면:
  - [ ] BioIK.BioIK 컴포넌트가 활성화되어 있는가?
  - [ ] Position objective의 Target이 IK_Target인가?
  - [ ] Position objective의 Weight가 1.0인가?
- [ ] 인터넷 연결이 되어 있는가?
- [ ] OpenAI API 서비스가 정상인가?

---

## 🔬 상세 디버그 로깅 활성화

더 많은 정보가 필요한 경우:

1. **각 스크립트의 Debug.Log 확인**

   Console 필터 설정:
   ```
   [ControlManager] - 명령 처리 흐름
   [OpenAIClient] - API 통신
   [Parser] - 응답 파싱
   [CommandValidator] - 검증
   [IKRobotController] - 이동 실행
   [WebUIBridge] - 웹 통신
   [Setup] - 설정 과정
   ```

2. **Tester 스크립트 사용**
   ```
   LLMRobotControlTester 추가
   → Check Setup 버튼 클릭
   → 모든 컴포넌트 상태 확인
   ```

3. **Scene View 시각화**
   ```
   Scene View에서 확인:
   - 녹색 큐브: 작업 공간 경계
   - 녹색 구체: IK_Target 현재 위치
   - 빨간 구체: 목표 위치 (이동 중)
   - 노란 선: 이동 경로
   ```

---

## 💡 빠른 해결 팁

### 가장 빠른 테스트 방법

1. **LLMRobotControlTester.cs를 RobotControlSystem에 추가**
2. **Play 모드 진입**
3. **키보드 C 키 눌러 설정 확인**
4. **키보드 T 키 눌러 테스트 명령 실행**
5. **Console 로그 확인**

### 즉시 확인할 항목 (90% 문제 해결)

1. **API 키가 입력되어 있는가?**
2. **IK_Target GameObject가 존재하고 할당되어 있는가?**
3. **모든 컴포넌트 참조가 연결되어 있는가?**
4. **Bio IK Position objective의 Target이 IK_Target인가?**

---

## 📞 추가 도움

문제가 계속되면:

1. Console의 전체 로그 복사
2. Inspector 스크린샷 촬영
3. 어떤 명령을 입력했는지 기록
4. 어떤 단계까지 진행되었는지 확인

---

**다시 확인할 핵심 사항:**
- ✅ API 키 입력됨
- ✅ IK_Target 할당됨
- ✅ Bio IK Position objective Target = IK_Target
- ✅ 모든 컴포넌트 참조 연결됨
- ✅ 인터넷 연결 확인

이 가이드로 문제를 해결할 수 있을 것입니다!
