# 웹 UI 문제 해결 완료

## 🎯 문제 요약

**사용자 보고:**
- RobotControlConfig에 API 키 설정 완료
- HTML 인터페이스에서 "앞으로 30cm 가" 입력
- **IK Target이 전혀 움직이지 않음**

## 🔍 근본 원인 분석

### 1차 분석: WebUIBridge 검토
초기 분석에서는 `SimpleHTTPServer` 클래스가 누락된 것으로 판단했으나, 재검토 결과:

**발견:**
- `WebUIBridge.cs`에 `SimpleHTTPServer` 클래스가 완전히 구현되어 있음 (라인 274-409)
- HTTP 서버는 정상적으로 작동 중
- `http://localhost:8080` 연결 가능

### 2차 분석: HTML 파일 검토 (근본 원인 발견!)

**기존 `text_input.html` 문제점:**

```javascript
// 라인 10-12: Webots API 사용
import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';
window.robotWindow = new RobotWindow();

// 라인 20: Webots 전용 메서드
robotWindow.send(userInput);
```

**근본 원인:**
- 기존 HTML은 **Webots 시뮬레이터 전용** RobotWindow API를 사용
- Unity SimpleHTTPServer는 표준 HTTP POST/GET을 사용
- API 불일치로 인해 명령이 Unity에 도달하지 못함

### Unity SimpleHTTPServer가 기대하는 통신 방식:

```javascript
// WebUIBridge.cs가 처리하는 방식:
POST /command           → 명령 수신 (ReceiveMessage 호출)
GET  /status            → 상태 반환 (GetNextOutgoingMessage 호출)
GET  /[filename].html   → HTML 파일 제공
```

## ✅ 해결 방법

### 새로운 `text_input_unity.html` 생성

**주요 개선 사항:**

1. **Unity 호환 통신:**
```javascript
async function sendCommand() {
  const response = await fetch('http://localhost:8080/command', {
    method: 'POST',
    headers: { 'Content-Type': 'text/plain' },
    body: command
  });
}
```

2. **연결 상태 모니터링:**
```javascript
async function checkServerStatus() {
  const response = await fetch('http://localhost:8080/status');
  updateConnectionStatus(response.ok);
}
// 5초마다 자동 확인
```

3. **사용자 편의 기능:**
- 🟢/🔴 실시간 연결 상태 표시
- 📝 명령 예제 버튼 (클릭으로 즉시 입력)
- ⌨️ Enter 키 지원
- ✅ 명령 전송 성공/실패 피드백
- 🎨 개선된 한국어 UI

## 🚀 사용 방법

### 1. Unity 씬 실행
```
1. Unity에서 Play 모드 진입
2. Console에서 확인:
   [WebUIBridge] Web server started on port 8080
   [WebUIBridge] Web UI Bridge initialized
```

### 2. 웹 브라우저에서 접속
```
주소: http://localhost:8080/text_input_unity.html
```

### 3. 연결 확인
```
우측 상단에 "🟢 연결됨" 표시 확인
```

### 4. 명령 실행
```
방법 1: 예제 버튼 클릭
  → "앞으로 30cm", "뒤로 20cm" 등

방법 2: 직접 입력
  → 입력창에 "앞으로 30cm 이동" 입력
  → Enter 키 또는 실행 버튼 클릭
```

### 5. 결과 확인
```
Scene View에서:
  - 녹색 구체 (IK_Target)가 이동
  - 로봇 관절이 따라 움직임 (Bio IK 사용 시)

Console에서:
  [ControlManager] Processing command: 앞으로 30cm 이동
  [OpenAIClient] Sending request to OpenAI API...
  [IKRobotController] Moving from (x, y, z) to (x, y, z+0.3)
```

## 📊 파일 비교

### 기존 text_input.html (작동 안함)
```
✗ Webots RobotWindow.js 의존
✗ robotWindow.send() 사용
✗ Unity와 통신 불가
✗ 연결 상태 표시 없음
```

### 신규 text_input_unity.html (작동함!)
```
✓ 표준 Fetch API 사용
✓ POST /command로 명령 전송
✓ Unity SimpleHTTPServer와 완벽 호환
✓ 연결 상태 실시간 표시
✓ 예제 버튼 및 개선된 UI
✓ 한국어 완벽 지원
```

## 🎯 대체 테스트 방법

웹 UI 외에도 3가지 테스트 방법 제공:

### 방법 1: 신규 웹 UI (권장!)
```
http://localhost:8080/text_input_unity.html
→ 가장 사용하기 편리함
→ 연결 상태 시각적 확인
```

### 방법 2: LLMRobotControlTester
```
RobotControlSystem에 컴포넌트 추가
→ 키보드 T: 테스트 명령 실행
→ 키보드 C: 설정 확인
```

### 방법 3: Unity UI
```
Canvas + TMP_InputField + Button
→ LLMRobotControlManager.ProcessCommand 연결
→ HTML 없이 Unity 내부에서 직접 테스트
```

## 📋 검증 체크리스트

테스트 전 확인사항:

- [ ] Unity Play 모드 실행 중
- [ ] Console에 "Web server started" 메시지 확인
- [ ] `http://localhost:8080/text_input_unity.html` 접속
- [ ] 우측 상단 "🟢 연결됨" 표시 확인
- [ ] RobotControlConfig에 API 키 입력됨
- [ ] IK_Target GameObject가 씬에 존재
- [ ] IKRobotController에 IK Target 할당됨
- [ ] Bio IK 사용 시: Position objective Target = IK_Target

## 🐛 여전히 작동하지 않는 경우

### 1. 웹 UI 연결 안됨 (🔴 연결 안됨)
```
원인: Unity Play 모드가 아님
해결: Play 버튼 클릭 후 다시 시도
```

### 2. 명령 전송은 되는데 IK Target이 안 움직임
```
원인: IK Target 할당 누락 또는 Bio IK 설정 오류
해결: TROUBLESHOOTING_KR.md의 "문제 5" 참조
```

### 3. API 오류 발생
```
Console 확인:
[OpenAIClient] API Error: Invalid API key
→ RobotControlConfig의 API 키 재확인

[OpenAIClient] Request timeout
→ API Timeout 값을 60초로 증가
```

## 📚 관련 문서

- [README_KR.md](README_KR.md) - 전체 시스템 설명
- [TROUBLESHOOTING_KR.md](TROUBLESHOOTING_KR.md) - 상세 문제 해결 가이드
- [QUICK_REFERENCE_KR.md](QUICK_REFERENCE_KR.md) - 빠른 참조 카드

## 🎉 결론

**문제:** 기존 HTML이 Webots API를 사용하여 Unity와 통신 불가

**해결:** Unity SimpleHTTPServer와 호환되는 새로운 HTML 생성

**결과:** `text_input_unity.html`을 통해 웹 브라우저에서 완벽하게 로봇 제어 가능!

---

**파일 위치:**
```
Assets/Report/text_input_unity.html  ← 새로운 Unity 호환 HTML (사용!)
Assets/Report/text_input.html       ← 기존 Webots 전용 HTML (사용 안함)
```

**테스트 완료 후 다음 단계:**
1. 간단한 명령으로 기본 동작 확인
2. 복잡한 명령으로 LLM 파싱 테스트
3. Bio IK 통합 확인
4. 안전 제약 조건 테스트

즐거운 로봇 제어 되세요! 🤖✨
