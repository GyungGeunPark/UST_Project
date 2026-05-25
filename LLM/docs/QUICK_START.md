# Isaac Sim LLM Robot Control - 빠른 시작 가이드

5분 안에 시스템을 실행하는 방법입니다.

---

## 1단계: 의존성 설치

```bash
cd /workspace/isaaclab/ust_ws/LLM
pip install -r requirements.txt
```

## 2단계: API 키 설정

```bash
# OpenAI 사용 시
export OPENAI_API_KEY="sk-your-openai-api-key"

# 또는 Anthropic 사용 시
export ANTHROPIC_API_KEY="sk-ant-your-anthropic-key"
```

## 3단계: 서버 실행

```bash
python scripts/run_standalone.py
```

## 4단계: 웹 UI 접속

브라우저에서 열기:
```
http://localhost:8000
```

## 5단계: 명령 전송

웹 UI에서 명령 입력:
```
앞으로 10cm
그리퍼 열어
위로 5cm
```

---

## API로 명령 전송

```bash
# 명령 전송
curl -X POST http://localhost:8000/api/command \
  -H "Content-Type: application/json" \
  -d '{"command": "앞으로 10cm"}'

# 상태 확인
curl http://localhost:8000/api/status
```

---

## 지원 명령어

| 명령 | 예시 |
|------|------|
| 이동 | "앞으로 10cm", "위로 5cm" |
| 그리퍼 | "그리퍼 열어", "그리퍼 닫아" |
| 정지 | "정지", "stop" |

---

## 다음 단계

- 상세 설정: [INSTALLATION_GUIDE.md](./INSTALLATION_GUIDE.md)
- Isaac Sim 연동: [INSTALLATION_GUIDE.md#5-isaac-sim-연동](./INSTALLATION_GUIDE.md#5-isaac-sim-연동)
