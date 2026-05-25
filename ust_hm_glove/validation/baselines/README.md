# Baselines — UDCAP→Isaac Lab Finger Precision Validation

이 폴더는 **회귀 비교의 절대 기준** 으로 보존되는 검증 데이터셋. 향후 코드 변경 시 동일 데이터를 다시 처리해서 메트릭 비교.

## baseline_2026-05-02_c8c10 — 정밀제어 매칭 시스템 완성 시점

| 파일 | 크기 | 용도 |
|---|---|---|
| `baseline_2026-05-02_c8c10.vmc.jsonl` | 12 MB | 30s UDCAP raw VMC 녹화 (1938 thumb_proximal sample / 1000 frames) |
| `baseline_2026-05-02_c8c10.hdf5` | 0.2 MB | Layer-2 headless replay 결과 (target/actual joint pos × 22 × 1000 frame) |
| `baseline_2026-05-02_c8c10.png` | 0.7 MB | 22 joint × time 시각화 (target vs actual 거의 완전 겹침) |

### 적용 시점 코드 상태

- `ust_hm_glove/teleop/fourier_hand_mapper.py`: **9.19 (C8) + 9.20 (C10) patches 적용**
  - `_quat_to_yaw` → `_quat_to_pitch` (X 축 thumb opposition 추출)
  - 음수 thumb_yaw branch → 0 truncation (URDF clamp 회피)
- `ust_hm_glove/kitchen_sorting_gr1t2_env_cfg.py`: 변경 없음
- `ust_hm_glove.validation/scripts/run_replay_headless.py`: idle arm 14D + QP warmup
- 사용자 환경: Windows 11 + Pico 4 Ultra + Virtual Desktop + UDCAP

### 사용자 동작

30초 finger-by-finger sequence (사용자 발언):
- thumb 굽혔다 폈다 ~6s
- index ~6s
- middle ~6s
- ring ~6s
- pinky ~6s

**중요**: thumb opposition (좌우 흔들기) 풀 ROM 안 함 → coverage 26%/17%/35% (thumb 관련 3 슬롯).

### 정량 메트릭 (analyze_replay_hdf5)

```
Tracking error (max across 22 joints):  0.0069 rad (0.4°)  ← PhysX 한계 부근
Latency:                                  0 frames
Coverage (4 finger × 2 손 proximal):      63-80%  ALL PASS
Coverage (8 intermediate):                49-56%  PASS (URDF range 더 넓음)
Coverage (3 thumb 슬롯):                  17-35%  사용자 동작 한계
```

### 회귀 비교 절차 (향후)

```powershell
# 1. 코드 변경 후 동일 baseline 데이터로 재실행
python -m ust_ws.ust_hm_glove.validation.scripts.run_replay_headless `
    --replay ust_ws\ust_hm_gloveust_ws\validation\baselines\baseline_2026-05-02_c8c10.vmc.jsonl `
    --output ust_ws\ust_hm_gloveust_ws\validation\results\after_change.hdf5 `
    --steps 1000 --headless --subtract-rest

# 2. 직접 메트릭 비교
python -m ust_ws.ust_hm_glove.validation.tools.analyze_replay_hdf5 `
    ust_ws\ust_hm_gloveust_ws\validation\results\after_change.hdf5

# 3. 회귀 기준
#   - tracking max < 0.01 rad: PASS
#   - tracking max > 0.05 rad: 회귀 발생, 변경 검토 필수
```

### 본 baseline 의 역할

- §34 의 7-cause 분석 + C8 + C10 fix 가 **정량 입증된 시점** 의 스냅샷
- 9.21+ 미래 코드 변경 시 회귀 검출의 근거
- §35 의 정밀제어 매칭 시스템 통과 증거 (Layer-1 + Layer-2)

---

## 새 baseline 추가 시 명명 규칙

```
baseline_<YYYY-MM-DD>_<patch_set>.{vmc.jsonl,hdf5,png}

예시:
  baseline_2026-05-02_c8c10.*       (현재)
  baseline_2026-05-15_c11.*         (가설: thumb yaw range 확장 patch 후)
  baseline_2026-06-01_full_rom.*    (가설: 풀 ROM 재녹화 후)
```

각 baseline 마다 본 README 에 한 섹션 추가해서 코드 상태 + 메트릭 + 사용자 행동 기록.
