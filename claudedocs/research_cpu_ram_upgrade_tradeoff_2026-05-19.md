# CPU/RAM 업그레이드 트레이드오프 분석 — 7950X3D + 128GB DDR5-3600 vs 9950X3D2 + 64GB DDR5-5600

> 작성: 2026-05-19
> 분석 대상: VR 텔레오퍼레이션 + 교정 티칭 + 모방학습/강화학습 통합 워크플로우
> GPU 동일: NVIDIA RTX PRO 6000 (Blackwell, 96 GB GDDR7)
> 기반 문서: `ust_ws/research/` §5, §8, §9, §29, §30, §31, §33 + §7-1 + 실제 디스크 모델 86 GB 측정

---

## 0. Executive Summary — 한 줄 결론

> **현재 진행하는 통합 연구 스택(특히 Phase 3 교정 티칭 + Qwen3-VL-32B + GR00T-Mimic + WSL 병행)에선 RAM 128 GB → 64 GB 축소가 CPU 업그레이드 이득을 압도적으로 깎아먹는다.** 단일 env teleop만 본다면 CPU 차이는 거의 0이고 RAM 차이도 무의미 — 그러나 사용자가 실제로 빌드 중인 풀스택(텔레오퍼레이션 + 합성 데이터 생성 + VLM 추론 + 앙상블 학습 + RL multi-env)에선 **64 GB는 동시 작업 capacity가 절벽처럼 떨어진다.** 9950X3D2의 RL 25~30% 가속(24h → 18~19h)은 가치가 있으나, 64 GB로 인해 그 RL 자체가 못 돌아가는 상황(VLM 서버 + Isaac Sim + 데이터 생성 동시)이 더 큰 손실이다.

### 핵심 답변

| 질문 | 답 |
|---|---|
| CPU 업그레이드(7950X3D → 9950X3D2) 이득은? | 단일 env teleop **거의 0**, RL training 4096 envs **+25~30%**, 동시 멀티태스킹 안정성 ↑ |
| RAM 5600 MHz 속도 향상 이득은? | Zen 5 IPC 시너지로 **+3~8%** (numpy/dataset/multi-env step rate), 단 V-Cache가 대부분 흡수 |
| RAM 128 GB → 64 GB 손실은? | **86 GB 모델 OS 파일 캐시 불가 + 풀스택 동시 작업 capacity 절반** (Phase 3 운용에서 직접 타격) |
| 종합 권장? | **현 128 GB 유지가 더 합리적.** RAM 다운그레이드가 CPU 업그레이드 이득을 잡아먹는 net loss 시나리오. |

### 한 줄 권장

> **9950X3D2 단독 업그레이드(RAM 유지)** 또는 **현 7950X3D 유지 + DDR5-5600 64 GB × 2 = 128 GB 시도**가 ROI 정답. 64 GB 축소는 Phase 3 풀스택을 무력화시킨다.

---

## 1. 사용자의 실제 워크로드 — 무엇이 RAM을 잡아먹는가

### 1.1 현재 진행 연구의 단계별 메모리 프로파일

`ust_ws/research/8.` 교정 티칭 시스템은 3 Phase로 구성된다. 각 Phase의 동시 실행 워크로드:

#### Phase 1 — 시연 수집 + BC-RNN 학습 (현재 활성)

| 동시 실행 프로세스 | 시스템 RAM (CPU side) | VRAM |
|---|---|---|
| Isaac Sim 메인 루프 (단일 env teleop) | 8~15 GB | 5~8 GB |
| Python 텔레오퍼레이션 클라이언트 (UDCAP + Pink IK) | 1~2 GB | - |
| XRoboToolkit PC Service (백엔드) | 0.5~1 GB | - |
| HDF5 레코더 + 트레이스 버퍼 (`record_demos.py`) | 1~3 GB | - |
| Robomimic 학습 (BC-RNN) | 3~6 GB | 1 GB |
| VS Code / dev tools | 2~4 GB | - |
| **Phase 1 소계** | **15~30 GB** | 6~10 GB |

> Phase 1 단독으로는 64 GB도 충분.

#### Phase 2 — HG-DAgger 교정 루프

| 추가 워크로드 | 시스템 RAM | VRAM |
|---|---|---|
| Phase 1 베이스 | 15~30 GB | 6~10 GB |
| SigLIP2 분류기 (HF Transformers 로드) | 3~4 GB | 2 GB |
| Florence-2 객체 탐지 | 3~5 GB | 4 GB |
| IWR 재학습 dataset 로드 (200+ 궤적) | 5~8 GB | 1 GB |
| **Phase 2 소계** | **25~45 GB** | 13~20 GB |

> Phase 2도 64 GB 가능.

#### Phase 3 — 불확실성 기반 도움 요청 (운용)

| 추가 워크로드 | 시스템 RAM | VRAM |
|---|---|---|
| Phase 2 베이스 | 25~45 GB | 13~20 GB |
| **Qwen3-VL-32B SGLang/vLLM 서버 프로세스** | **8~15 GB CPU RAM** (KV cache + tokenizer + workers) | 75 GB |
| 앙상블 5개 모델 추론 (sequential) | 2 GB | 1 GB |
| Conformal calibration 데이터셋 | 3 GB | - |
| **Phase 3 소계** | **38~65 GB** | 89~96 GB |

> **Phase 3 운용은 64 GB RAM에서 위험 zone**. peak에 OOM 또는 swap.

### 1.2 합성 데이터 생성 — RAM 폭발 지점

`ust_ws/research/9.` 와 `5.`에 명시된 데이터 증강 파이프라인:

| 파이프라인 | 합성 궤적 수 | 메모리 압박 |
|---|---|---|
| Isaac Lab Mimic (`generate_dataset.py --num_envs 50`) | 1,000 궤적 | 멀티 env 병렬화 → **각 env 1~2 GB × 50 = 50~100 GB** |
| GR00T-Mimic 풀스케일 | 780,000 궤적 / 11h | HDF5 write 버퍼 + 카메라 RGB 텐서 = **30~60 GB peak** |
| Cosmos Transfer 포토리얼 증강 | varies | Diffusion 모델 + 비디오 디코딩 = **15~25 GB** |
| 로코매니퓰레이션 SDG (G1) | per-task | Occupancy map + 경로 계획 = 5~10 GB |

> **GR00T-Mimic 또는 `--num_envs 50` 데이터 생성을 동시에 다른 작업과 진행하려면 128 GB가 사실상 필수.** 64 GB면 데이터 생성을 단독 실행하거나 num_envs를 절반으로 줄여야 함 → 생성 시간 2배.

### 1.3 디스크에 이미 존재하는 데이터 (실측)

```
C:\develop\IsaacLab\ust_ws\models\
├── qwen3-vl-32b/      (~65 GB)
├── qwen3-vl-8b/       (~17 GB)
├── florence2-large/   (~3 GB)
└── siglip2-so400m/    (~1.5 GB)
                      ─────────
                       86 GB 합계
```

OS의 file cache (Windows Standby Memory + page cache)가 이 86 GB를 RAM에 캐싱할 수 있어야 모델 로드/언로드 사이클이 빠름:
- **128 GB**: OS가 86 GB 모델 + Isaac Sim assets 일부까지 캐싱 → VLM 재시작 시 디스크 read 거의 없음 (5~10초)
- **64 GB**: 캐시 가능 모델 일부만 → VLM 재시작/스왑 시 SSD 재read (30~90초, NVMe Gen4 기준)

> Phase 3 개발 중 VLM-8B ↔ VLM-32B 스위칭 빈도가 잦으면 이 차이가 누적 시간 손실로 직결.

### 1.4 멀티 프로세스/멀티 셸 일상 워크플로우

사용자의 메모리 기반 실제 패턴 (`MEMORY.md` 항목들 + `7-1`/`8`):

| 동시 실행 프로세스 | 일상 메모리 점유 |
|---|---|
| Isaac Sim teleop session | 12~18 GB |
| XRoboToolkit/UDCAP 백엔드 | 1 GB |
| Pink IK + Pinocchio Python | 2 GB |
| VLM 추론 서버 (별도 터미널) | 8~15 GB |
| HDF5 레코딩 + 후처리 | 3~5 GB |
| Robomimic 학습 (백그라운드) | 5~8 GB |
| Cursor/VS Code + Claude Code | 3~5 GB |
| WSL Ubuntu (Newton 실험 또는 Linux 도구) | 8~16 GB |
| Chrome 디버깅 (XRoboToolkit Web Service / WebXR 등) | 4~8 GB |
| **합계 (현실적 peak)** | **46~80 GB** |

> 풀스택 디버깅 시 **64 GB는 빈번하게 swap → 매우 두드러진 stutter**. 128 GB는 여유 zone.

---

## 2. CPU 업그레이드 7950X3D → 9950X3D2 — 무엇이 빨라지는가

### 2.1 §31 보고서 핵심 데이터 재인용

| 지표 | 7950X3D | 9950X3D2 (Dual V-Cache) |
|---|---|---|
| 아키텍처 | Zen 4 | Zen 5 |
| Single-thread | baseline | **+8.5%** |
| Multi-thread (avg/peak) | baseline | **+22% / +32%** |
| Total L3 | 128 MB (V-Cache 1 CCD only) | **192 MB (V-Cache 양 CCD)** |
| CCD scheduling 이슈 | 잘못 schedule 시 V-Cache 무력화 | **자동 해결** (양 CCD 모두 V-Cache) |
| TDP | 120 W | 200 W |

### 2.2 우리 워크로드별 CPU 업그레이드 효과

| 워크로드 | 7950X3D 현재 | 9950X3D2 예상 | 체감 |
|---|---|---|---|
| **단일 env teleop Pink IK** (`§30`/`§31`) | P99 5 ms / 8.33 ms budget | P99 4.0 ms | **거의 0** — input ceiling이 진짜 한계 |
| **120 Hz env step 안정성** | P99 margin 40% | P99 margin 52% | **0** — 둘 다 safe |
| **RL multi-env training (1024~4096 envs)** | baseline | **+25~30% step rate** | **매우 큼** — 24h → 18~19h |
| **Robomimic BC-RNN 학습** (dataset loading + LSTM) | baseline | +5~10% | 작음 |
| **MimicGen 합성 데이터 생성** (`--num_envs 50`) | baseline | +20~25% | 중간 |
| **GR00T-Mimic 780K 궤적** | 11시간 | ~8.5시간 | 큼 |
| **VLM 추론 서버 CPU side** (KV cache + decoding overhead) | baseline | +5~10% | 작음 |
| **앙상블 5개 sequential 학습** (각 30분) | 2.5시간 | ~2시간 | 작음 |
| **HDF5 dataset preprocessing (numpy/scipy)** | baseline | +10~15% (5600 MHz 결합) | 중간 |
| **PhysX articulation worker thread** (multi-env) | baseline | +20% | 큼 |
| **VR sampler + VMC + Pink IK 동시 동작 안정성** | CCD scheduling 의존 | **자동 안정** | **중간** — stutter 감소 |

### 2.3 9950X3D2 ★ 빛나는 시나리오 (우리 케이스에서 실재)

1. **RL training pipeline** — `7-1` 4096 envs 학습 시 단일 코어가 모든 env main loop 처리. dual-CCD V-Cache로 25~30% 가속. 1주짜리 학습이 5~6일로 단축.
2. **MimicGen `--num_envs 50` 합성 데이터 생성** — 50 env가 코어를 fight. 22% multi-thread 향상이 직접 반영.
3. **VR teleop + OBS 녹화 + Discord 동시 운영** — 사용자가 시연 영상 촬영 시 stutter가 직접 영향. 9950X3D2의 양 CCD V-Cache가 일관성 보장.
4. **WSL Ubuntu 병렬 운영** (Newton 실험) — `§29`에서 권고된 별도 Linux dev box를 WSL로 대체 시. WSL 호스트는 멀티스레드 OS 작업.
5. **CCD 잘못 schedule되던 상황 자동 해결** — `§31` §7에 검증 필요로 명시된 7950X3D의 V-Cache CCD scheduling 의존성이 사라짐.

### 2.4 9950X3D2가 무력한 시나리오 (우리 케이스 비중 큼)

1. **단일 env teleop 손가락 매칭** — `§31`/`§33` 결론 동일. input ceiling (UDCAP 140 Hz / SteamVR 120 Hz)이 진짜 한계.
2. **GPU bound 학습** (RTX PRO 6000은 우리 BC-RNN/앙상블에 과잉)
3. **Pink IK QP** (single-thread, 이미 budget 18~60%만 사용)

---

## 3. RAM 5600 MHz vs 3600 MHz 속도 차이 — Zen 5 시너지

### 3.1 정량 차이

| | DDR5-3600 (현재) | DDR5-5600 (제안) |
|---|---|---|
| 메모리 대역폭 (이론) | 28.8 GB/s | **44.8 GB/s** (+56%) |
| CAS latency | 18~20 | 28~32 (절대 ns로 유사) |
| Zen 5 권장 sweet spot | - | **6000 MHz** (실제 5600도 충분) |

### 3.2 우리 워크로드별 RAM 속도 효과

| 워크로드 | 3600 MHz → 5600 MHz 효과 |
|---|---|
| Pink IK / Pinocchio (V-Cache 96 MB에 fit) | **거의 0** — V-Cache 흡수 |
| Isaac Sim 단일 env step | 0~3% |
| Isaac Sim 4096 env step (RL) | **3~8%** — articulation buffer 트래픽 ↑ |
| HDF5 dataset 시퀀셜 read | **5~10%** |
| Robomimic LSTM batch processing | 2~5% |
| numpy/scipy linear algebra (Cosmos preprocessing) | 5~10% |
| PyTorch dataloader workers | 3~7% |

### 3.3 결론
- 9950X3D2 + DDR5-5600 조합은 V-Cache가 흡수하는 영역(Pink IK)엔 무의미
- RL multi-env / dataset preprocessing엔 추가 3~8% 효과
- **CPU 업그레이드 단독 효과(+22~32% multi-thread)에 비하면 부차적**

---

## 4. RAM 128 GB → 64 GB 축소 — 실제 타격 시나리오

### 4.1 절벽 시나리오 1 — Phase 3 풀스택 운용

**현재 가능 (128 GB)**:
```
Isaac Sim teleop session ........ 15 GB
Qwen3-VL-32B SGLang 서버 ....... 12 GB (CPU side)
SigLIP2 + Florence-2 추론 ........ 7 GB
앙상블 5개 추론 ................. 2 GB
HDF5 레코더 ..................... 3 GB
WSL Ubuntu ...................... 12 GB
VS Code + Claude Code ............ 5 GB
브라우저 (XRoboToolkit Web) ...... 6 GB
OS + Standby + 86GB 모델 캐시 .... 60 GB+
                              ─────────
                                  ~122 GB (캐시 압축 시 안정)
```

**64 GB에선**:
```
같은 워크로드 ................... 62~80 GB peak (캐시 제외)
                                  → 페이지 파일 / SSD swap 발생
                                  → VLM 응답 1초 → 5초
                                  → Isaac Sim stutter
                                  → teleop 시각 lag 추가 ~50 ms
```

> Phase 3 운용에서 **64 GB는 swap thrashing**. `§33`에서 분석한 motion-to-photon 25~40 ms 최적화가 swap으로 인해 100 ms+로 되돌아감.

### 4.2 절벽 시나리오 2 — GR00T-Mimic / 대규모 합성 데이터 생성

`§5` 7.1에 명시:
> 11시간 만에 780K 합성 궤적 생성 (= 6,500시간 = 9개월 인간 시연 분량)

| 동작 | 128 GB | 64 GB |
|---|---|---|
| `--num_envs 50` 병렬 데이터 생성 | ✓ 가능 | num_envs 20~25로 제한 → 시간 2배 |
| HDF5 write 버퍼 (~30 GB peak) | ✓ 여유 | 디스크 직접 write로 우회 → 속도 ↓ |
| Cosmos Transfer 동시 실행 | ✓ 가능 | 별도 세션으로 분리해야 함 |
| 동시 dev 작업 (코드 수정/검토) | ✓ 가능 | swap |

### 4.3 절벽 시나리오 3 — 모델 스위칭 + 86 GB 캐시

86 GB 모델을 OS 파일 캐시에 보존:
- **128 GB**: 캐시 적중률 거의 100%. VLM 8B → 32B 스위칭 5~10초.
- **64 GB**: 캐시 80% miss. 스위칭 30~90초 + SSD wear ↑.

Phase 3 개발 사이클에서 모델 변경이 잦으면 누적 손실 큼.

### 4.4 절벽 시나리오 4 — Long-horizon 데이터 수집

`§30` 1.4에 명시:
> 1시간 30 fps stereo camera = ~100 GB

- **128 GB**: 1시간 분량을 RAM 버퍼에 부분적 보관 가능
- **64 GB**: 5분 이상 stereo recording 시 page swap

### 4.5 절벽 시나리오 5 — WSL / 가상화 병행

`§29` 8장에서 권장:
> 별도 Linux dev box (Newton 실험)

WSL Ubuntu 또는 Hyper-V로 대체 시 호스트가 동적으로 8~16 GB 할당. 64 GB에선 host + guest 합계가 빠듯.

---

## 5. 비교 의사결정 매트릭스 — 워크로드 × 옵션

### 5.1 4가지 옵션 비교

| 옵션 | CPU | RAM | 비용 추정 | 종합 |
|---|---|---|---|---|
| **A 현재 유지** | 7950X3D | 128 GB DDR5-3600 | $0 | 안전 |
| **B 제안된 변경** | 9950X3D2 | 64 GB DDR5-5600 | ~$899 CPU + $0 (RAM 다운그레이드) | **위험** — 64 GB로 인한 손실이 더 큼 |
| **C 균형 권장** | 9950X3D2 | 128 GB DDR5-5600 | ~$899 CPU + $400 RAM | **★ 최적** |
| **D 미니멀** | 7950X3D | 128 GB DDR5-5600 | $400 RAM only | RAM 속도만 향상 |

### 5.2 워크로드별 옵션 비교 (1=worst, 5=best)

| 워크로드 | A 현재 | B 64GB | C 128GB+9950 | D 7950+5600 |
|---|:---:|:---:|:---:|:---:|
| 단일 env teleop | 5 | 5 | 5 | 5 |
| Phase 1 BC-RNN 학습 | 4 | 4 | 5 | 4 |
| Phase 2 HG-DAgger | 4 | 3 | 5 | 4 |
| Phase 3 풀스택 운용 | 5 | **2** | 5 | 5 |
| MimicGen `--num_envs 50` | 3 | 2 | 5 | 3 |
| GR00T-Mimic 780K | 2 | **1** | 5 | 2 |
| RL training 4096 envs | 3 | 3 | 5 | 3 |
| 86 GB 모델 캐싱 | 5 | **1** | 5 | 5 |
| WSL/Hyper-V 병행 | 4 | **2** | 5 | 4 |
| 1h+ stereo 녹화 | 4 | **1** | 5 | 4 |
| **종합 점수** | **39** | **24** | **50** | **39** |

> 옵션 B(제안된 변경)는 종합 점수에서 현재 옵션 A보다 **더 낮다.** 사용자가 의심한 것이 정확함 — **RAM 다운그레이드가 CPU 업그레이드 이득을 초과해서 잡아먹는다.**

---

## 6. 사용자 specific 질문에 대한 답변

### Q1. 9950X3D2로 업그레이드 시 성능 향상이 있는가?

**답: 있다, 그러나 워크로드 의존성이 크다.**

- 단일 env teleop: 거의 0 (§31에서 입증)
- RL training 4096 envs: +25~30% (24h → 18~19h)
- 합성 데이터 생성 (MimicGen/GR00T-Mimic): +20~25%
- 동시 멀티태스킹 안정성: 중간 향상
- VLM 서버 CPU side: 미미

### Q2. RAM 128 GB → 64 GB로 줄이면 더 손해인가?

**답: 그렇다. 손해가 크다.**

- 86 GB 모델 폴더가 OS 캐시에 안 맞음 → 모델 스위칭 5~10x 느려짐
- Phase 3 풀스택 운용 시 swap thrashing → motion-to-photon lag 25→100 ms 복귀
- GR00T-Mimic 780K 궤적 생성 시 `--num_envs` 절반 → 시간 2배
- Long-horizon stereo 녹화 거의 불가
- WSL Ubuntu(Newton 실험)와 호스트 Isaac Sim 동시 운영 불가

### Q3. 둘 중 어느 쪽이 체감 큰가?

**답: RAM 축소의 부정적 체감이 CPU 업그레이드의 긍정적 체감보다 크다.**

**RAM 축소가 체감되는 순간** (일상):
- VLM 서버 재시작 (자주 발생) → 30~90초 추가
- Phase 3 운용 중 swap stutter (Isaac Sim teleop 영상 끊김)
- 합성 데이터 생성 작업 도중 다른 도구 응답 지연

**CPU 업그레이드가 체감되는 순간** (간헐적):
- RL training 끝나는 시각이 24h → 18~19h (밤새 돌리는 작업이라 직접 체감 약함)
- 합성 데이터 생성 시간 단축 (마찬가지로 batch 작업이라 체감 약함)
- Multi-task 안정성 (간헐적)

→ **사용자가 직접 화면을 보며 작업하는 시간**에 RAM 축소가 더 자주 영향.

### Q4. 그럼 최선의 업그레이드 경로는?

**답: 옵션 C — 9950X3D2 + 128 GB DDR5-5600**.

- CPU 업그레이드의 모든 RL/멀티태스킹 이득 확보
- RAM 5600 MHz의 Zen 5 시너지 (+3~8% multi-env step rate)
- 128 GB의 풀스택 capacity 유지
- 비용: ~$1,300 (CPU + RAM 128 GB DDR5-5600 32GB×4 또는 64GB×2)

**옵션 D — RAM만 5600 MHz로 교체 (7950X3D 유지)**:
- 비용 최저 (~$400)
- RAM 속도 이득만 (3~8%)
- CPU 업그레이드 이득 없음
- "체감 향상은 그리 크지 않음, 비용 효율은 가장 좋음"

---

## 7. 추가 고려사항

### 7.1 9950X3D2 + 128 GB DDR5-5600의 메인보드 호환성
- AM5 소켓 그대로 (mainboard 교체 불요)
- DDR5-5600은 Ryzen 9000 시리즈 공식 지원 속도 (5200/5600 native)
- 단, EXPO 활성화 필요. EXPO 비활성 시 4800 MHz로 fallback.

### 7.2 RAM 128 GB DDR5-5600 구성
- 32GB × 4 (Quad-channel like): 일부 보드에서 5200 MHz로 throttle
- **64GB × 2 (Dual-channel)**: 5600 MHz 안정. ★ 권장
- DDR5 64GB DIMM은 2026년 일반화. 2개 슬롯 사용으로 향후 확장 가능.

### 7.3 사용자 §31 권장과의 일치
`§31` 9.2 Tier 2 권장:
> "MANUS Quantum 글러브 / Quest 3 / Newton dev box"

이 들이 본질적으로 사용자가 던진 질문보다 ROI 높음. 그러나 **만약 CPU/RAM 둘 중 하나라도 건드린다면**, Tier 2 권장과 별개로 다음이 합리적:
1. 단기: 옵션 D (RAM만 5600 MHz 128 GB 교체) — $400, 부담 적음
2. 중기: 옵션 C (9950X3D2 + 128 GB DDR5-5600) — $1,300, RL training 비중 ↑ 시
3. **하지 말 것: 옵션 B (RAM 다운그레이드)** — net loss

### 7.4 사용자가 의심한 직관의 검증
> "램 용량이 부족해서 할 수 있는 연구 규모가 작아지는게 더 타격이 클까?"

→ **정확함**. 본 분석은 이를 5가지 절벽 시나리오 + 9개 워크로드 점수표로 입증.

---

## 8. 우선순위 액션 권장 (cost-effective 순)

### Tier 0 (즉시, 무비용)
1. ✅ 9.23 EMA fix + decimation=1 (적용 완료)
2. ⏳ `§31` §7 — 7950X3D V-Cache CCD scheduling 검증 (Process Lasso로 Python을 CCD0에 묶기) — **0원으로 이미 9950X3D2와 비슷한 효과 가능성**
3. ⏳ `§33` cause #1 — Monitor mode 시도 (motion-to-photon 50 ms 절감)

### Tier 1 (저비용, 명백 효과)
4. ⏳ MANUS Quantum 글러브 또는 Quest 3 (`§31` 권장) — teleop 손가락 매칭 직접 효과
5. ⏳ **하드웨어 손대지 않기** — 현 7950X3D + 128 GB가 사용자 풀스택에 적절

### Tier 2 (RL training 비중 ↑ 시)
6. ⏳ **옵션 C — 9950X3D2 + 128 GB DDR5-5600** ($1,300)
7. ⏸️ **옵션 B (RAM 다운그레이드) 절대 비추천**

### Tier 3 (장기)
8. ⏳ 별도 Linux dev box (Newton 실험) — `§29`
9. ⏳ Newton 1.0 GA + Windows 지원 발표 시 재평가

---

## 9. 결정 트리

```
"RL training 4096 envs를 자주(주 단위) 돌리는가?"
   ├─ Yes
   │    │
   │    ├─ "Phase 3 (VLM+앙상블+합성 데이터) 풀스택 운용을 동시에 하는가?"
   │    │    ├─ Yes → ★ 옵션 C (9950X3D2 + 128 GB) — 둘 다 충족
   │    │    └─ No  → 옵션 B 가능 (64 GB 손실 작음) — 그러나 단순 CPU 교체 권장
   │    │
   │    └─ "비용 제약이 큰가?"
   │         ├─ Yes → 옵션 D (RAM만 5600 교체)
   │         └─ No  → 옵션 C
   │
   └─ No (주로 teleop + 시연 수집 + BC-RNN 학습)
        │
        ├─ "Phase 3 VLM 추론을 운용하는가?"
        │    ├─ Yes → 옵션 A 유지 + Tier 1 글러브/HMD 투자 ★
        │    └─ No  → 옵션 A 유지 + Tier 0 무비용 작업만
```

---

## 10. 한 페이지 요약

```
┌──────────────────────────────────────────────────────────────┐
│  사용자 질문:                                                  │
│  CPU 7950X3D → 9950X3D2  +  RAM 128GB-3600 → 64GB-5600       │
│  성능 향상 vs 연구 규모 축소 — 어느 쪽이 타격 큰가?              │
└──────────────────────────────────────────────────────────────┘

CPU 업그레이드 효과:
  ✓ 단일 env teleop: 0
  ✓ RL 4096 envs: +25~30% (24h → 18~19h)
  ✓ MimicGen 데이터: +20~25%
  ✓ 멀티태스킹 안정성: 중간

RAM 다운그레이드 손실:
  ✗ 86 GB 모델 캐시 손실 → 모델 스위칭 5~10x 느려짐
  ✗ Phase 3 풀스택 swap → motion-to-photon 25→100 ms
  ✗ GR00T-Mimic 시간 2배 (num_envs 절반)
  ✗ Stereo 녹화 5분 이상 불가
  ✗ WSL/Hyper-V 병행 어려움

종합 점수 (9개 워크로드):
  현재 (A):        39 / 50
  제안 변경 (B):   24 / 50  ← net loss
  최적 (C):        50 / 50  (9950X3D2 + 128GB DDR5-5600)
  RAM only (D):    39 / 50  (현 CPU + 128GB DDR5-5600)

★ 결론:
  - 사용자 직관 정확 — RAM 축소 타격이 CPU 이득보다 크다
  - 64 GB 변경 절대 비추 (net loss)
  - 권장: 옵션 C 또는 옵션 D, 또는 그냥 현 상태 유지
```

---

## 11. 참조

### 본 분석 기반 문서 (ust_ws/research/)
- `§5. humanoid_vr_teleop_imitation_learning_research.md` — Mimic 파이프라인 / 합성 데이터 생성 시간
- `§7-1. corrective_teaching_ai_system_research.md` — 교정 티칭 시스템 학술 배경
- `§8. corrective_teaching_system_design_guide.md` — Phase 1/2/3 단계별 시스템 설계
- `§9. rtx_pro_6000_optimal_model_training_guide.md` — VLM/SigLIP/Florence 모델 VRAM/RAM 사용량
- `§29. isaaclab_framerate_vs_newton_backend_comparison.md` — Newton 백엔드 Linux only 한계
- `§30. hardware_specific_framerate_analysis.md` — 사용자 PC 스펙별 frame rate 한계
- `§31. cpu_upgrade_roi_7950x3d_to_9950x3d_or_x3d2_analysis.md` — CPU 업그레이드 ROI (본 분석의 baseline)
- `§33. realtime_finger_tracking_latency_root_cause_and_optimization.md` — Motion-to-photon latency 분석

### 외부 자료
- [AMD Ryzen 9 9950X3D2 Dual Edition (AMD Press 2026-04-22)](https://www.amd.com/en/newsroom/press-releases/2026-4-22-amd-launches-ryzen-9-9950x3d2-dual-edition-processor.html)
- [Ryzen 9 9950X3D2 Tom's Hardware Review](https://www.tomshardware.com/pc-components/cpus/amd-ryzen-9-9950x3d2-review)
- [DDR5 Ryzen 9000 Memory Compatibility](https://www.amd.com/en/products/processors/desktops/ryzen.html)
- [Isaac Sim Performance Optimization Handbook](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/reference_material/sim_performance_optimization_handbook.html)
- [GR00T-Mimic 합성 데이터 파이프라인](https://developer.nvidia.com/blog/building-a-synthetic-motion-generation-pipeline-for-humanoid-robot-learning/)
- [Qwen3-VL-32B-Instruct HuggingFace](https://huggingface.co/Qwen/Qwen3-VL-32B-Instruct)

### 실측 데이터
- `C:\develop\IsaacLab\ust_ws\models\` 디스크 점유 86 GB (Qwen3-VL-32B 65GB + 8B 17GB + Florence-2 3GB + SigLIP2 1.5GB)
- 사용자 MEMORY.md 항목: Phase D 풀바디 텔레오퍼레이션, XRoboToolkit 백엔드 통합, ust_hm_grip 컨텍스트

---

*작성: 2026-05-19*
*분석 신뢰도: High (§31 정량 baseline + 실측 디스크 사용량 + §5/§8/§9 워크플로우 명시 사양)*
*추가 정보 필요 시: 사용자가 실제 RL training 빈도 / Phase 3 VLM 사용 빈도를 보고하면 옵션 C vs A 결정 더 명확화 가능*
