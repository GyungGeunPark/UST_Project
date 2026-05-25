#!/bin/bash
# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
#
# UST CloudXR 환경변수 설정 스크립트
#
# 사용법: source ./setup_cloudxr_env.sh
#
# 이 스크립트는 Isaac Lab이 CloudXR Runtime에 연결하기 위해 필요한
# 환경변수를 설정합니다. CloudXR Runtime Docker 컨테이너가
# ./ust_ws/openxr 폴더에 마운트되어 이미 실행 중이어야 합니다.
#
# ★ 주의: 반드시 source 로 실행해야 환경변수가 현재 셸에 적용됩니다.
#    source ./setup_cloudxr_env.sh  (O)
#    ./setup_cloudxr_env.sh         (X - 서브셸에만 적용됨)

# ============================================================
# 경로 설정
# ============================================================

# Isaac Lab 루트 디렉토리
ISAACLAB_ROOT="/workspace/isaaclab"

# CloudXR Runtime이 생성한 openxr 공유 폴더 경로
# CloudXR Docker를 실행할 때 --mount type=bind,src=./ust_ws/openxr,dst=/openxr 로 마운트한 폴더
OPENXR_DIR="${ISAACLAB_ROOT}/ust_ws/openxr"

# ============================================================
# 사전 검증
# ============================================================

# openxr 디렉토리 존재 확인
if [ ! -d "${OPENXR_DIR}" ]; then
    echo "[ERROR] openxr 디렉토리가 존재하지 않습니다: ${OPENXR_DIR}"
    echo "        CloudXR Runtime Docker를 먼저 실행하세요."
    return 1 2>/dev/null || exit 1
fi

# IPC 소켓 파일 확인
if [ ! -S "${OPENXR_DIR}/run/ipc_cloudxr" ]; then
    echo "[WARNING] CloudXR IPC 소켓이 존재하지 않습니다: ${OPENXR_DIR}/run/ipc_cloudxr"
    echo "          CloudXR Runtime Docker가 실행 중인지 확인하세요."
    echo "          (소켓 없이도 환경변수는 설정합니다)"
fi

# OpenXR JSON 매니페스트 확인
if [ ! -f "${OPENXR_DIR}/share/openxr/1/openxr_cloudxr.json" ]; then
    echo "[WARNING] OpenXR 런타임 매니페스트가 존재하지 않습니다."
    echo "          CloudXR Runtime Docker가 정상 실행되었는지 확인하세요."
fi

# ============================================================
# 환경변수 설정
# ============================================================

# ★ 핵심 환경변수 1: IPC 소켓 디렉토리
# Isaac Lab의 OpenXR 로더가 CloudXR Runtime의 IPC 소켓을 찾는 경로
export XDG_RUNTIME_DIR="${OPENXR_DIR}/run"

# ★ 핵심 환경변수 2: OpenXR 런타임 매니페스트 JSON 파일
# Isaac Lab이 어떤 OpenXR 런타임(CloudXR)을 로드할지 결정하는 설정 파일
export XR_RUNTIME_JSON="${OPENXR_DIR}/share/openxr/1/openxr_cloudxr.json"

# ★ 핵심 환경변수 3: IPC 버전 검사 무시
# Kit 번들 OpenXR 클라이언트(5.0.1)와 CloudXR Runtime(6.0.1) 버전 불일치 방지
# 라이브러리 교체 후에도 안전장치로 유지
export IPC_IGNORE_VERSION=1

# 선택적 CloudXR 최적화 환경변수
export NV_GPU_INDEX=0                     # 사용할 GPU 인덱스 (멀티 GPU 시스템)
export NV_PACER_FIXED_TIME_STEP_MS=11    # 고정 타임스텝 (~90Hz VR 렌더링)

# CUDA 디바이스 (단일 GPU)
export CUDA_VISIBLE_DEVICES=0

# Isaac Lab 경로
export ISAACLAB_PATH="${ISAACLAB_ROOT}"

# UST 프로젝트 PYTHONPATH 추가
export PYTHONPATH="${PYTHONPATH}:${ISAACLAB_ROOT}/ust_ws/260207_ust"

# ============================================================
# 설정 확인 출력
# ============================================================

echo ""
echo "============================================"
echo "  UST CloudXR 환경변수 설정 완료"
echo "============================================"
echo ""
echo "  XDG_RUNTIME_DIR  = ${XDG_RUNTIME_DIR}"
echo "  XR_RUNTIME_JSON  = ${XR_RUNTIME_JSON}"
echo "  NV_GPU_INDEX     = ${NV_GPU_INDEX}"
echo "  ISAACLAB_PATH    = ${ISAACLAB_PATH}"
echo ""

# IPC 소켓 상태 확인
if [ -S "${XDG_RUNTIME_DIR}/ipc_cloudxr" ]; then
    echo "  CloudXR IPC 소켓: ✓ 활성 (CloudXR Runtime 실행 중)"
else
    echo "  CloudXR IPC 소켓: ✗ 비활성 (CloudXR Runtime을 시작하세요)"
fi

# runtime_started 플래그 확인
if [ -f "${XDG_RUNTIME_DIR}/runtime_started" ]; then
    echo "  CloudXR Runtime:  ✓ 시작됨"
else
    echo "  CloudXR Runtime:  ✗ 미시작"
fi

echo ""
echo "============================================"
echo ""
echo "[다음 단계]"
echo "  1. CloudXR Runtime Docker 확인: docker ps | grep cloudxr"
echo "  2. 텔레오퍼레이션 실행:"
echo "     ./isaaclab.sh -p ust_ws/260207_ust/scripts/run_teleop.py --teleop_device handtracking"
echo "  3. 키보드 테스트 (VR 없이):"
echo "     ./isaaclab.sh -p ust_ws/260207_ust/scripts/run_teleop.py --teleop_device keyboard"
echo ""
