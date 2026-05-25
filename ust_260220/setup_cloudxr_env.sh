#!/bin/bash
# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
#
# CloudXR 6.0.1 환경변수 설정 스크립트 (G1 Kitchen Sorting)
#
# 사용법: source ust_ws/ust_260220/setup_cloudxr_env.sh
#
# CloudXR Runtime Docker 컨테이너가 ./ust_ws/openxr 폴더에 마운트되어
# 이미 실행 중이어야 합니다.
#
# ★ 주의: 반드시 source 로 실행해야 환경변수가 현재 셸에 적용됩니다.

# ============================================================
# 경로 설정
# ============================================================

ISAACLAB_ROOT="/workspace/isaaclab"
OPENXR_DIR="${ISAACLAB_ROOT}/ust_ws/openxr"

# ============================================================
# 사전 검증
# ============================================================

if [ ! -d "${OPENXR_DIR}" ]; then
    echo "[ERROR] openxr 디렉토리가 존재하지 않습니다: ${OPENXR_DIR}"
    echo "        CloudXR Runtime Docker를 먼저 실행하세요."
    return 1 2>/dev/null || exit 1
fi

if [ ! -S "${OPENXR_DIR}/run/ipc_cloudxr" ]; then
    echo "[WARNING] CloudXR IPC 소켓이 존재하지 않습니다."
    echo "          CloudXR Runtime Docker가 실행 중인지 확인하세요."
fi

if [ ! -f "${OPENXR_DIR}/share/openxr/1/openxr_cloudxr.json" ]; then
    echo "[WARNING] OpenXR 런타임 매니페스트가 존재하지 않습니다."
fi

# ============================================================
# 환경변수 설정
# ============================================================

# 핵심 환경변수 1: IPC 소켓 디렉토리
export XDG_RUNTIME_DIR="${OPENXR_DIR}/run"

# 핵심 환경변수 2: OpenXR 런타임 매니페스트
export XR_RUNTIME_JSON="${OPENXR_DIR}/share/openxr/1/openxr_cloudxr.json"

# 핵심 환경변수 3: IPC 버전 검사 무시 (Kit 5.0.1 ↔ Runtime 6.0.1)
export IPC_IGNORE_VERSION=1

# RTX PRO 6000 GPU 설정
export NV_GPU_INDEX=0
export NV_PACER_FIXED_TIME_STEP_MS=11   # ~90Hz VR 렌더링
export CUDA_VISIBLE_DEVICES=0

# Isaac Lab 경로
export ISAACLAB_PATH="${ISAACLAB_ROOT}"

# Kitchen Sorting 프로젝트 PYTHONPATH
export PYTHONPATH="${PYTHONPATH}:${ISAACLAB_ROOT}/ust_ws"

# ============================================================
# 설정 확인
# ============================================================

echo ""
echo "============================================"
echo "  G1 Kitchen Sorting - CloudXR 환경 설정 완료"
echo "============================================"
echo ""
echo "  XDG_RUNTIME_DIR  = ${XDG_RUNTIME_DIR}"
echo "  XR_RUNTIME_JSON  = ${XR_RUNTIME_JSON}"
echo "  IPC_IGNORE_VERSION = ${IPC_IGNORE_VERSION}"
echo "  NV_GPU_INDEX     = ${NV_GPU_INDEX}"
echo "  GPU              = RTX PRO 6000"
echo ""

if [ -S "${XDG_RUNTIME_DIR}/ipc_cloudxr" ]; then
    echo "  CloudXR IPC 소켓: ✓ 활성"
else
    echo "  CloudXR IPC 소켓: ✗ 비활성"
fi

if [ -f "${XDG_RUNTIME_DIR}/runtime_started" ]; then
    echo "  CloudXR Runtime:  ✓ 시작됨"
else
    echo "  CloudXR Runtime:  ✗ 미시작"
fi

echo ""
echo "============================================"
echo ""
echo "[다음 단계]"
echo "  1. CloudXR Runtime 확인: docker ps | grep cloudxr"
echo "  2. VR 텔레오퍼레이션:"
echo "     python ust_ws/ust_260220/scripts/run_teleop.py --teleop_device handtracking"
echo "  3. 기본 환경 테스트:"
echo "     python ust_ws/ust_260220/scripts/run_env.py --num_envs 1"
echo "  4. 카메라 포함 테스트:"
echo "     python ust_ws/ust_260220/scripts/run_env.py --enable_cameras"
echo ""
