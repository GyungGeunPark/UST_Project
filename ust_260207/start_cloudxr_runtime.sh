#!/bin/bash
# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
#
# UST CloudXR Runtime Docker 시작 스크립트
#
# 이 스크립트는 CloudXR Runtime Docker 컨테이너를 시작합니다.
# 공유 폴더로 /workspace/isaaclab/ust_ws/openxr 를 사용합니다.
#
# 사용법: ./start_cloudxr_runtime.sh
#
# ★ 반드시 Isaac Lab 실행 전에 먼저 이 스크립트를 실행하세요.

set -e

# ============================================================
# 설정
# ============================================================

ISAACLAB_ROOT="/workspace/isaaclab"

# ★ CloudXR Runtime과 Isaac Lab이 공유하는 openxr 폴더
OPENXR_HOST_DIR="${ISAACLAB_ROOT}/ust_ws/openxr"

# CloudXR Runtime 이미지 버전
CLOUDXR_IMAGE="nvcr.io/nvidia/cloudxr-runtime-early-access:6.0.1-webrtc"

# 컨테이너 이름
CONTAINER_NAME="ust-cloudxr-runtime"

# ============================================================
# 사전 검증
# ============================================================

echo "============================================"
echo "  UST CloudXR Runtime 시작"
echo "============================================"
echo ""

# openxr 디렉토리 생성 (없으면)
if [ ! -d "${OPENXR_HOST_DIR}" ]; then
    echo "[INFO] openxr 디렉토리 생성: ${OPENXR_HOST_DIR}"
    mkdir -p "${OPENXR_HOST_DIR}"
fi

# 기존 컨테이너 정리
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[INFO] 기존 ${CONTAINER_NAME} 컨테이너 정리 중..."
    docker stop "${CONTAINER_NAME}" 2>/dev/null || true
    docker rm "${CONTAINER_NAME}" 2>/dev/null || true
fi

# GPU 확인
echo "[INFO] GPU 상태 확인..."
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null || {
    echo "[ERROR] nvidia-smi 실행 실패. GPU 드라이버를 확인하세요."
    exit 1
}

echo ""

# ============================================================
# CloudXR Runtime Docker 실행
# ============================================================

echo "[INFO] CloudXR Runtime Docker 시작..."
echo "  이미지:     ${CLOUDXR_IMAGE}"
echo "  openxr 경로: ${OPENXR_HOST_DIR}"
echo "  컨테이너명: ${CONTAINER_NAME}"
echo ""

docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network host \
    --gpus=all \
    -e "ACCEPT_EULA=Y" \
    --mount "type=bind,src=${OPENXR_HOST_DIR},dst=/openxr" \
    "${CLOUDXR_IMAGE}"

# 이 지점은 컨테이너가 종료된 후에 도달합니다
echo ""
echo "[INFO] CloudXR Runtime 종료됨."
