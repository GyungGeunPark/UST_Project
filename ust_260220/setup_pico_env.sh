#!/bin/bash
# ============================================================================
# PICO 4 Ultra + UDCAP 텔레오퍼레이션 환경 시작 스크립트
# 호스트(Ubuntu 24.04)에서 실행
#
# 사용법:
#   source ust_ws/ust_260220/setup_pico_env.sh
#
# 전제 조건:
#   1. XRoboToolkit PC Service 빌드 완료
#   2. Windows 미니PC UDCAP 셋업 완료
#   3. CloudXR Runtime 6.0.1 Docker 실행 중
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
XRT_DIR="$HOME/ust_ws/XRoboToolkit-PC-Service/RoboticsService"

echo "===== PICO + UDCAP Teleop Environment Setup ====="
echo "  워크스페이스: $WORKSPACE_DIR"
echo "  XRoboToolkit: $XRT_DIR"
echo ""

# ─── CloudXR 환경 변수 (기존, 변경 없음) ───
export XDG_RUNTIME_DIR="$WORKSPACE_DIR/ust_ws/openxr/run"
export XR_RUNTIME_JSON="$WORKSPACE_DIR/ust_ws/openxr/share/openxr/1/openxr_cloudxr.json"
export IPC_IGNORE_VERSION=1
export NV_PACER_FIXED_TIME_STEP_MS=11  # ~90Hz VR

echo "[1/5] CloudXR 환경변수 설정 완료"
echo "  XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
echo "  XR_RUNTIME_JSON=$XR_RUNTIME_JSON"
echo "  IPC_IGNORE_VERSION=$IPC_IGNORE_VERSION"

# ─── XRoboToolkit PC Service 시작 ───
echo ""
echo "[2/5] XRoboToolkit PC Service 시작..."

if [ -f "$XRT_DIR/bin/RoboticsServiceProcess" ]; then
    # 이미 실행 중인지 확인
    if pgrep -f "RoboticsServiceProcess" > /dev/null 2>&1; then
        echo "  이미 실행 중 (PID: $(pgrep -f RoboticsServiceProcess))"
    else
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$XRT_DIR/Redistributable/linux:$XRT_DIR/Redistributable/linux/grpc/lib"
        cd "$XRT_DIR"
        ./bin/RoboticsServiceProcess &
        XRT_PID=$!
        echo "  시작됨 (PID: $XRT_PID)"
        cd "$WORKSPACE_DIR"
    fi
else
    echo "  [경고] RoboticsServiceProcess 미발견: $XRT_DIR/bin/"
    echo "  qt-gcc.sh로 빌드가 필요합니다."
fi

# ─── 통합 브릿지 (ConsoleDemo subprocess 포함) ───
echo ""
echo "[3/5] 통합 텔레오퍼레이션 브릿지 시작..."
echo "  (ConsoleDemo는 브릿지 내부에서 자동 실행됩니다)"

BRIDGE_SCRIPT="$SCRIPT_DIR/teleop/unified_bridge.py"
if [ -f "$BRIDGE_SCRIPT" ]; then
    if pgrep -f "unified_bridge.py" > /dev/null 2>&1; then
        echo "  이미 실행 중 (PID: $(pgrep -f unified_bridge.py))"
    else
        python3 "$BRIDGE_SCRIPT" \
            --xrt_mode subprocess \
            --vmc_port 39539 \
            --output_port 8889 &
        echo "  시작됨 (PID: $!, port 8889)"
    fi
else
    echo "  [경고] unified_bridge.py 미발견: $BRIDGE_SCRIPT"
fi

# ─── 상태 출력 ───
echo ""
echo "[4/4] 환경 설정 완료"
echo ""
HOST_IP=$(hostname -I | awk '{print $1}')
echo "===== 연결 정보 ====="
echo "  호스트 IP: $HOST_IP"
echo ""
echo "  PICO 헤드셋에서:"
echo "    1. XRoboToolkit App → PC Service IP: $HOST_IP"
echo "    2. CloudXR.js 브라우저 → https://$HOST_IP:8080"
echo ""
echo "  Windows 미니PC (UDCAP):"
echo "    VMC 대상 IP: $HOST_IP, 포트: 39539"
echo ""
echo "  Docker Isaac Lab에서:"
echo "    # 방안3: 모니터 뷰 (XR 없음, 즉시 사용 가능)"
echo "    ./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py \\"
echo "        --teleop_device pico --render_mode monitor"
echo ""
echo "    # 방안1: PICO Connect (SteamVR 필요)"
echo "    ./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py \\"
echo "        --teleop_device pico --render_mode pico_connect"
echo ""
echo "  프로세스 확인:"
echo "    pgrep -af 'RoboticsServiceProcess|ConsoleDemo|unified_bridge'"
echo ""
echo "  전체 종료:"
echo "    pkill -f 'RoboticsServiceProcess|ConsoleDemo|unified_bridge'"
echo "================================="
