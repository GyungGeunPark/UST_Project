#!/bin/bash
# ============================================================================
# 방안1: PICO Connect (PCVR) + XRoboToolkit 텔레오퍼레이션 환경 설정
#
# PICO Connect = PICO 내장 PCVR 스트리밍 (시스템 서비스)
# XRoboToolkit = 전신/손 트래킹 (포그라운드 앱)
#
# 아키텍처:
#   [PICO 헤드셋]
#     ├── PICO Connect (시스템 서비스) ←→ SteamVR (PC) → VR 렌더링
#     └── XRoboToolkit App → gRPC → RoboticsServiceProcess (PC) → 트래킹 데이터
#   [Windows 미니PC]
#     └── UDCAP → VMC/OSC UDP:39539 → 통합 브릿지 (PC) → 손가락 데이터
#
# 사용법:
#   source ust_ws/ust_260220/setup_pico_connect_env.sh
#
# 전제 조건:
#   1. SteamVR 설치 (steam + steamvr)
#   2. PICO Connect PC 드라이버 설치
#   3. PICO 4 Ultra에서 PICO Connect 활성화
#   4. XRoboToolkit PC Service 빌드 완료
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
XRT_DIR="$HOME/ust_ws/XRoboToolkit-PC-Service/RoboticsService"

echo "===== PICO Connect + XRoboToolkit Teleop Setup ====="
echo "  워크스페이스: $WORKSPACE_DIR"
echo ""

# ─── SteamVR OpenXR 런타임 설정 ───
echo "[1/4] SteamVR OpenXR 런타임 설정..."

# SteamVR OpenXR 런타임 JSON 경로 탐색
STEAMVR_RUNTIME_PATHS=(
    "$HOME/.steam/steam/steamapps/common/SteamVR/steamxr_linux64.json"
    "$HOME/.local/share/Steam/steamapps/common/SteamVR/steamxr_linux64.json"
    "/usr/share/steamvr/steamxr_linux64.json"
)

STEAMVR_JSON=""
for path in "${STEAMVR_RUNTIME_PATHS[@]}"; do
    if [ -f "$path" ]; then
        STEAMVR_JSON="$path"
        break
    fi
done

if [ -z "$STEAMVR_JSON" ]; then
    echo "  [경고] SteamVR OpenXR 런타임을 찾을 수 없습니다."
    echo "  SteamVR를 먼저 설치하세요:"
    echo "    1. Steam 설치: sudo apt install steam"
    echo "    2. Steam에서 SteamVR 설치"
    echo "    3. SteamVR 한 번 실행 후 종료"
    echo ""
    echo "  수동 설정:"
    echo "    export XR_RUNTIME_JSON=/path/to/steamxr_linux64.json"
else
    export XR_RUNTIME_JSON="$STEAMVR_JSON"
    echo "  XR_RUNTIME_JSON=$XR_RUNTIME_JSON"
fi

# IPC 버전 무시 (혹시 CloudXR 런타임과 혼용 시)
export IPC_IGNORE_VERSION=1

# ─── XRoboToolkit PC Service 시작 ───
echo ""
echo "[2/4] XRoboToolkit PC Service 시작..."

if [ -f "$XRT_DIR/bin/RoboticsServiceProcess" ]; then
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
fi

# ─── 통합 브릿지 (ConsoleDemo subprocess 포함) ───
echo ""
echo "[3/4] 통합 텔레오퍼레이션 브릿지 시작..."

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
echo "    1. PICO Connect 활성화 (설정 → 일반 → PICO Connect)"
echo "       - PC 소프트웨어: PICO Connect (PC에서 설치)"
echo "       - WiFi 또는 USB-C로 연결"
echo "    2. XRoboToolkit App 실행 → PC Service IP: $HOST_IP"
echo "       - PICO Connect는 시스템 서비스로 백그라운드에서 동작"
echo "       - XRoboToolkit은 트래킹 데이터만 전송 (렌더링 없음)"
echo ""
echo "  Windows 미니PC (UDCAP):"
echo "    VMC 대상 IP: $HOST_IP, 포트: 39539"
echo ""
echo "  PC에서 SteamVR 시작 후:"
echo "    # Docker 내부에서 실행"
echo "    ./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py \\"
echo "        --teleop_device pico --render_mode pico_connect"
echo ""
echo "  프로세스 확인:"
echo "    pgrep -af 'RoboticsServiceProcess|unified_bridge|vrserver'"
echo ""
echo "  전체 종료:"
echo "    pkill -f 'RoboticsServiceProcess|unified_bridge'"
echo "================================="
