#!/usr/bin/env python3
"""통합 텔레오퍼레이션 데이터 브릿지.

XRoboToolkit (Body/Hand) + UDCAP (Finger) → TCP → Docker (Isaac Lab)
**호스트(Ubuntu 24.04)에서 실행**, Docker 내부가 아님.

사용법 (호스트에서):
    # 기본: ConsoleDemo를 subprocess로 직접 실행 (추천)
    python3 unified_bridge.py --vmc_port 39539 --output_port 8889

    # 레거시: 외부 TCP 서버 연결 모드
    python3 unified_bridge.py --xrt_mode tcp --xrt_port 7777

데이터 흐름:
  ConsoleDemo (subprocess stdout) ──┐
  또는 TCP :7777 (레거시)          ├─→ 통합 JSON ─→ TCP :8889 → Docker
  UDCAP VMC 수신 (UDP :39539) ────┘

전제 조건:
  - RoboticsServiceProcess 실행 중 (PICO와 gRPC 연결)
  - subprocess 모드: ConsoleDemo 바이너리 필요 (자동 탐색)
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import threading
import time
from typing import Dict, Optional, Tuple


class XRTReceiver:
    """XRoboToolkit 트래킹 데이터 수신기.

    두 가지 모드 지원:
      1. subprocess: ConsoleDemo를 직접 실행하고 stdout에서 읽기 (기본, 추천)
      2. tcp: 외부 TCP 서버에 연결 (레거시 호환용)
    """

    # ConsoleDemo stdout 접두사
    _DEVICE_DATA_PREFIX = "device data"

    def __init__(
        self,
        host: str = "localhost",
        port: int = 7777,
        mode: str = "subprocess",
        console_demo_path: Optional[str] = None,
    ):
        self.host = host
        self.port = port
        self.mode = mode
        self._latest: Optional[dict] = None
        self._lock = threading.Lock()
        self._running = False
        self._process: Optional[subprocess.Popen] = None
        self._recv_count = 0

        # ConsoleDemo 경로 자동 탐색
        if console_demo_path:
            self.console_demo_path = console_demo_path
        else:
            self.console_demo_path = self._find_console_demo()

    @staticmethod
    def _find_console_demo() -> str:
        """ConsoleDemo 바이너리 경로 탐색."""
        candidates = [
            os.path.expanduser(
                "~/ust_ws/XRoboToolkit-PC-Service/RoboticsService/Redistributable/linux/ConsoleDemo"
            ),
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "..", "XRoboToolkit-PC-Service", "RoboticsService",
                "Redistributable", "linux", "ConsoleDemo",
            ),
        ]
        for path in candidates:
            resolved = os.path.realpath(path)
            if os.path.isfile(resolved) and os.access(resolved, os.X_OK):
                return resolved
        return ""

    def start(self):
        self._running = True
        if self.mode == "subprocess":
            threading.Thread(target=self._subprocess_loop, daemon=True).start()
        else:
            threading.Thread(target=self._tcp_recv_loop, daemon=True).start()

    # ── subprocess 모드: ConsoleDemo stdout 직접 읽기 ──

    def _subprocess_loop(self):
        """ConsoleDemo를 subprocess로 실행하고 stdout 파싱."""
        if not self.console_demo_path:
            print("[XRT] ConsoleDemo 바이너리를 찾을 수 없습니다.")
            print("      --console_demo 옵션으로 경로를 지정하세요.")
            return

        # LD_LIBRARY_PATH 설정 (SDK 공유 라이브러리)
        lib_dir = os.path.dirname(self.console_demo_path)
        grpc_lib = os.path.join(lib_dir, "grpc", "lib")
        env = os.environ.copy()
        ld_paths = [lib_dir, grpc_lib]
        if env.get("LD_LIBRARY_PATH"):
            ld_paths.append(env["LD_LIBRARY_PATH"])
        env["LD_LIBRARY_PATH"] = ":".join(ld_paths)

        # CRITICAL: ConsoleDemo는 std::cout 사용 → stdout이 PIPE면 block-buffered (~4KB).
        # JSON 트래킹 데이터가 버퍼에 쌓여 큰 지연 발생.
        # `stdbuf -oL`로 강제 line-buffered 변환 (없으면 fallback).
        import shutil
        stdbuf = shutil.which("stdbuf")
        if stdbuf:
            cmd = [stdbuf, "-oL", "-eL", self.console_demo_path]
        else:
            print("[XRT] WARN: stdbuf 미설치 → ConsoleDemo stdout 버퍼링으로 지연 가능")
            cmd = [self.console_demo_path]

        while self._running:
            try:
                print(f"[XRT] ConsoleDemo 시작: {' '.join(cmd)}")
                print(f"[XRT]   cwd: {lib_dir}")
                print(f"[XRT]   LD_LIBRARY_PATH: {env.get('LD_LIBRARY_PATH', 'N/A')}")
                self._process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    cwd=lib_dir,
                    env=env,
                )
                self._read_stdout(self._process)

                # 프로세스 종료 후 stderr 및 종료 코드 출력
                retcode = self._process.wait(timeout=5)
                stderr_out = self._process.stderr.read().decode("utf-8", errors="replace").strip()
                if stderr_out:
                    print(f"[XRT] ConsoleDemo stderr:\n{stderr_out}")
                print(f"[XRT] ConsoleDemo 종료 코드: {retcode}")

            except FileNotFoundError:
                print(f"[XRT] ConsoleDemo 실행 실패: {self.console_demo_path}")
                if self._running:
                    time.sleep(5)
            except Exception as e:
                print(f"[XRT] ConsoleDemo 오류: {e}")
                if self._running:
                    time.sleep(2)
            finally:
                if self._process and self._process.poll() is None:
                    self._process.terminate()
                    try:
                        self._process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        self._process.kill()
                    self._process = None

            if self._running:
                print("[XRT] ConsoleDemo 종료됨, 5초 후 재시작...")
                time.sleep(5)

    def _read_stdout(self, proc: subprocess.Popen):
        """ConsoleDemo stdout에서 'device data{json}' 라인 파싱."""
        for raw_line in iter(proc.stdout.readline, b""):
            if not self._running:
                break
            line = raw_line.decode("utf-8", errors="replace").strip()

            # 'device data' 접두사 제거 후 JSON 파싱
            if line.startswith(self._DEVICE_DATA_PREFIX):
                json_str = line[len(self._DEVICE_DATA_PREFIX):].strip()
                if json_str:
                    try:
                        data = json.loads(json_str)
                        with self._lock:
                            self._latest = data
                            self._recv_count += 1
                        if self._recv_count == 1:
                            print("[XRT] ✓ 첫 번째 트래킹 데이터 수신!")
                            keys = self._extract_inner_keys(data)
                            print(f"[XRT]   inner keys: {keys}")
                            print(f"[XRT]   raw size: {len(json_str)} bytes (SDK buffer 16352)")
                        elif self._recv_count % 500 == 0:
                            keys = self._extract_inner_keys(data)
                            print(f"[XRT] #{self._recv_count} | keys={keys}")
                    except json.JSONDecodeError as e:
                        if self._recv_count == 0:
                            print(f"[XRT] JSON 파싱 실패 (첫 라인): {e}")
                            print(f"[XRT]   line[:200]: {json_str[:200]}")
            # 디버그: 첫 50 라인은 모두 출력 (start/connect 메시지 확인용)
            elif self._recv_count == 0 and line and not line.startswith(" main loop"):
                print(f"[XRT raw] {line[:200]}")

    @staticmethod
    def _extract_inner_keys(data: dict) -> list:
        """XRT JSON {"value":"<json>"} 에서 트래킹 키 추출."""
        if not isinstance(data, dict):
            return []
        value = data.get("value", data)
        if isinstance(value, str):
            try:
                value = json.loads(value.replace("\\", ""))
            except (json.JSONDecodeError, TypeError):
                return []
        if not isinstance(value, dict):
            return []
        return [k for k in ("Head", "Controller", "Hand", "Body", "Motion") if k in value]

    # ── TCP 모드: 외부 TCP 서버에 연결 (레거시) ──

    def _tcp_recv_loop(self):
        """TCP 서버에 연결하여 JSON 라인 수신 (레거시 모드)."""
        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self.host, self.port))
                sock.settimeout(None)
                print(f"[XRT] TCP 연결: {self.host}:{self.port}")
                buffer = ""
                while self._running:
                    chunk = sock.recv(65536).decode("utf-8")
                    if not chunk:
                        break
                    buffer += chunk
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        if line.strip():
                            try:
                                data = json.loads(line.strip())
                                with self._lock:
                                    self._latest = data
                            except json.JSONDecodeError:
                                pass
            except (ConnectionRefusedError, ConnectionResetError, OSError, socket.timeout):
                if self._running:
                    print("[XRT] TCP 연결 실패, 2초 후 재시도...")
                    time.sleep(2)

    def get_latest(self) -> Optional[dict]:
        with self._lock:
            return self._latest

    def stop(self):
        self._running = False
        if self._process and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()


class VMCReceiver:
    """UDCAP VMC/OSC UDP 수신기.

    python-osc 라이브러리를 사용하여 VMC 프로토콜 수신.
    """

    def __init__(self, listen_ip: str = "0.0.0.0", listen_port: int = 39539):
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self._left_bones: Dict[str, Tuple[float, float, float, float]] = {}
        self._right_bones: Dict[str, Tuple[float, float, float, float]] = {}
        self._lock = threading.Lock()
        self._running = False
        self._osc_available = False

    def start(self):
        self._running = True
        self._kill_previous_port_user()
        try:
            from pythonosc import dispatcher, osc_server

            disp = dispatcher.Dispatcher()
            disp.map("/VMC/Ext/Bone/Pos", self._on_bone_pos)
            disp.set_default_handler(self._on_any)

            self._server = osc_server.ThreadingOSCUDPServer(
                (self.listen_ip, self.listen_port), disp
            )
            self._server.socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
            )
            self._osc_available = True
            threading.Thread(target=self._server.serve_forever, daemon=True).start()
            print(f"[VMC] Listening on {self.listen_ip}:{self.listen_port}")
        except ImportError:
            print("[VMC] python-osc 미설치. UDCAP 핑거 데이터 수신 불가.")
            print("      pip install python-osc 로 설치하세요.")
            self._osc_available = False

    def _kill_previous_port_user(self):
        """이전 프로세스가 포트를 점유 중이면 종료."""
        import subprocess
        try:
            result = subprocess.run(
                ["lsof", "-t", "-i", f"UDP:{self.listen_port}"],
                capture_output=True, text=True,
            )
            for pid_str in result.stdout.strip().split("\n"):
                pid = int(pid_str)
                if pid != os.getpid():
                    print(f"[VMC] 포트 {self.listen_port} 점유 프로세스(PID {pid}) 종료")
                    os.kill(pid, signal.SIGTERM)
                    time.sleep(0.3)
        except (ValueError, ProcessLookupError, FileNotFoundError):
            pass

    def _on_bone_pos(self, address, *args):
        """VMC bone pose: /VMC/Ext/Bone/Pos name px py pz qx qy qz qw"""
        if len(args) < 8:
            return
        bone_name = str(args[0])
        qx, qy, qz, qw = float(args[4]), float(args[5]), float(args[6]), float(args[7])

        with self._lock:
            if "Left" in bone_name:
                self._left_bones[bone_name] = (qx, qy, qz, qw)
            elif "Right" in bone_name:
                self._right_bones[bone_name] = (qx, qy, qz, qw)

    def _on_any(self, address, *args):
        pass  # 디버그 시 print 추가

    def get_finger_data(self) -> dict:
        """UDCAP 핑거 데이터 반환."""
        with self._lock:
            left = {k: list(v) for k, v in self._left_bones.items()}
            right = {k: list(v) for k, v in self._right_bones.items()}
        return {"left": left, "right": right}

    def stop(self):
        self._running = False
        if self._osc_available:
            self._server.shutdown()


class UnifiedBridge:
    """XRT + UDCAP 통합 브릿지.

    90Hz로 통합 데이터를 TCP 클라이언트(Isaac Lab)에 전달.
    """

    def __init__(
        self,
        xrt_host: str = "localhost",
        xrt_port: int = 7777,
        vmc_port: int = 39539,
        output_port: int = 8889,
        rate_hz: float = 90.0,
        xrt_mode: str = "subprocess",
        console_demo_path: Optional[str] = None,
    ):
        self.xrt = XRTReceiver(
            xrt_host, xrt_port,
            mode=xrt_mode,
            console_demo_path=console_demo_path,
        )
        self.vmc = VMCReceiver(listen_port=vmc_port)
        self.output_port = output_port
        self.interval = 1.0 / rate_hz
        self._running = False

    def start(self):
        """모든 수신기 시작 + 출력 서버 시작."""
        self._running = True
        self.xrt.start()
        self.vmc.start()

        print(f"\n[Bridge] 통합 브릿지 시작")
        if self.xrt.mode == "subprocess":
            print(f"  XRT 입력: ConsoleDemo subprocess ({self.xrt.console_demo_path or 'NOT FOUND'})")
        else:
            print(f"  XRT 입력: TCP {self.xrt.host}:{self.xrt.port}")
        print(f"  VMC 입력: 0.0.0.0:{self.vmc.listen_port}")
        print(f"  출력 포트: {self.output_port}")
        print(f"  전송 속도: {1/self.interval:.0f} Hz\n")

        self._serve()

    def _serve(self):
        """TCP 서버: Isaac Lab 클라이언트에 통합 데이터 전달."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", self.output_port))
        server.listen(1)

        while self._running:
            print(f"[Bridge] Isaac Lab 클라이언트 대기 중 (port {self.output_port})...")
            try:
                client, addr = server.accept()
                print(f"[Bridge] 클라이언트 연결: {addr}")
                self._stream_to_client(client)
            except OSError:
                break

    def _stream_to_client(self, client: socket.socket):
        """클라이언트에 90Hz로 통합 데이터 스트리밍."""
        try:
            while self._running:
                xrt_data = self.xrt.get_latest() or {}
                finger_data = self.vmc.get_finger_data()

                merged = {
                    "timestamp": time.time_ns(),
                    "xrt": xrt_data,
                    "udcap_fingers": finger_data,
                }

                msg = json.dumps(merged, separators=(",", ":")) + "\n"
                client.sendall(msg.encode("utf-8"))
                time.sleep(self.interval)

        except (BrokenPipeError, ConnectionResetError):
            print("[Bridge] 클라이언트 연결 끊김. 재연결 대기...")

    def stop(self):
        self._running = False
        self.xrt.stop()
        self.vmc.stop()


def main():
    parser = argparse.ArgumentParser(description="PICO + UDCAP 통합 텔레오퍼레이션 브릿지")
    parser.add_argument("--xrt_host", type=str, default="localhost")
    parser.add_argument("--xrt_port", type=int, default=7777, help="XRT TCP 포트 (tcp 모드)")
    parser.add_argument("--vmc_port", type=int, default=39539, help="UDCAP VMC 수신 포트")
    parser.add_argument("--output_port", type=int, default=8889, help="Isaac Lab 출력 포트")
    parser.add_argument("--rate", type=float, default=90.0, help="전송 속도 (Hz)")
    parser.add_argument(
        "--xrt_mode", type=str, default="subprocess",
        choices=["subprocess", "tcp"],
        help="XRT 수신 모드: subprocess(ConsoleDemo 직접실행, 기본) / tcp(외부 TCP 서버)",
    )
    parser.add_argument(
        "--console_demo", type=str, default=None,
        help="ConsoleDemo 바이너리 경로 (subprocess 모드)",
    )
    args = parser.parse_args()

    bridge = UnifiedBridge(
        xrt_host=args.xrt_host,
        xrt_port=args.xrt_port,
        vmc_port=args.vmc_port,
        output_port=args.output_port,
        rate_hz=args.rate,
        xrt_mode=args.xrt_mode,
        console_demo_path=args.console_demo,
    )

    def _shutdown(sig, frame):
        print("\n[Bridge] 종료...")
        bridge.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        bridge.start()
    except KeyboardInterrupt:
        print("\n[Bridge] 종료...")
        bridge.stop()


if __name__ == "__main__":
    main()
