"""PICO 4 Ultra + Motion Tracker + UDCAP 통합 텔레오퍼레이션 디바이스.

XRoboToolkit PC Service + UDCAP VMC/OSC 데이터를 통합하여
Isaac Lab의 G1 38D 액션 텐서를 생성합니다.

데이터 흐름:
  통합 브릿지 (호스트 TCP :8889) → 이 디바이스 (Docker) → 38D action

통합 브릿지 JSON 구조:
  {
    "timestamp": int (nanoseconds),
    "xrt": { ... XRoboToolkit SDK JSON ... },
    "udcap_fingers": {
      "left": { "LeftIndexProximal": [qx,qy,qz,qw], ... },
      "right": { "RightIndexProximal": [qx,qy,qz,qw], ... }
    }
  }
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Dict, Optional, Tuple

import torch

from .g1_retargeter import G1PICORetargeter
from .xrt_data_parser import XRTDataParser, XRTFrame


class PICOFullBodyTeleopDevice:
    """PICO 4 Ultra Full-Body 텔레오퍼레이션 디바이스.

    Isaac Lab의 텔레오퍼레이션 디바이스 인터페이스를 구현하여
    기존 OpenXRDevice를 대체합니다.

    Usage:
        device = PICOFullBodyTeleopDevice(bridge_port=8889)
        device.start()

        while running:
            action = device.advance()  # (38,) tensor
            obs = env.step(action)

        device.stop()
    """

    def __init__(
        self,
        bridge_host: str = "localhost",
        bridge_port: int = 8889,
        use_udcap_fingers: bool = True,
        position_scale: float = 1.0,
        rotation_scale: float = 1.0,
        gripper_threshold: float = 0.5,
        body_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        controller_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        debug: bool = True,
    ):
        """
        Args:
            bridge_host: 통합 브릿지 TCP 호스트 (Docker network_mode:host → localhost)
            bridge_port: 통합 브릿지 TCP 포트
            use_udcap_fingers: UDCAP 핑거 데이터 사용 여부
            position_scale: EEF 위치 스케일 (사용자 키 → 로봇 비율)
            rotation_scale: EEF 회전 스케일
            gripper_threshold: 그리퍼 닫힘 트리거 임계값
            body_pos_offset: PICO Body 좌표 → G1 pelvis 프레임 평행이동
            controller_pos_offset: PICO Controller 좌표 → G1 pelvis 프레임 평행이동
            debug: XRT 키/소스 진단 로그 활성화
        """
        self.bridge_host = bridge_host
        self.bridge_port = bridge_port
        self.use_udcap_fingers = use_udcap_fingers
        self.debug = debug

        self._retargeter = G1PICORetargeter(
            position_scale=position_scale,
            rotation_scale=rotation_scale,
            gripper_threshold=gripper_threshold,
            body_pos_offset=body_pos_offset,
            controller_pos_offset=controller_pos_offset,
            debug=debug,
        )

        self._latest_raw: Optional[dict] = None
        self._latest_frame: Optional[XRTFrame] = None
        self._latest_udcap: Optional[dict] = None
        self._lock = threading.Lock()
        self._running = False
        self._connected = False
        self._recv_thread: Optional[threading.Thread] = None
        self._recv_count = 0

        # 진단: XRT JSON 키 수신 통계
        self._key_stats: Dict[str, int] = {}

    def start(self):
        """TCP 수신 스레드 시작."""
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        print(f"[PICODevice] 브릿지 연결 시도: {self.bridge_host}:{self.bridge_port}")

    def stop(self):
        """수신 중단."""
        self._running = False

    def _recv_loop(self):
        """TCP 수신 루프 (재연결 포함)."""
        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self.bridge_host, self.bridge_port))
                sock.settimeout(None)
                self._connected = True
                print(f"[PICODevice] 브릿지 연결 성공: {self.bridge_host}:{self.bridge_port}")

                buffer = ""
                while self._running:
                    chunk = sock.recv(65536).decode("utf-8")
                    if not chunk:
                        break
                    buffer += chunk
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            raw = json.loads(line)
                            xrt_raw = raw.get("xrt", {})

                            # XRT JSON은 {"functionName": "Tracking", "value": "<json string>"} 구조
                            # 진단을 위해 inner value JSON 키도 확인
                            inner_keys = self._extract_inner_keys(xrt_raw)
                            for k in inner_keys:
                                self._key_stats[k] = self._key_stats.get(k, 0) + 1

                            frame = XRTDataParser.parse(xrt_raw)
                            udcap = self._parse_udcap(raw.get("udcap_fingers", {}))

                            with self._lock:
                                self._latest_raw = raw
                                self._latest_frame = frame
                                self._latest_udcap = udcap
                                self._recv_count += 1

                            # 디버그: 첫 수신 + 주기적 상태 출력
                            if self._recv_count == 1:
                                print(f"[PICODevice] ✓ 첫 데이터 수신!")
                                print(f"  outer keys: {list(xrt_raw.keys())}")
                                print(f"  inner keys: {inner_keys}")
                                print(f"  Head pose: ({frame.head_pose.x:.3f}, {frame.head_pose.y:.3f}, {frame.head_pose.z:.3f})")
                                print(f"  R-ctrl pose: ({frame.right_controller.pose.x:.3f}, {frame.right_controller.pose.y:.3f}, {frame.right_controller.pose.z:.3f})")
                                print(f"  L-ctrl pose: ({frame.left_controller.pose.x:.3f}, {frame.left_controller.pose.y:.3f}, {frame.left_controller.pose.z:.3f})")
                                print(f"  R-hand active: {frame.right_hand.is_active}, joints: {len(frame.right_hand.joints)}")
                                print(f"  Body joints: {len(frame.body_joints)} (24 expected for Body+HighAcc)")
                                print(f"  Motion trackers: {len(frame.trackers)}")
                                print(f"  UDCAP bones: {len(udcap) if udcap else 0}")
                            elif self._recv_count % 300 == 0:
                                print(f"[PICODevice] #{self._recv_count} | "
                                      f"keys={list(self._key_stats.keys())} | "
                                      f"body={len(frame.body_joints)} trackers={len(frame.trackers)} | "
                                      f"R-ctrl=({frame.right_controller.pose.x:.2f},{frame.right_controller.pose.y:.2f},{frame.right_controller.pose.z:.2f})")
                        except (json.JSONDecodeError, KeyError, ValueError) as e:
                            if not hasattr(self, '_err_logged'):
                                print(f"[PICODevice] 파싱 오류: {e}")
                                print(f"  raw line (first 200): {line[:200]}")
                                self._err_logged = True

            except (ConnectionRefusedError, ConnectionResetError, OSError, socket.timeout) as e:
                self._connected = False
                if self._running:
                    time.sleep(1)  # 1초 후 재연결

            finally:
                self._connected = False
                try:
                    sock.close()
                except Exception:
                    pass

    @staticmethod
    def _extract_inner_keys(xrt_raw: dict) -> list:
        """XRT JSON의 inner value 구조에서 어떤 트래킹 키가 들어있는지 확인.

        구조:  {"functionName":"Tracking", "value":"{\"Head\":...,\"Body\":...}"}
        반환: ["Head", "Controller", "Hand", "Body", "Motion"] 중 활성된 키들
        """
        if not xrt_raw:
            return []
        value = xrt_raw.get("value", xrt_raw)
        if isinstance(value, str):
            try:
                value = json.loads(value.replace("\\", ""))
            except (json.JSONDecodeError, TypeError):
                return []
        if not isinstance(value, dict):
            return []
        return [k for k in ("Head", "Controller", "Hand", "Body", "Motion") if k in value]

    @staticmethod
    def _parse_udcap(udcap_raw: dict) -> Optional[Dict[str, Tuple[float, float, float, float]]]:
        """UDCAP JSON → bone name → quaternion 딕셔너리."""
        if not udcap_raw:
            return None
        bones = {}
        for side_key in ("left", "right"):
            side_data = udcap_raw.get(side_key, {})
            for bone_name, quat in side_data.items():
                if isinstance(quat, (list, tuple)) and len(quat) >= 4:
                    bones[bone_name] = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        return bones if bones else None

    def advance(self) -> Optional[torch.Tensor]:
        """최신 38D 액션 텐서 반환 (Pink IK 절대 좌표 포맷).

        Pink IK format: [left_pos(3), left_quat(4), right_pos(3), right_quat(4), hand(24)]
        리타게터가 직접 절대 좌표 38D를 출력합니다 (델타 누적 X).

        데이터 소스 우선순위 (left/right 각각 독립):
          1. Body+HighAcc joints[20/21] (5 트래커 풀바디)
          2. Controller pose
          3. 디폴트 idle 포즈

        Returns:
            (38,) tensor 또는 데이터 없으면 None
        """
        with self._lock:
            frame = self._latest_frame
            udcap = self._latest_udcap

        if frame is None:
            if not hasattr(self, '_none_warned'):
                print("[PICODevice] advance(): frame is None — 브릿지/ConsoleDemo 데이터 미수신")
                print("  체크: ConsoleDemo 실행 여부, PICO 앱 PC 연결 상태, 브릿지 stdout 'device data' 라인")
                self._none_warned = True
            return None

        # 리타게터가 38D 절대 Pink IK 액션을 직접 반환
        pink_action = self._retargeter.retarget(
            xrt_frame=frame,
            udcap_bones=udcap,
            use_udcap=self.use_udcap_fingers,
        )

        # 첫 액션 디버그
        if not hasattr(self, '_action_logged'):
            sources = self._retargeter.get_source_info()
            print(f"[PICODevice] ✓ 첫 Pink IK 액션:")
            print(f"  소스: L={sources['left']} R={sources['right']}")
            print(f"  L-EEF: pos={pink_action[0:3].tolist()}, quat={pink_action[3:7].tolist()}")
            print(f"  R-EEF: pos={pink_action[7:10].tolist()}, quat={pink_action[10:14].tolist()}")
            hand = pink_action[14:38]
            print(f"  Hand nonzero: {(hand.abs() > 1e-4).sum().item()}/24")
            self._action_logged = True

        return pink_action

    def get_raw_data(self) -> Optional[dict]:
        """원시 JSON 데이터 반환 (디버그/개입 트리거용)."""
        with self._lock:
            return self._latest_raw

    def get_xrt_frame(self) -> Optional[XRTFrame]:
        """파싱된 XRTFrame 반환."""
        with self._lock:
            return self._latest_frame

    def get_controller_data(self) -> Optional[dict]:
        """컨트롤러 버튼 상태 반환 (개입 인터페이스용).

        Returns:
            {"left": ControllerData, "right": ControllerData} 또는 None
        """
        with self._lock:
            frame = self._latest_frame
        if frame is None:
            return None
        return {
            "left": frame.left_controller,
            "right": frame.right_controller,
        }

    def reset(self):
        """디바이스 상태 리셋."""
        self._retargeter.reset()
        with self._lock:
            self._latest_frame = None
            self._latest_udcap = None
        # 디버그 플래그 리셋
        for attr in ('_action_logged', '_none_warned', '_err_logged'):
            if hasattr(self, attr):
                delattr(self, attr)

    @property
    def is_connected(self) -> bool:
        return self._connected
