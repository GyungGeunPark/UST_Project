"""XRoboToolkit JSON 데이터 파서.

XRoboToolkit PC Service SDK에서 전달되는 JSON 트래킹 데이터를
정규화된 Python 딕셔너리로 변환합니다.

XRT JSON 구조:
  {
    "functionName": "Tracking",
    "value": "{\"Head\":{...},\"Controller\":{...},\"Hand\":{...},\"Body\":{...},\"Motion\":{...}}"
  }
  value는 문자열로 전달되므로 이중 파싱 필요.

좌표계:
  XRT: 오른손 좌표계, X right, Y up, Z in (화면 안쪽)
  Isaac Lab/USD: X right, Y up, Z out (화면 바깥) → Z축 반전 필요
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class Pose7D:
    """위치(3D) + 쿼터니언(4D) = 7D 포즈."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    @classmethod
    def from_csv(cls, csv_str: str) -> "Pose7D":
        """콤마 구분 문자열에서 파싱: 'x,y,z,qx,qy,qz,qw'."""
        parts = csv_str.split(",")
        if len(parts) < 7:
            return cls()
        return cls(
            x=float(parts[0]), y=float(parts[1]), z=float(parts[2]),
            qx=float(parts[3]), qy=float(parts[4]), qz=float(parts[5]), qw=float(parts[6]),
        )

    def to_isaac_lab(self) -> "Pose7D":
        """XRT 좌표계 → Isaac Lab 좌표계 (Z축 반전)."""
        return Pose7D(
            x=self.x, y=self.y, z=-self.z,
            qx=self.qx, qy=self.qy, qz=-self.qz, qw=self.qw,
        )

    def as_tuple(self) -> Tuple[float, ...]:
        return (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)


@dataclass
class ControllerData:
    """컨트롤러 데이터."""

    pose: Pose7D = field(default_factory=Pose7D)
    trigger: float = 0.0
    grip: float = 0.0
    axis_x: float = 0.0
    axis_y: float = 0.0
    primary_button: bool = False
    secondary_button: bool = False
    menu_button: bool = False


@dataclass
class HandJoint:
    """핸드 트래킹 관절."""

    pose: Pose7D = field(default_factory=Pose7D)
    status: int = 0
    radius: float = 0.0


@dataclass
class HandData:
    """핸드 트래킹 데이터 (26관절)."""

    is_active: bool = False
    joints: List[HandJoint] = field(default_factory=list)
    scale: float = 1.0


@dataclass
class BodyJoint:
    """바디 트래킹 관절."""

    pose: Pose7D = field(default_factory=Pose7D)


@dataclass
class XRTFrame:
    """하나의 XRoboToolkit 트래킹 프레임 (정규화된 데이터)."""

    timestamp_ns: int = 0
    input_mode: int = 0  # 0=head, 1=controller, 2=gesture

    # Head
    head_pose: Pose7D = field(default_factory=Pose7D)
    head_status: int = 0

    # Controllers
    left_controller: ControllerData = field(default_factory=ControllerData)
    right_controller: ControllerData = field(default_factory=ControllerData)

    # Hands (26 joints each)
    left_hand: HandData = field(default_factory=HandData)
    right_hand: HandData = field(default_factory=HandData)

    # Body (24 joints)
    body_joints: List[BodyJoint] = field(default_factory=list)

    # Motion Trackers (up to 3)
    trackers: List[Pose7D] = field(default_factory=list)


class XRTDataParser:
    """XRoboToolkit JSON → XRTFrame 파서."""

    @staticmethod
    def parse(raw_json: dict) -> XRTFrame:
        """원시 JSON → 정규화된 XRTFrame.

        Args:
            raw_json: XRoboToolkit SDK 콜백에서 수신한 JSON.
                통합 브릿지의 "xrt" 필드 또는 직접 SDK 출력.
        """
        frame = XRTFrame()

        # value가 문자열이면 이중 파싱
        value = raw_json.get("value", raw_json)
        if isinstance(value, str):
            try:
                value = json.loads(value.replace("\\", ""))
            except (json.JSONDecodeError, TypeError):
                return frame

        frame.timestamp_ns = value.get("timeStampNs", 0)
        frame.input_mode = value.get("Input", 0)

        # Head
        head = value.get("Head", {})
        if head:
            frame.head_pose = Pose7D.from_csv(head.get("pose", ""))
            frame.head_status = head.get("status", 0)

        # Controllers
        ctrl = value.get("Controller", {})
        if ctrl:
            frame.left_controller = XRTDataParser._parse_controller(ctrl.get("left", {}))
            frame.right_controller = XRTDataParser._parse_controller(ctrl.get("right", {}))

        # Hands
        hand = value.get("Hand", {})
        if hand:
            frame.left_hand = XRTDataParser._parse_hand(hand.get("leftHand", {}))
            frame.right_hand = XRTDataParser._parse_hand(hand.get("rightHand", {}))

        # Body
        body = value.get("Body", {})
        if body:
            for joint_data in body.get("joints", []):
                bj = BodyJoint(pose=Pose7D.from_csv(joint_data.get("p", "")))
                frame.body_joints.append(bj)

        # Motion Trackers
        motion = value.get("Motion", {})
        if motion:
            for tracker_data in motion.get("joints", []):
                frame.trackers.append(Pose7D.from_csv(tracker_data.get("p", "")))

        return frame

    @staticmethod
    def _parse_controller(data: dict) -> ControllerData:
        if not data:
            return ControllerData()
        return ControllerData(
            pose=Pose7D.from_csv(data.get("pose", "")),
            trigger=float(data.get("trigger", 0.0)),
            grip=float(data.get("grip", 0.0)),
            axis_x=float(data.get("axisX", 0.0)),
            axis_y=float(data.get("axisY", 0.0)),
            primary_button=bool(data.get("primaryButton", False)),
            secondary_button=bool(data.get("secondaryButton", False)),
            menu_button=bool(data.get("menuButton", False)),
        )

    @staticmethod
    def _parse_hand(data: dict) -> HandData:
        if not data:
            return HandData()
        hand = HandData(
            is_active=bool(data.get("isActive", 0)),
            scale=float(data.get("scale", 1.0)),
        )
        for jl in data.get("HandJointLocations", []):
            joint = HandJoint(
                pose=Pose7D.from_csv(jl.get("p", "")),
                status=int(jl.get("s", 0)),
                radius=float(jl.get("r", 0.0)),
            )
            hand.joints.append(joint)
        return hand
