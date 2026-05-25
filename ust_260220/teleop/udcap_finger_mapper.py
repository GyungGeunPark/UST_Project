"""UDCAP VR Gloves VMC/OSC 핑거 데이터 → Inspire 5지 핸드 관절 매핑.

UDCAP은 VMC 프로토콜(/VMC/Ext/Bone/Pos)로 각 손가락 bone rotation을 전송.
이를 Inspire 5지 핸드의 12 관절 각도로 매핑합니다.

VMC Bone Names (per hand):
  {Side}ThumbMetacarpal, {Side}ThumbProximal, {Side}ThumbDistal
  {Side}IndexProximal, {Side}IndexIntermediate, {Side}IndexDistal
  {Side}MiddleProximal, {Side}MiddleIntermediate, {Side}MiddleDistal
  {Side}RingProximal, {Side}RingIntermediate, {Side}RingDistal
  {Side}LittleProximal, {Side}LittleIntermediate, {Side}LittleDistal

Inspire 12 DOF/hand:
  [0] thumb_bend, [1] thumb_rotation, [2] index_bend, [3] index_spread,
  [4] middle_bend, [5] middle_spread, [6] ring_bend, [7] ring_spread,
  [8] little_bend, [9] little_spread, [10] thumb_opposition, [11] wrist_rotation
"""

from __future__ import annotations

import math
from typing import Dict, Tuple

import torch


# UDCAP VMC bone name patterns
_FINGER_BONES = {
    "Thumb": {"Metacarpal": 0, "Proximal": 1, "Distal": 2},
    "Index": {"Proximal": 0, "Intermediate": 1, "Distal": 2},
    "Middle": {"Proximal": 0, "Intermediate": 1, "Distal": 2},
    "Ring": {"Proximal": 0, "Intermediate": 1, "Distal": 2},
    "Little": {"Proximal": 0, "Intermediate": 1, "Distal": 2},
}

# Inspire 관절 인덱스
THUMB_BEND = 0
THUMB_ROTATION = 1
INDEX_BEND = 2
INDEX_SPREAD = 3
MIDDLE_BEND = 4
MIDDLE_SPREAD = 5
RING_BEND = 6
RING_SPREAD = 7
LITTLE_BEND = 8
LITTLE_SPREAD = 9
THUMB_OPPOSITION = 10
WRIST_ROTATION = 11

INSPIRE_JOINT_DIM = 12


def _quat_to_bend(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 → 굽힘 각도 (0~1 정규화).

    손가락 굽힘은 주로 X축 회전. 간단한 각도 추출.
    """
    angle = 2.0 * math.acos(max(-1.0, min(1.0, abs(qw))))
    return min(angle / math.pi, 1.0)


def _quat_to_spread(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 → 벌림 각도 (-1~1 정규화).

    손가락 벌림은 주로 Z축 회전 (adduction/abduction).
    """
    spread = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
    return max(-1.0, min(1.0, spread / (math.pi / 4)))


def _quat_to_rotation(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 → 엄지 회전 각도 (-1~1)."""
    rotation = math.atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy**2 + qz**2))
    return max(-1.0, min(1.0, rotation / (math.pi / 2)))


class UDCAPFingerMapper:
    """UDCAP VMC bone rotation → Inspire 12D 관절 매퍼.

    Usage:
        mapper = UDCAPFingerMapper()
        left_joints = mapper.map_hand(vmc_left_bones, is_right=False)
        right_joints = mapper.map_hand(vmc_right_bones, is_right=True)
    """

    def __init__(self, bend_scale: float = 1.0, spread_scale: float = 0.5):
        """
        Args:
            bend_scale: 굽힘 스케일 (1.0 = 전체 범위)
            spread_scale: 벌림 스케일 (기본 0.5, Inspire spread 범위가 작음)
        """
        self.bend_scale = bend_scale
        self.spread_scale = spread_scale

    def map_hand(
        self, bones: Dict[str, Tuple[float, float, float, float]], is_right: bool
    ) -> torch.Tensor:
        """VMC bone 데이터 → Inspire 12D 관절 텐서.

        Args:
            bones: bone_name → (qx, qy, qz, qw) 딕셔너리
            is_right: 오른손 여부

        Returns:
            (12,) tensor: Inspire 관절 각도
        """
        inspire = torch.zeros(INSPIRE_JOINT_DIM)
        side = "Right" if is_right else "Left"

        # Thumb
        thumb_meta = bones.get(f"{side}ThumbMetacarpal")
        thumb_prox = bones.get(f"{side}ThumbProximal")
        if thumb_prox:
            inspire[THUMB_BEND] = _quat_to_bend(*thumb_prox) * self.bend_scale
        if thumb_meta:
            inspire[THUMB_ROTATION] = _quat_to_rotation(*thumb_meta)
            inspire[THUMB_OPPOSITION] = _quat_to_bend(*thumb_meta) * self.bend_scale

        # Index
        index_prox = bones.get(f"{side}IndexProximal")
        if index_prox:
            inspire[INDEX_BEND] = _quat_to_bend(*index_prox) * self.bend_scale
            inspire[INDEX_SPREAD] = _quat_to_spread(*index_prox) * self.spread_scale

        # Middle
        middle_prox = bones.get(f"{side}MiddleProximal")
        if middle_prox:
            inspire[MIDDLE_BEND] = _quat_to_bend(*middle_prox) * self.bend_scale
            inspire[MIDDLE_SPREAD] = _quat_to_spread(*middle_prox) * self.spread_scale

        # Ring
        ring_prox = bones.get(f"{side}RingProximal")
        if ring_prox:
            inspire[RING_BEND] = _quat_to_bend(*ring_prox) * self.bend_scale
            inspire[RING_SPREAD] = _quat_to_spread(*ring_prox) * self.spread_scale

        # Little
        little_prox = bones.get(f"{side}LittleProximal")
        if little_prox:
            inspire[LITTLE_BEND] = _quat_to_bend(*little_prox) * self.bend_scale
            inspire[LITTLE_SPREAD] = _quat_to_spread(*little_prox) * self.spread_scale

        return inspire

    def map_from_xrt_hand(
        self, hand_joints: list, is_right: bool
    ) -> torch.Tensor:
        """XRT hand tracking 26관절 → Inspire 12D (UDCAP 미사용 시 폴백).

        Args:
            hand_joints: XRTFrame.HandData.joints 리스트 (HandJoint)
            is_right: 오른손 여부

        Returns:
            (12,) tensor
        """
        inspire = torch.zeros(INSPIRE_JOINT_DIM)
        if len(hand_joints) < 26:
            return inspire

        # XRT joint indices: 0=Palm, 1=Wrist, 2-5=Thumb, 6-10=Index,
        #                     11-15=Middle, 16-20=Ring, 21-25=Little
        finger_map = [
            (3, THUMB_BEND, None),       # Thumb proximal
            (7, INDEX_BEND, INDEX_SPREAD),  # Index proximal
            (12, MIDDLE_BEND, MIDDLE_SPREAD),
            (17, RING_BEND, RING_SPREAD),
            (22, LITTLE_BEND, LITTLE_SPREAD),
        ]

        for joint_idx, bend_idx, spread_idx in finger_map:
            if joint_idx < len(hand_joints):
                j = hand_joints[joint_idx]
                p = j.pose
                inspire[bend_idx] = _quat_to_bend(p.qx, p.qy, p.qz, p.qw) * self.bend_scale
                if spread_idx is not None:
                    inspire[spread_idx] = _quat_to_spread(p.qx, p.qy, p.qz, p.qw) * self.spread_scale

        # Thumb extras from metacarpal (index 2)
        if len(hand_joints) > 2:
            p = hand_joints[2].pose
            inspire[THUMB_ROTATION] = _quat_to_rotation(p.qx, p.qy, p.qz, p.qw)
            inspire[THUMB_OPPOSITION] = _quat_to_bend(p.qx, p.qy, p.qz, p.qw) * self.bend_scale

        return inspire
