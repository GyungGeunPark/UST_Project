"""XRoboToolkit + UDCAP → G1 Pink IK 액션 리타게터.

Pink IK 38D 절대 좌표 액션을 직접 출력합니다 (델타 누적 X).

출력: 38D action tensor (Pink IK 포맷)
  [0:3]   left  EEF target position  (3D, robot pelvis frame)
  [3:7]   left  EEF target quaternion (wxyz, 4D)
  [7:10]  right EEF target position  (3D)
  [10:14] right EEF target quaternion (wxyz, 4D)
  [14:38] 24 hand joints (Inspire FTP)

데이터 우선순위:
  1. Body tracking joints[20]/[21] (LEFT_WRIST/RIGHT_WRIST) — Body+HighAcc 모드에서 활성
  2. Motion tracker (수동 매핑 가능) — Motion 독립 트래킹 모드에서 활성
  3. Controller pose — 컨트롤러를 들고 있을 때만

좌표계 변환:
  XRT (X right, Y up, Z in) → IsaacLab/USD (X right, Y up, Z out)
  → Z축 반전: pos.z *= -1, quat.qz *= -1
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import torch

from .xrt_data_parser import BodyJoint, ControllerData, Pose7D, XRTFrame
from .udcap_finger_mapper import UDCAPFingerMapper


# BodyTrackerRole 인덱스 (PICO SDK)
BODY_PELVIS = 0
BODY_LEFT_WRIST = 20
BODY_RIGHT_WRIST = 21
BODY_LEFT_HAND = 22
BODY_RIGHT_HAND = 23

# G1 idle 포즈 (pelvis 프레임, Pink IK target)
DEFAULT_LEFT_POS = (-0.1487, 0.2038, 1.0952)
DEFAULT_LEFT_QUAT = (0.707, 0.0, 0.0, 0.707)  # wxyz
DEFAULT_RIGHT_POS = (0.1487, 0.2038, 1.0952)
DEFAULT_RIGHT_QUAT = (0.707, 0.0, 0.0, 0.707)


def _is_invalid_pose(pose: Pose7D) -> bool:
    """기본 식별 포즈 (qw=1, 나머지 0) 또는 NaN이면 무효."""
    if math.isnan(pose.x) or math.isnan(pose.qw):
        return True
    if pose.qw == 1.0 and pose.qx == 0.0 and pose.qy == 0.0 and pose.qz == 0.0:
        if pose.x == 0.0 and pose.y == 0.0 and pose.z == 0.0:
            return True
    return False


class G1PICORetargeter:
    """XRT(Body/Controller) + UDCAP(Finger) → G1 Pink IK 38D 절대 액션."""

    ACTION_DIM = 38

    def __init__(
        self,
        position_scale: float = 1.0,
        rotation_scale: float = 1.0,
        gripper_threshold: float = 0.5,
        body_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        controller_pos_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        debug: bool = False,
    ):
        """
        Args:
            position_scale: 트래킹 → 로봇 위치 스케일 (사용자 키 보정)
            rotation_scale: 회전 스케일 (보통 1.0)
            gripper_threshold: 트리거 → 그리퍼 닫힘 임계값
            body_pos_offset: PICO Body 좌표 → G1 pelvis 프레임 변환 오프셋
            controller_pos_offset: PICO Controller → G1 pelvis 프레임 변환 오프셋
            debug: 매 프레임 데이터 소스 로깅
        """
        self.position_scale = position_scale
        self.rotation_scale = rotation_scale
        self.gripper_threshold = gripper_threshold
        self.body_pos_offset = torch.tensor(body_pos_offset)
        self.controller_pos_offset = torch.tensor(controller_pos_offset)
        self.debug = debug

        self.finger_mapper = UDCAPFingerMapper()

        # 디폴트 (idle) 타깃
        self._default_left_pos = torch.tensor(DEFAULT_LEFT_POS)
        self._default_left_quat = torch.tensor(DEFAULT_LEFT_QUAT)
        self._default_right_pos = torch.tensor(DEFAULT_RIGHT_POS)
        self._default_right_quat = torch.tensor(DEFAULT_RIGHT_QUAT)

        # 통계
        self._frames = 0
        self._source_left = "default"
        self._source_right = "default"

    # ─────────────────────────────────────────────
    # 메인 진입점
    # ─────────────────────────────────────────────

    def retarget(
        self,
        xrt_frame: XRTFrame,
        udcap_bones: Optional[Dict[str, Tuple[float, float, float, float]]] = None,
        use_udcap: bool = True,
    ) -> torch.Tensor:
        """XRTFrame → 38D Pink IK 절대 액션 텐서."""
        action = torch.zeros(self.ACTION_DIM)

        # ── 1. 좌/우 EEF 절대 타깃 결정 ──
        left_pos, left_quat, src_l = self._resolve_eef_target(xrt_frame, side="left")
        right_pos, right_quat, src_r = self._resolve_eef_target(xrt_frame, side="right")

        self._source_left = src_l
        self._source_right = src_r

        # Pink IK 포맷 [L_pos(3), L_quat(4), R_pos(3), R_quat(4), hand(24)]
        action[0:3] = left_pos
        action[3:7] = left_quat
        action[7:10] = right_pos
        action[10:14] = right_quat

        # ── 2. 핸드 24관절 ──
        hand_24 = self._resolve_hand_joints(xrt_frame, udcap_bones, use_udcap)
        action[14:38] = hand_24

        self._frames += 1
        if self.debug and self._frames % 60 == 0:
            n_body = len(xrt_frame.body_joints)
            n_track = len(xrt_frame.trackers)
            print(
                f"[Retarget #{self._frames}] L={src_l} R={src_r} | "
                f"body_joints={n_body} trackers={n_track} | "
                f"L_pos=({left_pos[0]:.3f},{left_pos[1]:.3f},{left_pos[2]:.3f}) "
                f"R_pos=({right_pos[0]:.3f},{right_pos[1]:.3f},{right_pos[2]:.3f})"
            )

        return action

    # ─────────────────────────────────────────────
    # EEF 타깃 결정 (Body → Controller → Default)
    # ─────────────────────────────────────────────

    def _resolve_eef_target(
        self, frame: XRTFrame, side: str
    ) -> Tuple[torch.Tensor, torch.Tensor, str]:
        """좌/우 EEF 절대 타깃 결정.

        우선순위:
          1. Body+HighAcc joints[20/21] (LEFT/RIGHT_WRIST) — 5 트래커 풀바디 모드
          2. Controller pose — 컨트롤러 들고 있을 때
          3. 디폴트 idle 포즈
        """
        # 1. Body 트래킹 (24 관절)
        if len(frame.body_joints) >= 24:
            wrist_idx = BODY_LEFT_WRIST if side == "left" else BODY_RIGHT_WRIST
            wrist_pose = frame.body_joints[wrist_idx].pose
            if not _is_invalid_pose(wrist_pose):
                pos, quat = self._pico_body_to_g1_eef(wrist_pose)
                return pos, quat, "body"

        # 2. Controller pose
        ctrl = frame.left_controller if side == "left" else frame.right_controller
        if not _is_invalid_pose(ctrl.pose):
            pos, quat = self._pico_controller_to_g1_eef(ctrl.pose)
            return pos, quat, "controller"

        # 3. Default idle
        if side == "left":
            return self._default_left_pos.clone(), self._default_left_quat.clone(), "default"
        return self._default_right_pos.clone(), self._default_right_quat.clone(), "default"

    def _pico_body_to_g1_eef(self, pose: Pose7D) -> Tuple[torch.Tensor, torch.Tensor]:
        """PICO Body 좌표 (origin=헤드셋 시작 위치) → G1 pelvis frame EEF.

        XRT Z-in → IsaacLab Z-out 변환 후 사용자 정의 오프셋 적용.
        """
        pos = torch.tensor([
            pose.x * self.position_scale + self.body_pos_offset[0],
            pose.y * self.position_scale + self.body_pos_offset[1],
            -pose.z * self.position_scale + self.body_pos_offset[2],  # Z 반전
        ])
        # 쿼터니언 wxyz, Z축 반전
        quat = torch.tensor([pose.qw, pose.qx, pose.qy, -pose.qz])
        quat = quat / (quat.norm() + 1e-8)
        return pos, quat

    def _pico_controller_to_g1_eef(self, pose: Pose7D) -> Tuple[torch.Tensor, torch.Tensor]:
        """PICO Controller 좌표 → G1 pelvis frame EEF (좌표계 변환 + 오프셋)."""
        pos = torch.tensor([
            pose.x * self.position_scale + self.controller_pos_offset[0],
            pose.y * self.position_scale + self.controller_pos_offset[1],
            -pose.z * self.position_scale + self.controller_pos_offset[2],
        ])
        quat = torch.tensor([pose.qw, pose.qx, pose.qy, -pose.qz])
        quat = quat / (quat.norm() + 1e-8)
        return pos, quat

    # ─────────────────────────────────────────────
    # 핸드 (24 관절)
    # ─────────────────────────────────────────────

    def _resolve_hand_joints(
        self,
        frame: XRTFrame,
        udcap_bones: Optional[Dict[str, Tuple[float, float, float, float]]],
        use_udcap: bool,
    ) -> torch.Tensor:
        """우선 UDCAP 글러브 → XRT hand tracking 폴백 → zeros.

        리타게터 출력은 Inspire 12 DOF/hand. 24 관절 매핑은 호출 측에서.
        하지만 Pink IK 포맷이 24 관절을 직접 요구하므로 여기서 직접 매핑.
        """
        right_12 = torch.zeros(12)
        left_12 = torch.zeros(12)

        if use_udcap and udcap_bones:
            right_bones = {k: v for k, v in udcap_bones.items() if "Right" in k}
            left_bones = {k: v for k, v in udcap_bones.items() if "Left" in k}
            if right_bones:
                right_12 = self.finger_mapper.map_hand(right_bones, is_right=True)
            if left_bones:
                left_12 = self.finger_mapper.map_hand(left_bones, is_right=False)
        else:
            if frame.right_hand.is_active and frame.right_hand.joints:
                right_12 = self.finger_mapper.map_from_xrt_hand(
                    frame.right_hand.joints, is_right=True
                )
            if frame.left_hand.is_active and frame.left_hand.joints:
                left_12 = self.finger_mapper.map_from_xrt_hand(
                    frame.left_hand.joints, is_right=False
                )

        return self._inspire12_to_24joints(right_12, left_12)

    @staticmethod
    def _inspire12_to_24joints(right_12: torch.Tensor, left_12: torch.Tensor) -> torch.Tensor:
        """Inspire 12 DOF/hand → Pink IK 24 hand joint 순서."""
        j = torch.zeros(24)
        # Left proximal (5)
        j[0] = left_12[2]
        j[1] = left_12[4]
        j[2] = left_12[8]
        j[3] = left_12[6]
        j[4] = left_12[1]
        # Right proximal (5)
        j[5] = right_12[2]
        j[6] = right_12[4]
        j[7] = right_12[8]
        j[8] = right_12[6]
        j[9] = right_12[1]
        # Left intermediate (5)
        j[10] = left_12[2]
        j[11] = left_12[4]
        j[12] = left_12[8]
        j[13] = left_12[6]
        j[14] = left_12[0]
        # Right intermediate (5)
        j[15] = right_12[2]
        j[16] = right_12[4]
        j[17] = right_12[8]
        j[18] = right_12[6]
        j[19] = right_12[0]
        # Thumb intermediate/distal (4)
        j[20] = left_12[10]
        j[21] = right_12[10]
        j[22] = left_12[0]
        j[23] = right_12[0]
        return j

    # ─────────────────────────────────────────────
    # 진단/리셋
    # ─────────────────────────────────────────────

    def get_source_info(self) -> Dict[str, str]:
        """현재 활성 데이터 소스 (디버그용)."""
        return {"left": self._source_left, "right": self._source_right}

    def reset(self):
        self._frames = 0
        self._source_left = "default"
        self._source_right = "default"
