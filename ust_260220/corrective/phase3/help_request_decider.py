"""다중 불확실성 신호 통합 도움 요청 결정기.

3-Tier 추론 + 다중 불확실성 신호를 통합하여 도움 요청을 결정합니다:
1. 앙상블 행동 분산 (action uncertainty)
2. SigLIP2 빠른 분류 확률 (Tier 1, 매 스텝)
3. VLM 물체 인식 신뢰도 (Tier 3, 불확실 시)
4. Conformal prediction 집합 크기 (category uncertainty)
"""

from __future__ import annotations

import numpy as np
import torch
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .ensemble_policy import EnsemblePolicy
from .vlm_analyzer import CachedVLMAnalyzer, VLMObjectAnalyzer, SigLIP2Classifier
from .conformal_predictor import ConformalCategoryPredictor
from .adaptive_threshold import AdaptiveThreshold


class HelpRequestType(Enum):
    """도움 요청 유형."""

    NONE = "none"
    ACTION_CORRECTION = "action_correction"
    OBJECT_IDENTIFICATION = "object_identification"
    CATEGORY_SELECTION = "category_selection"
    COMBINED = "combined"


# 도움 요청 유형 상수 (HDF5 저장용)
HELP_TYPE_NONE = 0
HELP_TYPE_ACTION = 1
HELP_TYPE_OBJECT = 2
HELP_TYPE_CATEGORY = 3
HELP_TYPE_COMBINED = 4

HELP_TYPE_MAP = {
    HelpRequestType.NONE: HELP_TYPE_NONE,
    HelpRequestType.ACTION_CORRECTION: HELP_TYPE_ACTION,
    HelpRequestType.OBJECT_IDENTIFICATION: HELP_TYPE_OBJECT,
    HelpRequestType.CATEGORY_SELECTION: HELP_TYPE_CATEGORY,
    HelpRequestType.COMBINED: HELP_TYPE_COMBINED,
}


@dataclass
class HelpDecision:
    """도움 요청 결정 결과."""

    help_type: HelpRequestType = HelpRequestType.NONE
    action: Optional[torch.Tensor] = None
    uncertainty_scores: Dict[str, float] = None
    question: Optional[Dict] = None
    details: str = ""

    def __post_init__(self):
        if self.uncertainty_scores is None:
            self.uncertainty_scores = {}

    @property
    def needs_help(self) -> bool:
        return self.help_type != HelpRequestType.NONE

    @property
    def help_type_int(self) -> int:
        return HELP_TYPE_MAP.get(self.help_type, HELP_TYPE_NONE)


class HelpRequestDecider:
    """다중 불확실성 신호 통합 도움 요청 결정기.

    앙상블 정책 불확실성, VLM 물체 인식, conformal prediction을
    통합하여 도움 요청 여부와 유형을 결정합니다.

    사용법:
        decider = HelpRequestDecider(ensemble, vlm, conformal, threshold)
        decision = decider.decide(obs, rgb_image)
        if decision.needs_help:
            # VR HMD에 질문 표시
            send_question_to_vr(decision.question)
    """

    def __init__(
        self,
        ensemble_policy: EnsemblePolicy,
        vlm_analyzer: Optional[CachedVLMAnalyzer] = None,
        conformal_predictor: Optional[ConformalCategoryPredictor] = None,
        adaptive_threshold: Optional[AdaptiveThreshold] = None,
        siglip_classifier: Optional[SigLIP2Classifier] = None,
    ):
        """
        Args:
            ensemble_policy: 앙상블 정책 (행동 불확실성)
            vlm_analyzer: 캐시된 VLM 분석기 (Tier 3, 물체 인식)
            conformal_predictor: Conformal 카테고리 예측기
            adaptive_threshold: 적응적 불확실성 임계값
            siglip_classifier: SigLIP2 빠른 분류기 (Tier 1)
        """
        self.ensemble = ensemble_policy
        self.vlm = vlm_analyzer
        self.conformal = conformal_predictor
        self.threshold = adaptive_threshold or AdaptiveThreshold()
        self.siglip = siglip_classifier

        # 통계
        self._total_decisions = 0
        self._help_decisions = 0
        self._decision_history: List[HelpDecision] = []

    def decide(
        self,
        obs: torch.Tensor,
        rgb_image: Optional[np.ndarray] = None,
        current_target_object: Optional[str] = None,
    ) -> HelpDecision:
        """도움 요청 결정.

        Args:
            obs: 관찰 텐서 (batch, obs_dim)
            rgb_image: RGB 이미지 (VLM 분석용)
            current_target_object: 현재 처리 중인 물체 이름

        Returns:
            HelpDecision 결과
        """
        self._total_decisions += 1
        decision = HelpDecision()

        # 1. 앙상블 행동 불확실성
        mean_action, action_uncertainty, action_uncertain = (
            self.ensemble.predict_with_uncertainty(obs)
        )
        unc_scalar = action_uncertainty.mean().item()
        decision.uncertainty_scores["action"] = unc_scalar

        # 적응적 임계값으로 판단
        action_help = self.threshold.should_request_help(unc_scalar)

        # 2. VLM 물체 인식 (간헐적 호출)
        object_uncertain = False
        uncertain_objects = []
        if self.vlm is not None and rgb_image is not None:
            scene_analysis = self.vlm.step(rgb_image)
            uncertain_objects = self.vlm.get_uncertain_objects()
            object_uncertain = len(uncertain_objects) > 0
            if uncertain_objects:
                decision.uncertainty_scores["object"] = min(
                    o.get("confidence", 1.0) for o in uncertain_objects
                )
            else:
                decision.uncertainty_scores["object"] = 1.0

        # 3. 도움 요청 결정
        if action_help and object_uncertain:
            # 복합 불확실성
            decision.help_type = HelpRequestType.COMBINED
            obj_name = (
                uncertain_objects[0]["name"] if uncertain_objects else "물체"
            )
            decision.question = {
                "type": "combined",
                "text": (
                    f"행동도 불확실하고, '{obj_name}' 물체도 인식하지 못합니다. "
                    f"도와주세요."
                ),
                "options": ["VR 교정 시연", "음성/텍스트 안내", "무시 (계속 진행)"],
            }
            decision.details = (
                f"Action uncertainty: {unc_scalar:.4f}, "
                f"Unknown objects: {[o['name'] for o in uncertain_objects]}"
            )

        elif action_help:
            # 행동만 불확실 → VR 교정 요청
            decision.help_type = HelpRequestType.ACTION_CORRECTION
            decision.question = {
                "type": "action_correction",
                "text": "현재 동작이 불확실합니다. VR로 올바른 동작을 시연해 주세요.",
                "options": ["VR 교정 시작", "무시 (계속 진행)"],
            }
            decision.details = f"Action uncertainty: {unc_scalar:.4f}"

        elif object_uncertain and uncertain_objects:
            # 물체만 불확실 → 질문
            obj = uncertain_objects[0]
            if self.conformal is not None:
                vlm_probs = self._get_category_probs(obj, rgb_image)
                prediction_set, should_ask = self.conformal.predict(
                    obj["name"], vlm_probs
                )
                if should_ask:
                    decision.help_type = HelpRequestType.CATEGORY_SELECTION
                    decision.question = self.conformal.generate_question(
                        obj["name"], prediction_set
                    )
                # 확신이면 도움 불필요
            else:
                decision.help_type = HelpRequestType.OBJECT_IDENTIFICATION
                decision.question = {
                    "type": "identification",
                    "text": f"테이블 위의 '{obj['name']}' 물체가 무엇인가요?",
                    "options": ["주방용품", "식품", "기타", "무시"],
                }
            decision.details = (
                f"Unknown object: {obj['name']} "
                f"(conf: {obj.get('confidence', 0):.2f})"
            )

        else:
            # 확실 → 자율 실행
            decision.action = mean_action

        # 통계 갱신
        if decision.needs_help:
            self._help_decisions += 1

        self._decision_history.append(decision)

        return decision

    def _get_category_probs(
        self, obj_info: Dict, rgb_image: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """물체 정보에서 카테고리 확률 추출.

        우선순위: SigLIP2 (Tier 1) > VLM 캐시 > VLM 분석기 > 균등분포
        """
        # Tier 1: SigLIP2 실시간 분류 (가장 빠르고 정확)
        if self.siglip is not None and rgb_image is not None:
            return self.siglip.classify(rgb_image)

        # SigLIP2 캐시 결과 (CachedVLMAnalyzer에서 가져옴)
        if self.vlm is not None and hasattr(self.vlm, "get_siglip_probs"):
            probs = self.vlm.get_siglip_probs()
            if probs is not None:
                return probs

        # VLM 분석기에서 카테고리 확률 추출
        if self.vlm is not None and hasattr(self.vlm, "analyzer"):
            return self.vlm.analyzer.get_category_probabilities(obj_info)

        # 기본 uniform
        n_categories = len(
            self.conformal.categories if self.conformal else ["kitchen", "food", "misc"]
        )
        return np.ones(n_categories) / n_categories

    def process_human_response(
        self, decision: HelpDecision, response: str
    ):
        """사람의 응답 처리.

        Args:
            decision: 원래 도움 요청
            response: 사람의 응답
        """
        if decision.help_type == HelpRequestType.CATEGORY_SELECTION:
            # Conformal predictor 갱신
            if self.conformal is not None and decision.question:
                obj_name = decision.question.get("text", "").split("'")[1]
                vlm_probs = self._get_category_probs({"name": obj_name})
                self.conformal.update_with_feedback(obj_name, vlm_probs, response)

    def adapt_threshold(self):
        """적응적 임계값 조정."""
        self.threshold.adapt()

    @property
    def help_request_rate(self) -> float:
        if self._total_decisions == 0:
            return 0.0
        return self._help_decisions / self._total_decisions

    def get_stats(self) -> dict:
        """결정 통계."""
        type_counts = {}
        for d in self._decision_history:
            t = d.help_type.value
            type_counts[t] = type_counts.get(t, 0) + 1

        return {
            "total_decisions": self._total_decisions,
            "help_decisions": self._help_decisions,
            "help_request_rate": self.help_request_rate,
            "type_distribution": type_counts,
            "threshold": self.threshold.threshold,
        }
