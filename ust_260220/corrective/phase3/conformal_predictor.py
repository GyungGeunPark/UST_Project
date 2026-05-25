"""KnowNo 스타일 Conformal Prediction 기반 카테고리 예측.

VLM의 카테고리 예측에 형식적 보장을 추가합니다:
  P(올바른 카테고리 ∈ 예측 집합) ≥ 1 - α

예측 집합 크기가 1이면 자율 결정, >1이면 사람에게 질문합니다.
"""

from __future__ import annotations

import numpy as np
from typing import Dict, List, Optional, Tuple


class ConformalCategoryPredictor:
    """KnowNo 스타일 conformal prediction 기반 카테고리 예측.

    사용법:
        predictor = ConformalCategoryPredictor(alpha=0.05)
        predictor.calibrate(calibration_data)
        pred_set, should_ask = predictor.predict("unknown_object", vlm_probs)
        if should_ask:
            question = predictor.generate_question("unknown_object", pred_set)
    """

    def __init__(
        self,
        categories: Optional[List[str]] = None,
        alpha: float = 0.05,
    ):
        """
        Args:
            categories: 카테고리 목록 (기본: kitchen, food, misc)
            alpha: 오류율 (5% = 95% 커버리지 보장)
        """
        self.categories = categories or ["kitchen", "food", "misc"]
        self.alpha = alpha
        self.threshold: float = 0.0
        self.calibration_scores: List[float] = []
        self._is_calibrated = False

    def calibrate(self, calibration_data: List[Tuple[np.ndarray, int]]):
        """교정 데이터로 conformal 임계값 설정.

        Args:
            calibration_data: [(softmax_probs, true_label_index), ...]
                softmax_probs: (num_categories,) 확률
                true_label_index: 올바른 카테고리 인덱스
        """
        if not calibration_data:
            print("[Conformal] 교정 데이터 없음. 기본 임계값 사용.")
            self.threshold = 0.5
            self._is_calibrated = True
            return

        # 비적합 점수(nonconformity score) 계산
        scores = []
        for probs, true_label in calibration_data:
            probs = np.asarray(probs, dtype=np.float64)
            if true_label < 0 or true_label >= len(probs):
                continue
            score = 1.0 - probs[true_label]
            scores.append(score)

        if not scores:
            self.threshold = 0.5
            self._is_calibrated = True
            return

        self.calibration_scores = sorted(scores)

        # Conformal 임계값: (1-α)(n+1)/n 분위수
        n = len(scores)
        quantile_index = int(np.ceil((1 - self.alpha) * (n + 1))) - 1
        quantile_index = min(quantile_index, n - 1)
        quantile_index = max(quantile_index, 0)
        self.threshold = self.calibration_scores[quantile_index]
        self._is_calibrated = True

        print(
            f"[Conformal] 교정 완료: {n}개 샘플, "
            f"α={self.alpha}, 임계값={self.threshold:.4f}"
        )

    def predict(
        self, object_name: str, vlm_probs: np.ndarray
    ) -> Tuple[List[str], bool]:
        """Conformal prediction으로 카테고리 예측.

        Args:
            object_name: 물체 이름
            vlm_probs: VLM이 예측한 카테고리 확률 (softmax)

        Returns:
            prediction_set: 가능한 카테고리 집합
            should_ask: 사람에게 질문 여부
        """
        if not self._is_calibrated:
            # 미교정 시 보수적으로 전체 집합 반환
            return self.categories.copy(), True

        vlm_probs = np.asarray(vlm_probs, dtype=np.float64)

        # 예측 집합 구성: nonconformity ≤ threshold인 카테고리 포함
        prediction_set = []
        for i, category in enumerate(self.categories):
            if i < len(vlm_probs):
                nonconformity = 1.0 - vlm_probs[i]
                if nonconformity <= self.threshold:
                    prediction_set.append(category)

        # 빈 집합이면 전체 집합으로
        if not prediction_set:
            prediction_set = self.categories.copy()

        # 도움 요청 판단: 집합 크기가 1이면 확신, >1이면 질문
        should_ask = len(prediction_set) != 1

        if should_ask:
            print(
                f"[KnowNo] '{object_name}': 불확실 → "
                f"예측 집합 = {prediction_set}"
            )
        else:
            print(
                f"[KnowNo] '{object_name}': 확신 → "
                f"카테고리 = {prediction_set[0]}"
            )

        return prediction_set, should_ask

    def generate_question(
        self, object_name: str, prediction_set: List[str]
    ) -> dict:
        """사람에게 보여줄 질문 생성.

        Args:
            object_name: 물체 이름
            prediction_set: 가능한 카테고리 집합

        Returns:
            질문 딕셔너리 {"type", "text", "options"}
        """
        if len(prediction_set) == len(self.categories):
            # 완전 불확실
            return {
                "type": "identification",
                "text": f"'{object_name}'이(가) 무엇인가요? 어디에 넣어야 하나요?",
                "options": self.categories + ["기타"],
            }
        else:
            # 부분 불확실
            return {
                "type": "selection",
                "text": f"'{object_name}'은(는) 어느 카테고리에 해당하나요?",
                "options": prediction_set,
            }

    def update_with_feedback(
        self, object_name: str, vlm_probs: np.ndarray, true_category: str
    ):
        """사람의 피드백으로 교정 데이터 갱신.

        Args:
            object_name: 물체 이름
            vlm_probs: VLM 확률
            true_category: 올바른 카테고리 (사람 응답)
        """
        if true_category not in self.categories:
            return

        true_idx = self.categories.index(true_category)
        vlm_probs = np.asarray(vlm_probs, dtype=np.float64)
        score = 1.0 - vlm_probs[true_idx]

        self.calibration_scores.append(score)
        self.calibration_scores.sort()

        # 임계값 재계산
        n = len(self.calibration_scores)
        quantile_index = int(np.ceil((1 - self.alpha) * (n + 1))) - 1
        quantile_index = min(quantile_index, n - 1)
        quantile_index = max(quantile_index, 0)
        self.threshold = self.calibration_scores[quantile_index]

    def get_stats(self) -> dict:
        """통계."""
        return {
            "is_calibrated": self._is_calibrated,
            "threshold": self.threshold,
            "alpha": self.alpha,
            "num_calibration_samples": len(self.calibration_scores),
            "categories": self.categories,
        }
