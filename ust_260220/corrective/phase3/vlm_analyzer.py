"""VLM 기반 물체 인식 및 불확실성 추정.

3-Tier 추론 아키텍처 (RTX PRO 6000 최적화):
  Tier 1: SigLIP2 빠른 분류 (<10ms, 매 스텝)
  Tier 2: Florence-2 물체 탐지 (<50ms, 30스텝마다)
  Tier 3: Qwen3-VL VLM 상세 분석 (~1-2초, 불확실 시만)

모델 경로: /workspace/isaaclab/ust_ws/models/
로컬 VLM 서빙: SGLang/vLLM OpenAI-compatible API (http://localhost:8000/v1)
"""

from __future__ import annotations

import base64
import json
import os
import time
import numpy as np
from typing import Dict, List, Optional, Tuple


# 모델 기본 경로
DEFAULT_MODELS_DIR = os.environ.get(
    "UST_MODELS_DIR",
    "/workspace/isaaclab/ust_ws/models",
)

# 물체 카테고리 정의
KNOWN_CATEGORIES = {
    "kitchen": ["mug", "plate", "bowl", "cup", "spoon", "fork", "knife"],
    "food": ["can", "bottle", "apple", "banana", "box", "container"],
    "misc": ["sponge", "cloth", "remote", "pen", "toy", "teddy"],
}

# SigLIP2 카테고리 텍스트 (제로샷 분류용)
SIGLIP_CATEGORY_TEXTS = [
    "a kitchen utensil such as mug, plate, bowl, cup, spoon, fork or knife",
    "a food item such as can, bottle, apple, banana, box or container",
    "a miscellaneous object such as sponge, cloth, remote, pen, toy or teddy bear",
]


class SigLIP2Classifier:
    """SigLIP2 기반 빠른 물체 분류기 (Tier 1).

    제로샷 이미지-텍스트 유사도로 카테고리 확률을 계산합니다.
    VRAM: ~1.5-2.0 GB, 추론: <10ms/이미지

    사용법:
        classifier = SigLIP2Classifier()
        probs = classifier.classify(rgb_image)
        # probs: [kitchen, food, misc] 확률 벡터
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        category_texts: Optional[List[str]] = None,
        device: str = "cuda",
    ):
        self.model_path = model_path or os.path.join(
            DEFAULT_MODELS_DIR, "siglip2-so400m"
        )
        self.category_texts = category_texts or SIGLIP_CATEGORY_TEXTS
        self.device = device
        self.model = None
        self.processor = None
        self._loaded = False

    def load(self):
        """모델 로드 (lazy loading)."""
        if self._loaded:
            return

        try:
            import torch
            from transformers import AutoModel, AutoProcessor

            self.processor = AutoProcessor.from_pretrained(self.model_path)
            self.model = AutoModel.from_pretrained(self.model_path).to(self.device)
            self.model.eval()
            self._loaded = True
            print(f"[SigLIP2] 모델 로드 완료: {self.model_path}")
        except Exception as e:
            print(f"[SigLIP2] 모델 로드 실패: {e}")
            print("  → 모의 분류기로 대체합니다.")

    def classify(self, rgb_image: np.ndarray) -> np.ndarray:
        """이미지를 카테고리로 분류.

        Args:
            rgb_image: RGB 이미지 (H, W, 3) uint8

        Returns:
            probs: (3,) 확률 벡터 [kitchen, food, misc]
        """
        if not self._loaded:
            self.load()

        if self.model is None:
            return self._mock_classify()

        try:
            import torch
            from PIL import Image

            img = Image.fromarray(rgb_image.astype(np.uint8))
            inputs = self.processor(
                text=self.category_texts,
                images=img,
                return_tensors="pt",
                padding=True,
            ).to(self.device)

            with torch.no_grad():
                outputs = self.model(**inputs)
                # SigLIP2는 sigmoid 기반 (softmax 아님)
                logits = outputs.logits_per_image[0]
                probs = torch.sigmoid(logits).cpu().numpy()

            # 정규화 (합=1)
            probs = probs / (probs.sum() + 1e-8)
            return probs.astype(np.float64)

        except Exception as e:
            print(f"[SigLIP2] 분류 오류: {e}")
            return self._mock_classify()

    def _mock_classify(self) -> np.ndarray:
        """모의 분류 결과."""
        return np.array([0.6, 0.25, 0.15], dtype=np.float64)

    @property
    def is_loaded(self) -> bool:
        return self._loaded


class Florence2Detector:
    """Florence-2 기반 물체 탐지기 (Tier 2).

    바운딩박스 + 물체 레이블을 반환합니다.
    VRAM: ~3.0-4.5 GB, 추론: <50ms/이미지

    사용법:
        detector = Florence2Detector()
        detections = detector.detect(rgb_image)
        # detections: [{"label": "mug", "bbox": [x1,y1,x2,y2], "score": 0.95}, ...]
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        device: str = "cuda",
    ):
        self.model_path = model_path or os.path.join(
            DEFAULT_MODELS_DIR, "florence2-large"
        )
        self.device = device
        self.model = None
        self.processor = None
        self._loaded = False

    def load(self):
        """모델 로드 (lazy loading)."""
        if self._loaded:
            return

        try:
            import torch
            from transformers import AutoModelForCausalLM, AutoProcessor

            self.processor = AutoProcessor.from_pretrained(
                self.model_path, trust_remote_code=True
            )
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_path, trust_remote_code=True, torch_dtype=torch.float16
            ).to(self.device)
            self.model.eval()
            self._loaded = True
            print(f"[Florence-2] 모델 로드 완료: {self.model_path}")
        except Exception as e:
            print(f"[Florence-2] 모델 로드 실패: {e}")
            print("  → 모의 탐지기로 대체합니다.")

    def detect(self, rgb_image: np.ndarray) -> List[Dict]:
        """물체 탐지.

        Args:
            rgb_image: RGB 이미지 (H, W, 3) uint8

        Returns:
            탐지 결과 리스트 [{"label": str, "bbox": [x1,y1,x2,y2], "score": float}, ...]
        """
        if not self._loaded:
            self.load()

        if self.model is None:
            return self._mock_detect()

        try:
            import torch
            from PIL import Image

            img = Image.fromarray(rgb_image.astype(np.uint8))

            # Florence-2 Object Detection 프롬프트
            prompt = "<OD>"
            inputs = self.processor(text=prompt, images=img, return_tensors="pt").to(
                self.device, torch.float16
            )

            with torch.no_grad():
                generated_ids = self.model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=1024,
                    num_beams=3,
                )
                generated_text = self.processor.batch_decode(
                    generated_ids, skip_special_tokens=False
                )[0]

            # Florence-2 후처리
            result = self.processor.post_process_generation(
                generated_text, task="<OD>", image_size=img.size
            )

            detections = []
            if "<OD>" in result:
                od_result = result["<OD>"]
                labels = od_result.get("labels", [])
                bboxes = od_result.get("bboxes", [])
                for label, bbox in zip(labels, bboxes):
                    detections.append({
                        "label": label,
                        "bbox": [float(x) for x in bbox],
                        "score": 1.0,  # Florence-2는 confidence 미제공
                    })

            return detections

        except Exception as e:
            print(f"[Florence-2] 탐지 오류: {e}")
            return self._mock_detect()

    def _mock_detect(self) -> List[Dict]:
        """모의 탐지 결과."""
        return [
            {"label": "mug", "bbox": [100, 200, 200, 350], "score": 0.95},
            {"label": "plate", "bbox": [300, 150, 500, 300], "score": 0.90},
            {"label": "can", "bbox": [50, 300, 150, 450], "score": 0.85},
        ]

    @property
    def is_loaded(self) -> bool:
        return self._loaded


class VLMObjectAnalyzer:
    """VLM 기반 물체 인식 및 불확실성 추정 (Tier 3).

    로컬 SGLang/vLLM 서버 또는 API를 통해 상세 물체 분석을 수행합니다.
    기본 설정: 로컬 Qwen3-VL (OpenAI-compatible API)

    사용법:
        # 로컬 VLM (SGLang/vLLM)
        analyzer = VLMObjectAnalyzer(
            model="Qwen/Qwen3-VL-32B-Instruct",
            base_url="http://localhost:8000/v1",
        )

        # 클라우드 API (OpenAI)
        analyzer = VLMObjectAnalyzer(
            model="gpt-4o",
            base_url="https://api.openai.com/v1",
            api_key="sk-...",
        )
    """

    def __init__(
        self,
        model: str = "Qwen/Qwen3-VL-32B-Instruct",
        api_key: Optional[str] = None,
        base_url: str = "http://localhost:8000/v1",
        confidence_threshold: float = 0.7,
        known_categories: Optional[dict] = None,
        provider: str = "openai",
    ):
        """
        Args:
            model: VLM 모델 이름 (로컬: Qwen/Qwen3-VL-32B-Instruct)
            api_key: API 키 (로컬이면 "EMPTY" 또는 None)
            base_url: API 엔드포인트 (로컬: http://localhost:8000/v1)
            confidence_threshold: 확신도 임계값
            known_categories: 알려진 카테고리 딕셔너리
            provider: API 제공자 ("openai" 또는 "google")
        """
        self.model = model
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY", "EMPTY")
        self.base_url = base_url
        self.confidence_threshold = confidence_threshold
        self.known_categories = known_categories or KNOWN_CATEGORIES
        self.provider = provider

        # 인식 캐시
        self.recognition_cache: Dict[str, Dict] = {}
        self._call_count = 0
        self._total_latency = 0.0

    def analyze_scene(
        self,
        rgb_image: np.ndarray,
        semantic_mask: Optional[np.ndarray] = None,
    ) -> Dict[str, Dict]:
        """씬 분석: 물체 인식 + 카테고리 분류 + 신뢰도.

        Args:
            rgb_image: RGB 이미지 (H, W, 3) uint8
            semantic_mask: 시맨틱 마스크 (선택)

        Returns:
            {
                "object_0": {
                    "name": "mug",
                    "category": "kitchen",
                    "confidence": 0.95,
                    "is_known": True,
                    "description": "파란색 머그컵",
                },
                ...
            }
        """
        image_b64 = self._encode_image(rgb_image)
        prompt = self._build_prompt()

        start_time = time.time()
        response = self._call_vlm(image_b64, prompt)
        latency = time.time() - start_time

        self._call_count += 1
        self._total_latency += latency

        result = self._parse_response(response)
        return result

    def get_uncertain_objects(self, analysis: Dict[str, Dict]) -> List[Dict]:
        """불확실한 물체 목록 추출."""
        uncertain = []
        for obj_id, obj_info in analysis.items():
            if (
                obj_info.get("confidence", 1.0) < self.confidence_threshold
                or obj_info.get("category", "") == "unknown"
            ):
                uncertain.append({
                    "id": obj_id,
                    **obj_info,
                    "question_type": self._determine_question_type(obj_info),
                })
        return uncertain

    def get_category_probabilities(
        self, obj_info: Dict
    ) -> np.ndarray:
        """물체의 카테고리 확률 분포 추출.

        Returns:
            (3,) softmax 확률 [kitchen, food, misc]
        """
        category = obj_info.get("category", "unknown")
        confidence = obj_info.get("confidence", 0.33)

        categories = list(self.known_categories.keys())
        probs = np.ones(len(categories)) / len(categories)

        if category in categories:
            idx = categories.index(category)
            probs[idx] = confidence
            remaining = (1 - confidence) / (len(categories) - 1)
            for i in range(len(categories)):
                if i != idx:
                    probs[i] = remaining

        return probs

    def _determine_question_type(self, obj_info: Dict) -> str:
        """질문 유형 결정."""
        category = obj_info.get("category", "unknown")
        confidence = obj_info.get("confidence", 0.0)

        if category == "unknown":
            return "identification"
        elif confidence < 0.5:
            return "confirmation"
        else:
            return "categorization"

    def _encode_image(self, rgb_image: np.ndarray) -> str:
        """이미지를 base64로 인코딩."""
        import io

        try:
            from PIL import Image

            img = Image.fromarray(rgb_image.astype(np.uint8))
            buffer = io.BytesIO()
            img.save(buffer, format="PNG")
            return base64.b64encode(buffer.getvalue()).decode("utf-8")
        except ImportError:
            return base64.b64encode(rgb_image.tobytes()).decode("utf-8")

    def _build_prompt(self) -> str:
        """VLM 프롬프트 생성."""
        categories_str = ""
        for cat, items in self.known_categories.items():
            categories_str += f"- {cat}: {', '.join(items)}\n"

        return f"""당신은 주방 테이블 위 물체를 인식하는 AI입니다.

이미지에서 보이는 모든 물체를 식별하고 다음 JSON 형식으로 응답하세요:

{{
  "objects": [
    {{
      "name": "물체 이름 (영어)",
      "category": "kitchen" | "food" | "misc" | "unknown",
      "confidence": 0.0~1.0 (인식 확신도),
      "description": "물체 설명"
    }}
  ]
}}

알려진 카테고리:
{categories_str}
모르는 물체는 category="unknown", confidence를 낮게 설정하세요."""

    def _call_vlm(self, image_b64: str, prompt: str) -> str:
        """VLM API 호출."""
        if self.provider == "openai":
            return self._call_openai(image_b64, prompt)
        elif self.provider == "google":
            return self._call_google(image_b64, prompt)
        else:
            return self._mock_response()

    def _call_openai(self, image_b64: str, prompt: str) -> str:
        """OpenAI-compatible API 호출 (로컬 SGLang/vLLM 또는 OpenAI)."""
        try:
            import openai

            client = openai.OpenAI(
                api_key=self.api_key,
                base_url=self.base_url,
            )
            response = client.chat.completions.create(
                model=self.model,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/png;base64,{image_b64}",
                            },
                        },
                    ],
                }],
                max_tokens=1000,
            )
            return response.choices[0].message.content
        except Exception as e:
            print(f"[VLM] API 오류 ({self.base_url}): {e}")
            return self._mock_response()

    def _call_google(self, image_b64: str, prompt: str) -> str:
        """Google Gemini API 호출."""
        try:
            import google.generativeai as genai

            genai.configure(api_key=self.api_key)
            model = genai.GenerativeModel(self.model)

            image_bytes = base64.b64decode(image_b64)
            response = model.generate_content([
                prompt,
                {"mime_type": "image/png", "data": image_bytes},
            ])
            return response.text
        except Exception as e:
            print(f"[VLM] Google API 오류: {e}")
            return self._mock_response()

    def _mock_response(self) -> str:
        """API 미사용 시 모의 응답."""
        return json.dumps({
            "objects": [
                {"name": "mug", "category": "kitchen", "confidence": 0.95,
                 "description": "ceramic mug"},
                {"name": "plate", "category": "kitchen", "confidence": 0.90,
                 "description": "white plate"},
                {"name": "can", "category": "food", "confidence": 0.85,
                 "description": "aluminum can"},
                {"name": "unknown_item", "category": "unknown", "confidence": 0.30,
                 "description": "unidentified small object"},
            ]
        })

    def _parse_response(self, response: str) -> Dict[str, Dict]:
        """VLM 응답 파싱."""
        try:
            if "```json" in response:
                start = response.index("```json") + 7
                end = response.index("```", start)
                response = response[start:end].strip()
            elif "```" in response:
                start = response.index("```") + 3
                end = response.index("```", start)
                response = response[start:end].strip()

            data = json.loads(response)
            result = {}
            for i, obj in enumerate(data.get("objects", [])):
                category = obj.get("category", "unknown")
                is_known = category != "unknown"
                result[f"object_{i}"] = {
                    "name": obj.get("name", f"object_{i}"),
                    "category": category,
                    "confidence": float(obj.get("confidence", 0.5)),
                    "description": obj.get("description", ""),
                    "is_known": is_known,
                }
            return result
        except (json.JSONDecodeError, ValueError) as e:
            print(f"[VLM] 응답 파싱 오류: {e}")
            return {}

    def get_stats(self) -> dict:
        """VLM 호출 통계."""
        return {
            "model": self.model,
            "base_url": self.base_url,
            "call_count": self._call_count,
            "total_latency": self._total_latency,
            "avg_latency": (
                self._total_latency / self._call_count
                if self._call_count > 0
                else 0.0
            ),
        }


class CachedVLMAnalyzer:
    """캐시 및 배치 처리로 VLM 호출 최적화.

    3-Tier 아키텍처:
      - Tier 1 (SigLIP2): 매 스텝 (<10ms)
      - Tier 2 (Florence-2): tier2_interval 스텝마다 (<50ms)
      - Tier 3 (VLM): call_interval 스텝마다 또는 불확실 시 (~1-2초)
    """

    def __init__(
        self,
        base_analyzer: VLMObjectAnalyzer,
        call_interval: int = 30,
        siglip_classifier: Optional[SigLIP2Classifier] = None,
        florence_detector: Optional[Florence2Detector] = None,
        tier2_interval: int = 30,
    ):
        """
        Args:
            base_analyzer: VLM 분석기 (Tier 3)
            call_interval: VLM 호출 간격 (스텝). 30 = 1.5초@20Hz
            siglip_classifier: SigLIP2 분류기 (Tier 1, 선택)
            florence_detector: Florence-2 탐지기 (Tier 2, 선택)
            tier2_interval: Florence-2 호출 간격 (스텝)
        """
        self.analyzer = base_analyzer
        self.call_interval = call_interval
        self.siglip = siglip_classifier
        self.florence = florence_detector
        self.tier2_interval = tier2_interval

        self.cache: Dict[str, Dict] = {}
        self.step_count = 0
        self.last_call_step = -call_interval
        self.last_tier2_step = -tier2_interval

        # Tier 1 최신 분류 결과
        self.latest_siglip_probs: Optional[np.ndarray] = None
        # Tier 2 최신 탐지 결과
        self.latest_detections: List[Dict] = []

    def step(self, rgb_image: Optional[np.ndarray] = None) -> Dict[str, Dict]:
        """매 스텝 호출 (3-Tier 계층적 추론).

        Args:
            rgb_image: RGB 이미지 (없으면 캐시 반환)

        Returns:
            물체 분석 결과 (캐시 또는 새로운)
        """
        self.step_count += 1

        if rgb_image is None:
            return self.cache

        # Tier 1: SigLIP2 빠른 분류 (매 스텝)
        if self.siglip is not None:
            self.latest_siglip_probs = self.siglip.classify(rgb_image)

        # Tier 2: Florence-2 물체 탐지 (주기적)
        if (
            self.florence is not None
            and (self.step_count - self.last_tier2_step) >= self.tier2_interval
        ):
            self.latest_detections = self.florence.detect(rgb_image)
            self.last_tier2_step = self.step_count

        # Tier 3: VLM 상세 분석 (주기적)
        if (self.step_count - self.last_call_step) >= self.call_interval:
            self.cache = self.analyzer.analyze_scene(rgb_image)
            self.last_call_step = self.step_count

        return self.cache

    def get_uncertain_objects(self) -> List[Dict]:
        """캐시된 결과에서 불확실한 물체 추출."""
        return self.analyzer.get_uncertain_objects(self.cache)

    def get_siglip_probs(self) -> Optional[np.ndarray]:
        """최신 SigLIP2 카테고리 확률."""
        return self.latest_siglip_probs

    def get_detections(self) -> List[Dict]:
        """최신 Florence-2 탐지 결과."""
        return self.latest_detections

    def reset(self):
        """캐시 초기화."""
        self.cache = {}
        self.step_count = 0
        self.last_call_step = -self.call_interval
        self.last_tier2_step = -self.tier2_interval
        self.latest_siglip_probs = None
        self.latest_detections = []
