"""MimicGen 데이터 증강기.

50개 원본 VR 시연 → 1,000+ 증강 궤적을 생성합니다.
각 물체 처리의 서브태스크:
  ST1: 대기 → 접근 (approach)
  ST2: 접근 → 파지 (grasp)
  ST3: 파지 → 이동 → 해제 (transport + release)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


@dataclass
class SubtaskDefinition:
    """서브태스크 정의."""

    name: str
    object_ref: str = "target_object"
    subtask_term_signal: Optional[str] = None
    selection_strategy: str = "nearest_neighbor_object"
    selection_strategy_kwargs: Dict = field(default_factory=lambda: {"nn_k": 3})
    action_noise: float = 0.003
    num_interpolation_steps: int = 5
    description: str = ""


# 기본 서브태스크 설정
KITCHEN_SORTING_SUBTASKS = {
    "right_arm": [
        SubtaskDefinition(
            name="approach",
            object_ref="target_object",
            subtask_term_signal="approach",
            selection_strategy="nearest_neighbor_object",
            selection_strategy_kwargs={"nn_k": 3},
            action_noise=0.003,
            num_interpolation_steps=5,
            description="Approach target object",
        ),
        SubtaskDefinition(
            name="grasp",
            object_ref="target_object",
            subtask_term_signal="grasp",
            action_noise=0.002,
            num_interpolation_steps=3,
            description="Grasp target object",
        ),
        SubtaskDefinition(
            name="transport_release",
            object_ref="target_object",
            subtask_term_signal=None,  # 마지막 서브태스크
            action_noise=0.003,
            num_interpolation_steps=5,
            description="Transport to bin and release",
        ),
    ],
    "left_arm": [
        SubtaskDefinition(
            name="approach",
            object_ref="target_object",
            subtask_term_signal="approach",
            selection_strategy="nearest_neighbor_object",
            selection_strategy_kwargs={"nn_k": 3},
            action_noise=0.003,
            num_interpolation_steps=5,
            description="Approach target object (left)",
        ),
        SubtaskDefinition(
            name="grasp",
            object_ref="target_object",
            subtask_term_signal="grasp",
            action_noise=0.002,
            num_interpolation_steps=3,
            description="Grasp target object (left)",
        ),
        SubtaskDefinition(
            name="transport_release",
            object_ref="target_object",
            subtask_term_signal=None,
            action_noise=0.003,
            num_interpolation_steps=5,
            description="Transport to bin and release (left)",
        ),
    ],
}


@dataclass
class DataGenConfig:
    """데이터 생성 설정."""

    name: str = "kitchen_sorting_augmented"
    num_trials: int = 1000
    max_num_failures: int = 50
    generation_guarantee: bool = True
    generation_relative: bool = True
    seed: int = 42
    target_num_trajectories: int = 1000


class MimicGenAugmentor:
    """MimicGen 기반 데이터 증강기.

    Isaac Lab의 MimicGen 인프라를 활용하여
    원본 VR 시연을 서브태스크 단위로 분할하고 재조합합니다.
    """

    def __init__(
        self,
        source_hdf5_path: str,
        output_hdf5_path: str,
        subtask_configs: Optional[Dict[str, List[SubtaskDefinition]]] = None,
        datagen_config: Optional[DataGenConfig] = None,
    ):
        """
        Args:
            source_hdf5_path: 원본 시연 HDF5 경로
            output_hdf5_path: 증강 결과 HDF5 경로
            subtask_configs: 서브태스크 설정 (기본: KITCHEN_SORTING_SUBTASKS)
            datagen_config: 데이터 생성 설정
        """
        self.source_path = Path(source_hdf5_path)
        self.output_path = Path(output_hdf5_path)
        self.subtask_configs = subtask_configs or KITCHEN_SORTING_SUBTASKS
        self.datagen_config = datagen_config or DataGenConfig()

        if not self.source_path.exists():
            raise FileNotFoundError(f"원본 데이터를 찾을 수 없습니다: {source_hdf5_path}")

        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def create_mimicgen_config(self) -> dict:
        """Isaac Lab MimicGen 설정 생성.

        Returns:
            MimicGen datagen config dict
        """
        subtask_configs_serialized = {}
        for arm, subtasks in self.subtask_configs.items():
            subtask_configs_serialized[arm] = []
            for st in subtasks:
                subtask_configs_serialized[arm].append({
                    "object_ref": st.object_ref,
                    "subtask_term_signal": st.subtask_term_signal,
                    "selection_strategy": st.selection_strategy,
                    "selection_strategy_kwargs": st.selection_strategy_kwargs,
                    "action_noise": st.action_noise,
                    "num_interpolation_steps": st.num_interpolation_steps,
                })

        config = {
            "name": self.datagen_config.name,
            "source_dataset": str(self.source_path),
            "output_dataset": str(self.output_path),
            "subtask_configs": subtask_configs_serialized,
            "generation": {
                "guarantee": self.datagen_config.generation_guarantee,
                "num_trials": self.datagen_config.num_trials,
                "relative": self.datagen_config.generation_relative,
                "max_num_failures": self.datagen_config.max_num_failures,
                "seed": self.datagen_config.seed,
            },
            "target_num_trajectories": self.datagen_config.target_num_trajectories,
        }

        return config

    def generate(self, env=None) -> str:
        """데이터 증강 실행.

        Isaac Lab MimicGen 파이프라인을 호출합니다.

        Args:
            env: Isaac Lab 환경 (MimicGen 데이터 생성에 필요)

        Returns:
            출력 HDF5 경로
        """
        config = self.create_mimicgen_config()

        try:
            from isaaclab_mimic.datagen import DataGenerator

            generator = DataGenerator(
                env=env,
                config=config,
            )

            print(f"[MimicGen] 데이터 증강 시작:")
            print(f"  원본: {self.source_path}")
            print(f"  목표 궤적 수: {self.datagen_config.target_num_trajectories}")
            print(f"  출력: {self.output_path}")

            generator.generate()

            print(f"[MimicGen] 증강 완료: {self.output_path}")

        except ImportError:
            print("[WARNING] isaaclab_mimic 미설치.")
            print("  Isaac Lab MimicGen 확장을 설치해주세요:")
            print("  pip install isaaclab-mimic")
            print()
            print("  대안: 수동으로 노이즈 주입 증강을 수행합니다.")
            self._fallback_augment()

        return str(self.output_path)

    def _fallback_augment(self):
        """MimicGen 미설치 시 간단한 노이즈 증강 대안."""
        import h5py

        if not self.source_path.exists():
            print(f"[ERROR] 원본 데이터 없음: {self.source_path}")
            return

        print("[Fallback] 노이즈 주입 기반 단순 증강...")

        with h5py.File(self.source_path, "r") as src:
            num_demos = src.attrs.get("num_episodes", len(src["data"].keys()))
            demo_keys = sorted(src["data"].keys())

            augmented_episodes = []
            for demo_key in demo_keys:
                demo = src["data"][demo_key]
                obs = demo["obs"][:]
                actions = demo["actions"][:]

                # 원본 유지
                augmented_episodes.append({"obs": obs, "actions": actions})

                # 노이즈 증강 (원본당 약 20배)
                target_per_demo = max(
                    1,
                    self.datagen_config.target_num_trajectories // num_demos,
                )
                for aug_idx in range(target_per_demo - 1):
                    rng = np.random.RandomState(
                        self.datagen_config.seed + aug_idx
                    )
                    noise_scale = 0.003 * (1 + aug_idx * 0.001)
                    action_noise = rng.randn(*actions.shape).astype(
                        np.float32
                    ) * noise_scale
                    augmented_episodes.append({
                        "obs": obs.copy(),
                        "actions": (actions + action_noise).astype(np.float32),
                    })

        # 저장
        with h5py.File(self.output_path, "w") as dst:
            dst.attrs["num_episodes"] = len(augmented_episodes)
            dst.attrs["source"] = str(self.source_path)
            dst.attrs["augmentation"] = "noise_injection_fallback"

            data_grp = dst.create_group("data")
            for i, ep in enumerate(augmented_episodes):
                ep_grp = data_grp.create_group(f"demo_{i}")
                ep_grp.create_dataset(
                    "obs", data=ep["obs"], compression="gzip"
                )
                ep_grp.create_dataset(
                    "actions", data=ep["actions"], compression="gzip"
                )

            # 학습/검증 분할
            n = len(augmented_episodes)
            indices = np.random.permutation(n)
            n_train = int(n * 0.8)
            mask_grp = dst.create_group("mask")
            train_mask = np.zeros(n, dtype=bool)
            valid_mask = np.zeros(n, dtype=bool)
            train_mask[indices[:n_train]] = True
            valid_mask[indices[n_train:]] = True
            mask_grp.create_dataset("train", data=train_mask)
            mask_grp.create_dataset("valid", data=valid_mask)

        print(
            f"[Fallback] {num_demos}개 원본 → "
            f"{len(augmented_episodes)}개 증강 궤적 생성"
        )

    def get_statistics(self) -> dict:
        """증강 데이터 통계 반환."""
        import h5py

        if not self.output_path.exists():
            return {"error": "증강 데이터 없음"}

        with h5py.File(self.output_path, "r") as f:
            num_demos = f.attrs.get("num_episodes", 0)
            demo_keys = sorted(f["data"].keys())
            lengths = []
            for key in demo_keys:
                lengths.append(len(f["data"][key]["actions"]))

        return {
            "num_demos": num_demos,
            "avg_length": float(np.mean(lengths)) if lengths else 0,
            "total_steps": int(np.sum(lengths)) if lengths else 0,
            "min_length": int(np.min(lengths)) if lengths else 0,
            "max_length": int(np.max(lengths)) if lengths else 0,
        }
