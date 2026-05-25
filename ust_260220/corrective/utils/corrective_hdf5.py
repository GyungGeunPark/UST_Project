"""교정 학습용 확장 HDF5 레코더.

Phase 1 기본 HDF5 구조에 Phase 2/3 전용 필드를 추가합니다:
- intervention_labels: 자율/개입 구분 (Phase 2)
- intervention_ids: 개입 세션 ID (Phase 2)
- uncertainty_scores: 불확실성 점수 (Phase 3)
- help_request_types: 도움 요청 유형 (Phase 3)

HDF5 구조:
    /data/demo_i/
        obs, actions, dones, timestamps, rewards  (Phase 1)
        intervention_labels  (T,) int   # 0=자율, 1=개입 (Phase 2)
        intervention_ids     (T,) int   # 개입 세션 번호 (Phase 2)
        uncertainty_scores   (T,) float # 앙상블 불확실성 (Phase 3)
        help_request_types   (T,) int   # 0=없음, 1=액션, 2=물체, 3=카테고리 (Phase 3)
    /mask/train, /mask/valid
"""

from __future__ import annotations

import h5py
import numpy as np
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional
import json


# 개입 라벨 상수
LABEL_AUTONOMOUS = 0
LABEL_INTERVENTION = 1

# 도움 요청 유형 상수
HELP_NONE = 0
HELP_ACTION_CORRECTION = 1
HELP_OBJECT_IDENTIFY = 2
HELP_CATEGORY_SELECT = 3


class CorrectiveHDF5Recorder:
    """Phase 2/3 교정 학습용 확장 HDF5 레코더.

    Phase 1 HDF5DatasetRecorder의 기능을 포함하면서,
    개입 라벨, 불확실성 점수, 도움 요청 유형을 추가 저장합니다.
    """

    def __init__(
        self,
        file_path: str,
        action_dim: int = 38,
        include_images: bool = False,
        image_shape: tuple = (720, 1280, 3),
        compression: str = "gzip",
        compression_opts: int = 4,
        phase: int = 2,
    ):
        self.file_path = Path(file_path)
        self.file_path.parent.mkdir(parents=True, exist_ok=True)

        self.action_dim = action_dim
        self.include_images = include_images
        self.image_shape = image_shape
        self.compression = compression
        self.compression_opts = compression_opts
        self.phase = phase

        self.episodes: List[Dict[str, Any]] = []
        self.current_episode: Optional[Dict[str, List]] = None
        self._total_steps = 0
        self._total_intervention_steps = 0
        self._start_time = None

    def start_episode(self):
        """새 에피소드 시작."""
        if self.current_episode is not None:
            self.end_episode(success=False)

        self.current_episode = {
            "obs": [],
            "actions": [],
            "dones": [],
            "timestamps": [],
            "rewards": [],
            # Phase 2 필드
            "intervention_labels": [],
            "intervention_ids": [],
            # Phase 3 필드
            "uncertainty_scores": [],
            "help_request_types": [],
        }

        if self.include_images:
            self.current_episode["images_rgb"] = []
            self.current_episode["images_depth"] = []

        self._start_time = datetime.now()
        self._current_intervention_id = -1

    def add_step(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        done: bool = False,
        reward: float = 0.0,
        is_intervention: bool = False,
        intervention_id: int = -1,
        uncertainty_score: float = 0.0,
        help_request_type: int = HELP_NONE,
        rgb_image: Optional[np.ndarray] = None,
        depth_image: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ):
        """스텝 데이터 추가.

        Args:
            obs: 관측 벡터
            action: 38D 액션 벡터
            done: 종료 플래그
            reward: 보상
            is_intervention: 인간 개입 중 여부 (Phase 2)
            intervention_id: 개입 세션 ID (Phase 2)
            uncertainty_score: 앙상블 불확실성 (Phase 3)
            help_request_type: 도움 요청 유형 (Phase 3)
            rgb_image: 시뮬레이션 카메라 RGB
            depth_image: 시뮬레이션 카메라 깊이
            timestamp: 타임스탬프
        """
        if self.current_episode is None:
            raise RuntimeError("에피소드가 시작되지 않았습니다. start_episode()를 먼저 호출하세요.")

        obs = np.asarray(obs, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32)

        self.current_episode["obs"].append(obs.copy())
        self.current_episode["actions"].append(action.copy())
        self.current_episode["dones"].append(done)
        self.current_episode["rewards"].append(reward)
        self.current_episode["timestamps"].append(timestamp or datetime.now().timestamp())

        # Phase 2: 개입 라벨
        label = LABEL_INTERVENTION if is_intervention else LABEL_AUTONOMOUS
        self.current_episode["intervention_labels"].append(label)
        self.current_episode["intervention_ids"].append(intervention_id)

        # Phase 3: 불확실성
        self.current_episode["uncertainty_scores"].append(uncertainty_score)
        self.current_episode["help_request_types"].append(help_request_type)

        if self.include_images:
            if rgb_image is not None:
                self.current_episode["images_rgb"].append(
                    np.asarray(rgb_image, dtype=np.uint8).copy()
                )
            if depth_image is not None:
                self.current_episode["images_depth"].append(
                    np.asarray(depth_image, dtype=np.float32).copy()
                )

        self._total_steps += 1
        if is_intervention:
            self._total_intervention_steps += 1

    def end_episode(self, success: bool = False):
        """에피소드 종료."""
        if self.current_episode is None:
            return

        if len(self.current_episode["obs"]) == 0:
            self.current_episode = None
            return

        ep_labels = self.current_episode["intervention_labels"]
        n_intervention = sum(1 for l in ep_labels if l == LABEL_INTERVENTION)

        self.current_episode["success"] = success
        self.current_episode["length"] = len(self.current_episode["obs"])
        self.current_episode["duration"] = (datetime.now() - self._start_time).total_seconds()
        self.current_episode["intervention_ratio"] = n_intervention / len(ep_labels) if ep_labels else 0.0
        self.current_episode["num_interventions"] = len(set(
            iid for iid in self.current_episode["intervention_ids"] if iid >= 0
        ))

        self.episodes.append(self.current_episode)
        self.current_episode = None

        ep = self.episodes[-1]
        print(f"[CorrectiveRecorder] 에피소드 {len(self.episodes)-1} 종료. "
              f"길이: {ep['length']}, 성공: {success}, "
              f"개입비율: {ep['intervention_ratio']:.1%}")

    def save(self, train_split: float = 0.8):
        """HDF5 파일 저장."""
        if not self.episodes:
            print("[WARNING] 저장할 에피소드가 없습니다.")
            return

        print(f"[CorrectiveRecorder] {len(self.episodes)}개 에피소드 저장 중: {self.file_path}")

        with h5py.File(self.file_path, 'w') as f:
            f.attrs["num_episodes"] = len(self.episodes)
            f.attrs["total_steps"] = self._total_steps
            f.attrs["total_intervention_steps"] = self._total_intervention_steps
            f.attrs["creation_time"] = datetime.now().isoformat()
            f.attrs["action_dim"] = self.action_dim
            f.attrs["phase"] = self.phase

            env_args = {
                "env_name": "Isaac-KitchenSorting-G1-InspireFTP-v0",
                "robot": "unitree_g1_inspire_ftp",
                "ik_controller": "pink_ik",
                "action_dim": self.action_dim,
                "corrective_phase": self.phase,
            }
            f.attrs["env_args"] = json.dumps(env_args)

            data_grp = f.create_group("data")
            for i, ep in enumerate(self.episodes):
                ep_grp = data_grp.create_group(f"demo_{i}")

                for key in ["obs", "actions"]:
                    ep_grp.create_dataset(
                        key, data=np.array(ep[key], dtype=np.float32),
                        compression=self.compression, compression_opts=self.compression_opts,
                    )
                ep_grp.create_dataset("dones", data=np.array(ep["dones"], dtype=bool))
                ep_grp.create_dataset("timestamps", data=np.array(ep["timestamps"], dtype=np.float64))
                ep_grp.create_dataset("rewards", data=np.array(ep["rewards"], dtype=np.float32))

                # Phase 2/3 필드
                ep_grp.create_dataset("intervention_labels",
                                      data=np.array(ep["intervention_labels"], dtype=np.int32))
                ep_grp.create_dataset("intervention_ids",
                                      data=np.array(ep["intervention_ids"], dtype=np.int32))
                ep_grp.create_dataset("uncertainty_scores",
                                      data=np.array(ep["uncertainty_scores"], dtype=np.float32))
                ep_grp.create_dataset("help_request_types",
                                      data=np.array(ep["help_request_types"], dtype=np.int32))

                ep_grp.attrs["success"] = ep.get("success", False)
                ep_grp.attrs["length"] = ep["length"]
                ep_grp.attrs["duration"] = ep.get("duration", 0.0)
                ep_grp.attrs["intervention_ratio"] = ep.get("intervention_ratio", 0.0)
                ep_grp.attrs["num_interventions"] = ep.get("num_interventions", 0)

                if self.include_images:
                    if "images_rgb" in ep and ep["images_rgb"]:
                        ep_grp.create_dataset(
                            "images_rgb", data=np.array(ep["images_rgb"], dtype=np.uint8),
                            compression=self.compression, compression_opts=self.compression_opts,
                        )

            # 학습/검증 분할
            mask_grp = f.create_group("mask")
            n = len(self.episodes)
            n_train = int(n * train_split)
            indices = np.random.permutation(n)
            train_mask = np.zeros(n, dtype=bool)
            valid_mask = np.zeros(n, dtype=bool)
            train_mask[indices[:n_train]] = True
            valid_mask[indices[n_train:]] = True
            mask_grp.create_dataset("train", data=train_mask)
            mask_grp.create_dataset("valid", data=valid_mask)

        print(f"[CorrectiveRecorder] 저장 완료: {self.file_path}")
        self._print_statistics()

    def _print_statistics(self):
        """통계 출력."""
        if not self.episodes:
            return
        lengths = [ep["length"] for ep in self.episodes]
        successes = [ep.get("success", False) for ep in self.episodes]
        intervention_ratios = [ep.get("intervention_ratio", 0.0) for ep in self.episodes]

        print(f"\n=== 교정 데이터셋 통계 ===")
        print(f"  에피소드 수: {len(self.episodes)}")
        print(f"  총 스텝: {self._total_steps}")
        print(f"  성공률: {np.mean(successes):.1%}")
        print(f"  평균 길이: {np.mean(lengths):.1f}")
        print(f"  총 개입 스텝: {self._total_intervention_steps}")
        print(f"  평균 개입 비율: {np.mean(intervention_ratios):.1%}")
        print(f"==========================\n")

    @property
    def num_episodes(self) -> int:
        return len(self.episodes)


class CorrectiveHDF5Reader:
    """교정 학습 HDF5 데이터셋 리더."""

    def __init__(self, file_path: str):
        self.file_path = Path(file_path)
        if not self.file_path.exists():
            raise FileNotFoundError(f"데이터셋을 찾을 수 없습니다: {file_path}")
        self._file = h5py.File(self.file_path, 'r')

    def close(self):
        if self._file:
            self._file.close()
            self._file = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    @property
    def num_episodes(self) -> int:
        return self._file.attrs.get("num_episodes", 0)

    @property
    def phase(self) -> int:
        return self._file.attrs.get("phase", 1)

    def get_episode(self, idx: int) -> Dict[str, Any]:
        """에피소드 데이터 로드."""
        ep = self._file[f"data/demo_{idx}"]
        result = {
            "obs": ep["obs"][:],
            "actions": ep["actions"][:],
            "dones": ep["dones"][:],
            "timestamps": ep["timestamps"][:],
            "rewards": ep["rewards"][:] if "rewards" in ep else None,
            "intervention_labels": ep["intervention_labels"][:] if "intervention_labels" in ep else None,
            "intervention_ids": ep["intervention_ids"][:] if "intervention_ids" in ep else None,
            "uncertainty_scores": ep["uncertainty_scores"][:] if "uncertainty_scores" in ep else None,
            "help_request_types": ep["help_request_types"][:] if "help_request_types" in ep else None,
            "success": ep.attrs.get("success", False),
            "length": ep.attrs.get("length", len(ep["obs"])),
            "intervention_ratio": ep.attrs.get("intervention_ratio", 0.0),
        }
        return result

    def get_iwr_weights(self, autonomous_weight: float = 1.0,
                        intervention_weight: float = 5.0) -> List[np.ndarray]:
        """IWR 가중치 배열 반환 (Phase 2)."""
        weights = []
        for i in range(self.num_episodes):
            ep = self.get_episode(i)
            labels = ep["intervention_labels"]
            if labels is None:
                w = np.ones(ep["length"], dtype=np.float32) * autonomous_weight
            else:
                w = np.where(labels == LABEL_INTERVENTION,
                             intervention_weight, autonomous_weight).astype(np.float32)
            weights.append(w)
        return weights

    def get_train_episodes(self) -> List[int]:
        return list(np.where(self._file["mask/train"][:])[0])

    def get_valid_episodes(self) -> List[int]:
        return list(np.where(self._file["mask/valid"][:])[0])
