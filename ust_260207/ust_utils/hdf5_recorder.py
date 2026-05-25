# Copyright (c) 2026 UST Project
# SPDX-License-Identifier: MIT
"""HDF5 Dataset Recorder for Teleoperation Data Collection."""

from __future__ import annotations

import h5py
import numpy as np
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional, Union
import json


class HDF5DatasetRecorder:
    """텔레오퍼레이션 데이터 HDF5 레코더.

    VR 텔레오퍼레이션 데이터를 HDF5 형식으로 저장합니다.
    Robomimic 및 LeRobot과 호환되는 형식을 지원합니다.

    데이터 구조:
        /data
            /demo_0
                /obs                 (T, obs_dim)
                /actions             (T, action_dim)
                /dones               (T,)
                /timestamps          (T,)
                /images_rgb          (T, H, W, 3) [선택적]
                /images_depth        (T, H, W) [선택적]
            /demo_1
                ...
        /mask
            /train                   (N,) bool
            /valid                   (N,) bool
    """

    def __init__(
        self,
        file_path: str,
        observation_keys: List[str],
        action_dim: int,
        include_images: bool = True,
        image_shape: tuple = (720, 1280, 3),
        compression: str = "gzip",
        compression_opts: int = 4,
    ):
        """레코더 초기화.

        Args:
            file_path: 저장할 HDF5 파일 경로
            observation_keys: 관측 키 이름 리스트
            action_dim: 액션 차원
            include_images: 이미지 저장 여부
            image_shape: 이미지 형태 (H, W, C)
            compression: 압축 방식 ("gzip", "lzf", None)
            compression_opts: 압축 옵션 (gzip: 1-9)
        """
        self.file_path = Path(file_path)
        self.file_path.parent.mkdir(parents=True, exist_ok=True)

        self.observation_keys = observation_keys
        self.action_dim = action_dim
        self.include_images = include_images
        self.image_shape = image_shape
        self.compression = compression
        self.compression_opts = compression_opts

        # 에피소드 데이터 저장
        self.episodes: List[Dict[str, List]] = []
        self.current_episode: Optional[Dict[str, List]] = None

        # 통계 추적
        self._total_steps = 0
        self._start_time = None

    def start_episode(self):
        """새 에피소드 시작."""
        if self.current_episode is not None:
            print("[WARNING] Previous episode was not ended. Ending now.")
            self.end_episode(success=False)

        self.current_episode = {
            "obs": [],
            "actions": [],
            "dones": [],
            "timestamps": [],
            "rewards": [],  # 옵션
        }

        if self.include_images:
            self.current_episode["images_rgb"] = []
            self.current_episode["images_depth"] = []

        self._start_time = datetime.now()
        print(f"[INFO] Episode {len(self.episodes)} started.")

    def add_step(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        done: bool = False,
        reward: float = 0.0,
        rgb_image: Optional[np.ndarray] = None,
        depth_image: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ):
        """스텝 데이터 추가.

        Args:
            obs: 관측 벡터
            action: 액션 벡터
            done: 에피소드 종료 여부
            reward: 보상 (옵션)
            rgb_image: RGB 이미지 (H, W, 3) [옵션]
            depth_image: 깊이 이미지 (H, W) [옵션]
            timestamp: 타임스탬프 (None이면 현재 시간)
        """
        if self.current_episode is None:
            raise RuntimeError("Episode not started. Call start_episode() first.")

        # 데이터 검증
        obs = np.asarray(obs, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32)

        if action.shape[-1] != self.action_dim:
            raise ValueError(f"Action dim mismatch: expected {self.action_dim}, got {action.shape[-1]}")

        # 데이터 추가
        self.current_episode["obs"].append(obs.copy())
        self.current_episode["actions"].append(action.copy())
        self.current_episode["dones"].append(done)
        self.current_episode["rewards"].append(reward)
        self.current_episode["timestamps"].append(timestamp or datetime.now().timestamp())

        # 이미지 추가
        if self.include_images:
            if rgb_image is not None:
                rgb_image = np.asarray(rgb_image, dtype=np.uint8)
                self.current_episode["images_rgb"].append(rgb_image.copy())
            if depth_image is not None:
                depth_image = np.asarray(depth_image, dtype=np.float32)
                self.current_episode["images_depth"].append(depth_image.copy())

        self._total_steps += 1

    def end_episode(self, success: bool = True):
        """에피소드 종료.

        Args:
            success: 에피소드 성공 여부
        """
        if self.current_episode is None:
            print("[WARNING] No episode to end.")
            return

        if len(self.current_episode["obs"]) == 0:
            print("[WARNING] Empty episode discarded.")
            self.current_episode = None
            return

        # 메타데이터 추가
        self.current_episode["success"] = success
        self.current_episode["length"] = len(self.current_episode["obs"])
        self.current_episode["duration"] = (datetime.now() - self._start_time).total_seconds()

        # 에피소드 저장
        self.episodes.append(self.current_episode)
        self.current_episode = None

        episode_idx = len(self.episodes) - 1
        print(f"[INFO] Episode {episode_idx} ended. Length: {self.episodes[-1]['length']}, Success: {success}")

    def save(self, train_split: float = 0.8):
        """HDF5 파일로 저장.

        Args:
            train_split: 학습 데이터 비율 (0-1)
        """
        if len(self.episodes) == 0:
            print("[WARNING] No episodes to save.")
            return

        print(f"[INFO] Saving {len(self.episodes)} episodes to {self.file_path}")

        with h5py.File(self.file_path, 'w') as f:
            # 메타데이터
            f.attrs["num_episodes"] = len(self.episodes)
            f.attrs["total_steps"] = self._total_steps
            f.attrs["creation_time"] = datetime.now().isoformat()
            f.attrs["observation_keys"] = json.dumps(self.observation_keys)
            f.attrs["action_dim"] = self.action_dim
            f.attrs["include_images"] = self.include_images

            # 환경 정보
            env_args = {
                "env_name": "UST-MobileManipulator-v0",
                "type": "teleoperation",
                "robot": "turtlebot3_waffle_pi_openmanipulator_x",
            }
            f.attrs["env_args"] = json.dumps(env_args)

            # 데이터 그룹
            data_grp = f.create_group("data")

            for i, ep in enumerate(self.episodes):
                ep_grp = data_grp.create_group(f"demo_{i}")

                # 관측/액션
                obs_data = np.array(ep["obs"], dtype=np.float32)
                actions_data = np.array(ep["actions"], dtype=np.float32)
                dones_data = np.array(ep["dones"], dtype=bool)
                timestamps_data = np.array(ep["timestamps"], dtype=np.float64)
                rewards_data = np.array(ep["rewards"], dtype=np.float32)

                ep_grp.create_dataset(
                    "obs",
                    data=obs_data,
                    compression=self.compression,
                    compression_opts=self.compression_opts,
                )
                ep_grp.create_dataset(
                    "actions",
                    data=actions_data,
                    compression=self.compression,
                    compression_opts=self.compression_opts,
                )
                ep_grp.create_dataset("dones", data=dones_data)
                ep_grp.create_dataset("timestamps", data=timestamps_data)
                ep_grp.create_dataset("rewards", data=rewards_data)

                # 에피소드 메타데이터
                ep_grp.attrs["success"] = ep.get("success", True)
                ep_grp.attrs["length"] = ep.get("length", len(obs_data))
                ep_grp.attrs["duration"] = ep.get("duration", 0.0)

                # 이미지 (있는 경우)
                if self.include_images:
                    if "images_rgb" in ep and len(ep["images_rgb"]) > 0:
                        rgb_data = np.array(ep["images_rgb"], dtype=np.uint8)
                        ep_grp.create_dataset(
                            "images_rgb",
                            data=rgb_data,
                            compression=self.compression,
                            compression_opts=self.compression_opts,
                        )
                    if "images_depth" in ep and len(ep["images_depth"]) > 0:
                        depth_data = np.array(ep["images_depth"], dtype=np.float32)
                        ep_grp.create_dataset(
                            "images_depth",
                            data=depth_data,
                            compression=self.compression,
                            compression_opts=self.compression_opts,
                        )

            # 마스크 그룹 (train/valid 분할)
            mask_grp = f.create_group("mask")
            n_episodes = len(self.episodes)
            n_train = int(n_episodes * train_split)

            # 무작위 셔플
            indices = np.random.permutation(n_episodes)
            train_mask = np.zeros(n_episodes, dtype=bool)
            valid_mask = np.zeros(n_episodes, dtype=bool)

            train_mask[indices[:n_train]] = True
            valid_mask[indices[n_train:]] = True

            mask_grp.create_dataset("train", data=train_mask)
            mask_grp.create_dataset("valid", data=valid_mask)

        print(f"[INFO] Dataset saved: {self.file_path}")
        self._print_statistics()

    def _print_statistics(self):
        """데이터셋 통계 출력."""
        stats = self.get_statistics()
        print("\n=== Dataset Statistics ===")
        print(f"  Total episodes: {stats['num_episodes']}")
        print(f"  Total steps: {stats['total_steps']}")
        print(f"  Success rate: {stats['success_rate']:.1%}")
        print(f"  Average length: {stats['avg_episode_length']:.1f}")
        print(f"  Min length: {stats['min_length']}")
        print(f"  Max length: {stats['max_length']}")
        print("==========================\n")

    def get_statistics(self) -> Dict[str, Any]:
        """데이터셋 통계 반환.

        Returns:
            통계 딕셔너리
        """
        if not self.episodes:
            return {
                "num_episodes": 0,
                "total_steps": 0,
                "success_rate": 0.0,
                "avg_episode_length": 0.0,
                "min_length": 0,
                "max_length": 0,
            }

        lengths = [ep.get("length", len(ep["obs"])) for ep in self.episodes]
        successes = [ep.get("success", True) for ep in self.episodes]

        return {
            "num_episodes": len(self.episodes),
            "total_steps": sum(lengths),
            "success_rate": float(np.mean(successes)),
            "avg_episode_length": float(np.mean(lengths)),
            "min_length": min(lengths),
            "max_length": max(lengths),
            "std_episode_length": float(np.std(lengths)),
        }

    def discard_current_episode(self):
        """현재 에피소드 폐기."""
        if self.current_episode is not None:
            length = len(self.current_episode.get("obs", []))
            print(f"[INFO] Episode discarded. {length} steps lost.")
            self.current_episode = None

    def get_current_episode_length(self) -> int:
        """현재 에피소드 길이 반환."""
        if self.current_episode is None:
            return 0
        return len(self.current_episode.get("obs", []))


class HDF5DatasetReader:
    """HDF5 데이터셋 리더.

    저장된 HDF5 데이터셋을 읽고 탐색합니다.
    """

    def __init__(self, file_path: str):
        """리더 초기화.

        Args:
            file_path: HDF5 파일 경로
        """
        self.file_path = Path(file_path)
        if not self.file_path.exists():
            raise FileNotFoundError(f"Dataset not found: {file_path}")

        self._file = None
        self._open()

    def _open(self):
        """파일 열기."""
        self._file = h5py.File(self.file_path, 'r')

    def close(self):
        """파일 닫기."""
        if self._file is not None:
            self._file.close()
            self._file = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @property
    def num_episodes(self) -> int:
        """에피소드 수."""
        return self._file.attrs.get("num_episodes", 0)

    @property
    def total_steps(self) -> int:
        """총 스텝 수."""
        return self._file.attrs.get("total_steps", 0)

    def get_episode(self, idx: int) -> Dict[str, np.ndarray]:
        """에피소드 데이터 반환.

        Args:
            idx: 에피소드 인덱스

        Returns:
            에피소드 데이터 딕셔너리
        """
        ep_grp = self._file[f"data/demo_{idx}"]
        return {
            "obs": ep_grp["obs"][:],
            "actions": ep_grp["actions"][:],
            "dones": ep_grp["dones"][:],
            "timestamps": ep_grp["timestamps"][:],
            "rewards": ep_grp["rewards"][:] if "rewards" in ep_grp else None,
            "images_rgb": ep_grp["images_rgb"][:] if "images_rgb" in ep_grp else None,
            "images_depth": ep_grp["images_depth"][:] if "images_depth" in ep_grp else None,
            "success": ep_grp.attrs.get("success", True),
            "length": ep_grp.attrs.get("length", len(ep_grp["obs"])),
        }

    def get_train_episodes(self) -> List[int]:
        """학습용 에피소드 인덱스 리스트."""
        mask = self._file["mask/train"][:]
        return list(np.where(mask)[0])

    def get_valid_episodes(self) -> List[int]:
        """검증용 에피소드 인덱스 리스트."""
        mask = self._file["mask/valid"][:]
        return list(np.where(mask)[0])

    def print_info(self):
        """데이터셋 정보 출력."""
        print("\n=== Dataset Info ===")
        print(f"  File: {self.file_path}")
        print(f"  Episodes: {self.num_episodes}")
        print(f"  Total steps: {self.total_steps}")

        if "env_args" in self._file.attrs:
            env_args = json.loads(self._file.attrs["env_args"])
            print(f"  Environment: {env_args.get('env_name', 'Unknown')}")

        print(f"  Action dim: {self._file.attrs.get('action_dim', 'Unknown')}")
        print(f"  Include images: {self._file.attrs.get('include_images', False)}")
        print("====================\n")
