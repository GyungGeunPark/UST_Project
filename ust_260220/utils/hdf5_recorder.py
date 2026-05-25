"""HDF5 Dataset Recorder for G1 Kitchen Sorting Demo Collection.

Adapted from ust_ws/ust_260207/ust_utils/hdf5_recorder.py for the
G1 INSPIRE 5-finger hand kitchen sorting task.

Supports:
- Per-key observation storage (not flattened)
- Action recording (38D: 14 EEF + 24 hand joints)
- Optional RGB/depth image recording
- Train/valid split masks
- Statistics and data validation
"""

from __future__ import annotations

import h5py
import numpy as np
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional
import json


class HDF5DatasetRecorder:
    """HDF5 demo recorder for G1 Kitchen Sorting task.

    Data structure:
        /data
            /demo_0
                /obs                 (T, obs_dim)
                /actions             (T, 38)
                /dones               (T,)
                /timestamps          (T,)
                /rewards             (T,)
                /images_rgb          (T, H, W, 3) [optional]
                /images_depth        (T, H, W)    [optional]
            /demo_1
                ...
        /mask
            /train                   (N,) bool
            /valid                   (N,) bool
    """

    def __init__(
        self,
        file_path: str,
        action_dim: int = 38,
        include_images: bool = False,
        image_shape: tuple = (720, 1280, 3),
        compression: str = "gzip",
        compression_opts: int = 4,
    ):
        """Initialize recorder.

        Args:
            file_path: HDF5 output file path.
            action_dim: Action dimension (38 for G1 Pink IK + INSPIRE hand).
            include_images: Whether to record camera images.
            image_shape: Image shape (H, W, C).
            compression: HDF5 compression ("gzip", "lzf", None).
            compression_opts: Compression level (gzip: 1-9).
        """
        self.file_path = Path(file_path)
        self.file_path.parent.mkdir(parents=True, exist_ok=True)

        self.action_dim = action_dim
        self.include_images = include_images
        self.image_shape = image_shape
        self.compression = compression
        self.compression_opts = compression_opts

        self.episodes: List[Dict[str, List]] = []
        self.current_episode: Optional[Dict[str, List]] = None

        self._total_steps = 0
        self._start_time = None

    def start_episode(self):
        """Start a new episode."""
        if self.current_episode is not None:
            print("[WARNING] Previous episode was not ended. Ending now.")
            self.end_episode(success=False)

        self.current_episode = {
            "obs": [],
            "actions": [],
            "dones": [],
            "timestamps": [],
            "rewards": [],
        }

        if self.include_images:
            self.current_episode["images_rgb"] = []
            self.current_episode["images_depth"] = []

        self._start_time = datetime.now()
        print(f"[Recorder] Episode {len(self.episodes)} started.")

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
        """Record a single step.

        Args:
            obs: Observation vector (flattened).
            action: Action vector (38D).
            done: Episode done flag.
            reward: Step reward.
            rgb_image: RGB image (H, W, 3) [optional].
            depth_image: Depth image (H, W) [optional].
            timestamp: Step timestamp (defaults to now).
        """
        if self.current_episode is None:
            raise RuntimeError("Episode not started. Call start_episode() first.")

        obs = np.asarray(obs, dtype=np.float32)
        action = np.asarray(action, dtype=np.float32)

        if action.shape[-1] != self.action_dim:
            raise ValueError(f"Action dim mismatch: expected {self.action_dim}, got {action.shape[-1]}")

        self.current_episode["obs"].append(obs.copy())
        self.current_episode["actions"].append(action.copy())
        self.current_episode["dones"].append(done)
        self.current_episode["rewards"].append(reward)
        self.current_episode["timestamps"].append(timestamp or datetime.now().timestamp())

        if self.include_images:
            if rgb_image is not None:
                self.current_episode["images_rgb"].append(np.asarray(rgb_image, dtype=np.uint8).copy())
            if depth_image is not None:
                self.current_episode["images_depth"].append(np.asarray(depth_image, dtype=np.float32).copy())

        self._total_steps += 1

    def end_episode(self, success: bool = False):
        """End current episode and store it.

        Args:
            success: Whether the episode was successful (all objects sorted).
        """
        if self.current_episode is None:
            print("[WARNING] No episode to end.")
            return

        if len(self.current_episode["obs"]) == 0:
            print("[WARNING] Empty episode discarded.")
            self.current_episode = None
            return

        self.current_episode["success"] = success
        self.current_episode["length"] = len(self.current_episode["obs"])
        self.current_episode["duration"] = (datetime.now() - self._start_time).total_seconds()

        self.episodes.append(self.current_episode)
        self.current_episode = None

        ep_idx = len(self.episodes) - 1
        ep = self.episodes[-1]
        print(f"[Recorder] Episode {ep_idx} ended. Length: {ep['length']}, "
              f"Success: {success}, Duration: {ep['duration']:.1f}s")

    def save(self, train_split: float = 0.8):
        """Save all episodes to HDF5 file.

        Args:
            train_split: Fraction of episodes for training (rest for validation).
        """
        if len(self.episodes) == 0:
            print("[WARNING] No episodes to save.")
            return

        print(f"[Recorder] Saving {len(self.episodes)} episodes to {self.file_path}")

        with h5py.File(self.file_path, 'w') as f:
            # Global metadata
            f.attrs["num_episodes"] = len(self.episodes)
            f.attrs["total_steps"] = self._total_steps
            f.attrs["creation_time"] = datetime.now().isoformat()
            f.attrs["action_dim"] = self.action_dim
            f.attrs["include_images"] = self.include_images

            env_args = {
                "env_name": "Isaac-KitchenSorting-G1-InspireFTP-v0",
                "type": "teleoperation",
                "robot": "unitree_g1_inspire_ftp",
                "action_dim": self.action_dim,
                "ik_controller": "pink_ik",
            }
            f.attrs["env_args"] = json.dumps(env_args)

            # Data group
            data_grp = f.create_group("data")

            for i, ep in enumerate(self.episodes):
                ep_grp = data_grp.create_group(f"demo_{i}")

                obs_data = np.array(ep["obs"], dtype=np.float32)
                actions_data = np.array(ep["actions"], dtype=np.float32)
                dones_data = np.array(ep["dones"], dtype=bool)
                timestamps_data = np.array(ep["timestamps"], dtype=np.float64)
                rewards_data = np.array(ep["rewards"], dtype=np.float32)

                ep_grp.create_dataset("obs", data=obs_data,
                                      compression=self.compression, compression_opts=self.compression_opts)
                ep_grp.create_dataset("actions", data=actions_data,
                                      compression=self.compression, compression_opts=self.compression_opts)
                ep_grp.create_dataset("dones", data=dones_data)
                ep_grp.create_dataset("timestamps", data=timestamps_data)
                ep_grp.create_dataset("rewards", data=rewards_data)

                ep_grp.attrs["success"] = ep.get("success", False)
                ep_grp.attrs["length"] = ep.get("length", len(obs_data))
                ep_grp.attrs["duration"] = ep.get("duration", 0.0)

                # Images (if recorded)
                if self.include_images:
                    if "images_rgb" in ep and len(ep["images_rgb"]) > 0:
                        rgb_data = np.array(ep["images_rgb"], dtype=np.uint8)
                        ep_grp.create_dataset("images_rgb", data=rgb_data,
                                              compression=self.compression, compression_opts=self.compression_opts)
                    if "images_depth" in ep and len(ep["images_depth"]) > 0:
                        depth_data = np.array(ep["images_depth"], dtype=np.float32)
                        ep_grp.create_dataset("images_depth", data=depth_data,
                                              compression=self.compression, compression_opts=self.compression_opts)

            # Train/valid split masks
            mask_grp = f.create_group("mask")
            n_episodes = len(self.episodes)
            n_train = int(n_episodes * train_split)

            indices = np.random.permutation(n_episodes)
            train_mask = np.zeros(n_episodes, dtype=bool)
            valid_mask = np.zeros(n_episodes, dtype=bool)
            train_mask[indices[:n_train]] = True
            valid_mask[indices[n_train:]] = True

            mask_grp.create_dataset("train", data=train_mask)
            mask_grp.create_dataset("valid", data=valid_mask)

        print(f"[Recorder] Dataset saved: {self.file_path}")
        self._print_statistics()

    def _print_statistics(self):
        """Print dataset statistics."""
        stats = self.get_statistics()
        print("\n=== Dataset Statistics ===")
        print(f"  Total episodes: {stats['num_episodes']}")
        print(f"  Total steps: {stats['total_steps']}")
        print(f"  Success rate: {stats['success_rate']:.1%}")
        print(f"  Average length: {stats['avg_episode_length']:.1f}")
        print(f"  Min length: {stats['min_length']}")
        print(f"  Max length: {stats['max_length']}")
        print(f"  Avg duration: {stats['avg_duration']:.1f}s")
        print("==========================\n")

    def get_statistics(self) -> Dict[str, Any]:
        """Return dataset statistics."""
        if not self.episodes:
            return {
                "num_episodes": 0, "total_steps": 0, "success_rate": 0.0,
                "avg_episode_length": 0.0, "min_length": 0, "max_length": 0,
                "avg_duration": 0.0,
            }

        lengths = [ep.get("length", len(ep["obs"])) for ep in self.episodes]
        successes = [ep.get("success", False) for ep in self.episodes]
        durations = [ep.get("duration", 0.0) for ep in self.episodes]

        return {
            "num_episodes": len(self.episodes),
            "total_steps": sum(lengths),
            "success_rate": float(np.mean(successes)),
            "avg_episode_length": float(np.mean(lengths)),
            "min_length": min(lengths),
            "max_length": max(lengths),
            "std_episode_length": float(np.std(lengths)),
            "avg_duration": float(np.mean(durations)),
        }

    def discard_current_episode(self):
        """Discard the current in-progress episode."""
        if self.current_episode is not None:
            length = len(self.current_episode.get("obs", []))
            print(f"[Recorder] Episode discarded. {length} steps lost.")
            self.current_episode = None

    def get_current_episode_length(self) -> int:
        """Return the current episode's step count."""
        if self.current_episode is None:
            return 0
        return len(self.current_episode.get("obs", []))

    @property
    def num_episodes(self) -> int:
        """Number of completed episodes."""
        return len(self.episodes)


class HDF5DatasetReader:
    """HDF5 dataset reader for loading saved demos.

    Supports train/valid split access and episode-level queries.
    """

    def __init__(self, file_path: str):
        self.file_path = Path(file_path)
        if not self.file_path.exists():
            raise FileNotFoundError(f"Dataset not found: {file_path}")
        self._file = h5py.File(self.file_path, 'r')

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @property
    def num_episodes(self) -> int:
        return self._file.attrs.get("num_episodes", 0)

    @property
    def total_steps(self) -> int:
        return self._file.attrs.get("total_steps", 0)

    def get_episode(self, idx: int) -> Dict[str, Any]:
        """Load a single episode by index."""
        ep_grp = self._file[f"data/demo_{idx}"]
        result = {
            "obs": ep_grp["obs"][:],
            "actions": ep_grp["actions"][:],
            "dones": ep_grp["dones"][:],
            "timestamps": ep_grp["timestamps"][:],
            "rewards": ep_grp["rewards"][:] if "rewards" in ep_grp else None,
            "success": ep_grp.attrs.get("success", False),
            "length": ep_grp.attrs.get("length", len(ep_grp["obs"])),
        }
        if "images_rgb" in ep_grp:
            result["images_rgb"] = ep_grp["images_rgb"][:]
        if "images_depth" in ep_grp:
            result["images_depth"] = ep_grp["images_depth"][:]
        return result

    def get_train_episodes(self) -> List[int]:
        """Return indices of training episodes."""
        mask = self._file["mask/train"][:]
        return list(np.where(mask)[0])

    def get_valid_episodes(self) -> List[int]:
        """Return indices of validation episodes."""
        mask = self._file["mask/valid"][:]
        return list(np.where(mask)[0])

    def print_info(self):
        """Print dataset summary."""
        print(f"\n=== Dataset Info ===")
        print(f"  File: {self.file_path}")
        print(f"  Episodes: {self.num_episodes}")
        print(f"  Total steps: {self.total_steps}")
        if "env_args" in self._file.attrs:
            env_args = json.loads(self._file.attrs["env_args"])
            print(f"  Robot: {env_args.get('robot', 'Unknown')}")
            print(f"  IK: {env_args.get('ik_controller', 'Unknown')}")
        print(f"  Action dim: {self._file.attrs.get('action_dim', 'Unknown')}")
        print(f"  Include images: {self._file.attrs.get('include_images', False)}")
        n_train = len(self.get_train_episodes())
        n_valid = len(self.get_valid_episodes())
        print(f"  Train/Valid split: {n_train}/{n_valid}")
        print("====================\n")
