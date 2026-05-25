# Configuration Loader

import os
import re
import logging
from typing import Dict, Any, Optional
from pathlib import Path

import yaml

logger = logging.getLogger(__name__)


class ConfigLoader:
    """Configuration file loader with environment variable support"""

    def __init__(self, config_dir: Optional[str] = None):
        """Initialize config loader

        Args:
            config_dir: Configuration directory path
        """
        if config_dir:
            self.config_dir = Path(config_dir)
        else:
            # Default to config directory relative to this file
            self.config_dir = Path(__file__).parent.parent / "config"

        self._config: Dict[str, Any] = {}

    def load_all(self) -> Dict[str, Any]:
        """Load all configuration files

        Returns:
            Merged configuration dictionary
        """
        config_files = [
            "robot_config.yaml",
            "workspace_config.yaml",
            "llm_config.yaml",
            "server_config.yaml"
        ]

        self._config = {}

        for filename in config_files:
            filepath = self.config_dir / filename
            if filepath.exists():
                key = filename.replace("_config.yaml", "")
                self._config[key] = self._load_file(filepath)
                logger.info(f"Loaded config: {filename}")
            else:
                logger.warning(f"Config file not found: {filepath}")

        return self._config

    def load_file(self, filename: str) -> Dict[str, Any]:
        """Load a specific configuration file

        Args:
            filename: Configuration filename

        Returns:
            Configuration dictionary
        """
        filepath = self.config_dir / filename
        return self._load_file(filepath)

    def _load_file(self, filepath: Path) -> Dict[str, Any]:
        """Load and parse a YAML file

        Args:
            filepath: Path to YAML file

        Returns:
            Parsed configuration dictionary
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()

            # Resolve environment variables
            content = self._resolve_env_vars(content)

            config = yaml.safe_load(content)
            return config or {}

        except Exception as e:
            logger.error(f"Error loading config file {filepath}: {e}")
            return {}

    def _resolve_env_vars(self, content: str) -> str:
        """Resolve environment variables in config content

        Supports ${VAR_NAME} syntax

        Args:
            content: Configuration file content

        Returns:
            Content with resolved variables
        """
        pattern = r'\$\{([^}]+)\}'

        def replacer(match):
            var_name = match.group(1)
            value = os.environ.get(var_name, "")
            if not value:
                logger.warning(f"Environment variable not set: {var_name}")
            return value

        return re.sub(pattern, replacer, content)

    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value by key

        Args:
            key: Configuration key (supports dot notation)
            default: Default value if not found

        Returns:
            Configuration value
        """
        keys = key.split('.')
        value = self._config

        for k in keys:
            if isinstance(value, dict):
                value = value.get(k)
                if value is None:
                    return default
            else:
                return default

        return value

    def set(self, key: str, value: Any):
        """Set configuration value

        Args:
            key: Configuration key (supports dot notation)
            value: Value to set
        """
        keys = key.split('.')
        config = self._config

        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]

        config[keys[-1]] = value

    @property
    def config(self) -> Dict[str, Any]:
        """Get full configuration"""
        return self._config


def load_config(config_dir: Optional[str] = None) -> Dict[str, Any]:
    """Convenience function to load all configuration

    Args:
        config_dir: Configuration directory path

    Returns:
        Merged configuration dictionary
    """
    loader = ConfigLoader(config_dir)
    return loader.load_all()


def get_default_config() -> Dict[str, Any]:
    """Get default configuration values

    Returns:
        Default configuration dictionary
    """
    return {
        "robot": {
            "name": "stretch",
            "prim_path": "/World/stretch"
        },
        "workspace": {
            "bounds": {
                "min": [-2.0, -2.0, 0.0],
                "max": [2.0, 2.0, 1.5]
            },
            "velocity_limits": {
                "manipulator": {
                    "max_linear": 0.5,
                    "max_angular": 1.0
                },
                "base": {
                    "max_linear": 1.0,
                    "max_angular": 1.5
                }
            },
            "safety": {
                "workspace_margin": 0.05,
                "self_collision_check": True,
                "environment_collision_check": True
            }
        },
        "llm": {
            "provider": "openai",
            "openai": {
                "model": "gpt-4o",
                "temperature": 0.1,
                "max_tokens": 1024
            },
            "cache": {
                "enabled": True,
                "max_size": 100
            }
        },
        "server": {
            "host": "0.0.0.0",
            "port": 8000
        }
    }
