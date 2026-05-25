# Logging Configuration

import os
import logging
import logging.handlers
from typing import Dict, Any, Optional
from pathlib import Path


def setup_logging(
    config: Optional[Dict[str, Any]] = None,
    log_dir: Optional[str] = None
) -> logging.Logger:
    """Setup logging configuration

    Args:
        config: Logging configuration dictionary
        log_dir: Log directory path

    Returns:
        Root logger
    """
    config = config or {}

    # Get configuration values
    level_str = config.get("level", "INFO")
    level = getattr(logging, level_str.upper(), logging.INFO)

    format_str = config.get(
        "format",
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    # Create formatter
    formatter = logging.Formatter(format_str)

    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)

    # Clear existing handlers
    root_logger.handlers.clear()

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # File handler (optional)
    log_file = config.get("file")
    if log_file:
        # Resolve log file path
        if log_dir:
            log_path = Path(log_dir) / log_file
        else:
            log_path = Path(log_file)

        # Create log directory if needed
        log_path.parent.mkdir(parents=True, exist_ok=True)

        # Rotating file handler
        max_bytes = config.get("max_bytes", 10 * 1024 * 1024)  # 10MB
        backup_count = config.get("backup_count", 5)

        file_handler = logging.handlers.RotatingFileHandler(
            log_path,
            maxBytes=max_bytes,
            backupCount=backup_count
        )
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)

        logging.info(f"Log file: {log_path}")

    # Set levels for noisy loggers
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)

    logging.info(f"Logging initialized at level: {level_str}")

    return root_logger


def get_logger(name: str) -> logging.Logger:
    """Get a named logger

    Args:
        name: Logger name

    Returns:
        Logger instance
    """
    return logging.getLogger(name)


class LoggerMixin:
    """Mixin class to add logging to classes"""

    @property
    def logger(self) -> logging.Logger:
        """Get logger for this class"""
        if not hasattr(self, '_logger'):
            self._logger = logging.getLogger(
                f"{self.__class__.__module__}.{self.__class__.__name__}"
            )
        return self._logger
