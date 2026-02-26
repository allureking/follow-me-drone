"""Shared utilities for logging setup and signal handling."""

from __future__ import annotations

import logging
import signal
import sys
from typing import Callable


def setup_logging(debug: bool = False) -> None:
    """Configure root logger with timestamp format.

    Args:
        debug: If True, set level to DEBUG; otherwise INFO.
    """
    level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def register_shutdown_handler(
    cleanup_fn: Callable[[], None],
    logger: logging.Logger,
) -> None:
    """Register SIGINT/SIGTERM handlers that call cleanup before exiting.

    Args:
        cleanup_fn: Function to call on shutdown (e.g., drone.cleanup).
        logger: Logger for shutdown messages.
    """

    def _handler(signum: int, _frame) -> None:
        sig_name = signal.Signals(signum).name
        logger.info("Received %s, shutting down...", sig_name)
        cleanup_fn()
        sys.exit(0)

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
