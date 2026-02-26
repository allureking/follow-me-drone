"""Command definitions for drone actions triggered by gestures."""

from __future__ import annotations

import logging
from enum import IntEnum
from typing import Optional

logger = logging.getLogger(__name__)


class Command(IntEnum):
    """Drone commands mapped from gesture snap counts."""

    CIRCLE_MOTION = 1
    TAKE_PHOTO = 2

    @classmethod
    def from_snap_count(cls, count: int) -> Optional[Command]:
        """Convert a snap count to a Command, or None if unrecognized.

        Args:
            count: Number of consecutive snaps detected.

        Returns:
            Corresponding Command, or None for unknown snap counts.
        """
        try:
            return cls(count)
        except ValueError:
            logger.warning("Unknown snap count %d, no command mapped", count)
            return None
