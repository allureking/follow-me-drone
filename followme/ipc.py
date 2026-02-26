"""Inter-process command channel using atomic file operations."""

from __future__ import annotations

import logging
import os
import pickle
from pathlib import Path
from typing import List, Optional

from followme.commands import Command
from followme.config import IPCConfig

logger = logging.getLogger(__name__)


class CommandChannel:
    """File-based IPC for passing gesture commands to the drone controller.

    The gesture recognition process writes snap counts to a pickle cache file.
    The drone controller reads new commands since its last read.
    Writes are atomic (temp file + os.replace) to prevent corruption.

    Args:
        config: IPC configuration with cache file path.
    """

    def __init__(self, config: IPCConfig) -> None:
        self._cache_path = Path(config.cache_file)
        self._read_index: int = 0

    def initialize(self) -> None:
        """Create an empty command cache file. Called once by the writer at startup."""
        self._write_atomic([])
        logger.info("Initialized command cache at %s", self._cache_path)

    def write_commands(self, commands: List[int]) -> None:
        """Atomically write the full command list to the cache file.

        Args:
            commands: Complete list of snap counts accumulated so far.
        """
        self._write_atomic(commands)

    def read_new_commands(self) -> List[Optional[Command]]:
        """Read commands added since the last read.

        Returns:
            List of new Command values (or None for unrecognized snap counts).
        """
        try:
            with open(self._cache_path, "rb") as f:
                all_snaps: List[int] = pickle.load(f)
        except FileNotFoundError:
            return []
        except (pickle.UnpicklingError, EOFError) as e:
            logger.warning("Failed to read command cache: %s", e)
            return []

        new_commands = []
        while self._read_index < len(all_snaps):
            snap_count = all_snaps[self._read_index]
            self._read_index += 1
            cmd = Command.from_snap_count(snap_count)
            if cmd is not None:
                new_commands.append(cmd)
                logger.info("Read command: %s (snap count: %d)", cmd.name, snap_count)

        return new_commands

    def _write_atomic(self, data: List[int]) -> None:
        """Write data to a temp file then atomically replace the cache file."""
        tmp_path = self._cache_path.with_suffix(".pkl.tmp")
        with open(tmp_path, "wb") as f:
            pickle.dump(data, f)
        os.replace(tmp_path, self._cache_path)
