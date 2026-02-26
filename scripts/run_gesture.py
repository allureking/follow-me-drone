#!/usr/bin/env python3
"""Entry point for the gesture recognition sensor reader."""

from __future__ import annotations

import argparse
import logging

from followme.config import load_config
from followme.gesture import SerialIMUReader, SnapDetector
from followme.ipc import CommandChannel
from followme.utils import setup_logging

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Follow-Me Drone: Gesture Recognition")
    parser.add_argument(
        "--config", type=str, default=None,
        help="Path to YAML config file (default: config/default.yaml)",
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="Enable debug logging",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    setup_logging(debug=args.debug)

    config = load_config(args.config)
    logger.info("Configuration loaded")

    # Initialize components
    reader = SerialIMUReader(config.gesture)
    detector = SnapDetector(config.gesture)
    commands = CommandChannel(config.ipc)
    commands.initialize()

    all_snaps: list[int] = []

    try:
        reader.connect()
        logger.info("Gesture recognition running. Press Ctrl+C to stop.")

        while True:
            reading = reader.read_sensors()
            if reading is None:
                continue

            s1, s2 = reading
            snap_count = detector.process_reading(s1, s2)

            if snap_count is not None:
                logger.info("Gesture detected: %d snaps", snap_count)
                all_snaps.append(snap_count)
                commands.write_commands(all_snaps)
                logger.debug("Command cache updated: %s", all_snaps)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error("Unexpected error: %s", e, exc_info=True)
    finally:
        reader.close()
        logger.info("Gesture recognition stopped")


if __name__ == "__main__":
    main()
