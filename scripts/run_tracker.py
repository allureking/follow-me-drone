#!/usr/bin/env python3
"""Entry point for the face-tracking drone controller."""

from __future__ import annotations

import argparse
import logging
import time

import cv2

from followme.commands import Command
from followme.config import load_config
from followme.drone_controller import CircleRecorder, DroneConnection, FaceTracker, take_picture
from followme.face_detector import FaceDetector
from followme.ipc import CommandChannel
from followme.utils import register_shutdown_handler, setup_logging

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Follow-Me Drone: Face Tracking Controller")
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
    drone = DroneConnection(config.drone)
    detector = FaceDetector(config.face_detection)
    tracker = FaceTracker(config.tracking, config.drone)
    commands = CommandChannel(config.ipc)
    circle = CircleRecorder(config.circle_motion)

    # Register graceful shutdown
    register_shutdown_handler(drone.cleanup, logger)

    try:
        drone.connect()
        drone.takeoff()

        logger.info("Entering main control loop")
        while True:
            frame = drone.get_frame()
            face = detector.detect(frame)

            if face is not None:
                FaceDetector.draw_annotations(frame, face)

            # Compute and send tracking control
            lr, fb, ud, yaw = tracker.compute_control(face, drone)

            # Height safety enforcement
            if drone.get_height() > config.drone.height_limit:
                logger.warning("Height limit reached, descending")
                drone.send_control(0, 0, -10, 0)
            else:
                drone.send_control(lr, fb, ud, yaw)

            # Process gesture commands
            new_commands = commands.read_new_commands()
            for cmd in new_commands:
                if cmd == Command.CIRCLE_MOTION:
                    logger.info("Executing circle motion command")
                    tracker.reset()
                    circle.execute(drone)
                elif cmd == Command.TAKE_PHOTO:
                    logger.info("Executing take photo command")
                    take_picture(frame)

            # Display video feed
            cv2.imshow("Output", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                logger.info("Quit key pressed")
                break

            time.sleep(config.tracking.loop_interval)

    except Exception as e:
        logger.error("Unexpected error: %s", e, exc_info=True)
    finally:
        drone.cleanup()
        cv2.destroyAllWindows()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
