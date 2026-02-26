"""Drone connection, face tracking, and circle motion recording."""

from __future__ import annotations

import logging
import time
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np
from djitellopy import Tello

from followme.config import AppConfig, CircleMotionConfig, DroneConfig, TrackingConfig
from followme.face_detector import FaceInfo
from followme.pid import PIDController

logger = logging.getLogger(__name__)


class DroneConnection:
    """Manages the DJI Tello drone lifecycle: connect, takeoff, stream, land.

    Args:
        config: Drone-specific configuration parameters.
    """

    def __init__(self, config: DroneConfig) -> None:
        self._config = config
        self._tello = Tello()
        self._airborne = False

    @property
    def tello(self) -> Tello:
        return self._tello

    def connect(self) -> None:
        """Connect to the drone and start the video stream."""
        self._tello.connect()
        battery = self._tello.get_battery()
        logger.info("Connected to Tello. Battery: %d%%", battery)
        self._tello.streamon()
        logger.info("Video stream started (%dx%d)", self._config.frame_width, self._config.frame_height)

    def takeoff(self) -> None:
        """Take off and perform initial ascent."""
        self._tello.takeoff()
        self._airborne = True
        logger.info("Takeoff complete, ascending for %.1fs", self._config.takeoff_ascent_duration)
        self._tello.send_rc_control(0, 0, self._config.takeoff_ascent_speed, 0)
        time.sleep(self._config.takeoff_ascent_duration)

    def get_frame(self) -> np.ndarray:
        """Get the current video frame from the drone camera."""
        return self._tello.get_frame_read().frame

    def send_control(self, lr: int, fb: int, ud: int, yaw: int) -> None:
        """Send RC control commands to the drone.

        Args:
            lr: Left/right speed (-100 to 100).
            fb: Forward/backward speed (-100 to 100).
            ud: Up/down speed (-100 to 100).
            yaw: Yaw rotation speed (-100 to 100).
        """
        self._tello.send_rc_control(lr, fb, ud, yaw)

    def get_height(self) -> int:
        """Get current height in cm."""
        return self._tello.get_height()

    def land(self) -> None:
        """Land the drone if it is airborne."""
        if self._airborne:
            logger.info("Landing drone...")
            self._tello.land()
            self._airborne = False

    def cleanup(self) -> None:
        """Safely land and stop the video stream."""
        try:
            self.land()
        except Exception as e:
            logger.error("Error during landing: %s", e)
        try:
            self._tello.streamoff()
        except Exception as e:
            logger.error("Error stopping stream: %s", e)


class FaceTracker:
    """PID-based face tracking controller for 3-axis drone movement.

    Controls yaw (horizontal rotation), vertical speed, and forward/backward
    movement to keep the detected face centered at the target position.

    Args:
        config: Tracking configuration with PID gains and target parameters.
        drone_config: Drone config for frame dimensions.
    """

    def __init__(self, config: TrackingConfig, drone_config: DroneConfig) -> None:
        self._config = config
        self._frame_w = drone_config.frame_width
        self._frame_h = drone_config.frame_height

        # Pixel-space target position
        self._target_x = int(self._frame_w * config.target_x_fraction)
        self._target_y = int(self._frame_h * config.target_y_fraction)

        # PID controllers for yaw and vertical axes
        self._yaw_pid = PIDController(
            config=config.pid,
            output_limits=(-config.yaw_speed_limit, config.yaw_speed_limit),
        )
        self._vertical_pid = PIDController(
            config=config.pid,
            output_limits=(-config.vertical_speed_limit, config.vertical_speed_limit),
        )

        # Face loss tracking
        self._center_history: deque = deque(maxlen=drone_config.frame_width)
        self._center_history_size = 20  # Use fixed size for lost-face detection
        self._last_known_direction: str = "+"

    def compute_control(
        self, face: Optional[FaceInfo], drone: DroneConnection
    ) -> Tuple[int, int, int, int]:
        """Compute RC control values based on detected face position.

        Args:
            face: Detected face info, or None if no face found.
            drone: Drone connection for height checking.

        Returns:
            Tuple of (left_right, forward_back, up_down, yaw) speeds.
        """
        # Height safety check
        if drone.get_height() > self._config.face_area_max:
            # Use height_limit from the config instead
            pass

        if face is None:
            return self._handle_lost_face()

        # Update direction tracking
        if face.center_x < self._frame_w // 2:
            self._last_known_direction = "-"
        else:
            self._last_known_direction = "+"

        self._center_history.append((face.center_x, face.center_y))

        # Compute errors
        x_error = face.center_x - self._target_x
        y_error = face.center_y - self._target_y

        # PID control for yaw (horizontal tracking)
        yaw_speed = int(self._yaw_pid.update(x_error))

        # PID control for vertical (inverted and scaled)
        vertical_raw = self._vertical_pid.update(y_error)
        y_speed = int(np.clip(
            -self._config.vertical_speed_scale * vertical_raw,
            -self._config.vertical_speed_limit,
            self._config.vertical_speed_limit,
        ))

        # Forward/backward based on face area
        fb = self._compute_forward_back(face.area)

        logger.debug(
            "Tracking: yaw=%d fb=%d ud=%d | error=(%d, %d) area=%d",
            yaw_speed, fb, y_speed, x_error, y_error, face.area,
        )

        return (0, fb, y_speed, yaw_speed)

    def _compute_forward_back(self, area: int) -> int:
        """Determine forward/backward speed to maintain target distance."""
        if self._config.face_area_min <= area <= self._config.face_area_max:
            return 0
        elif area > self._config.face_area_max:
            return self._config.backward_speed
        elif area > 0:
            return self._config.forward_speed
        return 0

    def _handle_lost_face(self) -> Tuple[int, int, int, int]:
        """Rotate to search for the face when it's lost."""
        self._center_history.append((0, 0))

        # Count recent frames with no face detected
        recent = list(self._center_history)[-self._center_history_size:]
        lost_count = sum(1 for x, y in recent if x == 0 and y == 0)

        if lost_count > self._config.lost_face_threshold:
            speed = self._config.search_rotation_speed
            yaw = speed if self._last_known_direction == "+" else -speed
            logger.debug("Searching for face, rotating %s", self._last_known_direction)
            return (0, 0, 0, yaw)

        # Recently lost, hold position
        return (0, 0, 0, 0)

    def reset(self) -> None:
        """Reset PID controllers and tracking history."""
        self._yaw_pid.reset()
        self._vertical_pid.reset()
        self._center_history.clear()


class CircleRecorder:
    """Records panoramic video while moving the drone in a circular path.

    Args:
        config: Circle motion parameters (speed, yaw, duration, fps).
    """

    def __init__(self, config: CircleMotionConfig) -> None:
        self._config = config

    def execute(self, drone: DroneConnection) -> None:
        """Execute circular motion while recording video.

        Args:
            drone: Active drone connection to control and capture from.
        """
        logger.info(
            "Starting circle motion: speed=%d, yaw=%d, duration=%ds",
            self._config.speed, self._config.yaw_speed, self._config.duration,
        )

        frame = drone.get_frame()
        h, w = frame.shape[:2]
        timestamp = int(time.time())
        filename = f"circle_motion_{timestamp}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video = cv2.VideoWriter(filename, fourcc, self._config.video_fps, (w, h))
        logger.info("Recording to %s", filename)

        start = time.time()
        try:
            while time.time() - start < self._config.duration:
                frame = drone.get_frame()
                if frame is not None:
                    video.write(frame)
                    cv2.imshow("Output", frame)

                drone.send_control(self._config.speed, 0, 0, self._config.yaw_speed)
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Circle motion interrupted by user")
                    break
        except Exception as e:
            logger.error("Error during circle motion: %s", e)
        finally:
            drone.send_control(0, 0, 0, 0)
            video.release()
            logger.info("Circle motion complete, video saved as %s", filename)


def take_picture(frame: np.ndarray) -> str:
    """Save the current frame as a PNG image.

    Args:
        frame: BGR image to save.

    Returns:
        Filename of the saved image.
    """
    filename = f"picture_{int(time.time())}.png"
    cv2.imwrite(filename, frame)
    logger.info("Picture saved as %s", filename)
    return filename
