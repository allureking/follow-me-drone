"""Gesture recognition via IMU snap detection over serial."""

from __future__ import annotations

import logging
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np
import serial
from serial.tools import list_ports

from followme.config import GestureConfig

logger = logging.getLogger(__name__)


class SnapDetector:
    """State machine for detecting snap gestures from dual IMU sensor readings.

    Snaps are detected when the difference between two sensor values exceeds
    a threshold. Consecutive snaps within a time window are grouped into a
    single gesture with the total snap count.

    Args:
        config: Gesture configuration with threshold and timing parameters.
    """

    def __init__(self, config: GestureConfig) -> None:
        self._threshold = config.snap_threshold
        self._time_window = config.snap_time_window
        self._valid = True  # Hysteresis flag to prevent duplicate detections
        self._snap_count = 0
        self._prev_snap_time: Optional[float] = None

    def process_reading(self, sensor1: int, sensor2: int) -> Optional[int]:
        """Process a pair of sensor readings and detect snap gestures.

        Args:
            sensor1: First IMU sensor value.
            sensor2: Second IMU sensor value.

        Returns:
            Total snap count when a gesture sequence completes (time window
            expires after last snap), or None if no complete gesture yet.
        """
        diff = sensor2 - sensor1
        now = time.time()

        # Detect snap edge
        is_snap = self._detect_edge(diff)

        if is_snap:
            if self._snap_count == 0:
                # First snap in a new sequence
                self._snap_count = 1
                self._prev_snap_time = now
            else:
                # Consecutive snap within time window
                self._snap_count += 1
                self._prev_snap_time = now
            logger.debug("Snap detected! Count: %d", self._snap_count)
            return None

        # Check if a sequence has completed (no snap within time window)
        if self._snap_count > 0 and self._prev_snap_time is not None:
            if now - self._prev_snap_time > self._time_window:
                count = self._snap_count
                self._snap_count = 0
                self._prev_snap_time = None
                logger.info("Snap sequence complete: %d snaps", count)
                return count

        return None

    def _detect_edge(self, diff: int) -> bool:
        """Detect a snap using hysteresis to prevent false positives."""
        if diff > self._threshold and self._valid:
            self._valid = False
            return True
        elif diff < self._threshold and not self._valid:
            self._valid = True
        return False

    def reset(self) -> None:
        """Clear all detection state."""
        self._valid = True
        self._snap_count = 0
        self._prev_snap_time = None


class SerialIMUReader:
    """Reads dual IMU sensor data from a microcontroller over serial.

    Expects comma-separated integer pairs (sensor1, sensor2) per line
    from a Teensy 4.1 with two MPU6050 sensors.

    Args:
        config: Gesture configuration with serial port and baud rate.
    """

    def __init__(self, config: GestureConfig) -> None:
        self._port = config.serial_port
        self._baud = config.baud_rate
        self._max_buffer = config.max_sensor_buffer
        self._ser: Optional[serial.Serial] = None
        self._sensor1_history: deque = deque(maxlen=config.max_sensor_buffer)
        self._sensor2_history: deque = deque(maxlen=config.max_sensor_buffer)

    def connect(self) -> None:
        """Open the serial connection and list available ports for debugging."""
        self._list_available_ports()
        self._ser = serial.Serial(self._port, self._baud)
        logger.info("Serial port opened: %s @ %d baud", self._port, self._baud)

    def read_sensors(self) -> Optional[Tuple[int, int]]:
        """Read one line of sensor data from serial.

        Returns:
            Tuple of (sensor1, sensor2) integer values, or None on error.
        """
        if self._ser is None or not self._ser.is_open:
            return None

        try:
            line = self._ser.readline().decode("utf-8").strip()
            if not line:
                return None
            values = [int(x) for x in line.split(", ")]
            if len(values) < 2:
                return None

            s1, s2 = values[0], values[1]
            self._sensor1_history.append(s1)
            self._sensor2_history.append(s2)
            return (s1, s2)

        except (ValueError, UnicodeDecodeError) as e:
            logger.debug("Failed to parse serial data: %s", e)
            return None

    def close(self) -> None:
        """Close the serial connection."""
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            logger.info("Serial port closed")

    @staticmethod
    def _list_available_ports() -> None:
        """Log all available serial ports for debugging."""
        ports = list(list_ports.comports())
        if ports:
            logger.info("Available serial ports:")
            for port in ports:
                logger.info("  - %s", port.device)
        else:
            logger.warning("No serial ports found")
