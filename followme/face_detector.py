"""Face detection module using OpenCV Haar Cascade classifier."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from followme.config import FaceDetectionConfig, PROJECT_ROOT

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class FaceInfo:
    """Detected face information."""

    center_x: int
    center_y: int
    area: int
    bbox: Tuple[int, int, int, int]  # (x, y, w, h)


class FaceDetector:
    """Detects faces in video frames using a Haar Cascade classifier.

    The classifier is loaded once on initialization for efficiency,
    rather than being reloaded on every frame.

    Args:
        config: Face detection configuration parameters.
    """

    def __init__(self, config: FaceDetectionConfig) -> None:
        self._config = config
        cascade_path = self._resolve_cascade_path(config.cascade_path)
        self._cascade = cv2.CascadeClassifier(str(cascade_path))
        if self._cascade.empty():
            raise FileNotFoundError(
                f"Failed to load Haar Cascade from {cascade_path}"
            )
        logger.info("Loaded face cascade from %s", cascade_path)

    @staticmethod
    def _resolve_cascade_path(path_str: str) -> Path:
        """Resolve cascade file path, trying project-relative then absolute."""
        path = Path(path_str)
        if path.is_absolute() and path.exists():
            return path

        # Try relative to project root
        project_path = PROJECT_ROOT / path_str
        if project_path.exists():
            return project_path

        # Try relative to current working directory
        cwd_path = Path.cwd() / path_str
        if cwd_path.exists():
            return cwd_path

        raise FileNotFoundError(
            f"Cascade file not found at {path_str}, "
            f"{project_path}, or {cwd_path}"
        )

    def detect(self, frame: np.ndarray) -> Optional[FaceInfo]:
        """Detect the largest face in the frame.

        Args:
            frame: BGR image from the camera.

        Returns:
            FaceInfo for the largest detected face, or None if no face found.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self._cascade.detectMultiScale(
            gray,
            scaleFactor=self._config.scale_factor,
            minNeighbors=self._config.min_neighbors,
        )

        if len(faces) == 0:
            return None

        # Select the largest face by area
        areas = [w * h for (_, _, w, h) in faces]
        idx = int(np.argmax(areas))
        x, y, w, h = faces[idx]

        return FaceInfo(
            center_x=x + w // 2,
            center_y=y + h // 2,
            area=w * h,
            bbox=(int(x), int(y), int(w), int(h)),
        )

    @staticmethod
    def draw_annotations(frame: np.ndarray, face: FaceInfo) -> None:
        """Draw bounding box and center marker on the frame.

        Args:
            frame: BGR image to annotate (modified in place).
            face: Detected face information.
        """
        x, y, w, h = face.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.circle(frame, (face.center_x, face.center_y), 5, (0, 255, 0), cv2.FILLED)
        cv2.putText(
            frame,
            f"area={face.area}",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
