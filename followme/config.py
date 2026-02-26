"""Centralized configuration with typed dataclasses and YAML loading."""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

logger = logging.getLogger(__name__)

# Project root: two levels up from this file (followme/config.py -> follow-me-drone/)
PROJECT_ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class PIDConfig:
    kp: float = 0.2
    ki: float = 0.04
    kd: float = 0.005


@dataclass(frozen=True)
class DroneConfig:
    frame_width: int = 960
    frame_height: int = 720
    height_limit: int = 220
    takeoff_ascent_speed: int = 25
    takeoff_ascent_duration: float = 3.0


@dataclass(frozen=True)
class TrackingConfig:
    pid: PIDConfig = field(default_factory=PIDConfig)
    face_area_min: int = 14000
    face_area_max: int = 15000
    yaw_speed_limit: int = 100
    vertical_speed_limit: int = 40
    vertical_speed_scale: float = 0.5
    forward_speed: int = 20
    backward_speed: int = -20
    target_x_fraction: float = 0.5
    target_y_fraction: float = 0.25
    lost_face_threshold: int = 15
    search_rotation_speed: int = 40
    loop_interval: float = 0.1


@dataclass(frozen=True)
class FaceDetectionConfig:
    cascade_path: str = "models/haarcascade_frontalface_default.xml"
    scale_factor: float = 1.2
    min_neighbors: int = 8
    center_history_size: int = 20


@dataclass(frozen=True)
class CircleMotionConfig:
    speed: int = -15
    yaw_speed: int = 35
    duration: int = 20
    video_fps: int = 30


@dataclass(frozen=True)
class GestureConfig:
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 38400
    snap_threshold: int = 50000
    snap_time_window: float = 0.7
    max_sensor_buffer: int = 1000


@dataclass(frozen=True)
class IPCConfig:
    cache_file: str = "cache.pkl"


@dataclass(frozen=True)
class AppConfig:
    drone: DroneConfig = field(default_factory=DroneConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    face_detection: FaceDetectionConfig = field(default_factory=FaceDetectionConfig)
    circle_motion: CircleMotionConfig = field(default_factory=CircleMotionConfig)
    gesture: GestureConfig = field(default_factory=GestureConfig)
    ipc: IPCConfig = field(default_factory=IPCConfig)


def _build_nested(cls, data: dict):
    """Recursively build a frozen dataclass from a dict, handling nested dataclasses."""
    if data is None:
        return cls()
    fields = {f.name: f for f in cls.__dataclass_fields__.values()}
    kwargs = {}
    for key, value in data.items():
        if key in fields and hasattr(fields[key].type, "__dataclass_fields__"):
            # Resolve the actual class for nested dataclasses
            nested_cls = _resolve_type(fields[key].type)
            if nested_cls:
                kwargs[key] = _build_nested(nested_cls, value)
                continue
        if key in fields:
            kwargs[key] = value
    return cls(**kwargs)


def _resolve_type(type_hint) -> Optional[type]:
    """Resolve a type hint string or type to an actual class."""
    type_map = {
        "PIDConfig": PIDConfig,
        "DroneConfig": DroneConfig,
        "TrackingConfig": TrackingConfig,
        "FaceDetectionConfig": FaceDetectionConfig,
        "CircleMotionConfig": CircleMotionConfig,
        "GestureConfig": GestureConfig,
        "IPCConfig": IPCConfig,
    }
    if isinstance(type_hint, str):
        return type_map.get(type_hint)
    if isinstance(type_hint, type) and hasattr(type_hint, "__dataclass_fields__"):
        return type_hint
    return None


def load_config(path: Optional[str] = None) -> AppConfig:
    """Load configuration from a YAML file, falling back to defaults.

    Args:
        path: Path to YAML config file. If None, looks for config/default.yaml
              relative to the project root.

    Returns:
        Populated AppConfig instance with all settings.
    """
    if path is None:
        path = str(PROJECT_ROOT / "config" / "default.yaml")

    resolved = Path(path)
    if not resolved.is_absolute():
        resolved = PROJECT_ROOT / resolved

    if not resolved.exists():
        logger.warning("Config file not found at %s, using defaults", resolved)
        return AppConfig()

    with open(resolved) as f:
        raw = yaml.safe_load(f) or {}

    return _build_nested(AppConfig, raw)
