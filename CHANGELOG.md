# Changelog

## [1.0.0] - 2026-02-26

### Added
- Modular Python package architecture with `followme` namespace
- Generic PID controller with anti-windup protection and output clamping
- `FaceDetector` class with one-time Haar Cascade loading
- `DroneConnection` lifecycle management with safe takeoff/landing
- `FaceTracker` with dual-axis PID control and lost-face recovery
- `CircleRecorder` for panoramic video recording during circular flight
- `SnapDetector` state machine for IMU-based gesture recognition
- `SerialIMUReader` with bounded memory buffers
- `CommandChannel` for atomic file-based inter-process communication
- `Command` enum replacing magic numbers for drone actions
- Centralized YAML configuration system with typed dataclasses
- CLI entry points with `--config` and `--debug` flags
- Comprehensive documentation with system architecture guide

### Fixed
- Haar Cascade classifier was reloaded on every single frame (performance)
- Variable name mismatch (`me` vs `tello`) would crash circle motion
- Unbounded sensor arrays caused memory leak during long sessions
- No graceful shutdown on interrupt signals

### Changed
- All hardcoded values extracted to `config/default.yaml`
- `print()` statements replaced with Python `logging` module
- Raw pickle IPC wrapped in `CommandChannel` abstraction
- Global state replaced with encapsulated classes and type hints
