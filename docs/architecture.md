# System Architecture

## Overview

The Follow-Me Drone system consists of two independent processes communicating via an atomic file-based IPC channel.

```
┌─────────────────────────────────────────────────────────────┐
│                    Follow-Me Drone System                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Process 1: run_tracker.py                                  │
│  ├─ DroneConnection   → DJI Tello (WiFi)                   │
│  ├─ FaceDetector      → Haar Cascade face detection         │
│  ├─ FaceTracker       → Dual PID control (yaw + vertical)  │
│  ├─ CircleRecorder    → Panoramic video recording           │
│  └─ CommandChannel    ← Reads gesture commands              │
│                                                             │
│  Process 2: run_gesture.py                                  │
│  ├─ SerialIMUReader   → Teensy 4.1 (USB serial)            │
│  ├─ SnapDetector      → State machine gesture recognition   │
│  └─ CommandChannel    → Writes gesture commands             │
│                                                             │
│  IPC: cache.pkl (atomic write via temp file + os.replace)   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

### Face Tracking Pipeline

```
Camera Frame
    │
    ▼
FaceDetector.detect(frame)
    │
    ├─ No face → FaceTracker._handle_lost_face()
    │                │
    │                └─ Rotate to search (after threshold frames)
    │
    └─ FaceInfo (center_x, center_y, area)
         │
         ▼
    FaceTracker.compute_control(face, drone)
         │
         ├─ Yaw PID:     error = center_x - target_x  → yaw speed
         ├─ Vertical PID: error = center_y - target_y  → up/down speed
         └─ Area control: area vs [min, max] range     → forward/back
              │
              ▼
    DroneConnection.send_control(lr, fb, ud, yaw)
```

### Gesture Command Pipeline

```
MPU6050 Sensors (x2)
    │
    ▼
Teensy 4.1 (USB Serial @ 38400 baud)
    │
    ▼
SerialIMUReader.read_sensors() → (sensor1, sensor2)
    │
    ▼
SnapDetector.process_reading(s1, s2)
    │
    ├─ diff > threshold & valid → Snap detected, increment count
    ├─ Within time window       → Continue counting
    └─ Time window expired      → Emit snap count
         │
         ▼
    CommandChannel.write_commands([...snap counts...])
         │
         ▼
    cache.pkl (atomic write)
         │
         ▼
    CommandChannel.read_new_commands() [in tracker process]
         │
         ▼
    Command.CIRCLE_MOTION (1 snap) or Command.TAKE_PHOTO (2 snaps)
```

## PID Controller

The system uses a standard PID controller:

```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de/dt
```

Where:
- `Kp = 0.2` (proportional gain)
- `Ki = 0.04` (integral gain)
- `Kd = 0.005` (derivative gain)

Two independent PID instances control:
1. **Yaw axis**: Rotates drone left/right to center face horizontally
2. **Vertical axis**: Moves drone up/down to keep face at upper-quarter of frame (scaled by 0.5x)

Forward/backward movement uses simpler threshold logic based on face bounding box area.

## Safety Mechanisms

- **Height limit**: Drone descends if it exceeds the configured ceiling height (default: 220cm)
- **Graceful shutdown**: SIGINT/SIGTERM handlers safely land the drone before exit
- **Lost face recovery**: After a configurable number of lost frames, the drone rotates in the last known direction of the face
- **Atomic IPC**: File writes use temp-file-then-rename to prevent corruption from concurrent access

## Configuration

All tunable parameters are centralized in `config/default.yaml` and loaded into frozen dataclasses at startup. This prevents accidental mutation of shared configuration state during runtime.
