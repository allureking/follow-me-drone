# Follow-Me Drone

Autonomous follow-me selfie drone system with real-time face tracking and gesture control, built for the DJI Tello.

Designed for solo athletes and content creators who need hands-free recording during activities like mountain biking, kayaking, or other sports where both hands are occupied.

## Features

- **Real-time face tracking** with PID-controlled 3-axis movement (yaw, vertical, forward/backward)
- **Gesture commands** via IMU snap detection for hands-free control
  - Single snap: trigger circular panoramic video recording
  - Double snap: capture a still photo
- **Lost-face recovery** -- automatically rotates to search when the subject moves out of frame
- **Indoor safety** -- configurable height limit to prevent ceiling collisions
- **Modular architecture** -- cleanly separated modules for detection, control, gestures, and IPC

## Hardware Requirements

| Component | Purpose |
|-----------|---------|
| DJI Tello Drone | Flight platform with camera |
| Teensy 4.1 Microcontroller | Sensor data acquisition via USB serial |
| 2x MPU6050 IMU Sensors | Acceleration-based snap gesture detection |

## Software Requirements

- Python 3.9+
- WiFi connection to DJI Tello
- USB connection to Teensy 4.1

## Installation

```bash
git clone https://github.com/allureking/follow-me-drone.git
cd follow-me-drone
pip install -r requirements.txt
```

## Usage

Run two scripts in separate terminals:

**Terminal 1** -- Start gesture recognition (reads IMU sensor data):
```bash
python scripts/run_gesture.py
```

**Terminal 2** -- Start face tracking and drone control:
```bash
python scripts/run_tracker.py
```

### CLI Options

| Flag | Description |
|------|-------------|
| `--config PATH` | Custom YAML config file (default: `config/default.yaml`) |
| `--debug` | Enable verbose debug logging |

Press `q` in the video window to land and exit.

## Configuration

All parameters are centralized in [`config/default.yaml`](config/default.yaml), including:
- PID controller gains
- Face detection thresholds
- Drone speed limits and safety parameters
- Serial port settings
- Circle motion recording parameters

## Project Structure

```
follow-me-drone/
├── config/
│   └── default.yaml              # All configurable parameters
├── models/
│   └── haarcascade_frontalface_default.xml
├── followme/
│   ├── __init__.py
│   ├── config.py                 # Typed dataclasses + YAML loader
│   ├── pid.py                    # Generic PID controller
│   ├── face_detector.py          # Haar Cascade face detection
│   ├── drone_controller.py       # Drone connection, tracking, recording
│   ├── gesture.py                # Snap detection + serial IMU reader
│   ├── ipc.py                    # Atomic file-based command channel
│   ├── commands.py               # Command enum definitions
│   └── utils.py                  # Logging and signal handling
├── scripts/
│   ├── run_tracker.py            # Face tracking entry point
│   └── run_gesture.py            # Gesture recognition entry point
└── docs/
    └── architecture.md           # System architecture documentation
```

## How It Works

1. **Face Detection**: Each video frame is processed by a Haar Cascade classifier to locate the largest face, returning its center coordinates and bounding box area.

2. **PID Tracking**: Two PID controllers compute yaw (horizontal rotation) and vertical speed to center the face at the target position. Forward/backward movement is controlled by maintaining the face within a target area range.

3. **Gesture Recognition**: A separate process reads dual IMU sensors via serial. When the acceleration difference between sensors exceeds a threshold, a "snap" is detected. Consecutive snaps within a time window form a gesture command.

4. **Inter-Process Communication**: Gesture commands are written atomically to a pickle cache file, which the tracking controller polls for new entries.

## Acknowledgments

Based on an EECS 206A (Introduction to Robotics) team project at UC Berkeley.

## License

MIT License. See [LICENSE](LICENSE) for details.
