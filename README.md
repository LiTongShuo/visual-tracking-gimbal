# Visual Tracking Gimbal

English | [中文](README.zh-CN.md)

[![Python Verification](https://github.com/LiTongShuo-edu/visual-tracking-gimbal/actions/workflows/python.yml/badge.svg)](https://github.com/LiTongShuo-edu/visual-tracking-gimbal/actions/workflows/python.yml)

A Python / OpenCV two-axis visual tracking gimbal project. Running on an Orange Pi 5 Pro 8G, it identifies a target in 720p USB camera frames, estimates its distance and displacement from the image center, and converts these into commands for a 3D-printed gimbal driven by two MG90S servos.

The project turns hardware debugging notes into publicly verifiable code: control logic is unit-testable, hardware output can be replaced by a mock controller, and shared state between vision, control, and terminal threads is protected by locks.

## Workflow

```text
Camera frame
    |
OpenCV grayscale, blur, edge detection, thresholding
    |
Contour hierarchy filtering and target center detection
    |
PnP distance estimation and error calculation
    |
Thread-safe shared state
    |
YAW software PWM + PITCH field-of-view mapping
    |
wiringpi hardware / FakeGimbal hardware-free testing
```

## Modules

| Module | Purpose |
| --- | --- |
| `visual_tracking_gimbal/vision.py` | OpenCV target detection, contour point ordering, PnP distance estimation |
| `visual_tracking_gimbal/control.py` | YAW software PWM, dead zone, limits, PITCH angle mapping and smoothing |
| `visual_tracking_gimbal/state.py` | Thread-safe state shared by vision, control, and terminal threads |
| `visual_tracking_gimbal/hardware.py` | `wiringpi` hardware control and `FakeGimbal` mock |
| `visual_tracking_gimbal/cli.py` | Terminal commands and control-step entry point |
| `tests/` | Hardware-free tests for control, state, CLI, and vision helpers |

## Hardware

Orange Pi 5 Pro 8G, a self-built 3D-printed two-axis gimbal, two MG90S servos, and a 720p USB camera. Hardware output uses Linux GPIO/PWM through `wiringpi`.

## Control strategy

YAW drives horizontal rotation using software PWM: larger errors increase the duty cycle, while entering the center dead zone stops output to reduce overshoot. PITCH maps vertical image error to a target angle using the field of view, then approaches it with smoothing rather than continuously accumulating commands. When confidence is insufficient or the servos are disabled, PITCH holds its angle and YAW output is zero.

## Installation and use

Python 3.12 is recommended:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
# Test dependencies:
.\.venv\Scripts\python.exe -m pip install -r requirements-test.txt

# Options, hardware-free demo, and interactive terminal:
.\.venv\Scripts\python.exe -m visual_tracking_gimbal --help
.\.venv\Scripts\python.exe -m visual_tracking_gimbal --demo-steps 20
.\.venv\Scripts\python.exe -m visual_tracking_gimbal
```

| Command | Purpose |
| --- | --- |
| `gui on` / `gui off` | Toggle GUI status |
| `servo on` / `servo off` | Enable or disable servo output |
| `search` | Enter scanning mode |
| `y` | Accept target and return to tracking |
| `n` | Reject target and enter escape mode |
| `status` | Print shared state |
| `q` / `quit` / `exit` | Exit safely |

For the hardware setup described above:

```powershell
.\.venv\Scripts\python.exe -m visual_tracking_gimbal --hardware --yaw-pin 21 --pitch-pin 22
```

Before connecting the gimbal, check wiring, voltage range, servo direction, and mechanical limits.

## Tests and validation

```powershell
.\.venv\Scripts\python.exe -m pytest
```

Tests cover YAW dead zone, limits and PWM duty cycle; PITCH limits, mapping and smoothing; protection during low confidence, target loss or disabled servos; concurrent `SharedState` updates and snapshots; `FakeGimbal`; CLI parsing; and vision helpers, PnP distance estimation and synthetic target detection. GitHub Actions runs the same hardware-free tests after commits.

## Documentation and scope

- [Debugging and optimization notes](docs/debug-notes.md)
- [Resume project summary](docs/resume-summary.md)
- [Sources and publication boundaries](docs/source-boundary.md)

This project demonstrates Python, OpenCV, device control, multithreaded state management, debugging, optimization, and automated testing. It does not claim gRPC/Protobuf, Appium, or UIAutomation2 experience; those require separate evidence. Original Gemini HTML exports and browser assets are retained only as local source material and are not core public project content.
