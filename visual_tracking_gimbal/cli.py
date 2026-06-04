from __future__ import annotations

import argparse
import time

from .control import GimbalController
from .hardware import GimbalHardware, create_gimbal
from .state import GimbalMode, SharedState


def apply_command(command: str, state: SharedState) -> str:
    parts = command.strip().lower().split()
    if not parts:
        return "empty"

    cmd = parts[0]
    if cmd in {"q", "quit", "exit"}:
        state.stop()
        return "stopping"
    if cmd == "gui" and len(parts) == 2 and parts[1] in {"on", "off"}:
        state.update(gui_enabled=parts[1] == "on")
        return f"gui-{parts[1]}"
    if cmd == "servo" and len(parts) == 2 and parts[1] in {"on", "off"}:
        state.update(servo_active=parts[1] == "on")
        return f"servo-{parts[1]}"
    if cmd == "search":
        state.update(mode=GimbalMode.SCANNING)
        return "search"
    if cmd == "y":
        state.update(mode=GimbalMode.TRACKING)
        return "accepted"
    if cmd == "n":
        state.update(mode=GimbalMode.ESCAPING)
        return "rejected"
    if cmd == "status":
        return str(state.snapshot())
    return "unknown"


def motor_control_step(state: SharedState, controller: GimbalController, gimbal: GimbalHardware) -> str:
    snapshot = state.snapshot()
    output = controller.update(
        snapshot.error_x,
        snapshot.error_y,
        snapshot.distance_mm,
        snapshot.confidence if snapshot.target_found else 0.0,
        servo_active=snapshot.servo_active,
        scanning=snapshot.mode == GimbalMode.SCANNING,
    )
    gimbal.set_yaw_speed(output.yaw_speed)
    if output.pitch_enabled:
        gimbal.set_pitch_angle(output.pitch_angle)
    return output.reason


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="OpenCV visual tracking gimbal controller")
    parser.add_argument("--hardware", action="store_true", help="use wiringpi hardware output")
    parser.add_argument("--yaw-pin", type=int, default=21, help="wiringPi pin for continuous YAW servo")
    parser.add_argument("--pitch-pin", type=int, default=22, help="wiringPi pin for PITCH servo")
    parser.add_argument("--demo-steps", type=int, default=0, help="run a short no-camera fake-control demo")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    state = SharedState()
    controller = GimbalController()
    gimbal = create_gimbal(use_hardware=args.hardware, yaw_pin=args.yaw_pin, pitch_pin=args.pitch_pin)

    if args.demo_steps:
        state.update(servo_active=True, target_found=True, error_x=180.0, error_y=-40.0, confidence=0.9)
        for _ in range(args.demo_steps):
            motor_control_step(state, controller, gimbal)
            time.sleep(0.005)
        gimbal.safe_stop()
        return 0

    print("Visual tracking gimbal CLI. Commands: gui on/off, servo on/off, search, y, n, status, quit")
    try:
        while state.snapshot().is_running:
            result = apply_command(input("> "), state)
            print(result)
    finally:
        gimbal.safe_stop()
    return 0
