from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol


class GimbalHardware(Protocol):
    def set_yaw_speed(self, speed: float) -> None:
        ...

    def set_pitch_angle(self, angle: float) -> None:
        ...

    def safe_stop(self) -> None:
        ...


@dataclass
class FakeGimbal:
    yaw_history: list[float] = field(default_factory=list)
    pitch_history: list[float] = field(default_factory=list)
    stopped: bool = False

    def set_yaw_speed(self, speed: float) -> None:
        self.yaw_history.append(speed)

    def set_pitch_angle(self, angle: float) -> None:
        self.pitch_history.append(angle)

    def safe_stop(self) -> None:
        self.stopped = True
        self.set_yaw_speed(0.0)


class WiringPiGimbal:
    def __init__(
        self,
        *,
        yaw_pin: int,
        pitch_pin: int,
        pitch_center: float = 94.0,
        yaw_trim: int = 0,
    ) -> None:
        import wiringpi

        self.wp = wiringpi
        self.yaw_pin = yaw_pin
        self.pitch_pin = pitch_pin
        self.pitch_center = pitch_center
        self.yaw_trim = yaw_trim

        self.wp.wiringPiSetup()
        for pin in (self.yaw_pin, self.pitch_pin):
            self.wp.pinMode(pin, self.wp.PWM_OUTPUT)
            self.wp.pwmSetMode(pin, self.wp.PWM_MODE_MS)
            self.wp.pwmSetClock(pin, 240)
            self.wp.pwmSetRange(pin, 2000)
        self.set_yaw_speed(0.0)
        self.set_pitch_angle(self.pitch_center)

    def set_yaw_speed(self, speed: float) -> None:
        speed = max(-100.0, min(100.0, speed))
        duty = int(150 + (speed / 100.0) * 100 + self.yaw_trim)
        self.wp.pwmWrite(self.yaw_pin, duty)

    def set_pitch_angle(self, angle: float) -> None:
        angle = max(0.0, min(180.0, angle))
        duty = int(50 + (angle / 180.0) * 200)
        self.wp.pwmWrite(self.pitch_pin, duty)

    def safe_stop(self) -> None:
        self.set_yaw_speed(0.0)
        self.set_pitch_angle(self.pitch_center)
        self.wp.pinMode(self.yaw_pin, self.wp.INPUT)
        self.wp.pinMode(self.pitch_pin, self.wp.INPUT)


def create_gimbal(*, use_hardware: bool, yaw_pin: int = 21, pitch_pin: int = 22) -> GimbalHardware:
    if use_hardware:
        return WiringPiGimbal(yaw_pin=yaw_pin, pitch_pin=pitch_pin)
    return FakeGimbal()
