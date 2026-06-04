from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class ControlConfig:
    frame_width: int = 1280
    frame_height: int = 720
    pitch_center: float = 94.0
    pitch_min: float = 30.0
    pitch_max: float = 150.0
    fov_ratio: float = 0.0625
    pitch_distance_gain: float = 0.15
    pitch_smoothing: float = 0.25
    yaw_dead_zone_px: int = 25
    pitch_dead_zone_px: int = 12
    yaw_max_speed: float = 100.0
    yaw_kp: float = 0.14
    pwm_amplitude: float = 38.0
    pwm_period_frames: int = 8
    min_track_confidence: float = 0.55
    scan_speed: float = 20.0
    scan_duty_cycle: float = 0.45


@dataclass(frozen=True)
class ControlOutput:
    yaw_speed: float
    pitch_angle: float
    yaw_enabled: bool
    pitch_enabled: bool
    reason: str


class GimbalController:
    """Converts tracking snapshots into safe two-axis gimbal commands."""

    def __init__(self, config: ControlConfig | None = None) -> None:
        self.config = config or ControlConfig()
        self._pitch_angle = self.config.pitch_center
        self._pwm_frame = 0

    @property
    def pitch_angle(self) -> float:
        return self._pitch_angle

    def reset(self) -> None:
        self._pitch_angle = self.config.pitch_center
        self._pwm_frame = 0

    def update(
        self,
        error_x: float,
        error_y: float,
        distance_mm: float | None,
        confidence: float,
        *,
        servo_active: bool,
        scanning: bool = False,
    ) -> ControlOutput:
        if not servo_active:
            return ControlOutput(0.0, self._pitch_angle, False, False, "servo-disabled")

        if scanning:
            yaw_speed = self._software_pwm(self.config.scan_speed, self.config.scan_duty_cycle)
            return ControlOutput(yaw_speed, self._pitch_angle, yaw_speed != 0.0, False, "scanning")

        if confidence < self.config.min_track_confidence:
            return ControlOutput(0.0, self._pitch_angle, False, False, "low-confidence")

        yaw_speed = self._yaw_command(error_x)
        previous_pitch = self._pitch_angle
        pitch_angle = self._pitch_command(error_y, distance_mm)
        return ControlOutput(
            yaw_speed,
            pitch_angle,
            yaw_speed != 0.0,
            pitch_angle != previous_pitch,
            "tracking",
        )

    def _yaw_command(self, error_x: float) -> float:
        if abs(error_x) <= self.config.yaw_dead_zone_px:
            self._pwm_frame = 0
            return 0.0

        ideal_speed = clamp(
            error_x * self.config.yaw_kp,
            -self.config.yaw_max_speed,
            self.config.yaw_max_speed,
        )
        duty = clamp(abs(ideal_speed) / self.config.yaw_max_speed, 0.0, 1.0)
        return self._software_pwm(self.config.pwm_amplitude, duty) * (1 if ideal_speed > 0 else -1)

    def _software_pwm(self, amplitude: float, duty: float) -> float:
        period = max(1, self.config.pwm_period_frames)
        on_frames = round(clamp(duty, 0.0, 1.0) * period)
        active = (self._pwm_frame % period) < on_frames
        self._pwm_frame = (self._pwm_frame + 1) % period
        return amplitude if active and on_frames > 0 else 0.0

    def _pitch_command(self, error_y: float, distance_mm: float | None) -> float:
        if abs(error_y) <= self.config.pitch_dead_zone_px:
            return self._pitch_angle

        distance_gain = 1.0
        if distance_mm is not None and distance_mm > 0:
            distance_gain += self.config.pitch_distance_gain * clamp((distance_mm - 800.0) / 1200.0, -1.0, 1.0)

        target_angle = self.config.pitch_center - error_y * self.config.fov_ratio * distance_gain
        target_angle = clamp(target_angle, self.config.pitch_min, self.config.pitch_max)
        self._pitch_angle += (target_angle - self._pitch_angle) * self.config.pitch_smoothing
        self._pitch_angle = clamp(self._pitch_angle, self.config.pitch_min, self.config.pitch_max)
        return self._pitch_angle
