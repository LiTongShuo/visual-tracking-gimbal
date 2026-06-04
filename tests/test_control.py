from visual_tracking_gimbal.control import ControlConfig, GimbalController


def test_servo_disabled_freezes_output() -> None:
    controller = GimbalController()
    output = controller.update(300, 100, 900, 0.9, servo_active=False)

    assert output.yaw_speed == 0.0
    assert output.pitch_angle == controller.pitch_angle
    assert output.reason == "servo-disabled"


def test_low_confidence_stops_yaw_and_freezes_pitch() -> None:
    controller = GimbalController()
    before = controller.pitch_angle
    output = controller.update(300, 100, 900, 0.1, servo_active=True)

    assert output.yaw_speed == 0.0
    assert output.pitch_angle == before
    assert output.reason == "low-confidence"


def test_yaw_dead_zone_stops_command() -> None:
    controller = GimbalController(ControlConfig(yaw_dead_zone_px=30))
    output = controller.update(20, 100, 900, 0.9, servo_active=True)

    assert output.yaw_speed == 0.0


def test_yaw_software_pwm_limits_duty() -> None:
    controller = GimbalController(ControlConfig(pwm_period_frames=4, pwm_amplitude=40, yaw_kp=1.0))

    speeds = [
        controller.update(50, 0, 900, 0.9, servo_active=True).yaw_speed
        for _ in range(4)
    ]

    assert speeds.count(40) == 2
    assert speeds.count(0.0) == 2


def test_pitch_smoothing_and_limits() -> None:
    controller = GimbalController(
        ControlConfig(pitch_center=94, pitch_min=80, pitch_max=100, pitch_smoothing=0.5)
    )
    output = controller.update(0, -300, 2000, 0.9, servo_active=True)

    assert 94 < output.pitch_angle <= 100
    assert controller.pitch_angle == output.pitch_angle


def test_scanning_uses_fixed_pwm_without_pitch_change() -> None:
    controller = GimbalController(ControlConfig(pwm_period_frames=4, scan_speed=30, scan_duty_cycle=0.5))
    outputs = [
        controller.update(0, 0, None, 0.0, servo_active=True, scanning=True)
        for _ in range(4)
    ]

    assert [item.yaw_speed for item in outputs].count(30) == 2
    assert all(item.reason == "scanning" for item in outputs)
