from threading import Thread

from visual_tracking_gimbal.cli import apply_command, motor_control_step
from visual_tracking_gimbal.control import GimbalController
from visual_tracking_gimbal.hardware import FakeGimbal
from visual_tracking_gimbal.state import GimbalMode, SharedState


def test_shared_state_is_safe_for_concurrent_updates() -> None:
    state = SharedState()

    def worker(offset: int) -> None:
        for index in range(100):
            state.update(error_x=offset + index, target_found=True, confidence=0.8)

    threads = [Thread(target=worker, args=(offset,)) for offset in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    snapshot = state.snapshot()
    assert snapshot.target_found is True
    assert snapshot.confidence == 0.8


def test_cli_commands_update_state() -> None:
    state = SharedState()

    assert apply_command("servo on", state) == "servo-on"
    assert state.snapshot().servo_active is True
    assert apply_command("gui off", state) == "gui-off"
    assert state.snapshot().gui_enabled is False
    assert apply_command("search", state) == "search"
    assert state.snapshot().mode == GimbalMode.SCANNING
    assert apply_command("n", state) == "rejected"
    assert state.snapshot().mode == GimbalMode.ESCAPING
    assert apply_command("quit", state) == "stopping"
    assert state.snapshot().is_running is False


def test_motor_step_uses_fake_hardware_without_wiringpi() -> None:
    state = SharedState()
    state.update(servo_active=True, target_found=True, error_x=400, error_y=-80, confidence=0.9)
    fake = FakeGimbal()

    reason = motor_control_step(state, GimbalController(), fake)

    assert reason == "tracking"
    assert fake.yaw_history
    assert fake.pitch_history


def test_motor_step_stops_when_target_not_found() -> None:
    state = SharedState()
    state.update(servo_active=True, target_found=False, error_x=400, error_y=-80, confidence=0.9)
    fake = FakeGimbal()

    reason = motor_control_step(state, GimbalController(), fake)

    assert reason == "low-confidence"
    assert fake.yaw_history == [0.0]
    assert fake.pitch_history == []
