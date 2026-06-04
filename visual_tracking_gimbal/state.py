from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from threading import Lock


class GimbalMode(str, Enum):
    TRACKING = "tracking"
    SCANNING = "scanning"
    VALIDATING = "validating"
    WAITING_USER = "waiting_user"
    ESCAPING = "escaping"


@dataclass(frozen=True)
class TrackingSnapshot:
    error_x: float = 0.0
    error_y: float = 0.0
    distance_mm: float | None = None
    confidence: float = 0.0
    target_found: bool = False
    mode: GimbalMode = GimbalMode.TRACKING
    servo_active: bool = False
    gui_enabled: bool = True
    is_running: bool = True


class SharedState:
    """Thread-safe holder for vision, motor and terminal command state."""

    def __init__(self, initial: TrackingSnapshot | None = None) -> None:
        self._lock = Lock()
        self._snapshot = initial or TrackingSnapshot()

    def snapshot(self) -> TrackingSnapshot:
        with self._lock:
            return replace(self._snapshot)

    def update(self, **changes: object) -> TrackingSnapshot:
        with self._lock:
            self._snapshot = replace(self._snapshot, **changes)
            return replace(self._snapshot)

    def stop(self) -> TrackingSnapshot:
        return self.update(is_running=False)
