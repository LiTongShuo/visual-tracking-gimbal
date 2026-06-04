import cv2
import numpy as np
import pytest

from visual_tracking_gimbal.vision import VisionConfig, detect_target, estimate_distance_mm, order_points


def test_order_points_returns_stable_order() -> None:
    points = np.array([[20, 20], [10, 10], [20, 10], [10, 20]], dtype=np.float32)

    ordered = order_points(points)

    assert ordered.tolist() == [[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]]


def test_order_points_rejects_wrong_shape() -> None:
    with pytest.raises(ValueError):
        order_points(np.array([[0, 0], [1, 1], [2, 2]], dtype=np.float32))


def test_estimate_distance_returns_positive_value() -> None:
    config = VisionConfig(frame_width=1280, frame_height=720)
    image_points = np.array([[540, 310], [740, 310], [740, 450], [540, 450]], dtype=np.float32)

    distance = estimate_distance_mm(image_points, config)

    assert distance is not None
    assert distance > 0


def test_detect_target_rejects_empty_frame() -> None:
    with pytest.raises(ValueError):
        detect_target(np.array([], dtype=np.uint8))


def test_detect_target_finds_nested_rectangular_target() -> None:
    frame = np.full((240, 320, 3), 255, dtype=np.uint8)
    cv2.rectangle(frame, (110, 70), (210, 170), (0, 0, 0), -1)
    cv2.rectangle(frame, (145, 105), (175, 135), (255, 255, 255), -1)

    detection = detect_target(
        frame,
        VisionConfig(
            frame_width=320,
            frame_height=240,
            threshold=200,
            min_parent_area=500,
            close_size=3,
            erode_size=1,
        ),
    )

    assert detection.found is True
    assert detection.center is not None
    assert abs(detection.error_x) < 20
    assert abs(detection.error_y) < 20
