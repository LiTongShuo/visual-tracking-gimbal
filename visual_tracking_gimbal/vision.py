from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class VisionConfig:
    frame_width: int = 1280
    frame_height: int = 720
    canny_low: int = 50
    canny_high: int = 100
    threshold: int = 60
    blur_size: int = 5
    close_size: int = 3
    erode_size: int = 1
    min_parent_area: float = 1000.0
    target_width_mm: float = 297.0
    target_height_mm: float = 210.0


@dataclass(frozen=True)
class Detection:
    found: bool
    center: tuple[int, int] | None = None
    error_x: float = 0.0
    error_y: float = 0.0
    distance_mm: float | None = None
    confidence: float = 0.0
    contour: np.ndarray | None = None


class EMAFilter:
    def __init__(self, alpha: float = 0.3) -> None:
        self.alpha = alpha
        self._last: np.ndarray | None = None

    def update(self, value: tuple[float, float]) -> tuple[float, float]:
        current = np.asarray(value, dtype=np.float32)
        if self._last is None:
            self._last = current
        else:
            self._last = self.alpha * current + (1.0 - self.alpha) * self._last
        return float(self._last[0]), float(self._last[1])


def order_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if pts.shape[0] != 4:
        raise ValueError("order_points expects exactly four 2D points")
    rect = np.zeros((4, 2), dtype=np.float32)
    sums = pts.sum(axis=1)
    rect[0] = pts[np.argmin(sums)]
    rect[2] = pts[np.argmax(sums)]
    diffs = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diffs)]
    rect[3] = pts[np.argmax(diffs)]
    return rect


def estimate_distance_mm(
    image_points: np.ndarray,
    config: VisionConfig,
    camera_matrix: np.ndarray | None = None,
    dist_coeffs: np.ndarray | None = None,
) -> float | None:
    image_points = order_points(image_points)
    object_points = np.array(
        [
            [-config.target_width_mm / 2, -config.target_height_mm / 2, 0.0],
            [config.target_width_mm / 2, -config.target_height_mm / 2, 0.0],
            [config.target_width_mm / 2, config.target_height_mm / 2, 0.0],
            [-config.target_width_mm / 2, config.target_height_mm / 2, 0.0],
        ],
        dtype=np.float32,
    )
    if camera_matrix is None:
        camera_matrix = np.array(
            [
                [1000.0, 0.0, config.frame_width / 2],
                [0.0, 1000.0, config.frame_height / 2],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
    if dist_coeffs is None:
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    ok, _rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    if not ok:
        return None
    return float(np.linalg.norm(tvec))


def detect_target(frame: np.ndarray, config: VisionConfig | None = None) -> Detection:
    if frame is None or frame.size == 0:
        raise ValueError("frame must be a non-empty image")

    cfg = config or VisionConfig(frame_width=frame.shape[1], frame_height=frame.shape[0])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    blur_size = max(1, cfg.blur_size // 2 * 2 + 1)
    blurred = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
    edges = cv2.Canny(blurred, cfg.canny_low, cfg.canny_high)
    _, binary = cv2.threshold(blurred, cfg.threshold, 255, cv2.THRESH_BINARY_INV)
    combined = cv2.bitwise_or(edges, binary)
    closed = cv2.morphologyEx(
        combined,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (cfg.close_size, cfg.close_size)),
    )
    mask = cv2.erode(
        closed,
        cv2.getStructuringElement(cv2.MORPH_RECT, (cfg.erode_size, cfg.erode_size)),
        iterations=1,
    )

    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    target = _select_target_contour(contours, hierarchy, cfg)
    if target is None:
        return Detection(found=False)

    contour = _ensure_quad(target)
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return Detection(found=False)

    center_x = int(moments["m10"] / moments["m00"])
    center_y = int(moments["m01"] / moments["m00"])
    distance = estimate_distance_mm(contour.reshape(-1, 2), cfg) if len(contour) == 4 else None
    area = cv2.contourArea(contour)
    confidence = float(min(1.0, area / (cfg.frame_width * cfg.frame_height * 0.12)))
    return Detection(
        found=True,
        center=(center_x, center_y),
        error_x=center_x - cfg.frame_width / 2,
        error_y=center_y - cfg.frame_height / 2,
        distance_mm=distance,
        confidence=confidence,
        contour=contour,
    )


def _select_target_contour(
    contours: tuple[np.ndarray, ...],
    hierarchy: np.ndarray | None,
    config: VisionConfig,
) -> np.ndarray | None:
    if hierarchy is None:
        return None
    for index, contour in enumerate(contours):
        parent_index = hierarchy[0][index][3]
        if parent_index == -1:
            continue
        child_area = cv2.contourArea(contour)
        parent = contours[parent_index]
        parent_area = cv2.contourArea(parent)
        if parent_area <= config.min_parent_area:
            continue
        ratio = child_area / parent_area if parent_area else 0.0
        if not 0.005 < ratio < 0.5:
            continue
        approx = cv2.approxPolyDP(parent, 0.03 * cv2.arcLength(parent, True), True)
        if 4 <= len(approx) <= 6 and cv2.isContourConvex(approx):
            x, y, width, height = cv2.boundingRect(approx)
            if height > 0 and 0.5 <= float(width) / height <= 2.0:
                return approx
    return None


def _ensure_quad(contour: np.ndarray) -> np.ndarray:
    if len(contour) == 4:
        return contour.reshape(4, 2).astype(np.float32)
    rect = cv2.minAreaRect(contour)
    return cv2.boxPoints(rect).astype(np.float32)
