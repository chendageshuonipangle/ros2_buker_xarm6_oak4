"""Small SE(3) helpers shared by the calibration executables."""

from __future__ import annotations

import math

import cv2
import numpy as np


def inverse_transform(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def transform_from_rt(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return result


def transform_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return transform_from_rt(rotation, tvec)


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion, dtype=float).reshape(4)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return an XYZW quaternion for a proper 3x3 rotation matrix."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=float)
    return quaternion / np.linalg.norm(quaternion)


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        raise ValueError("at least one rotation is required")
    quaternions = [rotation_to_quaternion(rotation) for rotation in rotations]
    reference = quaternions[0]
    aligned = [q if np.dot(q, reference) >= 0 else -q for q in quaternions]
    _, vectors = np.linalg.eigh(sum(np.outer(q, q) for q in aligned))
    average = vectors[:, -1]
    if np.dot(average, reference) < 0:
        average = -average
    return quaternion_to_rotation(average)


def mean_transform(transforms: list[np.ndarray]) -> np.ndarray:
    return transform_from_rt(
        mean_rotation([transform[:3, :3] for transform in transforms]),
        np.mean([transform[:3, 3] for transform in transforms], axis=0),
    )


def rotation_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    delta = first[:3, :3].T @ second[:3, :3]
    cosine = float(np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def transform_from_ros(transform) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    return transform_from_rt(
        quaternion_to_rotation([rotation.x, rotation.y, rotation.z, rotation.w]),
        [translation.x, translation.y, translation.z],
    )
