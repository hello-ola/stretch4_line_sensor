"""Helpers for transforming obstacle-filter centroids."""

from __future__ import annotations

import numpy as np


def translation_rotation_to_matrix(
    tx: float,
    ty: float,
    tz: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> np.ndarray:
    """Build a 4x4 homogeneous matrix from translation and quaternion."""
    x, y, z, w = qx, qy, qz, qw
    rot = np.array([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])

    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = [tx, ty, tz]
    return mat


def transform_point(matrix: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to a 3D point."""
    hom = np.array([point[0], point[1], point[2], 1.0], dtype=np.float64)
    return (matrix @ hom)[:3]
