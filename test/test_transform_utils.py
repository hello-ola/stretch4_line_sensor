"""Unit tests for transform helpers."""

import numpy as np

from stretch4_line_sensor.transform_utils import (
    transform_point,
    translation_rotation_to_matrix,
)


def test_translation_rotation_to_matrix_identity():
    mat = translation_rotation_to_matrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(mat, np.eye(4), atol=1e-9)


def test_translation_rotation_to_matrix_translation():
    mat = translation_rotation_to_matrix(0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    point = transform_point(mat, np.array([0.35, 0.0, 0.05]))
    np.testing.assert_allclose(point, np.array([0.50, 0.0, 0.05]), atol=1e-9)
