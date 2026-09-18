from __future__ import annotations

import numpy as np
import pytest

from dog_patrol_perception_face.recognition import prepare_recognition_input


def test_mobilefacenet_preprocessing_converts_bgr_to_normalized_rgb() -> None:
    image = np.array([[[0, 127.5, 255]]], dtype=np.float32)

    blob = prepare_recognition_input(
        image, (1, 3, 1, 1)
    )

    assert blob.shape == (1, 3, 1, 1)
    assert blob[0, :, 0, 0] == pytest.approx([1.0, 0.0, -1.0])


def test_mobilefacenet_preprocessing_supports_nhwc_engine() -> None:
    image = np.zeros((2, 3, 3), dtype=np.uint8)

    blob = prepare_recognition_input(
        image, (1, 2, 3, 3)
    )

    assert blob.shape == (1, 2, 3, 3)
    assert np.all(blob == -1.0)


def test_recognition_preprocessing_rejects_wrong_image_shape() -> None:
    with pytest.raises(ValueError, match="does not match engine input"):
        prepare_recognition_input(
            np.zeros((111, 112, 3), dtype=np.uint8),
            (1, 3, 112, 112),
        )
