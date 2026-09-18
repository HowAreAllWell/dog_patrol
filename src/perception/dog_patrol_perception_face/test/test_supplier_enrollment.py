from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dog_patrol_perception_face.supplier_enrollment import (
    SupplierEnrollmentSample,
    build_supplier_centroid,
    install_supplier_centroid,
)
from dog_patrol_perception_face.whitelist import load_whitelist_npy


def _sample(index: int, sharpness: float, embedding: list[float]):
    return SupplierEnrollmentSample(
        source="registration.mp4",
        frame_index=index,
        sharpness=sharpness,
        embedding=np.asarray(embedding, dtype=np.float32),
    )


def test_supplier_centroid_uses_sharpest_samples_and_normalizes_mean() -> None:
    samples = [
        _sample(index, float(index), [1.0, 0.0] if index < 6 else [0.0, 1.0])
        for index in range(10)
    ]

    centroid, selected = build_supplier_centroid(
        samples,
        max_samples=5,
        min_samples=5,
    )

    assert [sample.frame_index for sample in selected] == [9, 8, 7, 6, 5]
    assert centroid == pytest.approx(
        np.asarray([0.2, 0.8]) / np.linalg.norm([0.2, 0.8])
    )
    assert np.linalg.norm(centroid) == pytest.approx(1.0)


def test_supplier_centroid_requires_five_qualified_faces() -> None:
    samples = [_sample(index, 100.0, [1.0, 0.0]) for index in range(4)]

    with pytest.raises(ValueError, match="Only 4 qualified faces"):
        build_supplier_centroid(samples, max_samples=80, min_samples=5)


def test_supplier_centroid_rejects_inconsistent_embedding_dimensions() -> None:
    samples = [
        _sample(index, 100.0, [1.0, 0.0] if index < 4 else [1.0, 0.0, 0.0])
        for index in range(5)
    ]

    with pytest.raises(ValueError, match="inconsistent dimensions"):
        build_supplier_centroid(samples, max_samples=80, min_samples=5)


def test_install_supplier_centroid_writes_runtime_compatible_single_template(
    tmp_path: Path,
) -> None:
    whitelist = tmp_path / "whitelist_mobilefacenet"
    first = np.zeros(512, dtype=np.float32)
    first[3] = 1.0

    destination, backup = install_supplier_centroid(
        whitelist_dir=whitelist,
        identity="person_001",
        centroid=first,
    )

    assert destination == whitelist / "person_001"
    assert backup is None
    assert [path.name for path in destination.glob("*.npy")] == [
        "g0_0px_f0.npy"
    ]
    database = load_whitelist_npy(whitelist)
    assert database["person_001"][0].shape == (1, 512)
    assert database["person_001"][0][0] == pytest.approx(first)

    second = np.zeros(512, dtype=np.float32)
    second[7] = 1.0
    destination, backup = install_supplier_centroid(
        whitelist_dir=whitelist,
        identity="person_001",
        centroid=second,
    )

    assert destination.is_dir()
    assert backup is not None and backup.is_dir()
    database = load_whitelist_npy(whitelist)
    assert database["person_001"][0][0] == pytest.approx(second)
