from __future__ import annotations

import cv2
import numpy as np

from dog_patrol_perception_face.alignment import align_face_5pts


_KEYPOINTS = [
    [10.0, 11.0, 0.9],
    [20.0, 21.0, 0.9],
    [30.0, 31.0, 0.9],
    [40.0, 41.0, 0.9],
    [50.0, 51.0, 0.9],
]


def _capture_alignment_source(monkeypatch, profile: str) -> np.ndarray:
    captured = {}

    def fake_estimate(source, destination):
        del destination
        captured["source"] = source.copy()
        return None, None

    monkeypatch.setattr(cv2, "estimateAffinePartial2D", fake_estimate)
    align_face_5pts(
        np.zeros((64, 64, 3), dtype=np.uint8),
        _KEYPOINTS,
        profile,
    )
    return captured["source"]


def test_arcface_profile_preserves_yolo_landmark_order(monkeypatch) -> None:
    source = _capture_alignment_source(monkeypatch, "arcface_112")

    assert source.tolist() == [point[:2] for point in _KEYPOINTS]


def test_legacy_sface_profile_keeps_historical_landmark_swap(monkeypatch) -> None:
    source = _capture_alignment_source(monkeypatch, "legacy_sface")

    assert source.tolist() == [
        _KEYPOINTS[index][:2]
        for index in (1, 0, 2, 4, 3)
    ]
