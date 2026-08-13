from __future__ import annotations

import numpy as np

from dog_patrol_perception_face.crop import DecodedCrop
from dog_patrol_perception_face.verifier import ProductionFaceVerifier


class FakeDetector:
    def __init__(self, detections) -> None:
        self._detections = detections
        self.calls = 0
        self.raise_on = None

    def infer_frame(self, image):
        del image
        self.calls += 1
        if self.raise_on is not None:
            raise self.raise_on
        return {}, None

    def postprocess(self, raw, pad):
        del raw, pad
        return list(self._detections)


class FakeRecognition:
    def __init__(self, embedding) -> None:
        self._embedding = np.asarray(embedding, dtype=np.float32)
        self.calls = 0

    def infer(self, aligned):
        del aligned
        self.calls += 1
        return self._embedding.copy()


def _face_det(kpts):
    return {
        "class": "face",
        "conf": 0.9,
        "box": [10.0, 10.0, 40.0, 44.0],
        "kpts": kpts,
    }


_KPTS = [
    [12.0, 14.0, 1.0],
    [36.0, 14.0, 1.0],
    [24.0, 26.0, 1.0],
    [14.0, 36.0, 1.0],
    [34.0, 36.0, 1.0],
]


def _crop():
    return DecodedCrop(
        target_id=1,
        source_stamp_ns=1,
        source_frame_id="cam",
        source_frame_number=1,
        source_frame_number_available=True,
        source_image_width=640,
        source_image_height=480,
        bbox_x=0,
        bbox_y=0,
        bbox_width=48,
        bbox_height=48,
        image=np.zeros((48, 48, 3), dtype=np.uint8),
    )


def _gallery(emb):
    gallery = np.asarray(emb, dtype=np.float32)
    gallery = gallery / np.linalg.norm(gallery)
    return {"alice": (gallery[None, :], np.array([20.0]))}


def test_no_face_is_not_decided() -> None:
    detector = FakeDetector([])
    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=FakeRecognition(np.ones(8)),
        database=_gallery(np.ones(8)),
        min_frames_for_decision=2,
    )
    result = verifier.verify(_crop())
    assert not result.accepted and not result.decided
    assert "no face" in result.detail
    assert result.error is None


def test_face_with_insufficient_keypoints_is_skipped() -> None:
    detector = FakeDetector([_face_det([[12.0, 14.0, 1.0], [36.0, 14.0, 1.0]])])
    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=FakeRecognition(np.ones(8)),
        database=_gallery(np.ones(8)),
        min_frames_for_decision=2,
    )
    result = verifier.verify(_crop())
    assert not result.accepted and not result.decided


def test_collects_frames_before_decision() -> None:
    detector = FakeDetector([_face_det(_KPTS)])
    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=FakeRecognition(np.ones(8)),
        database=_gallery(np.ones(8)),
        min_frames_for_decision=2,
    )
    first = verifier.verify(_crop())
    assert first.detail == "collecting frames"
    second = verifier.verify(_crop())
    assert second.accepted
    assert "matched whitelist" in second.detail


def test_matching_face_is_accepted() -> None:
    emb = np.arange(1, 9, dtype=np.float32)
    detector = FakeDetector([_face_det(_KPTS)])
    recognition = FakeRecognition(emb)
    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=recognition,
        database=_gallery(emb),
        min_frames_for_decision=1,
    )
    result = verifier.verify(_crop())
    assert result.accepted and result.decided
    assert recognition.calls == 1


def test_nonmatching_face_is_not_accepted() -> None:
    detector = FakeDetector([_face_det(_KPTS)])
    gallery_emb = np.arange(1, 9, dtype=np.float32)
    probe_emb = np.arange(1, 9, dtype=np.float32)
    probe_emb[0] = 100.0  # far from gallery
    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=FakeRecognition(probe_emb),
        database=_gallery(gallery_emb),
        min_frames_for_decision=1,
    )
    result = verifier.verify(_crop())
    assert not result.accepted and result.decided
    assert "not in whitelist" in result.detail


def test_detector_failure_surfaces_error() -> None:
    detector = FakeDetector([])
    detector.raise_on = RuntimeError("gpu failed")
    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=FakeRecognition(np.ones(8)),
        database=_gallery(np.ones(8)),
        min_frames_for_decision=1,
    )
    result = verifier.verify(_crop())
    assert result.error is not None
    assert "gpu failed" in result.error


def test_reset_clears_temporal_buffer() -> None:
    detector = FakeDetector([_face_det(_KPTS)])
    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=FakeRecognition(np.ones(8)),
        database=_gallery(np.ones(8)),
        min_frames_for_decision=2,
    )
    verifier.verify(_crop())
    verifier.reset()
    result = verifier.verify(_crop())
    assert result.detail == "collecting frames"
