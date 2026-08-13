"""Face verification engine: detection -> alignment -> embedding -> whitelist.

The production verifier mirrors the standalone pipeline from ``yolo-rec.py``
(including temporal embedding smoothing) but consumes a ``DecodedCrop`` instead
of a full camera frame.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .alignment import align_face_5pts
from .crop import DecodedCrop
from .whitelist import match_embedding


@dataclass(frozen=True)
class FaceVerification:
    """Outcome of verifying a single crop against the whitelist."""

    accepted: bool
    decided: bool = True
    detail: str = ""
    error: str | None = None
    # Face bounding box in crop-image coordinates: (x1, y1, x2, y2).
    # None when no face was detected. Consumed by the overlay visualizer.
    box: tuple[int, int, int, int] | None = None


class FaceVerifier:
    """Streaming seam consumed by the evidence controller."""

    def reset(self) -> None:
        """Clear per-session state (e.g. the temporal embedding buffer)."""

    def close(self) -> None:
        """Release GPU resources; must be called on the thread that owns them."""

    def verify(self, crop: DecodedCrop) -> FaceVerification:
        raise NotImplementedError


class ProductionFaceVerifier(FaceVerifier):
    def __init__(
        self,
        *,
        detector,
        recognition,
        database,
        similarity_threshold=0.55,
        size_sigma=20.0,
        temporal_window=10,
        min_frames_for_decision=5,
        min_score=0.5,
        min_box_ratio=0.08,
    ):
        self._detector = detector
        self._recognition = recognition
        self._database = database
        self._threshold = float(similarity_threshold)
        self._size_sigma = float(size_sigma)
        self._min_frames = max(1, int(min_frames_for_decision))
        self._buffer = deque(maxlen=max(1, int(temporal_window)))
        self._decided_frames = 0
        self._min_score = float(min_score)
        self._min_box_ratio = float(min_box_ratio)

    def reset(self) -> None:
        self._buffer.clear()
        self._decided_frames = 0

    def verify(self, crop: DecodedCrop) -> FaceVerification:
        try:
            raw, pad = self._detector.infer_frame(crop.image)
            dets = self._detector.postprocess(raw, pad)
        except Exception as exc:  # GPU/inference failure must surface as ERROR.
            return FaceVerification(
                accepted=False,
                decided=False,
                detail="",
                error=f"{type(exc).__name__}: {exc}",
            )

        faces = [
            d
            for d in dets
            if d.get("class") == "face" and len(d.get("kpts", [])) >= 5
        ]
        if not faces:
            return FaceVerification(
                accepted=False, decided=False, detail="no face detected"
            )
        best = max(faces, key=lambda d: d["conf"])

        x1, y1, x2, y2 = (int(v) for v in best["box"])
        crop_h, crop_w = crop.image.shape[:2]
        face_w = max(0, x2 - x1)
        face_h = max(0, y2 - y1)
        box_area_ratio = (face_w * face_h) / max(1.0, float(crop_w * crop_h))
        if (
            best.get("conf", 0.0) < self._min_score
            or box_area_ratio < self._min_box_ratio
        ):
            confidence = best.get("conf", 0.0)
            return FaceVerification(
                accepted=False,
                decided=False,
                detail=(
                    f"low quality face (conf={confidence:.2f}, "
                    f"area={box_area_ratio:.3f})"
                ),
                box=(x1, y1, x2, y2),
            )

        try:
            aligned = align_face_5pts(crop.image, best["kpts"][:5])
            emb = self._recognition.infer(aligned)
        except Exception as exc:
            return FaceVerification(
                accepted=False,
                decided=False,
                detail="",
                error=f"{type(exc).__name__}: {exc}",
            )

        x1, y1, x2, y2 = (int(v) for v in best["box"])
        box = (x1, y1, x2, y2)

        self._buffer.append(emb)
        if len(self._buffer) < self._min_frames:
            return FaceVerification(
                accepted=False, decided=False, detail="collecting frames", box=box
            )

        avg = np.mean(self._buffer, axis=0)
        norm = np.linalg.norm(avg)
        avg = avg / norm if norm > 0 else avg

        fsize = np.sqrt(max(0, x2 - x1) * max(0, y2 - y1))
        matched_name, sim = match_embedding(
            avg, self._database, self._threshold, fsize, self._size_sigma
        )
        self._decided_frames += 1
        if matched_name is None:
            return FaceVerification(
                accepted=False,
                decided=True,
                detail=f"face not in whitelist (sim={sim:.3f})",
                box=box,
            )
        return FaceVerification(
            accepted=True,
            decided=True,
            detail=f"face matched whitelist (sim={sim:.3f})",
            box=box,
        )

    def close(self) -> None:
        self._detector.close()
        self._recognition.close()
