"""Persist face-verification evidence crops without changing recognition results."""

from __future__ import annotations

from pathlib import Path
import re
import time

import cv2

from .crop import DecodedCrop


_SAFE_PART = re.compile(r"[^A-Za-z0-9_.-]+")


class EvidencePhotoStore:
    """Bounded, atomic photo store owned by the face provider."""

    def __init__(
        self,
        *,
        enabled: bool,
        root: str,
        image_format: str = "jpg",
        jpeg_quality: int = 90,
        retention_seconds: int = 604800,
        max_files: int = 10000,
    ) -> None:
        self.enabled = bool(enabled)
        self.root = Path(root).expanduser()
        self.image_format = image_format.lower().lstrip(".")
        self.jpeg_quality = int(jpeg_quality)
        self.retention_seconds = int(retention_seconds)
        self.max_files = int(max_files)
        if self.image_format not in {"jpg", "png"}:
            raise ValueError("evidence_photo_format must be 'jpg' or 'png'")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("evidence_photo_jpeg_quality must be in [1, 100]")
        if self.retention_seconds <= 0 or self.max_files <= 0:
            raise ValueError("evidence photo retention and max_files must be positive")
        if self.enabled:
            if not str(root).strip():
                raise ValueError("evidence_photo_root must not be empty when enabled")
            resolved = self.root.resolve()
            if resolved == Path(resolved.anchor):
                raise ValueError("evidence_photo_root must not be a filesystem root")
            self.root.mkdir(parents=True, exist_ok=True)

    def save(
        self,
        crop: DecodedCrop,
        *,
        state_seq: int,
        target_id: int,
        stage: str,
        outcome: str,
    ) -> str:
        if not self.enabled:
            return ""
        self._cleanup()
        frame_part = (
            str(crop.source_frame_number)
            if crop.source_frame_number_available
            else str(crop.source_stamp_ns)
        )
        directory = self.root / f"state-{int(state_seq)}" / f"target-{int(target_id)}"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (
            f"{self._safe(stage)}_{self._safe(outcome)}_{frame_part}.{self.image_format}"
        )
        extension = ".jpg" if self.image_format == "jpg" else ".png"
        params = (
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            if self.image_format == "jpg"
            else [cv2.IMWRITE_PNG_COMPRESSION, 3]
        )
        ok, encoded = cv2.imencode(extension, crop.image, params)
        if not ok:
            raise RuntimeError("OpenCV failed to encode evidence photo")
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_bytes(encoded.tobytes())
        temporary.replace(path)
        return path.resolve().as_uri()

    @staticmethod
    def _safe(value: str) -> str:
        return _SAFE_PART.sub("-", str(value).strip()).strip("-") or "unknown"

    def _cleanup(self) -> None:
        files = sorted(
            (
                path
                for path in self.root.glob("state-*/target-*/*")
                if path.is_file() and path.suffix.lower() in {".jpg", ".png"}
            ),
            key=lambda path: path.stat().st_mtime,
        )
        cutoff = time.time() - self.retention_seconds
        expired = [path for path in files if path.stat().st_mtime < cutoff]
        overflow = max(0, len(files) - len(expired) - self.max_files + 1)
        victims = expired + [path for path in files if path not in expired][:overflow]
        for path in victims:
            path.unlink(missing_ok=True)
