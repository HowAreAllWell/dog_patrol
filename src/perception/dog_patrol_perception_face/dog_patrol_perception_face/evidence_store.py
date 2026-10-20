"""Persist face-verification evidence crops without changing recognition results."""

from __future__ import annotations

import os
from pathlib import Path
import re
import time
from uuid import uuid4

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
        directory = self.root / f"state-{int(state_seq)}" / f"target-{int(target_id)}"
        directory.mkdir(parents=True, exist_ok=True)
        frame_part = (
            f"f{crop.source_frame_number}"
            if crop.source_frame_number_available
            else "fna"
        )
        stem = (
            f"{max(0, int(crop.source_stamp_ns))}_target-{int(target_id)}_"
            f"{self._safe(stage)}_{self._safe(outcome)}_{frame_part}"
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
        temporary = directory / f".{uuid4().hex}.tmp"
        temporary.write_bytes(encoded.tobytes())
        path = self._claim_unique_path(directory, stem, temporary)
        # Never let the quota/retention sweep remove the photo whose URI is
        # about to be returned, even when filesystem mtimes have coarse or
        # identical resolution.
        self._cleanup(protected=path)
        return path.resolve().as_uri()

    def _claim_unique_path(self, directory: Path, stem: str, temporary: Path) -> Path:
        """Atomically publish a photo without replacing existing history."""
        try:
            for suffix in range(10000):
                tail = "" if suffix == 0 else f"-{suffix}"
                candidate = directory / f"{stem}{tail}.{self.image_format}"
                try:
                    os.link(temporary, candidate)
                    return candidate
                except FileExistsError:
                    continue
            raise RuntimeError("too many evidence photo filename collisions")
        finally:
            temporary.unlink(missing_ok=True)

    @staticmethod
    def _safe(value: str) -> str:
        return _SAFE_PART.sub("-", str(value).strip()).strip("-") or "unknown"

    def _cleanup(self, protected: Path | None = None) -> None:
        protected = protected.resolve() if protected is not None else None
        entries: list[tuple[Path, int]] = []
        for path in self.root.glob("state-*/target-*/*"):
            if not path.is_file() or path.suffix.lower() not in {".jpg", ".png"}:
                continue
            try:
                entries.append((path, path.stat().st_mtime_ns))
            except FileNotFoundError:
                continue
        entries.sort(key=lambda entry: (entry[1], entry[0].as_posix()))
        cutoff_ns = time.time_ns() - self.retention_seconds * 1_000_000_000

        def is_protected(path: Path) -> bool:
            return protected is not None and path.resolve() == protected

        expired = [
            path for path, mtime_ns in entries
            if mtime_ns < cutoff_ns and not is_protected(path)
        ]
        survivors = [path for path, _ in entries if path not in expired]
        overflow = max(0, len(survivors) - self.max_files)
        overflow_candidates = [path for path in survivors if not is_protected(path)]
        victims = expired + overflow_candidates[:overflow]
        for path in victims:
            path.unlink(missing_ok=True)
