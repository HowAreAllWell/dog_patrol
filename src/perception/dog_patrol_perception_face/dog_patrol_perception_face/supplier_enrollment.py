"""Supplier-compatible MobileFaceNet whitelist enrollment.

This keeps the enrollment algorithm shipped with the MobileFaceNet package:
YuNet landmarks, the ArcFace 112 template, sharpness-ranked samples, and one
normalized mean prototype per identity.  Only the persistence format differs:
the prototype is written into the per-identity NPY layout consumed by the
production provider instead of the supplier's standalone NPZ gallery.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Iterable, Iterator

import numpy as np

from .config import load_face_config
from .enrollment import (
    EnrollmentSample,
    install_identity_templates,
    validate_identity_name,
)


_IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".webp"}
_VIDEO_SUFFIXES = {".avi", ".m4v", ".mkv", ".mov", ".mp4"}
_ARCFACE_TEMPLATE = np.asarray(
    [
        [38.2946, 51.6963],
        [73.5318, 51.5014],
        [56.0252, 71.7366],
        [41.5493, 92.3655],
        [70.7299, 92.2041],
    ],
    dtype=np.float32,
)
_DEFAULT_ASSETS_ROOT = (
    "/mnt/nvme/workspace/dog_patrol/src/perception/"
    "dog_patrol_perception_assets_20260813"
)


@dataclass(frozen=True)
class SupplierEnrollmentSample:
    source: str
    frame_index: int
    sharpness: float
    embedding: np.ndarray


def _normalize(embedding: np.ndarray) -> np.ndarray:
    value = np.asarray(embedding, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(value))
    if (
        value.size == 0
        or not np.isfinite(value).all()
        or not np.isfinite(norm)
        or norm <= 0
    ):
        raise ValueError("enrollment produced an unusable embedding")
    return value / norm


def build_supplier_centroid(
    samples: Iterable[SupplierEnrollmentSample],
    *,
    max_samples: int = 80,
    min_samples: int = 5,
) -> tuple[np.ndarray, list[SupplierEnrollmentSample]]:
    if max_samples <= 0 or min_samples <= 0:
        raise ValueError("sample limits must be positive")
    accepted = sorted(
        list(samples),
        key=lambda sample: sample.sharpness,
        reverse=True,
    )[:max_samples]
    if len(accepted) < min_samples:
        raise ValueError(f"Only {len(accepted)} qualified faces")
    dimensions = {
        np.asarray(sample.embedding, dtype=np.float32).reshape(-1).size
        for sample in accepted
    }
    if len(dimensions) != 1:
        raise ValueError("qualified embeddings have inconsistent dimensions")
    normalized = [_normalize(sample.embedding) for sample in accepted]
    return _normalize(np.mean(normalized, axis=0)), accepted


def install_supplier_centroid(
    *,
    whitelist_dir: Path | str,
    identity: str,
    centroid: np.ndarray,
) -> tuple[Path, Path | None]:
    identity = validate_identity_name(identity)
    prototype = _normalize(centroid)
    sample = EnrollmentSample(
        source_label="supplier-centroid",
        source_index=0,
        embedding=prototype,
        face_size=0.0,
        confidence=1.0,
        brightness=0.0,
        sharpness=0.0,
    )
    return install_identity_templates(
        whitelist_dir=whitelist_dir,
        identity=identity,
        samples=[sample],
        replace=(
            Path(whitelist_dir).expanduser().resolve() / identity
        ).exists(),
    )


def _read_image(path: Path):
    import cv2

    encoded = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def iter_supplier_frames(
    source: Path | str,
    *,
    stride: int = 5,
) -> Iterator[tuple[str, int, np.ndarray]]:
    import cv2

    if stride <= 0:
        raise ValueError("stride must be positive")
    source_path = Path(source).expanduser().resolve()
    if not source_path.exists():
        raise ValueError(f"registration source is missing: {source_path}")
    files = [source_path] if source_path.is_file() else sorted(
        path for path in source_path.rglob("*") if path.is_file()
    )
    supported = 0
    for path in files:
        suffix = path.suffix.lower()
        if suffix in _IMAGE_SUFFIXES:
            supported += 1
            image = _read_image(path)
            if image is not None:
                yield str(path), 0, image
        elif suffix in _VIDEO_SUFFIXES:
            supported += 1
            capture = cv2.VideoCapture(str(path))
            try:
                frame_index = 0
                while True:
                    ok, image = capture.read()
                    if not ok:
                        break
                    if frame_index % stride == 0:
                        yield str(path), frame_index, image
                    frame_index += 1
            finally:
                capture.release()
    if supported == 0:
        raise ValueError(
            f"no supported registration media found in {source_path}"
        )


class SupplierMobileFaceNetEnroller:
    """YuNet and TensorRT runner matching the supplier enrollment pipeline."""

    def __init__(
        self,
        yunet_model: Path | str,
        recognition_engine: Path | str,
    ):
        import cv2

        from .recognition import RecModel

        self._cv2 = cv2
        self._detector = cv2.FaceDetectorYN.create(
            str(yunet_model),
            "",
            (160, 160),
            0.25,
            0.3,
            5000,
        )
        self._recognition = RecModel(
            str(recognition_engine),
            model="mobilefacenet_arcface",
            expected_embedding_dim=512,
        )

    def extract(
        self,
        image: np.ndarray,
    ) -> tuple[np.ndarray | None, dict | str]:
        cv2 = self._cv2
        height, width = image.shape[:2]
        scale = max(1.0, 160.0 / min(height, width))
        resized = cv2.resize(
            image,
            (round(width * scale), round(height * scale)),
        )
        self._detector.setInputSize((resized.shape[1], resized.shape[0]))
        _, faces = self._detector.detect(resized)
        if faces is None:
            return None, "no_face"
        faces = sorted(
            faces,
            key=lambda face: float(face[2] * face[3]),
            reverse=True,
        )
        if (
            len(faces) > 1
            and faces[1][2] * faces[1][3]
            > 0.6 * faces[0][2] * faces[0][3]
        ):
            return None, "multiple_faces"
        face = faces[0]
        if min(face[2], face[3]) < 40:
            return None, "face_too_small"
        matrix, _ = cv2.estimateAffinePartial2D(
            face[4:14].reshape(5, 2).astype(np.float32),
            _ARCFACE_TEMPLATE,
            method=cv2.LMEDS,
        )
        if matrix is None:
            return None, "alignment_failed"
        aligned = cv2.warpAffine(resized, matrix, (112, 112), borderValue=0)
        gray = cv2.cvtColor(aligned, cv2.COLOR_BGR2GRAY)
        brightness = float(gray.mean())
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if not 25 <= brightness <= 225:
            return None, "bad_brightness"
        if sharpness < 25:
            return None, "blurred"
        return _normalize(self._recognition.infer(aligned)), {
            "luma": brightness,
            "sharpness": sharpness,
        }

    def close(self) -> None:
        self._recognition.close()


def enroll_with_supplier_pipeline(
    *,
    identity: str,
    source: Path | str,
    yunet_model: Path | str,
    recognition_engine: Path | str,
    whitelist_dir: Path | str,
    stride: int = 5,
    max_samples: int = 80,
) -> dict:
    identity = validate_identity_name(identity)
    if identity.lower() == "unknown":
        raise ValueError("identity name must not be 'unknown'")
    yunet_path = Path(yunet_model).expanduser().resolve()
    engine_path = Path(recognition_engine).expanduser().resolve()
    required_paths = (
        ("YuNet model", yunet_path),
        ("recognition engine", engine_path),
    )
    for label, path in required_paths:
        if not path.is_file():
            raise ValueError(f"{label} is missing: {path}")

    from .detector import ensure_cuda_context

    cuda_context = ensure_cuda_context()
    enroller = None
    rejected: dict[str, int] = {}
    accepted = []
    try:
        enroller = SupplierMobileFaceNetEnroller(yunet_path, engine_path)
        frames = iter_supplier_frames(source, stride=stride)
        for filename, frame_index, image in frames:
            embedding, info = enroller.extract(image)
            if embedding is None:
                reason = str(info)
                rejected[reason] = rejected.get(reason, 0) + 1
                continue
            accepted.append(
                SupplierEnrollmentSample(
                    source=filename,
                    frame_index=frame_index,
                    sharpness=float(info["sharpness"]),
                    embedding=embedding,
                )
            )
        centroid, selected = build_supplier_centroid(
            accepted,
            max_samples=max_samples,
            min_samples=5,
        )
        destination, backup = install_supplier_centroid(
            whitelist_dir=whitelist_dir,
            identity=identity,
            centroid=centroid,
        )
        return {
            "status": "updated" if backup is not None else "added",
            "identity": identity,
            "qualified": len(selected),
            "rejected": rejected,
            "whitelist": str(destination),
            "backup": str(backup) if backup is not None else None,
            "total_candidates": len(accepted),
        }
    finally:
        if enroller is not None:
            enroller.close()
        if cuda_context is not None:
            cuda_context.pop()
            cuda_context.detach()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Enroll MobileFaceNet using the supplier YuNet pipeline."
    )
    parser.add_argument("--name", required=True)
    parser.add_argument("--source", required=True)
    parser.add_argument(
        "--assets-root",
        default=os.environ.get("DOG_PATROL_ASSETS_ROOT", _DEFAULT_ASSETS_ROOT),
    )
    parser.add_argument("--config")
    parser.add_argument("--yunet-model")
    parser.add_argument("--recognition-engine")
    parser.add_argument("--whitelist-dir")
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--max-samples", type=int, default=80)
    return parser


def main() -> None:
    args = _parser().parse_args()
    assets_root = Path(args.assets_root).expanduser().resolve()
    config_path = (
        Path(args.config).expanduser().resolve()
        if args.config
        else assets_root / "runtime" / "face_mobilefacenet.yaml"
    )
    config = load_face_config(config_path)
    if config.recognition_model != "mobilefacenet_arcface":
        raise ValueError(
            "supplier enrollment requires the MobileFaceNet configuration"
        )
    report = enroll_with_supplier_pipeline(
        identity=args.name,
        source=args.source,
        yunet_model=(
            args.yunet_model
            or assets_root / "face" / "face_detection_yunet_2023mar.onnx"
        ),
        recognition_engine=(
            args.recognition_engine
            or config.recognition_engine
            or assets_root / "face" / "mobilefacenet_fp16.engine"
        ),
        whitelist_dir=(
            args.whitelist_dir
            or config.whitelist_dir
            or assets_root / "face" / "whitelist_mobilefacenet"
        ),
        stride=args.stride,
        max_samples=args.max_samples,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
