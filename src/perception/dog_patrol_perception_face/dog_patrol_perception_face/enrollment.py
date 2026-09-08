"""Enroll face embeddings into the production per-identity whitelist."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Iterable

import numpy as np

from .alignment import align_face_5pts
from .config import FaceConfig, load_face_config
from .whitelist import load_whitelist_npy, match_embedding


_IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff"}
_VIDEO_SUFFIXES = {".avi", ".mkv", ".mov", ".mp4", ".webm"}
_DEFAULT_ASSETS_ROOT = (
    "/mnt/nvme/workspace/dog_patrol/src/perception/"
    "dog_patrol_perception_assets_20260813"
)


@dataclass(frozen=True)
class EnrollmentSample:
    source_label: str
    source_index: int
    embedding: np.ndarray
    face_size: float
    confidence: float
    brightness: float
    sharpness: float


@dataclass(frozen=True)
class QualityThresholds:
    min_confidence: float = 0.5
    min_face_size: float = 35.0
    min_brightness: float = 35.0
    max_brightness: float = 230.0
    min_sharpness: float = 15.0


def validate_identity_name(name: str) -> str:
    if not isinstance(name, str) or not name or name != name.strip():
        raise ValueError("identity name must be non-empty and must not contain outer whitespace")
    if name in {".", ".."} or "/" in name or "\\" in name or "\x00" in name:
        raise ValueError("identity name must be one safe directory name")
    if not all(character.isalnum() or character in {"_", "-", "."} for character in name):
        raise ValueError("identity name may contain letters, numbers, '_', '-' and '.' only")
    return name


def build_video_sample_plan(
    frame_count: int,
    sample_stride: int = 5,
    holdout_count: int = 6,
    max_candidate_count: int = 120,
) -> tuple[list[int], list[int]]:
    if frame_count <= 0:
        raise ValueError("video frame count must be positive")
    if sample_stride <= 0:
        raise ValueError("sample stride must be positive")
    if holdout_count < 0:
        raise ValueError("holdout count must not be negative")
    if max_candidate_count <= 0:
        raise ValueError("maximum candidate count must be positive")
    if holdout_count == 0:
        holdouts: list[int] = []
    else:
        holdouts = sorted(
            set(
                int(value)
                for value in np.linspace(
                    frame_count * 0.075,
                    frame_count * 0.97,
                    holdout_count,
                )
            )
        )
    exclusion_radius = max(2, sample_stride // 2)
    candidates = [
        frame_index
        for frame_index in range(sample_stride, max(sample_stride, frame_count - sample_stride), sample_stride)
        if all(abs(frame_index - holdout) > exclusion_radius for holdout in holdouts)
    ]
    # Some screen-recorded MP4 files report a millisecond timebase as an
    # apparent 1000 FPS stream. Limit inference work while retaining coverage
    # across the complete video and keeping holdouts independent.
    if len(candidates) > max_candidate_count:
        positions = np.linspace(0, len(candidates) - 1, max_candidate_count).round().astype(int)
        candidates = [candidates[int(position)] for position in positions]
    if not candidates:
        raise ValueError("video does not contain enough frames for registration sampling")
    return candidates, holdouts


def select_scale_distributed_samples(
    samples: Iterable[EnrollmentSample],
    template_count: int,
) -> list[EnrollmentSample]:
    if template_count <= 0:
        raise ValueError("template count must be positive")
    ordered = sorted(samples, key=lambda sample: (sample.face_size, sample.source_index))
    if not ordered:
        raise ValueError("no enrollment samples passed quality checks")
    if len(ordered) <= template_count:
        return ordered
    positions = np.linspace(0, len(ordered) - 1, template_count).round().astype(int)
    return [ordered[int(position)] for position in positions]


def _normalized_gallery(samples: Iterable[EnrollmentSample]) -> tuple[np.ndarray, np.ndarray]:
    normalized = []
    sizes = []
    dimensions = set()
    for sample in samples:
        embedding = np.asarray(sample.embedding, dtype=np.float32).reshape(-1)
        norm = float(np.linalg.norm(embedding))
        if not np.isfinite(embedding).all() or not np.isfinite(norm) or norm <= 0.0:
            raise ValueError(f"unusable embedding from {sample.source_label}")
        normalized.append(embedding / norm)
        sizes.append(float(sample.face_size))
        dimensions.add(embedding.size)
    if not normalized:
        raise ValueError("at least one enrollment sample is required")
    if len(dimensions) != 1:
        raise ValueError("enrollment embeddings have inconsistent dimensions")
    return np.stack(normalized), np.asarray(sizes, dtype=np.float32)


def validate_holdout_samples(
    *,
    identity: str,
    templates: Iterable[EnrollmentSample],
    holdouts: Iterable[EnrollmentSample],
    existing_database: dict,
    threshold: float,
    size_sigma: float,
) -> list[dict]:
    identity = validate_identity_name(identity)
    gallery, sizes = _normalized_gallery(templates)
    database = dict(existing_database)
    database[identity] = (gallery, sizes)
    results = []
    failures = []
    for sample in holdouts:
        name, similarity = match_embedding(
            sample.embedding,
            database,
            threshold,
            sample.face_size,
            size_sigma,
        )
        result = {
            "source": sample.source_label,
            "source_index": sample.source_index,
            "face_size_px": round(float(sample.face_size), 1),
            "matched_name": name or "unknown",
            "similarity": round(float(similarity), 4),
        }
        results.append(result)
        if name != identity or similarity < threshold:
            failures.append(result)
    if failures:
        details = ", ".join(
            f"{item['source']}->{item['matched_name']}({item['similarity']:.4f})"
            for item in failures
        )
        raise ValueError(f"holdout validation failed: {details}")
    return results


def install_identity_templates(
    *,
    whitelist_dir: Path | str,
    identity: str,
    samples: Iterable[EnrollmentSample],
    replace: bool,
) -> tuple[Path, Path | None]:
    identity = validate_identity_name(identity)
    root = Path(whitelist_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    destination = root / identity
    if destination.exists() and not replace:
        raise FileExistsError(
            f"identity {identity!r} already exists; use --replace to update it"
        )

    sample_list = list(samples)
    gallery, sizes = _normalized_gallery(sample_list)
    staging_root = root.parent / f".{root.name}_staging"
    staging_root.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f"{identity}-", dir=staging_root))
    backup = None
    try:
        for index, (sample, embedding, size) in enumerate(zip(sample_list, gallery, sizes)):
            filename = f"g{index}_{int(round(float(size)))}px_f{sample.source_index}.npy"
            np.save(staging / filename, embedding.astype(np.float32))

        # Validate exactly what was serialized before exposing the directory to
        # readiness/provider loaders.
        serialized = sorted(staging.glob("*.npy"))
        if len(serialized) != len(sample_list):
            raise RuntimeError("not all whitelist templates were serialized")
        for path in serialized:
            loaded = np.asarray(np.load(path, allow_pickle=False), dtype=np.float32).reshape(-1)
            if loaded.size != gallery.shape[1] or not np.isfinite(loaded).all():
                raise RuntimeError(f"serialized whitelist template is invalid: {path.name}")

        if destination.exists():
            backup_parent = root.parent / f"{root.name}_backups" / identity
            backup_parent.mkdir(parents=True, exist_ok=True)
            backup = backup_parent / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            os.replace(destination, backup)
        try:
            os.replace(staging, destination)
        except Exception:
            if backup is not None and backup.exists() and not destination.exists():
                os.replace(backup, destination)
                backup = None
            raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)
        try:
            staging_root.rmdir()
        except OSError:
            pass
    return destination, backup


def _sample_summary(sample: EnrollmentSample) -> dict:
    return {
        "source": sample.source_label,
        "source_index": sample.source_index,
        "confidence": round(float(sample.confidence), 4),
        "face_size_px": round(float(sample.face_size), 1),
        "brightness": round(float(sample.brightness), 1),
        "sharpness": round(float(sample.sharpness), 1),
    }


def _extract_sample(
    frame: np.ndarray,
    *,
    source_label: str,
    source_index: int,
    detector,
    recognition,
    quality: QualityThresholds,
) -> tuple[EnrollmentSample | None, str | None]:
    import cv2

    raw, padding = detector.infer_frame(frame)
    faces = [
        face
        for face in detector.postprocess(raw, padding)
        if face.get("class") == "face" and len(face.get("kpts", [])) >= 5
    ]
    if len(faces) != 1:
        return None, f"expected exactly one face, detected {len(faces)}"
    face = faces[0]
    confidence = float(face.get("conf", 0.0))
    if confidence < quality.min_confidence:
        return None, f"face confidence {confidence:.3f} is below {quality.min_confidence:.3f}"
    x1, y1, x2, y2 = (int(value) for value in face["box"])
    face_size = float(np.sqrt(max(0, x2 - x1) * max(0, y2 - y1)))
    if face_size < quality.min_face_size:
        return None, f"face size {face_size:.1f}px is below {quality.min_face_size:.1f}px"

    aligned = align_face_5pts(frame, face["kpts"][:5])
    gray = cv2.cvtColor(aligned, cv2.COLOR_BGR2GRAY)
    brightness = float(gray.mean())
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    if not quality.min_brightness <= brightness <= quality.max_brightness:
        return None, f"face brightness {brightness:.1f} is outside the accepted range"
    if sharpness < quality.min_sharpness:
        return None, f"face sharpness {sharpness:.1f} is below {quality.min_sharpness:.1f}"

    embedding = np.asarray(recognition.infer(aligned), dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(embedding))
    if embedding.size != 128 or not np.isfinite(embedding).all() or norm <= 0.0:
        return None, f"recognition returned an invalid embedding shape={embedding.shape}"
    return EnrollmentSample(
        source_label=source_label,
        source_index=source_index,
        embedding=embedding / norm,
        face_size=face_size,
        confidence=confidence,
        brightness=brightness,
        sharpness=sharpness,
    ), None


def _load_existing_database(whitelist_dir: Path) -> dict:
    if not whitelist_dir.is_dir():
        return {}
    try:
        return load_whitelist_npy(str(whitelist_dir))
    except ValueError as exception:
        if "contains no usable embeddings" in str(exception):
            return {}
        raise


def _extract_video_samples(
    source: Path,
    *,
    detector,
    recognition,
    quality: QualityThresholds,
    sample_stride: int,
    holdout_count: int,
    max_candidate_count: int,
) -> tuple[list[EnrollmentSample], list[EnrollmentSample], list[dict]]:
    import cv2

    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise ValueError(f"cannot open video: {source}")
    try:
        # CAP_PROP_FRAME_COUNT is unreliable for recordings whose container
        # advertises a timebase as FPS. Count frames that OpenCV can actually
        # decode before choosing indices; otherwise registration may seek to
        # thousands of non-existent frames and produce no holdout samples.
        frame_count = 0
        while True:
            ok, _ = capture.read()
            if not ok:
                break
            frame_count += 1
        capture.release()
        if frame_count <= 0:
            raise ValueError(f"video contains no decodable frames: {source}")
        capture = cv2.VideoCapture(str(source))
        if not capture.isOpened():
            raise ValueError(f"cannot reopen video: {source}")
        candidate_indices, holdout_indices = build_video_sample_plan(
            frame_count,
            sample_stride,
            holdout_count,
            max_candidate_count,
        )
        samples = []
        holdouts = []
        rejected = []
        for group, indices in ((samples, candidate_indices), (holdouts, holdout_indices)):
            for frame_index in indices:
                capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
                ok, frame = capture.read()
                if not ok:
                    rejected.append({"source_index": frame_index, "reason": "frame read failed"})
                    continue
                sample, reason = _extract_sample(
                    frame,
                    source_label=f"{source.name}:frame:{frame_index}",
                    source_index=frame_index,
                    detector=detector,
                    recognition=recognition,
                    quality=quality,
                )
                if sample is None:
                    rejected.append({"source_index": frame_index, "reason": reason})
                else:
                    group.append(sample)
        return samples, holdouts, rejected
    finally:
        capture.release()


def _extract_image_samples(
    source: Path,
    *,
    detector,
    recognition,
    quality: QualityThresholds,
    holdout_count: int,
) -> tuple[list[EnrollmentSample], list[EnrollmentSample], list[dict]]:
    import cv2

    if source.is_dir():
        paths = sorted(
            path for path in source.iterdir()
            if path.is_file() and path.suffix.lower() in _IMAGE_SUFFIXES
        )
    else:
        paths = [source]
    if not paths:
        raise ValueError(f"no supported registration images found in {source}")

    holdout_positions = set()
    if len(paths) >= 4 and holdout_count > 0:
        count = min(holdout_count, max(1, len(paths) // 4))
        holdout_positions = set(
            int(position)
            for position in np.linspace(1, len(paths) - 1, count).round().astype(int)
        )
    samples = []
    holdouts = []
    rejected = []
    for index, path in enumerate(paths):
        frame = cv2.imread(str(path))
        if frame is None:
            rejected.append({"source": str(path), "reason": "image read failed"})
            continue
        sample, reason = _extract_sample(
            frame,
            source_label=path.name,
            source_index=index,
            detector=detector,
            recognition=recognition,
            quality=quality,
        )
        if sample is None:
            rejected.append({"source": str(path), "reason": reason})
        elif index in holdout_positions:
            holdouts.append(sample)
        else:
            samples.append(sample)
    return samples, holdouts, rejected


def enroll_from_source(
    *,
    identity: str,
    source: Path | str,
    detector_engine: Path | str,
    recognition_engine: Path | str,
    whitelist_dir: Path | str,
    face_config: FaceConfig,
    template_count: int = 12,
    sample_stride: int = 5,
    holdout_count: int = 6,
    max_candidate_count: int = 120,
    replace: bool = False,
    quality: QualityThresholds | None = None,
) -> dict:
    identity = validate_identity_name(identity)
    source_path = Path(source).expanduser().resolve()
    detector_path = Path(detector_engine).expanduser().resolve()
    recognition_path = Path(recognition_engine).expanduser().resolve()
    whitelist_path = Path(whitelist_dir).expanduser().resolve()
    for label, path in (
        ("source", source_path),
        ("detector engine", detector_path),
        ("recognition engine", recognition_path),
    ):
        if not path.exists():
            raise ValueError(f"{label} is missing: {path}")
    if (whitelist_path / identity).exists() and not replace:
        raise FileExistsError(
            f"identity {identity!r} already exists; use --replace to update it"
        )

    from .detector import TRTInference, ensure_cuda_context
    from .recognition import RecModel

    quality = quality or QualityThresholds(min_confidence=face_config.face_min_score)
    cuda_context = ensure_cuda_context()
    detector = None
    recognition = None
    try:
        detector = TRTInference(
            str(detector_path),
            conf_thres=face_config.detector_conf_thres,
            iou_thres=face_config.detector_iou_thres,
        )
        detector._ensure_gpu_preproc()
        recognition = RecModel(str(recognition_path))
        if source_path.is_dir() or source_path.suffix.lower() in _IMAGE_SUFFIXES:
            candidates, holdouts, rejected = _extract_image_samples(
                source_path,
                detector=detector,
                recognition=recognition,
                quality=quality,
                holdout_count=holdout_count,
            )
        elif source_path.suffix.lower() in _VIDEO_SUFFIXES:
            candidates, holdouts, rejected = _extract_video_samples(
                source_path,
                detector=detector,
                recognition=recognition,
                quality=quality,
                sample_stride=sample_stride,
                holdout_count=holdout_count,
                max_candidate_count=max_candidate_count,
            )
        else:
            raise ValueError(f"unsupported registration source: {source_path}")

        templates = select_scale_distributed_samples(candidates, template_count)
        if source_path.suffix.lower() in _VIDEO_SUFFIXES and len(templates) < min(6, template_count):
            raise ValueError("fewer than six video templates passed quality checks")
        if source_path.suffix.lower() in _VIDEO_SUFFIXES and len(holdouts) < min(3, holdout_count):
            raise ValueError("fewer than three video holdouts passed quality checks")

        existing_database = _load_existing_database(whitelist_path)
        holdout_results = validate_holdout_samples(
            identity=identity,
            templates=templates,
            holdouts=holdouts,
            existing_database=existing_database,
            threshold=face_config.similarity_threshold,
            size_sigma=face_config.size_sigma,
        )
        destination, backup = install_identity_templates(
            whitelist_dir=whitelist_path,
            identity=identity,
            samples=templates,
            replace=replace,
        )
        # Reload the complete deployed directory, including competing identities.
        deployed_database = load_whitelist_npy(str(whitelist_path))
        deployed_templates, _ = deployed_database[identity]
        if deployed_templates.shape[0] != len(templates):
            raise RuntimeError("deployed whitelist template count does not match enrollment")
        return {
            "identity": identity,
            "source": str(source_path),
            "destination": str(destination),
            "backup": str(backup) if backup is not None else None,
            "similarity_threshold": face_config.similarity_threshold,
            "size_sigma": face_config.size_sigma,
            "templates": [_sample_summary(sample) for sample in templates],
            "holdouts": holdout_results,
            "rejected": rejected,
        }
    finally:
        if detector is not None:
            detector.close()
        if recognition is not None:
            recognition.close()
        if cuda_context is not None:
            cuda_context.pop()
            cuda_context.detach()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Enroll one identity into the dog patrol face whitelist."
    )
    parser.add_argument("--name", required=True, help="identity directory name")
    parser.add_argument("--source", required=True, help="image, video, or image directory")
    parser.add_argument(
        "--assets-root",
        default=os.environ.get("DOG_PATROL_ASSETS_ROOT", _DEFAULT_ASSETS_ROOT),
        help="perception asset root",
    )
    parser.add_argument("--config", help="face YAML; defaults to <assets-root>/runtime/face.yaml")
    parser.add_argument("--detector-engine")
    parser.add_argument("--recognition-engine")
    parser.add_argument("--whitelist-dir")
    parser.add_argument("--templates", type=int, default=12)
    parser.add_argument("--sample-stride", type=int, default=5)
    parser.add_argument("--holdout-count", type=int, default=6)
    parser.add_argument(
        "--max-candidate-frames",
        type=int,
        default=120,
        help="maximum video frames sent through the detector; spread across the full video",
    )
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--report", help="optional JSON report output path")
    return parser


def main() -> None:
    args = _parser().parse_args()
    assets_root = Path(args.assets_root).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve() if args.config else assets_root / "runtime" / "face.yaml"
    config = load_face_config(str(config_path))
    detector_engine = args.detector_engine or assets_root / "face" / "detector.engine"
    recognition_engine = args.recognition_engine or assets_root / "face" / "recognition.engine"
    whitelist_dir = args.whitelist_dir or assets_root / "face" / "whitelist"
    report = enroll_from_source(
        identity=args.name,
        source=args.source,
        detector_engine=detector_engine,
        recognition_engine=recognition_engine,
        whitelist_dir=whitelist_dir,
        face_config=config,
        template_count=args.templates,
        sample_stride=args.sample_stride,
        holdout_count=args.holdout_count,
        max_candidate_count=args.max_candidate_frames,
        replace=args.replace,
    )
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered)
    if args.report:
        report_path = Path(args.report).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
