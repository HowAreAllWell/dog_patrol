"""Face provider configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class FaceConfig:
    detector_engine: str = ""
    detector_conf_thres: float = 0.25
    detector_iou_thres: float = 0.5
    recognition_engine: str = ""
    recognition_model: str = "mobilefacenet_arcface"
    recognition_embedding_dim: int = 512
    alignment_profile: str = "arcface_112"
    whitelist_dir: str = ""
    similarity_threshold: float = 0.3274939358
    size_sigma: float = 20.0
    size_weighting_enabled: bool = False
    temporal_window: int = 10
    min_frames_for_decision: int = 5
    window_timeout_seconds: float = 3.0
    crop_freshness_timeout_seconds: float = 1.0
    overlay_inference_hz: float = 10.0
    face_min_score: float = 0.4
    face_min_box_ratio: float = 0.01
    stop_after_matched: bool = True
    matched_hold_seconds: float = 5.0

    @property
    def required_inputs(self) -> tuple[str, ...]:
        return (self.detector_engine, self.recognition_engine, self.whitelist_dir)


_FIELDS = tuple(FaceConfig.__dataclass_fields__.keys())


def load_face_config(path) -> FaceConfig:
    config_file = Path(path)
    if not config_file.is_file():
        raise ValueError(f"face config is missing: {config_file}")
    try:
        raw = yaml.safe_load(config_file.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f"face config is not valid YAML: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("face config root must be a mapping")

    defaults = {field: getattr(FaceConfig, field) for field in _FIELDS}

    def _coerce(key, value):
        default = defaults[key]
        if value is None:
            return default
        if isinstance(default, bool):
            if not isinstance(value, bool):
                raise ValueError(f"field {key!r} must be a bool")
            return value
        if isinstance(default, int):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"field {key!r} must be a number")
            return int(value)
        if isinstance(default, float):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"field {key!r} must be a number")
            return float(value)
        if isinstance(default, str):
            if not isinstance(value, str):
                raise ValueError(f"field {key!r} must be a string")
            return value
        return value

    try:
        values = {key: _coerce(key, raw.get(key)) for key in _FIELDS}
    except ValueError as exc:
        raise ValueError(f"face config is invalid: {exc}") from exc

    for key in (
        "temporal_window",
        "min_frames_for_decision",
        "recognition_embedding_dim",
    ):
        if values[key] <= 0:
            raise ValueError(f"face config field {key!r} must be positive")
    for key in ("window_timeout_seconds", "crop_freshness_timeout_seconds"):
        if values[key] <= 0.0:
            raise ValueError(f"face config field {key!r} must be positive")
    if values["recognition_model"] not in {"sface", "mobilefacenet_arcface"}:
        raise ValueError(
            "face config field 'recognition_model' must be 'sface' or "
            "'mobilefacenet_arcface'"
        )
    if values["alignment_profile"] not in {"legacy_sface", "arcface_112"}:
        raise ValueError(
            "face config field 'alignment_profile' must be 'legacy_sface' or "
            "'arcface_112'"
        )
    expected_dimensions = {"sface": 128, "mobilefacenet_arcface": 512}
    expected_dimension = expected_dimensions[values["recognition_model"]]
    if values["recognition_embedding_dim"] != expected_dimension:
        raise ValueError(
            f"face config model {values['recognition_model']!r} requires "
            f"recognition_embedding_dim={expected_dimension}"
        )
    expected_alignments = {
        "sface": "legacy_sface",
        "mobilefacenet_arcface": "arcface_112",
    }
    expected_alignment = expected_alignments[values["recognition_model"]]
    if values["alignment_profile"] != expected_alignment:
        raise ValueError(
            f"face config model {values['recognition_model']!r} requires "
            f"alignment_profile={expected_alignment!r}"
        )

    return FaceConfig(**values)
