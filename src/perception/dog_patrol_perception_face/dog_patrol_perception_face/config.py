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
    whitelist_dir: str = ""
    similarity_threshold: float = 0.55
    size_sigma: float = 20.0
    temporal_window: int = 10
    min_frames_for_decision: int = 5
    window_timeout_seconds: float = 3.0
    crop_freshness_timeout_seconds: float = 1.0
    overlay_inference_hz: float = 10.0
    face_min_score: float = 0.5
    face_min_box_ratio: float = 0.03
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
    ):
        if values[key] <= 0:
            raise ValueError(f"face config field {key!r} must be positive")
    for key in ("window_timeout_seconds", "crop_freshness_timeout_seconds"):
        if values[key] <= 0.0:
            raise ValueError(f"face config field {key!r} must be positive")

    return FaceConfig(**values)
