from __future__ import annotations

from pathlib import Path

import pytest

from dog_patrol_perception_face.config import FaceConfig, load_face_config


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "face.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def test_defaults_without_config() -> None:
    config = FaceConfig()
    assert config.window_timeout_seconds == 3.0
    assert config.similarity_threshold == 0.55


def test_load_face_config_roundtrip(tmp_path: Path) -> None:
    path = _write(
        tmp_path,
        """
detector_engine: /opt/face/det.engine
recognition_engine: /opt/face/sface.engine
whitelist_dir: /opt/face/whitelist
similarity_threshold: 0.60
window_timeout_seconds: 4.0
""",
    )
    config = load_face_config(str(path))
    assert config.detector_engine == "/opt/face/det.engine"
    assert config.similarity_threshold == 0.60
    assert config.window_timeout_seconds == 4.0
    assert config.temporal_window == 10


def test_missing_config_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="missing"):
        load_face_config(str(tmp_path / "nope.yaml"))


def test_invalid_yaml_raises(tmp_path: Path) -> None:
    path = _write(tmp_path, "detector_engine: [unclosed")
    with pytest.raises(ValueError, match="YAML"):
        load_face_config(str(path))


def test_wrong_type_raises(tmp_path: Path) -> None:
    path = _write(tmp_path, "window_timeout_seconds: many")
    with pytest.raises(ValueError, match="invalid"):
        load_face_config(str(path))


def test_nonpositive_window_raises(tmp_path: Path) -> None:
    path = _write(tmp_path, "window_timeout_seconds: 0")
    with pytest.raises(ValueError, match="positive"):
        load_face_config(str(path))
