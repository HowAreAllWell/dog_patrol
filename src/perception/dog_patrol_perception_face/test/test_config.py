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
    assert config.similarity_threshold == pytest.approx(0.3274939358)
    assert config.recognition_model == "mobilefacenet_arcface"
    assert config.recognition_embedding_dim == 512
    assert config.alignment_profile == "arcface_112"
    assert config.size_weighting_enabled is False


def test_load_face_config_roundtrip(tmp_path: Path) -> None:
    path = _write(
        tmp_path,
        """
detector_engine: /opt/face/det.engine
recognition_engine: /opt/face/sface.engine
recognition_model: mobilefacenet_arcface
recognition_embedding_dim: 512
alignment_profile: arcface_112
whitelist_dir: /opt/face/whitelist
similarity_threshold: 0.60
size_weighting_enabled: false
window_timeout_seconds: 4.0
""",
    )
    config = load_face_config(str(path))
    assert config.detector_engine == "/opt/face/det.engine"
    assert config.recognition_model == "mobilefacenet_arcface"
    assert config.recognition_embedding_dim == 512
    assert config.alignment_profile == "arcface_112"
    assert config.size_weighting_enabled is False
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


def test_invalid_recognition_model_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="recognition_model"):
        load_face_config(str(_write(tmp_path, "recognition_model: auto")))


def test_invalid_embedding_dimension_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="recognition_embedding_dim"):
        load_face_config(str(_write(tmp_path, "recognition_embedding_dim: 0")))


def test_recognition_model_and_embedding_dimension_must_match(tmp_path: Path) -> None:
    path = _write(
        tmp_path,
        "recognition_model: mobilefacenet_arcface\nrecognition_embedding_dim: 128",
    )

    with pytest.raises(ValueError, match="requires recognition_embedding_dim=512"):
        load_face_config(str(path))


def test_invalid_alignment_profile_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="alignment_profile"):
        load_face_config(str(_write(tmp_path, "alignment_profile: guessed")))


def test_mobilefacenet_requires_arcface_alignment(tmp_path: Path) -> None:
    path = _write(
        tmp_path,
        "\n".join(
            (
                "recognition_model: mobilefacenet_arcface",
                "recognition_embedding_dim: 512",
                "alignment_profile: legacy_sface",
            )
        ),
    )

    with pytest.raises(ValueError, match="requires alignment_profile='arcface_112'"):
        load_face_config(str(path))
