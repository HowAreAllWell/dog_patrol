from __future__ import annotations

from pathlib import Path

import pytest

from dog_patrol_perception_voice.config import default_config, load_voice_config


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_installed_voice_defaults_are_replayable_from_package_config() -> None:
    assert default_config() == load_voice_config(PACKAGE_ROOT / "config" / "voice.yaml")


def test_voice_config_rejects_unknown_fields(tmp_path) -> None:
    path = tmp_path / "voice.yaml"
    path.write_text("passphrase: blue star\nnot_a_runtime_option: true\n", encoding="utf-8")

    with pytest.raises(ValueError, match="unknown field"):
        load_voice_config(path)


def test_voice_config_rejects_empty_passphrase(tmp_path) -> None:
    path = tmp_path / "voice.yaml"
    path.write_text("passphrase: '  '\n", encoding="utf-8")

    with pytest.raises(ValueError, match="passphrase"):
        load_voice_config(path)
