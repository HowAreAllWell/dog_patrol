from __future__ import annotations

import sys
import types

import pytest

from dog_patrol_perception_voice.vosk import load_vosk_model


def test_vosk_loader_disables_native_logs_and_returns_recognizer_factory(tmp_path, monkeypatch) -> None:
    observed: dict[str, object] = {}

    class FakeModel:
        def __init__(self, path: str) -> None:
            observed["model_path"] = path

    class FakeRecognizer:
        def __init__(self, model, rate: int, grammar: str) -> None:
            observed["recognizer"] = (model, rate, grammar)

    fake_vosk = types.ModuleType("vosk")
    fake_vosk.SetLogLevel = lambda level: observed.setdefault("log_level", level)
    fake_vosk.Model = FakeModel
    fake_vosk.KaldiRecognizer = FakeRecognizer
    monkeypatch.setitem(sys.modules, "vosk", fake_vosk)

    model, factory = load_vosk_model(tmp_path)
    recognizer = factory(model, 16_000, '["blue star", "[unk]"]')

    assert observed == {
        "log_level": -1,
        "model_path": str(tmp_path),
        "recognizer": (model, 16_000, '["blue star", "[unk]"]'),
    }
    assert isinstance(recognizer, FakeRecognizer)


def test_vosk_loader_requires_a_model_directory(tmp_path) -> None:
    with pytest.raises(ValueError, match="directory"):
        load_vosk_model(tmp_path / "missing")
