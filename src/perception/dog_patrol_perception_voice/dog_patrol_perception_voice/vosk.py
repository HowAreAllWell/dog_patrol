"""Lazy Vosk model loading for the six-microphone window recognizer."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol


class VoskRecognizerFactory(Protocol):
    def __call__(self, model: Any, sample_rate: int, grammar: str) -> Any: ...


def load_vosk_model(model_dir: str | Path) -> tuple[Any, VoskRecognizerFactory]:
    path = Path(model_dir)
    if not path.is_dir():
        raise ValueError(f"Vosk model directory does not exist: {path}")
    try:
        import vosk
    except ImportError as exc:
        raise RuntimeError("the Vosk runtime dependency is not installed") from exc
    set_log_level = getattr(vosk, "SetLogLevel", None)
    if set_log_level is not None:
        set_log_level(-1)
    try:
        model = vosk.Model(str(path))
    except (OSError, RuntimeError, ValueError) as exc:
        raise RuntimeError(f"could not load Vosk model {path}: {exc}") from exc

    def make_recognizer(loaded_model: Any, sample_rate: int, grammar: str) -> Any:
        return vosk.KaldiRecognizer(loaded_model, sample_rate, grammar)

    return model, make_recognizer
