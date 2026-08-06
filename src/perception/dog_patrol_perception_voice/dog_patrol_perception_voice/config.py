"""Configuration for the production voice verification core."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


DEFAULT_FIRST_PROMPT = "Please face my camera directly or say the passphrase loudly."
DEFAULT_RETRY_PROMPT = "Once again, please face my camera directly or say the passphrase loudly."


@dataclass(frozen=True)
class VoiceConfig:
    passphrase: str = "blue star"
    first_prompt: str = DEFAULT_FIRST_PROMPT
    retry_prompt: str = DEFAULT_RETRY_PROMPT
    response_timeout_seconds: float = 20.0
    vote_guard_seconds: float = 0.75
    adb_executable: str = "adb"
    device_serial: str | None = None
    adb_timeout_seconds: float = 15.0
    start_timeout_seconds: float = 5.0
    restore_timeout_seconds: float = 20.0
    max_buffer_seconds: float = 4.0
    prompt_device: str = "plughw:CARD=Device,DEV=0"
    prompt_mixer_card: str = "Device"
    prompt_mixer_control: str = "PCM"
    prompt_volume_percent: int = 100

    def __post_init__(self) -> None:
        if not self.passphrase.strip():
            raise ValueError("passphrase must not be empty")
        if not self.first_prompt.strip() or not self.retry_prompt.strip():
            raise ValueError("prompts must not be empty")
        for name, value in (
            ("response_timeout_seconds", self.response_timeout_seconds),
            ("vote_guard_seconds", self.vote_guard_seconds),
        ):
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.response_timeout_seconds == 0:
            raise ValueError("response_timeout_seconds must be positive")
        for name, value in (
            ("adb_timeout_seconds", self.adb_timeout_seconds),
            ("start_timeout_seconds", self.start_timeout_seconds),
            ("restore_timeout_seconds", self.restore_timeout_seconds),
            ("max_buffer_seconds", self.max_buffer_seconds),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name, value in (
            ("adb_executable", self.adb_executable),
            ("prompt_device", self.prompt_device),
            ("prompt_mixer_card", self.prompt_mixer_card),
            ("prompt_mixer_control", self.prompt_mixer_control),
        ):
            if not value.strip():
                raise ValueError(f"{name} must not be empty")
        if self.device_serial is not None and not self.device_serial.strip():
            raise ValueError("device_serial must not be empty")
        if (
            isinstance(self.prompt_volume_percent, bool)
            or not isinstance(self.prompt_volume_percent, int)
            or not 0 <= self.prompt_volume_percent <= 100
        ):
            raise ValueError("prompt_volume_percent must be an integer between 0 and 100")


def default_config() -> VoiceConfig:
    return VoiceConfig()


def load_voice_config(path: str | Path) -> VoiceConfig:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"could not read voice config {config_path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in voice config {config_path}: {exc}") from exc
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError("voice configuration must be a mapping")
    allowed = {
        "passphrase",
        "first_prompt",
        "retry_prompt",
        "response_timeout_seconds",
        "vote_guard_seconds",
        "adb_executable",
        "device_serial",
        "adb_timeout_seconds",
        "start_timeout_seconds",
        "restore_timeout_seconds",
        "max_buffer_seconds",
        "prompt_device",
        "prompt_mixer_card",
        "prompt_mixer_control",
        "prompt_volume_percent",
    }
    unknown = set(raw) - allowed
    if unknown:
        names = ", ".join(sorted(repr(name) for name in unknown))
        raise ValueError(f"voice configuration contains unknown field(s): {names}")
    try:
        return VoiceConfig(
            passphrase=_string(raw.get("passphrase", "blue star"), "passphrase"),
            first_prompt=_string(raw.get("first_prompt", DEFAULT_FIRST_PROMPT), "first_prompt"),
            retry_prompt=_string(raw.get("retry_prompt", DEFAULT_RETRY_PROMPT), "retry_prompt"),
            response_timeout_seconds=_finite(
                raw.get("response_timeout_seconds", 20.0), "response_timeout_seconds"
            ),
            vote_guard_seconds=_finite(raw.get("vote_guard_seconds", 0.75), "vote_guard_seconds"),
            adb_executable=_string(raw.get("adb_executable", "adb"), "adb_executable"),
            device_serial=_optional_string(raw.get("device_serial"), "device_serial"),
            adb_timeout_seconds=_finite(raw.get("adb_timeout_seconds", 15.0), "adb_timeout_seconds"),
            start_timeout_seconds=_finite(
                raw.get("start_timeout_seconds", 5.0), "start_timeout_seconds"
            ),
            restore_timeout_seconds=_finite(
                raw.get("restore_timeout_seconds", 20.0), "restore_timeout_seconds"
            ),
            max_buffer_seconds=_finite(raw.get("max_buffer_seconds", 4.0), "max_buffer_seconds"),
            prompt_device=_string(raw.get("prompt_device", "plughw:CARD=Device,DEV=0"), "prompt_device"),
            prompt_mixer_card=_string(raw.get("prompt_mixer_card", "Device"), "prompt_mixer_card"),
            prompt_mixer_control=_string(raw.get("prompt_mixer_control", "PCM"), "prompt_mixer_control"),
            prompt_volume_percent=_integer(
                raw.get("prompt_volume_percent", 100), "prompt_volume_percent"
            ),
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid voice configuration: {exc}") from exc


def _string(value: Any, location: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{location} must be a string")
    return value


def _optional_string(value: Any, location: str) -> str | None:
    if value is None:
        return None
    return _string(value, location)


def _finite(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{location} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{location} must be finite")
    return result


def _integer(value: Any, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{location} must be an integer")
    return value
