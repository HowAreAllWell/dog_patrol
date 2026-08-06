from __future__ import annotations

import subprocess

import pytest

import dog_patrol_perception_voice.prompt as prompt_module
from dog_patrol_perception_voice.prompt import FfmpegAlsaPromptPlayer


def test_prompt_player_synthesizes_then_plays_without_a_shell(monkeypatch) -> None:
    calls: list[tuple[list[str], dict[str, object]]] = []

    def fake_run(command, **settings):
        calls.append((command, settings))
        if command[0] == "ffmpeg":
            return subprocess.CompletedProcess(command, 0, b"wav-bytes", b"")
        return subprocess.CompletedProcess(command, 0, b"", b"")

    monkeypatch.setattr(prompt_module.subprocess, "run", fake_run)

    FfmpegAlsaPromptPlayer(command_timeout_seconds=3.0).play("say 'hello'")

    assert [command[0] for command, _ in calls] == ["ffmpeg", "amixer", "aplay"]
    assert calls[-1][1]["input"] == b"wav-bytes"
    assert all(settings["timeout"] == 3.0 for _, settings in calls)
    assert all(settings["check"] is False for _, settings in calls)


def test_prompt_player_rejects_invalid_settings() -> None:
    with pytest.raises(ValueError, match="volume"):
        FfmpegAlsaPromptPlayer(volume_percent=101)
    with pytest.raises(ValueError, match="timeout"):
        FfmpegAlsaPromptPlayer(command_timeout_seconds=0)


def test_prompt_player_rejects_empty_prompt() -> None:
    with pytest.raises(ValueError, match="prompt"):
        FfmpegAlsaPromptPlayer().play(" ")
