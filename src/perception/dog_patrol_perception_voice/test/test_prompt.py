from __future__ import annotations

import subprocess
import threading

import pytest

import dog_patrol_perception_voice.prompt as prompt_module
from dog_patrol_perception_voice.prompt import FfmpegAlsaPromptPlayer


def test_prompt_player_synthesizes_then_plays_without_a_shell(monkeypatch) -> None:
    calls: list[tuple[list[str], dict[str, object], bytes | None]] = []

    class FakeProcess:
        def __init__(self, command, **settings):
            self.command = command
            self.settings = settings
            self.returncode = 0

        def communicate(self, *, input=None, timeout=None):
            del timeout
            calls.append((self.command, self.settings, input))
            if self.command[0] == "ffmpeg":
                return b"wav-bytes", b""
            return b"", b""

        def terminate(self):
            self.returncode = -15

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            del timeout
            return self.returncode

    def fake_popen(command, **settings):
        return FakeProcess(command, **settings)

    monkeypatch.setattr(prompt_module.subprocess, "Popen", fake_popen)

    prompt_module.FfmpegAlsaPromptPlayer(command_timeout_seconds=3.0).play("say 'hello'")

    assert [command[0] for command, _, _ in calls] == ["ffmpeg", "amixer", "aplay"]
    assert calls[-1][2] == b"wav-bytes"
    assert all("stdin" not in settings for _, settings, _ in calls[:2])
    assert calls[-1][1]["stdin"] is subprocess.PIPE


def test_prompt_player_request_stop_terminates_active_command(monkeypatch) -> None:
    started = threading.Event()
    terminated = threading.Event()

    class FakeProcess:
        def __init__(self, command, **_settings):
            self.command = command
            self.returncode = 0

        def communicate(self, *, input=None, timeout=None):
            del input, timeout
            if self.command[0] == "aplay":
                started.set()
                assert terminated.wait(timeout=1.0)
            return b"wav-bytes" if self.command[0] == "ffmpeg" else b"", b""

        def terminate(self):
            self.returncode = -15
            terminated.set()

        def kill(self):
            self.returncode = -9
            terminated.set()

        def wait(self, timeout=None):
            del timeout
            return self.returncode

    monkeypatch.setattr(prompt_module.subprocess, "Popen", FakeProcess)
    player = prompt_module.FfmpegAlsaPromptPlayer(command_timeout_seconds=3.0)
    errors: list[BaseException] = []

    worker = threading.Thread(target=lambda: _capture_error(errors, player.play, "hello"))
    worker.start()
    assert started.wait(timeout=1.0)
    player.request_stop()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert len(errors) == 1
    assert "cancelled" in str(errors[0])


def test_run_command_keeps_complete_audio_input_until_process_finishes(monkeypatch) -> None:
    calls: list[tuple[bytes | None, float | None]] = []

    class FakeProcess:
        returncode = 0

        def __init__(self, command, **_settings):
            self.command = command

        def communicate(self, *, input=None, timeout=None):
            calls.append((input, timeout))
            if timeout is not None and timeout < 1.0:
                raise subprocess.TimeoutExpired(self.command, timeout)
            return b"", b""

        def terminate(self):
            self.returncode = -15

        def kill(self):
            self.returncode = -9

        def wait(self, timeout=None):
            del timeout
            return self.returncode

    monkeypatch.setattr(prompt_module.subprocess, "Popen", FakeProcess)

    result = prompt_module._run_command(
        ["aplay"],
        stage="Prompt playback",
        timeout_seconds=2.0,
        input=b"complete-wav",
    )

    assert result.returncode == 0
    assert calls == [(b"complete-wav", 2.0)]


def _capture_error(errors: list[BaseException], operation, *args) -> None:
    try:
        operation(*args)
    except BaseException as exc:
        errors.append(exc)


def test_prompt_player_rejects_invalid_settings() -> None:
    with pytest.raises(ValueError, match="volume"):
        FfmpegAlsaPromptPlayer(volume_percent=101)
    with pytest.raises(ValueError, match="timeout"):
        FfmpegAlsaPromptPlayer(command_timeout_seconds=0)


def test_prompt_player_rejects_empty_prompt() -> None:
    with pytest.raises(ValueError, match="prompt"):
        FfmpegAlsaPromptPlayer().play(" ")
