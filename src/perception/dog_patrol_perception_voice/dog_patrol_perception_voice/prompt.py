"""Prompt playback for the production voice task."""

from __future__ import annotations

import math
import subprocess
from typing import Any


class FfmpegAlsaPromptPlayer:
    """Render one short Prompt and synchronously play it through ALSA."""

    def __init__(
        self,
        *,
        device: str = "plughw:CARD=Device,DEV=0",
        mixer_card: str = "Device",
        mixer_control: str = "PCM",
        volume_percent: int = 100,
        command_timeout_seconds: float = 10.0,
    ) -> None:
        if not device.strip() or not mixer_card.strip() or not mixer_control.strip():
            raise ValueError("Prompt playback device and mixer settings must not be empty")
        if isinstance(volume_percent, bool) or not 0 <= volume_percent <= 100:
            raise ValueError("volume_percent must be between 0 and 100")
        if not math.isfinite(command_timeout_seconds) or command_timeout_seconds <= 0:
            raise ValueError("command_timeout_seconds must be finite and positive")
        self._device = device
        self._mixer_card = mixer_card
        self._mixer_control = mixer_control
        self._volume_percent = volume_percent
        self._command_timeout_seconds = command_timeout_seconds

    def play(self, prompt: str) -> None:
        normalized_prompt = prompt.strip()
        if not normalized_prompt:
            raise ValueError("prompt must not be empty")
        escaped_prompt = normalized_prompt.replace("\\", "\\\\").replace("'", "\\'")
        synthesis = _run_command(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "lavfi",
                "-i",
                f"flite=text='{escaped_prompt}':voice=kal16",
                "-ar",
                "48000",
                "-ac",
                "2",
                "-f",
                "wav",
                "pipe:1",
            ],
            stage="Prompt synthesis",
            timeout_seconds=self._command_timeout_seconds,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _run_command(
            [
                "amixer",
                "-q",
                "-c",
                self._mixer_card,
                "set",
                self._mixer_control,
                f"{self._volume_percent}%",
            ],
            stage="Prompt volume update",
            timeout_seconds=self._command_timeout_seconds,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _run_command(
            ["aplay", "-q", "-D", self._device, "-t", "wav"],
            stage="Prompt playback",
            timeout_seconds=self._command_timeout_seconds,
            input=synthesis.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )


def _run_command(
    command: list[str],
    *,
    stage: str,
    timeout_seconds: float,
    **kwargs: Any,
) -> subprocess.CompletedProcess[bytes]:
    try:
        result = subprocess.run(
            command,
            check=False,
            timeout=timeout_seconds,
            **kwargs,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(f"{stage} executable was not found: {command[0]}") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"{stage} timed out after {timeout_seconds:g} seconds") from exc
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(
            f"{stage} exited with status {result.returncode}" + (f": {detail}" if detail else "")
        )
    return result
