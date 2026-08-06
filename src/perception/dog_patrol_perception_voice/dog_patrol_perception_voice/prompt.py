"""Prompt playback for the production voice task."""

from __future__ import annotations

from collections.abc import Callable
import math
import subprocess
import threading
import time
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
        self._cancel_requested = threading.Event()
        self._process_lock = threading.Lock()
        self._active_process: subprocess.Popen[bytes] | None = None

    def reset_stop(self) -> None:
        """Clear the previous task's stop request before a new task starts."""
        self._cancel_requested.clear()

    def request_stop(self) -> None:
        """Ask the active Prompt process to terminate without waiting for it."""
        self._cancel_requested.set()
        with self._process_lock:
            process = self._active_process
        if process is not None:
            try:
                process.terminate()
            except OSError:
                pass

    def play(self, prompt: str) -> None:
        self._raise_if_cancelled()
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
            cancel_event=self._cancel_requested,
            process_callback=self._set_active_process,
            clear_process_callback=self._clear_active_process,
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
            cancel_event=self._cancel_requested,
            process_callback=self._set_active_process,
            clear_process_callback=self._clear_active_process,
        )
        _run_command(
            ["aplay", "-q", "-D", self._device, "-t", "wav"],
            stage="Prompt playback",
            timeout_seconds=self._command_timeout_seconds,
            input=synthesis.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            cancel_event=self._cancel_requested,
            process_callback=self._set_active_process,
            clear_process_callback=self._clear_active_process,
        )

    def _raise_if_cancelled(self) -> None:
        if self._cancel_requested.is_set():
            raise RuntimeError("Prompt playback was cancelled")

    def _set_active_process(self, process: subprocess.Popen[bytes]) -> None:
        with self._process_lock:
            self._active_process = process

    def _clear_active_process(self, process: subprocess.Popen[bytes]) -> None:
        with self._process_lock:
            if self._active_process is process:
                self._active_process = None


def _run_command(
    command: list[str],
    *,
    stage: str,
    timeout_seconds: float,
    cancel_event: threading.Event | None = None,
    process_callback: Callable[[subprocess.Popen[bytes]], None] | None = None,
    clear_process_callback: Callable[[subprocess.Popen[bytes]], None] | None = None,
    **kwargs: Any,
) -> subprocess.CompletedProcess[bytes]:
    input_data = kwargs.pop("input", None)
    if input_data is not None:
        kwargs["stdin"] = subprocess.PIPE
    try:
        process = subprocess.Popen(command, **kwargs)
    except FileNotFoundError as exc:
        raise RuntimeError(f"{stage} executable was not found: {command[0]}") from exc
    if process_callback is not None:
        process_callback(process)
    try:
        started_at = time.monotonic()
        pending_input = input_data
        while True:
            if cancel_event is not None and cancel_event.is_set():
                _terminate_process(process)
                raise RuntimeError(f"{stage} was cancelled")
            remaining = timeout_seconds - (time.monotonic() - started_at)
            if remaining <= 0:
                _terminate_process(process)
                raise RuntimeError(f"{stage} timed out after {timeout_seconds:g} seconds")
            try:
                stdout, stderr = process.communicate(
                    input=pending_input,
                    timeout=min(remaining, 0.05),
                )
                break
            except subprocess.TimeoutExpired:
                pending_input = None
        result = subprocess.CompletedProcess(
            command,
            process.returncode,
            stdout,
            stderr,
        )
        if cancel_event is not None and cancel_event.is_set():
            raise RuntimeError(f"{stage} was cancelled")
    finally:
        if clear_process_callback is not None:
            clear_process_callback(process)
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(
            f"{stage} exited with status {result.returncode}" + (f": {detail}" if detail else "")
        )
    return result


def _terminate_process(process: subprocess.Popen[bytes]) -> None:
    try:
        process.terminate()
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    except OSError:
        pass
