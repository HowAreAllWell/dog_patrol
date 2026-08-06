"""Production Adapter for finite-lifetime R818 voice verification tasks."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from .config import VoiceConfig
from .result import VoiceWindowResult


class TaskStream(Protocol):
    def start(self) -> None: ...

    def close(self) -> None: ...


class WindowRecognizer(Protocol):
    def recognize(self, attempt_number: int, timeout_seconds: float) -> VoiceWindowResult: ...


class PromptPlayer(Protocol):
    def play(self, prompt: str) -> None: ...


class R818VoiceAdapter:
    """Create task sessions without combining their independent windows."""

    def __init__(
        self,
        *,
        config: VoiceConfig | None = None,
        prompt_player: PromptPlayer,
        stream: TaskStream | None = None,
        stream_factory: Callable[[], TaskStream] | None = None,
        recognizer: WindowRecognizer | None = None,
        recognizer_factory: Callable[[TaskStream], WindowRecognizer] | None = None,
    ) -> None:
        if stream is not None and stream_factory is not None:
            raise ValueError("stream and stream_factory are mutually exclusive")
        if recognizer is not None and recognizer_factory is not None:
            raise ValueError("recognizer and recognizer_factory are mutually exclusive")
        if stream is None and stream_factory is None:
            raise ValueError("stream or stream_factory is required")
        if recognizer is None and recognizer_factory is None:
            raise ValueError("recognizer or recognizer_factory is required")
        self._config = config if config is not None else VoiceConfig()
        self._prompt_player = prompt_player
        self._stream_factory = stream_factory or _single_value_factory(stream)
        self._recognizer = recognizer
        self._recognizer_factory = recognizer_factory

    @classmethod
    def from_model_dir(
        cls,
        model_dir: str,
        *,
        config: VoiceConfig | None = None,
        adb: TaskStream | None = None,
        prompt_player: PromptPlayer | None = None,
        helper_path: str | None = None,
    ) -> R818VoiceAdapter:
        """Build the production Adapter from deployment-provided model assets."""
        from .adb import SubprocessAdbFileTransfer
        from .prompt import FfmpegAlsaPromptPlayer
        from .r818_stream import R818StreamingVoskSession, SubprocessAdbEncodedPcmStream
        from .vosk import load_vosk_model

        selected_config = config if config is not None else VoiceConfig()
        selected_adb = adb
        if selected_adb is None:
            selected_adb = SubprocessAdbFileTransfer(
                selected_config.adb_executable,
                device_serial=selected_config.device_serial,
                timeout_seconds=selected_config.adb_timeout_seconds,
            )
        selected_prompt_player = prompt_player
        if selected_prompt_player is None:
            selected_prompt_player = FfmpegAlsaPromptPlayer(
                device=selected_config.prompt_device,
                mixer_card=selected_config.prompt_mixer_card,
                mixer_control=selected_config.prompt_mixer_control,
                volume_percent=selected_config.prompt_volume_percent,
            )
        model, recognizer_factory = load_vosk_model(model_dir)

        def make_stream() -> TaskStream:
            settings: dict[str, object] = {
                "start_timeout_seconds": selected_config.start_timeout_seconds,
                "restore_timeout_seconds": selected_config.restore_timeout_seconds,
                "max_buffer_seconds": selected_config.max_buffer_seconds,
            }
            if helper_path is not None:
                settings["helper_path"] = helper_path
            return SubprocessAdbEncodedPcmStream(selected_adb, **settings)

        def make_recognizer(stream: TaskStream) -> WindowRecognizer:
            return R818StreamingVoskSession(
                stream,
                selected_config,
                model=model,
                recognizer_factory=recognizer_factory,
            )

        return cls(
            config=selected_config,
            prompt_player=selected_prompt_player,
            stream_factory=make_stream,
            recognizer_factory=make_recognizer,
        )

    def task(self) -> R818TaskSession:
        """Return one session; its stream is started and closed by the context."""
        stream = self._stream_factory()
        recognizer = self._recognizer
        if self._recognizer_factory is not None:
            recognizer = self._recognizer_factory(stream)
        assert recognizer is not None
        return R818TaskSession(stream, recognizer, self._prompt_player, self._config)


class R818TaskSession:
    """One R818 ownership period serving at most two independent windows."""

    def __init__(
        self,
        stream: TaskStream,
        recognizer: WindowRecognizer,
        prompt_player: PromptPlayer,
        config: VoiceConfig,
    ) -> None:
        self._stream = stream
        self._recognizer = recognizer
        self._prompt_player = prompt_player
        self._config = config
        self._active = False
        self._closed = False
        self._attempt_number = 0

    def __enter__(self) -> R818TaskSession:
        if self._active or self._closed:
            raise RuntimeError("R818 task session cannot be entered again")
        self._stream.start()
        self._active = True
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._stream.close()
        finally:
            self._active = False

    def respond(
        self,
        prompt: str | None = None,
        *,
        timeout_seconds: float | None = None,
    ) -> VoiceWindowResult:
        if not self._active:
            raise RuntimeError("R818 task session must be entered before respond")
        if self._attempt_number >= 2:
            raise RuntimeError("R818 task sessions support at most two response windows")
        self._attempt_number += 1
        selected_prompt = prompt
        if selected_prompt is None:
            selected_prompt = (
                self._config.first_prompt if self._attempt_number == 1 else self._config.retry_prompt
            )
        if not selected_prompt.strip():
            raise ValueError("prompt must not be empty")
        response_timeout = (
            self._config.response_timeout_seconds if timeout_seconds is None else timeout_seconds
        )
        self._prompt_player.play(selected_prompt)
        result = self._recognizer.recognize(self._attempt_number, response_timeout)
        if result.attempt_number not in (0, self._attempt_number):
            raise ValueError("recognizer returned a result for the wrong response window")
        if result.attempt_number == 0:
            return VoiceWindowResult(
                accepted=result.accepted,
                decision_time_seconds=result.decision_time_seconds,
                attempt_number=self._attempt_number,
                pcm_bytes=result.pcm_bytes,
                captured_duration_seconds=result.captured_duration_seconds,
                channel_results=result.channel_results,
                matching_channels=result.matching_channels,
                vote_counts=result.vote_counts,
            )
        return result


def _single_value_factory(value: TaskStream | None) -> Callable[[], TaskStream]:
    assert value is not None
    used = False

    def make() -> TaskStream:
        nonlocal used
        if used:
            raise RuntimeError("a single injected stream can only serve one task")
        used = True
        return value

    return make
