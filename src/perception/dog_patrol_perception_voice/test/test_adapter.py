from __future__ import annotations

from collections import deque
import threading

from dog_patrol_perception_voice.adapter import R818VoiceAdapter, VoiceTaskCancelled
from dog_patrol_perception_voice.config import VoiceConfig
from dog_patrol_perception_voice.result import VoiceWindowResult


class FakeTaskStream:
    def __init__(self) -> None:
        self.calls: list[object] = []

    def start(self) -> None:
        self.calls.append("start")

    def window_chunks(self, timeout_seconds: float):
        self.calls.append(("window", timeout_seconds))
        return iter(())

    def close(self) -> None:
        self.calls.append("close")

    def request_stop(self) -> None:
        self.calls.append("request_stop")


class FakeWindowRecognizer:
    def __init__(self, results: list[VoiceWindowResult]) -> None:
        self._results = deque(results)
        self.calls: list[tuple[int, float]] = []

    def recognize(self, attempt_number: int, timeout_seconds: float) -> VoiceWindowResult:
        self.calls.append((attempt_number, timeout_seconds))
        return self._results.popleft()


class FakePromptPlayer:
    def __init__(self) -> None:
        self.prompts: list[str] = []

    def play(self, prompt: str) -> None:
        self.prompts.append(prompt)


def test_task_session_runs_two_independent_windows_with_one_stream_lifecycle() -> None:
    stream = FakeTaskStream()
    recognizer = FakeWindowRecognizer(
        [
            VoiceWindowResult(False, 20.0),
            VoiceWindowResult(True, 0.4),
        ]
    )
    prompt_player = FakePromptPlayer()
    adapter = R818VoiceAdapter(
        stream=stream,
        recognizer=recognizer,
        prompt_player=prompt_player,
        config=VoiceConfig(response_timeout_seconds=20.0),
    )

    with adapter.task() as task:
        first = task.respond()
        second = task.respond()

    assert first.accepted is False
    assert second.accepted is True
    assert first is not second
    assert recognizer.calls == [(1, 20.0), (2, 20.0)]
    assert prompt_player.prompts == [
        "Please face my camera directly or say the passphrase loudly.",
        "Once again, please face my camera directly or say the passphrase loudly.",
    ]
    assert stream.calls == ["start", "close"]


def test_task_cancel_interrupts_recognition_and_leaves_close_to_context_cleanup() -> None:
    class CancellableStream(FakeTaskStream):
        def __init__(self) -> None:
            super().__init__()
            self.stop_requested = threading.Event()

        def request_stop(self) -> None:
            super().request_stop()
            self.stop_requested.set()

    class BlockingRecognizer:
        def __init__(self, stream: CancellableStream) -> None:
            self.stream = stream

        def recognize(self, _attempt_number: int, _timeout_seconds: float):
            assert self.stream.stop_requested.wait(timeout=1.0)
            raise RuntimeError("recognizer interrupted")

    stream = CancellableStream()
    adapter = R818VoiceAdapter(
        stream=stream,
        recognizer=BlockingRecognizer(stream),
        prompt_player=FakePromptPlayer(),
    )
    errors: list[BaseException] = []

    with adapter.task() as task:
        worker = threading.Thread(
            target=lambda: _capture_error(errors, task.respond),
            daemon=True,
        )
        worker.start()
        task.cancel()
        worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], VoiceTaskCancelled)
    assert stream.calls == ["start", "request_stop", "close"]


def _capture_error(errors: list[BaseException], operation) -> None:
    try:
        operation()
    except BaseException as exc:
        errors.append(exc)
