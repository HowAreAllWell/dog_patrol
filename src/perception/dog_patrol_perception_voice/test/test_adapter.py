from __future__ import annotations

from collections import deque

from dog_patrol_perception_voice.adapter import R818VoiceAdapter
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
