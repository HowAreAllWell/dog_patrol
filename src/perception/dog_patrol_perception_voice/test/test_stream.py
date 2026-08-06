from __future__ import annotations

import base64
import io
import json
import queue
import threading
from collections import deque
from types import SimpleNamespace

import pytest

from dog_patrol_perception_voice.r818_stream import (
    R818HardwareUnreadyError,
    R818_ORIGIN_FRAME_BYTES,
    R818StreamingVoskSession,
    SubprocessAdbEncodedPcmStream,
    decode_base64_pcm_chunks,
)
from dog_patrol_perception_voice.config import VoiceConfig
from dog_patrol_perception_voice.result import VoiceWindowResult


def test_base64_decoder_reassembles_input_and_only_yields_complete_eight_channel_frames() -> None:
    pcm = bytes(range(32))
    encoded = base64.b64encode(pcm)

    decoded = b"".join(decode_base64_pcm_chunks([encoded[:3], encoded[3:11], encoded[11:]]))

    assert decoded == pcm


def test_base64_decoder_rejects_corrupt_encoding() -> None:
    with pytest.raises(R818HardwareUnreadyError, match="Base64"):
        list(decode_base64_pcm_chunks([b"not-base64"]))


def test_base64_decoder_rejects_an_incomplete_interleaved_frame() -> None:
    encoded = base64.b64encode(b"\x00" * 15)

    with pytest.raises(R818HardwareUnreadyError, match="16-byte"):
        list(decode_base64_pcm_chunks([encoded]))


class BlockingReader:
    def __init__(self) -> None:
        self._chunks: queue.Queue[bytes] = queue.Queue()

    def feed(self, chunk: bytes) -> None:
        self._chunks.put(chunk)

    def read(self, _size: int) -> bytes:
        return self._chunks.get()

    def close(self) -> None:
        self.feed(b"")


class NotifyingReader(BlockingReader):
    def __init__(self) -> None:
        super().__init__()
        self.data_read = threading.Event()

    def read(self, size: int) -> bytes:
        data = super().read(size)
        if data:
            self.data_read.set()
        return data


class FakeProcess:
    def __init__(self, stdout: BlockingReader) -> None:
        self.stdout = stdout
        self.stderr = io.BytesIO()
        self.returncode: int | None = None

    def poll(self) -> int | None:
        return self.returncode

    def terminate(self) -> None:
        self.returncode = 0
        self.stdout.close()

    def kill(self) -> None:
        self.terminate()

    def wait(self, timeout: float | None = None) -> int:
        self.returncode = 0
        return 0


class FakeAdb:
    def __init__(self, reader: BlockingReader) -> None:
        self.calls: list[object] = []
        self.process = FakeProcess(reader)

    def ensure_ready(self) -> None:
        self.calls.append("ensure_ready")

    def push(self, local_path, remote_path) -> None:
        self.calls.append(("push", local_path, remote_path))

    def shell(self, arguments, *, timeout_seconds=None, allow_failure=False):
        self.calls.append(("shell", arguments, timeout_seconds, allow_failure))
        if arguments[:2] == ("cat", "/proc/asound/card1/pcm0c/sub0/status"):
            return SimpleNamespace(stdout="state: RUNNING\nowner_pid: 123\n")
        return SimpleNamespace(stdout="")

    def start_shell_script(self, script):
        self.calls.append(("start_shell_script", script))
        return self.process

    def shell_script(self, script):
        self.calls.append(("shell_script", script))
        return SimpleNamespace(stdout="")


def _frame(marker: int) -> bytes:
    return bytes([marker]) * R818_ORIGIN_FRAME_BYTES


def test_stream_discards_complete_prompt_frames_before_opening_the_response_window(tmp_path) -> None:
    reader = BlockingReader()
    prompt_audio = _frame(1)
    response_audio = _frame(2)
    reader.feed(base64.b64encode(prompt_audio))
    helper = tmp_path / "helper"
    helper.write_bytes(b"helper")
    stream = SubprocessAdbEncodedPcmStream(
        FakeAdb(reader),
        helper_path=helper,
        max_buffer_seconds=1.0,
    )
    stream.start()
    encoded_response = base64.b64encode(response_audio)

    chunks = stream.window_chunks(0.5)
    try:
        reader.feed(encoded_response[:5])
        reader.feed(encoded_response[5:])
        assert next(chunks) == response_audio
    finally:
        chunks.close()
        stream.close()


def test_window_deadline_is_not_extended_by_queued_audio(tmp_path) -> None:
    now = [0.0]
    reader = NotifyingReader()
    reader.feed(base64.b64encode(_frame(0)))
    helper = tmp_path / "helper"
    helper.write_bytes(b"helper")
    stream = SubprocessAdbEncodedPcmStream(
        FakeAdb(reader),
        helper_path=helper,
        clock=lambda: now[0],
    )
    stream.start()
    reader.data_read.clear()
    chunks = stream.window_chunks(1.0)
    reader.feed(base64.b64encode(_frame(1)))
    assert reader.data_read.wait(timeout=1.0)
    now[0] = 1.01

    try:
        assert list(chunks) == []
    finally:
        chunks.close()
        stream.close()


def test_stream_takes_over_and_restores_vendor_audio_once_per_task(tmp_path) -> None:
    reader = BlockingReader()

    helper = tmp_path / "helper"
    helper.write_bytes(b"helper")
    adb = FakeAdb(reader)
    reader.feed(base64.b64encode(_frame(1)))
    stream = SubprocessAdbEncodedPcmStream(adb, helper_path=helper)

    stream.start()
    stream.close()
    stream.close()

    assert adb.calls[0:5] == [
        "ensure_ready",
        ("push", helper, "/tmp/dog-patrol-r818-base64"),
        ("shell", ("chmod", "700", "/tmp/dog-patrol-r818-base64"), None, False),
        ("shell", ("/etc/init.d/vtn_init", "stop"), None, False),
        ("shell", ("killall", "demo"), None, True),
    ]
    assert sum(isinstance(call, tuple) and call[0] == "start_shell_script" for call in adb.calls) == 1
    assert sum(isinstance(call, tuple) and call[0] == "shell_script" for call in adb.calls) == 1
    assert sum(
        call == ("shell", ("/etc/init.d/vtn_init", "start"), None, False)
        for call in adb.calls
    ) == 1


class FakeWindowStream:
    def __init__(self, windows: list[list[bytes]]) -> None:
        self._windows = deque(windows)
        self.calls: list[object] = []

    def window_chunks(self, timeout_seconds: float):
        self.calls.append(("window", timeout_seconds))
        return iter(self._windows.popleft())


class FakeVoskRecognizer:
    def __init__(self, *, result: str = "", final_result: str = "", partial: str = "") -> None:
        self._result = result
        self._final_result = final_result
        self._partial = partial
        self.audio = bytearray()

    def AcceptWaveform(self, data: bytes) -> bool:
        self.audio.extend(data)
        return bool(self._result)

    def Result(self) -> str:
        return json.dumps({"text": self._result})

    def FinalResult(self) -> str:
        return json.dumps({"text": self._final_result})

    def PartialResult(self) -> str:
        return json.dumps({"partial": self._partial})


class SequencedFactory:
    def __init__(self, recognizers: list[FakeVoskRecognizer]) -> None:
        self._recognizers = deque(recognizers)
        self.grammars: list[list[str]] = []

    def __call__(self, _model, sample_rate: int, grammar: str) -> FakeVoskRecognizer:
        assert sample_rate == 16_000
        self.grammars.append(json.loads(grammar))
        return self._recognizers.popleft()


def test_six_microphone_window_uses_restricted_grammar_and_returns_one_window_result() -> None:
    stream = FakeWindowStream([[bytes(range(16))]])
    factory = SequencedFactory(
        [FakeVoskRecognizer(result="blue star"), *(FakeVoskRecognizer() for _ in range(5))]
    )
    recognizer = R818StreamingVoskSession(
        stream,
        VoiceConfig(vote_guard_seconds=0.0),
        model=object(),
        recognizer_factory=factory,
    )

    result = recognizer.recognize(1, 20.0)

    assert isinstance(result, VoiceWindowResult)
    assert result.accepted is True
    assert result.attempt_number == 1
    assert result.matching_channels == (0,)
    assert factory.grammars == [["blue star", "[unk]"]] * 6


def test_six_microphone_window_does_not_accept_partial_text() -> None:
    stream = FakeWindowStream([[bytes(range(16))]])
    factory = SequencedFactory([FakeVoskRecognizer(partial="blue star") for _ in range(6)])
    recognizer = R818StreamingVoskSession(
        stream,
        VoiceConfig(vote_guard_seconds=0.0),
        model=object(),
        recognizer_factory=factory,
    )

    result = recognizer.recognize(1, 20.0)

    assert result.accepted is False
    assert result.matching_channels == ()


def test_six_microphone_window_waits_for_the_configured_vote_guard() -> None:
    frame = bytes(range(16)) * 160
    stream = FakeWindowStream([[frame, frame]])
    factory = SequencedFactory(
        [FakeVoskRecognizer(result="blue star"), *(FakeVoskRecognizer() for _ in range(5))]
    )
    recognizer = R818StreamingVoskSession(
        stream,
        VoiceConfig(vote_guard_seconds=0.005),
        model=object(),
        recognizer_factory=factory,
    )

    result = recognizer.recognize(1, 20.0)

    assert result.accepted is True
    assert result.decision_time_seconds == pytest.approx(0.015)


def test_six_microphone_window_rejects_incomplete_origin_frame_as_technical_error() -> None:
    stream = FakeWindowStream([[b"\x00" * 15]])
    factory = SequencedFactory([FakeVoskRecognizer() for _ in range(6)])
    recognizer = R818StreamingVoskSession(
        stream,
        VoiceConfig(vote_guard_seconds=0.0),
        model=object(),
        recognizer_factory=factory,
    )

    with pytest.raises(R818HardwareUnreadyError, match="16-byte"):
        recognizer.recognize(1, 20.0)
