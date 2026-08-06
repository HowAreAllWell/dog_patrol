"""Task-scoped R818 Base64 transport and six-microphone response windows."""

from __future__ import annotations

import base64
import binascii
import json
import math
import subprocess
import threading
import time
from collections.abc import Iterable, Iterator
from pathlib import Path
from typing import Any, Callable, Protocol

import numpy as np

from .config import VoiceConfig
from .result import VoiceWindowResult

R818_SAMPLE_RATE = 16_000
R818_ORIGIN_CHANNELS = 8
R818_MIC_CHANNELS = 6
R818_ORIGIN_FRAME_BYTES = R818_ORIGIN_CHANNELS * 2


class R818HardwareUnreadyError(RuntimeError):
    """The R818 stream cannot provide trustworthy response-window audio."""


class AdbStreamTransport(Protocol):
    def ensure_ready(self) -> None: ...

    def push(self, local_path: Path, remote_path: str) -> None: ...

    def shell(
        self,
        arguments: tuple[str, ...],
        *,
        timeout_seconds: float | None = None,
        allow_failure: bool = False,
    ) -> object: ...

    def shell_script(self, script: str) -> object: ...

    def start_shell_script(self, script: str) -> subprocess.Popen[bytes]: ...


def decode_base64_pcm_chunks(encoded_chunks: Iterable[bytes]) -> Iterator[bytes]:
    """Decode newline-free Base64 while preserving complete 16-byte frames.

    The remote helper emits an unpadded continuous stream until it exits. The
    decoder therefore keeps both the Base64 quartet and PCM frame remainder
    across reads. A final partial frame is a transport failure, never a silent
    recognition miss.
    """

    encoded_pending = bytearray()
    pcm_pending = bytearray()
    for encoded in encoded_chunks:
        if not isinstance(encoded, bytes):
            raise TypeError("encoded R818 chunks must be bytes")
        encoded_pending.extend(byte for byte in encoded if byte not in b" \t\r\n")
        complete_size = len(encoded_pending) - len(encoded_pending) % 4
        if complete_size == 0:
            continue
        block = bytes(encoded_pending[:complete_size])
        del encoded_pending[:complete_size]
        try:
            pcm_pending.extend(base64.b64decode(block, validate=True))
        except (binascii.Error, ValueError) as exc:
            raise R818HardwareUnreadyError(f"Base64 R818 stream is corrupt: {exc}") from exc
        aligned_size = len(pcm_pending) - len(pcm_pending) % R818_ORIGIN_FRAME_BYTES
        if aligned_size:
            yield bytes(pcm_pending[:aligned_size])
            del pcm_pending[:aligned_size]

    if encoded_pending:
        raise R818HardwareUnreadyError("Base64 R818 stream ended with an incomplete quartet")
    if pcm_pending:
        raise R818HardwareUnreadyError("R818 stream ended with an incomplete 16-byte frame")


class InterleavedPcmTaskStream(Protocol):
    """Own the R818 audio device for one complete verification task."""

    def start(self) -> None: ...

    def window_chunks(self, timeout_seconds: float) -> Iterator[bytes]: ...

    def close(self) -> None: ...

    def request_stop(self) -> None: ...


_BUNDLED_HELPER = Path(__file__).with_name("assets") / "r818_pcm_base64_aarch64"
_REMOTE_HELPER = "/tmp/dog-patrol-r818-base64"
_REMOTE_WORK = "/tmp/dog-patrol-r818-stream"


class SubprocessAdbEncodedPcmStream:
    """Continuously stream eight-channel PCM while exposing bounded windows."""

    def __init__(
        self,
        adb: AdbStreamTransport,
        *,
        helper_path: str | Path = _BUNDLED_HELPER,
        start_timeout_seconds: float = 5.0,
        restore_timeout_seconds: float = 20.0,
        max_buffer_seconds: float = 4.0,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        for name, value in (
            ("start_timeout_seconds", start_timeout_seconds),
            ("restore_timeout_seconds", restore_timeout_seconds),
            ("max_buffer_seconds", max_buffer_seconds),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self._adb = adb
        self._helper_path = Path(helper_path)
        self._start_timeout_seconds = start_timeout_seconds
        self._restore_timeout_seconds = restore_timeout_seconds
        self._maximum_buffer_bytes = round(
            max_buffer_seconds * R818_SAMPLE_RATE * R818_ORIGIN_FRAME_BYTES
        )
        self._clock = clock
        self._sleep = sleep
        self._remote_shell: subprocess.Popen[bytes] | None = None
        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()
        self._window_condition = threading.Condition()
        self._stream_ready = False
        self._reader_error: BaseException | None = None
        self._window_generation = 0
        self._active_window_generation: int | None = None
        self._window_started_at: float | None = None
        self._window_deadline: float | None = None
        self._window_data: list[bytes] = []
        self._window_buffered_bytes = 0
        self._started = False
        self._remote_prepared = False
        self._vendor_stopped = False
        self._close_called = False

    def start(self) -> None:
        if self._started or self._close_called:
            raise RuntimeError("R818 PCM stream has already been used")
        try:
            if not self._helper_path.is_file():
                raise OSError(f"R818 stream helper is missing: {self._helper_path}")
            self._adb.ensure_ready()
            self._adb.push(self._helper_path, _REMOTE_HELPER)
            self._remote_prepared = True
            self._adb.shell(("chmod", "700", _REMOTE_HELPER))
            self._adb.shell(("/etc/init.d/vtn_init", "stop"))
            self._vendor_stopped = True
            self._adb.shell(("killall", "demo"), allow_failure=True)
            self._reader_stop.clear()
            self._reader_error = None
            self._stream_ready = False
            self._remote_shell = self._adb.start_shell_script(self._remote_capture_script())
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                name="dog-patrol-r818-pcm-reader",
                daemon=True,
            )
            self._reader_thread.start()
            self._wait_until_streaming()
            self._started = True
        except BaseException as exc:
            cleanup_error = self._close_resources()
            self._started = False
            if cleanup_error is not None:
                raise R818HardwareUnreadyError(
                    f"R818 PCM stream could not start: {exc}; cleanup failed: {cleanup_error}"
                ) from cleanup_error
            if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                raise
            raise R818HardwareUnreadyError(f"R818 PCM stream could not start: {exc}") from exc

    def window_chunks(self, timeout_seconds: float) -> Iterator[bytes]:
        if self._reader_stop.is_set():
            raise R818HardwareUnreadyError("R818 PCM stream stop was requested")
        if not self._started or self._remote_shell is None:
            raise RuntimeError("R818 PCM stream is not active")
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be finite and positive")
        with self._window_condition:
            if self._active_window_generation is not None:
                raise RuntimeError("R818 PCM stream already has an active response window")
            if self._reader_error is not None:
                raise R818HardwareUnreadyError(
                    f"R818 PCM stream reader failed: {self._reader_error}"
                )
            started_at = self._clock()
            deadline = started_at + timeout_seconds
            self._window_generation += 1
            generation = self._window_generation
            self._active_window_generation = generation
            self._window_started_at = started_at
            self._window_deadline = deadline
            self._window_data.clear()
            self._window_buffered_bytes = 0
        return _EncodedWindowChunks(self, generation, deadline)

    def close(self) -> None:
        if self._close_called:
            return
        self._close_called = True
        cleanup_error = self._close_resources()
        self._started = False
        if cleanup_error is not None:
            raise R818HardwareUnreadyError(
                f"R818 PCM stream cleanup failed: {cleanup_error}"
            ) from cleanup_error

    def request_stop(self) -> None:
        """Interrupt the current response window; close() still owns restoration."""
        if self._close_called:
            return
        self._reader_stop.set()
        with self._window_condition:
            self._active_window_generation = None
            self._window_started_at = None
            self._window_deadline = None
            self._window_data.clear()
            self._window_buffered_bytes = 0
            self._window_condition.notify_all()

    def _wait_until_streaming(self) -> None:
        deadline = self._clock() + self._start_timeout_seconds
        with self._window_condition:
            while (
                not self._stream_ready
                and self._reader_error is None
                and not self._reader_stop.is_set()
            ):
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise TimeoutError("R818 PCM stream did not produce audio before the start deadline")
                self._window_condition.wait(timeout=min(remaining, 0.25))
            if self._reader_stop.is_set():
                raise R818HardwareUnreadyError("R818 PCM stream stop was requested during startup")
            if self._reader_error is not None:
                raise OSError(f"R818 PCM stream reader failed during startup: {self._reader_error}")

    def _reader_loop(self) -> None:
        assert self._remote_shell is not None
        assert self._remote_shell.stdout is not None
        encoded_pending = bytearray()
        pcm_pending = bytearray()
        try:
            while not self._reader_stop.is_set():
                encoded = self._remote_shell.stdout.read(8192)
                if not encoded:
                    raise OSError(self._closed_stream_detail())
                encoded_pending.extend(byte for byte in encoded if byte not in b" \t\r\n")
                complete_size = len(encoded_pending) - len(encoded_pending) % 4
                if complete_size == 0:
                    continue
                block = bytes(encoded_pending[:complete_size])
                del encoded_pending[:complete_size]
                try:
                    pcm_pending.extend(base64.b64decode(block, validate=True))
                except (binascii.Error, ValueError) as exc:
                    raise OSError(f"Base64 R818 stream is corrupt: {exc}") from exc
                aligned_size = len(pcm_pending) - len(pcm_pending) % R818_ORIGIN_FRAME_BYTES
                if aligned_size == 0:
                    continue
                data = bytes(pcm_pending[:aligned_size])
                del pcm_pending[:aligned_size]
                received_at = self._clock()
                with self._window_condition:
                    if not self._stream_ready:
                        self._stream_ready = True
                        self._window_condition.notify_all()
                    if self._active_window_generation is None:
                        continue
                    if self._window_deadline is not None and received_at >= self._window_deadline:
                        continue
                    if self._window_buffered_bytes + len(data) > self._maximum_buffer_bytes:
                        raise OSError("R818 PCM consumer fell behind the live stream")
                    self._window_data.append(data)
                    self._window_buffered_bytes += len(data)
                    self._window_condition.notify_all()
        except BaseException as exc:
            if not self._reader_stop.is_set():
                with self._window_condition:
                    self._reader_error = exc
                    self._window_condition.notify_all()

    def _closed_stream_detail(self) -> str:
        assert self._remote_shell is not None
        status = self._remote_shell.poll()
        detail = "R818 ADB PCM stream closed"
        if status is not None:
            detail += f" with status {status}"
            if self._remote_shell.stderr is not None:
                diagnostic = self._remote_shell.stderr.read(4096).decode(
                    "utf-8", errors="replace"
                ).strip()
                if diagnostic:
                    detail += f": {diagnostic}"
        return detail

    def _next_window_data(self, generation: int, deadline: float) -> bytes:
        with self._window_condition:
            if self._active_window_generation != generation:
                raise StopIteration
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise StopIteration
            while not self._window_data and self._reader_error is None:
                self._window_condition.wait(timeout=remaining)
                if self._active_window_generation != generation:
                    raise StopIteration
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise StopIteration
            if self._reader_error is not None:
                raise R818HardwareUnreadyError(
                    f"R818 PCM stream reader failed: {self._reader_error}"
                )
            if deadline <= self._clock():
                raise StopIteration
            data = self._window_data.pop(0)
            self._window_buffered_bytes -= len(data)
            return data

    def _finish_window(self, generation: int) -> None:
        with self._window_condition:
            if self._active_window_generation != generation:
                return
            self._active_window_generation = None
            self._window_started_at = None
            self._window_deadline = None
            self._window_data.clear()
            self._window_buffered_bytes = 0

    def _close_resources(self) -> BaseException | None:
        errors: list[BaseException] = []
        self._reader_stop.set()
        if self._remote_shell is not None:
            remote_shell = self._remote_shell
            try:
                remote_shell.terminate()
                remote_shell.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                remote_shell.kill()
                remote_shell.wait()
            except OSError as exc:
                errors.append(exc)
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)
            if self._reader_thread.is_alive():
                errors.append(TimeoutError("R818 PCM reader thread did not stop"))
            self._reader_thread = None
        self._remote_shell = None
        with self._window_condition:
            self._stream_ready = False
            self._active_window_generation = None
            self._window_started_at = None
            self._window_deadline = None
            self._window_data.clear()
            self._window_buffered_bytes = 0
        if self._remote_prepared:
            try:
                self._adb.shell_script(self._remote_cleanup_script())
            except BaseException as exc:
                errors.append(exc)
            self._remote_prepared = False
        if self._vendor_stopped:
            try:
                self._restore_vendor_service()
            except BaseException as exc:
                errors.append(exc)
            self._vendor_stopped = False
        return errors[0] if errors else None

    def _restore_vendor_service(self) -> None:
        self._adb.shell(("/etc/init.d/vtn_init", "start"))
        deadline = self._clock() + self._restore_timeout_seconds
        while self._clock() < deadline:
            try:
                status = self._adb.shell(
                    ("cat", "/proc/asound/card1/pcm0c/sub0/status"),
                    timeout_seconds=2.0,
                ).stdout
            except OSError:
                status = ""
            if "state: RUNNING" in status and "owner_pid" in status:
                return
            self._sleep(0.25)
        raise TimeoutError("R818 vendor audio service did not reacquire AC107")

    @staticmethod
    def _remote_capture_script() -> str:
        return (
            f"work={_REMOTE_WORK}; fifo=$work.pcm; pid=$work.pid; err=$work.err; helper={_REMOTE_HELPER}; "
            "rm -f $fifo $pid $err; mkfifo $fifo || exit 1; "
            "cleanup() { test -f $pid && kill $(cat $pid) 2>/dev/null; rm -f $fifo $pid; }; "
            "trap cleanup EXIT INT TERM; "
            "arecord -q -D hw:1,0 -c 8 -r 16000 -f S16_LE -t raw > $fifo 2> $err & recorder=$!; "
            "$helper < $fifo 2>> $err & encoder=$!; "
            "echo $recorder $encoder > $pid; wait $encoder"
        )

    @staticmethod
    def _remote_cleanup_script() -> str:
        return (
            f"pid={_REMOTE_WORK}.pid; "
            "if test -f $pid; then processes=$(cat $pid); kill $processes 2>/dev/null; "
            "count=0; alive=1; while test $alive -eq 1 && test $count -lt 20; do alive=0; "
            "for process in $processes; do kill -0 $process 2>/dev/null && alive=1; done; "
            "test $alive -eq 1 && sleep 0.05; count=$((count + 1)); done; "
            "kill -9 $processes 2>/dev/null || true; fi; "
            f"rm -f {_REMOTE_WORK}.pcm $pid {_REMOTE_WORK}.err {_REMOTE_HELPER}"
        )


class _EncodedWindowChunks:
    def __init__(
        self,
        stream: SubprocessAdbEncodedPcmStream,
        generation: int,
        deadline: float,
    ) -> None:
        self._stream = stream
        self._generation = generation
        self._deadline = deadline
        self._closed = False

    def __iter__(self) -> _EncodedWindowChunks:
        return self

    def __next__(self) -> bytes:
        if self._closed:
            raise StopIteration
        try:
            return self._stream._next_window_data(self._generation, self._deadline)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stream._finish_window(self._generation)


class R818StreamingVoskSession:
    """Recognize one independent six-microphone response window at a time."""

    def __init__(
        self,
        stream: InterleavedPcmTaskStream,
        config: VoiceConfig,
        *,
        model: Any,
        recognizer_factory: Callable[[Any, int, str], Any],
    ) -> None:
        self._stream = stream
        self._config = config
        self._model = model
        self._recognizer_factory = recognizer_factory

    @classmethod
    def from_model_dir(
        cls,
        stream: InterleavedPcmTaskStream,
        config: VoiceConfig,
        *,
        model_dir: str | Path,
    ) -> R818StreamingVoskSession:
        from .vosk import load_vosk_model

        model, recognizer_factory = load_vosk_model(model_dir)
        return cls(
            stream,
            config,
            model=model,
            recognizer_factory=recognizer_factory,
        )

    def recognize(self, attempt_number: int, timeout_seconds: float) -> VoiceWindowResult:
        if isinstance(attempt_number, bool) or attempt_number < 1:
            raise ValueError("attempt_number must be positive")
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be finite and positive")
        chunks = iter(self._stream.window_chunks(timeout_seconds))
        try:
            grammar = json.dumps([self._config.passphrase, "[unk]"])
            channel_recognizers = tuple(
                self._recognizer_factory(self._model, R818_SAMPLE_RATE, grammar)
                for _ in range(R818_MIC_CHANNELS)
            )
        except BaseException:
            close = getattr(chunks, "close", None)
            if close is not None:
                close()
            raise

        channel_results = [""] * R818_MIC_CHANNELS
        pcm_bytes = 0
        elapsed_seconds = 0.0
        target_seen_at: float | None = None
        accepted_at: float | None = None
        try:
            for chunk in chunks:
                if len(chunk) % R818_ORIGIN_FRAME_BYTES:
                    raise R818HardwareUnreadyError(
                        "R818 PCM stream produced an incomplete 16-byte frame"
                    )
                if not chunk:
                    continue
                frames = len(chunk) // R818_ORIGIN_FRAME_BYTES
                elapsed_seconds += frames / R818_SAMPLE_RATE
                pcm_bytes += len(chunk)
                interleaved = np.frombuffer(chunk, dtype="<i2").reshape(
                    frames, R818_ORIGIN_CHANNELS
                )
                for channel, recognizer in enumerate(channel_recognizers):
                    if recognizer.AcceptWaveform(
                        interleaved[:, channel].astype("<i2", copy=False).tobytes()
                    ):
                        text = _result_text(recognizer.Result())
                        channel_results[channel] = text
                if any(text == self._config.passphrase for text in channel_results):
                    if target_seen_at is None:
                        target_seen_at = elapsed_seconds
                    if elapsed_seconds >= target_seen_at + self._config.vote_guard_seconds:
                        accepted_at = target_seen_at + self._config.vote_guard_seconds
                        break
        finally:
            close = getattr(chunks, "close", None)
            if close is not None:
                close()

        if accepted_at is None:
            for channel, recognizer in enumerate(channel_recognizers):
                text = _result_text(recognizer.FinalResult())
                if text:
                    channel_results[channel] = text
            if any(text == self._config.passphrase for text in channel_results):
                accepted_at = elapsed_seconds

        matching_channels = tuple(
            channel
            for channel, text in enumerate(channel_results)
            if text == self._config.passphrase
        )
        accepted = bool(matching_channels)
        decision_time = accepted_at if accepted_at is not None else (
            elapsed_seconds if accepted else timeout_seconds
        )
        return VoiceWindowResult(
            accepted=accepted,
            decision_time_seconds=decision_time,
            attempt_number=attempt_number,
            pcm_bytes=pcm_bytes,
            captured_duration_seconds=elapsed_seconds,
            channel_results=tuple(channel_results),
            matching_channels=matching_channels,
            vote_counts={self._config.passphrase: len(matching_channels)}
            if matching_channels
            else {},
        )


def _result_text(payload: str) -> str:
    try:
        result = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Vosk returned invalid JSON: {exc}") from exc
    if not isinstance(result, dict):
        raise RuntimeError("Vosk result must be a JSON object")
    text = result.get("text", "")
    if not isinstance(text, str):
        raise RuntimeError("Vosk result field 'text' must be a string")
    return " ".join(text.lower().split())
