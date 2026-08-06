"""ROS-independent sequencing for voice capability readiness."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
import threading

from .preflight import (
    ERROR,
    NOT_READY,
    READY,
    VoicePreflightOutcome,
)


class VoiceReadinessController:
    """Run one preflight at a time and publish only its current STARTUP result."""

    def __init__(
        self,
        preflight: Callable[[], VoicePreflightOutcome],
        publish: Callable[[VoicePreflightOutcome], None],
        *,
        startup_state: int = 0,
    ) -> None:
        self._preflight = preflight
        self._publish = publish
        self._startup_state = int(startup_state)
        self._condition = threading.Condition()
        self._desired: tuple[int, int] | None = None
        self._generation = 0
        self._completed_generation = 0
        self._stopping = False
        self._worker = threading.Thread(
            target=self._run,
            name="dog-patrol-voice-readiness",
            daemon=True,
        )
        self._worker.start()

    def observe(self, state_seq: int, state: int) -> None:
        """Observe the latest MissionState without running preflight inline."""
        normalized = (int(state_seq), int(state))
        with self._condition:
            if normalized == self._desired:
                return
            self._desired = (
                normalized
                if normalized[0] > 0 and normalized[1] == self._startup_state
                else None
            )
            self._generation += 1
            self._condition.notify_all()

    def stop(self) -> None:
        with self._condition:
            if self._stopping:
                return
            self._stopping = True
            self._condition.notify_all()
        self._worker.join()

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._stopping and (
                    self._desired is None
                    or self._completed_generation == self._generation
                ):
                    self._condition.wait()
                if self._stopping:
                    return
                desired = self._desired
                generation = self._generation
                assert desired is not None

            try:
                outcome = self._preflight()
            except Exception as exc:
                outcome = VoicePreflightOutcome(ERROR, _exception_detail(exc))

            with self._condition:
                if (
                    not self._stopping
                    and self._generation == generation
                    and self._desired == desired
                ):
                    self._publish(replace(outcome, startup_state_seq=desired[0]))
                    self._completed_generation = generation
                self._condition.notify_all()


def _exception_detail(exc: Exception) -> str:
    detail = str(exc).strip()
    return f"{type(exc).__name__}: {detail}" if detail else type(exc).__name__


__all__ = [
    "ERROR",
    "NOT_READY",
    "READY",
    "VoicePreflightOutcome",
    "VoiceReadinessController",
]
