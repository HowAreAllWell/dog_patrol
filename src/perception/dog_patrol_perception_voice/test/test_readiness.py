from __future__ import annotations

import threading
import time

from dog_patrol_perception_voice.readiness import (
    ERROR,
    NOT_READY,
    READY,
    VoicePreflightOutcome,
    VoiceReadinessController,
)


def _wait_for(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_current_startup_sequence_publishes_one_matching_preflight_result() -> None:
    published = []
    controller = VoiceReadinessController(
        lambda: VoicePreflightOutcome(READY, "voice preflight ready"), published.append
    )
    try:
        controller.observe(17, 0)
        assert _wait_for(lambda: len(published) == 1)
        controller.observe(17, 0)
        time.sleep(0.05)

        assert published == [VoicePreflightOutcome(READY, "voice preflight ready", 17)]
    finally:
        controller.stop()


def test_old_preflight_result_cannot_publish_after_startup_sequence_changes() -> None:
    first_started = threading.Event()
    release_first = threading.Event()
    calls = []

    def preflight():
        calls.append(len(calls) + 1)
        if len(calls) == 1:
            first_started.set()
            assert release_first.wait(timeout=2.0)
        return VoicePreflightOutcome(NOT_READY, "voice model is unavailable")

    published = []
    controller = VoiceReadinessController(preflight, published.append)
    try:
        controller.observe(21, 0)
        assert first_started.wait(timeout=1.0)
        controller.observe(22, 0)
        release_first.set()

        assert _wait_for(lambda: len(published) == 1)
        assert published[0] == VoicePreflightOutcome(
            NOT_READY, "voice model is unavailable", 22
        )
        assert calls == [1, 2]
    finally:
        release_first.set()
        controller.stop()


def test_non_startup_state_does_not_run_or_publish_preflight() -> None:
    calls = []
    published = []
    controller = VoiceReadinessController(
        lambda: calls.append(True) or VoicePreflightOutcome(ERROR, "unexpected"),
        published.append,
    )
    try:
        controller.observe(31, 1)
        time.sleep(0.05)
        assert calls == []
        assert published == []
    finally:
        controller.stop()


def test_zero_startup_sequence_is_not_a_publishable_mission_sequence() -> None:
    calls = []
    published = []
    controller = VoiceReadinessController(
        lambda: calls.append(True) or VoicePreflightOutcome(READY, "unexpected"),
        published.append,
    )
    try:
        controller.observe(0, 0)
        time.sleep(0.05)
        assert calls == []
        assert published == []
    finally:
        controller.stop()
