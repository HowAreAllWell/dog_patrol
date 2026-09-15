from __future__ import annotations

import threading
import time

from dog_patrol_perception_face.preflight import READY, FacePreflightOutcome
from dog_patrol_perception_face.readiness import FaceReadinessController


def test_leaving_startup_for_ready_suppresses_in_flight_preflight_result() -> None:
    started = threading.Event()
    release = threading.Event()
    published = []

    def preflight():
        started.set()
        assert release.wait(timeout=2.0)
        return FacePreflightOutcome(READY, "late startup result")

    controller = FaceReadinessController(preflight, published.append)
    try:
        controller.observe(41, 0)
        assert started.wait(timeout=1.0)
        controller.observe(42, 7)
        release.set()
        time.sleep(0.1)
        assert published == []
    finally:
        release.set()
        controller.stop()


def test_late_old_startup_snapshot_cannot_restart_preflight_after_ready() -> None:
    calls = []
    published = []
    controller = FaceReadinessController(
        lambda: calls.append(True) or FacePreflightOutcome(READY, "unexpected"),
        published.append,
    )
    try:
        controller.observe(42, 7)
        controller.observe(41, 0)
        time.sleep(0.1)

        assert calls == []
        assert published == []
    finally:
        controller.stop()


def test_heartbeat_expiry_suppresses_in_flight_preflight_result() -> None:
    started = threading.Event()
    release = threading.Event()
    published = []

    def preflight():
        started.set()
        assert release.wait(timeout=2.0)
        return FacePreflightOutcome(READY, "expired startup result")

    controller = FaceReadinessController(preflight, published.append)
    try:
        controller.observe(50, 0)
        assert started.wait(timeout=1.0)
        controller.expire()
        release.set()
        time.sleep(0.1)
        assert published == []
    finally:
        release.set()
        controller.stop()
