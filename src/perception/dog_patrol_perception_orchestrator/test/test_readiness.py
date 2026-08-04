import pytest

from dog_patrol_perception_orchestrator.readiness import (
    ERROR,
    NOT_READY,
    READY,
    CapabilitySample,
    ReadinessCoordinator,
)


STARTUP = 0
PATROL = 1
REQUIRED = ("detection_tracking", "face", "voice")


def sample(capability, status=READY, seq=10):
    return CapabilitySample(capability, status, seq)


def test_emits_once_only_after_all_required_capabilities_match_startup():
    coordinator = ReadinessCoordinator(REQUIRED)
    assert coordinator.observe_mission(10, STARTUP, STARTUP) is None
    assert coordinator.observe_capability(sample("detection_tracking"), STARTUP) is None
    assert coordinator.observe_capability(sample("face"), STARTUP) is None
    assert coordinator.observe_capability(sample("voice"), STARTUP) == 10
    assert coordinator.observe_capability(sample("voice"), STARTUP) is None


@pytest.mark.parametrize("status", [NOT_READY, ERROR])
def test_not_ready_and_error_block_until_current_status_recovers(status):
    coordinator = ReadinessCoordinator(REQUIRED)
    coordinator.observe_mission(10, STARTUP, STARTUP)
    coordinator.observe_capability(sample("detection_tracking"), STARTUP)
    coordinator.observe_capability(sample("face", status), STARTUP)
    assert coordinator.observe_capability(sample("voice"), STARTUP) is None
    assert coordinator.observe_capability(sample("face", READY), STARTUP) == 10


def test_non_startup_old_wrong_and_unknown_statuses_do_not_emit():
    coordinator = ReadinessCoordinator(REQUIRED)
    coordinator.observe_mission(10, PATROL, STARTUP)
    for capability in REQUIRED:
        assert coordinator.observe_capability(sample(capability), STARTUP) is None
    assert coordinator.observe_mission(9, STARTUP, STARTUP) is None
    assert coordinator.observe_mission(11, STARTUP, STARTUP) is None
    assert coordinator.observe_capability(sample("unknown", READY, 11), STARTUP) is None
    assert coordinator.observe_capability(sample("face", 99, 11), STARTUP) is None


def test_statuses_received_before_mission_support_late_orchestrator_ordering():
    coordinator = ReadinessCoordinator(REQUIRED)
    for capability in REQUIRED:
        assert coordinator.observe_capability(sample(capability, seq=23), STARTUP) is None
    assert coordinator.observe_mission(23, STARTUP, STARTUP) == 23


def test_stale_capability_cannot_overwrite_newer_status():
    coordinator = ReadinessCoordinator(REQUIRED)
    coordinator.observe_capability(sample("face", READY, 31), STARTUP)
    coordinator.observe_capability(sample("face", ERROR, 30), STARTUP)
    coordinator.observe_capability(sample("detection_tracking", READY, 31), STARTUP)
    coordinator.observe_capability(sample("voice", READY, 31), STARTUP)
    assert coordinator.observe_mission(31, STARTUP, STARTUP) == 31


def test_required_capabilities_are_fixed_and_validated():
    coordinator = ReadinessCoordinator(REQUIRED)
    assert coordinator.required_capabilities == REQUIRED
    with pytest.raises(ValueError):
        ReadinessCoordinator(("face", "face"))
