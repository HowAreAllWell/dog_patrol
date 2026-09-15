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
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    assert coordinator.observe_mission(10, STARTUP) is None
    assert coordinator.observe_capability(sample("detection_tracking")) is None
    assert coordinator.observe_capability(sample("face")) is None
    assert coordinator.observe_capability(sample("voice")) == 10
    assert coordinator.observe_capability(sample("voice")) is None


@pytest.mark.parametrize("status", [NOT_READY, ERROR])
def test_not_ready_and_error_block_until_current_status_recovers(status):
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    coordinator.observe_mission(10, STARTUP)
    coordinator.observe_capability(sample("detection_tracking"))
    coordinator.observe_capability(sample("face", status))
    assert coordinator.observe_capability(sample("voice")) is None
    assert coordinator.observe_capability(sample("face", READY)) == 10


def test_non_startup_old_wrong_and_unknown_statuses_do_not_emit():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    coordinator.observe_mission(10, PATROL)
    for capability in REQUIRED:
        assert coordinator.observe_capability(sample(capability)) is None
    assert coordinator.observe_mission(9, STARTUP) is None
    assert coordinator.observe_mission(11, STARTUP) is None
    assert coordinator.observe_capability(sample("unknown", READY, 11)) is None
    assert coordinator.observe_capability(sample("face", 99, 11)) is None


def test_statuses_received_before_mission_support_late_orchestrator_ordering():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    for capability in REQUIRED:
        assert coordinator.observe_capability(sample(capability, seq=23)) is None
    assert coordinator.observe_mission(23, STARTUP) == 23


def test_stale_capability_cannot_overwrite_newer_status():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    coordinator.observe_capability(sample("face", READY, 31))
    coordinator.observe_capability(sample("face", ERROR, 30))
    coordinator.observe_capability(sample("detection_tracking", READY, 31))
    coordinator.observe_capability(sample("voice", READY, 31))
    assert coordinator.observe_mission(31, STARTUP) == 31


def test_required_capabilities_are_fixed_and_validated():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    assert coordinator.required_capabilities == REQUIRED
    with pytest.raises(ValueError):
        ReadinessCoordinator(("face", "face"), startup_state=STARTUP)


def test_zero_state_sequence_cannot_form_readiness():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    assert coordinator.observe_mission(0, STARTUP) is None
    for capability in REQUIRED:
        assert coordinator.observe_capability(sample(capability, READY, 0)) is None


def test_same_sequence_capability_status_progression_is_allowed():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    coordinator.observe_mission(41, STARTUP)
    coordinator.observe_capability(sample("detection_tracking", READY, 41))
    coordinator.observe_capability(sample("face", READY, 41))
    coordinator.observe_capability(sample("voice", NOT_READY, 41))
    assert coordinator.observe_capability(sample("voice", READY, 41)) == 41


def test_same_sequence_mission_conflict_fails_closed_until_new_sequence():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    coordinator.observe_mission(41, STARTUP)
    for capability in REQUIRED:
        coordinator.observe_capability(sample(capability, READY, 41))
    assert coordinator.observe_mission(41, PATROL) is None
    assert coordinator.observe_mission(41, STARTUP) is None
    for capability in REQUIRED:
        coordinator.observe_capability(sample(capability, READY, 42))
    assert coordinator.observe_mission(42, STARTUP) == 42


def test_mission_heartbeat_expiry_blocks_late_capability_until_state_recovers():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    coordinator.observe_mission(50, STARTUP)
    coordinator.observe_capability(sample("detection_tracking", READY, 50))
    coordinator.observe_capability(sample("face", READY, 50))
    coordinator.expire_mission()

    assert coordinator.observe_capability(sample("voice", READY, 50)) is None
    assert coordinator.observe_mission(50, STARTUP) == 50


def test_same_sequence_startup_target_conflict_cannot_emit_ready():
    coordinator = ReadinessCoordinator(REQUIRED, startup_state=STARTUP)
    coordinator.observe_mission(60, STARTUP, 0, 0)
    coordinator.observe_mission(60, STARTUP, 1, 0)
    for capability in REQUIRED:
        coordinator.observe_capability(sample(capability, READY, 60))
    assert coordinator.observe_mission(60, STARTUP, 0, 0) is None
