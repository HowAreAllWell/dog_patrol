import pytest

from dog_patrol_perception_orchestrator.authorization import (
    AuthorizationCoordinator,
    AuthorizationOutcome,
    AuthorizationResult,
    AuthorizationSession,
)


@pytest.fixture
def session():
    return AuthorizationSession(observed_state_seq=17, target_id=42)


def test_two_not_passed_results_complete_as_not_passed(session):
    coordinator = AuthorizationCoordinator()
    coordinator.start(session)

    assert coordinator.record(session, AuthorizationResult.NOT_PASSED) is None
    assert coordinator.record(
        session, AuthorizationResult.NOT_PASSED
    ) == AuthorizationOutcome(session, AuthorizationResult.NOT_PASSED)
    assert coordinator.active_session is None


@pytest.mark.parametrize(
    "result",
    [
        AuthorizationResult.PASSED,
        AuthorizationResult.ERROR,
        AuthorizationResult.CANCELLED,
    ],
)
def test_terminal_result_completes_current_session_immediately(
    session, result
):
    coordinator = AuthorizationCoordinator()
    coordinator.start(session)

    assert coordinator.record(session, result) == AuthorizationOutcome(
        session, result
    )
    assert coordinator.active_session is None


def test_error_does_not_become_not_passed(session):
    coordinator = AuthorizationCoordinator()
    coordinator.start(session)

    assert coordinator.record(
        session, AuthorizationResult.ERROR
    ) == AuthorizationOutcome(session, AuthorizationResult.ERROR)
    assert coordinator.record(session, AuthorizationResult.NOT_PASSED) is None


def test_starting_replacement_session_rejects_stale_results(session):
    coordinator = AuthorizationCoordinator()
    replacement = AuthorizationSession(observed_state_seq=18, target_id=43)
    coordinator.start(session)
    coordinator.start(replacement)

    assert coordinator.record(session, AuthorizationResult.PASSED) is None
    assert coordinator.active_session == replacement
    assert coordinator.record(
        replacement, AuthorizationResult.PASSED
    ) == AuthorizationOutcome(replacement, AuthorizationResult.PASSED)


def test_results_are_rejected_after_session_completes(session):
    coordinator = AuthorizationCoordinator()
    coordinator.start(session)
    coordinator.record(session, AuthorizationResult.CANCELLED)

    assert coordinator.record(session, AuthorizationResult.PASSED) is None


def test_start_resets_not_passed_progress(session):
    coordinator = AuthorizationCoordinator()
    replacement = AuthorizationSession(observed_state_seq=18, target_id=42)
    coordinator.start(session)
    assert coordinator.record(session, AuthorizationResult.NOT_PASSED) is None

    coordinator.start(replacement)

    assert (
        coordinator.record(replacement, AuthorizationResult.NOT_PASSED) is None
    )
    assert coordinator.active_session == replacement
