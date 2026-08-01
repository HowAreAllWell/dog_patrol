from dog_patrol_perception.authorization import (
    AuthorizationCoordinator,
    AuthorizationOutcome,
    AuthorizationResult,
    AuthorizationSession,
)


def test_two_valid_not_passed_results_complete_as_not_passed():
    coordinator = AuthorizationCoordinator(required_not_passed=2)
    session = AuthorizationSession(observed_state_seq=17, target_id=42)
    coordinator.start(session)

    assert coordinator.record(session, AuthorizationResult.NOT_PASSED) is None
    assert coordinator.record(
        session, AuthorizationResult.NOT_PASSED
    ) == AuthorizationOutcome(session, AuthorizationResult.NOT_PASSED)
    assert coordinator.active_session is None


def test_passed_result_completes_current_session_immediately():
    coordinator = AuthorizationCoordinator(required_not_passed=2)
    session = AuthorizationSession(observed_state_seq=18, target_id=43)
    coordinator.start(session)

    assert coordinator.record(
        session, AuthorizationResult.PASSED
    ) == AuthorizationOutcome(session, AuthorizationResult.PASSED)
    assert coordinator.active_session is None


def test_error_completes_as_error_without_becoming_not_passed():
    coordinator = AuthorizationCoordinator(required_not_passed=2)
    session = AuthorizationSession(observed_state_seq=19, target_id=44)
    coordinator.start(session)

    assert coordinator.record(
        session, AuthorizationResult.ERROR
    ) == AuthorizationOutcome(session, AuthorizationResult.ERROR)
    assert coordinator.record(session, AuthorizationResult.NOT_PASSED) is None


def test_cancelled_completes_without_an_authorization_decision():
    coordinator = AuthorizationCoordinator(required_not_passed=2)
    session = AuthorizationSession(observed_state_seq=20, target_id=45)
    coordinator.start(session)

    assert coordinator.record(
        session, AuthorizationResult.CANCELLED
    ) == AuthorizationOutcome(session, AuthorizationResult.CANCELLED)
    assert coordinator.active_session is None


def test_stale_result_cannot_complete_replacement_session():
    coordinator = AuthorizationCoordinator(required_not_passed=2)
    stale_session = AuthorizationSession(observed_state_seq=20, target_id=45)
    current_session = AuthorizationSession(observed_state_seq=21, target_id=46)
    coordinator.start(stale_session)
    coordinator.start(current_session)

    assert (
        coordinator.record(stale_session, AuthorizationResult.PASSED) is None
    )
    assert coordinator.active_session == current_session
    assert coordinator.record(
        current_session, AuthorizationResult.PASSED
    ) == AuthorizationOutcome(current_session, AuthorizationResult.PASSED)
