import pytest

from dog_patrol_perception_orchestrator.authorization import (
    AuthorizationCommand,
    AuthorizationCoordinator,
    AuthorizationObservation,
    AuthorizationOutcome,
    AuthorizationProvider,
    AuthorizationResult,
    AuthorizationSession,
    AuthorizationStage,
    AuthorizationTransition,
)


@pytest.fixture
def session():
    return AuthorizationSession(observed_state_seq=17, target_id=42)


def test_new_session_starts_with_face_only(session):
    coordinator = AuthorizationCoordinator()

    transition = coordinator.observe(session)

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.INITIAL_FACE),
    )
    assert transition.outcome is None


def test_initial_face_miss_starts_first_dual_round(session):
    coordinator = AuthorizationCoordinator()
    coordinator.observe(session)

    transition = coordinator.record_observation(
        AuthorizationObservation(
            session,
            AuthorizationStage.INITIAL_FACE,
            AuthorizationProvider.FACE,
            AuthorizationResult.NOT_PASSED,
        )
    )

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.DUAL_FIRST),
    )
    assert transition.outcome is None


def test_dual_round_waits_for_both_providers(session):
    coordinator = AuthorizationCoordinator()
    coordinator.observe(session)
    coordinator.record_observation(
        AuthorizationObservation(
            session,
            AuthorizationStage.INITIAL_FACE,
            AuthorizationProvider.FACE,
            AuthorizationResult.NOT_PASSED,
        )
    )

    transition = coordinator.record_observation(
        AuthorizationObservation(
            session,
            AuthorizationStage.DUAL_FIRST,
            AuthorizationProvider.FACE,
            AuthorizationResult.NOT_PASSED,
        )
    )

    assert transition.commands == ()
    assert transition.outcome is None
    assert coordinator.active_session == session


def test_first_dual_round_misses_start_second_dual_round(session):
    coordinator = AuthorizationCoordinator()
    coordinator.observe(session)
    coordinator.record_observation(
        AuthorizationObservation(
            session,
            AuthorizationStage.INITIAL_FACE,
            AuthorizationProvider.FACE,
            AuthorizationResult.NOT_PASSED,
        )
    )
    for provider in (AuthorizationProvider.FACE, AuthorizationProvider.VOICE):
        transition = coordinator.record_observation(
            AuthorizationObservation(
                session,
                AuthorizationStage.DUAL_FIRST,
                provider,
                AuthorizationResult.NOT_PASSED,
            )
        )

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.DUAL_SECOND),
    )
    assert transition.outcome is None


def test_second_dual_round_misses_complete_as_not_passed(session):
    coordinator = AuthorizationCoordinator()
    coordinator.observe(session)
    coordinator.record_observation(
        AuthorizationObservation(
            session,
            AuthorizationStage.INITIAL_FACE,
            AuthorizationProvider.FACE,
            AuthorizationResult.NOT_PASSED,
        )
    )
    for stage in (AuthorizationStage.DUAL_FIRST, AuthorizationStage.DUAL_SECOND):
        for provider in (AuthorizationProvider.FACE, AuthorizationProvider.VOICE):
            transition = coordinator.record_observation(
                AuthorizationObservation(
                    session,
                    stage,
                    provider,
                    AuthorizationResult.NOT_PASSED,
                )
            )

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.CANCEL),
    )
    assert transition.outcome == AuthorizationOutcome(
        session, AuthorizationResult.NOT_PASSED
    )
    assert coordinator.active_session is None


@pytest.mark.parametrize(
    ("stage", "provider"),
    [
        (AuthorizationStage.INITIAL_FACE, AuthorizationProvider.FACE),
        (AuthorizationStage.DUAL_FIRST, AuthorizationProvider.FACE),
        (AuthorizationStage.DUAL_FIRST, AuthorizationProvider.VOICE),
        (AuthorizationStage.DUAL_SECOND, AuthorizationProvider.FACE),
        (AuthorizationStage.DUAL_SECOND, AuthorizationProvider.VOICE),
    ],
)
def test_any_expected_pass_completes_as_authorized(session, stage, provider):
    coordinator = _coordinator_at_stage(session, stage)

    transition = coordinator.record_observation(
        AuthorizationObservation(
            session, stage, provider, AuthorizationResult.PASSED
        )
    )

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.CANCEL),
    )
    assert transition.outcome == AuthorizationOutcome(
        session, AuthorizationResult.PASSED
    )


@pytest.mark.parametrize(
    ("stage", "provider"),
    [
        (AuthorizationStage.INITIAL_FACE, AuthorizationProvider.FACE),
        (AuthorizationStage.DUAL_FIRST, AuthorizationProvider.FACE),
        (AuthorizationStage.DUAL_FIRST, AuthorizationProvider.VOICE),
        (AuthorizationStage.DUAL_SECOND, AuthorizationProvider.FACE),
        (AuthorizationStage.DUAL_SECOND, AuthorizationProvider.VOICE),
    ],
)
def test_any_expected_error_completes_as_error(session, stage, provider):
    coordinator = _coordinator_at_stage(session, stage)

    transition = coordinator.record_observation(
        AuthorizationObservation(
            session, stage, provider, AuthorizationResult.ERROR
        )
    )

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.CANCEL),
    )
    assert transition.outcome == AuthorizationOutcome(
        session, AuthorizationResult.ERROR
    )


def test_replacing_session_cancels_old_before_starting_new(session):
    coordinator = AuthorizationCoordinator()
    replacement = AuthorizationSession(observed_state_seq=18, target_id=43)
    coordinator.observe(session)

    transition = coordinator.observe(replacement)

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.CANCEL),
        AuthorizationCommand(replacement, AuthorizationStage.INITIAL_FACE),
    )
    assert coordinator.active_session == replacement


def test_observing_no_session_cancels_active_session(session):
    coordinator = AuthorizationCoordinator()
    coordinator.observe(session)

    transition = coordinator.observe(None)

    assert transition.commands == (
        AuthorizationCommand(session, AuthorizationStage.CANCEL),
    )
    assert transition.outcome is None
    assert coordinator.active_session is None


@pytest.mark.parametrize(
    "observation",
    [
        AuthorizationObservation(
            AuthorizationSession(17, 42),
            AuthorizationStage.INITIAL_FACE,
            AuthorizationProvider.VOICE,
            AuthorizationResult.PASSED,
        ),
        AuthorizationObservation(
            AuthorizationSession(16, 42),
            AuthorizationStage.INITIAL_FACE,
            AuthorizationProvider.FACE,
            AuthorizationResult.PASSED,
        ),
        AuthorizationObservation(
            AuthorizationSession(17, 42),
            AuthorizationStage.DUAL_FIRST,
            AuthorizationProvider.FACE,
            AuthorizationResult.PASSED,
        ),
    ],
)
def test_unexpected_observation_is_ignored(session, observation):
    coordinator = AuthorizationCoordinator()
    coordinator.observe(session)

    assert coordinator.record_observation(observation) == AuthorizationTransition()
    assert coordinator.active_session == session


def _coordinator_at_stage(session, stage):
    coordinator = AuthorizationCoordinator()
    coordinator.observe(session)
    if stage is AuthorizationStage.INITIAL_FACE:
        return coordinator
    coordinator.record_observation(
        AuthorizationObservation(
            session,
            AuthorizationStage.INITIAL_FACE,
            AuthorizationProvider.FACE,
            AuthorizationResult.NOT_PASSED,
        )
    )
    if stage is AuthorizationStage.DUAL_FIRST:
        return coordinator
    for provider in (AuthorizationProvider.FACE, AuthorizationProvider.VOICE):
        coordinator.record_observation(
            AuthorizationObservation(
                session,
                AuthorizationStage.DUAL_FIRST,
                provider,
                AuthorizationResult.NOT_PASSED,
            )
        )
    return coordinator
