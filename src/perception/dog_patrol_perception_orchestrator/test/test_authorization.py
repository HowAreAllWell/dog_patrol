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


@pytest.mark.parametrize("count", [1, 3, 5])
def test_configured_round_count_exhausts_without_requiring_extra_face_windows(session, count):
    coordinator = AuthorizationCoordinator(voice_prompt_count=count)
    coordinator.observe(session)
    transition = coordinator.record_observation(AuthorizationObservation(
        session, AuthorizationStage.INITIAL_FACE,
        AuthorizationProvider.FACE, AuthorizationResult.NOT_PASSED,
    ))
    for number in range(1, count + 1):
        command, = transition.commands
        assert command.round_number == number
        stage = AuthorizationStage.DUAL_FIRST if number == 1 else AuthorizationStage.DUAL_SECOND
        if number > 2:
            stale = coordinator.record_observation(AuthorizationObservation(
                session, stage, AuthorizationProvider.VOICE, AuthorizationResult.PASSED,
                number - 1,
            ))
            assert stale == AuthorizationTransition()
        transition = coordinator.record_observation(AuthorizationObservation(
            session, stage, AuthorizationProvider.VOICE, AuthorizationResult.NOT_PASSED, number,
        ))
    assert transition.outcome == AuthorizationOutcome(session, AuthorizationResult.NOT_PASSED)
    assert coordinator.active_session is None


def test_count_update_applies_to_next_session(session):
    coordinator = AuthorizationCoordinator(voice_prompt_count=1)
    coordinator.observe(session)
    coordinator.set_voice_prompt_count(3)
    # Heartbeat recovery keeps the same mission identity and policy snapshot.
    coordinator.observe(None)
    coordinator.observe(session)
    coordinator.record_observation(AuthorizationObservation(
        session, AuthorizationStage.INITIAL_FACE,
        AuthorizationProvider.FACE, AuthorizationResult.NOT_PASSED,
    ))
    for provider in (AuthorizationProvider.FACE, AuthorizationProvider.VOICE):
        transition = coordinator.record_observation(AuthorizationObservation(
            session, AuthorizationStage.DUAL_FIRST, provider, AuthorizationResult.NOT_PASSED,
        ))
    assert transition.outcome is not None
    next_session = AuthorizationSession(18, 43)
    coordinator.observe(next_session)
    coordinator.record_observation(AuthorizationObservation(
        next_session, AuthorizationStage.INITIAL_FACE,
        AuthorizationProvider.FACE, AuthorizationResult.NOT_PASSED,
    ))
    for provider in (AuthorizationProvider.FACE, AuthorizationProvider.VOICE):
        transition = coordinator.record_observation(AuthorizationObservation(
            next_session, AuthorizationStage.DUAL_FIRST, provider, AuthorizationResult.NOT_PASSED,
        ))
    assert transition.outcome is None
    assert transition.commands[0].round_number == 2


@pytest.mark.parametrize("value", [0, -1, True, 2.5, 1 << 32])
def test_invalid_prompt_count_is_rejected(value):
    with pytest.raises(ValueError, match="voice_prompt_count"):
        AuthorizationCoordinator(voice_prompt_count=value)


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


def test_joint_face_miss_does_not_advance_voice_round(session):
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


@pytest.mark.parametrize("round_number", [2, 3, 5])
@pytest.mark.parametrize("result", [AuthorizationResult.PASSED, AuthorizationResult.ERROR])
def test_continuous_face_result_is_valid_in_later_voice_rounds(session, round_number, result):
    coordinator = AuthorizationCoordinator(voice_prompt_count=5)
    coordinator.observe(session)
    coordinator.record_observation(AuthorizationObservation(
        session, AuthorizationStage.INITIAL_FACE,
        AuthorizationProvider.FACE, AuthorizationResult.NOT_PASSED,
    ))
    for number in range(1, round_number):
        coordinator.record_observation(AuthorizationObservation(
            session,
            AuthorizationStage.DUAL_FIRST if number == 1 else AuthorizationStage.DUAL_SECOND,
            AuthorizationProvider.VOICE, AuthorizationResult.NOT_PASSED, number,
        ))
    # The face provider retains its original joint-stage identity throughout.
    transition = coordinator.record_observation(AuthorizationObservation(
        session, AuthorizationStage.DUAL_FIRST, AuthorizationProvider.FACE, result,
    ))
    assert transition.outcome == AuthorizationOutcome(session, result)
    assert transition.commands == (AuthorizationCommand(session, AuthorizationStage.CANCEL),)


@pytest.mark.parametrize("result", [AuthorizationResult.PASSED, AuthorizationResult.ERROR])
def test_joint_stage_still_rejects_stale_face_and_voice_results(session, result):
    coordinator = _coordinator_at_stage(session, AuthorizationStage.DUAL_SECOND)
    for observation in (
        AuthorizationObservation(session, AuthorizationStage.INITIAL_FACE, AuthorizationProvider.FACE, result),
        AuthorizationObservation(AuthorizationSession(16, 42), AuthorizationStage.DUAL_FIRST, AuthorizationProvider.FACE, result),
        AuthorizationObservation(session, AuthorizationStage.DUAL_FIRST, AuthorizationProvider.VOICE, result, 1),
    ):
        assert coordinator.record_observation(observation) == AuthorizationTransition()
    coordinator.observe(None)
    assert coordinator.record_observation(AuthorizationObservation(
        session, AuthorizationStage.DUAL_FIRST, AuthorizationProvider.FACE, result,
    )) == AuthorizationTransition()


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
