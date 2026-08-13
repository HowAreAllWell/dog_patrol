from dataclasses import dataclass
from enum import Enum
from typing import Optional


class AuthorizationResult(Enum):
    PASSED = "passed"
    NOT_PASSED = "not_passed"
    ERROR = "error"
    CANCELLED = "cancelled"


class AuthorizationStage(Enum):
    INITIAL_FACE = "initial_face"
    DUAL_FIRST = "dual_first"
    DUAL_SECOND = "dual_second"
    CANCEL = "cancel"


class AuthorizationProvider(Enum):
    FACE = "face"
    VOICE = "voice"


@dataclass(frozen=True)
class AuthorizationSession:
    observed_state_seq: int
    target_id: int


@dataclass(frozen=True)
class AuthorizationOutcome:
    session: AuthorizationSession
    result: AuthorizationResult


@dataclass(frozen=True)
class AuthorizationCommand:
    session: AuthorizationSession
    stage: AuthorizationStage


@dataclass(frozen=True)
class AuthorizationObservation:
    session: AuthorizationSession
    stage: AuthorizationStage
    provider: AuthorizationProvider
    result: AuthorizationResult


@dataclass(frozen=True)
class AuthorizationTransition:
    commands: tuple[AuthorizationCommand, ...] = ()
    outcome: Optional[AuthorizationOutcome] = None


class AuthorizationCoordinator:
    """Combines verification results without exposing attempt state."""

    def __init__(self) -> None:
        self._active_session: Optional[AuthorizationSession] = None
        self._active_stage: Optional[AuthorizationStage] = None
        self._stage_results: dict[AuthorizationProvider, AuthorizationResult] = {}

    @property
    def active_session(self) -> Optional[AuthorizationSession]:
        return self._active_session

    def observe(
        self, session: Optional[AuthorizationSession]
    ) -> AuthorizationTransition:
        if session == self._active_session:
            return AuthorizationTransition()
        previous = self._active_session
        commands = []
        if previous is not None:
            commands.append(
                AuthorizationCommand(previous, AuthorizationStage.CANCEL)
            )
        self._reset()
        if session is None:
            return AuthorizationTransition(commands=tuple(commands))
        self._active_session = session
        self._active_stage = AuthorizationStage.INITIAL_FACE
        commands.append(
            AuthorizationCommand(session, AuthorizationStage.INITIAL_FACE)
        )
        return AuthorizationTransition(commands=tuple(commands))

    def _reset(self) -> None:
        self._active_session = None
        self._active_stage = None
        self._stage_results.clear()

    def record_observation(
        self, observation: AuthorizationObservation
    ) -> AuthorizationTransition:
        if (
            observation.session != self._active_session
            or observation.stage != self._active_stage
        ):
            return AuthorizationTransition()
        expected_providers = {
            AuthorizationStage.INITIAL_FACE: {AuthorizationProvider.FACE},
            AuthorizationStage.DUAL_FIRST: {
                AuthorizationProvider.FACE,
                AuthorizationProvider.VOICE,
            },
            AuthorizationStage.DUAL_SECOND: {
                AuthorizationProvider.FACE,
                AuthorizationProvider.VOICE,
            },
        }.get(self._active_stage, set())
        if observation.provider not in expected_providers:
            return AuthorizationTransition()
        if observation.result in (
            AuthorizationResult.PASSED,
            AuthorizationResult.ERROR,
        ):
            outcome = AuthorizationOutcome(
                observation.session, observation.result
            )
            self._reset()
            return AuthorizationTransition(
                commands=(
                    AuthorizationCommand(
                        observation.session, AuthorizationStage.CANCEL
                    ),
                ),
                outcome=outcome,
            )
        if (
            self._active_stage is AuthorizationStage.INITIAL_FACE
            and observation.result is AuthorizationResult.NOT_PASSED
        ):
            self._active_stage = AuthorizationStage.DUAL_FIRST
            self._stage_results.clear()
            return AuthorizationTransition(
                commands=(
                    AuthorizationCommand(
                        observation.session, AuthorizationStage.DUAL_FIRST
                    ),
                )
            )
        if (
            self._active_stage
            in (AuthorizationStage.DUAL_FIRST, AuthorizationStage.DUAL_SECOND)
            and observation.provider
            in (AuthorizationProvider.FACE, AuthorizationProvider.VOICE)
            and observation.result is AuthorizationResult.NOT_PASSED
        ):
            self._stage_results[observation.provider] = observation.result
            if set(self._stage_results) == {
                AuthorizationProvider.FACE,
                AuthorizationProvider.VOICE,
            }:
                if self._active_stage is AuthorizationStage.DUAL_FIRST:
                    self._active_stage = AuthorizationStage.DUAL_SECOND
                    self._stage_results.clear()
                    return AuthorizationTransition(
                        commands=(
                            AuthorizationCommand(
                                observation.session,
                                AuthorizationStage.DUAL_SECOND,
                            ),
                        )
                    )
                outcome = AuthorizationOutcome(
                    observation.session, AuthorizationResult.NOT_PASSED
                )
                self._reset()
                return AuthorizationTransition(
                    commands=(
                        AuthorizationCommand(
                            observation.session, AuthorizationStage.CANCEL
                        ),
                    ),
                    outcome=outcome,
                )
        return AuthorizationTransition()
