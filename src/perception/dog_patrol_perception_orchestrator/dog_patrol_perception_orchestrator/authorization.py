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
    round_number: int = 0

    def __post_init__(self) -> None:
        if self.round_number == 0 and self.stage in (
            AuthorizationStage.DUAL_FIRST, AuthorizationStage.DUAL_SECOND
        ):
            object.__setattr__(
                self, "round_number", 1 if self.stage is AuthorizationStage.DUAL_FIRST else 2
            )


@dataclass(frozen=True)
class AuthorizationObservation:
    session: AuthorizationSession
    stage: AuthorizationStage
    provider: AuthorizationProvider
    result: AuthorizationResult
    round_number: int = 0


@dataclass(frozen=True)
class AuthorizationTransition:
    commands: tuple[AuthorizationCommand, ...] = ()
    outcome: Optional[AuthorizationOutcome] = None


class AuthorizationCoordinator:
    """Combines verification results without exposing attempt state."""

    def __init__(self, *, voice_prompt_count: int = 2) -> None:
        self.set_voice_prompt_count(voice_prompt_count)
        self._active_session: Optional[AuthorizationSession] = None
        self._active_stage: Optional[AuthorizationStage] = None
        self._round_number = 0
        self._session_prompt_count = voice_prompt_count
        self._policy_session: Optional[AuthorizationSession] = None

    def set_voice_prompt_count(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 0xFFFFFFFF:
            raise ValueError("voice_prompt_count must be an integer between 1 and 4294967295")
        self._voice_prompt_count = value

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
        if session != self._policy_session:
            self._session_prompt_count = self._voice_prompt_count
            self._policy_session = session
        self._active_stage = AuthorizationStage.INITIAL_FACE
        commands.append(
            AuthorizationCommand(session, AuthorizationStage.INITIAL_FACE)
        )
        return AuthorizationTransition(commands=tuple(commands))

    def _reset(self) -> None:
        self._active_session = None
        self._active_stage = None
        self._round_number = 0

    def record_observation(
        self, observation: AuthorizationObservation
    ) -> AuthorizationTransition:
        if observation.session != self._active_session:
            return AuthorizationTransition()
        joint_stages = (AuthorizationStage.DUAL_FIRST, AuthorizationStage.DUAL_SECOND)
        # Face runs continuously across voice rounds and keeps the stage of its
        # original joint command. Voice evidence still belongs to one exact round.
        continuous_face = (
            observation.provider is AuthorizationProvider.FACE
            and self._active_stage in joint_stages
            and observation.stage in joint_stages
        )
        if observation.stage != self._active_stage and not continuous_face:
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
        if observation.provider is AuthorizationProvider.VOICE:
            # Zero identifies old providers, which only knew rounds one and two.
            round_number = observation.round_number or (
                1 if observation.stage is AuthorizationStage.DUAL_FIRST else 2
            )
            if round_number != self._round_number:
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
            self._round_number = 1
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
            and observation.provider is AuthorizationProvider.VOICE
            and observation.result is AuthorizationResult.NOT_PASSED
        ):
            if self._round_number < self._session_prompt_count:
                self._round_number += 1
                self._active_stage = AuthorizationStage.DUAL_SECOND
                return AuthorizationTransition(
                    commands=(
                        AuthorizationCommand(
                            observation.session,
                            AuthorizationStage.DUAL_SECOND,
                            self._round_number,
                        ),
                    ),
                )
            outcome = AuthorizationOutcome(
                observation.session, AuthorizationResult.NOT_PASSED
            )
            self._reset()
            return AuthorizationTransition(
                commands=(
                    AuthorizationCommand(observation.session, AuthorizationStage.CANCEL),
                ),
                outcome=outcome,
            )
        return AuthorizationTransition()
