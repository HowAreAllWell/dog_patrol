from dataclasses import dataclass
from enum import Enum
from typing import Optional


class AuthorizationResult(Enum):
    PASSED = "passed"
    NOT_PASSED = "not_passed"
    ERROR = "error"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class AuthorizationSession:
    observed_state_seq: int
    target_id: int


@dataclass(frozen=True)
class AuthorizationOutcome:
    session: AuthorizationSession
    result: AuthorizationResult


class AuthorizationCoordinator:
    """Combines verification results without exposing attempt state."""

    _TERMINAL_RESULTS = frozenset(
        {
            AuthorizationResult.PASSED,
            AuthorizationResult.ERROR,
            AuthorizationResult.CANCELLED,
        }
    )

    def __init__(self, required_not_passed: int = 2) -> None:
        self._required_not_passed = max(1, int(required_not_passed))
        self._active_session: Optional[AuthorizationSession] = None
        self._not_passed_count = 0

    @property
    def active_session(self) -> Optional[AuthorizationSession]:
        return self._active_session

    def start(self, session: AuthorizationSession) -> None:
        self._active_session = session
        self._not_passed_count = 0

    def reset(self) -> None:
        self._active_session = None
        self._not_passed_count = 0

    def record(
        self,
        session: AuthorizationSession,
        result: AuthorizationResult,
    ) -> Optional[AuthorizationOutcome]:
        if session != self._active_session:
            return None
        if result in self._TERMINAL_RESULTS:
            return self._complete(session, result)
        if result is not AuthorizationResult.NOT_PASSED:
            raise ValueError(f"unknown authorization result: {result}")

        self._not_passed_count += 1
        if self._not_passed_count < self._required_not_passed:
            return None

        return self._complete(session, AuthorizationResult.NOT_PASSED)

    def _complete(
        self,
        session: AuthorizationSession,
        result: AuthorizationResult,
    ) -> AuthorizationOutcome:
        outcome = AuthorizationOutcome(session, result)
        self._active_session = None
        self._not_passed_count = 0
        return outcome
