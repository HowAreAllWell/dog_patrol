"""Pure assertions for the final fake-navigation perception acceptance."""

from dataclasses import dataclass

from .authorization import (
    AuthorizationProvider,
    AuthorizationResult,
    AuthorizationStage,
)


@dataclass(frozen=True)
class AcceptanceEvidence:
    provider: AuthorizationProvider
    stage: AuthorizationStage
    result: AuthorizationResult


def acceptance_case_passes(
    case: str,
    outcome: AuthorizationResult,
    evidence: list[AcceptanceEvidence],
) -> bool:
    """Return whether evidence proves the selected internal-flow shape."""
    if case == "general":
        return outcome in (
            AuthorizationResult.PASSED,
            AuthorizationResult.NOT_PASSED,
        )

    non_cancelled = [
        item for item in evidence if item.result is not AuthorizationResult.CANCELLED
    ]
    initial_face = [
        item
        for item in non_cancelled
        if item.provider is AuthorizationProvider.FACE
        and item.stage is AuthorizationStage.INITIAL_FACE
    ]
    dual = [
        item
        for item in non_cancelled
        if item.stage in (AuthorizationStage.DUAL_FIRST, AuthorizationStage.DUAL_SECOND)
    ]

    if case == "initial_face_pass":
        return (
            outcome is AuthorizationResult.PASSED
            and [item.result for item in initial_face] == [AuthorizationResult.PASSED]
            and not any(
                item.provider is AuthorizationProvider.VOICE
                for item in evidence
            )
        )
    if case == "dual_pass":
        dual_first_providers = {
            item.provider
            for item in dual
            if item.stage is AuthorizationStage.DUAL_FIRST
        }
        return (
            outcome is AuthorizationResult.PASSED
            and bool(initial_face)
            and initial_face[0].result is AuthorizationResult.NOT_PASSED
            and dual_first_providers == {AuthorizationProvider.FACE, AuthorizationProvider.VOICE}
            and any(item.result is AuthorizationResult.PASSED for item in dual)
        )
    if case == "dual_reject":
        required_misses = {
            (provider, stage)
            for provider in (AuthorizationProvider.FACE, AuthorizationProvider.VOICE)
            for stage in (
                AuthorizationStage.DUAL_FIRST,
                AuthorizationStage.DUAL_SECOND,
            )
        }
        actual_misses = {
            (item.provider, item.stage)
            for item in dual
            if item.result is AuthorizationResult.NOT_PASSED
        }
        return (
            outcome is AuthorizationResult.NOT_PASSED
            and bool(initial_face)
            and initial_face[0].result is AuthorizationResult.NOT_PASSED
            and required_misses.issubset(actual_misses)
        )
    raise ValueError(f"unknown acceptance case: {case}")
