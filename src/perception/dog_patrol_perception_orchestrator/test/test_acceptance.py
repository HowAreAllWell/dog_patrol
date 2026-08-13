import pytest

from dog_patrol_perception_orchestrator.acceptance import (
    AcceptanceEvidence,
    acceptance_case_passes,
)
from dog_patrol_perception_orchestrator.authorization import (
    AuthorizationProvider as Provider,
    AuthorizationResult as Result,
    AuthorizationStage as Stage,
)


def item(provider, stage, result):
    return AcceptanceEvidence(provider, stage, result)


def test_initial_face_pass_requires_no_voice_work():
    evidence = [item(Provider.FACE, Stage.INITIAL_FACE, Result.PASSED)]
    assert acceptance_case_passes("initial_face_pass", Result.PASSED, evidence)
    assert not acceptance_case_passes(
        "initial_face_pass",
        Result.PASSED,
        evidence + [item(Provider.VOICE, Stage.DUAL_FIRST, Result.CANCELLED)],
    )


def test_dual_pass_requires_initial_face_miss_and_a_dual_pass():
    evidence = [
        item(Provider.FACE, Stage.INITIAL_FACE, Result.NOT_PASSED),
        item(Provider.FACE, Stage.DUAL_FIRST, Result.NOT_PASSED),
        item(Provider.VOICE, Stage.DUAL_FIRST, Result.PASSED),
    ]
    assert acceptance_case_passes("dual_pass", Result.PASSED, evidence)
    assert not acceptance_case_passes(
        "dual_pass",
        Result.PASSED,
        [evidence[0], evidence[1], item(Provider.FACE, Stage.DUAL_FIRST, Result.PASSED)],
    )


def test_dual_reject_requires_both_providers_to_miss_both_rounds():
    evidence = [item(Provider.FACE, Stage.INITIAL_FACE, Result.NOT_PASSED)]
    evidence.extend(
        item(provider, stage, Result.NOT_PASSED)
        for stage in (Stage.DUAL_FIRST, Stage.DUAL_SECOND)
        for provider in (Provider.FACE, Provider.VOICE)
    )
    assert acceptance_case_passes("dual_reject", Result.NOT_PASSED, evidence)
    assert not acceptance_case_passes(
        "dual_reject", Result.NOT_PASSED, evidence[:-1]
    )


def test_unknown_case_is_rejected():
    with pytest.raises(ValueError, match="unknown"):
        acceptance_case_passes("other", Result.PASSED, [])
