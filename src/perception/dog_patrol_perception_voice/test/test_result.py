from dataclasses import FrozenInstanceError

import pytest

from dog_patrol_perception_voice.result import (
    VoiceWindowEvent, VoiceWindowObservation, VoiceWindowResult, confidence_percent_for_votes,
)


@pytest.mark.parametrize("invalid", [-1, 7, True, 1.5, "2"])
def test_confidence_rejects_invalid_vote_counts(invalid):
    with pytest.raises(ValueError, match="vote_count"):
        confidence_percent_for_votes(invalid)


@pytest.mark.parametrize("channels", [(0, 0), (6,), (-1,), (True,)])
def test_snapshot_rejects_duplicate_and_invalid_microphones(channels):
    with pytest.raises(ValueError, match="matching_channels"):
        VoiceWindowObservation(VoiceWindowEvent.PASSED, channels)


def test_zero_votes_are_distinct_from_no_measurement():
    waiting = VoiceWindowObservation(VoiceWindowEvent.WAIT_STARTED)
    failed = VoiceWindowResult(False, 10.0).observation
    assert waiting.confidence_valid is False
    assert waiting.confidence_percent is None
    assert failed.confidence_valid is True
    assert failed.confidence_percent == 0
    # Injected/legacy results without a measured channel must not invent a score.
    assert VoiceWindowResult(True, 0.1).confidence_percent is None


def test_snapshot_owns_immutable_channels_and_rejects_conflicting_results():
    channels = [0, 2, 4]
    snapshot = VoiceWindowObservation(VoiceWindowEvent.PASSED, channels)
    channels.clear()
    assert snapshot.vote_count == 3
    assert snapshot.confidence_percent == 90
    with pytest.raises(FrozenInstanceError):
        snapshot.matching_channels = ()
    with pytest.raises(ValueError, match="agree"):
        VoiceWindowObservation(VoiceWindowEvent.NOT_PASSED, (0,))
    with pytest.raises(ValueError, match="WAIT_STARTED"):
        VoiceWindowObservation(VoiceWindowEvent.WAIT_STARTED, ())
