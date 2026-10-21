"""Results returned by one independent response window."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


class VoiceWindowEvent(Enum):
    WAIT_STARTED = "wait_started"
    PASSED = "passed"
    NOT_PASSED = "not_passed"


def confidence_percent_for_votes(vote_count: int) -> int:
    """Business score for distinct matching microphone channels (not ASR probability)."""
    if isinstance(vote_count, bool) or not isinstance(vote_count, int) or not 0 <= vote_count <= 6:
        raise ValueError("vote_count must be an integer between 0 and 6")
    return (0, 50, 75, 90, 100, 100, 100)[vote_count]


def _validate_channels(channels: tuple[int, ...]) -> None:
    if any(
        isinstance(channel, bool) or not isinstance(channel, int) or not 0 <= channel < 6
        for channel in channels
    ) or len(set(channels)) != len(channels):
        raise ValueError("matching_channels must contain distinct microphone indices 0 through 5")


@dataclass(frozen=True)
class VoiceWindowObservation:
    """Immutable decision-time snapshot, emitted before task/hardware cleanup."""

    event: VoiceWindowEvent
    # None means no measurement, including WAIT_STARTED. An empty tuple is a valid zero vote.
    matching_channels: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.event, VoiceWindowEvent):
            raise ValueError("event must be a VoiceWindowEvent")
        if self.matching_channels is not None:
            object.__setattr__(self, "matching_channels", tuple(self.matching_channels))
            _validate_channels(self.matching_channels)
            if self.event is VoiceWindowEvent.WAIT_STARTED:
                raise ValueError("WAIT_STARTED has no confidence measurement")
            if (self.event is VoiceWindowEvent.PASSED) != bool(self.matching_channels):
                raise ValueError("window event must agree with its matching channels")

    @property
    def confidence_valid(self) -> bool:
        return self.matching_channels is not None

    @property
    def vote_count(self) -> int:
        return len(self.matching_channels or ())

    @property
    def confidence_percent(self) -> int | None:
        return confidence_percent_for_votes(self.vote_count) if self.confidence_valid else None


@dataclass(frozen=True)
class VoiceWindowResult:
    """The result of exactly one Prompt/response window."""

    accepted: bool
    decision_time_seconds: float
    attempt_number: int = 0
    pcm_bytes: int = 0
    captured_duration_seconds: float = 0.0
    channel_results: tuple[str, ...] = ()
    matching_channels: tuple[int, ...] = ()
    vote_counts: dict[str, int] = field(default_factory=dict)

    @property
    def observation(self) -> VoiceWindowObservation:
        # Legacy/injected accepted results may not contain channel measurements.
        channels = self.matching_channels if self.matching_channels or not self.accepted else None
        return VoiceWindowObservation(
            VoiceWindowEvent.PASSED if self.accepted else VoiceWindowEvent.NOT_PASSED, channels
        )

    @property
    def vote_count(self) -> int:
        return self.observation.vote_count

    @property
    def confidence_percent(self) -> int | None:
        return self.observation.confidence_percent

    def __post_init__(self) -> None:
        if not isinstance(self.accepted, bool):
            raise ValueError("accepted must be a boolean")
        if not math.isfinite(self.decision_time_seconds) or self.decision_time_seconds < 0:
            raise ValueError("decision_time_seconds must be finite and non-negative")
        if (
            isinstance(self.attempt_number, bool)
            or not isinstance(self.attempt_number, int)
            or self.attempt_number < 0
        ):
            raise ValueError("attempt_number must be a non-negative integer")
        if (
            isinstance(self.pcm_bytes, bool)
            or not isinstance(self.pcm_bytes, int)
            or self.pcm_bytes < 0
            or self.captured_duration_seconds < 0
        ):
            raise ValueError("capture measurements must be non-negative")
        if not math.isfinite(self.captured_duration_seconds):
            raise ValueError("captured_duration_seconds must be finite")
        object.__setattr__(self, "matching_channels", tuple(self.matching_channels))
        _validate_channels(self.matching_channels)
        if self.matching_channels and not self.accepted:
            raise ValueError("a rejected window cannot contain matching channels")
