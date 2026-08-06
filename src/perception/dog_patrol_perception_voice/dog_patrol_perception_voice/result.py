"""Results returned by one independent response window."""

from __future__ import annotations

import math
from dataclasses import dataclass, field


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
