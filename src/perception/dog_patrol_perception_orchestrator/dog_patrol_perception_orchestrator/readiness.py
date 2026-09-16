"""Perception capability readiness aggregation, independent of ROS transport."""

from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple


NOT_READY = 0
READY = 1
ERROR = 2
VALID_STATUSES = frozenset((NOT_READY, READY, ERROR))
UINT32_MASK = (1 << 32) - 1
UINT32_HALF_RANGE = 1 << 31


@dataclass(frozen=True)
class CapabilitySample:
    capability: str
    status: int
    observed_startup_state_seq: int
    diagnostic: str = ""


def _is_current_or_newer(candidate: int, current: int) -> bool:
    delta = (int(candidate) - int(current)) & UINT32_MASK
    return delta == 0 or delta < UINT32_HALF_RANGE


class ReadinessCoordinator:
    """Emits at most one action after all required capabilities match STARTUP."""

    def __init__(self, required_capabilities: Iterable[str], *, startup_state: int) -> None:
        required = tuple(str(name) for name in required_capabilities)
        if not required or any(not name for name in required):
            raise ValueError("required capability names must be non-empty")
        if len(set(required)) != len(required):
            raise ValueError("required capability names must be unique")
        self._required: Tuple[str, ...] = required
        self._startup_state = int(startup_state)
        self._samples: Dict[str, CapabilitySample] = {}
        self._mission_seq: Optional[int] = None
        self._mission_state: Optional[int] = None
        self._mission_signature = None
        self._mission_conflicted = False
        self._emitted_startup_seq: Optional[int] = None

    @property
    def required_capabilities(self) -> Tuple[str, ...]:
        return self._required

    def observe_mission(
        self,
        state_seq: int,
        state: int,
        target_id: int = 0,
        handled_target_id: int = 0,
    ) -> Optional[int]:
        state_seq = int(state_seq) & UINT32_MASK
        state = int(state)
        signature = (state, int(target_id), int(handled_target_id))
        if state_seq == 0:
            return None
        if self._mission_seq is not None:
            if not _is_current_or_newer(state_seq, self._mission_seq):
                return None
            if state_seq == self._mission_seq:
                if self._mission_conflicted or signature != self._mission_signature:
                    self._mission_conflicted = True
                    self._mission_state = None
                    return None
                return self._ready_startup_sequence()
        self._mission_seq = state_seq
        self._mission_state = state
        self._mission_signature = signature
        self._mission_conflicted = False
        return self._ready_startup_sequence()

    def observe_capability(self, sample: CapabilitySample) -> Optional[int]:
        if (
            sample.capability not in self._required
            or int(sample.status) not in VALID_STATUSES
            or int(sample.observed_startup_state_seq) <= 0
        ):
            return None
        normalized = CapabilitySample(
            capability=sample.capability,
            status=int(sample.status),
            observed_startup_state_seq=int(sample.observed_startup_state_seq) & UINT32_MASK,
            diagnostic=sample.diagnostic,
        )
        previous = self._samples.get(normalized.capability)
        if previous is not None:
            if normalized.observed_startup_state_seq == previous.observed_startup_state_seq:
                # A capability is expected to progress within one STARTUP
                # sequence (e.g. NOT_READY -> READY). Keep the newest status;
                # only authoritative MissionState same-sequence conflicts are
                # locked out.
                self._samples[normalized.capability] = normalized
                return self._ready_startup_sequence()
            if not _is_current_or_newer(
                normalized.observed_startup_state_seq,
                previous.observed_startup_state_seq,
            ):
                return None
        self._samples[normalized.capability] = normalized
        return self._ready_startup_sequence()

    def expire_mission(self) -> None:
        """Invalidate the current MissionState after heartbeat loss."""
        self._mission_seq = None
        self._mission_state = None
        self._mission_signature = None
        self._mission_conflicted = False

    def _ready_startup_sequence(self) -> Optional[int]:
        if (
            self._mission_seq is None
            or self._mission_seq == 0
            or self._mission_conflicted
            or self._mission_state != self._startup_state
            or self._mission_signature[1:] != (0, 0)
        ):
            return None
        if self._emitted_startup_seq == self._mission_seq:
            return None
        for capability in self._required:
            sample = self._samples.get(capability)
            if (
                sample is None
                or sample.observed_startup_state_seq != self._mission_seq
                or sample.status != READY
            ):
                return None
        self._emitted_startup_seq = self._mission_seq
        return self._mission_seq
