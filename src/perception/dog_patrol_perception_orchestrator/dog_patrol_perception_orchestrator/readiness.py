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

    def __init__(self, required_capabilities: Iterable[str]) -> None:
        required = tuple(str(name) for name in required_capabilities)
        if not required or any(not name for name in required):
            raise ValueError("required capability names must be non-empty")
        if len(set(required)) != len(required):
            raise ValueError("required capability names must be unique")
        self._required: Tuple[str, ...] = required
        self._samples: Dict[str, CapabilitySample] = {}
        self._mission_seq: Optional[int] = None
        self._mission_state: Optional[int] = None
        self._emitted_startup_seq: Optional[int] = None

    @property
    def required_capabilities(self) -> Tuple[str, ...]:
        return self._required

    def observe_mission(self, state_seq: int, state: int, startup_state: int) -> Optional[int]:
        state_seq = int(state_seq) & UINT32_MASK
        state = int(state)
        if self._mission_seq is not None:
            if not _is_current_or_newer(state_seq, self._mission_seq):
                return None
            if state_seq == self._mission_seq and state != self._mission_state:
                return None
        self._mission_seq = state_seq
        self._mission_state = state
        return self._ready_action(startup_state)

    def observe_capability(self, sample: CapabilitySample, startup_state: int) -> Optional[int]:
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
        if previous is not None and not _is_current_or_newer(
            normalized.observed_startup_state_seq,
            previous.observed_startup_state_seq,
        ):
            return None
        self._samples[normalized.capability] = normalized
        return self._ready_action(startup_state)

    def _ready_action(self, startup_state: int) -> Optional[int]:
        if self._mission_seq is None or self._mission_state != int(startup_state):
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
