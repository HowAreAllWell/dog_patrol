from tools.navigation_coordinator import mock_mission_events
from types import SimpleNamespace


def test_state_sequence_comparison_is_wrap_aware() -> None:
    compare = mock_mission_events.is_current_or_newer_state_seq

    assert compare(12, 12)
    assert compare(13, 12)
    assert not compare(11, 12)
    assert compare(1, 0xFFFFFFFF)
    assert not compare(0xFFFFFFFF, 1)


def test_mock_event_source_locks_conflicting_snapshot_until_newer_sequence() -> None:
    node = SimpleNamespace(
        _state=SimpleNamespace(state_seq=7, state=1, target_id=0, handled_target_id=0),
        _state_conflicted=False,
        _state_signature=(1, 0, 0),
    )
    conflict = SimpleNamespace(state_seq=7, state=7, target_id=0, handled_target_id=0)
    newer = SimpleNamespace(state_seq=8, state=1, target_id=0, handled_target_id=0)

    mock_mission_events.MockMissionEvents._on_state(node, conflict)
    assert node._state_conflicted is True
    mock_mission_events.MockMissionEvents._on_state(node, newer)

    assert node._state is newer
    assert node._state_conflicted is False
