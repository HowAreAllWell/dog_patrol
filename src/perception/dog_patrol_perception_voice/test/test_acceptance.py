from __future__ import annotations

import json

import pytest

import dog_patrol_perception_voice.acceptance as acceptance
from dog_patrol_perception_voice.acceptance import (
    AcceptanceFixture,
    load_acceptance_fixture,
    run_fixture_acceptance,
)


def test_acceptance_fixture_is_strict_and_contains_only_task_outcomes(tmp_path) -> None:
    fixture_path = tmp_path / "voice-acceptance.json"
    fixture_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "tasks": [
                    {"windows": [{"accepted": True, "decision_time_seconds": 0.4}]},
                    {
                        "windows": [
                            {"accepted": False, "decision_time_seconds": 20.0},
                            {"accepted": True, "decision_time_seconds": 0.7},
                        ]
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    fixture = load_acceptance_fixture(fixture_path, expected_cycles=2)

    assert isinstance(fixture, AcceptanceFixture)
    assert [len(task.windows) for task in fixture.tasks] == [1, 2]
    assert fixture.tasks[1].windows[1].accepted is True

    fixture_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "tasks": [{"windows": [{"accepted": True}], "pcm_path": "capture.pcm"}],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="pcm_path"):
        load_acceptance_fixture(fixture_path)


def test_fixture_acceptance_covers_ros_lifecycle_and_failure_matrix() -> None:
    if acceptance.rclpy is None:
        pytest.skip("ROS 2 Python bindings are unavailable on this development host")
    fixture = AcceptanceFixture.from_tasks(
        [
            [{"accepted": True, "decision_time_seconds": 0.2}],
            [
                {"accepted": False, "decision_time_seconds": 20.0},
                {"accepted": True, "decision_time_seconds": 0.6},
            ],
        ]
    )

    report = run_fixture_acceptance(fixture, cycles=2)

    assert report["passed"] is True
    assert report["readiness"]["status"] == "READY"
    assert len(report["cycles"]) == 2
    assert all(cycle["cleanup"]["complete"] for cycle in report["cycles"])
    assert all(scenario["passed"] for scenario in report["failure_matrix"])
