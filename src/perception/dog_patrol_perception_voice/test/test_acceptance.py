from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

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


def test_hardware_environment_gate_requires_explicit_unified_pass(monkeypatch) -> None:
    assert acceptance.run_unified_environment_check(None)["passed"] is False

    monkeypatch.setattr(
        acceptance.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0,
            stdout="PERCEPTION ENVIRONMENT: PASS\n",
        ),
    )
    result = acceptance.run_unified_environment_check(("environment-check",))

    assert result["passed"] is True
    assert result["status"] == "PASS"


def test_acceptance_defaults_to_minimal_representative_cycle_set() -> None:
    args = acceptance.build_parser().parse_args(["--fixture", "fixture.json"])

    assert args.cycles == 3


def test_hardware_failure_matrix_allows_second_window_stage(monkeypatch, tmp_path) -> None:
    observed: list[float] = []

    def fake_run_failure_scenario(name, **kwargs):
        del name
        observed.append(kwargs["stage_timeout_seconds"])
        return {"passed": True}

    monkeypatch.setattr(acceptance, "_run_failure_scenario", fake_run_failure_scenario)
    config = acceptance.VoiceConfig()

    acceptance.run_hardware_failure_matrix(
        config,
        helper_path=Path(tmp_path / "helper"),
        adb=object(),
    )

    assert observed == [30.0] * len(acceptance._FAILURE_SCENARIOS)


def test_host_pcm_check_only_reports_new_capture_artifacts() -> None:
    before = {"paths": ["/tmp/old.wav"], "arecord_processes": ["old arecord"]}
    after = {
        "paths": ["/tmp/old.wav", "/tmp/new.pcm"],
        "arecord_processes": ["old arecord", "new arecord"],
    }

    result = acceptance._host_pcm_check(before, after)

    assert result["passed"] is False
    assert result["new_paths"] == ["/tmp/new.pcm"]
    assert result["new_arecord_processes"] == ["new arecord"]
