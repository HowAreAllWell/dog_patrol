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


def test_cycle_requires_exactly_one_r818_session() -> None:
    if acceptance.rclpy is None:
        pytest.skip("ROS 2 Python bindings are unavailable on this development host")
    fixture = AcceptanceFixture.from_tasks([[{"accepted": True, "decision_time_seconds": 0.2}]])
    session_counts = iter((0, 2))

    report = acceptance._run_cycles(
        fixture,
        cycles=1,
        adapter_factory=acceptance._FixtureAdapterFactory(
            acceptance._cycle_behaviors(fixture)
        ),
        preflight=lambda: acceptance.VoicePreflightOutcome(
            acceptance.READY, "fixture readiness ready"
        ),
        session_count=lambda: next(session_counts),
    )

    assert report["passed"] is False
    assert report["cycles"][0]["hardware_sessions_started"] == 2
    assert report["cycles"][0]["cleanup"]["complete"] is False
    assert report["cycles"][0]["passed"] is False


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


def test_field_mode_uses_the_fixed_minimal_user_matrix_without_a_fixture() -> None:
    args = acceptance.build_parser().parse_args(
        [
            "--mode",
            "field",
            "--model-dir",
            "/srv/dog-patrol/vosk-model",
            "--automated-report",
            "/srv/dog-patrol/issue37_voice_acceptance.json",
        ]
    )

    assert args.fixture is None
    assert args.cycles == 3
    assert args.automated_report == "/srv/dog-patrol/issue37_voice_acceptance.json"
    assert [
        [window.accepted for window in task.windows]
        for task in acceptance.minimal_field_matrix().tasks
    ] == [[True], [False, True], [False, False]]


def test_field_mode_requires_a_matching_passed_issue37_hardware_report(tmp_path) -> None:
    model_dir = tmp_path / "vosk-model"
    model_dir.mkdir()
    (model_dir / "model.bin").write_bytes(b"model")
    config_file = tmp_path / "voice.yaml"
    config_file.write_text("response_timeout_seconds: 20\n", encoding="utf-8")
    helper_path = tmp_path / "r818_pcm_base64"
    helper_path.write_bytes(b"helper")
    report_path = tmp_path / "issue37.json"
    report = {
        "issue": 37,
        "mode": "hardware",
        "passed": True,
        "deployment_assets": {
            "model": acceptance._path_fingerprint(model_dir),
            "config": acceptance._path_fingerprint(config_file),
            "helper": acceptance._path_fingerprint(helper_path),
        },
    }
    report_path.write_text(json.dumps(report), encoding="utf-8")

    missing_acceptance_gate = acceptance._field_automatic_gate(
        report_path,
        model_dir=model_dir,
        config_file=config_file,
        helper_path=helper_path,
    )

    assert missing_acceptance_gate["passed"] is False
    assert missing_acceptance_gate["diagnostic"].endswith("acceptance")

    report["deployment_assets"]["acceptance"] = acceptance._path_fingerprint(
        Path(acceptance.__file__)
    )
    report_path.write_text(json.dumps(report), encoding="utf-8")

    missing_matrix_gate = acceptance._field_automatic_gate(
        report_path,
        model_dir=model_dir,
        config_file=config_file,
        helper_path=helper_path,
    )

    assert missing_matrix_gate["passed"] is False
    assert "normal task matrix" in missing_matrix_gate["diagnostic"]

    report.update(
        {
            "cycles_completed": 3,
            "cycles_aborted": False,
            "cycles": [
                {
                    "passed": True,
                    "hardware_sessions_started": 1,
                    "expected_evidence": ["PASSED"],
                    "expected_event": "AUTHORIZED",
                    "cleanup": {"complete": True, "late_evidence": False},
                },
                {
                    "passed": True,
                    "hardware_sessions_started": 1,
                    "expected_evidence": ["NOT_PASSED", "PASSED"],
                    "expected_event": "AUTHORIZED",
                    "cleanup": {"complete": True, "late_evidence": False},
                },
                {
                    "passed": True,
                    "hardware_sessions_started": 1,
                    "expected_evidence": ["NOT_PASSED", "NOT_PASSED"],
                    "expected_event": "UNAUTHORIZED",
                    "cleanup": {"complete": True, "late_evidence": False},
                },
            ],
            "failure_matrix": [
                {
                    "name": name,
                    "passed": True,
                    "late_evidence": False,
                    "max_active_sessions": 0 if name == "adb_failure" else 1,
                    "cleanup": {"complete": True},
                }
                for name in acceptance._FAILURE_SCENARIOS
            ],
        }
    )
    report_path.write_text(json.dumps(report), encoding="utf-8")

    gate = acceptance._field_automatic_gate(
        report_path,
        model_dir=model_dir,
        config_file=config_file,
        helper_path=helper_path,
    )

    assert gate["passed"] is True

    helper_path.write_bytes(b"different helper")

    mismatched_gate = acceptance._field_automatic_gate(
        report_path,
        model_dir=model_dir,
        config_file=config_file,
        helper_path=helper_path,
    )

    assert mismatched_gate["passed"] is False
    assert mismatched_gate["diagnostic"].endswith("helper")


def test_field_acceptance_stops_before_hardware_when_automatic_gate_is_missing(
    tmp_path,
) -> None:
    report = acceptance.run_field_acceptance(
        model_dir=tmp_path / "vosk-model",
        config_file=tmp_path / "voice.yaml",
        automated_report=tmp_path / "missing-issue37.json",
        helper_path=tmp_path / "r818_pcm_base64",
    )

    assert report["passed"] is False
    assert report["readiness"]["status"] == "BLOCKED"
    assert report["cycles"] == []
    assert [entry["name"] for entry in report["field_matrix"]] == [
        "first_window_pass",
        "second_window_pass",
        "two_windows_not_passed",
    ]


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
