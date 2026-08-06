from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

import dog_patrol_perception_voice.adb as adb_module
from dog_patrol_perception_voice.adb import SubprocessAdbFileTransfer


def test_adb_transport_uses_selected_device_and_argument_bound_commands(tmp_path, monkeypatch) -> None:
    commands: list[list[str]] = []

    def fake_run(command, **settings):
        commands.append(command)
        assert settings == {
            "check": False,
            "capture_output": True,
            "text": True,
            "timeout": 7.5,
        }
        return subprocess.CompletedProcess(command, 0, "device\n", "")

    monkeypatch.setattr(adb_module.subprocess, "run", fake_run)
    local_helper = tmp_path / "helper"
    local_helper.write_bytes(b"helper")
    adb = SubprocessAdbFileTransfer(
        "/opt/android/adb",
        device_serial="R818-01",
        timeout_seconds=7.5,
    )

    adb.ensure_ready()
    adb.push(local_helper, "/tmp/helper")
    adb.shell(("chmod", "700", "/tmp/helper"))
    adb.shell_script("value=hello")

    assert commands == [
        ["/opt/android/adb", "-s", "R818-01", "get-state"],
        ["/opt/android/adb", "-s", "R818-01", "push", str(local_helper), "/tmp/helper"],
        ["/opt/android/adb", "-s", "R818-01", "shell", "chmod", "700", "/tmp/helper"],
        ["/opt/android/adb", "-s", "R818-01", "shell", "sh", "-c", "value=hello"],
    ]


def test_adb_transport_rejects_invalid_configuration() -> None:
    with pytest.raises(ValueError, match="executable"):
        SubprocessAdbFileTransfer("")
    with pytest.raises(ValueError, match="serial"):
        SubprocessAdbFileTransfer(device_serial=" ")
    with pytest.raises(ValueError, match="finite"):
        SubprocessAdbFileTransfer(timeout_seconds=0)


def test_adb_push_requires_a_local_file(tmp_path) -> None:
    adb = SubprocessAdbFileTransfer()

    with pytest.raises(OSError, match="source is not a file"):
        adb.push(Path(tmp_path) / "missing", "/tmp/helper")
