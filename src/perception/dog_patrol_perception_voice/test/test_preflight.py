from __future__ import annotations

from pathlib import Path

from dog_patrol_perception_voice.preflight import (
    ERROR,
    NOT_READY,
    READY,
    VoicePreflight,
)


class FakeAdb:
    def __init__(self, *, ready: bool = True) -> None:
        self.ready = ready
        self.ensure_ready_calls = 0

    def ensure_ready(self) -> None:
        self.ensure_ready_calls += 1
        if not self.ready:
            raise OSError("ADB device is unauthorized")


def _write_config(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "passphrase: blue star",
                "prompt_device: plughw:CARD=Device,DEV=0",
                "prompt_mixer_card: Device",
                "prompt_mixer_control: PCM",
            ]
        ),
        encoding="utf-8",
    )


def _write_helper(path: Path, content: bytes = b"helper") -> str:
    path.write_bytes(content)
    path.chmod(0o755)
    return __import__("hashlib").sha256(content).hexdigest()


def _commands(command):
    if command[0].endswith("ffmpeg"):
        return 0, " ... flite ...\n"
    if command[0].endswith("aplay"):
        return 0, "null\nplughw:CARD=Device,DEV=0\n"
    if command[0].endswith("amixer"):
        return 0, "Simple mixer control 'PCM',0\n"
    raise AssertionError(f"unexpected command: {command}")


def _preflight(tmp_path: Path, *, adb: FakeAdb | None = None, helper_content: bytes = b"helper"):
    model_dir = tmp_path / "vosk-model"
    model_dir.mkdir()
    config_file = tmp_path / "voice.yaml"
    _write_config(config_file)
    helper_path = tmp_path / "r818_pcm_base64_aarch64"
    expected_hash = _write_helper(helper_path, helper_content)
    package_share = tmp_path / "share" / "dog_patrol_perception_voice"
    (package_share / "launch").mkdir(parents=True)
    (package_share / "package.xml").write_text("<package />", encoding="utf-8")
    (package_share / "requirements.txt").write_text("vosk==0.3.45\n", encoding="utf-8")
    (package_share / "launch" / "voice.launch.py").write_text("", encoding="utf-8")
    prefix = package_share.parents[1]
    marker = prefix / "share" / "ament_index" / "resource_index" / "packages"
    marker.mkdir(parents=True)
    (marker / "dog_patrol_perception_voice").write_text("", encoding="utf-8")
    executables = prefix / "lib" / "dog_patrol_perception_voice"
    executables.mkdir(parents=True)
    for name in ("perception_voice_provider", "perception_voice_readiness"):
        executable = executables / name
        executable.write_text("", encoding="utf-8")
        executable.chmod(0o755)
    return VoicePreflight(
        model_dir=model_dir,
        config_file=config_file,
        helper_path=helper_path,
        expected_helper_sha256=expected_hash,
        package_share=package_share,
        adb=adb or FakeAdb(),
        executable_lookup=lambda name: f"/usr/bin/{name}",
        command_runner=_commands,
        model_loader=lambda path: object(),
    )


def test_real_preflight_reports_ready_without_starting_or_writing_to_r818(tmp_path: Path) -> None:
    adb = FakeAdb()
    result = _preflight(tmp_path, adb=adb).run()

    assert result.status == READY
    assert "ready" in result.diagnostic.lower()
    assert adb.ensure_ready_calls == 1


def test_missing_model_or_unavailable_adb_reports_not_ready(tmp_path: Path) -> None:
    preflight = _preflight(tmp_path, adb=FakeAdb(ready=False))
    model_dir = tmp_path / "vosk-model"
    for path in model_dir.iterdir():
        path.unlink()
    model_dir.rmdir()

    result = preflight.run()

    assert result.status == NOT_READY
    assert "model" in result.diagnostic.lower()
    assert "adb" in result.diagnostic.lower()


def test_invalid_helper_checksum_is_an_error(tmp_path: Path) -> None:
    preflight = _preflight(tmp_path)
    preflight.expected_helper_sha256 = "0" * 64

    result = preflight.run()

    assert result.status == ERROR
    assert "checksum" in result.diagnostic.lower()


def test_preflight_rejects_an_incomplete_package_install(tmp_path: Path) -> None:
    preflight = _preflight(tmp_path)
    (preflight.package_share / "launch" / "voice.launch.py").unlink()

    result = preflight.run()

    assert result.status == NOT_READY
    assert "clean-installed voice package" in result.diagnostic


def test_preflight_rejects_non_executable_installed_entrypoint(tmp_path: Path) -> None:
    preflight = _preflight(tmp_path)
    entrypoint = (
        preflight.package_share.parents[1]
        / "lib"
        / "dog_patrol_perception_voice"
        / "perception_voice_provider"
    )
    entrypoint.chmod(0o644)

    result = preflight.run()

    assert result.status == NOT_READY
    assert "clean-installed voice package" in result.diagnostic


def test_preflight_does_not_call_prompt_playback_or_r818_write_commands(tmp_path: Path) -> None:
    commands = []

    def record_command(command):
        commands.append(tuple(command))
        return _commands(command)

    _preflight_instance = _preflight(tmp_path)
    _preflight_instance.command_runner = record_command
    result = _preflight_instance.run()

    assert result.status == READY
    assert all(
        command[0].rsplit("/", 1)[-1] in {"ffmpeg", "aplay", "amixer"}
        for command in commands
    )
    assert all("push" not in command and "shell" not in command for command in commands)
    assert all(
        command[0].rsplit("/", 1)[-1] != "aplay" or "-D" not in command
        for command in commands
    )
