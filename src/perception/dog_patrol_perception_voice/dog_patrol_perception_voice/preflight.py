"""Read-only deployment preflight for the production voice capability."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
from typing import Protocol

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - ROS is present in installed deployments.
    get_package_share_directory = None

from .adb import SubprocessAdbFileTransfer
from .config import VoiceConfig, default_config, load_voice_config
from .vosk import load_vosk_model


NOT_READY = 0
READY = 1
ERROR = 2

DEFAULT_HELPER_SHA256 = (
    "c2517d85e60845679acaeab4aa6c4f439b828393c5d73599dcef0e4fa68c0f52"
)


@dataclass(frozen=True)
class VoicePreflightOutcome:
    status: int
    diagnostic: str
    startup_state_seq: int | None = None


class AdbReadiness(Protocol):
    def ensure_ready(self) -> None: ...


CommandRunner = Callable[[Sequence[str]], tuple[int, str]]
ExecutableLookup = Callable[[str], str | None]
ModelLoader = Callable[[str | Path], object]


class VoicePreflight:
    """
    Check voice deployment inputs without starting a voice task.

    The preflight only loads the local Vosk model and executes read-only tool
    queries. It never creates an ``R818VoiceAdapter`` task, pushes the helper,
    runs an ADB shell command, changes mixer state, or plays a prompt.
    """

    def __init__(
        self,
        *,
        model_dir: str | Path | None,
        config_file: str | Path,
        helper_path: str | Path | None = None,
        expected_helper_sha256: str = DEFAULT_HELPER_SHA256,
        package_share: str | Path | None = None,
        adb: AdbReadiness | None = None,
        executable_lookup: ExecutableLookup = shutil.which,
        command_runner: CommandRunner | None = None,
        model_loader: ModelLoader = load_vosk_model,
    ) -> None:
        self.model_dir = Path(model_dir) if model_dir is not None else None
        self.config_file = Path(config_file)
        self.helper_path = (
            Path(helper_path) if helper_path is not None else default_helper_path()
        )
        self.expected_helper_sha256 = expected_helper_sha256
        self.package_share = (
            Path(package_share)
            if package_share is not None
            else _default_package_share()
        )
        self.adb = adb
        self.executable_lookup = executable_lookup
        self.command_runner = command_runner or _run_command
        self.model_loader = model_loader

    def run(self) -> VoicePreflightOutcome:
        not_ready: list[str] = []
        errors: list[str] = []
        config = self._load_config(not_ready, errors)

        self._check_install(not_ready)
        self._check_model(not_ready, errors)
        self._check_helper(not_ready, errors)
        self._check_tools(config, not_ready, errors)
        self._check_adb(config, not_ready)

        if errors:
            return VoicePreflightOutcome(ERROR, "; ".join(errors + not_ready))
        if not_ready:
            return VoicePreflightOutcome(NOT_READY, "; ".join(not_ready))
        return VoicePreflightOutcome(READY, "voice preflight ready")

    def _check_install(self, not_ready: list[str]) -> None:
        if self.package_share is None:
            not_ready.append("dog_patrol_perception_voice is not installed")
            return
        required = (
            self.package_share / "package.xml",
            self.package_share / "requirements.txt",
            self.package_share / "launch" / "voice.launch.py",
            self.package_share.parent
            / "ament_index"
            / "resource_index"
            / "packages"
            / "dog_patrol_perception_voice",
        )
        executables = (
            self.package_share.parents[1]
            / "lib"
            / "dog_patrol_perception_voice"
            / "perception_voice_provider",
            self.package_share.parents[1]
            / "lib"
            / "dog_patrol_perception_voice"
            / "perception_voice_readiness",
        )
        missing = [str(path) for path in required if not path.is_file()]
        missing.extend(
            str(path)
            for path in executables
            if not path.is_file() or not os.access(path, os.X_OK)
        )
        if missing:
            not_ready.append(
                "clean-installed voice package is incomplete: " + ", ".join(missing)
            )

    def _load_config(
        self,
        not_ready: list[str],
        errors: list[str],
    ) -> VoiceConfig:
        if not self.config_file.is_file():
            not_ready.append(f"voice config is missing: {self.config_file}")
            return default_config()
        try:
            return load_voice_config(self.config_file)
        except ValueError as exc:
            errors.append(f"voice config is invalid: {exc}")
            return default_config()

    def _check_model(self, not_ready: list[str], errors: list[str]) -> None:
        if self.model_dir is None or not self.model_dir.is_dir():
            path = self.model_dir if self.model_dir is not None else "<unset>"
            not_ready.append(f"Vosk model directory is missing: {path}")
            return
        try:
            self.model_loader(self.model_dir)
        except Exception as exc:
            errors.append(f"Vosk model cannot be loaded: {_exception_detail(exc)}")

    def _check_helper(self, not_ready: list[str], errors: list[str]) -> None:
        if not self.helper_path.is_file() or not os.access(self.helper_path, os.X_OK):
            not_ready.append(
                "installed R818 helper is missing or not executable: "
                f"{self.helper_path}"
            )
            return
        digest = hashlib.sha256(self.helper_path.read_bytes()).hexdigest()
        if digest != self.expected_helper_sha256:
            errors.append(
                "R818 helper checksum mismatch: "
                f"expected {self.expected_helper_sha256}, got {digest}"
            )

    def _check_tools(
        self,
        config: VoiceConfig,
        not_ready: list[str],
        errors: list[str],
    ) -> None:
        ffmpeg = self._find_executable("ffmpeg", not_ready)
        if ffmpeg is not None:
            code, output = self._run((ffmpeg, "-hide_banner", "-filters"))
            if code != 0 or not re.search(r"\bflite\b", output, re.IGNORECASE):
                not_ready.append("FFmpeg does not provide the flite audio filter")

        aplay = self._find_executable("aplay", not_ready)
        if aplay is not None:
            code, output = self._run((aplay, "-L"))
            if code != 0:
                not_ready.append("cannot enumerate ALSA playback devices with aplay")
            elif config.prompt_device not in output:
                not_ready.append(
                    f"configured ALSA playback device is not enumerated: {config.prompt_device}"
                )

        amixer = self._find_executable("amixer", not_ready)
        if amixer is not None:
            code, output = self._run(
                (amixer, "-c", config.prompt_mixer_card, "scontrols")
            )
            if code != 0:
                not_ready.append(
                    f"cannot enumerate ALSA mixer controls for card {config.prompt_mixer_card}"
                )
            elif config.prompt_mixer_control not in output:
                not_ready.append(
                    f"configured ALSA mixer control is not enumerated: "
                    f"{config.prompt_mixer_control}"
                )

    def _check_adb(self, config: VoiceConfig, not_ready: list[str]) -> None:
        adb = self.adb
        if adb is None:
            try:
                adb = SubprocessAdbFileTransfer(
                    config.adb_executable,
                    device_serial=config.device_serial,
                    timeout_seconds=config.adb_timeout_seconds,
                )
            except (OSError, ValueError) as exc:
                not_ready.append(f"ADB configuration is unavailable: {_exception_detail(exc)}")
                return
        try:
            adb.ensure_ready()
        except Exception as exc:
            not_ready.append(f"ADB device is not ready: {_exception_detail(exc)}")

    def _find_executable(self, name: str, not_ready: list[str]) -> str | None:
        executable = self.executable_lookup(name)
        if executable is None:
            not_ready.append(f"required executable is missing: {name}")
        return executable

    def _run(self, command: Sequence[str]) -> tuple[int, str]:
        try:
            return self.command_runner(command)
        except Exception as exc:
            return 127, _exception_detail(exc)


def default_helper_path() -> Path:
    """Return the helper beside the installed Python package."""
    package_share = _default_package_share()
    if package_share is None:
        return Path("/__dog_patrol_voice_install_missing__/r818_pcm_base64_aarch64")
    prefix = package_share.parents[1]
    candidates = list(
        prefix.glob(
            "lib/python*/site-packages/dog_patrol_perception_voice/assets/"
            "r818_pcm_base64_aarch64"
        )
    )
    candidates.extend(
        prefix.glob(
            "*/lib/python*/site-packages/dog_patrol_perception_voice/assets/"
            "r818_pcm_base64_aarch64"
        )
    )
    return candidates[0] if candidates else Path(
        "/__dog_patrol_voice_install_missing__/r818_pcm_base64_aarch64"
    )


def _default_package_share() -> Path | None:
    if get_package_share_directory is None:
        return None
    try:
        return Path(get_package_share_directory("dog_patrol_perception_voice"))
    except Exception:
        return None


def _run_command(command: Sequence[str]) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            list(command),
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except OSError as exc:
        return 127, _exception_detail(exc)
    return completed.returncode, completed.stdout.strip()


def _exception_detail(exc: Exception) -> str:
    detail = str(exc).strip()
    return f"{type(exc).__name__}: {detail}" if detail else type(exc).__name__
