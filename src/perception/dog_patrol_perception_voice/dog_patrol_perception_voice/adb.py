"""Small, argument-safe ADB transport used by the R818 stream."""

from __future__ import annotations

import math
import shlex
import subprocess
from pathlib import Path


class SubprocessAdbFileTransfer:
    """Run bounded ADB operations against one selected device."""

    def __init__(
        self,
        executable: str | Path = "adb",
        *,
        device_serial: str | None = None,
        timeout_seconds: float = 15.0,
    ) -> None:
        executable_text = str(executable).strip()
        normalized_serial = device_serial.strip() if device_serial is not None else None
        if not executable_text:
            raise ValueError("ADB executable must not be empty")
        if normalized_serial == "":
            raise ValueError("ADB device serial must not be empty")
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("ADB timeout must be finite and positive")
        self._executable = executable_text
        self._device_serial = normalized_serial
        self._timeout_seconds = timeout_seconds

    def ensure_ready(self) -> None:
        state = self._run(("get-state",)).stdout.strip()
        if state != "device":
            raise OSError(f"ADB device is not ready: {state or 'no state returned'}")

    def push(self, local_path: Path, remote_path: str) -> None:
        if not local_path.is_file():
            raise OSError(f"ADB push source is not a file: {local_path}")
        self._run(("push", str(local_path), remote_path))

    def shell(
        self,
        arguments: tuple[str, ...],
        *,
        timeout_seconds: float | None = None,
        allow_failure: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        try:
            return self._run(("shell", *arguments), timeout_seconds=timeout_seconds)
        except OSError:
            if not allow_failure:
                raise
            return subprocess.CompletedProcess([], 1, "", "shell command failed")

    def shell_script(
        self,
        script: str,
        *,
        timeout_seconds: float | None = None,
    ) -> subprocess.CompletedProcess[str]:
        return self.shell(
            ("sh", "-c", shlex.quote(script)),
            timeout_seconds=timeout_seconds,
        )

    def start_shell_script(self, script: str) -> subprocess.Popen[bytes]:
        return self.start_shell_stream(("sh", "-c", shlex.quote(script)))

    def start_shell_stream(self, arguments: tuple[str, ...]) -> subprocess.Popen[bytes]:
        command = self._command(("shell", *arguments))
        try:
            return subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except FileNotFoundError as exc:
            raise OSError(f"ADB executable was not found: {self._executable}") from exc

    def _run(
        self,
        arguments: tuple[str, ...],
        *,
        timeout_seconds: float | None = None,
    ) -> subprocess.CompletedProcess[str]:
        command = self._command(arguments)
        effective_timeout = self._timeout_seconds if timeout_seconds is None else timeout_seconds
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=effective_timeout,
            )
        except FileNotFoundError as exc:
            raise OSError(f"ADB executable was not found: {self._executable}") from exc
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError(f"ADB command timed out after {effective_timeout:g} seconds") from exc
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip() or "no diagnostic output"
            raise OSError(f"ADB command failed with status {completed.returncode}: {detail}")
        return completed

    def _command(self, arguments: tuple[str, ...]) -> list[str]:
        command = [self._executable]
        if self._device_serial is not None:
            command.extend(("-s", self._device_serial))
        command.extend(arguments)
        return command
