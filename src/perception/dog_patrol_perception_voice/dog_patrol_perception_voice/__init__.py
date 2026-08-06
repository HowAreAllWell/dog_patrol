"""Portable task-scoped R818/Vosk voice verification core."""

from .adapter import (
    R818TaskSession,
    R818VoiceAdapter,
    VoiceTaskCancelled,
    VoiceTaskCleanupError,
)
from .adb import SubprocessAdbFileTransfer
from .config import VoiceConfig, default_config, load_voice_config
from .prompt import FfmpegAlsaPromptPlayer
from .preflight import (
    DEFAULT_HELPER_SHA256,
    ERROR,
    NOT_READY,
    READY,
    VoicePreflight,
    VoicePreflightOutcome,
    default_helper_path,
)
from .readiness import VoiceReadinessController
from .r818_stream import (
    R818HardwareUnreadyError,
    R818StreamingVoskSession,
    SubprocessAdbEncodedPcmStream,
    decode_base64_pcm_chunks,
)
from .result import VoiceWindowResult

__all__ = [
    "R818TaskSession",
    "R818VoiceAdapter",
    "VoiceTaskCancelled",
    "VoiceTaskCleanupError",
    "R818HardwareUnreadyError",
    "R818StreamingVoskSession",
    "SubprocessAdbEncodedPcmStream",
    "SubprocessAdbFileTransfer",
    "VoiceConfig",
    "VoiceWindowResult",
    "FfmpegAlsaPromptPlayer",
    "default_config",
    "decode_base64_pcm_chunks",
    "load_voice_config",
    "DEFAULT_HELPER_SHA256",
    "ERROR",
    "NOT_READY",
    "READY",
    "VoicePreflight",
    "VoicePreflightOutcome",
    "VoiceReadinessController",
    "default_helper_path",
]
