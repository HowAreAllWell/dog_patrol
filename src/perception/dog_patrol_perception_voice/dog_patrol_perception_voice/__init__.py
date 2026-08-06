"""Portable task-scoped R818/Vosk voice verification core."""

from .adapter import R818TaskSession, R818VoiceAdapter
from .adb import SubprocessAdbFileTransfer
from .config import VoiceConfig, default_config, load_voice_config
from .prompt import FfmpegAlsaPromptPlayer
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
]
