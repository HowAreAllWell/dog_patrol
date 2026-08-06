from __future__ import annotations

import hashlib
import struct
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
HELPER = PACKAGE_ROOT / "dog_patrol_perception_voice" / "assets" / "r818_pcm_base64_aarch64"
SOURCE = PACKAGE_ROOT / "tools" / "r818_pcm_base64.c"


def test_bundled_helper_is_the_verified_static_arm64_binary() -> None:
    data = HELPER.read_bytes()
    assert hashlib.sha256(data).hexdigest() == (
        "c2517d85e60845679acaeab4aa6c4f439b828393c5d73599dcef0e4fa68c0f52"
    )
    assert data[:4] == b"\x7fELF"
    assert data[4] == 2  # ELFCLASS64
    assert data[5] == 1  # little endian
    assert struct.unpack_from("<H", data, 16)[0] == 2  # executable
    assert struct.unpack_from("<H", data, 18)[0] == 183  # AArch64
    assert SOURCE.read_text(encoding="utf-8").startswith("#include <errno.h>")
