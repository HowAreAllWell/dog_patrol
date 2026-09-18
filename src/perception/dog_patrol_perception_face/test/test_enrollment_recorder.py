from __future__ import annotations

from pathlib import Path

import pytest

from dog_patrol_perception_face.enrollment_recorder import (
    recording_codec,
    validate_output_path,
)


def test_recording_rejects_unsupported_container(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="must end in .mp4 or .avi"):
        validate_output_path(str(tmp_path / "person.mkv"), overwrite=False)


@pytest.mark.parametrize(
    ("filename", "codec"),
    (("person.mp4", "libx264"), ("person.avi", "FFV1")),
)
def test_recording_codec_follows_output_extension(
    tmp_path: Path,
    filename: str,
    codec: str,
) -> None:
    output = validate_output_path(str(tmp_path / filename), overwrite=False)

    assert recording_codec(output) == codec


def test_recording_refuses_to_overwrite_by_default(tmp_path: Path) -> None:
    output = tmp_path / "person.mp4"
    output.touch()

    with pytest.raises(FileExistsError, match="--overwrite"):
        validate_output_path(str(output), overwrite=False)


def test_recording_creates_parent_and_allows_explicit_overwrite(
    tmp_path: Path,
) -> None:
    output = tmp_path / "nested" / "person.mp4"

    resolved = validate_output_path(str(output), overwrite=True)

    assert resolved == output.resolve()
    assert output.parent.is_dir()
