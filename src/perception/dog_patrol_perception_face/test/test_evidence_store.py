from pathlib import Path

import numpy as np

from dog_patrol_perception_face.crop import DecodedCrop
from dog_patrol_perception_face.evidence_store import EvidencePhotoStore


def _crop(frame=7):
    return DecodedCrop(
        target_id=17,
        source_stamp_ns=123456789,
        source_frame_id="camera",
        source_frame_number=frame,
        source_frame_number_available=True,
        source_image_width=64,
        source_image_height=48,
        bbox_x=0,
        bbox_y=0,
        bbox_width=32,
        bbox_height=24,
        image=np.full((24, 32, 3), 127, dtype=np.uint8),
    )


def test_saves_atomic_file_url_with_traceable_name(tmp_path: Path):
    store = EvidencePhotoStore(
        enabled=True, root=str(tmp_path), retention_seconds=60, max_files=10
    )
    url = store.save(
        _crop(), state_seq=12, target_id=17,
        stage="initial_face", outcome="passed",
    )
    assert url.startswith("file:")
    saved = list(tmp_path.rglob("*.jpg"))
    assert [path.name for path in saved] == ["initial_face_passed_7.jpg"]
    assert not list(tmp_path.rglob("*.tmp"))


def test_disabled_store_does_not_write(tmp_path: Path):
    store = EvidencePhotoStore(
        enabled=False, root=str(tmp_path), retention_seconds=60, max_files=10
    )
    assert store.save(
        _crop(), state_seq=1, target_id=17,
        stage="initial_face", outcome="passed",
    ) == ""
    assert not list(tmp_path.iterdir())


def test_cleanup_is_bounded_and_preserves_unrelated_files(tmp_path: Path):
    unrelated = tmp_path / "keep.txt"
    unrelated.write_text("keep", encoding="utf-8")
    store = EvidencePhotoStore(
        enabled=True, root=str(tmp_path), retention_seconds=60, max_files=1
    )
    store.save(_crop(1), state_seq=1, target_id=17, stage="initial_face", outcome="passed")
    store.save(_crop(2), state_seq=1, target_id=17, stage="initial_face", outcome="passed")
    assert [path.name for path in tmp_path.rglob("*.jpg")] == [
        "initial_face_passed_2.jpg"
    ]
    assert unrelated.read_text(encoding="utf-8") == "keep"
