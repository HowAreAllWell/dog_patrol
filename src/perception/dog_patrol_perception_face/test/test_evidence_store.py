import os
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
    assert [path.name for path in saved] == [
        "123456789_target-17_initial_face_passed_f7.jpg"
    ]
    assert not list(tmp_path.rglob("*.tmp"))


def test_restart_collision_keeps_both_photos(tmp_path: Path):
    first = EvidencePhotoStore(
        enabled=True, root=str(tmp_path), retention_seconds=60, max_files=10
    )
    second = EvidencePhotoStore(
        enabled=True, root=str(tmp_path), retention_seconds=60, max_files=10
    )
    first_url = first.save(
        _crop(), state_seq=12, target_id=17,
        stage="initial_face", outcome="not_passed",
    )
    second_url = second.save(
        _crop(), state_seq=12, target_id=17,
        stage="initial_face", outcome="not_passed",
    )
    assert first_url != second_url
    assert len(list(tmp_path.rglob("*.jpg"))) == 2


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
    latest_url = store.save(
        _crop(2), state_seq=1, target_id=17,
        stage="initial_face", outcome="passed",
    )
    assert [path.name for path in tmp_path.rglob("*.jpg")] == [
        "123456789_target-17_initial_face_passed_f2.jpg"
    ]
    assert latest_url == next(tmp_path.rglob("*.jpg")).resolve().as_uri()
    assert unrelated.read_text(encoding="utf-8") == "keep"


def test_cleanup_keeps_just_saved_photo_when_mtimes_are_equal(
    tmp_path: Path, monkeypatch
):
    fixed_ns = 1_800_000_000_000_000_000
    real_link = os.link

    def link_with_fixed_mtime(source, destination):
        real_link(source, destination)
        os.utime(destination, ns=(fixed_ns, fixed_ns))

    monkeypatch.setattr(os, "link", link_with_fixed_mtime)
    store = EvidencePhotoStore(
        enabled=True, root=str(tmp_path), retention_seconds=60, max_files=1
    )
    first_url = store.save(
        _crop(1), state_seq=1, target_id=17,
        stage="initial_face", outcome="passed",
    )
    latest_url = store.save(
        _crop(2), state_seq=1, target_id=17,
        stage="initial_face", outcome="passed",
    )

    saved = list(tmp_path.rglob("*.jpg"))
    assert len(saved) == 1
    assert saved[0].name.endswith("_f2.jpg")
    assert latest_url == saved[0].resolve().as_uri()
    assert first_url != latest_url
