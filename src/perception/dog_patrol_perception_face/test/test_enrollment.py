from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dog_patrol_perception_face.enrollment import (
    EnrollmentSample,
    build_video_sample_plan,
    install_identity_templates,
    select_scale_distributed_samples,
    validate_holdout_samples,
    validate_identity_name,
)
from dog_patrol_perception_face.whitelist import load_whitelist_npy


def _sample(frame: int, size: float, axis: int = 0) -> EnrollmentSample:
    embedding = np.zeros(8, dtype=np.float32)
    embedding[axis] = 1.0
    return EnrollmentSample(
        source_label=f"frame-{frame}",
        source_index=frame,
        embedding=embedding,
        face_size=size,
        confidence=0.9,
        brightness=120.0,
        sharpness=100.0,
    )


@pytest.mark.parametrize("name", ["", ".", "..", "a/b", "a\\b", " name "])
def test_validate_identity_name_rejects_unsafe_names(name: str) -> None:
    with pytest.raises(ValueError):
        validate_identity_name(name)


def test_validate_identity_name_accepts_deployment_identity() -> None:
    assert validate_identity_name("zby") == "zby"
    assert validate_identity_name("张三") == "张三"


def test_video_sample_plan_keeps_holdouts_out_of_registration_candidates() -> None:
    candidates, holdouts = build_video_sample_plan(
        frame_count=315,
        sample_stride=5,
        holdout_count=6,
    )

    assert len(holdouts) == 6
    assert candidates
    assert not set(candidates) & set(holdouts)
    assert all(
        abs(candidate - holdout) > 2
        for candidate in candidates
        for holdout in holdouts
    )


def test_video_sample_plan_caps_millisecond_timebase_work() -> None:
    candidates, holdouts = build_video_sample_plan(
        frame_count=10583,
        sample_stride=5,
        holdout_count=6,
        max_candidate_count=120,
    )

    assert len(candidates) == 120
    assert len(holdouts) == 6
    assert not set(candidates) & set(holdouts)
    assert candidates[0] < candidates[-1]


def test_scale_distributed_selection_covers_small_and_large_faces() -> None:
    samples = [_sample(index, float(size)) for index, size in enumerate(range(100, 401, 10))]

    selected = select_scale_distributed_samples(samples, template_count=6)

    assert len(selected) == 6
    assert selected[0].face_size == 100.0
    assert selected[-1].face_size == 400.0
    assert [sample.face_size for sample in selected] == sorted(
        sample.face_size for sample in selected
    )


def test_holdout_validation_uses_full_database_and_rejects_competitor() -> None:
    templates = [_sample(1, 100.0, axis=0)]
    holdouts = [_sample(2, 100.0, axis=1)]
    competitor = np.zeros((1, 8), dtype=np.float32)
    competitor[0, 1] = 1.0
    existing = {"other": (competitor, np.array([100.0], dtype=np.float32))}

    with pytest.raises(ValueError, match="other"):
        validate_holdout_samples(
            identity="zby",
            templates=templates,
            holdouts=holdouts,
            existing_database=existing,
            threshold=0.55,
            size_sigma=20.0,
        )


def test_install_templates_is_loadable_and_requires_replace_for_existing_identity(
    tmp_path: Path,
) -> None:
    whitelist = tmp_path / "whitelist"
    whitelist.mkdir()
    samples = [_sample(10, 101.4), _sample(20, 155.6)]

    destination, backup = install_identity_templates(
        whitelist_dir=whitelist,
        identity="zby",
        samples=samples,
        replace=False,
    )

    assert destination == whitelist / "zby"
    assert backup is None
    assert sorted(path.name for path in destination.glob("*.npy")) == [
        "g0_101px_f10.npy",
        "g1_156px_f20.npy",
    ]
    database = load_whitelist_npy(str(whitelist))
    assert database["zby"][0].shape == (2, 8)

    with pytest.raises(FileExistsError, match="--replace"):
        install_identity_templates(
            whitelist_dir=whitelist,
            identity="zby",
            samples=[_sample(30, 200.0)],
            replace=False,
        )

    destination, backup = install_identity_templates(
        whitelist_dir=whitelist,
        identity="zby",
        samples=[_sample(30, 200.0)],
        replace=True,
    )
    assert destination.is_dir()
    assert backup is not None and backup.is_dir()
    assert backup.parent.parent == tmp_path / "whitelist_backups"
    assert len(list(destination.glob("*.npy"))) == 1


def test_install_templates_accepts_a_one_shot_sample_iterable(tmp_path: Path) -> None:
    whitelist = tmp_path / "whitelist"
    samples = (_sample(frame, size) for frame, size in ((1, 100.0), (2, 120.0)))

    destination, _ = install_identity_templates(
        whitelist_dir=whitelist,
        identity="zby",
        samples=samples,
        replace=False,
    )

    assert len(list(destination.glob("*.npy"))) == 2
