from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dog_patrol_perception_face.whitelist import load_whitelist_npy, match_embedding


def _write_whitelist(root: Path, entries: dict[str, list[np.ndarray]]) -> None:
    for name, embs in entries.items():
        person_dir = root / name
        person_dir.mkdir(parents=True)
        for i, emb in enumerate(embs):
            np.save(str(person_dir / f"{i}_100px.npy"), emb)


def test_load_whitelist_returns_normalized_embeddings(tmp_path: Path) -> None:
    emb = np.arange(8, dtype=np.float32) / 10.0
    _write_whitelist(tmp_path, {"alice": [emb, emb * 2]})
    database = load_whitelist_npy(str(tmp_path))
    assert set(database) == {"alice"}
    embeddings, sizes = database["alice"]
    assert embeddings.shape == (2, 8)
    assert np.allclose(np.linalg.norm(embeddings, axis=1), 1.0)
    assert np.allclose(sizes, [100.0, 100.0])


def test_load_whitelist_missing_dir_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="missing"):
        load_whitelist_npy(str(tmp_path / "nope"))


def test_load_whitelist_empty_dir_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="no usable"):
        load_whitelist_npy(str(tmp_path))


def test_load_whitelist_zero_norm_raises(tmp_path: Path) -> None:
    _write_whitelist(tmp_path, {"alice": [np.zeros(8, dtype=np.float32)]})
    with pytest.raises(ValueError, match="unusable"):
        load_whitelist_npy(str(tmp_path))


def test_match_embedding_accepts_close_probe() -> None:
    gallery = np.zeros(8, dtype=np.float32)
    gallery[0] = 1.0
    database = {"alice": (gallery[None, :], np.array([100.0]))}
    name, sim = match_embedding(gallery, database, 0.55, 100.0, 20.0)
    assert name == "alice"
    assert sim == pytest.approx(1.0)


def test_match_embedding_unknown_below_threshold() -> None:
    gallery = np.zeros(8, dtype=np.float32)
    gallery[0] = 1.0
    database = {"alice": (gallery[None, :], np.array([100.0]))}
    probe = np.zeros(8, dtype=np.float32)
    probe[1] = 1.0
    name, sim = match_embedding(probe, database, 0.55, 100.0, 20.0)
    assert name is None
    assert sim < 0.55


def test_match_embedding_size_weight_penalizes_distant_probe_size() -> None:
    gallery = np.zeros(8, dtype=np.float32)
    gallery[0] = 1.0
    database = {"alice": (gallery[None, :], np.array([100.0]))}
    name, sim = match_embedding(gallery, database, 0.55, 300.0, 20.0)
    assert name is None
    assert sim < 1.0


def test_match_embedding_zero_probe_returns_none() -> None:
    database = {"alice": (np.ones((1, 8), dtype=np.float32), np.array([100.0]))}
    name, sim = match_embedding(np.zeros(8), database, 0.55, None, None)
    assert name is None
    assert sim == 0.0
