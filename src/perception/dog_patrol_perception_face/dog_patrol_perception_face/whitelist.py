"""Face whitelist loading and matching (adapted from ``yolo-rec.py``)."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np


def load_whitelist_npy(whitelist_dir):
    """Load pre-computed ``.npy`` embeddings.

    Returns ``{name: (embeddings: (N, D), sizes: (N,))}`` where the per-file
    face size in pixels is parsed from the filename (``...NNNpx.npy``).
    Raises ``ValueError`` when the directory is missing, empty, or contains an
    unusable embedding.
    """
    root = Path(whitelist_dir)
    if not root.is_dir():
        raise ValueError(f"whitelist directory is missing: {root}")
    database = {}
    embedding_dimension = None
    for entry in sorted(p.name for p in root.iterdir() if p.is_dir()):
        person_path = root / entry
        embs = []
        sizes = []
        for f in sorted(p.name for p in person_path.iterdir()):
            if not f.endswith(".npy"):
                continue
            emb = np.load(str(person_path / f))
            emb = np.asarray(emb, dtype=np.float32).reshape(-1)
            norm = np.linalg.norm(emb)
            if norm == 0 or not np.isfinite(norm):
                raise ValueError(f"whitelist embedding is unusable: {entry}/{f}")
            emb = emb / norm
            m = re.search(r"(\d+)px", f)
            sizes.append(float(m.group(1)) if m else 0.0)
            embs.append(emb)
        if embs:
            if len({len(e) for e in embs}) != 1:
                raise ValueError(
                    f"whitelist embeddings have inconsistent dimensions: {entry}"
                )
            person_dimension = len(embs[0])
            if embedding_dimension is None:
                embedding_dimension = person_dimension
            elif person_dimension != embedding_dimension:
                raise ValueError(
                    "whitelist embeddings have inconsistent dimensions across "
                    f"identities: expected {embedding_dimension}, "
                    f"{entry} has {person_dimension}"
                )
            database[entry] = (np.stack(embs), np.array(sizes, dtype=np.float32))
    if not database:
        raise ValueError(f"whitelist directory contains no usable embeddings: {root}")
    return database


def match_embedding(emb, database, threshold, probe_size=None, size_sigma=30.0):
    """Match ``emb`` against the whitelist with size-weighted cosine similarity.

    Returns ``(matched_name | None, best_similarity)``. ``None`` means no entry
    exceeded ``threshold``.
    """
    emb = np.asarray(emb, dtype=np.float32)
    n = np.linalg.norm(emb)
    if n == 0:
        return None, 0.0
    emb = emb / n
    best_name = None
    best_sim = 0.0
    for name, (gallery, sizes) in database.items():
        if gallery.ndim != 2 or gallery.shape[1] != emb.size:
            raise ValueError(
                f"probe embedding dimension {emb.size} does not match "
                f"whitelist dimension {gallery.shape[-1]} for {name}"
            )
        sims = gallery @ emb
        if (
            probe_size is not None
            and size_sigma is not None
            and sizes.sum() > 0
        ):
            weight = np.exp(-0.5 * ((sizes - probe_size) / size_sigma) ** 2)
            sims = sims * weight
        max_sim = float(np.max(sims))
        if max_sim > best_sim:
            best_sim = max_sim
            best_name = name
    if best_name is None or best_sim < threshold:
        return None, best_sim
    return best_name, best_sim
