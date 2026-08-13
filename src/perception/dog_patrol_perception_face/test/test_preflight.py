from __future__ import annotations

import numpy as np

from dog_patrol_perception_face.preflight import (
    ERROR,
    NOT_READY,
    READY,
    FacePreflight,
)


def _write_engine(path, payload: bytes = b"engine") -> None:
    path.write_bytes(payload)


def _write_whitelist(path) -> None:
    (path / "alice").mkdir(parents=True)
    np.save(str(path / "alice" / "0_100px.npy"), np.ones(8, dtype=np.float32))


def test_all_inputs_present_is_ready(tmp_path) -> None:
    detector = tmp_path / "det.engine"
    recognition = tmp_path / "sface.engine"
    whitelist = tmp_path / "whitelist"
    _write_engine(detector)
    _write_engine(recognition)
    _write_whitelist(whitelist)

    loaded = []
    outcome = FacePreflight(
        detector_engine=str(detector),
        recognition_engine=str(recognition),
        whitelist_dir=str(whitelist),
        detector_loader=lambda path: loaded.append(("det", path)),
        recognition_loader=lambda path: loaded.append(("rec", path)),
    ).run()
    assert outcome.status == READY
    assert set(loaded) == {("det", str(detector)), ("rec", str(recognition))}


def test_preflight_exercises_and_closes_models_before_cuda_context_pop(
    tmp_path, monkeypatch
) -> None:
    detector_path = tmp_path / "det.engine"
    recognition_path = tmp_path / "sface.engine"
    whitelist = tmp_path / "whitelist"
    _write_engine(detector_path)
    _write_engine(recognition_path)
    _write_whitelist(whitelist)
    calls = []

    class Detector:
        def _ensure_gpu_preproc(self):
            calls.append("detector_preproc")

        def infer_frame(self, frame):
            assert frame.shape == (64, 64, 3)
            calls.append("detector_infer")

        def close(self):
            calls.append("detector_close")

    class Recognition:
        def infer(self, frame):
            assert frame.shape == (112, 112, 3)
            calls.append("recognition_infer")

        def close(self):
            calls.append("recognition_close")

    class Context:
        def pop(self):
            calls.append("context_pop")

    monkeypatch.setattr(
        "dog_patrol_perception_face.preflight._push_cuda_context",
        lambda: Context(),
    )
    outcome = FacePreflight(
        detector_engine=str(detector_path),
        recognition_engine=str(recognition_path),
        whitelist_dir=str(whitelist),
        detector_loader=lambda path: Detector(),
        recognition_loader=lambda path: Recognition(),
    ).run()

    assert outcome.status == READY
    assert calls == [
        "detector_preproc",
        "detector_infer",
        "detector_close",
        "recognition_infer",
        "recognition_close",
        "context_pop",
    ]


def test_missing_engine_is_not_ready(tmp_path) -> None:
    whitelist = tmp_path / "whitelist"
    _write_whitelist(whitelist)
    outcome = FacePreflight(
        detector_engine=str(tmp_path / "missing.engine"),
        recognition_engine=str(tmp_path / "sface.engine"),
        whitelist_dir=str(whitelist),
        detector_loader=lambda path: None,
        recognition_loader=lambda path: None,
    ).run()
    assert outcome.status == NOT_READY
    assert "missing" in outcome.diagnostic


def test_load_failure_is_error(tmp_path) -> None:
    detector = tmp_path / "det.engine"
    recognition = tmp_path / "sface.engine"
    whitelist = tmp_path / "whitelist"
    _write_engine(detector)
    _write_engine(recognition)
    _write_whitelist(whitelist)

    def fail_loader(path):
        raise RuntimeError("bad engine")

    outcome = FacePreflight(
        detector_engine=str(detector),
        recognition_engine=str(recognition),
        whitelist_dir=str(whitelist),
        detector_loader=fail_loader,
        recognition_loader=lambda path: None,
    ).run()
    assert outcome.status == ERROR
    assert "cannot be loaded" in outcome.diagnostic


def test_whitelist_load_failure_is_error(tmp_path) -> None:
    detector = tmp_path / "det.engine"
    recognition = tmp_path / "sface.engine"
    _write_engine(detector)
    _write_engine(recognition)
    whitelist = tmp_path / "whitelist"
    whitelist.mkdir()

    outcome = FacePreflight(
        detector_engine=str(detector),
        recognition_engine=str(recognition),
        whitelist_dir=str(whitelist),
        detector_loader=lambda path: None,
        recognition_loader=lambda path: None,
    ).run()
    assert outcome.status == ERROR
    assert "whitelist" in outcome.diagnostic


def test_unset_input_is_not_ready() -> None:
    outcome = FacePreflight(
        detector_engine=None,
        recognition_engine=None,
        whitelist_dir=None,
    ).run()
    assert outcome.status == NOT_READY
    assert "detector engine" in outcome.diagnostic
    assert "recognition engine" in outcome.diagnostic
    assert "whitelist directory" in outcome.diagnostic
