"""Read-only deployment preflight for the production face capability."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .whitelist import load_whitelist_npy


NOT_READY = 0
READY = 1
ERROR = 2


@dataclass(frozen=True)
class FacePreflightOutcome:
    status: int
    diagnostic: str
    startup_state_seq: int | None = None


DetectorLoader = Callable[[str], object]
RecognitionLoader = Callable[[str], object]
WhitelistLoader = Callable[[str], dict]


def _load_detector(engine_path: str) -> object:
    from .detector import TRTInference

    return TRTInference(engine_path)


def _load_recognition(engine_path: str) -> object:
    from .recognition import RecModel

    return RecModel(engine_path)


def _push_cuda_context():
    """Create and push a PyCUDA context on the current thread, if needed.

    TensorRT engine construction requires a current CUDA context; without one
    TRTInference/RecModel fail with "invalid device context". Returns the
    pushed context (caller must pop) or ``None`` when no context was needed.
    """
    from .detector import ensure_cuda_context

    return ensure_cuda_context()


class FacePreflight:
    """Check face deployment inputs without running a verification session.

    Loads both TensorRT engines and the whitelist once to prove they are
    usable, but never consumes crops or publishes evidence.
    """

    def __init__(
        self,
        *,
        detector_engine: str | None,
        recognition_engine: str | None,
        whitelist_dir: str | None,
        detector_loader: DetectorLoader = _load_detector,
        recognition_loader: RecognitionLoader = _load_recognition,
        whitelist_loader: WhitelistLoader = load_whitelist_npy,
    ) -> None:
        self.detector_engine = detector_engine
        self.recognition_engine = recognition_engine
        self.whitelist_dir = whitelist_dir
        self.detector_loader = detector_loader
        self.recognition_loader = recognition_loader
        self.whitelist_loader = whitelist_loader

    def run(self) -> FacePreflightOutcome:
        not_ready: list[str] = []
        errors: list[str] = []

        self._check_input("detector engine", self.detector_engine, not_ready)
        self._check_input("recognition engine", self.recognition_engine, not_ready)
        self._check_input("whitelist directory", self.whitelist_dir, not_ready)

        if not not_ready:
            ctx = _push_cuda_context()
            try:
                self._check_detector(errors)
                self._check_recognition(errors)
                self._check_whitelist(errors)
            finally:
                if ctx is not None:
                    ctx.pop()

        if errors:
            return FacePreflightOutcome(ERROR, "; ".join(errors + not_ready))
        if not_ready:
            return FacePreflightOutcome(NOT_READY, "; ".join(not_ready))
        return FacePreflightOutcome(READY, "face preflight ready")

    @staticmethod
    def _check_input(name: str, value: str | None, not_ready: list[str]) -> None:
        if value is None or not Path(value).exists():
            not_ready.append(f"{name} is missing or unset: {value or '<unset>'}")

    def _check_detector(self, errors: list[str]) -> None:
        detector = None
        try:
            detector = self.detector_loader(self.detector_engine)
            ensure_preproc = getattr(detector, "_ensure_gpu_preproc", None)
            if callable(ensure_preproc):
                ensure_preproc()
            infer = getattr(detector, "infer_frame", None)
            if callable(infer):
                infer(np.zeros((64, 64, 3), dtype=np.uint8))
        except Exception as exc:
            errors.append(f"face detector engine cannot be loaded: {_exception_detail(exc)}")
        finally:
            self._close_resource("detector", detector, errors)

    def _check_recognition(self, errors: list[str]) -> None:
        recognition = None
        try:
            recognition = self.recognition_loader(self.recognition_engine)
            infer = getattr(recognition, "infer", None)
            if callable(infer):
                infer(np.zeros((112, 112, 3), dtype=np.uint8))
        except Exception as exc:
            errors.append(
                f"face recognition engine cannot be loaded: {_exception_detail(exc)}"
            )
        finally:
            self._close_resource("recognition", recognition, errors)

    def _check_whitelist(self, errors: list[str]) -> None:
        try:
            self.whitelist_loader(self.whitelist_dir)
        except Exception as exc:
            errors.append(f"face whitelist cannot be loaded: {_exception_detail(exc)}")

    @staticmethod
    def _close_resource(name: str, resource: object, errors: list[str]) -> None:
        close = getattr(resource, "close", None)
        if not callable(close):
            return
        try:
            close()
        except Exception as exc:
            errors.append(
                f"face {name} resource cannot be closed: {_exception_detail(exc)}"
            )


def _exception_detail(exc: Exception) -> str:
    detail = str(exc).strip()
    return f"{type(exc).__name__}: {detail}" if detail else type(exc).__name__


__all__ = [
    "ERROR",
    "NOT_READY",
    "READY",
    "FacePreflight",
    "FacePreflightOutcome",
]
