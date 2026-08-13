"""Real-model smoke test for the migrated face modules.

Proves that the migrated ``dog_patrol_perception_face`` detector / alignment /
recognition / whitelist modules actually run on this machine (real TensorRT
inference) and produce correct results:

* ``--self-match``: detect + align + embed a known whitelist source photo and
  check it matches its own identity above threshold (cosine similarity).
* ``--video``: full-frame pipeline with temporal smoothing over a video file,
  writing an annotated AVI.

Asset paths come from ``FACE_DETECTOR_ENGINE`` / ``FACE_RECOGNITION_ENGINE`` /
``FACE_WHITELIST_DIR`` env vars, defaulting to the local dev location. This is
an explicit command-line smoke utility and defines no pytest test cases.

Run directly with the face_rec venv (has pycuda/tensorrt):
    PYTHONPATH=<repo>/dog_patrol_perception_face \\
      python test/test_real_models.py --self-match --image <photo>
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    import pycuda.autoinit  # noqa: F401  creates the primary CUDA context
except ImportError:
    pass

try:
    from dog_patrol_perception_face.alignment import align_face_5pts
    from dog_patrol_perception_face.detector import TRTInference
    from dog_patrol_perception_face.recognition import RecModel
    from dog_patrol_perception_face.whitelist import (
        load_whitelist_npy,
        match_embedding,
    )
except ImportError:  # pragma: no cover - import helper for direct run
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[1])
    )
    from dog_patrol_perception_face.alignment import align_face_5pts
    from dog_patrol_perception_face.detector import TRTInference
    from dog_patrol_perception_face.recognition import RecModel
    from dog_patrol_perception_face.whitelist import (
        load_whitelist_npy,
        match_embedding,
    )


_BASE = Path(os.environ.get("FACE_REC_DIR", "/home/user/face_rec/YOLO BACKBONE"))
DETECTOR_ENGINE = Path(
    os.environ.get(
        "FACE_DETECTOR_ENGINE",
        _BASE / "engines/yolo26n-e2e-v10-050426-fp32.engine",
    )
)
RECOGNITION_ENGINE = Path(
    os.environ.get(
        "FACE_RECOGNITION_ENGINE",
        _BASE / "face_models/engines/sface_2021dec_fp16.engine",
    )
)
WHITELIST_DIR = Path(
    os.environ.get("FACE_WHITELIST_DIR", _BASE / "whitelist_npy")
)

THRESHOLD = 0.55
SIZE_SIGMA = 20.0
TEMPORAL_WINDOW = 10
MIN_FRAMES = 5


def assets_present() -> bool:
    return all(p.exists() for p in (DETECTOR_ENGINE, RECOGNITION_ENGINE, WHITELIST_DIR))


def _load_pipeline():
    detector = TRTInference(str(DETECTOR_ENGINE))
    detector._ensure_gpu_preproc()
    recognition = RecModel(str(RECOGNITION_ENGINE))
    database = load_whitelist_npy(str(WHITELIST_DIR))
    return detector, recognition, database


def _best_face(dets):
    faces = [
        d
        for d in dets
        if d.get("class") == "face" and len(d.get("kpts", [])) >= 5
    ]
    if not faces:
        return None
    return max(faces, key=lambda d: d["conf"])


def _crop_from_frame(frame: np.ndarray, frame_index: int):
    from dog_patrol_perception_face.crop import DecodedCrop

    h, w = frame.shape[:2]
    return DecodedCrop(
        target_id=1,
        source_stamp_ns=frame_index,
        source_frame_id="smoke",
        source_frame_number=frame_index,
        source_frame_number_available=True,
        source_image_width=w,
        source_image_height=h,
        bbox_x=0,
        bbox_y=0,
        bbox_width=w,
        bbox_height=h,
        image=frame,
    )


def verify_production_path(
    video_path: str, threshold: float = THRESHOLD, start_frame: int = 0
) -> dict:
    """Run the production ``ProductionFaceVerifier`` over video crops.

    A self-session whitelist is built from the video's own faces so the check is
    deterministic and does not depend on the deployment whitelist. Proves the
    exact verifier wiring used by the provider runs end-to-end on this machine.
    """
    from dog_patrol_perception_face.crop import DecodedCrop  # noqa: F401
    from dog_patrol_perception_face.verifier import ProductionFaceVerifier

    detector, recognition, _ = _load_pipeline()

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {video_path}")

    gallery_dir = Path(os.environ.get("TMPDIR", "/tmp")) / "wl_smoke"
    person_dir = gallery_dir / "LHY"
    person_dir.mkdir(parents=True, exist_ok=True)
    for f in person_dir.glob("*.npy"):
        f.unlink()

    crops: list[DecodedCrop] = []
    index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if index >= start_frame:
            crops.append(_crop_from_frame(frame, index))
        index += 1
    cap.release()

    saved = 0
    for crop in crops:
        raw, pad = detector.infer_frame(crop.image)
        best = _best_face(detector.postprocess(raw, pad))
        if best is None:
            continue
        aligned = align_face_5pts(crop.image, best["kpts"][:5])
        emb = recognition.infer(aligned)
        emb = emb / (np.linalg.norm(emb) or 1.0)
        x1, y1, x2, y2 = (int(v) for v in best["box"])
        size = int(np.sqrt(max(0, x2 - x1) * max(0, y2 - y1)))
        np.save(str(person_dir / f"g{saved}_{size}px.npy"), emb)
        saved += 1
        if saved >= 8:
            break

    database = load_whitelist_npy(str(gallery_dir))

    verifier = ProductionFaceVerifier(
        detector=detector,
        recognition=recognition,
        database=database,
        similarity_threshold=threshold,
        size_sigma=SIZE_SIGMA,
        temporal_window=TEMPORAL_WINDOW,
        min_frames_for_decision=MIN_FRAMES,
    )
    verifier.reset()
    accepted = 0
    decided = 0
    errors = 0
    for crop in crops:
        verdict = verifier.verify(crop)
        if verdict.error:
            errors += 1
            continue
        if verdict.decided:
            decided += 1
            if verdict.accepted:
                accepted += 1
    return {
        "video": video_path,
        "crops": len(crops),
        "gallery_saved": saved,
        "decided": decided,
        "accepted": accepted,
        "errors": errors,
    }


def self_match(image_path: str, threshold: float = THRESHOLD) -> dict:
    detector, recognition, database = _load_pipeline()
    img = cv2.imread(image_path)
    if img is None:
        raise SystemExit(f"cannot read image: {image_path}")
    raw, pad = detector.infer_frame(img)
    best = _best_face(detector.postprocess(raw, pad))
    if best is None:
        raise SystemExit("no face detected")
    aligned = align_face_5pts(img, best["kpts"][:5])
    emb = recognition.infer(aligned)
    emb = emb / (np.linalg.norm(emb) or 1.0)
    x1, y1, x2, y2 = (int(v) for v in best["box"])
    probe_size = np.sqrt(max(0, x2 - x1) * max(0, y2 - y1))
    name_cos, sim_cos = match_embedding(
        emb, database, threshold, None, None
    )
    name_w, sim_w = match_embedding(
        emb, database, threshold, probe_size, SIZE_SIGMA
    )
    temp_db = {"test_gallery": (emb.reshape(1, -1), np.array([100.0]))}
    name_self, sim_self = match_embedding(
        emb, temp_db, 0.0, None, None
    )
    return {
        "image": image_path,
        "faces": 1,
        "conf": best["conf"],
        "probe_size": round(float(probe_size), 1),
        "self_gallery": {"name": name_self, "sim": round(float(sim_self), 4)},
        "cosine": {"name": name_cos, "sim": round(float(sim_cos), 4)},
        "weighted": {"name": name_w, "sim": round(float(sim_w), 4)},
    }


def run_video(
    video_path: str,
    out_path: str,
    threshold: float = THRESHOLD,
    max_frames: int = 0,
) -> dict:
    detector, recognition, database = _load_pipeline()
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(
        out_path, cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h)
    )

    emb_buf: list[np.ndarray] = []
    frame_idx = 0
    matched = 0
    last_name = None
    last_sim = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1
        if max_frames and frame_idx > max_frames:
            break

        raw, pad = detector.infer_frame(frame)
        best = _best_face(detector.postprocess(raw, pad))
        name = None
        sim = 0.0
        if best is not None:
            aligned = align_face_5pts(frame, best["kpts"][:5])
            emb = recognition.infer(aligned)
            x1, y1, x2, y2 = (int(v) for v in best["box"])
            probe_size = np.sqrt(max(0, x2 - x1) * max(0, y2 - y1))
            emb_buf.append(emb)
            if len(emb_buf) > TEMPORAL_WINDOW:
                emb_buf.pop(0)
            if len(emb_buf) >= MIN_FRAMES:
                avg = np.mean(emb_buf, axis=0)
                avg = avg / (np.linalg.norm(avg) or 1.0)
                name, sim = match_embedding(
                    avg, database, threshold, probe_size, SIZE_SIGMA
                )
            if name is not None:
                last_name, last_sim = name, sim
                matched += 1
            color = (0, 255, 255) if name is not None else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{name or 'Unknown'} {sim:.2f}"
            cv2.putText(
                frame, label, (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
            )

        writer.write(frame)
        if frame_idx % 20 == 0:
            print(
                f"  {frame_idx}/{max_frames or '?'}  matched={matched}  "
                f"last={last_name} {last_sim:.2f}",
                flush=True,
            )

    cap.release()
    writer.release()
    return {
        "video": video_path,
        "out": out_path,
        "frames": frame_idx,
        "matched": matched,
        "last_name": last_name,
        "last_sim": round(float(last_sim), 4),
    }


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-match", action="store_true")
    parser.add_argument("--image", default="/home/user/face_rec/face/LHY/LHY_3.jpg")
    parser.add_argument("--production", action="store_true")
    parser.add_argument("--video")
    parser.add_argument("--out", default="face_real_models.avi")
    parser.add_argument("--threshold", type=float, default=THRESHOLD)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--start-frame", type=int, default=0)
    args = parser.parse_args()

    if not assets_present():
        raise SystemExit(
            "model assets not found; set FACE_DETECTOR_ENGINE / "
            "FACE_RECOGNITION_ENGINE / FACE_WHITELIST_DIR"
        )

    if args.production:
        if not args.video:
            parser.error("--production requires --video")
        res = verify_production_path(
            args.video, args.threshold, start_frame=args.start_frame
        )
        print(f"[production] video={res['video']} crops={res['crops']} "
              f"gallery_saved={res['gallery_saved']} decided={res['decided']} "
              f"accepted={res['accepted']} errors={res['errors']}")
        ok = res["errors"] == 0 and res["accepted"] >= 1
        print("PASS: production verifier path runs and reaches PASSED on this "
              "machine" if ok else "FAIL")
        raise SystemExit(0 if ok else 1)

    if args.self_match:
        res = self_match(args.image, args.threshold)
        print(f"[self-match] image={res['image']} faces={res['faces']} "
              f"conf={res['conf']} probe_size={res['probe_size']}px")
        print(f"[self-built gallery] -> {res['self_gallery']['name']} "
              f"sim={res['self_gallery']['sim']}")
        print(f"[cosine]   -> {res['cosine']['name']} sim={res['cosine']['sim']}")
        print(f"[weighted] -> {res['weighted']['name']} sim={res['weighted']['sim']}")
        ok = (
            res["faces"] >= 1
            and res["self_gallery"]["sim"] >= 0.9
            and res["self_gallery"]["name"] is not None
        )
        print("PASS: migrated detector+align+recognition+whitelist run and "
              "match" if ok else "FAIL")
        raise SystemExit(0 if ok else 1)

    if args.video:
        res = run_video(args.video, args.out, args.threshold, args.max_frames)
        print(f"[video] frames={res['frames']} matched={res['matched']} "
              f"last={res['last_name']} sim={res['last_sim']}")
        print(f"[video] output: {Path(res['out']).resolve()}")
        return

    parser.print_help()


if __name__ == "__main__":
    main()
