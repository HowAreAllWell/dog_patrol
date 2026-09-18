"""Face alignment helpers (adapted from ``yolo2.py``)."""

from __future__ import annotations

import numpy as np


def align_face_5pts(img_bgr, kpts_5, profile="legacy_sface"):
    """Align a 5-point face to 112x112.

    ``kpts_5`` is a list of 5 ``[x, y, conf]`` landmarks in model order:
    right eye (image-left), left eye (image-right), nose, right mouth
    (image-left), left mouth (image-right). MobileFaceNet's ArcFace template
    uses this order directly. The legacy SFace profile retains the historical
    eye/mouth swap so existing SFace embeddings remain compatible.
    """
    import cv2

    source_orders = {
        "legacy_sface": (1, 0, 2, 4, 3),
        "arcface_112": (0, 1, 2, 3, 4),
    }
    templates = {
        "legacy_sface": [
            [38.2946, 51.6963],
            [73.5318, 51.6963],
            [56.0252, 71.7366],
            [41.5493, 92.3655],
            [70.7299, 92.3655],
        ],
        "arcface_112": [
            [38.2946, 51.6963],
            [73.5318, 51.5014],
            [56.0252, 71.7366],
            [41.5493, 92.3655],
            [70.7299, 92.2041],
        ],
    }
    if profile not in templates:
        raise ValueError(f"unsupported face alignment profile: {profile!r}")
    src = np.float32(
        [
            [kpts_5[index][0], kpts_5[index][1]]
            for index in source_orders[profile]
        ]
    )
    dst = np.float32(templates[profile])
    matrix, _ = cv2.estimateAffinePartial2D(src, dst)
    if matrix is not None:
        return cv2.warpAffine(
            img_bgr, matrix, (112, 112), borderMode=cv2.BORDER_CONSTANT
        )
    h, w = img_bgr.shape[:2]
    s = min(h, w)
    y0 = (h - s) // 2
    x0 = (w - s) // 2
    return cv2.resize(
        img_bgr[y0:y0 + s, x0:x0 + s],
        (112, 112),
    )
