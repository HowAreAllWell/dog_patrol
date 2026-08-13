"""Face alignment helpers (adapted from ``yolo2.py``)."""

from __future__ import annotations

import numpy as np


def align_face_5pts(img_bgr, kpts_5):
    """Align a 5-point face to 112x112.

    ``kpts_5`` is a list of 5 ``[x, y, conf]`` landmarks in YOLO order:
    right eye, left eye, nose, right mouth, left mouth. Returns a 112x112
    aligned BGR face.
    """
    import cv2

    src = np.float32(
        [
            [kpts_5[1][0], kpts_5[1][1]],  # left eye
            [kpts_5[0][0], kpts_5[0][1]],  # right eye
            [kpts_5[2][0], kpts_5[2][1]],  # nose
            [kpts_5[4][0], kpts_5[4][1]],  # left mouth
            [kpts_5[3][0], kpts_5[3][1]],  # right mouth
        ]
    )
    dst = np.float32(
        [
            [38.2946, 51.6963],
            [73.5318, 51.6963],
            [56.0252, 71.7366],
            [41.5493, 92.3655],
            [70.7299, 92.3655],
        ]
    )
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
