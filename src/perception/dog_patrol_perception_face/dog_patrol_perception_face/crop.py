"""Validation and decoding of ``TrackedTargetImage`` crops.

Implements the input contract from ``COLLABORATION.md``: validate encoding,
dimensions, step, data length, positive ``target_id``, bbox and source image
dimensions. Invalid messages are dropped and reported; they never raise out of
the consumer callback.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EXPECTED_ENCODING = "bgr8"
BGR_CHANNELS = 3

_DROP_REASON_EMPTY_ENCODING = "encoding must not be empty"
_DROP_REASON_ENCODING = f"unsupported encoding, expected {EXPECTED_ENCODING}"
_DROP_REASON_DIMENSION = "crop dimensions must be positive"
_DROP_REASON_STEP = "crop_step must be at least width * 3"
_DROP_REASON_LENGTH = "crop_data length does not match step * height"
_DROP_REASON_TARGET = "target_id must be positive"
_DROP_REASON_BBOX = "bbox must be inside the source image"
_DROP_REASON_SOURCE = "source image dimensions must be positive"
_DROP_REASON_OVERSIZE = "crop exceeds the configured maximum size"


@dataclass(frozen=True)
class DecodedCrop:
    """Validated, decoded view of a single tracked target crop."""

    target_id: int
    source_stamp_ns: int
    source_frame_id: str
    source_frame_number: int
    source_frame_number_available: bool
    source_image_width: int
    source_image_height: int
    bbox_x: int
    bbox_y: int
    bbox_width: int
    bbox_height: int
    image: np.ndarray


def decode_crop(message, max_crop_bytes=8 * 1024 * 1024):
    """Validate and decode a ``TrackedTargetImage`` message.

    Returns a ``DecodedCrop`` or ``(None, reason)`` when the message is
    invalid. The caller drops the crop and records a bounded diagnostic.
    """
    encoding = str(getattr(message, "encoding", "") or "").strip()
    if not encoding:
        return None, _DROP_REASON_EMPTY_ENCODING
    if encoding != EXPECTED_ENCODING:
        return None, _DROP_REASON_ENCODING

    crop_width = int(message.crop_width)
    crop_height = int(message.crop_height)
    crop_step = int(message.crop_step)
    if crop_width <= 0 or crop_height <= 0:
        return None, _DROP_REASON_DIMENSION
    min_step = crop_width * BGR_CHANNELS
    if crop_step < min_step:
        return None, _DROP_REASON_STEP

    data = message.crop_data
    expected_length = crop_step * crop_height
    if len(data) != expected_length:
        return None, _DROP_REASON_LENGTH
    if expected_length > max_crop_bytes:
        return None, _DROP_REASON_OVERSIZE

    target_id = int(message.target_id)
    if target_id <= 0:
        return None, _DROP_REASON_TARGET

    src_w = int(message.source_image_width)
    src_h = int(message.source_image_height)
    if src_w <= 0 or src_h <= 0:
        return None, _DROP_REASON_SOURCE
    bbox_x = int(message.bbox_x)
    bbox_y = int(message.bbox_y)
    bbox_w = int(message.bbox_width)
    bbox_h = int(message.bbox_height)
    if (
        bbox_w <= 0
        or bbox_h <= 0
        or bbox_x < 0
        or bbox_y < 0
        or bbox_x + bbox_w > src_w
        or bbox_y + bbox_h > src_h
    ):
        return None, _DROP_REASON_BBOX

    raw = np.frombuffer(bytes(data), dtype=np.uint8)
    if crop_step == min_step:
        image = raw.reshape((crop_height, crop_width, BGR_CHANNELS))
    else:
        padded = raw.reshape((crop_height, crop_step))
        image = padded[:, :min_step].reshape(
            (crop_height, crop_width, BGR_CHANNELS)
        )
    image = np.ascontiguousarray(image)

    return (
        DecodedCrop(
            target_id=target_id,
            source_stamp_ns=_stamp_ns(message.source_stamp),
            source_frame_id=str(message.source_frame_id),
            source_frame_number=int(message.source_frame_number),
            source_frame_number_available=bool(
                message.source_frame_number_available
            ),
            source_image_width=src_w,
            source_image_height=src_h,
            bbox_x=bbox_x,
            bbox_y=bbox_y,
            bbox_width=bbox_w,
            bbox_height=bbox_h,
            image=image,
        ),
        None,
    )


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
