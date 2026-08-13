from __future__ import annotations

from dog_patrol_perception_interfaces.msg import TrackedTargetImage

from dog_patrol_perception_face.crop import decode_crop


def _valid_message():
    msg = TrackedTargetImage()
    msg.encoding = "bgr8"
    msg.crop_width = 32
    msg.crop_height = 24
    msg.crop_step = 32 * 3
    msg.crop_data = bytes(32 * 3 * 24)
    msg.target_id = 7
    msg.source_image_width = 640
    msg.source_image_height = 480
    msg.bbox_x = 10
    msg.bbox_y = 20
    msg.bbox_width = 32
    msg.bbox_height = 24
    msg.source_stamp.sec = 1
    msg.source_stamp.nanosec = 2
    msg.source_frame_id = "camera_optical"
    msg.source_frame_number = 99
    msg.source_frame_number_available = True
    return msg


def test_valid_crop_decodes() -> None:
    crop, reason = decode_crop(_valid_message())
    assert reason is None
    assert crop is not None
    assert crop.target_id == 7
    assert crop.image.shape == (24, 32, 3)
    assert crop.source_stamp_ns == 1_000_000_002
    assert crop.source_frame_id == "camera_optical"
    assert crop.source_frame_number == 99
    assert crop.source_frame_number_available is True
    assert (crop.bbox_x, crop.bbox_y, crop.bbox_width, crop.bbox_height) == (
        10,
        20,
        32,
        24,
    )


def test_blank_encoding_is_dropped() -> None:
    msg = _valid_message()
    msg.encoding = ""
    crop, reason = decode_crop(msg)
    assert crop is None and "encoding" in reason


def test_unsupported_encoding_is_dropped() -> None:
    msg = _valid_message()
    msg.encoding = "rgb8"
    crop, reason = decode_crop(msg)
    assert crop is None and "encoding" in reason


def test_nonpositive_dimensions_are_dropped() -> None:
    msg = _valid_message()
    msg.crop_width = 0
    crop, reason = decode_crop(msg)
    assert crop is None and "dimensions" in reason


def test_step_smaller_than_row_is_dropped() -> None:
    msg = _valid_message()
    msg.crop_step = 32 * 3 - 1
    crop, reason = decode_crop(msg)
    assert crop is None and "step" in reason


def test_length_mismatch_is_dropped() -> None:
    msg = _valid_message()
    msg.crop_data = bytes(32 * 3 * 24 - 1)
    crop, reason = decode_crop(msg)
    assert crop is None and "length" in reason


def test_nonpositive_target_id_is_dropped() -> None:
    msg = _valid_message()
    msg.target_id = 0
    crop, reason = decode_crop(msg)
    assert crop is None and "target_id" in reason


def test_bbox_outside_source_is_dropped() -> None:
    msg = _valid_message()
    msg.bbox_x = 600
    msg.bbox_width = 100
    crop, reason = decode_crop(msg)
    assert crop is None and "bbox" in reason


def test_oversized_crop_is_dropped() -> None:
    msg = _valid_message()
    msg.crop_width = 64
    msg.crop_height = 64
    msg.crop_step = 64 * 3
    msg.crop_data = bytes(64 * 3 * 64)
    crop, reason = decode_crop(msg, max_crop_bytes=1024)
    assert crop is None and "maximum" in reason


def test_padded_step_uses_only_row_bytes() -> None:
    msg = _valid_message()
    msg.crop_step = 32 * 3 + 4
    msg.crop_data = bytes((32 * 3 + 4) * 24)
    crop, reason = decode_crop(msg)
    assert reason is None
    assert crop is not None
    assert crop.image.shape == (24, 32, 3)
    assert crop.image.flags["C_CONTIGUOUS"]
