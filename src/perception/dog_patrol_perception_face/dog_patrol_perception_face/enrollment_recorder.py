"""Record a ROS Image topic as an enrollment video."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import time


def validate_output_path(path: str, overwrite: bool) -> Path:
    output = Path(path).expanduser().resolve()
    if output.suffix.lower() not in {".avi", ".mp4"}:
        raise ValueError("enrollment recording output must end in .mp4 or .avi")
    if output.exists() and not overwrite:
        raise FileExistsError(
            f"recording already exists: {output}; use --overwrite to replace it"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def recording_codec(output: Path) -> str:
    return "libx264" if output.suffix.lower() == ".mp4" else "FFV1"


class H264Writer:
    def __init__(self, output: Path, fps: float, size: tuple[int, int]):
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg is required to record H.264 MP4")
        width, height = size
        self._process = subprocess.Popen(
            [
                ffmpeg,
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s:v",
                f"{width}x{height}",
                "-r",
                f"{fps:.6f}",
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-crf",
                "18",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(output),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

    def is_opened(self) -> bool:
        return self._process.poll() is None and self._process.stdin is not None

    def write(self, frame) -> None:
        if self._process.stdin is None or self._process.poll() is not None:
            raise RuntimeError("H.264 encoder stopped before recording completed")
        self._process.stdin.write(frame.tobytes())

    def release(self) -> None:
        if self._process.stdin is not None:
            self._process.stdin.close()
            self._process.stdin = None
        stderr = self._process.stderr.read() if self._process.stderr is not None else b""
        return_code = self._process.wait()
        if return_code != 0:
            detail = stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(
                f"H.264 encoder failed with exit code {return_code}: {detail}"
            )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record a ROS Image topic as H.264 MP4 or lossless AVI."
    )
    parser.add_argument(
        "--topic",
        default="/left_camera/image_raw",
        help="sensor_msgs/msg/Image topic",
    )
    parser.add_argument("--output", required=True, help="output .mp4 or .avi path")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main() -> None:
    args, ros_args = _parser().parse_known_args()
    if args.fps <= 0:
        raise ValueError("fps must be positive")
    if args.duration <= 0:
        raise ValueError("duration must be positive")
    output = validate_output_path(args.output, args.overwrite)
    codec = recording_codec(output)

    import cv2
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class EnrollmentRecorder(Node):
        def __init__(self):
            super().__init__("face_enrollment_recorder")
            self.bridge = CvBridge()
            self.writer = None
            self.frame_size = None
            self.frame_count = 0
            self.started_at = None
            self.done = False
            self.subscription = self.create_subscription(
                Image,
                args.topic,
                self._on_image,
                qos_profile_sensor_data,
            )

        def _on_image(self, message: Image) -> None:
            try:
                frame = self.bridge.imgmsg_to_cv2(
                    message,
                    desired_encoding="bgr8",
                )
            except Exception as exception:
                self.get_logger().error(f"cannot decode camera image: {exception}")
                return
            height, width = frame.shape[:2]
            size = (width, height)
            if self.writer is None:
                if codec == "libx264":
                    self.writer = H264Writer(output, args.fps, size)
                    opened = self.writer.is_opened()
                else:
                    self.writer = cv2.VideoWriter(
                        str(output),
                        cv2.VideoWriter_fourcc(*codec),
                        args.fps,
                        size,
                    )
                    opened = self.writer.isOpened()
                if not opened:
                    self.done = True
                    raise RuntimeError(f"cannot open enrollment video: {output}")
                self.frame_size = size
                self.started_at = time.monotonic()
                self.get_logger().info(
                    f"recording {width}x{height} BGR with {codec} at "
                    f"{args.fps:.2f} FPS to {output}"
                )
            if size != self.frame_size:
                self.get_logger().error(
                    f"camera resolution changed from {self.frame_size} to {size}; stopping"
                )
                self.done = True
                return
            self.writer.write(frame)
            self.frame_count += 1
            if time.monotonic() - self.started_at >= args.duration:
                self.done = True

        def close(self) -> None:
            if self.writer is not None:
                self.writer.release()
                self.writer = None

    rclpy.init(args=ros_args)
    node = EnrollmentRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        close_error = None
        try:
            node.close()
        except Exception as exception:
            close_error = exception
        frame_count = node.frame_count
        node.destroy_node()
        rclpy.shutdown()
        if close_error is not None:
            raise close_error
    if frame_count == 0:
        raise RuntimeError(f"no images received from {args.topic}")
    print(f"recorded {frame_count} frames to {output}")


if __name__ == "__main__":
    main()
