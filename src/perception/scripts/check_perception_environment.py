#!/usr/bin/env python3
"""Preflight the complete, currently implemented perception deployment scope."""

from __future__ import annotations

import argparse
import ast
import ctypes
from dataclasses import dataclass
import glob
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
from typing import Dict, Optional, Sequence, Tuple


@dataclass(frozen=True)
class VersionPolicy:
    supported_prefix: str
    exact: Optional[str] = None


@dataclass(frozen=True)
class TargetPolicy:
    versions: Dict[str, VersionPolicy]
    verification_note: Optional[str] = None


TARGET_POLICIES = {
    "perception-orin": TargetPolicy(
        versions={
            "jetpack": VersionPolicy("6.2", "6.2.1"),
            "l4t": VersionPolicy("R36.4.", "R36.4.7"),
            "cuda": VersionPolicy("12.6", "12.6"),
            "tensorrt": VersionPolicy("10.3", "10.3"),
            "mvs": VersionPolicy("0x0408", "0x04080003"),
        }
    ),
    "navigation-orin": TargetPolicy(
        versions={
            "jetpack": VersionPolicy("6.2"),
            "l4t": VersionPolicy("R36.4."),
            "cuda": VersionPolicy("12.6"),
            "tensorrt": VersionPolicy("10.3"),
            "mvs": VersionPolicy("0x0408"),
        },
        verification_note="navigation Orin exact versions are pending on-device verification",
    ),
}
SUPPORTED_TARGETS = tuple(TARGET_POLICIES)
EXPECTED_MODULES = {"tracking", "face", "voice", "orchestrator"}
ALLOWED_MODULE_STATUSES = {"implemented", "integrating", "not-integrated"}


class Reporter:
    def __init__(self) -> None:
        self.failures = 0

    def pass_(self, message: str) -> None:
        print(f"[PASS] {message}")

    def info(self, message: str) -> None:
        print(f"[INFO] {message}")

    def fail(self, message: str) -> None:
        self.failures += 1
        print(f"[FAIL] {message}")


def run(command: Sequence[str]) -> Tuple[int, str]:
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except OSError as exc:
        return 127, str(exc)
    return completed.returncode, completed.stdout.strip()


def parse_l4t_release(text: str) -> Optional[str]:
    release = re.search(r"\bR(\d+)\b", text)
    revision = re.search(r"\bREVISION:\s*([0-9.]+)", text, re.IGNORECASE)
    if not release or not revision:
        return None
    return f"R{release.group(1)}.{revision.group(1)}"


def parse_cuda_version(text: str) -> Optional[str]:
    match = re.search(r"\brelease\s+([0-9]+\.[0-9]+)", text)
    return match.group(1) if match else None


def parse_tensorrt_version(text: str) -> Optional[str]:
    match = re.search(r"\b(\d+)\.(\d+)(?:\.\d+)?", text)
    return f"{match.group(1)}.{match.group(2)}" if match else None


def parse_jetpack_version(text: str) -> Optional[str]:
    match = re.match(r"(\d+\.\d+(?:\.\d+)?)", text)
    return match.group(1) if match else None


def _scalar(value: str) -> object:
    value = value.strip()
    if not value:
        return ""
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        return value


def parse_ros_parameters(path: Path) -> Dict[str, object]:
    """Parse the simple nested scalars used by a ROS 2 parameters YAML file.

    This deliberately does not implement general YAML. Deployment files using
    anchors, flow maps, or substitutions should be materialized before preflight.
    """
    flattened: Dict[str, object] = {}
    stack: list[Tuple[int, str]] = []
    in_parameters = False
    parameters_indent = -1
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        stripped = raw.strip()
        if ":" not in stripped:
            raise ValueError(f"line {line_number}: expected key: value")
        key, value = stripped.split(":", 1)
        key = key.strip().strip("'\"")
        value = value.split(" #", 1)[0].strip()
        if key == "ros__parameters":
            in_parameters = True
            parameters_indent = indent
            stack.clear()
            continue
        if not in_parameters or indent <= parameters_indent:
            continue
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if not value:
            stack.append((indent, key))
            continue
        prefix = ".".join(item[1] for item in stack)
        flattened[f"{prefix}.{key}" if prefix else key] = _scalar(value)
    if not in_parameters:
        raise ValueError("missing ros__parameters mapping")
    return flattened


def parse_module_statuses(path: Path) -> Dict[str, str]:
    statuses: Dict[str, str] = {}
    row = re.compile(r"^\|\s*([a-z][a-z0-9_-]*)\s*\|\s*`([^`]+)`\s*\|")
    for raw in path.read_text(encoding="utf-8").splitlines():
        match = row.match(raw)
        if match:
            statuses[match.group(1)] = match.group(2)
    return statuses


def find_install_artifact(prefix: Path, relative: Path) -> Optional[Path]:
    direct = prefix / relative
    if direct.exists():
        return direct
    candidates = list(prefix.glob(f"*/{relative}"))
    return candidates[0] if candidates else None


def detect_tensorrt_version() -> Optional[str]:
    for package in ("libnvinfer10", "libnvinfer8", "libnvinfer-dev"):
        code, output = run(["dpkg-query", "-W", "-f=${Version}", package])
        if code == 0:
            return parse_tensorrt_version(output)
    return None


def detect_jetpack_version() -> Optional[str]:
    code, output = run(["dpkg-query", "-W", "-f=${Version}", "nvidia-jetpack"])
    return parse_jetpack_version(output) if code == 0 else None


def detect_mvs_version(library: Path) -> Optional[str]:
    try:
        sdk = ctypes.CDLL(str(library))
        get_version = sdk.MV_CC_GetSDKVersion
        get_version.restype = ctypes.c_uint
        return f"0x{get_version():08x}"
    except (OSError, AttributeError):
        return None


def check_supported_version(
    reporter: Reporter,
    name: str,
    actual: Optional[str],
    supported_prefix: str,
    current_expected: Optional[str],
) -> None:
    if actual is None:
        reporter.fail(f"cannot determine {name} version; install its runtime/development package")
        return
    if not actual.startswith(supported_prefix):
        reporter.fail(f"{name} {actual} is outside supported {supported_prefix} series")
        return
    if current_expected is not None and actual != current_expected:
        reporter.fail(
            f"{name} is {actual}; current perception Orin baseline requires {current_expected}"
        )
        return
    reporter.pass_(f"{name} {actual}")


def check_file(reporter: Reporter, label: str, path: Path, nonempty: bool = False) -> None:
    if path.is_file() and (not nonempty or path.stat().st_size > 0):
        reporter.pass_(f"{label}: {path}")
    else:
        qualifier = "non-empty file" if nonempty else "file"
        reporter.fail(f"{label} missing ({qualifier} required): {path}")


def parameter_path(
    reporter: Reporter, parameters: Dict[str, object], name: str
) -> Optional[Path]:
    value = parameters.get(name)
    if not isinstance(value, str) or not value.strip():
        reporter.fail(f"deployment parameter {name} must be a non-empty absolute path")
        return None
    path = Path(value)
    if not path.is_absolute():
        reporter.fail(f"deployment parameter {name} must be absolute: {value}")
        return None
    return path


def check_runtime_asset_load(
    reporter: Reporter,
    validator: Optional[Path],
    asset: Path,
    missing_validator_message: str,
    success_message: str,
    failure_message: str,
) -> None:
    if validator is None or not os.access(validator, os.X_OK):
        reporter.fail(missing_validator_message)
        return
    code, output = run([str(validator), str(asset)])
    if code == 0:
        reporter.pass_(success_message)
        return
    detail = output.splitlines()[-1] if output else "validator returned no detail"
    reporter.fail(f"{failure_message} ({detail})")


def normalize_reid_backend(backend: object) -> str:
    """Mirror ReIdEmbedder::NormalizeBackend for deployment asset gating."""
    if not isinstance(backend, str):
        return "light" if backend is None else str(backend).lower()
    normalized = backend.lower()
    if normalized in {"onnx", "osnet", "osnet_onnx", "true_reid"}:
        return "osnet_onnx"
    if not normalized:
        return "light"
    if normalized in {"tracker", "light_tracker"}:
        return "light_tracker"
    if normalized in {"identity", "semantic", "light_identity"}:
        return "light_identity"
    return normalized


def check_platform(reporter: Reporter, target: str) -> None:
    policy = TARGET_POLICIES[target]
    architecture = platform.machine()
    if architecture == "aarch64":
        reporter.pass_("architecture aarch64")
    else:
        reporter.fail(f"architecture is {architecture}; Jetson Orin requires aarch64")

    version = policy.versions["jetpack"]
    check_supported_version(
        reporter,
        "JetPack",
        detect_jetpack_version(),
        version.supported_prefix,
        version.exact,
    )

    release_path = Path("/etc/nv_tegra_release")
    l4t = (
        parse_l4t_release(release_path.read_text(encoding="utf-8"))
        if release_path.is_file()
        else None
    )
    version = policy.versions["l4t"]
    check_supported_version(reporter, "L4T", l4t, version.supported_prefix, version.exact)

    nvcc = shutil.which("nvcc")
    cuda = parse_cuda_version(run([nvcc, "--version"])[1]) if nvcc else None
    version = policy.versions["cuda"]
    check_supported_version(reporter, "CUDA", cuda, version.supported_prefix, version.exact)

    tensorrt = detect_tensorrt_version()
    version = policy.versions["tensorrt"]
    check_supported_version(
        reporter, "TensorRT", tensorrt, version.supported_prefix, version.exact
    )

    ros2 = shutil.which("ros2")
    ros_distro = os.environ.get("ROS_DISTRO")
    prefix_code, rclpy_prefix = run([ros2, "pkg", "prefix", "rclpy"]) if ros2 else (127, "")
    if (
        Path("/opt/ros/humble/setup.bash").is_file()
        and ros_distro == "humble"
        and prefix_code == 0
        and Path(rclpy_prefix).resolve() == Path("/opt/ros/humble")
    ):
        reporter.pass_("active ROS 2 distribution is Humble (/opt/ros/humble)")
    else:
        reporter.fail(
            "active ROS 2 environment is not /opt/ros/humble; install Humble and run "
            "`source /opt/ros/humble/setup.bash` before this check"
        )


def check_mvs_and_camera(reporter: Reporter, target: str) -> None:
    header = Path("/opt/MVS/include/MvCameraControl.h")
    libraries = [Path(item) for item in glob.glob("/opt/MVS/lib/*/libMvCameraControl.so")]
    check_file(reporter, "MVS header", header)
    if not libraries:
        reporter.fail("MVS library missing: expected /opt/MVS/lib/*/libMvCameraControl.so")
    else:
        mvs = detect_mvs_version(libraries[0])
        version = TARGET_POLICIES[target].versions["mvs"]
        check_supported_version(
            reporter, "MVS SDK", mvs, version.supported_prefix, version.exact
        )

    lsusb = shutil.which("lsusb")
    if not lsusb:
        reporter.fail("lsusb missing; install usbutils to enumerate the Hikrobot camera")
        return
    code, output = run([lsusb, "-d", "2bdf:0001"])
    if code == 0 and "2bdf:0001" in output.lower():
        reporter.pass_("Hikrobot MV-CU013-A0UC USB device enumerated (2bdf:0001)")
    else:
        reporter.fail(
            "Hikrobot MV-CU013-A0UC not enumerated; check USB3 cable/power/permissions, "
            "then run `lsusb -d 2bdf:0001`"
        )


def check_parameters_and_assets(
    reporter: Reporter,
    params_file: Path,
    tracker_config: Path,
    engine_validator: Optional[Path],
    onnx_validator: Optional[Path],
) -> None:
    check_file(reporter, "deployment parameters", params_file)
    if not params_file.is_file():
        return
    try:
        parameters = parse_ros_parameters(params_file)
    except (OSError, ValueError) as exc:
        reporter.fail(f"cannot parse deployment parameters {params_file}: {exc}")
        return

    expected_scalars = {
        "camera.mvs_model": "MV-CU013-A0UC",
        "camera.width": 1280,
        "camera.height": 1024,
        "camera.fps": 30.0,
    }
    for name, expected in expected_scalars.items():
        actual = parameters.get(name)
        if actual == expected:
            reporter.pass_(f"deployment parameter {name}={actual}")
        else:
            reporter.fail(f"deployment parameter {name} must be {expected!r}, got {actual!r}")

    serial = parameters.get("camera.mvs_serial")
    if isinstance(serial, str) and serial.strip():
        reporter.pass_("deployment parameter camera.mvs_serial is set")
    else:
        reporter.fail("deployment parameter camera.mvs_serial must identify the deployed camera")

    engine = parameter_path(reporter, parameters, "detector.runtime_path")
    if engine is not None:
        if engine.suffix != ".engine":
            reporter.fail(f"detector.runtime_path must point to a local .engine file: {engine}")
        else:
            check_file(reporter, "local TensorRT engine", engine, nonempty=True)
            if engine.is_file() and engine.stat().st_size > 0:
                check_runtime_asset_load(
                    reporter,
                    engine_validator,
                    engine,
                    (
                        "installed validate_tensorrt_engine tool missing; rebuild "
                        "dog_patrol_perception_tracking with full Orin runtime"
                    ),
                    "local TensorRT engine loads with the production detector runtime",
                    (
                        "local TensorRT engine cannot be loaded on this Orin; regenerate it "
                        "with the documented export script"
                    ),
                )

    reid_assets = (
        ("tracker.reid_backend", "tracker.reid_model_path"),
        ("sid.reid_backend", "sid.reid_model_path"),
    )
    for backend_name, name in reid_assets:
        backend = normalize_reid_backend(parameters.get(backend_name))
        reporter.pass_(f"deployment parameter {backend_name}={backend}")
        if backend != "osnet_onnx":
            reporter.info(f"{name} is not required by backend {backend}")
            continue
        model = parameter_path(reporter, parameters, name)
        if model is not None:
            if model.suffix != ".onnx":
                reporter.fail(f"{name} must point to an ONNX model: {model}")
            else:
                check_file(reporter, name, model, nonempty=True)
                external_data = Path(f"{model}.data")
                if external_data.exists():
                    check_file(
                        reporter,
                        f"{name} external data",
                        external_data,
                        nonempty=True,
                    )
                else:
                    reporter.info(f"{name} has no sibling .onnx.data file")
                if model.is_file() and model.stat().st_size > 0:
                    check_runtime_asset_load(
                        reporter,
                        onnx_validator,
                        model,
                        (
                            "installed validate_reid_onnx tool missing; rebuild "
                            "dog_patrol_perception_tracking"
                        ),
                        f"{name} loads with the tracking OpenCV runtime",
                        f"{name} cannot be loaded; provide all ONNX external data",
                    )

    check_file(reporter, "tracker configuration", tracker_config, nonempty=True)


def check_build(
    reporter: Reporter, workspace: Path, install_prefix: Path, build_base: Path
) -> None:
    packages = (
        "dog_patrol_interfaces",
        "dog_patrol_perception_interfaces",
        "dog_patrol_perception_orchestrator",
        "dog_patrol_perception_voice",
        "dog_patrol_perception_tracking",
    )
    for package in packages:
        source_manifest = next(workspace.glob(f"src/**/{package}/package.xml"), None)
        if source_manifest is None:
            reporter.fail(f"source package missing: {package}")
            continue
        marker = find_install_artifact(
            install_prefix,
            Path("share/ament_index/resource_index/packages") / package,
        )
        if marker is None:
            reporter.fail(
                f"built package missing from {install_prefix}: {package}; run the "
                "documented colcon build"
            )
        else:
            reporter.pass_(f"built package: {package}")

    executable = find_install_artifact(
        install_prefix,
        Path("lib/dog_patrol_perception_tracking/dog_patrol_perception_tracking_node"),
    )
    if executable is not None and os.access(executable, os.X_OK):
        reporter.pass_("full Orin tracking executable installed")
    else:
        reporter.fail(
            "full tracking executable missing; rebuild with -DTRACKING_ENABLE_ORIN_RUNTIME=ON"
        )

    cmake_cache = build_base / "dog_patrol_perception_tracking" / "CMakeCache.txt"
    if cmake_cache.is_file() and re.search(
        r"^TRACKING_ENABLE_ORIN_RUNTIME:BOOL=ON$",
        cmake_cache.read_text(encoding="utf-8", errors="replace"),
        re.MULTILINE,
    ):
        reporter.pass_("tracking build configured with full Orin runtime")
    else:
        reporter.fail(
            f"full-runtime CMake result missing in {cmake_cache}; rebuild with "
            "-DTRACKING_ENABLE_ORIN_RUNTIME=ON"
        )

    colcon = shutil.which("colcon")
    if not colcon:
        reporter.fail("colcon missing; cannot inspect test results")
    else:
        code, output = run([colcon, "test-result", "--test-result-base", str(build_base)])
        summary = re.search(r"Summary:\s+(\d+) tests,\s+(\d+) errors,\s+(\d+) failures", output)
        passed = (
            code == 0
            and summary is not None
            and int(summary.group(1)) > 0
            and summary.group(2) == "0"
            and summary.group(3) == "0"
        )
        if passed and summary is not None:
            reporter.pass_(f"colcon test result: {summary.group(0)}")
        else:
            detail = output.splitlines()[-1] if output else "no test results found"
            reporter.fail(
                f"colcon test results are missing or unsuccessful under {build_base}: {detail}"
            )


def report_module_status(reporter: Reporter, workspace: Path, requirements: Path) -> None:
    try:
        statuses = parse_module_statuses(requirements)
    except OSError as exc:
        reporter.fail(f"cannot read perception requirements module statuses: {exc}")
        return
    if set(statuses) != EXPECTED_MODULES:
        reporter.fail(
            "requirements module table must contain exactly tracking, face, voice, "
            "and orchestrator"
        )
        return
    for module, status in statuses.items():
        if status not in ALLOWED_MODULE_STATUSES:
            reporter.fail(f"module {module} has unsupported status in requirements: {status}")
            continue
        reporter.info(f"module {module}: {status}")
        package_matches = list(workspace.glob(f"src/perception/dog_patrol_perception_{module}*"))
        if status == "not-integrated" and package_matches:
            reporter.fail(
                f"module {module} is marked not-integrated but a source package exists; "
                "update requirements"
            )


def build_parser() -> argparse.ArgumentParser:
    script = Path(__file__).resolve()
    default_workspace = script.parents[3]
    parser = argparse.ArgumentParser(
        description="Check the complete dog_patrol perception deployment environment."
    )
    parser.add_argument("--target", choices=SUPPORTED_TARGETS, required=True)
    parser.add_argument("--params-file", type=Path, required=True)
    parser.add_argument("--tracker-config", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, default=default_workspace)
    parser.add_argument("--install-prefix", type=Path, default=default_workspace / "install")
    parser.add_argument("--build-base", type=Path, default=default_workspace / "build")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    reporter = Reporter()
    workspace = args.workspace.resolve()
    install_prefix = args.install_prefix.resolve()
    build_base = args.build_base.resolve()
    target_policy = TARGET_POLICIES[args.target]
    print(f"[INFO] target: {args.target}")
    print(f"[INFO] workspace: {workspace}")
    print(f"[INFO] install prefix: {install_prefix}")
    print(f"[INFO] build base: {build_base}")
    if target_policy.verification_note:
        reporter.info(target_policy.verification_note)

    check_platform(reporter, args.target)
    check_mvs_and_camera(reporter, args.target)
    onnx_validator = find_install_artifact(
        install_prefix,
        Path("lib/dog_patrol_perception_tracking/validate_reid_onnx"),
    )
    engine_validator = find_install_artifact(
        install_prefix,
        Path("lib/dog_patrol_perception_tracking/validate_tensorrt_engine"),
    )
    check_parameters_and_assets(
        reporter,
        args.params_file.resolve(),
        args.tracker_config.resolve(),
        engine_validator,
        onnx_validator,
    )
    check_build(reporter, workspace, install_prefix, build_base)
    report_module_status(
        reporter, workspace, workspace / "src/perception/requirements.md"
    )

    if reporter.failures:
        print(f"PERCEPTION ENVIRONMENT: FAIL ({reporter.failures} critical checks failed)")
        return 1
    print("PERCEPTION ENVIRONMENT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
