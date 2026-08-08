from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).parents[1] / "scripts" / "check_perception_environment.py"
SPEC = importlib.util.spec_from_file_location("check_perception_environment", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class EnvironmentCheckTest(unittest.TestCase):
    def test_parses_platform_versions(self):
        self.assertEqual(
            MODULE.parse_l4t_release("# R36 (release), REVISION: 4.7, GCID: 123"),
            "R36.4.7",
        )
        self.assertEqual(
            MODULE.parse_cuda_version("Cuda compilation, release 12.6, V12.6.85"),
            "12.6",
        )
        self.assertEqual(MODULE.parse_tensorrt_version("10.3.0.30-1+cuda12.5"), "10.3")
        self.assertEqual(MODULE.parse_jetpack_version("6.2.1+b38"), "6.2.1")

    def test_flattens_required_ros_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.yaml"
            path.write_text(
                """node:
  ros__parameters:
    camera:
      mvs_model: "MV-CU013-A0UC"
      mvs_serial: "SERIAL"
      width: 1280
      height: 1024
      fps: 30.0
    detector:
      runtime_path: "/srv/models/detector.engine"
""",
                encoding="utf-8",
            )
            parsed = MODULE.parse_ros_parameters(path)
        self.assertEqual(parsed["camera.mvs_model"], "MV-CU013-A0UC")
        self.assertEqual(parsed["camera.width"], 1280)
        self.assertEqual(parsed["camera.fps"], 30.0)
        self.assertEqual(parsed["detector.runtime_path"], "/srv/models/detector.engine")

    def test_parses_repository_tracking_defaults(self):
        path = (
            Path(__file__).parents[1]
            / "dog_patrol_perception_tracking"
            / "config"
            / "perception_tracking_params.yaml"
        )
        parsed = MODULE.parse_ros_parameters(path)
        self.assertEqual(parsed["camera.mvs_model"], "MV-CU013-A0UC")
        self.assertEqual(parsed["target_image.max_publish_hz"], 10.0)
        self.assertEqual(parsed["tracker.reid_backend"], "light")
        self.assertEqual(parsed["tracker.reid_model_path"], "")
        self.assertEqual(parsed["sid.reid_backend"], "light")
        self.assertEqual(parsed["sid.reid_model_path"], "")

    def test_requirements_are_module_status_source_of_truth(self):
        path = Path(__file__).parents[1] / "requirements.md"
        self.assertEqual(
            MODULE.parse_module_statuses(path),
            {
                "tracking": "implemented",
                "face": "scaffolded/not-integrated",
                "voice": "integrating",
                "orchestrator": "integrating",
            },
        )

    def test_voice_preflight_checks_install_assets_and_read_only_runtime_queries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "vosk-model"
            model.mkdir()
            config = root / "voice.yaml"
            config.write_text(
                "\n".join(
                    [
                        "adb_executable: adb",
                        "prompt_device: plughw:CARD=Device,DEV=0",
                        "prompt_mixer_card: Device",
                        "prompt_mixer_control: PCM",
                    ]
                ),
                encoding="utf-8",
            )
            helper = root / "r818_pcm_base64_aarch64"
            helper.write_bytes(b"helper")
            helper.chmod(0o755)
            commands = []

            def fake_run(command):
                commands.append(command)
                if command[-1] == "get-state":
                    return 0, "device"
                if command[0] == "/usr/bin/ffmpeg":
                    return 0, "flite"
                if command[0] == "/usr/bin/aplay":
                    return 0, "plughw:CARD=Device,DEV=0"
                if command[0] == "/usr/bin/amixer":
                    return 0, "Simple mixer control 'PCM',0"
                raise AssertionError(command)

            reporter = MODULE.Reporter()
            with (
                mock.patch.object(
                    MODULE,
                    "VOICE_HELPER_SHA256",
                    "e81d3b0e9d82feaaf5f6e55bdff24731d7eee08632ffa63801e6397290c5d20a",
                ),
                mock.patch.object(
                    MODULE.shutil,
                    "which",
                    side_effect=lambda name: f"/usr/bin/{name}",
                ),
                mock.patch.object(MODULE, "run", side_effect=fake_run),
                mock.patch.object(MODULE, "load_vosk_model", return_value=object()),
            ):
                MODULE.check_voice_environment(reporter, model, config, helper)

        self.assertEqual(reporter.failures, 0)
        self.assertIn("get-state", commands[0])
        self.assertTrue(all("push" not in command and "shell" not in command for command in commands))

    def test_voice_preflight_rejects_missing_model_and_bad_helper_checksum(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "voice.yaml"
            config.write_text("passphrase: blue star\n", encoding="utf-8")
            helper = root / "helper"
            helper.write_bytes(b"wrong")
            helper.chmod(0o755)
            reporter = MODULE.Reporter()
            with mock.patch.object(MODULE.shutil, "which", return_value=None):
                MODULE.check_voice_environment(
                    reporter,
                    root / "missing-model",
                    config,
                    helper,
                )

        self.assertGreaterEqual(reporter.failures, 2)

    def test_voice_preflight_uses_strict_package_config_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "voice.yaml"
            config.write_text("unknown_field: true\n", encoding="utf-8")
            reporter = MODULE.Reporter()
            output = io.StringIO()
            with (
                mock.patch.object(MODULE.shutil, "which", return_value=None),
                redirect_stdout(output),
            ):
                MODULE.check_voice_environment(
                    reporter,
                    root / "missing-model",
                    config,
                    root / "missing-helper",
                )

        self.assertIn("voice configuration is invalid", output.getvalue())
        self.assertGreaterEqual(reporter.failures, 2)

    def test_rejects_parameter_file_without_ros_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.yaml"
            path.write_text("camera:\n  width: 1280\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing ros__parameters"):
                MODULE.parse_ros_parameters(path)

    def test_embedded_onnx_does_not_require_external_data_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            engine = root / "detector.engine"
            reid = root / "reid.onnx"
            tracker = root / "bot_sort.yaml"
            for path in (engine, reid, tracker):
                path.write_bytes(b"fixture")
            params = root / "params.yaml"
            params.write_text(
                f"""node:
  ros__parameters:
    camera:
      mvs_model: "MV-CU013-A0UC"
      mvs_serial: "SERIAL"
      width: 1280
      height: 1024
      fps: 30.0
    detector:
      runtime_path: "{engine}"
    tracker:
      reid_backend: "osnet_onnx"
      reid_model_path: "{reid}"
    sid:
      reid_backend: "osnet_onnx"
      reid_model_path: "{reid}"
""",
                encoding="utf-8",
            )
            reporter = MODULE.Reporter()
            with (
                mock.patch.object(MODULE.shutil, "which", return_value="/bin/true"),
                mock.patch.object(MODULE, "run", return_value=(0, "engine loaded")),
                redirect_stdout(io.StringIO()),
            ):
                MODULE.check_parameters_and_assets(
                    reporter, params, tracker, Path("/bin/true"), Path("/bin/true")
                )
        self.assertEqual(reporter.failures, 0)

    def test_onnx_runtime_load_failure_is_critical(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            engine = root / "detector.engine"
            reid = root / "reid.onnx"
            tracker = root / "bot_sort.yaml"
            for path in (engine, reid, tracker):
                path.write_bytes(b"fixture")
            params = root / "params.yaml"
            params.write_text(
                f"""node:
  ros__parameters:
    camera:
      mvs_model: "MV-CU013-A0UC"
      mvs_serial: "SERIAL"
      width: 1280
      height: 1024
      fps: 30.0
    detector:
      runtime_path: "{engine}"
    tracker:
      reid_backend: "onnx"
      reid_model_path: "{reid}"
    sid:
      reid_backend: "true_reid"
      reid_model_path: "{reid}"
""",
                encoding="utf-8",
            )

            def runtime_load(command):
                if command[-1].endswith("detector.engine"):
                    return 0, "engine loaded"
                return 1, "missing data"

            reporter = MODULE.Reporter()
            with (
                mock.patch.object(MODULE.shutil, "which", return_value="/bin/true"),
                mock.patch.object(MODULE, "run", side_effect=runtime_load),
                redirect_stdout(io.StringIO()),
            ):
                MODULE.check_parameters_and_assets(
                    reporter, params, tracker, Path("/bin/true"), Path("/bin/true")
                )
        self.assertEqual(reporter.failures, 2)

    def test_default_light_backends_do_not_require_or_load_onnx(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            engine = root / "detector.engine"
            tracker = root / "bot_sort.yaml"
            engine.write_bytes(b"fixture")
            tracker.write_bytes(b"fixture")
            params = root / "params.yaml"
            params.write_text(
                f"""node:
  ros__parameters:
    camera:
      mvs_model: "MV-CU013-A0UC"
      mvs_serial: "SERIAL"
      width: 1280
      height: 1024
      fps: 30.0
    detector:
      runtime_path: "{engine}"
    tracker:
      reid_backend: "light"
      reid_model_path: ""
    sid:
      reid_backend: ""
      reid_model_path: ""
""",
                encoding="utf-8",
            )
            reporter = MODULE.Reporter()
            with (
                mock.patch.object(MODULE, "run", return_value=(0, "engine loaded")) as run,
                redirect_stdout(io.StringIO()),
            ):
                MODULE.check_parameters_and_assets(
                    reporter, params, tracker, Path("/bin/true"), None
                )
        self.assertEqual(reporter.failures, 0)
        self.assertEqual(run.call_args_list, [mock.call(["/bin/true", str(engine)])])

    def test_all_onnx_backend_aliases_require_model_path(self):
        for alias in ("onnx", "osnet", "osnet_onnx", "true_reid", "OSNET"):
            with self.subTest(alias=alias), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                engine = root / "detector.engine"
                tracker = root / "bot_sort.yaml"
                engine.write_bytes(b"fixture")
                tracker.write_bytes(b"fixture")
                params = root / "params.yaml"
                params.write_text(
                    f"""node:
  ros__parameters:
    camera:
      mvs_model: "MV-CU013-A0UC"
      mvs_serial: "SERIAL"
      width: 1280
      height: 1024
      fps: 30.0
    detector:
      runtime_path: "{engine}"
    tracker:
      reid_backend: "{alias}"
      reid_model_path: ""
    sid:
      reid_backend: "light"
      reid_model_path: ""
""",
                    encoding="utf-8",
                )
                reporter = MODULE.Reporter()
                with (
                    mock.patch.object(MODULE, "run", return_value=(0, "engine loaded")),
                    redirect_stdout(io.StringIO()),
                ):
                    MODULE.check_parameters_and_assets(
                        reporter, params, tracker, Path("/bin/true"), Path("/bin/true")
                    )
                self.assertEqual(reporter.failures, 1)

    def test_navigation_policy_matches_documented_jetpack_compatibility_line(self):
        versions = MODULE.TARGET_POLICIES["navigation-orin"].versions
        self.assertEqual(versions["jetpack"].supported_prefix, "6.2")
        self.assertEqual(versions["l4t"].supported_prefix, "R36.4.")
        self.assertEqual(versions["cuda"].supported_prefix, "12.6")
        self.assertEqual(versions["tensorrt"].supported_prefix, "10.3")
        self.assertEqual(versions["mvs"].supported_prefix, "0x0408")

    def test_install_artifact_supports_merged_and_isolated_layouts(self):
        relative = Path("share/ament_index/resource_index/packages/example")
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            isolated = prefix / "example" / relative
            isolated.parent.mkdir(parents=True)
            isolated.touch()
            self.assertEqual(MODULE.find_install_artifact(prefix, relative), isolated)

    def test_check_build_requires_voice_package(self):
        packages = (
            "dog_patrol_interfaces",
            "dog_patrol_perception_interfaces",
            "dog_patrol_perception_orchestrator",
            "dog_patrol_perception_voice",
            "dog_patrol_perception_tracking",
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            workspace = root / "workspace"
            install_prefix = root / "install"
            for package in packages:
                (workspace / "src" / "fixture" / package).mkdir(parents=True)
                (workspace / "src" / "fixture" / package / "package.xml").touch()
                marker = (
                    install_prefix
                    / "share/ament_index/resource_index/packages"
                    / package
                )
                marker.parent.mkdir(parents=True, exist_ok=True)
                marker.touch()
            executable = (
                install_prefix
                / "lib/dog_patrol_perception_tracking/dog_patrol_perception_tracking_node"
            )
            executable.parent.mkdir(parents=True)
            executable.touch(mode=0o755)
            for voice_executable in ("perception_voice_provider", "perception_voice_readiness"):
                voice_path = install_prefix / "lib/dog_patrol_perception_voice" / voice_executable
                voice_path.parent.mkdir(parents=True, exist_ok=True)
                voice_path.touch(mode=0o755)
            cache = root / "build/dog_patrol_perception_tracking/CMakeCache.txt"
            cache.parent.mkdir(parents=True)
            cache.write_text("TRACKING_ENABLE_ORIN_RUNTIME:BOOL=ON\n", encoding="utf-8")

            reporter = MODULE.Reporter()
            with (
                mock.patch.object(MODULE.shutil, "which", return_value="/usr/bin/colcon"),
                mock.patch.object(
                    MODULE,
                    "run",
                    return_value=(0, "Summary: 1 tests, 0 errors, 0 failures"),
                ),
                redirect_stdout(io.StringIO()) as output,
            ):
                MODULE.check_build(reporter, workspace, install_prefix, root / "build")

        self.assertEqual(reporter.failures, 0)
        self.assertIn("built package: dog_patrol_perception_voice", output.getvalue())

    def test_main_prints_pass_only_when_checks_have_no_failures(self):
        arguments = [
            "--target",
            "perception-orin",
            "--params-file",
            "/tmp/deploy.yaml",
            "--tracker-config",
            "/tmp/tracker.yaml",
            "--voice-model-dir",
            "/tmp/voice-model",
        ]
        output = io.StringIO()
        with (
            mock.patch.object(MODULE, "check_platform"),
            mock.patch.object(MODULE, "check_mvs_and_camera"),
            mock.patch.object(MODULE, "check_parameters_and_assets"),
            mock.patch.object(MODULE, "check_build"),
            mock.patch.object(MODULE, "check_voice_environment"),
            mock.patch.object(MODULE, "report_module_status"),
            redirect_stdout(output),
        ):
            status = MODULE.main(arguments)
        self.assertEqual(status, 0)
        self.assertTrue(output.getvalue().rstrip().endswith("PERCEPTION ENVIRONMENT: PASS"))

    def test_main_returns_nonzero_and_prints_fail_summary(self):
        arguments = [
            "--target",
            "navigation-orin",
            "--params-file",
            "/tmp/deploy.yaml",
            "--tracker-config",
            "/tmp/tracker.yaml",
            "--voice-model-dir",
            "/tmp/voice-model",
        ]

        def fail_platform(reporter, _target):
            reporter.fail("synthetic missing dependency")

        output = io.StringIO()
        with (
            mock.patch.object(MODULE, "check_platform", side_effect=fail_platform),
            mock.patch.object(MODULE, "check_mvs_and_camera"),
            mock.patch.object(MODULE, "check_parameters_and_assets"),
            mock.patch.object(MODULE, "check_build"),
            mock.patch.object(MODULE, "check_voice_environment"),
            mock.patch.object(MODULE, "report_module_status"),
            redirect_stdout(output),
        ):
            status = MODULE.main(arguments)
        self.assertEqual(status, 1)
        self.assertIn("PERCEPTION ENVIRONMENT: FAIL (1 critical checks failed)", output.getvalue())


if __name__ == "__main__":
    unittest.main()
