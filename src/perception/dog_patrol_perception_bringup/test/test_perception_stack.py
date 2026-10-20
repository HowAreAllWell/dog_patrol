import importlib.util
from pathlib import Path
import signal
import time

import yaml

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration, TimerAction
from launch.event_handlers import OnShutdown
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def stack_description():
    path = Path(__file__).resolve().parents[1] / "launch/perception_stack.launch.py"
    spec = importlib.util.spec_from_file_location("perception_stack", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_stack_starts_one_reid_and_shares_gate_endpoints_with_tracking():
    description = stack_description()
    context = LaunchContext()
    context.launch_configurations.update({
        "assets_root": "/relocated/assets",
        "mission_state_topic": "/test/mission/state",
        "capability_status_topic": "/test/capabilities",
        "reid_service_name": "/test/reid/dedup",
        "reid_handled_topic": "/test/reid/handled",
    })
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name not in context.launch_configurations:
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    reids = [node for node in nodes if node.node_package == "dog_patrol_reid_provider"]
    assert len(reids) == 1
    reid = reids[0]
    assert reid.node_executable == "reid_node"
    params = evaluate_parameters(context, reid._Node__parameters)
    assert str(params[0]) == "/relocated/assets/runtime/reid.yaml"
    overrides = params[-1]
    assert overrides["model_path"] == "/relocated/assets/reid/reid_clipreid_vitb16_after.fp16.engine"
    assert overrides["device"] == "tensorrt"
    assert overrides["mission_state_topic"] == "/test/mission/state"
    assert overrides["capability_status_topic"] == "/test/capabilities"
    tracking = next(node for node in nodes if node.node_package == "dog_patrol_perception_tracking")
    tracking_params = evaluate_parameters(context, tracking._Node__parameters)[-1]
    assert tracking_params["reid.enable"] is True
    assert tracking_params["reid.service_name"] == overrides["service_name"] == "/test/reid/dedup"
    assert tracking_params["reid.handled_topic"] == overrides["handled_topic"] == "/test/reid/handled"


def test_evidence_and_verification_settings_reach_their_owners():
    description = stack_description()
    context = LaunchContext()
    context.launch_configurations.update({
        "assets_root": "/relocated/assets",
        "photos_root": "/tmp/patrol photos",
        "photos_enabled": "true",
        "lost_event_timeout_sec": "4.5",
        "voice_prompt_count": "3",
        "voice_verification_event_topic": "/test/voice/windows",
        "response_timeout_seconds": "8.0",
        "first_prompt": "Please say the passphrase.",
        "retry_prompt": "Please try again.",
        "passphrase": "blue star",
    })
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument) and action.name not in context.launch_configurations:
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    tracking = next(node for node in nodes if node.node_package == "dog_patrol_perception_tracking")
    settings = evaluate_parameters(context, tracking._Node__parameters)[-1]
    assert settings["photos.root"] == "/tmp/patrol photos"
    assert settings["photos.enabled"] is True
    assert settings["photos.voice_event_topic"] == "/test/voice/windows"
    assert settings["target.lost_event_timeout_sec"] == 4.5
    authorization = next(node for node in nodes if node.node_executable == "perception_authorization")
    assert evaluate_parameters(context, authorization._Node__parameters)[-1]["voice_prompt_count"] == 3
    for action in description.entities:
        if isinstance(action, IncludeLaunchDescription):
            arguments = dict(action.launch_arguments)
            if "voice_verification_event_topic" in arguments:
                resolved = {name: perform_substitutions(
                    context, normalize_to_list_of_substitutions(value)
                ) for name, value in arguments.items()}
                assert resolved["voice_verification_event_topic"] == "/test/voice/windows"
                assert resolved["config_file"] == "/relocated/assets/runtime/voice_verification.yaml"
                assert resolved["response_timeout_seconds"] == "8.0"
                assert resolved["first_prompt"] == "Please say the passphrase."
                assert resolved["retry_prompt"] == "Please try again."
                assert resolved["passphrase"] == "blue star"
                break
    else:
        raise AssertionError("voice launch did not receive the evidence/configuration arguments")
    face_include = next(
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
        and "photos_root" in dict(action.launch_arguments)
    )
    face_arguments = {
        name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
        for name, value in face_include.launch_arguments
    }
    assert face_arguments["photos_root"] == "/tmp/patrol photos"
    assert face_arguments["photos_enabled"] == "true"


def test_formal_face_assets_enable_reviewed_runtime_and_match_build_output():
    perception_root = Path(__file__).resolve().parents[2]
    runtime_config = (
        perception_root / "dog_patrol_perception_assets_20260813/runtime/face.yaml"
    )
    values = yaml.safe_load(runtime_config.read_text(encoding="utf-8"))
    assert values["initial_face_window_seconds"] == 3.0
    assert values["dual_verification_window_seconds"] == 10.0
    assert values["evidence_photo_enabled"] is True

    build_script = (
        perception_root
        / "dog_patrol_perception_face/scripts/build_mobilefacenet_engine.sh"
    ).read_text(encoding="utf-8")
    launch_file = (
        Path(__file__).resolve().parents[1] / "launch/perception_stack.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DETECTOR_ENGINE_PATH="$ASSETS_ROOT/face/detector.engine"' in build_script
    assert '[assets_root, "face", "detector.engine"]' in launch_file


def test_reid_exit_shuts_down_the_launch_group(tmp_path):
    # Run the actual production ReID action with a missing engine and a dummy
    # peer, so this tests group lifetime without camera/voice hardware or GPU.
    (tmp_path / "runtime").mkdir()
    (tmp_path / "runtime/reid.yaml").write_text(
        "/perception_reid:\n  ros__parameters:\n"
        f"    data_dir: '{tmp_path / 'data'}'\n"
        "    cloud.enable: false\n"
    )
    description = stack_description()
    reid = next(action for action in description.entities
                if isinstance(action, Node) and action.node_package == "dog_patrol_reid_provider")
    reasons = []
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        SetLaunchConfiguration("assets_root", str(tmp_path)),
        *[action for action in description.entities if isinstance(action, DeclareLaunchArgument)],
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: reasons.append(event.reason))),
        ExecuteProcess(cmd=["/bin/sleep", "60"]),
        reid,
        TimerAction(period=2.0, actions=[EmitEvent(event=SignalProcess(
            signal_number=signal.SIGINT, process_matcher=matches_action(reid)))]),
        TimerAction(period=8.0, actions=[EmitEvent(event=Shutdown(reason="test fallback timeout"))]),
    ]))
    started = time.monotonic()
    service.run()
    assert "ReID provider exited" in reasons
    assert "test fallback timeout" not in reasons
    assert time.monotonic() - started < 8.0
