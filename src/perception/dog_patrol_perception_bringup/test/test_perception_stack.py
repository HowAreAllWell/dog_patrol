import importlib.util
from pathlib import Path
import signal
import time

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.actions import SetLaunchConfiguration, TimerAction
from launch.event_handlers import OnShutdown
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
from launch.utilities import perform_substitutions
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
