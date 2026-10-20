"""Face launch arguments must reach the nodes that own them."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def test_photo_settings_reach_evidence_provider_not_readiness():
    path = Path(__file__).resolve().parents[1] / "launch" / "face.launch.py"
    spec = importlib.util.spec_from_file_location("face_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update({
        "photos_root": "/tmp/patrol photos",
        "photos_enabled": "false",
    })
    for action in description.entities:
        if (
            isinstance(action, DeclareLaunchArgument)
            and action.name not in context.launch_configurations
        ):
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value
            )

    nodes = [action for action in description.entities if isinstance(action, Node)]
    readiness_node = next(
        node for node in nodes if node.node_executable == "perception_face_readiness"
    )
    provider_node = next(
        node for node in nodes if node.node_executable == "perception_face_provider"
    )
    readiness = evaluate_parameters(context, readiness_node._Node__parameters)[0]
    provider = evaluate_parameters(context, provider_node._Node__parameters)[0]
    assert "photos_root" not in readiness
    assert "photos_enabled" not in readiness
    assert provider["photos_root"] == "/tmp/patrol photos"
    assert provider["photos_enabled"] is False
