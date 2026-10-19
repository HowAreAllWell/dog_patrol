"""Optional launch overrides must preserve deployment YAML when omitted."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def _provider_parameters(monkeypatch, overrides):
    path = Path(__file__).resolve().parents[1] / "launch" / "voice.launch.py"
    spec = importlib.util.spec_from_file_location("voice_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = []

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return kwargs

    monkeypatch.setattr(module, "Node", capture_node)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update({
        "response_timeout_seconds": "", "first_prompt": "",
        "retry_prompt": "", "passphrase": "", **overrides,
    })
    action = next(item for item in description.entities if isinstance(item, OpaqueFunction))
    action.execute(context)
    return nodes[-1]["parameters"][0], context, nodes


def test_omitted_policy_arguments_preserve_yaml(monkeypatch):
    parameters, _, _ = _provider_parameters(monkeypatch, {})
    assert not {"response_timeout_seconds", "first_prompt", "retry_prompt", "passphrase"} & parameters.keys()


def test_explicit_launch_policy_values_have_the_right_ros_types(monkeypatch):
    parameters, context, _ = _provider_parameters(monkeypatch, {
        "response_timeout_seconds": "6.5", "first_prompt": "Say: yes",
        "passphrase": "yes",
    })
    assert parameters["response_timeout_seconds"].evaluate(context) == 6.5
    assert parameters["first_prompt"].evaluate(context) == "Say: yes"
    assert parameters["passphrase"].evaluate(context) == "yes"
    assert "retry_prompt" not in parameters


@pytest.mark.parametrize("value", [None, "true"])
def test_both_voice_nodes_share_the_selected_clock(monkeypatch, value):
    overrides = {} if value is None else {"use_sim_time": value}
    _, context, nodes = _provider_parameters(monkeypatch, overrides)
    assert len(nodes) == 2
    for node in nodes:
        assert node["parameters"][0]["use_sim_time"].evaluate(context) is (value == "true")
