import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_forwards_authoritative_state_topics_to_coordinator() -> None:
    source = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    assignment = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "mission_coordinator"
            for target in node.targets
        )
    )
    launch_arguments = next(
        keyword.value
        for keyword in assignment.value.keywords
        if keyword.arg == "launch_arguments"
    )
    assert isinstance(launch_arguments, ast.Call)
    forwarded = launch_arguments.func.value
    assert isinstance(forwarded, ast.Dict)
    keys = {key.value for key in forwarded.keys if isinstance(key, ast.Constant)}

    assert "mission_state_topic" in keys
    assert "task_state_topic" in keys


def test_production_waypoint_launch_is_safe_by_default() -> None:
    source = (PACKAGE_ROOT / "launch" / "priest_external_nav.launch.py").read_text(
        encoding="utf-8"
    )

    assert "respawn=False" in source
    assert '"use_static_goals": False' in source
    assert '"auto_start_on_click": False' in source
    assert '"enable_debug_control_topics": False' in source
    assert '"task_state_timeout": 0.75' in source
    assert source.count("on_exit=Shutdown") >= 6
    assert "pause_patrol" not in source
    assert "resume_patrol" not in source


def test_nav2_critical_nodes_shutdown_the_group_instead_of_respawning() -> None:
    source = (PACKAGE_ROOT / "launch" / "nav_core.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'DeclareLaunchArgument("use_respawn", default_value="false")' in source
    assert source.count("on_exit=Shutdown") == 4
