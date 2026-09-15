from dog_patrol_manager.mission_supervisor import MissionSupervisor
from dog_patrol_manager.state_machine import HomePoseData, WaypointStateData


def test_start_command_data_includes_latest_route_and_home_pose():
    route = WaypointStateData(7, 3, 1, 1, False, True)
    home = HomePoseData(pose=object())

    command = MissionSupervisor._task_command_data(0, route, home)

    assert command.command == 0
    assert command.route is route
    assert command.home_pose is home


def test_non_start_command_data_uses_latest_route_for_resume_validation():
    route = WaypointStateData(9, 2, 0, 0, False, True)

    command = MissionSupervisor._task_command_data(2, route, None)

    assert command.command == 2
    assert command.route is route
    assert command.home_pose is None
