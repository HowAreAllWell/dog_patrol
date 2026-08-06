"""ROS 2 adapter that publishes voice capability readiness."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import CapabilityStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .preflight import VoicePreflight, VoicePreflightOutcome, default_helper_path
from .readiness import VoiceReadinessController


class VoiceReadinessNode(Node):
    """Publish one transient-local voice status for each current STARTUP."""

    def __init__(
        self,
        *,
        parameter_overrides: Sequence[Parameter] | None = None,
        preflight: Callable[[], VoicePreflightOutcome] | None = None,
    ) -> None:
        super().__init__("perception_voice_readiness", parameter_overrides=parameter_overrides)
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("capability_status_topic", "/perception/capability_status")
        self.declare_parameter("capability", "voice")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("config_file", _default_config_file())
        self.declare_parameter("helper_path", "")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=16,
        )
        self._capability = str(self.get_parameter("capability").value).strip()
        if not self._capability:
            raise ValueError("capability must not be empty")
        self._status_pub = self.create_publisher(
            CapabilityStatus,
            str(self.get_parameter("capability_status_topic").value),
            qos,
        )
        self._state_sub = self.create_subscription(
            MissionState,
            str(self.get_parameter("mission_state_topic").value),
            self._on_mission,
            qos,
        )
        self._controller = VoiceReadinessController(
            preflight or self._run_preflight,
            self._publish_status,
            startup_state=MissionState.STARTUP,
        )

    def destroy_node(self):
        self._controller.stop()
        return super().destroy_node()

    def _on_mission(self, msg: MissionState) -> None:
        self._controller.observe(int(msg.state_seq), int(msg.state))

    def _publish_status(self, outcome: VoicePreflightOutcome) -> None:
        msg = CapabilityStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.capability = self._capability
        msg.status = int(outcome.status)
        msg.diagnostic = outcome.diagnostic
        msg.observed_startup_state_seq = int(outcome.startup_state_seq or 0)
        self._status_pub.publish(msg)

    def _run_preflight(self) -> VoicePreflightOutcome:
        model_dir = str(self.get_parameter("model_dir").value).strip()
        config_file = str(self.get_parameter("config_file").value).strip()
        helper_path = str(self.get_parameter("helper_path").value).strip()
        return VoicePreflight(
            model_dir=model_dir or None,
            config_file=config_file or _default_config_file(),
            helper_path=helper_path or default_helper_path(),
        ).run()


def _default_config_file() -> str:
    try:
        return str(
            Path(get_package_share_directory("dog_patrol_perception_voice"))
            / "config"
            / "voice.yaml"
        )
    except Exception:
        return "/__dog_patrol_voice_install_missing__/config/voice.yaml"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceReadinessNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
