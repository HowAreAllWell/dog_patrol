"""ROS 2 adapter that publishes face capability readiness."""

from __future__ import annotations

from collections.abc import Callable, Sequence

import rclpy
from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import CapabilityStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .preflight import FacePreflight, FacePreflightOutcome
from .readiness import FaceReadinessController
from .config import FaceConfig, load_face_config


class FaceReadinessNode(Node):
    """Publish one transient-local face status for each current STARTUP."""

    def __init__(
        self,
        *,
        parameter_overrides: Sequence[Parameter] | None = None,
        preflight: Callable[[], FacePreflightOutcome] | None = None,
    ) -> None:
        super().__init__(
            "perception_face_readiness", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter(
            "capability_status_topic", "/perception/capability_status"
        )
        self.declare_parameter("capability", "face")
        self.declare_parameter("config_file", "")
        self.declare_parameter("detector_engine", "")
        self.declare_parameter("recognition_engine", "")
        self.declare_parameter("whitelist_dir", "")

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
        self._controller = FaceReadinessController(
            preflight or self._run_preflight,
            self._publish_status,
            startup_state=MissionState.STARTUP,
        )

    def destroy_node(self):
        self._controller.stop()
        return super().destroy_node()

    def _on_mission(self, msg: MissionState) -> None:
        self._controller.observe(int(msg.state_seq), int(msg.state))

    def _publish_status(self, outcome: FacePreflightOutcome) -> None:
        msg = CapabilityStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.capability = self._capability
        msg.status = int(outcome.status)
        msg.diagnostic = outcome.diagnostic
        msg.observed_startup_state_seq = int(outcome.startup_state_seq or 0)
        self._status_pub.publish(msg)

    def _run_preflight(self) -> FacePreflightOutcome:
        config_file = str(self.get_parameter("config_file").value).strip()
        config = load_face_config(config_file) if config_file else FaceConfig()
        return FacePreflight(
            detector_engine=str(self.get_parameter("detector_engine").value).strip()
            or config.detector_engine
            or None,
            recognition_engine=str(
                self.get_parameter("recognition_engine").value
            ).strip()
            or config.recognition_engine
            or None,
            whitelist_dir=str(self.get_parameter("whitelist_dir").value).strip()
            or config.whitelist_dir
            or None,
        ).run()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceReadinessNode()
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
