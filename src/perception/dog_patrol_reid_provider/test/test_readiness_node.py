"""Installed provider + real aggregate; GPU cases use an explicit test engine.

Every process, topic and identity directory belongs to this test. No live robot
service or production gallery is queried, reset or restarted.
"""
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import CapabilityStatus
from dog_patrol_perception_orchestrator.readiness_node import PerceptionReadinessNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile


class Harness:
    def __init__(self, directory, engine, device="tensorrt"):
        self.directory = directory
        self.prefix = "/reid_readiness_" + uuid.uuid4().hex
        rclpy.init()
        self.node = Node("reid_readiness_probe_" + uuid.uuid4().hex)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        qos = QoSProfile(depth=16, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.node.create_publisher(MissionState, self.prefix + "/state", qos)
        self.cap_pub = self.node.create_publisher(CapabilityStatus, self.prefix + "/capability", qos)
        self.statuses, self.events = [], []
        self.node.create_subscription(CapabilityStatus, self.prefix + "/capability", self.statuses.append, qos)
        self.node.create_subscription(MissionEvent, self.prefix + "/event", self.events.append, 10)
        self.aggregate = PerceptionReadinessNode(parameter_overrides=[
            Parameter("mission_state_topic", value=self.prefix + "/state"),
            Parameter("capability_status_topic", value=self.prefix + "/capability"),
            Parameter("mission_event_topic", value=self.prefix + "/event"),
            Parameter("state_timeout", value=0.4),
        ])
        self.executor.add_node(self.aggregate)
        binary = Path(get_package_prefix("dog_patrol_reid_provider")) / "lib/dog_patrol_reid_provider/reid_node"
        self.log = (directory / "provider.log").open("w")
        self.process = subprocess.Popen([
            str(binary), "--ros-args",
            "-p", f"device:={device}", "-p", f"model_path:={engine}",
            "-p", "cloud.enable:=false", "-p", "model_idle_unload_sec:=0.0",
            "-p", f"data_dir:={directory / 'data'}", "-p", "state_timeout:=0.4",
            "-p", f"service_name:={self.prefix}/dedup",
            "-p", f"status_topic:={self.prefix}/status",
            "-p", f"handled_topic:={self.prefix}/handled",
            "-p", f"mission_state_topic:={self.prefix}/state",
            "-p", f"capability_status_topic:={self.prefix}/capability",
        ], stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin(self, duration, seq=None, state=MissionState.STARTUP):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert self.process.poll() is None, (self.directory / "provider.log").read_text()
            if seq is not None:
                msg = MissionState()
                msg.state_seq, msg.state = seq, state
                self.state_pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.03)

    def capabilities(self, seq):
        for name in ("detection_tracking", "face", "voice"):
            msg = CapabilityStatus()
            msg.capability, msg.status = name, CapabilityStatus.READY
            msg.observed_startup_state_seq = seq
            self.cap_pub.publish(msg)

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
        self.log.close()
        self.executor.shutdown()
        self.aggregate.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()


def test_missing_model_does_not_form_perception_ready(tmp_path):
    harness = Harness(tmp_path, tmp_path / "missing.onnx", device="cpu")
    try:
        harness.capabilities(10)
        harness.spin(1.5, 10)
        statuses = [s for s in harness.statuses if s.capability == "reid"]
        assert statuses, "provider must report why startup is blocked"
        assert all(s.status != CapabilityStatus.READY for s in statuses)
        assert harness.events == []
    finally:
        harness.close()


@pytest.mark.skipif(not os.environ.get("DOG_PATROL_REID_TEST_ENGINE"),
                    reason="set DOG_PATROL_REID_TEST_ENGINE for isolated Orin GPU acceptance")
def test_model_recovery_ready_reset_late_states_and_heartbeat(tmp_path):
    engine = Path(os.environ["DOG_PATROL_REID_TEST_ENGINE"])
    assert engine.is_file()
    staged = tmp_path / "staged.engine"
    harness = Harness(tmp_path, staged)
    try:
        harness.capabilities(20)
        harness.spin(1.2, 20)
        assert harness.events == []
        assert not any(s.capability == "reid" and s.status == CapabilityStatus.READY
                       for s in harness.statuses)
        # Initial failure can recover inside the same STARTUP without an image
        # query and without inserting a synthetic person into the daily gallery.
        staged.symlink_to(engine)
        harness.spin(4.0, 20)
        assert [(e.event, e.observed_state_seq) for e in harness.events] == [(MissionEvent.READY, 20)]
        harness.spin(0.4, 21, MissionState.READY)
        count = len(harness.statuses)
        harness.spin(1.2, 20)  # stale STARTUP cannot reactivate capability output
        assert len(harness.statuses) == count
        harness.capabilities(22)
        harness.spin(0.6, 22)
        assert [e.observed_state_seq for e in harness.events] == [20, 22]
        harness.spin(0.9)  # expire heartbeat and drain previously queued output
        count = len(harness.statuses)
        harness.spin(1.2)
        assert len(harness.statuses) == count
        harness.spin(0.3, 23, MissionState.READY)
        harness.spin(0.3, 23)  # same-sequence conflict
        count = len(harness.statuses)
        harness.spin(1.2, 23)
        assert len(harness.statuses) == count
        harness.capabilities(24)
        harness.spin(0.6, 24)
        assert [e.observed_state_seq for e in harness.events] == [20, 22, 24]
        assert all(e.target_id == 0 for e in harness.events)
    finally:
        harness.close()
