#!/usr/bin/env python3
"""Run the fixed local fake-navigation/fake-face integration scenario.

Real tracking and voice are launched as production executables. The fake nodes
only provide navigation READY/transitions and face READY/crop observation.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import re

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import AuthorizationEvidence
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def volatile_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Observer(Node):
    def __init__(self, prefix: str) -> None:
        super().__init__("fake_integration_observer")
        self.states: list[dict] = []
        self.events: list[dict] = []
        self.evidence: list[dict] = []
        self.saw_verify = False
        self.final_state: int | None = None
        self.loss_seen = False
        self.reacquired_seen = False
        self.create_subscription(MissionState, f"{prefix}/mission/state", self.on_state, qos())
        self.create_subscription(MissionEvent, f"{prefix}/mission/event", self.on_event, volatile_qos())
        self.create_subscription(
            AuthorizationEvidence,
            f"{prefix}/perception/authorization_evidence",
            self.on_evidence,
            volatile_qos(),
        )

    def on_state(self, msg: MissionState) -> None:
        item = {
            "state_seq": int(msg.state_seq),
            "state": int(msg.state),
            "target_id": int(msg.target_id),
            "blocked": bool(msg.blocked),
            "stamp": self.get_clock().now().nanoseconds,
        }
        self.states.append(item)
        if msg.state == MissionState.VERIFY_IDENTITY:
            self.saw_verify = True
        if self.saw_verify and msg.state in (MissionState.PATROL, MissionState.TRACK_INTRUDER):
            self.final_state = int(msg.state)

    def on_event(self, msg: MissionEvent) -> None:
        self.events.append({
            "state_seq": int(msg.observed_state_seq),
            "target_id": int(msg.target_id),
            "source": int(msg.source),
            "event": int(msg.event),
            "detail": str(msg.detail),
        })
        if msg.event == MissionEvent.TARGET_LOST:
            self.loss_seen = True
            print("[fake-integration] TARGET_LOST observed; return to camera view now", flush=True)
        elif msg.event == MissionEvent.TARGET_REACQUIRED:
            self.reacquired_seen = True
            print("[fake-integration] TARGET_REACQUIRED observed", flush=True)

    def on_evidence(self, msg: AuthorizationEvidence) -> None:
        self.evidence.append({
            "state_seq": int(msg.observed_state_seq),
            "target_id": int(msg.target_id),
            "result": int(msg.result),
            "provider": str(msg.provider),
            "detail": str(msg.detail),
        })


def remaps(prefix: str) -> list[str]:
    return [
        "--ros-args",
        "-r", f"/mission/state:={prefix}/mission/state",
        "-r", f"/mission/event:={prefix}/mission/event",
        "-r", f"/perception/capability_status:={prefix}/perception/capability_status",
        "-r", f"/perception/authorization_evidence:={prefix}/perception/authorization_evidence",
        "-r", f"/perception/selected_target_bbox:={prefix}/perception/selected_target_bbox",
        "-r", f"/perception/tracked_target_image:={prefix}/perception/tracked_target_image",
    ]


def spawn(command: list[str], log: Path) -> subprocess.Popen:
    stream = log.open("w", encoding="utf-8")
    return subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)


def _range(values: list[float]) -> dict[str, float | int] | None:
    if not values:
        return None
    return {
        "min": round(min(values), 3),
        "max": round(max(values), 3),
        "average": round(sum(values) / len(values), 3),
        "samples": len(values),
    }


def parse_tegrastats(path: Path) -> dict:
    ram_used: list[float] = []
    gpu_load: list[float] = []
    tj: list[float] = []
    cpu_load: list[float] = []
    total_ram = None
    pattern = re.compile(r"RAM (\d+)/(\d+)MB.*?CPU \[([^]]+)\] GR3D_FREQ (\d+)%.*?tj@([\d.]+)C")
    if path.exists():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = pattern.search(line)
            if not match:
                continue
            ram, total, cpus, gpu, temperature = match.groups()
            ram_used.append(float(ram))
            total_ram = int(total)
            gpu_load.append(float(gpu))
            tj.append(float(temperature))
            cpu_load.extend(float(item.split("%")[0]) for item in cpus.split(","))
    return {
        "samples": len(ram_used),
        "ram_used_mb": _range(ram_used),
        "ram_total_mb": total_ram,
        "cpu_utilization_percent": _range(cpu_load),
        "gpu_gr3d_utilization_percent": _range(gpu_load),
        "tj_temperature_c": _range(tj),
    }


def process_snapshot(processes: list[subprocess.Popen]) -> list[dict]:
    snapshots = []
    for process in processes:
        try:
            status = Path(f"/proc/{process.pid}/status").read_text(encoding="utf-8")
            rss = next(
                int(line.split()[1]) for line in status.splitlines() if line.startswith("VmRSS:")
            )
            stat_fields = Path(f"/proc/{process.pid}/stat").read_text(encoding="utf-8").split()
            cpu_ticks = int(stat_fields[13]) + int(stat_fields[14])
            command = Path(f"/proc/{process.pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
            snapshots.append({
                "pid": process.pid,
                "command": command.strip(),
                "rss_kb": rss,
                "cpu_time_ticks": cpu_ticks,
            })
        except (FileNotFoundError, StopIteration, PermissionError):
            continue
    return snapshots


def tracking_detection_samples(path: Path) -> list[dict]:
    samples = []
    pattern = re.compile(r"\[INFO\] \[([\d.]+)\].*runtime_monitor.*\bdet=(\d+)")
    if not path.exists():
        return samples
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = pattern.search(line)
        if match:
            samples.append({"timestamp": float(match.group(1)), "detections": int(match.group(2))})
    return samples


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=Path("data/diagnostics/fake_integration"))
    parser.add_argument("--namespace", default="/dog_patrol/fake_it")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--scenario",
        choices=(
            "normal",
            "authorized_reencounter",
            "tracking_reacquire",
            "tracking_loss_timeout",
            "startup_visible",
        ),
        default="normal",
    )
    parser.add_argument("--tracking-params", type=Path, required=True)
    parser.add_argument("--tracker-config", type=Path, required=True)
    parser.add_argument("--voice-model-dir", type=Path)
    parser.add_argument("--voice-config", type=Path)
    parser.add_argument("--voice-helper", type=Path)
    parser.add_argument(
        "--preview",
        action="store_true",
        help="enable the tracking diagnostic overlay preview (requires DISPLAY)",
    )
    args = parser.parse_args()
    root = args.output_root / time.strftime("%Y%m%d_%H%M%S")
    root.mkdir(parents=True, exist_ok=True)
    prefix = args.namespace.rstrip("/")
    processes: list[subprocess.Popen] = []
    resource_process: subprocess.Popen | None = None
    process_samples: list[dict] = []
    next_process_sample = 0.0
    try:
        voice_scenarios = ("normal", "authorized_reencounter")
        if args.scenario in voice_scenarios and not all(
            (args.voice_model_dir, args.voice_config, args.voice_helper)
        ):
            parser.error(f"{args.scenario} scenario requires voice model/config/helper")
        if args.preview and not os.environ.get("DISPLAY"):
            parser.error("--preview requires an interactive graphical session with DISPLAY set")
        if args.scenario in ("tracking_reacquire", "tracking_loss_timeout", "startup_visible"):
            print(
                "[fake-integration] tracking-only scenario: be visible before starting and remain visible "
                "until the result is printed",
                flush=True,
            )
        tegrastats = shutil.which("tegrastats")
        if tegrastats:
            resource_process = spawn(
                [tegrastats, "--interval", "1000", "--logfile", str(root / "tegrastats.log")],
                root / "tegrastats.stderr.log",
            )
        base = ["ros2", "run"]
        common = remaps(prefix)
        processes.append(spawn(base + ["dog_patrol_manager", "mission_supervisor"] + common,
                               root / "mission_supervisor.log"))
        processes.append(spawn(base + ["dog_patrol_perception_orchestrator", "perception_readiness"] + common,
                               root / "perception_readiness.log"))
        if args.scenario in voice_scenarios:
            processes.append(spawn(base + ["dog_patrol_perception_orchestrator", "perception_authorization"] + common,
                                   root / "perception_authorization.log"))
        tracking = base + ["dog_patrol_perception_tracking", "dog_patrol_perception_tracking_node",
                           "--ros-args", "--params-file", str(args.tracking_params),
                           "-p", f"tracker.config_path:={args.tracker_config}"] + common[1:]
        if args.preview:
            tracking += ["-p", "visualization.enable:=true"]
        processes.append(spawn(tracking, root / "tracking.log"))
        if args.scenario in voice_scenarios:
            for executable, log in (("perception_voice_readiness", "voice_readiness.log"),
                                    ("perception_voice_provider", "voice_provider.log")):
                voice = base + ["dog_patrol_perception_voice", executable, "--ros-args",
                                "-p", f"model_dir:={args.voice_model_dir}",
                                "-p", f"config_file:={args.voice_config}",
                                "-p", f"helper_path:={args.voice_helper}"] + common[1:]
                processes.append(spawn(voice, root / log))
        fake = ["python3", str(Path(__file__).with_name("fake_nodes.py"))]
        nav = fake + ["--role", "navigation",
                      "--state-topic", f"{prefix}/mission/state",
                      "--event-topic", f"{prefix}/mission/event",
                      "--bbox-topic", f"{prefix}/perception/selected_target_bbox"]
        if args.scenario in voice_scenarios:
            face = fake + ["--role", "face",
                           "--state-topic", f"{prefix}/mission/state",
                           "--capability-topic", f"{prefix}/perception/capability_status",
                           "--image-topic", f"{prefix}/perception/tracked_target_image"]
        else:
            face = fake + ["--role", "capabilities",
                           "--state-topic", f"{prefix}/mission/state",
                           "--capability-topic", f"{prefix}/perception/capability_status"]
        processes.append(spawn(nav, root / "fake_navigation.log"))
        fake_capability_log = "fake_face.log" if args.scenario in voice_scenarios else "fake_capabilities.log"
        processes.append(spawn(face, root / fake_capability_log))

        rclpy.init()
        observer = Observer(prefix)
        deadline = time.monotonic() + args.timeout
        loss_timeout_deadline = None
        authorized_at = None
        authorized_wall_time = None
        absence_observed_wall_time = None
        return_prompt_wall_time = None
        return_observed_wall_time = None
        reencounter_observe_deadline = None
        evidence_count_at_authorization = None
        state_count_at_authorization = None
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.2)
            now = time.monotonic()
            if now >= next_process_sample:
                process_samples.append({"timestamp": time.time(), "processes": process_snapshot(processes)})
                next_process_sample = now + 1.0
            if args.scenario == "normal" and observer.final_state is not None:
                break
            if args.scenario == "authorized_reencounter" and observer.final_state is not None:
                if authorized_at is None:
                    authorized_at = now
                    authorized_wall_time = time.time()
                    evidence_count_at_authorization = len(observer.evidence)
                    state_count_at_authorization = len(observer.states)
                    print(
                        "[fake-integration] AUTHORIZED; leave camera view now",
                        flush=True,
                    )
                elif absence_observed_wall_time is None:
                    live_samples = tracking_detection_samples(root / "tracking.log")
                    absent_sample = next(
                        (
                            sample
                            for sample in live_samples
                            if sample["timestamp"] >= authorized_wall_time
                            and sample["detections"] == 0
                        ),
                        None,
                    )
                    if absent_sample is not None:
                        absence_observed_wall_time = absent_sample["timestamp"]
                        return_prompt_wall_time = time.time()
                        print(
                            "[fake-integration] target absence observed; return to camera view now",
                            flush=True,
                        )
                elif return_observed_wall_time is None:
                    live_samples = tracking_detection_samples(root / "tracking.log")
                    returned_sample = next(
                        (
                            sample
                            for sample in live_samples
                            if sample["timestamp"] >= return_prompt_wall_time
                            and sample["detections"] > 0
                        ),
                        None,
                    )
                    if returned_sample is not None:
                        return_observed_wall_time = returned_sample["timestamp"]
                        reencounter_observe_deadline = now + 10.0
                        print(
                            "[fake-integration] target return observed; observing suppression for 10 seconds",
                            flush=True,
                        )
                elif (
                    reencounter_observe_deadline is not None
                    and now >= reencounter_observe_deadline
                ):
                    break
            if args.scenario == "tracking_reacquire" and observer.reacquired_seen:
                break
            if args.scenario == "startup_visible":
                observed_states = {item["state"] for item in observer.states}
                target_confirmed = any(
                    item["event"] == MissionEvent.TARGET_CONFIRMED for item in observer.events
                )
                if target_confirmed and (
                    MissionState.CONFIRM_TARGET in observed_states
                    or bool(
                        {MissionState.APPROACH_TARGET, MissionState.VERIFY_IDENTITY}
                        & observed_states
                    )
                ):
                    break
            if args.scenario == "tracking_loss_timeout":
                if observer.loss_seen and loss_timeout_deadline is None:
                    loss_timeout_deadline = now + 8.0
                    print("[fake-integration] TARGET_LOST observed; keep target out of view", flush=True)
                if loss_timeout_deadline is not None and now >= loss_timeout_deadline:
                    break
        required_states = {
            MissionState.PATROL,
            MissionState.CONFIRM_TARGET,
            MissionState.APPROACH_TARGET,
            MissionState.VERIFY_IDENTITY,
        }
        visited_states = {item["state"] for item in observer.states}
        functional_pass = (
            observer.final_state is not None
            and observer.saw_verify
            and required_states.issubset(visited_states)
            and any(item["result"] != AuthorizationEvidence.CANCELLED for item in observer.evidence)
        )
        post_authorization_states = []
        detection_samples = tracking_detection_samples(root / "tracking.log")
        reencounter_observation = {
            "authorized_wall_time": authorized_wall_time,
            "absence_observed_wall_time": absence_observed_wall_time,
            "return_prompt_wall_time": return_prompt_wall_time,
            "return_observed_wall_time": return_observed_wall_time,
            "absence_observed": absence_observed_wall_time is not None,
            "return_observed": return_observed_wall_time is not None,
        }
        if args.scenario == "tracking_reacquire":
            functional_pass = observer.loss_seen and observer.reacquired_seen
        elif args.scenario == "tracking_loss_timeout":
            functional_pass = observer.loss_seen and not observer.reacquired_seen
        elif args.scenario == "startup_visible":
            target_confirmed = any(
                item["event"] == MissionEvent.TARGET_CONFIRMED for item in observer.events
            )
            functional_pass = (
                target_confirmed
                and MissionState.PATROL in visited_states
                and bool(
                    {MissionState.APPROACH_TARGET, MissionState.VERIFY_IDENTITY}
                    & visited_states
                )
            )
        elif args.scenario == "authorized_reencounter":
            post_authorization_states = observer.states[state_count_at_authorization or 0:]
            functional_pass = (
                authorized_at is not None
                and reencounter_observe_deadline is not None
                and any(item["result"] == AuthorizationEvidence.PASSED for item in observer.evidence)
                and len(observer.evidence) == evidence_count_at_authorization
                and all(item["state"] == MissionState.PATROL for item in post_authorization_states)
                and reencounter_observation["absence_observed"]
                and reencounter_observation["return_observed"]
            )
        report = {
            "test_config": {
                "preview_enabled": args.preview,
            },
            "functional": {"status": "PASS" if functional_pass else "FAIL",
                            "scenario": args.scenario,
                            "saw_verify_identity": observer.saw_verify,
                            "target_lost_seen": observer.loss_seen,
                            "target_reacquired_seen": observer.reacquired_seen,
                            "visited_states": sorted(visited_states),
                            "final_state": observer.final_state,
                            "evidence_count": len(observer.evidence),
                            "evidence_count_at_authorization": evidence_count_at_authorization,
                            "post_authorization_states": post_authorization_states,
                            "reencounter_observation": reencounter_observation},
            "performance": {
                "status": "OBSERVED",
                "collection_complete": bool(tegrastats),
                "resource_log": str(root / "tegrastats.log") if tegrastats else None,
                "tegrastats_summary": parse_tegrastats(root / "tegrastats.log"),
                "process_samples": process_samples,
            },
            "states": observer.states,
            "events": observer.events,
            "evidence": observer.evidence,
        }
        (root / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        observer.destroy_node()
        rclpy.shutdown()
        return 0 if functional_pass else 1
    finally:
        if resource_process is not None and resource_process.poll() is None:
            try:
                os.killpg(resource_process.pid, signal.SIGINT)
                resource_process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                resource_process.kill()
        for process in reversed(processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()


if __name__ == "__main__":
    raise SystemExit(main())
