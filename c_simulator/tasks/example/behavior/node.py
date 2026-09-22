"""Independent observation task pack; illustrative rules, not a competition year."""

import json
import math
from pathlib import Path
import yaml
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Empty
from std_srvs.srv import Trigger
from riptide_sim_config.profiles import load_behavior, create_mechanisms


class TaskSimulator(Node):
    def __init__(self):
        super().__init__("task_simulator")
        self.cfg = yaml.safe_load(Path(self.declare_parameter("task_config", "").value).read_text())
        vehicle = yaml.safe_load(
            Path(self.declare_parameter("vehicle_config", "").value).read_text()
        )
        self.mechanisms = create_mechanisms(self, vehicle, self.cfg)
        self.regions = self.cfg["regions"]
        self.rules = self.cfg["scoring_rules"]
        self.time = 0.0
        self.position = None
        self.pose_time = None
        self.reset()

        self.pub = self.create_publisher(String, "simulator/run_score", 10)
        self.events = self.create_publisher(String, "simulator/task_events", 10)
        self.create_subscription(Odometry, "simulator/ground_truth", self.pose, 10)
        self.create_subscription(Float64, "simulator/time", self.clock, 10)
        self.create_subscription(String, "simulator/run_command", self.command, 10)
        self.create_subscription(Empty, "simulator/reset_tasks", self.reset_topic, 10)
        self.create_service(Trigger, "simulator/reset_tasks", self.reset_service)
        self.create_timer(0.1, self.publish)

    def reset(self):
        self.running = False
        self.started = self.time
        self.elapsed = 0.0
        self.dwell = {r["id"]: 0.0 for r in self.regions}
        self.completed = set()
        if hasattr(self.mechanisms, "reset"):
            self.mechanisms.reset()

    # Publish resets immediately, including when the simulation-time timer is paused.
    def reset_topic(self, msg):
        self.reset()
        self.publish()

    def reset_service(self, request, response):
        self.reset()
        self.publish()
        response.success = True
        response.message = "Run reset"
        return response

    def command(self, msg):
        try:
            action = json.loads(msg.data)["action"]
            if action == "start":
                self.reset()
                self.running = True
            elif action == "stop":
                self.running = False
            elif action == "reset":
                self.reset()
            else:
                raise ValueError("Unknown action")
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(str(error))
        self.publish()

    def pose(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)
        self.pose_time = self.time

    def clock(self, msg):
        if not math.isfinite(msg.data):
            return
        # Vehicle resets rewind elapsed simulation time; discard progress and the old pose.
        if msg.data < self.time:
            self.time = msg.data
            self.pose_time = None
            self.reset()
        dt = max(0.0, msg.data - self.time)
        self.time = msg.data
        if hasattr(self.mechanisms, "step"):
            self.mechanisms.step(self.time)
        if not self.running:
            return
        self.elapsed = self.time - self.started
        if self.position is None or self.pose_time is None or self.time - self.pose_time > 0.5:
            return
        # Dwell must be continuous, and each region can score only once per run.
        for region in self.regions:
            key = region["id"]
            inside = math.dist(self.position, region["position"]) <= region["radius"]
            self.dwell[key] = self.dwell[key] + dt if inside else 0.0
            if self.dwell[key] >= region["dwell"] and key not in self.completed:
                self.completed.add(key)
                event = String()
                event.data = json.dumps(
                    dict(entity=key, type="region_observed", time=self.time, year=self.cfg["year"])
                )
                self.events.publish(event)

    def publish(self):
        rows = [
            dict(
                key=r["id"],
                label=r["id"],
                points=self.rules["region_points"] if r["id"] in self.completed else 0,
            )
            for r in self.regions
        ]
        rows.append(
            dict(
                key="complete",
                label="Completion bonus",
                points=(
                    self.rules["completion_bonus"]
                    if len(self.completed) == len(self.regions)
                    else 0
                ),
            )
        )
        msg = String()
        msg.data = json.dumps(
            dict(
                running=self.running,
                elapsed=self.elapsed,
                total=sum(r["points"] for r in rows),
                rows=rows,
                ui=self.cfg["ui"],
                year=self.cfg["year"],
                config_id=self.cfg.get("config_id"),
                runtime=self.cfg.get("runtime", {}),
                message="Observe each region",
                ended_reason="",
            )
        )
        self.pub.publish(msg)
