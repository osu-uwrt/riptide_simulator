"""Robot-only lifecycle for scenarios without a competition behavior."""

from pathlib import Path
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from std_srvs.srv import Trigger
from .profiles import load_behavior, create_mechanisms


class RobotRuntime(Node):
    def __init__(self):
        super().__init__("task_simulator")
        config = yaml.safe_load(Path(self.declare_parameter("task_config", "").value).read_text())
        vehicle = yaml.safe_load(
            Path(self.declare_parameter("vehicle_config", "").value).read_text()
        )
        self.mechanisms = create_mechanisms(self, vehicle, config)
        self.time = 0.0
        self.create_subscription(Float64, "simulator/time", self.step, 10)
        self.create_subscription(Empty, "simulator/reset_tasks", lambda _: self.reset(), 10)
        self.create_service(Trigger, "simulator/reset_tasks", self.reset_service)

    def reset(self):
        if hasattr(self.mechanisms, "reset"):
            self.mechanisms.reset()

    def reset_service(self, request, response):
        self.reset()
        response.success = True
        response.message = "Mechanisms reset"
        return response

    def step(self, msg):
        if msg.data < self.time:
            self.reset()
        self.time = msg.data
        if hasattr(self.mechanisms, "step"):
            self.mechanisms.step(self.time)
