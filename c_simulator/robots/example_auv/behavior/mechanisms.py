"""Example timed beacon, independent of the competition task pack."""

from std_msgs.msg import Bool


class Mechanisms:
    def __init__(self, node, vehicle, config):
        self.active = False
        self.until = 0.0
        self.time = 0.0
        self.hold = config["beacon"]["hold_seconds"]
        self.subscription = node.create_subscription(
            Bool, config["beacon"]["topic"], self.command, 10
        )
        self.publisher = node.create_publisher(Bool, "simulator/beacon", 10)

    def command(self, msg):
        self.active = bool(msg.data)
        self.until = self.time + self.hold

    # Called with plant time so the pulse pauses with the rest of the simulation.
    def step(self, now):
        self.time = now
        if now >= self.until:
            self.active = False
        msg = Bool()
        msg.data = self.active
        self.publisher.publish(msg)

    def reset(self):
        self.active = False
        self.until = self.time = 0.0
