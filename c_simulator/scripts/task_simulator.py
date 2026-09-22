#!/usr/bin/env python3
"""Start only the behavior registered by the resolved year configuration."""
import sys
import yaml
import rclpy
from rclpy.node import Node
from riptide_sim_config.profiles import load_behavior


def main():
    rclpy.init()
    probe = Node("task_simulator")
    path = probe.declare_parameter("task_config", "").value
    cfg = yaml.safe_load(open(path))
    probe.destroy_node()
    behavior = cfg.get("behavior")
    if behavior:
        node = load_behavior(behavior)()
    else:
        from riptide_sim_config.runtime import RobotRuntime

        node = RobotRuntime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "claw", None):
            node.claw.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
