"""Talos UWRT actuator adapter and launcher/claw command behavior.

bind(runtime) attaches to the payload/contact runtime provided by a task pack.
The runtime owns truth and integrated bodies; this adapter owns command gating,
reload/slot sequencing and native UWRT command/status conventions.
"""

import math
import time
import numpy as np
from std_msgs.msg import Bool, Empty, Float32
from std_srvs.srv import Trigger, SetBool
from riptide_msgs2.msg import KillSwitchReport
from payload_model import payload_mounts, launch_speed


def quat_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class Mechanisms:
    def __init__(self, node, vehicle, config):
        self.vehicle = vehicle
        self.config = config

    def payload_mounts(self, task, kind):
        return payload_mounts(self.vehicle, task, kind)

    def bind(self, runtime):
        self.runtime = runtime
        runtime.create_subscription(
            Bool, "command/actuator/claw", lambda m: runtime.reply(runtime.claw_command(m.data)), 10
        )
        runtime.create_service(SetBool, "command/actuator/claw", runtime.claw_service)
        runtime.create_subscription(Float32, "command/actuator/claw_move_s", runtime.claw_timed, 10)
        runtime.create_subscription(KillSwitchReport, "command/software_kill", runtime.kill, 10)
        runtime.create_subscription(
            Bool, "command/actuator/arm", lambda m: runtime.reply(runtime.arm(m.data)), 10
        )
        runtime.create_service(SetBool, "command/actuator/arm", runtime.arm_service)
        runtime.create_subscription(
            Empty, "command/actuator/notify_reload", lambda _: runtime.reply(runtime.reload()), 10
        )
        runtime.create_service(Trigger, "command/actuator/notify_reload", runtime.reload_service)
        for kind in ("torpedo", "dropper"):
            runtime.create_subscription(
                Empty,
                "command/actuator/" + kind,
                lambda _, k=kind: runtime.reply(runtime.fire(k)),
                10,
            )
            runtime.create_service(
                Trigger,
                "command/actuator/" + kind,
                lambda req, res, k=kind: runtime.fire_service(req, res, k),
            )
            runtime.create_subscription(
                Empty,
                "command/simple_" + kind + "_fire",
                lambda _, k=kind: runtime.simple_fire(k),
                10,
            )

    def claw_service(self, req, res):
        runtime = self.runtime
        res.success, res.message = runtime.claw_command(req.data)
        return res

    def claw_timed(self, msg):
        runtime = self.runtime
        if not math.isfinite(msg.data):
            runtime.reply((False, "Claw duration must be finite"))
            return
        runtime.reply(runtime.claw_command(msg.data > 0, abs(msg.data)))

    def claw_command(self, opened, duration=None):
        runtime = self.runtime
        if runtime.claw is None:
            return False, "No claw configured"
        if runtime.killed or not runtime.armed:
            return False, "Actuators are disarmed or vehicle is killed"
        if runtime.truth is None or time.monotonic() - runtime.truth_wall > 1:
            return False, "No fresh vehicle ground truth"
        if duration == 0:
            runtime.claw.stop()
        else:
            runtime.claw.command(opened, duration)
        return True, "Claw opening" if opened else "Claw closing"

    def kill(self, msg):
        runtime = self.runtime
        if msg.kill_switch_id == 1:
            runtime.killed = msg.switch_asserting_kill
            if runtime.killed:
                runtime.armed = False

    def arm(self, value):
        runtime = self.runtime
        if value and runtime.killed:
            return False, "Vehicle is killed; enable it first"
        runtime.armed = value
        return True, "Armed" if value else "Disarmed"

    def reload(self):
        runtime = self.runtime
        runtime.armed = False
        runtime.busy_until = 0.0
        runtime.available = {k: runtime.cfg[k]["count"] for k in runtime.available}
        return True, "Reloaded and disarmed"

    def reply(self, result):
        runtime = self.runtime
        msg = Bool()
        msg.data = result[0]
        runtime.feedback.publish(msg)
        if not result[0]:
            runtime.get_logger().warning(result[1])

    def arm_service(self, req, res):
        runtime = self.runtime
        res.success, res.message = runtime.arm(req.data)
        return res

    def reload_service(self, req, res):
        runtime = self.runtime
        res.success, res.message = runtime.reload()
        return res

    def fire_service(self, req, res, kind):
        runtime = self.runtime
        res.success, res.message = runtime.fire(kind)
        return res

    def simple_fire(self, kind):
        runtime = self.runtime
        result = runtime.arm(True)
        runtime.reply(runtime.fire(kind) if result[0] else result)

    def slot_state(self, kind, index):
        runtime = self.runtime
        c = runtime.cfg[kind]
        t = runtime.truth.pose.pose
        r = quat_rotation(t.orientation)
        slot = runtime.mounts[kind][index]
        mount = slot[:3, 3]
        orientation = r @ slot[:3, :3]
        axis = orientation[:, 0]
        position = np.array([t.position.x, t.position.y, t.position.z]) + r @ mount
        return position, axis, mount, orientation

    def fire(self, kind):
        runtime = self.runtime
        if not runtime.armed or runtime.killed:
            return False, "Actuators are disarmed or vehicle is killed"
        if runtime.time < runtime.busy_until:
            return False, "Actuator is busy"
        if runtime.available[kind] <= 0:
            return False, "No " + kind + " ammunition; reload first"
        if runtime.truth is None or time.monotonic() - runtime.truth_wall > 1:
            return False, "No fresh vehicle ground truth"
        c = runtime.cfg[kind]
        t = runtime.truth.pose.pose
        r = quat_rotation(t.orientation)
        index = c["count"] - runtime.available[kind]
        position, axis, mount, orientation = runtime.slot_state(kind, index)
        twist = runtime.truth.twist.twist
        v = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        w = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        velocity = r @ (v + np.cross(w, mount)) + axis * launch_speed(
            c, runtime.hydro["water_density"]
        )
        angular_velocity = r @ w
        if "center_of_mass" in c:
            velocity += np.cross(angular_velocity, axis * c["center_of_mass"])
        runtime.payloads.append(
            dict(
                id=runtime.next_id,
                kind=kind,
                position=position,
                velocity=velocity,
                axis=axis,
                orientation=orientation,
                slot=index,
                angular_velocity=angular_velocity,
                age=0.0,
                active=True,
                scored=False,
                entered=set(),
            )
        )
        runtime.payloads = runtime.payloads[-64:]
        runtime.next_id += 1
        runtime.available[kind] -= 1
        runtime.busy_until = runtime.time + c["cooldown"]
        runtime.busy_kind = kind
        runtime.payload_released(runtime.payloads[-1])
        return True, kind + " released"
