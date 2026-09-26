"""CPU rigid-body task world; the Fossen plant remains authoritative for the AUV.

Bullet handles prop/table/basket/floor and jaw contacts. Opposing jaw contact
establishes a finite-force grasp constraint, not a proximity/teleport pickup.
TPU is rigid. The rack drives hold position against environmental contact.
Prop mass, drag and friction are tuning priors; travel stalls at contact.
"""

from pathlib import Path
import math
import os
import numpy as np
import pybullet as pb
from pybullet_utils.bullet_client import BulletClient


class HeadlessBulletClient(BulletClient):
    """Keep per-world client routing without parsing an empty CLI options string."""

    def __init__(self):
        # BulletClient always supplies options="", which Bullet prints as argv[0]=.
        # Omitting options bypasses its CLI parser and preserves all physics logs.
        self._shapes = {}
        self._pid = os.getpid()
        self._client = -1
        self._client = pb.connect(pb.DIRECT)


def rotation(q):
    return np.asarray(pb.getMatrixFromQuaternion(q)).reshape(3, 3)


def pose_matrix(p, q):
    t = np.eye(4)
    t[:3, :3] = rotation(q)
    t[:3, 3] = p
    return t


def quaternion(r):
    # Stable at 180 degrees; select the largest quaternion component.
    candidates = np.array(
        [
            1 + np.trace(r),
            1 + 2 * r[0, 0] - np.trace(r),
            1 + 2 * r[1, 1] - np.trace(r),
            1 + 2 * r[2, 2] - np.trace(r),
        ]
    )
    i = int(np.argmax(candidates))
    s = 2 * math.sqrt(max(candidates[i], 0))
    if i == 0:
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
    else:
        a = i - 1
        b = (a + 1) % 3
        c = (a + 2) % 3
        q = np.zeros(4)
        q[a] = s / 4
        q[b] = (r[b, a] + r[a, b]) / s
        q[c] = (r[c, a] + r[a, c]) / s
        q[3] = (r[c, b] - r[b, c]) / s
    return np.asarray(q) / np.linalg.norm(q)


class ClawWorld:
    def __init__(
        self, cfg, frames, mount, mesh_folder, map_to_pool, table_collision=None, world=None
    ):
        world = world or dict(length=50.0, width=22.86, depth=2.1336, water_level=0.0)
        length, width, depth, surface = [
            world[k] for k in ("length", "width", "depth", "water_level")
        ]
        self.cfg = cfg
        self.frames = frames
        self.mount = mount
        self.table_collision = table_collision or dict(
            slab_size=[0.635, 0.635, 0.01905], slab_center=[0, 0, -0.009525]
        )
        self.mesh_folder = Path(mesh_folder)
        self.map_to_pool = map_to_pool
        for key in (
            "max_gap",
            "min_gap",
            "jaw_speed",
            "hold_force",
            "friction",
            "contact_margin",
            "grasp_dwell",
            "slip_distance",
        ):
            if not math.isfinite(cfg[key]) or cfg[key] <= 0:
                raise ValueError("Invalid claw " + key)
        if cfg["max_gap"] <= cfg["min_gap"]:
            raise ValueError("Invalid claw travel")
        self.b = HeadlessBulletClient()
        self.b.setGravity(0, 0, -9.81)
        self.b.setPhysicsEngineParameter(numSolverIterations=60, deterministicOverlappingPairs=1)
        self.events = []
        self.props = {}
        self.statics = {}
        self.travel = (cfg["max_gap"] - cfg["min_gap"]) / 2
        self.target = 0.0
        self.command_until = None
        self.direction = 0
        self.held = None
        self.constraint = None
        self.contact_time = {}
        self.body = np.eye(4)
        floor = self.b.createCollisionShape(pb.GEOM_PLANE)
        self.floor = self.b.createMultiBody(0, floor, basePosition=[0, 0, surface - depth])
        self.scenery = [self.floor]
        for key, mesh in [
            ("table", "table"),
            ("helmet", "table_basket_helmet"),
            ("warning", "table_basket_warning"),
        ]:
            shape = self.b.createCollisionShape(
                pb.GEOM_MESH,
                fileName=str(self.mesh_folder / (mesh + ".obj")),
                flags=pb.GEOM_FORCE_CONCAVE_TRIMESH,
            )
            t = frames[key]
            uid = self.b.createMultiBody(
                0, shape, basePosition=t[:3, 3], baseOrientation=quaternion(t[:3, :3])
            )
            self.b.changeDynamics(uid, -1, lateralFriction=0.7, restitution=0.05)
            self.statics[key] = uid
            self.scenery.append(uid)
        solid = self.table_collision
        t = frames["table"]
        shape = self.b.createCollisionShape(
            pb.GEOM_BOX, halfExtents=np.asarray(solid["slab_size"]) / 2
        )
        self.table_solid = self.b.createMultiBody(
            0,
            shape,
            basePosition=(t @ np.r_[solid["slab_center"], 1])[:3],
            baseOrientation=quaternion(t[:3, :3]),
        )
        self.scenery.append(self.table_solid)
        pool_to_map = np.linalg.inv(map_to_pool)
        for center, size in [
            ([-0.1, width / 2, surface - depth / 2], [0.2, width, depth + 1]),
            ([length + 0.1, width / 2, surface - depth / 2], [0.2, width, depth + 1]),
            ([length / 2, -0.1, surface - depth / 2], [length, 0.2, depth + 1]),
            ([length / 2, width + 0.1, surface - depth / 2], [length, 0.2, depth + 1]),
        ]:
            shape = self.b.createCollisionShape(pb.GEOM_BOX, halfExtents=np.asarray(size) / 2)
            self.scenery.append(
                self.b.createMultiBody(
                    0,
                    shape,
                    basePosition=(pool_to_map @ np.r_[center, 1])[:3],
                    baseOrientation=quaternion(pool_to_map[:3, :3]),
                )
            )
        for key, c in cfg["props"].items():
            if c["mass"] <= 0 or c["volume"] <= 0:
                raise ValueError("Invalid prop mass/volume")
            path = self.mesh_folder / (c["mesh"] + ".obj")
            v = np.array(
                [
                    [float(x) for x in line.split()[1:4]]
                    for line in path.read_text().splitlines()
                    if line.startswith("v ")
                ]
            )
            center = (v.min(0) + v.max(0)) / 2
            half = (v.max(0) - v.min(0)) / 2
            shape = self.b.createCollisionShape(pb.GEOM_MESH, vertices=(v - center).tolist())
            t = frames[key]
            p = t[:3, 3] + t[:3, :3] @ center
            uid = self.b.createMultiBody(
                c["mass"], shape, basePosition=p, baseOrientation=quaternion(t[:3, :3])
            )
            self.b.changeDynamics(
                uid,
                -1,
                lateralFriction=0.8,
                spinningFriction=0.003,
                rollingFriction=0.001,
                restitution=0.05,
                linearDamping=0.02,
                angularDamping=0.1,
                collisionMargin=0.0005,
                ccdSweptSphereRadius=0.008,
                contactProcessingThreshold=0,
            )
            self.props[key] = dict(
                id=uid,
                center=center,
                half=half,
                config=c,
                settled=0.0,
                scored=False,
                picked=False,
                surfaced=False,
            )
        # Position-held rack drives: environmental impacts cannot back-drive
        # these joints. Vehicle/scenery collision response runs in the Fossen
        # plant; the prop solver sees two fixed, commanded contact surfaces.
        self.claw = self.b.createMultiBody(0)
        self.pads = []
        self.q = 0.0
        for file in ("claw_pad.obj", "claw_pad_right.obj"):
            shape = self.b.createCollisionShape(pb.GEOM_MESH, fileName=str(self.mesh_folder / file))
            uid = self.b.createMultiBody(0, shape, basePosition=[0, 0, 10])
            self.b.changeDynamics(
                uid,
                -1,
                lateralFriction=cfg["friction"],
                spinningFriction=0.01,
                restitution=0,
                collisionMargin=0.0005,
            )
            self.pads.append(uid)
        self._reset_bodies()

    def close(self):
        self.b.disconnect()

    def reset(self):
        # Recreate this small contact world to clear broadphase/manifold caches
        # and constraints as well as poses. The independent AUV plant keeps running.
        args = (
            self.cfg,
            self.frames,
            self.mount,
            self.mesh_folder,
            self.map_to_pool,
            self.table_collision,
        )
        self.close()
        self.__init__(*args)

    def _reset_bodies(self):
        self.release("reset")
        self.target = 0.0
        self.direction = 0
        self.command_until = None
        self.contact_time.clear()
        self.events.clear()
        self.q = 0.0
        self._place_pads(np.eye(4), self.q, np.zeros(3), np.zeros(3), 0)
        for key, p in self.props.items():
            t = self.frames[key]
            self.b.resetBasePositionAndOrientation(
                p["id"], t[:3, 3] + t[:3, :3] @ p["center"], quaternion(t[:3, :3])
            )
            self.b.resetBaseVelocity(p["id"], [0, 0, 0], [0, 0, 0])
            p.update(settled=0.0, scored=False, picked=False, surfaced=False)

    def command(self, opened, duration=None):
        self.target = self.travel if opened else 0.0
        self.direction = 1 if opened else -1
        self.command_until = duration
        if opened:
            self.release("released")

    def stop(self):
        self.target = sum(self.joints()) / 2
        self.direction = 0
        self.command_until = None

    def joints(self):
        return [self.q, self.q]

    def _place_pads(self, mount, q, velocity, angular, qdot):
        for uid, sign in zip(self.pads, (1, -1)):
            offset = mount[:3, 1] * (sign * q)
            self.b.resetBasePositionAndOrientation(
                uid, mount[:3, 3] + offset, quaternion(mount[:3, :3])
            )
            self.b.resetBaseVelocity(
                uid, velocity + np.cross(angular, offset) + mount[:3, 1] * (sign * qdot), angular
            )

    def _drive_jaws(self, dt, mount, velocity, angular):
        old = self.q
        proposed = float(
            np.clip(self.target, old - self.cfg["jaw_speed"] * dt, old + self.cfg["jaw_speed"] * dt)
        )
        direction = np.sign(proposed - old)
        self._place_pads(mount, proposed, velocity, angular, 0)
        if direction:
            # Stall commanded travel against solid contacts. Never let the
            # scenery choose a different jaw opening or push the racks apart.
            for uid, sign in zip(self.pads, (1, -1)):
                axis = mount[:3, 1] * sign
                for other in self.scenery + [p["id"] for p in self.props.values()]:
                    for c in self.b.getClosestPoints(uid, other, 0):
                        normal_speed = np.dot(c[7], axis) * direction
                        if normal_speed < -0.2 and c[8] < -0.0008:
                            proposed -= direction * min(
                                abs(proposed - old), (-0.0008 - c[8]) / (-normal_speed)
                            )
            self.q = proposed
        self._place_pads(mount, self.q, velocity, angular, (self.q - old) / dt)

    def release(self, reason):
        if self.constraint is not None:
            self.b.removeConstraint(self.constraint)
            self.constraint = None
        if self.held is not None:
            for uid in self.pads:
                self.b.setCollisionFilterPair(uid, self.props[self.held]["id"], -1, -1, 1)
            self.events.append((self.held, reason, ""))
        self.held = None

    def prop_pose(self, key):
        p = self.props[key]
        xyz, q = self.b.getBasePositionAndOrientation(p["id"])
        t = pose_matrix(xyz, q)
        t[:3, 3] -= t[:3, :3] @ p["center"]
        return t

    def basket_destination(self, key, _resting=()):
        p = self.props[key]
        if key == self.held:
            return None
        pos, q = self.b.getBasePositionAndOrientation(p["id"])
        for basket in ("helmet", "warning"):
            t = self.frames[basket]
            local = t[:3, :3].T @ (np.asarray(pos) - t[:3, 3])
            extent = np.abs(t[:3, :3].T @ rotation(q)) @ p["half"]
            if (
                np.all(np.abs(local[:2]) + extent[:2] < [0.087, 0.130])
                and -0.001 < local[2] < 0.077
            ):
                if self.b.getContactPoints(p["id"], self.statics[basket]):
                    return basket
                # Stacked: resting on another prop that is itself in this basket.
                resting = (*_resting, key)
                for other, o in self.props.items():
                    if (
                        other not in resting
                        and self.b.getContactPoints(p["id"], o["id"])
                        and self.basket_destination(other, resting) == basket
                    ):
                        return basket
        return None

    def basket_contents(self):
        return {
            key: basket
            for key, p in self.props.items()
            if p["settled"] > 0.5 and (basket := self.basket_destination(key)) is not None
        }

    def step(self, dt, body, velocity, angular, water, density, enabled=True):
        self.body = body
        mount = body @ self.mount
        self.b.setTimeStep(dt)
        if self.command_until is not None:
            self.command_until -= dt
            if self.command_until <= 0:
                self.stop()
        if not enabled:
            self.stop()
        self.b.resetBasePositionAndOrientation(self.claw, mount[:3, 3], quaternion(mount[:3, :3]))
        self.b.resetBaseVelocity(
            self.claw, velocity + np.cross(angular, mount[:3, 3] - body[:3, 3]), angular
        )
        self._drive_jaws(
            dt, mount, velocity + np.cross(angular, mount[:3, 3] - body[:3, 3]), angular
        )
        for key, p in self.props.items():
            xyz, q = self.b.getBasePositionAndOrientation(p["id"])
            r = rotation(q)
            vel, omega = self.b.getBaseVelocity(p["id"])
            rel = np.asarray(vel) - water
            half_z = (np.abs(r) @ p["half"])[2]
            wet = float(np.clip((half_z - xyz[2]) / (2 * half_z), 0, 1))
            local = r.T @ rel
            size = 2 * p["half"]
            area = np.array([size[1] * size[2], size[0] * size[2], size[0] * size[1]])
            force = -0.5 * density * wet * (r @ (area * local * np.abs(local)))
            force[2] += density * 9.81 * p["config"]["volume"] * wet
            self.b.applyExternalForce(p["id"], -1, force, xyz, pb.WORLD_FRAME)
            self.b.applyExternalTorque(
                p["id"], -1, -0.005 * wet * np.asarray(omega), pb.WORLD_FRAME
            )
        self.b.stepSimulation()
        if self.held is not None:
            p = self.props[self.held]
            pos, _ = self.b.getBasePositionAndOrientation(p["id"])
            expected = (mount @ self.held_relative)[:3, 3]
            if np.linalg.norm(np.asarray(pos) - expected) > self.cfg["slip_distance"]:
                self.release("slipped")
        if self.held is None and self.direction < 0 and enabled:
            for key, p in self.props.items():
                contacts = [self.b.getContactPoints(uid, p["id"]) for uid in self.pads]
                # Only the inward perforated/silicone-coated faces can pinch;
                # brushing both pad backs or undersides is not a grasp.
                touching = all(
                    any(
                        c[8] < self.cfg["contact_margin"]
                        and c[9] > 0.01
                        and np.dot(c[7], mount[:3, 1] * sign) > 0.6
                        for c in cs
                    )
                    for sign, cs in zip((1, -1), contacts)
                )
                self.contact_time[key] = self.contact_time.get(key, 0) + dt if touching else 0
                if self.contact_time[key] >= self.cfg["grasp_dwell"]:
                    pos, q = self.b.getBasePositionAndOrientation(p["id"])
                    relative = np.linalg.inv(mount) @ pose_matrix(pos, q)
                    self.constraint = self.b.createConstraint(
                        self.claw,
                        -1,
                        p["id"],
                        -1,
                        pb.JOINT_FIXED,
                        [0, 0, 0],
                        relative[:3, 3],
                        [0, 0, 0],
                        parentFrameOrientation=quaternion(relative[:3, :3]),
                        childFrameOrientation=[0, 0, 0, 1],
                    )
                    self.b.changeConstraint(self.constraint, maxForce=self.cfg["hold_force"])
                    # A grasped object and its holding pads form one assembly.
                    # Internal contacts must not fight the grasp constraint.
                    for uid in self.pads:
                        self.b.setCollisionFilterPair(uid, p["id"], -1, -1, 0)
                    self.held = key
                    self.held_relative = relative
                    p.update(picked=True, scored=False, settled=0.0, surfaced=False)
                    self.target = sum(self.joints()) / 2
                    self.direction = 0
                    self.events.append((key, "grasped", ""))
                    break
        for key, p in self.props.items():
            pos, q = self.b.getBasePositionAndOrientation(p["id"])
            half = np.abs(rotation(q)) @ p["half"]
            if p["picked"] and pos[2] - half[2] > 0 and not p["surfaced"]:
                p["surfaced"] = True
                self.events.append((key, "surfaced", ""))
            if key == self.held or p["scored"]:
                continue
            vel, _ = self.b.getBaseVelocity(p["id"])
            destination = self.basket_destination(key)
            if p["picked"] and self.b.getContactPoints(p["id"], self.floor):
                destination = "pool_floor"
            if destination and np.linalg.norm(vel) < 0.04:
                p["settled"] += dt
                if p["settled"] > 0.5:
                    result = (
                        "miss"
                        if destination == "pool_floor"
                        else "success" if destination == p["config"]["basket"] else "wrong_target"
                    )
                    self.events.append((key, result, destination))
                    p["scored"] = True
            else:
                p["settled"] = 0.0
