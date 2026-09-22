"""Small CPU payload model; world axes are ROS Z up, distances in metres."""

import numpy as np


def payload_mounts(vehicle, task, kind):
    """Payload centers: actuator TF composed with CAD seating offsets."""

    def pose(values):
        if len(values) != 6 or not np.isfinite(values).all():
            raise ValueError("Payload poses must contain six finite xyz/rpy values")
        r, p, y = values[3:]
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        t = np.eye(4)
        t[:3, 3] = values[:3]
        t[:3, :3] = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
        return t

    actuator = vehicle["torpedoes" if kind == "torpedo" else "droppers"]
    mount = pose(actuator["pose"])
    mount[:3, 3] -= np.asarray(vehicle["base_link"], float)
    cfg = task[kind]
    slots = cfg["slot_offsets"]
    if cfg["count"] <= 0 or len(slots) != cfg["count"]:
        raise ValueError("Each payload needs a seating offset in its actuator frame")
    if kind == "torpedo" and "baseline" in actuator:
        baseline = actuator["baseline"]
        if cfg["count"] != 2 or not np.isfinite(baseline) or baseline < 0:
            raise ValueError("Torpedoes require two slots and a finite nonnegative baseline")
        result = [
            mount @ pose([0, sign * baseline / 2, 0, 0, 0, 0]) @ pose(slot)
            for sign, slot in zip((-1, 1), slots)
        ]
    else:
        result = [mount @ pose(slot) for slot in slots]
    if not np.isfinite(result).all():
        raise ValueError("Payload mounts must be finite")
    return result


def effective_mass(cfg, density):
    return density * cfg["displaced_volume"] if cfg.get("neutral_buoyancy", False) else cfg["mass"]


def launch_speed(cfg, density):
    # Effective spring energy after mechanism losses; same exterior/spring with
    # different infill gives different exit speed through different inertia.
    return np.sqrt(2 * cfg["spring_energy"] / (effective_mass(cfg, density) + cfg["added_mass"]))


def support_extent(axis, cfg):
    """Conservative oriented capsule extent for floor, rim and wall contacts."""
    return cfg["radius"] + max(0, cfg["length"] / 2 - cfg["radius"]) * np.abs(axis)


def advance(position, velocity, axis, cfg, water, density, dt):
    """Fixed-axis projectile with buoyancy, added inertia and directional drag."""
    mass = effective_mass(cfg, density)

    def acceleration(v):
        relative = v - water
        axial = axis * np.dot(relative, axis)
        lateral = relative - axial
        drag = (
            -cfg["drag_axial"] * np.linalg.norm(axial) * axial
            - cfg["drag_lateral"] * np.linalg.norm(lateral) * lateral
        )
        wet = position[2] < cfg.get("water_level", 0.0)
        force = drag if wet else np.zeros(3)
        force = force + np.array(
            [0.0, 0.0, 9.80665 * ((density * cfg["displaced_volume"] if wet else 0) - mass)]
        )
        return force / (mass + (cfg["added_mass"] if wet else 0))

    a = acceleration(velocity)
    b = acceleration(velocity + dt * a / 2)
    c = acceleration(velocity + dt * b / 2)
    d = acceleration(velocity + dt * c)
    # RK4 for [position, velocity] with uniform environment over this short step.
    p = position + dt * velocity + dt * dt * (a + b + c) / 6
    v = velocity + dt * (a + 2 * b + 2 * c + d) / 6
    return p, v


def _cross3(a, b):
    # These are always three-component vectors. Avoid np.cross's axis and
    # broadcasting setup in the 500 Hz flight integrator.
    return np.array(
        [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    )


def advance_rotating(position, velocity, orientation, angular_velocity, cfg, water, density, dt):
    """RK4 flight about the COM; input/output position is the visible mesh center.

    Velocity is COM velocity and angular velocity is in world coordinates.
    Axial COM/COB/drag offsets model a finned, approximately axisymmetric body.
    """
    mass = effective_mass(cfg, density)
    com = cfg["center_of_mass"]
    # Cylinder inertia plus parallel-axis shift and an added-inertia prior.
    inertia_mass = mass + (cfg["added_mass"] if position[2] < cfg.get("water_level", 0.0) else 0)
    inertia = inertia_mass * np.array(
        [
            cfg["radius"] ** 2 / 2,
            (cfg["length"] ** 2 + 3 * cfg["radius"] ** 2) / 12 + com**2,
            (cfg["length"] ** 2 + 3 * cfg["radius"] ** 2) / 12 + com**2,
        ]
    )
    state = np.concatenate(
        (position + orientation[:, 0] * com, velocity, orientation.ravel(), angular_velocity)
    )

    def derivative(state):
        center, v = state[:3], state[3:6]
        r, omega = state[6:15].reshape(3, 3), state[15:18]
        axis = r[:, 0]
        wet = (center - axis * com)[2] < cfg.get("water_level", 0.0)
        force = np.array([0.0, 0.0, -mass * 9.80665])
        torque = np.zeros(3)
        if wet:
            buoyancy = np.array([0.0, 0.0, density * cfg["displaced_volume"] * 9.80665])
            drag_arm = axis * (cfg["center_of_drag"] - com)
            relative = v + _cross3(omega, drag_arm) - water
            axial = axis * np.dot(relative, axis)
            lateral = relative - axial
            drag = (
                -cfg["drag_axial"] * np.linalg.norm(axial) * axial
                - cfg["drag_lateral"] * np.linalg.norm(lateral) * lateral
            )
            force += buoyancy + drag
            torque = (
                _cross3(axis * (cfg["center_of_buoyancy"] - com), buoyancy)
                + _cross3(drag_arm, drag)
                - cfg["angular_damping"] * (r @ ((r.T @ omega) * inertia / inertia[1]))
            )
        body_omega = r.T @ omega
        angular_acceleration = r @ (
            (r.T @ torque - _cross3(body_omega, inertia * body_omega)) / inertia
        )
        # omega cross each orientation column, expressed as a skew matrix.
        wx, wy, wz = omega
        rotation_rate = np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]]) @ r
        return np.concatenate(
            (
                v,
                force / (mass + (cfg["added_mass"] if wet else 0)),
                rotation_rate.ravel(),
                angular_acceleration,
            )
        )

    # Resolve the angular damping time constant, including larger caller steps.
    steps = max(1, int(np.ceil(dt * cfg["angular_damping"] / inertia[1])))
    h = dt / steps
    for _ in range(steps):
        a = derivative(state)
        b = derivative(state + h * a / 2)
        c = derivative(state + h * b / 2)
        d = derivative(state + h * c)
        state += h * (a + 2 * b + 2 * c + d) / 6
    # Remove roundoff drift while preserving the complete fin orientation.
    u, _, vt = np.linalg.svd(state[6:15].reshape(3, 3))
    r = u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt
    return state[:3] - r[:, 0] * com, state[3:6], r, state[15:18]


def crossing(start, end, coordinate, plane=0.0):
    """Segment/plane crossing, avoiding high-speed tunneling and repeat hits."""
    a, b = start[coordinate] - plane, end[coordinate] - plane
    if a * b > 0 or a == b or a == 0:
        return None
    return start + (end - start) * (-a / (b - a))


def torpedo_contact(start, end, axis, cfg):
    hit = crossing(start, end, 0)
    if hit is None or max(abs(hit[1]), abs(hit[2])) > cfg["panel_half_size"]:
        return None
    clearance = cfg["radius"] / max(0.1, abs(axis[0]))
    for hole in cfg["holes"]:
        center = (np.asarray(hole["uv"]) - 0.5) * (2 * cfg["panel_half_size"])
        radius = hole["radius_uv"] * (2 * cfg["panel_half_size"])
        if np.linalg.norm(hit[1:] - center) + clearance <= radius:
            return ("pass", hole["name"], hole["class"], hit)
    return ("blocked", "vinyl", "", hit)
