"""Competition bin lights: measured proximity and a latched simulated-time dwell."""
import math
import numpy as np


def pose(values):
    """xyz/rpy (metres/radians), matching robot actuator and renderer transforms."""
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError('Expected six finite pose values')
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = [x, y, z]
    return t


class MagnetLights:
    def __init__(self, cfg, frames, vehicle):
        self.distance = float(cfg['trigger_distance'])
        self.delay = float(cfg['activation_time'])
        if not math.isfinite(self.distance) or self.distance <= 0:
            raise ValueError('Magnet trigger distance must be positive and finite')
        if not math.isfinite(self.delay) or not 0 < self.delay <= .5:
            raise ValueError('Magnet activation must take at most 0.5 seconds')
        self.initial = dict(cfg['targets'])
        if not self.initial or any(v not in ('red', 'green') for v in self.initial.values()):
            raise ValueError('Magnet light initial states must be red or green')
        offset = np.asarray(cfg['sensor_offset'], float)
        tip = np.asarray(cfg['robot_tip_offset'], float)
        if offset.shape != (3,) or tip.shape != (3,) or not np.isfinite([offset, tip]).all():
            raise ValueError('Magnet offsets must be finite xyz vectors')
        self.faces = {key: frames[key] @ pose(cfg['face_pose']) for key in self.initial}
        self.sensors = {key: (t @ np.r_[offset, 1.])[:3] for key, t in self.faces.items()}
        mount = pose(vehicle['magnet']['pose'])
        mount[:3, 3] -= np.asarray(vehicle['base_link'], float)
        self.tip = (mount @ np.r_[tip, 1.])[:3]
        self.reset()

    def reset(self):
        self.states = self.initial.copy()
        self.dwell = dict.fromkeys(self.initial, 0.)

    def step(self, dt, body=None):
        """Return newly activated targets. Missing/stale truth breaks the dwell.

        This empirical distance model uses the robot's magnet tip, not its base.
        A permanent magnet requires neither actuator arming nor electrical power.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('Magnet time step must be nonnegative and finite')
        tip = None if body is None else (body @ np.r_[self.tip, 1.])[:3]
        activated = []
        for key, sensor in self.sensors.items():
            if self.states[key] == 'green':
                continue
            if tip is None or not np.isfinite(tip).all() or np.linalg.norm(tip-sensor) > self.distance:
                self.dwell[key] = 0.
                continue
            self.dwell[key] += dt
            if self.dwell[key] + 1e-12 >= self.delay:
                self.states[key] = 'green'
                activated.append(key)
        return activated
