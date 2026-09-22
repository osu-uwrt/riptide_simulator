# riptide_simulator

CPU-based Fossen AUV physics with an OpenGL 3.3 RoboSub viewer and forward/downward
ZED X Mini camera outputs. No CUDA is required by the simulator.

Use the existing entry point after building and sourcing the workspace:

```bash
ros2 launch riptide_bringup2 simulation.launch.py robot:=talos
```

Simulation runs on simulated time. The physics node integrates its own clock,
stamps every sensor, TF and ground-truth message from it, and publishes it on
`/clock`; `simulation.launch.py` sets `use_sim_time` for the whole stack, so
navigation and control follow the plant even when physics cannot keep up with
wall time. Sensors fire from the physics loop at simulated-time intervals rather
than wall timers. To run faster or slower than wall time, launch with
`real_time_factor:=<simulated seconds per wall second>` or change it live:

```bash
ros2 param set /talos/physics_simulator real_time_factor 0.25
```

The viewer's toolbar has a **Sync sim to ROS** button (service
`sync_sim_to_estimate`, std_srvs/Trigger) that teleports the plant to the EKF's
current base_link pose, keeping velocities and leaving the EKF untouched. It is
the opposite of the startup alignment and clears accumulated estimation error
from the simulator's side, for example the lever-arm offset the EKF picks up
during large roll/pitch motion.

Running navigation next to the simulator without the full bringup? Use
`ros2 launch c_simulator sim_navigation.launch.py robot:=talos` so it shares the
clock; a wall-clock EKF against a sim-time simulator will lag or ignore data.

- [Viewer controls, lighting, camera topics and calibration](camera_faker/README.md)
- [Physics equations, parameters, validation and identification](c_simulator/docs/PHYSICS.md)
- [Torpedoes, markers, crate geometry and scoring](c_simulator/docs/TASKS.md)
- [Talos hydrodynamics profile](c_simulator/config/talos_hydrodynamics.yaml)
- [Proposed robot/task independence plan](docs/REUSABILITY_PLAN.md)

The plant includes full added mass and Coriolis coupling, dissipative drag,
orientation-dependent buoyancy, currents and delayed thrusters. It is independent
of controller tuning. **The supplied hydrodynamic coefficients are unvalidated
estimates:** pool measurements are needed before claiming Talos accuracy.

Free camera uses click-to-capture mouse look, WASD, Space/Shift and Ctrl; Escape
releases the mouse. Lighting switches between indoor and adjustable outdoor sun
with water glare. Both simulated cameras see the same lighting changes.

The viewer uses the lightweight Talos model with a CAD-derived launcher and four
separate finned payloads, adjustable water color/attenuation and depth noise, and
an expandable course map. Four lined milk crates surround the existing bin vinyls.
Torpedoes and dropped markers have simulated flight, contact and task scoring;
these use approximate contact geometry and unvalidated flight coefficients.
The ribbed CAD claw can pick up all four table props and release them into
baskets, with CPU rigid-body contacts and delivery scoring. Claw/table and
carried-object contacts stop the whole AUV through its Fossen mass/inertia model;
impacts do not change the jaw opening. TPU compliance and payload hydrodynamic
loads on the AUV are not modeled. Install `libbullet-dev` and
`c_simulator/requirements.txt` before building on a new machine. The bringup may launch
other autonomy/perception packages with their own dependencies.

The shared `scene_info.yaml` supplies vehicle/collision assets and initial AprilTag
visibility to both packages. Course poses live in `config.yaml`; visual meshes
and offsets live in `riptide_rviz/config/markers.yaml`.

The viewer uses the vehicle configuration's `base_link` offset to place the
CAD origin, exactly as the robot URDF does. At startup and after `set_sim_pose`,
physics initializes the real EKF from the simulator's base-link pose once
navigation is ready. This gives both origin frames the same starting pose;
subsequent estimation error remains visible. TF is shown at the rates published
by the normal robot bringup.

The [Talos sensor profile](c_simulator/config/talos_sensors.yaml) sets simulated
DVL noise, IMU gravity calibration, and imposed heading drift independently of
the EKF. Its gravity value matches the existing Talos calibration, its DVL noise
defaults to a 1 mm/s bottom-lock prior, and the legacy 1.5 degree/minute heading
ramp is disabled. The separate FOG stream retains small white noise and reports
matching uncertainty, so navigation can trust it appropriately. These are
configurable simulation assumptions. The real EKF configuration is unchanged.
