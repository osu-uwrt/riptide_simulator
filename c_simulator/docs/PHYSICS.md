# AUV plant and identification

The simulator now has an independent six-DOF marine dynamics library, a delayed
thruster model, and a ROS adapter. It runs on the CPU; visualization uses OpenGL
3.3. The existing bringup, control, navigation, camera and vision interfaces remain.
No MPC/NMPC controller is implemented by this change.

**Talos coefficients are unvalidated priors, not measured vehicle dynamics.**
The equations have numerical/physical consistency tests; that is different from
validating their predictions against Talos in the pool. Use this plant for software
integration, controller development and sensitivity studies. Realistic gain
transfer requires identification and held-out pool trials.

## Model choice

An explicit Fossen model is useful here because the dominant uncertainty is the
vehicle's hydrodynamic parameters. A general rigid-body engine does not identify
those parameters, and a visually convincing fluid simulation does not establish
accurate vehicle forces. This implementation exposes the same state derivative
for simulation and future controller prediction, with no renderer or ROS dependency
in the dynamics library. CFD, wake interactions and detailed contact mechanics
remain separate possible extensions.

## Equations and coordinate conventions

World is `map`, Z up. Body uses ROS forward/left/up. State is
`[x,y,z,qw,qx,qy,qz,u,v,w,p,q,r]`: position at **COM**, body-to-world unit quaternion,
and absolute body linear/angular velocity. ROS ground-truth pose and reset poses
refer to **base_link**, with the rotating COM-to-base offset applied explicitly.
All distances, angles, forces and torques use SI units and radians.

For uniform, possibly time-varying inertial current `v_c`, let
`c = [Rᵀ v_c; 0]`, `nu_r = nu-c`, and
`dc/dt = [Rᵀ a_c - omega × (Rᵀ v_c); 0]`. The implemented body equation is:

```
(M_RB + M_A) nu_dot = tau_prop + tau_drag + tau_hydrostatic + tau_pressure
                     - C_RB(nu) nu - C_A(nu_r) nu_r + M_A dc/dt
```

`M_RB = diag(m I3, I_COM)`. Full symmetric positive-semidefinite 6×6 added mass
and linear damping are supported, including off-diagonal coupling. Coriolis uses
the momentum construction from [Fossen's marine model](https://www.fossen.biz/html/marineCraftModel.html)
and [MSS m2c](https://github.com/cybergalactic/MSS/blob/master/LIBRARY/modeling/m2c.m).
The current derivative is multiplied by **added mass**, not total mass, because
the state stores absolute body velocity. A body rotating in constant world current
must not acquire a fictitious rigid-body acceleration.

Damping is linear plus axis-wise quadratic at a configurable drag center. Both
velocity and wrench are transformed to/from that point, so drag cannot add energy
relative to the water. Positive damping values oppose motion. This differs from
sources that tabulate negative hydrodynamic derivatives: convert their signs.
To convert FRD body coefficients to FLU at the same origin, use
`S=diag(1,-1,-1,1,-1,-1)`, `M_FLU=S M_FRD Sᵀ` (likewise linear damping).
Also convert world frames, offsets and any reference-point shifts explicitly.

Weight acts at COM. Buoyancy acts at the centroid of the submerged portion of an
oriented ellipsoid; volume fraction and centroid change continuously through the
waterline. The envelope's radii determine that transition; `displaced_volume`
sets full buoyancy independently. For uniform accelerating water, an additional
pressure force `rho * submerged_volume * Rᵀ a_c` acts at that wet centroid.
This is the uniform-flow Froude–Krylov approximation; see the pressure and added-mass
distinction in [Lind et al.](https://doi.org/10.1007/s40722-016-0056-4).
The optional oscillatory current is a disturbance scenario, not a wave-field solver.

## Parameters and provenance

Default plant: [Talos hydrodynamics](../robots/talos/config/hydrodynamics.yaml).
It is separate from controller/feed-forward configuration: tuning the controller
does not silently retune its simulated plant. The loader rejects nonfinite,
nonsymmetric or nonphysical inertia/damping. Another vehicle needs its own profile.

| Quantity | Current source / assumption |
| --- | --- |
| Mass, COM, sensor/thruster mounts | `riptide_descriptions2/config/talos.yaml` |
| Rigid inertia about COM | Legacy simulator inertia, retained as a CAD prior |
| Added mass | Potential-flow equivalent ellipsoid, scaled to displaced volume |
| Linear/quadratic drag | Nonnegative fit to legacy simulator curves over 0–0.6 m/s and 0–1 rad/s |
| Displaced volume | Assumed +3 N full-submergence buoyancy in freshwater |
| COB relative to COM | Legacy estimate, not measured trim |
| Partial-submergence envelope | Existing collision envelope, not measured sealed volume |
| Actuator delay, lag, force limits | Legacy starting assumptions; independently adjustable |

The ellipsoid prior includes translational and rotational added inertia. Its
sphere and prolate limits are regression-tested against analytic results,
including [MSS imlay61](https://github.com/cybergalactic/MSS/blob/master/LIBRARY/modeling/imlay61.m).
An open-frame AUV is not an ellipsoid; this only supplies explicit nonzero starting
values. Cross-coupling defaults to zero until supported by data.

Reproduce these priors from the workspace root (this overwrites the output file,
so use a separate path if you have started identifying coefficients):

```bash
python3 src/riptide_simulator/c_simulator/scripts/generate_hydro_prior.py \
  --vehicle-config src/riptide_core/riptide_descriptions/config/talos.yaml \
  --legacy-simulator-config src/riptide_core/riptide_descriptions/config/simulator.yaml \
  --output /tmp/talos_prior.yaml
```

Matrix entries are row-major; added mass and linear damping accept either nested
6×6 arrays or 36 flat entries. Translational added mass is kg, rotational added
inertia kg m², and mixed blocks kg m. Linear drag has force/velocity and
torque/angular-velocity units; quadratic drag uses their squared velocities.
COB and damping-center offsets are body vectors **relative to COM**. Vehicle YAML
mounts and `base_link` are CAD-origin coordinates; the adapter subtracts COM.
Sensor mounts and rates come from the vehicle and legacy sensor configuration.
The simulation-only [`robots/talos/config/sensors.yaml`](../robots/talos/config/sensors.yaml) supplies `imu_gravity`,
`imu_yaw_drift`, `dvl_noise_stddev` and `dvl_variance`; select another profile with the
`sensor_config` launch argument. Talos's IMU output uses its existing EKF gravity
calibration (9.755455 m/s²), while plant dynamics retain physical gravity.
Using 9.80665 in that sensor output left a false 0.051195 m/s² acceleration after
gravity removal. This correction changes the sensor message, not EKF settings.
The DVL uses a configurable 0.001 m/s bottom-lock white-noise prior (previously
0.005) and matching variance 1e-6 (m/s)². These are simulation assumptions, not
measured hardware specifications; calibrate them against stationary DVL logs.
Neither the DVL nor FOG model adds a constant bias or random-walk drift term.
The Talos profile also sets `imu_yaw_drift` to zero (degrees/minute), disabling
the legacy 1.5 degree/minute orientation ramp inherited from the vehicle YAML.
IMU sample noise remains enabled. A custom profile can restore an imposed ramp
for degraded-sensor testing; vehicles without this override retain their configured drift.
The separate `gyro/twist` stream supplies the FOG yaw rate expected by Talos's
hardware EKF (which disables VectorNav yaw and yaw rate). It uses the `fog` mount,
defaults to 500 Hz, and reports covariance matching its simulated noise.
`gyro_noise_stddev` defaults to 0.000174533 rad/s (an unvalidated white-noise
prior, not a measured FOG drift model); `gyro_rate` and `gyro_variance` are also
launch parameters. Unless explicitly overridden, the reported gyro variance is
`max(1e-9, gyro_noise_stddev²)`, approximately 3.046e-8 (rad/s)² with the default
noise. The previous 0.0001 variance understated the simulated FOG's precision.
The normal sensor-noise switches apply to the FOG as well. Simulation uses the regular
vehicle EKF configuration without additional overrides or continuous reseeding.
The one-time startup/reset pose alignment is described below.

For an end-to-end position regression with the actual controller, DVL, and EKF:

```bash
ROS_DOMAIN_ID=134 python3 src/riptide_simulator/c_simulator/test/ros_navigation_drift.py
```

This runs bringup without camera rendering or RViz (`with_rviz:=False`), verifies
the initial reset, holds position, follows a closed XY path while turning, then
holds again with sensor noise enabled. It compares timestamp-aligned estimates
with independent ground truth and checks DVL velocity at its physical mount.
It writes JSON metrics, NPZ samples and a launch log under `/tmp/riptide-navigation-drift.*`.
Use `--with-viewer --duration 300` for a five-minute run with rendering and RViz,
or `--no-noise --duration 60` to check deterministic sensor/estimator errors.
Use `--passive --duration 180` to let the unpowered vehicle surface and tilt;
this exposes sustained false-acceleration drift that a level maneuver can miss.
Use `--launch-arg sensor_config:=/path/to/sensors.yaml` for profile comparisons.

For distance and flip regressions without production controllers or their
persistent tuning, use the test-only truth-feedback thruster driver:

```bash
ROS_DOMAIN_ID=134 python3 src/riptide_simulator/c_simulator/test/ros_navigation_drift.py \
  --duration 70 --distance 8 --flip-axis pitch --report-only --launch-arg collisions:=false
```

This crosses the pitch poles, then drives 8 m and holds. Use `--flip-axis roll`
for a barrel roll, or omit the flip and add `--heading 90` to test a different
travel direction. Truth feedback commands only the test driver's thrusters;
navigation still receives the simulated sensor streams and initial reset only.
The existing hardware EKF currently fails this pitch-flip regression; removing
`--report-only` makes the navigation error limits enforce that failure. The
simulator does not replace the estimator or correct its pose during motion.
In a 70 s noise-disabled pitch-flip run followed by 8 m of travel, the final
heading error was 5.88 degrees and horizontal error was 0.77 m despite accurate
FOG/DVL measurements. A level 12 m run at a 90-degree heading ended within 0.03 m.
The [upstream EKF prediction](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ekf.cpp)
uses Euler angles with `1 / cos(pitch)` and `tan(pitch)` terms, which are singular
at +/-90 degrees. Fixing that estimator limitation belongs in shared navigation;
changing simulated sensor data to hide it would invalidate this regression.

## Actuation, integration and contact

Commands are the existing per-thruster `thruster_forces` array in newtons. Each
thruster has transport delay, separate rise/fall time constants, forward/reverse
scaling and limits, deadband, slew limit, and efficiency (zero disables it).
The core supports per-thruster parameters; the ROS YAML currently shares dynamics
parameters and exposes individual `thruster_efficiencies`.

A 0.5 s stale-command watchdog clears targets and queued commands. Software kill
clears them immediately, while realized force decays with motor response. A
geometric submerged-propeller disk fraction reduces applied force near the surface.
This is an approximation to ventilation, not a propeller/wake model.

The default body step is fixed at 2 ms with quaternion RK4. Actuators advance half
a step, hold midpoint force for the body step, then advance the remaining half:
the combined actuator/body integration uses second-order splitting. Core body
rollouts with fixed wrench use RK4. Contact impulses use the full inverse mass,
restitution and bounded tangential friction. Legacy static task collision boxes
are aligned to the same mapping configuration as the renderer; pool floor/walls
use its exact corner-to-map transform. Task boxes remain coarse proxies. Crate floors/walls are generated from the shared
task configuration. The lattice is treated as a solid wall for contact.

Wall-clock mode catches up at most 20 steps before warning and dropping backlog.
`simulator/time` reports actually integrated seconds. `use_sim_time:=true` consumes
an external ROS clock; this simulator does not publish `/clock`. Sensor timers
remain wall timers, so external-clock pauses may repeat sensor timestamps. Use
core rollouts for deterministic offline optimization; live ROS scheduling is not
bitwise deterministic, even with a fixed random seed.

## ROS usage and controller development

Build from the release workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select c_simulator camera_faker \
  --allow-overriding c_simulator camera_faker --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch riptide_bringup2 simulation.launch.py robot:=talos
```

Restart existing simulator processes after rebuilding. For a standalone plant,
use `ros2 launch c_simulator physics_simulator.launch.py`; normal navigation
provides robot-state and joint-state publishers needed for sensor mount TF.
Launch arguments include `hydrodynamics_config`, `mapping_config`, `physics_step`,
`sensor_noise`, `sensor_config`, `random_seed`, `collisions`, `with_tasks`, `task_config`, and the
existing `sync_odom`. Torpedoes, markers and scoring run in a separate CPU node
using the integrated plant clock; see [TASKS.md](TASKS.md).
`sync_odom:=true` makes sensor observations idealized; leave it false for normal
EKF/controller testing. `sensor_noise:=false` independently disables noise.
Node parameters `contact_friction` and `restitution` control contact response.

Existing IMU, DVL, depth, state, telemetry, kill and reset interfaces are preserved.
Additional topics under `/talos`:

| Topic | Meaning |
| --- | --- |
| `simulator/ground_truth` | `nav_msgs/Odometry`: base_link pose in map; body twist at base_link |
| `simulator/actual_thruster_forces` | Actuator force after lag/limits, before submerged-disk reduction |
| `simulator/time` | Integrated model time in seconds, resets with vehicle reset |

`set_sim_pose` accepts a base_link pose in map or a transformable named frame. It
clears velocities and actuator history, and does not require the EKF to be running.
At startup and after a reset, the simulator waits for EKF odometry, its reset
service, and the map-to-EKF-world transform, then initializes the EKF once from
the current base-link pose. The request is expressed in the EKF world frame so
its own TF listener does not race startup. This aligns the robot and simulator
CAD origins without continuously feeding truth into navigation; subsequent
sensor noise and drift remain visible. Physics can run without an EKF, and a
late-starting EKF receives the current pose rather than a stale reset pose.

The exported C++ library is `marine_dynamics`. Include
`c_simulator/MarineDynamics.h`; call `configure`, `configureDamping`, and
`configureHydrostatics`, then `derivative` or `step` with a COM body wrench.
Consumers can use `find_package(c_simulator REQUIRED)` and link `marine_dynamics`.
The library uses doubles/Eigen, not automatic differentiation or a CasADi graph.
It provides a tested reference for future MPC/NMPC prediction and finite-difference
linearization; identification and controller integration remain separate work.

## Validation and remaining limits

```bash
ctest --test-dir build/c_simulator -R 'test_marine_dynamics|test_collision_box|hydro_prior|payload_model' --output-on-failure
ROS_DOMAIN_ID=129 python3 src/riptide_simulator/c_simulator/test/ros_physics_smoke.py
ROS_DOMAIN_ID=132 python3 src/riptide_simulator/c_simulator/test/ros_task_smoke.py
ROS_DOMAIN_ID=133 python3 src/riptide_simulator/c_simulator/test/ros_origin_alignment.py
```

Tests cover Coriolis energy neutrality, coupled free-motion energy conservation,
drag dissipation, integration refinement, uniform/accelerating current handling,
buoyancy/centroid geometry, delayed actuation, limits, kill/watchdog, collision
geometry, reset/frame alignment, sensor rates and DVL covariance placement.
The origin regression uses the real EKF with delayed startup and a nonidentity
map/odom transform, and verifies that truth is only injected at startup/reset.
Camera integration has its own tests in `camera_faker/test`.

Fully submerged maneuvering is the model's main intended regime. Added mass remains
constant through surfacing; there is no slamming, radiation memory, resolved
free-surface flow, wall proximity hydrodynamics, thruster inflow/wash interaction,
cavitation, battery model or flexible tether. Torpedo passage and marker landings
are scored by the task node, with simpler fixed-axis projectile dynamics. Vehicle
contact uses coarse hull boxes plus CAD claw/held-prop hulls against the table
and baskets. These contacts resolve within the Fossen step using its full inverse
mass; the tabletop has solid backing shared with the prop solver. See TASKS.md
for grasping and the limits of payload/contact coupling. Flexible vinyl is not modeled.

Next calibration steps: measure mass/displacement and static trim; calibrate
individual thruster force/lag in both directions; collect coast-down and driven
maneuvers in all six axes; fit positive mass/damping with confidence bounds; then
validate on separate trials. Acceleration alone identifies effective rigid plus
added inertia: separate them using reliable CAD/bench rigid inertia. Log commands,
body rates, orientation, depth/DVL and timestamps; vary plant parameters while
keeping controller parameters fixed when testing robustness.
