# Torpedoes, markers and bins

For handbook points, roles, the octagon and manual run timing, see
[RoboSub 2026 run scoring](SCORING.md). The original event counters below remain
available separately from the point ledger.

`physics_simulator.launch.py` starts `task_simulator.py` by default, so the normal
`riptide_bringup2 simulation.launch.py robot:=talos` entry includes it. Disable
the task node with `with_tasks:=false`. It consumes vehicle ground truth and the
plant's integrated `simulator/time`; payloads pause with that clock. It requires
fresh ground truth to fire. Resetting the vehicle clears payloads and scores,
reloads both mechanisms and disarms them.

In the viewer, open **Tasks & claw**. Release the simulator's software
kill using the existing vehicle controls, arm the actuators, then fire or drop.
There are two torpedoes and two markers by default, all visible in their loaded
positions. Use **Focus → Payloads** / **Inspect launcher** to see the mechanism.
Firing removes the selected loaded round and starts its flight at the same pose;
it does not teleport a generic projectile outside the launcher. Reload refills both and
disarms them; existing payloads and scores remain. **Reset all tasks** clears
released torpedoes/markers and scores, restores magnet lights and table props,
reloads both mechanisms and disarms them. It works while paused or killed and
keeps the vehicle pose and simulation clock unchanged. Demo
mode is a visual preview and disables actuator controls.

## ROS interfaces

Names below are relative to `/talos`. Existing actuator APIs are preserved:

| Interface | Type / behavior |
| --- | --- |
| `command/actuator/arm` | `std_srvs/SetBool` service and `std_msgs/Bool` topic |
| `command/actuator/torpedo` | `std_srvs/Trigger` service and `std_msgs/Empty` topic |
| `command/actuator/dropper` | `std_srvs/Trigger` service and `std_msgs/Empty` topic |
| `command/actuator/notify_reload` | `std_srvs/Trigger` service and `std_msgs/Empty` topic |
| `simulator/reset_tasks` | `std_srvs/Trigger` service and `std_msgs/Empty` topic; reset all task state, reload and disarm |
| `command/simple_torpedo_fire`, `command/simple_dropper_fire` | `std_msgs/Empty`; auto-arm then fire, matching the simple hardware command |
| `state/actuator/status` | `riptide_msgs2/ActuatorStatus`; counts, armed/busy state |
| `state/actuator/busy`, `state/actuator/cmd_status` | `std_msgs/Bool`; busy and topic-command feedback |
| `simulator/projectiles` | `visualization_msgs/MarkerArray`; map poses rendered by the viewer |
| `simulator/task_events` | JSON in `std_msgs/String`: id, kind, slot, result, target, integrated time |
| `simulator/task_score` | JSON cumulative success, wrong_target, blocked and miss counts |

Software kill disarms actuators. Services report rejection reasons for kill,
disarm, empty ammunition, cooldown and stale/missing vehicle state. Topic failures
publish false feedback and log the reason. Each payload contributes at most one
score, even if it subsequently lands on the floor. The viewer retains the latest
events. The renderer retains at most 64 released objects plus the remaining loaded rounds.
Marker namespaces `torpedo_loaded` and `dropper_loaded` identify loaded slots
0/1; `torpedo` and `dropper` identify released rounds. Markers include the same
CAD mesh URI, scale and orientation for RViz and the OpenGL viewer.

## Shared geometry and assumptions

[`talos_tasks.yaml`](../config/talos_tasks.yaml) supplies geometry to physics,
tasks and rendering. Pass the same `task_config` and `mapping_config` to separately
launched nodes. Normal combined bringup shares these launch arguments. Configuration
changes take effect on restart; viewer depth-noise controls are separately live.

The mechanism follows the team's design reports:

- [2024 TDR, Section III.A.2 / Figure 1](https://robonation.org/app/uploads/sites/4/2024/07/RS24_TDR_OhioStateUniv-compressed_1-1.pdf):
  the launcher has forward-facing torpedoes and downward-facing markers, coupled
  through bevel gears and a single servo.
- [2025 TDR, Sections I.C and II.D](https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_Ohio-State-University-Underwater-Robotics-compressed.pdf):
  forward firing and downward marker release remain separate task actions.
- 2026 TDR, Section II.A.2 / Figure 4, read from the local competition submission
  `RS26-0000000046_202607222337.pdf` (PDF page 106): the torpedo keeps the same
  exterior as the sinking marker, adding an internal cavity for neutral buoyancy.

The extracted Talos3 projectile is approximately 83 × 26 × 26 mm with a watertight
CAD volume of 12 cm³. Both types share that finned mesh, dimensions, displacement
and exterior drag. Torpedoes use `neutral_buoyancy: false` and a 12.22 g mass,
about 2% heavier than their displaced water in the default freshwater profile.
This follows the team's observed slightly negative buoyancy: approximately two
feet of mostly straight travel, then a nose-down descent. Their center of mass
is 20 mm forward of the mesh center and center of buoyancy is 20 mm aft.
Rear drag and angular damping stabilize the initial flight while allowing the
balance torque to pitch the nose down. These offsets and coefficients are tuning
priors, not measured mass properties. Markers use a full-print mass prior of
14.88 g (1240 kg/m³ plastic density). No measured print masses are available.
The optional `neutral_buoyancy: true` mode still sets mass to water density ×
displacement; the default torpedo instead keeps its fixed mass as density changes.

Actuator reference frames follow `vehicle_config`, as in `robot_actuators.xacro`.
These aiming references are **not projectile centers**. Each `slot_offsets` entry
specifies a loaded center/orientation relative to its actuator TF: torpedoes use
`torpedo_0_link` / `torpedo_1_link`, and markers use `droppers_link`. Offsets are
xyz metres and rpy radians, with the mesh nose along local +X.

The supplied offsets restore all four original CAD seats in the extracted
launcher. Both the viewer and release physics compose the same transforms, so a
round leaves its visible seat continuously. Robot-config pose/baseline changes
still propagate through the mounts. The launcher housing retains its CAD shape
and placement. Restart after configuration changes.

The CAD contains four matching springs. Both simulated mechanisms therefore
receive the same configurable effective `spring_energy` at release, producing
speed `sqrt(2*energy/(mass+added_mass))`; this models a downward ejection followed
by sinking for markers. The 0.047 J starting estimate gives approximately 2.49 m/s
for torpedoes and 2.29 m/s for markers in the default freshwater profile.
This is an effective release impulse, not a model of cam/servo motion or a measured
spring constant. It includes no asserted prediction of real exit velocity.

The existing torpedo texture had printed circles over solid triangles. Rendering
now discards the inner ring regions in color, depth and shadow passes. Shared
normalized texture coordinates define four circular openings for passage checks.
A swept center segment detects panel crossings; the projected projectile radius
must clear a hole. The selected class (default `fire`) scores success; another
class scores wrong_target. Contact with the remaining vinyl scores blocked.
Existing mapping hole frames are unchanged; scoring uses the actual vinyl UV
geometry rather than those older approximate hole transforms.

Each of the four existing bin vinyls is treated as a crate's interior floor.
The navy procedural crates use [CleverMade's published 25 L dimensions](https://www.clevermade.com/products/clevercrate-milk-crate):
13.12 × 13.12 × 11 inches outside, 12 × 12 × 10.68 inches inside. White liners
cover half the wall height, as specified for the course; their 4 mm thickness is
an adjustable assumption. A marker must enter the top opening with clearance and
land inside to score. Mapping class determines success versus wrong_target.
Rim hits are blocked; a landing outside a crate is a miss. Crate walls/floors also
participate in coarse vehicle contact. Lattice openings are visible but contact
treats walls as solid proxies.

Payloads inherit vehicle translation and angular mount velocity, then use CPU
RK4 with gravity, buoyancy, added mass, quadratic axial/lateral drag and the
configured current. Torpedoes also inherit angular velocity and integrate their
orientation with buoyancy and drag moments about the COM, approximate cylinder
inertia and angular damping. The moving orientation drives both rendering and
contact clearance. In a level, still-water release the defaults drop about
1.8 cm over the first 0.61 m, and descend about 0.98 m by three seconds in free
flight. This is a qualitative fit to the team's description, not a measured
trajectory. `center_of_mass`, `center_of_buoyancy` and `center_of_drag` are axial
offsets in metres from the mesh center (+X nose); `angular_damping` is an effective
pitch/yaw coefficient in N m s/rad, with roll damping scaled by inertia.
**Mass, balance, displacement, delivered spring energy and drag remain adjustable
priors.** They are separate from the vehicle's six-DOF plant.
Markers retain fixed orientation. No launch recoil, vehicle mass change,
flexible vinyl, detailed payload mesh contact or wake interaction is modeled.
Claw and table contacts are described below.
The launcher mesh contains no baked-in ammunition; every loaded and released round
is rendered separately. The default chassis remains the original lightweight model.

## Verification

```bash
ctest --test-dir build/c_simulator -R payload_model --output-on-failure
ROS_DOMAIN_ID=132 python3 src/riptide_simulator/c_simulator/test/ros_task_smoke.py
# Display-backed combined physics/task/camera launch smoke check:
ROS_DOMAIN_ID=133 python3 src/riptide_simulator/c_simulator/test/ros_pipeline_smoke.py
```

The isolated ROS fixture checks all four mounted poses, identical exterior geometry,
continuous release at each slot, torpedo nose-down marker poses, loaded counts, kill/arm, cooldown, ammunition, both torpedo
classes, blocked vinyl, crate entry/landing, rim hits, misses, reload and reset.
These are software/geometry checks, not validation against physical payload tests.

## Magnetically activated bin lights

The two square bin placeholders are replaced by clear-cover Polycase ML-22F
sensor housings with a purple PCB, four screws, cable glands and 16 individual
LED lenses. Their faces tilt **upward 45 degrees**, as in the competition
reference. The existing mapping target frames are preserved; `face_pose` in
`magnet_lights` applies the visual/sensor tilt relative to those frames.
The robot carries a visible printed stick and magnet at its configured
`vehicle_config.magnet.pose`, transformed through the CAD origin/base-link
offset exactly once. `magnet_lights.robot_tip_offset` places the simulated magnet
5 cm above that TF; the rod is shortened by 5 cm to preserve its upper attachment.
The same offset applies to rendering and light activation.

Both boxes start red by default. Keeping the magnet tip within **0.1524 m
(six inches)** of a sensor for **0.5 seconds of simulation time** turns that
box green. Leaving range before the dwell completes restarts the timer. Green
latches after the magnet leaves. The sensor is modeled 8 mm behind the lid;
the range is measured to that point. This is an empirical proximity model based
on the team's magnet test, rather than a dipole-field/polarity simulation.
As a permanent magnet, it works independently of actuator arming and kill.
Missing/stale ground truth breaks an unfinished dwell; pausing simulation time
pauses the activation timer.

The behavior follows the user's competition specification. The linked
[RoboNation bench firmware](https://github.com/robonation/Underwater-Magnetically-Activated-Light)
also has flash/rearm and diagnostic modes; those are not used here. Each box
has its own state. Ammunition reload leaves the lights alone. Vehicle/course
time reset restores configured initial colors, as does the explicit service:

```bash
ros2 service call /talos/simulator/reset_magnet_lights std_srvs/srv/Trigger '{}'
```

`simulator/magnet_lights` publishes a `MarkerArray` with target names and colors;
the viewer applies these to the actual LED meshes in observer, FFC and DFC views.
`simulator/task_events` emits one `kind: magnet`, `result: activated` event per
activation. There is no fabricated perception output. Open **Tasks & claw →
Inspect lights** for a close view. Light colors also appear in that tab.

Tune `magnet_lights` in `talos_tasks.yaml`: initial target colors, trigger
distance, dwell time, sensor inset, robot tip offset, face tilt and
`led_radiance` (HDR brightness). Restart after edits. Bright lenses produce
camera bloom indoors and outdoors; the clear cover remains visible and supplies
the enclosure's metric depth. Appearance/material parameters are rendering
priors, not calibrated radiometry. See
[asset provenance](../../camera_faker/models/magnet_lights/README.md).

```bash
ctest --test-dir build/c_simulator -R '^magnet_lights$' --output-on-failure
ROS_DOMAIN_ID=138 python3 src/riptide_simulator/camera_faker/test/ros_magnet_smoke.py \
  --weights src/riptide_perception/tensor_detector/weights/rs26_ffc_woollett.pt
```

The display-backed test checks both targets, the six-inch boundary, dwell,
interruption, paused time, latching, ammo reload, explicit/course reset, rendered
red/green RGB and cover depth. Optional `--weights` runs the actual detector on
saved RGB images and writes detections to `/tmp/magnet-validation/report.json`;
that report distinguishes detector performance from simulation correctness.

## Claw and table objects

Install the CPU contact dependencies before building (already installed on this
machine):

```bash
sudo apt install libbullet-dev
python3 -m pip install -r src/riptide_simulator/c_simulator/requirements.txt
```

The normal bringup launches this automatically. `task_simulator.py` runs a Bullet
DIRECT contact world on the plant's integrated simulation time. The four table
props are dynamic rigid bodies with gravity, approximate buoyancy (including
emergence), water drag, rotation, friction and restitution. They collide with
one another, the table, basket bottoms/walls, pool walls/floor and the claw.
The viewer and both cameras follow their simulated poses; held props follow the
same per-frame vehicle transform as the claw.

The ribbed TPU pad mesh was extracted from the supplied native SolidWorks CAD;
[asset provenance](../../camera_faker/models/claw/README.md) records the conversion.
Two opposing position-held rack drives move rigid convex pad envelopes. Their
opening changes only under actuator commands; commanded travel stalls at solid
contacts. Table impacts cannot back-drive the jaws. Grasping
requires sustained contact with **both** jaws on the same object. A finite-force
constraint then holds the existing relative pose (no snapping); opening releases
it, and excessive constraint error produces a `slipped` event. Internal pad/object
contacts are disabled while held so they do not fight the grasp constraint;
contacts with the environment remain active. Objects can tip,
fall off the table, strike a basket rim, or settle inside a basket. A closed empty
claw never acquires nearby objects merely by moving close to them.

The claw starts closed and returns to closed when tasks or table props reset.
Enable the vehicle in RViz and arm actuators, then use **Tasks & claw → Open
claw / Close claw / Stop claw**. Stop holds the current jaw opening; **Inspect
claw** centers the observer on the mechanism.
The existing hardware/autonomy commands are supported under the robot namespace:

| Interface | Meaning |
| --- | --- |
| `command/actuator/claw` Bool topic or SetBool service | `true` opens, `false` closes |
| `command/actuator/claw_move_s` Float32 topic | Positive seconds open, negative close; zero stops |
| `simulator/reset_table` Trigger service | Restore four props and close claw, clearing grasp state |
| `simulator/task_objects` MarkerArray | Current prop poses, body-relative while held |
| `simulator/claw_joints` Float64MultiArray | Actual left/right rack travel in metres |
| TF `simulator/<robot>/claw_tool` | Bottom-center tool reference relative to `<robot>/base_link` |
| `state/actuator/status` | Opening/closing/open/closed/disarmed status |
| `simulator/task_events`, `simulator/task_score` | Grasp, release, slip, surface and delivery results |

Timed commands stop after the requested *simulation* duration, including partial
travel. Kill/disarm or stale vehicle state stops commanded jaw travel. Reloading
payloads does not reset the table. Resetting plant time also resets the table.

The default configurable sorting is pill/bandage → helmet (medical basket),
nut-and-bolt/plug → warning. A delivery is scored once after the released object
settles inside the basket and contacts it; the wrong basket is `wrong_target`,
and settling on the pool floor after pickup is `miss`. Surfacing emits its own
event; it is not currently required for a delivery score. Re-grasping permits a
new delivery attempt. Edit `claw.props.*.basket` for a different course assignment.

The table has raised corner posts. Narrow the jaws before descending near the
nut-and-bolt or plug; a fully open jaw can hit a post and push the object away.
Approach the nut-and-bolt near the fingertips and move clear of the post before
lifting. The contact regression uses opposing pad contacts to establish each
grasp, then retains external collisions throughout lifting and delivery.

Limits: TPU deformation is not modeled. Pad collision envelopes fill small rib
recesses/holes; props use convex collision hulls, while table and baskets retain
triangle geometry. Mount, carriers and racks use the saved SolidWorks assembly
placements, but only pads have claw collision bodies. The original pads are not
reshaped or independently rotated. The simulator bottom-center tool reference
is `claw.pose` in the task YAML; hardware calibration files are unchanged.
Jaw speed, holding force, friction, prop mass/volume and drag are estimates.
Claw drives currently model ideal position holding and contact-limited travel,
not a measured servo torque/compliance curve.

Vehicle contact now runs synchronously inside the Fossen physics step. The
`TaskContacts` collision world uses the CAD pad hulls, table/basket meshes and a
solid tabletop backing shared with the prop world (`table_collision`). Claw and
held-object contacts apply separation and normal/friction impulses to the whole
vehicle using its full inverse mass matrix, including added mass and inertia.
A supported free prop also blocks downward crushing by the vehicle. Jaws keep
their commanded separation during an impact. Collision correction runs before
and after integration so penetrating vehicle poses are not published.

Free-prop support checks in the vehicle solver use the solid tabletop backing,
floor, and basket surfaces. Detailed table legs and hardware remain active for
vehicle/carried-object collisions and in the dynamic prop solver. The vehicle
solver avoids duplicating expensive free-prop/triangle checks at every step.

This is not a fully coupled multi-body hydrodynamics model: payload weight,
buoyancy and drag do not yet change the vehicle plant's coefficients. Free props
still receive lateral pushes in the prop solver without feeding their small
momentum changes back to the vehicle. Mount/rack visuals have no separate contact
shapes. TPU grip strength and servo compliance remain unvalidated.

Validation: `test_claw_world.py` exercises real mesh contacts, all four prop
shapes, lifting/releasing, surface events, correct/wrong baskets, pool-floor
misses, wall rejection, empty grasps and motor stop. `ros_pipeline_smoke.py`
checks live physics, both cameras, dynamic-object topics, claw services and timed
commands in an isolated ROS domain.

`test_task_contacts.cpp` checks open/closed jaw impacts under sustained pressure,
upward release, held-object contact and supported-prop crushing prevention.
`ros_claw_contact_smoke.py` drives the live Fossen plant with thruster commands
against the tabletop and checks penetration, velocity and unchanged jaw gap.
