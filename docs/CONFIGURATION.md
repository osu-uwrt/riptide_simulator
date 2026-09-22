# Changing and extending the simulator

Most changes belong in YAML. Add code when a mechanism needs different behavior
or a task needs a new interaction or scoring rule. Start with the working example
profiles: the [example robot](../c_simulator/robots/example_auv/robot.yaml) and
[example task pack](../c_simulator/tasks/example/competition.yaml) are small
enough to read end to end.

All paths below are relative to `c_simulator/` unless stated otherwise.

## Where to make a change

| Change | File to edit |
| --- | --- |
| Mass, thrusters, COM, or sensor mounts | The vehicle YAML referenced by `robots/<robot>/robot.yaml` |
| Drag, buoyancy, or motor response | `robots/<robot>/config/hydrodynamics.yaml` |
| Sensor noise or drift | `robots/<robot>/config/sensors.yaml` |
| Camera CPU/CUDA selection, depth noise, or water appearance | The robot manifest’s `viewer.camera_settings` YAML (Talos uses `camera_faker/config/cameras.yaml`) |
| Cameras, calibration, or model assets | `robots/<robot>/robot.yaml` and its referenced files |
| Launcher, gripper, or other equipment | `robots/<robot>/config/equipment.yaml` |
| Course layout and visuals | `tasks/<year>/config/`; 2026 uses `mapping.yaml`, `markers.yaml`, and `scene.yaml` |
| Task dimensions or interaction thresholds | `tasks/<year>/config/tasks.yaml` |
| Points and supported scoring limits | `tasks/<year>/config/scoring.yaml` |
| Run options, labels, and defaults | `tasks/<year>/competition.yaml` → `ui` |
| Pool size, water level, or current | `worlds/<pool>.yaml` |
| Starting pose or a practice variation | `tasks/<year>/scenarios/<scenario>.yaml` |

For example, change `points.gate` in the 2026 scoring file to change the gate
award. Heading/role coin-flip defaults live in the 2026 manifest's `ui.run_options`.
The old aggregate configs in `config/` and the repository root remain regression
fixtures/legacy inputs; normal launches use the profile folders above.

Simulation bringup loads the mapping node's initial estimates from
`riptide_mapping2/config/config.yaml`, separately from the simulator's true course
poses in `tasks/2026/config/mapping.yaml`. Use `mapping_config_yaml:=/path/to/file.yaml`
to override the mapping node's config; `mapping_config:=/path/to/file.yaml` overrides
the simulator course layout. Both `simulation.launch.py` and
`simulation_no_autonomy.launch.py` keep these inputs separate.

Restart after editing YAML. With a symlink install, existing files usually need
no rebuild; adding files requires rebuilding the packages. Use the
[build and test commands](DEVELOPMENT.md#checking-a-change) after a change.

## Add a robot

1. Copy `robots/example_auv/` to `robots/my_auv/`; set `robot.yaml` → `id: my_auv`.
2. Point `vehicle` at the real robot description, or edit the copied vehicle YAML.
   Set mass, COM, `base_link`, thrusters, and mounts. Provide appropriate visual
   and collision assets and hydrodynamic parameters.
3. List the cameras in `robot.yaml`. Each `name` must match a vehicle mount.
   Supply a calibration/config file or direct `intrinsics` with
   `width`, `height`, `fx`, `fy`, `cx`, `cy`, and `rate`. Set `topic_root` and
   `truth_tf_owner` (`physics` or `viewer`). `cameras: []` is valid.
4. Use `adapter: generic` for standalone commands. Configure equipment and
   capabilities; remove the example beacon's `behavior`, `controls`, and
   capability entry if the robot has no such mechanism. Keep an equipment YAML
   mapping, even if it is just `{}`.
5. Rebuild and first try an empty pool:

```bash
ros2 launch c_simulator full_simulator.launch.py robot:=my_auv scenario:=empty_pool
```

Thruster count follows the vehicle list. IMU, DVL, depth, FOG, and acoustics may
be absent. The generic adapter accepts a `std_msgs/Bool` on `simulator/enable`
and a `std_msgs/Float32MultiArray` on `thruster_forces`, containing one force in
newtons per thruster. Topics are under the robot namespace; the command timeout
still applies. `adapter: uwrt` retains the existing kill and telemetry conventions.

For new mechanism logic, register `behavior: behavior/mechanisms.py:Mechanisms`.
The class receives `(node, vehicle, equipment)`. The provided runtimes call
optional `step(simulation_time)` and `reset()` methods. The example beacon shows
a timed command; its manifest also adds a `bool_action` viewer button. A custom
year node must explicitly create and step/reset its mechanisms, as that example does.

Talos's 2026 integration additionally uses `bind(runtime)` and
`payload_mounts(task, kind)` for its payload/contact adapter. Capability names
advertise compatibility; another robot must also implement the required behavior
contract. They do not translate an arbitrary hardware protocol automatically.

## Add tasks or a competition year

For another instance of an existing task, edit that pack's task config. For
example, append an observation region to `tasks/example/config/tasks.yaml`:

```yaml
- id: station_c
  position: [7.0, 3.0, -1.0]
  radius: 0.6
  dwell: 0.2
```

That entry goes under `regions`. Give it a unique ID. Add a corresponding visual
to `config/scene.yaml` if needed; visuals and scoring regions are separate entries.

For different tasks or rules:

1. Copy `tasks/example/` to `tasks/2027/` and set `competition.yaml` → `id: '2027'`.
   The example is an observation challenge, not a template for official rules.
2. Edit task instances, scene geometry, and `config/scoring.yaml`. Register the
   task node with `behavior: behavior/node.py:TaskSimulator`; add or update its
   validator for pack-specific config checks.
3. Implement new interactions in `behavior/`. Keep adjustable dimensions,
   thresholds, and point values in config. List required robot capabilities in
   `required_capabilities`; use `equipment_sections` only for robot equipment
   that this pack composes into its task/contact configuration.
4. Set the title, focus targets, and run controls in `ui`, then choose the world
   and starting pose in `scenarios/default.yaml`.
5. Rebuild and run with an independently selected robot:

```bash
ros2 launch c_simulator full_simulator.launch.py robot:=example_auv year:=2027
```

The task node reads `simulator/ground_truth` and `simulator/time`. It should clear
progress on time rewind and support the `simulator/reset_tasks` Trigger service
and Empty topic, including while paused. Publish state and scores for the viewer;
do not put scoring logic in the viewer.

`simulator/run_command` and `simulator/run_score` carry JSON in `std_msgs/String`.
Commands contain `action` plus pack-specific fields. Score messages contain
`running`, `elapsed`, `total`, and `rows: [{key, label, points}]`.
The manifest's `ui` supports boolean, choice, and numeric run options, action
buttons, extra score fields, and an optional manual adjustment control.
See the example node for the small implementation and the 2026 pack for richer rules.

## Add a pool

1. Copy `worlds/small_pool.yaml` to `worlds/practice_pool.yaml` and change `id`.
2. Set `length`, `width`, `depth`, `water_level`, and `deck_height` in metres.
   Set `water_density` in kg/m³ and current vectors in m/s. Keep the copied
   current amplitude/frequency fields even when the current is zero.
3. Add `tasks/<year>/scenarios/practice.yaml`:

```yaml
schema_version: 1
world: package://c_simulator/worlds/practice_pool.yaml
tasks: false
start_pose: [3.0, 3.0, -1.0, 0.0, 0.0, 0.0]
```

4. Rebuild and launch with `scenario:=practice`. Set `tasks: true` when the course
   is ready for that pool; moving the pool does not automatically reposition its tasks.
   A scenario shared by all years can instead live in `scenarios/`. Shared names
   take precedence over year-local names.

For a practice scenario saved in `tasks/example/scenarios/`:

```bash
ros2 launch c_simulator full_simulator.launch.py robot:=example_auv year:=example scenario:=practice
```

The pool floor is `water_level - depth`; walls lie at pool X/Y coordinates
`0..length` and `0..width`. Physics and rendering read the same dimensions.
For imported 2026 courses, `map_origin_pool` transforms between map and pool
coordinates. Check the course fits after changing pool size or depth.

Add fixed objects to a world's `entities` list, or to a year's scene:

```yaml
entities:
  - id: practice_block
    frame: map
    pose: [4.0, 3.0, -2.5, 0.0, 0.0, 0.0]
    size: [1.0, 0.5, 0.5]
    color: [0.3, 0.7, 0.8]
    collision: true
```

This creates a visible box and vehicle collision proxy with the same pose. With
an optional `mesh`, `size` scales the visual mesh and still defines the box proxy's
full dimensions. Use package URIs for mesh assets: relative entity mesh paths
currently resolve from the year directory, including world-owned entities.
The separate Talos prop solver retains its own supported contact geometry.

## Paths, overrides, and limits

Manifests, worlds, and scenarios use `schema_version: 1`. Profile names use
letters, digits, and underscores. Poses are `[x,y,z,roll,pitch,yaw]` in metres and
radians; `start_pose` locates the COM in map coordinates. Imported UWRT mapping
`pose.yaw` and `map_origin_pool[2]` use degrees.

Manifest references resolve relative to their manifest; scenario `world` paths
resolve relative to the scenario. `package://<package>/<path>` uses installed ROS
resources and is convenient for shared assets. Behavior references add `:ClassName`
or `:function_name` to a Python path.

Scenarios can set `task_overrides`, `scoring_overrides`, `run_defaults`, and
`runtime`. Robot/year selections stay fixed. File launch overrides include
`world_config`, `task_config`, `mapping_config`, `scene_config`,
`hydrodynamics_config`, and `sensor_config`. A `task_config` replaces the task and
composed equipment data before scenario overrides; selected behavior and metadata
still come from the manifests. World water properties remain authoritative.

The 2026 course currently enables as one unit. Python behaviors and box entities
are extensible through config; new compiled contact types need C++ work. Generic
runs can skip UWRT autonomy, but the packages retain UWRT message build dependencies.
