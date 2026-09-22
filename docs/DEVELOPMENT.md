# How the simulator works

A run combines four inputs: a **robot** describes hardware, a **year pack** describes
tasks and scoring, a **world** describes the pool, and a **scenario** puts them
together. `robot:=` and `year:=` are independent; changing one keeps the other.
See [the configuration guide](CONFIGURATION.md) for edits and examples.

## Startup and the simulation loop

1. Launch reads the selected manifests and scenario. The profile loader checks
   compatibility and writes one resolved set of YAML files for all nodes.
2. The physics node integrates the vehicle and thrusters, handles vehicle
   contacts, and publishes ground truth, sensor readings, TF, and `/clock`.
3. The selected task node consumes ground truth and elapsed simulation time.
   Robot behavior handles mechanism commands; year behavior detects interactions
   and awards points. The existing task runtime also simulates payloads and props.
4. The viewer draws the same scene, renders camera images at the physical camera
   poses, and displays task state and scores. Its buttons send commands to ROS.

The plant owns vehicle motion; task/contact runtimes own their payloads and props.
The viewer does not decide physical outcomes or award points. Keep that ownership
when adding behavior, so two components do not integrate the same object.

## Code map

Paths below are relative to the repository root.

| File or folder | Read it when changing… |
| --- | --- |
| `c_simulator/riptide_sim_config/profiles.py` | Profile loading, validation, assets, or resolved YAML |
| `c_simulator/riptide_sim_config/launching.py` | Selection forwarding and launch defaults |
| `c_simulator/launch/` | Which simulator nodes start |
| `c_simulator/src/physics_simulator.cpp` | Physics scheduling, ROS interfaces, resets, or vehicle contacts |
| `c_simulator/src/robot_class.cpp` | Vehicle parameters, mounted sensors, or body/COM transforms |
| `c_simulator/src/marine_dynamics.cpp`, `thruster_dynamics.cpp` | Vehicle equations or motor response |
| `c_simulator/scripts/task_simulator.py` | The entry point that loads the selected behavior |
| `c_simulator/robots/<robot>/behavior/` | Native mechanism commands and robot-specific behavior |
| `c_simulator/tasks/<year>/behavior/` | Task interactions and that year's scoring rules |
| `c_simulator/scripts/payload_model.py`, `claw_world.py`, `magnet_light_model.py` | Shared interaction models |
| `c_simulator/src/task_contacts.cpp` | The existing claw/prop contact bridge into vehicle physics |
| `camera_faker/src/pool_viewer/main.cpp` | Camera publication, TF, viewer controls, or ROS commands |
| `camera_faker/src/pool_viewer/camera_processor.cpp`, `camera_cuda.cu` | CPU/CUDA selection, camera JPEG, preview/cloud generation, or fallback |
| `camera_faker/src/pool_viewer/renderer.cpp` | Scene construction and rendering |
| `camera_faker/include/pool_viewer/`, `shaders/pool/` | Camera geometry, depth effects, and shaders |

The complete UWRT entry point lives in the separate `riptide_launch` repository,
under `riptide_bringup/launch/simulation.launch.py`.

## Conventions worth preserving

**Time:** `/clock` timestamps ROS data; `simulator/time` is elapsed plant time used
by behavior. Advance tasks from simulation time, reset on rewind, and publish a
reset result immediately even while paused. Use wall time only where wall time
is intended, such as checking whether vehicle state has stopped arriving.

**Frames:** world Z points up. Physics stores the vehicle pose at its center of
mass; published ground truth and pose-reset requests refer to `base_link`.
Vehicle mounting coordinates are relative to the CAD origin. Ground-truth TF is
under `simulator/<namespace>/…`; UWRT estimated TF remains a separate tree.
Generic camera messages use truth optical frames, while UWRT cameras retain
estimated optical frame IDs. Neither convention changes the physical render pose.

**Configuration:** edit the source profiles, then restart. Resolved files live in
`${ROS_HOME:-~/.ros}/riptide_simulator/runs/<config-id>/`; inspect these when a
setting appears wrong. They are generated output. Behavior source hashes are
recorded, so even a comment-only behavior edit can produce a new run ID.

## Checking a change

Build from the workspace root using the command in the [README](../README.md).
Existing YAML/Python files usually update immediately with a symlink install;
rebuild when adding files or changing C++. Always restart running nodes.

Run the existing functional tests:

```bash
colcon test --packages-select c_simulator camera_faker \
  --ctest-args -E 'lint|flake8|pep257|copyright|uncrustify|cppcheck'
```

That command excludes lint checks. `pool_camera_processor` checks CPU depth, cloud
coordinates/colors/padding, JPEG decoding, preview orientation, and recovery from
an injected GPU failure. `pool_camera_cuda` checks the same outputs on CUDA and
skips when no CUDA device is available. It also exercises RGB-only acquisitions,
resizing, cropped inputs, and concurrent cameras. Run it on an NVIDIA machine
when editing `camera_cuda.cu`; CPU-only CI cannot validate GPU execution.

For a live profile/behavior change, also run:

```bash
ROS_DOMAIN_ID=179 python3 src/riptide_simulator/c_simulator/test/ros_profiles_smoke.py
ROS_DOMAIN_ID=180 python3 src/riptide_simulator/c_simulator/test/ros_profile_matrix_smoke.py
```

The first checks cameras, mechanisms, scoring, propulsion, pause, and reset. The
second checks different robot/year combinations and a staged zero-camera profile.
Both need a display. The `test/` directories also contain focused Talos physics,
scoring, reset, and camera smoke tests. Use separate nonzero ROS domains for
simultaneous tests.

Python formatting follows `pyproject.toml` (Black); C++ follows `.clang-format`.
Format files you edit, keep comments focused on units, ownership, or non-obvious
choices, and leave vendored code alone.
