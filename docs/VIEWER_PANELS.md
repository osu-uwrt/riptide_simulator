# Viewer compositions

Operator panels are opt-in extensions. The viewer reads the file selected by
`viewer.panels_config` in a robot profile, or the `panels_config:=/absolute/file.yaml`
launch override. The Talos profile includes Motion, Mapping, Actuators and Autonomy; other profiles
keep their existing empty viewer configuration. Pool and task profiles do not
select control interfaces. Configuration changes take effect on restart.

```bash
ros2 launch riptide_bringup2 simulation.launch.py robot:=talos year:=2026 with_rviz:=false
# Standalone viewer, alongside an already running robot/simulator:
ros2 launch camera_faker pool_viewer.launch.py robot:=talos use_sim_time:=true
# Explicitly omit all operator extensions:
ros2 launch camera_faker pool_viewer.launch.py panels_config:=$(ros2 pkg prefix camera_faker)/share/camera_faker/config/empty_panels.yaml
```

Use wall time (`use_sim_time:=false`) for an actual robot. This adds operator
capabilities to the simulator viewer; its existing camera imagery still depicts
the simulated scene. It does not replace live camera acquisition.

## Interaction

The Kill/Enable toggle stays above the scrolling panel area, including when Motion is
collapsed or hidden. Enable is a request; the separate robot-state line reports observed
kill feedback or unknown. No enable or motion command is sent at startup. Preview
(`demo:=true`) creates the panels without ROS providers.

Motion shows **Actual**, **Commanded**, **Error**, and editable **Target** values in the
displayed fixed frame (metres and degrees). Actual uses the robot's configured base TF,
not simulator truth. Current copies the actual pose into Target. Typing edits only the
draft; Command sends it after the control-mode request succeeds. Dive in place copies
actual X/Y/yaw, levels roll/pitch, and uses the robot-configured Z. It remains available
when the robot is already submerged. It is omitted unless `dive_z` is configured. Current, Command and Dive in place
share one row. The read-only Error column shows Commanded minus Actual; angular errors wrap
to ±180 degrees. Table headers do not highlight. Numbers align right (including idle
target fields) and table text is vertically centered. Current/Command/Dive in place form
a row spanning the panel width, with 40 px tall buttons.

The Position / Feedforward buttons show the selected mode. Selecting Position starts
holding the actual pose immediately. Dragging its arrows or rings then commands
continuously; no Command button is required. Selecting feedforward explicitly changes
controller mode using the current pose. The UWRT feedforward option selects the existing
controller's feedforward mode; it is not a raw force/torque editor. The standard ROS
provider does not offer feedforward. In position mode, the orbit viewport shows XYZ
shafts and RPY rings simultaneously. Translation arrows follow the commanded robot axes;
the centre square translates in its XY plane. Grab an arrow, shaft, or any part of a
ring. Solid rotation rings stay visually simple; the body-relative arrows show orientation.
Dragging around a ring supports continuous full turns in either direction. When a ring
is viewed edge-on, drag along its projected tangent instead. Robot dragging preserves
Follow and uses the camera projection captured at drag start, so camera movement does not change the drag mapping. Escape restores the drag's
starting target. Handles use a fixed size in metres, so they stay proportional to the
robot as the camera zooms. Edited numeric drafts remain intact during a drag.

Autonomy lists trees supplied by its configured service, with searchable friendly names
and full-path tooltips. The dropdown fits the longest tree name. Start requires linked
motion providers to be enabled and fresh. Stop cancels goals on the configured action
server, including trees started by another client. Starting a tree yields manual motion
immediately. Manual motion stays inactive after a tree ends until an explicit
Command/mode selection. The stack fills the remaining panel height. Stack feedback is
read-only; the last stack remains for inspection. External action status also blocks
manual commands. Kill stops the linked active tree as well as asserting kill.

Mapping provides **Tag cal**, **Reset mapping**, and **Set mapping target** using the same
UWRT action/services as RViz. Parent frame, tag frame, and sample count are editable;
the Talos tag-frame default is `estimated_origin_frame`. Calibration reports sample progress and can be canceled, including before the server
accepts the goal. Cancellation affects only this panel's goal. Reset/target requests
report failures or timeouts independently. The observed mapping target and map-lock
state are separate from the editable request; an empty target selects the closest
object automatically. Endpoint names and timeout values live in `uwrt.mapping` options.

Actuators contains only robot command and state interfaces. Its command list, labels,
message types, and armed-only restrictions come from configuration. `uwrt.actuators`
uses the same command topics as RViz and consumes `ActuatorStatus` with sensor-data QoS,
so it accepts best-effort hardware status as well as simulator status. Commands require
fresh status; armed-only commands also require observed arming. Ammunition counts come
from that message. This provider has no simulation task, event, magnet, or joint subscriptions.

Run tracking is a pool-toolbar button alongside TF and Detections. It opens an independent
window with Start/Stop, reset, task-profile run options, custom actions, awards, configured
score fields, and optional manual adjustment. The viewport timer reads the same provider.
`show_scorecard:=true` still opens the configured `run` instance's scorecard at startup.
The `sim.run` adapter owns the JSON command and score topics and rejects commands when
score feedback is stale. Task-specific labels, choices, and `status_fields` remain in
the task's `ui` document. No scoring-year names are embedded in the panel.
The scorecard sizes to its full content on opening, limited to the application work area;
smaller windows scroll, and manual resizing remains available.

Run tracking also owns task summaries, magnet target status, recent events (last five),
and simulated jaw-gap readings. Its optional `task_score_topic`, `events_topic`,
`lights_topic`, and `joints_topic` configure these inputs. Reset events clear prior
history. Configured scene inspection buttons live here under Inspect scene, using
`ui.run_inspections`; Reset run & tasks is likewise a simulator-only action.

Mapping and Actuators start collapsed to keep Motion and Autonomy visible.
Expand their headers or use the pool toolbar's **Panels** menu to change visibility;
set `open`, `visible`, and ordering in the composition for another default layout.

The operator sidebar defaults to 29% of the window until manually resized. Both sidebars resize from a 300 px minimum. The camera sidebar defaults to 29% of the
window, clamped to 335–445 px, until manually resized. The course map scales in both
dimensions with the camera sidebar while preserving the configured pool proportions. Pull either divider toward its
window edge below 80% of the minimum width to snap it closed. The gesture stays captured
until release, so pulling back out restores the panel. A hidden sidebar can be reopened
by dragging its edge inward (right for operator panels, left for cameras). Resize
handles, scrollbars and panel disclosure arrows appear only on hover or interaction;
their hit areas remain available when hidden. The sidebars extend to the bottom of the
window. A single Kill/Enable toggle appears above the panels, or in the pool-view
toolbar when the operator sidebar is hidden. Kill remains available during pending
requests and autonomy execution. The Panels menu in the pool-view toolbar changes
individual panel visibility. YAML controls initial panel order, width, visibility and
collapse.

The Motion panel toggles **Robot controls** for its provider. Hiding handles does not stop an
active position hold. **Pool Viewer** contains observer lighting controls and toggles
**Water**, **Pool walls**, and **Surface reflections**. Surface reflections default off.
Every Pool Viewer control, including shadows,
lighting mode, exposure, brightness and ambient light, affects only the observer view;
FFC/DFC images, published sensor data and physics keep their configured
environment. The pool floor remains visible. Scene settings still changes the shared
simulated environment and contains lighting (including the calibration board), water
optics and depth. Sync sim and Reset sim live in Simulation settings.
Scene lighting continues to affect sensor cameras;
Pool Viewer lighting applies separate rendering overrides and never mutates the sensor settings.
The Sterile lighting preset is a darker ambient-only observer mode with no direct light,
shadows, caustics or glare. It leaves sensor lighting unchanged.
Reset lighting restores the viewer lighting defaults (scene lighting, shadows enabled,
1x exposure/brightness/ambient) while preserving water, walls and surface-reflection visibility.

In Orbit, left-drag rotates and right/middle/Shift-left-drag pans in the camera plane.
Pan or **F** turns Follow off while preserving the selected orbit preset. Selecting
another orbit target enables Follow and tracks that target. Course keeps Follow
disabled. Scroll zooms. A shaded, depth-tested 3D focus disk appears only during
rotation, panning, or zooming while Follow is off. It stays hidden when Follow is on,
and disappears on release, with a short visibility interval after wheel input.

**F** focuses anywhere under the cursor, including objects, visible frames or markers,
and the pool environment. For empty background, it uses a camera-facing plane through
the current orbit target. F retains the camera position while changing its orbit target.

The main viewport fits the available window area without scrolling. Scrolling stays
inside each sidebar and never expands or scrolls the outer layout.

## Configuration and providers

See `c_simulator/robots/talos/config/viewer.yaml` for a complete UWRT composition
and `camera_faker/config/standard_panels.example.yaml` for standard ROS plumbing.
Relative endpoints resolve inside the selected robot namespace; absolute endpoints
remain absolute. `{namespace}` and `{fixed_frame}` substitutions are supported.
The fixed frame comes from the viewer's `fixed_frame` parameter, default `map`.

Top-level keys: `schema_version: 1`, `sidebar_width_fraction` (0–1, default 0.29),
or an explicit `sidebar_width` (300–600 px),
`sidebar_visible` (default true), `providers` (mapping), `panels`, `tools`, `overlays`, and `ownership` (lists).
Provider entries require `type` and `options`. View entries require `id`, `type`,
`provider`; optional `title`, `visible`, `open`, and `options` configure presentation.
Tool entries also accept `slot: settings` or `slot: overlays` (default).
Ownership links use `{motion: provider_id, autonomy: provider_id}`.

| Registered type | Required options | Optional options |
| --- | --- | --- |
| Provider `uwrt.motion` | `base_frame`, `command_frame`, `linear_topic`, `angular_topic`, `kill_topic`, `kill_state_topic`, `mode_service`, `kill_switch_id` (1–255) | `setpoint_frame`, `sender_prefix`, motion timing options below |
| Provider `ros.pose` | `base_frame`, `command_frame`, `pose_topic`, `enable_service`, `enabled_topic` | `setpoint_frame`, motion timing options below |
| Provider `uwrt.autonomy` | `action`, `list_service`, `stack_topic` | `request_timeout` (3 s), `stack_timeout` (5 s) |
| Panel `motion` | none | `dive_z`; legacy `dive_max_depth_z` accepted but no longer limits Dive |
| Panel `autonomy` | none | none |
| Overlay `pose_gizmo` | none | `size_metres` (0.3, up to 10), `hit_pixels` (20, up to 40) |

Motion timing options are `pose_timeout` (1 s), `ui_timeout` (0.75 s),
`request_timeout` (3 s), and `heartbeat_period` (0.05 s). Values must be positive,
finite, and at most 60 seconds (heartbeat at most 1 second). Pose freshness measures
TF timestamp advancement with steady time; paused simulation therefore revokes
enable. Lost UI liveness, stale poses and timed-out mode requests revoke enable.
Late replies cannot restore a canceled request. UWRT adapters also detect another
sender using the configured kill-switch ID.

`ros.pose` publishes `geometry_msgs/PoseStamped`, calls `std_srvs/SetBool` to
request enable/disable, and observes `std_msgs/Bool` enabled feedback. Its robot
backend must implement those semantics. It supplies no universal robot heartbeat
protocol. UWRT publishes its kill heartbeat on a shared ROS worker independently
of rendering. Process-death enforcement belongs to firmware; the existing
simulator does not enforce heartbeat expiry after the viewer is forcibly killed.

## Extending

1. Implement a `Panel` or `Overlay` against an interface in
   `camera_faker/include/pool_viewer/panels/capabilities.hpp`. Keep widget drafts and
   selection local; do not include ROS messages in presentation code.
2. Register its type, capability kind, strict option validator, and factory in the
   panel registry. Add its translation unit to `pool_panel_core`. Add a YAML
   instance; the main drawing loop does not change.
3. For a new protocol, implement a capability provider and register its factory.
   ROS providers share `RosRuntime`; protocol message types stay in their adapter.
   Add it to `pool_panel_ros` and its registration to `RosProviders`. A non-ROS
   provider can register directly at the application boundary.
4. For a new capability, add a typed contract and `Kind`; implement its views and
   providers independently. Avoid stringly typed message dictionaries or robot-name
   branches. Test the contract and protocol boundary using fake providers/servers.

Factories are compiled extensions, not dynamically loaded libraries. The core
owns instances; one composition may bind several views to a shared provider.
Validation rejects unknown keys/types, invalid values, duplicate IDs, missing
bindings and capability mismatches before its ROS executor starts. The application
stops the executor before destroying views/providers; provider shutdown performs
best-effort kill/cancel for sessions it has used. Do not persist enabled state or
pending operations. Existing scene widgets and legacy renderer ROS subscriptions
are outside this first extraction.

## Verification

`pool_panel_composition` exercises validation, empty/preview compositions and
ownership. `pool_panel_ros` uses an isolated ROS domain and mock services/actions
to exercise both protocols, frame/RPY conversion, feedforward, kill, watchdogs,
late/rejected requests, action results/cancellation and stack retention.

```bash
ctest --test-dir build/camera_faker -R '^pool_' --output-on-failure
DISPLAY=:0 ROS_DOMAIN_ID=184 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 src/riptide_simulator/camera_faker/test/panel_ui_smoke.py
```

The display-backed test covers direct XYZ/RPY dragging (including bare rings and
a hidden sidebar), draft/Current separation, Dive, mode changes, tree start/stop,
Kill canceling the linked tree, and the compact Kill action. It also covers
divider resizing, observer visibility, gizmo visibility and size during zoom,
Follow visibility, cursor focus with orbit selection retained, and mirrored
snap-to-close resizing. Camera/layout and control/autonomy checks run in separate
viewer sessions. Use `PANEL_LAYOUT_ONLY=1` or `PANEL_CONTROLS_ONLY=1` to run one phase.
`pool_gizmo_input` verifies body-relative XYZ movement and identical drag results
while a Follow camera moves. It also checks two full turns of each rotation ring in
both directions, tilted views, center crossing, and Escape restoration.
`pool_viewer_input` checks camera-plane panning, depth unprojection and TF/marker hit testing. The UI test's mocked camera TF uses the robot profile mount.

The display-backed test requires Python Xlib, OpenCV, an idle X11 display, and the
built ROS workspace sourced. It drives actual widgets against mocked robot state;
screenshots are saved under `/tmp/riptide-panel-ui`. No hardware is launched.

Delivery checks: 12 native tests passed (CUDA test skipped on this CPU build),
7 profile tests passed, and the display-backed workflows passed. The example AUV
preview starts without operator extensions. A full Talos simulation with RViz
disabled confirmed position commands reaching the controller, tracking after a
gizmo drag (5.6 cm to 0.4 cm in 8 seconds), and Kill disabling the controller and
asserting simulated kill feedback. These are simulation checks, not hardware
qualification.

### Host integration for document-driven panels

`Context.documents` supplies named, immutable YAML documents; the viewer publishes the
resolved task configuration as `task`. A panel/provider's optional `profile` selects
that document. The core has no task schema or ROS message dependency. `Context.focus`
is an optional navigation callback used by configured Inspect actions. Panels can
implement `drawWindows()` for independent detail windows; the composition renders them
even when the sidebar or panel is collapsed. `initialWindows` selects panel instance
IDs to open initially. ROS adapters remain registered only at the application boundary.

`pool_operator_panels` exercises mapping feedback, cancellation before acceptance,
rejection, service failure/timeout, observed-versus-requested target state, actuator
arming and stale-status restrictions, run command serialization, and disconnected/preview
panel rendering with mock servers in an isolated ROS domain.

### Simulator toolbar tools and RViz

`tools_config` defaults to `camera_faker/config/viewer_tools.yaml`, independently of the
robot's `panels_config`. Compositions can declare `tools` alongside `panels`; tools use
the same capability factories and validation but render in the pool toolbar without a
sidebar header. All robot profiles get the default Run tracking and Simulation settings
tools. An empty tools composition can disable them. Tool instances optionally declare
`slot: settings` to sit beside Scene settings; the default `slot: overlays` places them
alongside TF and Detections. Slots control layout only and do not change provider bindings.

Simulation settings reads and writes the configurable node/parameter named by
`ros.simulation_rate` (default: relative `physics_simulator`, `real_time_factor`).
The tool offers numeric Apply, a Pause/Resume toggle, and 1x, and reports rejections/timeouts.
Pause retains the last running speed in the grayed-out input; Resume restores it. The
adapter remembers positive speeds observed from external parameter changes too. Physics
still receives zero while paused, and wall-time polling allows Resume while /clock stops.
The default editing range is greater than zero up to 10x, configurable with `max_rate`.
Opening the menu never changes speed.

Sync sim and Reset sim use optional `sync_service` and `reset_service` Trigger endpoints
(defaults: `sync_sim_to_estimate` and `reset_sim_to_start`). Sync aligns the plant with the
estimate while keeping velocity; Reset returns to the start pose at rest and re-seeds the
estimator. The tool reports service results/timeouts and prevents duplicate pending
requests. These operations remain available while paused.

When `with_rviz:=true`, the viewer does not instantiate its robot-control composition:
Motion, Mapping, Actuators, Autonomy and pose overlays stay unloaded, including their
providers. Run tracking and Simulation settings remain available. Standalone launches
load robot panels by default; `operator_panels:=false` disables them explicitly. The
RViz choice always takes precedence.

Optional motion `setpoint_frame` publishes the commanded pose in TF after a command is
received, including during autonomy and with robot handles hidden. Talos uses
`ghost/base_link`, matching the RViz setpoint frame. The name supports `{namespace}`
substitution; omit it when another node already publishes that frame.
