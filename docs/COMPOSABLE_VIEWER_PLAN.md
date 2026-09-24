# Composable viewer panels

## Intent

Extend the existing robot/world/task configuration philosophy to operator UI and
ROS integration. A panel describes an interaction, a provider supplies a capability,
and configuration connects the two. Changing a robot, namespace, pool, or task pack
must not require editing panel widgets or the rendering loop.

The earlier operator-control prototype has been removed. Existing simulation,
rendering, camera, mechanism, scoring, and profile behavior is preserved.

## Required first delivery

- A narrow left sidebar with independently registered, ordered, collapsible panels.
- A motion panel with Enable/Kill, position/feedforward selection, Current,
  Command, and Dive in place. Current copies telemetry into editable fields;
  editing fields does not send motion. Command applies the draft explicitly.
- Six-axis actual and last-commanded readouts, alongside the editable target.
  Position is in metres and RPY in degrees in a clearly identified frame.
- Larger translation axes and RPY rings visible together. Screen-space sizing,
  generous hit regions, hover/drag highlights, and no camera orbit during a drag.
- An autonomy panel with discovery, filtering/selection, start/stop, execution
  status, and a readable execution stack. Full tree identifiers reach the server;
  friendly names are only presentation.
- Working Talos bindings, an empty composition for robots without these
  capabilities, and a standard-message motion provider for other robots.

## Boundaries and ownership

1. **Viewer core and renderer** retain scene, camera and input ownership. They
   instantiate a composition and pass viewport information to its overlays. They
   do not know robot names, command topics, kill IDs, or autonomy action types.
2. **Capability contracts** contain plain C++ state and requests: pose, freshness,
   command mode, enable state, mission list/activity/stack, and operation results.
   They contain no ROS or ImGui types. Report requested and observed state
   separately. Draft target, last sent command, and actual pose are distinct.
3. **Panels and overlays** depend only on these contracts. Each panel owns its
   editing/selection state. The pose gizmo is an overlay extension using the same
   motion capability as the motion panel. Panels never publish ROS messages.
4. **ROS providers** own node/executor lifetime, QoS, TF conversion, asynchronous
   services/actions, timeouts and protocol semantics. The ROS runtime is shared;
   adding a widget must not add another spin loop. Rendering cannot starve
   heartbeat scheduling. UI liveness and telemetry freshness use steady time.
5. **Protocol adapters** are explicit factory registrations. UWRT command/kill and
   ExecuteTree types live only in UWRT adapters; standard PoseStamped/SetBool
   plumbing lives in a separate adapter. The existing renderer still has legacy
   UWRT dependencies; this change does not claim to remove those unrelated paths.
6. **Composition** validates a versioned YAML document, builds providers, resolves
   panel/overlay bindings, and owns shutdown. Factories are compiled extensions
   initially. A runtime binary-plugin ABI is unnecessary for this delivery.

Dependencies flow from application -> composition -> abstract contracts; concrete
panel and provider factories register at the application boundary. Avoid a global
service locator, per-robot conditionals, generic untyped message dictionaries,
and a monolithic provider that switches on robot names.

## Configuration

A robot profile can supply `viewer.panels_config`. An explicit launch override
selects another composition. No configuration means no operator publishers or
panels. Preview mode creates UI previews without command-capable ROS providers.
Pool and task selection cannot implicitly enable robot control.

The composition declares `schema_version`, sidebar width, named providers,
ordered panel instances, and viewport overlays. Each instance has a stable ID,
registered type, capability binding, and options. Topic/service/action names,
namespace, fixed/base/command frames, freshness and heartbeat periods, kill-switch
identity and dive target are configuration, not widget constants. Namespace and
fixed-frame substitutions are explicit context values. Relative ROS names resolve
under the configured instance namespace; absolute names remain absolute.

Unknown types/keys, duplicate IDs, invalid ranges, missing references and capability
mismatches fail with the offending configuration path. Composition loading must
finish validation before providers are started. Each protocol validates its own
options. Config changes take effect on restart; live layout actions only change
presentation and never enable a capability or change an endpoint.

Start with configuration-defined order/visibility/width plus a Panels menu for
session visibility and collapse. Do not persist transient enabled state, pending
requests, edited targets, selected missions or control ownership. Durable layout
editing can follow without changing capability contracts.

## Control semantics

- Startup is passive. Enable is explicit; UI must not imply that a request clears
  physical kill conditions. Kill remains accessible regardless of panel collapse.
- UWRT position/feedforward reproduce the existing ControllerCommand modes.
  Feedforward here means the controller's mode, not a new raw-wrench publisher.
  Pose conversion uses the configured command frame, including orientation.
- Current fills the draft only. Command sends it after any required mode-service
  acknowledgement. Dive copies actual X/Y/yaw, uses configured Z and level R/P,
  and requests position mode. It remains available at any current depth.
- Switching to position initializes from actual pose; switching to feedforward
  explicitly changes mode. Unsupported modes are unavailable for other providers.
- A live gizmo sends position targets while dragged, preserves unselected axes,
  and shares the same last-sent state as the panel. It is unavailable during
  feedforward, stale telemetry or autonomy ownership. Numeric drafts are never
  overwritten while the user is editing them.
- Mission start immediately suspends manual commands; action acceptance, rejection,
  cancellation and completion update ownership without automatically resuming
  manual control. Observe externally started actions as well as local goals.
- Stale telemetry, expired mode requests and lost UI liveness revoke enable.
  Late replies cannot restore a canceled command. Keep firmware timeout enforcement
  separate from UI watchdog claims; the existing simulator does not enforce
  heartbeat expiry after process death.
- Autonomy refresh/start/stop never blocks drawing. Do not forge tree-stack messages
  to clear the UI. Preserve the last stack for inspection and mark it as inactive
  or stale; action status/result is the authority for execution state.

## Implementation sequence

1. Remove prototype and verify the simulator source baseline. Write this plan.
2. Add neutral capability contracts, strict composition parsing and factory
   registration, plus panel/overlay lifecycle ownership and configurable layout.
3. Implement shared ROS runtime, UWRT motion/mission providers, and a standard ROS
   motion provider. Configure Talos explicitly in its robot-owned viewer file.
4. Implement motion and autonomy panels against contracts; integrate a sidebar
   and a larger pose gizmo through composition hooks. Simplify the viewport toolbar
   so narrower layouts do not overflow. Preserve existing scene controls.
5. Add focused contract/configuration tests and isolated ROS tests for commands,
   TF conversion, modes, ownership, timeouts and mission lifecycle. Exercise actual
   UI input and inspect screenshots at normal and narrower window sizes.
6. Validate a full simulated control path and a non-Talos configuration. Document
   how to add a panel/provider and where robot-specific bindings belong.

## Acceptance and extension policy

All requested workflows must be usable without RViz. Adding a panel means a new
panel implementation, one registration, and a YAML instance; it does not modify
the central drawing loop. Supporting another protocol means implementing the
appropriate capability and registering a provider; existing panels are reused.
Changing endpoints, frames or robot namespace is configuration-only. Disabling
all panels leaves the original viewer behavior and no extra operator ROS traffic.

Keep tests at protocol and user-workflow boundaries. Verify empty composition,
invalid bindings, alternate namespaces, preview passivity, actual/sent/draft
separation, combined RPY, feedforward, dive, explicit command, kill, rejected/delayed
service responses, autonomy cancellation/result/error, and disconnect recovery.
Existing camera/TF/profile regressions remain applicable. Physical hardware
qualification, migrating every old scene widget, live binary loading and a general
ROS message editor are separate work, not implied by this implementation.

## Delivery refinements

The sidebar snaps closed when its divider is dragged below the collapse threshold
(initial state: `sidebar_visible`), with
a single Kill/Enable toggle contributed by motion panels through the generic
toolbar hook when hidden. Panel buttons match the viewport toolbar height; motion
values align right and vertically center beside their editors.
Selecting Position activates a hold at the actual pose; subsequent drags command
continuously. The gizmo uses body-relative RGB arrows and solid, continuous rotation rings. Dragging preserves Follow. Untouched initial targets follow telemetry until the first edit/command.
The screenshot test reads the configured camera mount so its FFC is not placed
inside the robot. Both sidebars resize and snap closed. The camera sidebar defaults to its original
26.5% window width (305–405 px), with full-width previews. Panels fill
the available window height. Resize handles, scrollbars and disclosure arrows
appear on hover. The Panels menu lives in the pool-view toolbar.
Observer-only visibility options control water, pool walls, above-water surface
reflections (off by default), and individual overlays. View lighting/exposure only
affect the observer render, independently of FFC/DFC; calibration-board
visibility lives in Scene settings. The motion table includes wrapped pose error
and plain text headers; the tree dropdown fits its longest filename.
Pose handles have a configured world size. Orbit panning follows the cursor, with
a 3D focus disk only during camera rotation, pan, or zoom with Follow off, and cursor focus on F,
including background.
Pan/F leaves Follow and preserves the orbit selection; choosing another orbit target enables Follow. Scrolling is confined to the sidebar children. A collapsed
sidebar retains a pull-to-open grab bar and preserves an in-progress resize gesture.

The implementation and validation are complete for this delivery. See
[VIEWER_PANELS.md](VIEWER_PANELS.md) for the configuration contract, extension
procedure, test commands, and observed simulation results.
