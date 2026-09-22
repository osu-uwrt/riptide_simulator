The claw uses the supplied `Talos_CAD/Talos.SLDASM` and its referenced native
SolidWorks parts. `prepare_claw_mesh.py` reads the saved component tree,
recursively composes its transforms, and extracts DisplayLists triangle strips.
Source CAD is read-only. No extra leveling rotation, sole, extrusion, scaling,
or reshaping is applied to the grippers. Their saved assembly orientation is
preserved, including any small mounting angles; opening does not rotate them.

The fixed assembly contains the robot bracket, perforated aluminum channel,
pins, rack-and-pinion housing and pinion. The channel retains its saved placement
in the housing. Each moving group contains a gripper carrier, rack and end cap;
the original left/right TPU pads translate with those groups along opposing Y
axes. The inner perforated faces are the gripping surfaces. The real silicone
coating is approximated through configurable friction; TPU deformation is not
modeled. The pinion is currently a stationary visual, and the imported servo
STEP body and virtual Part1 are omitted.

`provenance.json` records the source transforms, unchanged pad triangle counts
(4,388 and 4,412), volumes, and the bottom-center reference used by the simulator.
That reference is stored in `c_simulator/config/talos_tasks.yaml` (`claw.pose`),
in the same CAD coordinates as the payloads. Both the contact world and renderer
subtract the vehicle's base-link offset. Only the pads currently have claw
collision geometry; mount and rack visuals do not add collision bodies.

Offline reproduction:

```bash
python3 camera_faker/scripts/prepare_claw_mesh.py ~/Downloads/Talos_CAD \
  camera_faker/models/claw c_simulator/collision_files/tasks
```

Offline dependencies are numpy, trimesh, open3d and
[swformat](https://github.com/KenM76/swformat) (Apache-2.0). These are not runtime
CAD dependencies. The parser follows the saved DisplayLists descriptor and
triangle-strip structure; dense housing visuals are simplified to 8,000 faces
per part, while pads retain all original triangles. Pad contacts use convex
hulls, filling the small rib cavities and holes.

The vehicle plant now includes pad/scenery and held-object/scenery contacts.
Impact impulses act on the vehicle's Fossen mass/inertia, while the ideal rack
drives hold their commanded opening. See c_simulator/docs/TASKS.md for contact
coupling and the remaining hydrodynamic/servo limits.
