# Talos launcher and shared projectile

These assets are extracted from the workspace's original `Talos3.dae`; that file
is unchanged. The simulator uses the Talos3 body with its baked-in torpedo/marker
assembly removed, plus this separate launcher and four separately rendered payloads.

- `launcher.glb`: CAD-frame launcher assembly, with the original loaded round
  excluded; 82,669 triangles, about 2.3 MB. Covers, springs and supports remain.
- `projectile.glb`: the CAD `Sinky Torpedo Model (hole)` finned projectile, 5,794
  triangles, about 126 KB. Both torpedoes and markers use this same exterior.
- `projectile.obj` / `material.mtl`: the same shape for ROS mesh markers in RViz.
- `provenance.json`: source assembly bounds, projectile dimensions/volume and
  extraction counts. No report or private competition application is embedded.

The projectile is centered at its bounding-box center and normalized to unit
XYZ extents, with its nose pointing along +X. Runtime dimensions restore its
83 × 26 × 26 mm physical size. Its original watertight mesh volume is approximately
12 cm³. The mounting bore remains; the 2026 torpedo's additional internal buoyancy
cavity is not visible from outside and is represented through neutral mass in
the task configuration. Markers use a heavier full-print mass estimate.

Payload centers are obtained by composing the robot TF reference poses with
`torpedo.slot_offsets` / `dropper.slot_offsets` in
`c_simulator/config/talos_tasks.yaml`. The offsets restore the original CAD
seats in the housing. Aiming TF origins and projectile centers are distinct. See `c_simulator/docs/TASKS.md` for frame
conventions and simulation assumptions.

Regenerate from the release workspace (offline dependencies: NumPy, lxml, Open3D,
trimesh; none are needed by the renderer):

```bash
python3 src/riptide_simulator/camera_faker/scripts/prepare_payload_mesh.py \
  src/riptide_core/riptide_descriptions/meshes/Talos3.dae \
  src/riptide_simulator/camera_faker/models/payloads
```

The extractor retains full projectile geometry and simplifies the surrounding
mechanism to reduce render cost. Internal cam/servo rotation is not animated;
payload release, motion, remaining ammunition and reload are simulated.
