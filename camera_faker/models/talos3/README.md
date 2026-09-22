# Talos3 visual model

`Talos3.glb` is generated from the workspace's
`riptide_core/riptide_descriptions/meshes/Talos3.dae`. The original 549 MB CAD file
is unchanged. The GLB preserves CAD coordinates, assembly transforms and diffuse
colors while reducing 3,011,610 source triangles to 224,578 and batching them into
39 materials. Small fasteners and curved surfaces lose detail. It is a visual
asset; vehicle mass, inertia and collision envelopes are not inferred from it.

`Talos3.json` records source/output counts, bounds and named torpedo/dropper
assembly bounds used to inspect mount locations. The dropper opening and second
torpedo outlet are inferred; see `c_simulator/config/talos_tasks.yaml` for the
explicit simulation geometry and provenance.

To regenerate from the release workspace (offline Python dependencies: NumPy,
lxml, Open3D and trimesh):

```bash
python3 src/riptide_simulator/camera_faker/scripts/prepare_talos_mesh.py \
  src/riptide_core/riptide_descriptions/meshes/Talos3.dae \
  src/riptide_simulator/camera_faker/models/talos3/Talos3.glb
```

The converter streams individual parts to avoid loading the entire original
assembly into a modeling application. Runtime uses the packaged GLB through
Assimp; Open3D and the other conversion dependencies are not runtime requirements.
