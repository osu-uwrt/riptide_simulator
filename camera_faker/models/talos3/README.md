# Talos3 visual model

`Talos3_body.glb` is the default simulation body, derived from the same
`Talos3.dae` used by RViz. It excludes the complete `Torpedoes + Markers V8`
assembly (including flattened CAD names and its loaded round). The simulator
renders `models/payloads/launcher.glb` and the four moving payloads separately,
so no fixed launcher or ammunition is duplicated. The body has 1,087,230 triangles,
27 materials and a 41.6 MiB GLB, versus 2,679,494 triangles in the original body
without the launcher. `Talos3_body.json` records every excluded part and the
conversion settings. CAD coordinates are preserved to keep the body aligned
with the separate mechanisms.

The converter retains 40% of each part's triangles, with a minimum of 160 for
small parts. It transfers the original CAD corner normals onto simplified
surfaces and retains separate vertices at sharp normal discontinuities. This
keeps curved housings smooth and flat panel edges crisp. Nearly identical
position/normal pairs are combined, and parts are batched by diffuse material.
The previous body retained only 6.5% per part and recalculated all normals,
producing visibly faceted silhouettes and shading.

`material_repairs.json` replaces 206 placeholder assignments using materials
already present in the CAD: black thruster shrouds/struts and connector bodies,
red propellers, rubber protectors, aluminum rails, and clear polycarbonate.
Repairs are restricted to named component families and generic `color-*`,
`color_*`, or `White-PW-MT11050` assignments. Explicit named finishes are retained;
unrelated white components are not recolored. The conversion report records each
replacement. There are no image textures in this source CAD.

The converter preserves the clear material's alpha (about 0.23). The viewer
draws transparent CAD surfaces after opaque parts and emissive LEDs, so the
port-hull lights can be seen through the original polycarbonate window.

An area-weighted comparison against an unsimplified body export sampled 100,000
surface points in each direction (seed 42). The 95th-percentile distances were
0.36 and 0.38 mm; sampled maxima were 2.90 and 1.72 mm. These are sampled
measurements, not a guaranteed geometric error bound.

The original 548.2 MiB `riptide_core/riptide_descriptions/meshes/Talos3.dae`
used by RViz is unchanged. These are visual assets; vehicle mass, inertia and
collision envelopes are not inferred from them.

`Talos3.glb` is the older, aggressively simplified full-assembly reference
(224,578 triangles, 39 materials), retained for reference and not used by default.

`Talos3.json` records source/output counts, bounds and named torpedo/dropper
assembly bounds used to inspect mount locations. The dropper opening and second
torpedo outlet are inferred; see `c_simulator/config/talos_tasks.yaml` for the
explicit simulation geometry and provenance.

To regenerate from the release workspace (offline Python dependencies: NumPy,
lxml, Open3D and trimesh):

```bash
python3 src/riptide_simulator/camera_faker/scripts/prepare_talos_mesh.py \
  src/riptide_core/riptide_descriptions/meshes/Talos3.dae \
  src/riptide_simulator/camera_faker/models/talos3/Talos3_body.glb \
  --exclude-launcher --triangle-ratio 0.4 \
  --material-repairs src/riptide_simulator/camera_faker/models/talos3/material_repairs.json
```

Increase `--triangle-ratio` toward `1.0` to retain more geometry; `1.0` skips
decimation. Omit `--exclude-launcher` to export the complete assembly to a
different output path. RViz's source mesh is not modified.

The converter streams individual parts to avoid loading the entire original
assembly into a modeling application. Runtime uses the packaged GLB through
Assimp; Open3D and the other conversion dependencies are not runtime requirements.
