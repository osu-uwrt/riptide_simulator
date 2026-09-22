`table*.obj` are geometry-only exports of the identically named model.dae assets
in riptide_meshes. Coordinates and scale are unchanged. Table and baskets use
static triangle contacts, retaining basket interiors and walls. Dynamic props
use convex envelopes centered on their mesh bounds; visuals retain original UVs.
`claw_pad.obj` is the pad geometry described in camera_faker/models/claw.

The table's thin visual surfaces are backed by the solid slab specified in
`config/talos_tasks.yaml` under `table_collision`. Both vehicle and prop solvers
use that same backing; the original triangle mesh still supplies legs and rims.
