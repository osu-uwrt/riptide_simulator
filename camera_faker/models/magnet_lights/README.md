# Competition magnetic sensor lights

The assets replace `bin_magnet` placeholders in the pool viewer. Coordinates are
metres; +X points out of the lid, with the origin at its outer face center. The
task configuration applies the 45-degree upward tilt without changing the
existing CAD/mapping frames.

These are procedural visual approximations, based on:

- [Polycase ML-22F](https://www.polycase.com/ml-22f) and its
  [dimension drawing](https://www.polycase.com/media/catalog/product/file/44381476212108.pdf):
  clear-cover version, 2.625 × 2.620 inches, 1.672 inches deep, with mounting
  flanges, a silicone seal and four corner screws.
- [RoboNation build guide](https://robonation.gitbook.io/robosub-resources/section-3-autonomy-challenge/3.2-task-descriptions/build-guide-underwater-magnetically-activated-light):
  16 WS2812B LEDs and a centered TLV493D sensor 5–13 mm behind the cover.
- User-supplied `magnet_led_real` photograph and `magnet angle` diagram: purple
  circular board, exposed LED packages, clear cover, side cable glands, and
  the competition unit's 45-degree upward-facing installation.

The housing dimensions follow the manufacturer drawing; the board details,
44 mm ring, cables, and robot stick/head dimensions are visual estimates. The
robot magnet's tip is located by `vehicle_config.magnet.pose` plus
`magnet_lights.robot_tip_offset` (currently 5 cm upward). The rod extends to
11 cm above the mesh origin, preserving its upper attachment after that lift.
No manufacturer CAD or photo texture is embedded. The STEP download was not
available to the build environment; the enclosure is reconstructed geometry.

`housing.glb` contains the base, board, screws, wiring and LED packages.
`cover.glb` contains translucent polycarbonate; `leds.glb` contains 16 separately
modeled emissive lenses. The renderer applies red/green color independently to
each target. `robot_magnet.glb` is a printed stick and metallic disk tip.
All components participate in normal camera projection and depth rendering;
glow is an HDR camera effect, not a billboard or synthetic detection overlay.

Regenerate with numpy and trimesh installed:

```bash
python3 camera_faker/scripts/prepare_magnet_light.py camera_faker/models/magnet_lights
```

Runtime has no CAD or mesh-generation Python dependency.

## Perception check

The unchanged `tensor_detector/weights/rs26_ffc_woollett.pt` detects the red
light as `magnet` in both default target locations at a face-on 0.4 m camera
distance (0.964 / 0.929 confidence in the recorded run, versus the configured
0.65 threshold). Tests use actual rendered native-resolution FFC RGB, normal
pool lighting/water optics, and the detector's default 640-pixel inference.
At 0.25, 0.6 and 1.0 m, detections do not consistently exceed that threshold.
Green samples were not recognized as `magnet`. This is a limited compatibility
check, not evidence of reliable detection throughout an approach or a validation
against underwater photographs. No perception thresholds or weights were changed.

`ros_magnet_smoke.py --weights ...` records repeatable images and a JSON report
so future material/lighting changes can be evaluated against those same weights.
