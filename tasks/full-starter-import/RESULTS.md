# Full starter import evidence

## Source and local preparation

The supplied ZIP contains one URDF, 347 STL meshes, 1,349 links and 1,348 joints. It expands to 514,958,332 bytes, exceeding the previous 400 MB extraction limit. There are no collision elements or transmissions; only one joint is revolute and the others are fixed. The package is private and remains under Downloads; copies, prepared assets, profiles and TeamCode are ignored under `.local/full-rev-starter/`.

The import now uses a finite 1 GiB expanded budget with streamed extraction and a selectable backend budget capped at 2 GiB. Visual pose overrides are saved settings, preserved through bundle export/import and reviewed through migration choices. Existing profiles remain compatible.

The local preparation uses scale 1 in meters and rotates source −Y into robot +X. Six 90 mm wheel candidates have separate continuous joints, editable standard-template drive bindings and cylinder contact envelopes. The original fixed assembly mate tree placed hundreds of stationary descendants below wheel parts; stationary parts were reparented at their exact source world poses while retaining the 56-part exported arm subtree. This is a model-specific preparation, not an automatic importer capability.

Four zero-mass grip wheels contain assembly-relative visual poses inside already positioned links. An initial attempt to remove their link transforms incorrectly moved attached descendants; it was rejected. Saved per-visual corrections instead preserve link frames and descendant placement. Their missing mass is provisionally estimated at 0.0061938655 kg each using a solid rubber cylinder, not measured. Runtime robot mass is 4.4283955059 kg; motor presets and contact values are provisional.

The scene pins robot revision `0789c66b-0912-4558-8c2f-5e10bf844960` and the existing BIOBUZZ revision. A 25 mm initial root lift clears the conservative visual floor check; native physics settles the wheel hubs to approximately 45 mm above the floor. Original robot/field scale is unchanged.

## Native and rendering evidence

- Construction preview: two dynamic robot bodies and one constrained mechanism; saved exact-settings validation receipt and collision PNG.
- Headless `FullStarterCheck`: completed; forward displacement **0.194299 m**, all six configured wheels supported, finite poses, both drive encoders **373 ticks**, no residual motor command.
- Servo command: observed travel only **0.000882 rad**. Its source-limited arm remains near the lower limit. Usable servo motion is **not validated**; load, controller effort and source joint direction/limits require investigation. Raising effort or changing source limits without validation was not used to conceal this failure.
- Interactive `FullStarterTeleOp`: BIOBUZZ at **3.590237 × 3.590236 m**, **56 pieces**, **148 native bodies**, approximately **115 FPS** after startup.
- Local evidence: `.local/full-rev-starter/evidence/`; renderer log: `build/full-starter-renderer.log`. Standard arrow-key drive is active; Space/E issue servo commands, whose useful travel remains unresolved.

## Remaining setup

Intake, launcher and other mechanisms are fixed in this CAD export. Additional mechanical grouping, joints, hardware XML/names and motor/servo specifications are needed to actuate them. Prior base-robot intake/torus settings were not copied onto unrelated full-robot parts. Operating weight, friction and servo behavior still need measured calibration. Visual/native review does not certify real-world accuracy or human motion expectations.

## Regression gates

Python preparation suite: 53 tests passed. Java regression suite: 235 tests passed (18 SDK, 45 physics, 172 runner). Tests cover expanded-budget rejection/preserved streamed bytes, visual correction without moving descendants, portable round trips, invalid values and selectable changed-CAD migration.
