# Advanced physics and field behavior results

Implementation completed October 4, 2026 (America/New_York), from baseline `8ee8d9d`.
Plan: [PLAN.md](PLAN.md). User guide: [ADVANCED_PHYSICS.md](../../gui-runner/ADVANCED_PHYSICS.md).

## Physics accuracy improvements (item 2)

- Independent lateral stiffness/grip and opt-in relaxation response preserve the previous
  steady tire defaults and longitudinal measurement evidence. Relaxation acts on bounded
  contact utilization; actual normal load and native material grip remain instantaneous.
  Force reversal/impulse bounds prevent adding slip energy. Loss of native support or zero
  material grip resets history. A supported contact with zero solved load emits zero force.
- Four physically rotating wheel bodies share two native motor shafts with gear constraints.
  Their mass is included once; reflected motor inertia is shared once; encoders report native
  shaft motion. Aggregate/brush/native traction ownership conflicts are rejected.
- Passive native suspension, free chassis pitch/roll, ramp settling, tip-over and full IMU
  orientation/body rates are supported. Spring effort diagnostics explicitly label estimates.
  Native wheel contact friction is distinct from the empirical brush tire model.

Native evidence from generated URDF, actual Bullet integration:

| Check | Observed / assertion |
|---|---|
| Shared powered rotating wheels | About 0.342 m travel in 1 s, shafts rotate >1 rad; grouped shaft positions agree within 0.02 rad. |
| Unsupported powered wheels | Chassis speed <0.02 m/s after 0.5 s; no artificial airborne traction. |
| Ramp | 0.15 rad incline produces about −0.150 rad IMU pitch. |
| Tip-over | External torque lowers chassis up-vector dot vertical below 0.7. |
| Suspension | Each spring compresses about 11.5 mm, within configured travel; settled body speed <0.1 m/s. |
| Transient tire response | Exact exponential buildup across 30/120/480 Hz in pure checks; native 120/480/960 Hz travel differs <5 cm between adjacent resolutions when yaw is fixed for force-response isolation. |
| Native tire forces | Always within native grip cap, with equal/opposite contact shaft reaction; zero-friction floor produces zero traction. |

A prototype that repeatedly fed solved force caps into force history amplified timestep
sensitivity. The shipped model relaxes normalized contact utilization, keeping solved normal
load outside the filter. Tests also exposed unrestricted yaw/contact trajectory sensitivity:
free-yaw native runs could vary by >0.1 m at 480 vs 960 Hz. The convergence check explicitly
isolates yaw; free-turning contact checks remain in the existing native matrix. This is a
reported numerical limitation, not a physical calibration claim. Runtime brush/native-wheel
modes use 480 Hz; other supported models retain their existing step settings.

## Sensors and field behavior improvements (item 5)

- Native geometry drives configured distance/color/touch SDK devices. Poses may follow
  chassis or moving mechanism links; sampling, latency, seeded noise and sensitivity are
  portable parameters. Unconfigured devices retain disclosed placeholders.
- Geometric AprilTags provide metadata, timestamp, pose/units, front-face/frustum visibility,
  native occlusion and VisionPortal/processor lifecycle. Cameras bind from real robot XML.
  No image frames or image-processing pipeline are implied.
- Versioned practice objectives support occupancy/removal, entry deduplication, timed
  snapshots, native match clock, pause/reset and JSON evidence with rule bounds and poses.
  Field-only mode suppresses piece scoring. Robot objectives currently use chassis bounds.
- Optional reference poses and inventory auditing disclose field setup gaps. The supplied
  BIOBUZZ CAD matches guide inventory (2 hives, 4 flowers, 40 pollen, 8 red / 8 blue nectar).
  Its practice placements, hive spring/damper calibration and competition setup remain
  unverified. The report never turns count matches into placement certification.

A touch test initially triggered on a nearby floor contact. The fix adds a configurable
press-face normal cosine; the mounted switch responds to pressure opposing its +X face,
while ignoring the same nearby floor contact. The regression checks both states.

## Verification

- `./gradlew test :gui-runner:validateSimulator --args='--matrix'`: **223 tests**
  (45 physics, 18 SDK, 160 GUI/native/workflow) plus **174 matrix scenarios** passed.
- `python3 -m unittest discover -s tools -p 'test_*.py'`: **50 tests passed**, including
  export/import, changed CAD review, sensor mount rename and rotating-wheel settings.
- After final evidence-export/display edits, targeted scoring, sensor and diagnostics tests
  passed again, followed by the actual jME renderer capture.
- `git diff --check` clean.

Local artifacts (ignored generated output; no private CAD committed):

- `build/simulator-validation/run-7414100e-c6f3-4aac-a2da-f1275b3dbe27/report.md`
- `build/field-behavior/biobuzz-layout-audit.json`
- `build/advanced-fixture/renderer-final.png`

The captured application shows four supported native wheel bodies, suspension position and
estimated effort, a 1.609 m distance sample, red RGBA color, released touch switch, one
geometric tag, and paused practice scoring controls. Renderer frame capture was inspected;
it does not establish automated desktop mouse/keyboard coverage. The public generated
fixture command reproduces the setup without user CAD.

## Remaining evidence and scope

- Measure motor/drivetrain inertia, tire/lateral/relaxation properties and suspension parameters
  on hardware. Detailed Mecanum rollers, flexible tire/tread geometry, bearing losses and ground
  compliance are still approximations.
- Color is a scene/material sample; distance uses one collision ray; touch is local contact;
  tag detections are geometry, with robot self-occlusion and image effects omitted.
- IMU mounting parameters do not remap the simulator URDF frame (x forward/y left/z up).
- A complete BIOBUZZ evaluator would need stack ownership, hive damper contact/state changes,
  official placement/tag/tape/preload reference data and penalty/ranking decisions. Generic
  region rules provide practice objectives and are not that evaluator. The official guide
  additionally calls for physical hive calibration; CAD cannot supply it.

Official layout source:
[Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/field/eventfieldguide).

## Delivery

[PR #25](https://github.com/AtomicTrxn/ftc-forge/pull/25) merged to `main` as
`06e69f1` at 2026-10-05 00:50:50 UTC (October 4, 20:50:50 America/New_York).
Tested feature head: `f45acfa10faa0672b97e7e4611d8fc69f564220d`.
