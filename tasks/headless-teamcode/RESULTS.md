# Physics-backed headless TeamCode results

Implemented October 5, 2026 from `0efab92`. Plan: [PLAN.md](PLAN.md).
User guide: [HEADLESS_PHYSICS.md](../../gui-runner/HEADLESS_PHYSICS.md).

## Delivered

- `runHeadlessPhysics` and `Main ... --physics` compile/discover actual TeamCode and construct
  generic or reviewed imported robot/field scenes without a window. Metric scale, collision
  audits, placement checks, field modes, profile/scene settings, hardware, calibration and
  extra classpaths use the existing simulator contracts.
- The runner owns one fixed motor/battery/native clock. Aggregate drive, native wheel/tire
  contacts, articulated mechanisms, flexible intake/retention and scene sensors use the
  production physics classes. IMU and Pinpoint/OTOS receive native chassis feedback.
- Linear and iterative SDK lifecycles support INIT/PLAY, normal return, bounded duration,
  wall watchdog, user exceptions and cooperative stop. Commands are zeroed after evidence
  capture. An interrupt-ignoring daemon receives no more ticks; the separate CLI JVM exits.
  The in-process API rejects another run while an orphan remains alive.
- Atomic JSON reports include trajectory, initial/final pose and joint/encoder state, native
  diagnostics, sensors/scoring, settings/input hashes, scheduling lag, outcome and cleanup
  status. Trajectory sampling is capped at 10000 records. Startup failures also write reports.

## Bugs and validation findings

The powered suspension workflow exposed a real torque bug in the prior native wheel model.
Implicit back-EMF damping included a tiny carrier's standalone inverse inertia even though
its prismatic suspension locks carrier rotation to the chassis. The motor effort fell to
about 0.0015 N*m despite roughly 0.416 N*m commanded shaft torque, and the robot barely moved.
Native wheels now use rotor inertia for this stabilization; Bullet's constrained motor solve
owns reaction/load transfer. Existing rigid-wheel, ramp/tip-over and suspension tests remain
active; the new compiled TeamCode suspension test requires observed forward travel.

The full gate also reproduced an **existing** transient tire convergence failure. A fresh
baseline exported from `0efab92` passed once and failed on repeat, with 480/960 Hz travel
0.5053/0.4524 m (difference 5.29 cm). This is native contact-load variation, not a new headless
motor clock regression; the fixture has no articulated axes. A tentative map-order change
was investigated and reverted because it did not resolve the evidence.

The check now separates scopes: native contact runs retain finite slip, grip caps,
equal/opposite shaft reactions, zero-material-grip and movement checks at 120/480/960 Hz.
Flat steady-load ray support independently checks the existing **5 cm convergence bound**;
observed travel was 0.5767/0.5804/0.5810 m. No production tire equations were changed to get
this test passing. Variable-contact timestep sensitivity remains a physical/numerical limit;
the previous 5 cm statement must not be treated as a general native-contact guarantee.

An initial blocked-slide assertion expected the optional diagnostic label. Actual evidence
showed slide/wall load and encoder travel below 2 cm, but chassis recoil left about 7 mm/s
joint motion, above the label's 2 mm/s threshold. The test now asserts native contact load,
finite effort and restricted actual joint/encoder travel instead of requiring a heuristic label.

## Verification

- Eleven compiled headless workflow tests cover native closed-loop odometry/distance, IMU
  turning, iterative lifecycle, wall blocking, duration cleanup, user/startup failures,
  process watchdog isolation, cooperative stop, reload/static reset, imported field modes,
  powered rotating wheels/suspension/sensors/scoring and mechanism limit/obstacle feedback.
- Python model preparation/export/migration suite: **50 passed**.
- Documented CLI: `TurnAndResetOpMode --duration 3 --report build/headless-physics/turn.json`
  completed with `renderer=false`, 140 native run ticks, ~1.167 s simulation/~1.173 s wall
  time, maximum lag ~0.929 ms, ~138.93 degrees IMU yaw followed by 0 after reset.
- `./gradlew test :gui-runner:validateSimulator --args='--matrix'`: **235 Java tests**
  passed (45 physics, 18 SDK, 172 GUI/native/workflow; no skips/errors/failures) and
  **174 native scenarios passed**. Matrix evidence:
  `build/simulator-validation/run-9431a8e7-ab33-4671-a600-76be9da40a30/report.md`.
- `git diff --check`: clean.

## Limits

Unmodified SDK/TeamCode timers use wall time; these runs are paced and cannot promise
accelerated deterministic execution. Excessive lag fails explicitly. Compilation/construction
are outside the SDK watchdog, so CI should also set a process/job timeout. No real vendor
pathing jars or new physical recordings were validated. Generic geometry, empirical contact
parameters, geometric cameras and practice scoring retain their documented limits.

The plan's requested field/robot runtime parity uses the same native classes rather than a
large renderer refactor; headless scene construction mirrors its configuration branches.
Future runtime construction changes must update both paths. No new private CAD is committed.

## Delivery

Pending tested-head PR and merge.
