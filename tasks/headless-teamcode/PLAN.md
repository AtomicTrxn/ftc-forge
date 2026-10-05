# Physics-backed headless TeamCode (item 4)

Requested October 5, 2026. Baseline: `0efab92`. Implement, verify and ship as with items 2/5.

## Plan

- [x] Construct a headless scene using the runtime's native robot/field classes: generic or
  reviewed imported geometry, collision review, placement checks, field modes, articulated
  mechanisms, tire/native wheels, flexible intake and retention. Preserve metric scale,
  saved calibration, model/scene selection and extra classpaths.
- [x] Give the native runner sole ownership of fixed-step motor/battery advancement.
  Keep the existing ordinary headless executor available; avoid two motor clocks.
  Publish native IMU, Pinpoint/OTOS, configured scene sensors and intake/scoring state.
- [x] Run real compiled/discovered LinearOpMode and iterative OpMode lifecycles, including
  initialization/start, bounded duration, wall watchdog, exception reporting and stop cleanup.
  Pace against wall time because unmodified TeamCode sleeps/timers use wall time. Disclose
  scheduling variation; bound/report lag rather than implying deterministic accelerated execution.
- [x] Add a CLI/Gradle entry point and bounded JSON evidence with pose trajectory, native
  diagnostics, encoder/device state, timing, outcome and model assumptions. Report failures
  as failures, including startup/configuration problems, without losing useful evidence.
- [x] Verify actual compiled closed-loop autonomous code observes changing pose and sensor
  values, wall blocking and mechanism encoders/stall, both OpMode styles, duration/stop,
  watchdog/exception isolation, repeated runs and no renderer dependency. Cover imported
  rotating wheels/suspension and scene/field settings. Keep prior executor/SDK checks green.
- [x] Run full Java/Python regression gates and native matrix. Document reproduction and
  limitations; ship through a PR and record the tested head/merge in RESULTS.md.

## Decisions

- Opt-in `--physics` / `runHeadlessPhysics` preserves legacy `Main` invocations.
- A fixed native step is shared across motor integration, chassis/mechanism physics and
  sensor publication. Physics classes are the same ones used by the renderer and native probes.
- An optional unpowered settling period occurs before TeamCode starts. Native time and the
  initial reported odometry frame are recorded explicitly.
- User code stays on its SDK lifecycle thread. Daemon code that ignores stop/interrupt cannot
  be force-killed safely; the CLI's separate JVM is the isolation boundary. It receives no
  further physics or motor ticks after stopping. No accelerated/time-virtualized TeamCode claim.
- No new private CAD or physical calibration is inferred. Existing scoring/sensor limitations
  remain applicable to this execution mode.

## Progress

- Scene, fixed clock, SDK lifecycle, CLI/evidence and compiled workflow checks completed.
- Powered suspension regression exposed excessive damping from a carrier's free-body inertia;
  corrected native wheel damping while retaining Bullet's constrained reaction solve.
- Isolated baseline test execution reproduced an existing variable-contact tire convergence
  failure. Full native force/material/reaction checks are retained; flat steady-load convergence
  has its own 5 cm check. No production tire equations or tolerances changed for this issue.
- Python: 50 tests passed. Documented CLI turn/reset ran without a renderer and exported evidence.
- Full gate: 235 Java tests and 174 native scenarios passed; no skipped/error/failing cases.
- Shipped in PR #26: tested head `599567d`, squash merge `8126b56`.
