# Application review and delivery plan

Baseline: Phase 5 merge `2fc57e3`. Reviewed against the project vision and Phase 1–5 results on 2026-09-25.

## Findings

| Priority | Finding and evidence | Effect on project goal |
|---|---|---|
| P0 | `Executor.start` throws for any discovered `OpMode` that is not a `LinearOpMode`; `Main` always calls `runLinearOpMode`. | The advertised zero-refactor support for both FTC OpMode styles fails for iterative TeleOps. |
| P0 | `SimIMU` always returns zero yaw/rate and `resetYaw()` does nothing, even while `PhysicsWorld` rotates the chassis. | Heading-based autonomous control loops cannot be tested meaningfully. |
| P1 | No Gradle wrapper or root run guide; `TeamCodeCompiler` hardcodes `:` as the classpath separator and does not resolve relative `extraClasspath` entries against the team project. | A new team cannot reliably build/run the desktop simulator across platforms or use local Java libraries. |
| P1 | Phase 5 has a paired worked example and a real exporter attempt, but no successful real FTC-team Onshape export because API credentials were unavailable. | The importer is usable for supplied URDF/STL packages; the project's broader CAD onboarding claim remains unverified. |
| P1 | Phase 3 calibration has synthetic tests but no real robot log or drop-in on-robot logging wrapper. | Sim-to-real accuracy cannot yet be measured against a team robot. |
| P2 | Chassis drive sets velocity directly each tick, moving URDF mechanisms lack independent collision, and motor load inertia remains a placeholder. | External pushes and articulated contacts are less realistic than the high-fidelity vision. |

## Changes in this delivery

### 1. Run both FTC OpMode styles

- Unify the executor entry point and keep LinearOpMode threading/start semantics.
- Schedule iterative `init`, `init_loop`, `start`, `loop`, and `stop` at a bounded rate; honor `requestOpModeStop` and preserve watchdog isolation.
- Add a sample iterative OpMode and an executor test proving lifecycle order and normal termination.
- Validate the sample through the real headless runner; rerun existing LinearOpMode and watchdog scenarios.

### 2. Make IMU heading reflect physics

- Feed chassis heading and angular rate from Bullet into each simulated IMU in the renderer.
- Implement `resetYaw`, acquisition timestamps, and configurable latency using the existing sensor ring buffer. Keep pitch/roll zero for the level 2D chassis model.
- Add focused IMU tests and run a turning OpMode in the real renderer to show nonzero yaw, plus the existing forward/strafe and intake checks.

### 3. Make setup reproducible

- Add a Gradle wrapper and a root README with exact headless and 3D commands, config-file placement, and known limitations.
- Resolve `extraClasspath` relative to the team project and use the platform path separator; add a narrow compiler test.
- Verify from a clean checkout using the wrapper.

## Acceptance criteria

- All Gradle tests pass.
- Iterative and linear sample OpModes complete; the deliberately hung OpMode is still isolated.
- A turning OpMode produces a changing IMU yaw/rate in the 3D renderer, and `resetYaw` zeroes the reported reference.
- Existing Phase 4 forward/strafing and intake capture remain working.
- New users can run the repo with `./gradlew` and follow the root README without installing Gradle separately.
- Changes ship on a reviewable branch through a merged PR.

## Later work requiring separate evidence

Real FTC CAD export and Control Hub XML, real telemetry collection/calibration, full Road Runner/Pedro dependency compatibility, articulated mechanism contacts, and push-responsive drivetrain control need team assets or larger design work. Keep their limitations explicit; do not claim them solved by this delivery.

## Execution evidence (2026-09-25)

- `./gradlew test --offline --no-daemon` passed all module tests, including new iterative lifecycle, IMU latency/reset, and relative extra-classpath compile/runtime tests.
- Headless `IterativeDriveOpMode` completed with `loops=10`; existing `DashboardOpMode` completed with live encoder and dashboard output; `HungOpMode` still triggered the 300 ms watchdog and returned without blocking the runner.
- In the real 3D renderer, `TurnAndResetOpMode` reported `Yaw before reset : 106.919...` degrees and `Yaw after reset : 0.0`; the chassis stayed near the origin while turning.
- Existing 3D `BasicMecanumOpMode` completed its forward and strafe phases at chassis position `(0.712, 0.098, 0.700)` m. `IntakeDemoOpMode` finished with `gamePieceHeld=true`.
- The root README and robot import guide now use wrapper commands and document the actual IMU/CAD limits.
