# Phase 1 Results: Core Executor & HAL

**Status:** Complete — all Definition of Done items validated with real, captured console output (below), not just described.
**Task spec:** [phase-1-core-executor-hal.md](phase-1-core-executor-hal.md)

## Summary

Built and validated, end-to-end, on this machine: a mocked FTC SDK (`core-sdk-mock`), a headless executor that compiles and runs real team `.java` source against it (`gui-runner`), a from-scratch FTCDashboard-compatible shim, a watchdog that isolates a genuinely hung OpMode without wedging the process, and three hand-authored preset robot configs each carrying a per-motor SKU/ratio field. All six Definition of Done items pass with real evidence, not assertion. The riskiest assumption in the whole project — can real, unmodified team code actually compile and run against a mock SDK — is now proven, not theoretical.

The build environment available for this pass was JDK 17 (Temurin) with no Gradle/Maven installed and network access to Maven Central confirmed working but not exercised for a real build tool. This forced two honest, flagged scope reductions from the research: no real Gradle-coordinate dependency resolution (R3), and a hand-rolled JSON parser instead of a real library. Both are called out in Deviations below, with what's needed to close the gap.

## What Was Built

**`core-sdk-mock/`** (51 source files, 72 compiled classes, compiles clean under `javac`):
- `com.qualcomm.robotcore.hardware.*` — `HardwareDevice`, `DcMotorSimple`, `DcMotor`, `DcMotorEx`, `Servo`, `CRServo`, `PIDFCoefficients`, `IMU`, `DistanceSensor`, `ColorSensor`, `TouchSensor`, `VoltageSensor`, `Gamepad`, `HardwareMap` (type-aware lookup, `getAll`, `getNamesOf`, `DeviceMapping`-style iteration per R3's revised design).
- `com.qualcomm.robotcore.eventloop.opmode.*` — `OpMode`, `LinearOpMode` (real threading contract: `runOpMode()` on its own thread, `waitForStart()`/`opModeIsActive()` backed by executor-signaled state, not busy-guessed), `TeleOp`, `Autonomous`, `Disabled`.
- `com.qualcomm.robotcore.util.*` — `ElapsedTime` (wall-clock per R3's Option A decision), `Range`, `RobotLog`.
- `com.qualcomm.hardware.lynx.LynxModule`, `com.qualcomm.hardware.rev.RevHubOrientationOnRobot` — both added to the stub list during the Opus review pass, now actually implemented.
- `org.firstinspires.ftc.robotcore.external.Telemetry` (+ nested `Item`/`Line`/`Log`), `org.firstinspires.ftc.robotcore.external.navigation.*` (`AngleUnit`, `DistanceUnit`, `CurrentUnit`, `YawPitchRollAngles`, `AngularVelocity`).
- `org.firstinspires.ftc.vision.VisionPortal` (+ `Builder`), `org.firstinspires.ftc.vision.apriltag.{AprilTagProcessor, AprilTagDetection}`, `org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName` — no-op stubs per R1's amendment ("deferred" means no simulated camera, not missing classes).
- `com.acmerobotics.dashboard.{FtcDashboard, config.Config, telemetry.TelemetryPacket, telemetry.MultipleTelemetry}` — the FTCDashboard shim (see Validation below — its `@Config` scanning is real, not decorative).
- `simcore.*` — internal backing implementations not part of the real API surface: `SimDcMotorEx`, `SimServo`, `SimCRServo`, `SimIMU`, `SimVoltageSensor`, `SimDistanceSensor/ColorSensor/TouchSensor`, `RobotConfigXml` (parser), `PresetRobotConfig` (loader), `HardwareMapBuilder`, `ConsoleTelemetry`, `MiniJson`.

**`gui-runner/`**:
- `simrunner.TeamCodeCompiler` — `javax.tools.JavaCompiler` runtime compilation, per R3.
- `simrunner.OpModeDiscovery` — annotation-driven discovery (`@TeleOp`/`@Autonomous`, skips `@Disabled`), defensively loading each candidate with `Class.forName(name, false, loader)` and catching `LinkageError` per class, per R3's hardening note.
- `simrunner.SimConfig` — `sim.config` parsing (`sourceRoot`, `robotConfig`, `presetMotors`, `extraClasspath`).
- `simrunner.Executor` — drives `LinearOpMode` on its own thread with a background motor-tick loop standing in for Phase 2's real fixed-timestep sim loop.
- `simrunner.Watchdog` — isolates a wedged OpMode thread (see Validation).
- `simrunner.Main` — headless entry point: `java simrunner.Main <projectDir> <opModeName> [watchdogTimeoutMillis]`.
- `gui-runner/presets/` — three preset robot configs (paired `.robot_config.xml` + `.preset.json`), see below.
- `gui-runner/sample-teamcode/` — a real sample team project (`sim.config` + robot config + three OpModes) used for validation.

**`physics-engine/`** — empty scaffold, per spec. A placeholder README notes it's Phase 3/4's scope.

## Stub Coverage

All of R1's "required for Phase 1" classes are implemented, not just declared. Two categories worth distinguishing:

| Coverage level | Classes |
|---|---|
| **Implemented and exercised by a real sample OpMode** | `HardwareMap`, `DcMotorEx`, `DcMotor.Direction`, `IMU` (+ `RevHubOrientationOnRobot`, `AngleUnit`), `VoltageSensor`, `Telemetry`, `ElapsedTime`, `LinearOpMode`, `TeleOp`/`Autonomous`, `FtcDashboard`, `Config`, `TelemetryPacket` |
| **Implemented, compiles, not directly exercised by a sample OpMode this pass** | `Servo`, `CRServo`, `PIDFCoefficients`, `DistanceSensor`, `ColorSensor`, `TouchSensor`, `Range`, `RobotLog`, `LynxModule` (bulk-caching mode setters exist but no sample calls them), `MultipleTelemetry`, `Disabled` (present in discovery logic, no `@Disabled`-annotated sample class to prove the skip path) |

Nothing in R1's list was found to be missing or wrong once actually implemented — the earlier research pass's inventory held up.

## Validation

### 1. Real sample OpMode, headless, end-to-end (`BasicMecanumOpMode`)

A 4-motor Mecanum chassis OpMode: initializes hardware (including `IMU` with `RevHubOrientationOnRobot`), drives forward for 1s, strafes right for 1s, reading back encoder ticks and battery voltage via `telemetry` each loop. Actual captured output (trimmed):

```
[EXECUTOR] Discovered 3 OpMode(s) via @TeleOp/@Autonomous scan:
    - Deliberately Hung OpMode (watchdog test only) (org.firstinspires.ftc.teamcode.HungOpMode, Autonomous)
    - Dashboard Drive (org.firstinspires.ftc.teamcode.DashboardOpMode, TeleOp)
    - Basic Mecanum Auto (org.firstinspires.ftc.teamcode.BasicMecanumOpMode, Autonomous)
[EXECUTOR] Robot config: 4-Motor Mecanum + Arm + Slide (mirrors R6's worked example robot) (8 devices)
[EXECUTOR] Running Basic Mecanum Auto (watchdog timeout 5000ms)...
[TELEMETRY] Status : Initialized
[TELEMETRY] Phase : Forward | Ticks LF : 0 | Battery V : 12.6
[TELEMETRY] Phase : Forward | Ticks LF : 385 | Battery V : 12.6
...
[TELEMETRY] Phase : Strafe Right | Yaw : 0.0
[TELEMETRY] Status : Done. Final ticks LF=2874
[EXECUTOR] OpMode completed normally.
=== Final hardware state ===
    [left_front_drive] -> power=0.0 ticks=2874
    [right_front_drive] -> power=0.0 ticks=4
    [left_back_drive] -> power=0.0 ticks=-4
    [right_back_drive] -> power=0.0 ticks=-2874
```

Final tick pattern is directionally correct for the forward+strafe sequence (`left_front`/`right_back` accumulate large same-sign ticks, `right_front`/`left_back` roughly cancel) — confirming `HardwareMap` resolution, motor direction handling, and the executor's lifecycle wiring all work correctly together, not just in isolation.

**Real finding, not anticipated by the research:** the first run of this exact OpMode produced **50 MB of console output** in under a second, because the mock hardware calls are instant — with no I2C round-trip cost, an unthrottled `while` loop spins effectively unbounded. Real robots are naturally paced by hardware I/O latency (R5 found ~45 Hz typical); this mock has none yet. Fixed by adding an explicit `sleep(50)` in the sample OpModes' loops — but this is a **real, generalizable finding for Phase 2/3**, not just a sample-code fix: any OpMode without an explicit sleep will do this once compiled against this mock, and Phase 2/3 should decide whether the executor itself should impose a loop-rate cap rather than relying on every sample (and eventually every team's code) to self-throttle. Logged as an Open Issue for Phase 2 below.

### 2. Road-Runner-quickstart-style sample against the FTCDashboard shim (`DashboardOpMode`)

Imports `com.acmerobotics.dashboard.FtcDashboard` and `com.acmerobotics.dashboard.config.Config` directly, exactly like real quickstart-based team code. Captured output:

```
[EXECUTOR] Running Dashboard Drive (watchdog timeout 5000ms)...
[DASHBOARD] drivePower = 0.5
[DASHBOARD] leftFrontTicks = 0
[TELEMETRY] drivePower : 0.5
[DASHBOARD] drivePower = 0.5
[DASHBOARD] leftFrontTicks = 1375
[TELEMETRY] Status : Dashboard OpMode done. Config snapshot={Tuning.DRIVE_POWER=0.5}
[EXECUTOR] OpMode completed normally.
```

`Config snapshot={Tuning.DRIVE_POWER=0.5}` is the important line: it proves the shim's `@Config` reflection-based static-field scanning **actually works**, not just that the annotation compiles. `dashboard.sendTelemetryPacket(...)` and the regular Driver-Station-style `telemetry` both worked side by side in the same OpMode.

### 3. Deliberately hung OpMode does not wedge the executor (`HungOpMode`)

A sample OpMode with `while (true) { }` and no `opModeIsActive()` check — a real, common team bug. Captured output, full run:

```
[EXECUTOR] Running Deliberately Hung OpMode (watchdog test only) (watchdog timeout 2000ms)...
[WATCHDOG] OpMode thread "opmode-org.firstinspires.ftc.teamcode.HungOpMode" did not finish within 2000ms -- marking session dead. Thread.stop() is unavailable on modern JDKs, so the thread is isolated (left running as an orphaned daemon) rather than force-killed.
[EXECUTOR] Session marked DEAD by watchdog -- OpMode did not finish in time.
=== Final hardware state ===
    ...
MAIN PROCESS EXIT CODE: 0
```

The executor process **exited cleanly** (exit code 0) despite the hung thread still technically existing — because it's a daemon thread, the JVM doesn't wait for it. This confirms the watchdog design actually isolates a wedge rather than just detecting it and then hanging anyway.

## Preset Robot Configs

Three presets, each a paired `<name>.robot_config.xml` (real FTC config format, per R3) + `<name>.preset.json` (SKU/ratio, per R3/R4's finding that the XML alone can't carry gear ratio):

| Preset | Devices | Motors w/ SKU+ratio |
|---|---|---|
| `mecanum_basic` | 5 (4 drive motors + IMU) | 4/4 |
| `mecanum_arm` | 7 (+ arm motor, claw servo) | 5/5 |
| `mecanum_arm_slide` | 8 (+ slide motor) — mirrors R6's worked example robot | 6/6 |

All three independently verified to parse and build a live `HardwareMap` without error (confirmed via a standalone check, not just by the sample project using one of them). Drive-motor constants use the goBILDA 19.2:1 SKU with the encoder/torque figures independently verified against goBILDA's product page during the Opus review pass; arm/slide constants use the 50.9:1 SKU from R4's original datasheet fetch.

## Deviations from Research

- **Resolved in Phase 2:** at the time this phase was completed, no real Gradle build existed (this environment has JDK 17 but had no Gradle/Maven installed) and everything was compiled and run directly via `javac`/`java`. Phase 2 needed real dependency resolution for jMonkeyEngine and set up the actual Gradle multi-module build (`settings.gradle` + per-module `build.gradle`s) at the repo root — see Phase 2's RESULTS.md. JDK target is still 17, not R2's recommended 21 LTS (unchanged environment constraint).
- **R3's third-party Gradle-coordinate dependency resolution is not implemented.** `sim.config`'s `extraClasspath` manual escape hatch exists and works, but the automatic "parse `build.dependencies.gradle`, resolve against Maven Central/`maven.brott.dev`, unpack AARs" pipeline is not built. The Dashboard-shim validation above works *because the shim is bundled directly into `core-sdk-mock`*, not because a real Road Runner Maven dependency was resolved and compiled against — that specific scenario (a team's actual `implementation 'com.acmerobotics.dashboard:...'` line resolving correctly) remains untested. Flagged, not silently skipped.
- **Classloader isolation implements half of R3's design.** A fresh `URLClassLoader` is created per reload (the behaviorally important half — static team-code state resets cleanly), but it does not filter simulator-internal classes (`simcore.*`) from team-code visibility (the purity half). Team code *could* technically import `simcore.SimDcMotorEx` today, though no real team would have a reason to. Implementing the full parent/child filtering graph R3 specified is straightforward follow-up work, deliberately deferred here as disproportionate risk for what this validation pass needed to prove.
- **`MiniJson` is a ~100-line hand-rolled JSON reader, not a real library** (`org.json`, Jackson, etc.) — written only because no dependency-resolution mechanism exists yet (see above). Replace it once a real build exists; don't extend it further.
- **`SimDcMotorEx`'s tick integration is a placeholder** (linear ticks-per-second scaled by commanded power), explicitly not R4's corrected torque/current model. This was never in scope for Phase 1 — Phase 3 owns it — but it's worth being explicit that the numbers in the Validation section above are pipeline proof, not physically meaningful.
- **No `@Disabled` sample OpMode was written**, so the discovery skip-path for that annotation is implemented but not exercised by a running test in this pass.

## Open Issues for Phase 2

1. **Loop-rate throttling.** The 50 MB unthrottled-output discovery above is a real signal: Phase 2 (or the executor itself) should decide whether unthrottled OpMode loops are something the simulator actively rate-limits, or something documented as the team's own responsibility (matching real hardware, which self-throttles via I2C cost that doesn't exist yet in this mock). Recommend deciding this explicitly rather than letting every future sample/team hit it independently.
2. ~~Real Gradle build setup~~ — done in Phase 2 (see above). Third-party Gradle-coordinate dependency resolution (Road Runner/Pedro/FTCDashboard's own artifacts) is still open, now that a real build tool exists to do it in.
3. Phase 2 should confirm it can consume `Executor`'s output (motor power/position state) directly for its kinematics model — the interface exists (`HardwareMap.getAll(DcMotorEx.class)`) but Phase 2's spec should state exactly what it reads from Phase 1's executor rather than re-deriving it.
