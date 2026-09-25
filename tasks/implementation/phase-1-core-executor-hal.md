# Task P1: Core Executor & HAL

**Type:** Implementation
**Depends on (research):**
- [`tasks/research/01-sdk-surface-inventory.RESULTS.md`](../research/01-sdk-surface-inventory.RESULTS.md) — the stub class list to implement.
- [`tasks/research/02-engine-tech-stack-decision.RESULTS.md`](../research/02-engine-tech-stack-decision.RESULTS.md) — which engine/runtime to build on.
- [`tasks/research/03-classloading-local-directory-import.RESULTS.md`](../research/03-classloading-local-directory-import.RESULTS.md) — the classloading mechanism and `sim.config`/HardwareMap conventions to implement.
**Depends on (prior phase):** None — this is the first implementation phase.
**Produces:**
- Code: `core-sdk-mock/` (mocked SDK package), `gui-runner/` (headless executor + console logging entry point).
- Results doc: `tasks/implementation/phase-1-core-executor-hal.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 1
**Estimated duration (from plan):** 2 weeks

## Context

This is the first buildable phase and the riskiest assumption in the whole project: can real, unmodified team `OpMode` code actually load and run against a mock SDK? Everything else is built on top of this working. Do not proceed to Phase 2 until a real (or realistic sample) `LinearOpMode` runs headlessly end to end.

**Before writing any code**, read all three research RESULTS files listed above in full — they contain the specific decisions (which classes to stub, which engine, which classloading mechanism, the `sim.config` format, the `HardwareMap` naming convention) that this phase implements. Do not re-derive or second-guess those decisions here; if something in them seems wrong once you're implementing, note it in this phase's RESULTS file under "Deviations," don't silently diverge.

## Objective

1. A mocked `com.qualcomm.robotcore` package covering the classes R1 inventoried (this now explicitly includes no-op stub classes for vision — `VisionPortal`, `AprilTagProcessor`, etc. — since R1's amended results clarify that "vision deferred" means no simulated camera, not missing classes; real team code that merely constructs a `VisionPortal` in `init()` must still compile).
2. A headless `OpMode` executor: loads a team's code per R3's mechanism (including R3's third-party dependency resolution — team code importing Road Runner/Pedro/FTCDashboard must compile, not just the team's own source), drives the `init()`/`start()`/`loop()`/`stop()` lifecycle, and runs it against the mock.
3. **An `FtcDashboard`-compatible shim** (per R1's findings): a from-scratch implementation of the real `FtcDashboard`'s public API (`getInstance()`, `getTelemetry()`, `@Config` static-field scanning, `TelemetryPacket`), backed by the real, portable `DashboardCore` library, without the real class's Android-lifecycle machinery. This is required for Phase 1, not a later nice-to-have — R1 found that the standard Road Runner quickstart bundles FTCDashboard by default, so any quickstart-based team's code won't compile without it.
4. Basic console logging of `Telemetry` output and motor/servo commands.
5. 2–3 hand-authored preset robot configs (a static, minimal format — **not** R6's full importer; that's Phase 5) so there's something for an OpMode to actually drive. Include a per-motor SKU/ratio field even in this minimal format, since R3/R4 found the real robot-configuration XML likely can't carry gear-ratio information on its own.
6. A watchdog for runaway OpMode threads, per R3's design (daemon thread + dead-session isolation — `Thread.stop()` is not usable as of JDK 20+, confirmed in R2).

## Instructions

1. Set up the project structure: `core-sdk-mock/`, `physics-engine/` (empty scaffold for now), `gui-runner/`.
2. Implement the mock SDK classes from R1's inventory, in order of "trivial" → "hard" per R1's complexity rating. Each mock should be behaviorally correct enough for real code to run (e.g. `DcMotor.setPower()` actually stores a value the executor can read back; `HardwareMap.get()` performs the string-keyed lookup per R3's naming convention).
3. Implement the executor using the classloading mechanism from R3: point it at a folder containing a `sim.config` (per R3's format), compile/load the OpMode class, and drive its lifecycle on a simple fixed-timestep loop.
4. Wire `Telemetry.addData(...)` calls and motor/servo setter calls to console output so a developer can see the OpMode is actually executing and producing the expected commands.
5. Hand-author 2-3 preset robot configs as plain data (hardcoded objects or a minimal static JSON, whichever is less effort right now) representing, e.g., a 4-motor Mecanum chassis with goBILDA motor constants from R4 if it's done, or reasonable placeholder constants otherwise (note this in Deviations if R4 isn't ready yet).
6. **Validate:** take a real (or realistic sample) `LinearOpMode` that drives a Mecanum chassis via gamepad-style hardcoded inputs, and confirm it runs start-to-finish producing sane motor power output in the console log.

## Output contract

Save `tasks/implementation/phase-1-core-executor-hal.RESULTS.md` with:

- `## Summary`
- `## What Was Built` — module-by-module.
- `## Stub Coverage` — which R1 classes are implemented vs. still missing, and why if any are missing.
- `## Validation` — the sample OpMode used, and evidence it ran correctly (console output excerpt).
- `## Preset Robot Configs` — what was hand-authored, and their format (this format needs to stay compatible with what Phase 5 will eventually replace it with).
- `## Deviations from Research` — anything where you diverged from R1/R2/R3's decisions, and why.
- `## Open Issues for Phase 2`

## Definition of done

- [ ] All three research RESULTS files read and referenced.
- [ ] Mock SDK covers at minimum the classes R1 marked required, including the vision stub classes and the FtcDashboard shim.
- [ ] A real/realistic sample OpMode runs headlessly end-to-end with correct console output.
- [ ] A Road-Runner-quickstart-style sample (importing FTCDashboard) compiles and runs against the shim.
- [ ] 2-3 preset configs exist, are loadable, and carry a per-motor SKU/ratio field.
- [ ] A deliberately-hung OpMode is confirmed not to wedge the whole executor (watchdog isolates it).
- [ ] RESULTS.md saved at the path above.
