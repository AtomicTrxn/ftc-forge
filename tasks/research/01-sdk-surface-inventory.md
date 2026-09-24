# Task R1: SDK Surface Inventory

**Type:** Research
**Depends on:** None — this is the entry point of the pipeline.
**Produces:** `tasks/research/01-sdk-surface-inventory.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 3 Step 1

## Context

This project is an open-source FTC (FIRST Tech Challenge) robot simulator. Its core promise is **zero-refactor execution**: a team's real, unmodified Java `OpMode`/`LinearOpMode` source should run against a *mocked* version of the FTC SDK (`com.qualcomm.robotcore` and friends), inside a simulator, with no code changes. Before anything can be built or estimated, we need a precise inventory of what that mock has to cover. This task is purely investigative — no code, no engine choice yet. Its output is a direct input to Task R2 (engine decision), because whichever classes/libraries turn out to be hardest to mock will determine whether a web/WASM target is even viable.

## Objective

Produce a complete, versioned inventory of:
1. The `com.qualcomm.robotcore` (and related, e.g. `org.firstinspires.ftc.robotcore`) classes/interfaces that a mock SDK must implement to run typical competition OpModes.
2. The Android-specific coupling in the three most common third-party FTC libraries (Road Runner, Pedro Pathing, FTCDashboard) that a non-Android runtime would need to work around.
3. A firm answer on whether camera-based vision (`VisionPortal`, AprilTag) is in scope for v1.

## Instructions

1. **Pin the target SDK version.** Identify the current FTC competition season's SDK release (check the official FIRST Tech Challenge SDK GitHub repo for the latest tagged release). Record the exact version/tag you're inventorying against — this prevents scope drift as the SDK evolves season to season.
2. **Enumerate the stub surface.** Working from real sample OpModes (write a few yourself, or find public examples), list every `com.qualcomm.robotcore`-family class/interface a typical team touches: `HardwareMap`, `Gamepad`, `Telemetry`, `DcMotor`/`DcMotorEx`, `Servo`, `IMU`, `DistanceSensor`, `ColorSensor`, `TouchSensor`, `VoltageSensor`, `LinearOpMode`/`OpMode` base classes, `ElapsedTime`, etc. For each: note its public method surface (only what's actually called from OpModes — not the whole class), and whether it's trivial to stub (pure data/getter-setter) or has real behavioral logic to fake (e.g. motor mode switching, encoder semantics).
3. **Audit Road Runner.** Identify what it imports/depends on beyond the standard SDK surface — threading model, static initializers, file I/O (it reads/writes tuning config files), any Android-only APIs (`android.util.Log`, `SharedPreferences`, `Looper`).
4. **Audit Pedro Pathing.** Same audit as Road Runner.
5. **Audit FTCDashboard.** This one is the highest-risk: it runs an embedded web server (`NanoHTTPD`) inside the robot controller app for live tuning. Determine whether this can run unmodified outside Android, or needs its own mock/replacement.
6. **Decide on vision scope.** `VisionPortal`/AprilTag detection depends on native OpenCV-Android camera bindings — a fundamentally different and much larger mocking problem than motor/sensor stubbing. Make an explicit call: in scope for v1, or deferred. (Recommended default: **defer**. Document your reasoning either way.)

## Decisions required

| Question | Recommended default |
|---|---|
| Which SDK version is the pinned target? | Current competition season's release; re-pin each offseason. |
| Is vision (`VisionPortal`/AprilTag) in scope for v1? | **Defer.** It's a separate, larger mocking problem (native camera + OpenCV bindings) than motor/sensor stubbing, and the simulator delivers real value (motion/kinematics testing) without it. |

Document which default you used, or your justified alternative, in the RESULTS file.

## Output contract

Save to `tasks/research/01-sdk-surface-inventory.RESULTS.md` with these sections:

- `## Summary` — 3-5 sentences, the headline findings.
- `## Pinned SDK Version` — exact version/tag and link.
- `## Stub Class Inventory` — a table: Class/Interface | Methods Used by OpModes | Stub Complexity (trivial/moderate/hard) | Notes.
- `## Third-Party Library Coupling Audit` — one subsection each for Road Runner, Pedro Pathing, FTCDashboard: what's Android-coupled, how hard it'd be to run outside Android.
- `## Vision Scope Decision` — the call made, and the reasoning.
- `## Risks / Unknowns Remaining` — anything you couldn't fully resolve.
- `## Recommendation for R2 (Engine Decision)` — a direct, actionable note: given what you found, does anything here rule out the Web/WASM option outright? Be specific.

## Definition of done

- [ ] SDK version pinned and recorded.
- [ ] Stub class table covers at minimum: HardwareMap, Gamepad, Telemetry, DcMotor/DcMotorEx, Servo, IMU, DistanceSensor, ElapsedTime, OpMode/LinearOpMode.
- [ ] All three third-party libraries audited.
- [ ] Vision scope decision explicitly recorded.
- [ ] RESULTS.md saved at the path above, following the output contract.
