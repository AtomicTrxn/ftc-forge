# R1 Results: SDK Surface Inventory

**Status:** Complete
**Task spec:** [01-sdk-surface-inventory.md](01-sdk-surface-inventory.md)

## Summary

The FTC SDK's OpMode-facing API surface is smaller and more stable than the whole codebase suggests — a Mecanum-chassis autonomous/TeleOp OpMode realistically touches under 15 classes/interfaces, most of which are simple data holders. The two real risks are (1) third-party libraries reaching *past* that stable surface into internal SDK infrastructure classes, and (2) vision. FTC Dashboard in particular is not just "an embedded web server" as originally assumed — its FTC-facing wrapper is deeply wired into the Android app lifecycle (`Activity`, `Context`, annotation-driven `@OnCreate`/`@OnCreateEventLoop` hooks, `OpModeManagerImpl.Notifications`), while its actual dashboard server (`DashboardCore`) is a separable, portable Java library. Vision (`VisionPortal`/AprilTag) is confirmed to depend on OpenCV-Android native bindings via EasyOpenCV, a library independently confirmed to be fragile to internal SDK package moves — reinforcing the recommendation to defer it.

## Pinned SDK Version

**v12.0** — "the official release for the 2026/2027 (BIOBUZZ) season," from the [FIRST-Tech-Challenge/FtcRobotController releases page](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases). Requires Android Studio Narwhal 3 Feature Drop or later to build.

Note: the exact release date on that page could not be reliably extracted by the fetch tooling used here (it returned an internally inconsistent date) — confirm the date directly on the releases page if it matters for scheduling. The tag `v12.0` itself is solid and is what this inventory targets. **Re-pin this to whatever the current season's tag is at the start of each offseason**, per the plan's decision default.

## Stub Class Inventory

| Class/Interface | Package | Methods Used by OpModes | Stub Complexity | Notes |
|---|---|---|---|---|
| `HardwareMap` | `com.qualcomm.robotcore.hardware` | `get(Class, String)`, `get(String)`, `tryGet(...)` | **Moderate** | Core lookup registry. Behaviorally it's just a typed, string-keyed map, but every other stub's discoverability depends on getting this exactly right — this is the class R3 (classloading) and R6 (CAD import) both key off of. |
| `OpMode` | `com.qualcomm.robotcore.eventloop.opmode` | `init()`, `init_loop()`, `start()`, `loop()`, `stop()`; fields `gamepad1`, `gamepad2`, `telemetry`, `hardwareMap` | **Moderate** | Base class; the executor's lifecycle driver (Phase 1) is essentially a scheduler for these five methods. |
| `LinearOpMode` | `com.qualcomm.robotcore.eventloop.opmode` | `runOpMode()`, `waitForStart()`, `opModeIsActive()`, `isStopRequested()` | **Moderate** | The far more common style in modern team code. `runOpMode()` runs on its own thread in the real SDK — the mock must replicate that threading contract or blocking calls like `waitForStart()` will deadlock the executor. |
| `Gamepad` | `com.qualcomm.robotcore.hardware` | Public fields: `left_stick_x/y`, `right_stick_x/y`, `a/b/x/y`, `dpad_*`, `left_bumper`, `right_bumper`, `left_trigger`, `right_trigger`, etc. | **Trivial** | Plain data-holder class; the only design decision is how simulated/manual gamepad input gets written into these fields each tick. |
| `Telemetry` (+ `Telemetry.Item`, `.Line`, `.Log`) | `org.firstinspires.ftc.robotcore.external` | `addData(...)`, `update()`, `addLine(...)`, `clear()` | **Trivial** | Pure data-sink interface; console logging (Phase 1) is a direct, correct implementation. |
| `DcMotor` | `com.qualcomm.robotcore.hardware` | `setPower()`, `getPower()`, `setMode()`, `setDirection()`, `setZeroPowerBehavior()`, `getCurrentPosition()`, `setTargetPosition()` | **Moderate** | `RunMode` semantics (`RUN_WITHOUT_ENCODER`, `RUN_USING_ENCODER`, `RUN_TO_POSITION`, `STOP_AND_RESET_ENCODER`) have real behavioral differences a mock must actually implement, not just accept and ignore. |
| `DcMotorEx` | `com.qualcomm.robotcore.hardware` | `setVelocity()`, `getVelocity()`, `setPIDFCoefficients()`, `getCurrent()`, `setCurrentAlert()` | **Hard** | Extends `DcMotor`; the velocity/PIDF/current-draw surface is exactly what R4's motor model has to back — this stub is only as good as R4's math. |
| `Servo` | `com.qualcomm.robotcore.hardware` | `setPosition()`, `getPosition()`, `setDirection()` | **Trivial** | Simple normalized-position data holder. |
| `IMU` (unified interface, SDK 8.1+) | `com.qualcomm.robotcore.hardware` | `initialize(IMU.Parameters)`, `getRobotYawPitchRollAngles()`, `getRobotAngularVelocity()`, `resetYaw()` | **Moderate** | Replaced the old `BNO055IMU`-specific API; supports both BHI260AP and BNO055 orientation. Needs an `orientationOnRobot` parameter model even in the mock, since teams configure hub mounting orientation and get real behavioral differences from it. |
| `DistanceSensor` | `com.qualcomm.robotcore.hardware` | `getDistance(DistanceUnit)` | **Trivial** (interface) → **Moderate** (realistic behavior) | Trivial to stub as an interface; realistic behavior needs a raycast against the simulated field/game-piece geometry (a Phase 4 dependency, not Phase 1). |
| `ColorSensor` | `com.qualcomm.robotcore.hardware` | `red()`, `green()`, `blue()`, `alpha()` | **Trivial** (interface) → **Moderate** (realistic behavior) | Same pattern as `DistanceSensor` — trivial interface, non-trivial simulated behavior. |
| `TouchSensor` | `com.qualcomm.robotcore.hardware` | `isPressed()` | **Trivial** | Boolean state, driven by simulated collision in later phases. |
| `VoltageSensor` | `com.qualcomm.robotcore.hardware` | `getVoltage()` | **Trivial** | Direct pass-through of R4's battery sag model output. |
| `ElapsedTime` | `com.qualcomm.robotcore.util` | `reset()`, `seconds()`, `milliseconds()` | **Trivial** | Pure timer; needs to run off the simulator's own clock (not wall-clock) once the sim supports non-realtime/paused stepping. |

## Third-Party Library Coupling Audit

### Road Runner

- Written in **Kotlin** (`com.acmerobotics.roadrunner:core`), distributed via a custom Maven repo (`maven.brott.dev`), not Maven Central/JCenter.
- Depends on the standard FTC hardware surface above (`DcMotorEx`, `IMU`, `HardwareMap`) — no evidence found of direct Android-framework imports (`Activity`, `Looper`, `SharedPreferences`) in the core path-planning library itself.
- The commonly-cited "method reference / multidex" concern in its docs is an **Android DEX 64k-method-limit packaging issue**, which is irrelevant outside an Android APK build — this is actually good news for a desktop-JVM target, not a blocker.
- Kotlin bytecode is standard JVM bytecode, so native JVM classloading (Option A) handles it the same as Java. For a transpiled/web target (Option B), the transpiler (CheerpJ/TeaVM) would need Kotlin-stdlib compatibility, which is an added variable not required by the Java-only case.
- **Verdict: low Android coupling risk**, assuming the Kotlin stdlib is bundled with the mock SDK's classpath.

### Pedro Pathing

- Distributed as `com.pedropathing:ftc`, with FTCLib and SolversLib wrapper integrations also in circulation.
- **Could not verify internal source-level details** (threading model, file I/O, Android imports) through the tools available in this session — GitHub's repo page didn't expose source file contents through the fetch used here. This is a **genuine unknown**, not a confirmed "low risk" — see Risks below.
- What is confirmed: like Road Runner, it consumes the same `HardwareMap`/`DcMotorEx`/`IMU` surface as team code, so at minimum it doesn't introduce a *new* stub-class requirement beyond what's in the table above.

### FTCDashboard

- **Confirmed real risk, and worse than the plan's original framing.** The library is split into two pieces:
  - **`DashboardCore`** — described by its own docs as "a standalone library that can be used to create a dashboard server for any Java application." This part uses `NanoHTTPD`/`NanoWSD` (`fi.iki.elonen`), which is a **pure Java** HTTP/WebSocket library with no Android dependency at all.
  - **`FtcDashboard`** — the FTC-specific wrapper class inspected directly (`FtcDashboard.java`) imports `android.app.Activity`, `android.content.Context`, `android.content.SharedPreferences`, `android.content.res.AssetManager`, `android.graphics.Bitmap`/`Typeface`, `android.util.Log`, and Android UI classes (`Menu`, `MenuItem`, `LinearLayout`, `TextView`). It also implements `OpModeManagerImpl.Notifications` and uses **annotation-driven Android lifecycle hooks** (`@OnCreate`, `@OnCreateEventLoop`, `@OnCreateMenu`, `@OnDestroy`) tied to the robot controller app's internal event loop (`FtcEventLoop`, `RobotConfigFileManager`).
- **Implication:** the real `FtcDashboard` class cannot run unmodified outside Android. But because the actual server logic lives in the portable `DashboardCore`, the mock SDK can plausibly implement its own thin `FtcDashboard`-equivalent shim — same public API teams call (e.g. `FtcDashboard.getInstance()`, `.getTelemetry()`) — backed by the real `DashboardCore` server, without needing the Android lifecycle machinery at all.
- Separately confirmed: `OpModeManagerImpl` has moved internal packages before and **broke FTC Dashboard (≤0.4.4) and EasyOpenCV (≤1.5.1)** as a result — a concrete precedent that third-party libraries in this ecosystem do reach into non-public/internal SDK classes, not just the documented public surface. Any library audit for this project should be re-checked against the exact SDK version pinned above, every season.

## Vision Scope Decision

**Deferred for v1**, per the plan's recommended default — and this research reinforces rather than just accepts that default:

- `VisionPortal`/AprilTag detection depends on **EasyOpenCV**, which wraps native OpenCV-Android camera bindings.
- EasyOpenCV is independently confirmed (via the `OpModeManagerImpl` package-move incident above) to be a library that reaches into internal, non-public SDK infrastructure — meaning a vision mock isn't just "stub a camera interface," it's "stub a moving, version-sensitive internal integration surface," which is a materially larger and higher-maintenance undertaking than the rest of this inventory.
- Recommendation stands: ship motion/kinematics simulation without vision; revisit once the core executor and non-ideality modeling are proven out.

## Risks / Unknowns Remaining

- **Pedro Pathing's internal Android coupling is unverified**, not confirmed-safe. Before R2 finalizes an engine decision on the assumption that "third-party libraries are fine," someone should either compile Pedro Pathing's source directly (clone the repo, inspect imports with e.g. `grep -r "^import android" .`) or run a spike loading it in a plain JVM. This is the single biggest gap in this inventory.
- The exact release date of SDK v12.0 could not be reliably confirmed through the tooling used in this task — not load-bearing for the inventory itself, but worth a 30-second manual check if it affects scheduling.
- `DistanceSensor`/`ColorSensor` "moderate" complexity is deferred to Phase 4 (needs field/game-piece geometry) — flagged here so Phase 1 doesn't accidentally scope-creep into building that early.

## Recommendation for R2 (Engine Decision)

**Nothing found here outright disqualifies Web/WASM (Option B), but nothing found here makes it free, either:**

1. Road Runner's Kotlin bytecode adds a transpilation variable for Option B that doesn't exist for Option A (native JVM handles Kotlin bytecode identically to Java; CheerpJ/TeaVM compatibility with Kotlin-stdlib is a separate, unverified question).
2. FTCDashboard's real risk isn't transpilation at all — it's Android-app-lifecycle coupling, which is equally a problem for *both* options, since neither runs inside an actual Android robot controller app. This means R2's decision doesn't resolve the Dashboard problem either way; **a from-scratch `FtcDashboard`-compatible shim over `DashboardCore` is required regardless of engine choice**, and should be scoped as its own follow-up task rather than assumed away.
3. Pedro Pathing remains an open unknown that should be resolved (via the spike above) before, or in parallel with, R2 — if it turns out to have Android coupling as deep as FTCDashboard's, that's a materially different risk picture for Option B than assumed.
