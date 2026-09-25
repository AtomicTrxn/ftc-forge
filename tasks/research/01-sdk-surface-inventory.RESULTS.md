# R1 Results: SDK Surface Inventory

**Status:** Complete (amended after Opus review pass — see changelog)
**Task spec:** [01-sdk-surface-inventory.md](01-sdk-surface-inventory.md)

**Changelog:** stub table expanded (bulk-caching, orientation, navigation units, servo/PIDF, logging, OpMode annotations); clarified that "deferred" vision still needs stub classes for team code to compile; Road Runner audit extended to the quickstart/`road-runner-ftc` artifact; stale Pedro Pathing risk note removed (superseded by the resolution already recorded below).

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
| `com.qualcomm.hardware.lynx.LynxModule` | `com.qualcomm.hardware.lynx` | `getAll(LynxModule.class)`, `setBulkCachingMode(...)`, `clearBulkCache()` | **Moderate** | Bulk caching is standard-practice FTC code (reduces per-loop I2C round trips) and appears in the Road Runner quickstart. Needed for Phase 1 compile-correctness, and R5's logger needs to know each robot's caching mode to interpret `loop_time_ms` honestly. |
| `com.qualcomm.hardware.rev.RevHubOrientationOnRobot` | `com.qualcomm.hardware.rev` | Constructor + `LogoFacingDirection`/`UsbFacingDirection` enums | **Trivial** | Required to construct `IMU.Parameters` — team code that initializes the IMU (nearly all of it) references this class even though it never appeared in the original stub list. |
| `org.firstinspires.ftc.robotcore.external.navigation.{YawPitchRollAngles, AngularVelocity, AngleUnit, DistanceUnit, CurrentUnit}` | `org.firstinspires.ftc.robotcore.external.navigation` | Unit-conversion accessors (`getYaw(AngleUnit)`, etc.) | **Trivial** | Pure value/unit-conversion types returned by `IMU`/`DistanceSensor`/`DcMotorEx.getCurrent(CurrentUnit)` — omitted from the original table despite those methods already being listed. |
| `com.qualcomm.robotcore.hardware.{DcMotorSimple, DcMotorSimple.Direction, CRServo, PIDFCoefficients, HardwareDevice}` | `com.qualcomm.robotcore.hardware` | `setDirection()`, `setPower()` (CRServo), PIDF getters/setters | **Trivial–Moderate** | `DcMotorSimple` is the actual supertype `DcMotor` extends (direction lives here); `CRServo` is the continuous-rotation servo interface distinct from `Servo` — both are common in real drivetrains/intakes and were missing from the original table. |
| `com.qualcomm.robotcore.util.{Range, RobotLog}` | `com.qualcomm.robotcore.util` | `Range.clip(...)`, `RobotLog.a/d/e/ii(...)` | **Trivial** | `Range.clip` in particular shows up in almost every drivetrain/PID snippet teams copy from the docs. |
| `@TeleOp`, `@Autonomous`, `@Disabled` | `com.qualcomm.robotcore.eventloop.opmode` | Class-level annotations | **Trivial** | Not a runtime class at all, but must exist for compilation — and R3's discovery-by-annotation design (scanning for `@TeleOp`/`@Autonomous`) directly depends on these being the real, correctly-named annotation types, not a stand-in. |

**Also likely required, not independently verified in this pass:** `com.qualcomm.hardware.gobilda.GoBildaPinpointDriver` and `com.qualcomm.hardware.sparkfun.SparkFunOTOS` — both are increasingly common default localizer sensors in current Road Runner/Pedro configs. Confirm their exact package paths before Phase 1 locks the stub list.

**Required-for-Phase-1 column:** rather than reformat the table, the simple rule is: everything above except `DistanceSensor`/`ColorSensor`'s *realistic behavior* (needs Phase 4 geometry) and the two Pinpoint/OTOS drivers (needs confirmation) is required for Phase 1's compile-and-run goal.

## Third-Party Library Coupling Audit

### Road Runner

- Written in **Kotlin** (`com.acmerobotics.roadrunner:core`), distributed via a custom Maven repo (`maven.brott.dev`), not Maven Central/JCenter.
- Depends on the standard FTC hardware surface above (`DcMotorEx`, `IMU`, `HardwareMap`) — no evidence found of direct Android-framework imports (`Activity`, `Looper`, `SharedPreferences`) in the core path-planning library itself.
- The commonly-cited "method reference / multidex" concern in its docs is an **Android DEX 64k-method-limit packaging issue**, which is irrelevant outside an Android APK build — this is actually good news for a desktop-JVM target, not a blocker.
- Kotlin bytecode is standard JVM bytecode, so native JVM classloading (Option A) handles it the same as Java. For a transpiled/web target (Option B), the transpiler (CheerpJ/TeaVM) would need Kotlin-stdlib compatibility, which is an added variable not required by the Java-only case.
- **Verdict: low Android coupling risk**, assuming the Kotlin stdlib is bundled with the mock SDK's classpath.
- **Amendment: this audit covered only the `core` path-planning artifact.** Teams actually consume Road Runner through the `road-runner-quickstart` project, which bundles Road Runner **and FTCDashboard together** ("a full FTC Android Studio project with preinstalled Road Runner, FTC Dashboard, and tuning utilities" — confirmed directly in this research's own source search, not a new claim). Practical effect: any team using the standard Road Runner quickstart pulls in the FTCDashboard coupling described below whether or not they use the dashboard directly — the two libraries are not independent risks for a real team's codebase, they arrive together. **The FTCDashboard shim (see below) is therefore a Phase 1 blocker for Road-Runner-quickstart-based teams, not an optional nice-to-have.**

### Pedro Pathing

**Update (resolved during R2):** direct inspection of the repo's file tree and source imports (`Pedro-Pathing/PedroPathing`, `main` branch) confirms a clean two-module split:
- **`core/`** (`Follower`, `Localizer`, `Path`/`Curve`/`PathSegment`, `PIDController`, `Pose`/`Vector`/`Twist` math) — **zero Android or FTC SDK imports of any kind.** Pure Java, fully portable.
- **`revhub/`** (`Mecanum`, `Swerve`, `CachedMotor`, `CoaxialPod`) — imports only the standard stub surface already in the table above (`DcMotor`, `DcMotorEx`, `HardwareMap`, `org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit`). One file (`Mecanum.java`) imports `android.annotation.SuppressLint` — a compile-time-only, no-op annotation with no runtime behavior, trivially satisfied with an empty `@interface SuppressLint {}` stub.
- **Verdict: negligible Android coupling — cleaner than Road Runner's story, not worse.** This closes the gap flagged below; no further spike needed.

### FTCDashboard

- **Confirmed real risk, and worse than the plan's original framing.** The library is split into two pieces:
  - **`DashboardCore`** — described by its own docs as "a standalone library that can be used to create a dashboard server for any Java application." This part uses `NanoHTTPD`/`NanoWSD` (`fi.iki.elonen`), which is a **pure Java** HTTP/WebSocket library with no Android dependency at all.
  - **`FtcDashboard`** — the FTC-specific wrapper class inspected directly (`FtcDashboard.java`) imports `android.app.Activity`, `android.content.Context`, `android.content.SharedPreferences`, `android.content.res.AssetManager`, `android.graphics.Bitmap`/`Typeface`, `android.util.Log`, and Android UI classes (`Menu`, `MenuItem`, `LinearLayout`, `TextView`). It also implements `OpModeManagerImpl.Notifications` and uses **annotation-driven Android lifecycle hooks** (`@OnCreate`, `@OnCreateEventLoop`, `@OnCreateMenu`, `@OnDestroy`) tied to the robot controller app's internal event loop (`FtcEventLoop`, `RobotConfigFileManager`).
- **Implication:** the real `FtcDashboard` class cannot run unmodified outside Android. But because the actual server logic lives in the portable `DashboardCore`, the mock SDK can plausibly implement its own thin `FtcDashboard`-equivalent shim — same public API teams call (e.g. `FtcDashboard.getInstance()`, `.getTelemetry()`) — backed by the real `DashboardCore` server, without needing the Android lifecycle machinery at all.
- Separately confirmed: `OpModeManagerImpl` has moved internal packages before and **broke FTC Dashboard (≤0.4.4) and EasyOpenCV (≤1.5.1)** as a result — a concrete precedent that third-party libraries in this ecosystem do reach into non-public/internal SDK classes, not just the documented public surface. Any library audit for this project should be re-checked against the exact SDK version pinned above, every season.
- The shim's required scope is larger than just the notification interface: real teams also use `@Config`-annotated static fields for live-tunable constants (read by the dashboard's web UI), `MultipleTelemetry` (to mirror telemetry to both Driver Station and dashboard), `TelemetryPacket`, and `Canvas` (for drawing paths/robot pose on the dashboard's field view). A shim that only answers `FtcDashboard.getInstance().getTelemetry()` will compile far less real team code than one that also handles these.

## Vision Scope Decision

**Deferred for v1**, per the plan's recommended default — and this research reinforces rather than just accepts that default:

- `VisionPortal`/AprilTag detection depends on **EasyOpenCV**, which wraps native OpenCV-Android camera bindings.
- EasyOpenCV is independently confirmed (via the `OpModeManagerImpl` package-move incident above) to be a library that reaches into internal, non-public SDK infrastructure — meaning a vision mock isn't just "stub a camera interface," it's "stub a moving, version-sensitive internal integration surface," which is a materially larger and higher-maintenance undertaking than the rest of this inventory. (The real driver of that cost is native `libopencv`/AprilTag JNI binaries, real UVC webcam access, and rendering a synthetic camera image of the field for detection to run against — not primarily the SDK package-instability precedent, which is suggestive but weaker evidence on its own.)
- **"Deferred" needs to be precise: it means no simulated camera output, not that the vision classes don't exist.** Since SDK 8.2, `VisionPortal`/AprilTag is first-party code under `org.firstinspires.ftc.vision`, and a large share of current competitive autonomous OpModes call `VisionPortal.Builder`/`AprilTagProcessor.Builder` in `init()` even on robots that don't strictly need it for that match — "zero-refactor" fails at compile time the moment those classes are simply missing. **v1 must still ship no-op stub classes** for `org.firstinspires.ftc.vision.VisionPortal` (+ `.Builder`), `org.firstinspires.ftc.vision.apriltag.AprilTagProcessor` (+ `.Builder`), `AprilTagDetection`, and `org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName` — they compile and construct successfully, report the camera as unavailable, and return empty detection lists. The deferral is about not *simulating a camera*, not about omitting the classes.
- Recommendation stands: ship motion/kinematics simulation without a real simulated camera; revisit full vision simulation once the core executor and non-ideality modeling are proven out.

## Risks / Unknowns Remaining

- The exact release date of SDK v12.0 could not be reliably confirmed through the tooling used in this task — not load-bearing for the inventory itself, but worth a 30-second manual check if it affects scheduling.
- `DistanceSensor`/`ColorSensor` "moderate" complexity is deferred to Phase 4 (needs field/game-piece geometry) — flagged here so Phase 1 doesn't accidentally scope-creep into building that early.
- `GoBildaPinpointDriver`/`SparkFunOTOS` package paths are not independently verified — confirm before Phase 1 locks the stub list (see table note above).
- The FTCDashboard wrapper's implemented interface is recorded above as `OpModeManagerImpl.Notifications`, taken directly from a source fetch of `FtcDashboard.java` during this task — treat that as the sourced answer, not a guess, but re-confirm against whatever SDK version is pinned when the shim is actually implemented, since this class has moved before (see the `OpModeManagerImpl` package-move precedent above).
- Dependency resolution for third-party libraries (getting Road Runner/Pedro/FTCDashboard's own JARs/AARs onto team code's compile classpath, separate from the mock-SDK stubs this task inventories) is out of scope for this task but is a real, unassigned gap — see R3's results for where this now lives.

## Recommendation for R2 (Engine Decision)

**Nothing found here outright disqualifies Web/WASM (Option B), but nothing found here makes it free, either:**

1. Road Runner's Kotlin bytecode adds a transpilation variable for Option B that doesn't exist for Option A (native JVM handles Kotlin bytecode identically to Java; CheerpJ/TeaVM compatibility with Kotlin-stdlib is a separate, unverified question).
2. FTCDashboard's real risk isn't transpilation at all — it's Android-app-lifecycle coupling, which is equally a problem for *both* options, since neither runs inside an actual Android robot controller app. This means R2's decision doesn't resolve the Dashboard problem either way; **a from-scratch `FtcDashboard`-compatible shim over `DashboardCore` is required regardless of engine choice**, and should be scoped as its own follow-up task rather than assumed away.
3. Pedro Pathing: **resolved** (see the audit above) — negligible Android coupling, cleaner than Road Runner's own story. No longer a factor in this decision either way.
