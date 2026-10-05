# Library compatibility

`gui-runner/compat-fixtures` is a team-project layout (`sim.config`, robot XML with a Pinpoint and an OTOS,
motor presets) holding two OpModes. `CompatFixturesTest` compiles it, discovers the OpModes and runs them headless
through the real executor on every `./gradlew test`.

| Fixture | Pattern exercised |
|---|---|
| `RoadRunnerStyleOpMode` | Structure of the Road Runner quickstart's `MecanumDrive`: hub bulk caching (`getAll(LynxModule.class)`), `hardwareMap.voltageSensor` feed-forward, IMU init with `RevHubOrientationOnRobot`, `@Config`, `MultipleTelemetry`, `TelemetryPacket.fieldOverlay()` drawing. |
| `PedroStyleOpMode` | Structure of a Pedro Pathing Pinpoint localizer: offsets, pod resolution, encoder directions, `resetPosAndIMU`, `setPosition`, `update()`/`getPosition()` each loop, plus an OTOS in inches/degrees. |

## What this does and does not prove

- It proves the **SDK-facing calls** these libraries make compile and run against the stubs, with sagging hub voltage and
  moving encoders.
- It does **not** run the real Road Runner or Pedro Pathing jars. They are Kotlin libraries from custom Maven
  repositories and were not downloaded or tested here. To try them, add their jars (plus the Kotlin standard library) to
  `extraClasspath` in `sim.config`. Whatever then fails to compile is reported with a "Simulator support notes"
  section; please add real findings to this file.
- Pose devices follow native chassis motion in the 3D renderer and [physics-backed headless runs](HEADLESS_PHYSICS.md).
  Compiled closed-loop TeamCode tests validate native Pinpoint/OTOS and IMU feedback. Legacy motor-only headless runs
  keep the start pose. These tests do not establish compatibility with real vendor pathing jars.
- Method lists of the Pinpoint and OTOS stubs were taken from the vendors' public source
  (goBILDA `GoBildaPinpointDriver.java`, SparkFun `SparkFunOTOS.java`). The goBILDA file is a vendored driver in
  its own repository; the SDK's built-in copy may differ by version. Pin the version you test against.

See [`core-sdk-mock/SDK_COVERAGE.md`](../core-sdk-mock/SDK_COVERAGE.md) for the full stub roster.
