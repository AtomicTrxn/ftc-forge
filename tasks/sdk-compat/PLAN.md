# SDK coverage audit and library compatibility harness

Baseline: `996edfd`. Follows the next-round proposal items 2 and 1 (audit first, harness second).

## Findings that shaped the plan

- Many R1 blockers already existed: `LynxModule`, an FtcDashboard shim, inert vision stubs.
- Missing: Pinpoint and OTOS drivers, `Pose2D`, `UnnormalizedAngleUnit`, dashboard `Canvas`/`fieldOverlay()`.
- Hubs were plain `VoltageSensor`s, so `getAll(LynxModule.class)` (used by the Road Runner quickstart) returned nothing.
- Unsupported imports surfaced as raw javac errors with no hint that the code is fine on a robot.
- No test tied the stub surface to any list, so regressions were invisible.

## Item 2: SDK coverage audit

1. `core-sdk-mock/sdk-surface.txt`: manifest of every stub class, support status and required members.
2. `SdkSurfaceTest`: fails when a class/member vanishes or a new stub class is unlisted.
3. `UnsupportedApiHints`: compile failures on unsupported imports append "Simulator support notes" (vision, Android,
   vendored I2C drivers, Road Runner/Pedro jars, any unlisted SDK class).
4. `SDK_COVERAGE.md` explains levels and gaps.

## Item 1: compatibility harness

1. Hubs become `LynxModule`s that still report battery-sag voltage.
2. Add `GoBildaPinpointDriver`, `SparkFunOTOS`, `Pose2D`, `UnnormalizedAngleUnit`, unit conversions, `Canvas`.
3. `OdometryTracker`/`PoseSink`: devices report the physics chassis pose in the start frame; renderer feeds them each tick.
   Robot XML tags `goBILDAPinpoint` / `SparkFunOTOS` create them.
4. `gui-runner/compat-fixtures` with Road Runner- and Pedro-style OpModes, run headless by `CompatFixturesTest`.
5. `COMPATIBILITY.md` states exactly what is and is not proven.

## Out of scope (stated, not claimed)

Running the real Road Runner/Pedro jars, I2C latency and drift for odometry devices, bulk-read batching semantics,
vision. GPU-rendered verification of the pose feed.
