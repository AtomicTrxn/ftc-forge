# Run TeamCode with physics without opening a window

Use this mode to exercise autonomous paths, sensor feedback and mechanisms with the robot
and field selected in your project's `sim.config`. It compiles and discovers real TeamCode,
then advances the same native collision, drivetrain and mechanism classes used by the 3D
simulator. A renderer, display and gamepad are unnecessary. Gamepads start neutral.

## Quick start

From the repository root:

```sh
./gradlew :gui-runner:runHeadlessPhysics --args='gui-runner/sample-teamcode TurnAndResetOpMode --duration 3 --report build/headless-physics/turn.json'
```

The existing entry point also accepts the physics flag as its third argument:

```sh
./gradlew :gui-runner:run --args='gui-runner/sample-teamcode TurnAndResetOpMode --physics --duration 3 --report build/headless-physics/turn.json'
```

Replace the sample project with your project directory and the OpMode with its annotated
class name, fully qualified name or display name. Quote display names and paths containing
spaces within Gradle's argument string. A full JDK is required for runtime compilation.

| Option | Default | Meaning |
|---|---|---|
| `--duration` | 10 s | Maximum simulated lifecycle time, including INIT; greater than 0, at most 600 s. |
| `--watchdog-ms` | duration + 2000 ms | Wall limit after the SDK session launches; 1–605000 ms. |
| `--settle-seconds` | 0.25 s | Unpowered native settling before TeamCode; 0–5 s. |
| `--max-lag-ms` | 250 ms | Maximum permitted delay behind the fixed-step schedule; 1–5000 ms. |
| `--report` | `<project>/build/headless-physics/report.json` | Atomic JSON output; relative CLI paths use the repository working directory. |

Existing commands with a numeric third argument retain the legacy motor-only executor.
That mode has no chassis or scene physics. Physics runs honor reviewed model/scene selections,
CAD collision coverage and placement checks, robot XML, motor presets, extra classpaths,
calibration, field-only/piece modes, and configured sensor poses/materials. Import or review
a model using the [guided setup](MODEL_PREPARATION.md) first.

## Timing and lifecycle

One thread advances shared battery/motor dynamics, drive commands, native physics and pose
publication. There is no second background motor clock. Brush tires, rotating native wheels
and flexible intakes use 480 Hz; other configurations use 120 Hz. Native joints drive
mechanism encoders. IMU and Pinpoint/OTOS receive native chassis pose; configured scene
sensors sample native geometry with their saved rate/latency settings.

Settling runs before TeamCode and is excluded from duration/watchdog limits. The first
odometry origin is the settled robot pose. INIT runs on the SDK thread; the runner sends PLAY
after approximately 50 ms. Iterative `init`, `init_loop`, `start`, `loop`, and `stop` are
supported, as are linear `waitForStart`, sleeps and cooperative stop. Configured practice
scoring starts with PLAY and advances on native time.

The run is paced against wall time because SDK timers, sleeps and TeamCode scheduling use
wall time. It is **not accelerated or deterministic**. Every simulated step runs; none are
dropped. A scheduling delay above the configured limit produces `timing_overrun`. Smaller
delays and total native/wall time are reported. Use a suitably sized CI worker; raising the
lag limit permits more timing error and does not make that error disappear.

## Results and failure handling

`completed` means TeamCode returned normally. `duration_complete` means the requested run
window ended and cooperative cleanup finished; it does **not** prove an autonomous objective
was reached. The CLI exits successfully for these outcomes. Check the outcome and trajectory
when writing an autonomous assertion.

Watchdog, user exception, excessive lag, stop timeout, native-state failure, startup/config
failure and cleanup failure produce a failed result and nonzero CLI exit. Usage errors occur
before the runner starts. The report contains, when construction/execution reaches that stage:

- Initial/final native pose, joint positions, motor powers, shaft positions and delayed encoders.
- Trajectory samples at about 20 Hz, capped at 10000; truncation is disclosed and final state
  is retained separately. Sample position is FTC x forward, y left, z up in meters; yaw is radians.
- Native diagnostic contacts, support/load, wheel/tire data, joint effort and assumptions.
  Diagnostic vectors retain the diagnostics guide's **jME x, y up, z backward** convention.
- Sensor summaries/settings, practice scoring/events and piece poses, when configured.
- Input file hashes, saved robot runtime/parameters, field bounds/mode, mass, body/piece counts,
  simulation/wall duration, step count and maximum scheduling lag.
- Failure details, whether the SDK thread terminated, and whether stop commands were issued.
  Final pose/diagnostics are captured **before** cleanup commands zero motor powers.

Stop requests set both SDK stop flags and interrupt the daemon thread, then wait up to 100 ms.
No more motor or physics ticks occur. Code that ignores stop/interrupt cannot be force-killed
safely. A CLI invocation has its own JVM, which exits even if such code remains alive; an
in-process runner rejects another run while an orphan remains. The report discloses this state.
Restart the process before continuing. Compilation and native construction are outside the
SDK watchdog; CI systems should also impose a job/process timeout.

## Physical and SDK limits

This mode preserves the simulator's existing approximations: generic box chassis/aggregate
drive, configured imported collisions, empirical tire/contact parameters, optional flexible
intake/retention, geometric camera observations and practice scoring. It does not certify
real-world calibration, individual Mecanum rollers, image processing, real vendor pathing
libraries or official game scoring. Neutral gamepads can leave a TeleOp stationary.

Actual compiled closed-loop native tests are in `HeadlessPhysicsTest`; they cover odometry,
IMU turns, distance changes, wall blocking, mechanism travel/stall, reviewed imported field
modes, rotating wheels/suspension, scoring, reloads and process watchdog isolation.
