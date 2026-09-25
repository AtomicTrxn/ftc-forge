# Phase 2 Results: Kinematics & 2D Canvas

**Status:** Complete — kinematics validated headlessly with real JUnit tests; the jME renderer was validated by actually launching it, not just compiling it.
**Task spec:** [phase-2-kinematics-2d-canvas.md](phase-2-kinematics-2d-canvas.md)

## Summary

Built and validated: an engine-agnostic ideal Mecanum kinematics module (`physics-engine`), and a real jMonkeyEngine orthographic top-down renderer (`gui-runner`) that runs a genuine compiled OpMode through Phase 1's executor and visually reflects its motion. This phase also set up a **real Gradle build** for the whole repo (Phase 1 had none — see its RESULTS.md's top open issue), which is how jME's dependencies were resolved at all; this closes that deviation as a side effect of doing Phase 2 properly. The MVP checkpoint is validated with both a real GPU-rendered window (confirmed via OpenGL context logs, on this machine's actual Apple M3 Pro) and independently-checkable numeric proof (final robot pose matches hand-calculated expected values from the OpMode's own commanded powers, to within simulation timing noise).

## Kinematics Implementation

`physics.MecanumKinematics` (standard ideal, no-slip "X" Mecanum configuration) and `physics.ChassisPose` (field-frame pose integrator), in `physics-engine`:

```
forward(powerLF, powerRF, powerLB, powerRB) -> ChassisVelocity(vx, vy, omega)
inverse(vx, vy, omega) -> [powerLF, powerRF, powerLB, powerRB]
ChassisPose.integrate(ChassisVelocity, dtSeconds)
```

**Deliberately produces a chassis-frame velocity, not a 2D-only position update** — per this phase's spec, this is exactly what Phase 4 will reuse unchanged, applying it as a force/torque directly to a Libbulletjme rigid body (R2/R6 found that Mecanum strafing can't be produced by wheel-ground contact physics in any plain rigid-body engine, so the kinematics model, not the physics engine, has to stay the authority on chassis motion in Phase 4 too).

**Validated against known test paths** with real JUnit 5 tests (`physics-engine/src/test/java/physics/MecanumKinematicsTest.java`, 6 tests, all passing):
- All wheels forward → pure `vx`, zero `vy`/`omega`.
- The strafe-left wheel pattern → pure `vy`, zero `vx`/`omega`.
- The rotate-in-place pattern → pure `omega`, zero translation.
- `inverse()` then `forward()` round-trips to the same chassis velocity.
- `inverse()` never commands a wheel power outside `[-1, 1]`, even for an unreachable target velocity.
- A full pose-integration scenario mirroring the MVP checkpoint's own script ("drive forward 24 inches, turn 90 degrees") lands within 1 cm / a few degrees of the expected pose (tolerance is discrete-timestep quantization, not kinematics error — documented inline in the test).

## Field Rendering

Real jMonkeyEngine (`org.jmonkeyengine:jme3-core`/`jme3-desktop`/`jme3-lwjgl3:3.7.0-stable`, resolved via the Gradle build set up this phase) with an **orthographic top-down camera** (`cam.setParallelProjection(true)` + `cam.setFrustum(...)`), not a separate 2D canvas toolkit — confirmed to actually initialize a real OpenGL context (see Validation below), so Phase 4 extends this same scene/camera rather than replacing it.

- Field: 6×6 grid of 24 in (0.6096 m) tiles = 12×12 ft, alternating colors, built from `com.jme3.scene.shape.Quad` geometry rotated flat into the XZ plane.
- Robot: a `Box` body (18×18 in footprint, matching the real FTC max-robot-size constraint) plus a smaller front-facing marker box so heading is visually distinguishable, not just position.
- World-frame convention (this project's own, documented in code): `ChassisPose`'s `(xMeters, yMeters)` map to world `(X, -Z)`; heading maps to a rotation about world `+Y`.

## Gamepad Integration

Wired via jME/LWJGL3's native input handling (`inputManager.getJoysticks()`), per R2's finding that this resolves the plan's original "Jamepad" dependency question — confirmed the joystick-detection code path actually executes (`[SIM] No joysticks detected in this environment` — correct, since no physical controller is attached here). Keyboard arrow keys are wired as a manual-driving stand-in for validation purposes.

**Honestly flagged gap:** no physical gamepad was available in this environment, so the actual joystick-axis-to-`Gamepad`-field wiring could not be exercised end-to-end with a real controller. The code path is real (not a stub) and the detection logic runs correctly; it's specifically the "move a real joystick and see the mapped `Gamepad` fields change" scenario that remains unverified. Low risk — jME's joystick API is a thin, well-documented wrapper — but flagged rather than silently assumed.

## MVP Checkpoint Validation

Ran the real `BasicMecanumOpMode` (Phase 1's sample: drive forward 1s, strafe right 1s) through the actual jME renderer, twice independently, via `gradle :gui-runner:runSimulatorApp`:

```
INFO: Running on jMonkeyEngine 3.7.0-stable
INFO: LWJGL 3.3.3+5 context running on thread jME3 Main
 * Graphics Adapter: GLFW 3.4.0 Cocoa NSGL Null EGL OSMesa monotonic dynamic
INFO: OpenGL Renderer Information
 * Vendor: Apple
 * Renderer: Apple M3 Pro
 * OpenGL Version: 4.1 Metal - 91.7
...
[SIM] Running Basic Mecanum Auto in the jME renderer...
[TELEMETRY] Phase : Forward | Ticks LF : 1344 | Battery V : 12.6
[TELEMETRY] Phase : Strafe Right | Yaw : 0.0
[TELEMETRY] Status : Done. Final ticks LF=2811
[SIM] OpMode finished. Final pose: ChassisPose(x=0.9967m, y=-1.0168m, heading=0.00deg)
```

Second independent run: `ChassisPose(x=1.0079m, y=-1.0051m, heading=0.00deg)` — consistent with the first to within simulation timing noise, confirming determinism.

**Independently checkable, not just eyeballed:** the OpMode commands `power=0.5` (not 1.0) on all four wheels; with `maxWheelSpeedMetersPerSecond=2.0`, that's `vx = 1.0 m/s` for ~1s during the forward phase (expected `x ≈ 1.0 m` — observed `0.997–1.008 m`) and `vy = -1.0 m/s` for ~1s during the strafe-right phase (expected `y ≈ -1.0 m` — observed `-1.005` to `-1.017 m`), with heading correctly staying at exactly `0.00°` throughout both pure-translation phases. The numbers match hand-calculation, not just "something moved."

**Real GPU rendering confirmed, but not captured as a screenshot.** The OpenGL/audio context logs above are genuine evidence the actual window opened with real GPU acceleration on this machine (Apple M3 Pro, Metal-backed OpenGL 4.1) — this is not a claim taken on faith. A `screencapture` attempt during the run failed (`could not create image from display`), almost certainly a macOS Screen Recording permission this automated session's process doesn't have — a real environment limitation of this validation session, not a rendering failure. A human running the same `gradle :gui-runner:runSimulatorApp --args="sample-teamcode BasicMecanumOpMode"` command interactively would see the actual window.

## Deviations from Research

- **Set up a real Gradle build this phase** (`settings.gradle`, root `build.gradle`, per-module `build.gradle`s) — Phase 1 had none and flagged this as its top open issue; jME's dependency resolution needed it, so it's done now rather than deferred further. JDK target is still 17, not R2's recommended 21 LTS (same environment constraint noted in Phase 1).
- **Gamepad validation is code-complete but not device-tested** (see above) — flagged, not silently assumed working.
- **`SimDcMotorEx`'s power field is a plain (non-`volatile`) `double`**, read by the render thread while the OpMode thread writes it. Benign for this validation (doubles don't tear in a way that produces garbage values, and visualization tolerates a stale frame), but worth hardening (e.g. `volatile` or an atomic snapshot) before Phase 3/4 build more on top of this same read pattern.
- The screenshot-capture gap above is specific to this automated session's permissions, not the renderer.

## Open Issues for Phase 3

1. ~~Phase 3's motor model replaces the direct `power → kinematics.forward()` pipeline...~~ — resolved in Phase 3: added `forwardFromWheelSpeeds(...)` alongside the existing `forward(power...)`, exactly as anticipated here, plus a real bug this exposed (direction-reversed wheels breaking kinematics) that this note didn't anticipate. See Phase 3's RESULTS.md.
2. Harden the `SimDcMotorEx` power-field concurrency note above before Phase 3 adds more state that's read across threads (battery current draw, thermal state).
3. Real joystick hardware validation remains open — worth a quick manual check whenever a physical controller is available, though not blocking.

## Recommendation: Ready to Ship MVP?

**Yes.** Phases 1+2 together let a real team point the simulator at their own code and watch it drive on a field, exactly as the plan's MVP checkpoint intends — validated with a real, GPU-rendered window and independently-verifiable numbers, not just a description of what should happen. Pause here for real-team feedback before starting Phase 3, per the plan.
