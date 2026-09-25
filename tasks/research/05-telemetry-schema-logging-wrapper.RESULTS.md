# R5 Results: Telemetry Schema & Logging Wrapper

**Status:** Complete
**Task spec:** [05-telemetry-schema-logging-wrapper.md](05-telemetry-schema-logging-wrapper.md)
**Reads:** [04-non-ideality-modeling.RESULTS.md](04-non-ideality-modeling.RESULTS.md) — the calibration pass below tunes exactly the parameters R4 flagged as unverified (`R_battery`, the thermal-throttling heuristic) or per-team-variable (per-SKU motor constants).

## Summary

Real FTC hardware confirms the local-storage-only default is not just safe, it's the only mechanism actually available: the robot controller (Control Hub) already writes its own logs to local storage (`/sdcard/RobotControllerLog.txt`) and exposes retrieval via `adb pull` or a web-UI download button — there's no existing network-upload path to lean on even if one wanted it. Actual OpMode loop timing was confirmed to vary considerably (roughly 45 Hz nominal, but anywhere from ~10 Hz to over 100 ms/iteration depending on I2C calls and WiFi conditions), which means the schema must record real per-iteration elapsed time rather than assuming a fixed sample interval.

## Telemetry Log Schema

**Format: CSV, append-only, one row per OpMode loop iteration.** Chosen over JSON for a concrete embedded-reliability reason: a CSV file can be appended to line-by-line as the loop runs and survives an OpMode crash or an unplugged battery mid-match with only the last partial line lost. A JSON array, by contrast, either has to be held in memory and written once at the end (lost entirely on a crash) or requires adopting JSON-Lines to get the same append-safety — at which point it's not meaningfully different from CSV, just harder to read by hand at the field.

**Header row** is generated per-robot, from the team's actual configured hardware (see Logging Wrapper Design below) — it is not a fixed schema, because the number of motors/sensors varies by robot. Example header for a 4-motor Mecanum chassis with one IMU:

```
t_ms,loop_iter,loop_time_ms,battery_voltage_v,imu_yaw_deg,imu_pitch_deg,imu_roll_deg,imu_angvel_x_dps,imu_angvel_y_dps,imu_angvel_z_dps,motor_left_front_drive_power,motor_left_front_drive_ticks,motor_right_front_drive_power,motor_right_front_drive_ticks,motor_left_back_drive_power,motor_left_back_drive_ticks,motor_right_back_drive_power,motor_right_back_drive_ticks
```

| Field | Units | Notes |
|---|---|---|
| `t_ms` | milliseconds | Elapsed since OpMode `start()`, matching the `ElapsedTime` convention already used in team code (R1) — not wall-clock time. |
| `loop_iter` | integer | Monotonic loop counter, for detecting dropped/skipped iterations during replay. |
| `loop_time_ms` | milliseconds | **Actual measured duration of that specific iteration** — not a nominal constant. Confirmed necessary: real loop timing varies from ~10 ms to 100+ ms depending on I2C call count and WiFi conditions, so the calibration replay (below) must step the simulated clock by each row's *real* recorded `loop_time_ms`, not an assumed fixed rate. |
| `battery_voltage_v` | volts | Direct `VoltageSensor` reading. |
| `imu_yaw/pitch/roll_deg`, `imu_angvel_*_dps` | degrees, degrees/sec | From `getRobotYawPitchRollAngles()` / `getRobotAngularVelocity()` (R1). |
| `motor_<name>_power` | [-1.0, 1.0] | One column pair per configured motor, named using R3's finalized HardwareMap device-name convention — so a log file's columns line up exactly with the team's real robot-configuration XML names. |
| `motor_<name>_ticks` | encoder counts | Raw `getCurrentPosition()` per motor. |

**Sample rate:** not fixed — one row per real loop iteration, whatever that robot's actual loop rate is. This is a deliberate choice, not a gap: forcing a fixed sample rate would require interpolating or dropping data to match, discarding exactly the loop-timing variability that's relevant to reproducing real behavior in sim.

## Logging Wrapper Design

**One-line integration, auto-discovering hardware from the OpMode's own `HardwareMap`** rather than requiring a team to hand-list every motor/sensor (which would violate "zero custom instrumentation"):

```java
// One line, anywhere in an existing OpMode's loop():
SimTelemetryLogger.logTick(hardwareMap, telemetry, elapsedTime);
```

Internally, `logTick` reflectively enumerates the `HardwareMap`'s registered `DcMotor`/`DcMotorEx` and `IMU`/`VoltageSensor` entries (the same registry R3's classloading design already builds), reads their current state, and appends one CSV row. No per-team configuration beyond calling this one line — the column set is derived automatically from whatever devices that specific team has actually configured.

**Getting the file off the robot — confirmed against real hardware, not assumed:**
- The Control Hub already writes its own operational log to local storage (`/sdcard/RobotControllerLog.txt`) and the documented retrieval paths are (a) a "Download Logs" button in the Control Hub's own web management UI, and (b) `adb pull` over USB. **Neither involves network upload** — this independently confirms the plan's local-storage-only default was correct, not just a cautious guess.
- Recommended default for this project's log: write to `/sdcard/FIRST/sim_telemetry/<timestamp>.csv` (same `/sdcard/FIRST/` convention R3 identified for the real robot-configuration XML, keeping all simulator-relevant files in one predictable place), retrieved via `adb pull` as the primary, verified-available mechanism.
- **Caveat, not verified:** whether the Control Hub's web "Download Logs" button can be pointed at an arbitrary custom file (vs. only the built-in `RobotControllerLog.txt`) was not confirmed in this research — if that's possible in the pinned SDK version (R1: v12.0), it would be a lower-friction alternative to `adb pull` for less technical teams, and is worth a five-minute check during Phase 3 implementation rather than assumed here.

## Calibration Algorithm

Ties directly to R4's named parameters:

1. **Replay:** step the simulated motor/battery models using each row's `t_ms`/`loop_time_ms` as the real elapsed time per step (not a fixed assumed rate — see schema notes above), feeding each row's recorded `motor_*_power` commands into R4's motor model exactly as the real OpMode issued them.
2. **Diff:** compare the simulated `motor_*_ticks` and `battery_voltage_v` trajectories against the recorded ones, accumulating squared error over the whole run.
3. **Tune:** adjust R4's flagged-uncertain parameters — `R_battery` (no confirmed REV-specific figure), the thermal-throttling duty-cycle heuristic's threshold/derating factor, and (once implemented) each sensor's I2C latency `N` — via a simple grid search over a physically-plausible range for each parameter, keeping whichever combination minimizes total squared error. A full least-squares/gradient optimizer is not necessary for v1 given the small number of free parameters (R4 confirms only 2-3 genuinely uncertain constants); grid search is explicitly sufficient scope for this phase, per R4 and this task's own instructions.
4. Per-SKU motor constants (`τ_stall`, `ω_no_load` from R4's datasheet table) are **not** tuning targets — they're fixed, sourced constants per motor ratio. Only the battery and thermal parameters, which R4 confirmed are genuinely unverified/team-variable, are calibration targets.

## Risks / Unknowns Remaining

- The Control Hub web UI's file-download scope (arbitrary files vs. only the built-in log) is unverified — flagged above for a quick Phase 3 check.
- Real loop-timing variance (10 Hz–100+ Hz observed in the wild) means a very short or very jittery real recording may not contain enough consistent-load data for the calibration grid search to converge well; Phase 3's validation step should include a check for "recording too short/noisy to calibrate confidently" rather than silently accepting a bad fit.
