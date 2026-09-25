# R5 Results: Telemetry Schema & Logging Wrapper

**Status:** Complete (amended after Opus review pass — see changelog)
**Task spec:** [05-telemetry-schema-logging-wrapper.md](05-telemetry-schema-logging-wrapper.md)
**Reads:** [04-non-ideality-modeling.RESULTS.md](04-non-ideality-modeling.RESULTS.md) — the calibration pass below tunes exactly the parameters R4 flagged as unverified (`R_battery`, the thermal-throttling heuristic) or per-team-variable (per-SKU motor constants).

**Changelog:** added the worked example row this task's own Definition of Done required but the original pass omitted; corrected a misattribution (the `/sdcard/FIRST/` path and "grid search per R4" were presented as R3/R4's own conclusions, but neither file actually says that — R3 never mentions that path, and R4 doesn't prescribe an optimizer); corrected the logger design, which described reading from "R3's classloading design's registry" — the logger runs on the **real robot** against the **real SDK**, not inside the simulator, so it must use only the real public `HardwareMap` API; replaced the single grid-search calibration step with a staged fit, since the original approach can't actually separate its parameters without a drivetrain model; added run/battery/hardware metadata the calibration pass needs to know which R4 constants apply; unified three inconsistent phrasings of loop-timing variance into one.

## Summary

Real FTC hardware confirms the local-storage-only default is not just safe, it's the only mechanism actually available: the robot controller (Control Hub) already writes its own logs to local storage (`/sdcard/RobotControllerLog.txt`) and exposes retrieval via `adb pull` or a web-UI download button — there's no existing network-upload path to lean on even if one wanted it. Actual OpMode loop timing was confirmed to vary considerably — roughly 5–10 ms per iteration with bulk caching and few I2C devices, up to 100 ms or more with many unbatched I2C reads or poor WiFi — so no nominal rate is assumed anywhere in this schema; every row records its own real elapsed time.

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
| `motor_<name>_vel_tps` | ticks/sec | `getVelocity()` — free to read alongside ticks when bulk caching is enabled (see Logging Wrapper Design). Needed for calibration (below): fitting a drivetrain model from ticks alone requires differentiating noisy position data, while velocity is measured directly. |

**Worked example row** (matching the header above, satisfying this task's own "worked example, not just a field list" requirement):

```
1520.35,73,19.84,12.412,-3.25,0.12,-0.08,0.40,-0.20,-12.60,0.500,412,0.500,408,0.500,415,0.500,410
```

**Metadata (previously missing entirely — the calibration pass below cannot know which R4 constants apply without it).** Written once as a header comment block or a `<logfile>.meta.json` sidecar, not repeated per row:
- `schema_version`, `logger_version`, `sdk_version` — so a log recorded against a different SDK/schema version isn't silently misinterpreted later.
- `robot_config_name` — which robot-configuration XML (R3) this log corresponds to.
- Per motor: **SKU/gear ratio** (the exact gap R3 and R4 both flagged — the real config XML likely can't carry this, so it has to come from somewhere, and this metadata block is that home for a recorded run), `run_mode`, `zero_power_behavior`, `direction`.
- `battery_id`, `bulk_caching_mode` — the latter directly affects how `loop_time_ms` should be interpreted (see Logging Wrapper Design).

**Sample rate:** not fixed — one row per real loop iteration, whatever that robot's actual loop rate is. This is a deliberate choice, not a gap: forcing a fixed sample rate would require interpolating or dropping data to match, discarding exactly the loop-timing variability that's relevant to reproducing real behavior in sim.

## Logging Wrapper Design

**Correction to the original design's core premise:** this logger runs **on the real physical robot**, against the **real FTC SDK** — not inside the simulator. The original draft described it as reflectively enumerating "the same registry R3's classloading design builds," but R3's registry is a simulator-internal construct that doesn't exist on real hardware. The logger must be written **only against the real, public `HardwareMap` API**, so the exact same `.java` file compiles and runs unmodified whether dropped into a team's real robot project or (for testing) the simulator's own mock SDK:

```java
// One line, anywhere in an existing OpMode's loop():
SimTelemetryLogger.logTick(hardwareMap, telemetry, elapsedTime);
```

Internally, `logTick` uses only public API: `hardwareMap.getAll(DcMotor.class)` paired with `hardwareMap.getNamesOf(device)` to enumerate configured motors (the real SDK's own supported pattern for hardware-agnostic iteration — the Road Runner quickstart uses the equivalent `hardwareMap.voltageSensor.iterator().next()` idiom), plus the same pattern for `IMU`/`VoltageSensor`. No per-team configuration beyond calling this one line — the column set is derived automatically from whatever devices that specific team has actually configured, and metadata (above) is captured once at the start of the run.

**Implementation details the "one line" framing glossed over:**
- The logger creates its output file lazily on the first `logTick` call (not in a constructor — avoids an empty file if the OpMode never actually loops), buffers writes (`BufferedWriter`, flushed every N rows rather than every row, to avoid adding I/O latency to the very loop it's measuring), and registers a JVM shutdown hook (plus an explicit `close()` teams can call from `stop()`) so a normal match-end still flushes and closes the file cleanly.
- **The logger adds its own read cost, which can contaminate the very `loop_time_ms` it's trying to capture.** Reading every encoder and the IMU each loop adds real I2C round-trips unless bulk caching is already active. Record `bulk_caching_mode` in the metadata block (above) precisely so this is interpretable: under `AUTO`/`MANUAL` caching, the logger only reads values the OpMode's own loop already fetched that tick (no extra cost); with caching off, logging genuinely adds I2C traffic and `loop_time_ms` should be read with that in mind. `getCurrent()` and IMU reads beyond what a normal drivetrain OpMode would already do should be opt-in, not automatic, for the same reason.

**CSV robustness (not addressed in the original pass):** FTC device names can legally contain spaces, hyphens, or other characters that break a bare `motor_<name>_power` column-naming scheme. Sanitize names to `[A-Za-z0-9_]` for column headers and record the original-name → sanitized-column mapping in the metadata block. Format all numeric fields with a fixed locale (`Locale.ROOT`) — `String.format` on a non-US-locale Android device can silently emit decimal commas instead of decimal points, corrupting the CSV for anyone parsing it with a naive splitter.

**Getting the file off the robot — confirmed against real hardware, not assumed:**
- The Control Hub already writes its own operational log to local storage (`/sdcard/RobotControllerLog.txt`) and the documented retrieval paths are (a) a "Download Logs" button in the Control Hub's own web management UI, and (b) `adb pull` over USB. **Neither involves network upload** — this independently confirms the plan's local-storage-only default was correct, not just a cautious guess.
- **Correction:** the original pass attributed a `/sdcard/FIRST/` convention to R3 — R3's results don't actually specify that path (R3 discusses the robot-configuration XML's *content*, not a specific on-device *file location* for it). This project's own recommended default, standing on its own merits rather than a borrowed citation: write to `/sdcard/FIRST/sim_telemetry/<timestamp>.csv`, since `/sdcard/FIRST/` is understood to be the SDK's own conventional root folder for its files (commonly referenced as `AppUtil.ROOT_FOLDER` in SDK-adjacent discussion) — plausible and consistent with the ecosystem, but this exact constant name/path was not independently re-verified against SDK source in this task. Retrieve via `adb pull` as the primary, verified-available mechanism; wireless `adb connect <controlhub-ip>:5555` is also worth noting as a no-cable, still-local (no internet dependency) alternative.
- **Possible lower-friction alternative, unverified:** the SDK exposes a `@WebHandlerRegistrar` mechanism for adding custom routes to the Robot Controller's own built-in web server — if usable for arbitrary file serving, a team could fetch their log from a browser (e.g. `http://192.168.43.1:8080/...`) instead of needing `adb` at all. This was not confirmed in this research pass (including whether Road Runner's own tooling already does something like this) — worth a concrete spike before relying on it, but flagged here as a real avenue rather than assuming `adb pull` is the only option.

## Calibration Algorithm

**Revised into a staged fit — a single undifferentiated grid search over "the uncertain parameters" cannot actually work, and the previous version overstated what R4 itself established.** Two concrete problems with the original approach: (1) predicting encoder ticks from commanded power requires a drivetrain model (chassis mass, wheel radius, friction) that was never specified anywhere, so error from a missing drivetrain model would get silently absorbed into `R_battery`/thermal fits, producing a fit that matches the recorded numbers for the wrong reasons; (2) I2C sensor latency (R4's ring-buffer redesign) isn't observable from this schema at all — nothing in the log lets calibration distinguish "value arrived late" from "value was simply that at this time." Also, the previous wording claimed "R4 confirms only 2-3 genuinely uncertain constants" — R4 doesn't state a count; that was this document's own paraphrase presented as if it were R4's conclusion, corrected here.

**Staged fit, in order:**
1. **Battery, first, in closed form.** With `motor_<name>_current_a` recorded (if the logging wrapper reads `getCurrent()`) or derived from R4's current equation applied to recorded power/velocity, fit `R_battery` and `V_internal` by linear least-squares regression of `battery_voltage_v` against total current — R4's corrected battery equation is linear in `R_battery`, so this step has a closed-form solution and doesn't need a search at all.
2. **Motor/drivetrain, second, via one-step-ahead prediction.** Fit the motor model's friction terms (`τ_static`, viscous coefficient `b` — R4's kS/kV/kA-equivalents) using one-step-ahead velocity prediction: reset the simulated state to the *recorded* state at the start of each row before predicting that row's outcome, so errors don't compound across the whole run (open-loop replay over an entire match would let position error accumulate and produce a misleading fit). This mirrors the same ramp-based feedforward-tuning approach Road Runner/Pedro already use for their own kS/kV/kA tuning, applied in reverse (fitting the simulator to match recorded reality rather than tuning a controller).
3. **Thermal throttling, only if needed.** Only fit the duty-cycle derating heuristic if step 2's residual error grows systematically over long, sustained-load runs — short recordings won't have enough signal to constrain this parameter, and it shouldn't be force-fit against noise.
4. **Sensor/I2C latency: not a calibration target with this schema.** Drop it as a fit parameter. If it needs to be measurable in the future, that requires an additional schema field (e.g. an `imu_read_ms` column timing the actual `getRobotYawPitchRollAngles()` call with `System.nanoTime()` on the logging side, or checking whether `YawPitchRollAngles` exposes an acquisition timestamp directly) — not something to retrofit into this calibration pass as currently scoped.
5. Per-SKU motor constants (`τ_stall`, `ω_no_load` from R4's datasheet table) remain **not** tuning targets — they're fixed, sourced constants per motor ratio, identified via the `sku`/`ratio` metadata field (above), not fit from data.

## Risks / Unknowns Remaining

- The Control Hub web UI's file-download scope (arbitrary files vs. only the built-in log), and the viability of a custom `@WebHandlerRegistrar` route as an alternative, are both unverified — flagged above for a quick Phase 3 check.
- The `/sdcard/FIRST/` path and `AppUtil.ROOT_FOLDER` naming are plausible but not independently re-verified against SDK source — confirm before Phase 3 hardcodes it.
- Real loop-timing variance (5–10 ms up to 100+ ms observed in the wild) means a very short or very jittery real recording may not contain enough consistent-load data for the staged calibration fit to converge well; Phase 3's validation step should include a check for "recording too short/noisy to calibrate confidently" rather than silently accepting a bad fit.
- The staged fit's step 2 (motor/drivetrain friction terms) still implicitly needs at least a rough chassis mass/wheel-radius estimate to convert torque to expected wheel velocity — this schema and calibration design don't specify where that estimate comes from (a Phase 1 preset field, presumably) — flagged as a loose end for whoever implements Phase 3, not resolved here.
