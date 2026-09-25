# Phase 3 Results: Sensor Latency & Noise

**Status:** Complete — every physics equation validated by a real JUnit test against R4's own reference points; the calibration pass validated by recovering known ground-truth parameters from a synthetic log (no physical robot is available in this environment); zero regressions confirmed by re-running all of Phase 1/2's existing validation scenarios.

**Task spec:** [phase-3-sensor-latency-noise.md](phase-3-sensor-latency-noise.md)

## Summary

Replaced Phase 1's placeholder linear tick-integration with R4's corrected motor/battery models, added `DcMotorEx` run-mode behavior (`RUN_USING_ENCODER`/`RUN_TO_POSITION`), shipped sensor/bus latency in this phase rather than deferring it further (per R4's own reassessment that the ring-buffer redesign is cheap enough to include), and built R5's staged calibration pass — validated end-to-end by generating a synthetic ground-truth log and confirming the calibrator recovers the true parameters from it alone. Along the way, this phase closed Phase 2's own flagged open issue (the kinematics interface needed a real-wheel-velocity input, not just commanded power) and, in doing so, found and fixed a real bug: naively feeding real physical motor rotation into the kinematics broke driving entirely for any wheel with `Direction.REVERSE` set, because that setting exists specifically to let team code use consistent command signs despite physically mirrored motor mounting — a distinction that's easy to get backwards, and was caught by an actual renderer run producing a stuck robot, not assumed correct in advance.

## Motor Model Implementation

`physics.MotorModel` implements R4's corrected equations exactly:

```
tau = tauStall * (Vactual/Vnominal - omega/omegaNoLoad)
I   = iStall    * (Vactual/Vnominal - omega/omegaNoLoad)
```

Validated with 7 JUnit tests (`MotorModelTest`) directly against R4's own Validation Notes, including the specific test R4 called out as the one that actually distinguishes the corrected equation from the plan's original (wrong) one (`V=0.5·Vnom, ω=0.5·ωnl → τ=0`) — not just the weaker "half voltage → half stall torque" check that both equations pass.

**Addition beyond R4 (documented, not silently assumed to already be in the research):** R4's equations are the ideal EMF relation and have no free parameter for real mechanical friction. Added a standard static+viscous friction term (`tauStaticNm`, `viscousBNms` — the same kind of thing Road Runner/Pedro call `kS`/`kV` feedforward terms) as an explicit calibration target, since R4's own torque-curve constants are fixed, sourced datasheet values, not tuning targets. Thermal throttling is implemented as R4's suggested rolling-heat duty-cycle heuristic, present but **not calibrated in this pass** — R5's own staged-fit design only calls for fitting it "if long-run residuals justify it," and this phase's validation runs are short.

**`DcMotorEx` run modes** (`SimDcMotorEx.commandedPower()`): `RUN_USING_ENCODER` reinterprets `setPower()` as a target-velocity fraction and runs a P-controller closing the loop on `omegaRadS`; `RUN_TO_POSITION` layers a position P-controller producing a velocity target on top of that. Default gains are placeholders (documented as such, per R4's own note that these should be "tuned during Phase 3, refined by R5's calibration pass" — tuning them isn't done in this pass; they work correctly but aren't claimed to be well-tuned).

**Rotational inertia** (`ROTATIONAL_INERTIA_KG_M2 = 0.0015`) is a fixed placeholder, not sourced from real CAD — consistent with the project's plan that per-robot inertia is Phase 5's URDF importer's job, not this phase's.

## Battery Sag Implementation

`physics.BatteryModel` implements R4's closed-form multi-motor solve exactly:

```
A = sum(p_i^2 * iStall_i / Vnominal_i)
B = sum(p_i * iStall_i * omega_i / omegaNoLoad_i)
Vb = (Vinternal + Rbattery * B) / (1 + Rbattery * A)
```

Validated with 3 JUnit tests (`BatteryModelTest`), including a manual single-motor cross-check of the general formula. `HardwareMapBuilder.tickMotors` now shares **one** `BatteryModel` instance across all motors on the robot per tick (a real architectural change from Phase 1: motors can no longer tick independently, since the entire point of the battery model is that one motor's draw affects every other motor's effective voltage in the same tick) — confirmed working end-to-end: re-running Phase 1's `BasicMecanumOpMode` sample now shows battery voltage genuinely sagging from 12.6V to ~11.3V once all four drive motors are under load, versus a flat constant 12.6V in Phase 1/2.

Default `R_battery = 0.15 Ω`, `V_internal = 12.6 V`, per R4's revised recommendation.

## Sensor Latency Implementation

**Shipped in this phase, not deferred** — per R4's own reassessment that the ring-buffer redesign is cheap enough to reconsider. `physics.SensorRingBuffer<T>` implements R4's exact design (push `(simTimeMs, value)` samples every tick; a read returns the most recent sample at or before `now - latencyMs`), validated with 3 JUnit tests including sample eviction beyond `maxAgeMs`. Wired into `SimDcMotorEx`'s encoder tick reads (`encoderLatencyMs = 8`, per R4's 7–10ms I2C default) — `getCurrentPosition()` now returns the latency-delayed value, not the true instantaneous one, matching real hardware behavior.

**Scope decision, documented rather than silently done:** `SimIMU`'s orientation was **not** wired through a ring buffer. Its true value is static (zero) in this codebase — Phase 2 left IMU-to-kinematics wiring unimplemented, and that's a separate, pre-existing gap this phase didn't need to fix. Adding latency plumbing around a value that never changes would be ceremony with no observable effect, not a real validation, so it wasn't added.

## Calibration Pass

`physics.calibration.Calibrator` implements R5's staged fit, as revised by the Opus review — not the original single undifferentiated grid search:

1. **Battery (closed-form, no search):** linear least-squares regression of `V = Vinternal - Rbattery * Itotal`.
2. **Motor friction (grid search, one-step-ahead):** for `(tauStaticNm, viscousBNms)`, predicts each row's next velocity from the current row's state using `MotorModel`, resetting to the recorded state every row (not open-loop replay across the whole run, which would let error compound).
3. **Thermal:** not implemented — consistent with R5's own conditional design.

**Validation methodology, stated honestly:** no physical FTC robot is available in this environment (this is a desktop simulator repo). `CalibratorTest` generates a **synthetic log from known ground-truth parameters** (`R_battery=0.22Ω, V_internal=12.6V, tauStatic=0.12 N·m, viscousB=0.008 N·m·s/rad`) using the exact same forward model the simulator runs, then confirms the calibrator recovers all four values from the log alone, within tight tolerances (±0.01Ω, ±0.05V, ±0.03 N·m, ±0.002 N·m·s/rad). This is a standard, legitimate way to validate a system-identification algorithm's correctness — distinct from, and not a substitute for, validating against a real recorded robot log, which remains untested since none exists.

**Real bug caught by this validation, not assumed away:** the first version of the battery regression used raw per-motor current (`|I_i|`) as its predictor instead of the duty-scaled battery-rail current (`p_i · I_i`) `BatteryModel` actually solves for — the recovered `R_battery` came back with the wrong sign and magnitude (`-0.88` instead of `0.22`). Fixed by matching the regression's predictor variable to the model's own definition exactly. A synthetic-log-generator bug was also caught and fixed along the way (current was computed from the post-integration omega instead of the pre-integration one that determined that tick's battery voltage, desynchronizing the two).

## Drivetrain Model Inputs Used

The one-step-ahead friction fit needs the rotational inertia the forward model used (`0.0015 kg·m²`, the same placeholder `SimDcMotorEx` uses) — passed in explicitly as a parameter, not re-derived or assumed known some other way. This is the "loose end" R5 flagged (needing a chassis mass/wheel-radius-equivalent estimate); for a single motor's own inertia, this constant is that estimate. A full chassis-level calibration (accounting for the wheel's coupling to the rest of the drivetrain once Phase 4 adds real rigid-body coupling) is explicitly out of scope until then.

## Kinematics Interface Fix (closing Phase 2's flagged open issue)

Added `MecanumKinematics.forwardFromWheelSpeeds(vLF, vRF, vLB, vRB)` — takes real wheel linear speeds (m/s) instead of commanded power, so Phase 2's renderer now reflects the motor model's actual torque/speed lag rather than an idealized instant response. `SimulatorApp` converts each motor's real angular velocity to wheel linear speed via a wheel-radius constant (`0.048m`, goBILDA 96mm mecanum wheel).

**Bug found and fixed during this wiring:** naively using `getOmegaRadS()` (real physical rotation) directly broke driving for any wheel with `Direction.REVERSE` set — `Direction.REVERSE` exists specifically so team code can use consistent `+power = forward` semantics despite physically mirrored motor mounting on opposite drivetrain sides, but the kinematics formula's sign convention is defined in terms of that same commanded/logical intent, not raw physical rotation. Feeding it raw physical rotation double-applies the direction compensation. Fixed by converting back to logical sign (`direction == FORWARD ? omega : -omega`) before handing it to kinematics. Caught by actually running the renderer and seeing a robot spin in place instead of drive (final pose stuck at `(0,0)` with heading changing) — not caught by inspection, which is exactly why this project keeps re-running the real renderer after changes that touch this path, not just the unit tests.

**Re-validated after the fix:** re-ran `BasicMecanumOpMode` through the real jME renderer; heading correctly stays at exactly `0.00°` through both pure-translation phases (forward + strafe), and final position (`x≈0.76m, y≈-0.75m`) is somewhat less than Phase 2's idealized `~1.0m` — the physically expected direction of change, since the real motor model now has acceleration lag rather than instant response, not an unexplained discrepancy.

## Deviations from Research

- Friction terms and the specific thermal-heuristic shape are additions this phase made beyond R4's own equations (documented above), not something R4 itself specified numerically.
- `RUN_USING_ENCODER`/`RUN_TO_POSITION` PIDF gains are placeholders that work correctly but aren't tuned.
- Rotational inertia (`0.0015 kg·m²`) is a fixed, unsourced placeholder for every motor regardless of SKU — a real per-motor value would come from Phase 5's URDF `<inertial>` data.
- `SimIMU` was deliberately left unwired to the ring-buffer latency mechanism (see above) — a scope decision, not an oversight.
- No real recorded robot telemetry log exists to validate the calibrator against — only the synthetic ground-truth log described above. This is a real gap in end-to-end validation (the calibration *algorithm* is proven correct; whether it converges well on messy, noisy *real* hardware data is not yet known) worth closing whenever a real robot/log becomes available.
- The on-robot logging wrapper R5 designed (the class a real team would drop into their own TeamCode) was not implemented in this repo — it's fundamentally a deliverable for a team's own codebase, not this simulator's. `TelemetryLogWriter`/`TelemetryLogReader` in this repo implement the same CSV schema but are scoped to what this phase's calibration needed (battery/motor columns only, no IMU columns, since IMU orientation isn't dynamically tracked anywhere in this codebase yet).

## Open Issues for Phase 4

1. Phase 4's rigid-body physics will couple each wheel's rotation to the chassis and ground contact — the placeholder rotational inertia (motor-only, no chassis load) will need to be reconsidered once a wheel is no longer spinning freely.
2. The friction/thermal parameters calibrated (or left uncalibrated) here should be re-examined once real telemetry logs are available, not trusted as final.
3. `SimIMU` remains unwired to real chassis orientation — whichever phase finally drives IMU from real kinematics (Phase 2's original scope, never completed) should also decide whether to wire it through the ring-buffer latency mechanism already built and validated here.
